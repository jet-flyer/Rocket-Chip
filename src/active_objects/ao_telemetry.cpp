// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_Telemetry — Telemetry Protocol Active Object
//
// Protocol-only: CCSDS/MAVLink encoding, APID mux, rate dividers.
// No radio hardware references. Posts SIG_RADIO_TX to AO_Radio.
// Receives SIG_RADIO_RX for decode + output.
//============================================================================

#include "ao_telemetry.h"
#include "ao_radio.h"
#include "ao_flight_director.h"
#include "rocketchip/station_output_mode.h"
#include "rocketchip/ao_signals.h"
#include "rocketchip/telemetry_encoder.h"
#include "rocketchip/mavlink_rx.h"
// c_library_v2 (third-party, auto-generated) has a packed struct warning
// in mavlink_msg_obstacle_distance.h — we don't use that message.
// Suppressing here is standard practice for this library (ArduPilot does the same).
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "common/mavlink.h"
#pragma GCC diagnostic pop
#include "common/mavlink.h"        // MAVLink c_library_v2 (COMMAND_LONG encoding)
#include "rocketchip/radio_config.h"
#include "rocketchip/radio_config_table.h"          // T5.5: SET whitelist
#include "rocketchip/job.h"
#include "flight_director/mission_profile_data.h"  // kDefaultRocketRadioConfig
#include <math.h>                                   // lroundf (T5.5: float->int)
#ifdef ROCKETCHIP_JOB_STATION
// R-25-exec step 6 (2026-05-13): station_fault_inject.h migrated to
// src/safety/. Runtime test_mode_active() gate at each
// fault_force_station_* entry replaces compile-time gate; the global
// flags themselves are read-only here and harmless on inactive boots.
#include "safety/station_fault_inject.h"
#endif

#ifndef ROCKETCHIP_HOST_TEST
#include "pico/time.h"
#include "pico/stdio_usb.h"
#include "tusb.h"
#endif

#include "rocketchip/rc_log.h"
#include <string.h>

using namespace job;

// Internal signal (private)
enum : uint16_t {
    SIG_TELEM_TICK = rc::SIG_AO_MAX + 5
};

// ============================================================================
// AO State
// ============================================================================

// GCS connection state (IVP-62a)
enum class GcsState : uint8_t {
    kWaitingForGcs = 0,  // Send heartbeat only — no full telemetry
    kGcsConnected  = 1,  // GCS detected — full telemetry streaming
    kGcsLost       = 2,  // GCS heartbeat timeout — back to heartbeat-only
};

static constexpr uint32_t kGcsTimeoutMs = 5000;  // 5s without GCS heartbeat → lost

// Stage T Batch B prelim — airtime-scaled tracked-command retry timeout.
// History:
//   Pre-T6: hardcoded 3000 ms (chosen for SF7/BW125 + conservative).
//   IVP-T7: dropped to 500 ms flat (pinned for BW500 collision regime).
//   Batch B prelim: mutable — AO_Radio pushes a value derived from current
//   {SF, BW, max payload} airtime on every SET_RADIO_CONFIG apply. Keeps the
//   retry timeout sensible whether user selects BW500/SF7 (~200ms floor) or
//   BW125/SF12 long-range preset (~1000ms ceiling).
//   IVP-T14d wrap-up 2026-04-22: dropped seed to 250 ms (paired with retry
//   count bump to 8) to keep safety-class command round-trip under ~2 s
//   even in the half-duplex collision regime. This is a STOP-GAP tuning
//   pending a full CCSDS-compliant link-layer rework (see
//   docs/decisions/COP1_NOT_PURSUED.md for why COP-1 / FARM / FOP were
//   deferred, and the "Stage T latency re-baseline" whiteboard item).
// Seeded to 250 ms until first apply runs.
static uint32_t s_ack_retry_timeout_ms = 250U;

// Stage T Batch B IVP-T14d wrap-up — tracked-command retry budget.
// Aggressive retry for ALL tracked commands: latency matters more than
// airtime cost because (a) we send few commands, (b) half-duplex
// collisions with the vehicle's 5 Hz nav TX eat ~10-15% of retry slots
// blindly, so more retries directly buys more delivery probability, and
// (c) ABORT / ARM / DISARM want fast confirmation.
// 8 retries × 250 ms = ~2 s round-trip window before give-up.
// STOP-GAP: proper CCSDS TC-Layer + COP-1 would give us exactly-once
// delivery semantics without needing retry tuning at all. Parked until
// full CCSDS rework.
static constexpr uint8_t kAckMaxRetries = 8U;

struct TelemAo {
    QActive super;
    QTimeEvt tick_timer;    // 10Hz (every 10 ticks at 100Hz base)

    // Protocol state
    rc::CcsdsEncoder    ccsds_encoder;
    rc::MavlinkEncoder  mav_encoder;
    rc::TelemetryState  latest_telem;
    bool                telem_valid;
    uint8_t             rate_hz;
    uint32_t            interval_ms;
    uint32_t            last_tx_ms;
    uint32_t            last_heartbeat_ms;

    // GCS connection tracking (IVP-62a)
    GcsState            gcs_state;
    uint32_t            last_gcs_heartbeat_ms;

    // MAVLink RX parser (IVP-62b)
    rc::MavlinkRxState  mavlink_rx;
    uint8_t             gcs_heartbeat_count;    // Consecutive GCS heartbeats seen [JPL-1]
    int16_t             param_send_idx;         // >=0: send param at this index on next tick (-1=idle)

    // Station RX: latest decoded telemetry for CLI/WiFi access
    RxTelemSnapshot     rx_snapshot;
};

static TelemAo l_telemAo;

// Queue depth 8: non-blocking handlers (SIG_RADIO_TX posts, SIG_RADIO_RX decodes)
static QEvtPtr l_telemAoQueue[8];

// Forward declarations
static QState TelemAo_initial(TelemAo * const me, QEvt const * const e);
static QState TelemAo_running(TelemAo * const me, QEvt const * const e);

// ============================================================================
// Helpers
// ============================================================================

// USB CDC write — direct TinyUSB, bypasses stdio entirely.
// Drops data if buffer full (like lost radio packet). Never blocks.
// Safe to call from AO tick handlers.
static void usb_write_nonblocking(const uint8_t* buf, uint16_t len) {
#ifndef ROCKETCHIP_HOST_TEST
    if (!tud_cdc_connected()) { return; }
    if (tud_cdc_write_available() < len) { return; }  // Drop if won't fit
    tud_cdc_write(buf, len);
    // Don't call tud_cdc_write_flush() — it competes with stdio's tud_task().
    // Data flushes when stdio's background IRQ next calls tud_task().
    // With 1024B TX buffer, multiple frames accumulate and flush together.
#else
    (void)buf; (void)len;
#endif
}

static uint32_t now_ms() {
#ifndef ROCKETCHIP_HOST_TEST
    return to_ms_since_boot(get_absolute_time());
#else
    return 0;
#endif
}

// IVP-122: Vehicle-side pending ACK (queued for next TX opportunity)
static rc::ccsds::CommandAckPayload s_pending_ack = {};
static bool s_pending_ack_valid = false;
static uint16_t s_ack_seq = 0;

// Stage T IVP-T5.5: pending radio config storage lives in AO_Radio.
// This AO (Telemetry) owns ACK buffering and command dispatch; AO_Radio
// owns the actual radio reconfigure. After SET_RADIO_CONFIG validates,
// we call AO_Radio_set_pending_config() which queues the apply to fire
// on the next TxDone (which will be our outgoing ACK for this very
// command — per smell-test A.1 "apply on confirmed egress").

// IVP-122: Send pending command ACK before nav frame
static void send_pending_ack_if_any() {
    if (!s_pending_ack_valid) return;
    s_pending_ack_valid = false;

    uint8_t ack_buf[rc::ccsds::kCmdAckPacketLen];
    uint8_t ack_len = rc::ccsds_encode_cmd_ack(s_pending_ack, s_ack_seq, ack_buf);
    s_ack_seq = static_cast<uint16_t>((s_ack_seq + 1) & 0x3FFF);

    // Separate static event for ACK (can't reuse nav's txEvt — both in queue)
    // Stage T IVP-T5: RadioTxEvt.buf bumped to 256; uint8_t ack_len ≤ sizeof
    // check is now always-true. Removed redundant guard.
    static rc::RadioTxEvt s_ackTxEvt;
    s_ackTxEvt.super.sig = rc::SIG_RADIO_TX;
    s_ackTxEvt.super.refCtr_ = 0;
    memcpy(s_ackTxEvt.buf, ack_buf, ack_len);
    s_ackTxEvt.len = ack_len;
    QACTIVE_POST(AO_Radio, &s_ackTxEvt.super, AO_Telemetry);
}

static void encode_and_send(TelemAo* me) {
    if (!me->telem_valid) { return; }

    // ACK before rate-limit so it goes out ASAP, not on the next nav-frame tick.
    send_pending_ack_if_any();

    uint32_t t = now_ms();
    if (t - me->last_tx_ms < me->interval_ms) { return; }
    me->last_tx_ms = t;

    rc::EncodeResult result = {};
    if (rc::kDefaultRocketRadioConfig.protocol == rc::EncoderType::kMavlink) {
        uint8_t frame[128];
        uint16_t pos = 0;
        uint16_t len;
        len = me->mav_encoder.encode_heartbeat(me->latest_telem.flight_state, frame + pos);
        pos += len;
        len = me->mav_encoder.encode_attitude(me->latest_telem, t, frame + pos);
        pos += len;
        result.ok = (pos > 0);
        result.len = pos;
        memcpy(result.buf, frame, pos);
    } else {
        const rc::RadioConfig* cfg = AO_Radio_get_runtime_config();
        const bool just_changed = AO_Radio_consume_just_changed();
        me->ccsds_encoder.encode_nav_with_config(
            me->latest_telem, me->latest_telem.met_ms,
            *cfg, just_changed, result);
    }
    if (!result.ok || result.len == 0) { return; }

    // QV cooperative scheduling — static event safe (no concurrent access).
    static rc::RadioTxEvt txEvt;
    txEvt.super.sig = rc::SIG_RADIO_TX;
    txEvt.super.refCtr_ = 0;
    if (result.len > sizeof(txEvt.buf)) { return; }
    memcpy(txEvt.buf, result.buf, result.len);
    txEvt.len = static_cast<uint8_t>(result.len);

    QACTIVE_POST(AO_Radio, &txEvt.super, me);
}

// LoRa MAVLink RX — uses MAVLINK_COMM_2 (separate from USB on COMM_1)
// Stage T IVP-T5.5 sub 2b: SET_RADIO_CONFIG dispatcher. Returns ACK result.
// 3 validation gates (flight-state, whitelist, ±6 dB power delta) before
// queuing the new config. Separated out for JSF AV rule 1 compliance.
static uint8_t dispatch_set_radio_config(const mavlink_command_long_t& cmd) {
    uint16_t new_bw  = static_cast<uint16_t>(lroundf(cmd.param1));
    uint8_t  new_nav = static_cast<uint8_t> (lroundf(cmd.param2));
    uint8_t  new_sf  = static_cast<uint8_t> (lroundf(cmd.param3));
    uint8_t  new_cr  = static_cast<uint8_t> (lroundf(cmd.param4));
    uint8_t  new_pwr = static_cast<uint8_t> (lroundf(cmd.param5));

    if (!AO_FlightDirector_is_ground_state()) {
        return static_cast<uint8_t>(rc::ccsds::CmdAckResult::kDenied);
    }
    // Stage T Batch B prelim (2026-04-21): broader validation.
    // Previously only accepted presets from kRadioConfigTable. User flagged
    // that presets are just convenient defaults for the debug-menu digit
    // keys — the advanced-settings path should accept anything legal on
    // SX1276 hardware. See radio_config_table.h for both validators.
    if (!rc::radio_config_sx1276_legal(new_bw, new_nav, new_sf, new_cr, new_pwr)) {
        return static_cast<uint8_t>(rc::ccsds::CmdAckResult::kDenied);
    }
    const rc::RadioConfig* cur = AO_Radio_get_runtime_config();
    int pwr_delta = static_cast<int>(new_pwr) - static_cast<int>(cur->power_dbm);
    if (pwr_delta < 0) { pwr_delta = -pwr_delta; }
    if (pwr_delta > 6) {
        return static_cast<uint8_t>(rc::ccsds::CmdAckResult::kDenied);
    }

    rc::RadioConfig new_cfg = *cur;  // inherit mode/protocol
    new_cfg.nav_rate_hz      = new_nav;
    new_cfg.power_dbm        = new_pwr;
    new_cfg.spreading_factor = new_sf;
    new_cfg.bandwidth_khz    = new_bw;
    new_cfg.coding_rate      = new_cr;
    AO_Radio_set_pending_config(new_cfg);
    return static_cast<uint8_t>(rc::ccsds::CmdAckResult::kAccepted);
}

// Dispatch a single MAVLink COMMAND_LONG. Returns ack_result.
static uint8_t dispatch_command(TelemAo* me, const mavlink_command_long_t& cmd) {
    uint8_t ack_result = static_cast<uint8_t>(rc::ccsds::CmdAckResult::kAccepted);
    switch (cmd.command) {
    case MAV_CMD_COMPONENT_ARM_DISARM: {
        uint16_t sig = (cmd.param1 > 0.5f)
            ? static_cast<uint16_t>(rc::SIG_ARM)
            : static_cast<uint16_t>(rc::SIG_DISARM);
        AO_FlightDirector_dispatch_signal(sig);
        break;
    }
    case MAV_CMD_DO_FLIGHTTERMINATION:
        AO_FlightDirector_dispatch_signal(static_cast<uint16_t>(rc::SIG_ABORT));
        break;
    case MAV_CMD_USER_1: {
        // Stage L IVP-L5: GCS-initiated manual beacon. Static event per LL Entry 35.
        static QEvt s_beacon_cmd_evt;
        s_beacon_cmd_evt.sig = rc::SIG_BEACON_MANUAL;
        QActive_publish_(&s_beacon_cmd_evt, &me->super, me->super.prio);
        break;
    }
    case MAV_CMD_USER_2:
        // Stage T IVP-T5.5 sub 2b: SET_RADIO_CONFIG (3 gates inside).
        ack_result = dispatch_set_radio_config(cmd);
        break;
    case MAV_CMD_USER_3:
        // Sub 2e: QUERY_RADIO_CONFIG — read-only, echo fields populated below.
        break;
    default:
        ack_result = static_cast<uint8_t>(rc::ccsds::CmdAckResult::kDenied);
        break;
    }
    return ack_result;
}

// Build the pending CCSDS ACK for a dispatched command. Populates cfg-echo
// fields on QUERY responses (sub 2e).
static void stage_cmd_ack(const mavlink_command_long_t& cmd, uint8_t ack_result) {
    s_pending_ack.cmd_seq = static_cast<uint8_t>(cmd.confirmation);
    s_pending_ack.cmd_id  = cmd.command;
    s_pending_ack.result  = ack_result;
    s_pending_ack.reserved = 0;
    if (cmd.command == MAV_CMD_USER_3) {
        const rc::RadioConfig* cur = AO_Radio_get_runtime_config();
        s_pending_ack.cfg_bw_khz = cur->bandwidth_khz;
        s_pending_ack.cfg_nav_hz = cur->nav_rate_hz;
        s_pending_ack.cfg_sf     = cur->spreading_factor;
        s_pending_ack.cfg_cr     = cur->coding_rate;
    } else {
        s_pending_ack.cfg_bw_khz = 0;
        s_pending_ack.cfg_nav_hz = 0;
        s_pending_ack.cfg_sf     = 0;
        s_pending_ack.cfg_cr     = 0;
    }
    s_pending_ack_valid = true;
}

static void handle_parsed_mavlink(TelemAo* me, const mavlink_message_t& msg) {
    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        AO_Telemetry_notify_gcs_heartbeat();
    }
    if (msg.msgid != MAVLINK_MSG_ID_COMMAND_LONG) { return; }
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&msg, &cmd);
    uint8_t ack_result = dispatch_command(me, cmd);
    stage_cmd_ack(cmd, ack_result);
}

static void try_mavlink_rx(TelemAo* me, const uint8_t* buf, uint8_t len) {
    mavlink_message_t msg;
    mavlink_status_t status;

    for (uint8_t i = 0; i < len; ++i) {
        if (!mavlink_parse_char(MAVLINK_COMM_2, buf[i], &msg, &status)) {
            continue;
        }
        // Feed parser bookkeeping path — doesn't re-parse (COMM_2 consumed).
        rc::MavlinkRxResult result = {};
        rc::mavlink_rx_feed_byte(&me->mavlink_rx, buf[i],
                                  me->latest_telem.flight_state,
                                  now_ms(), &result);
        handle_parsed_mavlink(me, msg);
    }
}

// IVP-122: Station-side pending command state (ACK tracking)
// Stage T IVP-T5.5: +p1..p5 so resend_pending_cmd() replays the exact
// parameters the operator sent for multi-param commands (SET_RADIO_CONFIG).
static struct {
    bool pending;
    uint8_t seq;
    uint16_t cmd_id;
    uint32_t sent_ms;
    uint8_t retries_left;
    float p1;
    float p2;
    float p3;
    float p4;
    float p5;
} s_pending_cmd = {};

// Stage T Batch B IVP-T14c: last-command-result latch for dashboard display.
// Cleared by send_tracked_command; set on ACK or on retry exhaustion.
// Dashboard renders this for a brief hold window after result.
static struct {
    bool     valid;
    bool     ok;
    uint16_t cmd_id;
    uint16_t rtt_ms;
    uint32_t at_ms;
} s_last_cmd_result = {};

// ============================================================================
// Stage T Batch B T14b: retry instrumentation (always on).
//
// Per-command-type counters so we can distinguish collision regimes from
// sensitivity regimes and drive Batch C's T13 enable decision (post-field).
// Counters never reset during a session — cumulative since boot.
//
// Indexed by enum below rather than by raw MAV_CMD id to keep the table
// bounded. "Other" catches the long tail of infrequent/custom commands.
// ============================================================================
enum CmdClass : uint8_t {
    kCmdClassArm          = 0,   // MAV_CMD_COMPONENT_ARM_DISARM param1>0.5
    kCmdClassDisarm       = 1,   // MAV_CMD_COMPONENT_ARM_DISARM param1<0.5
    kCmdClassAbort        = 2,   // MAV_CMD_DO_FLIGHTTERMINATION
    kCmdClassSetConfig    = 3,   // MAV_CMD_USER_2
    kCmdClassQueryConfig  = 4,   // MAV_CMD_USER_3
    kCmdClassOther        = 5,   // fallthrough bucket
    kCmdClassCount        = 6,
};

struct RetryStats {
    uint32_t sent_count;
    uint32_t first_try_ack_count;  // ACK with retries_left == 3 (no retry used)
    uint32_t retry_ack_count;      // ACK after >=1 retry
    uint32_t fail_count;           // all retries exhausted
    uint32_t total_retries_used;   // sum of (3 - retries_left) over all acks+fails
};
static RetryStats s_retry_stats[kCmdClassCount] = {};

static CmdClass classify_tracked_cmd(uint16_t cmd_id, float p1) {
    if (cmd_id == MAV_CMD_COMPONENT_ARM_DISARM) {
        return p1 > 0.5f ? kCmdClassArm : kCmdClassDisarm;
    }
    if (cmd_id == MAV_CMD_DO_FLIGHTTERMINATION) { return kCmdClassAbort; }
    if (cmd_id == 31011U /* MAV_CMD_USER_2 */)   { return kCmdClassSetConfig; }
    if (cmd_id == 31012U /* MAV_CMD_USER_3 */)   { return kCmdClassQueryConfig; }
    return kCmdClassOther;
}

static const char* cmd_class_name(CmdClass c) {
    switch (c) {
    case kCmdClassArm:         return "ARM";
    case kCmdClassDisarm:      return "DISARM";
    case kCmdClassAbort:       return "ABORT";
    case kCmdClassSetConfig:   return "SET_CFG";
    case kCmdClassQueryConfig: return "QRY_CFG";
    case kCmdClassOther:       return "OTHER";
    case kCmdClassCount:       return "?";
    }
    return "?";
}


// Handle received packet from AO_Radio
// Handle a received CCSDS command ACK (IVP-122): match against the station-
// side pending command and clear if matched. Returns true if the buffer
// decoded as a CmdAck (caller returns immediately on true).
// Extracted from handle_rx_packet for JSF AV rule 1 compliance.
#ifdef ROCKETCHIP_JOB_STATION
// Sub 2c: ACK for SET_RADIO_CONFIG — station switches its own radio too.
static void station_on_set_radio_ack(float p1, float p2, float p3,
                                      float p4, float p5) {
    rc::RadioConfig new_cfg = *AO_Radio_get_runtime_config();
    new_cfg.bandwidth_khz    = static_cast<uint16_t>(lroundf(p1));
    new_cfg.nav_rate_hz      = static_cast<uint8_t> (lroundf(p2));
    new_cfg.spreading_factor = static_cast<uint8_t> (lroundf(p3));
    new_cfg.coding_rate      = static_cast<uint8_t> (lroundf(p4));
    new_cfg.power_dbm        = static_cast<uint8_t> (lroundf(p5));
    AO_Radio_set_pending_config(new_cfg);
    rc::rc_log("[CMD] station switching radio to BW=%u nav=%u SF=%u\n",
               static_cast<unsigned>(new_cfg.bandwidth_khz),
               static_cast<unsigned>(new_cfg.nav_rate_hz),
               static_cast<unsigned>(new_cfg.spreading_factor));
}

// Sub 2e: ACK for QUERY_RADIO_CONFIG — print echoed vehicle config.
static void station_on_query_ack(const rc::ccsds::CommandAckPayload& ack) {
    if (ack.cfg_bw_khz == 0) { return; }  // vehicle didn't populate
    rc::rc_log("[CMD] vehicle config: BW=%u nav=%u SF=%u CR=%u\n",
               static_cast<unsigned>(ack.cfg_bw_khz),
               static_cast<unsigned>(ack.cfg_nav_hz),
               static_cast<unsigned>(ack.cfg_sf),
               static_cast<unsigned>(ack.cfg_cr));
}
#endif

// Post-ACK bookkeeping: latch dashboard snapshot (IVP-T14c) + update
// retry stats (IVP-T14b). Extracted to keep try_handle_cmd_ack under
// the 60-line JSF AV Rule 1 cap.
static void record_ack_outcome(uint16_t matched_cmd, float matched_p1,
                               uint8_t retries_left_at_ack,
                               uint32_t rtt_ms, bool accepted) {
    // IVP-T14c dashboard latch.
    s_last_cmd_result.valid  = true;
    s_last_cmd_result.ok     = accepted;
    s_last_cmd_result.cmd_id = matched_cmd;
    s_last_cmd_result.rtt_ms = (rtt_ms > 0xFFFFU) ? 0xFFFFU
                                                   : static_cast<uint16_t>(rtt_ms);
    s_last_cmd_result.at_ms  = now_ms();

    // T14b retry stats.
    CmdClass cls = classify_tracked_cmd(matched_cmd, matched_p1);
    uint8_t retries_used = kAckMaxRetries - retries_left_at_ack;
    s_retry_stats[cls].total_retries_used += retries_used;
    if (retries_used == 0) {
        s_retry_stats[cls].first_try_ack_count++;
    } else {
        s_retry_stats[cls].retry_ack_count++;
    }
}

static bool try_handle_cmd_ack(const rc::RadioRxEvt* rxEvt) {
    rc::ccsds::CommandAckPayload ack{};
    if (!rc::ccsds_decode_cmd_ack(rxEvt->buf, rxEvt->len, ack)) {
        return false;
    }
#ifdef ROCKETCHIP_JOB_STATION
    // IVP-132a: fault-inject ACK suppression — forces station retry path.
    // R-25-exec step 6: flag is set only via fault_force_station_ack_suppress()
    // which checks test_mode_active() at entry; on production boots the flag
    // is always 0 and this branch is dead. (Runtime gate per Approach A.)
    if (g_fault_station_ack_suppress_remaining > 0) {
        g_fault_station_ack_suppress_remaining =
            g_fault_station_ack_suppress_remaining - 1;
        return true;
    }
#endif
    if (!s_pending_cmd.pending ||
        ack.cmd_seq != s_pending_cmd.seq ||
        ack.cmd_id != s_pending_cmd.cmd_id) {
        return true;
    }
    // Capture details before clearing — needed below for SET switch.
    const uint16_t matched_cmd = s_pending_cmd.cmd_id;
    const float matched_p1 = s_pending_cmd.p1;
    const float matched_p2 = s_pending_cmd.p2;
    const float matched_p3 = s_pending_cmd.p3;
    const float matched_p4 = s_pending_cmd.p4;
    const float matched_p5 = s_pending_cmd.p5;
    const uint8_t retries_left_at_ack = s_pending_cmd.retries_left;
    const uint32_t rtt_ms =
        static_cast<uint32_t>(now_ms() - s_pending_cmd.sent_ms);
    s_pending_cmd.pending = false;

    const bool accepted = (ack.result ==
        static_cast<uint8_t>(rc::ccsds::CmdAckResult::kAccepted));
    rc::rc_log("[CMD] %s (seq=%u)\n", accepted ? "ACK'd" : "DENIED", ack.cmd_seq);

    record_ack_outcome(matched_cmd, matched_p1, retries_left_at_ack,
                       rtt_ms, accepted);

#ifdef ROCKETCHIP_JOB_STATION
    if (accepted && matched_cmd == 31011 /* MAV_CMD_USER_2 */) {
        station_on_set_radio_ack(matched_p1, matched_p2, matched_p3,
                                  matched_p4, matched_p5);
    }
    if (accepted && matched_cmd == 31012 /* MAV_CMD_USER_3 */) {
        station_on_query_ack(ack);
    }
#else
    (void)matched_cmd;
    (void)matched_p1; (void)matched_p2; (void)matched_p3;
    (void)matched_p4; (void)matched_p5;
#endif
    return true;
}

// Dispatch a decoded Nav packet to the station-side output mode (MAVLink,
// CSV, ANSI, Menu). Extracted from handle_rx_packet for JSF AV rule 1
// compliance. Host-test builds skip the switch entirely.
#ifndef ROCKETCHIP_HOST_TEST
static void dispatch_nav_output(TelemAo* me,
                                 const rc::TelemetryState& telem,
                                 const rc::RadioRxEvt* rxEvt,
                                 uint16_t seq) {
    switch (AO_RCOS_get_output_mode()) {
    case StationOutputMode::kMavlink: {
        // MAVLink binary output on USB
        uint8_t frame[64];
        uint16_t len = 0;
        uint32_t t = now_ms();

        // 1 Hz heartbeat + SYS_STATUS
        if (t - me->last_heartbeat_ms >= 1000) {
            me->last_heartbeat_ms = t;
            len = me->mav_encoder.encode_heartbeat(telem.flight_state, frame);
            usb_write_nonblocking(frame, len);
            len = me->mav_encoder.encode_sys_status(telem, frame);
            usb_write_nonblocking(frame, len);
        }

        // ATTITUDE + GLOBAL_POSITION_INT per packet
        len = me->mav_encoder.encode_attitude(telem, t, frame);
        usb_write_nonblocking(frame, len);
        len = me->mav_encoder.encode_global_pos(telem, t, frame);
        usb_write_nonblocking(frame, len);
        break;
    }
    case StationOutputMode::kCsv:
        rc::rc_log("RX,%u,%d,%d\n",
                   static_cast<unsigned>(seq),
                   static_cast<int>(rxEvt->rssi),
                   static_cast<int>(rxEvt->snr));
        break;
    case StationOutputMode::kAnsi:
    case StationOutputMode::kMenu:
        // ANSI: rendered from main loop. Menu: output suppressed.
        break;
    }
}
#endif

static void handle_rx_packet(TelemAo* me, const rc::RadioRxEvt* rxEvt) {
#ifdef ROCKETCHIP_JOB_STATION
    // IVP-132a: fault-inject RX drop. R-25-exec step 6: flag is set
    // only via fault_force_station_rx_drop() which checks
    // test_mode_active() at entry; production boots leave the flag
    // at 0 and this branch is dead.
    if (g_fault_station_rx_drop_remaining > 0) {
        g_fault_station_rx_drop_remaining = g_fault_station_rx_drop_remaining - 1;
        return;
    }
#endif
    // Try CCSDS nav decode first (telemetry packets)
    rc::TelemetryState telem = {};
    uint16_t seq = 0;
    uint32_t met_ms = 0;
    rc::NavConfigEcho echo = {};
    if (!rc::ccsds_decode_nav(rxEvt->buf, rxEvt->len, telem, seq, met_ms, echo)) {
        // Not nav — try command ACK, then MAVLink command fallback
        if (try_handle_cmd_ack(rxEvt)) {
            return;
        }
        try_mavlink_rx(me, rxEvt->buf, rxEvt->len);
        return;
    }

    // Store for CLI/WiFi access
    me->rx_snapshot.telem = telem;
    me->rx_snapshot.met_ms = met_ms;
    me->rx_snapshot.seq = seq;
    me->rx_snapshot.valid = true;
    // Sub 2f: config echo. If this was an APID 0x001 legacy packet,
    // echo.bw_khz == 0 — leave last-known values in place so the
    // dashboard can still show "last seen" (only overwrite on real echoes).
    if (echo.bw_khz != 0) {
        me->rx_snapshot.echo_bw_khz       = echo.bw_khz;
        me->rx_snapshot.echo_nav_hz       = echo.nav_hz;
        me->rx_snapshot.echo_sf           = echo.sf;
        me->rx_snapshot.echo_cr           = echo.cr;
        me->rx_snapshot.echo_just_changed = echo.just_changed;
    }

#ifdef ROCKETCHIP_STAGE_T2_CHEAT
    // Stage T2 cheat-mode: fire pending command now that we know the vehicle
    // just completed TX and is entering kRxWindow. Decoupled from normal
    // X-press dispatch. Station build only.
    extern void stage_t2_fire_pending_if_any();
    stage_t2_fire_pending_if_any();
#endif

#ifndef ROCKETCHIP_HOST_TEST
    dispatch_nav_output(me, telem, rxEvt, seq);
#else
    (void)me;
#endif
}

// GCS connection state update (IVP-62a)
static void update_gcs_state(TelemAo* me, uint32_t t) {
    if (me->gcs_state == GcsState::kGcsConnected) {
        if (t - me->last_gcs_heartbeat_ms > kGcsTimeoutMs) {
            me->gcs_state = GcsState::kGcsLost;
        }
    }
}

// Direct USB MAVLink output for Vehicle mode (IVP-62a: heartbeat-only until GCS detected)
static void mavlink_direct_tick(TelemAo* me) {
#ifndef ROCKETCHIP_HOST_TEST
    if (AO_RCOS_get_output_mode() != StationOutputMode::kMavlink) { return; }
    if constexpr (kRadioModeRx) { return; }  // Station uses RX path
    if (!me->telem_valid) { return; }
    if (!stdio_usb_connected()) { return; }

    uint8_t frame[64];
    uint16_t len = 0;
    uint32_t t = now_ms();

    update_gcs_state(me, t);

    // Send deferred params one per tick (spread load, no blocking)
    if (me->param_send_idx >= 0 &&
        me->param_send_idx < static_cast<int16_t>(rc::mavlink_rx_param_count())) {
        const rc::MavParam* p = &rc::mavlink_rx_param_table()[me->param_send_idx];
        mavlink_message_t pmsg;
        mavlink_msg_param_value_pack(
            me->mav_encoder.system_id, me->mav_encoder.component_id, &pmsg,
            p->name, p->value, MAV_PARAM_TYPE_REAL32,
            static_cast<uint16_t>(rc::mavlink_rx_param_count()),
            static_cast<uint16_t>(me->param_send_idx));
        uint8_t pbuf[MAVLINK_MAX_PACKET_LEN];
        uint16_t plen = mavlink_msg_to_send_buffer(pbuf, &pmsg);
        usb_write_nonblocking(pbuf, plen);
        me->param_send_idx++;
        if (me->param_send_idx >= static_cast<int16_t>(rc::mavlink_rx_param_count())) {
            me->param_send_idx = -1;  // Done
        }
    }

    // Always send heartbeat at 1Hz (even before GCS detection)
    if (t - me->last_heartbeat_ms >= 1000) {
        me->last_heartbeat_ms = t;
        len = me->mav_encoder.encode_heartbeat(
            me->latest_telem.flight_state, frame);
        usb_write_nonblocking(frame, len);
        len = me->mav_encoder.encode_sys_status(me->latest_telem, frame);
        usb_write_nonblocking(frame, len);
        // No flush here — single fflush at end of tick
    }

    // Full telemetry — always stream when in MAVLink mode

    // 10 Hz ATTITUDE + GLOBAL_POSITION_INT
    len = me->mav_encoder.encode_attitude(me->latest_telem, t, frame);
    usb_write_nonblocking(frame, len);
    len = me->mav_encoder.encode_global_pos(me->latest_telem, t, frame);
    usb_write_nonblocking(frame, len);
    // No fflush — SDK background IRQ flushes CDC buffer automatically.
    // fflush blocks inside AO handler → queue overflow crash (LL Entry 32).
#else
    (void)me;
#endif
}

// ============================================================================
// State Handlers
// ============================================================================

static QState TelemAo_initial(TelemAo * const me, QEvt const * const e) {
    (void)e;

    me->ccsds_encoder.init();
    me->mav_encoder.init();
    me->telem_valid = false;
    // Output mode owned by AO_RCOS (station_output_mode.h)
    me->rate_hz = 5;
    me->interval_ms = 200;
    me->last_tx_ms = 0;
    me->last_heartbeat_ms = 0;
    me->gcs_state = GcsState::kWaitingForGcs;
    me->last_gcs_heartbeat_ms = 0;
    rc::mavlink_rx_init(&me->mavlink_rx, &me->mav_encoder);
    me->gcs_heartbeat_count = 0;
    me->param_send_idx = -1;

    // Subscribe to SIG_RADIO_RX from AO_Radio
    QActive_subscribe(&me->super, rc::SIG_RADIO_RX);
    QActive_subscribe(&me->super, rc::SIG_HEALTH_STATUS);  // IVP-105: health byte

    // 10Hz tick (every 10 ticks at 100Hz base)
    QTimeEvt_armX(&me->tick_timer, 10U, 10U);
    return Q_TRAN(&TelemAo_running);
}

static QState TelemAo_running(TelemAo * const me, QEvt const * const e) {
    switch (e->sig) {
    case SIG_TELEM_TICK: {
        // Vehicle TX: encode and post to AO_Radio
        if constexpr (!kRadioModeRx) {
            encode_and_send(me);
        }
        // Vehicle direct USB MAVLink (no radio)
        mavlink_direct_tick(me);
        return Q_HANDLED();
    }

    case rc::SIG_RADIO_RX: {
        const auto* rxEvt = reinterpret_cast<const rc::RadioRxEvt*>(e);
        handle_rx_packet(me, rxEvt);
        return Q_HANDLED();
    }

    default:
        break;
    }
    return Q_SUPER(&QHsm_top);
}

// ============================================================================
// Public API
// ============================================================================

QActive * const AO_Telemetry = &l_telemAo.super;

// CLI access — safe under QV cooperative scheduling
void AO_Telemetry_set_telem_snapshot(const rc::TelemetryState& telem) {
    l_telemAo.latest_telem = telem;
    l_telemAo.telem_valid = true;
}

// Legacy compat for vehicle mode (TX) — delegates to AO_RCOS
bool AO_Telemetry_get_mavlink_output() {
    return AO_RCOS_get_output_mode() == StationOutputMode::kMavlink;
}

void AO_Telemetry_toggle_mavlink() {
    if (AO_RCOS_get_output_mode() == StationOutputMode::kMavlink) {
        AO_RCOS_set_output_mode(StationOutputMode::kCsv);
    } else {
        AO_RCOS_set_output_mode(StationOutputMode::kMavlink);
    }
}

uint8_t AO_Telemetry_cycle_rate() {
    static constexpr uint8_t kRates[] = {2, 5, 10};
    for (uint8_t i = 0; i < 3; i++) {
        if (kRates[i] == l_telemAo.rate_hz) {
            uint8_t next = kRates[(i + 1) % 3];
            AO_Telemetry_set_rate(next);
            return next;
        }
    }
    AO_Telemetry_set_rate(5);
    return 5;
}

// Stage T Batch B prelim fix: wire nav_rate_hz into TX interval.
// Called from AO_Radio::apply_runtime_config() so SET_RADIO_CONFIG actually
// changes vehicle TX cadence. The radio_config_table whitelist is the policy
// gate for which rates are acceptable; this setter just computes the interval
// from whatever value arrived, with a guard against divide-by-zero and a
// sanity ceiling.
void AO_Telemetry_set_rate(uint8_t rate_hz) {
    if (rate_hz == 0) { rate_hz = 5; }
    if (rate_hz > 50) { rate_hz = 50; }  // sanity: 50 Hz ~ 20ms period
    l_telemAo.rate_hz = rate_hz;
    l_telemAo.interval_ms = 1000U / rate_hz;
}

// Stage T Batch B prelim fix: airtime-scaled ACK-retry timeout.
// Called from AO_Radio::apply_runtime_config() with a value derived from
// current {SF, BW, max payload} airtime. Seeds at 500 ms until first apply.
void AO_Telemetry_set_ack_retry_timeout_ms(uint32_t timeout_ms) {
    if (timeout_ms < 100U)  { timeout_ms = 100U; }
    if (timeout_ms > 5000U) { timeout_ms = 5000U; }
    s_ack_retry_timeout_ms = timeout_ms;
}

const RxTelemSnapshot* AO_Telemetry_get_rx_state() {
    return &l_telemAo.rx_snapshot;
}

// IVP-62c: encode + send MAVLink COMMAND_LONG over LoRa
void AO_Telemetry_send_command(uint16_t command, float p1, float p2,
                               float p3, float p4, float p5,
                               float p6, float p7) {
#ifndef ROCKETCHIP_HOST_TEST
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        255, 0,  // GCS sysid=255, compid=0
        &msg,
        1, 1,    // Target sysid=1, compid=1 (vehicle)
        command,
        0,       // Confirmation
        p1, p2, p3, p4, p5, p6, p7);

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    static rc::RadioTxEvt txEvt;
    txEvt.super.sig = rc::SIG_RADIO_TX;
    txEvt.super.refCtr_ = 0;
    if (len <= sizeof(txEvt.buf)) {
        memcpy(txEvt.buf, buf, len);
        txEvt.len = static_cast<uint8_t>(len);
        QACTIVE_POST(AO_Radio, &txEvt.super, &l_telemAo.super);
    }
#else
    (void)command; (void)p1; (void)p2; (void)p3; (void)p4; (void)p5; (void)p6; (void)p7;
#endif
}

// IVP-122: Send a tracked command — populates pending-cmd state for ACK tracking.
static uint8_t s_cmd_seq = 0;

// Stage T Batch B T14a: is this command class safety-critical? Per Round 2
// council consensus #5, ARM (COMPONENT_ARM_DISARM with param1 > 0.5) and
// flight-termination (ABORT) are safety-class — excluded from newest-wins
// dedupe so every deliberate operator press is preserved as its own
// tracked command. DISARM is NOT safety-class (DISARM is safer than ARM;
// mashing DISARM is a harmless noop if already disarmed).
static bool is_tracked_command_safety_class(uint16_t cmd_id, float p1) {
    if (cmd_id == MAV_CMD_DO_FLIGHTTERMINATION) {
        return true;
    }
    if (cmd_id == MAV_CMD_COMPONENT_ARM_DISARM && p1 > 0.5f) {
        return true;  // ARM only; DISARM (p1 < 0.5) is fine to dedupe.
    }
    return false;
}

#ifndef ROCKETCHIP_HOST_TEST
// Encode and TX a MAVLink COMMAND_LONG with the given seq/params.
static void tx_tracked_command_wire(uint16_t command, uint8_t seq, float p1,
                                    float p2, float p3, float p4, float p5) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        255, 0, &msg, 1, 1,
        command, seq,
        p1, p2, p3, p4, p5, 0, 0);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    static rc::RadioTxEvt txEvt;
    txEvt.super.sig = rc::SIG_RADIO_TX;
    txEvt.super.refCtr_ = 0;
    if (len <= sizeof(txEvt.buf)) {
        memcpy(txEvt.buf, buf, len);
        txEvt.len = static_cast<uint8_t>(len);
        QACTIVE_POST(AO_Radio, &txEvt.super, &l_telemAo.super);
    }
}

// Populate s_pending_cmd + params (used by both fresh-send and dedupe-replace).
static void populate_pending(uint16_t command, uint8_t seq, float p1,
                             float p2, float p3, float p4, float p5) {
    s_pending_cmd.pending = true;
    s_pending_cmd.seq = seq;
    s_pending_cmd.cmd_id = command;
    s_pending_cmd.sent_ms = to_ms_since_boot(get_absolute_time());
    s_pending_cmd.retries_left = kAckMaxRetries;
    s_pending_cmd.p1 = p1;
    s_pending_cmd.p2 = p2;
    s_pending_cmd.p3 = p3;
    s_pending_cmd.p4 = p4;
    s_pending_cmd.p5 = p5;

    // T14b: count a new send. Dedupe-replace path also reuses populate_pending
    // — but the command is semantically "a new send" (new seq, fresh ACK
    // window), so counting it is correct.
    CmdClass c = classify_tracked_cmd(command, p1);
    s_retry_stats[c].sent_count++;

    // IVP-T14c: clear stale last-result latch on new send so dashboard
    // transitions directly to "Try 0/N (pending)" without stale ACK text.
    s_last_cmd_result.valid = false;
}
#endif

void AO_Telemetry_send_tracked_command(uint16_t command, float p1,
                                       float p2, float p3,
                                       float p4, float p5) {
#ifndef ROCKETCHIP_HOST_TEST
    // Stage T Batch B T14a: MAV_CMD dedupe newest-wins. If a command of the
    // same cmd_id is already pending and the class is NOT safety-critical,
    // replace in-place (bump seq, overwrite params, reset retries). Prevents
    // operator-mashing self-collision: N rapid presses → 1 pending command
    // with most-recent params, seq monotonic.
    //
    // Safety-class commands (ARM, ABORT) bypass dedupe — every deliberate
    // press is preserved as its own tracked command with its own ACK window.
    if (s_pending_cmd.pending &&
        s_pending_cmd.cmd_id == command &&
        !is_tracked_command_safety_class(command, p1)) {
        uint8_t seq = s_cmd_seq++;
        populate_pending(command, seq, p1, p2, p3, p4, p5);
        tx_tracked_command_wire(command, seq, p1, p2, p3, p4, p5);
        return;
    }

    // Fresh send: allocate seq, populate pending, TX on wire.
    uint8_t seq = s_cmd_seq++;
    populate_pending(command, seq, p1, p2, p3, p4, p5);
    tx_tracked_command_wire(command, seq, p1, p2, p3, p4, p5);
#else
    (void)command; (void)p1; (void)p2; (void)p3; (void)p4; (void)p5;
#endif
}

bool AO_Telemetry_is_cmd_pending() {
    return s_pending_cmd.pending;
}

// Stage T Batch B IVP-T14c: dashboard-visible snapshot of pending/recent-ack
// state. Pure snapshot — Core 0 cooperative dispatch, no locks needed.
void AO_Telemetry_get_pending_cmd_status(PendingCmdStatus* out) {
    if (out == nullptr) { return; }
    out->pending      = s_pending_cmd.pending;
    out->cmd_id       = s_pending_cmd.cmd_id;
    out->retries_used = static_cast<uint8_t>(
        kAckMaxRetries - s_pending_cmd.retries_left);
    out->max_retries  = kAckMaxRetries;
    out->last_result_valid = s_last_cmd_result.valid;
    out->last_result_ok    = s_last_cmd_result.ok;
    out->last_cmd_id       = s_last_cmd_result.cmd_id;
    out->last_rtt_ms       = s_last_cmd_result.rtt_ms;
    out->last_result_ms    = s_last_cmd_result.at_ms;
}

// Stage T Batch B IVP-T14b: retry-stats snapshot for CLI/diag.
uint8_t AO_Telemetry_get_retry_stats(CmdRetryStatsLine* rows, uint8_t max_rows) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < static_cast<uint8_t>(kCmdClassCount) && n < max_rows; ++i) {
        rows[n].name               = cmd_class_name(static_cast<CmdClass>(i));
        rows[n].sent               = s_retry_stats[i].sent_count;
        rows[n].first_try          = s_retry_stats[i].first_try_ack_count;
        rows[n].retry_rescued      = s_retry_stats[i].retry_ack_count;
        rows[n].failed             = s_retry_stats[i].fail_count;
        rows[n].total_retries_used = s_retry_stats[i].total_retries_used;
        n++;
    }
    return n;
}

// Internal: re-send pending command with same seq (for retries)
static void resend_pending_cmd() {
#ifndef ROCKETCHIP_HOST_TEST
    mavlink_message_t msg;
    // Stage T IVP-T5.5: replay ALL cached params (was p1-only, hard-coded
    // for ARM). Multi-param commands like SET_RADIO_CONFIG depend on this.
    mavlink_msg_command_long_pack(
        255, 0, &msg, 1, 1,
        s_pending_cmd.cmd_id,
        s_pending_cmd.seq,  // Same seq as original
        s_pending_cmd.p1, s_pending_cmd.p2, s_pending_cmd.p3,
        s_pending_cmd.p4, s_pending_cmd.p5, 0, 0);

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    static rc::RadioTxEvt txEvt;
    txEvt.super.sig = rc::SIG_RADIO_TX;
    txEvt.super.refCtr_ = 0;
    if (len <= sizeof(txEvt.buf)) {
        memcpy(txEvt.buf, buf, len);
        txEvt.len = static_cast<uint8_t>(len);
        QACTIVE_POST(AO_Radio, &txEvt.super, &l_telemAo.super);
    }
    s_pending_cmd.sent_ms = to_ms_since_boot(get_absolute_time());
#endif
}

void AO_Telemetry_cmd_retry_tick(uint32_t now_ms) {
#ifndef ROCKETCHIP_HOST_TEST
    if (!s_pending_cmd.pending) return;

    uint32_t elapsed = now_ms - s_pending_cmd.sent_ms;
    if (elapsed >= s_ack_retry_timeout_ms) {
        if (s_pending_cmd.retries_left > 0) {
            s_pending_cmd.retries_left--;
            rc::rc_log("[CMD] Retry %u/%u (seq=%u)\n",
                       kAckMaxRetries - s_pending_cmd.retries_left,
                       kAckMaxRetries, s_pending_cmd.seq);
            resend_pending_cmd();
        } else {
            // T14b: record fail + retries used (all kAckMaxRetries).
            CmdClass cls = classify_tracked_cmd(s_pending_cmd.cmd_id,
                                                 s_pending_cmd.p1);
            s_retry_stats[cls].fail_count++;
            s_retry_stats[cls].total_retries_used += kAckMaxRetries;

            // IVP-T14c: latch failure for dashboard display.
            s_last_cmd_result.valid  = true;
            s_last_cmd_result.ok     = false;
            s_last_cmd_result.cmd_id = s_pending_cmd.cmd_id;
            s_last_cmd_result.rtt_ms = 0;
            s_last_cmd_result.at_ms  = now_ms;

            s_pending_cmd.pending = false;
            rc::rc_log("[CMD] No ACK after %u retries\n",
                       static_cast<unsigned>(kAckMaxRetries));
        }
    }
#else
    (void)now_ms;
#endif
}

// IVP-62b: feed USB input byte to MAVLink parser for GCS detection + commands.
// Uses COMM_0 (dedicated to USB input) — not COMM_1 (mavlink_rx module) which
// gets corrupted by CLI bytes. CRLF translation disabled so binary frames parse correctly.
void AO_Telemetry_feed_usb_byte(uint8_t byte) {
#ifndef ROCKETCHIP_HOST_TEST
    mavlink_message_t msg;
    mavlink_status_t status;

    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
        // GCS heartbeat detection
        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT && msg.sysid != 0) {
            AO_Telemetry_notify_gcs_heartbeat();
            l_telemAo.gcs_heartbeat_count++;
            AO_RCOS_set_output_mode(StationOutputMode::kMavlink);
        }
        // Param request — defer to tick handler
        if (msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST &&
            l_telemAo.param_send_idx < 0) {
            l_telemAo.param_send_idx = 0;
        }
        // Command dispatch (ARM/DISARM/ABORT from QGC)
        if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
            mavlink_command_long_t cmd;
            mavlink_msg_command_long_decode(&msg, &cmd);
            if (cmd.command == MAV_CMD_COMPONENT_ARM_DISARM) {
                uint16_t sig = (cmd.param1 > 0.5f)
                    ? static_cast<uint16_t>(rc::SIG_ARM)
                    : static_cast<uint16_t>(rc::SIG_DISARM);
                AO_FlightDirector_dispatch_signal(sig);
            } else if (cmd.command == MAV_CMD_DO_FLIGHTTERMINATION) {
                AO_FlightDirector_dispatch_signal(
                    static_cast<uint16_t>(rc::SIG_ABORT));
            }
        }
    }
#else
    (void)byte;
#endif
}

bool AO_Telemetry_is_gcs_connected() {
    return l_telemAo.gcs_state == GcsState::kGcsConnected;
}

// IVP-62a: notify that a GCS heartbeat was received (USB or LoRa)
void AO_Telemetry_notify_gcs_heartbeat() {
    l_telemAo.last_gcs_heartbeat_ms = now_ms();
    if (l_telemAo.gcs_state != GcsState::kGcsConnected) {
        l_telemAo.gcs_state = GcsState::kGcsConnected;
    }
}

void AO_Telemetry_start(uint8_t prio) {
    QActive_ctor(&l_telemAo.super,
                 Q_STATE_CAST(&TelemAo_initial));

    QTimeEvt_ctorX(&l_telemAo.tick_timer, &l_telemAo.super,
                   SIG_TELEM_TICK, 0U);

    memset(&l_telemAo.latest_telem, 0, sizeof(l_telemAo.latest_telem));
    l_telemAo.telem_valid = false;
    memset(&l_telemAo.rx_snapshot, 0, sizeof(l_telemAo.rx_snapshot));

    QActive_start(&l_telemAo.super,
                  Q_PRIO(prio, 0U),
                  l_telemAoQueue,
                  Q_DIM(l_telemAoQueue),
                  (void *)0, 0U,
                  (void *)0);
}
