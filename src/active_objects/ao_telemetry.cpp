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
#include "rocketchip/radio_config_table.h"          // SET_RADIO_CONFIG whitelist
#include "rocketchip/job.h"
#include "starcom_adapt/sc_air.h"
#ifdef ROCKETCHIP_USE_STARCOM
#include "starcom_adapt/byte_pump.h"
#include "starcom_adapt/nav_sdu.h"
#include "starcom_adapt/cmd_sdu.h"
#endif
#include "flight_director/mission_profile_data.h"  // kDefaultRocketRadioConfig
#include <math.h>                                   // lroundf (float→int for SET_RADIO_CONFIG)
#ifdef ROCKETCHIP_JOB_STATION
#include "safety/station_fault_inject.h"  // runtime-gated; flags are 0 unless armed
#endif

#ifndef ROCKETCHIP_HOST_TEST
#include "pico/time.h"
#include "pico/stdio_usb.h"
#include "tusb.h"
#endif

#include "rocketchip/rc_log.h"
#include <string.h>
#include <span>

using namespace job;

// Internal signal (private)
enum : uint16_t {
    SIG_TELEM_TICK = rc::SIG_AO_MAX + 5
};

// ============================================================================
// AO State
// ============================================================================

// GCS connection state
enum class GcsState : uint8_t {
    kWaitingForGcs = 0,  // Send heartbeat only — no full telemetry
    kGcsConnected  = 1,  // GCS detected — full telemetry streaming
    kGcsLost       = 2,  // GCS heartbeat timeout — back to heartbeat-only
};

static constexpr uint32_t kGcsTimeoutMs = 5000;  // 5s without GCS heartbeat → lost

// Airtime-scaled tracked-command retry timeout. AO_Radio pushes a value
// from {SF, BW, max payload} on every SET_RADIO_CONFIG apply. Seed 250 ms
// until first apply. 8 × 250 ms ≈ 2 s give-up window.
static uint32_t g_ackRetryTimeoutMs = 250U;

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

    // GCS connection tracking
    GcsState            gcs_state;
    uint32_t            last_gcs_heartbeat_ms;

    // MAVLink RX parser
    rc::MavlinkRxState  mavlink_rx;
    uint8_t             gcs_heartbeat_count;    // Consecutive GCS heartbeats seen [JPL-1]
    int16_t             param_send_idx;         // >=0: send param at this index on next tick (-1=idle)

    // Station RX: latest decoded telemetry for CLI/WiFi access
    RxTelemSnapshot     rx_snapshot;
};

static TelemAo g_telemAo;
#ifdef ROCKETCHIP_USE_STARCOM
static rc::starcom_adapt::BytePump g_pump;
#endif

// Queue depth 8: non-blocking handlers (SIG_RADIO_TX posts, SIG_RADIO_RX decodes)
static QEvtPtr g_telemAoQueue[8];

// Forward declarations
static QState telem_ao_initial(TelemAo * const me, QEvt const * const e);
static QState telem_ao_running(TelemAo * const me, QEvt const * const e);

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

// Vehicle-side pending ACK (queued for next TX opportunity)
static rc::ccsds::CommandAckPayload g_pendingAck = {};
static bool g_pendingAckValid = false;
#ifndef ROCKETCHIP_USE_STARCOM
static uint16_t g_ackSeq = 0;
#endif

// Radio reconfigure lives in AO_Radio. After SET_RADIO_CONFIG validates,
// AO_Radio_set_pending_config() applies on the next TxDone (outgoing ACK).

// Send pending command ACK before nav frame
#ifdef ROCKETCHIP_USE_STARCOM
// One PLTU per call. COP-P resends unacked seq if no peer PLCW; draining
// a window of frames every 10 Hz tick floods the half-duplex radio.
static constexpr uint8_t kStarcomDrainCap = 1;

static void starcom_post_pltu(std::span<const std::byte> octets) {
    if (octets.empty() || octets.size() > sizeof(rc::RadioTxEvt::buf)) {
        return;
    }
    static rc::RadioTxEvt g_txEvtSc;
    g_txEvtSc.super.sig = rc::SIG_RADIO_TX;
    g_txEvtSc.super.refCtr_ = 0;
    memcpy(g_txEvtSc.buf, octets.data(), octets.size());
    g_txEvtSc.len = static_cast<uint8_t>(octets.size());
    QACTIVE_POST(AO_Radio, &g_txEvtSc.super, AO_Telemetry);
}

static void starcom_drain_to_radio() {
    for (uint8_t i = 0; i < kStarcomDrainCap; ++i) {
        std::byte buf[rc::starcom_adapt::kAirMtu];
        const auto n = rc::starcom_adapt::pump_bytes_to_send(
            g_pump, std::span<std::byte>(buf, sizeof(buf)));
        if (!n.has_value() || *n == 0) {
            return;
        }
        starcom_post_pltu(std::span<const std::byte>(buf, *n));
    }
}
#endif

static void send_pending_ack_if_any() {
    if (!g_pendingAckValid) return;
    g_pendingAckValid = false;

#ifdef ROCKETCHIP_USE_STARCOM
    std::byte pkt[6u + rc::kAckSduUserBytes];
    const auto n = rc::starcom_adapt::pump_pack_ack_packet(
        pkt, g_pendingAck);
    if (n.has_value() && *n > 0) {
        (void)rc::starcom_adapt::pump_submit_sdu(
            g_pump, std::span<const std::byte>(pkt, *n), false);
        starcom_drain_to_radio();
    }
    return;
#else
    uint8_t ack_buf[rc::ccsds::kCmdAckPacketLen];
    uint8_t ack_len = rc::ccsds_encode_cmd_ack(g_pendingAck, g_ackSeq, ack_buf);
    g_ackSeq = static_cast<uint16_t>((g_ackSeq + 1) & 0x3FFF);

    // Separate static event for ACK (can't reuse nav's txEvt — both in queue)
    static rc::RadioTxEvt g_ackTxEvt;
    g_ackTxEvt.super.sig = rc::SIG_RADIO_TX;
    g_ackTxEvt.super.refCtr_ = 0;
    memcpy(g_ackTxEvt.buf, ack_buf, ack_len);
    g_ackTxEvt.len = ack_len;
    QACTIVE_POST(AO_Radio, &g_ackTxEvt.super, AO_Telemetry);
#endif
}

static void encode_and_send(TelemAo* me) {
    if (!me->telem_valid) { return; }

    // ACK before rate-limit so it goes out ASAP, not on the next nav-frame tick.
    send_pending_ack_if_any();

    uint32_t t = now_ms();
    if (t - me->last_tx_ms < me->interval_ms) { return; }
    me->last_tx_ms = t;

#ifdef ROCKETCHIP_USE_STARCOM
    if (rc::kDefaultRocketRadioConfig.protocol != rc::EncoderType::kMavlink) {
        std::byte pkt[6u + rc::kNavSduUserBytes];
        const auto n = rc::starcom_adapt::pump_pack_nav_packet(
            pkt, me->latest_telem);
        if (!n.has_value() || *n == 0) { return; }
        (void)rc::starcom_adapt::pump_submit_sdu(
            g_pump, std::span<const std::byte>(pkt, *n), false);
        starcom_drain_to_radio();
        return;
    }
#endif

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
    static rc::RadioTxEvt g_txEvt;
    g_txEvt.super.sig = rc::SIG_RADIO_TX;
    g_txEvt.super.refCtr_ = 0;
    if (result.len > sizeof(g_txEvt.buf)) { return; }
    memcpy(g_txEvt.buf, result.buf, result.len);
    g_txEvt.len = static_cast<uint8_t>(result.len);

    QACTIVE_POST(AO_Radio, &g_txEvt.super, me);
}

// LoRa MAVLink RX — uses MAVLINK_COMM_2 (separate from USB on COMM_1)
// SET_RADIO_CONFIG dispatcher. Returns ACK result.
// 3 gates (flight-state, SX1276-legal, ±6 dB power delta) before queue.
static uint8_t dispatch_set_radio_config(const mavlink_command_long_t& cmd) {
    uint16_t new_bw  = static_cast<uint16_t>(lroundf(cmd.param1));
    uint8_t  new_nav = static_cast<uint8_t> (lroundf(cmd.param2));
    uint8_t  new_sf  = static_cast<uint8_t> (lroundf(cmd.param3));
    uint8_t  new_cr  = static_cast<uint8_t> (lroundf(cmd.param4));
    uint8_t  new_pwr = static_cast<uint8_t> (lroundf(cmd.param5));

    if (!AO_FlightDirector_is_ground_state()) {
        return static_cast<uint8_t>(rc::ccsds::CmdAckResult::kDenied);
    }
    // Presets are debug-menu defaults; advanced path accepts SX1276-legal.
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
        uint16_t sig = (cmd.param1 > 0.5F)
            ? static_cast<uint16_t>(rc::SIG_ARM)
            : static_cast<uint16_t>(rc::SIG_DISARM);
        AO_FlightDirector_dispatch_signal(sig);
        break;
    }
    case MAV_CMD_DO_FLIGHTTERMINATION:
        AO_FlightDirector_dispatch_signal(static_cast<uint16_t>(rc::SIG_ABORT));
        break;
    case MAV_CMD_USER_1: {
        // GCS-initiated manual beacon. Static event (QP publish must outlive call).
        static QEvt g_beaconCmdEvt;
        g_beaconCmdEvt.sig = rc::SIG_BEACON_MANUAL;
        QActive_publish_(&g_beaconCmdEvt, &me->super, me->super.prio);
        break;
    }
    case MAV_CMD_USER_2:
        // SET_RADIO_CONFIG (3 gates inside).
        ack_result = dispatch_set_radio_config(cmd);
        break;
    case MAV_CMD_USER_3:
        // QUERY_RADIO_CONFIG — read-only, echo fields populated below.
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
    g_pendingAck.cmd_seq = static_cast<uint8_t>(cmd.confirmation);
    g_pendingAck.cmd_id  = cmd.command;
    g_pendingAck.result  = ack_result;
    g_pendingAck.reserved = 0;
    if (cmd.command == MAV_CMD_USER_3) {
        const rc::RadioConfig* cur = AO_Radio_get_runtime_config();
        g_pendingAck.cfg_bw_khz = cur->bandwidth_khz;
        g_pendingAck.cfg_nav_hz = cur->nav_rate_hz;
        g_pendingAck.cfg_sf     = cur->spreading_factor;
        g_pendingAck.cfg_cr     = cur->coding_rate;
    } else {
        g_pendingAck.cfg_bw_khz = 0;
        g_pendingAck.cfg_nav_hz = 0;
        g_pendingAck.cfg_sf     = 0;
        g_pendingAck.cfg_cr     = 0;
    }
    g_pendingAckValid = true;
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
        if (mavlink_parse_char(MAVLINK_COMM_2, buf[i], &msg, &status)) {
            // Feed parser bookkeeping path — doesn't re-parse (COMM_2 consumed).
            rc::MavlinkRxResult result = {};
            rc::mavlink_rx_feed_byte(&me->mavlink_rx, buf[i],
                                      me->latest_telem.flight_state,
                                      now_ms(), &result);
            handle_parsed_mavlink(me, msg);
        }
    }
}

// Station-side pending command. p1..p5 replay SET_RADIO_CONFIG on retry.
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
} g_pendingCmd = {};

// Last-command-result latch for dashboard. Cleared on send; set on ACK
// or retry exhaustion. Dashboard holds the result for a brief window.
static struct {
    bool     valid;
    bool     ok;
    uint16_t cmd_id;
    uint16_t rtt_ms;
    uint32_t at_ms;
} g_lastCmdResult = {};

// ============================================================================
// Retry counters (cumulative since boot). Indexed by CmdClass, not raw
// MAV_CMD, so the table stays bounded. "Other" is the long tail.
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
static RetryStats g_retryStats[kCmdClassCount] = {};

static CmdClass classify_tracked_cmd(uint16_t cmd_id, float p1) {
    if (cmd_id == MAV_CMD_COMPONENT_ARM_DISARM) {
        return p1 > 0.5F ? kCmdClassArm : kCmdClassDisarm;
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
// Match a CCSDS command ACK against the station pending command.
#ifdef ROCKETCHIP_JOB_STATION
// ACK for SET_RADIO_CONFIG — station switches its own radio too.
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

// ACK for QUERY_RADIO_CONFIG — print echoed vehicle config.
static void station_on_query_ack(const rc::ccsds::CommandAckPayload& ack) {
    if (ack.cfg_bw_khz == 0) { return; }  // vehicle didn't populate
    rc::rc_log("[CMD] vehicle config: BW=%u nav=%u SF=%u CR=%u\n",
               static_cast<unsigned>(ack.cfg_bw_khz),
               static_cast<unsigned>(ack.cfg_nav_hz),
               static_cast<unsigned>(ack.cfg_sf),
               static_cast<unsigned>(ack.cfg_cr));
}
#endif

// Post-ACK: dashboard latch + retry stats. Split out for function size.
static void record_ack_outcome(uint16_t matched_cmd, float matched_p1,
                               uint8_t retries_left_at_ack,
                               uint32_t rtt_ms, bool accepted) {
    // Dashboard latch.
    g_lastCmdResult.valid  = true;
    g_lastCmdResult.ok     = accepted;
    g_lastCmdResult.cmd_id = matched_cmd;
    g_lastCmdResult.rtt_ms = (rtt_ms > 0xFFFFU) ? 0xFFFFU
                                                   : static_cast<uint16_t>(rtt_ms);
    g_lastCmdResult.at_ms  = now_ms();

    // Retry stats.
    CmdClass cls = classify_tracked_cmd(matched_cmd, matched_p1);
    uint8_t retries_used = kAckMaxRetries - retries_left_at_ack;
    g_retryStats[cls].total_retries_used += retries_used;
    if (retries_used == 0) {
        g_retryStats[cls].first_try_ack_count++;
    } else {
        g_retryStats[cls].retry_ack_count++;
    }
}

static bool apply_cmd_ack_payload(const rc::ccsds::CommandAckPayload& ack) {
#ifdef ROCKETCHIP_JOB_STATION
    // Inject ACK suppression (runtime-gated; 0 on production boots).
    if (g_fault_station_ack_suppress_remaining > 0) {
        g_fault_station_ack_suppress_remaining =
            g_fault_station_ack_suppress_remaining - 1;
        return true;
    }
#endif
    if (!g_pendingCmd.pending ||
        ack.cmd_seq != g_pendingCmd.seq ||
        ack.cmd_id != g_pendingCmd.cmd_id) {
        return true;
    }
    // Capture details before clearing — needed below for SET switch.
    const uint16_t matched_cmd = g_pendingCmd.cmd_id;
    const float matched_p1 = g_pendingCmd.p1;
    const float matched_p2 = g_pendingCmd.p2;
    const float matched_p3 = g_pendingCmd.p3;
    const float matched_p4 = g_pendingCmd.p4;
    const float matched_p5 = g_pendingCmd.p5;
    const uint8_t retries_left_at_ack = g_pendingCmd.retries_left;
    const uint32_t rtt_ms =
        static_cast<uint32_t>(now_ms() - g_pendingCmd.sent_ms);
    g_pendingCmd.pending = false;

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

static bool try_handle_cmd_ack(const rc::RadioRxEvt* rx_evt) {
    rc::ccsds::CommandAckPayload ack{};
    if (!rc::ccsds_decode_cmd_ack(rx_evt->buf, rx_evt->len, ack)) {
        return false;
    }
    return apply_cmd_ack_payload(ack);
}

// Dispatch a decoded Nav packet to the station-side output mode (MAVLink,
// CSV, ANSI, Menu). Extracted from handle_rx_packet for JSF AV rule 1
// compliance. Host-test builds skip the switch entirely.
#ifndef ROCKETCHIP_HOST_TEST
static void dispatch_nav_output(TelemAo* me,
                                 const rc::TelemetryState& telem,
                                 const rc::RadioRxEvt* rx_evt,
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
                   static_cast<int>(rx_evt->rssi),
                   static_cast<int>(rx_evt->snr));
        break;
    case StationOutputMode::kAnsi:
    case StationOutputMode::kMenu:
        // ANSI: rendered from main loop. Menu: output suppressed.
        break;
    }
}
#endif

#ifdef ROCKETCHIP_USE_STARCOM
static bool starcom_handle_sdu(TelemAo* me, std::span<const std::byte> sdu) {
    const auto pkt = starcom::ccsds::decodeSpacePacket(sdu);
    if (!pkt) {
        return false;
    }
    if (pkt->fields.apid == rc::starcom_adapt::kNavApid) {
        rc::TelemetryState telem = {};
        uint8_t user[rc::kNavSduUserBytes];
        if (pkt->data.size() != rc::kNavSduUserBytes) {
            return false;
        }
        for (std::size_t i = 0; i < rc::kNavSduUserBytes; ++i) {
            user[i] = static_cast<uint8_t>(pkt->data[i]);
        }
        if (!rc::unpack_nav_sdu_user(user, rc::kNavSduUserBytes, &telem)) {
            return false;
        }
        me->rx_snapshot.telem = telem;
        me->rx_snapshot.valid = true;
        return true;
    }
    if (pkt->fields.apid != rc::starcom_adapt::kCmdApid) {
        return false;
    }
    if (pkt->fields.telecommand) {
        uint16_t cmd_id = 0;
        uint8_t seq = 0;
        float p1 = 0, p2 = 0, p3 = 0, p4 = 0, p5 = 0;
        uint8_t user[rc::kCmdSduUserBytes];
        if (pkt->data.size() != rc::kCmdSduUserBytes) {
            return false;
        }
        for (std::size_t i = 0; i < rc::kCmdSduUserBytes; ++i) {
            user[i] = static_cast<uint8_t>(pkt->data[i]);
        }
        if (!rc::unpack_cmd_sdu_user(user, rc::kCmdSduUserBytes, &cmd_id, &seq,
                                     &p1, &p2, &p3, &p4, &p5)) {
            return false;
        }
        mavlink_command_long_t cmd{};
        cmd.command = cmd_id;
        cmd.confirmation = seq;
        cmd.param1 = p1;
        cmd.param2 = p2;
        cmd.param3 = p3;
        cmd.param4 = p4;
        cmd.param5 = p5;
        const uint8_t ack_result = dispatch_command(me, cmd);
        stage_cmd_ack(cmd, ack_result);
        send_pending_ack_if_any();
        return false;
    }
    rc::ccsds::CommandAckPayload ack{};
    uint8_t user[rc::kAckSduUserBytes];
    if (pkt->data.size() != rc::kAckSduUserBytes) {
        return false;
    }
    for (std::size_t i = 0; i < rc::kAckSduUserBytes; ++i) {
        user[i] = static_cast<uint8_t>(pkt->data[i]);
    }
    if (rc::unpack_ack_sdu_user(user, rc::kAckSduUserBytes, &ack)) {
        (void)apply_cmd_ack_payload(ack);
    }
    return false;
}

static void starcom_handle_rx(TelemAo* me, const rc::RadioRxEvt* rx_evt) {
    std::byte in[256];
    const uint8_t n = rx_evt->len;
    if (n == 0) {
        return;
    }
    for (uint8_t i = 0; i < n; ++i) {
        in[i] = std::byte{rx_evt->buf[i]};
    }
    rc::starcom_adapt::pump_receive_bytes(
        g_pump, std::span<const std::byte>(in, n));
    starcom_drain_to_radio();
    for (uint8_t i = 0; i < kStarcomDrainCap; ++i) {
        std::byte sdu[starcom::ccsds::kCoppHold];
        const auto got = rc::starcom_adapt::pump_take_sdu(
            g_pump, std::span<std::byte>(sdu, sizeof(sdu)));
        if (!got.has_value() || *got == 0) {
            break;
        }
        if (starcom_handle_sdu(me, std::span<const std::byte>(sdu, *got))) {
#ifndef ROCKETCHIP_HOST_TEST
            dispatch_nav_output(me, me->rx_snapshot.telem, rx_evt,
                                me->rx_snapshot.seq);
#else
            (void)me;
#endif
        }
    }
}
#endif

static void handle_rx_packet(TelemAo* me, const rc::RadioRxEvt* rx_evt) {
#ifdef ROCKETCHIP_USE_STARCOM
    starcom_handle_rx(me, rx_evt);
    return;
#endif
#ifdef ROCKETCHIP_JOB_STATION
    // Inject RX drop (runtime-gated; 0 on production boots).
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
    if (!rc::ccsds_decode_nav(rx_evt->buf, rx_evt->len, telem, seq, met_ms, echo)) {
        // Not nav — try command ACK, then MAVLink command fallback
        if (try_handle_cmd_ack(rx_evt)) {
            return;
        }
        try_mavlink_rx(me, rx_evt->buf, rx_evt->len);
        return;
    }

    // Store for CLI/WiFi access
    me->rx_snapshot.telem = telem;
    me->rx_snapshot.met_ms = met_ms;
    me->rx_snapshot.seq = seq;
    me->rx_snapshot.valid = true;
    // Config echo. APID 0x001 legacy has echo.bw_khz == 0 — keep last seen.
    if (echo.bw_khz != 0) {
        me->rx_snapshot.echo_bw_khz       = echo.bw_khz;
        me->rx_snapshot.echo_nav_hz       = echo.nav_hz;
        me->rx_snapshot.echo_sf           = echo.sf;
        me->rx_snapshot.echo_cr           = echo.cr;
        me->rx_snapshot.echo_just_changed = echo.just_changed;
    }

#ifdef ROCKETCHIP_STAGE_T2_CHEAT
    // Cheat-mode: fire pending command once vehicle TX done / kRxWindow.
    extern void stage_t2_fire_pending_if_any();
    stage_t2_fire_pending_if_any();
#endif

#ifndef ROCKETCHIP_HOST_TEST
    dispatch_nav_output(me, telem, rx_evt, seq);
#else
    (void)me;
#endif
}

// GCS connection state update
static void update_gcs_state(TelemAo* me, uint32_t t) {
    if (me->gcs_state == GcsState::kGcsConnected) {
        if (t - me->last_gcs_heartbeat_ms > kGcsTimeoutMs) {
            me->gcs_state = GcsState::kGcsLost;
        }
    }
}

// Direct USB MAVLink output (heartbeat-only until GCS detected)
static void mavlink_direct_tick(TelemAo* me) {
#ifndef ROCKETCHIP_HOST_TEST
    if (AO_RCOS_get_output_mode() != StationOutputMode::kMavlink) { return; }
    if constexpr (job::kRadioModeRx) { return; }  // Station uses RX path
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

static QState telem_ao_initial(TelemAo * const me, QEvt const * const e) {
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
    QActive_subscribe(&me->super, rc::SIG_HEALTH_STATUS);  // health byte

    // 10Hz tick (every 10 ticks at 100Hz base)
    QTimeEvt_armX(&me->tick_timer, 10U, 10U);
    return Q_TRAN(&telem_ao_running);
}

static QState telem_ao_running(TelemAo * const me, QEvt const * const e) {
    switch (e->sig) {
    case SIG_TELEM_TICK: {
#ifdef ROCKETCHIP_USE_STARCOM
        rc::starcom_adapt::pump_tick(g_pump, static_cast<starcom::ccsds::Tick>(now_ms()));
#endif
        // Vehicle TX: encode and post to AO_Radio
        if constexpr (!job::kRadioModeRx) {
            encode_and_send(me);
        }
        // Vehicle direct USB MAVLink (no radio)
        mavlink_direct_tick(me);
        return Q_HANDLED();
    }

    case rc::SIG_RADIO_RX: {
        const auto* rx_evt = rc::evt_cast<rc::RadioRxEvt>(e);
        handle_rx_packet(me, rx_evt);
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

QActive * const AO_Telemetry = &g_telemAo.super;

// CLI access — safe under QV cooperative scheduling
void AO_Telemetry_set_telem_snapshot(const rc::TelemetryState& telem) {
    g_telemAo.latest_telem = telem;
    g_telemAo.telem_valid = true;
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
        if (kRates[i] == g_telemAo.rate_hz) {
            uint8_t next = kRates[(i + 1) % 3];
            AO_Telemetry_set_rate(next);
            return next;
        }
    }
    AO_Telemetry_set_rate(5);
    return 5;
}

// SET_RADIO_CONFIG → vehicle TX interval. Rate policy is radio_config_table.
void AO_Telemetry_set_rate(uint8_t rate_hz) {
    if (rate_hz == 0) { rate_hz = 5; }
    if (rate_hz > 50) { rate_hz = 50; }  // sanity: 50 Hz ~ 20ms period
    g_telemAo.rate_hz = rate_hz;
    g_telemAo.interval_ms = 1000U / rate_hz;
}

// Airtime-scaled ACK-retry timeout from AO_Radio ({SF, BW, payload}).
void AO_Telemetry_set_ack_retry_timeout_ms(uint32_t timeout_ms) {
    if (timeout_ms < 100U)  { timeout_ms = 100U; }
    if (timeout_ms > 5000U) { timeout_ms = 5000U; }
    g_ackRetryTimeoutMs = timeout_ms;
}

const RxTelemSnapshot* AO_Telemetry_get_rx_state() {
    return &g_telemAo.rx_snapshot;
}

// Encode + send MAVLink COMMAND_LONG over LoRa
void AO_Telemetry_send_command(uint16_t command, const MavCmdParams& params) {
#ifndef ROCKETCHIP_HOST_TEST
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        255, 0,  // GCS sysid=255, compid=0
        &msg,
        1, 1,    // Target sysid=1, compid=1 (vehicle)
        command,
        0,       // Confirmation
        params.p1, params.p2, params.p3, params.p4, params.p5,
        params.p6, params.p7);

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    static rc::RadioTxEvt g_txEvt;
    g_txEvt.super.sig = rc::SIG_RADIO_TX;
    g_txEvt.super.refCtr_ = 0;
    if (len <= sizeof(g_txEvt.buf)) {
        memcpy(g_txEvt.buf, buf, len);
        g_txEvt.len = static_cast<uint8_t>(len);
        QACTIVE_POST(AO_Radio, &g_txEvt.super, &l_telemAo.super);
    }
#else
    (void)command; (void)params;
#endif
}

// Tracked command — pending-cmd state for ACK tracking.
static uint8_t g_cmdSeq = 0;

#ifndef ROCKETCHIP_USE_STARCOM
// ARM and ABORT skip newest-wins dedupe so each press is its own ACK
// window. DISARM may dedupe (safer than ARM; mash is a no-op if disarmed).
static bool is_tracked_command_safety_class(uint16_t cmd_id, float p1) {
    if (cmd_id == MAV_CMD_DO_FLIGHTTERMINATION) {
        return true;
    }
    if (cmd_id == MAV_CMD_COMPONENT_ARM_DISARM && p1 > 0.5F) {
        return true;  // ARM only; DISARM (p1 < 0.5) is fine to dedupe.
    }
    return false;
}

#endif  // !ROCKETCHIP_USE_STARCOM

#ifndef ROCKETCHIP_HOST_TEST
#ifndef ROCKETCHIP_USE_STARCOM
// Encode and TX a MAVLink COMMAND_LONG with the given seq/params.
static void tx_tracked_command_wire(uint16_t command, uint8_t seq,
                                    const MavCmdParams& params) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        255, 0, &msg, 1, 1,
        command, seq,
        params.p1, params.p2, params.p3, params.p4, params.p5, 0, 0);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    static rc::RadioTxEvt g_txEvt;
    g_txEvt.super.sig = rc::SIG_RADIO_TX;
    g_txEvt.super.refCtr_ = 0;
    if (len <= sizeof(g_txEvt.buf)) {
        memcpy(g_txEvt.buf, buf, len);
        g_txEvt.len = static_cast<uint8_t>(len);
        QACTIVE_POST(AO_Radio, &g_txEvt.super, &l_telemAo.super);
    }
}
#endif  // !ROCKETCHIP_USE_STARCOM

// Populate s_pending_cmd + params (used by both fresh-send and dedupe-replace).
static void populate_pending(uint16_t command, uint8_t seq,
                             const MavCmdParams& params) {
    g_pendingCmd.pending = true;
    g_pendingCmd.seq = seq;
    g_pendingCmd.cmd_id = command;
    g_pendingCmd.sent_ms = to_ms_since_boot(get_absolute_time());
    g_pendingCmd.retries_left = kAckMaxRetries;
    g_pendingCmd.p1 = params.p1;
    g_pendingCmd.p2 = params.p2;
    g_pendingCmd.p3 = params.p3;
    g_pendingCmd.p4 = params.p4;
    g_pendingCmd.p5 = params.p5;

    // Dedupe-replace is still a new send (new seq, fresh ACK window).
    CmdClass c = classify_tracked_cmd(command, params.p1);
    g_retryStats[c].sent_count++;

    // Clear last-result so dashboard shows pending, not stale ACK.
    g_lastCmdResult.valid = false;
}
#endif  // !ROCKETCHIP_HOST_TEST

void AO_Telemetry_send_tracked_command(uint16_t command, float p1,
                                       float p2, float p3,
                                       float p4, float p5) {
#ifndef ROCKETCHIP_HOST_TEST
#ifdef ROCKETCHIP_USE_STARCOM
    {
        uint8_t seq = g_cmdSeq++;
        const MavCmdParams params{p1, p2, p3, p4, p5};
        populate_pending(command, seq, params);
        std::byte pkt[6u + rc::kCmdSduUserBytes];
        const auto n = rc::starcom_adapt::pump_pack_cmd_packet(
            pkt, command, seq, p1, p2, p3, p4, p5);
        if (n.has_value() && *n > 0) {
            (void)rc::starcom_adapt::pump_submit_sdu(
                g_pump, std::span<const std::byte>(pkt, *n), false);
            starcom_drain_to_radio();
        }
    }
#else
    if constexpr (!rc::kAirLoraCommandsEnabled) {
        rc::rc_log("[SC] LoRa command refused (starcom-prep; COP-P not linked)\n");
        (void)command;
        (void)p1;
        (void)p2;
        (void)p3;
        (void)p4;
        (void)p5;
        return;
    }
    // Newest-wins dedupe for non-safety cmds: mash → one pending, latest params.
    // ARM/ABORT bypass so each press keeps its own ACK window.
    if (g_pendingCmd.pending &&
        g_pendingCmd.cmd_id == command &&
        !is_tracked_command_safety_class(command, p1)) {
        uint8_t seq = g_cmdSeq++;
        const MavCmdParams params{p1, p2, p3, p4, p5};
        populate_pending(command, seq, params);
        tx_tracked_command_wire(command, seq, params);
        return;
    }

    // Fresh send: allocate seq, populate pending, TX on wire.
    uint8_t seq = g_cmdSeq++;
    const MavCmdParams params{p1, p2, p3, p4, p5};
    populate_pending(command, seq, params);
    tx_tracked_command_wire(command, seq, params);
#endif  // ROCKETCHIP_USE_STARCOM
#else
    (void)command; (void)p1; (void)p2; (void)p3; (void)p4; (void)p5;
#endif
}

bool AO_Telemetry_is_cmd_pending() {
    return g_pendingCmd.pending;
}

// Dashboard snapshot of pending/recent-ack. Core 0 cooperative, no locks.
void AO_Telemetry_get_pending_cmd_status(PendingCmdStatus* out) {
    if (out == nullptr) { return; }
    out->pending      = g_pendingCmd.pending;
    out->cmd_id       = g_pendingCmd.cmd_id;
    out->retries_used = static_cast<uint8_t>(
        kAckMaxRetries - g_pendingCmd.retries_left);
    out->max_retries  = kAckMaxRetries;
    out->last_result_valid = g_lastCmdResult.valid;
    out->last_result_ok    = g_lastCmdResult.ok;
    out->last_cmd_id       = g_lastCmdResult.cmd_id;
    out->last_rtt_ms       = g_lastCmdResult.rtt_ms;
    out->last_result_ms    = g_lastCmdResult.at_ms;
}

// Retry-stats snapshot for CLI/diag.
uint8_t AO_Telemetry_get_retry_stats(CmdRetryStatsLine* rows, uint8_t max_rows) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < static_cast<uint8_t>(kCmdClassCount) && n < max_rows; ++i) {
        rows[n].name               = cmd_class_name(static_cast<CmdClass>(i));
        rows[n].sent               = g_retryStats[i].sent_count;
        rows[n].first_try          = g_retryStats[i].first_try_ack_count;
        rows[n].retry_rescued      = g_retryStats[i].retry_ack_count;
        rows[n].failed             = g_retryStats[i].fail_count;
        rows[n].total_retries_used = g_retryStats[i].total_retries_used;
        n++;
    }
    return n;
}

// Internal: re-send pending command with same seq (for retries)
static void resend_pending_cmd() {
#ifndef ROCKETCHIP_HOST_TEST
    mavlink_message_t msg;
    // Replay all cached params (SET_RADIO_CONFIG is multi-param).
    mavlink_msg_command_long_pack(
        255, 0, &msg, 1, 1,
        g_pendingCmd.cmd_id,
        g_pendingCmd.seq,  // Same seq as original
        g_pendingCmd.p1, g_pendingCmd.p2, g_pendingCmd.p3,
        g_pendingCmd.p4, g_pendingCmd.p5, 0, 0);

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    static rc::RadioTxEvt g_txEvt;
    g_txEvt.super.sig = rc::SIG_RADIO_TX;
    g_txEvt.super.refCtr_ = 0;
    if (len <= sizeof(g_txEvt.buf)) {
        memcpy(g_txEvt.buf, buf, len);
        g_txEvt.len = static_cast<uint8_t>(len);
        QACTIVE_POST(AO_Radio, &g_txEvt.super, &l_telemAo.super);
    }
    g_pendingCmd.sent_ms = to_ms_since_boot(get_absolute_time());
#endif
}

void AO_Telemetry_cmd_retry_tick(uint32_t now_ms) {
#ifndef ROCKETCHIP_HOST_TEST
#ifdef ROCKETCHIP_USE_STARCOM
    (void)now_ms;
    return;  // COP-P owns resend; homemade ACK retry stays OFF-image only
#endif
    if (!g_pendingCmd.pending) return;

    uint32_t elapsed = now_ms - g_pendingCmd.sent_ms;
    if (elapsed >= g_ackRetryTimeoutMs) {
        if (g_pendingCmd.retries_left > 0) {
            g_pendingCmd.retries_left--;
            rc::rc_log("[CMD] Retry %u/%u (seq=%u)\n",
                       kAckMaxRetries - g_pendingCmd.retries_left,
                       kAckMaxRetries, g_pendingCmd.seq);
            resend_pending_cmd();
        } else {
            // Record fail + retries used.
            CmdClass cls = classify_tracked_cmd(g_pendingCmd.cmd_id,
                                                 g_pendingCmd.p1);
            g_retryStats[cls].fail_count++;
            g_retryStats[cls].total_retries_used += kAckMaxRetries;

            // Latch failure for dashboard.
            g_lastCmdResult.valid  = true;
            g_lastCmdResult.ok     = false;
            g_lastCmdResult.cmd_id = g_pendingCmd.cmd_id;
            g_lastCmdResult.rtt_ms = 0;
            g_lastCmdResult.at_ms  = now_ms;

            g_pendingCmd.pending = false;
            rc::rc_log("[CMD] No ACK after %u retries\n",
                       static_cast<unsigned>(kAckMaxRetries));
        }
    }
#else
    (void)now_ms;
#endif
}

// Feed USB input byte to MAVLink parser for GCS detection + commands.
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
            g_telemAo.gcs_heartbeat_count++;
            AO_RCOS_set_output_mode(StationOutputMode::kMavlink);
        }
        // Param request — defer to tick handler
        if (msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST &&
            g_telemAo.param_send_idx < 0) {
            g_telemAo.param_send_idx = 0;
        }
        // Command dispatch (ARM/DISARM/ABORT from QGC)
        if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
            mavlink_command_long_t cmd;
            mavlink_msg_command_long_decode(&msg, &cmd);
            if (cmd.command == MAV_CMD_COMPONENT_ARM_DISARM) {
                uint16_t sig = (cmd.param1 > 0.5F)
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
    return g_telemAo.gcs_state == GcsState::kGcsConnected;
}

// Notify that a GCS heartbeat was received (USB or LoRa)
void AO_Telemetry_notify_gcs_heartbeat() {
    g_telemAo.last_gcs_heartbeat_ms = now_ms();
    if (g_telemAo.gcs_state != GcsState::kGcsConnected) {
        g_telemAo.gcs_state = GcsState::kGcsConnected;
    }
}

void AO_Telemetry_start(uint8_t prio) {
    QActive_ctor(&g_telemAo.super,
                 Q_STATE_CAST(&telem_ao_initial));

    QTimeEvt_ctorX(&g_telemAo.tick_timer, &g_telemAo.super,
                   SIG_TELEM_TICK, 0U);

    memset(&g_telemAo.latest_telem, 0, sizeof(g_telemAo.latest_telem));
    g_telemAo.telem_valid = false;
    memset(&g_telemAo.rx_snapshot, 0, sizeof(g_telemAo.rx_snapshot));
#ifdef ROCKETCHIP_USE_STARCOM
    rc::starcom_adapt::pump_init_for_this_job(g_pump);
#endif

    QActive_start(&g_telemAo.super,
                  Q_PRIO(prio, 0U),
                  g_telemAoQueue,
                  Q_DIM(g_telemAoQueue),
                  nullptr, 0U,
                  nullptr);
}
