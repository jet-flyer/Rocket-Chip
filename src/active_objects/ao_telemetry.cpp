// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_Telemetry — Telemetry Protocol Active Object
//
// Air: Starcom COP-P (nav/cmd/ACK SDUs). USB GCS: station gcs_mavlink.
// No radio hardware references. Posts SIG_RADIO_TX to AO_Radio.
// Receives SIG_RADIO_RX for decode + station USB re-encode.
//============================================================================

#include "ao_telemetry.h"
#include "ao_radio.h"
#include "ao_flight_director.h"
#include "flight_director/flight_director.h"
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
#include "starcom_adapt/byte_pump.h"
#include "starcom_adapt/nav_sdu.h"
#include "starcom_adapt/cmd_sdu.h"
#include "station/gcs_mavlink.h"
#include "rocketchip/sensor_seqlock.h"
#include "flight_director/mission_profile_data.h"  // kDefaultRocketRadioConfig
#include <math.h>                                   // lroundf (float→int for SET_RADIO_CONFIG)
#ifdef ROCKETCHIP_JOB_STATION
#include "safety/station_fault_inject.h"  // runtime-gated; flags are 0 unless armed
#endif

#ifndef ROCKETCHIP_HOST_TEST
#include "pico/time.h"
#include "pico/stdio.h"
#include "tusb.h"
#endif

#include "rocketchip/rc_log.h"
#include "diag/radio_rate_counters.h"
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

// GCS connection state (USB lockout). Stream policy is gcs_mavlink.
enum class GcsState : uint8_t {
    kWaitingForGcs = 0,
    kGcsConnected  = 1,
    kGcsLost       = 2,
};


// Airtime-scaled tracked-command retry timeout. AO_Radio pushes a value
// from {SF, BW, max payload} on every SET_RADIO_CONFIG apply. Seed 250 ms
// until first apply. 8 × 250 ms ≈ 2 s give-up window.
static uint32_t g_ackRetryTimeoutMs = 250U;

static constexpr uint8_t kAckMaxRetries = 8U;

struct TelemAo {
    QActive super;
    QTimeEvt tick_timer;    // 100 Hz (QF 100 Hz / 1)

    // Protocol state (USB MAVLink is separate from LoRa Starcom)
    rc::MavlinkEncoder  mav_encoder;
    rc::GcsMavlink      gcs;
    rc::TelemetryState  latest_telem;
    bool                telem_valid;
    uint8_t             rate_hz;
    uint32_t            interval_ms;
    uint32_t            last_tx_ms;

    // GCS connection tracking
    GcsState            gcs_state;
    uint32_t            last_gcs_heartbeat_ms;

    // MAVLink RX parser (USB command ACKs / GCS detect)
    rc::MavlinkRxState  mavlink_rx;
    uint8_t             gcs_heartbeat_count;

    // Station RX: latest decoded telemetry for CLI/WiFi access
    RxTelemSnapshot     rx_snapshot;
    bool                starcom_nav_sdu;
};

static TelemAo g_telemAo;
static rc::starcom_adapt::BytePump g_pump;
// QF 100 Hz. Nav still 10 Hz (ticks_per_nav). HD MAC (211.0 §6) owns
// TX/RX contacts; do not skip nav to fake leftover RX.
static constexpr uint8_t kTelemTickHz = 100;
static uint8_t g_navTickAcc = 0;

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
    // tud_cdc_connected() is DTR; QGC 5.x blips DTR while CDC stays configured.
    // Match pico-sdk CONNECTION_WITHOUT_DTR (tud_ready()).
    if (!tud_ready()) { return; }
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

static void gcs_usb_write(const uint8_t* data, uint16_t len, void* ctx) {
    (void)ctx;
    usb_write_nonblocking(data, len);
}

static rc::GcsMavlinkSink gcs_usb_sink() {
    rc::GcsMavlinkSink sink{};
    sink.write = &gcs_usb_write;
    sink.ctx = nullptr;
    return sink;
}

static bool gcs_mode_active() {
#ifndef ROCKETCHIP_HOST_TEST
    return AO_RCOS_get_output_mode() == StationOutputMode::kMavlink;
#else
    return false;
#endif
}

static void gcs_station_rx_drain() {
#ifndef ROCKETCHIP_HOST_TEST
    if (!gcs_mode_active()) { return; }
    int c = getchar_timeout_us(0);
    while (c != PICO_ERROR_TIMEOUT) {
        AO_Telemetry_feed_usb_byte(static_cast<uint8_t>(c));
        c = getchar_timeout_us(0);
    }
#endif
}

// Per decoded nav SDU. Same as old dispatch_nav_output kMavlink
// (ATTITUDE + GLOBAL_POSITION_INT), plus GPS_RAW.
static void dispatch_nav_mavlink(TelemAo* me, const rc::TelemetryState& telem) {
#ifndef ROCKETCHIP_HOST_TEST
    if (AO_RCOS_get_output_mode() != StationOutputMode::kMavlink) {
        return;
    }
    (void)rc::gcs_mavlink_on_nav(&me->gcs, telem, now_ms(), gcs_usb_sink());
#else
    (void)me;
    (void)telem;
#endif
}

static void gcs_station_tick(TelemAo* me) {
    if constexpr (!job::kRadioModeRx) { return; }
#ifndef ROCKETCHIP_HOST_TEST
    if (!gcs_mode_active()) { return; }
    gcs_station_rx_drain();
    const uint32_t t = now_ms();
    if (me->rx_snapshot.valid) {
        me->gcs.last_telem = me->rx_snapshot.telem;
        me->gcs.telem_valid = true;
    }
    (void)rc::gcs_mavlink_tick(&me->gcs, t, gcs_usb_sink());
    shared_sensor_data_t snap = {};
    if (seqlock_read(&g_sensorSeqlock, &snap)) {
        rc::GcsStationGps gps{};
        gps.lat_1e7 = snap.gps_lat_1e7;
        gps.lon_1e7 = snap.gps_lon_1e7;
        gps.alt_mm = static_cast<int32_t>(lroundf(snap.gps_alt_msl_m * 1000.0F));
        gps.speed_mps = snap.gps_ground_speed_mps;
        gps.fix_type = snap.gps_fix_type;
        gps.valid = snap.gps_valid;
        (void)rc::gcs_mavlink_on_station_gps(&me->gcs, gps, t, gcs_usb_sink());
    }
#else
    (void)me;
#endif
}

// Vehicle-side pending ACK (queued for next TX opportunity)
static rc::ccsds::CommandAckPayload g_pendingAck = {};
static bool g_pendingAckValid = false;

// Station-side pending command. p1..p5 replay SET_RADIO_CONFIG on retry.
static struct {
    bool pending;
    uint8_t seq;
    uint8_t ns;  // FOP N(S) of the queued AD
    uint16_t cmd_id;
    uint32_t sent_ms;
    uint8_t retries_left;
    float p1;
    float p2;
    float p3;
    float p4;
    float p5;
} g_pendingCmd = {};

// Radio reconfigure lives in AO_Radio. After SET_RADIO_CONFIG validates,
// AO_Radio_set_pending_config() applies on the next TxDone (outgoing ACK).

// Send pending command ACK before nav frame
// One PLTU per call. COP-P resends unacked seq if no peer PLCW; draining
// a window of frames every 10 Hz tick floods the half-duplex radio.
// 211.0 table 6-14: NEED_PLCW then SDU. Cap 2 posts the SDU while PLCW
// is on air (hold until TxDone). Status PLCWs stop after one fill.
static constexpr uint8_t kStarcomDrainCap = 2;
// R-32 R3: station PLCW/ACK air, not mute. Named cell "B5r (PLCW every 4th)"
// in RADIO_SOAK_PASS_AB_2026-09-03.md (unrun). T1 B4 station-TX-on ~7.7 Hz
// vs T3 mute ~9.9 Hz; busy_drop=0 (H3). Skip drain keeps farm.need_plcw.
static constexpr uint8_t kStationPlcwEveryNthNav = 4;
static uint32_t g_stationLastAirMs = 0;
static uint8_t g_stationNavRxSincePlcw = 0;

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
    radio_rate_inc_pltu_post();
}

static bool station_plcw_cadence_allows() {
    if constexpr (!job::kRadioModeRx) {
        return true;
    }
    const uint32_t now = now_ms();
    if (g_stationLastAirMs == 0) {
        return true;
    }
    uint32_t nav_ms = g_telemAo.interval_ms;
    if (nav_ms == 0) {
        const uint8_t hz = rc::kDefaultRocketRadioConfig.nav_rate_hz;
        nav_ms = (hz == 0) ? 200U : (1000U / hz);
    }
    const uint32_t min_ms =
        static_cast<uint32_t>(kStationPlcwEveryNthNav) * nav_ms;
    return (now - g_stationLastAirMs) >= min_ms;
}

// Expedited FARM-P RE3 does not set need_plcw (Blue Book). Product still
// wants sparse status PLCWs. Arm every Nth CRC-ok station RX; drain
// cadence keeps air at ~nav_hz/N (B5r). Do not change starcom RE3.
static void station_arm_sparse_plcw() {
    if constexpr (!job::kRadioModeRx) {
        return;
    }
    g_stationNavRxSincePlcw++;
    if (g_stationNavRxSincePlcw < kStationPlcwEveryNthNav) {
        return;
    }
    g_stationNavRxSincePlcw = 0;
    g_pump.copp.farm.need_plcw = true;
}

#ifndef ROCKETCHIP_HOST_TEST
static rc::RadioConfig radio_from_catalog(uint8_t idx) {
    rc::RadioConfig cfg = rc::kDefaultRocketRadioConfig;
    if (idx >= rc::kRadioConfigTableSize) {
        return cfg;
    }
    const auto& e = rc::kRadioConfigTable[idx];
    cfg.bandwidth_khz = e.bw_khz;
    cfg.nav_rate_hz = e.nav_rate_hz;
    cfg.spreading_factor = e.sf;
    cfg.coding_rate = e.cr;
    cfg.power_dbm = e.power_dbm;
    return cfg;
}
static bool g_commChangeArmed = false;
static bool g_remoteApplyOnReceive = false;
static rc::RadioConfig g_commChangePending = rc::kDefaultRocketRadioConfig;

static void starcom_poll_mac_radio() {
    const auto n = rc::starcom_adapt::pump_poll_mac_notify(g_pump);
    if (n == starcom::ccsds::MacNotify::comm_change_apply_rx &&
        g_pump.pending_catalog_valid) {
        g_commChangePending = radio_from_catalog(g_pump.pending_catalog_idx);
        g_commChangeArmed = true;
        rc::rc_log("[SC] COMM_CHANGE RX armed idx=%u BW=%u nav=%u\n",
                   static_cast<unsigned>(g_pump.pending_catalog_idx),
                   static_cast<unsigned>(g_commChangePending.bandwidth_khz),
                   static_cast<unsigned>(g_commChangePending.nav_rate_hz));
    } else if (n == starcom::ccsds::MacNotify::comm_change_revert) {
        g_commChangeArmed = false;
        g_remoteApplyOnReceive = false;
        g_pump.local_comm_change = false;
        AO_Radio_apply_config_now(
            radio_from_catalog(g_pump.hail_catalog_idx));
        rc::rc_log("[SC] COMM_CHANGE revert hail\n");
    } else if (n == starcom::ccsds::MacNotify::hail_ok) {
        rc::rc_log("[SC] hail ok\n");
    } else if (n == starcom::ccsds::MacNotify::hail_fail) {
        rc::rc_log("[SC] hail fail, retry session\n");
        constexpr bool kCaller = job::kRadioModeRx;
        rc::starcom_adapt::pump_start_session(
            g_pump, kCaller, static_cast<starcom::ccsds::Tick>(now_ms()));
    }
    // E69: stay on the old PHY through the following send so the
    // initiator can hear a frame (E68) before anyone retunes.
    if (g_pump.remote_apply_now && g_pump.pending_catalog_valid) {
        g_pump.remote_apply_now = false;
        g_commChangePending = radio_from_catalog(g_pump.pending_catalog_idx);
        g_remoteApplyOnReceive = true;
        rc::rc_log("[SC] COMM_CHANGE remote armed idx=%u\n",
                   static_cast<unsigned>(g_pump.pending_catalog_idx));
    }
    if (g_pump.peer_comm_change && g_commChangeArmed) {
        g_pump.peer_comm_change = false;
        g_pump.local_comm_change = false;
        g_commChangeArmed = false;
        g_remoteApplyOnReceive = false;
        AO_Radio_apply_config_now(g_commChangePending);
        rc::rc_log("[SC] COMM_CHANGE ok\n");
    }
    const auto phy = rc::starcom_adapt::pump_mac_phy(g_pump);
    if (g_remoteApplyOnReceive && phy.receive && !phy.transmit) {
        AO_Radio_apply_config_now(g_commChangePending);
        g_remoteApplyOnReceive = false;
        rc::rc_log("[SC] COMM_CHANGE remote apply idx=%u\n",
                   static_cast<unsigned>(g_pump.pending_catalog_idx));
    }
    AO_Radio_set_mac_dir(
        phy.receive, phy.transmit,
        g_pump.mac.mode == starcom::ccsds::MacMode::active);
}
#endif

static bool starcom_drain_to_radio() {
#ifndef ROCKETCHIP_HOST_TEST
    // pump_bytes_to_send consumes a COP-P AD. Do not drain while the
    // radio is already sending — FOP-P would think the frame is in
    // flight (soak MIB synch_timeout=0, so SYNCH never expires).
    if (AO_Radio_tx_active()) {
        return false;
    }
    {
        const auto phy = rc::starcom_adapt::pump_mac_phy(g_pump);
        if (g_pump.mac.mode == starcom::ccsds::MacMode::active &&
            !phy.transmit) {
            return false;
        }
    }
    // R-32: status PLCWs ~nav/4. Seq cmd/ACK must not wait on that timer.
    if constexpr (job::kRadioModeRx) {
        const bool new_seq = g_pump.copp.seq_n != 0;
        const bool must_air_now = g_pendingCmd.pending || g_pendingAckValid;
        if (must_air_now) {
            g_pump.copp.farm.need_plcw = false;
        }
        if (!new_seq && !station_plcw_cadence_allows()) {
            return false;
        }
    }
#endif
    bool posted = false;
    for (uint8_t i = 0; i < kStarcomDrainCap; ++i) {
        std::byte buf[rc::starcom_adapt::kAirMtu];
        const auto n = rc::starcom_adapt::pump_air_to_send(
            g_pump, std::span<std::byte>(buf, sizeof(buf)));
        if (!n.has_value() || *n == 0) {
            return posted;
        }
        starcom_post_pltu(std::span<const std::byte>(buf, *n));
        posted = true;
        if constexpr (job::kRadioModeRx) {
            g_stationLastAirMs = now_ms();
        }
        if (g_pump.copp.seq_n == 0 && !g_pump.copp.exp_full) {
            return posted;
        }
    }
    return posted;
}

static void send_pending_ack_if_any() {
    if (!g_pendingAckValid) return;
    // TX busy: keep the latch and retry. 211.0 7.3.1 RE3: expedited ACK.
    if (AO_Radio_tx_active()) {
        return;
    }
    std::byte pkt[6u + rc::kAckSduUserBytes];
    const auto n = rc::starcom_adapt::pump_pack_ack_packet(
        pkt, g_pendingAck);
    if (!n.has_value() || *n == 0) {
        return;
    }
    // One exp slot; drop unsent nav so the ACK can submit.
    if (g_pump.copp.exp_full) {
        g_pump.copp.exp_full = false;
        g_pump.copp.exp_len = 0;
    }
    const auto sub = rc::starcom_adapt::pump_submit_sdu(
        g_pump, std::span<const std::byte>(pkt, *n), true);
    if (!sub.has_value()) {
        return;
    }
    g_pendingAckValid = false;
    rc::rc_log("[SC] ack SDU id=%u seq=%u\n",
               static_cast<unsigned>(g_pendingAck.cmd_id),
               static_cast<unsigned>(g_pendingAck.cmd_seq));
    starcom_drain_to_radio();
}

static bool nav_submit_due(TelemAo* me) {
    uint8_t hz = me->rate_hz;
    if (hz == 0U) {
        hz = 5U;
    }
    if ((kTelemTickHz % hz) == 0U) {
        const uint8_t ticks_per_nav = static_cast<uint8_t>(kTelemTickHz / hz);
        if (g_navTickAcc < ticks_per_nav) {
            g_navTickAcc++;
        }
        return g_navTickAcc >= ticks_per_nav;
    }
    const uint32_t t = now_ms();
    return ((t - me->last_tx_ms) + 1U >= me->interval_ms);
}

static bool starcom_tx_held_to_spdu() {
#ifndef ROCKETCHIP_HOST_TEST
    if (g_commChangeArmed) {
        const auto src = rc::starcom_adapt::pump_fifo_source(g_pump);
        if (src == starcom::ccsds::MacFifoSource::spdu) {
            (void)starcom_drain_to_radio();
        }
        return true;
    }
    const auto phy = rc::starcom_adapt::pump_mac_phy(g_pump);
    const auto mode = g_pump.mac.mode;
    // 211.0 table 6-10 S2 / 6.5.1: connecting-L TRANSMIT off.
    if (mode == starcom::ccsds::MacMode::connecting_l ||
        mode == starcom::ccsds::MacMode::inactive ||
        (mode == starcom::ccsds::MacMode::active &&
         (!phy.transmit || g_pump.mac.persistence))) {
        const auto src = rc::starcom_adapt::pump_fifo_source(g_pump);
        if (phy.transmit &&
            src == starcom::ccsds::MacFifoSource::spdu) {
            (void)starcom_drain_to_radio();
        }
        return true;
    }
#endif
    return false;
}

static void encode_and_send(TelemAo* me) {
    if (starcom_tx_held_to_spdu()) {
        return;
    }

    send_pending_ack_if_any();

    // 211.0 table 6-14: NEED_PLCW / seq / exp before a new nav submit.
    if (g_pump.copp.seq_n != 0 || g_pump.copp.exp_full ||
        g_pump.copp.farm.need_plcw || g_pump.mac.need_plcw) {
        (void)starcom_drain_to_radio();
#ifndef ROCKETCHIP_HOST_TEST
        if (!AO_Radio_tx_active() &&
            (g_pump.copp.seq_n != 0 || g_pump.copp.exp_full ||
             g_pump.copp.farm.need_plcw || g_pump.mac.need_plcw)) {
            (void)starcom_drain_to_radio();
        }
#endif
        if (g_pump.copp.seq_n != 0 || g_pump.copp.exp_full) {
            return;
        }
    }

    if (!me->telem_valid) {
        return;
    }

    if (!nav_submit_due(me)) {
        const auto src = rc::starcom_adapt::pump_fifo_source(g_pump);
        if (src == starcom::ccsds::MacFifoSource::spdu) {
            (void)starcom_drain_to_radio();
        }
        return;
    }
    g_navTickAcc = 0;
    me->last_tx_ms = now_ms();
    radio_rate_inc_nav_submit();

    if (AO_FlightDirector_is_initialized()) {
        me->latest_telem.flight_state = static_cast<uint8_t>(
            AO_FlightDirector_get_director()->state.current_phase);
    }

    std::byte pkt[6u + rc::kNavSduUserBytes];
    const auto n = rc::starcom_adapt::pump_pack_nav_packet(
        pkt, me->latest_telem);
    if (!n.has_value() || *n == 0) { return; }
    (void)rc::starcom_adapt::pump_submit_sdu(
        g_pump, std::span<const std::byte>(pkt, *n), true);
    (void)starcom_drain_to_radio();
}

// LoRa MAVLink RX — uses MAVLINK_COMM_2 (separate from USB on COMM_1)
// SET_RADIO_CONFIG dispatcher. Returns ACK result.
// 4 gates (flight-state, SX1276-legal, nav ToA fits Hz, ±6 dB power
// delta) before queue.
static uint8_t dispatch_set_radio_config(const mavlink_command_long_t& cmd) {
    uint16_t new_bw  = static_cast<uint16_t>(lroundf(cmd.param1));
    uint8_t  new_nav = static_cast<uint8_t> (lroundf(cmd.param2));
    uint8_t  new_sf  = static_cast<uint8_t> (lroundf(cmd.param3));
    uint8_t  new_cr  = static_cast<uint8_t> (lroundf(cmd.param4));
    uint8_t  new_pwr = static_cast<uint8_t> (lroundf(cmd.param5));

    if (!AO_FlightDirector_is_ground_state()) {
        rc::rc_log("[CMD] SET denied — not ground state\n");
        return static_cast<uint8_t>(rc::ccsds::CmdAckResult::kDenied);
    }
    // Presets are debug-menu defaults; advanced path accepts SX1276-legal
    // tuples whose nav PLTU still fits commanded Hz (125/10 SF7 does not).
    if (!rc::radio_config_sx1276_legal(new_bw, new_nav, new_sf, new_cr, new_pwr)) {
        rc::rc_log("[CMD] SET denied — illegal BW=%u nav=%u SF=%u CR=%u pwr=%u\n",
                   static_cast<unsigned>(new_bw),
                   static_cast<unsigned>(new_nav),
                   static_cast<unsigned>(new_sf),
                   static_cast<unsigned>(new_cr),
                   static_cast<unsigned>(new_pwr));
        return static_cast<uint8_t>(rc::ccsds::CmdAckResult::kDenied);
    }
    if (!rc::radio_config_nav_fits_hz(new_bw, new_nav, new_sf,
                                      rc::kRadioConfigNavPltuBytes)) {
        rc::rc_log("[CMD] SET denied — nav ToA does not fit %u Hz (BW=%u SF=%u)\n",
                   static_cast<unsigned>(new_nav),
                   static_cast<unsigned>(new_bw),
                   static_cast<unsigned>(new_sf));
        return static_cast<uint8_t>(rc::ccsds::CmdAckResult::kDenied);
    }
    const rc::RadioConfig* cur = AO_Radio_get_runtime_config();
    int pwr_delta = static_cast<int>(new_pwr) - static_cast<int>(cur->power_dbm);
    if (pwr_delta < 0) { pwr_delta = -pwr_delta; }
    if (pwr_delta > 6) {
        rc::rc_log("[CMD] SET denied — power delta %d dB\n", pwr_delta);
        return static_cast<uint8_t>(rc::ccsds::CmdAckResult::kDenied);
    }

    rc::RadioConfig new_cfg = *cur;  // inherit mode/protocol
    new_cfg.nav_rate_hz      = new_nav;
    new_cfg.power_dbm        = new_pwr;
    new_cfg.spreading_factor = new_sf;
    new_cfg.bandwidth_khz    = new_bw;
    new_cfg.coding_rate      = new_cr;
    AO_Radio_set_pending_config(new_cfg);
    rc::rc_log("[CMD] SET accepted BW=%u nav=%u SF=%u CR=%u pwr=%u\n",
               static_cast<unsigned>(new_bw),
               static_cast<unsigned>(new_nav),
               static_cast<unsigned>(new_sf),
               static_cast<unsigned>(new_cr),
               static_cast<unsigned>(new_pwr));
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
        rc::rc_log("[FD] radio %s\n",
                   (sig == static_cast<uint16_t>(rc::SIG_ARM)) ? "ARM" : "DISARM");
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
        // SET_RADIO_CONFIG (4 gates inside).
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
        rc::rc_log("[CMD] ack ignore seq=%u id=%u pend=%u\n",
                   static_cast<unsigned>(ack.cmd_seq),
                   static_cast<unsigned>(ack.cmd_id),
                   g_pendingCmd.pending ? 1u : 0u);
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

#ifndef ROCKETCHIP_HOST_TEST
static void dispatch_nav_csv(const rc::RadioRxEvt* rx_evt, uint16_t seq) {
    if (AO_RCOS_get_output_mode() != StationOutputMode::kCsv) {
        return;
    }
    rc::rc_log("RX,%u,%d,%d\n",
               static_cast<unsigned>(seq),
               static_cast<int>(rx_evt->rssi),
               static_cast<int>(rx_evt->snr));
}
#endif

static bool starcom_handle_nav_sdu(TelemAo* me, std::span<const std::byte> data,
                                   uint16_t seq) {
    rc::TelemetryState telem = {};
    uint8_t user[rc::kNavSduUserBytes];
    if (data.size() != rc::kNavSduUserBytes) {
        return false;
    }
    for (std::size_t i = 0; i < rc::kNavSduUserBytes; ++i) {
        user[i] = static_cast<uint8_t>(data[i]);
    }
    if (!rc::unpack_nav_sdu_user(user, rc::kNavSduUserBytes, &telem)) {
        return false;
    }
    me->rx_snapshot.telem = telem;
    me->rx_snapshot.met_ms = telem.met_ms;
    me->rx_snapshot.seq = seq;
    me->rx_snapshot.valid = true;
    me->starcom_nav_sdu = true;
    return true;
}

static bool starcom_handle_cmd_sdu(TelemAo* me, std::span<const std::byte> data) {
    uint16_t cmd_id = 0;
    uint8_t seq = 0;
    float p1 = 0, p2 = 0, p3 = 0, p4 = 0, p5 = 0;
    uint8_t user[rc::kCmdSduUserBytes];
    if (data.size() != rc::kCmdSduUserBytes) {
        return false;
    }
    for (std::size_t i = 0; i < rc::kCmdSduUserBytes; ++i) {
        user[i] = static_cast<uint8_t>(data[i]);
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
    rc::rc_log("[SC] cmd SDU id=%u seq=%u\n",
               static_cast<unsigned>(cmd_id), static_cast<unsigned>(seq));
    const uint8_t ack_result = dispatch_command(me, cmd);
    stage_cmd_ack(cmd, ack_result);
    send_pending_ack_if_any();
    return false;
}

static bool starcom_handle_sdu(TelemAo* me, std::span<const std::byte> sdu) {
    const auto pkt = starcom::ccsds::decodeSpacePacket(sdu);
    if (!pkt) {
        return false;
    }
    if (pkt->fields.apid == rc::starcom_adapt::kNavApid) {
        return starcom_handle_nav_sdu(me, pkt->data, pkt->fields.seq_count);
    }
    if (pkt->fields.apid != rc::starcom_adapt::kCmdApid) {
        return false;
    }
    if (pkt->fields.telecommand) {
        return starcom_handle_cmd_sdu(me, pkt->data);
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
        rc::rc_log("[SC] ack SDU id=%u seq=%u\n",
                   static_cast<unsigned>(ack.cmd_id),
                   static_cast<unsigned>(ack.cmd_seq));
        (void)apply_cmd_ack_payload(ack);
    }
    return false;
}

#ifdef ROCKETCHIP_JOB_STATION
static void note_seq_delivered() {
    if (!g_pendingCmd.pending) {
        return;
    }
    // 211.0 8.2.1 e: peer PLCW N(R) acknowledges the Sequence Controlled SDU.
    if (!starcom::ccsds::seqLt(g_pendingCmd.ns, g_pump.copp.fop.nn_r)) {
        return;
    }
    rc::ccsds::CommandAckPayload ack{};
    ack.cmd_seq = g_pendingCmd.seq;
    ack.cmd_id = g_pendingCmd.cmd_id;
    ack.result = static_cast<uint8_t>(rc::ccsds::CmdAckResult::kAccepted);
    (void)apply_cmd_ack_payload(ack);
}
#endif

static bool note_starcom_pltu_crc(std::span<const std::byte> octets) {
    if constexpr (!job::kRadioModeRx) {
        return false;
    }
    const auto pltu = starcom::ccsds::decodePltu(octets);
    if (pltu) {
        radio_rate_inc_rx_crc_ok();
        return true;
    }
    if (pltu.error() == starcom::ccsds::Error::bad_crc) {
        radio_rate_inc_rx_crc_fail();
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
    const bool crc_ok = note_starcom_pltu_crc(std::span<const std::byte>(in, n));
    rc::starcom_adapt::pump_handle_air(
        g_pump, std::span<const std::byte>(in, n));
#ifdef ROCKETCHIP_JOB_STATION
    note_seq_delivered();
#endif
#ifndef ROCKETCHIP_HOST_TEST
    starcom_poll_mac_radio();
#endif
    if (crc_ok) {
        station_arm_sparse_plcw();
    }
    starcom_drain_to_radio();
    // RX must empty the FARM queue, not stop at one SDU.
    for (uint8_t i = 0; i < starcom::ccsds::kCoppSeqSlots; ++i) {
        std::byte sdu[starcom::ccsds::kCoppHold];
        const auto got = rc::starcom_adapt::pump_take_sdu(
            g_pump, std::span<std::byte>(sdu, sizeof(sdu)));
        if (!got.has_value() || *got == 0) {
            break;
        }
        if (starcom_handle_sdu(me, std::span<const std::byte>(sdu, *got))) {
#ifndef ROCKETCHIP_HOST_TEST
            dispatch_nav_mavlink(me, me->rx_snapshot.telem);
            dispatch_nav_csv(rx_evt, me->rx_snapshot.seq);
#else
            (void)me;
#endif
        }
    }
}

static void handle_rx_packet(TelemAo* me, const rc::RadioRxEvt* rx_evt) {
#ifdef ROCKETCHIP_JOB_STATION
    // Inject RX drop (runtime-gated; 0 on production boots).
    if (g_fault_station_rx_drop_remaining > 0) {
        g_fault_station_rx_drop_remaining = g_fault_station_rx_drop_remaining - 1;
        return;
    }
#endif
    starcom_handle_rx(me, rx_evt);
}

// ============================================================================
// State Handlers
// ============================================================================

static QState telem_ao_initial(TelemAo * const me, QEvt const * const e) {
    (void)e;

    me->mav_encoder.init();
    me->telem_valid = false;
    // Output mode owned by AO_RCOS (station_output_mode.h)
    // Seed from kDefaultRocketRadioConfig so boot nav_rate_hz is commanded;
    // telem_ao_initial runs after AO_Radio_set_rate and would otherwise
    // snap back to a hardcoded 5 Hz.
    me->rate_hz = rc::kDefaultRocketRadioConfig.nav_rate_hz;
    if (me->rate_hz == 0) { me->rate_hz = 5; }
    me->interval_ms = 1000U / me->rate_hz;
    me->last_tx_ms = 0;
    me->gcs_state = GcsState::kWaitingForGcs;
    me->last_gcs_heartbeat_ms = 0;
    rc::mavlink_rx_init(&me->mavlink_rx, &me->mav_encoder);
    rc::gcs_mavlink_init(&me->gcs);
    me->gcs_heartbeat_count = 0;

    // Subscribe to SIG_RADIO_RX from AO_Radio
    QActive_subscribe(&me->super, rc::SIG_RADIO_RX);
    QActive_subscribe(&me->super, rc::SIG_HEALTH_STATUS);  // health byte

    QTimeEvt_armX(&me->tick_timer, 1U, 1U);
    return Q_TRAN(&telem_ao_running);
}

static QState telem_ao_running(TelemAo * const me, QEvt const * const e) {
    switch (e->sig) {
    case SIG_TELEM_TICK: {
        rc::starcom_adapt::pump_tick(g_pump, static_cast<starcom::ccsds::Tick>(now_ms()));
#ifndef ROCKETCHIP_HOST_TEST
        starcom_poll_mac_radio();
#endif
        starcom_drain_to_radio();  // 211.0 4.1.3.2: emit while TRANSMIT is on
        if constexpr (!job::kRadioModeRx) {
            encode_and_send(me);
        }
        gcs_station_tick(me);
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
static void boot_starcom_session() {
    rc::starcom_adapt::pump_init_for_this_job(g_pump);
#ifndef ROCKETCHIP_HOST_TEST
    const rc::RadioConfig* cfg = AO_Radio_get_runtime_config();
    if (cfg != nullptr) {
        const uint8_t idx = rc::radio_config_catalog_index(
            cfg->bandwidth_khz, cfg->nav_rate_hz, cfg->spreading_factor,
            cfg->coding_rate, cfg->power_dbm);
        if (idx != rc::kRadioConfigNoIndex) {
            g_pump.hail_catalog_idx = idx;
            starcom::ccsds::macLoadHailCommValue(
                g_pump.mac,
                rc::starcom_adapt::pump_comm_value_for_catalog(idx));
        }
    }
#endif
    constexpr bool kCaller = job::kRadioModeRx;
    rc::starcom_adapt::pump_start_session(
        g_pump, kCaller, static_cast<starcom::ccsds::Tick>(now_ms()));
}

void AO_Telemetry_on_radio_phy_applied() {
#ifndef ROCKETCHIP_HOST_TEST
    g_commChangeArmed = false;
    g_remoteApplyOnReceive = false;
#endif
    boot_starcom_session();
    rc::rc_log("[SC] COP-P/MAC reinit after radio PHY apply\n");
}

bool AO_Telemetry_request_comm_change(uint8_t catalog_idx) {
    const bool ok = rc::starcom_adapt::pump_begin_comm_change(
        g_pump, catalog_idx, static_cast<starcom::ccsds::Tick>(now_ms()));
    if (!ok) {
        return false;
    }
    (void)starcom_drain_to_radio();
    rc::rc_log("[SC] COMM_CHANGE queued idx=%u\n",
               static_cast<unsigned>(catalog_idx));
    return true;
}

void AO_Telemetry_set_rate(uint8_t rate_hz) {
    if (rate_hz == 0) { rate_hz = 5; }
    if (rate_hz > 50) { rate_hz = 50; }  // sanity: 50 Hz ~ 20ms period
    g_telemAo.rate_hz = rate_hz;
    g_telemAo.interval_ms = 1000U / rate_hz;
    g_navTickAcc = 0;
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

StarcomLinkStatus AO_Telemetry_get_starcom_link() {
    StarcomLinkStatus s{};
    s.on = true;
    s.peer_plcw = g_pump.copp.fop.plcw_heard;
    s.nav_sdu = g_telemAo.starcom_nav_sdu;
    s.v_s = g_pump.copp.fop.v_s;
    s.nn_r = g_pump.copp.fop.nn_r;
    s.farm_vr = g_pump.copp.farm.v_r;
    s.mac_mode = static_cast<uint8_t>(g_pump.mac.mode);
    s.mac_state = static_cast<uint8_t>(g_pump.mac.state);
    return s;
}

bool AO_Telemetry_drain_after_tx() {
    // 211.0 6.3.2.3: FIFO empty after TxDone, then table 6-14 next fill.
    rc::starcom_adapt::pump_spdu_air_complete(
        g_pump, static_cast<starcom::ccsds::Tick>(now_ms()));
#ifndef ROCKETCHIP_HOST_TEST
    starcom_poll_mac_radio();
#endif
    return starcom_drain_to_radio();
}

void AO_Telemetry_send_command(uint16_t command, const MavCmdParams& params) {
    AO_Telemetry_send_tracked_command(command, params.p1, params.p2,
                                      params.p3, params.p4, params.p5);
}

// Tracked command — pending-cmd state for ACK tracking.
static uint8_t g_cmdSeq = 0;

#ifndef ROCKETCHIP_HOST_TEST

// Populate s_pending_cmd + params (used by both fresh-send and dedupe-replace).
static void populate_pending(uint16_t command, uint8_t seq,
                             const MavCmdParams& params) {
    g_pendingCmd.pending = true;
    g_pendingCmd.seq = seq;
    g_pendingCmd.ns = g_pump.copp.fop.v_s;
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
    // 211.0 7.2.3 SE1: resend the Sent-queue; do not mint a new N(S).
    if (g_pendingCmd.pending) {
        rc::rc_log("[CMD] skip submit, FOP resends pending %u\n",
                   static_cast<unsigned>(g_pendingCmd.cmd_id));
        return;
    }
    uint8_t seq = g_cmdSeq++;
    const MavCmdParams params{p1, p2, p3, p4, p5};
    populate_pending(command, seq, params);
    std::byte pkt[6u + rc::kCmdSduUserBytes];
    const auto n = rc::starcom_adapt::pump_pack_cmd_packet(
        pkt, command, seq, p1, p2, p3, p4, p5);
    if (n.has_value() && *n > 0) {
        const auto sub = rc::starcom_adapt::pump_submit_sdu(
            g_pump, std::span<const std::byte>(pkt, *n), false);
        if (!sub.has_value()) {
            rc::rc_log("[CMD] COP-P submit failed\n");
        }
        starcom_drain_to_radio();
    }
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

void AO_Telemetry_cmd_retry_tick(uint32_t now_ms) {
#ifndef ROCKETCHIP_HOST_TEST
    // COP-P owns on-wire resend. Drop the CLI latch if no ACK
    // (8 × 250 ms seed — same give-up window as homemade retry).
    if (g_pendingCmd.pending) {
        const uint32_t elapsed = now_ms - g_pendingCmd.sent_ms;
        const uint32_t kPendingGiveUpMs =
            static_cast<uint32_t>(kAckMaxRetries) * g_ackRetryTimeoutMs;
        if (elapsed >= kPendingGiveUpMs) {
            g_lastCmdResult.valid  = true;
            g_lastCmdResult.ok     = false;
            g_lastCmdResult.cmd_id = g_pendingCmd.cmd_id;
            g_lastCmdResult.rtt_ms = 0;
            g_lastCmdResult.at_ms  = now_ms;
            g_pendingCmd.pending = false;
            rc::rc_log("[CMD] pending cleared (no ACK in %u ms)\n",
                       static_cast<unsigned>(kPendingGiveUpMs));
        }
    }
#else
    (void)now_ms;
#endif
}

// USB MAVLink in (QGC / Mission Planner). Station does not execute ARM
// on the pad MCU; pad ARM stays the operator path. QGC→Starcom commands
// are a later sitting.
void AO_Telemetry_feed_usb_byte(uint8_t byte) {
#ifndef ROCKETCHIP_HOST_TEST
    mavlink_message_t msg;
    mavlink_status_t status;

    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status) == 0) {
        return;
    }
    if ((msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) && (msg.sysid != 0)) {
        AO_Telemetry_notify_gcs_heartbeat();
        g_telemAo.gcs_heartbeat_count++;
        if constexpr (job::kRadioModeRx) {
            AO_RCOS_set_output_mode(StationOutputMode::kMavlink);
        }
    }
    (void)rc::gcs_mavlink_handle_usb_frame(
        &g_telemAo.gcs, &msg, gcs_usb_sink());
    if constexpr (!job::kRadioModeRx) {
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
    g_telemAo.starcom_nav_sdu = false;
    boot_starcom_session();

    QActive_start(&g_telemAo.super,
                  Q_PRIO(prio, 0U),
                  g_telemAoQueue,
                  Q_DIM(g_telemAoQueue),
                  nullptr, 0U,
                  nullptr);
}
