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
#include "rocketchip/telemetry_service.h"
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
#include "rocketchip/job.h"
#include "flight_director/mission_profile_data.h"  // kDefaultRocketRadioConfig

#ifndef ROCKETCHIP_HOST_TEST
#include "pico/time.h"
#include "pico/stdio_usb.h"
#include "tusb.h"
#include <stdio.h>
#endif

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

// Encode and post SIG_RADIO_TX to AO_Radio (Vehicle TX path)
static void encode_and_send(TelemAo* me) {
    if (!me->telem_valid) { return; }

    uint32_t t = now_ms();
    if (t - me->last_tx_ms < me->interval_ms) { return; }
    me->last_tx_ms = t;

    // Encode packet based on RadioConfig protocol selection (IVP-65)
    rc::EncodeResult result = {};
    if (rc::kDefaultRocketRadioConfig.protocol == rc::EncoderType::kMavlink) {
        // MAVLink native TX — encode heartbeat + attitude + position into single packet
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
        // CCSDS (default)
        me->ccsds_encoder.encode_nav(me->latest_telem, me->latest_telem.met_ms, result);
    }
    if (!result.ok || result.len == 0) { return; }

    // Post SIG_RADIO_TX to AO_Radio — file-scope static event.
    // QV cooperative scheduling: handler runs to completion before any other
    // AO processes it, so one static instance is safe (no concurrent access).
    static rc::RadioTxEvt txEvt;
    txEvt.super.sig = rc::SIG_RADIO_TX;
    txEvt.super.refCtr_ = 0;  // Reset ref counter (QP/C static event pattern)
    memcpy(txEvt.buf, result.buf, result.len);
    txEvt.len = static_cast<uint8_t>(result.len);

    QACTIVE_POST(AO_Radio, &txEvt.super, me);
}

// LoRa MAVLink RX — uses MAVLINK_COMM_2 (separate from USB on COMM_1)
static void try_mavlink_rx(TelemAo* me, const uint8_t* buf, uint8_t len) {
    mavlink_message_t msg;
    mavlink_status_t status;

    for (uint8_t i = 0; i < len; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_2, buf[i], &msg, &status)) {
            // Complete MAVLink frame parsed — dispatch through mavlink_rx handler
            rc::MavlinkRxResult result = {};
            rc::mavlink_rx_feed_byte(&me->mavlink_rx, buf[i],
                                      me->latest_telem.flight_state,
                                      now_ms(), &result);

            // Actually we need to dispatch the already-parsed message.
            // The feed_byte won't re-parse since COMM_2 already consumed it.
            // Instead, call the dispatcher directly with the parsed message.
            // For now, just check if we got a heartbeat for GCS detection.
            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                AO_Telemetry_notify_gcs_heartbeat();
            }

            // Generate ACK response for commands
            if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
                mavlink_command_long_t cmd;
                mavlink_msg_command_long_decode(&msg, &cmd);

                // Dispatch command to Flight Director
                if (cmd.command == MAV_CMD_COMPONENT_ARM_DISARM) {
                    uint16_t sig = (cmd.param1 > 0.5f)
                        ? static_cast<uint16_t>(rc::SIG_ARM)
                        : static_cast<uint16_t>(rc::SIG_DISARM);
                    AO_FlightDirector_dispatch_signal(sig);
                } else if (cmd.command == MAV_CMD_DO_FLIGHTTERMINATION) {
                    AO_FlightDirector_dispatch_signal(
                        static_cast<uint16_t>(rc::SIG_ABORT));
                }
                // No printf here — ASCII text corrupts binary MAVLink stream
            }
        }
    }
}

// Handle received packet from AO_Radio
static void handle_rx_packet(TelemAo* me, const rc::RadioRxEvt* rxEvt) {
    // Try CCSDS decode first (telemetry packets)
    rc::TelemetryState telem = {};
    uint16_t seq = 0;
    uint32_t met_ms = 0;
    if (!rc::ccsds_decode_nav(rxEvt->buf, rxEvt->len, telem, seq, met_ms)) {
        // Not CCSDS — try MAVLink (command packets from station)
        try_mavlink_rx(me, rxEvt->buf, rxEvt->len);
        return;
    }

    // Store for CLI/WiFi access
    me->rx_snapshot.telem = telem;
    me->rx_snapshot.met_ms = met_ms;
    me->rx_snapshot.seq = seq;
    me->rx_snapshot.valid = true;

#ifndef ROCKETCHIP_HOST_TEST
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
        // No flush — usb_write_nonblocking handles it
        break;
    }
    case StationOutputMode::kCsv:
        printf("RX,%u,%d,%d\n",
               static_cast<unsigned>(seq),
               static_cast<int>(rxEvt->rssi),
               static_cast<int>(rxEvt->snr));
        break;
    case StationOutputMode::kAnsi:
    case StationOutputMode::kMenu:
        // ANSI: rendered from main loop. Menu: output suppressed.
        break;
    }
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
    me->rate_hz = 2;
    me->interval_ms = 500;
    me->last_tx_ms = 0;
    me->last_heartbeat_ms = 0;
    me->gcs_state = GcsState::kWaitingForGcs;
    me->last_gcs_heartbeat_ms = 0;
    rc::mavlink_rx_init(&me->mavlink_rx, &me->mav_encoder);
    me->gcs_heartbeat_count = 0;
    me->param_send_idx = -1;

    // Subscribe to SIG_RADIO_RX from AO_Radio
    QActive_subscribe(&me->super, rc::SIG_RADIO_RX);

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
            l_telemAo.rate_hz = next;
            l_telemAo.interval_ms = 1000 / next;
            return next;
        }
    }
    l_telemAo.rate_hz = 2;
    l_telemAo.interval_ms = 500;
    return 2;
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
