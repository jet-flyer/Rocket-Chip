// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_Telemetry — Telemetry Protocol Active Object (IVP-94)
//
// Protocol-only: CCSDS/MAVLink encoding, APID mux, rate dividers.
// No radio hardware references. Posts SIG_RADIO_TX to AO_Radio.
// Receives SIG_RADIO_RX for decode + output.
//
// Replaces IVP-80 thin wrapper with proper protocol/hardware separation.
//============================================================================

#include "ao_telemetry.h"
#include "ao_radio.h"
#include "rocketchip/ao_signals.h"
#include "rocketchip/telemetry_encoder.h"
#include "rocketchip/telemetry_service.h"
#include "rocketchip/job.h"

#ifndef ROCKETCHIP_HOST_TEST
#include "pico/time.h"
#include "pico/stdio_usb.h"
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

struct TelemAo {
    QActive super;
    QTimeEvt tick_timer;    // 10Hz (every 10 ticks at 100Hz base)

    // Protocol state (moved from main.cpp g_telemService)
    rc::CcsdsEncoder    ccsds_encoder;
    rc::MavlinkEncoder  mav_encoder;
    rc::TelemetryState  latest_telem;
    bool                telem_valid;
    bool                mavlink_output;   // true = MAVLink binary, false = CSV text
    uint8_t             rate_hz;
    uint32_t            interval_ms;
    uint32_t            last_tx_ms;
    uint32_t            last_heartbeat_ms;
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

    // Encode CCSDS nav packet
    rc::EncodeResult result = {};
    me->ccsds_encoder.encode_nav(me->latest_telem, me->latest_telem.met_ms, result);
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

// Handle received packet from AO_Radio (Station RX path)
static void handle_rx_packet(TelemAo* me, const rc::RadioRxEvt* rxEvt) {
    // Decode CCSDS
    rc::TelemetryState telem = {};
    uint16_t seq = 0;
    uint32_t met_ms = 0;
    if (!rc::ccsds_decode_nav(rxEvt->buf, rxEvt->len, telem, seq, met_ms)) {
        return;  // CRC or header error
    }

#ifndef ROCKETCHIP_HOST_TEST
    if (me->mavlink_output) {
        // MAVLink binary output on USB
        uint8_t frame[64];
        uint16_t len = 0;
        uint32_t t = now_ms();

        // 1 Hz heartbeat + SYS_STATUS
        if (t - me->last_heartbeat_ms >= 1000) {
            me->last_heartbeat_ms = t;
            len = me->mav_encoder.encode_heartbeat(telem.flight_state, frame);
            fwrite(frame, 1, len, stdout);
            len = me->mav_encoder.encode_sys_status(telem, frame);
            fwrite(frame, 1, len, stdout);
        }

        // ATTITUDE + GLOBAL_POSITION_INT per packet
        len = me->mav_encoder.encode_attitude(telem, t, frame);
        fwrite(frame, 1, len, stdout);
        len = me->mav_encoder.encode_global_pos(telem, t, frame);
        fwrite(frame, 1, len, stdout);
        fflush(stdout);
    } else {
        // CSV text output — minimal for now, full format in telemetry_service
        // (Reuse the existing print_rx_csv pattern from telemetry_service.cpp)
        printf("RX,%u,%d,%d\n",
               static_cast<unsigned>(seq),
               static_cast<int>(rxEvt->rssi),
               static_cast<int>(rxEvt->snr));
    }
#else
    (void)me;
#endif
}

// Direct USB MAVLink output for Vehicle mode (same data, no radio)
static void mavlink_direct_tick(TelemAo* me) {
#ifndef ROCKETCHIP_HOST_TEST
    if (!me->mavlink_output) { return; }
    if constexpr (kRadioModeRx) { return; }  // Station uses RX path
    if (!me->telem_valid) { return; }
    if (!stdio_usb_connected()) { return; }

    uint8_t frame[64];
    uint16_t len = 0;
    uint32_t t = now_ms();

    // 1 Hz heartbeat + SYS_STATUS
    if (t - me->last_heartbeat_ms >= 1000) {
        me->last_heartbeat_ms = t;
        len = me->mav_encoder.encode_heartbeat(
            me->latest_telem.flight_state, frame);
        fwrite(frame, 1, len, stdout);
        len = me->mav_encoder.encode_sys_status(me->latest_telem, frame);
        fwrite(frame, 1, len, stdout);
    }

    // 10 Hz ATTITUDE + GLOBAL_POSITION_INT
    len = me->mav_encoder.encode_attitude(me->latest_telem, t, frame);
    fwrite(frame, 1, len, stdout);
    len = me->mav_encoder.encode_global_pos(me->latest_telem, t, frame);
    fwrite(frame, 1, len, stdout);
    fflush(stdout);
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
    me->mavlink_output = kDefaultMavlinkOutput;
    me->rate_hz = 2;
    me->interval_ms = 500;
    me->last_tx_ms = 0;
    me->last_heartbeat_ms = 0;

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

bool AO_Telemetry_get_mavlink_output() {
    return l_telemAo.mavlink_output;
}

void AO_Telemetry_toggle_mavlink() {
    l_telemAo.mavlink_output = !l_telemAo.mavlink_output;
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

void AO_Telemetry_start(uint8_t prio) {
    QActive_ctor(&l_telemAo.super,
                 Q_STATE_CAST(&TelemAo_initial));

    QTimeEvt_ctorX(&l_telemAo.tick_timer, &l_telemAo.super,
                   SIG_TELEM_TICK, 0U);

    memset(&l_telemAo.latest_telem, 0, sizeof(l_telemAo.latest_telem));
    l_telemAo.telem_valid = false;

    QActive_start(&l_telemAo.super,
                  Q_PRIO(prio, 0U),
                  l_telemAoQueue,
                  Q_DIM(l_telemAoQueue),
                  (void *)0, 0U,
                  (void *)0);
}
