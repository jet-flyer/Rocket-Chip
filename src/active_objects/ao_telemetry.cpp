// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_Telemetry — Telemetry Active Object (IVP-80)
//
// Thin wrapper: 10Hz time event drives telemetry_radio_tick() and
// mavlink_direct_tick(). The telemetry service state remains in main.cpp.
//============================================================================

#include "ao_telemetry.h"
#include "rocketchip/ao_signals.h"

#ifndef ROCKETCHIP_HOST_TEST
#include "pico/time.h"
#endif

// Internal signal (private)
enum : uint16_t {
    SIG_TELEM_TICK = rc::SIG_AO_MAX + 5
};

struct TelemAo {
    QActive super;
    QTimeEvt tick_timer;    // 10Hz (every 10 ticks at 100Hz base)
};

static TelemAo l_telemAo;

// Queue depth 4: tick events only, max 1 pending at a time. (A6)
static QEvtPtr l_telemAoQueue[4];

static QState TelemAo_initial(TelemAo * const me, QEvt const * const e);
static QState TelemAo_running(TelemAo * const me, QEvt const * const e);

// Bridges to tick functions in main.cpp
extern "C" void telemetry_radio_tick(uint32_t nowMs);
extern "C" void mavlink_direct_tick(uint32_t nowMs);

static QState TelemAo_initial(TelemAo * const me, QEvt const * const e) {
    (void)e;
    // 10Hz tick (every 10 ticks at 100Hz base)
    QTimeEvt_armX(&me->tick_timer, 10U, 10U);
    return Q_TRAN(&TelemAo_running);
}

static QState TelemAo_running(TelemAo * const me, QEvt const * const e) {
    (void)me;
    switch (e->sig) {
    case SIG_TELEM_TICK: {
#ifndef ROCKETCHIP_HOST_TEST
        uint32_t nowMs = to_ms_since_boot(get_absolute_time());
#else
        uint32_t nowMs = 0;
#endif
        telemetry_radio_tick(nowMs);
        mavlink_direct_tick(nowMs);
        return Q_HANDLED();
    }
    default:
        break;
    }
    return Q_SUPER(&QHsm_top);
}

QActive * const AO_Telemetry = &l_telemAo.super;

void AO_Telemetry_start(uint8_t prio) {
    QActive_ctor(&l_telemAo.super,
                 Q_STATE_CAST(&TelemAo_initial));

    QTimeEvt_ctorX(&l_telemAo.tick_timer, &l_telemAo.super,
                   SIG_TELEM_TICK, 0U);

    QActive_start(&l_telemAo.super,
                  Q_PRIO(prio, 0U),
                  l_telemAoQueue,
                  Q_DIM(l_telemAoQueue),
                  (void *)0, 0U,
                  (void *)0);
}
