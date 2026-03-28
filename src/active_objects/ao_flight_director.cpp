// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_FlightDirector — Flight Director Active Object (IVP-78)
//
// Thin wrapper: 100Hz time event drives flight_director_tick() logic.
// The existing FlightDirector QHsm + guard evaluator are called from
// within the AO's event handler, identical to the superloop pattern.
//
// The FlightDirector struct itself is NOT embedded in this AO — it remains
// a static global in main.cpp because many other modules read its state
// (CLI, logging, telemetry). This AO just owns the tick scheduling.
//============================================================================

#include "ao_flight_director.h"
#include "rocketchip/ao_signals.h"

// Internal signal for the 100Hz tick (private to this AO)
enum : uint16_t {
    SIG_FD_TICK_TIMER = rc::SIG_AO_MAX + 3
};

struct FdAo {
    QActive super;
    QTimeEvt tick_timer;    // 100Hz (every 1 tick at 100Hz base)
};

static FdAo l_fdAo;

// Queue depth 32: tick events accumulate while telemetry_radio_tick() blocks
// in QV_onIdle (rfm95w_send polls DIO0 for 50-150ms LoRa airtime). At 100Hz,
// 150ms = 15 events. Depth 32 gives 2x margin. Real fix: non-blocking LoRa
// driver (see whiteboard deferred notes). (A6, revised after HW test)
static QEvtPtr l_fdAoQueue[32];

// Forward declarations
static QState FdAo_initial(FdAo * const me, QEvt const * const e);
static QState FdAo_running(FdAo * const me, QEvt const * const e);

// Bridge to flight_director_tick() in main.cpp — same function, now called
// from AO event handler instead of qv_idle_bridge().
extern "C" void flight_director_tick(void);

static QState FdAo_initial(FdAo * const me, QEvt const * const e) {
    (void)e;
    // 100Hz tick (every 1 tick at 100Hz base)
    QTimeEvt_armX(&me->tick_timer, 1U, 1U);
    return Q_TRAN(&FdAo_running);
}

static QState FdAo_running(FdAo * const me, QEvt const * const e) {
    (void)me;
    switch (e->sig) {
    case SIG_FD_TICK_TIMER:
        flight_director_tick();
        return Q_HANDLED();
    default:
        break;
    }
    return Q_SUPER(&QHsm_top);
}

//----------------------------------------------------------------------------
// Public interface

QActive * const AO_FlightDirector = &l_fdAo.super;

void AO_FlightDirector_start(uint8_t prio) {
    QActive_ctor(&l_fdAo.super,
                 Q_STATE_CAST(&FdAo_initial));

    QTimeEvt_ctorX(&l_fdAo.tick_timer, &l_fdAo.super,
                   SIG_FD_TICK_TIMER, 0U);

    QActive_start(&l_fdAo.super,
                  Q_PRIO(prio, 0U),
                  l_fdAoQueue,
                  Q_DIM(l_fdAoQueue),
                  (void *)0, 0U,
                  (void *)0);
}
