// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_Blinker — Demo Active Object: Heartbeat LED (IVP-76)
//
// 2-state HSM: LED_OFF ↔ LED_ON, toggling on SIG_BLINKER_TIMEOUT (1Hz).
// Replaces heartbeat_tick() as the first module migrated to an AO.
//============================================================================

#include "ao_blinker.h"
#include "rocketchip/ao_signals.h"
#include "rocketchip/board.h"

// Internal signal for the blinker timer (not in system catalog — private)
enum : uint16_t {
    SIG_BLINKER_TIMEOUT = rc::SIG_AO_MAX  // Private signal, after system signals
};

// Blinker Active Object — extends QActive with a time event
struct Blinker {
    QActive super;          // QP/C base class (must be first)
    QTimeEvt timer;         // 1Hz periodic timer
};

static Blinker l_blinker;  // Single static instance

// Queue buffer — depth 4: only timer events, never more than 1 pending.
// Depth 4 provides margin for timer + potential future signals. (A6)
static QEvtPtr l_blinkerQueue[4];

// Forward declarations for state handlers
static QState Blinker_initial(Blinker * const me, QEvt const * const e);
static QState Blinker_off(Blinker * const me, QEvt const * const e);
static QState Blinker_on(Blinker * const me, QEvt const * const e);

//----------------------------------------------------------------------------
// State handlers

static QState Blinker_initial(Blinker * const me, QEvt const * const e) {
    (void)e;
    // Arm 1Hz periodic timer: 50 ticks initial (500ms), 50 ticks interval
    // at 100Hz tick rate → 500ms on, 500ms off = 1Hz blink
    QTimeEvt_armX(&me->timer, 50U, 50U);
    return Q_TRAN(&Blinker_off);
}

static QState Blinker_off(Blinker * const me, QEvt const * const e) {
    (void)me;
    switch (e->sig) {
    case Q_ENTRY_SIG:
        board::board_led_set(false);
        return Q_HANDLED();
    case SIG_BLINKER_TIMEOUT:
        return Q_TRAN(&Blinker_on);
    default:
        break;
    }
    return Q_SUPER(&QHsm_top);
}

static QState Blinker_on(Blinker * const me, QEvt const * const e) {
    (void)me;
    switch (e->sig) {
    case Q_ENTRY_SIG:
        board::board_led_set(true);
        return Q_HANDLED();
    case SIG_BLINKER_TIMEOUT:
        return Q_TRAN(&Blinker_off);
    default:
        break;
    }
    return Q_SUPER(&QHsm_top);
}

//----------------------------------------------------------------------------
// Public interface

QActive * const AO_Blinker = &l_blinker.super;

void AO_Blinker_start(uint8_t prio) {
    // Construct the AO
    QActive_ctor(&l_blinker.super,
                 Q_STATE_CAST(&Blinker_initial));

    // Construct the time event (tick rate 0 = our 100Hz base)
    QTimeEvt_ctorX(&l_blinker.timer, &l_blinker.super,
                   SIG_BLINKER_TIMEOUT, 0U);

    // Start the AO (QV: stkSto must be NULL, stkSize must be 0)
    QActive_start(&l_blinker.super,
                  Q_PRIO(prio, 0U),
                  l_blinkerQueue,
                  Q_DIM(l_blinkerQueue),
                  (void *)0, 0U,
                  (void *)0);
}
