// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_Logger — Flight Data Logger Active Object (IVP-79)
//
// Thin wrapper: 50Hz time event drives logging_tick() logic.
// The logging state (ring buffer, decimator, flash flush) remains in
// main.cpp — this AO owns the scheduling.
//============================================================================

#include "ao_logger.h"
#include "rocketchip/ao_signals.h"

// Internal signal (private)
enum : uint16_t {
    SIG_LOG_TICK = rc::SIG_AO_MAX + 4
};

struct LoggerAo {
    QActive super;
    QTimeEvt tick_timer;    // 50Hz (every 2 ticks at 100Hz base)
};

static LoggerAo l_loggerAo;

// Queue depth 32: tick events at 50Hz accumulate during LoRa TX blocking in
// idle (50-150ms). At 50Hz, 150ms = ~8 events. Depth 32 gives ample margin.
static QEvtPtr l_loggerAoQueue[32];

static QState LoggerAo_initial(LoggerAo * const me, QEvt const * const e);
static QState LoggerAo_running(LoggerAo * const me, QEvt const * const e);

// Bridge to logging_tick() in main.cpp
extern "C" void logging_tick(void);

static QState LoggerAo_initial(LoggerAo * const me, QEvt const * const e) {
    (void)e;
    // 50Hz tick (every 2 ticks at 100Hz base)
    QTimeEvt_armX(&me->tick_timer, 2U, 2U);
    return Q_TRAN(&LoggerAo_running);
}

static QState LoggerAo_running(LoggerAo * const me, QEvt const * const e) {
    (void)me;
    switch (e->sig) {
    case SIG_LOG_TICK:
        logging_tick();
        return Q_HANDLED();
    default:
        break;
    }
    return Q_SUPER(&QHsm_top);
}

QActive * const AO_Logger = &l_loggerAo.super;

void AO_Logger_start(uint8_t prio) {
    QActive_ctor(&l_loggerAo.super,
                 Q_STATE_CAST(&LoggerAo_initial));

    QTimeEvt_ctorX(&l_loggerAo.tick_timer, &l_loggerAo.super,
                   SIG_LOG_TICK, 0U);

    QActive_start(&l_loggerAo.super,
                  Q_PRIO(prio, 0U),
                  l_loggerAoQueue,
                  Q_DIM(l_loggerAoQueue),
                  (void *)0, 0U,
                  (void *)0);
}
