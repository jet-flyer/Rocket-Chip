// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_Counter — Demo Active Object: Jitter-Measuring Event Counter (IVP-76)
//
// Measures inter-dispatch timing of 10Hz QF time events. Every 50 events
// (~5 seconds), prints count + jitter statistics to serial. Jitter > 1ms
// signals QV_onIdle processing time regression (Council Amendment A5).
//============================================================================

#include "ao_counter.h"
#include "rocketchip/ao_signals.h"

#ifndef ROCKETCHIP_HOST_TEST
#include "pico/time.h"
#include <cstdio>
#endif

// Internal signal (private to this AO)
enum : uint16_t {
    SIG_COUNTER_TIMEOUT = rc::SIG_AO_MAX + 1  // After blinker's private signal
};

struct Counter {
    QActive super;          // QP/C base class (must be first)
    QTimeEvt timer;         // 10Hz periodic timer
    uint32_t count;         // Total events dispatched
    uint32_t last_us;       // Timestamp of previous dispatch (microseconds)
    uint32_t jitter_min_us; // Min inter-dispatch time in current window
    uint32_t jitter_max_us; // Max inter-dispatch time in current window
    uint32_t jitter_sum_us; // Sum for average calculation
    uint32_t window_count;  // Events in current measurement window
};

static Counter l_counter;

// Queue buffer — depth 4: only timer events, max 1 pending at a time.
// Depth 4 for margin (same rationale as blinker). (A6)
static QEvtPtr l_counterQueue[4];

// Forward declarations
static QState Counter_initial(Counter * const me, QEvt const * const e);
static QState Counter_running(Counter * const me, QEvt const * const e);

//----------------------------------------------------------------------------
// Platform-specific helpers

#ifndef ROCKETCHIP_HOST_TEST
static inline uint32_t now_us(void) {
    return time_us_32();
}
static inline void counter_print(uint32_t count, uint32_t min_us,
                                  uint32_t max_us, uint32_t avg_us) {
    printf("[AO_Counter] count=%lu jitter min=%lu avg=%lu max=%lu us\n",
           (unsigned long)count,
           (unsigned long)min_us,
           (unsigned long)avg_us,
           (unsigned long)max_us);
}
#else
static inline uint32_t now_us(void) { return 0; }
static inline void counter_print(uint32_t, uint32_t, uint32_t, uint32_t) {}
#endif

//----------------------------------------------------------------------------
// State handlers

static QState Counter_initial(Counter * const me, QEvt const * const e) {
    (void)e;
    me->count = 0U;
    me->last_us = 0U;
    me->jitter_min_us = UINT32_MAX;
    me->jitter_max_us = 0U;
    me->jitter_sum_us = 0U;
    me->window_count = 0U;

    // 10Hz periodic timer: 10 ticks at 100Hz base = 100ms period
    QTimeEvt_armX(&me->timer, 10U, 10U);
    return Q_TRAN(&Counter_running);
}

static QState Counter_running(Counter * const me, QEvt const * const e) {
    switch (e->sig) {
    case SIG_COUNTER_TIMEOUT: {
        me->count++;

        uint32_t now = now_us();
        if (me->last_us != 0U) {
            uint32_t delta = now - me->last_us;
            if (delta < me->jitter_min_us) {
                me->jitter_min_us = delta;
            }
            if (delta > me->jitter_max_us) {
                me->jitter_max_us = delta;
            }
            me->jitter_sum_us += delta;
            me->window_count++;
        }
        me->last_us = now;

        // Print every 50 events (~5 seconds at 10Hz)
        if (me->window_count >= 50U) {
            uint32_t avg = me->jitter_sum_us / me->window_count;
            counter_print(me->count, me->jitter_min_us,
                          me->jitter_max_us, avg);

            // Reset window
            me->jitter_min_us = UINT32_MAX;
            me->jitter_max_us = 0U;
            me->jitter_sum_us = 0U;
            me->window_count = 0U;
        }
        return Q_HANDLED();
    }
    default:
        break;
    }
    return Q_SUPER(&QHsm_top);
}

//----------------------------------------------------------------------------
// Public interface

QActive * const AO_Counter = &l_counter.super;

void AO_Counter_start(uint8_t prio) {
    QActive_ctor(&l_counter.super,
                 Q_STATE_CAST(&Counter_initial));

    QTimeEvt_ctorX(&l_counter.timer, &l_counter.super,
                   SIG_COUNTER_TIMEOUT, 0U);

    QActive_start(&l_counter.super,
                  Q_PRIO(prio, 0U),
                  l_counterQueue,
                  Q_DIM(l_counterQueue),
                  (void *)0, 0U,
                  (void *)0);
}
