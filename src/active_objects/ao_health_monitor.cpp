// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_HealthMonitor — System Health Active Object (Stage 13, IVP-105)
//
// Ticks at 10Hz via QF time event. Calls health_monitor_tick() and
// publishes SIG_HEALTH_STATUS on change + 1Hz periodic re-publish.
// Subscribes to SIG_PHASE_CHANGE for fault-latch phase tracking.
//============================================================================

#include "ao_health_monitor.h"
#include "safety/health_monitor.h"
#include "safety/test_mode.h"          // R-25-exec runtime gate
#include "rocketchip/ao_signals.h"

// ============================================================================
// Constants
// ============================================================================

// Private timer signal (after system signals)
enum : uint16_t {
    SIG_HEALTH_TIMEOUT = rc::SIG_AO_MAX + 1
};

// 10Hz health tick: 100Hz base / 10 = every 10 ticks
static constexpr uint16_t kHealthTickInterval = 10U;

// 1Hz re-publish: every 10th health tick (10Hz / 10 = 1Hz)
static constexpr uint8_t kRepublishDivider = 10U;

// ============================================================================
// AO struct
// ============================================================================

struct HealthMonitor {
    QActive super;          // QP/C base class (must be first)
    QTimeEvt timer;         // 10Hz periodic timer
    uint8_t republish_count;  // Divider for 1Hz re-publish
};

static HealthMonitor l_hm;

// Queue depth 8: 10Hz timer + occasional SIG_PHASE_CHANGE.
// Council: queue depth 8 is sufficient.
static QEvtPtr l_hmQueue[8];

// Forward declarations
static QState HM_initial(HealthMonitor * const me, QEvt const * const e);
static QState HM_running(HealthMonitor * const me, QEvt const * const e);

// ============================================================================
// Internal: publish health status
// ============================================================================

static void hm_publish(HealthMonitor * const me) {
    const rc::HealthState* hs = rc::health_monitor_get_state();
    static rc::HealthStatusEvt evt;
    evt.super.sig = rc::SIG_HEALTH_STATUS;
    evt.primary = hs->primary;
    evt.secondary = hs->secondary;
    QActive_publish_(&evt.super, &me->super, me->super.prio);
}

// ============================================================================
// State handlers
// ============================================================================

static QState HM_initial(HealthMonitor * const me, QEvt const * const e) {
    (void)e;

    // Subscribe to phase changes for fault-latch tracking
    QActive_subscribe(&me->super, rc::SIG_PHASE_CHANGE);

    // Initialize health monitor module
    rc::health_monitor_init();

    // Arm 10Hz periodic timer: initial delay 10 ticks, interval 10 ticks
    QTimeEvt_armX(&me->timer, kHealthTickInterval, kHealthTickInterval);

    // Publish initial state so consumers get health on startup
    hm_publish(me);

    me->republish_count = 0;

    return Q_TRAN(&HM_running);
}

static QState HM_running(HealthMonitor * const me, QEvt const * const e) {
    switch (e->sig) {
    case SIG_HEALTH_TIMEOUT: {
        // R-25-exec: re-evaluate the three-condition test-mode gate
        // (probe-arm-magic + phase==kIdle + boot-time-window). Cheap
        // (handful of memory reads + one phase accessor call); folded
        // into the existing 10 Hz health tick so test-mode liveness
        // tracks health monitor's existing surface.
        rc::test_mode_evaluate();

        bool changed = rc::health_monitor_tick();

        me->republish_count++;
        if (me->republish_count >= kRepublishDivider) {
            me->republish_count = 0;
            // 1Hz forced re-publish (council amendment 2: prevents startup-order bugs)
            hm_publish(me);
        } else if (changed) {
            // On-change publish
            hm_publish(me);
        }
        return Q_HANDLED();
    }

    case rc::SIG_PHASE_CHANGE: {
        auto const* pce = reinterpret_cast<rc::PhaseChangeEvt const*>(e);
        rc::health_monitor_set_phase(pce->phase);
        return Q_HANDLED();
    }

    default:
        break;
    }
    return Q_SUPER(&QHsm_top);
}

// ============================================================================
// Public interface
// ============================================================================

QActive * const AO_HealthMonitor = &l_hm.super;

void AO_HealthMonitor_start(uint8_t prio) {
    QActive_ctor(&l_hm.super, Q_STATE_CAST(&HM_initial));

    QTimeEvt_ctorX(&l_hm.timer, &l_hm.super, SIG_HEALTH_TIMEOUT, 0U);

    QActive_start(&l_hm.super,
                  Q_PRIO(prio, 0U),
                  l_hmQueue,
                  Q_DIM(l_hmQueue),
                  (void *)0, 0U,
                  (void *)0);
}
