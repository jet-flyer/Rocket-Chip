// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_Notify — Notification Hub Active Object
//
// Subscribes to state-producing signals (phase change, health, radio,
// beacon, calibration override). Maintains NotifyState with per-category
// typed intents. At 33Hz, runs the priority resolver and dispatches to
// registered output backends (LED, future audio).
//
// Priority: 5 (between HealthMonitor=6 and Logger=4 after IVP-114 reshuffle)
// Queue depth: 16
// Tick rate: 33Hz (every 3 ticks at 100Hz base)
//============================================================================

#include "ao_notify.h"
#include "rocketchip/ao_signals.h"
#include "rocketchip/led_patterns.h"
#include "safety/health_monitor.h"
#include "pico/time.h"

using namespace rc::notify;

// ============================================================================
// Private tick signal
// ============================================================================
enum : uint16_t {
    SIG_NOTIFY_TICK = rc::SIG_AO_MAX + 6
};

// ============================================================================
// AO struct
// ============================================================================
struct NotifyAo {
    QActive super;
    QTimeEvt tick_timer;        // 33Hz (every 3 ticks at 100Hz base)
    NotifyState state;          // Per-category intent state

    // Sensor evaluation fields (used in IVP-117 migration)
    uint32_t sensor_phase_start_ms;
    uint32_t last_core1_count;
    uint8_t  core1_stall_ticks;
};

static NotifyAo l_notifyAo;
static QEvtPtr  l_notifyQueue[16];

// Forward declarations
static QState Notify_initial(NotifyAo * const me, QEvt const * const e);
static QState Notify_running(NotifyAo * const me, QEvt const * const e);

// ============================================================================
// Mapping helpers
// ============================================================================

static PhaseIntent phase_from_flight_phase(uint8_t phase) {
    // FlightPhase enum: kIdle=0, kArmed=1, kBoost=2, kCoast=3,
    //   kDrogueDescent=4, kMainDescent=5, kLanded=6, kAbort=7
    switch (phase) {
        case 0: return PhaseIntent::kIdle;
        case 1: return PhaseIntent::kArmed;
        case 2: return PhaseIntent::kBoost;
        case 3: return PhaseIntent::kCoast;
        case 4: return PhaseIntent::kDrogue;
        case 5: return PhaseIntent::kMain;
        case 6: return PhaseIntent::kLanded;
        case 7: return PhaseIntent::kAbort;
        default: return PhaseIntent::kNone;
    }
}

static RadioIntent radio_intent_from_lq(uint8_t lq) {
    switch (lq) {
        case 1: return RadioIntent::kLost;
        case 2: return RadioIntent::kGap;
        case 3: return RadioIntent::kReceiving;
        default: return RadioIntent::kNone;
    }
}

static FaultIntent notify_decode_health_faults(uint8_t primary, uint8_t secondary) {
    FaultIntent max_fault = FaultIntent::kNone;

    if (rc::health_imu(primary) == rc::kHealthFault) {
        max_fault = FaultIntent::kImuFail;
    }
    if (rc::health_eskf(primary) == rc::kHealthFault &&
        FaultIntent::kEskfFail > max_fault) {
        max_fault = FaultIntent::kEskfFail;
    }
    if (rc::health_baro(primary) == rc::kHealthFault &&
        FaultIntent::kBaroFail > max_fault) {
        max_fault = FaultIntent::kBaroFail;
    }
    if ((secondary & rc::kHealthPioOk) == 0 &&
        FaultIntent::kPioWdt > max_fault) {
        max_fault = FaultIntent::kPioWdt;
    }
    if ((secondary & rc::kHealthWatchdogOk) == 0 &&
        FaultIntent::kSafeMode > max_fault) {
        max_fault = FaultIntent::kSafeMode;
    }
    // Core1 stall: will be added to secondary in IVP-117 (kHealthCore1Ok bit)
    return max_fault;
}

static CalIntent cal_intent_from_pattern(uint8_t pattern) {
    switch (pattern) {
        case rc::led::kCalGyro:        return CalIntent::kGyro;
        case rc::led::kCalLevel:       return CalIntent::kLevel;
        case rc::led::kCalBaro:        return CalIntent::kBaro;
        case rc::led::kCalAccelWait:   return CalIntent::kAccelWait;
        case rc::led::kCalAccelSample: return CalIntent::kAccelSample;
        case rc::led::kCalMag:         return CalIntent::kMag;
        case rc::led::kCalSuccess:     return CalIntent::kSuccess;
        case rc::led::kCalFail:        return CalIntent::kFail;
        case rc::led::kOff:
        default:                       return CalIntent::kNone;
    }
}

// ============================================================================
// State handlers
// ============================================================================

static QState Notify_initial(NotifyAo * const me, QEvt const * const e) {
    (void)e;

    // Zero-init intent state
    me->state = {};
    me->sensor_phase_start_ms = to_ms_since_boot(get_absolute_time());
    me->last_core1_count = 0;
    me->core1_stall_ticks = 0;

    // Subscribe to state-producing signals
    QActive_subscribe(&me->super, rc::SIG_PHASE_CHANGE);
    QActive_subscribe(&me->super, rc::SIG_RADIO_STATUS);
    QActive_subscribe(&me->super, rc::SIG_HEALTH_STATUS);
    QActive_subscribe(&me->super, rc::SIG_LED_OVERRIDE);    // Bridge until IVP-116
    QActive_subscribe(&me->super, rc::SIG_BEACON_ACTIVE);

    // 33Hz animation tick (every 3 ticks at 100Hz base)
    QTimeEvt_armX(&me->tick_timer, 3U, 3U);

    return Q_TRAN(&Notify_running);
}

static QState Notify_running(NotifyAo * const me, QEvt const * const e) {
    switch (e->sig) {

    case rc::SIG_PHASE_CHANGE: {
        const auto* pce = reinterpret_cast<const rc::PhaseChangeEvt*>(e);
        me->state.phase = phase_from_flight_phase(pce->phase);
        return Q_HANDLED();
    }

    case rc::SIG_RADIO_STATUS: {
        const auto* re = reinterpret_cast<const rc::RadioStatusEvt*>(e);
        me->state.radio = radio_intent_from_lq(re->link_quality);
        return Q_HANDLED();
    }

    case rc::SIG_HEALTH_STATUS: {
        const auto* he = reinterpret_cast<const rc::HealthStatusEvt*>(e);
        me->state.fault = notify_decode_health_faults(he->primary, he->secondary);
        return Q_HANDLED();
    }

    case rc::SIG_LED_OVERRIDE: {
        // Bridge: RCOS still posts old kCalNeo* pattern codes until IVP-116
        const auto* pe = reinterpret_cast<const rc::LedPatternEvt*>(e);
        me->state.cal = cal_intent_from_pattern(pe->pattern);
        return Q_HANDLED();
    }

    case rc::SIG_BEACON_ACTIVE: {
        me->state.phase = PhaseIntent::kBeacon;
        return Q_HANDLED();
    }

    case SIG_NOTIFY_TICK: {
        // IVP-114: no-op — resolver wired in IVP-116
        // IVP-117 will add seqlock read for sensor status here
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

QActive * const AO_Notify = &l_notifyAo.super;

void AO_Notify_start(uint8_t prio) {
    QActive_ctor(&l_notifyAo.super,
                 Q_STATE_CAST(&Notify_initial));

    QTimeEvt_ctorX(&l_notifyAo.tick_timer, &l_notifyAo.super,
                   SIG_NOTIFY_TICK, 0U);

    QActive_start(&l_notifyAo.super,
                  Q_PRIO(prio, 0U),
                  l_notifyQueue,
                  Q_DIM(l_notifyQueue),
                  (void *)0, 0U,
                  (void *)0);
}

void AO_Notify_post_cal_intent(CalIntent intent) {
    // IVP-114: stub. RCOS still posts SIG_LED_OVERRIDE directly to LedEngine.
    // AO_Notify receives it via subscription bridge (SIG_LED_OVERRIDE → cal_intent_from_pattern).
    // IVP-116 will rewire RCOS to call this function, which will post a
    // CalIntentEvt directly to AO_Notify's queue.
    (void)intent;
}
