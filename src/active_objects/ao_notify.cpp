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
#include "rocketchip/notify_backend.h"
#include "safety/health_monitor.h"
#include "pico/time.h"

using namespace rc::notify;

// ============================================================================
// Private signals (offsets from SIG_AO_MAX to avoid collision with other AOs)
// ============================================================================
enum : uint16_t {
    SIG_NOTIFY_TICK        = rc::SIG_AO_MAX + 6,
    SIG_NOTIFY_CAL_INTENT  = rc::SIG_AO_MAX + 7,  // Direct post from AO_RCOS
};

// Direct-post event from AO_RCOS carrying a typed CalIntent
struct CalIntentEvt {
    QEvt super;
    CalIntent intent;
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
static bool     s_notifyStarted = false;

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
    QActive_subscribe(&me->super, rc::SIG_BEACON_ACTIVE);
    // IVP-116: SIG_LED_OVERRIDE subscription removed. RCOS now posts
    // CalIntentEvt directly via AO_Notify_post_cal_intent().

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

    case SIG_NOTIFY_CAL_INTENT: {
        // Direct post from AO_RCOS via AO_Notify_post_cal_intent()
        const auto* ce = reinterpret_cast<const CalIntentEvt*>(e);
        me->state.cal = ce->intent;
        return Q_HANDLED();
    }

    case rc::SIG_BEACON_ACTIVE: {
        me->state.phase = PhaseIntent::kBeacon;
        return Q_HANDLED();
    }

    case SIG_NOTIFY_TICK: {
        // IVP-116: run resolver and dispatch to output backends.
        // IVP-117 will add seqlock read for sensor status before dispatch.
        rc::notify::notify_backend_led_update(me->state);
        rc::notify::notify_backend_audio_update(me->state);
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
    s_notifyStarted = true;
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

static CalIntent s_lastCalIntent = CalIntent::kNone;

void AO_Notify_post_cal_intent(CalIntent intent) {
    // No-op on builds where AO_Notify isn't started (Station/Relay roles).
    if (!s_notifyStarted) {
        return;
    }
    // Deduplicate — avoid queue spam from repeated posts of the same intent
    if (intent == s_lastCalIntent) {
        return;
    }
    s_lastCalIntent = intent;

    // Stack-local event — QV copies into AO_Notify's queue.
    CalIntentEvt evt;
    evt.super = QEVT_INITIALIZER(SIG_NOTIFY_CAL_INTENT);
    evt.intent = intent;
    QACTIVE_POST(&l_notifyAo.super, &evt.super, (void *)0);
}
