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
#include "rocketchip/prearm_fail_ticks.h"   // Stage L — pure helper
#include "rocketchip/sensor_seqlock.h"
#include "notify/notify_resolver.h"        // decode_health_faults()
#include "safety/health_monitor.h"
#include "core1/sensor_core1.h"            // g_gpsInitialized
#include "fusion/eskf_runner.h"            // eskf_runner_is_initialized()
#include "pico/time.h"

using namespace rc::notify;

// ============================================================================
// Private signals (offsets from SIG_AO_MAX to avoid collision with other AOs)
// ============================================================================
enum : uint16_t {
    SIG_NOTIFY_TICK           = rc::SIG_AO_MAX + 6,
    SIG_NOTIFY_CAL_INTENT     = rc::SIG_AO_MAX + 7,  // Direct post from AO_RCOS
    SIG_NOTIFY_PREARM_FAIL    = rc::SIG_AO_MAX + 8,  // Stage L — ARM rejected
    SIG_NOTIFY_VEHICLE_LOST   = rc::SIG_AO_MAX + 9,  // IVP-T14 #10 — link lost
    SIG_NOTIFY_VEHICLE_FOUND  = rc::SIG_AO_MAX + 10, // IVP-T14 #10 — re-acquired
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

    // Stage L — pre-arm-fail auto-clear counter (ticks remaining).
    // 0 = cleared; 1..kPreArmFailTicks = active. Decremented each 33Hz tick
    // via the pure helper in include/rocketchip/prearm_fail_ticks.h.
    uint32_t prearm_fail_ticks;

    // Stage L IVP-L4 — boot init rainbow minimum visibility. Counts the
    // first ~3s of kInit so the rainbow is seen even when ESKF + IMU are
    // already ready by the time Notify starts ticking (typical on warm
    // resets). At 33 Hz, 99 ticks ≈ 3.0 s.
    uint32_t init_min_ticks;
};

// Minimum boot-rainbow visibility before kInit can auto-clear (ticks at 33Hz).
static constexpr uint32_t kInitMinTicks = 99U;

static NotifyAo g_notifyAo;
static QEvtPtr  g_notifyQueue[16];
static bool     g_notifyStarted = false;

// Forward declarations
static QState notify_initial(NotifyAo * const me, QEvt const * const e);
static QState notify_running(NotifyAo * const me, QEvt const * const e);

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

// IVP-117: notify_decode_health_faults() moved to notify_resolver.h as
// an inline function so host tests can use it without AO dependencies.

// ============================================================================
// Sensor status evaluation (IVP-117: migrated from AO_LedEngine)
//
// Reads the current sensor seqlock snapshot and maps to SensorIntent.
// Called from the 33Hz tick handler.
// ============================================================================

// Sensor phase 5-min timeout — matches the pre-IVP-117 value in LedEngine.
static constexpr uint32_t kSensorPhaseTimeoutMs = 300000U;

static void notify_evaluate_sensor_status(NotifyAo * const me,
                                           const shared_sensor_data_t* snap) {
    const uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    if ((now_ms - me->sensor_phase_start_ms) >= kSensorPhaseTimeoutMs) {
        me->state.sensor = SensorIntent::kTimeout;
    } else if (!eskf_runner_is_initialized()) {
        me->state.sensor = SensorIntent::kEskfInit;
    } else if (g_gpsInitialized) {
        if (snap->gps_fix_type >= 3) {
            me->state.sensor = SensorIntent::kGps3d;
        } else if (snap->gps_fix_type == 2) {
            me->state.sensor = SensorIntent::kGps2d;
        } else if (snap->gps_read_count > 0) {
            me->state.sensor = SensorIntent::kGpsSearch;
        } else {
            me->state.sensor = SensorIntent::kGpsNoNmea;
        }
    } else {
        me->state.sensor = SensorIntent::kNoGps;
    }
}

// ============================================================================
// State handlers
// ============================================================================

static QState notify_initial(NotifyAo * const me, QEvt const * const e) {
    (void)e;

    // Zero-init intent state, then default to kInit so boot shows the
    // rainbow warmup visual until ESKF + IMU are both ready (Stage L).
    // Gets overridden by any higher-priority intent (fault, cal); gets
    // cleared by notify_evaluate_sensor_status() once ESKF is up.
    me->state = {};
    me->state.phase = PhaseIntent::kInit;
    me->sensor_phase_start_ms = to_ms_since_boot(get_absolute_time());
    me->prearm_fail_ticks = 0U;  // Stage L
    me->init_min_ticks = kInitMinTicks;  // Stage L IVP-L4

    // Subscribe to state-producing signals
    QActive_subscribe(&me->super, rc::SIG_PHASE_CHANGE);
    QActive_subscribe(&me->super, rc::SIG_RADIO_STATUS);
    QActive_subscribe(&me->super, rc::SIG_HEALTH_STATUS);
    QActive_subscribe(&me->super, rc::SIG_BEACON_ACTIVE);
    QActive_subscribe(&me->super, rc::SIG_BEACON_MANUAL);  // Stage L
    // IVP-116: SIG_LED_OVERRIDE subscription removed. RCOS now posts
    // CalIntentEvt directly via AO_Notify_post_cal_intent().

    // 33Hz animation tick (every 3 ticks at 100Hz base)
    QTimeEvt_armX(&me->tick_timer, 3U, 3U);

    return Q_TRAN(&notify_running);
}

// Apply a phase-change event to NotifyState. Extracted for JSF AV rule 1
// compliance. Stage L: also clears both beacon flags on exit from recovery-
// relevant phases — beacon stays active across LANDED and ABORT (the two
// phases where a recovery beacon is meaningful), any other transition clears.
// Stage L also clears the pre-arm-fail visual (any phase change invalidates
// the "ARM rejected" message).
static void handle_phase_change(NotifyAo * const me, QEvt const * const e) {
    const auto* pce = rc::evt_cast<rc::PhaseChangeEvt>(e);
    me->state.phase = phase_from_flight_phase(pce->phase);
    if (me->state.phase != PhaseIntent::kLanded &&
        me->state.phase != PhaseIntent::kAbort) {
        me->state.beacon_auto = false;
        me->state.beacon_manual = false;
    }
    // Stage L: phase change always invalidates an in-progress pre-arm-fail
    // flash. The FD override already replaced state.phase; clearing the
    // counter prevents a future tick from re-stamping kPreArmFail.
    me->prearm_fail_ticks = 0;
}

// 33Hz tick: refresh sensor intent from seqlock, run the pre-arm-fail
// auto-clear helper, and dispatch resolved pattern to both output
// backends. Extracted from Notify_running for JSF AV rule 1.
static void handle_notify_tick(NotifyAo * const me) {
    shared_sensor_data_t snap{};
    bool snap_ok = seqlock_read(&g_sensorSeqlock, &snap);
    if (snap_ok) {
        notify_evaluate_sensor_status(me, &snap);
    }
    // Stage L IVP-L4: boot-init rainbow auto-clear. Two gates must both be
    // satisfied before kInit flips to kIdle:
    //   1. Minimum visibility window elapsed (init_min_ticks counted down
    //      to 0 — ~3 s at 33Hz). ESKF + IMU are typically already ready
    //      by the time AO_Notify starts ticking, so this guarantees the
    //      rainbow is always seen at boot regardless of sensor-ready time.
    //   2. ESKF is initialized AND at least one IMU read is published.
    //      This keeps the rainbow up if the sensors aren't actually ready
    //      yet (cold start, ICM in a fault state, etc.).
    // Any explicit FD phase transition via handle_phase_change would have
    // already overridden state.phase directly.
    if (me->init_min_ticks > 0U) {
        --me->init_min_ticks;
    }
    if (me->state.phase == PhaseIntent::kInit &&
        me->init_min_ticks == 0U &&
        snap_ok && eskf_runner_is_initialized() &&
        snap.imu_read_count > 0U) {
        me->state.phase = PhaseIntent::kIdle;
    }
    // Stage L: tick the pre-arm-fail counter. When it hits 0, flip the
    // PhaseIntent back to kIdle (pre-arm fail can only be triggered from
    // IDLE — if we were elsewhere, handle_phase_change already updated
    // state.phase and zeroed the counter).
    me->prearm_fail_ticks =
        rc::prearm_fail_tick_next(me->prearm_fail_ticks, /*state_changed=*/false);
    if (me->prearm_fail_ticks == 0U &&
        me->state.phase == PhaseIntent::kPreArmFail) {
        me->state.phase = PhaseIntent::kIdle;
    }
    rc::notify::notify_backend_led_update(me->state);
    rc::notify::notify_backend_audio_update(me->state);
}

// Handle simple single-flag setters to keep Notify_running under the
// JSF AV Rule 1 60-line cap. All these branches just flip one bool or
// set one enum field and return — the dispatcher in Notify_running
// calls this and treats a non-zero return as "handled".
static bool handle_simple_flag_sig(NotifyAo * const me, QEvt const * const e) {
    switch (e->sig) {
    case rc::SIG_BEACON_ACTIVE:
        me->state.beacon_auto = true;
        return true;
    case rc::SIG_BEACON_MANUAL:
        me->state.beacon_manual = true;
        return true;
    case SIG_NOTIFY_VEHICLE_LOST:
        me->state.vehicle_lost = true;
        return true;
    case SIG_NOTIFY_VEHICLE_FOUND:
        me->state.vehicle_lost = false;
        return true;
    case SIG_NOTIFY_PREARM_FAIL:
        // Stage L — ARM command rejected. Each repost resets the
        // auto-clear counter to full (JPL council 2026-04-18).
        me->state.phase = PhaseIntent::kPreArmFail;
        me->prearm_fail_ticks = rc::kPreArmFailTicks;
        return true;
    default:
        return false;
    }
}

static QState notify_running(NotifyAo * const me, QEvt const * const e) {
    if (handle_simple_flag_sig(me, e)) {
        return Q_HANDLED();
    }
    switch (e->sig) {
    case rc::SIG_PHASE_CHANGE:
        handle_phase_change(me, e);
        return Q_HANDLED();

    case rc::SIG_RADIO_STATUS: {
        const auto* re = rc::evt_cast<rc::RadioStatusEvt>(e);
        me->state.radio = radio_intent_from_lq(re->link_quality);
        return Q_HANDLED();
    }

    case rc::SIG_HEALTH_STATUS: {
        const auto* he = rc::evt_cast<rc::HealthStatusEvt>(e);
        me->state.fault = decode_health_faults(he->primary, he->secondary);
        return Q_HANDLED();
    }

    case SIG_NOTIFY_CAL_INTENT: {
        // Direct post from AO_RCOS via AO_Notify_post_cal_intent()
        const auto* ce = rc::evt_cast<CalIntentEvt>(e);
        me->state.cal = ce->intent;
        return Q_HANDLED();
    }

    case SIG_NOTIFY_TICK:
        handle_notify_tick(me);
        return Q_HANDLED();

    default:
        break;
    }
    return Q_SUPER(&QHsm_top);
}

// ============================================================================
// Public interface
// ============================================================================

QActive * const AO_Notify = &g_notifyAo.super;

void AO_Notify_start(uint8_t prio) {
    g_notifyStarted = true;
    QActive_ctor(&g_notifyAo.super,
                 Q_STATE_CAST(&notify_initial));

    QTimeEvt_ctorX(&g_notifyAo.tick_timer, &g_notifyAo.super,
                   SIG_NOTIFY_TICK, 0U);

    QActive_start(&g_notifyAo.super,
                  Q_PRIO(prio, 0U),
                  g_notifyQueue,
                  Q_DIM(g_notifyQueue),
                  nullptr, 0U,
                  nullptr);
}

static CalIntent g_lastCalIntent = CalIntent::kNone;

void AO_Notify_post_cal_intent(CalIntent intent) {
    // No-op on builds where AO_Notify isn't started (Station/Relay roles).
    if (!g_notifyStarted) {
        return;
    }
    // Deduplicate — avoid queue spam from repeated posts of the same intent
    if (intent == g_lastCalIntent) {
        return;
    }
    g_lastCalIntent = intent;

    // Static event — QV does NOT copy posted events; it stores the pointer.
    // A stack-local event becomes a use-after-free once this function
    // returns (QV dispatches AO_Notify later in its while loop, after the
    // caller's stack has been reclaimed). Static is safe: this is only
    // called from handler context on Core 0 under cooperative QV, and the
    // dedup above prevents overlapping posts within one tick.
    static CalIntentEvt g_evt;
    g_evt.super = QEVT_INITIALIZER(SIG_NOTIFY_CAL_INTENT);
    g_evt.intent = intent;
    QACTIVE_POST(&g_notifyAo.super, &g_evt.super, nullptr);
}

// ============================================================================
// Stage L — pre-arm fail post (IVP-L3)
// Called by AO_RCOS (rc_os.cpp:dispatch_flight_command) after
// command_handler rejects a kArm command. Each call resets the counter
// to full — rapid-fire rejections refresh the window (JPL council).
// No dedup: intentional, so repeat rejections re-arm the visual.
// ============================================================================
void AO_Notify_post_prearm_fail() {
    if (!g_notifyStarted) {
        return;
    }
    // Static event per LL Entry 35. No QEvt subclass payload needed.
    static QEvt g_evt;
    g_evt = QEVT_INITIALIZER(SIG_NOTIFY_PREARM_FAIL);
    QACTIVE_POST(&g_notifyAo.super, &g_evt, nullptr);
}

// ============================================================================
// Stage T Batch B IVP-T14 Round 2 #10 — vehicle-lost / -found posts
// Called by AO_RfManager on the link-state transition edge (kTrack /
// kTrackDegraded → kAcq on forced-ACQ = lost; kTentative → kTrack on
// re-acquire = found). Station backends: LED shows unmissable red flash;
// audio backend plays the vehicle-lost tone once audio hardware is wired.
// ============================================================================
void AO_Notify_post_vehicle_lost() {
    if (!g_notifyStarted) { return; }
    static QEvt g_evt;
    g_evt = QEVT_INITIALIZER(SIG_NOTIFY_VEHICLE_LOST);
    QACTIVE_POST(&g_notifyAo.super, &g_evt, nullptr);
}

void AO_Notify_post_vehicle_found() {
    if (!g_notifyStarted) { return; }
    static QEvt g_evt;
    g_evt = QEVT_INITIALIZER(SIG_NOTIFY_VEHICLE_FOUND);
    QACTIVE_POST(&g_notifyAo.super, &g_evt, nullptr);
}
