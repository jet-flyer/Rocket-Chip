// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Flight Director — QEP State Machine Implementation
//
// Nine state handler functions implementing the flight phase statechart.
// Each handler processes QEP signals (Q_ENTRY_SIG, Q_EXIT_SIG, user signals)
// and returns Q_HANDLED(), Q_TRAN(), or Q_SUPER().
//
// ABORT behavior follows Council Amendment #1:
//   ABORT-from-BOOST:   fire drogue pyro (safety)
//   ABORT-from-COAST:   fire drogue pyro
//   ABORT-from-DESCENT: no-op (chutes already deployed)
//   Pad abort timeout:  → IDLE (never launched)
//   In-flight abort:    beacon after timeout, stays in ABORT (no LANDED)
//   Re-ARM from ABORT requires explicit RESET first
//============================================================================

#include "flight_director.h"
#include "flight_actions.h"
#include "safety/health_monitor.h"   // IVP-107: health_eskf() for lockout
#include "safety/crash_record.h"     // flight_in_progress_set/clear (fault-recovery 2026-05-14)
#include "rocketchip/rc_log.h"
#include <cmath>

#ifndef ROCKETCHIP_HOST_TEST
    #include "pico/time.h"
#else
    static uint32_t s_fake_time_ms = 0;
    static inline uint32_t to_ms_since_boot_stub() { return s_fake_time_ms; }
#endif

Q_DEFINE_THIS_MODULE("flight_director")

namespace rc {

// ============================================================================
// Forward declarations of state handlers
// ============================================================================
static QState state_initial(FlightDirector * const me, QEvt const * const e);
static QState state_idle(FlightDirector * const me, QEvt const * const e);
static QState state_armed(FlightDirector * const me, QEvt const * const e);
static QState state_boost(FlightDirector * const me, QEvt const * const e);
static QState state_coast(FlightDirector * const me, QEvt const * const e);
static QState state_descent(FlightDirector * const me, QEvt const * const e);
static QState state_drogue_descent(FlightDirector * const me, QEvt const * const e);
static QState state_main_descent(FlightDirector * const me, QEvt const * const e);
static QState state_landed(FlightDirector * const me, QEvt const * const e);
static QState state_abort(FlightDirector * const me, QEvt const * const e);

// ============================================================================
// Helper: record phase transition
// ============================================================================
static void enter_phase(FlightDirector* me, FlightPhase phase) {
    me->state.previous_phase = me->state.current_phase;
    me->state.current_phase = phase;
    me->state.phase_entry_ms = me->tick_ms;
    ++me->state.transition_count;

    // Flight-in-progress sentinel + fault-handler-observable phase pair.
    // See docs/decisions/FAULT_RECOVERY_2026-05-14.md (kAbort invariant + B.3).
    if (phase == FlightPhase::kArmed) {
        flight_in_progress_set();
    } else if (phase == FlightPhase::kLanded) {
        flight_in_progress_clear();
    }
    flight_phase_observable_set(phase);

    if (me->phase_change_cb) {
        me->phase_change_cb(phase, me->tick_ms);
    }
}

static void log_transition(const FlightDirector* me,
                            FlightPhase from, FlightPhase to) {
    rc::rc_log("[FD] %s -> %s\n",
               flight_phase_name(from),
               flight_phase_name(to));
}

// ============================================================================
// Helper: build ActionContext from FlightDirector state
// ============================================================================
static ActionContext make_action_ctx(FlightDirector* me,
                                      FlightPhase from, FlightPhase to) {
    ActionContext ctx{};
    ctx.markers = &me->state.markers;
    ctx.now_ms = me->tick_ms;
    ctx.from_phase = from;
    ctx.to_phase = to;
    ctx.set_led = me->set_led_cb;
    ctx.log_pyro = me->log_pyro_cb;
    return ctx;
}

// Helper: execute entry actions for a phase
static void run_entry_actions(FlightDirector* me, FlightPhase phase,
                               FlightPhase from) {
    uint8_t idx = static_cast<uint8_t>(phase);
    if (idx < static_cast<uint8_t>(FlightPhase::kCount)) {
        ActionContext ctx = make_action_ctx(me, from, phase);
        action_execute_list(kPhaseEntryActions[idx].entries,
                            kPhaseEntryActions[idx].count, &ctx);
    }
}

// Helper: execute transition actions (e.g. pyro fire)
static void run_transition_actions(FlightDirector* me,
                                    const ActionEntry* actions,
                                    uint32_t count,
                                    FlightPhase from, FlightPhase to) {
    ActionContext ctx = make_action_ctx(me, from, to);
    action_execute_list(actions, count, &ctx);
}

// ============================================================================
// Phase name lookup
// ============================================================================
static const char* const kPhaseNames[] = {
    "IDLE",
    "ARMED",
    "BOOST",
    "COAST",
    "DROGUE_DESCENT",
    "MAIN_DESCENT",
    "LANDED",
    "ABORT",
    "FAULT"
};

const char* flight_phase_name(FlightPhase phase) {
    uint8_t idx = static_cast<uint8_t>(phase);
    if (idx < static_cast<uint8_t>(FlightPhase::kCount)) {
        return kPhaseNames[idx];
    }
    return "UNKNOWN";
}

// ============================================================================
// Fault-handler-observable phase accessor (B.3) — checksummed pair
// ============================================================================
//
// Packing: low byte = phase, high byte (of low 16 bits) = ~phase.
// Initial value 0xFFFFFFFF would decode to phase=0xFF, complement=0xFF,
// validation fails → fault handler reads kFault, which is correct for the
// "uninitialized" case (we don't know what phase we're in yet).
static volatile uint32_t g_phaseObservablePair = 0xFFFFFFFFU;

void flight_phase_observable_set(FlightPhase phase) {
    const uint8_t p = static_cast<uint8_t>(phase);
    const uint8_t cp = static_cast<uint8_t>(~p);
    g_phaseObservablePair = (static_cast<uint32_t>(cp) << 8) | p;
}

FlightPhase flight_phase_observable_get() {
    const uint32_t raw = g_phaseObservablePair;
    const uint8_t p = static_cast<uint8_t>(raw & 0xFFU);
    const uint8_t cp = static_cast<uint8_t>((raw >> 8) & 0xFFU);
    if (static_cast<uint8_t>(~p) != cp) {
        // Corruption detected (or never-initialized). Safe-by-default.
        return FlightPhase::kFault;
    }
    if (p >= static_cast<uint8_t>(FlightPhase::kCount)) {
        return FlightPhase::kFault;
    }
    return static_cast<FlightPhase>(p);
}

// ============================================================================
// Signal name lookup
// ============================================================================
static const char* const kSignalNames[] = {
    "TICK",
    "ARM",
    "DISARM",
    "LAUNCH",
    "BURNOUT",
    "APOGEE",
    "MAIN_DEPLOY",
    "LANDING",
    "ABORT",
    "RESET"
};

const char* flight_signal_name(uint16_t sig) {
    if (sig >= SIG_TICK && sig < SIG_MAX) {
        return kSignalNames[sig - SIG_TICK];
    }
    return "?";
}

// ============================================================================
// Lifecycle
// ============================================================================

void flight_director_ctor(FlightDirector* me, const MissionProfile* profile) {
    QHsm_ctor(&me->super, Q_STATE_CAST(&state_initial));
    me->state.init();
    me->profile = profile;
    me->tick_ms = 0;
    me->guards_enabled = false;
    me->set_led_cb = nullptr;
    me->log_pyro_cb = nullptr;
    me->phase_change_cb = nullptr;
    me->beacon_cb = nullptr;
    me->reset_subsystems_cb = nullptr;
    // Init guard evaluator: 10ms tick period (100Hz)
    guard_evaluator_init(&me->guard_eval, *profile, 10);
    // Init combinator set from profile
    combinator_set_init(&me->combinator_set, *profile);
    // B.3: seed the fault-observable phase pair with kIdle so a fault that
    // fires before the first real enter_phase() call doesn't read uninitialized
    // bytes and trigger spurious kFault dispatch.
    flight_phase_observable_set(FlightPhase::kIdle);
}

void flight_director_init(FlightDirector* me) {
    QASM_INIT(&me->super, nullptr, 0U);
}

// ============================================================================
// Dispatch helpers
// ============================================================================

void flight_director_dispatch_tick(FlightDirector* me, uint32_t now_ms) {
    me->tick_ms = now_ms;
    QEvt const evt = QEVT_INITIALIZER(SIG_TICK);
    QASM_DISPATCH(&me->super, &evt, 0U);
}

void flight_director_dispatch_signal(FlightDirector* me, uint16_t sig) {
    QEvt const evt = QEVT_INITIALIZER(sig);
    QASM_DISPATCH(&me->super, &evt, 0U);
}

FlightPhase flight_director_phase(const FlightDirector* me) {
    return me->state.current_phase;
}

// Build the SafetyLockout snapshot used by the combinator evaluator.
// Extracted from flight_director_evaluate_guards for JSF AV rule 1 compliance.
static SafetyLockout build_safety_lockout(const FlightDirector* me,
                                           const FusedState& fused) {
    float vel_mag = sqrtf(fused.vel_n * fused.vel_n +
                          fused.vel_e * fused.vel_e +
                          fused.vel_d * fused.vel_d);
    uint32_t ms_since_launch = 0;
    if (me->state.markers.launch_ms > 0) {
        ms_since_launch = me->tick_ms - me->state.markers.launch_ms;
    }

    SafetyLockout lockout{};
    lockout.current_velocity_mps = vel_mag;
    lockout.ms_since_launch = ms_since_launch;
    lockout.deploy_lockout_mps = me->profile->deploy_lockout_mps;
    lockout.apogee_lockout_ms = me->profile->apogee_lockout_ms;
    lockout.eskf_healthy = (rc::health_eskf(fused.health_primary) >= rc::kHealthDegraded);
    lockout.confident = fused.confident;
    return lockout;
}

// IVP-121 MAIN_DESCENT multi-channel landing detection.
// Returns true if a LANDING dispatch was made (caller should bail).
// Two paths beyond the primary baro-stationary guard (IVP-120):
//   Path 1: ESKF-fault-confirmed-by-baro (conjunction, not disjunction)
//   Path 2: Last-resort backstop (time-based, fires distress beacon)
// Extracted from flight_director_evaluate_guards for JSF AV rule 1 compliance.
static bool handle_main_descent_landing(FlightDirector* me,
                                         const FusedState& fused) {
    uint32_t elapsed = me->tick_ms - me->state.phase_entry_ms;

    // Path 1: ESKF dead AND raw baro agrees we're stationary → LANDED.
    // ESKF fault alone is a health alert, not a landing. The conjunction
    // ensures we don't auto-LAND mid-descent just because the ESKF crashed.
    if (rc::health_eskf(fused.health_primary) == rc::kHealthFault &&
        guard_evaluator_is_sustained(&me->guard_eval, GuardId::kBaroStationary)) {
        rc::rc_log("[FD] MAIN_DESCENT: ESKF fault + baro stationary -> LANDED\n");
        flight_director_dispatch_signal(me, SIG_LANDING);
        return true;
    }

    // Path 2: Last-resort backstop. Fires only when ALL physical channels
    // have been silent for descent_max_duration_ms. System-level watchdog,
    // not a landing detector. Distress beacon for recovery.
    if (me->profile->descent_max_duration_ms > 0 &&
        elapsed >= me->profile->descent_max_duration_ms) {
        rc::rc_log("[FD] MAIN_DESCENT: max duration elapsed -> LANDED (distress)\n");
        if (me->beacon_cb) {
            me->beacon_cb();
        }
        flight_director_dispatch_signal(me, SIG_LANDING);
        return true;
    }

    return false;
}

void flight_director_evaluate_guards(FlightDirector* me,
                                      const FusedState& fused,
                                      float accel_z,
                                      float accel_mag) {
    // Guards only active in flight phases (ARMED through DESCENT)
    FlightPhase phase = me->state.current_phase;
    if (phase == FlightPhase::kIdle || phase == FlightPhase::kLanded ||
        phase == FlightPhase::kAbort) {
        return;
    }

    // Step 1: Update all guard sustain counters.
    // Unmanaged guards auto-dispatch; managed guards set sustained flag.
    uint16_t unmanaged_sig = guard_evaluator_tick(
        &me->guard_eval, phase, fused, accel_z, accel_mag);
    if (unmanaged_sig != SIG_MAX) {
        flight_director_dispatch_signal(me, unmanaged_sig);
        return;
    }

    // Step 2: Evaluate combinators (managed guards + lockouts + timer backup)
    SafetyLockout lockout = build_safety_lockout(me, fused);
    uint16_t combo_sig = combinator_set_evaluate(
        &me->combinator_set, phase, &me->guard_eval, lockout, 10);
    if (combo_sig != SIG_MAX) {
        flight_director_dispatch_signal(me, combo_sig);
        return;
    }

    // Step 3: MAIN_DESCENT multi-channel landing detection (IVP-121)
    if (phase == FlightPhase::kMainDescent) {
        (void)handle_main_descent_landing(me, fused);
    }
}

// ============================================================================
// Initial pseudostate → IDLE
// ============================================================================
static QState state_initial(FlightDirector * const me, QEvt const * const e) {
    Q_UNUSED_PAR(e);
    return Q_TRAN(&state_idle);
}

// ============================================================================
// IDLE — Default state. Waiting for ARM command.
//
// Accepts: SIG_ARM → ARMED
//          SIG_RESET → (already IDLE, no-op)
// ============================================================================
static QState state_idle(FlightDirector * const me, QEvt const * const e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            FlightPhase prev = me->state.current_phase;
            enter_phase(me, FlightPhase::kIdle);
            run_entry_actions(me, FlightPhase::kIdle, prev);
            if (me->state.transition_count > 1) {
                log_transition(me, prev, FlightPhase::kIdle);
                // Council 2026-05-20: any non-startup entry to IDLE must
                // force ESKF/Mahony re-init so re-ARM can succeed. Without
                // this, the ESKF stays UNHEALTHY across RESET (and pad-
                // abort auto-IDLE timeout), silently blocking re-ARM.
                if (me->reset_subsystems_cb) {
                    me->reset_subsystems_cb();
                }
            }
            return Q_HANDLED();
        }
        case SIG_ARM:
            return Q_TRAN(&state_armed);
        case SIG_TICK:
            return Q_HANDLED();
        default:
            break;
    }
    return Q_SUPER(&QHsm_top);
}

// ============================================================================
// ARMED — Pre-flight. Waiting for launch detection.
//
// Accepts: SIG_LAUNCH → BOOST
//          SIG_DISARM → IDLE
//          SIG_ABORT → ABORT
//          SIG_TICK — armed timeout check (Amendment #5)
// ============================================================================
static QState state_armed(FlightDirector * const me, QEvt const * const e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            FlightPhase prev = me->state.current_phase;
            enter_phase(me, FlightPhase::kArmed);
            run_entry_actions(me, FlightPhase::kArmed, prev);
            log_transition(me, prev, FlightPhase::kArmed);
            return Q_HANDLED();
        }
        case SIG_LAUNCH:
            return Q_TRAN(&state_boost);
        case SIG_DISARM:
            return Q_TRAN(&state_idle);
        case SIG_ABORT:
            return Q_TRAN(&state_abort);
        case SIG_TICK: {
            // Armed timeout — auto-disarm (Amendment #5)
            uint32_t elapsed = me->tick_ms - me->state.phase_entry_ms;
            if (elapsed >= me->profile->armed_timeout_ms) {
                rc::rc_log("[FD] ARMED timeout — auto-disarm\n");
                return Q_TRAN(&state_idle);
            }
            return Q_HANDLED();
        }
        default:
            break;
    }
    return Q_SUPER(&QHsm_top);
}

// ============================================================================
// BOOST — Motor burning. Waiting for burnout.
//
// Accepts: SIG_BURNOUT → COAST
//          SIG_ABORT → ABORT (fires drogue if profile flag set — Amendment #1)
// ============================================================================
static QState state_boost(FlightDirector * const me, QEvt const * const e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            FlightPhase prev = me->state.current_phase;
            enter_phase(me, FlightPhase::kBoost);
            run_entry_actions(me, FlightPhase::kBoost, prev);
            log_transition(me, prev, FlightPhase::kBoost);
            return Q_HANDLED();
        }
        case SIG_BURNOUT:
            return Q_TRAN(&state_coast);
        case SIG_ABORT:
            return Q_TRAN(&state_abort);
        case SIG_TICK:
            return Q_HANDLED();
        default:
            break;
    }
    return Q_SUPER(&QHsm_top);
}

// ============================================================================
// COAST — Unpowered ascent. Waiting for apogee.
//
// Accepts: SIG_APOGEE → DESCENT (drogue)
//          SIG_ABORT → ABORT (fires drogue if profile flag set — Amendment #1)
//          SIG_TICK — coast timeout fallback (Amendment #7)
// ============================================================================
static QState state_coast(FlightDirector * const me, QEvt const * const e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            FlightPhase prev = me->state.current_phase;
            enter_phase(me, FlightPhase::kCoast);
            run_entry_actions(me, FlightPhase::kCoast, prev);
            log_transition(me, prev, FlightPhase::kCoast);
            return Q_HANDLED();
        }
        case SIG_APOGEE:
            // Transition action: fire drogue pyro (if profile has pyro)
            if (me->profile->has_pyro) {
                run_transition_actions(me, kTransitionFireDrogue,
                    action_count(kTransitionFireDrogue),
                    FlightPhase::kCoast, FlightPhase::kDrogueDescent);
            }
            return Q_TRAN(&state_descent);
        case SIG_ABORT:
            return Q_TRAN(&state_abort);
        case SIG_TICK: {
            // Coast timeout — missed apogee fallback (Amendment #7)
            uint32_t elapsed = me->tick_ms - me->state.phase_entry_ms;
            if (elapsed >= me->profile->coast_timeout_ms) {
                rc::rc_log("[FD] Coast timeout — forced apogee\n");
                // Fire drogue pyro on timeout-forced apogee (same as SIG_APOGEE)
                if (me->profile->has_pyro) {
                    run_transition_actions(me, kTransitionFireDrogue,
                        action_count(kTransitionFireDrogue),
                        FlightPhase::kCoast, FlightPhase::kDrogueDescent);
                }
                return Q_TRAN(&state_descent);
            }
            return Q_HANDLED();
        }
        default:
            break;
    }
    return Q_SUPER(&QHsm_top);
}

// ============================================================================
// DESCENT — Superstate containing DROGUE_DESCENT and MAIN_DESCENT.
//
// Handles SIG_ABORT from any descent sub-state: no-op (chutes deployed).
// Initial sub-state: DROGUE_DESCENT.
// ============================================================================
static QState state_descent(FlightDirector * const me, QEvt const * const e) {
    switch (e->sig) {
        case Q_INIT_SIG:
            return Q_TRAN(&state_drogue_descent);
        case SIG_ABORT:
            // ABORT from DESCENT: no-op — chutes already deployed (Amendment #1)
            // Stay in current descent sub-state, don't transition to ABORT
            rc::rc_log("[FD] ABORT in DESCENT — ignored (chutes deployed)\n");
            return Q_HANDLED();
        default:
            break;
    }
    return Q_SUPER(&QHsm_top);
}

// ============================================================================
// DROGUE_DESCENT — Under drogue chute. Waiting for main deploy altitude.
//
// Accepts: SIG_MAIN_DEPLOY → MAIN_DESCENT
//          SIG_LANDING → LANDED (skip main if already low enough)
// ============================================================================
static QState state_drogue_descent(FlightDirector * const me, QEvt const * const e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            FlightPhase prev = me->state.current_phase;
            enter_phase(me, FlightPhase::kDrogueDescent);
            run_entry_actions(me, FlightPhase::kDrogueDescent, prev);
            log_transition(me, prev, FlightPhase::kDrogueDescent);
            return Q_HANDLED();
        }
        case SIG_MAIN_DEPLOY:
            // Transition action: fire main pyro (if profile has pyro)
            if (me->profile->has_pyro) {
                run_transition_actions(me, kTransitionFireMain,
                    action_count(kTransitionFireMain),
                    FlightPhase::kDrogueDescent, FlightPhase::kMainDescent);
            }
            return Q_TRAN(&state_main_descent);
        case SIG_LANDING:
            return Q_TRAN(&state_landed);
        case SIG_TICK:
            return Q_HANDLED();
        default:
            break;
    }
    return Q_SUPER(&state_descent);
}

// ============================================================================
// MAIN_DESCENT — Under main chute. Waiting for landing.
//
// Accepts: SIG_LANDING → LANDED
// ============================================================================
static QState state_main_descent(FlightDirector * const me, QEvt const * const e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            FlightPhase prev = me->state.current_phase;
            enter_phase(me, FlightPhase::kMainDescent);
            run_entry_actions(me, FlightPhase::kMainDescent, prev);
            log_transition(me, prev, FlightPhase::kMainDescent);
            return Q_HANDLED();
        }
        case SIG_LANDING:
            return Q_TRAN(&state_landed);
        case SIG_TICK:
            return Q_HANDLED();
        default:
            break;
    }
    return Q_SUPER(&state_descent);
}

// ============================================================================
// LANDED — Flight complete. Waiting for RESET.
//
// Accepts: SIG_RESET → IDLE
// ============================================================================
static QState state_landed(FlightDirector * const me, QEvt const * const e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            FlightPhase prev = me->state.current_phase;
            enter_phase(me, FlightPhase::kLanded);
            run_entry_actions(me, FlightPhase::kLanded, prev);
            log_transition(me, prev, FlightPhase::kLanded);
            return Q_HANDLED();
        }
        case SIG_RESET:
            return Q_TRAN(&state_idle);
        case SIG_TICK:
            return Q_HANDLED();
        default:
            break;
    }
    return Q_SUPER(&QHsm_top);
}

// ============================================================================
// ABORT — Emergency state. Terminal for this flight — no transition to LANDED.
//
// ABORT is a sink state: stays in ABORT until manual RESET → IDLE.
// Pad abort (from ARMED): timeout → IDLE (auto-safe, never launched).
// In-flight abort (from BOOST/COAST): activates beacon after timeout for
//   recovery tracking. Stays in ABORT — landing moment determined in
//   post-processing from flight log. RESET required to return to IDLE.
//
// Accepts: SIG_RESET → IDLE
//          SIG_TICK — pad abort timeout, in-flight beacon activation
// ============================================================================
static QState state_abort(FlightDirector * const me, QEvt const * const e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            FlightPhase prev = me->state.current_phase;
            enter_phase(me, FlightPhase::kAbort);
            run_entry_actions(me, FlightPhase::kAbort, prev);
            log_transition(me, prev, FlightPhase::kAbort);

            // ABORT transition actions depend on source phase (Amendment #1)
            if (prev == FlightPhase::kBoost &&
                me->profile->abort_fires_drogue_from_boost) {
                run_transition_actions(me, kTransitionFireDrogue,
                    action_count(kTransitionFireDrogue),
                    prev, FlightPhase::kAbort);
                rc::rc_log("[FD] ABORT from BOOST — drogue pyro intent\n");
            } else if (prev == FlightPhase::kCoast &&
                       me->profile->abort_fires_drogue_from_coast) {
                run_transition_actions(me, kTransitionFireDrogue,
                    action_count(kTransitionFireDrogue),
                    prev, FlightPhase::kAbort);
                rc::rc_log("[FD] ABORT from COAST — drogue pyro intent\n");
            }
            // ABORT from ARMED: no pyro action (still on pad)
            return Q_HANDLED();
        }
        case SIG_RESET:
            return Q_TRAN(&state_idle);
        case SIG_TICK: {
            uint32_t elapsed = me->tick_ms - me->state.phase_entry_ms;
            if (elapsed >= me->profile->abort_timeout_ms) {
                if (me->state.markers.launch_ms == 0) {
                    // Pad abort: never launched — auto-return to IDLE
                    rc::rc_log("[FD] ABORT timeout (pad) — auto-IDLE\n");
                    return Q_TRAN(&state_idle);
                }
                // In-flight abort: activate beacon for recovery, stay in ABORT.
                // Use marker as one-shot flag — landing_ms is otherwise unused
                // in ABORT (landing time determined in post-processing).
                if (me->state.markers.landing_ms == 0) {
                    me->state.markers.landing_ms = me->tick_ms;
                    if (me->set_led_cb) {
                        me->set_led_cb(kLedPhaseBeacon);
                    }
                    rc::rc_log("[FD] ABORT timeout (in-flight) — beacon active\n");
                }
            }
            return Q_HANDLED();
        }
        default:
            break;
    }
    return Q_SUPER(&QHsm_top);
}

// ============================================================================
// Launch abort — module-scope safety posture flag
//
// See docs/USER_GUIDE.md "Safety State Model" for the three-level
// classification (flight hold / safe mode / launch abort). Launch
// abort is level 3 — power-cycle-only clear by design.
// ============================================================================
static bool g_launchAbort = false;

void flight_director_set_launch_abort() {
    g_launchAbort = true;
}

bool flight_director_launch_abort() {
    return g_launchAbort;
}

#ifdef ROCKETCHIP_HOST_TEST
void flight_director_test_reset_launch_abort() {
    g_launchAbort = false;
}
#endif

// ============================================================================
// Host test support: set fake time for timeout testing
// ============================================================================
#ifdef ROCKETCHIP_HOST_TEST
void flight_director_test_set_time(FlightDirector* me, uint32_t ms) {
    me->tick_ms = ms;
}
#endif

} // namespace rc
