// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Flight Director — QEP State Machine Implementation
//
// IVP-68: QEP Integration + Phase Skeleton (Stage 8: Flight Director)
// IVP-72: Action Executor — entry/exit/transition actions wired into handlers
//
// Nine state handler functions implementing the flight phase statechart.
// Each handler processes QEP signals (Q_ENTRY_SIG, Q_EXIT_SIG, user signals)
// and returns Q_HANDLED(), Q_TRAN(), or Q_SUPER().
//
// ABORT behavior follows Council Amendment #1:
//   ABORT-from-BOOST:   fire drogue pyro (safety)
//   ABORT-from-COAST:   fire drogue pyro
//   ABORT-from-DESCENT: no-op (chutes already deployed)
//   ABORT has 5-min timeout → auto-transition to LANDED
//   Re-ARM from ABORT requires explicit RESET first
//============================================================================

#include "flight_director.h"
#include "flight_actions.h"
#include <cmath>

#ifndef ROCKETCHIP_HOST_TEST
    #include "pico/time.h"
    #include <cstdio>
#else
    #include <cstdio>
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
}

static void log_transition(const FlightDirector* me,
                            FlightPhase from, FlightPhase to) {
    printf("[FD] %s -> %s\n",
           flight_phase_name(from),
           flight_phase_name(to));
}

// ============================================================================
// Helper: build ActionContext from FlightDirector state (IVP-72)
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
    "ABORT"
};

const char* flight_phase_name(FlightPhase phase) {
    uint8_t idx = static_cast<uint8_t>(phase);
    if (idx < static_cast<uint8_t>(FlightPhase::kCount)) {
        return kPhaseNames[idx];
    }
    return "UNKNOWN";
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
    // Init guard evaluator: 10ms tick period (100Hz)
    guard_evaluator_init(&me->guard_eval, *profile, 10);
    // Init combinator set from profile (IVP-71)
    combinator_set_init(&me->combinator_set, *profile);
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

    // Step 2: Build lockout snapshot for combinator
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
    lockout.eskf_healthy = fused.eskf_healthy;
    lockout.confident = fused.confident;    // IVP-85: confidence gate

    // Step 3: Evaluate combinators (managed guards + lockouts + timer backup)
    uint16_t combo_sig = combinator_set_evaluate(
        &me->combinator_set, phase, &me->guard_eval, lockout, 10);

    if (combo_sig != SIG_MAX) {
        flight_director_dispatch_signal(me, combo_sig);
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
                printf("[FD] ARMED timeout — auto-disarm\n");
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
//          SIG_ABORT → ABORT (fires drogue — Amendment #1)
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
//          SIG_ABORT → ABORT (fires drogue — Amendment #1)
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
                printf("[FD] Coast timeout — forced apogee\n");
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
            printf("[FD] ABORT in DESCENT — ignored (chutes deployed)\n");
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
// ABORT — Emergency state. Has 5-min timeout → LANDED.
// Requires RESET before re-ARM (no direct ABORT → ARMED).
//
// Accepts: SIG_RESET → IDLE
//          SIG_TICK — abort timeout check
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
                printf("[FD] ABORT from BOOST — drogue pyro intent\n");
            } else if (prev == FlightPhase::kCoast &&
                       me->profile->abort_fires_drogue_from_coast) {
                run_transition_actions(me, kTransitionFireDrogue,
                    action_count(kTransitionFireDrogue),
                    prev, FlightPhase::kAbort);
                printf("[FD] ABORT from COAST — drogue pyro intent\n");
            }
            // ABORT from ARMED: no pyro action (still on pad)
            return Q_HANDLED();
        }
        case SIG_RESET:
            return Q_TRAN(&state_idle);
        case SIG_TICK: {
            // Abort timeout → auto-transition to LANDED
            uint32_t elapsed = me->tick_ms - me->state.phase_entry_ms;
            if (elapsed >= me->profile->abort_timeout_ms) {
                printf("[FD] ABORT timeout — auto-LANDED\n");
                return Q_TRAN(&state_landed);
            }
            return Q_HANDLED();
        }
        default:
            break;
    }
    return Q_SUPER(&QHsm_top);
}

// ============================================================================
// Host test support: set fake time for timeout testing
// ============================================================================
#ifdef ROCKETCHIP_HOST_TEST
void flight_director_test_set_time(FlightDirector* me, uint32_t ms) {
    me->tick_ms = ms;
}
#endif

} // namespace rc
