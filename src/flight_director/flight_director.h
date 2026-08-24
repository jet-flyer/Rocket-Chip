// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Flight Director — QEP Hierarchical State Machine
//
// The Flight Director is a QHsm (hierarchical state machine) that tracks
// flight phase from IDLE through LANDED. It processes signals generated
// by guard functions, CLI commands, and internal timers.
//
// HSM hierarchy:
//   [top]
//     ├── state_idle         (initial)
//     ├── state_armed
//     ├── state_boost
//     ├── state_coast
//     ├── state_descent      (superstate)
//     │     ├── state_drogue_descent  (initial sub-state)
//     │     └── state_main_descent
//     ├── state_landed
//     └── state_abort
//
// Signals (after Q_USER_SIG):
//   SIG_TICK        — 100Hz tick from main loop
//   SIG_ARM         — CLI/command: transition to ARMED
//   SIG_DISARM      — CLI/command: return to IDLE from ARMED
//   SIG_LAUNCH      — Guard: body-Z accel threshold sustained
//   SIG_BURNOUT     — Guard: accel magnitude below threshold
//   SIG_APOGEE      — Guard combinator (vel/baro). Coast timeout is a separate HSM path.
//   SIG_MAIN_DEPLOY — Guard: AGL altitude below threshold
//   SIG_LANDING     — Guard: stationary for sustained period
//   SIG_ABORT       — CLI/command/guard: emergency abort
//   SIG_RESET       — CLI/command: IDLE from LANDED or ABORT only
//
// The FlightDirector is a C struct (not C++ class) for QEP compatibility.
// It extends QHsm with a FlightState struct and a const MissionProfile*.
//============================================================================
#ifndef ROCKETCHIP_FLIGHT_DIRECTOR_H
#define ROCKETCHIP_FLIGHT_DIRECTOR_H

#include <stdint.h>

extern "C" {
#include "qp_port.h"
#include "qsafe.h"
}

#include "rocketchip/ao_signals.h"  // System-wide signal catalog
#include "flight_state.h"
#include "mission_profile.h"
#include "guard_evaluator.h"
#include "guard_combinator.h"
#include "action_executor.h"

namespace rc {

// Flight Director signals live in ao_signals.h (RcSignal). FlightSignal is
// a live alias (command_handler, ao_flight_director). Values unchanged.

// Signal name strings for logging
const char* flight_signal_name(uint16_t sig);

// ============================================================================
// Flight Director — QHsm subclass
//
// Usage:
//   static rc::FlightDirector g_director;
//   rc::flight_director_ctor(&g_director, &rc::kDefaultRocketProfile);
//   rc::flight_director_init(&g_director);
//   // In 100Hz tick:
//   rc::flight_director_dispatch_tick(&g_director, now_ms);
//   // For commands:
//   rc::flight_director_dispatch_signal(&g_director, rc::SIG_ARM);
// ============================================================================
struct FlightDirector {
    QHsm super;                     // QEP base class (must be first member)
    FlightState state;              // Runtime phase tracking
    const MissionProfile* profile;  // Active flight profile (boot-locked)
    GuardEvaluator guard_eval;      // Guard sustain evaluator
    CombinatorSet combinator_set;   // Guard combinators + lockouts
    uint32_t tick_ms;               // Current tick timestamp (set each tick)
    bool guards_enabled;            // Unused after ctor. Enablement is evaluate_guards' phase check.
};

// Lifecycle
void flight_director_ctor(FlightDirector* me, const MissionProfile* profile);
void flight_director_init(FlightDirector* me);

// Dispatch
void flight_director_dispatch_tick(FlightDirector* me, uint32_t now_ms);
void flight_director_dispatch_signal(FlightDirector* me, uint16_t sig);

// Guard evaluation — call after dispatch_tick with current sensor data.
// Runs guard evaluator and auto-dispatches any fired signal.
// accel_z: calibrated body-Z accel (m/s^2), accel_mag: |A| (m/s^2)
void flight_director_evaluate_guards(FlightDirector* me,
                                      const FusedState& fused,
                                      float accel_z,
                                      float accel_mag);

// Query
FlightPhase flight_director_phase(const FlightDirector* me);

// Launch abort — power-cycle-only clear. See docs/USER_GUIDE.md
// "Safety State Model".
// Module-scope flag: no FlightDirector pointer needed. BSS-zero on
// power cycle clears automatically; no CLI command to clear, by design.
void flight_director_set_launch_abort();
bool flight_director_launch_abort();

// P10-9: named side effects (was a function-pointer table). Target
// definitions in ao_flight_director.cpp; host recorders below.
void fd_effect_set_led(uint8_t led_value);
void fd_effect_log_pyro(PyroChannel channel);
void fd_effect_phase_change(FlightPhase phase, uint32_t timestamp_ms);
void fd_effect_beacon();
void fd_effect_reset_subsystems();  // Any non-startup IDLE entry, not only RESET.

#ifdef ROCKETCHIP_HOST_TEST
void fd_effect_host_reset();
uint8_t fd_effect_host_last_led();
int fd_effect_host_led_calls();
PyroChannel fd_effect_host_last_pyro();
int fd_effect_host_pyro_calls();
int fd_effect_host_reset_calls();
#endif

} // namespace rc

#endif // ROCKETCHIP_FLIGHT_DIRECTOR_H
