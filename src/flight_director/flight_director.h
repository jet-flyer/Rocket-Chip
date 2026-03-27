// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Flight Director — QEP Hierarchical State Machine
//
// IVP-68: QEP Integration + Phase Skeleton (Stage 8: Flight Director)
//
// The Flight Director is a QHsm (hierarchical state machine) that tracks
// flight phase from IDLE through LANDED. It processes signals generated
// by guard functions (IVP-70), CLI commands (IVP-69), and internal
// timers.
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
//   SIG_APOGEE      — Guard: velocity zero-cross or coast timeout
//   SIG_MAIN_DEPLOY — Guard: AGL altitude below threshold
//   SIG_LANDING     — Guard: stationary for sustained period
//   SIG_ABORT       — CLI/command/guard: emergency abort
//   SIG_RESET       — CLI/command: return to IDLE from any state
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

#include "rocketchip/ao_signals.h"  // System-wide signal catalog (IVP-76)
#include "flight_state.h"
#include "mission_profile.h"
#include "guard_evaluator.h"
#include "guard_combinator.h"
#include "action_executor.h"

namespace rc {

// Flight Director signals are now defined in ao_signals.h as part of the
// system-wide RcSignal enum. FlightSignal is a backward-compatible alias.
// Signal values (SIG_TICK=4 through SIG_RESET=13) are unchanged.

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
    GuardEvaluator guard_eval;      // Guard sustain evaluator (IVP-70)
    CombinatorSet combinator_set;   // Guard combinators + lockouts (IVP-71)
    uint32_t tick_ms;               // Current tick timestamp (set each tick)
    bool guards_enabled;            // False in IDLE/LANDED, true in flight phases

    // Action callbacks (IVP-72) — set by main.cpp, stubbed in host tests
    void (*set_led_cb)(uint8_t led_value);      // NeoPixel override
    void (*log_pyro_cb)(PyroChannel channel);   // Pyro intent logging
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

} // namespace rc

#endif // ROCKETCHIP_FLIGHT_DIRECTOR_H
