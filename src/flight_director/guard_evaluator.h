// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file guard_evaluator.h
 * @brief Guard sustain evaluator (IVP-70, updated IVP-71)
 *
 * Manages sustain counters for each guard. A guard "fires" (returns a
 * signal) only when its condition is true for N consecutive ticks.
 * One false tick resets the counter to zero.
 *
 * IVP-71: Guards are either "unmanaged" (auto-dispatch on sustain) or
 * "managed" (combinator reads sustained[] and decides dispatch).
 * Managed/unmanaged is a compile-time property (Council A4).
 *
 * Phase validity: each guard is only active in specific phases.
 * The evaluator skips inactive guards and resets their counters on
 * phase transitions.
 */

#ifndef ROCKETCHIP_GUARD_EVALUATOR_H
#define ROCKETCHIP_GUARD_EVALUATOR_H

#include <cstdint>
#include "flight_state.h"
#include "rocketchip/fused_state.h"
#include "mission_profile.h"

namespace rc {

// Guard identifiers
enum class GuardId : uint8_t {
    kLaunchAccel    = 0,
    kBurnoutAccel   = 1,
    kApogeeVelocity = 2,
    kBaroPeak       = 3,
    kMainDeploy     = 4,
    kStationary     = 5,
    kCount          = 6,
};

// Compile-time managed flag (Council A4).
// Managed guards track sustain but don't auto-dispatch — the combinator
// layer reads their sustained status and decides when to fire.
// Unmanaged guards auto-dispatch on first sustain (IVP-70 behavior).
//
// DO NOT modify at runtime. This array is the contract between the
// evaluator and the combinator.
inline constexpr bool kGuardManaged[static_cast<uint8_t>(GuardId::kCount)] = {
    false,  // kLaunchAccel    — unmanaged (single guard, no combinator)
    false,  // kBurnoutAccel   — unmanaged (single guard)
    true,   // kApogeeVelocity — managed (AND combinator with baro peak)
    true,   // kBaroPeak       — managed (AND combinator with velocity)
    true,   // kMainDeploy     — managed (lockout-gated)
    false,  // kStationary     — unmanaged (single guard, long sustain)
};

// Per-guard runtime state
struct GuardState {
    uint32_t sustain_count;     // Consecutive true ticks
    uint32_t sustain_required;  // Ticks required to fire (from profile ms / tick_ms)
    float threshold;            // Guard-specific threshold from MissionProfile
    uint16_t signal;            // Signal to emit when sustained
    uint8_t valid_phases;       // Bitmask: (1 << FlightPhase) for active phases
    bool fired;                 // Edge detection: true after first fire, reset on phase change
    bool sustained;             // True when sustain_count >= sustain_required (IVP-71)
};

// Guard evaluator — holds state for all guards, evaluates per tick
struct GuardEvaluator {
    GuardState guards[static_cast<uint8_t>(GuardId::kCount)];
    FlightPhase last_phase;     // For detecting phase transitions
};

// Initialize the evaluator from a MissionProfile.
// tick_period_ms: how often evaluate() is called (typically 10ms for 100Hz)
void guard_evaluator_init(GuardEvaluator* ev,
                           const MissionProfile& profile,
                           uint32_t tick_period_ms);

// Evaluate all active guards for the current phase.
// - Unmanaged guards: returns signal on first sustain (first-wins), or SIG_MAX
// - Managed guards: updates sustained flag only, returns SIG_MAX
//
// Caller should check sustained[] for managed guards via the combinator.
//
// @param ev          Evaluator state
// @param phase       Current flight phase
// @param fused       ESKF fused state (velocity, altitude, baro)
// @param accel_z     Calibrated body-Z acceleration (m/s^2)
// @param accel_mag   Calibrated acceleration magnitude (m/s^2)
uint16_t guard_evaluator_tick(GuardEvaluator* ev,
                               FlightPhase phase,
                               const FusedState& fused,
                               float accel_z,
                               float accel_mag);

// Reset all sustain counters, fired flags, and sustained flags
void guard_evaluator_reset(GuardEvaluator* ev);

// Read sustained status for a specific guard (for combinator use)
bool guard_evaluator_is_sustained(const GuardEvaluator* ev, GuardId id);

} // namespace rc

#endif // ROCKETCHIP_GUARD_EVALUATOR_H
