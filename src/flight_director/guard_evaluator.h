// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file guard_evaluator.h
 * @brief Guard sustain evaluator (IVP-70)
 *
 * Manages sustain counters for each guard. A guard "fires" (returns a
 * signal) only when its condition is true for N consecutive ticks.
 * One false tick resets the counter to zero.
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

// Per-guard runtime state
struct GuardState {
    uint32_t sustain_count;     // Consecutive true ticks
    uint32_t sustain_required;  // Ticks required to fire (from profile ms / tick_ms)
    float threshold;            // Guard-specific threshold from MissionProfile
    uint16_t signal;        // Signal to emit when sustained
    uint8_t valid_phases;       // Bitmask: (1 << FlightPhase) for active phases
    bool fired;                 // Edge detection: true after first fire, reset on phase change
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
// Returns a signal if any guard fires (first-wins), or SIG_MAX if none.
// Reads accel_z and accel_mag from the seqlock snapshot (not FusedState —
// those are raw calibrated values needed for launch/burnout detection).
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

// Reset all sustain counters and fired flags (call on phase transition)
void guard_evaluator_reset(GuardEvaluator* ev);

} // namespace rc

#endif // ROCKETCHIP_GUARD_EVALUATOR_H
