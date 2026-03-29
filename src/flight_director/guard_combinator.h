// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file guard_combinator.h
 * @brief Guard combinator + safety lockout gates (IVP-71)
 *
 * Three-layer safety architecture for deployment-critical transitions:
 *   Layer 1: Lockout gates (velocity + min-time) — block if unsafe
 *   Layer 2: Sensor combinators (AND/OR) — primary detection
 *   Layer 3: Timer backup — fires if sensors don't, gated by lockouts
 *
 * Industry standard: Altus Metrum velocity lockout + apogee lockout,
 * Featherweight dual-channel, NASA timer-primary for sounding rockets.
 *
 * Council review 2026-03-25: 6 amendments incorporated (A1-A6).
 */

#ifndef ROCKETCHIP_GUARD_COMBINATOR_H
#define ROCKETCHIP_GUARD_COMBINATOR_H

#include <cstdint>
#include "flight_state.h"
#include "guard_evaluator.h"

namespace rc {

// Combinator types
enum class CombinatorType : uint8_t {
    kAnd,               // All guards must be sustained
    kOr,                // Any guard sustained fires
    kPrimaryPlusTimeout // Any guard OR timeout (whichever first)
};

// Safety lockout snapshot — populated by caller each tick
struct SafetyLockout {
    float current_velocity_mps;     // ESKF velocity magnitude (m/s)
    uint32_t ms_since_launch;       // Time since launch marker (0 if not launched)
    float deploy_lockout_mps;       // From MissionProfile
    uint32_t apogee_lockout_ms;     // From MissionProfile
    bool eskf_healthy;              // ESKF health flag (Council A2)
    bool confident;                 // Confidence gate flag (IVP-85)
};

// Guard combinator configuration (one per managed transition)
struct GuardCombinator {
    CombinatorType type;
    GuardId guard_ids[4];           // Guards in this combinator
    uint8_t num_guards;
    uint16_t signal;                // Signal to emit when combinator fires
    uint32_t backup_timeout_ms;     // Layer 3 timer backup (0 = no backup)
    uint8_t valid_phases;           // Bitmask of phases where active

    // Runtime state
    uint32_t elapsed_ms;            // Time in current phase (for timer backup)
    bool fired;                     // Edge detection
};

// Maximum combinators (apogee AND, main deploy lockout-gated)
static constexpr uint8_t kMaxCombinators = 4;

// Combinator set — all combinators for a flight profile
struct CombinatorSet {
    GuardCombinator combinators[kMaxCombinators];
    uint8_t num_combinators;
    FlightPhase last_phase;     // For detecting phase transitions
};

// Initialize combinators from MissionProfile.
void combinator_set_init(CombinatorSet* cs, const MissionProfile& profile);

// Evaluate all active combinators for the current phase.
// Returns signal if any combinator fires, or SIG_MAX if none.
// tick_ms: tick period in ms (for elapsed time tracking).
//
// Layer 1 (lockouts) checked first — blocks layers 2+3 if active.
// Layer 2 (sensor combinators) checked next.
// Layer 3 (timer backup) checked last — only if layer 2 didn't fire.
// Council A2: ESKF-unhealthy bypasses velocity lockout for timer backup only.
uint16_t combinator_set_evaluate(CombinatorSet* cs,
                                  FlightPhase phase,
                                  const GuardEvaluator* ev,
                                  const SafetyLockout& lockout,
                                  uint32_t tick_ms);

// Reset all combinator state (call on phase transition)
void combinator_set_reset(CombinatorSet* cs);

} // namespace rc

#endif // ROCKETCHIP_GUARD_COMBINATOR_H
