// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Fire after N consecutive true ticks; a false condition zeros the count.
// Unmanaged: first-wins signal. Managed: GuardState.sustained only.
// Also in docs/IVP.md IVP-71 / IVP-120 and docs/plans/STAGE8_FLIGHT_DIRECTOR.md.

#ifndef ROCKETCHIP_GUARD_EVALUATOR_H
#define ROCKETCHIP_GUARD_EVALUATOR_H

#include <cstdint>
#include "flight_state.h"
#include "rocketchip/fused_state.h"
#include "mission_profile.h"

namespace rc {

// Guard identifiers
enum class GuardId : uint8_t {
    kLaunchAccel     = 0,
    kBurnoutAccel    = 1,
    kApogeeVelocity  = 2,
    kBaroPeak        = 3,
    kMainDeploy      = 4,
    kStationary      = 5,
    kBaroStationary  = 6,  // raw baro alt rate near zero (ESKF-independent)
    kCount           = 7,
};

// constexpr contract with the combinator. Managed: combinator fires.
// Unmanaged: evaluator auto-dispatches on first sustain.
inline constexpr bool kGuardManaged[static_cast<uint8_t>(GuardId::kCount)] = {
    false,  // kLaunchAccel    — unmanaged (single guard, no combinator)
    false,  // kBurnoutAccel   — unmanaged (single guard)
    true,   // kApogeeVelocity — managed (AND combinator with baro peak)
    true,   // kBaroPeak       — managed (AND combinator with velocity)
    true,   // kMainDeploy     — managed (lockout-gated)
    false,  // kStationary     — unmanaged (single guard, long sustain)
    false,  // kBaroStationary — unmanaged (ESKF-independent landing detect)
};

static_assert(!kGuardManaged[static_cast<uint8_t>(GuardId::kLaunchAccel)]);
static_assert(!kGuardManaged[static_cast<uint8_t>(GuardId::kBurnoutAccel)]);
static_assert(kGuardManaged[static_cast<uint8_t>(GuardId::kApogeeVelocity)]);
static_assert(kGuardManaged[static_cast<uint8_t>(GuardId::kBaroPeak)]);
static_assert(kGuardManaged[static_cast<uint8_t>(GuardId::kMainDeploy)]);
static_assert(!kGuardManaged[static_cast<uint8_t>(GuardId::kStationary)]);
static_assert(!kGuardManaged[static_cast<uint8_t>(GuardId::kBaroStationary)]);

// Per-guard runtime state
struct GuardState {
    uint32_t sustain_count;     // Consecutive true ticks
    uint32_t sustain_required;  // Ticks required to fire (from profile ms / tick_ms)
    float threshold;            // From MissionProfile where used; kBaroPeak stores 0 unused
    uint16_t signal;            // Signal to emit when sustained
    uint8_t valid_phases;       // Bitmask: (1 << FlightPhase) for active phases
    bool fired;                 // Unmanaged auto-dispatch latch; managed never set
    bool sustained;             // Eligible, not fired, and count >= required
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

// Unmanaged first-wins returns a signal. Managed: flag only.
// accel_z / accel_mag are calibrated body-frame (m/s^2).
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
