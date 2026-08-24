// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#ifndef ROCKETCHIP_FUSION_CONFIDENCE_GATE_H
#define ROCKETCHIP_FUSION_CONFIDENCE_GATE_H

// Confidence gate: compile-time thresholds (not MissionProfile).
// Consumed by FD SafetyLockout.confident — pyro locked when false.
// Loss / recovery: kLossDebounceMs / kRecoveryDebounceMs hysteresis.

#include <cstdint>

namespace rc {

// Input to the confidence gate evaluation. Caller populates from ESKF,
// Mahony AHRS, and innovation monitor state each tick.
struct ConfidenceInput {
    float mahony_div_deg;       // ESKF vs Mahony quaternion angular distance (degrees)
    bool eskf_healthy;          // ESKF::healthy() result
    float max_innov_ratio;      // max alpha across all innovation channels
    float p_att_max;            // max P diagonal [0..2] (attitude, rad^2)
    float p_vel_max;            // max P diagonal [6..8] (velocity, m^2/s^2)
    uint32_t now_ms;            // current time (ms since boot)
};

// Gate output + hysteresis on one public struct. FD SafetyLockout reads
// confident / ahrs_divergence_deg / time_since_confident_ms.
// bad/good/last_confident_ms are hysteresis — only the gate functions write.
struct ConfidenceState {
    bool confident;                     // safe to execute irreversible actions
    float ahrs_divergence_deg;          // ESKF vs Mahony angle at last eval
    uint32_t time_since_confident_ms;   // 0 when confident, counts up when not
    bool phase_agreement;               // unused (reserved)

    // Hysteresis (gate functions write these)
    uint32_t bad_since_ms;              // 0 = not in bad period
    uint32_t good_since_ms;             // 0 = not in good period
    uint32_t last_confident_ms;         // timestamp of last confident=true eval
};

// Platform safety thresholds — NOT user-tunable, NOT in MissionProfile.
// All values VALIDATE — tune with flight data.
namespace confidence {
    constexpr float kAhrsDivMaxDeg       = 15.0F;   // VALIDATE: max AHRS divergence
    constexpr float kInnovRatioMax       = 5.0F;    // VALIDATE: max innovation ratio
    constexpr float kPAttMaxRad2         = 0.5F;    // VALIDATE: max P attitude diagonal
    constexpr float kPVelMaxM2s2         = 50.0F;   // VALIDATE: max P velocity diagonal
    constexpr uint32_t kLossDebounceMs   = 500;     // VALIDATE: sustained bad before loss
    constexpr uint32_t kRecoveryDebounceMs = 2000;  // VALIDATE: sustained good before recovery
}  // namespace confidence

// Initialize confidence gate to confident state.
void confidence_gate_init(ConfidenceState* cs);

// Evaluate confidence conditions and update state with hysteresis.
// cs must be non-null. now_ms == 0 is the host-test clock.
void confidence_gate_evaluate(ConfidenceState* cs, const ConfidenceInput& input);

}  // namespace rc

#endif  // ROCKETCHIP_FUSION_CONFIDENCE_GATE_H
