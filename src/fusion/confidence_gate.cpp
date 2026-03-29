// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#include "confidence_gate.h"

namespace rc {

void confidence_gate_init(ConfidenceState* cs) {
    cs->confident = true;
    cs->ahrs_divergence_deg = 0.0f;
    cs->time_since_confident_ms = 0;
    cs->phase_agreement = true;
    cs->bad_since_ms = 0;
    cs->good_since_ms = 0;
    cs->last_confident_ms = 0;
}

void confidence_gate_evaluate(ConfidenceState* cs, const ConfidenceInput& input) {
    cs->ahrs_divergence_deg = input.mahony_div_deg;

    // Evaluate raw health conditions (all must pass)
    const bool raw_ok =
        input.eskf_healthy &&
        input.mahony_div_deg < confidence::kAhrsDivMaxDeg &&
        input.max_innov_ratio < confidence::kInnovRatioMax &&
        input.p_att_max < confidence::kPAttMaxRad2 &&
        input.p_vel_max < confidence::kPVelMaxM2s2;

    // Hysteresis state machine
    if (raw_ok) {
        cs->bad_since_ms = 0;  // reset bad timer
        if (!cs->confident) {
            // Uncertain → trying to recover
            if (cs->good_since_ms == 0) {
                cs->good_since_ms = input.now_ms;
            }
            const uint32_t good_duration = input.now_ms - cs->good_since_ms;
            if (good_duration >= confidence::kRecoveryDebounceMs) {
                cs->confident = true;
                cs->good_since_ms = 0;
            }
        } else {
            cs->good_since_ms = 0;
        }
    } else {
        cs->good_since_ms = 0;  // reset good timer
        if (cs->confident) {
            // Confident → checking if loss is sustained
            if (cs->bad_since_ms == 0) {
                cs->bad_since_ms = input.now_ms;
            }
            const uint32_t bad_duration = input.now_ms - cs->bad_since_ms;
            if (bad_duration >= confidence::kLossDebounceMs) {
                cs->confident = false;
                cs->bad_since_ms = 0;
            }
        } else {
            cs->bad_since_ms = 0;
        }
    }

    // Update timing
    if (cs->confident) {
        cs->last_confident_ms = input.now_ms;
        cs->time_since_confident_ms = 0;
    } else {
        cs->time_since_confident_ms =
            (cs->last_confident_ms > 0)
                ? (input.now_ms - cs->last_confident_ms)
                : input.now_ms;
    }
}

}  // namespace rc
