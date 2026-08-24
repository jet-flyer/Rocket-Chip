// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#include "confidence_gate.h"
#ifndef NDEBUG
#include <cstdlib>
#endif

// Confidence gate: hysteresis on ESKF vs Mahony disagreement.
// now_ms == 0 is the host-test clock (debounce still uses the same field).

namespace rc {

void confidence_gate_init(ConfidenceState* cs) {
#ifndef NDEBUG
    if (cs == nullptr) { std::abort(); }
#endif
    if (cs == nullptr) { return; }
    cs->confident = true;
    cs->ahrs_divergence_deg = 0.0F;
    cs->time_since_confident_ms = 0;
    cs->phase_agreement = true;
    cs->bad_since_ms = 0;
    cs->good_since_ms = 0;
    cs->last_confident_ms = 0;
}

static bool confidence_raw_ok(const ConfidenceInput& input) {
    return input.eskf_healthy
        && input.mahony_div_deg < confidence::kAhrsDivMaxDeg
        && input.max_innov_ratio < confidence::kInnovRatioMax
        && input.p_att_max < confidence::kPAttMaxRad2
        && input.p_vel_max < confidence::kPVelMaxM2s2;
}

static void confidence_apply_hysteresis(ConfidenceState* cs,
                                       const ConfidenceInput& input,
                                       bool raw_ok) {
    if (raw_ok) {
        cs->bad_since_ms = 0;
        if (!cs->confident) {
            if (cs->good_since_ms == 0) {
                cs->good_since_ms = input.now_ms;
            }
            if (input.now_ms - cs->good_since_ms >= confidence::kRecoveryDebounceMs) {
                cs->confident = true;
                cs->good_since_ms = 0;
            }
        } else {
            cs->good_since_ms = 0;
        }
        return;
    }
    cs->good_since_ms = 0;
    if (cs->confident) {
        if (cs->bad_since_ms == 0) {
            cs->bad_since_ms = input.now_ms;
        }
        if (input.now_ms - cs->bad_since_ms >= confidence::kLossDebounceMs) {
            cs->confident = false;
            cs->bad_since_ms = 0;
        }
    } else {
        cs->bad_since_ms = 0;
    }
}

void confidence_gate_evaluate(ConfidenceState* cs, const ConfidenceInput& input) {
#ifndef NDEBUG
    if (cs == nullptr) { std::abort(); }
#endif
    if (cs == nullptr) { return; }
    cs->ahrs_divergence_deg = input.mahony_div_deg;
    confidence_apply_hysteresis(cs, input, confidence_raw_ok(input));
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
