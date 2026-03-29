// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#ifndef ROCKETCHIP_FUSION_INNOVATION_MONITOR_H
#define ROCKETCHIP_FUSION_INNOVATION_MONITOR_H

// Per-channel innovation ratio (NIS) sliding-window tracker (IVP-83).
// Pure C++ — no Pico SDK dependencies.
//
// Tracks alpha = mean(nu^2 / S) over a sliding window. When alpha > 1.0,
// the filter's process noise (Q) is too low for the actual sensor noise.
// The q_scale output can be used to inflate Q diagonals adaptively.
//
// Design properties (Council A7):
//   - One-directional: inflates Q only, never deflates below baseline.
//     This is a deliberate safety property — the filter should never
//     become overconfident via innovation feedback.
//   - Capped at kMaxQInflation to prevent unbounded Q growth.
//   - Caller freezes adaptation during phase transition ramps.

#include <cstdint>

namespace rc {

struct InnovationChannel {
    static constexpr uint8_t kWindowSize = 20;    // VALIDATE: sliding window depth
    static constexpr float kMaxQInflation = 10.0f; // VALIDATE: max Q multiplier

    float window[kWindowSize];
    float alpha;        // running mean of NIS values in window
    float sum;          // running sum for O(1) mean update
    uint8_t head;       // next write index (circular)
    uint8_t count;      // number of valid samples (0..kWindowSize)
};

// Initialize channel to empty state.
void innovation_channel_init(InnovationChannel* ch);

// Push a new NIS sample into the sliding window.
// NIS = nu^2 / S where nu = innovation, S = innovation covariance.
void innovation_channel_push(InnovationChannel* ch, float nis);

// Return Q scaling factor based on current alpha.
// Returns 1.0 when alpha <= 1.0 (no adaptation needed).
// Returns min(alpha, kMaxQInflation) when alpha > 1.0.
float innovation_channel_q_scale(const InnovationChannel* ch);

}  // namespace rc

#endif  // ROCKETCHIP_FUSION_INNOVATION_MONITOR_H
