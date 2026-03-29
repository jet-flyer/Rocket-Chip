// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#include "innovation_monitor.h"

#include <cmath>

namespace rc {

void innovation_channel_init(InnovationChannel* ch) {
    for (uint8_t i = 0; i < InnovationChannel::kWindowSize; ++i) {
        ch->window[i] = 0.0f;
    }
    ch->alpha = 0.0f;
    ch->sum = 0.0f;
    ch->head = 0;
    ch->count = 0;
}

void innovation_channel_push(InnovationChannel* ch, float nis) {
    // Reject non-finite values
    if (!std::isfinite(nis) || nis < 0.0f) {
        return;
    }

    // Subtract oldest value from running sum if window is full
    if (ch->count == InnovationChannel::kWindowSize) {
        ch->sum -= ch->window[ch->head];
    }

    // Write new value
    ch->window[ch->head] = nis;
    ch->sum += nis;

    // Advance circular index
    ch->head = static_cast<uint8_t>(
        (ch->head + 1) % InnovationChannel::kWindowSize);

    if (ch->count < InnovationChannel::kWindowSize) {
        ++ch->count;
    }

    // Update alpha (running mean)
    ch->alpha = (ch->count > 0) ? (ch->sum / static_cast<float>(ch->count))
                                : 0.0f;
}

float innovation_channel_q_scale(const InnovationChannel* ch) {
    // One-directional: never deflate Q below baseline (Council A7).
    if (ch->alpha <= 1.0f) {
        return 1.0f;
    }
    // Cap inflation to prevent unbounded Q growth.
    if (ch->alpha > InnovationChannel::kMaxQInflation) {
        return InnovationChannel::kMaxQInflation;
    }
    return ch->alpha;
}

}  // namespace rc
