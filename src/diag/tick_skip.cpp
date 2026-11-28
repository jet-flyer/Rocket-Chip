// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#include "diag/tick_skip.h"

namespace rc {

TickSkipStep tick_skip_step(uint64_t target_us, uint64_t now_us,
                            uint32_t period_us) {
    const uint64_t period = (period_us == 0U) ? 1U : period_us;
    uint64_t skipped = 0U;
    if (now_us > target_us) {
        skipped = (now_us - target_us) / period;
    }
    TickSkipStep step{};
    step.advance_us = (skipped + 1U) * period;
    step.next_target_us = target_us + step.advance_us;
    step.skipped = (skipped > UINT32_MAX) ? UINT32_MAX
                                          : static_cast<uint32_t>(skipped);
    return step;
}

void tick_skip_note(TickSkipStats& stats, uint32_t skipped) {
    ++stats.fired;
    if (skipped == 0U) {
        return;
    }
    ++stats.bursts;
    const uint32_t room = UINT32_MAX - stats.skipped_total;
    stats.skipped_total += (skipped > room) ? room : skipped;
    if (skipped > stats.max_skip) {
        stats.max_skip = skipped;
    }
}

} // namespace rc
