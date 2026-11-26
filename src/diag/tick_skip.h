// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// QF tick skip. After a stall (flash erase, long handler), the 100 Hz QF
// tick fires one time, not one time for each missed period. It counts the
// missed periods and keeps the tick on its start phase grid.
// Pure code: no Pico SDK and no QP, so the host tests can use it.
// diag/qf_tick.cpp calls it from the SDK alarm callback.
#ifndef ROCKETCHIP_DIAG_TICK_SKIP_H
#define ROCKETCHIP_DIAG_TICK_SKIP_H

#include <stdint.h>

namespace rc {

struct TickSkipStep {
    uint32_t skipped;         // Grid periods that passed with no tick
    uint64_t advance_us;      // (skipped + 1) * period, added to the target
    uint64_t next_target_us;  // target + advance_us. Always after now.
};

struct TickSkipStats {
    uint32_t fired;          // Ticks that fired
    uint32_t skipped_total;  // Sum of skipped periods (saturates)
    uint32_t max_skip;       // Largest skip in one tick
    uint32_t bursts;         // Ticks that skipped one or more periods
};

// target_us: the grid time this tick was set for. now_us: the time it runs.
// period_us 0 is not valid; the function uses 1 in its place.
// A tick that runs before its target (not possible from the SDK) skips 0.
TickSkipStep tick_skip_step(uint64_t target_us, uint64_t now_us,
                            uint32_t period_us);

// Record one fired tick and its skip count.
void tick_skip_note(TickSkipStats& stats, uint32_t skipped);

} // namespace rc

#endif // ROCKETCHIP_DIAG_TICK_SKIP_H
