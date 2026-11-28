// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// QF tick skip: after a stall the tick fires one time, counts the missed
// periods, and stays on the start phase grid.
#include <gtest/gtest.h>

#include "diag/tick_skip.h"

#include <stdint.h>

using rc::TickSkipStats;
using rc::TickSkipStep;
using rc::tick_skip_note;
using rc::tick_skip_step;

namespace {
constexpr uint32_t kPeriodUs = 10000U;
} // namespace

TEST(TickSkip, OnTimeTickSkipsNone) {
    const TickSkipStep s = tick_skip_step(100000U, 100000U, kPeriodUs);
    EXPECT_EQ(s.skipped, 0U);
    EXPECT_EQ(s.advance_us, kPeriodUs);
    EXPECT_EQ(s.next_target_us, 110000U);
}

TEST(TickSkip, LateInsidePeriodSkipsNone) {
    const TickSkipStep s = tick_skip_step(100000U, 109999U, kPeriodUs);
    EXPECT_EQ(s.skipped, 0U);
    EXPECT_EQ(s.next_target_us, 110000U);
}

TEST(TickSkip, EarlyTickSkipsNone) {
    const TickSkipStep s = tick_skip_step(100000U, 99000U, kPeriodUs);
    EXPECT_EQ(s.skipped, 0U);
    EXPECT_EQ(s.next_target_us, 110000U);
}

// A 70 ms boot erase: 7 grid points pass, the tick fires one time.
TEST(TickSkip, SeventyMsStallFiresOneTick) {
    const TickSkipStep s = tick_skip_step(100000U, 170500U, kPeriodUs);
    EXPECT_EQ(s.skipped, 7U);
    EXPECT_EQ(s.advance_us, 8U * kPeriodUs);
    EXPECT_EQ(s.next_target_us, 180000U);
    EXPECT_GT(s.next_target_us, 170500U);
}

TEST(TickSkip, ExactBoundaryCountsThatPeriod) {
    const TickSkipStep s = tick_skip_step(100000U, 130000U, kPeriodUs);
    EXPECT_EQ(s.skipped, 3U);
    EXPECT_EQ(s.next_target_us, 140000U);
}

TEST(TickSkip, ZeroPeriodUsesOne) {
    const TickSkipStep s = tick_skip_step(10U, 15U, 0U);
    EXPECT_EQ(s.skipped, 5U);
    EXPECT_EQ(s.next_target_us, 16U);
}

// Simulate the alarm across a stall: the old SDK repeating timer fires one
// tick for each grid point; the skip tick fires one, and the grid phase does
// not change.
TEST(TickSkip, StallBurstGivesOneTickAndKeepsGrid) {
    TickSkipStats stats{};
    uint64_t target = 10000U;
    uint32_t fired_during_stall = 0U;
    // 20 on-time ticks.
    for (int i = 0; i < 20; ++i) {
        const TickSkipStep s = tick_skip_step(target, target + 50U, kPeriodUs);
        tick_skip_note(stats, s.skipped);
        target = s.next_target_us;
    }
    // Stall: IRQs blocked 72 ms, then the alarm runs.
    const uint64_t stall_end = target + 72000U;
    const TickSkipStep late = tick_skip_step(target, stall_end, kPeriodUs);
    tick_skip_note(stats, late.skipped);
    ++fired_during_stall;
    target = late.next_target_us;
    // The next target is in the future, so the SDK loop does not fire again.
    EXPECT_GT(target, stall_end);
    EXPECT_EQ(fired_during_stall, 1U);
    EXPECT_EQ(late.skipped, 7U);
    // Grid phase: same as the start (10000 + n * period).
    EXPECT_EQ((target - 10000U) % kPeriodUs, 0U);
    EXPECT_EQ(stats.fired, 21U);
    EXPECT_EQ(stats.skipped_total, 7U);
    EXPECT_EQ(stats.max_skip, 7U);
    EXPECT_EQ(stats.bursts, 1U);
    // Fired + skipped = grid points that passed.
    EXPECT_EQ(stats.fired + stats.skipped_total, 28U);
}

TEST(TickSkip, StatsKeepMaxAndSaturate) {
    TickSkipStats stats{};
    tick_skip_note(stats, 3U);
    tick_skip_note(stats, 9U);
    tick_skip_note(stats, 2U);
    EXPECT_EQ(stats.max_skip, 9U);
    EXPECT_EQ(stats.skipped_total, 14U);
    EXPECT_EQ(stats.bursts, 3U);
    stats.skipped_total = UINT32_MAX - 1U;
    tick_skip_note(stats, 5U);
    EXPECT_EQ(stats.skipped_total, UINT32_MAX);
}
