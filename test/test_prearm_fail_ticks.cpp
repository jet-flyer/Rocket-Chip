// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Stage L IVP-L3 — prearm_fail_tick_next() pure helper tests
//
// The helper drives the ~3-second auto-clear window for the
// PhaseIntent::kPreArmFail visual. It's a pure function over
// (remaining, state_changed) so the tests run host-side with no QP
// dependency.
//
// Semantics verified:
//   - state_changed always clears (returns 0) regardless of remaining
//   - 0 stays at 0 (no underflow)
//   - otherwise decrements by 1
//   - repost reset-to-full is done at the caller site (AO_Notify) — this
//     helper only handles the per-tick decrement
//============================================================================

#include <gtest/gtest.h>
#include "rocketchip/prearm_fail_ticks.h"

TEST(PreArmFailTick, DecrementsFromFullWindow) {
    // Starting at kPreArmFailTicks, each call with state_changed=false
    // should decrement by exactly one.
    uint32_t r = rc::kPreArmFailTicks;
    EXPECT_EQ(rc::prearm_fail_tick_next(r, false), rc::kPreArmFailTicks - 1U);
}

TEST(PreArmFailTick, DecrementsFromMidWindow) {
    EXPECT_EQ(rc::prearm_fail_tick_next(42U, false), 41U);
}

TEST(PreArmFailTick, DecrementsFromOne) {
    // The last tick — goes to 0 (cleared).
    EXPECT_EQ(rc::prearm_fail_tick_next(1U, false), 0U);
}

TEST(PreArmFailTick, ZeroStaysZero) {
    // Already cleared — stays cleared. No underflow.
    EXPECT_EQ(rc::prearm_fail_tick_next(0U, false), 0U);
}

TEST(PreArmFailTick, StateChangedClearsFromFull) {
    // Phase change mid-window clears immediately (ARM accepted, DISARM, etc.)
    EXPECT_EQ(rc::prearm_fail_tick_next(rc::kPreArmFailTicks, true), 0U);
}

TEST(PreArmFailTick, StateChangedClearsFromOne) {
    EXPECT_EQ(rc::prearm_fail_tick_next(1U, true), 0U);
}

TEST(PreArmFailTick, StateChangedLeavesZeroAtZero) {
    // No-op safety: already 0 + state_changed = still 0.
    EXPECT_EQ(rc::prearm_fail_tick_next(0U, true), 0U);
}

TEST(PreArmFailTick, FullDecrementSequenceHitsZero) {
    // Loop: starting at full, drive to zero with state_changed=false only.
    // At 33 Hz, kPreArmFailTicks = 99 → ~3s total.
    uint32_t r = rc::kPreArmFailTicks;
    uint32_t iterations = 0;
    while (r > 0U) {
        r = rc::prearm_fail_tick_next(r, false);
        ++iterations;
        ASSERT_LT(iterations, rc::kPreArmFailTicks + 10U)  // runaway guard
            << "helper never reached 0 within expected window";
    }
    EXPECT_EQ(iterations, rc::kPreArmFailTicks);
    EXPECT_EQ(r, 0U);
}

TEST(PreArmFailTick, ConstantSanityCheck) {
    // kPreArmFailTicks ~= 3s at 33 Hz (99 ticks). Allow ±5% slop in case
    // someone tunes the constant slightly.
    EXPECT_GE(rc::kPreArmFailTicks, 90U);
    EXPECT_LE(rc::kPreArmFailTicks, 110U);
}
