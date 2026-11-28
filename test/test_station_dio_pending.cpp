// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Vehicle DIO bench pending latch. Header-only. No SPI.

#include <gtest/gtest.h>

#include "diag/station_dio_pending.h"

#if defined(ROCKETCHIP_DEV_MODE)

using rc::dio_bench::Kind;
using rc::dio_bench::Pending;
using rc::dio_bench::RequestAction;
using rc::dio_bench::TickOutcome;
using rc::dio_bench::arm;
using rc::dio_bench::blocks_tx;
using rc::dio_bench::clear;
using rc::dio_bench::kIdleTimeoutTicks;
using rc::dio_bench::kIdleTimeoutMs;
using rc::dio_bench::kRadioTickHz;
using rc::dio_bench::request_action;
using rc::dio_bench::step;

TEST(DioPending, TimeoutBudgetIsOneSecondInTicks) {
    EXPECT_EQ(kRadioTickHz, 100U);
    EXPECT_EQ(kIdleTimeoutMs, 1000U);
    EXPECT_EQ(kIdleTimeoutTicks, 100U);
}

TEST(DioPending, PendingIdleThenReady) {
    Pending p{};
    arm(p, Kind::kToggle);
    EXPECT_TRUE(blocks_tx(p));
    EXPECT_EQ(step(p, true), TickOutcome::kWaiting);
    EXPECT_TRUE(blocks_tx(p));
    EXPECT_EQ(p.kind, Kind::kToggle);
    EXPECT_EQ(step(p, false), TickOutcome::kReady);
    EXPECT_TRUE(blocks_tx(p));
    EXPECT_EQ(p.kind, Kind::kToggle);
    clear(p);
    EXPECT_FALSE(blocks_tx(p));
    EXPECT_EQ(step(p, false), TickOutcome::kIdle);
}

TEST(DioPending, ReadyOnFirstIdleTick) {
    Pending p{};
    arm(p, Kind::kDepth, 100);
    EXPECT_EQ(step(p, false), TickOutcome::kReady);
    EXPECT_EQ(p.kind, Kind::kDepth);
}

TEST(DioPending, TimeoutThenRefused) {
    Pending p{};
    arm(p, Kind::kDepth, 3);
    EXPECT_EQ(step(p, true), TickOutcome::kWaiting);
    EXPECT_EQ(step(p, true), TickOutcome::kWaiting);
    EXPECT_EQ(step(p, true), TickOutcome::kTimeout);
    EXPECT_EQ(p.ticks_left, 0U);
    EXPECT_TRUE(blocks_tx(p));
    clear(p);
    EXPECT_FALSE(blocks_tx(p));
    EXPECT_EQ(step(p, true), TickOutcome::kIdle);
}

TEST(DioPending, DefaultTimeoutExpiresOnHundredthBusyTick) {
    Pending p{};
    arm(p, Kind::kToggle);
    for (uint32_t i = 0; i < (kIdleTimeoutTicks - 1U); ++i) {
        EXPECT_EQ(step(p, true), TickOutcome::kWaiting) << "tick " << i;
    }
    EXPECT_EQ(step(p, true), TickOutcome::kTimeout);
}

TEST(DioPending, TelemetrySuppressedWhilePending) {
    Pending p{};
    EXPECT_FALSE(blocks_tx(p));
    arm(p, Kind::kDepth);
    EXPECT_TRUE(blocks_tx(p));
    EXPECT_TRUE(blocks_tx(p));  // still blocked through waiting
    EXPECT_EQ(step(p, true), TickOutcome::kWaiting);
    EXPECT_TRUE(blocks_tx(p));
    EXPECT_EQ(step(p, false), TickOutcome::kReady);
    EXPECT_TRUE(blocks_tx(p));  // stays blocked until the caller clears
    clear(p);
    EXPECT_FALSE(blocks_tx(p));
}

TEST(DioPending, VehicleArmsOnTxStationRefuses) {
    EXPECT_EQ(request_action(true, false), RequestAction::kRunNow);
    EXPECT_EQ(request_action(false, false), RequestAction::kRunNow);
    EXPECT_EQ(request_action(true, true), RequestAction::kArm);
    EXPECT_EQ(request_action(false, true), RequestAction::kRefuseTx);
}

#else  // field (no ROCKETCHIP_DEV_MODE)

TEST(DioPendingField, PendingApiOmitted) {
#if defined(RC_DIO_BENCH_PENDING_API)
    FAIL() << "DIO pending API must not exist in a field build";
#endif
}

#endif
