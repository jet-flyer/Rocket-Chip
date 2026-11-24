// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Core 1 IMU recover policy (Council 2026-11-21, plan section 4 gap 1):
// no action on finished NACKs with idle lines; bus recover only when the
// lines are stuck; IMU re-init on stuck lines or many failed reads, at
// most once per kReinitMinIntervalUs.

#include <gtest/gtest.h>
#include <stdint.h>

#include "core1/imu_recover_policy.h"
#include "rocketchip/board.h"

using imu_recover::Action;
using imu_recover::decide;
using imu_recover::kBusCheckFails;
using imu_recover::kReinitConsecFails;
using imu_recover::kReinitMinIntervalUs;

TEST(ImuRecoverPolicy, Thresholds) {
    EXPECT_EQ(kBusCheckFails, 10U);
    EXPECT_EQ(kReinitConsecFails, 50U);
    EXPECT_EQ(kReinitMinIntervalUs, 1000000U);
    // 50 reads x 2 transfers x 2 ms = 200 ms worst case before a re-init.
    EXPECT_EQ(imu_recover::kWorstHungBeforeReinitUs,
              kReinitConsecFails * 2U * board::kImuI2cTimeoutUs);
    EXPECT_LE(imu_recover::kWorstHungBeforeReinitUs,
              imu_recover::kMaxHungBeforeReinitUs);
}

TEST(ImuRecoverPolicy, NothingBetweenCheckPoints) {
    for (uint32_t n = 0U; n < 200U; n++) {
        if (imu_recover::is_check_point(n)) {
            continue;
        }
        EXPECT_EQ(decide(n, false, true), Action::kNone) << n;
        EXPECT_EQ(decide(n, true, true), Action::kNone) << n;
    }
    EXPECT_FALSE(imu_recover::is_check_point(0U));
    EXPECT_TRUE(imu_recover::is_check_point(10U));
}

TEST(ImuRecoverPolicy, FinishedNacksWithIdleLinesDoNothingBeforeThreshold) {
    for (uint32_t n = kBusCheckFails; n < kReinitConsecFails; n += kBusCheckFails) {
        EXPECT_EQ(decide(n, true, true), Action::kNone) << n;
    }
}

TEST(ImuRecoverPolicy, StuckLinesRecoverBusAndReinitWhenDue) {
    EXPECT_EQ(decide(kBusCheckFails, false, true), Action::kBusRecoverThenReinit);
    EXPECT_EQ(decide(kBusCheckFails, false, false), Action::kBusRecover);
    EXPECT_EQ(decide(kReinitConsecFails, false, false), Action::kBusRecover);
}

TEST(ImuRecoverPolicy, RepeatedFailsWithIdleLinesReinitWhenDue) {
    EXPECT_EQ(decide(kReinitConsecFails, true, true), Action::kReinit);
    EXPECT_EQ(decide(kReinitConsecFails, true, false), Action::kNone);
    // Not due at 50: the count keeps going and the next due check re-inits.
    EXPECT_EQ(decide(kReinitConsecFails + kBusCheckFails, true, true),
              Action::kReinit);
}

TEST(ImuRecoverPolicy, ReinitDueSpacing) {
    EXPECT_TRUE(imu_recover::reinit_due(5U, 0U, false));
    EXPECT_FALSE(imu_recover::reinit_due(1000U, 1000U, true));
    EXPECT_FALSE(imu_recover::reinit_due(1000U + kReinitMinIntervalUs - 1U, 1000U, true));
    EXPECT_TRUE(imu_recover::reinit_due(1000U + kReinitMinIntervalUs, 1000U, true));
    // time_us_32() wrap.
    const uint32_t last = 0xFFFFFF00U;
    EXPECT_FALSE(imu_recover::reinit_due(last + 0x200U, last, true));
    EXPECT_TRUE(imu_recover::reinit_due(last + kReinitMinIntervalUs, last, true));
}
