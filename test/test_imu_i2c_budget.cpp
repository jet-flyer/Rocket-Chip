// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// IMU I2C transfer budget (Council 2026-11-21, plan 3.4): ~2 ms from the
// board pack, with margin over the longest IMU transfer at the shared bus
// speed. Host board.h resolves to the Feather pack.

#include <gtest/gtest.h>
#include <stdint.h>

#include "drivers/i2c_timing.h"
#include "drivers/i2c_master.h"
#include "drivers/icm20948.h"
#include "rocketchip/board.h"

TEST(I2cTiming, BytesTimeRoundsUp) {
    EXPECT_EQ(i2c_bytes_time_us(1U, 400000U), 23U);    // 22.5 us
    EXPECT_EQ(i2c_bytes_time_us(17U, 400000U), 383U);  // 382.5 us
    EXPECT_EQ(i2c_bytes_time_us(17U, 100000U), 1530U);
    EXPECT_EQ(i2c_bytes_time_us(0U, 400000U), 0U);
}

TEST(ImuI2cBudget, PackBudgetIsTwoMsWithMargin) {
    EXPECT_EQ(board::kImuI2cTimeoutUs, 2000U);
    EXPECT_LT(board::kImuI2cTimeoutUs, kI2cMasterDefaultTimeoutUs);
    const uint32_t burst_us =
        i2c_bytes_time_us(kIcm20948LongestXferBytes, kI2cMasterFreqHz);
    EXPECT_GE(board::kImuI2cTimeoutUs, 4U * burst_us);
}

TEST(ImuI2cBudget, LongestTransferIsTheBurst) {
    EXPECT_EQ(kIcm20948ImuBurstBytes, 14U);
    EXPECT_EQ(kIcm20948LongestXferBytes, 17U);
}

TEST(ImuI2cBudget, HundredKilohertzWouldNotFit) {
    // The static_assert in icm20948.cpp trips if the shared bus drops to
    // 100 kHz without raising the IMU budget.
    EXPECT_LT(board::kImuI2cTimeoutUs,
              4U * i2c_bytes_time_us(kIcm20948LongestXferBytes, 100000U));
}
