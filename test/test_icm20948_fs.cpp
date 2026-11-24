// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// ICM-20948 full-scale codes and scale factors (src/drivers/icm20948_fs.h)
// and the board pack's range (Council 2026-11-21: ±16 g / ±2000 dps).
// Host board.h resolves to the Feather pack.

#include <gtest/gtest.h>

#include "drivers/icm20948_fs.h"
#include "rocketchip/board.h"

namespace fs = icm20948_fs;

static constexpr float kPi = 3.14159265358979F;

TEST(Icm20948Fs, PackRangeIsCouncilRange) {
    static_assert(fs::accel_range_valid(board::kImuAccelRangeG), "pack accel");
    static_assert(fs::gyro_range_valid(board::kImuGyroRangeDps), "pack gyro");
    EXPECT_EQ(board::kImuAccelRangeG, 16U);
    EXPECT_EQ(board::kImuGyroRangeDps, 2000U);
    EXPECT_EQ(fs::accel_fs_from_range(board::kImuAccelRangeG),
              ICM20948_ACCEL_FS_16G);
    EXPECT_EQ(fs::gyro_fs_from_range(board::kImuGyroRangeDps),
              ICM20948_GYRO_FS_2000DPS);
}

TEST(Icm20948Fs, RangeCodesRoundTrip) {
    const uint16_t accel[] = {2U, 4U, 8U, 16U};
    for (uint16_t g : accel) {
        EXPECT_TRUE(fs::accel_range_valid(g));
        EXPECT_EQ(fs::accel_range_g(fs::accel_fs_from_range(g)), g);
    }
    const uint16_t gyro[] = {250U, 500U, 1000U, 2000U};
    for (uint16_t dps : gyro) {
        EXPECT_TRUE(fs::gyro_range_valid(dps));
        EXPECT_EQ(fs::gyro_range_dps(fs::gyro_fs_from_range(dps)), dps);
    }
}

TEST(Icm20948Fs, RejectsRangesTheChipDoesNotHave) {
    EXPECT_FALSE(fs::accel_range_valid(0U));
    EXPECT_FALSE(fs::accel_range_valid(6U));
    EXPECT_FALSE(fs::accel_range_valid(32U));
    EXPECT_FALSE(fs::gyro_range_valid(0U));
    EXPECT_FALSE(fs::gyro_range_valid(4000U));
}

TEST(Icm20948Fs, ScaleMatchesDatasheetSensitivity) {
    // DS-000189 Table 2: 2048 LSB/g at ±16 g; Table 1: 16.4 LSB/dps at ±2000.
    EXPECT_FLOAT_EQ(fs::accel_scale(ICM20948_ACCEL_FS_16G), 9.80665F / 2048.0F);
    EXPECT_FLOAT_EQ(fs::accel_scale(ICM20948_ACCEL_FS_4G), 9.80665F / 8192.0F);
    const float deg_to_rad = kPi / 180.0F;
    EXPECT_FLOAT_EQ(fs::gyro_scale(ICM20948_GYRO_FS_2000DPS), deg_to_rad / 16.4F);
    EXPECT_FLOAT_EQ(fs::gyro_scale(ICM20948_GYRO_FS_500DPS), deg_to_rad / 65.5F);
}

TEST(Icm20948Fs, FullScaleCountsReachRange) {
    // int16 full scale (32767 LSB) times the scale is the configured range.
    const float accel_max = 32767.0F * fs::accel_scale(ICM20948_ACCEL_FS_16G);
    EXPECT_NEAR(accel_max, 16.0F * 9.80665F, 0.01F);
    const float gyro_max_dps =
        32767.0F * fs::gyro_scale(ICM20948_GYRO_FS_2000DPS) * 180.0F / kPi;
    EXPECT_NEAR(gyro_max_dps, 32767.0F / 16.4F, 0.01F);  // 1998 dps
}
