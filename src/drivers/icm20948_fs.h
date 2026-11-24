// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// ICM-20948 full-scale codes and scale factors (DS-000189 v1.3 Tables 1-2).
// Pure constexpr: shared by the driver, the CLI and host tests.

#ifndef ROCKETCHIP_ICM20948_FS_H
#define ROCKETCHIP_ICM20948_FS_H

#include <stdint.h>
#include "drivers/icm20948.h"

namespace icm20948_fs {

inline constexpr float kStdGravityMs2 = 9.80665F;  // ISO 80000-3
inline constexpr float kDegToRad = 3.14159265358979F / 180.0F;

// Sensitivity per FS_SEL code 0..3 (LSB/g, LSB/dps).
inline constexpr float kAccelLsbPerG[4] = {16384.0F, 8192.0F, 4096.0F, 2048.0F};
inline constexpr float kGyroLsbPerDps[4] = {131.0F, 65.5F, 32.8F, 16.4F};

inline constexpr bool accel_range_valid(uint16_t range_g) {
    return (range_g == 2U) || (range_g == 4U) || (range_g == 8U) ||
           (range_g == 16U);
}

inline constexpr bool gyro_range_valid(uint16_t range_dps) {
    return (range_dps == 250U) || (range_dps == 500U) ||
           (range_dps == 1000U) || (range_dps == 2000U);
}

// Caller static_asserts accel_range_valid first; anything else maps to 2 g.
inline constexpr icm20948_accel_fs_t accel_fs_from_range(uint16_t range_g) {
    if (range_g == 16U) {
        return ICM20948_ACCEL_FS_16G;
    }
    if (range_g == 8U) {
        return ICM20948_ACCEL_FS_8G;
    }
    if (range_g == 4U) {
        return ICM20948_ACCEL_FS_4G;
    }
    return ICM20948_ACCEL_FS_2G;
}

// Caller static_asserts gyro_range_valid first; anything else maps to 250 dps.
inline constexpr icm20948_gyro_fs_t gyro_fs_from_range(uint16_t range_dps) {
    if (range_dps == 2000U) {
        return ICM20948_GYRO_FS_2000DPS;
    }
    if (range_dps == 1000U) {
        return ICM20948_GYRO_FS_1000DPS;
    }
    if (range_dps == 500U) {
        return ICM20948_GYRO_FS_500DPS;
    }
    return ICM20948_GYRO_FS_250DPS;
}

inline constexpr uint16_t accel_range_g(icm20948_accel_fs_t fs) {
    return static_cast<uint16_t>(2U << static_cast<unsigned>(fs));
}

inline constexpr uint16_t gyro_range_dps(icm20948_gyro_fs_t fs) {
    return static_cast<uint16_t>(250U << static_cast<unsigned>(fs));
}

// m/s^2 per LSB.
inline constexpr float accel_scale(icm20948_accel_fs_t fs) {
    return kStdGravityMs2 / kAccelLsbPerG[static_cast<unsigned>(fs)];
}

// rad/s per LSB.
inline constexpr float gyro_scale(icm20948_gyro_fs_t fs) {
    return kDegToRad / kGyroLsbPerDps[static_cast<unsigned>(fs)];
}

}  // namespace icm20948_fs

#endif  // ROCKETCHIP_ICM20948_FS_H
