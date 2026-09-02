// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// ICM-20948 + AK09916 (bypass on the external bus).
// Prior Art: DS-000189; AKD09916; ArduPilot Invensensev2 bypass.

#ifndef ROCKETCHIP_ICM20948_H
#define ROCKETCHIP_ICM20948_H

#include <stdint.h>
#include <stdbool.h>
#include "rocketchip/i2c_strap.h"

constexpr uint8_t kIcm20948AddrDefault = kI2cAddrIcm20948;
constexpr uint8_t kIcm20948AddrAlt     = kI2cAddrIcm20948Alt;
constexpr uint8_t kIcm20948WhoAmI      = 0xEA;
constexpr uint8_t kAk09916WhoAmI       = 0x09;

typedef enum {
    ICM20948_ACCEL_FS_2G  = 0,
    ICM20948_ACCEL_FS_4G  = 1,
    ICM20948_ACCEL_FS_8G  = 2,
    ICM20948_ACCEL_FS_16G = 3,
} icm20948_accel_fs_t;

typedef enum {
    ICM20948_GYRO_FS_250DPS  = 0,
    ICM20948_GYRO_FS_500DPS  = 1,
    ICM20948_GYRO_FS_1000DPS = 2,
    ICM20948_GYRO_FS_2000DPS = 3,
} icm20948_gyro_fs_t;

typedef struct {
    float x;
    float y;
    float z;
} icm20948_vec3_t;

typedef struct {
    icm20948_vec3_t accel;
    icm20948_vec3_t gyro;
    icm20948_vec3_t mag;
    float temperature_c;
    bool accel_valid;
    bool gyro_valid;
    bool mag_valid;
} icm20948_data_t;

typedef struct {
    uint8_t addr;
    bool initialized;
    bool mag_initialized;
    icm20948_accel_fs_t accel_fs;
    icm20948_gyro_fs_t gyro_fs;
    float accel_scale;
    float gyro_scale;
    float mag_scale;
    uint8_t current_bank;
} icm20948_t;

bool icm20948_init(icm20948_t* dev, uint8_t addr);
bool icm20948_ready(icm20948_t* dev);
// CLKSEL + sensors on + bypass. No DEVICE_RESET (STEMMA has no nRESET).
bool icm20948_ensure_awake(icm20948_t* dev);
bool icm20948_read(icm20948_t* dev, icm20948_data_t* data);
bool icm20948_read_config_registers(icm20948_t* dev, uint8_t* accel_config,
                                    uint8_t* gyro_config1, uint8_t* gyro_smplrt);

#endif
