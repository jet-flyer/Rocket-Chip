// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// ICM-20948 9-axis IMU driver (Accel + Gyro + AK09916 Magnetometer)
// TDK InvenSense ICM-20948:
// - Accelerometer: ±2/4/8/16g, 16-bit
// - Gyroscope: ±250/500/1000/2000 dps, 16-bit
// - Magnetometer (AK09916): ±4900µT, 16-bit
// Reference: ICM-20948 datasheet, AK09916 datasheet

#ifndef ROCKETCHIP_ICM20948_H
#define ROCKETCHIP_ICM20948_H

#include <stdint.h>
#include <stdbool.h>
#include "i2c_bus.h"

// ============================================================================
// Configuration
// ============================================================================

constexpr uint8_t kIcm20948AddrDefault = kI2cAddrIcm20948;
constexpr uint8_t kIcm20948AddrAlt     = kI2cAddrIcm20948Alt;

// Device IDs
constexpr uint8_t kIcm20948WhoAmI       = 0xEA;
constexpr uint8_t kAk09916WhoAmI        = 0x09;

// ============================================================================
// Types
// ============================================================================

typedef enum {
    ICM20948_ACCEL_FS_2G  = 0,  // ±2g
    ICM20948_ACCEL_FS_4G  = 1,  // ±4g
    ICM20948_ACCEL_FS_8G  = 2,  // ±8g
    ICM20948_ACCEL_FS_16G = 3,  // ±16g
} icm20948_accel_fs_t;

typedef enum {
    ICM20948_GYRO_FS_250DPS  = 0,  // ±250 dps
    ICM20948_GYRO_FS_500DPS  = 1,  // ±500 dps
    ICM20948_GYRO_FS_1000DPS = 2,  // ±1000 dps
    ICM20948_GYRO_FS_2000DPS = 3,  // ±2000 dps
} icm20948_gyro_fs_t;

typedef enum {
    AK09916_MODE_POWER_DOWN   = 0x00,
    AK09916_MODE_SINGLE       = 0x01,  // Single measurement
    AK09916_MODE_CONT_10HZ    = 0x02,  // Continuous 10 Hz
    AK09916_MODE_CONT_20HZ    = 0x04,  // Continuous 20 Hz
    AK09916_MODE_CONT_50HZ    = 0x06,  // Continuous 50 Hz
    AK09916_MODE_CONT_100HZ   = 0x08,  // Continuous 100 Hz
} ak09916_mode_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} icm20948_raw_t;

typedef struct {
    float x;
    float y;
    float z;
} icm20948_vec3_t;

typedef struct {
    icm20948_vec3_t accel;      // Acceleration in m/s²
    icm20948_vec3_t gyro;       // Angular rate in rad/s
    icm20948_vec3_t mag;        // Magnetic field in µT
    float temperature_c;         // Die temperature in °C
    bool accel_valid;
    bool gyro_valid;
    bool mag_valid;
} icm20948_data_t;

typedef struct {
    uint8_t addr;
    bool initialized;
    bool mag_initialized;

    // Configuration
    icm20948_accel_fs_t accel_fs;
    icm20948_gyro_fs_t gyro_fs;
    ak09916_mode_t mag_mode;

    // Scale factors (calculated from FS settings)
    float accel_scale;  // LSB to m/s²
    float gyro_scale;   // LSB to rad/s
    float mag_scale;    // LSB to µT (fixed for AK09916)
} icm20948_t;

// ============================================================================
// Initialization
// ============================================================================

bool icm20948_init(icm20948_t* dev, uint8_t addr);

bool icm20948_ready(icm20948_t* dev);

bool icm20948_reset(icm20948_t* dev);

// ============================================================================
// Configuration
// ============================================================================

bool icm20948_set_accel_fs(icm20948_t* dev, icm20948_accel_fs_t fs);

bool icm20948_set_gyro_fs(icm20948_t* dev, icm20948_gyro_fs_t fs);

bool icm20948_set_mag_mode(icm20948_t* dev, ak09916_mode_t mode);

bool icm20948_set_low_power(icm20948_t* dev, bool enable);

// ============================================================================
// Data Reading
// ============================================================================

bool icm20948_read(icm20948_t* dev, icm20948_data_t* data);

// Six-byte burst from ACCEL_XOUT_H does not reach TEMP_OUT_L, so the
// IMU data-ready flag is never cleared. After ~200 calls the output
// registers freeze at zeros. Sampling loops must use icm20948_read()
// (14-byte burst through TEMP_OUT_L).
bool icm20948_read_accel(icm20948_t* dev, icm20948_vec3_t* accel);

bool icm20948_read_gyro(icm20948_t* dev, icm20948_vec3_t* gyro);

bool icm20948_read_mag(icm20948_t* dev, icm20948_vec3_t* mag);

bool icm20948_read_temperature(icm20948_t* dev, float* tempC);

bool icm20948_data_ready(icm20948_t* dev, bool* accelReady, bool* gyroReady);

bool icm20948_read_config_registers(icm20948_t* dev,
                                     uint8_t* accel_config,
                                     uint8_t* gyro_config1,
                                     uint8_t* gyro_smplrt);

// Stuck-slave recovery (27 SCL pulses + Bank 0 + PWR_MGMT_1 reset).
// Lives on the IMU driver, not the generic I2C bus (WN-078).
bool icm20948_stuck_slave_recovery(uint8_t addr);

#endif // ROCKETCHIP_ICM20948_H
