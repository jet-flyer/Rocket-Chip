/**
 * @file storage_layout.h
 * @brief Storage region layout for RocketChip
 *
 * Defines logical regions within Tier 1 storage (AP_FlashStorage).
 * Total storage: 4KB, divided into calibration, config, and mission regions.
 *
 * See docs/AP_HAL_RP2350_PLAN.md Section 9.4 for details.
 */

#pragma once

#include <stdint.h>
#include "flash_map.h"

namespace rocketchip {

// ============================================================================
// Tier 1 Storage Regions
// ============================================================================
// | Region      | Offset | Size  | Contents                           |
// |-------------|--------|-------|-------------------------------------|
// | Calibration | 0      | 512B  | Sensor offsets, scales, cross-axis |
// | Config      | 512    | 512B  | Mission settings, preferences      |
// | Missions    | 1024   | 3KB   | Waypoints, geofence, rally points  |
// ============================================================================

// Calibration region (sensor calibration data)
constexpr uint16_t kCalibrationOffset = 0;
constexpr uint16_t kCalibrationSize = 512;

// Config region (system configuration)
constexpr uint16_t kConfigOffset = 512;
constexpr uint16_t kConfigSize = 512;

// Missions region (waypoints, geofence, etc.)
constexpr uint16_t kMissionsOffset = 1024;
constexpr uint16_t kMissionsSize = 3072;

// ============================================================================
// Calibration Data Structure
// ============================================================================

// Magic number to identify valid calibration data
constexpr uint32_t kCalibrationMagic = 0x52434341;  // "RCCA" - RocketChip CAl

// Calibration data version (increment when structure changes)
constexpr uint8_t kCalibrationVersion = 1;

/**
 * Sensor calibration data stored in flash.
 * Total size must fit within kCalibrationSize (512 bytes).
 */
struct SensorCalibration {
    // Header
    uint32_t magic;              // Must be kCalibrationMagic
    uint8_t version;             // Data structure version
    uint8_t flags;               // Bit flags for which calibrations are valid
    uint16_t reserved;           // Alignment padding

    // Accelerometer calibration (36 bytes)
    struct {
        float offset[3];         // Offset in m/s^2
        float scale[3];          // Scale factor (nominally 1.0)
        float cross_axis[3];     // Cross-axis correction
    } accel;

    // Gyroscope calibration (12 bytes)
    struct {
        float offset[3];         // Offset in rad/s
    } gyro;

    // Magnetometer calibration (40 bytes)
    // Matches CompassCalibrator output format:
    // - offset: hard iron correction (subtracted from raw)
    // - diag: diagonal soft iron (symmetric matrix diagonal)
    // - offdiag: off-diagonal soft iron (symmetric matrix off-diagonal)
    // - scale_factor: overall scale correction
    // Soft iron matrix is:
    //   [diag.x,    offdiag.x, offdiag.y]
    //   [offdiag.x, diag.y,    offdiag.z]
    //   [offdiag.y, offdiag.z, diag.z   ]
    struct {
        float offset[3];         // Hard iron offset (mGauss)
        float diag[3];           // Soft iron diagonal
        float offdiag[3];        // Soft iron off-diagonal
        float scale_factor;      // Overall scale factor
    } mag;

    // Barometer calibration (8 bytes)
    struct {
        float pressure_offset;   // Pressure offset in Pa
        float temp_offset;       // Temperature offset in C
    } baro;

    // CRC32 for data integrity (4 bytes)
    uint32_t crc32;

    // Total: 8 + 36 + 12 + 40 + 8 + 4 = 108 bytes
    // Leaves room for future expansion within 512 byte region
};

// Calibration flags
constexpr uint8_t kCalFlagAccel = 0x01;
constexpr uint8_t kCalFlagGyro  = 0x02;
constexpr uint8_t kCalFlagMag   = 0x04;
constexpr uint8_t kCalFlagBaro  = 0x08;
constexpr uint8_t kCalFlagAll   = 0x0F;

static_assert(sizeof(SensorCalibration) <= kCalibrationSize,
              "SensorCalibration exceeds allocated storage");

}  // namespace rocketchip
