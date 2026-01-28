/**
 * @file ParamStorageLayout.h
 * @brief Storage layout for AP_Param system
 *
 * Defines storage offsets for all persistent parameters within the
 * 4KB HAL_STORAGE_SIZE region managed by AP_FlashStorage.
 *
 * Part of RocketChip's ArduPilot-compatible parameter system.
 */

#pragma once

#include <cstdint>

namespace RocketChip {

// ============================================================================
// Storage Header
// ============================================================================

// Magic number to identify valid parameter storage
constexpr uint32_t kParamMagic = 0x524B4350;  // "RKCP" (RocketChip Param)

// Storage format version (increment when layout changes)
constexpr uint16_t kParamVersion = 1;

// Header structure offsets
constexpr uint16_t kHeaderMagicOffset = 0x000;    // uint32_t magic
constexpr uint16_t kHeaderVersionOffset = 0x004;  // uint16_t version
constexpr uint16_t kHeaderFlagsOffset = 0x006;    // uint16_t flags
constexpr uint16_t kHeaderSize = 0x008;

// Reserved space after header
constexpr uint16_t kReservedEnd = 0x100;

// ============================================================================
// Accelerometer Calibration (0x100 - 0x1FF)
// ============================================================================
// Each Vector3f is 12 bytes (3 x float)
// Supports up to 2 IMU instances

constexpr uint16_t kAccelCalBase = 0x100;

// Instance 0 (primary IMU)
constexpr uint16_t kAccelOffset0 = 0x100;      // Vector3f offset (m/s^2)
constexpr uint16_t kAccelScale0 = 0x10C;       // Vector3f scale factors
constexpr uint16_t kAccelOffdiag0 = 0x118;     // Vector3f off-diagonal terms

// Instance 1 (secondary IMU, if present)
constexpr uint16_t kAccelOffset1 = 0x150;
constexpr uint16_t kAccelScale1 = 0x15C;
constexpr uint16_t kAccelOffdiag1 = 0x168;

// Stride between instances
constexpr uint16_t kAccelInstanceStride = 0x50;  // 80 bytes per instance

// ============================================================================
// Compass Calibration (0x200 - 0x2FF)
// ============================================================================
// Supports up to 2 compass instances

constexpr uint16_t kCompassCalBase = 0x200;

// Instance 0 (primary compass)
constexpr uint16_t kCompassOffset0 = 0x200;    // Vector3f hard iron offset
constexpr uint16_t kCompassDiag0 = 0x20C;      // Vector3f diagonal soft iron
constexpr uint16_t kCompassOffdiag0 = 0x218;   // Vector3f off-diagonal soft iron
constexpr uint16_t kCompassScaleFactor0 = 0x224;  // float scale factor

// Instance 1 (secondary compass, if present)
constexpr uint16_t kCompassOffset1 = 0x250;
constexpr uint16_t kCompassDiag1 = 0x25C;
constexpr uint16_t kCompassOffdiag1 = 0x268;
constexpr uint16_t kCompassScaleFactor1 = 0x274;

// Stride between instances
constexpr uint16_t kCompassInstanceStride = 0x50;  // 80 bytes per instance

// ============================================================================
// Gyroscope Calibration (0x300 - 0x3FF)
// ============================================================================
// Gyro typically only needs offset calibration (no scale/cross-axis)

constexpr uint16_t kGyroCalBase = 0x300;

// Instance 0
constexpr uint16_t kGyroOffset0 = 0x300;       // Vector3f offset (rad/s)

// Instance 1
constexpr uint16_t kGyroOffset1 = 0x350;

// Stride between instances
constexpr uint16_t kGyroInstanceStride = 0x50;

// ============================================================================
// Barometer Calibration (0x380 - 0x3FF)
// ============================================================================

constexpr uint16_t kBaroCalBase = 0x380;

constexpr uint16_t kBaroPressureOffset0 = 0x380;  // float pressure offset (Pa)
constexpr uint16_t kBaroTempOffset0 = 0x384;      // float temperature offset (C)

// ============================================================================
// Configuration Parameters (0x400 - 0x7FF)
// ============================================================================
// Reserved for future mission/system configuration

constexpr uint16_t kConfigBase = 0x400;
constexpr uint16_t kConfigEnd = 0x800;

// ============================================================================
// Mission Storage (0x800 - 0xFFF)
// ============================================================================
// Reserved for waypoints, geofence, rally points

constexpr uint16_t kMissionBase = 0x800;
constexpr uint16_t kMissionEnd = 0x1000;  // 4KB total

// ============================================================================
// Calibration Validity Flags (stored in header)
// ============================================================================

constexpr uint16_t kCalFlagAccel0 = 0x0001;
constexpr uint16_t kCalFlagAccel1 = 0x0002;
constexpr uint16_t kCalFlagCompass0 = 0x0004;
constexpr uint16_t kCalFlagCompass1 = 0x0008;
constexpr uint16_t kCalFlagGyro0 = 0x0010;
constexpr uint16_t kCalFlagGyro1 = 0x0020;
constexpr uint16_t kCalFlagBaro = 0x0040;

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Get accelerometer offset storage address for given instance
 * @param instance IMU instance (0 or 1)
 * @return Storage offset for accel offset Vector3f
 */
inline constexpr uint16_t getAccelOffsetAddr(uint8_t instance) {
    return kAccelCalBase + (instance * kAccelInstanceStride);
}

/**
 * @brief Get accelerometer scale storage address for given instance
 */
inline constexpr uint16_t getAccelScaleAddr(uint8_t instance) {
    return kAccelCalBase + (instance * kAccelInstanceStride) + 0x0C;
}

/**
 * @brief Get accelerometer off-diagonal storage address for given instance
 */
inline constexpr uint16_t getAccelOffdiagAddr(uint8_t instance) {
    return kAccelCalBase + (instance * kAccelInstanceStride) + 0x18;
}

/**
 * @brief Get compass offset storage address for given instance
 */
inline constexpr uint16_t getCompassOffsetAddr(uint8_t instance) {
    return kCompassCalBase + (instance * kCompassInstanceStride);
}

/**
 * @brief Get gyro offset storage address for given instance
 */
inline constexpr uint16_t getGyroOffsetAddr(uint8_t instance) {
    return kGyroCalBase + (instance * kGyroInstanceStride);
}

}  // namespace RocketChip
