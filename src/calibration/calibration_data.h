/**
 * @file calibration_data.h
 * @brief Calibration data structures for all sensors
 *
 * Defines the calibration parameters that are stored persistently
 * and applied to sensor readings during operation.
 */

#ifndef ROCKETCHIP_CALIBRATION_DATA_H
#define ROCKETCHIP_CALIBRATION_DATA_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Magic Numbers and Version
// ============================================================================

#define CALIBRATION_MAGIC       0x52434341  // "RCCA" - RocketChip Calibration
#define CALIBRATION_VERSION     1

// ============================================================================
// Calibration Status Flags
// ============================================================================

typedef enum {
    CAL_STATUS_NONE         = 0,
    CAL_STATUS_LEVEL        = (1 << 0),  // Basic level cal done
    CAL_STATUS_ACCEL_6POS   = (1 << 1),  // Full 6-position accel cal done
    CAL_STATUS_GYRO         = (1 << 2),  // Gyro bias calibration done
    CAL_STATUS_MAG          = (1 << 3),  // Magnetometer cal done
    CAL_STATUS_BARO         = (1 << 4),  // Barometer ground pressure set
} cal_status_flags_t;

// ============================================================================
// 3D Vector (for offsets/scales)
// ============================================================================

typedef struct {
    float x;
    float y;
    float z;
} cal_vec3_t;

// ============================================================================
// Accelerometer Calibration
// ============================================================================

/**
 * @brief Accelerometer calibration data
 *
 * ArduPilot convention: corrected = (raw + offset) * scale
 * For full 6-position cal, offset and scale factors are computed.
 * For level cal only, offsets are computed assuming scale=1.
 */
typedef struct {
    cal_vec3_t offset;      ///< Offset in m/s² (add to raw, per ArduPilot convention)
    cal_vec3_t scale;       ///< Scale factors (multiply after offset)
    float temperature_ref;  ///< Temperature at calibration time (°C)
    uint8_t status;         ///< CAL_STATUS_LEVEL or CAL_STATUS_ACCEL_6POS
} accel_cal_t;

// ============================================================================
// Gyroscope Calibration
// ============================================================================

/**
 * @brief Gyroscope calibration data
 *
 * Gyro calibration is primarily bias removal at rest.
 * Temperature compensation can be added later.
 */
typedef struct {
    cal_vec3_t bias;        ///< Bias in rad/s (subtract from raw)
    float temperature_ref;  ///< Temperature at calibration time (°C)
    uint8_t status;         ///< CAL_STATUS_GYRO if calibrated
} gyro_cal_t;

// ============================================================================
// Magnetometer Calibration
// ============================================================================

/**
 * @brief Magnetometer calibration data
 *
 * Full magnetometer calibration includes:
 * - Hard iron offset (additive bias from nearby ferrous materials)
 * - Soft iron matrix (distortion of the magnetic field)
 *
 * corrected = soft_iron * (raw - hard_iron)
 *
 * For simplicity, we start with hard iron only.
 */
typedef struct {
    cal_vec3_t hard_iron;   ///< Hard iron offset in µT
    float soft_iron[9];     ///< 3x3 soft iron matrix (row-major)
    float declination_deg;  ///< Magnetic declination for location
    float field_strength;   ///< Expected field strength (for validation)
    uint8_t status;         ///< CAL_STATUS_MAG if calibrated
} mag_cal_t;

// ============================================================================
// Barometer Calibration
// ============================================================================

/**
 * @brief Barometer calibration data
 *
 * Barometer cal stores the ground-level pressure reference
 * for altitude calculations.
 */
typedef struct {
    float ground_pressure_pa;   ///< Pressure at ground level
    float ground_temperature_c; ///< Temperature at calibration
    float altitude_offset_m;    ///< Altitude offset (if at known elevation)
    uint8_t status;             ///< CAL_STATUS_BARO if calibrated
} baro_cal_t;

// ============================================================================
// Complete Calibration Storage Structure
// ============================================================================

/**
 * @brief Complete calibration data stored in flash
 *
 * This structure is persisted to flash and loaded at boot.
 * Total size should be < 512 bytes to fit in Tier 1 storage.
 */
typedef struct {
    // Header
    uint32_t magic;         ///< CALIBRATION_MAGIC
    uint16_t version;       ///< CALIBRATION_VERSION
    uint16_t crc16;         ///< CRC16 of data following this field

    // Calibration data
    accel_cal_t accel;
    gyro_cal_t gyro;
    mag_cal_t mag;
    baro_cal_t baro;

    // Overall status
    uint32_t cal_flags;     ///< Bitfield of cal_status_flags_t
    uint32_t cal_timestamp; ///< Unix timestamp of last calibration

    // Padding for future expansion
    uint8_t reserved[32];
} calibration_store_t;

// Verify size fits in Tier 1 storage (512 bytes)
_Static_assert(sizeof(calibration_store_t) <= 512,
               "calibration_store_t exceeds 512 byte limit");

// ============================================================================
// Default Values
// ============================================================================

/**
 * @brief Initialize calibration data with defaults
 * @param cal Calibration store to initialize
 */
void calibration_init_defaults(calibration_store_t* cal);

/**
 * @brief Check if calibration data is valid
 * @param cal Calibration store to validate
 * @return true if magic, version, and CRC are valid
 */
bool calibration_validate(const calibration_store_t* cal);

/**
 * @brief Update CRC after modifying calibration data
 * @param cal Calibration store to update
 */
void calibration_update_crc(calibration_store_t* cal);

/**
 * @brief Check if a specific calibration has been performed
 * @param cal Calibration store
 * @param flag Flag from cal_status_flags_t
 * @return true if that calibration is complete
 */
bool calibration_has(const calibration_store_t* cal, cal_status_flags_t flag);

#ifdef __cplusplus
}
#endif

#endif // ROCKETCHIP_CALIBRATION_DATA_H
