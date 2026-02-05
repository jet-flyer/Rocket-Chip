/**
 * @file calibration_data.h
 * @brief Calibration data structures for RocketChip sensors
 *
 * Defines calibration parameters stored persistently in flash.
 * All values use SI units (m/s², rad/s, µT, Pa).
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
// 3D Vector
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
 * Convention: corrected = (raw + offset) * scale
 */
typedef struct {
    cal_vec3_t offset;      ///< Offset in m/s² (add to raw)
    cal_vec3_t scale;       ///< Scale factors (multiply after offset)
    float temperature_ref;  ///< Temperature at calibration time (°C)
    uint8_t status;         ///< CAL_STATUS_LEVEL or CAL_STATUS_ACCEL_6POS
    uint8_t _pad[3];
} accel_cal_t;

// ============================================================================
// Gyroscope Calibration
// ============================================================================

/**
 * @brief Gyroscope calibration data
 *
 * Convention: corrected = raw - bias
 */
typedef struct {
    cal_vec3_t bias;        ///< Bias in rad/s (subtract from raw)
    float temperature_ref;  ///< Temperature at calibration time (°C)
    uint8_t status;         ///< CAL_STATUS_GYRO if calibrated
    uint8_t _pad[3];
} gyro_cal_t;

// ============================================================================
// Barometer Calibration
// ============================================================================

/**
 * @brief Barometer calibration data
 */
typedef struct {
    float ground_pressure_pa;   ///< Pressure at ground level
    float ground_temperature_c; ///< Temperature at calibration
    uint8_t status;             ///< CAL_STATUS_BARO if calibrated
    uint8_t _pad[3];
} baro_cal_t;

// ============================================================================
// Complete Calibration Storage Structure
// ============================================================================

/**
 * @brief Complete calibration data stored in flash
 *
 * Total size should be < 256 bytes to fit in a single flash page.
 */
typedef struct {
    // Header (8 bytes)
    uint32_t magic;         ///< CALIBRATION_MAGIC
    uint16_t version;       ///< CALIBRATION_VERSION
    uint16_t crc16;         ///< CRC16 of data following this field

    // Calibration data
    accel_cal_t accel;      // 32 bytes
    gyro_cal_t gyro;        // 20 bytes
    baro_cal_t baro;        // 12 bytes

    // Overall status
    uint32_t cal_flags;     ///< Bitfield of cal_status_flags_t

    // Reserved for future expansion
    uint8_t reserved[24];
} calibration_store_t;

// Verify size fits in a flash page
_Static_assert(sizeof(calibration_store_t) <= 256,
               "calibration_store_t exceeds 256 byte limit");

// ============================================================================
// Functions
// ============================================================================

/**
 * @brief Initialize calibration data with defaults
 */
void calibration_init_defaults(calibration_store_t* cal);

/**
 * @brief Validate calibration data (magic, version, CRC)
 * @return true if valid
 */
bool calibration_validate(const calibration_store_t* cal);

/**
 * @brief Update CRC after modifying calibration data
 */
void calibration_update_crc(calibration_store_t* cal);

/**
 * @brief Check if a specific calibration has been performed
 */
bool calibration_has(const calibration_store_t* cal, cal_status_flags_t flag);

#ifdef __cplusplus
}
#endif

#endif // ROCKETCHIP_CALIBRATION_DATA_H
