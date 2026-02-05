/**
 * @file calibration_manager.h
 * @brief Calibration routine manager
 *
 * Manages sensor calibration workflows:
 * - Gyro bias calibration (device at rest)
 * - Accelerometer level calibration (device flat)
 * - Barometer ground reference
 * - 6-position accel calibration (IVP-17, not yet implemented)
 */

#ifndef ROCKETCHIP_CALIBRATION_MANAGER_H
#define ROCKETCHIP_CALIBRATION_MANAGER_H

#include "calibration_data.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Calibration State
// ============================================================================

typedef enum {
    CAL_STATE_IDLE = 0,
    CAL_STATE_GYRO_SAMPLING,
    CAL_STATE_ACCEL_LEVEL_SAMPLING,
    CAL_STATE_BARO_SAMPLING,
    CAL_STATE_COMPLETE,
    CAL_STATE_FAILED,
} cal_state_t;

typedef enum {
    CAL_RESULT_OK = 0,
    CAL_RESULT_BUSY,            // Calibration already in progress
    CAL_RESULT_NO_DATA,         // Not enough samples
    CAL_RESULT_MOTION_DETECTED, // Device moved during calibration
    CAL_RESULT_TIMEOUT,         // Calibration took too long
    CAL_RESULT_INVALID_DATA,    // Data out of expected range
    CAL_RESULT_STORAGE_ERROR,   // Failed to save
} cal_result_t;

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize the calibration manager
 *
 * Loads existing calibration from storage, or uses defaults.
 */
void calibration_manager_init(void);

/**
 * @brief Get the current calibration data
 */
const calibration_store_t* calibration_manager_get(void);

/**
 * @brief Get the current calibration state
 */
cal_state_t calibration_manager_get_state(void);

// ============================================================================
// Gyro Calibration (IVP-15)
// ============================================================================

/**
 * @brief Start gyroscope bias calibration
 *
 * Device must be stationary. Takes ~2 seconds.
 */
cal_result_t calibration_start_gyro(void);

/**
 * @brief Feed gyro sample during calibration
 *
 * @param gx, gy, gz Gyro readings in rad/s
 * @param temperature_c Temperature in °C
 */
void calibration_feed_gyro(float gx, float gy, float gz, float temperature_c);

// ============================================================================
// Accelerometer Level Calibration (IVP-16)
// ============================================================================

/**
 * @brief Start accelerometer level calibration
 *
 * Device must be flat and stationary. Takes ~1 second.
 */
cal_result_t calibration_start_accel_level(void);

/**
 * @brief Feed accel sample during calibration
 */
void calibration_feed_accel(float ax, float ay, float az, float temperature_c);

// ============================================================================
// Barometer Calibration
// ============================================================================

/**
 * @brief Start barometer ground reference calibration
 *
 * Sets current pressure as ground reference. Takes ~1 second.
 */
cal_result_t calibration_start_baro(void);

/**
 * @brief Feed baro sample during calibration
 */
void calibration_feed_baro(float pressure_pa, float temperature_c);

// ============================================================================
// Calibration Control
// ============================================================================

/**
 * @brief Cancel any active calibration
 */
void calibration_cancel(void);

/**
 * @brief Reset state to IDLE after completion acknowledged
 */
void calibration_reset_state(void);

/**
 * @brief Check if calibration is in progress
 */
bool calibration_is_active(void);

/**
 * @brief Get progress of current calibration (0-100%)
 */
uint8_t calibration_get_progress(void);

/**
 * @brief Get result of last calibration
 */
cal_result_t calibration_get_result(void);

// ============================================================================
// Applying Calibration
// ============================================================================

/**
 * @brief Apply gyro calibration to raw reading
 */
void calibration_apply_gyro(float gx_raw, float gy_raw, float gz_raw,
                            float* gx_cal, float* gy_cal, float* gz_cal);

/**
 * @brief Apply accel calibration to raw reading
 */
void calibration_apply_accel(float ax_raw, float ay_raw, float az_raw,
                             float* ax_cal, float* ay_cal, float* az_cal);

/**
 * @brief Get altitude above ground from pressure
 */
float calibration_get_altitude_agl(float pressure_pa);

// ============================================================================
// Storage
// ============================================================================

/**
 * @brief Save current calibration to flash
 */
cal_result_t calibration_save(void);

/**
 * @brief Load calibration from flash
 */
cal_result_t calibration_load(void);

/**
 * @brief Reset calibration to factory defaults
 */
cal_result_t calibration_reset(void);

#ifdef __cplusplus
}
#endif

#endif // ROCKETCHIP_CALIBRATION_MANAGER_H
