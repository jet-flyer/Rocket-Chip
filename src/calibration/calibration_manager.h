/**
 * @file calibration_manager.h
 * @brief Calibration routine manager
 *
 * Manages sensor calibration workflows including:
 * - Gyro bias calibration (device at rest)
 * - Accelerometer level calibration (device flat)
 * - Barometer ground reference
 * - Future: 6-position accel cal, magnetometer cal
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
    CAL_STATE_ACCEL_6POS,           // 6-position accel calibration
    CAL_STATE_BARO_SAMPLING,
    CAL_STATE_COMPLETE,
    CAL_STATE_FAILED,
} cal_state_t;

typedef enum {
    CAL_RESULT_OK = 0,
    CAL_RESULT_BUSY,            // Calibration already in progress
    CAL_RESULT_NO_DATA,         // Not enough samples collected
    CAL_RESULT_MOTION_DETECTED, // Device moved during calibration
    CAL_RESULT_TIMEOUT,         // Calibration took too long
    CAL_RESULT_INVALID_DATA,    // Data out of expected range
    CAL_RESULT_STORAGE_ERROR,   // Failed to save calibration
} cal_result_t;

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize the calibration manager
 *
 * Loads existing calibration from storage, or initializes defaults.
 */
void calibration_manager_init(void);

/**
 * @brief Get the current calibration data
 * @return Pointer to current calibration store
 */
const calibration_store_t* calibration_manager_get(void);

/**
 * @brief Get the current calibration state
 * @return Current calibration state
 */
cal_state_t calibration_manager_get_state(void);

// ============================================================================
// Gyro Calibration
// ============================================================================

/**
 * @brief Start gyroscope bias calibration
 *
 * Device must be stationary. Collects samples to compute gyro bias.
 * Typically takes 2-3 seconds.
 *
 * @return CAL_RESULT_OK if calibration started, error otherwise
 */
cal_result_t calibration_start_gyro(void);

/**
 * @brief Feed gyro sample to calibration routine
 *
 * Called by SensorTask with each gyro reading during calibration.
 *
 * @param gx Gyro X in rad/s
 * @param gy Gyro Y in rad/s
 * @param gz Gyro Z in rad/s
 * @param temperature_c Current temperature
 */
void calibration_feed_gyro(float gx, float gy, float gz, float temperature_c);

// ============================================================================
// Accelerometer Level Calibration
// ============================================================================

/**
 * @brief Start accelerometer level calibration
 *
 * Device must be flat and stationary. Determines gravity reference.
 * Typically takes 1-2 seconds.
 *
 * @return CAL_RESULT_OK if calibration started, error otherwise
 */
cal_result_t calibration_start_accel_level(void);

/**
 * @brief Feed accel sample to calibration routine
 *
 * Called by SensorTask with each accel reading during calibration.
 *
 * @param ax Accel X in m/s²
 * @param ay Accel Y in m/s²
 * @param az Accel Z in m/s²
 * @param temperature_c Current temperature
 */
void calibration_feed_accel(float ax, float ay, float az, float temperature_c);

// ============================================================================
// 6-Position Accelerometer Calibration
// ============================================================================

/**
 * @brief Start 6-position accelerometer calibration
 *
 * Full calibration computing offset and scale for all axes.
 * User must place device in 6 orientations when prompted.
 *
 * @return CAL_RESULT_OK if calibration started, error otherwise
 */
cal_result_t calibration_start_accel_6pos(void);

/**
 * @brief Get current 6-position calibration position
 *
 * Returns the position user should place device in.
 *
 * @return Position index (0-5), or -1 if not in 6-pos calibration
 */
int8_t calibration_get_6pos_position(void);

/**
 * @brief Get name of current 6-position calibration position
 * @return Position name string (e.g., "LEVEL (flat)")
 */
const char* calibration_get_6pos_position_name(void);

/**
 * @brief User signals device is in position, start collecting
 *
 * Call when user has placed device in the requested orientation.
 *
 * @return true if collection started
 */
bool calibration_accept_6pos_position(void);

/**
 * @brief Get 6-position calibration current position progress
 * @return Progress 0-100% for current position
 */
uint8_t calibration_get_6pos_position_progress(void);

// ============================================================================
// Barometer Calibration
// ============================================================================

/**
 * @brief Start barometer ground reference calibration
 *
 * Sets the current pressure as ground reference.
 * Typically takes 1 second for averaging.
 *
 * @return CAL_RESULT_OK if calibration started, error otherwise
 */
cal_result_t calibration_start_baro(void);

/**
 * @brief Feed baro sample to calibration routine
 *
 * @param pressure_pa Pressure in Pascals
 * @param temperature_c Temperature in Celsius
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
 * @brief Reset state to IDLE after completion/failure acknowledged
 *
 * Call after reading the result to allow starting a new calibration.
 */
void calibration_reset_state(void);

/**
 * @brief Check if calibration is in progress
 * @return true if calibration is active
 */
bool calibration_is_active(void);

/**
 * @brief Get progress of current calibration (0-100%)
 * @return Progress percentage
 */
uint8_t calibration_get_progress(void);

/**
 * @brief Get result of last calibration
 * @return Result code
 */
cal_result_t calibration_get_result(void);

// ============================================================================
// Applying Calibration
// ============================================================================

/**
 * @brief Apply gyro calibration to raw reading
 * @param gx_raw Raw gyro X (rad/s)
 * @param gy_raw Raw gyro Y (rad/s)
 * @param gz_raw Raw gyro Z (rad/s)
 * @param gx_cal Output calibrated X
 * @param gy_cal Output calibrated Y
 * @param gz_cal Output calibrated Z
 */
void calibration_apply_gyro(float gx_raw, float gy_raw, float gz_raw,
                            float* gx_cal, float* gy_cal, float* gz_cal);

/**
 * @brief Apply accel calibration to raw reading
 * @param ax_raw Raw accel X (m/s²)
 * @param ay_raw Raw accel Y (m/s²)
 * @param az_raw Raw accel Z (m/s²)
 * @param ax_cal Output calibrated X
 * @param ay_cal Output calibrated Y
 * @param az_cal Output calibrated Z
 */
void calibration_apply_accel(float ax_raw, float ay_raw, float az_raw,
                             float* ax_cal, float* ay_cal, float* az_cal);

/**
 * @brief Get ground reference altitude from calibrated pressure
 * @param pressure_pa Current pressure in Pascals
 * @return Altitude above ground in meters
 */
float calibration_get_altitude_agl(float pressure_pa);

// ============================================================================
// Storage
// ============================================================================

/**
 * @brief Save current calibration to persistent storage
 * @return CAL_RESULT_OK on success
 */
cal_result_t calibration_save(void);

/**
 * @brief Load calibration from persistent storage
 * @return CAL_RESULT_OK on success, or initializes defaults
 */
cal_result_t calibration_load(void);

/**
 * @brief Reset calibration to factory defaults
 * @return CAL_RESULT_OK on success
 */
cal_result_t calibration_reset(void);

#ifdef __cplusplus
}
#endif

#endif // ROCKETCHIP_CALIBRATION_MANAGER_H
