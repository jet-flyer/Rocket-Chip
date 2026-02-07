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
    CAL_RESULT_FIT_FAILED,      // Ellipsoid fit did not converge or params out of range
} cal_result_t;

/**
 * @brief Callback to read one accelerometer sample
 *
 * Used by 6-pos calibration to decouple from sensor driver.
 * Implementation should block until a fresh sample is available (~10ms).
 * @return true on success
 */
typedef bool (*accel_read_fn)(float *ax, float *ay, float *az, float *temp_c);

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
// 6-Position Accelerometer Calibration (IVP-17)
// ============================================================================

/**
 * @brief Collect samples for one position of 6-pos calibration
 *
 * Blocking function: reads 50 samples via callback, checks motion
 * and orientation. Positions must be collected in order 0-5.
 *
 * @param pos Position index (0-5)
 * @param read_fn Callback that reads one accel sample (blocks ~10ms)
 * @return CAL_RESULT_OK on success,
 *         CAL_RESULT_MOTION_DETECTED if device moved,
 *         CAL_RESULT_INVALID_DATA if orientation doesn't match expected
 */
cal_result_t calibration_collect_6pos_position(uint8_t pos, accel_read_fn read_fn);

/**
 * @brief Run Gauss-Newton ellipsoid fit on collected 6-pos data
 *
 * Requires all 6 positions collected first.
 * On success, stores offset/scale/offdiag to calibration data.
 *
 * @return CAL_RESULT_OK on success, CAL_RESULT_FIT_FAILED if fit diverged
 */
cal_result_t calibration_compute_6pos(void);

/**
 * @brief Reset all 6-pos collection state
 */
void calibration_reset_6pos(void);

/**
 * @brief Get average readings for a completed position
 * @return Pointer to float[3] {ax, ay, az} or NULL if invalid
 */
const float* calibration_get_6pos_avg(uint8_t pos);

/**
 * @brief Get human-readable name for a position
 */
const char* calibration_get_6pos_name(uint8_t pos);

// ============================================================================
// Applying Calibration
// ============================================================================

/**
 * @brief Apply gyro calibration to raw reading (uses global calibration)
 */
void calibration_apply_gyro(float gx_raw, float gy_raw, float gz_raw,
                            float* gx_cal, float* gy_cal, float* gz_cal);

/**
 * @brief Apply accel calibration to raw reading (uses global calibration)
 */
void calibration_apply_accel(float ax_raw, float ay_raw, float az_raw,
                             float* ax_cal, float* ay_cal, float* az_cal);

/**
 * @brief Apply gyro calibration using explicit calibration data
 *
 * For cross-core use where Core 1 has its own calibration copy.
 */
void calibration_apply_gyro_with(const calibration_store_t *cal,
                                  float gx_raw, float gy_raw, float gz_raw,
                                  float* gx_cal, float* gy_cal, float* gz_cal);

/**
 * @brief Apply accel calibration using explicit calibration data
 *
 * For cross-core use where Core 1 has its own calibration copy.
 */
void calibration_apply_accel_with(const calibration_store_t *cal,
                                   float ax_raw, float ay_raw, float az_raw,
                                   float* ax_cal, float* ay_cal, float* az_cal);

/**
 * @brief Load calibration into a caller-supplied buffer
 *
 * Reads from the cached copy in RAM (no flash access). Safe to call from Core 1.
 * @return true if valid calibration was loaded
 */
bool calibration_load_into(calibration_store_t* dest);

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
