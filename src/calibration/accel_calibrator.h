/**
 * @file accel_calibrator.h
 * @brief 6-position accelerometer calibrator (ArduPilot AccelCalibrator pattern)
 *
 * Implements ArduPilot's ellipsoid-fitting algorithm for accelerometer calibration.
 * Device is placed in 6 orientations, samples collected, then ellipsoid fit
 * to determine offset and scale factors.
 *
 * Reference: ArduPilot AccelCalibrator class
 */

#ifndef ROCKETCHIP_ACCEL_CALIBRATOR_H
#define ROCKETCHIP_ACCEL_CALIBRATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "calibration_data.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Configuration
// ============================================================================

#define ACCEL_CAL_NUM_POSITIONS     6       // Standard 6-position calibration
#define ACCEL_CAL_SAMPLES_PER_POS   10      // Samples to average per position
#define ACCEL_CAL_MAX_ITERATIONS    50      // Gauss-Newton iteration limit
#define ACCEL_CAL_FITNESS_TOLERANCE 0.1f    // Target residual (m/s²)
#define ACCEL_CAL_MOTION_THRESHOLD  0.5f    // Motion detection (m/s²)

// Gravity constant
#define GRAVITY_MSS                 9.80665f

// ============================================================================
// Types
// ============================================================================

/**
 * @brief Calibration positions
 */
typedef enum {
    ACCEL_CAL_POS_LEVEL = 0,    // Z-up (flat on table)
    ACCEL_CAL_POS_LEFT,         // Y-up (left side down)
    ACCEL_CAL_POS_RIGHT,        // Y-down (right side down)
    ACCEL_CAL_POS_NOSE_DOWN,    // X-down (nose pointing down)
    ACCEL_CAL_POS_NOSE_UP,      // X-up (nose pointing up)
    ACCEL_CAL_POS_BACK,         // Z-down (inverted)
    ACCEL_CAL_POS_COUNT
} accel_cal_position_t;

/**
 * @brief Calibrator state
 */
typedef enum {
    ACCEL_CAL_STATE_IDLE = 0,
    ACCEL_CAL_STATE_WAITING,        // Waiting for device to be placed
    ACCEL_CAL_STATE_COLLECTING,     // Collecting samples at current position
    ACCEL_CAL_STATE_POSITION_DONE,  // Position complete, waiting for next
    ACCEL_CAL_STATE_FITTING,        // Running ellipsoid fit
    ACCEL_CAL_STATE_SUCCESS,        // Calibration complete and valid
    ACCEL_CAL_STATE_FAILED,         // Calibration failed
} accel_cal_state_t;

/**
 * @brief Failure reasons
 */
typedef enum {
    ACCEL_CAL_FAIL_NONE = 0,
    ACCEL_CAL_FAIL_MOTION,          // Motion detected during sampling
    ACCEL_CAL_FAIL_SAMPLE_SIMILAR,  // Sample too similar to existing
    ACCEL_CAL_FAIL_FIT_FAILED,      // Ellipsoid fit didn't converge
    ACCEL_CAL_FAIL_OFFSET_OOB,      // Offset out of bounds (>5 m/s²)
    ACCEL_CAL_FAIL_SCALE_OOB,       // Scale out of bounds (0.8-1.2)
    ACCEL_CAL_FAIL_TIMEOUT,         // Took too long
} accel_cal_fail_t;

/**
 * @brief Collected sample (averaged reading at one position)
 */
typedef struct {
    float x, y, z;
} accel_sample_t;

/**
 * @brief Calibration parameters (6-parameter axis-aligned fit)
 */
typedef struct {
    cal_vec3_t offset;              // Offset per axis
    cal_vec3_t diag;                // Scale (diagonal) per axis
} accel_cal_params_t;

/**
 * @brief Calibrator instance
 */
typedef struct {
    // State
    accel_cal_state_t state;
    accel_cal_fail_t fail_reason;
    accel_cal_position_t current_position;
    uint8_t positions_completed;

    // Sample collection
    accel_sample_t samples[ACCEL_CAL_NUM_POSITIONS];
    uint8_t sample_count;               // Current position sample count
    float sample_sum_x, sample_sum_y, sample_sum_z;
    float sample_min_x, sample_max_x;
    float sample_min_y, sample_max_y;
    float sample_min_z, sample_max_z;
    float temperature_sum;

    // Result
    accel_cal_params_t params;
    float fitness;                      // Mean squared residual
    float temperature_ref;
} accel_calibrator_t;

// ============================================================================
// Public API
// ============================================================================

/**
 * @brief Initialize the calibrator
 * @param cal Calibrator instance
 */
void accel_cal_init(accel_calibrator_t* cal);

/**
 * @brief Start 6-position calibration
 * @param cal Calibrator instance
 * @return true if started successfully
 */
bool accel_cal_start(accel_calibrator_t* cal);

/**
 * @brief Get current calibration state
 * @param cal Calibrator instance
 * @return Current state
 */
accel_cal_state_t accel_cal_get_state(accel_calibrator_t* cal);

/**
 * @brief Get current position being calibrated
 * @param cal Calibrator instance
 * @return Current position (or next position if waiting)
 */
accel_cal_position_t accel_cal_get_position(accel_calibrator_t* cal);

/**
 * @brief Get position name string
 * @param pos Position enum
 * @return Human-readable position name
 */
const char* accel_cal_position_name(accel_cal_position_t pos);

/**
 * @brief Feed a new accel sample
 *
 * Call this continuously with IMU data. Calibrator will collect
 * samples when in COLLECTING state.
 *
 * @param cal Calibrator instance
 * @param x Accel X in m/s²
 * @param y Accel Y in m/s²
 * @param z Accel Z in m/s²
 * @param temp_c Temperature in Celsius
 */
void accel_cal_feed_sample(accel_calibrator_t* cal,
                           float x, float y, float z, float temp_c);

/**
 * @brief User signals device is in position, start collecting
 *
 * Call this when user has placed device in the requested position.
 *
 * @param cal Calibrator instance
 * @return true if collection started
 */
bool accel_cal_accept_position(accel_calibrator_t* cal);

/**
 * @brief Get overall progress (0-100%)
 * @param cal Calibrator instance
 * @return Progress percentage
 */
uint8_t accel_cal_get_progress(accel_calibrator_t* cal);

/**
 * @brief Get current position collection progress (0-100%)
 * @param cal Calibrator instance
 * @return Progress percentage
 */
uint8_t accel_cal_get_position_progress(accel_calibrator_t* cal);

/**
 * @brief Get failure reason
 * @param cal Calibrator instance
 * @return Failure reason (NONE if not failed)
 */
accel_cal_fail_t accel_cal_get_fail_reason(accel_calibrator_t* cal);

/**
 * @brief Get calibration result
 *
 * Only valid after state == SUCCESS.
 *
 * @param cal Calibrator instance
 * @param offset Output: offset vector
 * @param scale Output: scale vector
 * @param fitness Output: mean squared residual (lower is better)
 * @return true if result is valid
 */
bool accel_cal_get_result(accel_calibrator_t* cal,
                          cal_vec3_t* offset, cal_vec3_t* scale, float* fitness);

/**
 * @brief Cancel calibration in progress
 * @param cal Calibrator instance
 */
void accel_cal_cancel(accel_calibrator_t* cal);

/**
 * @brief Apply calibration to a raw reading
 * @param params Calibration parameters
 * @param raw Raw acceleration (m/s²)
 * @param calibrated Output calibrated acceleration
 */
void accel_cal_apply(const accel_cal_params_t* params,
                     const cal_vec3_t* raw, cal_vec3_t* calibrated);

#ifdef __cplusplus
}
#endif

#endif // ROCKETCHIP_ACCEL_CALIBRATOR_H
