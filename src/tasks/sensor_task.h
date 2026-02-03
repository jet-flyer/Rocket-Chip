/**
 * @file sensor_task.h
 * @brief Sensor sampling and calibration task
 *
 * High-priority FreeRTOS task that:
 * - Initializes all sensors (IMU, baro, GPS)
 * - Samples sensors at appropriate rates
 * - Applies calibration corrections
 * - Feeds samples to calibration routines when active
 * - Provides calibrated data to other tasks via getter functions
 */

#ifndef ROCKETCHIP_SENSOR_TASK_H
#define ROCKETCHIP_SENSOR_TASK_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Sensor Data Structures
// ============================================================================

/**
 * @brief Raw IMU data from ICM-20948
 */
typedef struct {
    float accel_x, accel_y, accel_z;    // m/s²
    float gyro_x, gyro_y, gyro_z;       // rad/s
    float mag_x, mag_y, mag_z;          // µT
    float temperature_c;
    uint32_t timestamp_us;
} imu_data_t;

/**
 * @brief Barometer data from DPS310
 */
typedef struct {
    float pressure_pa;
    float temperature_c;
    float altitude_m;                    // Computed from calibration
    uint32_t timestamp_us;
} baro_data_t;

/**
 * @brief Sensor status flags
 */
typedef struct {
    bool imu_ready;
    bool baro_ready;
    bool gps_ready;
    bool calibration_valid;
    uint32_t imu_sample_count;
    uint32_t baro_sample_count;
} sensor_status_t;

// ============================================================================
// Task Control
// ============================================================================

/**
 * @brief Initialize the sensor task
 *
 * Must be called before creating the FreeRTOS task.
 * Initializes I2C bus and sensor drivers.
 *
 * @return true if all required sensors initialized
 */
bool sensor_task_init(void);

/**
 * @brief Create and start the sensor FreeRTOS task
 *
 * Creates a high-priority task pinned to Core 0.
 * Sensor sampling begins immediately.
 *
 * @return true if task created successfully
 */
bool sensor_task_create(void);

/**
 * @brief Get current sensor status
 * @param status Output structure for status
 */
void sensor_task_get_status(sensor_status_t* status);

// ============================================================================
// Data Access (Thread-safe)
// ============================================================================

/**
 * @brief Get latest calibrated IMU data
 *
 * Returns the most recent IMU reading with calibration applied.
 * Thread-safe: uses critical section for atomic copy.
 *
 * @param data Output structure for IMU data
 * @return true if valid data available
 */
bool sensor_task_get_imu(imu_data_t* data);

/**
 * @brief Get latest barometer data
 *
 * Returns the most recent baro reading with calibration applied.
 * Altitude is relative to ground reference if calibrated.
 *
 * @param data Output structure for baro data
 * @return true if valid data available
 */
bool sensor_task_get_baro(baro_data_t* data);

// ============================================================================
// Calibration Control
// ============================================================================

/**
 * @brief Start gyroscope calibration
 *
 * Device must be stationary during calibration.
 * Takes ~2 seconds. Check progress with sensor_task_get_cal_progress().
 *
 * @return true if calibration started
 */
bool sensor_task_start_gyro_cal(void);

/**
 * @brief Start accelerometer level calibration
 *
 * Device must be flat and stationary during calibration.
 * Takes ~1 second.
 *
 * @return true if calibration started
 */
bool sensor_task_start_accel_level_cal(void);

/**
 * @brief Start barometer ground calibration
 *
 * Sets current pressure as ground reference.
 * Takes ~1 second.
 *
 * @return true if calibration started
 */
bool sensor_task_start_baro_cal(void);

// ============================================================================
// 6-Position Accelerometer Calibration
// ============================================================================

/**
 * @brief Start 6-position accelerometer calibration
 *
 * Full calibration computing offset and scale for all axes.
 * User must place device in 6 orientations when prompted.
 * Positions: LEVEL, LEFT, RIGHT, NOSE_DOWN, NOSE_UP, BACK
 *
 * Use sensor_task_get_6pos_position() to get current required position.
 * Use sensor_task_accept_6pos_position() when device is in position.
 *
 * @return true if calibration started
 */
bool sensor_task_start_accel_6pos_cal(void);

/**
 * @brief Get current 6-position calibration position index
 * @return Position index (0-5), or -1 if not in 6-pos calibration
 */
int8_t sensor_task_get_6pos_position(void);

/**
 * @brief Get name of current 6-position calibration position
 * @return Position name string (e.g., "LEVEL (flat)"), or NULL if not calibrating
 */
const char* sensor_task_get_6pos_position_name(void);

/**
 * @brief Signal that device is in position, start collecting samples
 *
 * Call when user has placed device in the requested orientation.
 * Will collect samples for ~1 second, then advance to next position.
 *
 * @return true if collection started
 */
bool sensor_task_accept_6pos_position(void);

/**
 * @brief Get progress of current position sample collection
 * @return Progress 0-100% for current position, 0 if not collecting
 */
uint8_t sensor_task_get_6pos_position_progress(void);

// ============================================================================
// Calibration Status
// ============================================================================

/**
 * @brief Get calibration progress
 * @return Progress 0-100%, or 0 if not calibrating
 */
uint8_t sensor_task_get_cal_progress(void);

/**
 * @brief Check if calibration is in progress
 * @return true if any calibration is active
 */
bool sensor_task_is_calibrating(void);

/**
 * @brief Save current calibration to flash
 * @return true on success
 */
bool sensor_task_save_calibration(void);

/**
 * @brief Reset calibration to defaults
 * @return true on success
 */
bool sensor_task_reset_calibration(void);

#ifdef __cplusplus
}
#endif

#endif // ROCKETCHIP_SENSOR_TASK_H
