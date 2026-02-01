/**
 * @file SensorTask.h
 * @brief High-rate sensor sampling FreeRTOS task
 *
 * Samples IMU at 1kHz, barometer at 50Hz, GPS at 10Hz.
 * Updates shared SensorData structure protected by mutex.
 *
 * @note Part of RocketChip Services Layer
 * @see docs/SAD.md Section 4.1 for data structure definitions
 * @see docs/SAD.md Section 5.1 for task architecture
 */

#ifndef ROCKETCHIP_SERVICES_SENSOR_TASK_H
#define ROCKETCHIP_SERVICES_SENSOR_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <cstdint>

// For MAV_RESULT enum (used by calibration API)
#include <GCS_MAVLink/GCS.h>

// For accel_cal_status_t enum
#include <AP_AccelCal/AccelCalibrator.h>

namespace rocketchip {
namespace services {

/**
 * @brief 3D vector for sensor data (matches HAL Vector3f)
 */
struct Vector3f {
    float x;
    float y;
    float z;

    Vector3f() : x(0.0f), y(0.0f), z(0.0f) {}
    Vector3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

/**
 * @brief Shared sensor data structure (SAD Section 4.1)
 *
 * Protected by g_sensorDataMutex. All tasks reading sensor data
 * should acquire the mutex first.
 */
struct SensorData {
    // IMU (updated at IMU_RATE, e.g., 1kHz)
    Vector3f accel;           // m/s^2 in body frame
    Vector3f gyro;            // rad/s in body frame
    Vector3f mag;             // gauss (if available)
    uint32_t imu_timestamp_us;

    // Barometer (updated at BARO_RATE, e.g., 50Hz)
    float pressure_pa;
    float temperature_c;
    uint32_t baro_timestamp_us;

    // GPS (updated at GPS_RATE, e.g., 10Hz)
    bool gps_valid;
    double latitude_deg;
    double longitude_deg;
    float altitude_msl_m;
    float ground_speed_mps;
    float course_deg;
    uint8_t satellites;
    uint32_t gps_timestamp_us;
};

/**
 * @brief SensorTask statistics for monitoring
 */
struct SensorTaskStats {
    uint32_t imu_samples;
    uint32_t baro_samples;
    uint32_t mag_samples;
    uint32_t gps_samples;
    uint32_t imu_errors;
    uint32_t baro_errors;
    uint32_t mag_errors;
    uint32_t gps_errors;
    uint32_t loop_overruns;
    uint32_t free_heap;
};

/**
 * @brief SensorTask configuration
 */
struct SensorTaskConfig {
    static constexpr uint32_t IMU_RATE_HZ = 1000;    // 1kHz IMU sampling
    static constexpr uint32_t BARO_RATE_HZ = 50;     // 50Hz barometer
    static constexpr uint32_t GPS_RATE_HZ = 10;      // 10Hz GPS
    static constexpr uint32_t TASK_PRIORITY = 5;     // Highest priority
    static constexpr uint32_t STACK_SIZE = 1024;     // 1KB stack
    // Core 0 for SensorTask - Core 1 is used by TinyUSB (per PD3 in RP2350_FULL_AP_PORT.md)
    static constexpr UBaseType_t CORE_AFFINITY = (1 << 0);  // Core 0
};

// Global shared data (extern declarations)
extern SensorData g_sensorData;
extern SemaphoreHandle_t g_sensorDataMutex;
extern SensorTaskStats g_sensorTaskStats;

/**
 * @brief Initialize sensor hardware and task resources
 *
 * Must be called before SensorTask_Create().
 * Initializes I2C buses, sensors, and mutex.
 *
 * @return true if all sensors initialized successfully
 */
bool SensorTask_Init();

/**
 * @brief Create and start the SensorTask
 *
 * Creates the FreeRTOS task pinned to Core 1 with priority 5.
 *
 * @return true if task created successfully
 */
bool SensorTask_Create();

/**
 * @brief Get copy of current sensor data (thread-safe)
 *
 * @param data Output: copy of current sensor data
 * @return true if data was copied successfully
 */
bool SensorTask_GetData(SensorData& data);

/**
 * @brief Get current task statistics
 *
 * @return Copy of current statistics
 */
SensorTaskStats SensorTask_GetStats();

/**
 * @brief Print sensor status to serial (for debug key commands)
 */
void SensorTask_PrintStatus();

// ============================================================================
// Calibration API
// ============================================================================

/**
 * @brief Start accelerometer 6-position calibration
 *
 * Initiates the ArduPilot AccelCalibrator. User must place device
 * in 6 orientations when prompted. Results saved to AP_Param.
 *
 * @return true if calibration started successfully
 */
bool SensorTask_StartAccelCal();

/**
 * @brief Start simple 1D accelerometer calibration
 *
 * Quick level calibration - device must be level when called.
 * Results saved to AP_Param.
 *
 * @return MAV_RESULT indicating success/failure
 */
MAV_RESULT SensorTask_SimpleAccelCal();

/**
 * @brief Confirm vehicle position for accel calibration
 *
 * Called when GCS (or user) confirms device is in requested position.
 *
 * @param position ACCELCAL_VEHICLE_POS enum value
 * @return true if position acknowledged
 */
bool SensorTask_AccelCalConfirmPosition(int position);

/**
 * @brief Start compass calibration
 *
 * Initiates the ArduPilot CompassCalibrator. User must rotate
 * device through all orientations. Results saved to AP_Param.
 *
 * @return true if calibration started successfully
 */
bool SensorTask_StartCompassCal();

/**
 * @brief Calibrate barometer (set ground pressure reference)
 *
 * @return true if calibration successful
 */
bool SensorTask_CalibrateBaro();

/**
 * @brief Check if any calibration is in progress
 *
 * @return true if accel or compass calibration is running
 */
bool SensorTask_IsCalibrating();

/**
 * @brief Cancel any running calibration
 */
void SensorTask_CancelCalibration();

/**
 * @brief Check if sensor initialization is complete
 *
 * CLI waits for this before showing user prompt to avoid
 * interleaved output with sensor init debug messages.
 *
 * @return true if sensor init has completed (success or failure)
 */
bool SensorTask_IsInitComplete();

/**
 * @brief Get current accel calibration step
 *
 * Returns the position AP_AccelCal is waiting for confirmation on.
 * Only valid when accel calibration is running.
 *
 * @return Current step (1-6) or 0 if not calibrating
 *         1=LEVEL, 2=LEFT, 3=RIGHT, 4=NOSEDOWN, 5=NOSEUP, 6=BACK
 */
uint8_t SensorTask_GetAccelCalStep();

/**
 * @brief Get current accel calibration state
 *
 * Returns the AP_AccelCal state machine status.
 *
 * @return ACCEL_CAL_NOT_STARTED, ACCEL_CAL_WAITING_FOR_ORIENTATION,
 *         ACCEL_CAL_COLLECTING_SAMPLE, ACCEL_CAL_SUCCESS, or ACCEL_CAL_FAILED
 */
accel_cal_status_t SensorTask_GetAccelCalStatus();

/**
 * @brief Get human-readable name for accel calibration position
 *
 * @param step Position number (1-6)
 * @return Position name string (e.g., "LEVEL", "LEFT SIDE")
 */
const char* SensorTask_GetAccelCalPositionName(uint8_t step);

} // namespace services
} // namespace rocketchip

#endif // ROCKETCHIP_SERVICES_SENSOR_TASK_H
