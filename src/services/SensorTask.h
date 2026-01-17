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
    uint32_t gps_samples;
    uint32_t imu_errors;
    uint32_t baro_errors;
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
    static constexpr UBaseType_t CORE_AFFINITY = (1 << 1);  // Core 1
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

} // namespace services
} // namespace rocketchip

#endif // ROCKETCHIP_SERVICES_SENSOR_TASK_H
