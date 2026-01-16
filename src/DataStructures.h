/**
 * @file DataStructures.h
 * @brief Core data structures for RocketChip production architecture
 *
 * Defines shared data structures for sensor data, fused state, and mission state
 * as specified in SAD Section 4.1.
 *
 * All shared data structures should be protected by FreeRTOS mutexes for
 * thread-safe access across tasks.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 3D Vector structure for acceleration, gyro, magnetometer data
 */
typedef struct {
    float x;
    float y;
    float z;
} Vector3f;

/**
 * @brief GPS data structure
 */
typedef struct {
    bool valid;                 // GPS has valid fix
    double latitude_deg;        // Latitude in degrees
    double longitude_deg;       // Longitude in degrees
    float altitude_msl_m;       // Altitude above mean sea level (meters)
    float ground_speed_mps;     // Ground speed (m/s)
    float course_deg;           // Course over ground (degrees)
    uint8_t satellites;         // Number of satellites in view
    uint32_t timestamp_us;      // Timestamp in microseconds
} GPSData;

/**
 * @brief Shared sensor data (protected by g_sensorDataMutex)
 *
 * This structure contains raw sensor readings from the IMU, magnetometer,
 * barometer, and GPS. It is updated by SensorTask at high rate (1kHz for IMU)
 * and consumed by FusionTask for state estimation.
 *
 * Access pattern:
 * - SensorTask: Write (lock mutex, update, unlock)
 * - FusionTask: Read (lock mutex, copy data, unlock)
 */
typedef struct {
    // IMU (updated at IMU_RATE, e.g., 1kHz)
    Vector3f accel;             // Acceleration in body frame (m/s²)
    Vector3f gyro;              // Gyroscope in body frame (rad/s)
    Vector3f mag;               // Magnetometer (µT)
    uint32_t imu_timestamp_us;  // IMU sample timestamp

    // Barometer (updated at BARO_RATE, e.g., 50Hz)
    float pressure_pa;          // Pressure in Pascals
    float temperature_c;        // Temperature in Celsius
    uint32_t baro_timestamp_us; // Baro sample timestamp

    // GPS (updated at GPS_RATE, e.g., 10Hz)
    GPSData gps;                // GPS data structure
} SensorData;

/**
 * @brief Quaternion structure for attitude representation
 */
typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion;

/**
 * @brief Fused state estimates (protected by g_fusedStateMutex)
 *
 * This structure contains sensor fusion outputs from the EKF including
 * attitude, position, and velocity estimates.
 *
 * PLANNED for Phase 4: Sensor Fusion
 */
typedef struct {
    // Attitude
    Quaternion attitude;        // Quaternion orientation
    Vector3f euler_deg;         // Roll, pitch, yaw (degrees)

    // Position/velocity
    float altitude_agl_m;       // Above ground level (launch point)
    float altitude_msl_m;       // Above mean sea level
    float velocity_z_mps;       // Vertical velocity (+ up)
    float velocity_xy_mps;      // Horizontal velocity magnitude

    // Derived/tracked maximums
    float max_altitude_m;
    float max_velocity_mps;
    float max_accel_g;

    uint32_t timestamp_us;
} FusedState;

/**
 * @brief Mission state (protected by g_missionStateMutex)
 *
 * This structure tracks the current flight state machine state and
 * flight event markers.
 *
 * PLANNED for Phase 5: Mission Engine
 */
typedef struct {
    uint8_t current_state;
    uint8_t previous_state;
    uint32_t state_entry_time_ms;
    uint32_t mission_start_time_ms;

    bool is_armed;
    bool is_logging;
    bool is_transmitting;

    // Flight markers
    uint32_t launch_time_ms;
    uint32_t apogee_time_ms;
    uint32_t landing_time_ms;
    float apogee_altitude_m;
} MissionState;

/**
 * @brief Action message for MissionTask -> ActionExecutor communication
 *
 * PLANNED for Phase 5: Mission Engine
 */
typedef struct {
    uint8_t action_type;        // ACTION_BEEP, ACTION_LED, ACTION_LOG, etc.
    int32_t param1;
    int32_t param2;
    uint32_t timestamp_ms;
} ActionMessage;

/**
 * @brief Log message for All Tasks -> LoggerTask communication
 *
 * PLANNED for Phase 6: Data Logging
 */
typedef struct {
    uint8_t msg_type;           // LOG_SENSOR, LOG_STATE, LOG_EVENT, etc.
    uint32_t timestamp_us;
    uint8_t data[64];           // Payload
    uint8_t data_len;
} LogMessage;

#ifdef __cplusplus
}
#endif
