/**
 * @file SensorTask.h
 * @brief High-rate sensor sampling task for RocketChip
 *
 * SensorTask is responsible for:
 * - Reading IMU data at 1kHz (ISM330DHCX accel + gyro, LIS3MDL mag)
 * - Reading barometer data at 50Hz (DPS310)
 * - Reading GPS data at 10Hz (PA1010D)
 * - Updating shared SensorData structure with mutex protection
 *
 * Architecture:
 * - Priority: 5 (highest)
 * - Core affinity: Core 1 (real-time core)
 * - Rate: 1kHz base rate
 * - Stack: 2KB
 *
 * Phase: 2 (Sensors)
 * Status: Production implementation (replaces validation scaffolding)
 */

#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "../DataStructures.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Task configuration
 */
#define SENSOR_TASK_PRIORITY        5
#define SENSOR_TASK_STACK_SIZE      (2048 / sizeof(StackType_t))
#define SENSOR_TASK_CORE            1

/**
 * @brief Sampling rates
 */
#define SENSOR_IMU_RATE_HZ          1000    // 1kHz for IMU
#define SENSOR_BARO_RATE_HZ         50      // 50Hz for barometer
#define SENSOR_GPS_RATE_HZ          10      // 10Hz for GPS

/**
 * @brief Shared sensor data (protected by g_sensorDataMutex)
 */
extern SensorData g_sensorData;
extern SemaphoreHandle_t g_sensorDataMutex;

/**
 * @brief Create and start the sensor task
 *
 * This function creates the FreeRTOS task and sets its core affinity.
 * Must be called after HAL initialization.
 *
 * @return TaskHandle_t Handle to the created task, or NULL on failure
 */
TaskHandle_t SensorTask_Create(void);

/**
 * @brief Sensor task entry point
 *
 * This is the main task function that runs at 1kHz and samples sensors.
 *
 * @param pvParameters Task parameters (unused)
 */
void SensorTask_Run(void* pvParameters);

#ifdef __cplusplus
}
#endif
