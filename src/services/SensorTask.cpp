/**
 * @file SensorTask.cpp
 * @brief High-rate sensor sampling task implementation
 *
 * This task runs at 1kHz base rate on Core 1 (real-time core) and samples:
 * - IMU (ISM330DHCX + LIS3MDL) at 1kHz
 * - Barometer (DPS310) at 50Hz (every 20 cycles)
 * - GPS (PA1010D) at 10Hz (every 100 cycles)
 *
 * All sensor data is written to the shared g_sensorData structure with
 * mutex protection for thread-safe access by other tasks.
 */

#include "SensorTask.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>
#include <cmath>

// HAL includes
#include "HAL.h"
#include "IMU_ISM330DHCX.h"
#include "Mag_LIS3MDL.h"
#include "Baro_DPS310.h"
#include "GPS_PA1010D.h"

using namespace rocketchip::hal;

// Global shared sensor data (protected by mutex)
SensorData g_sensorData;
SemaphoreHandle_t g_sensorDataMutex = nullptr;

// Static sensor instances (initialized in SensorTask_Create)
static IMU_ISM330DHCX* s_imu = nullptr;
static Mag_LIS3MDL* s_mag = nullptr;
static Baro_DPS310* s_baro = nullptr;
static GPS_PA1010D* s_gps = nullptr;

// Statistics for debugging
static uint32_t s_sample_count = 0;
static uint32_t s_imu_errors = 0;
static uint32_t s_mag_errors = 0;
static uint32_t s_baro_errors = 0;
static uint32_t s_gps_errors = 0;

/**
 * @brief Initialize sensor hardware
 *
 * @return true if all sensors initialized successfully
 */
static bool initializeSensors() {
    printf("[SensorTask] Initializing sensors...\n");

    // Create I2C bus for sensors (using Feather RP2350 Qwiic pins)
    static I2CBus imu_bus(i2c0, IMU_ISM330DHCX::I2C_ADDR_DEFAULT,
                          FeatherRP2350::I2C_SDA, FeatherRP2350::I2C_SCL,
                          400000); // 400kHz I2C

    static I2CBus mag_bus(i2c0, Mag_LIS3MDL::I2C_ADDR_DEFAULT,
                          FeatherRP2350::I2C_SDA, FeatherRP2350::I2C_SCL,
                          400000);

    static I2CBus baro_bus(i2c0, Baro_DPS310::I2C_ADDR_DEFAULT,
                           FeatherRP2350::I2C_SDA, FeatherRP2350::I2C_SCL,
                           400000);

    static I2CBus gps_bus(i2c0, GPS_PA1010D::I2C_ADDR_DEFAULT,
                          FeatherRP2350::I2C_SDA, FeatherRP2350::I2C_SCL,
                          400000);

    // Initialize IMU (ISM330DHCX)
    s_imu = new IMU_ISM330DHCX(&imu_bus);
    if (!s_imu->begin()) {
        printf("[SensorTask] ERROR: Failed to initialize IMU\n");
        return false;
    }
    s_imu->setAccelRange(AccelRange::RANGE_16G);  // ±16g for rocket flight
    s_imu->setGyroRange(GyroRange::RANGE_2000DPS); // ±2000 dps
    s_imu->setODR(ODR::ODR_1666HZ); // 1666 Hz ODR (task will sample at 1kHz)
    printf("[SensorTask] IMU initialized: ISM330DHCX\n");

    // Initialize Magnetometer (LIS3MDL)
    s_mag = new Mag_LIS3MDL(&mag_bus);
    if (!s_mag->begin()) {
        printf("[SensorTask] ERROR: Failed to initialize Magnetometer\n");
        return false;
    }
    printf("[SensorTask] Magnetometer initialized: LIS3MDL\n");

    // Initialize Barometer (DPS310)
    s_baro = new Baro_DPS310(&baro_bus);
    if (!s_baro->begin()) {
        printf("[SensorTask] ERROR: Failed to initialize Barometer\n");
        return false;
    }
    s_baro->configurePressure(BaroRate::RATE_64HZ, BaroOversample::OSR_16);
    printf("[SensorTask] Barometer initialized: DPS310\n");

    // Initialize GPS (PA1010D) - optional, may not be present
    s_gps = new GPS_PA1010D(&gps_bus);
    if (!s_gps->begin()) {
        printf("[SensorTask] WARNING: GPS not found (optional)\n");
        // Don't fail - GPS is optional
        delete s_gps;
        s_gps = nullptr;
    } else {
        printf("[SensorTask] GPS initialized: PA1010D\n");
    }

    return true;
}

/**
 * @brief Main sensor task function
 *
 * Runs at 1kHz base rate. Samples IMU every cycle, baro every 20 cycles (50Hz),
 * GPS every 100 cycles (10Hz).
 */
void SensorTask_Run(void* pvParameters) {
    (void)pvParameters;

    printf("[SensorTask] Starting on Core %d\n", get_core_num());

    // Initialize sensors
    if (!initializeSensors()) {
        printf("[SensorTask] FATAL: Sensor initialization failed\n");
        vTaskDelete(nullptr);
        return;
    }

    printf("[SensorTask] Running at 1kHz\n");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1); // 1ms = 1kHz

    uint32_t cycle_count = 0;
    uint32_t last_status_print = 0;

    while (1) {
        cycle_count++;

        // Get current timestamp in microseconds
        uint32_t timestamp_us = to_us_since_boot(get_absolute_time());

        // Read IMU every cycle (1kHz)
        Vector3f accel, gyro;
        if (s_imu->read(accel, gyro)) {
            // Convert HAL Vector3f to DataStructures Vector3f
            Vector3f mag_reading;
            if (s_mag && s_mag->read(mag_reading)) {
                // Lock mutex and update shared data
                if (xSemaphoreTake(g_sensorDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    // Update IMU data (convert from g to m/s² for accel)
                    g_sensorData.accel.x = accel.x * 9.80665f;
                    g_sensorData.accel.y = accel.y * 9.80665f;
                    g_sensorData.accel.z = accel.z * 9.80665f;

                    // Gyro is already in rad/s from driver (actually dps, convert)
                    const float deg_to_rad = 0.017453292519943295f; // π/180
                    g_sensorData.gyro.x = gyro.x * deg_to_rad;
                    g_sensorData.gyro.y = gyro.y * deg_to_rad;
                    g_sensorData.gyro.z = gyro.z * deg_to_rad;

                    // Magnetometer (µT)
                    g_sensorData.mag.x = mag_reading.x;
                    g_sensorData.mag.y = mag_reading.y;
                    g_sensorData.mag.z = mag_reading.z;

                    g_sensorData.imu_timestamp_us = timestamp_us;

                    xSemaphoreGive(g_sensorDataMutex);
                }
            } else {
                s_mag_errors++;
            }
        } else {
            s_imu_errors++;
        }

        // Read Barometer every 20 cycles (50Hz)
        if ((cycle_count % 20) == 0) {
            float pressure_pa, temp_c;
            if (s_baro && s_baro->read(pressure_pa, temp_c)) {
                if (xSemaphoreTake(g_sensorDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    g_sensorData.pressure_pa = pressure_pa;
                    g_sensorData.temperature_c = temp_c;
                    g_sensorData.baro_timestamp_us = timestamp_us;
                    xSemaphoreGive(g_sensorDataMutex);
                }
            } else {
                s_baro_errors++;
            }
        }

        // Read GPS every 100 cycles (10Hz)
        if (s_gps && (cycle_count % 100) == 0) {
            GPSData gps_data_temp;
            if (s_gps->read(gps_data_temp)) {
                if (xSemaphoreTake(g_sensorDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    // Copy GPS data to shared structure
                    memcpy(&g_sensorData.gps, &gps_data_temp, sizeof(GPSData));
                    xSemaphoreGive(g_sensorDataMutex);
                }
            } else {
                s_gps_errors++;
            }
        }

        // Print status every 5 seconds (5000 cycles at 1kHz)
        if ((cycle_count - last_status_print) >= 5000) {
            last_status_print = cycle_count;

            // Lock mutex and read current data
            if (xSemaphoreTake(g_sensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                printf("\n[SensorTask] Status @ %lu samples:\n", cycle_count);
                printf("  IMU:   accel(%.2f, %.2f, %.2f) m/s²  gyro(%.2f, %.2f, %.2f) rad/s\n",
                       g_sensorData.accel.x, g_sensorData.accel.y, g_sensorData.accel.z,
                       g_sensorData.gyro.x, g_sensorData.gyro.y, g_sensorData.gyro.z);
                printf("  Mag:   (%.2f, %.2f, %.2f) µT\n",
                       g_sensorData.mag.x, g_sensorData.mag.y, g_sensorData.mag.z);
                printf("  Baro:  %.2f Pa, %.2f °C\n",
                       g_sensorData.pressure_pa, g_sensorData.temperature_c);

                if (s_gps && g_sensorData.gps.valid) {
                    printf("  GPS:   lat=%.6f lon=%.6f alt=%.1f m, sats=%d\n",
                           g_sensorData.gps.latitude_deg, g_sensorData.gps.longitude_deg,
                           g_sensorData.gps.altitude_msl_m, g_sensorData.gps.satellites);
                } else {
                    printf("  GPS:   No fix\n");
                }

                printf("  Errors: IMU=%lu Mag=%lu Baro=%lu GPS=%lu\n",
                       s_imu_errors, s_mag_errors, s_baro_errors, s_gps_errors);
                printf("  Heap:   %lu bytes free\n\n", xPortGetFreeHeapSize());

                xSemaphoreGive(g_sensorDataMutex);
            }
        }

        // Wait for next cycle
        vTaskDelayUntil(&last_wake_time, period);
    }
}

/**
 * @brief Create and start the sensor task
 */
TaskHandle_t SensorTask_Create(void) {
    // Create mutex for shared sensor data
    g_sensorDataMutex = xSemaphoreCreateMutex();
    if (g_sensorDataMutex == nullptr) {
        printf("[SensorTask] ERROR: Failed to create mutex\n");
        return nullptr;
    }

    // Initialize sensor data to zero
    memset(&g_sensorData, 0, sizeof(SensorData));

    // Create task
    TaskHandle_t task_handle = nullptr;
    BaseType_t result = xTaskCreate(
        SensorTask_Run,
        "Sensor",
        SENSOR_TASK_STACK_SIZE,
        nullptr,
        SENSOR_TASK_PRIORITY,
        &task_handle
    );

    if (result != pdPASS) {
        printf("[SensorTask] ERROR: Failed to create task\n");
        vSemaphoreDelete(g_sensorDataMutex);
        g_sensorDataMutex = nullptr;
        return nullptr;
    }

    // Set core affinity to Core 1 (real-time core)
    vTaskCoreAffinitySet(task_handle, (1 << SENSOR_TASK_CORE));

    printf("[SensorTask] Task created successfully\n");

    return task_handle;
}
