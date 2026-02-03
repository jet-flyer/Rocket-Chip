/**
 * @file sensor_task.c
 * @brief Sensor sampling and calibration task implementation
 *
 * High-priority task (Core 0) that:
 * - Samples IMU at 100Hz (10ms period)
 * - Samples barometer at 50Hz (20ms period)
 * - Applies calibration corrections
 * - Feeds samples to active calibration routines
 */

#include "sensor_task.h"
#include "calibration_manager.h"
#include "calibration_storage.h"
#include "i2c_bus.h"
#include "icm20948.h"
#include "baro_dps310.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "pico/time.h"
#include <string.h>
#include <stdio.h>
#include "debug/debug_stream.h"

// ============================================================================
// Configuration (matches config.h but C-compatible)
// ============================================================================

#define SENSOR_TASK_PRIORITY    5       // kPrioritySensor
#define SENSOR_TASK_STACK       512     // kStackSensor (words) - increased for ellipsoid fit
// CRITICAL: Core 1 for sensors - Core 0 hosts USB IRQ handlers
// USB low_priority_worker_irq preempts I2C if both on Core 0, causing deadlock
#define SENSOR_CORE             1       // Core 1 = sensor I/O, Core 0 = USB

#define SENSOR_TASK_PERIOD_MS   10      // 100Hz main loop
#define BARO_SAMPLE_DIVIDER     2       // Sample baro every 2nd loop = 50Hz

// ============================================================================
// Private State
// ============================================================================

static TaskHandle_t g_taskHandle = NULL;
static SemaphoreHandle_t g_dataMutex = NULL;

// ICM-20948 device handle
static icm20948_t g_imu;

// Latest sensor data (protected by mutex)
static imu_data_t g_imuData;
static baro_data_t g_baroData;
static sensor_status_t g_status;

// Sample counters
static uint32_t g_loopCount = 0;

// ============================================================================
// Private Functions
// ============================================================================

/**
 * @brief Read and process IMU data
 */
static void process_imu(void) {
    icm20948_data_t raw;

    if (!icm20948_read(&g_imu, &raw)) {
        return;
    }

    // Get calibration state
    cal_state_t calState = calibration_manager_get_state();

    // Feed to calibration if active
    if (calState == CAL_STATE_GYRO_SAMPLING) {
        calibration_feed_gyro(raw.gyro.x, raw.gyro.y, raw.gyro.z, raw.temperature_c);
    }
    else if (calState == CAL_STATE_ACCEL_LEVEL_SAMPLING ||
             calState == CAL_STATE_ACCEL_6POS) {
        calibration_feed_accel(raw.accel.x, raw.accel.y, raw.accel.z, raw.temperature_c);
    }

    // Apply calibration to get corrected values
    float ax_cal, ay_cal, az_cal;
    float gx_cal, gy_cal, gz_cal;

    calibration_apply_accel(raw.accel.x, raw.accel.y, raw.accel.z,
                            &ax_cal, &ay_cal, &az_cal);
    calibration_apply_gyro(raw.gyro.x, raw.gyro.y, raw.gyro.z,
                           &gx_cal, &gy_cal, &gz_cal);

    // Update shared data (with mutex)
    if (xSemaphoreTake(g_dataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        g_imuData.accel_x = ax_cal;
        g_imuData.accel_y = ay_cal;
        g_imuData.accel_z = az_cal;
        g_imuData.gyro_x = gx_cal;
        g_imuData.gyro_y = gy_cal;
        g_imuData.gyro_z = gz_cal;
        g_imuData.mag_x = raw.mag.x;
        g_imuData.mag_y = raw.mag.y;
        g_imuData.mag_z = raw.mag.z;
        g_imuData.temperature_c = raw.temperature_c;
        g_imuData.timestamp_us = time_us_32();

        g_status.imu_sample_count++;
        xSemaphoreGive(g_dataMutex);
    }
}

/**
 * @brief Read and process barometer data
 */
static void process_baro(void) {
    baro_dps310_data_t raw;

    if (!baro_dps310_read(&raw)) {
        return;
    }

    // Feed to calibration if active
    cal_state_t calState = calibration_manager_get_state();
    if (calState == CAL_STATE_BARO_SAMPLING) {
        calibration_feed_baro(raw.pressure_pa, raw.temperature_c);
    }

    // Compute altitude using calibration ground reference
    float altitude_agl = calibration_get_altitude_agl(raw.pressure_pa);

    // Update shared data (with mutex)
    if (xSemaphoreTake(g_dataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        g_baroData.pressure_pa = raw.pressure_pa;
        g_baroData.temperature_c = raw.temperature_c;
        g_baroData.altitude_m = altitude_agl;
        g_baroData.timestamp_us = time_us_32();

        g_status.baro_sample_count++;
        xSemaphoreGive(g_dataMutex);
    }
}

/**
 * @brief Main sensor task loop
 */
static void sensor_task_main(void* pvParameters) {
    (void)pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        // Always sample IMU (100Hz)
        if (g_status.imu_ready) {
            process_imu();
        }

        // Sample baro at reduced rate (50Hz)
        if (g_status.baro_ready && (g_loopCount % BARO_SAMPLE_DIVIDER == 0)) {
            process_baro();
        }

        g_loopCount++;

        // Maintain precise timing
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
    }
}

// ============================================================================
// Public API - Initialization
// ============================================================================

bool sensor_task_init(void) {
    // Clear state
    memset(&g_imuData, 0, sizeof(g_imuData));
    memset(&g_baroData, 0, sizeof(g_baroData));
    memset(&g_status, 0, sizeof(g_status));
    memset(&g_imu, 0, sizeof(g_imu));

    // Create mutex for data access
    g_dataMutex = xSemaphoreCreateMutex();
    if (g_dataMutex == NULL) {
        return false;
    }

    // Initialize I2C bus
    if (!i2c_bus_init()) {
        return false;
    }

    // Initialize calibration storage (loads from flash)
    calibration_storage_init();

    // Initialize calibration manager (loads or creates defaults)
    calibration_manager_init();

    // Check calibration status
    const calibration_store_t* cal = calibration_manager_get();
    g_status.calibration_valid = calibration_validate(cal);

    // Initialize IMU
    if (icm20948_init(&g_imu, ICM20948_ADDR_DEFAULT)) {
        g_status.imu_ready = true;
    }

    // Initialize barometer
    if (baro_dps310_init(BARO_DPS310_ADDR_DEFAULT)) {
        // Start continuous measurement
        if (baro_dps310_start_continuous()) {
            g_status.baro_ready = true;
        }
    }

    // At least one sensor must be ready
    return g_status.imu_ready || g_status.baro_ready;
}

bool sensor_task_create(void) {
    BaseType_t result = xTaskCreate(
        sensor_task_main,
        "Sensor",
        SENSOR_TASK_STACK,
        NULL,
        SENSOR_TASK_PRIORITY,
        &g_taskHandle
    );

    if (result != pdPASS) {
        return false;
    }

    // Pin to Core 0 (real-time core)
    vTaskCoreAffinitySet(g_taskHandle, (1 << SENSOR_CORE));

    return true;
}

void sensor_task_get_status(sensor_status_t* status) {
    if (status == NULL) return;

    if (xSemaphoreTake(g_dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        *status = g_status;
        xSemaphoreGive(g_dataMutex);
    }
}

// ============================================================================
// Public API - Data Access
// ============================================================================

bool sensor_task_get_imu(imu_data_t* data) {
    if (data == NULL || !g_status.imu_ready) {
        return false;
    }

    if (xSemaphoreTake(g_dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        *data = g_imuData;
        xSemaphoreGive(g_dataMutex);
        return true;
    }

    return false;
}

bool sensor_task_get_baro(baro_data_t* data) {
    if (data == NULL || !g_status.baro_ready) {
        return false;
    }

    if (xSemaphoreTake(g_dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        *data = g_baroData;
        xSemaphoreGive(g_dataMutex);
        return true;
    }

    return false;
}

// ============================================================================
// Public API - Calibration Control
// ============================================================================

bool sensor_task_start_gyro_cal(void) {
    return calibration_start_gyro() == CAL_RESULT_OK;
}

bool sensor_task_start_accel_level_cal(void) {
    return calibration_start_accel_level() == CAL_RESULT_OK;
}

bool sensor_task_start_baro_cal(void) {
    return calibration_start_baro() == CAL_RESULT_OK;
}

uint8_t sensor_task_get_cal_progress(void) {
    return calibration_get_progress();
}

bool sensor_task_is_calibrating(void) {
    return calibration_is_active();
}

bool sensor_task_save_calibration(void) {
    cal_result_t result = calibration_save();
    if (result == CAL_RESULT_OK) {
        g_status.calibration_valid = true;
        return true;
    }
    return false;
}

bool sensor_task_reset_calibration(void) {
    cal_result_t result = calibration_reset();
    if (result == CAL_RESULT_OK) {
        // Re-check validity
        const calibration_store_t* cal = calibration_manager_get();
        g_status.calibration_valid = calibration_validate(cal);
        return true;
    }
    return false;
}

// ============================================================================
// Public API - 6-Position Accelerometer Calibration
// ============================================================================

bool sensor_task_start_accel_6pos_cal(void) {
    return calibration_start_accel_6pos() == CAL_RESULT_OK;
}

int8_t sensor_task_get_6pos_position(void) {
    return calibration_get_6pos_position();
}

const char* sensor_task_get_6pos_position_name(void) {
    return calibration_get_6pos_position_name();
}

bool sensor_task_accept_6pos_position(void) {
    return calibration_accept_6pos_position();
}

uint8_t sensor_task_get_6pos_position_progress(void) {
    return calibration_get_6pos_position_progress();
}
