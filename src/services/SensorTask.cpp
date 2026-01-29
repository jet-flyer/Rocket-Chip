/**
 * @file SensorTask.cpp
 * @brief High-rate sensor sampling FreeRTOS task implementation
 *
 * Uses ArduPilot's native AP_InertialSensor and Compass for sensor access.
 * Samples sensors at configured rates and updates shared SensorData.
 */

#include "SensorTask.h"
#include "HAL.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/timer.h"

#include "debug.h"

// ArduPilot HAL and sensor libraries
#include <AP_HAL_RP2350/HAL_RP2350_Class.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Param/AP_Param.h>

// RocketChip calibration storage (for barometer only now)
#include "calibration/CalibrationStore.h"
#include <AP_HAL_RP2350/hwdef.h>  // For GRAVITY_MSS, DEG_TO_RAD

#include <cstdio>
#include <cmath>

using namespace rocketchip::hal;

// ArduPilot HAL reference
extern const AP_HAL::HAL& hal;

namespace rocketchip {
namespace services {

// ============================================================================
// Global Data
// ============================================================================

SensorData g_sensorData = {};
SemaphoreHandle_t g_sensorDataMutex = nullptr;
SensorTaskStats g_sensorTaskStats = {};

// ============================================================================
// ArduPilot Sensor Instances (pointers - initialized AFTER HAL init)
// Constructors access AP_Param which needs HAL, so can't be static globals
// ============================================================================

static AP_InertialSensor* g_ins = nullptr;
static Compass* g_compass = nullptr;
static AP_Baro* g_baro = nullptr;

// ============================================================================
// Private Data
// ============================================================================

static TaskHandle_t s_taskHandle = nullptr;

// Legacy calibration for barometer (IMU/Compass use AP_Param)
static CalibrationStore s_calibrationStore;
static SensorCalibration s_calibration;
static bool s_calibrationLoaded = false;

// Barometer uses ArduPilot AP_Baro (initialized after HAL)

// Initialization flags
static bool s_imuInitialized = false;
static bool s_magInitialized = false;
static bool s_baroInitialized = false;

// Timing dividers (all relative to 1kHz base rate)
static constexpr uint32_t kBaroDivider = SensorTaskConfig::IMU_RATE_HZ / SensorTaskConfig::BARO_RATE_HZ;  // 20
static constexpr uint32_t kMagDivider = 10;  // 100Hz magnetometer

// ============================================================================
// Sensor Initialization
// ============================================================================

// Diagnostic blink helper (1 blink = checkpoint passed)
static void diag_blink(int count) {
    for (int i = 0; i < count; i++) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        busy_wait_ms(150);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        busy_wait_ms(150);
    }
    busy_wait_ms(300);  // Pause between checkpoints
}

static bool initSensors() {
    DBG_PRINT("[SensorTask] Initializing sensors via ArduPilot...\n");

    diag_blink(1);  // Checkpoint: entered initSensors

    // Initialize ArduPilot HAL (I2C, SPI, etc.)
    DBG_PRINT("  HAL init:         ");
    RP2350::hal_init();
    DBG_PRINT("OK\n");

    diag_blink(1);  // Checkpoint: hal_init done

    // Initialize AP_Param for calibration persistence
    DBG_PRINT("  AP_Param:         ");
    AP_Param::setup();
    DBG_PRINT("OK\n");

    diag_blink(1);  // Checkpoint: AP_Param done

    // Create sensor instances (must be AFTER HAL init - constructors access AP_Param)
    g_ins = new AP_InertialSensor();

    diag_blink(1);  // Checkpoint: AP_InertialSensor created

    g_compass = new Compass();

    diag_blink(1);  // Checkpoint: Compass created

    // Initialize AP_InertialSensor (probes ICM-20948 via HAL_INS_PROBE_LIST)
    // NOTE: This now runs INSIDE a FreeRTOS task, so scheduler timer/IO tasks exist
    DBG_PRINT("  AP_InertialSensor: ");
    g_ins->init(100);  // 100Hz main loop rate
    s_imuInitialized = (g_ins->get_accel_count() > 0 && g_ins->get_gyro_count() > 0);
    if (s_imuInitialized) {
        DBG_PRINT("OK (accel=%u, gyro=%u)\n", g_ins->get_accel_count(), g_ins->get_gyro_count());
    } else {
        DBG_ERROR("FAILED (accel=%u, gyro=%u)\n", g_ins->get_accel_count(), g_ins->get_gyro_count());
    }

    diag_blink(1);  // Checkpoint: g_ins->init() done

    // Initialize Compass (probes AK09916 via HAL_MAG_PROBE_LIST)
    DBG_PRINT("  AP_Compass:        ");
    g_compass->init();
    s_magInitialized = (g_compass->get_count() > 0);
    if (s_magInitialized) {
        DBG_PRINT("OK (count=%u)\n", g_compass->get_count());
    } else {
        DBG_PRINT("N/A (count=%u)\n", g_compass->get_count());
    }

    diag_blink(1);  // Checkpoint: g_compass->init() done
    if (s_magInitialized) {
        DBG_PRINT("OK (count=%u)\n", g_compass->get_count());
    } else {
        DBG_PRINT("N/A (count=%u)\n", g_compass->get_count());
    }

    // Initialize Barometer via ArduPilot (probes DPS310 via HAL_BARO_PROBE_LIST)
    DBG_PRINT("  AP_Baro:           ");
    g_baro = new AP_Baro();
    g_baro->init();
    s_baroInitialized = (g_baro->num_instances() > 0);

    diag_blink(1);  // Checkpoint: baro init done

    if (s_baroInitialized) {
        // Calibrate barometer (sets ground pressure reference)
        g_baro->calibrate(false);  // false = don't save to storage yet
        DBG_PRINT("OK (count=%u)\n", g_baro->num_instances());
    } else {
        DBG_ERROR("FAILED (count=%u)\n", g_baro->num_instances());
    }

    // Load legacy calibration from flash (for barometer, etc.)
    // IMU/Compass calibration now uses AP_Param automatically
    DBG_PRINT("  Legacy cal:        ");
    s_calibrationStore.init();
    if (s_calibrationStore.load(s_calibration)) {
        s_calibrationLoaded = true;
        DBG_PRINT("Loaded (flags=0x%02X)\n", s_calibration.flags);
    } else {
        s_calibrationLoaded = false;
        CalibrationStore::get_defaults(s_calibration);
        DBG_PRINT("None (defaults)\n");
    }

    // At least IMU must be working for flight
    return s_imuInitialized;
}

// ============================================================================
// Sensor Reading Functions
// ============================================================================

static void readIMU() {
    // ArduPilot's INS handles blocking wait for sample internally
    // For FreeRTOS task, we use update() which is non-blocking
    g_ins->update();

    // Get calibrated sensor data (calibration applied by ArduPilot via AP_Param)
    // Use global namespace :: to get ArduPilot's Vector3f, not rocketchip::services::Vector3f
    const ::Vector3f& accel = g_ins->get_accel(0);
    const ::Vector3f& gyro = g_ins->get_gyro(0);

    // Take mutex and update shared data
    if (xSemaphoreTake(g_sensorDataMutex, 0) == pdTRUE) {
        // ArduPilot returns accel in m/s^2, gyro in rad/s (already calibrated)
        g_sensorData.accel.x = accel.x;
        g_sensorData.accel.y = accel.y;
        g_sensorData.accel.z = accel.z;

        g_sensorData.gyro.x = gyro.x;
        g_sensorData.gyro.y = gyro.y;
        g_sensorData.gyro.z = gyro.z;

        g_sensorData.imu_timestamp_us = time_us_32();
        xSemaphoreGive(g_sensorDataMutex);
    }
    g_sensorTaskStats.imu_samples++;
}

static void readMag() {
    // Read compass via ArduPilot backend
    if (!g_compass->read()) {
        g_sensorTaskStats.mag_errors++;
        return;
    }

    // Get calibrated magnetic field (milliGauss, calibration applied by ArduPilot)
    const ::Vector3f& field = g_compass->get_field(0);

    if (xSemaphoreTake(g_sensorDataMutex, 0) == pdTRUE) {
        // Convert milliGauss to gauss (for consistency with SensorData)
        g_sensorData.mag.x = field.x / 1000.0f;
        g_sensorData.mag.y = field.y / 1000.0f;
        g_sensorData.mag.z = field.z / 1000.0f;

        xSemaphoreGive(g_sensorDataMutex);
    }
    g_sensorTaskStats.mag_samples++;
}

static void readBaro() {
    // Update barometer - this reads new samples from hardware
    g_baro->update();

    if (g_baro->healthy()) {
        if (xSemaphoreTake(g_sensorDataMutex, 0) == pdTRUE) {
            g_sensorData.pressure_pa = g_baro->get_pressure();
            g_sensorData.temperature_c = g_baro->get_temperature();
            g_sensorData.baro_timestamp_us = time_us_32();
            xSemaphoreGive(g_sensorDataMutex);
        }
        g_sensorTaskStats.baro_samples++;
    } else {
        g_sensorTaskStats.baro_errors++;
    }
}

// ============================================================================
// Task Function
// ============================================================================

static void SensorTask_Run(void* pvParameters) {
    (void)pvParameters;

    // DIAGNOSTIC: 7 rapid blinks = SensorTask started (before any other code)
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    for (int i = 0; i < 7; i++) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        busy_wait_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        busy_wait_ms(100);
    }
    busy_wait_ms(500);

    DBG_PRINT("[SensorTask] Starting on Core %d\n", get_core_num());

    // =========================================================================
    // CRITICAL: Initialize sensors HERE, inside the task, AFTER scheduler starts
    // This ensures hal_init() sees scheduler as RUNNING and creates timer/IO tasks
    // which are required for AP_InertialSensor periodic callbacks to work.
    // =========================================================================
    DBG_PRINT("[SensorTask] Initializing sensors (scheduler running)...\n");
    bool sensorsOk = initSensors();
    if (!sensorsOk) {
        DBG_ERROR("[SensorTask] Sensor init failed - task will idle\n");
    }
    DBG_PRINT("[SensorTask] Sensor init complete, entering main loop\n");

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1);  // 1ms = 1kHz

    uint32_t loopCount = 0;
    uint32_t statusCount = 0;

    while (true) {
        // Always read IMU at 1kHz
        if (s_imuInitialized) {
            readIMU();
        }

        // Read magnetometer at 100Hz (every 10th iteration)
        if (s_magInitialized && (loopCount % kMagDivider == 0)) {
            readMag();
        }

        // Read barometer at 50Hz (every 20th iteration)
        if (s_baroInitialized && (loopCount % kBaroDivider == 0)) {
            readBaro();
        }

        loopCount++;
        statusCount++;

        // Print status every 5000 iterations (5 seconds at 1kHz)
        if (statusCount >= 5000) {
            statusCount = 0;
            g_sensorTaskStats.free_heap = xPortGetFreeHeapSize();

            DBG_PRINT("\n[SensorTask] Status (ArduPilot Native):\n");
            DBG_PRINT("  AP_INS: accel=%u, gyro=%u, rate=%uHz\n",
                   g_ins->get_accel_count(), g_ins->get_gyro_count(), g_ins->get_raw_gyro_rate_hz(0));
            DBG_PRINT("  AP_Compass: count=%u, available=%s\n",
                   g_compass->get_count(), g_compass->available() ? "YES" : "NO");
            DBG_PRINT("  Baro: %s\n", s_baroInitialized ? "OK" : "FAIL");
            DBG_PRINT("  IMU samples:  %lu (errors: %lu)\n",
                   g_sensorTaskStats.imu_samples, g_sensorTaskStats.imu_errors);
            DBG_PRINT("  Mag samples:  %lu (errors: %lu)\n",
                   g_sensorTaskStats.mag_samples, g_sensorTaskStats.mag_errors);
            DBG_PRINT("  Baro samples: %lu (errors: %lu)\n",
                   g_sensorTaskStats.baro_samples, g_sensorTaskStats.baro_errors);
            DBG_PRINT("  Loop overruns: %lu\n", g_sensorTaskStats.loop_overruns);
            DBG_PRINT("  Free heap:     %lu bytes\n", g_sensorTaskStats.free_heap);

            // Print current sensor values
            SensorData data;
            if (SensorTask_GetData(data)) {
                DBG_PRINT("  Accel: [%+7.2f, %+7.2f, %+7.2f] m/s^2\n",
                       data.accel.x, data.accel.y, data.accel.z);
                DBG_PRINT("  Gyro:  [%+7.3f, %+7.3f, %+7.3f] rad/s\n",
                       data.gyro.x, data.gyro.y, data.gyro.z);
                DBG_PRINT("  Mag:   [%+7.3f, %+7.3f, %+7.3f] gauss\n",
                       data.mag.x, data.mag.y, data.mag.z);
                DBG_PRINT("  Baro:  %.2f Pa, %.2f C\n",
                       data.pressure_pa, data.temperature_c);
            }
        }

        // Wait until next period
        BaseType_t wasDelayed = xTaskDelayUntil(&lastWakeTime, period);
        if (wasDelayed == pdFALSE) {
            g_sensorTaskStats.loop_overruns++;
        }
    }
}

// ============================================================================
// Public API
// ============================================================================

bool SensorTask_Init() {
    // Create mutex for shared data
    // NOTE: Sensor initialization happens INSIDE SensorTask_Run() after scheduler starts.
    // This is critical because AP_HAL scheduler.init() requires FreeRTOS to be running
    // to create the timer/IO tasks needed for AP_InertialSensor callbacks.
    g_sensorDataMutex = xSemaphoreCreateMutex();
    if (g_sensorDataMutex == nullptr) {
        DBG_ERROR("[SensorTask] Failed to create mutex\n");
        return false;
    }

    DBG_PRINT("[SensorTask] Mutex created (sensors init deferred to task)\n");
    return true;
}

bool SensorTask_Create() {
    if (g_sensorDataMutex == nullptr) {
        DBG_ERROR("[SensorTask] Not initialized - call SensorTask_Init() first\n");
        return false;
    }

    BaseType_t result = xTaskCreate(
        SensorTask_Run,
        "Sensor",
        SensorTaskConfig::STACK_SIZE,
        nullptr,
        SensorTaskConfig::TASK_PRIORITY,
        &s_taskHandle
    );

    if (result != pdPASS) {
        DBG_ERROR("[SensorTask] Failed to create task\n");
        return false;
    }

    // Pin to Core 1
    vTaskCoreAffinitySet(s_taskHandle, SensorTaskConfig::CORE_AFFINITY);

    DBG_PRINT("[SensorTask] Task created (priority %lu, Core 1)\n",
           SensorTaskConfig::TASK_PRIORITY);
    return true;
}

bool SensorTask_GetData(SensorData& data) {
    if (g_sensorDataMutex == nullptr) {
        return false;
    }

    if (xSemaphoreTake(g_sensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        data = g_sensorData;
        xSemaphoreGive(g_sensorDataMutex);
        return true;
    }
    return false;
}

SensorTaskStats SensorTask_GetStats() {
    return g_sensorTaskStats;
}

void SensorTask_PrintStatus() {
    g_sensorTaskStats.free_heap = xPortGetFreeHeapSize();

    printf("\n[SensorTask] Status (ArduPilot Native):\n");
    printf("  AP_INS: accel=%u, gyro=%u, rate=%uHz\n",
           g_ins->get_accel_count(), g_ins->get_gyro_count(), g_ins->get_raw_gyro_rate_hz(0));
    printf("  AP_Compass: count=%u, available=%s\n",
           g_compass->get_count(), g_compass->available() ? "YES" : "NO");
    printf("  Baro: %s\n", s_baroInitialized ? "OK" : "FAIL");
    printf("  IMU samples:  %lu (errors: %lu)\n",
           g_sensorTaskStats.imu_samples, g_sensorTaskStats.imu_errors);
    printf("  Mag samples:  %lu (errors: %lu)\n",
           g_sensorTaskStats.mag_samples, g_sensorTaskStats.mag_errors);
    printf("  Baro samples: %lu (errors: %lu)\n",
           g_sensorTaskStats.baro_samples, g_sensorTaskStats.baro_errors);
    printf("  Loop overruns: %lu\n", g_sensorTaskStats.loop_overruns);
    printf("  Free heap:     %lu bytes\n", g_sensorTaskStats.free_heap);

    // Print current sensor values
    SensorData data;
    if (SensorTask_GetData(data)) {
        printf("  Accel: [%+7.2f, %+7.2f, %+7.2f] m/s^2\n",
               data.accel.x, data.accel.y, data.accel.z);
        printf("  Gyro:  [%+7.3f, %+7.3f, %+7.3f] rad/s\n",
               data.gyro.x, data.gyro.y, data.gyro.z);
        printf("  Mag:   [%+7.3f, %+7.3f, %+7.3f] gauss\n",
               data.mag.x, data.mag.y, data.mag.z);
        printf("  Baro:  %.1f Pa, %.1f C\n",
               data.pressure_pa, data.temperature_c);
    }
    printf("\n");
}

} // namespace services
} // namespace rocketchip
