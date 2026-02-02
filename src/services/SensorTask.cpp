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
#include "hardware/i2c.h"
#include "hardware/gpio.h"

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

// Rocket vehicle class for AP_Param var_info integration
#include "vehicle/Rocket.h"

// Compile-time diagnostic: verify HAL_INS_PROBE_LIST is defined
#ifdef HAL_INS_PROBE_LIST
#pragma message "HAL_INS_PROBE_LIST is defined in SensorTask.cpp"
#else
#pragma message "WARNING: HAL_INS_PROBE_LIST is NOT defined in SensorTask.cpp"
#endif

// GCS MAVLink for calibration callbacks
#include <GCS_MAVLink/GCS.h>

// AP_Notify for boot status feedback
#include <AP_Notify/AP_Notify.h>

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
// ArduPilot Sensor Instances
// Sensors are now owned by the Rocket vehicle class (src/vehicle/Rocket.h).
// Access via rocket.ins(), rocket.compass(), rocket.baro().
// This enables AP_Param var_info table for calibration persistence.
// ============================================================================

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

// Calibration state
static bool s_accelCalRunning = false;
static bool s_compassCalRunning = false;

// Accel calibration step tracking (local cache to avoid 1-second GCS broadcast delay)
static accel_cal_status_t s_lastAccelCalStatus = ACCEL_CAL_NOT_STARTED;
static uint8_t s_accelCalStepCache = 0;

// Init complete flag (for CLI synchronization)
static volatile bool s_initComplete = false;

// Timing dividers (relative to 500Hz base loop rate)
// IMU data arrives at 1125Hz via DeviceBus callbacks; we fetch at 500Hz
static constexpr uint32_t kBaroDivider = 10;  // 500/10 = 50Hz baro
static constexpr uint32_t kMagDivider = 5;    // 500/5 = 100Hz magnetometer

// ============================================================================
// Sensor Initialization
// ============================================================================

// I2C scan functions removed - use debug probe instead of printf diagnostics

static bool initSensors() {
    // Set AP_Notify flag: we're initializing, don't move the vehicle
    AP_Notify::flags.initialising = true;

    // Initialize ArduPilot HAL (I2C, SPI, etc.)
    RP2350::hal_init();

    // Initialize AP_Param storage FIRST (before sensor allocation)
    // This validates/initializes the flash header before any sensor constructors run.
    // Note: With no var_info set yet, load_all() is effectively a no-op.
    AP_Param::setup();

    // Allocate sensor instances on Rocket vehicle (must be AFTER HAL init)
    // Sensor constructors call AP_Param::setup_object_defaults() which uses their
    // own var_info tables (not the vehicle-level table).
    rocket.allocate_sensors();

    // Register vehicle-level var_info table with AP_Param
    // This enables calibration persistence via INS_ACCSCAL, INS_ACCOFFS, etc.
    rocket.init_params();

    // Load saved calibrations from flash into sensor objects
    // This restores calibration data from previous sessions
    AP_Param::load_all();

    // Initialize AP_InertialSensor (probes ICM-20948 via HAL_INS_PROBE_LIST)
    rocket.ins()->init(100);  // 100Hz main loop rate
    s_imuInitialized = (rocket.ins()->get_accel_count() > 0 && rocket.ins()->get_gyro_count() > 0);

    // Initialize Compass (probes AK09916 via HAL_MAG_PROBE_LIST)
    rocket.compass()->init();
    s_magInitialized = (rocket.compass()->get_count() > 0);

    // Initialize Barometer (probes DPS310 via HAL_BARO_PROBE_LIST)
    rocket.baro()->init();
    s_baroInitialized = (rocket.baro()->num_instances() > 0);

    // Calibrate barometer (sets ground pressure reference)
    // Takes ~2 seconds - AP_Notify::flags.initialising provides visual feedback
    if (s_baroInitialized) {
        rocket.baro()->calibrate(false);  // false = don't save to storage yet
    }

    // Load legacy calibration from flash (for barometer, etc.)
    // IMU/Compass calibration now uses AP_Param automatically
    s_calibrationStore.init();
    if (s_calibrationStore.load(s_calibration)) {
        s_calibrationLoaded = true;
    } else {
        s_calibrationLoaded = false;
        CalibrationStore::get_defaults(s_calibration);
    }

    // Initialize GCS and wire up calibration callbacks
    GCS::get_singleton().init();

    // Wire up calibration callbacks from GCS to our API
    GCS::get_singleton().set_simple_accel_cal_callback([]() -> MAV_RESULT {
        return SensorTask_SimpleAccelCal();
    });
    GCS::get_singleton().set_accel_cal_start_callback([]() -> bool {
        return SensorTask_StartAccelCal();
    });
    GCS::get_singleton().set_accel_cal_pos_callback([](int pos) -> bool {
        return SensorTask_AccelCalConfirmPosition(pos);
    });
    GCS::get_singleton().set_compass_cal_start_callback([]() -> bool {
        return SensorTask_StartCompassCal();
    });
    GCS::get_singleton().set_baro_cal_callback([]() -> bool {
        return SensorTask_CalibrateBaro();
    });

    // Initialization complete - clear the flag
    AP_Notify::flags.initialising = false;

    // At least IMU must be working for flight
    return s_imuInitialized;
}

// ============================================================================
// Sensor Reading Functions
// ============================================================================

static void readIMU() {
    // NON-BLOCKING: Don't call update() - it internally calls wait_for_sample()
    // which blocks for 1-2ms due to vTaskDelay() timing on RP2350.
    //
    // DeviceBus callbacks already populate backend buffers at 1125Hz.
    // We just read the latest available calibrated data.
    //
    // See RP2350_FULL_AP_PORT.md PD14 for details on vTaskDelay timing issues.

    // Check if IMU has valid data before reading
    if (!rocket.ins()->get_gyro_health(0)) {
        g_sensorTaskStats.imu_errors++;
        return;
    }

    // Get calibrated sensor data (calibration applied by ArduPilot via AP_Param)
    // Use global namespace :: to get ArduPilot's Vector3f, not rocketchip::services::Vector3f
    const ::Vector3f& accel = rocket.ins()->get_accel(0);
    const ::Vector3f& gyro = rocket.ins()->get_gyro(0);

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
    if (!rocket.compass()->read()) {
        g_sensorTaskStats.mag_errors++;
        return;
    }

    // Get calibrated magnetic field (milliGauss, calibration applied by ArduPilot)
    const ::Vector3f& field = rocket.compass()->get_field(0);

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
    rocket.baro()->update();

    if (rocket.baro()->healthy()) {
        if (xSemaphoreTake(g_sensorDataMutex, 0) == pdTRUE) {
            g_sensorData.pressure_pa = rocket.baro()->get_pressure();
            g_sensorData.temperature_c = rocket.baro()->get_temperature();
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

    // Initialize LED for status
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // NOTE: No USB wait here - sensors don't need USB to initialize.
    // CLITask handles USB wait before printing prompts.

    // =========================================================================
    // CRITICAL: Initialize sensors HERE, inside the task, AFTER scheduler starts
    // This ensures hal_init() sees scheduler as RUNNING and creates timer/IO tasks
    // which are required for AP_InertialSensor periodic callbacks to work.
    // =========================================================================
    initSensors();

    // Signal CLI that it's safe to show prompt (no more init output)
    s_initComplete = true;

    // Reset stats after init so we only count steady-state overruns
    g_sensorTaskStats = {};

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(2);  // 2ms = 500Hz (IMU data arrives at 1125Hz via callbacks)

    uint32_t loopCount = 0;

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

        // Update calibration state machines at 100Hz (every 10th iteration)
        if (loopCount % 10 == 0) {
            // NOTE: GCS::update() not called here - UITask handles all serial input
            // and routes MAVLink bytes to GCS::parse_byte() vs CLI commands.

            // Update accelerometer calibration if running
            if (s_accelCalRunning && rocket.ins() != nullptr) {
                // CRITICAL: Must call update() to feed samples to the calibrator!
                // This calls _backends[i]->update() -> update_accel() -> _publish_accel()
                // which feeds delta_velocity samples to AccelCalibrator::new_sample().
                // Without this, calibration gets NO samples and immediately fails.
                //
                // Note: update() internally calls wait_for_sample() which may block
                // 1-2ms due to PD14 timing issues, but this is acceptable for
                // interactive calibration (not flight-critical).
                rocket.ins()->update();

                rocket.ins()->acal_update();

                AP_AccelCal* acal = rocket.ins()->get_acal();
                if (acal != nullptr) {
                    accel_cal_status_t status = acal->get_status();

                    // Debug: Show status changes with accel count and armed state
                    if (status != s_lastAccelCalStatus) {
                        printf("[AccelCal] status: %d -> %d, step_cache=%d, accel_count=%d, armed=%d\n",
                               (int)s_lastAccelCalStatus, (int)status, s_accelCalStepCache,
                               rocket.ins()->get_accel_count(), (int)AP_HAL::get_HAL().util->get_soft_armed());
                    }

                    // Detect state transition: sample collection complete -> waiting for next position
                    // This updates our step cache immediately instead of waiting for 1-second GCS broadcast
                    if (s_lastAccelCalStatus == ACCEL_CAL_COLLECTING_SAMPLE &&
                        status == ACCEL_CAL_WAITING_FOR_ORIENTATION) {
                        s_accelCalStepCache++;  // Move to next position (2, 3, 4, 5, 6)
                        printf("[AccelCal] Step incremented to %d\n", s_accelCalStepCache);
                    }
                    s_lastAccelCalStatus = status;

                    // Check if calibration finished
                    if (status == ACCEL_CAL_SUCCESS || status == ACCEL_CAL_FAILED) {
                        s_accelCalRunning = false;
                        s_lastAccelCalStatus = ACCEL_CAL_NOT_STARTED;
                        s_accelCalStepCache = 0;
                        printf("[SensorTask] Accel calibration %s\n",
                               status == ACCEL_CAL_SUCCESS ? "SUCCEEDED" : "FAILED");
                    }
                }
            }

            // Update compass calibration if running
            if (s_compassCalRunning && rocket.compass() != nullptr) {
                rocket.compass()->cal_update();
                // Check if calibration finished
                if (!rocket.compass()->is_calibrating()) {
                    s_compassCalRunning = false;
                    printf("[SensorTask] Compass calibration finished\n");
                }
            }
        }

        // Send heartbeat at 1Hz (every 1000th iteration) only when GCS is connected
        // This prevents garbage binary output when using CLI terminal
        if (loopCount % 1000 == 0 && GCS::get_singleton().is_connected()) {
            GCS::get_singleton().send_heartbeat();
        }

        loopCount++;

        // Periodic status disabled - use 's' command in CLI for on-demand status
        // (Automatic printing interferes with interactive CLI menus)

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
        return false;
    }

    return true;
}

bool SensorTask_Create() {
    if (g_sensorDataMutex == nullptr) {
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
        return false;
    }

    // Pin to Core 1 (SMP only)
#if configNUMBER_OF_CORES > 1
    vTaskCoreAffinitySet(s_taskHandle, SensorTaskConfig::CORE_AFFINITY);
#endif

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
           rocket.ins()->get_accel_count(), rocket.ins()->get_gyro_count(), rocket.ins()->get_raw_gyro_rate_hz(0));
    printf("  AP_Compass: count=%u, available=%s (ptr=%p, singleton=%p)\n",
           rocket.compass()->get_count(), rocket.compass()->available() ? "YES" : "NO",
           (void*)rocket.compass(), (void*)Compass::get_singleton());
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

// ============================================================================
// Calibration API Implementation
// ============================================================================

bool SensorTask_StartAccelCal() {
    if (rocket.ins() == nullptr || !s_imuInitialized) {
        printf("[SensorTask] Cannot start accel cal - IMU not initialized\n");
        return false;
    }

    if (s_accelCalRunning || s_compassCalRunning) {
        printf("[SensorTask] Cannot start accel cal - calibration already running\n");
        return false;
    }

    printf("[SensorTask] Starting 6-position accelerometer calibration...\n");

    // Initialize the calibrator (creates AP_AccelCal if needed)
    rocket.ins()->acal_init();

    AP_AccelCal* acal = rocket.ins()->get_acal();
    if (acal == nullptr) {
        printf("[SensorTask] Failed to create AP_AccelCal\n");
        return false;
    }

    // Start the calibration with our GCS_MAVLINK instance
    GCS_MAVLINK* gcs = GCS_MAVLINK::get_singleton();
    acal->start(gcs, MAVLINK_SYS_ID, MAVLINK_COMP_ID);

    s_accelCalRunning = true;
    s_accelCalStepCache = 1;  // Start at position 1 (LEVEL)
    s_lastAccelCalStatus = ACCEL_CAL_WAITING_FOR_ORIENTATION;
    printf("[SensorTask] Accel calibration started - place device LEVEL and confirm\n");
    return true;
}

MAV_RESULT SensorTask_SimpleAccelCal() {
    if (rocket.ins() == nullptr || !s_imuInitialized) {
        printf("[SensorTask] Cannot run simple accel cal - IMU not initialized\n");
        return MAV_RESULT_FAILED;
    }

    if (s_accelCalRunning || s_compassCalRunning) {
        printf("[SensorTask] Cannot run simple accel cal - calibration already running\n");
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }

    printf("[SensorTask] Running simple 1D accelerometer calibration (keep device level)...\n");
    MAV_RESULT result = rocket.ins()->simple_accel_cal();

    if (result == MAV_RESULT_ACCEPTED) {
        printf("[SensorTask] Simple accel calibration completed successfully\n");
    } else {
        printf("[SensorTask] Simple accel calibration failed (result=%d)\n", result);
    }

    return result;
}

bool SensorTask_AccelCalConfirmPosition(int position) {
    if (!s_accelCalRunning || rocket.ins() == nullptr) {
        return false;
    }

    AP_AccelCal* acal = rocket.ins()->get_acal();
    if (acal == nullptr) {
        return false;
    }

    // Tell AP_AccelCal that the vehicle is in the requested position
    return acal->gcs_vehicle_position(static_cast<float>(position));
}

bool SensorTask_StartCompassCal() {
    if (rocket.compass() == nullptr || !s_magInitialized) {
        printf("[SensorTask] Cannot start compass cal - compass not initialized\n");
        return false;
    }

    if (s_accelCalRunning || s_compassCalRunning) {
        printf("[SensorTask] Cannot start compass cal - calibration already running\n");
        return false;
    }

    printf("[SensorTask] Starting compass calibration...\n");
    printf("[SensorTask] Rotate device slowly through all orientations\n");

    // Start calibration for all compass instances
    // Parameters: retry=false, autosave=true, delay=0, autoreboot=false
    bool started = rocket.compass()->start_calibration_all(false, true, 0.0f, false);

    if (started) {
        s_compassCalRunning = true;
        printf("[SensorTask] Compass calibration started\n");
    } else {
        printf("[SensorTask] Failed to start compass calibration\n");
    }

    return started;
}

bool SensorTask_CalibrateBaro() {
    if (rocket.baro() == nullptr || !s_baroInitialized) {
        printf("[SensorTask] Cannot calibrate baro - not initialized\n");
        return false;
    }

    printf("[SensorTask] Calibrating barometer (setting ground pressure)...\n");
    rocket.baro()->calibrate(true);  // true = save to storage
    printf("[SensorTask] Barometer calibration complete\n");
    return true;
}

bool SensorTask_IsCalibrating() {
    return s_accelCalRunning || s_compassCalRunning;
}

void SensorTask_CancelCalibration() {
    if (s_accelCalRunning && rocket.ins() != nullptr) {
        AP_AccelCal* acal = rocket.ins()->get_acal();
        if (acal != nullptr) {
            acal->cancel();
        }
        s_accelCalRunning = false;
        printf("[SensorTask] Accel calibration cancelled\n");
    }

    if (s_compassCalRunning && rocket.compass() != nullptr) {
        rocket.compass()->cancel_calibration_all();
        s_compassCalRunning = false;
        printf("[SensorTask] Compass calibration cancelled\n");
    }
}

bool SensorTask_IsInitComplete() {
    return s_initComplete;
}

uint8_t SensorTask_GetAccelCalStep() {
    if (!s_accelCalRunning) {
        return 0;
    }
    // Use locally cached step (updated immediately on state transition)
    // rather than GCS position tracker (which has 1-second broadcast delay)
    return s_accelCalStepCache;
}

accel_cal_status_t SensorTask_GetAccelCalStatus() {
    if (!s_accelCalRunning || rocket.ins() == nullptr) {
        return ACCEL_CAL_NOT_STARTED;
    }
    AP_AccelCal* acal = rocket.ins()->get_acal();
    if (acal == nullptr) {
        return ACCEL_CAL_NOT_STARTED;
    }
    return acal->get_status();
}

const char* SensorTask_GetAccelCalPositionName(uint8_t step) {
    switch (step) {
        case 1: return "LEVEL (flat on table)";
        case 2: return "LEFT SIDE";
        case 3: return "RIGHT SIDE";
        case 4: return "NOSE DOWN";
        case 5: return "NOSE UP";
        case 6: return "BACK (upside down)";
        default: return "UNKNOWN";
    }
}

} // namespace services
} // namespace rocketchip
