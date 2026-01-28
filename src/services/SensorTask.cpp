/**
 * @file SensorTask.cpp
 * @brief High-rate sensor sampling FreeRTOS task implementation
 *
 * Samples sensors at configured rates and updates shared SensorData.
 */

#include "SensorTask.h"
#include "HAL.h"
#include "IMU_ISM330DHCX.h"
#include "Mag_LIS3MDL.h"
#include "Baro_DPS310.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"

#include "debug.h"

// RocketChip calibration storage
#include "calibration/CalibrationStore.h"
#include <AP_HAL_RP2350/hwdef.h>  // For GRAVITY_MSS, DEG_TO_RAD

#include <cstdio>
#include <cmath>

using namespace rocketchip::hal;

namespace rocketchip {
namespace services {

// ============================================================================
// Hardware Configuration (k prefix per JSF AV naming conventions)
// ============================================================================

// I2C pins for Feather RP2350 (STEMMA QT / Qwiic)
static constexpr uint8_t kI2cSda = 2;
static constexpr uint8_t kI2cScl = 3;

// I2C addresses
static constexpr uint8_t kAddrIsm330dhcx = 0x6A;
static constexpr uint8_t kAddrLis3mdl    = 0x1C;
static constexpr uint8_t kAddrDps310     = 0x77;

// ============================================================================
// Global Data
// ============================================================================

SensorData g_sensorData = {};
SemaphoreHandle_t g_sensorDataMutex = nullptr;
SensorTaskStats g_sensorTaskStats = {};

// ============================================================================
// Private Data
// ============================================================================

static TaskHandle_t s_taskHandle = nullptr;

// Calibration
static CalibrationStore s_calibrationStore;
static SensorCalibration s_calibration;
static bool s_calibrationLoaded = false;

// Sensor instances
static I2CBus* s_imuBus = nullptr;
static I2CBus* s_magBus = nullptr;
static I2CBus* s_baroBus = nullptr;

static IMU_ISM330DHCX* s_imu = nullptr;
static Mag_LIS3MDL* s_mag = nullptr;
static Baro_DPS310* s_baro = nullptr;

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

static bool initSensors() {
    DBG_PRINT("[SensorTask] Initializing sensors...\n");

    // Create bus instances (all share I2C1 hardware)
    s_imuBus = new I2CBus(i2c1, kAddrIsm330dhcx, kI2cSda, kI2cScl, 400000);
    s_magBus = new I2CBus(i2c1, kAddrLis3mdl, kI2cSda, kI2cScl, 400000);
    s_baroBus = new I2CBus(i2c1, kAddrDps310, kI2cSda, kI2cScl, 400000);

    // Create sensor instances
    s_imu = new IMU_ISM330DHCX(s_imuBus);
    s_mag = new Mag_LIS3MDL(s_magBus);
    s_baro = new Baro_DPS310(s_baroBus);

    // Initialize IMU
    DBG_PRINT("  ISM330DHCX (IMU): ");
    s_imuInitialized = s_imu->begin();
    if (s_imuInitialized) {
        // Configure for 1kHz operation
        s_imu->setODR(ODR::ODR_1666HZ);  // Closest to 1kHz that's higher
        s_imu->setAccelRange(AccelRange::RANGE_8G);
        s_imu->setGyroRange(GyroRange::RANGE_1000DPS);
        DBG_PRINT("OK\n");
    } else {
        DBG_ERROR("FAILED\n");
    }

    // Initialize Magnetometer
    DBG_PRINT("  LIS3MDL (Mag):    ");
    s_magInitialized = s_mag->begin();
    if (s_magInitialized) {
        s_mag->setRange(MagRange::RANGE_4GAUSS);
        s_mag->setDataRate(MagDataRate::HP_80Hz);
        DBG_PRINT("OK\n");
    } else {
        DBG_ERROR("FAILED\n");
    }

    // Initialize Barometer
    DBG_PRINT("  DPS310 (Baro):    ");
    s_baroInitialized = s_baro->begin();
    if (s_baroInitialized) {
        // Configure for 50Hz continuous mode
        s_baro->configurePressure(BaroRate::RATE_64HZ, BaroOversample::OSR_8);
        s_baro->configureTemperature(BaroRate::RATE_64HZ, BaroOversample::OSR_8);
        s_baro->startContinuous();
        DBG_PRINT("OK\n");
    } else {
        DBG_ERROR("FAILED\n");
    }

    // Load calibration from flash
    DBG_PRINT("  Calibration:      ");
    s_calibrationStore.init();
    if (s_calibrationStore.load(s_calibration)) {
        s_calibrationLoaded = true;
        DBG_PRINT("Loaded (flags=0x%02X)\n", s_calibration.flags);
        if (s_calibration.flags & kCalFlagAccel) {
            DBG_PRINT("    Accel ofs: [%.3f, %.3f, %.3f] m/s^2\n",
                   s_calibration.accel.offset[0],
                   s_calibration.accel.offset[1],
                   s_calibration.accel.offset[2]);
        }
        if (s_calibration.flags & kCalFlagGyro) {
            DBG_PRINT("    Gyro ofs:  [%.4f, %.4f, %.4f] rad/s\n",
                   s_calibration.gyro.offset[0],
                   s_calibration.gyro.offset[1],
                   s_calibration.gyro.offset[2]);
        }
        if (s_calibration.flags & kCalFlagMag) {
            DBG_PRINT("    Mag ofs:   [%.0f, %.0f, %.0f] mGauss\n",
                   s_calibration.mag.offset[0],
                   s_calibration.mag.offset[1],
                   s_calibration.mag.offset[2]);
        }
    } else {
        s_calibrationLoaded = false;
        CalibrationStore::get_defaults(s_calibration);
        DBG_PRINT("Not calibrated (using defaults)\n");
    }

    // At least IMU must be working for flight
    return s_imuInitialized;
}

// ============================================================================
// Sensor Reading Functions
// ============================================================================

static void readIMU() {
    hal::Vector3f accel, gyro;

    if (s_imu->read(accel, gyro)) {
        // Take mutex and update shared data
        if (xSemaphoreTake(g_sensorDataMutex, 0) == pdTRUE) {
            // Convert accel from g to m/s^2
            float ax = accel.x * GRAVITY_MSS;
            float ay = accel.y * GRAVITY_MSS;
            float az = accel.z * GRAVITY_MSS;

            // Apply accel calibration if available
            if (s_calibrationLoaded && (s_calibration.flags & kCalFlagAccel)) {
                // Apply offset and scale correction
                ax = (ax - s_calibration.accel.offset[0]) * s_calibration.accel.scale[0];
                ay = (ay - s_calibration.accel.offset[1]) * s_calibration.accel.scale[1];
                az = (az - s_calibration.accel.offset[2]) * s_calibration.accel.scale[2];
            }

            g_sensorData.accel.x = ax;
            g_sensorData.accel.y = ay;
            g_sensorData.accel.z = az;

            // Convert gyro from dps to rad/s
            float gx = gyro.x * DEG_TO_RAD;
            float gy = gyro.y * DEG_TO_RAD;
            float gz = gyro.z * DEG_TO_RAD;

            // Apply gyro calibration if available
            if (s_calibrationLoaded && (s_calibration.flags & kCalFlagGyro)) {
                // Apply offset correction
                gx -= s_calibration.gyro.offset[0];
                gy -= s_calibration.gyro.offset[1];
                gz -= s_calibration.gyro.offset[2];
            }

            g_sensorData.gyro.x = gx;
            g_sensorData.gyro.y = gy;
            g_sensorData.gyro.z = gz;

            g_sensorData.imu_timestamp_us = time_us_32();
            xSemaphoreGive(g_sensorDataMutex);
        }
        g_sensorTaskStats.imu_samples++;
    } else {
        g_sensorTaskStats.imu_errors++;
    }
}

static void readMag() {
    hal::Vector3f mag;

    if (s_mag->read(mag)) {
        if (xSemaphoreTake(g_sensorDataMutex, 0) == pdTRUE) {
            // Convert to mGauss (calibration uses mGauss)
            float mx = mag.x * 1000.0f;
            float my = mag.y * 1000.0f;
            float mz = mag.z * 1000.0f;

            // Apply mag calibration if available
            if (s_calibrationLoaded && (s_calibration.flags & kCalFlagMag)) {
                // Apply hard iron correction (subtract offset)
                mx -= s_calibration.mag.offset[0];
                my -= s_calibration.mag.offset[1];
                mz -= s_calibration.mag.offset[2];

                // Apply soft iron correction (matrix multiply)
                // Soft iron matrix:
                //   [diag.x,    offdiag.x, offdiag.y]
                //   [offdiag.x, diag.y,    offdiag.z]
                //   [offdiag.y, offdiag.z, diag.z   ]
                float cx = s_calibration.mag.diag[0] * mx +
                           s_calibration.mag.offdiag[0] * my +
                           s_calibration.mag.offdiag[1] * mz;
                float cy = s_calibration.mag.offdiag[0] * mx +
                           s_calibration.mag.diag[1] * my +
                           s_calibration.mag.offdiag[2] * mz;
                float cz = s_calibration.mag.offdiag[1] * mx +
                           s_calibration.mag.offdiag[2] * my +
                           s_calibration.mag.diag[2] * mz;

                // Apply scale factor
                mx = cx * s_calibration.mag.scale_factor;
                my = cy * s_calibration.mag.scale_factor;
                mz = cz * s_calibration.mag.scale_factor;
            }

            // Store as Gauss (convert back from mGauss)
            g_sensorData.mag.x = mx * 0.001f;
            g_sensorData.mag.y = my * 0.001f;
            g_sensorData.mag.z = mz * 0.001f;
            xSemaphoreGive(g_sensorDataMutex);
        }
    }
}

static void readBaro() {
    float pressure_pa, temp_c;

    if (s_baro->getLastResult(temp_c, pressure_pa)) {
        if (xSemaphoreTake(g_sensorDataMutex, 0) == pdTRUE) {
            g_sensorData.pressure_pa = pressure_pa;
            g_sensorData.temperature_c = temp_c;
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

    DBG_PRINT("[SensorTask] Starting on Core %d\n", get_core_num());

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

            DBG_PRINT("\n[SensorTask] Status:\n");
            DBG_PRINT("  IMU samples:  %lu (errors: %lu)\n",
                   g_sensorTaskStats.imu_samples, g_sensorTaskStats.imu_errors);
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
    // Initialize HAL (I2C, etc.)
    HALInitResult result = initHAL();
    if (!result.success) {
        DBG_ERROR("[SensorTask] HAL init failed: %s\n", result.error_msg);
        return false;
    }

    // Create mutex for shared data
    g_sensorDataMutex = xSemaphoreCreateMutex();
    if (g_sensorDataMutex == nullptr) {
        DBG_ERROR("[SensorTask] Failed to create mutex\n");
        return false;
    }

    // Initialize sensors
    return initSensors();
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

} // namespace services
} // namespace rocketchip
