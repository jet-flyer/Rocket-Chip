/**
 * @file calibration_test.cpp
 * @brief Accelerometer calibration smoke test using ArduPilot AccelCalibrator
 *
 * Uses ArduPilot's AccelCalibrator algorithm with our existing sensor driver.
 * This validates the calibration algorithm works while we work on full
 * AP_InertialSensor integration (requires hwdef).
 *
 * Hardware required:
 * - Adafruit Feather RP2350 HSTX
 * - ISM330DHCX + LIS3MDL FeatherWing (or ICM-20948)
 *
 * NeoPixel Status Colors:
 * - Magenta blink: Waiting for USB
 * - Yellow pulsing: Initializing
 * - Blue slow blink: Waiting for user input
 * - Blue solid: Collecting samples for current orientation
 * - Green fast blink: Calibration successful
 * - Red solid: Error
 */

#include <cstdio>
#include <cmath>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// RocketChip HAL
#include "HAL.h"
#include "Bus.h"
#include "Timing.h"
#include "PIO.h"
#include "IMU_ICM20948.h"

// ArduPilot HAL for storage
#include <AP_HAL_RP2350/HAL_RP2350_Class.h>

// ArduPilot calibrator (standalone, no full INS required)
#include <AP_AccelCal/AccelCalibrator.h>
#include <AP_Math/AP_Math.h>

using namespace rocketchip::hal;

// ============================================================================
// Configuration Constants
// ============================================================================

static constexpr uint8_t kNeoPixelPin = 21;
static constexpr uint32_t kI2cSpeedHz = 400000;

// Timing constants (in milliseconds)
static constexpr uint32_t kUsbWaitBlinkMs = 200;
static constexpr uint32_t kUsbSettleTimeMs = 500;
static constexpr uint32_t kBlinkSlowPeriodMs = 500;
static constexpr uint32_t kBlinkFastPeriodMs = 100;
static constexpr uint32_t kAnimUpdatePeriodMs = 20;
static constexpr uint32_t kProgressIndicatorMs = 200;

static constexpr uint8_t kLedBrightnessPercent = 50;

// Calibration parameters
static constexpr uint8_t kNumOrientations = 6;
static constexpr float kSampleTimeSec = 0.5f;

// ============================================================================
// Static Data
// ============================================================================

static const char* const kOrientationNames[] = {
    "LEVEL (Z pointing UP)",
    "ON LEFT SIDE (Y pointing UP)",
    "ON RIGHT SIDE (Y pointing DOWN)",
    "NOSE DOWN (X pointing DOWN)",
    "NOSE UP (X pointing UP)",
    "UPSIDE DOWN (Z pointing DOWN)"
};

// ============================================================================
// Module State
// ============================================================================

static WS2812* g_statusLed = nullptr;
static uint32_t g_lastAnimUpdate = 0;
static float g_pulsePhase = 0.0f;
static bool g_blinkState = false;

enum class CalibrationState {
    INITIALIZING,
    WAITING_FOR_ORIENTATION,
    COLLECTING_SAMPLES,
    PROCESSING,
    SUCCESS,
    ERROR
};

static CalibrationState g_currentState = CalibrationState::INITIALIZING;

// ============================================================================
// LED Status Functions
// ============================================================================

void updateStatusLED() {
    if (!g_statusLed) return;

    uint32_t now = Timing::millis32();
    RGB color;

    switch (g_currentState) {
        case CalibrationState::INITIALIZING:
            if (now - g_lastAnimUpdate >= kAnimUpdatePeriodMs) {
                g_pulsePhase += 0.1f;
                float brightness = (sinf(g_pulsePhase) + 1.0f) * 0.5f;
                color = RGB(
                    static_cast<uint8_t>(255 * brightness),
                    static_cast<uint8_t>(255 * brightness),
                    0
                );
                g_lastAnimUpdate = now;
            }
            break;

        case CalibrationState::WAITING_FOR_ORIENTATION:
            if (now - g_lastAnimUpdate >= kBlinkSlowPeriodMs) {
                g_blinkState = !g_blinkState;
                g_lastAnimUpdate = now;
            }
            color = g_blinkState ? Colors::BLUE : Colors::OFF;
            break;

        case CalibrationState::COLLECTING_SAMPLES:
            color = Colors::BLUE;
            break;

        case CalibrationState::PROCESSING:
            color = Colors::YELLOW;
            break;

        case CalibrationState::SUCCESS:
            if (now - g_lastAnimUpdate >= kBlinkFastPeriodMs) {
                g_blinkState = !g_blinkState;
                g_lastAnimUpdate = now;
            }
            color = g_blinkState ? Colors::GREEN : Colors::OFF;
            break;

        case CalibrationState::ERROR:
            color = Colors::RED;
            break;
    }

    g_statusLed->setPixel(0, color);
    g_statusLed->show();
}

// ============================================================================
// User Input Functions
// ============================================================================

int waitForKey() {
    int c;
    while ((c = getchar_timeout_us(10000)) == PICO_ERROR_TIMEOUT) {
        updateStatusLED();
    }
    while (getchar_timeout_us(1000) != PICO_ERROR_TIMEOUT) {}
    return c;
}

void waitForEnter() {
    printf("Press Enter when ready...\n");
    fflush(stdout);
    int c;
    while ((c = getchar_timeout_us(10000)) != '\n' && c != '\r') {
        updateStatusLED();
        if (c == PICO_ERROR_TIMEOUT) continue;
    }
    while (getchar_timeout_us(1000) != PICO_ERROR_TIMEOUT) {}
}

// ============================================================================
// Main
// ============================================================================

int main() {
    // Initialize NeoPixel early for visual feedback
    g_statusLed = new WS2812(kNeoPixelPin, 1);
    if (!g_statusLed->begin()) {
        delete g_statusLed;
        g_statusLed = nullptr;
    } else {
        g_statusLed->setBrightness(kLedBrightnessPercent);
        g_statusLed->setPixel(0, Colors::YELLOW);
        g_statusLed->show();
    }

    // Initialize ArduPilot HAL (for storage - future use)
    RP2350::hal_init();

    // Clear BASEPRI before USB init
    __asm volatile ("mov r0, #0\nmsr basepri, r0" ::: "r0");

    // Initialize USB
    stdio_init_all();

    // Wait for USB connection
    bool ledState = false;
    while (!stdio_usb_connected()) {
        if (g_statusLed) {
            g_statusLed->setPixel(0, ledState ? RGB(128, 0, 128) : Colors::OFF);
            g_statusLed->show();
        }
        ledState = !ledState;
        sleep_ms(kUsbWaitBlinkMs);
    }
    sleep_ms(kUsbSettleTimeMs);

    printf("\n");
    printf("========================================\n");
    printf("  RocketChip Accelerometer Calibration\n");
    printf("  Using ArduPilot AccelCalibrator\n");
    printf("  Sensor: ICM-20948\n");
    printf("========================================\n\n");

    g_currentState = CalibrationState::INITIALIZING;
    updateStatusLED();

    // ========================================================================
    // Initialize I2C and IMU
    // ========================================================================
    i2c_init(i2c1, kI2cSpeedHz);
    gpio_set_function(2, GPIO_FUNC_I2C);
    gpio_set_function(3, GPIO_FUNC_I2C);
    gpio_pull_up(2);
    gpio_pull_up(3);

    I2CBus imuBus(i2c1, IMU_ICM20948::I2C_ADDR_ALT, 2, 3, kI2cSpeedHz);  // 0x69 on this board
    imuBus.begin();  // Initialize the bus wrapper
    IMU_ICM20948 imu(&imuBus);

    printf("Initializing IMU...\n");
    fflush(stdout);
    if (!imu.begin()) {
        printf("ERROR: IMU initialization failed!\n");
        fflush(stdout);
        g_currentState = CalibrationState::ERROR;
        while (true) { updateStatusLED(); sleep_ms(100); }
    }

    imu.setAccelRange(AccelRange::RANGE_4G);
    imu.setGyroRange(GyroRange::RANGE_500DPS);
    // Note: ICM-20948 ODR is set via sample rate dividers (not implemented in basic driver)

    printf("IMU initialized successfully.\n\n");
    fflush(stdout);

    // ========================================================================
    // Run Calibration using AccelCalibrator
    // ========================================================================
    printf("Starting 6-position calibration...\n");
    printf("You will need to hold the device in 6 different orientations.\n");
    printf("Hold steady for %.1f seconds in each position.\n\n", kSampleTimeSec);
    fflush(stdout);

    // Create calibrator instance
    static AccelCalibrator calibrator;
    calibrator.start(ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID, kNumOrientations, kSampleTimeSec);

    uint8_t orientation = 0;
    uint32_t sampleStartTime = 0;
    uint32_t lastSampleTime = 0;
    bool collectingSamples = false;

    while (calibrator.get_status() != ACCEL_CAL_SUCCESS &&
           calibrator.get_status() != ACCEL_CAL_FAILED) {

        calibrator.check_for_timeout();

        // Prompt for next orientation
        if (calibrator.get_status() == ACCEL_CAL_WAITING_FOR_ORIENTATION) {
            if (!collectingSamples && orientation < kNumOrientations) {
                g_currentState = CalibrationState::WAITING_FOR_ORIENTATION;
                updateStatusLED();

                printf("\n--- Orientation %d of %d ---\n", orientation + 1, kNumOrientations);
                printf("Place device: %s\n", kOrientationNames[orientation]);
                fflush(stdout);
                waitForEnter();

                printf("Collecting samples");
                fflush(stdout);

                calibrator.collect_sample();
                collectingSamples = true;
                sampleStartTime = Timing::millis32();
                lastSampleTime = sampleStartTime;
                g_currentState = CalibrationState::COLLECTING_SAMPLES;
            }
        }

        // Feed samples at ~100Hz
        uint32_t now = Timing::millis32();
        if (collectingSamples && (now - lastSampleTime >= 10)) {
            rocketchip::hal::Vector3f accel, gyro;
            if (imu.read(accel, gyro)) {
                // Convert from g to m/s² and compute delta_velocity
                float dt = (now - lastSampleTime) / 1000.0f;
                ::Vector3f delta_velocity(
                    accel.x * GRAVITY_MSS * dt,
                    accel.y * GRAVITY_MSS * dt,
                    accel.z * GRAVITY_MSS * dt
                );
                calibrator.new_sample(delta_velocity, dt);

                // Progress indicator
                if ((now - sampleStartTime) % kProgressIndicatorMs < 50) {
                    printf(".");
                    fflush(stdout);
                }
            }
            lastSampleTime = now;
        }

        // Check if moved to next orientation
        if (collectingSamples &&
            calibrator.get_status() == ACCEL_CAL_WAITING_FOR_ORIENTATION &&
            calibrator.get_num_samples_collected() > orientation) {
            printf(" Done!\n");
            orientation++;
            collectingSamples = false;
        }

        if (calibrator.get_status() == ACCEL_CAL_COLLECTING_SAMPLE) {
            g_currentState = CalibrationState::COLLECTING_SAMPLES;
        }

        updateStatusLED();
        sleep_ms(1);
    }

    // ========================================================================
    // Report Results
    // ========================================================================
    printf("\n\n========================================\n");

    if (calibrator.get_status() == ACCEL_CAL_SUCCESS) {
        g_currentState = CalibrationState::PROCESSING;
        updateStatusLED();

        printf("CALIBRATION SUCCESSFUL!\n");
        printf("========================================\n\n");

        printf("Fitness (MSE): %.6f\n", static_cast<double>(calibrator.get_fitness()));

        // Get calibration parameters
        ::Vector3f offset, diag, offdiag;
        calibrator.get_calibration(offset, diag, offdiag);

        printf("\nCalibration Results:\n");
        printf("-----------------------\n");
        printf("Offsets (m/s^2):\n");
        printf("  X: %+.6f\n", static_cast<double>(offset.x));
        printf("  Y: %+.6f\n", static_cast<double>(offset.y));
        printf("  Z: %+.6f\n", static_cast<double>(offset.z));

        printf("\nScale factors (diagonal):\n");
        printf("  X: %.6f\n", static_cast<double>(diag.x));
        printf("  Y: %.6f\n", static_cast<double>(diag.y));
        printf("  Z: %.6f\n", static_cast<double>(diag.z));

        printf("\nCross-axis (off-diagonal):\n");
        printf("  XY: %.6f\n", static_cast<double>(offdiag.x));
        printf("  XZ: %.6f\n", static_cast<double>(offdiag.y));
        printf("  YZ: %.6f\n", static_cast<double>(offdiag.z));

        // Test mode - show corrected readings
        printf("\n========================================\n");
        printf("TEST MODE: Showing corrected readings\n");
        printf("Press any key to exit\n");
        printf("========================================\n\n");

        g_currentState = CalibrationState::SUCCESS;

        while (getchar_timeout_us(100000) == PICO_ERROR_TIMEOUT) {
            rocketchip::hal::Vector3f accel, gyro;
            if (imu.read(accel, gyro)) {
                // Apply calibration manually
                // corrected = diag * (raw - offset) + offdiag_correction
                ::Vector3f raw(accel.x * GRAVITY_MSS, accel.y * GRAVITY_MSS, accel.z * GRAVITY_MSS);
                ::Vector3f corrected;
                corrected.x = diag.x * (raw.x - offset.x);
                corrected.y = diag.y * (raw.y - offset.y);
                corrected.z = diag.z * (raw.z - offset.z);
                // Off-diagonal terms for cross-axis correction
                corrected.x += offdiag.x * (raw.y - offset.y) + offdiag.y * (raw.z - offset.z);
                corrected.y += offdiag.x * (raw.x - offset.x) + offdiag.z * (raw.z - offset.z);
                corrected.z += offdiag.y * (raw.x - offset.x) + offdiag.z * (raw.y - offset.y);

                float raw_mag = raw.length();
                float cor_mag = corrected.length();

                printf("Raw: [%+7.3f, %+7.3f, %+7.3f] |%5.3f|  "
                       "Cor: [%+7.3f, %+7.3f, %+7.3f] |%5.3f|\r",
                       static_cast<double>(raw.x), static_cast<double>(raw.y), static_cast<double>(raw.z),
                       static_cast<double>(raw_mag),
                       static_cast<double>(corrected.x), static_cast<double>(corrected.y), static_cast<double>(corrected.z),
                       static_cast<double>(cor_mag));
                fflush(stdout);
            }
            updateStatusLED();
        }
        printf("\n");

    } else {
        g_currentState = CalibrationState::ERROR;
        printf("CALIBRATION FAILED!\n");
        printf("========================================\n\n");
        printf("Status code: %d\n", calibrator.get_status());
        printf("Samples collected: %d\n", calibrator.get_num_samples_collected());
    }

    printf("\n========================================\n");
    printf("NOTE: Calibration storage requires hwdef setup.\n");
    printf("Values above can be applied manually or saved\n");
    printf("once full AP_InertialSensor integration is complete.\n");
    printf("========================================\n");

    while (true) {
        updateStatusLED();
        sleep_ms(50);
    }

    return 0;
}
