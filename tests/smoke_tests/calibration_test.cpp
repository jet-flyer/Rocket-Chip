/**
 * @file calibration_test.cpp
 * @brief Accelerometer calibration smoke test
 *
 * Interactive 6-position accelerometer calibration using ArduPilot's
 * AccelCalibrator library.
 *
 * Hardware required:
 * - Adafruit Feather RP2350 HSTX
 * - ISM330DHCX + LIS3MDL FeatherWing
 *
 * NeoPixel Status Colors:
 * - Yellow pulsing: Initializing
 * - Yellow solid: Calibrating (collecting samples)
 * - Blue slow blink: Waiting for user input
 * - Blue solid: Collecting samples for current orientation
 * - Green fast blink: Calibration successful
 * - Red solid: Error
 */

#include <cstdio>
#include <cmath>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// HAL includes
#include "HAL.h"
#include "Bus.h"
#include "Timing.h"
#include "PIO.h"
#include "IMU_ISM330DHCX.h"

// ArduPilot calibrator
#include <AP_AccelCal/AccelCalibrator.h>

using namespace rocketchip::hal;

// ============================================================================
// Configuration Constants
// ============================================================================

// Hardware pins (per HARDWARE.md Feather RP2350 HSTX Built-in Hardware)
static constexpr uint8_t kNeoPixelPin = 21;

// Timing constants (in milliseconds)
static constexpr uint32_t kUsbWaitBlinkMs = 200;      // USB wait blink period
static constexpr uint32_t kUsbSettleTimeMs = 500;     // Settle after USB connect
static constexpr uint32_t kBlinkSlowPeriodMs = 500;   // Slow blink (waiting)
static constexpr uint32_t kBlinkFastPeriodMs = 100;   // Fast blink (success)
static constexpr uint32_t kAnimUpdatePeriodMs = 20;   // Pulse animation update
static constexpr uint32_t kProgressIndicatorMs = 200; // Sample progress dots

// I2C configuration
static constexpr uint32_t kI2cSpeedHz = 400000;       // 400kHz Fast Mode

// LED brightness (0-255, but using percentage for readability)
static constexpr uint8_t kLedBrightnessPercent = 50;  // 20% equivalent

// Calibration parameters
static constexpr uint8_t kNumOrientations = 6;
static constexpr float kSampleDurationSec = 2.0f;

// ============================================================================
// Static Data
// ============================================================================

// Orientation names for user prompts
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

// NeoPixel for status indication
static WS2812* g_statusLed = nullptr;

// Animation state
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

/**
 * @brief Update NeoPixel based on current state
 */
void updateStatusLED() {
    if (!g_statusLed) return;

    uint32_t now = Timing::millis32();
    RGB color;

    switch (g_currentState) {
        case CalibrationState::INITIALIZING:
            // Yellow pulsing
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
            // Blue slow blink
            if (now - g_lastAnimUpdate >= kBlinkSlowPeriodMs) {
                g_blinkState = !g_blinkState;
                g_lastAnimUpdate = now;
            }
            color = g_blinkState ? Colors::BLUE : Colors::OFF;
            break;

        case CalibrationState::COLLECTING_SAMPLES:
            // Solid blue
            color = Colors::BLUE;
            break;

        case CalibrationState::PROCESSING:
            // Yellow solid
            color = Colors::YELLOW;
            break;

        case CalibrationState::SUCCESS:
            // Green fast blink
            if (now - g_lastAnimUpdate >= kBlinkFastPeriodMs) {
                g_blinkState = !g_blinkState;
                g_lastAnimUpdate = now;
            }
            color = g_blinkState ? Colors::GREEN : Colors::OFF;
            break;

        case CalibrationState::ERROR:
            // Solid red
            color = Colors::RED;
            break;
    }

    g_statusLed->setPixel(0, color);
    g_statusLed->show();
}

/**
 * @brief Wait for user to press Enter
 */
void waitForEnter() {
    printf("Press Enter when ready...\n");
    int c;
    while ((c = getchar_timeout_us(10000)) != '\n' && c != '\r') {
        updateStatusLED();
        if (c == PICO_ERROR_TIMEOUT) {
            continue;
        }
    }
    // Clear any remaining input
    while (getchar_timeout_us(1000) != PICO_ERROR_TIMEOUT) {}
}

int main() {
    // Initialize stdio
    stdio_init_all();

    // Initialize NeoPixel early for visual feedback during USB wait
    // (per DEBUG_OUTPUT.md: visual indicators provide immediate feedback)
    g_statusLed = new WS2812(kNeoPixelPin, 1);
    if (!g_statusLed->begin()) {
        // Continue without LED - will be handled later
        delete g_statusLed;
        g_statusLed = nullptr;
    } else {
        g_statusLed->setBrightness(kLedBrightnessPercent);
    }

    // Wait for USB connection with visual feedback
    // (per DEBUG_OUTPUT.md: LED blink while waiting indicates "connect terminal now")
    bool ledState = false;
    while (!stdio_usb_connected()) {
        if (g_statusLed) {
            // Magenta blink = waiting for USB
            g_statusLed->setPixel(0, ledState ? RGB(128, 0, 128) : Colors::OFF);
            g_statusLed->show();
        }
        ledState = !ledState;
        sleep_ms(kUsbWaitBlinkMs);
    }
    sleep_ms(kUsbSettleTimeMs);  // Brief settle time per DEBUG_OUTPUT.md

    printf("\n");
    printf("========================================\n");
    printf("  RocketChip Accelerometer Calibration\n");
    printf("  Using ArduPilot AccelCalibrator\n");
    printf("========================================\n\n");

    // NeoPixel already initialized above - just set initial state
    g_currentState = CalibrationState::INITIALIZING;
    updateStatusLED();

    // Initialize I2C for sensors
    i2c_init(i2c1, kI2cSpeedHz);
    gpio_set_function(2, GPIO_FUNC_I2C);  // SDA
    gpio_set_function(3, GPIO_FUNC_I2C);  // SCL
    gpio_pull_up(2);
    gpio_pull_up(3);

    // Initialize IMU
    I2CBus imuBus(i2c1, 0x6A, 2, 3, kI2cSpeedHz);
    IMU_ISM330DHCX imu(&imuBus);

    printf("Initializing IMU...\n");
    if (!imu.begin()) {
        printf("ERROR: IMU initialization failed!\n");
        g_currentState = CalibrationState::ERROR;
        while (true) {
            updateStatusLED();
            sleep_ms(100);
        }
    }

    // Configure IMU for calibration
    imu.setAccelRange(AccelRange::RANGE_4G);
    imu.setGyroRange(GyroRange::RANGE_500DPS);
    imu.setODR(ODR::ODR_104HZ);

    printf("IMU initialized successfully.\n\n");

    // Create ArduPilot calibrator
    AccelCalibrator calibrator;

    // Start calibration with 6 samples, 2 seconds per sample
    printf("Starting 6-position calibration...\n");
    printf("You will need to hold the device in 6 different orientations.\n");
    printf("Hold steady for 2 seconds in each position.\n\n");

    calibrator.start(ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID, kNumOrientations, kSampleDurationSec);

    uint8_t orientation = 0;
    uint32_t sampleStartTime = 0;
    uint32_t lastSampleTime = 0;
    bool collectingSamples = false;

    while (calibrator.get_status() != ACCEL_CAL_SUCCESS &&
           calibrator.get_status() != ACCEL_CAL_FAILED) {

        // Check for timeout
        calibrator.check_for_timeout();

        // Prompt for next orientation if needed
        if (calibrator.get_status() == ACCEL_CAL_WAITING_FOR_ORIENTATION) {
            if (!collectingSamples && orientation < kNumOrientations) {
                g_currentState = CalibrationState::WAITING_FOR_ORIENTATION;
                updateStatusLED();

                printf("\n--- Orientation %d of %d ---\n", orientation + 1, kNumOrientations);
                printf("Place device: %s\n", kOrientationNames[orientation]);
                waitForEnter();

                printf("Collecting samples");
                fflush(stdout);

                calibrator.collect_sample();
                collectingSamples = true;
                sampleStartTime = Timing::millis32();
                g_currentState = CalibrationState::COLLECTING_SAMPLES;
            }
        }

        // Feed samples to calibrator at ~100Hz
        uint32_t now = Timing::millis32();
        if (collectingSamples && (now - lastSampleTime >= 10)) {
            rocketchip::hal::Vector3f accel, gyro;
            if (imu.read(accel, gyro)) {
                // Convert from RocketChip Vector3f to ArduPilot Vector3f
                // Note: AccelCalibrator expects delta_velocity, so we multiply by dt
                float dt = (now - lastSampleTime) / 1000.0f;
                ::Vector3f ap_accel(accel.x * dt, accel.y * dt, accel.z * dt);
                calibrator.new_sample(ap_accel, dt);

                // Progress indicator
                if ((now - sampleStartTime) % kProgressIndicatorMs < 50) {
                    printf(".");
                    fflush(stdout);
                }
            }
            lastSampleTime = now;
        }

        // Check if we moved to next orientation
        if (collectingSamples &&
            calibrator.get_status() == ACCEL_CAL_WAITING_FOR_ORIENTATION &&
            calibrator.get_num_samples_collected() > orientation) {
            printf(" Done!\n");
            orientation++;
            collectingSamples = false;
        }

        // Check if calibration is being processed
        if (calibrator.get_status() == ACCEL_CAL_COLLECTING_SAMPLE) {
            g_currentState = CalibrationState::COLLECTING_SAMPLES;
        }

        updateStatusLED();
        sleep_ms(1);
    }

    // Report results
    printf("\n\n========================================\n");

    if (calibrator.get_status() == ACCEL_CAL_SUCCESS) {
        g_currentState = CalibrationState::SUCCESS;
        printf("CALIBRATION SUCCESSFUL!\n");
        printf("========================================\n\n");

        printf("Fitness (mean squared residual): %.6f\n\n", calibrator.get_fitness());

        // Get calibration parameters
        ::Vector3f offset, diag, offdiag;
        calibrator.get_calibration(offset, diag, offdiag);

        printf("Calibration Parameters:\n");
        printf("-----------------------\n");
        printf("Offsets (g):\n");
        printf("  X: %+.6f\n", offset.x);
        printf("  Y: %+.6f\n", offset.y);
        printf("  Z: %+.6f\n", offset.z);

        printf("\nScale factors (diagonal):\n");
        printf("  X: %.6f\n", diag.x);
        printf("  Y: %.6f\n", diag.y);
        printf("  Z: %.6f\n", diag.z);

        printf("\nOff-diagonal (cross-axis):\n");
        printf("  XY: %.6f\n", offdiag.x);
        printf("  XZ: %.6f\n", offdiag.y);
        printf("  YZ: %.6f\n", offdiag.z);

        printf("\n// Copy these values to your configuration:\n");
        printf("constexpr float ACCEL_OFFSET_X = %+.6ff;\n", offset.x);
        printf("constexpr float ACCEL_OFFSET_Y = %+.6ff;\n", offset.y);
        printf("constexpr float ACCEL_OFFSET_Z = %+.6ff;\n", offset.z);
        printf("constexpr float ACCEL_SCALE_X  = %.6ff;\n", diag.x);
        printf("constexpr float ACCEL_SCALE_Y  = %.6ff;\n", diag.y);
        printf("constexpr float ACCEL_SCALE_Z  = %.6ff;\n", diag.z);

    } else {
        g_currentState = CalibrationState::ERROR;
        printf("CALIBRATION FAILED!\n");
        printf("========================================\n\n");
        printf("Status code: %d\n", calibrator.get_status());
        printf("Samples collected: %d\n", calibrator.get_num_samples_collected());
    }

    printf("\n========================================\n");

    // Keep LED status visible
    while (true) {
        updateStatusLED();
        sleep_ms(50);
    }

    return 0;
}
