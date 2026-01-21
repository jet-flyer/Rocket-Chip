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

// NeoPixel pin for Feather RP2350 HSTX
static constexpr uint8_t NEOPIXEL_PIN = 21;

// Orientation names for user prompts
static const char* ORIENTATION_NAMES[] = {
    "LEVEL (Z pointing UP)",
    "ON LEFT SIDE (Y pointing UP)",
    "ON RIGHT SIDE (Y pointing DOWN)",
    "NOSE DOWN (X pointing DOWN)",
    "NOSE UP (X pointing UP)",
    "UPSIDE DOWN (Z pointing DOWN)"
};

// NeoPixel for status indication
static WS2812* statusLed = nullptr;

// Animation state
static uint32_t lastAnimUpdate = 0;
static float pulsePhase = 0.0f;
static bool blinkState = false;

enum class CalibrationState {
    INITIALIZING,
    WAITING_FOR_ORIENTATION,
    COLLECTING_SAMPLES,
    PROCESSING,
    SUCCESS,
    ERROR
};

static CalibrationState currentState = CalibrationState::INITIALIZING;

/**
 * @brief Update NeoPixel based on current state
 */
void updateStatusLED() {
    if (!statusLed) return;

    uint32_t now = Timing::millis32();
    RGB color;

    switch (currentState) {
        case CalibrationState::INITIALIZING:
            // Yellow pulsing
            if (now - lastAnimUpdate >= 20) {
                pulsePhase += 0.1f;
                float brightness = (sinf(pulsePhase) + 1.0f) * 0.5f;
                color = RGB(
                    static_cast<uint8_t>(255 * brightness),
                    static_cast<uint8_t>(255 * brightness),
                    0
                );
                lastAnimUpdate = now;
            }
            break;

        case CalibrationState::WAITING_FOR_ORIENTATION:
            // Blue slow blink (500ms)
            if (now - lastAnimUpdate >= 500) {
                blinkState = !blinkState;
                lastAnimUpdate = now;
            }
            color = blinkState ? Colors::BLUE : Colors::OFF;
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
            // Green fast blink (100ms)
            if (now - lastAnimUpdate >= 100) {
                blinkState = !blinkState;
                lastAnimUpdate = now;
            }
            color = blinkState ? Colors::GREEN : Colors::OFF;
            break;

        case CalibrationState::ERROR:
            // Solid red
            color = Colors::RED;
            break;
    }

    statusLed->setPixel(0, color);
    statusLed->show();
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
    statusLed = new WS2812(NEOPIXEL_PIN, 1);
    if (!statusLed->begin()) {
        // Continue without LED - will be handled later
        delete statusLed;
        statusLed = nullptr;
    } else {
        statusLed->setBrightness(50);  // 20% brightness
    }

    // Wait for USB connection with visual feedback
    // (per DEBUG_OUTPUT.md: LED blink while waiting indicates "connect terminal now")
    bool ledState = false;
    while (!stdio_usb_connected()) {
        if (statusLed) {
            // Magenta blink = waiting for USB
            statusLed->setPixel(0, ledState ? RGB(128, 0, 128) : Colors::OFF);
            statusLed->show();
        }
        ledState = !ledState;
        sleep_ms(200);
    }
    sleep_ms(500);  // Brief settle time per DEBUG_OUTPUT.md

    printf("\n");
    printf("========================================\n");
    printf("  RocketChip Accelerometer Calibration\n");
    printf("  Using ArduPilot AccelCalibrator\n");
    printf("========================================\n\n");

    // NeoPixel already initialized above - just set initial state
    currentState = CalibrationState::INITIALIZING;
    updateStatusLED();

    // Initialize I2C for sensors
    i2c_init(i2c1, 400000);
    gpio_set_function(2, GPIO_FUNC_I2C);  // SDA
    gpio_set_function(3, GPIO_FUNC_I2C);  // SCL
    gpio_pull_up(2);
    gpio_pull_up(3);

    // Initialize IMU
    I2CBus imuBus(i2c1, 0x6A, 2, 3, 400000);
    IMU_ISM330DHCX imu(&imuBus);

    printf("Initializing IMU...\n");
    if (!imu.begin()) {
        printf("ERROR: IMU initialization failed!\n");
        currentState = CalibrationState::ERROR;
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

    calibrator.start(ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID, 6, 2.0f);

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
            if (!collectingSamples && orientation < 6) {
                currentState = CalibrationState::WAITING_FOR_ORIENTATION;
                updateStatusLED();

                printf("\n--- Orientation %d of 6 ---\n", orientation + 1);
                printf("Place device: %s\n", ORIENTATION_NAMES[orientation]);
                waitForEnter();

                printf("Collecting samples");
                fflush(stdout);

                calibrator.collect_sample();
                collectingSamples = true;
                sampleStartTime = Timing::millis32();
                currentState = CalibrationState::COLLECTING_SAMPLES;
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
                if ((now - sampleStartTime) % 200 < 50) {
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
            currentState = CalibrationState::COLLECTING_SAMPLES;
        }

        updateStatusLED();
        sleep_ms(1);
    }

    // Report results
    printf("\n\n========================================\n");

    if (calibrator.get_status() == ACCEL_CAL_SUCCESS) {
        currentState = CalibrationState::SUCCESS;
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
        currentState = CalibrationState::ERROR;
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
