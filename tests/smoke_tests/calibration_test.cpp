/**
 * @file calibration_test.cpp
 * @brief Accelerometer calibration smoke test with persistent storage
 *
 * Interactive 6-position accelerometer calibration using ArduPilot's
 * AccelCalibrator library. Calibration results are saved to flash via
 * AP_Param and persist across power cycles.
 *
 * Hardware required:
 * - Adafruit Feather RP2350 HSTX
 * - ISM330DHCX + LIS3MDL FeatherWing
 *
 * NeoPixel Status Colors:
 * - Magenta blink: Waiting for USB
 * - Yellow pulsing: Initializing
 * - Blue slow blink: Waiting for user input
 * - Blue solid: Collecting samples for current orientation
 * - Green fast blink: Calibration successful (saved to flash)
 * - Cyan solid: Existing calibration loaded from flash
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

// ArduPilot HAL for storage
#include <AP_HAL_RP2350/HAL_RP2350_Class.h>

// ArduPilot parameter system and inertial sensor
#include <AP_Param/AP_Param.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

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
static constexpr uint8_t kLedBrightnessPercent = 50;

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
    LOADED_FROM_FLASH,
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

        case CalibrationState::LOADED_FROM_FLASH:
            // Cyan solid (calibration loaded from storage)
            color = RGB(0, 255, 255);
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
 * @brief Recover USB CDC input after flash operations
 *
 * Flash operations disable interrupts for extended periods, which can
 * corrupt USB CDC state. This function attempts to recover input by
 * draining stale data and giving the USB host time to re-sync.
 */
static void recoverUsbInput() {
    // Check if USB is still connected
    if (!stdio_usb_connected()) {
        printf("WARNING: USB disconnected during init!\n");
        printf("Waiting for reconnection...\n");
        fflush(stdout);
        while (!stdio_usb_connected()) {
            if (g_statusLed) {
                // Orange blink = USB reconnecting
                static bool blink = false;
                g_statusLed->setPixel(0, blink ? RGB(255, 128, 0) : Colors::OFF);
                g_statusLed->show();
                blink = !blink;
            }
            sleep_ms(200);
        }
        sleep_ms(500);  // Extra settle time after reconnect
        printf("USB reconnected!\n");
        fflush(stdout);
    }

    // Drain any garbage that accumulated during flash operations
    int drained = 0;
    while (getchar_timeout_us(1000) != PICO_ERROR_TIMEOUT) {
        drained++;
    }
    if (drained > 0) {
        printf("Drained %d stale bytes from USB\n", drained);
        fflush(stdout);
    }

    // Give USB host time to recover from any missed packets
    // Some USB hosts are slow to re-sync after device stops responding
    sleep_ms(500);

    // Drain again in case more garbage arrived
    while (getchar_timeout_us(1000) != PICO_ERROR_TIMEOUT) {}

    // Flush stdio to sync state
    stdio_flush();

    // One more delay to let everything settle
    sleep_ms(200);
}

/**
 * @brief Wait for user to press Enter, or specific key
 * @param key Key to wait for (Enter = '\n' or '\r')
 * @return Character received
 */
int waitForKey() {
    int c;
    while ((c = getchar_timeout_us(10000)) == PICO_ERROR_TIMEOUT) {
        updateStatusLED();
    }
    // Clear any remaining input
    while (getchar_timeout_us(1000) != PICO_ERROR_TIMEOUT) {}
    return c;
}

void waitForEnter() {
    printf("Press Enter when ready...\n");
    fflush(stdout);
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

/**
 * @brief Print current calibration values
 */
void printCalibration(AP_InertialSensor& ins) {
    // Use ArduPilot Vector3f type (explicit namespace resolution)
    const ::Vector3f& offset = ins.get_accel_offset(0);
    const ::Vector3f& scale = ins.get_accel_scale(0);
    const ::Vector3f& offdiag = ins.get_accel_offdiag(0);

    printf("\nCalibration Parameters:\n");
    printf("-----------------------\n");
    printf("Offsets (g):\n");
    printf("  X: %+.6f\n", static_cast<double>(offset.x));
    printf("  Y: %+.6f\n", static_cast<double>(offset.y));
    printf("  Z: %+.6f\n", static_cast<double>(offset.z));

    printf("\nScale factors (diagonal):\n");
    printf("  X: %.6f\n", static_cast<double>(scale.x));
    printf("  Y: %.6f\n", static_cast<double>(scale.y));
    printf("  Z: %.6f\n", static_cast<double>(scale.z));

    printf("\nOff-diagonal (cross-axis):\n");
    printf("  XY: %.6f\n", static_cast<double>(offdiag.x));
    printf("  XZ: %.6f\n", static_cast<double>(offdiag.y));
    printf("  YZ: %.6f\n", static_cast<double>(offdiag.z));
}

int main() {
    // ========================================================================
    // CRITICAL: Do flash operations BEFORE USB is initialized!
    // Per RP2350_FULL_AP_PORT.md PD1+PD3: Flash ops make flash inaccessible,
    // which breaks TinyUSB interrupt handlers that live in flash.
    // Solution: Initialize HAL (which does storage.init() flash ops) BEFORE
    // calling stdio_init_all().
    // ========================================================================

    // Initialize NeoPixel early for visual feedback (doesn't need USB)
    g_statusLed = new WS2812(kNeoPixelPin, 1);
    if (!g_statusLed->begin()) {
        delete g_statusLed;
        g_statusLed = nullptr;
    } else {
        g_statusLed->setBrightness(kLedBrightnessPercent);
    }

    // Yellow = initializing (flash ops happening, USB not started yet)
    if (g_statusLed) {
        g_statusLed->setPixel(0, Colors::YELLOW);
        g_statusLed->show();
    }

    // Initialize HAL - this does storage.init() which performs flash operations
    // Do this BEFORE stdio_init_all() so USB isn't affected by flash ops
    hal.init();

    // CRITICAL: Clear BASEPRI before USB init
    // FreeRTOS or HAL init may leave BASEPRI elevated (configMAX_SYSCALL_INTERRUPT_PRIORITY=16)
    // which blocks USB interrupts (priority 0x80). Clear it so USB IRQs can fire.
    __asm volatile ("mov r0, #0\nmsr basepri, r0" ::: "r0");

    // NOW initialize USB - after all flash operations are complete
    stdio_init_all();

    // Initialize AP_Param and AP_InertialSensor AFTER USB is initialized
    // (these don't do flash ops, just setup)
    AP_Param::setup();
    AP_InertialSensor& ins = *AP_InertialSensor::get_singleton();
    ins.init();

    // Wait for USB connection - flash ops are done, USB should work properly
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
    sleep_ms(kUsbSettleTimeMs);

    printf("\n");
    printf("========================================\n");
    printf("  RocketChip Accelerometer Calibration\n");
    printf("  Using ArduPilot AccelCalibrator\n");
    printf("  WITH PERSISTENT STORAGE\n");
    printf("========================================\n\n");

    // Print HAL status (init happened before USB, so print it now)
    printf("HAL initialized: %s\n", hal.storage.healthy() ? "OK" : "WARNING: storage unhealthy");
    printf("Board: %s\n", HAL_BOARD_NAME);
    printf("Storage: %s\n", AP_Param::erased_on_boot() ? "freshly initialized" : "loaded from flash");
    fflush(stdout);

    g_currentState = CalibrationState::INITIALIZING;
    updateStatusLED();

    // Check if we have existing calibration
    if (ins.accel_calibrated(0)) {
        g_currentState = CalibrationState::LOADED_FROM_FLASH;
        updateStatusLED();

        printf("\n========================================\n");
        printf("EXISTING CALIBRATION FOUND IN FLASH!\n");
        printf("========================================\n");
        fflush(stdout);

        printCalibration(ins);
        fflush(stdout);

        printf("\n========================================\n");
        printf("Options:\n");
        printf("  [R] Recalibrate (run new calibration)\n");
        printf("  [E] Erase calibration\n");
        printf("  [T] Test corrected readings\n");
        printf("  [Any] Exit\n");
        printf("========================================\n");
        fflush(stdout);

        int choice = waitForKey();
        if (choice != 'r' && choice != 'R' &&
            choice != 'e' && choice != 'E' &&
            choice != 't' && choice != 'T') {
            printf("Exiting. Calibration preserved in flash.\n");
            while (true) {
                updateStatusLED();
                sleep_ms(50);
            }
        }

        if (choice == 'e' || choice == 'E') {
            printf("Erasing calibration...\n");
            ins.reset_calibration();
            printf("Calibration erased. Reboot to start fresh.\n");
            while (true) {
                g_currentState = CalibrationState::ERROR;
                updateStatusLED();
                sleep_ms(50);
            }
        }

        if (choice == 't' || choice == 'T') {
            // Test mode - show corrected readings
            printf("\nTest Mode - Showing corrected readings\n");
            printf("Press any key to exit\n\n");

            // Initialize I2C for sensors
            i2c_init(i2c1, kI2cSpeedHz);
            gpio_set_function(2, GPIO_FUNC_I2C);
            gpio_set_function(3, GPIO_FUNC_I2C);
            gpio_pull_up(2);
            gpio_pull_up(3);

            I2CBus imuBus(i2c1, 0x6A, 2, 3, kI2cSpeedHz);
            IMU_ISM330DHCX imu(&imuBus);
            imu.begin();
            imu.setAccelRange(AccelRange::RANGE_4G);
            imu.setODR(ODR::ODR_104HZ);

            while (getchar_timeout_us(100000) == PICO_ERROR_TIMEOUT) {
                rocketchip::hal::Vector3f accel, gyro;
                if (imu.read(accel, gyro)) {
                    // Raw reading (use ArduPilot Vector3f for calibration API)
                    ::Vector3f raw(accel.x, accel.y, accel.z);

                    // Corrected reading
                    ::Vector3f corrected = ins.correct_accel(raw, 0);

                    // Calculate magnitude (should be ~1g when stationary)
                    float raw_mag = sqrtf(raw.x*raw.x + raw.y*raw.y + raw.z*raw.z);
                    float cor_mag = sqrtf(corrected.x*corrected.x +
                                          corrected.y*corrected.y +
                                          corrected.z*corrected.z);

                    printf("Raw: [%+7.3f, %+7.3f, %+7.3f] |%5.3f|g  "
                           "Cor: [%+7.3f, %+7.3f, %+7.3f] |%5.3f|g\r",
                           raw.x, raw.y, raw.z, raw_mag,
                           corrected.x, corrected.y, corrected.z, cor_mag);
                    fflush(stdout);
                }
                updateStatusLED();
            }
            printf("\n");
            while (true) {
                updateStatusLED();
                sleep_ms(50);
            }
        }

        // Fall through to recalibration
        printf("\nStarting recalibration...\n");
    }

    // ========================================================================
    // Initialize I2C for sensors
    // ========================================================================

    i2c_init(i2c1, kI2cSpeedHz);
    gpio_set_function(2, GPIO_FUNC_I2C);
    gpio_set_function(3, GPIO_FUNC_I2C);
    gpio_pull_up(2);
    gpio_pull_up(3);

    // Initialize IMU
    I2CBus imuBus(i2c1, 0x6A, 2, 3, kI2cSpeedHz);
    IMU_ISM330DHCX imu(&imuBus);

    printf("Initializing IMU...\n");
    fflush(stdout);
    if (!imu.begin()) {
        printf("ERROR: IMU initialization failed!\n");
        fflush(stdout);
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
    fflush(stdout);

    // ========================================================================
    // Run Calibration
    // ========================================================================

    printf("Starting 6-position calibration...\n");
    printf("You will need to hold the device in 6 different orientations.\n");
    printf("Hold steady for 2 seconds in each position.\n\n");
    fflush(stdout);

    // Start calibration via AP_InertialSensor
    ins.accel_calibrate_start(kNumOrientations, kSampleDurationSec,
                              ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID);

    AccelCalibrator* calibrator = ins.get_calibrator(0);

    uint8_t orientation = 0;
    uint32_t sampleStartTime = 0;
    uint32_t lastSampleTime = 0;
    bool collectingSamples = false;

    while (calibrator->get_status() != ACCEL_CAL_SUCCESS &&
           calibrator->get_status() != ACCEL_CAL_FAILED) {

        // Check for timeout
        calibrator->check_for_timeout();

        // Prompt for next orientation if needed
        if (calibrator->get_status() == ACCEL_CAL_WAITING_FOR_ORIENTATION) {
            if (!collectingSamples && orientation < kNumOrientations) {
                g_currentState = CalibrationState::WAITING_FOR_ORIENTATION;
                updateStatusLED();

                printf("\n--- Orientation %d of %d ---\n", orientation + 1, kNumOrientations);
                printf("Place device: %s\n", kOrientationNames[orientation]);
                fflush(stdout);
                waitForEnter();

                printf("Collecting samples");
                fflush(stdout);

                calibrator->collect_sample();
                collectingSamples = true;
                sampleStartTime = Timing::millis32();
                lastSampleTime = sampleStartTime;
                g_currentState = CalibrationState::COLLECTING_SAMPLES;
            }
        }

        // Feed samples to calibrator at ~100Hz
        uint32_t now = Timing::millis32();
        if (collectingSamples && (now - lastSampleTime >= 10)) {
            rocketchip::hal::Vector3f accel, gyro;
            if (imu.read(accel, gyro)) {
                // Convert from RocketChip Vector3f to ArduPilot Vector3f
                // IMU returns acceleration in g units, AccelCalibrator expects m/s²
                // delta_velocity = accel_mss * dt = accel_g * GRAVITY_MSS * dt
                float dt = (now - lastSampleTime) / 1000.0f;
                ::Vector3f ap_accel(accel.x * GRAVITY_MSS * dt,
                                    accel.y * GRAVITY_MSS * dt,
                                    accel.z * GRAVITY_MSS * dt);
                calibrator->new_sample(ap_accel, dt);

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
            calibrator->get_status() == ACCEL_CAL_WAITING_FOR_ORIENTATION &&
            calibrator->get_num_samples_collected() > orientation) {
            printf(" Done!\n");
            orientation++;
            collectingSamples = false;
        }

        // Check if calibration is being processed
        if (calibrator->get_status() == ACCEL_CAL_COLLECTING_SAMPLE) {
            g_currentState = CalibrationState::COLLECTING_SAMPLES;
        }

        updateStatusLED();
        sleep_ms(1);
    }

    // ========================================================================
    // Report Results and Save
    // ========================================================================

    printf("\n\n========================================\n");

    if (calibrator->get_status() == ACCEL_CAL_SUCCESS) {
        g_currentState = CalibrationState::PROCESSING;
        updateStatusLED();

        printf("CALIBRATION SUCCESSFUL!\n");
        printf("========================================\n\n");

        printf("Fitness (mean squared residual): %.6f\n", calibrator->get_fitness());

        // Save calibration via AP_InertialSensor
        printf("\nSaving calibration to flash...\n");
        ins._acal_save_calibrations();
        printf("Calibration saved!\n");

        // Print the saved values
        printCalibration(ins);

        // Verify by reloading from flash
        printf("\n========================================\n");
        printf("VERIFICATION: Reloading calibration from flash...\n");

        // Reload calibration data from flash to verify persistence
        ins.load_calibration();

        if (ins.accel_calibrated(0)) {
            printf("SUCCESS: Calibration verified in flash!\n");
            printCalibration(ins);
            g_currentState = CalibrationState::SUCCESS;
        } else {
            printf("WARNING: Verification failed - calibration may not be saved\n");
            g_currentState = CalibrationState::ERROR;
        }

    } else {
        g_currentState = CalibrationState::ERROR;
        printf("CALIBRATION FAILED!\n");
        printf("========================================\n\n");
        printf("Status code: %d\n", calibrator->get_status());
        printf("Samples collected: %d\n", calibrator->get_num_samples_collected());
    }

    printf("\n========================================\n");

    // Keep LED status visible
    while (true) {
        updateStatusLED();
        sleep_ms(50);
    }

    return 0;
}
