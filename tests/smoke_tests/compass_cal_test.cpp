/**
 * @file compass_cal_test.cpp
 * @brief Interactive compass calibration test with persistent storage
 *
 * User physically rotates device through all orientations while the
 * CompassCalibrator collects samples for sphere/ellipsoid fitting.
 * Calibration results are saved to flash via CalibrationStore.
 *
 * BATTERY-POWERED OPERATION:
 * This test can run on battery without USB connected. All serial output
 * is buffered internally. When USB is connected (before or after cal),
 * the buffered log is dumped to the terminal.
 *
 * Hardware required:
 * - Adafruit Feather RP2350 HSTX
 * - ISM330DHCX + LIS3MDL FeatherWing
 *
 * NeoPixel Status Colors:
 * - Yellow pulsing: Initializing hardware
 * - RED slow blink: Waiting for user to start (press BOOT button)
 * - Red→Yellow→Green: Calibrating (color shows progress %)
 * - Green fast flash: Calibration at 100%, processing
 * - Green slow flash: Calibration successful (saved to flash)
 * - Red SOLID: Error (not blinking)
 * - Cyan solid: Existing calibration loaded from flash
 * - Magenta blink: Waiting for USB connection
 *
 * The calibration follows ArduPilot's CompassCalibrator algorithm:
 * - Step 1: Sphere fit (300 samples) - finds hard iron offset
 * - Step 2: Ellipsoid fit - finds soft iron correction
 *
 * Progress indication via NeoPixel brightness. Serial log available after.
 */

#include <cstdio>
#include <cstdarg>
#include <cmath>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// HAL includes (must come before ArduPilot HAL to set guard macros)
#include <AP_HAL_RP2350/HAL_RP2350_Class.h>

// ArduPilot libraries
#include <AP_Compass/AP_Compass.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

// RocketChip HAL includes
#include "HAL.h"
#include "Bus.h"
#include "Timing.h"
#include "PIO.h"
#include "Mag_LIS3MDL.h"

// Namespace alias to avoid Vector3f conflict with ArduPilot's Vector3f
namespace rh = rocketchip::hal;

// Global HAL reference is provided by AP_HAL_RP2350/HAL_RP2350_Class.h:
// extern const AP_HAL::HAL& hal;

// ============================================================================
// Log Buffer for Battery-Powered Operation
// ============================================================================

// Circular buffer for storing log output when USB not connected
static constexpr size_t kLogBufferSize = 16384;  // 16KB log buffer
static char g_logBuffer[kLogBufferSize];
static size_t g_logWritePos = 0;
static bool g_logWrapped = false;

/**
 * @brief Write to log buffer (thread-safe for single producer)
 */
void logWrite(const char* str) {
    while (*str) {
        g_logBuffer[g_logWritePos] = *str++;
        g_logWritePos++;
        if (g_logWritePos >= kLogBufferSize) {
            g_logWritePos = 0;
            g_logWrapped = true;
        }
    }
}

/**
 * @brief Printf-like logging to buffer
 */
void logPrintf(const char* fmt, ...) {
    char temp[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(temp, sizeof(temp), fmt, args);
    va_end(args);
    logWrite(temp);
}

/**
 * @brief Dump log buffer to USB serial
 */
void logDump() {
    printf("\n========== BUFFERED LOG START ==========\n");
    if (g_logWrapped) {
        // Buffer wrapped - print from writePos to end, then start to writePos
        for (size_t i = g_logWritePos; i < kLogBufferSize; i++) {
            if (g_logBuffer[i]) putchar(g_logBuffer[i]);
        }
    }
    // Print from start to writePos
    for (size_t i = 0; i < g_logWritePos; i++) {
        putchar(g_logBuffer[i]);
    }
    printf("\n=========== BUFFERED LOG END ===========\n");
    fflush(stdout);
}

/**
 * @brief Combined log - writes to buffer AND to USB if connected
 */
#define LOG(fmt, ...) do { \
    logPrintf(fmt, ##__VA_ARGS__); \
    if (stdio_usb_connected()) { \
        printf(fmt, ##__VA_ARGS__); \
        fflush(stdout); \
    } \
} while(0)

// ============================================================================
// Configuration Constants
// ============================================================================

// Hardware pins (per HARDWARE.md Feather RP2350 HSTX Built-in Hardware)
static constexpr uint8_t kNeoPixelPin = 21;
static constexpr uint8_t kI2cSda = 2;
static constexpr uint8_t kI2cScl = 3;
// Note: On Feather RP2350 HSTX, GPIO 7 is the RED LED, NOT a button.
// The BOOTSEL button is NOT accessible as GPIO on this board (it is on Pimoroni Tiny2350).
// For input without USB, use an external button on a GPIO pin, or pre-program the test.

// Timing constants (in milliseconds)
static constexpr uint32_t kUsbWaitBlinkMs = 200;
static constexpr uint32_t kUsbSettleTimeMs = 500;
static constexpr uint32_t kBlinkSlowPeriodMs = 500;
static constexpr uint32_t kBlinkFastPeriodMs = 100;
static constexpr uint32_t kAnimUpdatePeriodMs = 20;
static constexpr uint32_t kProgressUpdateMs = 500;
static constexpr uint32_t kButtonDebounceMs = 50;

// Magnetometer sampling
static constexpr uint32_t kMagSamplePeriodMs = 50;    // 20Hz sampling

// Calibration parameters
static constexpr float kCalTolerance = 10.0f;         // Fitness tolerance (relaxed for noisy environments)
static constexpr uint32_t kCalTimeoutMs = 180000;     // 3 minute timeout (more time for battery operation)

// ============================================================================
// Colors
// ============================================================================

static const rh::RGB kColorPurple(128, 0, 255);
static const rh::RGB kColorPurpleDim(64, 0, 128);
static const rh::RGB kColorCyan(0, 255, 255);

// ============================================================================
// Module State
// ============================================================================

static rh::WS2812* g_statusLed = nullptr;
static uint32_t g_lastAnimUpdate = 0;
static float g_pulsePhase = 0.0f;
static bool g_blinkState = false;

enum class CalibrationState {
    INITIALIZING,
    WAITING_TO_START,
    CALIBRATING,
    PROCESSING,
    SUCCESS,
    LOADED_FROM_FLASH,
    ERROR
};

static CalibrationState g_currentState = CalibrationState::INITIALIZING;

// Magnetometer driver and bus
static rh::I2CBus* g_i2cBus = nullptr;
static rh::Mag_LIS3MDL* g_mag = nullptr;

/**
 * @brief Update NeoPixel based on current state
 */
void updateStatusLED() {
    if (!g_statusLed) return;

    uint32_t now = rh::Timing::millis32();
    rh::RGB color;

    switch (g_currentState) {
        case CalibrationState::INITIALIZING:
            if (now - g_lastAnimUpdate >= kAnimUpdatePeriodMs) {
                g_pulsePhase += 0.1f;
                float brightness = (sinf(g_pulsePhase) + 1.0f) * 0.5f;
                color = rh::RGB(
                    static_cast<uint8_t>(255 * brightness),
                    static_cast<uint8_t>(255 * brightness),
                    0
                );
                g_lastAnimUpdate = now;
            }
            break;

        case CalibrationState::WAITING_TO_START:
            // Flash RED while waiting for BOOT button - easy to see
            if (now - g_lastAnimUpdate >= kBlinkSlowPeriodMs) {
                g_blinkState = !g_blinkState;
                g_lastAnimUpdate = now;
            }
            color = g_blinkState ? rh::Colors::RED : rh::Colors::OFF;
            break;

        case CalibrationState::CALIBRATING: {
            AP_Compass& compass = AP_Compass::get_singleton();
            float pct = compass.compass_cal_completion_pct() / 100.0f;

            if (pct >= 0.99f) {
                // At 100% - flash green rapidly to indicate completion
                if (now - g_lastAnimUpdate >= 50) {
                    g_blinkState = !g_blinkState;
                    g_lastAnimUpdate = now;
                }
                color = g_blinkState ? rh::Colors::GREEN : rh::RGB(0, 64, 0);
            } else {
                // Red → Yellow → Green gradient based on progress
                // 0% = red (255,0,0), 50% = yellow (255,255,0), 100% = green (0,255,0)
                uint8_t red, green;
                if (pct < 0.5f) {
                    // Red to Yellow (0-50%)
                    red = 255;
                    green = static_cast<uint8_t>(pct * 2.0f * 255);
                } else {
                    // Yellow to Green (50-100%)
                    red = static_cast<uint8_t>((1.0f - pct) * 2.0f * 255);
                    green = 255;
                }
                color = rh::RGB(red, green, 0);
            }
            break;
        }

        case CalibrationState::PROCESSING:
            if (now - g_lastAnimUpdate >= kBlinkFastPeriodMs) {
                g_blinkState = !g_blinkState;
                g_lastAnimUpdate = now;
            }
            color = g_blinkState ? kColorPurple : kColorPurpleDim;
            break;

        case CalibrationState::SUCCESS:
            if (now - g_lastAnimUpdate >= kBlinkFastPeriodMs) {
                g_blinkState = !g_blinkState;
                g_lastAnimUpdate = now;
            }
            color = g_blinkState ? rh::Colors::GREEN : rh::Colors::OFF;
            break;

        case CalibrationState::LOADED_FROM_FLASH:
            color = kColorCyan;
            break;

        case CalibrationState::ERROR:
            color = rh::Colors::RED;
            break;
    }

    g_statusLed->setPixel(0, color);
    g_statusLed->show();
}

/**
 * @brief Wait for USB CDC connection with visual feedback (optional)
 * @param timeout_ms Max time to wait, 0 = forever
 * @return true if USB connected, false if timeout
 *
 * For battery operation, use a short timeout so calibration can proceed
 * without USB. The log will be dumped when USB connects later.
 */
bool waitForUsb(uint32_t timeout_ms = 0) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    uint32_t start = rh::Timing::millis32();
    bool blink = false;

    while (!stdio_usb_connected()) {
        // Check timeout
        if (timeout_ms > 0 && (rh::Timing::millis32() - start) > timeout_ms) {
            return false;
        }

        // Blink both NeoPixel and onboard LED
        if (g_statusLed) {
            g_statusLed->setPixel(0, blink ? rh::RGB(255, 0, 255) : rh::Colors::OFF);
            g_statusLed->show();
        }
        gpio_put(PICO_DEFAULT_LED_PIN, blink ? 1 : 0);
        blink = !blink;

        sleep_ms(kUsbWaitBlinkMs);
    }
    sleep_ms(kUsbSettleTimeMs);
    return true;
}

/**
 * @brief Read magnetometer and convert to milliGauss
 * @param field Output: magnetic field in mGauss (ArduPilot Vector3f)
 * @return true if reading successful
 */
bool readMagnetometer(Vector3f& field) {
    if (!g_mag) {
        return false;
    }

    // Read into local Vector3f (rocketchip namespace)
    rh::Vector3f mag_gauss;
    if (!g_mag->read(mag_gauss)) {
        return false;
    }

    // Convert Gauss to milliGauss (CompassCalibrator expects mGauss)
    field.x = mag_gauss.x * 1000.0f;
    field.y = mag_gauss.y * 1000.0f;
    field.z = mag_gauss.z * 1000.0f;

    return true;
}

// ============================================================================
// Helper: Check for external button (not implemented - no accessible button on this board)
// On Feather RP2350 HSTX, BOOTSEL is not GPIO-accessible. To add button input:
// - Wire a button between GND and an unused GPIO (e.g., GPIO5)
// - Enable pull-up, read LOW for pressed
// ============================================================================

bool isExternalButtonPressed() {
    // No external button configured - always return false
    // To enable: gpio_init(pin), gpio_set_dir(IN), gpio_pull_up(pin), return !gpio_get(pin)
    return false;
}

// ============================================================================
// Main Entry Point
// ============================================================================

int main() {
    // Note: BOOTSEL button doesn't need GPIO init - it's read via get_bootsel_button()

    // Initialize NeoPixel early for visual feedback (PIO-based, doesn't need USB)
    g_statusLed = new rh::WS2812(kNeoPixelPin, 1);
    if (!g_statusLed->begin()) {
        // NeoPixel init failed - fall back to onboard LED
        gpio_init(PICO_DEFAULT_LED_PIN);
        gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);  // LED on = error indicator
    }
    g_statusLed->fill(rh::Colors::YELLOW);
    g_statusLed->show();

    // CRITICAL: Initialize HAL subsystems BEFORE stdio_init_all()
    // Flash operations in storage.init() conflict with USB CDC if USB is already running
    RP2350::hal_init();

    // CRITICAL: Clear BASEPRI before USB init
    // FreeRTOS or HAL init may leave BASEPRI elevated (configMAX_SYSCALL_INTERRUPT_PRIORITY=16)
    // which blocks USB interrupts (priority 0x80). Clear it so USB IRQs can fire.
    __asm volatile ("mov r0, #0\nmsr basepri, r0" ::: "r0");

    // Initialize stdio after HAL (USB now safe)
    stdio_init_all();

    // Wait briefly for USB (3 seconds) - if no USB, proceed anyway (battery mode)
    bool usbConnected = waitForUsb(3000);

    // Log to buffer (will be dumped when USB connects, either now or later)
    LOG("\n");
    LOG("=================================================\n");
    LOG("Compass Calibration Test (LIS3MDL + AP_Compass)\n");
    LOG("Build: v16-offset-fix\n");
    LOG("=================================================\n\n");

    LOG("HAL initialized.\n");
    LOG("USB: %s\n", usbConnected ? "connected" : "not connected (battery mode)");

    // Initialize GCS for MAVLink STATUSTEXT
    GCS::get_singleton().init();

    g_currentState = CalibrationState::INITIALIZING;
    updateStatusLED();

    // Initialize I2C bus for magnetometer
    LOG("Initializing I2C bus...\n");
    g_i2cBus = new rh::I2CBus(i2c1, rh::Mag_LIS3MDL::I2C_ADDR_DEFAULT, kI2cSda, kI2cScl);

    // Initialize magnetometer
    LOG("Initializing LIS3MDL magnetometer...\n");
    g_mag = new rh::Mag_LIS3MDL(g_i2cBus);
    if (!g_mag->begin()) {
        LOG("ERROR: Failed to initialize LIS3MDL!\n");
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Compass: LIS3MDL init failed");
        g_currentState = CalibrationState::ERROR;
        while (true) {
            updateStatusLED();
            sleep_ms(100);
        }
    }
    LOG("  LIS3MDL initialized.\n");

    // Configure magnetometer for calibration
    g_mag->setDataRate(rh::MagDataRate::HP_80Hz);
    g_mag->setRange(rh::MagRange::RANGE_4GAUSS);

    // Verify magnetometer is reading
    Vector3f initial_reading;
    if (!readMagnetometer(initial_reading)) {
        LOG("ERROR: Cannot read from magnetometer!\n");
        g_currentState = CalibrationState::ERROR;
        while (true) {
            updateStatusLED();
            sleep_ms(100);
        }
    }
    LOG("  Initial reading: (%.1f, %.1f, %.1f) mGauss\n",
           initial_reading.x, initial_reading.y, initial_reading.z);
    LOG("  Field magnitude: %.1f mGauss\n", initial_reading.length());

    // Initialize AP_Compass
    LOG("\nInitializing AP_Compass...\n");
    AP_Compass& compass = AP_Compass::get_singleton();
    compass.init();

    // Check if calibration already exists
    if (compass.compass_calibrated(0)) {
        LOG("\nExisting calibration found in flash!\n");
        const Vector3f& offsets = compass.get_offsets(0);
        const Vector3f& diag = compass.get_diagonals(0);
        const Vector3f& offdiag = compass.get_offdiagonals(0);
        float scale = compass.get_scale_factor(0);

        LOG("  Offset: (%.1f, %.1f, %.1f) mGauss\n",
               offsets.x, offsets.y, offsets.z);
        LOG("  Diagonal: (%.4f, %.4f, %.4f)\n",
               diag.x, diag.y, diag.z);
        LOG("  Off-diagonal: (%.4f, %.4f, %.4f)\n",
               offdiag.x, offdiag.y, offdiag.z);
        LOG("  Scale: %.4f\n", scale);

        // Show corrected reading
        Vector3f raw;
        readMagnetometer(raw);
        Vector3f corrected = compass.correct_field(raw, 0);
        LOG("\nCurrent reading (corrected):\n");
        LOG("  Raw: (%.1f, %.1f, %.1f) mGauss, |mag|=%.1f\n",
               raw.x, raw.y, raw.z, raw.length());
        LOG("  Corrected: (%.1f, %.1f, %.1f) mGauss, |mag|=%.1f\n",
               corrected.x, corrected.y, corrected.z, corrected.length());

        LOG("\nPress ENTER to recalibrate...\n");
        g_currentState = CalibrationState::LOADED_FROM_FLASH;
    } else {
        LOG("\nNo existing calibration found.\n");
        LOG("Press ENTER to start calibration...\n");
        g_currentState = CalibrationState::WAITING_TO_START;
    }

    // Wait for user input OR auto-start after timeout (for battery operation)
    static constexpr uint32_t kAutoStartTimeoutMs = 10000;  // 10 seconds

    if (usbConnected) {
        LOG("Press ENTER to start calibration...\n");
    } else {
        LOG("No USB - auto-starting in 10 seconds...\n");
    }

    uint32_t waitStartTime = rh::Timing::millis32();
    while (true) {
        updateStatusLED();

        // Auto-start after timeout if no USB connected (battery mode)
        if (!usbConnected && (rh::Timing::millis32() - waitStartTime) >= kAutoStartTimeoutMs) {
            LOG(">>> Auto-starting calibration (no USB)\n");
            break;
        }

        // Check USB serial input (if connected)
        if (usbConnected) {
            int c = getchar_timeout_us(50000);  // 50ms timeout
            if (c != PICO_ERROR_TIMEOUT && (c == '\n' || c == '\r')) {
                LOG(">>> ENTER received - starting calibration!\n");
                break;
            }
        }

        sleep_ms(50);
    }

    // Start calibration
    LOG("\n");
    LOG("=================================================\n");
    LOG("Starting Compass Calibration\n");
    LOG("=================================================\n\n");

    LOG("Rotate the device slowly through all orientations:\n");
    LOG("  - All 6 faces (top, bottom, 4 sides)\n");
    LOG("  - Multiple angles between faces\n");
    LOG("  - Try to cover the entire sphere of orientations\n\n");

    if (!compass.compass_calibrate_start(0, kCalTolerance)) {
        LOG("ERROR: Failed to start calibration!\n");
        g_currentState = CalibrationState::ERROR;
        while (true) {
            updateStatusLED();
            sleep_ms(100);
        }
    }

    g_currentState = CalibrationState::CALIBRATING;

    // Calibration loop
    uint32_t lastSampleTime = 0;
    uint32_t lastProgressTime = 0;
    uint32_t calStartTime = rh::Timing::millis32();
    float lastPct = 0.0f;

    LOG("Collecting samples... (rotate device continuously)\n\n");
    uint32_t sampleCount = 0;

    while (compass.compass_calibrating()) {
        uint32_t now = rh::Timing::millis32();

        // Check timeout
        if (now - calStartTime > kCalTimeoutMs) {
            LOG("\nTimeout! Calibration took too long.\n");
            compass.compass_calibrate_cancel();
            g_currentState = CalibrationState::ERROR;
            break;
        }

        // Sample magnetometer at fixed rate
        if (now - lastSampleTime >= kMagSamplePeriodMs) {
            Vector3f sample;
            if (readMagnetometer(sample)) {
                compass.compass_cal_new_sample(sample);
                sampleCount++;
            }
            lastSampleTime = now;
        }

        // Update calibrator state machine
        compass.compass_cal_update();

        // Log progress every 5 seconds (less frequent since we have NeoPixel)
        float pct = compass.compass_cal_completion_pct();
        if (now - lastProgressTime >= 5000) {
            LOG("  Progress: %.0f%% (samples: %lu)\n", pct, (unsigned long)sampleCount);
            lastPct = pct;
            lastProgressTime = now;
        }

        updateStatusLED();
        sleep_ms(1);
    }

    LOG("Calibration loop complete. Samples fed: %lu\n", (unsigned long)sampleCount);

    // Check result
    if (compass.compass_cal_success()) {
        LOG("\n\n");
        LOG("=================================================\n");
        LOG("Calibration Successful!\n");
        LOG("=================================================\n\n");

        g_currentState = CalibrationState::SUCCESS;

        const Vector3f& offsets = compass.get_offsets(0);
        const Vector3f& diag = compass.get_diagonals(0);
        const Vector3f& offdiag = compass.get_offdiagonals(0);
        float scale = compass.get_scale_factor(0);

        LOG("Calibration Results:\n");
        LOG("  Offset: (%.1f, %.1f, %.1f) mGauss\n",
               offsets.x, offsets.y, offsets.z);
        LOG("  Diagonal: (%.4f, %.4f, %.4f)\n",
               diag.x, diag.y, diag.z);
        LOG("  Off-diagonal: (%.4f, %.4f, %.4f)\n",
               offdiag.x, offdiag.y, offdiag.z);
        LOG("  Scale: %.4f\n", scale);
        LOG("  Fitness: %.2f\n", compass.compass_cal_fitness());

        // Save to flash
        LOG("\nSaving calibration to flash...\n");
        if (compass._cal_save_calibrations()) {
            LOG("  Calibration saved successfully!\n");
        } else {
            LOG("  WARNING: Failed to save calibration!\n");
        }

        // Show corrected readings
        LOG("\nTesting calibration (sample corrected readings):\n");
        for (int i = 0; i < 10; i++) {
            Vector3f raw;
            if (readMagnetometer(raw)) {
                Vector3f corrected = compass.correct_field(raw, 0);
                LOG("  Raw: %6.0f,%6.0f,%6.0f |%.0f|  Corr: %6.0f,%6.0f,%6.0f |%.0f|\n",
                       raw.x, raw.y, raw.z, raw.length(),
                       corrected.x, corrected.y, corrected.z, corrected.length());
            }
            sleep_ms(200);
            updateStatusLED();
        }

    } else if (g_currentState != CalibrationState::ERROR) {
        LOG("\n\nCalibration failed.\n");
        g_currentState = CalibrationState::ERROR;
    }

    LOG("\n=================================================\n");
    LOG("Test complete. Connect USB to view full log.\n");
    LOG("=================================================\n");

    // Final state - wait for USB to dump log, then stay showing result
    bool logDumped = false;
    while (true) {
        updateStatusLED();

        // If USB just connected and we haven't dumped yet, dump the log
        if (stdio_usb_connected() && !logDumped) {
            sleep_ms(kUsbSettleTimeMs);  // Let terminal connect
            logDump();
            logDumped = true;
            printf("\n[Real-time from here. LED shows result.]\n");
            fflush(stdout);
        }

        // No physical button on this board - log can be re-dumped by reconnecting terminal

        sleep_ms(100);
    }

    return 0;
}
