/**
 * @file compass_cal_synthetic_test.cpp
 * @brief Synthetic compass calibration test
 *
 * Verifies CompassCalibrator's sphere/ellipsoid fitting algorithm
 * works correctly using synthetic data with known offsets.
 *
 * This is a software-only test - no magnetometer hardware required.
 * Validates the calibration algorithm before testing with real sensors.
 *
 * Test procedure:
 * 1. Generate 300+ samples on a sphere centered at known offset
 * 2. Feed samples to CompassCalibrator
 * 3. Run calibration to completion
 * 4. Verify recovered offset matches known offset within tolerance
 */

#include <cstdio>
#include <cmath>
#include <cstdlib>

#include "pico/stdlib.h"
#include "FreeRTOS.h"  // For xPortGetFreeHeapSize

// AP_HAL for HAL initialization and timing
#include <AP_HAL_RP2350/HAL_RP2350_Class.h>

// CompassCalibrator
#include <AP_Compass/CompassCalibrator.h>

// AP_Math for Vector3f
#include <AP_Math/AP_Math.h>

// GCS for STATUSTEXT output
#include <GCS_MAVLink/GCS.h>

// HAL reference is declared in HAL_RP2350_Class.h as:
// extern const AP_HAL::HAL& hal;

// Static calibrator to avoid stack allocation issues
static CompassCalibrator g_calibrator;

// ============================================================================
// Test Configuration
// ============================================================================

// Known hard iron offset to simulate (in milliGauss)
// These will be recovered by the calibrator
static constexpr float kTrueOffsetX = 100.0f;
static constexpr float kTrueOffsetY = 50.0f;
static constexpr float kTrueOffsetZ = -75.0f;

// Simulated Earth field magnitude (typical: 250-650 mGauss)
static constexpr float kFieldMagnitude = 450.0f;

// Number of samples to generate (must be >= 300 per CompassCalibrator)
static constexpr uint32_t kNumSamples = 350;

// Acceptance tolerance (in mGauss)
// CompassCalibrator should recover offsets within this tolerance
static constexpr float kOffsetTolerance = 5.0f;

// Maximum acceptable fitness (RMS error)
static constexpr float kMaxFitness = 5.0f;

// Timing
static constexpr uint32_t kUsbWaitBlinkMs = 200;
static constexpr uint32_t kUsbSettleTimeMs = 500;

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Generate a random float in range [-1, 1]
 */
static float random_normalized() {
    return 2.0f * ((float)rand() / RAND_MAX) - 1.0f;
}

/**
 * @brief Generate a random unit vector on a sphere
 * Uses rejection sampling for uniform distribution
 */
static Vector3f random_unit_vector() {
    Vector3f v;
    float len_sq;

    do {
        v.x = random_normalized();
        v.y = random_normalized();
        v.z = random_normalized();
        len_sq = v.x*v.x + v.y*v.y + v.z*v.z;
    } while (len_sq < 0.01f || len_sq > 1.0f);

    float len = sqrtf(len_sq);
    v.x /= len;
    v.y /= len;
    v.z /= len;

    return v;
}

/**
 * @brief Generate a synthetic magnetometer sample
 *
 * Creates a sample as if from a magnetometer with hard iron offset.
 * The "true" field is a point on a sphere of radius kFieldMagnitude,
 * offset by the hard iron bias.
 *
 * @param direction Unit vector in Earth field direction
 * @return Simulated magnetometer reading (mGauss)
 */
static Vector3f generate_sample(const Vector3f& direction) {
    // True Earth field (no offset)
    Vector3f field = direction * kFieldMagnitude;

    // Add hard iron offset (simulates sensor bias)
    field.x += kTrueOffsetX;
    field.y += kTrueOffsetY;
    field.z += kTrueOffsetZ;

    // Add small noise (simulates sensor noise)
    field.x += random_normalized() * 2.0f;
    field.y += random_normalized() * 2.0f;
    field.z += random_normalized() * 2.0f;

    return field;
}

/**
 * @brief Convert calibrator status to string
 */
static const char* status_to_string(CompassCalibrator::Status status) {
    switch (status) {
        case CompassCalibrator::Status::NOT_STARTED: return "NOT_STARTED";
        case CompassCalibrator::Status::WAITING_TO_START: return "WAITING_TO_START";
        case CompassCalibrator::Status::RUNNING_STEP_ONE: return "RUNNING_STEP_ONE";
        case CompassCalibrator::Status::RUNNING_STEP_TWO: return "RUNNING_STEP_TWO";
        case CompassCalibrator::Status::SUCCESS: return "SUCCESS";
        case CompassCalibrator::Status::FAILED: return "FAILED";
        case CompassCalibrator::Status::BAD_ORIENTATION: return "BAD_ORIENTATION";
        case CompassCalibrator::Status::BAD_RADIUS: return "BAD_RADIUS";
        default: return "UNKNOWN";
    }
}

// ============================================================================
// Main Test
// ============================================================================

int main() {
    // CRITICAL: Initialize HAL subsystems BEFORE stdio_init_all()
    // Flash operations in storage.init() conflict with USB CDC if USB is already running
    // See REBUILD_CONTEXT.md for details on flash/USB interaction
    RP2350::hal_init();

    // CRITICAL: Clear BASEPRI before USB init
    // FreeRTOS or HAL init may leave BASEPRI elevated (configMAX_SYSCALL_INTERRUPT_PRIORITY=16)
    // which blocks USB interrupts (priority 0x80). Clear it so USB IRQs can fire.
    __asm volatile ("mov r0, #0\nmsr basepri, r0" ::: "r0");

    // Initialize stdio after HAL (USB now safe)
    stdio_init_all();

    // Initialize random seed with time
    srand(time_us_32());

    // Wait for USB connection (per DEBUG_OUTPUT.md pattern)
    while (!stdio_usb_connected()) {
        gpio_init(PICO_DEFAULT_LED_PIN);
        gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(kUsbWaitBlinkMs);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(kUsbWaitBlinkMs);
    }
    sleep_ms(kUsbSettleTimeMs);

    printf("\n");
    printf("=================================================\n");
    printf("Compass Calibration Synthetic Test\n");
    printf("Build: v3-static-calibrator\n");
    printf("=================================================\n\n");

    printf("HAL initialized.\n");

    // TEMPORARILY DISABLED for debugging crash
    // GCS::get_singleton().init();

    // Print test parameters
    printf("\nTest Parameters:\n");
    printf("  True offset: (%.1f, %.1f, %.1f) mGauss\n",
           kTrueOffsetX, kTrueOffsetY, kTrueOffsetZ);
    printf("  Field magnitude: %.1f mGauss\n", kFieldMagnitude);
    printf("  Number of samples: %lu\n", (unsigned long)kNumSamples);
    printf("  Offset tolerance: %.1f mGauss\n", kOffsetTolerance);
    printf("  Max fitness: %.1f\n", kMaxFitness);
    fflush(stdout);

    // Using static calibrator (g_calibrator) to avoid stack allocation issues
    printf("Using static CompassCalibrator...\n");
    fflush(stdout);
    printf("  CompassCalibrator ready.\n");
    fflush(stdout);

    // Start calibration
    // Parameters: retry=false, delay=0, offset_max=1800, compass_idx=0, tolerance=5.0
    printf("Starting calibration...\n");
    fflush(stdout);
    g_calibrator.start(false, 0.0f, 1800, 0, kMaxFitness);
    printf("  Calibration started.\n");
    fflush(stdout);

    // Set orientation (ROTATION_NONE, not external, don't fix, don't always_45_deg)
    g_calibrator.set_orientation(ROTATION_NONE, false, false, false);

    // Feed samples
    printf("\nFeeding %lu samples...\n", (unsigned long)kNumSamples);

    CompassCalibrator::Status last_status = CompassCalibrator::Status::NOT_STARTED;
    uint32_t sample_count = 0;

    for (uint32_t i = 0; i < kNumSamples; i++) {
        // Generate random direction for this sample
        Vector3f direction = random_unit_vector();

        // Generate synthetic sample with hard iron offset
        Vector3f sample = generate_sample(direction);

        // Feed to calibrator
        g_calibrator.new_sample(sample);

        // Update calibrator state machine
        g_calibrator.update();

        // Check for status changes
        CompassCalibrator::State state = g_calibrator.get_state();
        if (state.status != last_status) {
            printf("  Status: %s -> %s (sample %lu)\n",
                   status_to_string(last_status),
                   status_to_string(state.status),
                   (unsigned long)i);
            last_status = state.status;
        }

        // Print progress every 100 samples (reduced to minimize heap pressure from printf)
        if ((i + 1) % 100 == 0) {
            printf("  Samples: %lu, Completion: %.0f%%, Free heap: %lu\n",
                   (unsigned long)(i + 1), state.completion_pct,
                   (unsigned long)xPortGetFreeHeapSize());
            fflush(stdout);
        }

        sample_count++;

        // Small delay to simulate real-time sampling
        sleep_ms(1);
    }

    // Continue feeding samples and updating until calibration completes
    // After thin_samples(), we need more samples to reach 300 again for fitting to continue
    printf("\nContinuing calibration...\n");
    fflush(stdout);
    uint32_t timeout_ms = 60000;  // 60 seconds
    uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    uint32_t loop_count = 0;

    while (g_calibrator.running()) {
        // Feed a new sample every iteration
        Vector3f direction = random_unit_vector();
        Vector3f sample = generate_sample(direction);
        g_calibrator.new_sample(sample);
        g_calibrator.update();

        // Check for status changes
        CompassCalibrator::State state = g_calibrator.get_state();
        if (state.status != last_status) {
            printf("  Status: %s -> %s\n",
                   status_to_string(last_status),
                   status_to_string(state.status));
            fflush(stdout);
            last_status = state.status;
        }

        // Print progress occasionally (every 500 iterations)
        if (loop_count % 500 == 0 && loop_count > 0) {
            printf("  Completion: %.0f%% (loop %lu)\n", state.completion_pct, (unsigned long)loop_count);
            fflush(stdout);
        }

        loop_count++;
        sleep_ms(1);

        if (to_ms_since_boot(get_absolute_time()) - start_ms > timeout_ms) {
            printf("  TIMEOUT after %lu loops\n", (unsigned long)loop_count);
            fflush(stdout);
            break;
        }
    }

    // Get results
    CompassCalibrator::Report report = g_calibrator.get_report();

    printf("\n=================================================\n");
    printf("Calibration Results\n");
    printf("=================================================\n\n");

    printf("Status: %s\n", status_to_string(report.status));
    printf("Fitness: %.2f\n", report.fitness);
    printf("\nRecovered offset: (%.1f, %.1f, %.1f) mGauss\n",
           report.ofs.x, report.ofs.y, report.ofs.z);
    // Note: CompassCalibrator reports the CORRECTION offset (negative of hard iron)
    // So we expect report.ofs ≈ -kTrueOffset
    printf("Expected offset:  (%.1f, %.1f, %.1f) mGauss (negative of hard iron)\n",
           -kTrueOffsetX, -kTrueOffsetY, -kTrueOffsetZ);

    // Calculate error (comparing to negative of true offset)
    float error_x = fabsf(report.ofs.x - (-kTrueOffsetX));
    float error_y = fabsf(report.ofs.y - (-kTrueOffsetY));
    float error_z = fabsf(report.ofs.z - (-kTrueOffsetZ));
    float max_error = fmaxf(fmaxf(error_x, error_y), error_z);

    printf("\nOffset error: (%.2f, %.2f, %.2f) mGauss\n", error_x, error_y, error_z);
    printf("Max error: %.2f mGauss\n", max_error);

    printf("\nDiagonal scale: (%.4f, %.4f, %.4f)\n",
           report.diag.x, report.diag.y, report.diag.z);
    printf("Off-diagonal: (%.4f, %.4f, %.4f)\n",
           report.offdiag.x, report.offdiag.y, report.offdiag.z);
    printf("Scale factor: %.4f\n", report.scale_factor);

    // Evaluate test
    printf("\n=================================================\n");
    printf("Test Evaluation\n");
    printf("=================================================\n\n");

    bool passed = true;

    if (report.status != CompassCalibrator::Status::SUCCESS) {
        printf("FAIL: Calibration did not succeed (status: %s)\n",
               status_to_string(report.status));
        passed = false;
    }

    if (report.fitness > kMaxFitness) {
        printf("FAIL: Fitness %.2f exceeds max %.2f\n", report.fitness, kMaxFitness);
        passed = false;
    }

    if (max_error > kOffsetTolerance) {
        printf("FAIL: Offset error %.2f exceeds tolerance %.2f\n",
               max_error, kOffsetTolerance);
        passed = false;
    }

    if (passed) {
        printf("PASS: All checks passed!\n");
        printf("  - Calibration succeeded\n");
        printf("  - Fitness %.2f < %.2f\n", report.fitness, kMaxFitness);
        printf("  - Offset error %.2f < %.2f mGauss\n", max_error, kOffsetTolerance);
    }

    // Send result via MAVLink (TEMPORARILY DISABLED for debugging)
    // if (passed) {
    //     GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Compass synthetic cal: PASS");
    // } else {
    //     GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Compass synthetic cal: FAIL");
    // }

    printf("\n=================================================\n");
    printf("Test complete. LED indicates result.\n");
    printf("=================================================\n");

    // Visual indication
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    while (true) {
        if (passed) {
            // Solid LED for success
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(1000);
        } else {
            // Fast blink for failure
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(100);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(100);
        }
    }

    return 0;
}
