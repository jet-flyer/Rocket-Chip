/**
 * @file compass_minimal_test.cpp
 * @brief Minimal test to isolate compass calibration crash
 */

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include "pico/stdlib.h"
#include <AP_HAL_RP2350/HAL_RP2350_Class.h>
#include <AP_Compass/CompassCalibrator.h>
#include <GCS_MAVLink/GCS.h>

extern RP2350::HAL_RP2350 hal;

// Test constants - same as compass_cal_synthetic_test.cpp
static constexpr float kTrueOffsetX = 100.0f;
static constexpr float kTrueOffsetY = 50.0f;
static constexpr float kTrueOffsetZ = -75.0f;
static constexpr float kFieldMagnitude = 450.0f;
static constexpr uint32_t kNumSamples = 350;
static constexpr float kOffsetTolerance = 5.0f;
static constexpr float kMaxFitness = 5.0f;

int main() {
    // CRITICAL: Initialize HAL BEFORE stdio_init_all()
    hal.init();

    // CRITICAL: Clear BASEPRI before USB init
    __asm volatile ("mov r0, #0\nmsr basepri, r0" ::: "r0");

    // Initialize stdio after HAL
    stdio_init_all();

    // Wait for USB connection
    while (!stdio_usb_connected()) {
        gpio_init(PICO_DEFAULT_LED_PIN);
        gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(200);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(200);
    }
    sleep_ms(500);

    printf("\n");
    printf("=================================================\n");
    printf("Compass Minimal Test - Crash Isolation\n");
    printf("=================================================\n\n");

    printf("Step 1: Basic printf\n");
    fflush(stdout);

    printf("Step 2: Float printf - kTrueOffsetX = %.1f\n", kTrueOffsetX);
    fflush(stdout);

    printf("Step 3: Multiple floats - (%.1f, %.1f, %.1f)\n",
           kTrueOffsetX, kTrueOffsetY, kTrueOffsetZ);
    fflush(stdout);

    printf("Step 4: kFieldMagnitude = %.1f\n", kFieldMagnitude);
    fflush(stdout);

    printf("Step 5: All printf with floats passed\n");
    fflush(stdout);

    printf("\nNow testing CompassCalibrator creation...\n");
    fflush(stdout);

    printf("Step 7: Creating CompassCalibrator...\n");
    fflush(stdout);

    CompassCalibrator calibrator;

    printf("Step 8: CompassCalibrator created!\n");
    fflush(stdout);

    printf("\n*** ALL TESTS PASSED ***\n");

    // Success - solid LED
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    while (true) {
        sleep_ms(1000);
    }

    return 0;
}
