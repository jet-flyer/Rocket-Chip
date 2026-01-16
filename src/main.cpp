/**
 * @file main.cpp
 * @brief RocketChip Production Entry Point
 *
 * This is the main entry point for the RocketChip production firmware.
 * It initializes the hardware abstraction layer, creates the initial
 * FreeRTOS tasks, and starts the scheduler.
 *
 * Phase: 2 (Sensors)
 * Status: Production implementation (replaces validation main.c)
 *
 * Architecture:
 * - Core 0: UITask (future)
 * - Core 1: SensorTask (real-time, 1kHz)
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// HAL
#include "HAL.h"

// Services
#include "services/SensorTask.h"

using namespace rocketchip::hal;

/**
 * @brief LED blink pattern for status indication
 */
static void blinkPattern(uint8_t count, uint32_t on_ms, uint32_t off_ms) {
    gpio_init(FeatherRP2350::LED);
    gpio_set_dir(FeatherRP2350::LED, GPIO_OUT);

    for (uint8_t i = 0; i < count; i++) {
        gpio_put(FeatherRP2350::LED, 1);
        busy_wait_ms(on_ms);
        gpio_put(FeatherRP2350::LED, 0);
        if (i < count - 1) {
            busy_wait_ms(off_ms);
        }
    }
}

/**
 * @brief Fatal error handler
 */
static void fatalError(const char* message) {
    printf("\n!!! FATAL ERROR: %s !!!\n", message);

    // Rapid blink forever
    gpio_init(FeatherRP2350::LED);
    gpio_set_dir(FeatherRP2350::LED, GPIO_OUT);

    while (1) {
        gpio_put(FeatherRP2350::LED, 1);
        busy_wait_ms(100);
        gpio_put(FeatherRP2350::LED, 0);
        busy_wait_ms(100);
    }
}

/**
 * @brief Main entry point
 */
int main(void) {
    // Initialize LED for early debugging
    gpio_init(FeatherRP2350::LED);
    gpio_set_dir(FeatherRP2350::LED, GPIO_OUT);

    // Quick blink pattern: 3 fast blinks = reached main()
    blinkPattern(3, 100, 100);

    // Initialize USB serial
    stdio_init_all();
    sleep_ms(2000); // Wait for USB enumeration

    // 2 slow blinks = USB ready
    blinkPattern(2, 250, 250);

    printf("\n\n");
    printf("========================================\n");
    printf("RocketChip Production Firmware\n");
    printf("Target: Adafruit Feather RP2350 HSTX\n");
    printf("Platform: FreeRTOS SMP (dual-core)\n");
    printf("Phase: 2 (Sensors)\n");
    printf("========================================\n\n");

    // Check reset reason
    ResetReason reset_reason = getResetReason();
    printf("Reset reason: %s\n\n", getResetReasonString(reset_reason));

    if (reset_reason == ResetReason::WATCHDOG) {
        printf("WARNING: System recovered from watchdog reset!\n\n");
    }

    // Get platform info
    PlatformInfo platform = getPlatformInfo();
    printf("Platform Information:\n");
    printf("  Chip:        %s\n", platform.chip_name);
    printf("  Board:       %s\n", platform.board_name);
    printf("  CPU:         %lu MHz\n", platform.cpu_freq_mhz);
    printf("  Cores:       %u\n", platform.num_cores);
    printf("  Flash:       %lu bytes\n", platform.flash_size_bytes);
    printf("  SRAM:        %lu bytes\n", platform.sram_size_bytes);
    printf("  PSRAM:       %lu bytes\n\n", platform.psram_size_bytes);

    // Initialize HAL
    printf("Initializing HAL...\n");
    HALInitResult hal_result = initHAL();
    if (!hal_result.success) {
        printf("ERROR: HAL initialization failed: %s\n", hal_result.error_msg);
        fatalError("HAL init failed");
    }
    printf("HAL initialized successfully\n\n");

    // Get HAL version
    HALVersion hal_version = getHALVersion();
    printf("HAL Version: %u.%u.%u (built %s)\n\n",
           hal_version.major, hal_version.minor, hal_version.patch,
           hal_version.build_date);

    // One long blink = About to create tasks
    gpio_put(FeatherRP2350::LED, 1);
    sleep_ms(500);
    gpio_put(FeatherRP2350::LED, 0);
    sleep_ms(500);

    // Create SensorTask
    printf("Creating SensorTask...\n");
    TaskHandle_t sensor_task = SensorTask_Create();
    if (sensor_task == nullptr) {
        fatalError("Failed to create SensorTask");
    }
    printf("SensorTask created successfully\n\n");

    // Future tasks will be created here:
    // - FusionTask (Phase 4)
    // - MissionTask (Phase 5)
    // - LoggerTask (Phase 6)
    // - TelemetryTask (Phase 7)
    // - UITask (Phase 8)

    printf("Starting FreeRTOS scheduler...\n\n");

    // Two long blinks = About to start scheduler
    blinkPattern(2, 500, 500);

    // Enable watchdog (5s timeout) for production
    // Note: Disabled for now during development
    // watchdog_enable(5000, 1);

    // Start the scheduler
    vTaskStartScheduler();

    // Should never reach here
    fatalError("Scheduler returned");

    return 0;
}
