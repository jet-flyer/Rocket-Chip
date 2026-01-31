/**
 * @file main.cpp
 * @brief RocketChip Production Entry Point
 *
 * Production firmware entry point. Initializes HAL, creates FreeRTOS tasks,
 * and starts the scheduler.
 *
 * Current functionality (Phase 2):
 * - SensorTask: Real sensor sampling at 1kHz (IMU), 50Hz (baro)
 * - CLITask: Terminal-based UI with calibration menus
 *
 * @note Part of RocketChip - Modular Motion Tracking Platform
 * @see docs/SAD.md for architecture details
 */

#include <cstdio>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include "services/SensorTask.h"
#include "AP_InternalError/AP_InternalError.h"
#include <GCS_MAVLink/GCS.h>
#include "cli/RC_OS.h"

using namespace rocketchip::services;

// ============================================================================
// Configuration Constants (k prefix per JSF AV naming conventions)
// ============================================================================

static constexpr uint8_t kLedPin = PICO_DEFAULT_LED_PIN;

static constexpr uint32_t kCliTaskPriority = 1;
static constexpr uint32_t kCliStackSize = configMINIMAL_STACK_SIZE * 4;

// MAVLink start markers
static constexpr uint8_t kMavlinkV1Start = 0xFE;
static constexpr uint8_t kMavlinkV2Start = 0xFD;

// ============================================================================
// Menu State Machine
// ============================================================================

enum class MenuMode {
    Main,
    Calibration
};

static MenuMode g_menuMode = MenuMode::Main;

// ============================================================================
// Status Print Helper
// ============================================================================

static void printSystemStatus() {
    printf("\n========================================\n");
    printf("  RocketChip System Status\n");
    printf("========================================\n");
    printf("  Target: Adafruit Feather RP2350\n");
    printf("  Phase: 2 (Sensors)\n");
    printf("  Heap free: %u bytes\n", xPortGetFreeHeapSize());
    printf("  Uptime: %llu ms\n", to_ms_since_boot(get_absolute_time()));
    printf("----------------------------------------\n");
    printf("Commands:\n");
    printf("  h - This help\n");
    printf("  s - Sensor status\n");
    printf("  c - Calibration menu\n");
    printf("========================================\n\n");
}

static void printCalibrationMenu() {
    printf("\n========================================\n");
    printf("  Calibration Menu\n");
    printf("========================================\n");
    printf("  w - Full calibration wizard\n");
    printf("  l - Level calibration (quick accel)\n");
    printf("  a - 6-position accel calibration\n");
    printf("  g - Gyro calibration\n");
    printf("  m - Compass calibration\n");
    printf("  b - Baro calibration\n");
    printf("  x - Return to main menu\n");
    printf("========================================\n\n");
}

static void runCalibrationWizard() {
    printf("\n=== Calibration Wizard ===\n");
    printf("This wizard will guide you through all calibrations.\n\n");

    // Step 1: Level cal (quick, single position) - via MAVLink path
    printf("Step 1: Level Calibration (quick)\n");
    printf("Place device FLAT and STILL on a level surface.\n");
    printf("Press ENTER when ready, or 'x' to skip...\n");

    while (true) {
        int c = getchar_timeout_us(0);
        if (c == '\r' || c == '\n') {
            while (getchar_timeout_us(1000) != PICO_ERROR_TIMEOUT) {}
            printf("Calibrating...");
            fflush(stdout);
            MAV_RESULT result = RC_OS::cmd_level_cal();  // MAVLink path
            if (result == MAV_RESULT_ACCEPTED) {
                printf(" OK!\n\n");
            } else {
                printf(" FAILED (%d)\n\n", result);
            }
            break;
        } else if (c == 'x' || c == 'X') {
            printf("Skipped.\n\n");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Step 2: 6-position accel cal (full, more accurate) - via MAVLink path
    printf("Step 2: 6-Position Accel Calibration\n");
    printf("You will place device in 6 orientations.\n");
    printf("Press ENTER to start, or 'x' to skip...\n");

    while (true) {
        int c = getchar_timeout_us(0);
        if (c == '\r' || c == '\n') {
            while (getchar_timeout_us(1000) != PICO_ERROR_TIMEOUT) {}
            MAV_RESULT result = RC_OS::cmd_accel_cal_6pos();  // MAVLink path
            if (result == MAV_RESULT_ACCEPTED) {
                printf("Follow position prompts. Press ENTER for each, 'x' to cancel.\n");
                while (SensorTask_IsCalibrating()) {
                    int cmd = getchar_timeout_us(0);
                    if (cmd == 'x' || cmd == 'X' || cmd == 27) {
                        SensorTask_CancelCalibration();
                        printf("Accel cal cancelled.\n\n");
                        break;
                    } else if (cmd == '\r' || cmd == '\n' || cmd == ' ') {
                        while (getchar_timeout_us(1000) != PICO_ERROR_TIMEOUT) {}
                        RC_OS::cmd_confirm_position(0);  // MAVLink path
                        printf("Position confirmed.\n");
                    }
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
            } else {
                printf("Failed to start calibration (%d)\n\n", result);
            }
            break;
        } else if (c == 'x' || c == 'X') {
            printf("Skipped.\n\n");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Step 3: Compass cal - via MAVLink path
    printf("Step 3: Compass Calibration\n");
    printf("Rotate device slowly in all orientations.\n");
    printf("Press ENTER to start, or 'x' to skip...\n");

    while (true) {
        int c = getchar_timeout_us(0);
        if (c == '\r' || c == '\n') {
            while (getchar_timeout_us(1000) != PICO_ERROR_TIMEOUT) {}
            MAV_RESULT result = RC_OS::cmd_compass_cal();  // MAVLink path
            if (result == MAV_RESULT_ACCEPTED) {
                printf("Rotate device... press 'x' when done.\n");
                while (SensorTask_IsCalibrating()) {
                    int cmd = getchar_timeout_us(0);
                    if (cmd == 'x' || cmd == 'X') {
                        SensorTask_CancelCalibration();
                        printf("Compass cal complete.\n\n");
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
            } else {
                printf("Failed to start calibration (%d)\n\n", result);
            }
            break;
        } else if (c == 'x' || c == 'X') {
            printf("Skipped.\n\n");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    printf("(Baro calibrates automatically at boot)\n\n");
    printf("=== Wizard Complete ===\n\n");
}

// ============================================================================
// CLI Task - RC_OS Terminal Interface
//
// DECOUPLED ARCHITECTURE (v0.3):
// - CLI ONLY runs when terminal is connected (stdio_usb_connected())
// - Calibration commands route through MAVLink internally (RC_OS::cmd_*)
// - This ensures CLI uses identical code paths as external GCS (Mission Planner)
// - Terminal connection affects ONLY output, never sensor/flight code behavior
//
// This resolves Entry 15 (LESSONS_LEARNED.md): terminal connection was affecting
// program state because CLI code was too tightly coupled to sensor code.
// ============================================================================

static void CLITask(void* pvParameters) {
    (void)pvParameters;

    // Initialize LED for visual feedback
    gpio_init(kLedPin);
    gpio_set_dir(kLedPin, GPIO_OUT);

    bool wasConnected = false;
    bool ledState = false;
    uint32_t ledCounter = 0;

    while (true) {
        bool isConnected = stdio_usb_connected();

        // =====================================================================
        // DISCONNECTED STATE: Just blink LED slowly, don't touch USB I/O
        // This is the key fix for Entry 15 - we do NOTHING when disconnected
        // =====================================================================
        if (!isConnected) {
            // Slow blink (1Hz) = system running, no terminal
            gpio_put(kLedPin, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_put(kLedPin, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
            wasConnected = false;
            continue;  // Skip all USB I/O
        }

        // =====================================================================
        // JUST CONNECTED: Show banner once
        // =====================================================================
        if (!wasConnected) {
            wasConnected = true;

            // Wait for sensors to be ready before showing banner
            // Fast blink while waiting
            while (!SensorTask_IsInitComplete()) {
                gpio_put(kLedPin, 1);
                vTaskDelay(pdMS_TO_TICKS(50));
                gpio_put(kLedPin, 0);
                vTaskDelay(pdMS_TO_TICKS(50));
            }

            // Brief settle time for USB CDC
            vTaskDelay(pdMS_TO_TICKS(200));

            // Print startup banner
            printf("\n\n");
            printf("========================================\n");
            printf("  RocketChip OS v0.3.1\n");
            printf("  Press 'h' for help\n");
            printf("========================================\n\n");

            // Print initial sensor status
            SensorTask_PrintStatus();

            // Drain any garbage from USB input buffer before accepting commands
            // Terminal connection often has noise bytes that could trigger unwanted actions
            while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {
                // Discard all pending input
            }

            g_menuMode = MenuMode::Main;
        }

        // =====================================================================
        // CONNECTED: Normal CLI operation with 1Hz LED blink
        // =====================================================================

        // Toggle LED every 500ms (1Hz blink = system running + terminal connected)
        if (++ledCounter >= 10) {
            ledCounter = 0;
            ledState = !ledState;
            gpio_put(kLedPin, ledState);
        }

        // Check for input (non-blocking) - route to MAVLink or CLI
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            uint8_t byte = static_cast<uint8_t>(c);

            // Check for MAVLink start marker
            if (byte == kMavlinkV1Start || byte == kMavlinkV2Start) {
                // MAVLink message starting - feed to GCS parser
                GCS::get_singleton().parse_byte(byte);

                // Continue reading until message complete or timeout
                // Max MAVLink message is ~280 bytes, read with 1ms timeout per byte
                for (int i = 0; i < 300; i++) {
                    int next = getchar_timeout_us(1000);  // 1ms timeout
                    if (next == PICO_ERROR_TIMEOUT) {
                        break;  // No more bytes, message probably complete or aborted
                    }
                    if (GCS::get_singleton().parse_byte(static_cast<uint8_t>(next))) {
                        break;  // Complete message received
                    }
                }
            } else {
                // CLI command - handle based on menu mode
                if (g_menuMode == MenuMode::Main) {
                    // Main menu commands
                    switch (c) {
                        case 'h':
                        case 'H':
                        case '?':
                            printSystemStatus();
                            break;
                        case 's':
                        case 'S':
                            SensorTask_PrintStatus();  // Read-only via mutex
                            break;
                        case 'c':
                        case 'C':
                            g_menuMode = MenuMode::Calibration;
                            printCalibrationMenu();
                            break;
                        case '\r':
                        case '\n':
                            printf("\n");
                            break;
                    }
                } else if (g_menuMode == MenuMode::Calibration) {
                    // Calibration menu commands - ALL route through MAVLink (RC_OS::cmd_*)
                    switch (c) {
                        case 'w':
                        case 'W':
                            runCalibrationWizard();
                            g_menuMode = MenuMode::Main;
                            break;

                        case 'l':
                        case 'L':
                            // Level cal via MAVLink path
                            printf("\nLevel Cal - keep device FLAT and STILL...");
                            fflush(stdout);
                            {
                                MAV_RESULT result = RC_OS::cmd_level_cal();
                                if (result == MAV_RESULT_ACCEPTED) {
                                    printf(" OK!\n");
                                } else {
                                    printf(" FAILED (%d)\n", result);
                                }
                            }
                            g_menuMode = MenuMode::Main;
                            break;

                        case 'a':
                        case 'A':
                            // 6-pos accel cal via MAVLink path
                            while (getchar_timeout_us(1000) != PICO_ERROR_TIMEOUT) {}
                            printf("\nStarting 6-position accel calibration...\n");
                            printf("Press ENTER to confirm each position, 'x' to cancel.\n");
                            {
                                MAV_RESULT result = RC_OS::cmd_accel_cal_6pos();
                                if (result == MAV_RESULT_ACCEPTED) {
                                    // Stay in calibration mode until complete
                                    while (SensorTask_IsCalibrating()) {
                                        int cmd = getchar_timeout_us(0);
                                        if (cmd == 'x' || cmd == 'X' || cmd == 27) {
                                            SensorTask_CancelCalibration();
                                            printf("Cancelled.\n");
                                            break;
                                        } else if (cmd == '\r' || cmd == '\n' || cmd == ' ') {
                                            while (getchar_timeout_us(1000) != PICO_ERROR_TIMEOUT) {}
                                            // Confirm position via MAVLink
                                            // TODO: Get current position from calibration state
                                            RC_OS::cmd_confirm_position(0);
                                            printf("Position confirmed.\n");
                                        }
                                        if (++ledCounter >= 5) {
                                            ledCounter = 0;
                                            ledState = !ledState;
                                            gpio_put(kLedPin, ledState);
                                        }
                                        vTaskDelay(pdMS_TO_TICKS(50));
                                    }
                                } else {
                                    printf("Failed to start calibration (%d)\n", result);
                                }
                            }
                            g_menuMode = MenuMode::Main;
                            break;

                        case 'm':
                        case 'M':
                            // Compass cal via MAVLink path
                            printf("\nStarting compass calibration...\n");
                            printf("Rotate device slowly. Press 'x' when done.\n");
                            {
                                MAV_RESULT result = RC_OS::cmd_compass_cal();
                                if (result == MAV_RESULT_ACCEPTED) {
                                    while (SensorTask_IsCalibrating()) {
                                        int cmd = getchar_timeout_us(0);
                                        if (cmd == 'x' || cmd == 'X' || cmd == 27) {
                                            SensorTask_CancelCalibration();
                                            printf("Calibration complete.\n");
                                            break;
                                        }
                                        if (++ledCounter >= 5) {
                                            ledCounter = 0;
                                            ledState = !ledState;
                                            gpio_put(kLedPin, ledState);
                                        }
                                        vTaskDelay(pdMS_TO_TICKS(50));
                                    }
                                } else {
                                    printf("Failed to start calibration (%d)\n", result);
                                }
                            }
                            g_menuMode = MenuMode::Main;
                            break;

                        case 'g':
                        case 'G':
                            // Gyro cal via MAVLink path
                            printf("\nGyro Cal - keep device STILL...");
                            fflush(stdout);
                            {
                                MAV_RESULT result = RC_OS::cmd_gyro_cal();
                                if (result == MAV_RESULT_ACCEPTED) {
                                    printf(" OK!\n");
                                } else if (result == MAV_RESULT_UNSUPPORTED) {
                                    printf(" not yet implemented\n");
                                } else {
                                    printf(" FAILED (%d)\n", result);
                                }
                            }
                            break;

                        case 'b':
                        case 'B':
                            // Baro cal via MAVLink path
                            printf("\nBaro Cal...");
                            fflush(stdout);
                            {
                                MAV_RESULT result = RC_OS::cmd_baro_cal();
                                if (result == MAV_RESULT_ACCEPTED) {
                                    printf(" OK!\n");
                                } else if (result == MAV_RESULT_UNSUPPORTED) {
                                    printf(" (calibrates automatically at boot)\n");
                                } else {
                                    printf(" FAILED (%d)\n", result);
                                }
                            }
                            break;

                        case 'x':
                        case 'X':
                        case 27:  // ESC
                            if (SensorTask_IsCalibrating()) {
                                SensorTask_CancelCalibration();
                                printf("\nCalibration cancelled.\n");
                            }
                            printf("Returning to main menu.\n");
                            g_menuMode = MenuMode::Main;
                            break;

                        case 'h':
                        case 'H':
                        case '?':
                            printCalibrationMenu();
                            break;

                        case '\r':
                        case '\n':
                            break;
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ============================================================================
// FreeRTOS Hooks (required for static allocation)
// ============================================================================

extern "C" {

void vApplicationMallocFailedHook(void) {
    // Report to ArduPilot error tracking system
    INTERNAL_ERROR(AP_InternalError::error_t::mem_guard);

    // Fatal errors retain minimal printf for post-mortem debugging
    // LED blink codes are primary indication in production
    printf("\n!!! FATAL: Malloc failed - heap exhausted !!!\n");
    printf("Free heap: %u bytes\n", xPortGetFreeHeapSize());

    gpio_init(kLedPin);
    gpio_set_dir(kLedPin, GPIO_OUT);

    // Fast blink = malloc failure
    portDISABLE_INTERRUPTS();
    while (true) {
        gpio_put(kLedPin, 1);
        busy_wait_us(100000);
        gpio_put(kLedPin, 0);
        busy_wait_us(100000);
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
    (void)xTask;

    // Report to ArduPilot error tracking system
    INTERNAL_ERROR(AP_InternalError::error_t::stack_overflow);

    // Fatal errors retain minimal printf for post-mortem debugging
    printf("\n!!! FATAL: Stack overflow in task: %s !!!\n", pcTaskName);

    gpio_init(kLedPin);
    gpio_set_dir(kLedPin, GPIO_OUT);

    // Slow blink = stack overflow
    portDISABLE_INTERRUPTS();
    while (true) {
        gpio_put(kLedPin, 1);
        busy_wait_us(500000);
        gpio_put(kLedPin, 0);
        busy_wait_us(500000);
    }
}

// Idle task memory (Core 0)
static StaticTask_t s_idleTaskTCB;
static StackType_t s_idleTaskStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer,
                                   StackType_t** ppxIdleTaskStackBuffer,
                                   uint32_t* pulIdleTaskStackSize) {
    *ppxIdleTaskTCBBuffer = &s_idleTaskTCB;
    *ppxIdleTaskStackBuffer = s_idleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

// Timer task memory
static StaticTask_t s_timerTaskTCB;
static StackType_t s_timerTaskStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer,
                                    StackType_t** ppxTimerTaskStackBuffer,
                                    uint32_t* pulTimerTaskStackSize) {
    *ppxTimerTaskTCBBuffer = &s_timerTaskTCB;
    *ppxTimerTaskStackBuffer = s_timerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

// Passive idle task memory (Core 1 for SMP)
static StaticTask_t s_passiveIdleTaskTCB;
static StackType_t s_passiveIdleTaskStack[configMINIMAL_STACK_SIZE];

void vApplicationGetPassiveIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer,
                                          StackType_t** ppxIdleTaskStackBuffer,
                                          uint32_t* pulIdleTaskStackSize,
                                          BaseType_t xCoreID) {
    (void)xCoreID;
    *ppxIdleTaskTCBBuffer = &s_passiveIdleTaskTCB;
    *ppxIdleTaskStackBuffer = s_passiveIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

} // extern "C"

// ============================================================================
// Main
// ============================================================================

int main() {
    // =========================================================================
    // CRITICAL: HAL init (flash operations) BEFORE USB per RP2350_FULL_AP_PORT.md PD1
    // Flash ops make entire flash inaccessible - if USB is active, it breaks!
    // =========================================================================
    // Initialize SensorTask - this calls hal_init() which does flash operations
    // MUST happen before stdio_init_all() to avoid breaking USB
    // =========================================================================
    SensorTask_Init();

    // Clear BASEPRI after HAL init - may be elevated, blocking USB IRQs
    // Per LESSONS_LEARNED.md Entry 3
    __asm volatile ("mov r0, #0\nmsr basepri, r0" ::: "r0");

    // NOW safe to start USB - all flash operations complete
    stdio_init_all();

    // Create tasks
    SensorTask_Create();

    TaskHandle_t cliHandle;
    BaseType_t result = xTaskCreate(
        CLITask,
        "CLI",
        kCliStackSize,
        nullptr,
        kCliTaskPriority,
        &cliHandle
    );

    if (result == pdPASS) {
        vTaskCoreAffinitySet(cliHandle, (1 << 0));  // Pin to Core 0
    }

    // Start the scheduler - should never return
    vTaskStartScheduler();

    // If we get here, something went wrong - this is a fatal error
    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    DBG_ERROR("ERROR: Scheduler returned!\n");

    while (true) {
        gpio_put(kLedPin, 1);
        busy_wait_ms(50);
        gpio_put(kLedPin, 0);
        busy_wait_ms(50);
    }

    return 0;
}
