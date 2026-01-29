/**
 * @file main.cpp
 * @brief RocketChip Production Entry Point
 *
 * Production firmware entry point. Initializes HAL, creates FreeRTOS tasks,
 * and starts the scheduler.
 *
 * Current functionality (Phase 2):
 * - SensorTask: Real sensor sampling at 1kHz (IMU), 50Hz (baro)
 * - UITask: LED status and periodic status output
 *
 * @note Part of RocketChip - Modular Motion Tracking Platform
 * @see docs/SAD.md for architecture details
 */

#include <cstdio>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "services/SensorTask.h"
#include "AP_InternalError/AP_InternalError.h"

using namespace rocketchip::services;

// ============================================================================
// Configuration Constants (k prefix per JSF AV naming conventions)
// ============================================================================

static constexpr uint8_t kLedPin = PICO_DEFAULT_LED_PIN;

static constexpr uint32_t kUiTaskPriority = 1;
static constexpr uint32_t kUiStackSize = configMINIMAL_STACK_SIZE * 4;

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
    printf("  Press 's' for sensor status\n");
    printf("  Press '?' or 'h' for this help\n");
    printf("========================================\n\n");
}

// ============================================================================
// UI Task - LED blinking, key commands, and status
// ============================================================================

static void UITask(void* pvParameters) {
    (void)pvParameters;

    gpio_init(kLedPin);
    gpio_set_dir(kLedPin, GPIO_OUT);

    // Wait for keypress before showing init output
    // Now under FreeRTOS, USB CDC works properly
    uint32_t promptCount = 0;
    while (true) {
        // Blink LED while waiting
        gpio_put(kLedPin, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_put(kLedPin, 0);
        vTaskDelay(pdMS_TO_TICKS(100));

        // Print prompt every 2 seconds
        if (promptCount % 10 == 0) {
            printf("\rPress any key to start...   ");
        }
        promptCount++;

        // Check for keypress
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            break;
        }
    }

    // Print startup banner
    printf("\n\n");
    printf("========================================\n");
    printf("  RocketChip Production Firmware\n");
    printf("  Target: Adafruit Feather RP2350\n");
    printf("  Phase: 2 (Sensors)\n");
    printf("  Press '?' or 'h' for status\n");
    printf("========================================\n\n");

    // Print initial sensor status
    SensorTask_PrintStatus();

    // Main UI loop
    bool ledState = false;

    while (true) {
        // Toggle LED every 500ms (1Hz blink = system running)
        ledState = !ledState;
        gpio_put(kLedPin, ledState);

        // Check for key commands (non-blocking)
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            switch (c) {
                case '?':
                case 'h':
                case 'H':
                    printSystemStatus();
                    break;
                case 's':
                case 'S':
                    // Trigger SensorTask status print
                    SensorTask_PrintStatus();
                    break;
                case '\r':
                case '\n':
                    // Just echo a newline for visual feedback
                    printf("\n");
                    break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
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
    // Initialize LED for early debugging
    gpio_init(kLedPin);
    gpio_set_dir(kLedPin, GPIO_OUT);

    // Initialize USB serial - don't do any USB I/O before FreeRTOS starts!
    stdio_init_all();

    // Initialize sensors before starting scheduler
    // (USB wait and output will happen in UITask after scheduler starts)
    if (!SensorTask_Init()) {
        // Can't print error here - no FreeRTOS yet, USB unreliable
        // UITask will show sensor status later
    }

    // Create SensorTask
    SensorTask_Create();

    // Create UI task (handles keypress wait, LED, and status output)
    TaskHandle_t uiHandle;
    BaseType_t result = xTaskCreate(
        UITask,
        "UI",
        kUiStackSize,
        nullptr,
        kUiTaskPriority,
        &uiHandle
    );

    if (result == pdPASS) {
        vTaskCoreAffinitySet(uiHandle, (1 << 0));  // Pin to Core 0
    }

    // Start the scheduler - should never return
    // All USB I/O happens in tasks after this point
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
