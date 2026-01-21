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
// UI Task - LED blinking and status
// ============================================================================

static void UITask(void* pvParameters) {
    (void)pvParameters;

    gpio_init(kLedPin);
    gpio_set_dir(kLedPin, GPIO_OUT);

    bool ledState = false;

    while (true) {
        // Toggle LED every 500ms (1Hz blink = system running)
        ledState = !ledState;
        gpio_put(kLedPin, ledState);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ============================================================================
// FreeRTOS Hooks (required for static allocation)
// ============================================================================

extern "C" {

void vApplicationMallocFailedHook(void) {
    // Report to ArduPilot error tracking system
    AP_InternalError::error(AP_InternalError::error_t::mem_guard);

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
    AP_InternalError::error(AP_InternalError::error_t::stack_overflow);

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

    // Blink 3 times rapidly to show we reached main()
    for (int i = 0; i < 3; i++) {
        gpio_put(kLedPin, 1);
        busy_wait_ms(100);
        gpio_put(kLedPin, 0);
        busy_wait_ms(100);
    }

    // Initialize USB serial
    stdio_init_all();
    sleep_ms(2000);  // Wait for USB enumeration

    // Blink 2 times slowly = USB init done
    for (int i = 0; i < 2; i++) {
        gpio_put(kLedPin, 1);
        sleep_ms(250);
        gpio_put(kLedPin, 0);
        sleep_ms(250);
    }

    DBG_PRINT("\n\n");
    DBG_PRINT("========================================\n");
    DBG_PRINT("  RocketChip Production Firmware\n");
    DBG_PRINT("  Target: Adafruit Feather RP2350\n");
    DBG_PRINT("  Phase: 2 (Sensors)\n");
    DBG_PRINT("========================================\n\n");

    // Initialize SensorTask (HAL + sensors)
    DBG_PRINT("[Main] Initializing SensorTask...\n");
    if (!SensorTask_Init()) {
        DBG_ERROR("[Main] SensorTask init FAILED - continuing anyway\n");
        // Don't fail completely - let the system run for debugging
    }

    DBG_PRINT("[Main] Creating tasks...\n");

    // Create SensorTask
    if (!SensorTask_Create()) {
        DBG_ERROR("[Main] ERROR: Failed to create SensorTask\n");
    }

    // Create UI task (LED blink)
    TaskHandle_t uiHandle;
    BaseType_t result = xTaskCreate(
        UITask,
        "UI",
        kUiStackSize,
        nullptr,
        kUiTaskPriority,
        &uiHandle
    );

    if (result != pdPASS) {
        DBG_ERROR("[Main] ERROR: Failed to create UITask\n");
    } else {
        vTaskCoreAffinitySet(uiHandle, (1 << 0));  // Pin to Core 0
    }

    DBG_PRINT("[Main] Starting scheduler...\n\n");

    // One long blink before scheduler starts
    gpio_put(kLedPin, 1);
    sleep_ms(500);
    gpio_put(kLedPin, 0);
    sleep_ms(500);

    // Start the scheduler - should never return
    vTaskStartScheduler();

    // If we get here, something went wrong - this is a fatal error
    AP_InternalError::error(AP_InternalError::error_t::flow_of_control);
    DBG_ERROR("ERROR: Scheduler returned!\n");

    while (true) {
        gpio_put(kLedPin, 1);
        busy_wait_ms(50);
        gpio_put(kLedPin, 0);
        busy_wait_ms(50);
    }

    return 0;
}
