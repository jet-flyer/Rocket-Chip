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

#include "services/SensorTask.h"

using namespace rocketchip::services;

// ============================================================================
// Configuration
// ============================================================================

static constexpr uint8_t LED_PIN = PICO_DEFAULT_LED_PIN;

static constexpr uint32_t UI_TASK_PRIORITY = 1;
static constexpr uint32_t UI_STACK_SIZE = configMINIMAL_STACK_SIZE * 4;

// ============================================================================
// UI Task - LED blinking and status
// ============================================================================

static void UITask(void* pvParameters) {
    (void)pvParameters;

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    bool ledState = false;

    while (true) {
        // Toggle LED every 500ms (1Hz blink = system running)
        ledState = !ledState;
        gpio_put(LED_PIN, ledState);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ============================================================================
// FreeRTOS Hooks (required for static allocation)
// ============================================================================

extern "C" {

void vApplicationMallocFailedHook(void) {
    printf("\n!!! FATAL: Malloc failed - heap exhausted !!!\n");
    printf("Free heap: %u bytes\n", xPortGetFreeHeapSize());

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    portDISABLE_INTERRUPTS();
    while (true) {
        gpio_put(LED_PIN, 1);
        busy_wait_us(100000);
        gpio_put(LED_PIN, 0);
        busy_wait_us(100000);
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
    (void)xTask;
    printf("\n!!! FATAL: Stack overflow in task: %s !!!\n", pcTaskName);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    portDISABLE_INTERRUPTS();
    while (true) {
        gpio_put(LED_PIN, 1);
        busy_wait_us(500000);
        gpio_put(LED_PIN, 0);
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
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Blink 3 times rapidly to show we reached main()
    for (int i = 0; i < 3; i++) {
        gpio_put(LED_PIN, 1);
        busy_wait_ms(100);
        gpio_put(LED_PIN, 0);
        busy_wait_ms(100);
    }

    // Initialize USB serial
    stdio_init_all();
    sleep_ms(2000);  // Wait for USB enumeration

    // Blink 2 times slowly = USB init done
    for (int i = 0; i < 2; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(250);
        gpio_put(LED_PIN, 0);
        sleep_ms(250);
    }

    printf("\n\n");
    printf("========================================\n");
    printf("  RocketChip Production Firmware\n");
    printf("  Target: Adafruit Feather RP2350\n");
    printf("  Phase: 2 (Sensors)\n");
    printf("========================================\n\n");

    // Initialize SensorTask (HAL + sensors)
    printf("[Main] Initializing SensorTask...\n");
    if (!SensorTask_Init()) {
        printf("[Main] SensorTask init FAILED - continuing anyway\n");
        // Don't fail completely - let the system run for debugging
    }

    printf("[Main] Creating tasks...\n");

    // Create SensorTask
    if (!SensorTask_Create()) {
        printf("[Main] ERROR: Failed to create SensorTask\n");
    }

    // Create UI task (LED blink)
    TaskHandle_t uiHandle;
    BaseType_t result = xTaskCreate(
        UITask,
        "UI",
        UI_STACK_SIZE,
        nullptr,
        UI_TASK_PRIORITY,
        &uiHandle
    );

    if (result != pdPASS) {
        printf("[Main] ERROR: Failed to create UITask\n");
    } else {
        vTaskCoreAffinitySet(uiHandle, (1 << 0));  // Pin to Core 0
    }

    printf("[Main] Starting scheduler...\n\n");

    // One long blink before scheduler starts
    gpio_put(LED_PIN, 1);
    sleep_ms(500);
    gpio_put(LED_PIN, 0);
    sleep_ms(500);

    // Start the scheduler - should never return
    vTaskStartScheduler();

    // If we get here, something went wrong
    printf("ERROR: Scheduler returned!\n");

    while (true) {
        gpio_put(LED_PIN, 1);
        busy_wait_ms(50);
        gpio_put(LED_PIN, 0);
        busy_wait_ms(50);
    }

    return 0;
}
