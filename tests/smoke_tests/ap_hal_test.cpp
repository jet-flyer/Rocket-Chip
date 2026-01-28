/**
 * @file ap_hal_test.cpp
 * @brief Minimal smoke test for AP_HAL_RP2350
 *
 * Stripped down to match smoke_storage structure exactly.
 */

#include "AP_HAL_RP2350/AP_HAL_RP2350.h"

#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"

#include <cstdio>

// ============================================================================
// Main Test Task
// ============================================================================

static void test_task(void* params) {
    (void)params;

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // Initialize HAL subsystems for testing
    RP2350::hal_init();

    // Success - slow blink (500ms on/off)
    while (true) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ============================================================================
// Main Entry Point (matching smoke_storage exactly)
// ============================================================================

int main() {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    stdio_init_all();

    // Create test task (static allocation)
    static StaticTask_t task_buffer;
    static StackType_t task_stack[2048];

    xTaskCreateStatic(
        test_task,
        "APHALTest",
        sizeof(task_stack) / sizeof(StackType_t),
        nullptr,
        tskIDLE_PRIORITY + 1,
        task_stack,
        &task_buffer
    );

    // Start scheduler
    vTaskStartScheduler();

    // Should never reach here
    while (true) {
        tight_loop_contents();
    }

    return 0;
}

// ============================================================================
// FreeRTOS Hooks (matching smoke_storage exactly)
// ============================================================================

extern "C" {

void vApplicationMallocFailedHook() {
    while (true) {
        tight_loop_contents();
    }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char* name) {
    (void)task;
    (void)name;
    while (true) {
        tight_loop_contents();
    }
}

// Idle task memory (Core 0)
static StaticTask_t idle_task_tcb;
static StackType_t idle_task_stack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t** tcb,
                                    StackType_t** stack,
                                    configSTACK_DEPTH_TYPE* stack_size) {
    *tcb = &idle_task_tcb;
    *stack = idle_task_stack;
    *stack_size = configMINIMAL_STACK_SIZE;
}

#if (configNUMBER_OF_CORES > 1)
// Passive idle task memory (Core 1)
static StaticTask_t passive_idle_tcb;
static StackType_t passive_idle_stack[configMINIMAL_STACK_SIZE];

void vApplicationGetPassiveIdleTaskMemory(StaticTask_t** tcb,
                                           StackType_t** stack,
                                           configSTACK_DEPTH_TYPE* stack_size,
                                           BaseType_t core) {
    (void)core;
    *tcb = &passive_idle_tcb;
    *stack = passive_idle_stack;
    *stack_size = configMINIMAL_STACK_SIZE;
}
#endif

// Timer task memory
static StaticTask_t timer_task_tcb;
static StackType_t timer_task_stack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t** tcb,
                                     StackType_t** stack,
                                     configSTACK_DEPTH_TYPE* stack_size) {
    *tcb = &timer_task_tcb;
    *stack = timer_task_stack;
    *stack_size = configTIMER_TASK_STACK_DEPTH;
}

}  // extern "C"
