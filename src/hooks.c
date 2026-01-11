/*
 * FreeRTOS Hook Functions for RP2350 SMP
 *
 * These hooks are required by FreeRTOS and handle error conditions
 * and memory allocation for system tasks.
 */

#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"

/*
 * Malloc Failed Hook
 * Called when FreeRTOS fails to allocate memory from the heap
 */
void vApplicationMallocFailedHook(void) {
    printf("\n!!! FATAL: Malloc failed - heap exhausted !!!\n");
    printf("Free heap: %u bytes\n", xPortGetFreeHeapSize());

    // Flash LED rapidly to indicate error
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    portDISABLE_INTERRUPTS();
    while (1) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        busy_wait_us(100000); // 100ms
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        busy_wait_us(100000);
    }
}

/*
 * Stack Overflow Hook
 * Called when FreeRTOS detects a task has overflowed its stack
 * (only when configCHECK_FOR_STACK_OVERFLOW > 0)
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;

    printf("\n!!! FATAL: Stack overflow in task: %s !!!\n", pcTaskName);
    printf("Free heap: %u bytes\n", xPortGetFreeHeapSize());

    // Flash LED slowly to indicate error
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    portDISABLE_INTERRUPTS();
    while (1) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        busy_wait_us(500000); // 500ms
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        busy_wait_us(500000);
    }
}

/*
 * Idle Task Memory Allocation (Core 0)
 * FreeRTOS requires static memory for the Idle task
 */
static StaticTask_t idle_task_tcb;
static StackType_t idle_task_stack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
    *ppxIdleTaskTCBBuffer = &idle_task_tcb;
    *ppxIdleTaskStackBuffer = idle_task_stack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/*
 * Timer Task Memory Allocation
 * FreeRTOS requires static memory for the Timer daemon task
 */
static StaticTask_t timer_task_tcb;
static StackType_t timer_task_stack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize) {
    *ppxTimerTaskTCBBuffer = &timer_task_tcb;
    *ppxTimerTaskStackBuffer = timer_task_stack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

/*
 * Passive Idle Task Memory Allocation (Core 1 for SMP)
 * Required when configNUMBER_OF_CORES == 2
 * This idle task runs on Core 1 when no other tasks are ready
 */
static StaticTask_t passive_idle_task_tcb;
static StackType_t passive_idle_task_stack[configMINIMAL_STACK_SIZE];

void vApplicationGetPassiveIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                          StackType_t **ppxIdleTaskStackBuffer,
                                          uint32_t *pulIdleTaskStackSize,
                                          BaseType_t xCoreID) {
    (void)xCoreID; // Only Core 1 needs passive idle (Core 0 has standard idle)

    *ppxIdleTaskTCBBuffer = &passive_idle_task_tcb;
    *ppxIdleTaskStackBuffer = passive_idle_task_stack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
