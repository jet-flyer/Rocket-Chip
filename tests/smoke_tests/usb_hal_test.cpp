/**
 * @file usb_hal_test.cpp
 * @brief Test USB enumeration with HAL init order
 *
 * Tests whether calling hal.init() before stdio_init_all() breaks USB.
 * This test does NOT use NeoPixel to isolate the HAL as the variable.
 */

#include <cstdio>
#include "pico/stdlib.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

// ArduPilot HAL
#include <AP_HAL_RP2350/HAL_RP2350_Class.h>

// Simple LED for status (no PIO/NeoPixel)
static constexpr uint8_t kLedPin = 7;  // Feather RP2350 onboard LED

static void ledBlink(int count, int on_ms, int off_ms) {
    gpio_init(kLedPin);
    gpio_set_dir(kLedPin, GPIO_OUT);
    for (int i = 0; i < count; i++) {
        gpio_put(kLedPin, 1);
        sleep_ms(on_ms);
        gpio_put(kLedPin, 0);
        sleep_ms(off_ms);
    }
}

int main() {
    // Initialize simple LED
    gpio_init(kLedPin);
    gpio_set_dir(kLedPin, GPIO_OUT);

    // Blink once = starting
    ledBlink(1, 200, 200);

    // Initialize HAL BEFORE stdio_init_all()
    // This does storage.init() (flash ops), scheduler, gpio, i2c, spi
    hal.init();

    // Blink twice = HAL done
    ledBlink(2, 200, 200);

    // NOW initialize USB - after all HAL operations
    stdio_init_all();

    // Blink 3 times = USB init called
    ledBlink(3, 200, 200);

    // Wait for USB connection - fast blink
    while (!stdio_usb_connected()) {
        gpio_put(kLedPin, 1);
        sleep_ms(100);
        gpio_put(kLedPin, 0);
        sleep_ms(100);
    }
    sleep_ms(500);

    // Solid LED = USB connected
    gpio_put(kLedPin, 1);

    printf("\n========================================\n");
    printf("  USB + HAL Test (No NeoPixel)\n");
    printf("========================================\n\n");
    printf("hal.init() was called BEFORE stdio_init_all().\n");
    printf("If you can see this, USB works with HAL pre-init.\n\n");

    printf("Press any key to confirm input works: ");
    fflush(stdout);

    int c;
    while ((c = getchar_timeout_us(100000)) == PICO_ERROR_TIMEOUT) {
        // Wait
    }
    printf("Got: 0x%02X ('%c')\n", c, (c >= 32 && c < 127) ? c : '?');

    printf("\nTest PASSED - HAL init before stdio_init_all() is safe.\n");

    while (true) {
        sleep_ms(1000);
    }

    return 0;
}

// ============================================================================
// FreeRTOS Hooks (required by ap_hal_rp2350)
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

#if configNUMBER_OF_CORES > 1
// Idle task memory (Core 1) for SMP
static StaticTask_t idle_task_tcb_core1;
static StackType_t idle_task_stack_core1[configMINIMAL_STACK_SIZE];

void vApplicationGetPassiveIdleTaskMemory(StaticTask_t** tcb,
                                           StackType_t** stack,
                                           configSTACK_DEPTH_TYPE* stack_size,
                                           BaseType_t core_id) {
    (void)core_id;
    *tcb = &idle_task_tcb_core1;
    *stack = idle_task_stack_core1;
    *stack_size = configMINIMAL_STACK_SIZE;
}
#endif

}  // extern "C"
