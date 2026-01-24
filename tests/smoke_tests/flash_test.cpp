/**
 * @file flash_test.cpp
 * @brief Minimal flash erase test - no FreeRTOS, no HAL
 *
 * Tests if basic flash operations work on the RP2350.
 */

#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 7
#endif

// Test flash address - well after firmware (at 512KB mark)
#define TEST_FLASH_OFFSET (512 * 1024)

// RAM-resident flash erase
static void __not_in_flash_func(do_erase)(void) {
    flash_range_erase(TEST_FLASH_OFFSET, 4096);
}

int main() {
    // Init LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // 3 quick blinks = main reached
    for (int i = 0; i < 3; i++) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(100);
    }

    sleep_ms(500);

    // 1 long blink = about to do flash erase
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    sleep_ms(500);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    sleep_ms(500);

    // Do flash erase with interrupts disabled
    uint32_t saved = save_and_disable_interrupts();
    do_erase();
    restore_interrupts(saved);

    // 2 quick blinks = flash erase completed!
    for (int i = 0; i < 2; i++) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(100);
    }

    // Slow blink = success, running
    while (true) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(500);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(500);
    }

    return 0;
}
