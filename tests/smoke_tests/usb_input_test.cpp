/**
 * @file usb_input_test.cpp
 * @brief Minimal USB CDC input test - no flash operations
 *
 * This test verifies USB CDC input works without any flash operations.
 * Used to diagnose whether USB input issues are caused by flash operations.
 */

#include <cstdio>
#include "pico/stdlib.h"

int main() {
    stdio_init_all();

    // Wait for USB connection
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    sleep_ms(500);

    printf("\n========================================\n");
    printf("  USB CDC Input Test (No Flash Ops)\n");
    printf("========================================\n\n");

    printf("This test does NOT use any flash operations.\n");
    printf("If input works here but not in calibration_test,\n");
    printf("then flash operations are the cause.\n\n");

    for (int i = 0; i < 5; i++) {
        printf("Test %d/5: Press any key... ", i + 1);
        fflush(stdout);

        int c;
        while ((c = getchar_timeout_us(100000)) == PICO_ERROR_TIMEOUT) {
            // Keep waiting
        }

        printf("Got: 0x%02X ('%c')\n", c, (c >= 32 && c < 127) ? c : '?');
        fflush(stdout);
    }

    printf("\n========================================\n");
    printf("All 5 inputs received! USB input works.\n");
    printf("========================================\n");

    while (true) {
        sleep_ms(1000);
    }

    return 0;
}
