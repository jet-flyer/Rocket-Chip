/**
 * @file usb_neopixel_test.cpp
 * @brief Test USB enumeration with NeoPixel init order
 *
 * Tests whether initializing NeoPixel (PIO) before stdio_init_all() breaks USB.
 */

#include <cstdio>
#include "pico/stdlib.h"

// RocketChip HAL for NeoPixel
#include "HAL.h"
#include "PIO.h"

using namespace rocketchip::hal;

static constexpr uint8_t kNeoPixelPin = 21;

int main() {
    // Test 1: Initialize NeoPixel BEFORE stdio_init_all()
    // If USB doesn't enumerate, this is the problem
    WS2812* led = new WS2812(kNeoPixelPin, 1);
    if (led && led->begin()) {
        led->setBrightness(50);
        led->setPixel(0, RGB(255, 0, 0));  // Red = NeoPixel OK
        led->show();
    }

    // Now init USB
    stdio_init_all();

    // Wait for USB - cyan blink
    bool state = false;
    while (!stdio_usb_connected()) {
        if (led) {
            led->setPixel(0, state ? RGB(0, 255, 255) : RGB(0, 0, 0));
            led->show();
        }
        state = !state;
        sleep_ms(200);
    }
    sleep_ms(500);

    // Green = USB connected
    if (led) {
        led->setPixel(0, RGB(0, 255, 0));
        led->show();
    }

    printf("\n========================================\n");
    printf("  USB + NeoPixel Test\n");
    printf("========================================\n\n");
    printf("NeoPixel was initialized BEFORE stdio_init_all().\n");
    printf("If you can see this, USB works with NeoPixel pre-init.\n\n");

    printf("Press any key to confirm input works: ");
    fflush(stdout);

    int c;
    while ((c = getchar_timeout_us(100000)) == PICO_ERROR_TIMEOUT) {
        // Wait
    }
    printf("Got: 0x%02X ('%c')\n", c, (c >= 32 && c < 127) ? c : '?');

    printf("\nTest PASSED - NeoPixel before stdio_init_all() is safe.\n");

    while (true) {
        sleep_ms(1000);
    }

    return 0;
}
