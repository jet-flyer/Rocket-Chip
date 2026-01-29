/**
 * @file i2c_scan.cpp
 * @brief Simple I2C bus scanner
 *
 * Scans I2C bus 1 (Qwiic port) and reports all responding addresses.
 */

#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

// I2C1 pins (Feather RP2350 Qwiic port)
static constexpr uint8_t kI2cSda = 2;
static constexpr uint8_t kI2cScl = 3;
static constexpr uint32_t kI2cFreqHz = 400000;

int main() {
    // Init LED for visual feedback
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // Blink while waiting for USB
    stdio_init_all();
    while (!stdio_usb_connected()) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(100);
    }
    sleep_ms(500);

    printf("\n========================================\n");
    printf("  I2C Bus Scanner\n");
    printf("  Pins: SDA=%d, SCL=%d\n", kI2cSda, kI2cScl);
    printf("========================================\n\n");

    // Initialize I2C1
    i2c_init(i2c1, kI2cFreqHz);
    gpio_set_function(kI2cSda, GPIO_FUNC_I2C);
    gpio_set_function(kI2cScl, GPIO_FUNC_I2C);
    gpio_pull_up(kI2cSda);
    gpio_pull_up(kI2cScl);

    printf("Scanning I2C bus...\n\n");
    printf("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    int found = 0;
    for (int addr = 0; addr < 128; addr++) {
        if (addr % 16 == 0) {
            printf("%02X: ", addr);
        }

        // Skip reserved addresses
        if (addr < 0x08 || addr > 0x77) {
            printf("   ");
        } else {
            uint8_t rxdata;
            int ret = i2c_read_blocking(i2c1, addr, &rxdata, 1, false);
            if (ret >= 0) {
                printf("%02X ", addr);
                found++;
            } else {
                printf("-- ");
            }
        }

        if (addr % 16 == 15) {
            printf("\n");
        }
    }

    printf("\nFound %d device(s)\n\n", found);

    // Known devices
    printf("Expected devices:\n");
    printf("  0x69 = ICM-20948 (IMU) - AD0=HIGH on Adafruit board\n");
    printf("  0x77 or 0x76 = DPS310 (Baro)\n");
    printf("  0x10 = PA1010D (GPS)\n");

    // Heartbeat
    while (true) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(1000);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(1000);
    }

    return 0;
}
