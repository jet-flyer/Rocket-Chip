/**
 * @file i2c_scan.c
 * @brief I2C bus scanner - finds all responding devices
 */

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>

// I2C1 pins (STEMMA QT on Feather RP2350)
#define I2C_SDA 2
#define I2C_SCL 3

int main() {
    // Init LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // Quick blink
    for (int i = 0; i < 3; i++) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        busy_wait_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        busy_wait_ms(100);
    }

    // Init I2C
    i2c_init(i2c1, 100000);  // 100kHz for compatibility
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Init USB
    stdio_init_all();

    // Wait for USB
    while (!stdio_usb_connected()) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(100);
    }
    sleep_ms(500);

    printf("\n");
    printf("================================\n");
    printf("  I2C Bus Scanner (I2C1)\n");
    printf("  SDA=GPIO%d, SCL=GPIO%d\n", I2C_SDA, I2C_SCL);
    printf("================================\n\n");

    printf("Scanning addresses 0x00-0x7F...\n\n");
    printf("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    int found = 0;

    for (int row = 0; row < 8; row++) {
        printf("%02X: ", row * 16);

        for (int col = 0; col < 16; col++) {
            int addr = row * 16 + col;

            // Skip reserved addresses
            if (addr < 0x08 || addr > 0x77) {
                printf("   ");
                continue;
            }

            // Try to read one byte
            uint8_t dummy;
            int ret = i2c_read_timeout_us(i2c1, addr, &dummy, 1, false, 10000);

            if (ret >= 0) {
                printf("%02X ", addr);
                found++;
            } else {
                printf("-- ");
            }
        }
        printf("\n");
    }

    printf("\nFound %d device(s)\n\n", found);

    // Known addresses for current hardware
    printf("Expected devices:\n");
    printf("  0x68/0x69 - ICM-20948 IMU (9-axis)\n");
    printf("  0x77 - DPS310 Barometer\n");
    printf("\n");

    // Blink to show done
    while (1) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(1000);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(1000);
    }

    return 0;
}
