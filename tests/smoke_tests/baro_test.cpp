/**
 * @file baro_test.cpp
 * @brief Simple DPS310 barometer test to isolate restart hang issue
 *
 * Tests raw I2C communication with DPS310 without ArduPilot Device class
 * to determine if restart hang is IMU-specific or general I2C issue.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

// DPS310 at 0x77 on I2C1 (Qwiic/STEMMA QT on Feather RP2350)
#define DPS310_ADDR     0x77
#define DPS310_PROD_ID  0x0D    // Product ID register
#define DPS310_EXPECTED 0x10    // Expected product ID

// I2C1 pins (Qwiic on Feather RP2350)
#define I2C_SDA_PIN     2
#define I2C_SCL_PIN     3
#define I2C_INST        i2c1
#define I2C_FREQ_HZ     100000

static void init_i2c() {
    i2c_init(I2C_INST, I2C_FREQ_HZ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}

static bool read_dps310_id(uint8_t* id) {
    uint8_t reg = DPS310_PROD_ID;

    // Write register address
    int ret = i2c_write_timeout_us(I2C_INST, DPS310_ADDR, &reg, 1, true, 10000);
    if (ret < 0) {
        printf("  I2C write failed: %d\n", ret);
        return false;
    }

    // Read value
    ret = i2c_read_timeout_us(I2C_INST, DPS310_ADDR, id, 1, false, 10000);
    if (ret < 0) {
        printf("  I2C read failed: %d\n", ret);
        return false;
    }

    return true;
}

int main() {
    // Initialize stdio (USB CDC)
    stdio_init_all();

    // Wait for USB connection with LED feedback
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (!stdio_usb_connected()) {
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);
        sleep_ms(100);
    }
    sleep_ms(500);  // Settle time

    printf("\n========================================\n");
    printf("  DPS310 Barometer Test (Raw I2C)\n");
    printf("  No ArduPilot Device class\n");
    printf("========================================\n\n");

    // Initialize I2C
    printf("Initializing I2C...\n");
    init_i2c();
    printf("I2C initialized at %d Hz\n\n", I2C_FREQ_HZ);

    // Read DPS310 product ID
    printf("Reading DPS310 at 0x%02X...\n", DPS310_ADDR);
    uint8_t product_id = 0;

    if (read_dps310_id(&product_id)) {
        printf("  Product ID: 0x%02X (expected 0x%02X)\n", product_id, DPS310_EXPECTED);
        if (product_id == DPS310_EXPECTED) {
            printf("  PASS: DPS310 detected!\n");
        } else {
            printf("  WARN: Unexpected product ID\n");
        }
    } else {
        printf("  FAIL: Could not read DPS310\n");
    }

    printf("\n========================================\n");
    printf("Test complete. Press RESET to test restart.\n");
    printf("========================================\n");

    // Blink LED to show we're done
    while (true) {
        gpio_put(LED_PIN, 1);
        sleep_ms(1000);
        gpio_put(LED_PIN, 0);
        sleep_ms(1000);
    }

    return 0;
}
