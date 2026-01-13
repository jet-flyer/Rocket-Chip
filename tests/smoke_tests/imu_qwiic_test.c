/**
 * @file imu_qwiic_test.c
 * @brief Smoke test to verify ISM330DHCX + LIS3MDL FeatherWing Qwiic port
 *
 * This bare-metal test verifies that the Qwiic (STEMMA QT) connector on the
 * ISM330DHCX + LIS3MDL FeatherWing is connected to the onboard sensors and
 * not just a pass-through connector.
 *
 * Test procedure:
 * 1. Initialize I2C on STEMMA QT pins
 * 2. Scan for ISM330DHCX at expected addresses (0x6A/0x6B)
 * 3. Scan for LIS3MDL at expected addresses (0x1C/0x1E)
 * 4. Read WHO_AM_I registers to confirm sensor identity
 * 5. Report PASS/FAIL status
 *
 * Expected WHO_AM_I values:
 * - ISM330DHCX: 0x6B at register 0x0F
 * - LIS3MDL: 0x3D at register 0x0F
 *
 * Hardware: Adafruit Feather RP2350 + ISM330DHCX/LIS3MDL FeatherWing (#4569)
 */

#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

/* I2C Configuration for Feather RP2350 STEMMA QT connector */
#define I2C_PORT        i2c1
#define I2C_SDA_PIN     2
#define I2C_SCL_PIN     3
#define I2C_FREQ_HZ     400000  /* 400kHz Fast Mode */

/* ISM330DHCX (6-DoF Accel/Gyro) */
#define ISM330DHCX_ADDR_PRIMARY     0x6A
#define ISM330DHCX_ADDR_SECONDARY   0x6B
#define ISM330DHCX_WHO_AM_I_REG     0x0F
#define ISM330DHCX_WHO_AM_I_VALUE   0x6B

/* LIS3MDL (3-axis Magnetometer) */
#define LIS3MDL_ADDR_PRIMARY        0x1C
#define LIS3MDL_ADDR_SECONDARY      0x1E
#define LIS3MDL_WHO_AM_I_REG        0x0F
#define LIS3MDL_WHO_AM_I_VALUE      0x3D

/* Test timeout */
#define I2C_TIMEOUT_US  10000

/* Test result tracking */
typedef struct {
    bool ism330dhcx_found;
    bool lis3mdl_found;
    uint8_t ism330dhcx_addr;
    uint8_t lis3mdl_addr;
    uint8_t ism330dhcx_who_am_i;
    uint8_t lis3mdl_who_am_i;
} test_results_t;

static test_results_t results = {0};

/**
 * @brief Check if a device responds at a given I2C address
 */
static bool i2c_device_present(uint8_t addr) {
    uint8_t dummy;
    int ret = i2c_read_timeout_us(I2C_PORT, addr, &dummy, 1, false, I2C_TIMEOUT_US);
    return (ret >= 0);
}

/**
 * @brief Read a single register from an I2C device
 */
static bool i2c_read_register(uint8_t addr, uint8_t reg, uint8_t *value) {
    int ret;

    ret = i2c_write_timeout_us(I2C_PORT, addr, &reg, 1, true, I2C_TIMEOUT_US);
    if (ret < 0) {
        return false;
    }

    ret = i2c_read_timeout_us(I2C_PORT, addr, value, 1, false, I2C_TIMEOUT_US);
    if (ret < 0) {
        return false;
    }

    return true;
}

/**
 * @brief Scan for ISM330DHCX at both possible addresses
 */
static bool find_ism330dhcx(void) {
    uint8_t who_am_i;
    uint8_t addresses[] = {ISM330DHCX_ADDR_PRIMARY, ISM330DHCX_ADDR_SECONDARY};

    for (size_t i = 0; i < sizeof(addresses); i++) {
        printf("  Checking ISM330DHCX at 0x%02X... ", addresses[i]);

        if (!i2c_device_present(addresses[i])) {
            printf("not present\n");
            continue;
        }

        if (!i2c_read_register(addresses[i], ISM330DHCX_WHO_AM_I_REG, &who_am_i)) {
            printf("read failed\n");
            continue;
        }

        printf("WHO_AM_I = 0x%02X ", who_am_i);

        if (who_am_i == ISM330DHCX_WHO_AM_I_VALUE) {
            printf("(MATCH)\n");
            results.ism330dhcx_found = true;
            results.ism330dhcx_addr = addresses[i];
            results.ism330dhcx_who_am_i = who_am_i;
            return true;
        }

        printf("(expected 0x%02X)\n", ISM330DHCX_WHO_AM_I_VALUE);
    }

    return false;
}

/**
 * @brief Scan for LIS3MDL at both possible addresses
 */
static bool find_lis3mdl(void) {
    uint8_t who_am_i;
    uint8_t addresses[] = {LIS3MDL_ADDR_PRIMARY, LIS3MDL_ADDR_SECONDARY};

    for (size_t i = 0; i < sizeof(addresses); i++) {
        printf("  Checking LIS3MDL at 0x%02X... ", addresses[i]);

        if (!i2c_device_present(addresses[i])) {
            printf("not present\n");
            continue;
        }

        if (!i2c_read_register(addresses[i], LIS3MDL_WHO_AM_I_REG, &who_am_i)) {
            printf("read failed\n");
            continue;
        }

        printf("WHO_AM_I = 0x%02X ", who_am_i);

        if (who_am_i == LIS3MDL_WHO_AM_I_VALUE) {
            printf("(MATCH)\n");
            results.lis3mdl_found = true;
            results.lis3mdl_addr = addresses[i];
            results.lis3mdl_who_am_i = who_am_i;
            return true;
        }

        printf("(expected 0x%02X)\n", LIS3MDL_WHO_AM_I_VALUE);
    }

    return false;
}

/**
 * @brief Perform a full I2C bus scan and report all devices
 */
static void i2c_bus_scan(void) {
    printf("\nI2C Bus Scan (0x08-0x77):\n");
    printf("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < 128; addr++) {
        if (addr % 16 == 0) {
            printf("%02X: ", addr);
        }

        /* Skip reserved addresses */
        if (addr < 0x08 || addr > 0x77) {
            printf("   ");
        } else if (i2c_device_present((uint8_t)addr)) {
            printf("%02X ", addr);
        } else {
            printf("-- ");
        }

        if (addr % 16 == 15) {
            printf("\n");
        }
    }
}

/**
 * @brief Print test summary and verdict
 */
static void print_summary(void) {
    printf("\n");
    printf("========================================\n");
    printf("TEST SUMMARY\n");
    printf("========================================\n");

    printf("\nISM330DHCX (Accel/Gyro):\n");
    if (results.ism330dhcx_found) {
        printf("  Status:    FOUND\n");
        printf("  Address:   0x%02X\n", results.ism330dhcx_addr);
        printf("  WHO_AM_I:  0x%02X (expected 0x%02X)\n",
               results.ism330dhcx_who_am_i, ISM330DHCX_WHO_AM_I_VALUE);
    } else {
        printf("  Status:    NOT FOUND\n");
    }

    printf("\nLIS3MDL (Magnetometer):\n");
    if (results.lis3mdl_found) {
        printf("  Status:    FOUND\n");
        printf("  Address:   0x%02X\n", results.lis3mdl_addr);
        printf("  WHO_AM_I:  0x%02X (expected 0x%02X)\n",
               results.lis3mdl_who_am_i, LIS3MDL_WHO_AM_I_VALUE);
    } else {
        printf("  Status:    NOT FOUND\n");
    }

    printf("\n========================================\n");
    if (results.ism330dhcx_found && results.lis3mdl_found) {
        printf("VERDICT: PASS\n");
        printf("\nThe Qwiic port IS connected to the onboard sensors.\n");
        printf("Both ISM330DHCX and LIS3MDL respond correctly via I2C.\n");
    } else if (results.ism330dhcx_found || results.lis3mdl_found) {
        printf("VERDICT: PARTIAL PASS\n");
        printf("\nOnly one sensor detected. Check connections.\n");
    } else {
        printf("VERDICT: FAIL\n");
        printf("\nNo sensors detected on Qwiic port.\n");
        printf("Possible causes:\n");
        printf("  - FeatherWing not connected\n");
        printf("  - Qwiic cable not seated properly\n");
        printf("  - Qwiic port is pass-through only (not wired to sensors)\n");
        printf("  - Wrong I2C pins configured\n");
    }
    printf("========================================\n");
}

int main(void) {
    /* Initialize stdio (USB serial) */
    stdio_init_all();

    /* Wait for USB connection (with timeout) */
    for (int i = 0; i < 50; i++) {
        if (stdio_usb_connected()) {
            break;
        }
        sleep_ms(100);
    }
    sleep_ms(500);  /* Extra delay for terminal to settle */

    /* Print header */
    printf("\n\n");
    printf("========================================\n");
    printf("IMU QWIIC PORT SMOKE TEST\n");
    printf("========================================\n");
    printf("\nPurpose: Verify that the Qwiic connector on\n");
    printf("ISM330DHCX + LIS3MDL FeatherWing (#4569) is\n");
    printf("connected to the onboard sensors.\n\n");

    /* Initialize I2C */
    printf("Initializing I2C...\n");
    printf("  Port: I2C%d\n", (I2C_PORT == i2c0) ? 0 : 1);
    printf("  SDA:  GPIO %d\n", I2C_SDA_PIN);
    printf("  SCL:  GPIO %d\n", I2C_SCL_PIN);
    printf("  Freq: %d Hz\n", I2C_FREQ_HZ);

    i2c_init(I2C_PORT, I2C_FREQ_HZ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    printf("  I2C initialized.\n");

    /* Perform bus scan first */
    i2c_bus_scan();

    /* Look for specific sensors */
    printf("\n----------------------------------------\n");
    printf("Searching for ISM330DHCX...\n");
    find_ism330dhcx();

    printf("\n----------------------------------------\n");
    printf("Searching for LIS3MDL...\n");
    find_lis3mdl();

    /* Print summary */
    print_summary();

    /* Blink LED to indicate test complete */
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    printf("\nTest complete. LED blinking.\n");

    while (1) {
        /* Blink pattern indicates result */
        if (results.ism330dhcx_found && results.lis3mdl_found) {
            /* PASS: slow steady blink */
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(500);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(500);
        } else {
            /* FAIL: fast double blink */
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(100);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(100);
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(100);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(700);
        }
    }

    return 0;
}
