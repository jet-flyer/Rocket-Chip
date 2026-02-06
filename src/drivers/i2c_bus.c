/**
 * @file i2c_bus.c
 * @brief I2C bus driver implementation
 */

#include "i2c_bus.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <stdio.h>

// ============================================================================
// Private State
// ============================================================================

static bool g_initialized = false;

// ============================================================================
// Initialization
// ============================================================================

bool i2c_bus_init(void) {
    if (g_initialized) {
        return true;
    }

    // Bus recovery BEFORE i2c_init: if a previous session was interrupted
    // (e.g., picotool --force reboot), a sensor may be holding SDA low
    // mid-transaction. Clock out the stuck byte per I2C spec.
    i2c_bus_recover();

    // Initialize I2C peripheral
    uint actual_freq = i2c_init(I2C_BUS_INSTANCE, I2C_BUS_FREQ_HZ);
    if (actual_freq == 0) {
        return false;
    }

    // Configure GPIO pins for I2C function
    gpio_set_function(I2C_BUS_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_BUS_SCL_PIN, GPIO_FUNC_I2C);

    // Enable internal pull-ups (external pull-ups on STEMMA QT are also present)
    gpio_pull_up(I2C_BUS_SDA_PIN);
    gpio_pull_up(I2C_BUS_SCL_PIN);

    g_initialized = true;
    return true;
}

void i2c_bus_deinit(void) {
    if (!g_initialized) {
        return;
    }

    i2c_deinit(I2C_BUS_INSTANCE);
    gpio_set_function(I2C_BUS_SDA_PIN, GPIO_FUNC_NULL);
    gpio_set_function(I2C_BUS_SCL_PIN, GPIO_FUNC_NULL);

    g_initialized = false;
}

bool i2c_bus_probe(uint8_t addr) {
    if (!g_initialized) {
        return false;
    }

    // Try to read a single byte - if device ACKs, it's present
    uint8_t dummy;
    int ret = i2c_read_timeout_us(I2C_BUS_INSTANCE, addr, &dummy, 1, false, I2C_TIMEOUT_US);
    return (ret >= 0);
}

void i2c_bus_scan(void) {
    if (!g_initialized) {
        printf("I2C bus not initialized\n");
        return;
    }

    // Debug: check GPIO pin states (both should be HIGH with pull-ups)
    printf("I2C bus scan:\n");
    printf("  Instance: I2C%d\n", I2C_BUS_INSTANCE == i2c0 ? 0 : 1);
    printf("  SDA=GPIO%d (state=%d), SCL=GPIO%d (state=%d)\n",
           I2C_BUS_SDA_PIN, gpio_get(I2C_BUS_SDA_PIN),
           I2C_BUS_SCL_PIN, gpio_get(I2C_BUS_SCL_PIN));
    printf("  Configured freq: %d Hz\n", I2C_BUS_FREQ_HZ);

    int found = 0;

    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_bus_probe(addr)) {
            printf("  0x%02X", addr);

            // Identify known devices
            switch (addr) {
                case I2C_ADDR_DPS310:
                    printf(" (DPS310 Barometer)");
                    break;
                case I2C_ADDR_ICM20948:
                    printf(" (ICM-20948 IMU)");
                    break;
                case I2C_ADDR_AK09916:
                    printf(" (AK09916 Magnetometer)");
                    break;
                case I2C_ADDR_PA1010D:
                    printf(" (PA1010D GPS)");
                    break;
                case 0x68:
                    printf(" (ICM-20948 IMU, AD0=LOW)");
                    break;
                case 0x76:
                    printf(" (DPS310 Barometer, alt addr)");
                    break;
            }
            printf("\n");
            found++;
        }
    }

    if (found == 0) {
        printf("  No devices found\n");
    } else {
        printf("  %d device(s) found\n", found);
    }
}

// ============================================================================
// Read/Write Operations
// ============================================================================

int i2c_bus_write(uint8_t addr, const uint8_t* data, size_t len) {
    if (!g_initialized || data == NULL || len == 0) {
        return -1;
    }

    int ret = i2c_write_timeout_us(I2C_BUS_INSTANCE, addr, data, len, false, I2C_TIMEOUT_US);
    return ret;
}

int i2c_bus_read(uint8_t addr, uint8_t* data, size_t len) {
    if (!g_initialized || data == NULL || len == 0) {
        return -1;
    }

    int ret = i2c_read_timeout_us(I2C_BUS_INSTANCE, addr, data, len, false, I2C_TIMEOUT_US);
    return ret;
}

int i2c_bus_write_read(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
    if (!g_initialized || data == NULL || len == 0) {
        return -1;
    }

    // Write register address (with repeated start - nostop=true)
    int ret = i2c_write_timeout_us(I2C_BUS_INSTANCE, addr, &reg, 1, true, I2C_TIMEOUT_US);
    if (ret < 0) {
        return ret;
    }

    // Read data
    ret = i2c_read_timeout_us(I2C_BUS_INSTANCE, addr, data, len, false, I2C_TIMEOUT_US);
    return ret;
}

int i2c_bus_write_reg(uint8_t addr, uint8_t reg, uint8_t value) {
    if (!g_initialized) {
        return -1;
    }

    uint8_t buf[2] = {reg, value};
    int ret = i2c_write_timeout_us(I2C_BUS_INSTANCE, addr, buf, 2, false, I2C_TIMEOUT_US);
    return (ret == 2) ? 0 : -1;
}

int i2c_bus_read_reg(uint8_t addr, uint8_t reg, uint8_t* value) {
    if (!g_initialized || value == NULL) {
        return -1;
    }

    int ret = i2c_bus_write_read(addr, reg, value, 1);
    return (ret == 1) ? 0 : -1;
}

int i2c_bus_read_regs(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
    return i2c_bus_write_read(addr, reg, data, len);
}

// ============================================================================
// Bus Recovery (IVP-13a)
// ============================================================================

bool i2c_bus_recover(void) {
    // Temporarily switch pins to GPIO mode for bit-banging
    gpio_set_function(I2C_BUS_SDA_PIN, GPIO_FUNC_SIO);
    gpio_set_function(I2C_BUS_SCL_PIN, GPIO_FUNC_SIO);

    // Configure SDA as input (to read its state)
    gpio_set_dir(I2C_BUS_SDA_PIN, GPIO_IN);
    gpio_pull_up(I2C_BUS_SDA_PIN);

    // Configure SCL as output
    gpio_set_dir(I2C_BUS_SCL_PIN, GPIO_OUT);
    gpio_put(I2C_BUS_SCL_PIN, 1);

    // Always clock 9 pulses to clear any stuck transaction,
    // even if SDA appears high — a sensor may need the full
    // sequence to reset its internal state machine
    for (int i = 0; i < 9; i++) {
        gpio_put(I2C_BUS_SCL_PIN, 0);
        sleep_us(5);
        gpio_put(I2C_BUS_SCL_PIN, 1);
        sleep_us(5);
    }

    bool sda_released = gpio_get(I2C_BUS_SDA_PIN);

    // Always generate STOP condition: SDA low while SCL high, then SDA high
    gpio_set_dir(I2C_BUS_SDA_PIN, GPIO_OUT);
    gpio_put(I2C_BUS_SDA_PIN, 0);
    sleep_us(5);
    // SCL high (already high)
    sleep_us(5);
    // SDA high = STOP condition
    gpio_put(I2C_BUS_SDA_PIN, 1);
    sleep_us(5);

    // Restore I2C function on pins
    gpio_set_function(I2C_BUS_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_BUS_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_BUS_SDA_PIN);
    gpio_pull_up(I2C_BUS_SCL_PIN);

    return sda_released;
}

bool i2c_bus_reset(void) {
    // Deinitialize I2C
    i2c_bus_deinit();

    // Attempt recovery
    bool recovered = i2c_bus_recover();

    // Reinitialize I2C
    bool init_ok = i2c_bus_init();

    return recovered && init_ok;
}
