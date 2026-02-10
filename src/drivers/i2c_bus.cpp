/**
 * @file i2c_bus.c
 * @brief I2C bus driver implementation
 */

#include "i2c_bus.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <stdio.h>

// I2C bus scan range (7-bit addressing: 0x08–0x77)
constexpr uint8_t kI2cScanStart        = 0x08;
constexpr uint8_t kI2cScanEnd          = 0x78;  // Exclusive upper bound

// Known alternate device addresses for bus scan identification
constexpr uint8_t kI2cAddrIcm20948Alt  = 0x68;  // ICM-20948 with AD0=LOW
constexpr uint8_t kI2cAddrDps310Alt    = 0x76;  // DPS310 alternate address

// Bus recovery constants (I2C specification)
constexpr uint8_t  kBusRecoveryCycles  = 9;     // 9 clock pulses per I2C spec
constexpr uint32_t kBusRecoveryPulseUs = 5;     // Half-period for recovery clock

// ============================================================================
// Private State
// ============================================================================

static bool g_initialized = false;

// ============================================================================
// Initialization
// ============================================================================

bool i2c_bus_init() {
    if (g_initialized) {
        return true;
    }

    // Bus recovery BEFORE i2c_init: if a previous session was interrupted
    // (e.g., picotool --force reboot), a sensor may be holding SDA low
    // mid-transaction. Clock out the stuck byte per I2C spec.
    i2c_bus_recover();

    // Initialize I2C peripheral
    uint actualFreq = i2c_init(I2C_BUS_INSTANCE, kI2cBusFreqHz);
    if (actualFreq == 0) {
        return false;
    }

    // Configure GPIO pins for I2C function
    gpio_set_function(kI2cBusSdaPin, GPIO_FUNC_I2C);
    gpio_set_function(kI2cBusSclPin, GPIO_FUNC_I2C);

    // Enable internal pull-ups (external pull-ups on STEMMA QT are also present)
    gpio_pull_up(kI2cBusSdaPin);
    gpio_pull_up(kI2cBusSclPin);

    g_initialized = true;
    return true;
}

void i2c_bus_deinit() {
    if (!g_initialized) {
        return;
    }

    i2c_deinit(I2C_BUS_INSTANCE);
    gpio_set_function(kI2cBusSdaPin, GPIO_FUNC_NULL);
    gpio_set_function(kI2cBusSclPin, GPIO_FUNC_NULL);

    g_initialized = false;
}

bool i2c_bus_probe(uint8_t addr) {
    if (!g_initialized) {
        return false;
    }

    // Try to read a single byte - if device ACKs, it's present
    uint8_t dummy = 0;
    int ret = i2c_read_timeout_us(I2C_BUS_INSTANCE, addr, &dummy, 1, false, kI2cTimeoutUs);
    return (ret >= 0);
}

void i2c_bus_scan() {
    if (!g_initialized) {
        printf("I2C bus not initialized\n");
        return;
    }

    // Debug: check GPIO pin states (both should be HIGH with pull-ups)
    printf("I2C bus scan:\n");
    printf("  Instance: I2C%d\n", I2C_BUS_INSTANCE == i2c0 ? 0 : 1);
    printf("  SDA=GPIO%d (state=%d), SCL=GPIO%d (state=%d)\n",
           kI2cBusSdaPin, gpio_get(kI2cBusSdaPin),
           kI2cBusSclPin, gpio_get(kI2cBusSclPin));
    printf("  Configured freq: %lu Hz\n", (unsigned long)kI2cBusFreqHz);

    int found = 0;

    for (uint8_t addr = kI2cScanStart; addr < kI2cScanEnd; addr++) {
        // Skip PA1010D GPS — probing triggers I2C bus interference
        // (Pico SDK issue #252). Re-enable at IVP-31.
        if ((addr != kI2cAddrPa1010d) && i2c_bus_probe(addr)) {
            printf("  0x%02X", addr);

            // Identify known devices
            switch (addr) {
                case kI2cAddrDps310:
                    printf(" (DPS310 Barometer)");
                    break;
                case kI2cAddrIcm20948:
                    printf(" (ICM-20948 IMU)");
                    break;
                case kI2cAddrAk09916:
                    printf(" (AK09916 Magnetometer)");
                    break;
                case kI2cAddrPa1010d:
                    printf(" (PA1010D GPS)");
                    break;
                case kI2cAddrIcm20948Alt:
                    printf(" (ICM-20948 IMU, AD0=LOW)");
                    break;
                case kI2cAddrDps310Alt:
                    printf(" (DPS310 Barometer, alt addr)");
                    break;
                default:
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
    if (!g_initialized || data == nullptr || len == 0) {
        return -1;
    }

    int ret = i2c_write_timeout_us(I2C_BUS_INSTANCE, addr, data, len, false, kI2cTimeoutUs);
    return ret;
}

int i2c_bus_read(uint8_t addr, uint8_t* data, size_t len) {
    if (!g_initialized || data == nullptr || len == 0) {
        return -1;
    }

    int ret = i2c_read_timeout_us(I2C_BUS_INSTANCE, addr, data, len, false, kI2cTimeoutUs);
    return ret;
}

int i2c_bus_write_read(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
    if (!g_initialized || data == nullptr || len == 0) {
        return -1;
    }

    // Write register address (with repeated start - nostop=true)
    int ret = i2c_write_timeout_us(I2C_BUS_INSTANCE, addr, &reg, 1, true, kI2cTimeoutUs);
    if (ret < 0) {
        return ret;
    }

    // Read data
    ret = i2c_read_timeout_us(I2C_BUS_INSTANCE, addr, data, len, false, kI2cTimeoutUs);
    return ret;
}

int i2c_bus_write_reg(uint8_t addr, uint8_t reg, uint8_t value) {
    if (!g_initialized) {
        return -1;
    }

    uint8_t buf[2] = {reg, value};
    int ret = i2c_write_timeout_us(I2C_BUS_INSTANCE, addr, buf, 2, false, kI2cTimeoutUs);
    return (ret == 2) ? 0 : -1;
}

int i2c_bus_read_reg(uint8_t addr, uint8_t reg, uint8_t* value) {
    if (!g_initialized || value == nullptr) {
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

bool i2c_bus_recover() {
    // Disable I2C peripheral BEFORE switching GPIO functions.
    // Switching GPIO from I2C to SIO while the DW_apb_i2c is enabled
    // corrupts the peripheral's internal state machine — all subsequent
    // transactions silently fail. (Discovered 2026-02-10, see LL Entry 28.)
    i2c_deinit(I2C_BUS_INSTANCE);

    // Switch pins to GPIO mode for bit-banging
    gpio_set_function(kI2cBusSdaPin, GPIO_FUNC_SIO);
    gpio_set_function(kI2cBusSclPin, GPIO_FUNC_SIO);

    // Configure SDA as input (to read its state)
    gpio_set_dir(kI2cBusSdaPin, GPIO_IN);
    gpio_pull_up(kI2cBusSdaPin);

    // Configure SCL as output, drive high
    gpio_set_dir(kI2cBusSclPin, GPIO_OUT);
    gpio_put(kI2cBusSclPin, true);
    sleep_us(kBusRecoveryPulseUs);

    // Check SCL stuck low: if another device holds SCL, clock pulses
    // can't propagate — recovery is impossible (Linux kernel pattern)
    gpio_set_dir(kI2cBusSclPin, GPIO_IN);
    bool sclHigh = gpio_get(kI2cBusSclPin);
    gpio_set_dir(kI2cBusSclPin, GPIO_OUT);
    gpio_put(kI2cBusSclPin, true);

    bool sdaReleased = false;

    if (!sclHigh) {
        // SCL stuck low — a device is clock-stretching or bus is shorted.
        // Can't recover via clocking. Caller should consider power cycle.
        sdaReleased = false;
    } else {
        // Clock up to 9 pulses (NXP UM10204 3.1.16). Check SDA after
        // each rising SCL edge — early exit if SDA released (Linux kernel
        // pattern). Most stuck transactions clear in 1-3 pulses.
        for (uint8_t i = 0; i < kBusRecoveryCycles; i++) {
            gpio_put(kI2cBusSclPin, false);
            sleep_us(kBusRecoveryPulseUs);
            gpio_put(kI2cBusSclPin, true);
            sleep_us(kBusRecoveryPulseUs);

            if (gpio_get(kI2cBusSdaPin)) {
                sdaReleased = true;
                break;
            }
        }

        if (!sdaReleased) {
            sdaReleased = gpio_get(kI2cBusSdaPin);
        }
    }

    // Generate STOP condition: SDA low while SCL high, then SDA high
    gpio_set_dir(kI2cBusSdaPin, GPIO_OUT);
    gpio_put(kI2cBusSdaPin, false);
    sleep_us(kBusRecoveryPulseUs);
    gpio_put(kI2cBusSclPin, true);  // Ensure SCL high
    sleep_us(kBusRecoveryPulseUs);
    gpio_put(kI2cBusSdaPin, true);  // SDA rising = STOP
    sleep_us(kBusRecoveryPulseUs);

    // Restore GPIO to I2C function
    gpio_set_function(kI2cBusSdaPin, GPIO_FUNC_I2C);
    gpio_set_function(kI2cBusSclPin, GPIO_FUNC_I2C);
    gpio_pull_up(kI2cBusSdaPin);
    gpio_pull_up(kI2cBusSclPin);

    // Reinitialize I2C peripheral (configures clock, enables controller)
    i2c_init(I2C_BUS_INSTANCE, kI2cBusFreqHz);

    return sdaReleased;
}

bool i2c_bus_reset() {
    // i2c_bus_recover() handles deinit/reinit of the peripheral internally.
    // We just need to reset the initialized flag and restore it after.
    g_initialized = false;

    bool recovered = i2c_bus_recover();

    g_initialized = true;
    return recovered;
}
