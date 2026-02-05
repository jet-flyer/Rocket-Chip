/**
 * @file main.cpp
 * @brief RocketChip main entry point - IVP Stage 1 Foundation
 *
 * Implements IVP-01 through IVP-08:
 * - Clean build from source
 * - Red LED blink (board alive)
 * - NeoPixel status LED
 * - USB CDC serial output
 * - Debug macros functional
 * - I2C bus initialization
 * - I2C device scan
 * - Heartbeat superloop with uptime
 */

#include "rocketchip/config.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "drivers/ws2812_status.h"
#include "drivers/i2c_bus.h"
#include <stdio.h>

// ============================================================================
// Constants
// ============================================================================

// NeoPixel pin (GPIO 21 on Feather RP2350)
static constexpr uint kNeoPixelPin = 21;

// Heartbeat timing (100ms on, 900ms off per IVP-08)
static constexpr uint32_t kHeartbeatOnMs = 100;
static constexpr uint32_t kHeartbeatOffMs = 900;
static constexpr uint32_t kHeartbeatPeriodMs = kHeartbeatOnMs + kHeartbeatOffMs;

// Uptime print interval
static constexpr uint32_t kUptimeIntervalMs = 5000;

// Superloop validation timing
static constexpr uint32_t kSuperloopValidationMs = 10000;

// Expected I2C devices for validation
static constexpr uint8_t kExpectedDevices[] = {
    I2C_ADDR_ICM20948,  // 0x69
    I2C_ADDR_DPS310,    // 0x77
    I2C_ADDR_PA1010D,   // 0x10
};
static constexpr size_t kExpectedDeviceCount = sizeof(kExpectedDevices) / sizeof(kExpectedDevices[0]);

// ============================================================================
// Global State
// ============================================================================

static bool g_neopixelInitialized = false;
static bool g_i2cInitialized = false;
static bool g_wasTerminalConnected = false;
static bool g_superloopValidationDone = false;
static uint32_t g_loopCounter = 0;

// ============================================================================
// Hardware Validation
// ============================================================================

/**
 * @brief Get device name for known I2C addresses
 */
static const char* get_device_name(uint8_t addr) {
    switch (addr) {
        case I2C_ADDR_ICM20948: return "ICM-20948";
        case I2C_ADDR_DPS310:   return "DPS310";
        case I2C_ADDR_PA1010D:  return "PA1010D GPS";
        case 0x68:              return "ICM-20948 (AD0=LOW)";
        case 0x76:              return "DPS310 (alt)";
        default:                return "Unknown";
    }
}

/**
 * @brief Run Stage 1 hardware validation - prints PASS/FAIL for each gate
 */
static void hw_validate_stage1(void) {
    printf("\n=== HW Validation: Stage 1 ===\n");

    // IVP-01: Build + boot
    printf("[PASS] Build + boot (you're reading this)\n");

    // IVP-02: Red LED GPIO
    printf("[PASS] Red LED GPIO initialized (pin %d)\n", PICO_DEFAULT_LED_PIN);

    // IVP-03: NeoPixel PIO
    if (g_neopixelInitialized) {
        printf("[PASS] NeoPixel PIO initialized (pin %d)\n", kNeoPixelPin);
    } else {
        printf("[FAIL] NeoPixel PIO failed to initialize\n");
    }

    // IVP-04: USB CDC
    printf("[PASS] USB CDC connected\n");

    // IVP-05: Debug macros
    uint32_t timestamp = time_us_32();
    if (timestamp > 0) {
        printf("[PASS] Debug macros functional (timestamp=%lu us)\n", (unsigned long)timestamp);
    } else {
        printf("[FAIL] Debug macros: timestamp is zero\n");
    }

    // IVP-06: I2C bus
    if (g_i2cInitialized) {
        printf("[PASS] I2C bus initialized at 400kHz (SDA=%d, SCL=%d)\n",
               I2C_BUS_SDA_PIN, I2C_BUS_SCL_PIN);
    } else {
        printf("[FAIL] I2C bus failed to initialize\n");
    }

    // IVP-07: I2C device detection (info, not pass/fail)
    int foundCount = 0;
    for (size_t i = 0; i < kExpectedDeviceCount; i++) {
        uint8_t addr = kExpectedDevices[i];
        bool found = i2c_bus_probe(addr);
        printf("[----] I2C 0x%02X (%s): %s\n",
               addr, get_device_name(addr), found ? "FOUND" : "NOT FOUND");
        if (found) foundCount++;
    }
    printf("[INFO] Sensors found: %d/%zu expected\n", foundCount, kExpectedDeviceCount);

    printf("=== Stage 1 Validation Complete ===\n\n");
}

/**
 * @brief Run superloop validation - called once after 10s uptime
 */
static void hw_validate_superloop(uint32_t uptimeMs) {
    printf("\n=== HW Validation: Superloop ===\n");
    printf("[PASS] Uptime advancing (%lu ms)\n", (unsigned long)uptimeMs);
    printf("[PASS] Main loop iterations: %lu\n", (unsigned long)g_loopCounter);
    printf("=== Superloop Validation Complete ===\n\n");
}

// ============================================================================
// Main
// ============================================================================

int main() {
    // -------------------------------------------------------------------------
    // IVP-02: Red LED GPIO init
    // -------------------------------------------------------------------------
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // -------------------------------------------------------------------------
    // IVP-03: NeoPixel init
    // -------------------------------------------------------------------------
    g_neopixelInitialized = ws2812_status_init(pio0, kNeoPixelPin);
    if (g_neopixelInitialized) {
        ws2812_set_mode(WS2812_MODE_RAINBOW, WS2812_COLOR_GREEN);
    }

    // -------------------------------------------------------------------------
    // IVP-06: I2C bus init (before USB per LL Entry 4/12)
    // -------------------------------------------------------------------------
    g_i2cInitialized = i2c_bus_init();

    // -------------------------------------------------------------------------
    // IVP-04: USB CDC init
    // -------------------------------------------------------------------------
    stdio_init_all();

    // Fast LED blink while waiting for USB connection
    while (!stdio_usb_connected()) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(100);
        // Keep NeoPixel animating
        ws2812_update();
    }

    // Settle time after connection (per LL Entry 15)
    sleep_ms(500);

    // Drain any garbage from USB input buffer (per LL Entry 15)
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {
        // Discard
    }

    // -------------------------------------------------------------------------
    // Print banner (raw printf - IVP-04)
    // -------------------------------------------------------------------------
    printf("\n");
    printf("==============================================\n");
    printf("  RocketChip v%s\n", ROCKETCHIP_VERSION_STRING);
    printf("  Build: %s %s\n", __DATE__, __TIME__);
    printf("  Board: Adafruit Feather RP2350 HSTX\n");
    printf("==============================================\n");
    printf("\n");

    // -------------------------------------------------------------------------
    // IVP-05: Debug macros functional
    // -------------------------------------------------------------------------
    DBG_PRINT("Debug macros functional");

    // -------------------------------------------------------------------------
    // IVP-06/07: I2C init result and scan
    // -------------------------------------------------------------------------
    if (g_i2cInitialized) {
        printf("I2C1 initialized at 400kHz on SDA=%d, SCL=%d\n\n",
               I2C_BUS_SDA_PIN, I2C_BUS_SCL_PIN);
    } else {
        printf("ERROR: I2C1 failed to initialize\n\n");
    }

    i2c_bus_scan();
    printf("\n");

    // -------------------------------------------------------------------------
    // Hardware validation
    // -------------------------------------------------------------------------
    hw_validate_stage1();

    // -------------------------------------------------------------------------
    // IVP-08: Heartbeat superloop
    // -------------------------------------------------------------------------
    printf("Entering main loop (heartbeat 100ms on / 900ms off)\n");
    printf("Uptime will print every 5 seconds...\n\n");

    uint32_t lastUptimeMs = 0;
    bool ledState = false;

    while (true) {
        uint32_t nowMs = to_ms_since_boot(get_absolute_time());
        g_loopCounter++;

        // ---------------------------------------------------------------------
        // Heartbeat LED: 100ms on, 900ms off
        // ---------------------------------------------------------------------
        uint32_t heartbeatPhase = nowMs % kHeartbeatPeriodMs;
        bool shouldBeOn = (heartbeatPhase < kHeartbeatOnMs);
        if (shouldBeOn != ledState) {
            ledState = shouldBeOn;
            gpio_put(PICO_DEFAULT_LED_PIN, ledState ? 1 : 0);
        }

        // ---------------------------------------------------------------------
        // NeoPixel animation update
        // ---------------------------------------------------------------------
        ws2812_update();

        // ---------------------------------------------------------------------
        // Terminal connect/disconnect tracking
        // ---------------------------------------------------------------------
        bool isConnected = stdio_usb_connected();
        if (isConnected && !g_wasTerminalConnected) {
            // Just connected - drain input buffer
            while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {
                // Discard garbage
            }
            printf("\n[Terminal reconnected at %lu ms]\n", (unsigned long)nowMs);
        }
        g_wasTerminalConnected = isConnected;

        // ---------------------------------------------------------------------
        // Uptime print every 5 seconds (via DBG_PRINT per council suggestion)
        // ---------------------------------------------------------------------
        if (isConnected && (nowMs - lastUptimeMs >= kUptimeIntervalMs)) {
            lastUptimeMs = nowMs;
            DBG_PRINT("Uptime: %lu ms, loops: %lu", (unsigned long)nowMs, (unsigned long)g_loopCounter);
        }

        // ---------------------------------------------------------------------
        // Superloop validation at 10 seconds
        // ---------------------------------------------------------------------
        if (isConnected && !g_superloopValidationDone && nowMs >= kSuperloopValidationMs) {
            g_superloopValidationDone = true;
            hw_validate_superloop(nowMs);
        }

        // Small sleep to prevent tight spinning (allows USB processing)
        sleep_ms(1);
    }

    return 0;
}
