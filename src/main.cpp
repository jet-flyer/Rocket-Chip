/**
 * @file main.cpp
 * @brief RocketChip main entry point - IVP Stage 2
 *
 * Implements IVP-01 through IVP-09:
 *   IVP-01: Clean build from source
 *   IVP-02: Red LED heartbeat (100ms on / 900ms off)
 *   IVP-03: NeoPixel rainbow via PIO
 *   IVP-04: USB CDC serial output
 *   IVP-05: Debug macros functional
 *   IVP-06: I2C bus init at 400kHz with bus recovery
 *   IVP-07: I2C scan (detect all connected sensors)
 *   IVP-08: Heartbeat superloop with uptime
 *   IVP-09: ICM-20948 IMU initialization
 */

#include "rocketchip/config.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "drivers/ws2812_status.h"
#include "drivers/i2c_bus.h"
#include "drivers/icm20948.h"
#include <stdio.h>

// ============================================================================
// Constants
// ============================================================================

static constexpr uint kNeoPixelPin = 21;

// Heartbeat: 100ms on, 900ms off (per IVP-08)
static constexpr uint32_t kHeartbeatOnMs = 100;
static constexpr uint32_t kHeartbeatOffMs = 900;
static constexpr uint32_t kHeartbeatPeriodMs = kHeartbeatOnMs + kHeartbeatOffMs;

// Superloop validation at 10 seconds
static constexpr uint32_t kSuperloopValidationMs = 10000;

// ============================================================================
// Global State
// ============================================================================

static bool g_neopixelInitialized = false;
static bool g_i2cInitialized = false;
static bool g_imuInitialized = false;
static bool g_superloopValidationDone = false;
static uint32_t g_loopCounter = 0;

// IMU device handle (static per LL Entry 1 — avoid large objects on stack)
static icm20948_t g_imu;

// ============================================================================
// Hardware Validation
// ============================================================================

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

static void hw_validate_stage1(void) {
    printf("\n=== HW Validation: Stage 1 ===\n");

    // IVP-01: Build + boot
    printf("[PASS] Build + boot (you're reading this)\n");

    // IVP-02: Red LED
    printf("[PASS] Red LED GPIO initialized (pin %d)\n", PICO_DEFAULT_LED_PIN);

    // IVP-03: NeoPixel
    printf("[%s] NeoPixel PIO initialized (pin %d)\n",
           g_neopixelInitialized ? "PASS" : "FAIL", kNeoPixelPin);

    // IVP-04: USB CDC
    printf("[PASS] USB CDC connected\n");

    // IVP-05: Debug macros
    uint32_t ts = time_us_32();
    printf("[%s] Debug macros functional (timestamp=%lu us)\n",
           ts > 0 ? "PASS" : "FAIL", (unsigned long)ts);

    // IVP-06: I2C bus
    if (g_i2cInitialized) {
        printf("[PASS] I2C bus initialized at %dkHz (SDA=%d, SCL=%d)\n",
               I2C_BUS_FREQ_HZ / 1000, I2C_BUS_SDA_PIN, I2C_BUS_SCL_PIN);
    } else {
        printf("[FAIL] I2C bus failed to initialize\n");
    }

    // IVP-07: I2C device detection
    static const uint8_t expected[] = {
        I2C_ADDR_ICM20948,  // 0x69
        I2C_ADDR_DPS310,    // 0x77
        I2C_ADDR_PA1010D,   // 0x10
    };
    int foundCount = 0;
    for (size_t i = 0; i < sizeof(expected) / sizeof(expected[0]); i++) {
        bool found = i2c_bus_probe(expected[i]);
        printf("[----] I2C 0x%02X (%s): %s\n",
               expected[i], get_device_name(expected[i]),
               found ? "FOUND" : "NOT FOUND");
        if (found) foundCount++;
    }
    printf("[INFO] Sensors found: %d/%zu expected\n",
           foundCount, sizeof(expected) / sizeof(expected[0]));

    // IVP-09: IMU initialization
    if (g_imuInitialized) {
        printf("[PASS] ICM-20948 init (WHO_AM_I=0xEA)\n");
        printf("[%s] AK09916 magnetometer %s\n",
               g_imu.mag_initialized ? "PASS" : "WARN",
               g_imu.mag_initialized ? "ready" : "not ready");
    } else {
        printf("[FAIL] ICM-20948 init failed\n");
    }

    printf("=== Validation Complete ===\n\n");
}

static void hw_validate_superloop(uint32_t uptimeMs) {
    printf("\n=== HW Validation: Superloop (IVP-08) ===\n");
    printf("[PASS] Uptime advancing (%lu ms)\n", (unsigned long)uptimeMs);
    printf("[PASS] Main loop iterations: %lu\n", (unsigned long)g_loopCounter);
    printf("=== Superloop Validation Complete ===\n\n");
}

// ============================================================================
// Main
// ============================================================================

int main() {
    // -----------------------------------------------------------------
    // IVP-02: Red LED GPIO init
    // -----------------------------------------------------------------
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // -----------------------------------------------------------------
    // IVP-03: NeoPixel init
    // -----------------------------------------------------------------
    g_neopixelInitialized = ws2812_status_init(pio0, kNeoPixelPin);
    if (g_neopixelInitialized) {
        ws2812_set_mode(WS2812_MODE_RAINBOW, WS2812_COLOR_GREEN);
    }

    // -----------------------------------------------------------------
    // IVP-06: I2C bus init (before USB per LL Entry 4/12)
    // i2c_bus_init() includes bus recovery to handle stuck bus
    // from previous picotool reboot
    // -----------------------------------------------------------------
    g_i2cInitialized = i2c_bus_init();

    // Sensor power-up settling time
    // ICM-20948 datasheet: 11ms, DPS310: 40ms, generous margin
    sleep_ms(200);

    // -----------------------------------------------------------------
    // IVP-09: ICM-20948 IMU init (before USB, after I2C)
    // Accel ±4g, Gyro ±500dps, Mag continuous 100Hz
    // -----------------------------------------------------------------
    if (g_i2cInitialized) {
        g_imuInitialized = icm20948_init(&g_imu, ICM20948_ADDR_DEFAULT);
    }

    // -----------------------------------------------------------------
    // IVP-04: USB CDC init (after I2C/flash per LL Entry 4/12)
    // -----------------------------------------------------------------
    stdio_init_all();

    // Fast LED blink while waiting for USB connection
    while (!stdio_usb_connected()) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(100);
        ws2812_update();
    }

    // Settle time after connection (per LL Entry 15)
    sleep_ms(500);

    // Drain garbage from USB input buffer (per LL Entry 15)
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {}

    // -----------------------------------------------------------------
    // Banner
    // -----------------------------------------------------------------
    printf("\n");
    printf("==============================================\n");
    printf("  RocketChip v%s\n", ROCKETCHIP_VERSION_STRING);
    printf("  Build: %s %s\n", __DATE__, __TIME__);
    printf("  Board: Adafruit Feather RP2350 HSTX\n");
    printf("==============================================\n\n");

    // IVP-05: Debug macros
    DBG_PRINT("Debug macros functional");

    // IVP-06/07: I2C status and scan
    if (g_i2cInitialized) {
        printf("I2C1 initialized at %dkHz on SDA=%d, SCL=%d\n\n",
               I2C_BUS_FREQ_HZ / 1000, I2C_BUS_SDA_PIN, I2C_BUS_SCL_PIN);
    } else {
        printf("ERROR: I2C1 failed to initialize\n\n");
    }
    i2c_bus_scan();
    printf("\n");

    // IVP-09: IMU status
    if (g_imuInitialized) {
        printf("ICM-20948 init OK (WHO_AM_I=0xEA)\n");
        printf("  Accel: +/-%dg, Gyro: +/-%d dps\n",
               (2 << g_imu.accel_fs), 250 * (1 << g_imu.gyro_fs));
        printf("  Mag: %s (AK09916 via I2C master)\n",
               g_imu.mag_initialized ? "OK, continuous 100Hz" : "NOT READY");
    } else {
        printf("ICM-20948 init FAILED\n");
    }
    printf("\n");

    // Post-init I2C scan — verify bus integrity after IMU init
    printf("Post-init I2C scan (bus integrity check):\n");
    i2c_bus_scan();
    printf("\n");

    // Hardware validation
    hw_validate_stage1();

    // IVP-08: Superloop
    printf("Entering main loop\n\n");

    bool ledState = false;

    while (true) {
        uint32_t nowMs = to_ms_since_boot(get_absolute_time());
        g_loopCounter++;

        // Heartbeat LED: 100ms on, 900ms off
        uint32_t phase = nowMs % kHeartbeatPeriodMs;
        bool shouldBeOn = (phase < kHeartbeatOnMs);
        if (shouldBeOn != ledState) {
            ledState = shouldBeOn;
            gpio_put(PICO_DEFAULT_LED_PIN, ledState ? 1 : 0);
        }

        // NeoPixel animation
        ws2812_update();

        // Superloop validation at 10s
        if (stdio_usb_connected() && !g_superloopValidationDone &&
            nowMs >= kSuperloopValidationMs) {
            g_superloopValidationDone = true;
            hw_validate_superloop(nowMs);
        }

        // Small sleep to prevent tight spinning (allows USB processing)
        sleep_ms(1);
    }

    return 0;
}
