/**
 * @file main.cpp
 * @brief RocketChip main entry point - IVP Stage 2
 *
 * Implements IVP-01 through IVP-18:
 * - Stage 1: Foundation (LED, NeoPixel, USB, I2C, debug)
 * - Stage 2: Sensors (IMU, Baro, Calibration, CLI)
 *
 * RC_OS CLI provides terminal-based calibration and status.
 */

#include "rocketchip/config.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "drivers/ws2812_status.h"
#include "drivers/i2c_bus.h"
#include "drivers/icm20948.h"
#include "drivers/baro_dps310.h"
#include "calibration/calibration_storage.h"
#include "calibration/calibration_manager.h"
#include "cli/rc_os.h"
#include <stdio.h>
#include <math.h>

// ============================================================================
// Constants
// ============================================================================

// NeoPixel pin (GPIO 21 on Feather RP2350)
static constexpr uint kNeoPixelPin = 21;

// Heartbeat timing (100ms on, 900ms off per IVP-08)
static constexpr uint32_t kHeartbeatOnMs = 100;
static constexpr uint32_t kHeartbeatOffMs = 900;
static constexpr uint32_t kHeartbeatPeriodMs = kHeartbeatOnMs + kHeartbeatOffMs;

// Sensor sample interval (100Hz for calibration feeding)
static constexpr uint32_t kSensorSampleIntervalMs = 10;

// CLI polling interval (~20Hz)
static constexpr uint32_t kCliPollIntervalMs = 50;

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
static bool g_imuInitialized = false;
static bool g_baroInitialized = false;
static bool g_calStorageInitialized = false;
static bool g_superloopValidationDone = false;
static bool g_imuValidationDone = false;
static bool g_baroValidationDone = false;
static uint32_t g_loopCounter = 0;

// IMU device handle (static allocation per LL Entry 1)
static icm20948_t g_imu;

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

    // IVP-09: IMU initialization
    if (g_imuInitialized) {
        printf("[PASS] ICM-20948 initialized (accel ±8g, gyro ±1000dps)\n");
        if (g_imu.mag_initialized) {
            printf("[PASS] AK09916 magnetometer initialized (100Hz)\n");
        } else {
            printf("[WARN] AK09916 magnetometer failed to initialize\n");
        }
    } else {
        printf("[FAIL] ICM-20948 failed to initialize\n");
    }

    // IVP-11: Barometer initialization
    if (g_baroInitialized) {
        printf("[PASS] DPS310 barometer initialized (8Hz, 8x OS)\n");
    } else {
        printf("[FAIL] DPS310 barometer failed to initialize\n");
    }

    // IVP-14: Calibration storage
    if (g_calStorageInitialized) {
        printf("[PASS] Calibration storage initialized\n");
        const calibration_store_t* cal = calibration_manager_get();
        if (cal != NULL) {
            printf("[INFO] Cal flags: 0x%02lX", (unsigned long)cal->cal_flags);
            if (cal->cal_flags & CAL_STATUS_GYRO) printf(" GYRO");
            if (cal->cal_flags & CAL_STATUS_LEVEL) printf(" LEVEL");
            if (cal->cal_flags & CAL_STATUS_BARO) printf(" BARO");
            if (cal->cal_flags == 0) printf(" (none)");
            printf("\n");
        }
    } else {
        printf("[FAIL] Calibration storage failed to initialize\n");
    }

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

/**
 * @brief Run Stage 2 IMU validation - called after first successful read
 */
static void hw_validate_imu(const icm20948_data_t* data) {
    printf("\n=== HW Validation: IMU (IVP-09/10) ===\n");

    // Check accel magnitude (~1g when stationary)
    float accel_mag = sqrtf(data->accel.x * data->accel.x +
                            data->accel.y * data->accel.y +
                            data->accel.z * data->accel.z);
    if (accel_mag > 8.0f && accel_mag < 12.0f) {
        printf("[PASS] Accel magnitude: %.2f m/s² (expect ~9.8)\n", accel_mag);
    } else {
        printf("[WARN] Accel magnitude: %.2f m/s² (expect ~9.8)\n", accel_mag);
    }

    // Check gyro near zero when stationary
    float gyro_mag = sqrtf(data->gyro.x * data->gyro.x +
                           data->gyro.y * data->gyro.y +
                           data->gyro.z * data->gyro.z);
    if (gyro_mag < 0.5f) {  // < 0.5 rad/s when stationary
        printf("[PASS] Gyro magnitude: %.3f rad/s (expect ~0 when still)\n", gyro_mag);
    } else {
        printf("[WARN] Gyro magnitude: %.3f rad/s (expect ~0 when still)\n", gyro_mag);
    }

    // Check magnetometer (if available)
    if (data->mag_valid) {
        float mag_mag = sqrtf(data->mag.x * data->mag.x +
                              data->mag.y * data->mag.y +
                              data->mag.z * data->mag.z);
        // Earth's magnetic field ~25-65 µT
        if (mag_mag > 20.0f && mag_mag < 70.0f) {
            printf("[PASS] Mag magnitude: %.1f µT (expect 25-65)\n", mag_mag);
        } else {
            printf("[WARN] Mag magnitude: %.1f µT (expect 25-65)\n", mag_mag);
        }
    } else {
        printf("[INFO] Magnetometer not ready\n");
    }

    // Temperature sanity check
    if (data->temperature_c > 10.0f && data->temperature_c < 50.0f) {
        printf("[PASS] Temperature: %.1f °C\n", data->temperature_c);
    } else {
        printf("[WARN] Temperature: %.1f °C (unusual)\n", data->temperature_c);
    }

    printf("=== IMU Validation Complete ===\n\n");
}

/**
 * @brief Run Stage 2 barometer validation - called after first successful read
 */
static void hw_validate_baro(const baro_dps310_data_t* data) {
    printf("\n=== HW Validation: Barometer (IVP-11/12) ===\n");

    // Pressure sanity check (sea level ~101325 Pa, varies with weather/altitude)
    if (data->pressure_pa > 80000.0f && data->pressure_pa < 110000.0f) {
        printf("[PASS] Pressure: %.1f Pa (%.2f hPa)\n",
               data->pressure_pa, data->pressure_pa / 100.0f);
    } else {
        printf("[WARN] Pressure: %.1f Pa (unusual)\n", data->pressure_pa);
    }

    // Temperature sanity check
    if (data->temperature_c > 10.0f && data->temperature_c < 50.0f) {
        printf("[PASS] Temperature: %.1f °C\n", data->temperature_c);
    } else {
        printf("[WARN] Temperature: %.1f °C (unusual)\n", data->temperature_c);
    }

    // Altitude sanity check (relative to 101325 Pa sea level)
    printf("[INFO] Altitude: %.1f m (relative to std sea level)\n", data->altitude_m);

    printf("=== Barometer Validation Complete ===\n\n");
}

// ============================================================================
// Sensor Status Callback for RC_OS
// ============================================================================

// Last sensor readings (updated in main loop)
static icm20948_data_t g_lastImuData = {};
static baro_dps310_data_t g_lastBaroData = {};
static bool g_lastImuValid = false;
static bool g_lastBaroValid = false;

/**
 * @brief Print current sensor status (called by RC_OS 's' command)
 */
static void print_sensor_status(void) {
    printf("\n=== Sensor Status ===\n");

    // IMU
    if (g_imuInitialized && g_lastImuValid) {
        printf("IMU (ICM-20948):\n");
        printf("  Accel: [%+7.2f %+7.2f %+7.2f] m/s²\n",
               g_lastImuData.accel.x, g_lastImuData.accel.y, g_lastImuData.accel.z);
        printf("  Gyro:  [%+7.3f %+7.3f %+7.3f] rad/s\n",
               g_lastImuData.gyro.x, g_lastImuData.gyro.y, g_lastImuData.gyro.z);
        if (g_lastImuData.mag_valid) {
            printf("  Mag:   [%+7.1f %+7.1f %+7.1f] µT\n",
                   g_lastImuData.mag.x, g_lastImuData.mag.y, g_lastImuData.mag.z);
        } else {
            printf("  Mag:   not available\n");
        }
        printf("  Temp:  %.1f °C\n", g_lastImuData.temperature_c);
    } else if (g_imuInitialized) {
        printf("IMU: initialized but no data yet\n");
    } else {
        printf("IMU: NOT INITIALIZED\n");
    }

    // Barometer
    if (g_baroInitialized && g_lastBaroValid) {
        printf("Barometer (DPS310):\n");
        printf("  Pressure: %.1f Pa (%.2f hPa)\n",
               g_lastBaroData.pressure_pa, g_lastBaroData.pressure_pa / 100.0f);
        printf("  Temp:     %.1f °C\n", g_lastBaroData.temperature_c);
        printf("  Altitude: %.1f m (AGL: %.1f m)\n",
               g_lastBaroData.altitude_m,
               calibration_get_altitude_agl(g_lastBaroData.pressure_pa));
    } else if (g_baroInitialized) {
        printf("Barometer: initialized but no data yet\n");
    } else {
        printf("Barometer: NOT INITIALIZED\n");
    }

    printf("=====================\n\n");
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

    // Allow sensors time to power up after board power-on
    // ICM-20948 datasheet: 11ms startup, DPS310: 40ms
    // Add generous margin for power sequencing
    sleep_ms(200);

    // -------------------------------------------------------------------------
    // IVP-09: IMU initialization (with retry)
    // -------------------------------------------------------------------------
    if (g_i2cInitialized) {
        // Try IMU init up to 3 times with bus recovery between attempts
        for (int attempt = 0; attempt < 3 && !g_imuInitialized; attempt++) {
            if (attempt > 0) {
                i2c_bus_recover();
                sleep_ms(50);
            }
            g_imuInitialized = icm20948_init(&g_imu, I2C_ADDR_ICM20948);
        }
    }

    // -------------------------------------------------------------------------
    // IVP-11: Barometer initialization
    // -------------------------------------------------------------------------
    if (g_i2cInitialized) {
        g_baroInitialized = baro_dps310_init(I2C_ADDR_DPS310);
        if (g_baroInitialized) {
            baro_dps310_start_continuous();
        }
    }

    // -------------------------------------------------------------------------
    // IVP-14: Calibration storage init (before USB per LL Entry 4/12)
    // -------------------------------------------------------------------------
    g_calStorageInitialized = calibration_storage_init();

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
    // IVP-14: Calibration manager init (after USB connected)
    // -------------------------------------------------------------------------
    calibration_manager_init();

    // -------------------------------------------------------------------------
    // IVP-18: RC_OS CLI init
    // -------------------------------------------------------------------------
    rc_os_init();
    rc_os_print_sensor_status = print_sensor_status;  // Set callback
    rc_os_imu_available = g_imuInitialized;
    rc_os_baro_available = g_baroInitialized;

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
    // Main loop info
    // -------------------------------------------------------------------------
    printf("Entering main loop - press 'h' for help\n\n");

    uint32_t lastSensorSampleMs = 0;
    uint32_t lastCliPollMs = 0;
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
        // Sensor sampling at 100Hz (for calibration and status)
        // ---------------------------------------------------------------------
        if (nowMs - lastSensorSampleMs >= kSensorSampleIntervalMs) {
            lastSensorSampleMs = nowMs;

            // Read IMU
            if (g_imuInitialized) {
                g_lastImuValid = icm20948_read(&g_imu, &g_lastImuData);

                // One-time validation
                if (g_lastImuValid && !g_imuValidationDone && stdio_usb_connected()) {
                    g_imuValidationDone = true;
                    hw_validate_imu(&g_lastImuData);
                }

                // Feed to calibration if active
                if (calibration_is_active()) {
                    cal_state_t state = calibration_manager_get_state();

                    if (g_lastImuValid) {
                        if (state == CAL_STATE_GYRO_SAMPLING) {
                            calibration_feed_gyro(
                                g_lastImuData.gyro.x, g_lastImuData.gyro.y, g_lastImuData.gyro.z,
                                g_lastImuData.temperature_c);
                        } else if (state == CAL_STATE_ACCEL_LEVEL_SAMPLING) {
                            calibration_feed_accel(
                                g_lastImuData.accel.x, g_lastImuData.accel.y, g_lastImuData.accel.z,
                                g_lastImuData.temperature_c);
                        }
                    }
                }
            }

            // Read Barometer
            if (g_baroInitialized) {
                bool baroReadOk = baro_dps310_read(&g_lastBaroData);
                g_lastBaroValid = baroReadOk && g_lastBaroData.valid;

                // One-time validation
                if (g_lastBaroValid && !g_baroValidationDone && stdio_usb_connected()) {
                    g_baroValidationDone = true;
                    hw_validate_baro(&g_lastBaroData);
                }

                // Feed to baro calibration if active
                if (g_lastBaroValid && calibration_is_active()) {
                    cal_state_t state = calibration_manager_get_state();
                    if (state == CAL_STATE_BARO_SAMPLING) {
                        calibration_feed_baro(
                            g_lastBaroData.pressure_pa,
                            g_lastBaroData.temperature_c);
                    }
                }
            }
        }

        // ---------------------------------------------------------------------
        // RC_OS CLI polling (~20Hz)
        // ---------------------------------------------------------------------
        if (nowMs - lastCliPollMs >= kCliPollIntervalMs) {
            lastCliPollMs = nowMs;
            rc_os_update();
        }

        // ---------------------------------------------------------------------
        // Superloop validation at 10 seconds
        // ---------------------------------------------------------------------
        if (stdio_usb_connected() && !g_superloopValidationDone && nowMs >= kSuperloopValidationMs) {
            g_superloopValidationDone = true;
            hw_validate_superloop(nowMs);
        }

        // Small sleep to prevent tight spinning (allows USB processing)
        sleep_ms(1);
    }

    return 0;
}
