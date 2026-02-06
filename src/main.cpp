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
 *   IVP-10: IMU data validation (10Hz read, gate checks)
 *   IVP-11: DPS310 barometer initialization
 *   IVP-12: Barometer data validation
 *   IVP-13: Multi-sensor polling (single core)
 *   IVP-13a: I2C bus recovery under fault
 */

#include "rocketchip/config.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "drivers/ws2812_status.h"
#include "drivers/i2c_bus.h"
#include "drivers/icm20948.h"
#include "drivers/baro_dps310.h"
#include <stdio.h>
#include <math.h>

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

// IVP-10: IMU data validation at 10Hz
static constexpr uint32_t kImuReadIntervalMs = 100;  // 10Hz
static constexpr uint32_t kImuSampleCount = 50;       // 5 seconds of data
static constexpr uint32_t kImuValidationDelayMs = 2000; // Wait 2s after boot for sensor settle

// IVP-12: Baro data validation at 10Hz (DPS310 runs at 8Hz continuous)
static constexpr uint32_t kBaroReadIntervalMs = 100;  // 10Hz polling
static constexpr uint32_t kBaroSampleCount = 100;      // 10 seconds of data
static constexpr uint32_t kBaroValidationDelayMs = 1000; // 1s after IVP-10 completes

// IVP-13: Multi-sensor polling rates
// IMU 100Hz, Baro 50Hz — VALIDATE targets per IVP.md
static constexpr uint32_t kIvp13ImuIntervalUs = 10000;   // 100Hz = 10ms
static constexpr uint32_t kIvp13BaroIntervalUs = 20000;  // 50Hz = 20ms
static constexpr uint32_t kIvp13DurationMs = 60000;       // 60 seconds
static constexpr uint32_t kIvp13StatusIntervalMs = 1000;  // Print status every 1s

// IVP-13a: I2C bus recovery thresholds
static constexpr uint32_t kI2cRecoveryThreshold = 50;     // ~333ms of failures at 150 reads/s
static constexpr uint32_t kI2cRecoveryCooldownMs = 2000;   // Min ms between recovery attempts

// ============================================================================
// Global State
// ============================================================================

static bool g_neopixelInitialized = false;
static bool g_i2cInitialized = false;
static bool g_imuInitialized = false;
static bool g_baroInitialized = false;
static bool g_baroContinuous = false;
static bool g_superloopValidationDone = false;
static uint32_t g_loopCounter = 0;

// IVP-10: IMU data validation state
static uint32_t g_imuSampleNum = 0;
static uint32_t g_lastImuReadMs = 0;
static bool g_imuValidationStarted = false;
static bool g_imuValidationDone = false;

// IVP-10: Gate tracking — accumulate min/max/sum for validation
static float g_accelXSum, g_accelYSum, g_accelZSum;
static float g_accelZMin, g_accelZMax;
static float g_gyroAbsMax;
static float g_magMagMin, g_magMagMax;
static float g_tempMin, g_tempMax;
static uint32_t g_nanCount = 0;
static uint32_t g_magValidCount = 0;
static uint32_t g_validSampleCount = 0;

// IVP-12: Baro data validation state
static uint32_t g_baroSampleNum = 0;
static uint32_t g_lastBaroReadMs = 0;
static bool g_baroValidationStarted = false;
static bool g_baroValidationDone = false;
static uint32_t g_baroValidStartMs = 0;

// IVP-12: Baro gate tracking
static float g_baroPressMin, g_baroPressMax, g_baroPressSum;
static float g_baroTempMin, g_baroTempMax;
static uint32_t g_baroValidReads = 0;
static uint32_t g_baroInvalidReads = 0;
static uint32_t g_baroNanCount = 0;
static float g_baroLastPress;  // For stuck-value detection
static uint32_t g_baroStuckCount = 0;

// IVP-13: Multi-sensor polling state
static bool g_ivp13Started = false;
static bool g_ivp13Done = false;
static uint32_t g_ivp13StartMs = 0;
static uint64_t g_ivp13LastImuUs = 0;
static uint64_t g_ivp13LastBaroUs = 0;
static uint32_t g_ivp13LastStatusMs = 0;
static uint32_t g_ivp13ImuCount = 0;
static uint32_t g_ivp13BaroCount = 0;
static uint32_t g_ivp13ImuErrors = 0;
static uint32_t g_ivp13BaroErrors = 0;
static uint64_t g_ivp13ImuTimeSum = 0;   // Sum of per-read times in us
static uint64_t g_ivp13BaroTimeSum = 0;
static uint32_t g_ivp13ImuTimeMax = 0;
static uint32_t g_ivp13BaroTimeMax = 0;
// Per-second counters for rate measurement
static uint32_t g_ivp13ImuSecCount = 0;
static uint32_t g_ivp13BaroSecCount = 0;

// IVP-13a: I2C bus recovery state
static uint32_t g_i2cConsecErrors = 0;      // Consecutive I2C errors (any sensor)
static uint32_t g_i2cRecoveryAttempts = 0;   // Total recovery attempts
static uint32_t g_i2cRecoverySuccesses = 0;  // Successful recoveries
static uint32_t g_i2cLastRecoveryMs = 0;     // Timestamp of last recovery attempt
static uint32_t g_baroConsecFails = 0;       // Consecutive baro failures (for lazy reinit)
static bool g_imuLastReadOk = false;         // IMU last read succeeded (bus health proxy)

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
    // Note: PA1010D GPS (0x10) excluded — probing it triggers I2C bus
    // interference (Pico SDK #252). Re-add at IVP-31 with proper handling.
    static const uint8_t expected[] = {
        I2C_ADDR_ICM20948,  // 0x69
        I2C_ADDR_DPS310,    // 0x77
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

    // IVP-11: Barometer initialization
    if (g_baroInitialized && g_baroContinuous) {
        printf("[PASS] DPS310 init OK, continuous mode active\n");
    } else if (g_baroInitialized) {
        printf("[WARN] DPS310 init OK, continuous mode failed\n");
    } else {
        printf("[FAIL] DPS310 init failed\n");
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
// IVP-10: IMU Data Validation
// ============================================================================

static void ivp10_print_sample(uint32_t n, const icm20948_data_t* d) {
    printf("[%03lu] A: %7.3f %7.3f %7.3f  G: %7.4f %7.4f %7.4f  M:%s %6.1f %6.1f %6.1f  T: %.1fC\n",
           (unsigned long)n,
           (double)d->accel.x, (double)d->accel.y, (double)d->accel.z,
           (double)d->gyro.x, (double)d->gyro.y, (double)d->gyro.z,
           d->mag_valid ? "" : "?",
           (double)d->mag.x, (double)d->mag.y, (double)d->mag.z,
           (double)d->temperature_c);
}

static void ivp10_accumulate(const icm20948_data_t* d) {
    // Check for NaN/inf
    if (isnan(d->accel.x) || isnan(d->accel.y) || isnan(d->accel.z) ||
        isnan(d->gyro.x) || isnan(d->gyro.y) || isnan(d->gyro.z) ||
        isinf(d->accel.x) || isinf(d->accel.y) || isinf(d->accel.z) ||
        isinf(d->gyro.x) || isinf(d->gyro.y) || isinf(d->gyro.z)) {
        g_nanCount++;
        return;
    }

    g_validSampleCount++;

    // Accel sums for averaging
    g_accelXSum += d->accel.x;
    g_accelYSum += d->accel.y;
    g_accelZSum += d->accel.z;

    // Accel Z range for stability check
    if (d->accel.z < g_accelZMin) g_accelZMin = d->accel.z;
    if (d->accel.z > g_accelZMax) g_accelZMax = d->accel.z;

    // Gyro max absolute
    float gmax = fabsf(d->gyro.x);
    if (fabsf(d->gyro.y) > gmax) gmax = fabsf(d->gyro.y);
    if (fabsf(d->gyro.z) > gmax) gmax = fabsf(d->gyro.z);
    if (gmax > g_gyroAbsMax) g_gyroAbsMax = gmax;

    // Mag magnitude
    if (d->mag_valid) {
        float mag = sqrtf(d->mag.x * d->mag.x + d->mag.y * d->mag.y + d->mag.z * d->mag.z);
        if (g_magValidCount == 0 || mag < g_magMagMin) g_magMagMin = mag;
        if (g_magValidCount == 0 || mag > g_magMagMax) g_magMagMax = mag;
        g_magValidCount++;
    }

    // Temperature
    if (d->temperature_c < g_tempMin) g_tempMin = d->temperature_c;
    if (d->temperature_c > g_tempMax) g_tempMax = d->temperature_c;
}

static void ivp10_gate_check(void) {
    printf("\n=== IVP-10: IMU Data Validation Gate ===\n");

    bool pass = true;

    float n = (float)g_validSampleCount;
    float axAvg = (n > 0) ? g_accelXSum / n : 0.0f;
    float ayAvg = (n > 0) ? g_accelYSum / n : 0.0f;
    float azAvg = (n > 0) ? g_accelZSum / n : 0.0f;

    // Gate: Accel magnitude ~9.8 m/s^2 (±0.5)
    // Uses magnitude because board orientation on desk is unknown
    float accelMag = sqrtf(axAvg * axAvg + ayAvg * ayAvg + azAvg * azAvg);
    bool amOk = (accelMag > 9.3f && accelMag < 10.3f);
    printf("[%s] Accel magnitude: %.3f m/s^2 (expect 9.3-10.3)\n",
           amOk ? "PASS" : "FAIL", (double)accelMag);
    if (!amOk) pass = false;

    // Info: Individual axis averages (orientation-dependent)
    printf("[INFO] Accel axes avg: X=%.3f Y=%.3f Z=%.3f m/s^2\n",
           (double)axAvg, (double)ayAvg, (double)azAvg);

    // Gate: Gyro near zero (±0.05 rad/s)
    bool gOk = (g_gyroAbsMax < 0.05f);
    printf("[%s] Gyro max abs: %.4f rad/s (expect <0.05)\n",
           gOk ? "PASS" : "WARN", (double)g_gyroAbsMax);

    // Gate: Mag reasonable (10-60 µT), not zeros
    bool mOk = (g_magValidCount > 0 && g_magMagMin > 10.0f && g_magMagMax < 60.0f);
    if (g_magValidCount > 0) {
        printf("[%s] Mag range: %.1f-%.1f uT (%lu/%lu valid) (expect 10-60)\n",
               mOk ? "PASS" : "WARN", (double)g_magMagMin, (double)g_magMagMax,
               (unsigned long)g_magValidCount, (unsigned long)kImuSampleCount);
    } else {
        printf("[WARN] Mag: no valid samples (may need additional boot cycle)\n");
    }

    // Gate: Temperature plausible (15-40°C)
    bool tOk = (g_tempMin > 15.0f && g_tempMax < 40.0f);
    printf("[%s] Temp range: %.1f-%.1fC (expect 15-40)\n",
           tOk ? "PASS" : "WARN", (double)g_tempMin, (double)g_tempMax);

    // Gate: No NaN/inf
    bool nanOk = (g_nanCount == 0);
    printf("[%s] NaN/inf count: %lu\n",
           nanOk ? "PASS" : "FAIL", (unsigned long)g_nanCount);
    if (!nanOk) pass = false;

    // Gate: Accel stability (max-min < 0.5 over 5s)
    float azRange = g_accelZMax - g_accelZMin;
    bool azStable = (azRange < 0.5f);
    printf("[%s] Accel Z stability: range=%.3f m/s^2 (expect <0.5)\n",
           azStable ? "PASS" : "WARN", (double)azRange);

    // Sample quality
    printf("[INFO] Valid samples: %lu/%lu\n",
           (unsigned long)g_validSampleCount, (unsigned long)kImuSampleCount);

    printf("=== IVP-10: %s ===\n\n", pass ? "ALL GATES PASS" : "GATE FAILURES - see above");
}

// ============================================================================
// IVP-12: Barometer Data Validation
// ============================================================================

static void ivp12_accumulate(const baro_dps310_data_t* d) {
    if (!d->valid) {
        g_baroInvalidReads++;
        return;
    }

    if (isnan(d->pressure_pa) || isnan(d->temperature_c) ||
        isinf(d->pressure_pa) || isinf(d->temperature_c)) {
        g_baroNanCount++;
        return;
    }

    // Stuck value detection (same pressure to 0.01 Pa)
    if (g_baroValidReads > 0 && fabsf(d->pressure_pa - g_baroLastPress) < 0.01f) {
        g_baroStuckCount++;
    }
    g_baroLastPress = d->pressure_pa;

    // Pressure
    if (g_baroValidReads == 0 || d->pressure_pa < g_baroPressMin) g_baroPressMin = d->pressure_pa;
    if (g_baroValidReads == 0 || d->pressure_pa > g_baroPressMax) g_baroPressMax = d->pressure_pa;
    g_baroPressSum += d->pressure_pa;

    // Temperature
    if (g_baroValidReads == 0 || d->temperature_c < g_baroTempMin) g_baroTempMin = d->temperature_c;
    if (g_baroValidReads == 0 || d->temperature_c > g_baroTempMax) g_baroTempMax = d->temperature_c;

    g_baroValidReads++;
}

static void ivp12_gate_check(void) {
    printf("\n=== IVP-12: Barometer Data Validation Gate ===\n");

    bool pass = true;

    if (g_baroValidReads == 0) {
        printf("[FAIL] No valid baro readings\n");
        printf("=== IVP-12: GATE FAILURES ===\n\n");
        return;
    }

    float pressAvg = g_baroPressSum / (float)g_baroValidReads;

    // Gate: Pressure ~101325 Pa (±3000 Pa for typical altitudes)
    bool pOk = (pressAvg > 98325.0f && pressAvg < 104325.0f);
    printf("[%s] Pressure avg: %.1f Pa (expect 98325-104325)\n",
           pOk ? "PASS" : "FAIL", (double)pressAvg);
    if (!pOk) pass = false;

    // Gate: Temperature plausible (15-40°C)
    bool tOk = (g_baroTempMin > 15.0f && g_baroTempMax < 40.0f);
    printf("[%s] Temp range: %.1f-%.1fC (expect 15-40)\n",
           tOk ? "PASS" : "WARN", (double)g_baroTempMin, (double)g_baroTempMax);

    // Gate: All reads valid
    bool vOk = (g_baroInvalidReads == 0);
    printf("[%s] Valid reads: %lu/%lu\n",
           vOk ? "PASS" : "WARN",
           (unsigned long)g_baroValidReads, (unsigned long)kBaroSampleCount);

    // Gate: Pressure noise < ±5 Pa over 10 seconds
    float pressRange = g_baroPressMax - g_baroPressMin;
    bool nOk = (pressRange < 10.0f);  // ±5 Pa = 10 Pa total range
    printf("[%s] Pressure noise: %.2f Pa range (expect <10)\n",
           nOk ? "PASS" : "WARN", (double)pressRange);

    // Gate: No NaN/inf
    bool nanOk = (g_baroNanCount == 0);
    printf("[%s] NaN/inf count: %lu\n",
           nanOk ? "PASS" : "FAIL", (unsigned long)g_baroNanCount);
    if (!nanOk) pass = false;

    // Gate: No stuck values (>50% identical readings = stuck)
    bool stuckOk = (g_baroStuckCount < g_baroValidReads / 2);
    printf("[%s] Stuck values: %lu/%lu\n",
           stuckOk ? "PASS" : "FAIL",
           (unsigned long)g_baroStuckCount, (unsigned long)g_baroValidReads);
    if (!stuckOk) pass = false;

    // Info: altitude
    float alt = baro_dps310_pressure_to_altitude(pressAvg, 101325.0f);
    printf("[INFO] Altitude (std atm): %.1f m\n", (double)alt);

    printf("=== IVP-12: %s ===\n\n", pass ? "ALL GATES PASS" : "GATE FAILURES - see above");
}

// ============================================================================
// IVP-13: Multi-Sensor Polling Gate Check
// ============================================================================

static void ivp13_gate_check(void) {
    printf("\n=== IVP-13: Multi-Sensor Polling Gate ===\n");

    bool pass = true;
    uint32_t elapsedMs = to_ms_since_boot(get_absolute_time()) - g_ivp13StartMs;
    float elapsedS = elapsedMs / 1000.0f;

    // Actual rates
    float imuRate = g_ivp13ImuCount / elapsedS;
    float baroRate = g_ivp13BaroCount / elapsedS;

    // Gate: IMU rate within 5% of 100Hz target
    bool imuOk = (imuRate > 95.0f && imuRate < 105.0f);
    printf("[%s] IMU rate: %.1f Hz (target 100, expect 95-105)\n",
           imuOk ? "PASS" : "FAIL", (double)imuRate);
    if (!imuOk) pass = false;

    // Gate: Baro rate within 10% of 50Hz target
    bool baroOk = (baroRate > 45.0f && baroRate < 55.0f);
    printf("[%s] Baro rate: %.1f Hz (target 50, expect 45-55)\n",
           baroOk ? "PASS" : "FAIL", (double)baroRate);
    if (!baroOk) pass = false;

    // Gate: Zero I2C errors over 60 seconds
    uint32_t totalErrors = g_ivp13ImuErrors + g_ivp13BaroErrors;
    bool errOk = (totalErrors == 0);
    printf("[%s] I2C errors: %lu (IMU: %lu, Baro: %lu)\n",
           errOk ? "PASS" : "FAIL", (unsigned long)totalErrors,
           (unsigned long)g_ivp13ImuErrors, (unsigned long)g_ivp13BaroErrors);
    if (!errOk) pass = false;

    // Info: Per-read I2C timing
    if (g_ivp13ImuCount > 0) {
        uint32_t imuAvgUs = (uint32_t)(g_ivp13ImuTimeSum / g_ivp13ImuCount);
        printf("[INFO] IMU read time: avg %lu us, max %lu us\n",
               (unsigned long)imuAvgUs, (unsigned long)g_ivp13ImuTimeMax);
    }
    if (g_ivp13BaroCount > 0) {
        uint32_t baroAvgUs = (uint32_t)(g_ivp13BaroTimeSum / g_ivp13BaroCount);
        printf("[INFO] Baro read time: avg %lu us, max %lu us\n",
               (unsigned long)baroAvgUs, (unsigned long)g_ivp13BaroTimeMax);
    }

    printf("[INFO] Total samples: IMU %lu, Baro %lu over %.1f s\n",
           (unsigned long)g_ivp13ImuCount, (unsigned long)g_ivp13BaroCount,
           (double)elapsedS);

    // IVP-13a: Recovery statistics
    if (g_i2cRecoveryAttempts > 0) {
        printf("[INFO] Bus recoveries: %lu/%lu successful\n",
               (unsigned long)g_i2cRecoverySuccesses,
               (unsigned long)g_i2cRecoveryAttempts);
    }

    printf("=== IVP-13: %s ===\n\n", pass ? "ALL GATES PASS" : "GATE FAILURES - see above");
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
        // NeoPixel off until used for status reporting
        ws2812_set_mode(WS2812_MODE_OFF, WS2812_COLOR_OFF);
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
    // IVP-11: DPS310 barometer init (before USB, after I2C)
    // -----------------------------------------------------------------
    if (g_i2cInitialized) {
        g_baroInitialized = baro_dps310_init(BARO_DPS310_ADDR_DEFAULT);
        if (g_baroInitialized) {
            g_baroContinuous = baro_dps310_start_continuous();
        }
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

    // IVP-11: Baro status
    if (g_baroInitialized && g_baroContinuous) {
        printf("DPS310 init OK, continuous mode (8Hz, 8x OS)\n");
    } else if (g_baroInitialized) {
        printf("DPS310 init OK, continuous mode FAILED\n");
    } else {
        printf("DPS310 init FAILED\n");
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

        // IVP-10: IMU data read at 10Hz
        if (g_imuInitialized && !g_imuValidationDone && stdio_usb_connected()) {
            // Wait for sensor settle before starting
            if (!g_imuValidationStarted && nowMs >= kImuValidationDelayMs) {
                g_imuValidationStarted = true;
                g_lastImuReadMs = nowMs;
                // Initialize tracking
                g_accelXSum = 0.0f;  g_accelYSum = 0.0f;  g_accelZSum = 0.0f;
                g_accelZMin = 999.0f;  g_accelZMax = -999.0f;
                g_gyroAbsMax = 0.0f;
                g_magMagMin = 999.0f;  g_magMagMax = 0.0f;
                g_tempMin = 999.0f;    g_tempMax = -999.0f;
                g_nanCount = 0;        g_magValidCount = 0;
                g_validSampleCount = 0;
                printf("=== IVP-10: IMU Data Validation (10Hz x %lu samples) ===\n",
                       (unsigned long)kImuSampleCount);
            }

            if (g_imuValidationStarted && g_imuSampleNum < kImuSampleCount &&
                (nowMs - g_lastImuReadMs) >= kImuReadIntervalMs) {
                g_lastImuReadMs = nowMs;
                icm20948_data_t data;
                if (icm20948_read(&g_imu, &data)) {
                    ivp10_print_sample(g_imuSampleNum, &data);
                    ivp10_accumulate(&data);
                    g_imuSampleNum++;
                } else {
                    printf("[%03lu] READ FAILED\n", (unsigned long)g_imuSampleNum);
                }
            }

            if (g_imuSampleNum >= kImuSampleCount) {
                g_imuValidationDone = true;
                ivp10_gate_check();
            }
        }

        // IVP-12: Baro data read at 10Hz (starts after IVP-10 completes)
        if (g_baroContinuous && !g_baroValidationDone && g_imuValidationDone &&
            stdio_usb_connected()) {
            if (!g_baroValidationStarted) {
                g_baroValidationStarted = true;
                g_baroValidStartMs = nowMs;
                g_lastBaroReadMs = nowMs;
                g_baroPressSum = 0.0f;
                g_baroValidReads = 0;
                g_baroInvalidReads = 0;
                g_baroNanCount = 0;
                g_baroStuckCount = 0;
                printf("=== IVP-12: Baro Data Validation (10Hz x %lu samples) ===\n",
                       (unsigned long)kBaroSampleCount);
            }

            // Wait briefly for DPS310 to have data after starting continuous
            if ((nowMs - g_baroValidStartMs) >= kBaroValidationDelayMs &&
                g_baroSampleNum < kBaroSampleCount &&
                (nowMs - g_lastBaroReadMs) >= kBaroReadIntervalMs) {
                g_lastBaroReadMs = nowMs;
                baro_dps310_data_t bdata;
                bool ok = baro_dps310_read(&bdata);
                if (ok) {
                    ivp12_accumulate(&bdata);
                    // Print every 10th sample to keep output manageable
                    if (g_baroSampleNum % 10 == 0) {
                        printf("[%03lu] P: %.1f Pa  T: %.2fC  Alt: %.1fm  %s\n",
                               (unsigned long)g_baroSampleNum,
                               (double)bdata.pressure_pa, (double)bdata.temperature_c,
                               (double)bdata.altitude_m,
                               bdata.valid ? "OK" : "INVALID");
                    }
                } else {
                    g_baroInvalidReads++;
                    if (g_baroSampleNum % 10 == 0) {
                        printf("[%03lu] READ FAILED\n", (unsigned long)g_baroSampleNum);
                    }
                }
                g_baroSampleNum++;
            }

            if (g_baroSampleNum >= kBaroSampleCount) {
                g_baroValidationDone = true;
                ivp12_gate_check();
            }
        }

        // IVP-13: Multi-sensor polling at target rates (after IVP-10 + IVP-12)
        if (g_imuInitialized && g_baroContinuous &&
            g_imuValidationDone && g_baroValidationDone && !g_ivp13Done) {

            uint64_t nowUs = time_us_64();

            if (!g_ivp13Started) {
                g_ivp13Started = true;
                g_ivp13StartMs = nowMs;
                g_ivp13LastImuUs = nowUs;
                g_ivp13LastBaroUs = nowUs;
                g_ivp13LastStatusMs = nowMs;
                if (stdio_usb_connected()) {
                    printf("=== IVP-13: Multi-Sensor Polling (IMU 100Hz, Baro 50Hz, 60s) ===\n");
                }
            }

            // IMU read at 100Hz
            if ((nowUs - g_ivp13LastImuUs) >= kIvp13ImuIntervalUs) {
                g_ivp13LastImuUs += kIvp13ImuIntervalUs;
                // Prevent accumulation if we fell behind
                if ((nowUs - g_ivp13LastImuUs) > kIvp13ImuIntervalUs * 2) {
                    g_ivp13LastImuUs = nowUs;
                }
                uint64_t t0 = time_us_64();
                icm20948_data_t data;
                if (icm20948_read(&g_imu, &data)) {
                    uint32_t dt = (uint32_t)(time_us_64() - t0);
                    g_ivp13ImuCount++;
                    g_ivp13ImuSecCount++;
                    g_ivp13ImuTimeSum += dt;
                    if (dt > g_ivp13ImuTimeMax) g_ivp13ImuTimeMax = dt;
                    g_i2cConsecErrors = 0;
                    g_imuLastReadOk = true;
                } else {
                    g_ivp13ImuErrors++;
                    g_i2cConsecErrors++;
                    g_imuLastReadOk = false;
                }
            }

            // Baro read at 50Hz
            if ((nowUs - g_ivp13LastBaroUs) >= kIvp13BaroIntervalUs) {
                g_ivp13LastBaroUs += kIvp13BaroIntervalUs;
                if ((nowUs - g_ivp13LastBaroUs) > kIvp13BaroIntervalUs * 2) {
                    g_ivp13LastBaroUs = nowUs;
                }
                uint64_t t0 = time_us_64();
                baro_dps310_data_t bdata;
                if (baro_dps310_read(&bdata)) {
                    uint32_t dt = (uint32_t)(time_us_64() - t0);
                    g_ivp13BaroCount++;
                    g_ivp13BaroSecCount++;
                    g_ivp13BaroTimeSum += dt;
                    if (dt > g_ivp13BaroTimeMax) g_ivp13BaroTimeMax = dt;
                    g_i2cConsecErrors = 0;
                    g_baroConsecFails = 0;
                } else {
                    g_ivp13BaroErrors++;
                    g_i2cConsecErrors++;
                    g_baroConsecFails++;
                }
            }

            // IVP-13a: Lazy baro reinit — if IMU reads work but baro
            // doesn't, DPS310 lost continuous mode (e.g. after disconnect).
            // 100 consecutive baro fails at 50Hz = 2 seconds of failure.
            if (g_baroConsecFails >= 100 && g_imuLastReadOk) {
                g_baroConsecFails = 0;
                g_baroInitialized = baro_dps310_init(BARO_DPS310_ADDR_DEFAULT);
                g_baroContinuous = g_baroInitialized ?
                    baro_dps310_start_continuous() : false;
                if (stdio_usb_connected()) {
                    printf("[RECOVERY] Baro reinit: %s\n",
                           g_baroContinuous ? "OK" : "FAIL");
                }
            }

            // IVP-13a: Bus recovery when consecutive errors exceed threshold
            if (g_i2cConsecErrors >= kI2cRecoveryThreshold &&
                (nowMs - g_i2cLastRecoveryMs) >= kI2cRecoveryCooldownMs) {
                g_i2cLastRecoveryMs = nowMs;
                g_i2cRecoveryAttempts++;
                if (stdio_usb_connected()) {
                    printf("[RECOVERY] %lu consecutive errors — resetting I2C bus (attempt %lu)\n",
                           (unsigned long)g_i2cConsecErrors,
                           (unsigned long)g_i2cRecoveryAttempts);
                }
                bool recovered = i2c_bus_reset();
                if (recovered) {
                    g_i2cRecoverySuccesses++;
                    g_i2cConsecErrors = 0;

                    // Don't reinit sensors here — icm20948_init() blocks
                    // for 150ms+ with device reset and mag retries, which
                    // hangs if sensors are still disconnected. Instead:
                    // - IMU self-recovers from reads (no reinit needed)
                    // - Baro needs reinit for continuous mode (done below)

                    if (stdio_usb_connected()) {
                        printf("[RECOVERY] Bus reset OK — resuming polling\n");
                    }
                } else {
                    if (stdio_usb_connected()) {
                        printf("[RECOVERY] Bus reset FAILED — will retry after cooldown\n");
                    }
                }
            }

            // Status print every second
            if (stdio_usb_connected() && (nowMs - g_ivp13LastStatusMs) >= kIvp13StatusIntervalMs) {
                uint32_t elapsed = nowMs - g_ivp13StartMs;
                uint32_t totalErrors = g_ivp13ImuErrors + g_ivp13BaroErrors;
                if (g_i2cRecoveryAttempts > 0) {
                    printf("[%3lus] IMU: %lu/s  Baro: %lu/s  Err: %lu  Rec: %lu/%lu\n",
                           (unsigned long)(elapsed / 1000),
                           (unsigned long)g_ivp13ImuSecCount,
                           (unsigned long)g_ivp13BaroSecCount,
                           (unsigned long)totalErrors,
                           (unsigned long)g_i2cRecoverySuccesses,
                           (unsigned long)g_i2cRecoveryAttempts);
                } else {
                    printf("[%3lus] IMU: %lu/s  Baro: %lu/s  Err: %lu\n",
                           (unsigned long)(elapsed / 1000),
                           (unsigned long)g_ivp13ImuSecCount,
                           (unsigned long)g_ivp13BaroSecCount,
                           (unsigned long)totalErrors);
                }
                g_ivp13ImuSecCount = 0;
                g_ivp13BaroSecCount = 0;
                g_ivp13LastStatusMs = nowMs;
            }

            // Check if 60 seconds elapsed
            if ((nowMs - g_ivp13StartMs) >= kIvp13DurationMs) {
                g_ivp13Done = true;
                if (stdio_usb_connected()) {
                    ivp13_gate_check();
                }
            }
        }

        // Superloop validation at 10s
        if (stdio_usb_connected() && !g_superloopValidationDone &&
            nowMs >= kSuperloopValidationMs) {
            g_superloopValidationDone = true;
            hw_validate_superloop(nowMs);
        }

        // Small sleep to prevent tight spinning (allows USB processing)
        // Note: sleep_ms(1) limits max loop rate to ~1kHz; sufficient for
        // IVP-13 targets (100Hz IMU + 50Hz baro). Remove in Stage 3.
        sleep_ms(1);
    }

    return 0;
}
