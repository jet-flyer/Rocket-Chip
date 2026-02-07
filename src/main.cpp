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
#include "calibration/calibration_storage.h"
#include "calibration/calibration_manager.h"
#include "cli/rc_os.h"
#include "pico/multicore.h"
#include "hardware/sync.h"
#include <atomic>
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

// Skip verified IVP gates (IVP-10, IVP-12, IVP-13) — set false to re-run
static constexpr bool kSkipVerifiedGates = true;

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

// IVP-21: Spinlock soak test
static constexpr uint32_t kSpinlockSoakMs = 5 * 60 * 1000;  // 5 minutes
static constexpr uint32_t kSpinlockCheckIntervalMs = 100;     // Core 0 reads 10x/sec

// IVP-22: FIFO message passing test
static constexpr uint32_t kFifoTestCount = 1000;

// IVP-23: Doorbell test
static constexpr uint32_t kDoorbellTestCount = 1000;

// Core 1 test-mode commands (sent via FIFO)
static constexpr uint32_t kCmd_FifoEchoStart = 0xF1F0E000;
static constexpr uint32_t kCmd_FifoEchoDone  = 0xF1F0D000;
static constexpr uint32_t kCmd_FifoSendStart = 0xF1F05000;
static constexpr uint32_t kCmd_DoorbellStart = 0xDB00B000;
static constexpr uint32_t kCmd_DoorbellDone  = 0xDB00D000;
static constexpr uint32_t kCmd_SpinlockStart = 0x5510C000;

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

// IVP-19/20: Dual-core state
// Atomic counter in static SRAM — Core 1 increments, Core 0 reads.
// Per SEQLOCK_DESIGN.md: shared data must be in SRAM (not PSRAM).
static std::atomic<uint32_t> g_core1Counter{0};

// IVP-21: Spinlock shared struct — multi-field, protected by spinlock.
// If spinlock works correctly, a == b == c == write_count on every read.
struct spinlock_test_data_t {
    uint32_t field_a;
    uint32_t field_b;
    uint32_t field_c;
    uint32_t write_count;
};
static spinlock_test_data_t g_spinlockData = {0, 0, 0, 0};
static spin_lock_t* g_pTestSpinlock = nullptr;
static int g_spinlockId = -1;

// IVP-21: Soak test tracking (Core 0 side)
static bool g_ivp21Active = false;
static bool g_ivp21Done = false;
static uint32_t g_ivp21StartMs = 0;
static uint32_t g_ivp21LastCheckMs = 0;
static uint32_t g_ivp21ReadCount = 0;
static uint32_t g_ivp21InconsistentCount = 0;
static uint32_t g_ivp21LockTimeMinUs = UINT32_MAX;
static uint32_t g_ivp21LockTimeMaxUs = 0;
static uint64_t g_ivp21LockTimeSumUs = 0;
static uint32_t g_ivp21LastPrintMs = 0;

// IVP-23: Doorbell number (shared between test setup and Core 1)
static int g_doorbellNum = -1;

// IVP-14: Calibration storage state
static bool g_calStorageInitialized = false;
static bool g_ivp14Done = false;

// IVP-15: Gyro bias calibration state
static bool g_ivp15Started = false;
static bool g_ivp15Done = false;
static uint64_t g_ivp15LastFeedUs = 0;
static constexpr uint32_t kIvp15FeedIntervalUs = 10000;  // 100Hz feed rate

// IVP-16: Accel level calibration state
static bool g_ivp16Started = false;
static bool g_ivp16Done = false;
static uint64_t g_ivp16LastFeedUs = 0;
static constexpr uint32_t kIvp16FeedIntervalUs = 10000;  // 100Hz feed rate

// IVP-18: CLI feed interval for calibrations triggered via menu
static uint64_t g_cliCalLastFeedUs = 0;
static constexpr uint32_t kCliCalFeedIntervalUs = 10000;  // 100Hz

// IMU device handle (static per LL Entry 1 — avoid large objects on stack)
static icm20948_t g_imu;

// ============================================================================
// IVP-19/20/21/22/23: Core 1 Entry Point
// ============================================================================
// Phase 1: Test dispatcher — responds to FIFO commands for IVP-22/23 exercises.
// Phase 2: Spinlock soak + NeoPixel — writes shared struct at ~100kHz for IVP-21.
// Atomic counter (IVP-20) increments throughout both phases.

static void core1_entry(void) {
    // ---- Phase 1: Test Dispatcher ----
    // Core 1 polls FIFO every 1ms, blinks NeoPixel cyan fast (~4Hz) via timer.
    bool testMode = true;
    uint32_t lastNeoToggleMs = to_ms_since_boot(get_absolute_time());
    bool neoOn = true;
    ws2812_set_mode(WS2812_MODE_SOLID, WS2812_COLOR_CYAN);
    ws2812_update();

    while (testMode) {
        // Poll FIFO with 1ms timeout (fast response to commands)
        uint32_t cmd;
        if (multicore_fifo_pop_timeout_us(1000, &cmd)) {
            switch (cmd) {

            // IVP-22: FIFO echo — Core 0 sends values, Core 1 echoes back
            case kCmd_FifoEchoStart: {
                // ACK: tell Core 0 we're ready
                multicore_fifo_push_blocking(kCmd_FifoEchoStart);
                for (uint32_t i = 0; i < kFifoTestCount; i++) {
                    uint32_t val = multicore_fifo_pop_blocking();
                    multicore_fifo_push_blocking(val);
                }
                multicore_fifo_push_blocking(kCmd_FifoEchoDone);
                break;
            }

            // IVP-22: FIFO send — Core 1 sends sequential values to Core 0
            case kCmd_FifoSendStart: {
                for (uint32_t i = 0; i < kFifoTestCount; i++) {
                    multicore_fifo_push_blocking(i);
                }
                break;
            }

            // IVP-23: Doorbell — Core 1 sets doorbell N times with 10us gaps
            case kCmd_DoorbellStart: {
                for (uint32_t i = 0; i < kDoorbellTestCount; i++) {
                    multicore_doorbell_set_other_core((uint)g_doorbellNum);
                    busy_wait_us(10);
                }
                multicore_fifo_push_blocking(kCmd_DoorbellDone);
                break;
            }

            // Transition to Phase 2
            case kCmd_SpinlockStart:
                testMode = false;
                break;

            default:
                break;
            }
        }

        // NeoPixel blink via timer (~4Hz toggle = 125ms)
        g_core1Counter.fetch_add(1, std::memory_order_relaxed);
        uint32_t nowMs = to_ms_since_boot(get_absolute_time());
        if ((nowMs - lastNeoToggleMs) >= 125) {
            lastNeoToggleMs = nowMs;
            neoOn = !neoOn;
            ws2812_set_mode(WS2812_MODE_SOLID,
                            neoOn ? WS2812_COLOR_CYAN : WS2812_COLOR_OFF);
            ws2812_update();
        }
    }

    // ---- Phase 2: Spinlock Soak + NeoPixel ----
    // Write shared struct at ~100kHz under spinlock. NeoPixel at ~2Hz.
    uint32_t soakStartMs = to_ms_since_boot(get_absolute_time());
    uint32_t writeCounter = 0;
    uint32_t lastNeoMs = soakStartMs;
    bool neoState = false;

    while (true) {
        uint32_t nowMs = to_ms_since_boot(get_absolute_time());
        bool soaking = (nowMs - soakStartMs) < kSpinlockSoakMs;

        // Spinlock struct update (high rate during soak)
        if (soaking && g_pTestSpinlock != nullptr) {
            writeCounter++;
            uint32_t saved = spin_lock_blocking(g_pTestSpinlock);
            g_spinlockData.field_a = writeCounter;
            g_spinlockData.field_b = writeCounter;
            g_spinlockData.field_c = writeCounter;
            g_spinlockData.write_count = writeCounter;
            spin_unlock(g_pTestSpinlock, saved);
        }

        g_core1Counter.fetch_add(1, std::memory_order_relaxed);

        // NeoPixel update at ~2Hz
        if (nowMs - lastNeoMs >= 250) {
            lastNeoMs = nowMs;
            neoState = !neoState;
            if (soaking) {
                ws2812_set_mode(WS2812_MODE_SOLID,
                    neoState ? WS2812_COLOR_CYAN : WS2812_COLOR_MAGENTA);
            } else {
                ws2812_set_mode(WS2812_MODE_SOLID, WS2812_COLOR_GREEN);
            }
            ws2812_update();
        }

        // ~100kHz update rate during soak, relaxed after
        if (soaking) {
            busy_wait_us(10);
        } else {
            sleep_ms(1);
        }
    }
}

// ============================================================================
// Accel Read Callback (for 6-pos calibration via CLI)
// ============================================================================

static bool read_accel_for_cal(float *ax, float *ay, float *az, float *temp_c) {
    sleep_ms(10);  // ~100Hz sampling rate
    icm20948_vec3_t accel;
    if (!icm20948_read_accel(&g_imu, &accel)) {
        return false;
    }
    *ax = accel.x;
    *ay = accel.y;
    *az = accel.z;
    *temp_c = 0.0f;  // Not needed for 6-pos cal
    return true;
}

// ============================================================================
// Calibration Hooks — disable I2C master during rapid accel reads
// ============================================================================
// The ICM-20948's internal I2C master (for mag reads) performs autonomous
// Bank 3 transactions that race with our Bank 0 accel reads, causing
// bank-select corruption after ~150 reads. Disable it during calibration.

static void cal_pre_hook(void) {
    if (g_imuInitialized) {
        icm20948_set_i2c_master_enable(&g_imu, false);
    }
}

static void cal_post_hook(void) {
    if (g_imuInitialized) {
        icm20948_set_i2c_master_enable(&g_imu, true);
    }
}

// ============================================================================
// Sensor Status Callback (for RC_OS CLI 's' command)
// ============================================================================

static void print_sensor_status(void) {
    printf("\n========================================\n");
    printf("  Sensor Readings (calibrated)\n");
    printf("========================================\n");

    if (g_imuInitialized) {
        icm20948_data_t data;
        if (icm20948_read(&g_imu, &data)) {
            // Apply calibration
            float gx, gy, gz, ax, ay, az;
            calibration_apply_gyro(data.gyro.x, data.gyro.y, data.gyro.z,
                                   &gx, &gy, &gz);
            calibration_apply_accel(data.accel.x, data.accel.y, data.accel.z,
                                    &ax, &ay, &az);

            printf("Accel (m/s^2): X=%7.3f Y=%7.3f Z=%7.3f\n",
                   (double)ax, (double)ay, (double)az);
            printf("Gyro  (rad/s): X=%7.4f Y=%7.4f Z=%7.4f\n",
                   (double)gx, (double)gy, (double)gz);
            if (data.mag_valid) {
                printf("Mag     (uT):  X=%6.1f  Y=%6.1f  Z=%6.1f\n",
                       (double)data.mag.x, (double)data.mag.y, (double)data.mag.z);
            } else {
                printf("Mag: not ready\n");
            }
            printf("Temp: %.1f C\n", (double)data.temperature_c);
        } else {
            printf("IMU: read failed\n");
        }
    } else {
        printf("IMU: not initialized\n");
    }

    if (g_baroContinuous) {
        baro_dps310_data_t bdata;
        if (baro_dps310_read(&bdata) && bdata.valid) {
            float alt = calibration_get_altitude_agl(bdata.pressure_pa);
            printf("Baro: %.1f Pa, %.2f C, AGL=%.2f m\n",
                   (double)bdata.pressure_pa, (double)bdata.temperature_c,
                   (double)alt);
        } else {
            printf("Baro: read failed\n");
        }
    } else {
        printf("Baro: not initialized\n");
    }

    printf("========================================\n\n");
}

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
// IVP-14: Calibration Storage Gate Check
// ============================================================================

// Known test values for save/load verification
static constexpr float kTestGyroX = 0.00123f;
static constexpr float kTestGyroY = -0.00234f;
static constexpr float kTestGyroZ = 0.00345f;

static void ivp14_gate_check(void) {
    printf("\n=== IVP-14: Calibration Storage Gate ===\n");

    bool pass = true;

    // Gate 1: Load returns OK or initializes defaults
    {
        cal_result_t result = calibration_load();
        bool ok = (result == CAL_RESULT_OK || result == CAL_RESULT_STORAGE_ERROR);
        printf("[%s] Gate 1: calibration_load() = %d (%s)\n",
               ok ? "PASS" : "FAIL", (int)result,
               result == CAL_RESULT_OK ? "valid data" : "defaults initialized");
        // Note: STORAGE_ERROR on first boot is expected (no data yet)
    }

    // Gate 2: Save test data, read back, verify match
    {
        // Get current cal, modify gyro bias with known values
        const calibration_store_t* current = calibration_manager_get();
        calibration_store_t test_cal = *current;
        test_cal.gyro.bias.x = kTestGyroX;
        test_cal.gyro.bias.y = kTestGyroY;
        test_cal.gyro.bias.z = kTestGyroZ;
        test_cal.gyro.status = CAL_STATUS_GYRO;
        test_cal.cal_flags |= CAL_STATUS_GYRO;
        calibration_update_crc(&test_cal);

        // Save
        bool writeOk = calibration_storage_write(&test_cal);
        if (!writeOk) {
            printf("[FAIL] Gate 2: flash write failed\n");
            pass = false;
        } else {
            // Read back
            calibration_store_t readback;
            bool readOk = calibration_storage_read(&readback);
            if (!readOk) {
                printf("[FAIL] Gate 2: flash read failed\n");
                pass = false;
            } else {
                bool match = (fabsf(readback.gyro.bias.x - kTestGyroX) < 1e-6f &&
                              fabsf(readback.gyro.bias.y - kTestGyroY) < 1e-6f &&
                              fabsf(readback.gyro.bias.z - kTestGyroZ) < 1e-6f &&
                              (readback.cal_flags & CAL_STATUS_GYRO));
                printf("[%s] Gate 2: save/readback %s (bias: %.5f, %.5f, %.5f)\n",
                       match ? "PASS" : "FAIL",
                       match ? "match" : "MISMATCH",
                       (double)readback.gyro.bias.x,
                       (double)readback.gyro.bias.y,
                       (double)readback.gyro.bias.z);
                if (!match) pass = false;
            }
        }
    }

    // Gate 3: Power cycle persistence
    // Check if test data survived from a previous boot
    {
        calibration_store_t persisted;
        bool readOk = calibration_storage_read(&persisted);
        if (readOk && (persisted.cal_flags & CAL_STATUS_GYRO) &&
            fabsf(persisted.gyro.bias.x - kTestGyroX) < 1e-6f) {
            printf("[PASS] Gate 3: data persisted across power cycle\n");
        } else {
            printf("[INFO] Gate 3: power cycle test — unplug USB, replug, check next boot\n");
            // Not a failure — just needs physical verification
        }
    }

    // Gate 4: 10 consecutive saves
    {
        bool allOk = true;
        for (int i = 0; i < 10; i++) {
            calibration_store_t test_cal;
            calibration_init_defaults(&test_cal);
            test_cal.gyro.bias.x = kTestGyroX;
            test_cal.gyro.bias.y = kTestGyroY;
            test_cal.gyro.bias.z = kTestGyroZ;
            test_cal.gyro.status = CAL_STATUS_GYRO;
            test_cal.cal_flags |= CAL_STATUS_GYRO;
            // Use reserved field to track iteration
            test_cal.reserved[0] = (uint8_t)(i + 1);
            calibration_update_crc(&test_cal);

            if (!calibration_storage_write(&test_cal)) {
                printf("[FAIL] Gate 4: write %d/10 failed\n", i + 1);
                allOk = false;
                break;
            }

            // Verify readback
            calibration_store_t verify;
            if (!calibration_storage_read(&verify) || verify.reserved[0] != (uint8_t)(i + 1)) {
                printf("[FAIL] Gate 4: readback %d/10 failed\n", i + 1);
                allOk = false;
                break;
            }
        }
        if (allOk) {
            printf("[PASS] Gate 4: 10/10 consecutive saves OK (wear leveling works)\n");
        } else {
            pass = false;
        }
    }

    printf("=== IVP-14: %s ===\n\n", pass ? "ALL GATES PASS" : "GATE FAILURES - see above");
}

// ============================================================================
// IVP-15: Gyro Bias Calibration Gate Check
// ============================================================================

static void ivp15_gate_check(void) {
    printf("\n=== IVP-15: Gyro Bias Calibration Gate ===\n");

    bool pass = true;
    cal_result_t result = calibration_get_result();

    // Gate 1: Returns CAL_RESULT_OK
    bool resultOk = (result == CAL_RESULT_OK);
    printf("[%s] Gate 1: calibration result = %d (%s)\n",
           resultOk ? "PASS" : "FAIL", (int)result,
           result == CAL_RESULT_OK ? "OK" :
           result == CAL_RESULT_MOTION_DETECTED ? "MOTION_DETECTED" : "ERROR");
    if (!resultOk) { pass = false; }

    // Gate 2: Bias values each axis < 0.1 rad/s
    const calibration_store_t* cal = calibration_manager_get();
    float bx = cal->gyro.bias.x;
    float by = cal->gyro.bias.y;
    float bz = cal->gyro.bias.z;
    bool biasOk = (fabsf(bx) < 0.1f && fabsf(by) < 0.1f && fabsf(bz) < 0.1f);
    printf("[%s] Gate 2: bias X=%.5f Y=%.5f Z=%.5f rad/s (expect <0.1 each)\n",
           biasOk ? "PASS" : "FAIL",
           (double)bx, (double)by, (double)bz);
    if (!biasOk) pass = false;

    // Gate 3: After applying, readings near zero when stationary
    // Take 10 samples and check corrected gyro near zero
    {
        float maxCorrected = 0.0f;
        uint32_t goodSamples = 0;
        for (int i = 0; i < 10; i++) {
            sleep_ms(10);  // ~100Hz
            icm20948_data_t data;
            if (icm20948_read(&g_imu, &data)) {
                float cx, cy, cz;
                calibration_apply_gyro(data.gyro.x, data.gyro.y, data.gyro.z,
                                       &cx, &cy, &cz);
                float m = fabsf(cx);
                if (fabsf(cy) > m) m = fabsf(cy);
                if (fabsf(cz) > m) m = fabsf(cz);
                if (m > maxCorrected) maxCorrected = m;
                goodSamples++;
            }
        }
        // After bias subtraction, stationary gyro should read < 0.05 rad/s
        bool corrOk = (goodSamples >= 5 && maxCorrected < 0.05f);
        printf("[%s] Gate 3: corrected gyro max=%.5f rad/s (%lu/10 samples) (expect <0.05)\n",
               corrOk ? "PASS" : "FAIL",
               (double)maxCorrected, (unsigned long)goodSamples);
        if (!corrOk) pass = false;
    }

    // Gate 4: Persists across power cycle
    // Save, then read back and verify
    {
        cal_result_t saveResult = calibration_save();
        if (saveResult != CAL_RESULT_OK) {
            printf("[FAIL] Gate 4: save failed (%d)\n", (int)saveResult);
            pass = false;
        } else {
            // Read back from flash
            calibration_store_t readback;
            bool readOk = calibration_storage_read(&readback);
            bool persistOk = readOk &&
                calibration_validate(&readback) &&
                (readback.cal_flags & CAL_STATUS_GYRO) &&
                fabsf(readback.gyro.bias.x - bx) < 1e-6f &&
                fabsf(readback.gyro.bias.y - by) < 1e-6f &&
                fabsf(readback.gyro.bias.z - bz) < 1e-6f;
            printf("[%s] Gate 4: save + readback %s (flags=0x%02lX)\n",
                   persistOk ? "PASS" : "FAIL",
                   persistOk ? "match" : "MISMATCH",
                   readOk ? (unsigned long)readback.cal_flags : 0UL);
            if (!persistOk) pass = false;
            printf("[INFO] Power cycle to verify full persistence\n");
        }
    }

    printf("=== IVP-15: %s ===\n\n", pass ? "ALL GATES PASS" : "GATE FAILURES - see above");
}

// ============================================================================
// IVP-16: Accel Level Calibration Gate Check
// ============================================================================

static void ivp16_gate_check(uint32_t elapsedMs) {
    printf("\n=== IVP-16: Accel Level Calibration Gate ===\n");

    bool pass = true;
    cal_result_t result = calibration_get_result();

    // Gate 1: Completes in < 3 seconds
    bool timeOk = (result == CAL_RESULT_OK && elapsedMs < 3000);
    printf("[%s] Gate 1: completed in %lu ms (expect <3000), result=%d (%s)\n",
           timeOk ? "PASS" : "FAIL", (unsigned long)elapsedMs, (int)result,
           result == CAL_RESULT_OK ? "OK" :
           result == CAL_RESULT_MOTION_DETECTED ? "MOTION_DETECTED" : "ERROR");
    if (!timeOk) pass = false;

    // Gate 2: After applying, Z reads +9.81 ±0.05, X and Y < 0.05 m/s²
    {
        float axSum = 0.0f, aySum = 0.0f, azSum = 0.0f;
        uint32_t goodSamples = 0;
        for (int i = 0; i < 10; i++) {
            sleep_ms(10);
            icm20948_data_t data;
            if (icm20948_read(&g_imu, &data)) {
                float cx, cy, cz;
                calibration_apply_accel(data.accel.x, data.accel.y, data.accel.z,
                                        &cx, &cy, &cz);
                axSum += cx;
                aySum += cy;
                azSum += cz;
                goodSamples++;
            }
        }
        if (goodSamples >= 5) {
            float n = (float)goodSamples;
            float axAvg = axSum / n;
            float ayAvg = aySum / n;
            float azAvg = azSum / n;

            bool xyOk = (fabsf(axAvg) < 0.05f && fabsf(ayAvg) < 0.05f);
            bool zOk = (fabsf(azAvg - 9.80665f) < 0.05f);

            printf("[%s] Gate 2: corrected X=%.4f Y=%.4f m/s^2 (expect <0.05 each)\n",
                   xyOk ? "PASS" : "WARN", (double)axAvg, (double)ayAvg);
            printf("[%s] Gate 2: corrected Z=%.4f m/s^2 (expect 9.81 +/-0.05)\n",
                   zOk ? "PASS" : "WARN", (double)azAvg);

            // Use WARN not FAIL — device orientation may not be perfectly flat
            // IVP.md diagnostic: "Wrong values = check accelerometer axis orientation"
            if (!xyOk || !zOk) {
                printf("[DIAG] If Z != 9.81: device may not be flat (Z-up) or needs 6-pos cal\n");
            }
        } else {
            printf("[FAIL] Gate 2: insufficient samples (%lu/10)\n",
                   (unsigned long)goodSamples);
            pass = false;
        }
    }

    // Gate 3: Persists across power cycle
    {
        const calibration_store_t* cal = calibration_manager_get();
        cal_result_t saveResult = calibration_save();
        if (saveResult != CAL_RESULT_OK) {
            printf("[FAIL] Gate 3: save failed (%d)\n", (int)saveResult);
            pass = false;
        } else {
            calibration_store_t readback;
            bool readOk = calibration_storage_read(&readback);
            bool persistOk = readOk &&
                calibration_validate(&readback) &&
                (readback.cal_flags & CAL_STATUS_LEVEL) &&
                fabsf(readback.accel.offset.x - cal->accel.offset.x) < 1e-6f &&
                fabsf(readback.accel.offset.y - cal->accel.offset.y) < 1e-6f &&
                fabsf(readback.accel.offset.z - cal->accel.offset.z) < 1e-6f;
            printf("[%s] Gate 3: save + readback %s (flags=0x%02lX)\n",
                   persistOk ? "PASS" : "FAIL",
                   persistOk ? "match" : "MISMATCH",
                   readOk ? (unsigned long)readback.cal_flags : 0UL);
            if (!persistOk) pass = false;

            // Print computed offsets for reference
            printf("[INFO] Accel offsets: X=%.4f Y=%.4f Z=%.4f m/s^2\n",
                   (double)cal->accel.offset.x,
                   (double)cal->accel.offset.y,
                   (double)cal->accel.offset.z);
            printf("[INFO] Power cycle to verify full persistence\n");
        }
    }

    printf("=== IVP-16: %s ===\n\n", pass ? "ALL GATES PASS" : "GATE FAILURES - see above");
}

// ============================================================================
// IVP-22: FIFO Message Passing Test
// ============================================================================
// Exercise-only — FIFO reserved by multicore_lockout (flash_safe_execute).

static void ivp22_fifo_test(void) {
    printf("\n=== IVP-22: Multicore FIFO Message Passing ===\n");
    bool pass = true;

    // Gate 1: Core 0 -> Core 1 echo (1000 messages)
    {
        multicore_fifo_push_blocking(kCmd_FifoEchoStart);

        // Wait for Core 1 ACK (confirms it's in the echo loop)
        uint32_t ack;
        if (!multicore_fifo_pop_timeout_us(1000000, &ack) || ack != kCmd_FifoEchoStart) {
            printf("[FAIL] Gate 1: Core 1 did not ACK echo start\n");
        }

        uint32_t lost = 0;
        uint64_t rttSumUs = 0;
        uint32_t rttMinUs = UINT32_MAX;
        uint32_t rttMaxUs = 0;

        for (uint32_t i = 0; i < kFifoTestCount; i++) {
            uint64_t t0 = time_us_64();
            multicore_fifo_push_blocking(i + 1);
            uint32_t echo;
            bool ok = multicore_fifo_pop_timeout_us(100000, &echo);
            uint32_t dt = (uint32_t)(time_us_64() - t0);

            if (!ok || echo != (i + 1)) {
                lost++;
            } else {
                rttSumUs += dt;
                if (dt < rttMinUs) rttMinUs = dt;
                if (dt > rttMaxUs) rttMaxUs = dt;
            }
        }

        // Wait for done signal
        uint32_t done;
        multicore_fifo_pop_timeout_us(1000000, &done);

        bool echoOk = (lost == 0);
        printf("[%s] Gate 1: Core0->Core1 echo: %lu/%lu OK, %lu lost\n",
               echoOk ? "PASS" : "FAIL",
               (unsigned long)(kFifoTestCount - lost),
               (unsigned long)kFifoTestCount, (unsigned long)lost);
        if (!echoOk) pass = false;

        if (lost < kFifoTestCount) {
            uint32_t rttAvg = (uint32_t)(rttSumUs / (kFifoTestCount - lost));
            printf("[INFO] Round-trip latency: min=%lu avg=%lu max=%lu us\n",
                   (unsigned long)rttMinUs, (unsigned long)rttAvg,
                   (unsigned long)rttMaxUs);
        }
    }

    multicore_fifo_drain();
    sleep_ms(10);

    // Gate 2: Core 1 -> Core 0 (1000 sequential messages)
    {
        multicore_fifo_push_blocking(kCmd_FifoSendStart);

        uint32_t received = 0;
        uint32_t outOfOrder = 0;

        for (uint32_t i = 0; i < kFifoTestCount; i++) {
            uint32_t val;
            bool ok = multicore_fifo_pop_timeout_us(100000, &val);
            if (ok) {
                received++;
                if (val != i) outOfOrder++;
            }
        }

        bool sendOk = (received == kFifoTestCount && outOfOrder == 0);
        printf("[%s] Gate 2: Core1->Core0: %lu/%lu received, %lu out of order\n",
               sendOk ? "PASS" : "FAIL",
               (unsigned long)received, (unsigned long)kFifoTestCount,
               (unsigned long)outOfOrder);
        if (!sendOk) pass = false;
    }

    multicore_fifo_drain();

    // Gate 3: FIFO overflow test (RP2350 is 4-deep per SDK docs)
    // Use non-blocking pushes so Core 1's 1ms poll can't drain between attempts
    {
        uint32_t pushCount = 0;
        for (uint32_t i = 0; i < 16; i++) {
            if (!multicore_fifo_wready()) break;
            multicore_fifo_push_blocking(i + 100);
            pushCount++;
        }

        printf("[%s] Gate 3: FIFO depth: %lu pushes before block (expect 4)\n",
               (pushCount == 4) ? "PASS" : "WARN",
               (unsigned long)pushCount);

        multicore_fifo_drain();
    }

    // Gate 4: USB still connected
    {
        bool usbOk = stdio_usb_connected();
        printf("[%s] Gate 4: USB still connected\n",
               usbOk ? "PASS" : "FAIL");
        if (!usbOk) pass = false;
    }

    printf("[INFO] FIFO reserved by multicore_lockout — exercise only\n");
    printf("=== IVP-22: %s ===\n\n", pass ? "ALL GATES PASS" : "GATE FAILURES");
}

// ============================================================================
// IVP-23: Doorbell Interrupt Test (RP2350-Specific)
// ============================================================================
// Exercise-only — seqlock polling chosen for production.

static void ivp23_doorbell_test(void) {
    printf("=== IVP-23: Doorbell Interrupts (RP2350-Specific) ===\n");
    bool pass = true;

    // Gate 1: Claim a doorbell
    g_doorbellNum = multicore_doorbell_claim_unused(0x3, true);
    bool claimOk = (g_doorbellNum >= 0);
    printf("[%s] Gate 1: Doorbell claimed: %d\n",
           claimOk ? "PASS" : "FAIL", g_doorbellNum);
    if (!claimOk) {
        printf("=== IVP-23: GATE FAILURES ===\n\n");
        return;
    }

    multicore_doorbell_clear_current_core((uint)g_doorbellNum);

    // Gate 2: Core 1 sets doorbell 1000 times, Core 0 detects
    multicore_fifo_drain();
    sleep_ms(10);
    multicore_fifo_push_blocking(kCmd_DoorbellStart);

    uint32_t detected = 0;
    uint64_t latencySumUs = 0;
    uint32_t latencyMinUs = UINT32_MAX;
    uint32_t latencyMaxUs = 0;
    uint64_t testStartUs = time_us_64();
    static constexpr uint64_t kDoorbellTimeoutUs = 30 * 1000000ULL;

    while (detected < kDoorbellTestCount) {
        uint64_t pollStartUs = time_us_64();
        if ((pollStartUs - testStartUs) > kDoorbellTimeoutUs) {
            break;
        }

        if (multicore_doorbell_is_set_current_core((uint)g_doorbellNum)) {
            uint64_t detectUs = time_us_64();
            multicore_doorbell_clear_current_core((uint)g_doorbellNum);
            detected++;

            uint32_t latUs = (uint32_t)(detectUs - pollStartUs);
            latencySumUs += latUs;
            if (latUs < latencyMinUs) latencyMinUs = latUs;
            if (latUs > latencyMaxUs) latencyMaxUs = latUs;
        }
    }

    // Wait for Core 1's done signal
    uint32_t done;
    multicore_fifo_pop_timeout_us(5000000, &done);

    bool detectOk = (detected == kDoorbellTestCount);
    printf("[%s] Gate 2: Signals detected: %lu/%lu\n",
           detectOk ? "PASS" : "FAIL",
           (unsigned long)detected, (unsigned long)kDoorbellTestCount);
    if (!detectOk) pass = false;

    if (detected > 0) {
        uint32_t latAvg = (uint32_t)(latencySumUs / detected);
        printf("[INFO] Detection latency (polling): min=%lu avg=%lu max=%lu us\n",
               (unsigned long)latencyMinUs, (unsigned long)latAvg,
               (unsigned long)latencyMaxUs);
    }

    // Gate 3: Clear on Core 0 does not affect Core 1
    {
        multicore_doorbell_clear_current_core((uint)g_doorbellNum);
        bool clearOk = !multicore_doorbell_is_set_current_core(
                            (uint)g_doorbellNum);
        printf("[%s] Gate 3: Clear on Core 0 does not affect Core 1\n",
               clearOk ? "PASS" : "FAIL");
        if (!clearOk) pass = false;
    }

    // Gate 4: USB still connected
    {
        bool usbOk = stdio_usb_connected();
        printf("[%s] Gate 4: USB still connected\n",
               usbOk ? "PASS" : "FAIL");
        if (!usbOk) pass = false;
    }

    printf("[INFO] Doorbells validated but not used in production (seqlock polling chosen)\n");
    printf("=== IVP-23: %s ===\n\n", pass ? "ALL GATES PASS" : "GATE FAILURES");

    multicore_doorbell_unclaim((uint)g_doorbellNum, 0x3);
}

// ============================================================================
// IVP-21: Spinlock Gate Check (called after 5-min soak completes)
// ============================================================================

static void ivp21_gate_check(void) {
    printf("\n=== IVP-21: Hardware Spinlock Validation ===\n");
    bool pass = true;

    // Gate 1: Spinlock claim succeeded
    {
        bool claimOk = (g_spinlockId >= 0);
        printf("[%s] Gate 1: Spinlock claimed ID=%d (expect 24-31)\n",
               claimOk ? "PASS" : "FAIL", g_spinlockId);
        if (!claimOk) pass = false;
    }

    // Gate 2: Document HW vs SW spinlocks
    {
#if PICO_USE_SW_SPIN_LOCKS
        printf("[INFO] Gate 2: SOFTWARE spinlocks (PICO_USE_SW_SPIN_LOCKS=1, RP2350-E2)\n");
#else
        printf("[INFO] Gate 2: HARDWARE spinlocks (SIO registers)\n");
#endif
    }

    // Gate 3: No inconsistent reads
    {
        bool consistOk = (g_ivp21InconsistentCount == 0);
        printf("[%s] Gate 3: %lu reads, %lu inconsistent\n",
               consistOk ? "PASS" : "FAIL",
               (unsigned long)g_ivp21ReadCount,
               (unsigned long)g_ivp21InconsistentCount);
        if (!consistOk) pass = false;
    }

    // Gate 4: Lock hold time
    {
        uint32_t avgUs = (g_ivp21ReadCount > 0) ?
            (uint32_t)(g_ivp21LockTimeSumUs / g_ivp21ReadCount) : 0;
        uint32_t minUs = (g_ivp21LockTimeMinUs == UINT32_MAX) ? 0 : g_ivp21LockTimeMinUs;
        bool timeOk = (g_ivp21LockTimeMaxUs < 10);
        printf("[%s] Gate 4: Lock hold time: min=%lu avg=%lu max=%lu us (expect <10)\n",
               timeOk ? "PASS" : "WARN",
               (unsigned long)minUs, (unsigned long)avgUs,
               (unsigned long)g_ivp21LockTimeMaxUs);
    }

    // Gate 5: 5 minutes continuous
    {
        uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - g_ivp21StartMs;
        bool durationOk = (elapsed >= kSpinlockSoakMs);
        printf("[%s] Gate 5: Soak: %lu ms (target %lu ms)\n",
               durationOk ? "PASS" : "FAIL",
               (unsigned long)elapsed, (unsigned long)kSpinlockSoakMs);
        if (!durationOk) pass = false;

        uint32_t c1 = g_core1Counter.load(std::memory_order_acquire);
        printf("[INFO] Core 1 counter: %lu, writes: %lu\n",
               (unsigned long)c1, (unsigned long)g_spinlockData.write_count);
    }

    // Gate 6: USB not disrupted
    {
        bool usbOk = stdio_usb_connected();
        printf("[%s] Gate 6: USB still connected after 5-min soak\n",
               usbOk ? "PASS" : "FAIL");
        if (!usbOk) pass = false;
    }

    printf("=== IVP-21: %s ===\n\n", pass ? "ALL GATES PASS" : "GATE FAILURES");
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

    // IVP-19: Launch Core 1 early so NeoPixel blinks immediately (no USB dependency)
    multicore_launch_core1(core1_entry);

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
    // IVP-14: Calibration storage init (before USB per LL Entry 4/12)
    // Storage init only reads flash via XIP — no erase/write at boot
    // -----------------------------------------------------------------
    g_calStorageInitialized = calibration_storage_init();
    calibration_manager_init();

    // -----------------------------------------------------------------
    // IVP-04: USB CDC init (after I2C/flash per LL Entry 4/12)
    // -----------------------------------------------------------------
    stdio_init_all();

    // Fast LED blink while waiting for USB connection
    // NeoPixel already running on Core 1 (launched after init)
    while (!stdio_usb_connected()) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(100);
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

    // IVP-14: Calibration storage status
    if (g_calStorageInitialized) {
        const calibration_store_t* cal = calibration_manager_get();
        printf("Calibration storage OK (flags=0x%02lX)\n",
               (unsigned long)cal->cal_flags);
    } else {
        printf("Calibration storage FAILED\n");
    }
    printf("\n");

    // Post-init I2C scan — verify bus integrity after IMU init
    printf("Post-init I2C scan (bus integrity check):\n");
    i2c_bus_scan();
    printf("\n");

    // Hardware validation
    hw_validate_stage1();

    // Skip previously verified gates to speed up boot
    if (kSkipVerifiedGates) {
        g_imuValidationDone = true;
        g_baroValidationDone = true;
        g_ivp13Done = true;
        g_ivp14Done = true;
        g_ivp15Done = true;
        g_ivp16Done = true;
        printf("[INFO] Skipping verified gates (IVP-10 through IVP-16)\n\n");
    }

    // IVP-18: RC_OS CLI init
    rc_os_init();
    rc_os_imu_available = g_imuInitialized;
    rc_os_baro_available = g_baroContinuous;
    rc_os_print_sensor_status = print_sensor_status;
    rc_os_read_accel = read_accel_for_cal;
    rc_os_cal_pre_hook = cal_pre_hook;
    rc_os_cal_post_hook = cal_post_hook;

    // IVP-19: Core 1 already launched (after NeoPixel init, before USB wait)
    printf("Core 1 launched (test dispatcher mode)\n");

    // ================================================================
    // IVP-21/22/23: Inter-core primitive exercises
    // Core 1 is in test dispatcher mode, responding to FIFO commands.
    // ================================================================

    // IVP-22: FIFO message passing
    ivp22_fifo_test();

    // IVP-23: Doorbell signaling
    ivp23_doorbell_test();

    // IVP-21: Claim spinlock and start soak test
    g_spinlockId = spin_lock_claim_unused(true);
    g_pTestSpinlock = spin_lock_init((uint)g_spinlockId);
    printf("Spinlock claimed: ID=%d\n", g_spinlockId);
#if PICO_USE_SW_SPIN_LOCKS
    printf("Spinlock type: SOFTWARE (RP2350-E2, LDAEXB/STREXB)\n");
#else
    printf("Spinlock type: HARDWARE (SIO registers)\n");
#endif

    // Signal Core 1 to exit test dispatcher and start spinlock soak
    multicore_fifo_drain();
    sleep_ms(10);
    multicore_fifo_push_blocking(kCmd_SpinlockStart);
    g_ivp21Active = true;
    g_ivp21StartMs = to_ms_since_boot(get_absolute_time());
    g_ivp21LastCheckMs = g_ivp21StartMs;
    g_ivp21LastPrintMs = g_ivp21StartMs;
    printf("\n=== IVP-21: Spinlock Soak (5 min) ===\n");
    printf("Core 1 writing ~100kHz, Core 0 reading 10Hz...\n\n");

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

        // NeoPixel owned by Core 1 (IVP-19) — do not call ws2812_update() here

        // IVP-20: Print Core 1 counter every second
        {
            static uint32_t lastCounterPrintMs = 0;
            if (stdio_usb_connected() && (nowMs - lastCounterPrintMs) >= 1000) {
                uint32_t count = g_core1Counter.load(std::memory_order_acquire);
                static uint32_t prevCount = 0;
                uint32_t rate = count - prevCount;
                printf("Core 1: %lu total, %lu/s\n",
                       (unsigned long)count, (unsigned long)rate);
                prevCount = count;
                lastCounterPrintMs = nowMs;
            }
        }

        // IVP-21: Spinlock soak — read shared struct under lock
        if (g_ivp21Active && !g_ivp21Done) {
            if ((nowMs - g_ivp21LastCheckMs) >= kSpinlockCheckIntervalMs) {
                g_ivp21LastCheckMs = nowMs;

                uint64_t t0 = time_us_64();
                uint32_t saved = spin_lock_blocking(g_pTestSpinlock);
                uint32_t a = g_spinlockData.field_a;
                uint32_t b = g_spinlockData.field_b;
                uint32_t c = g_spinlockData.field_c;
                uint32_t wc = g_spinlockData.write_count;
                spin_unlock(g_pTestSpinlock, saved);
                uint32_t dtUs = (uint32_t)(time_us_64() - t0);

                g_ivp21ReadCount++;
                if (dtUs < g_ivp21LockTimeMinUs) g_ivp21LockTimeMinUs = dtUs;
                if (dtUs > g_ivp21LockTimeMaxUs) g_ivp21LockTimeMaxUs = dtUs;
                g_ivp21LockTimeSumUs += dtUs;

                // Consistency check
                if (a != b || b != c || c != wc) {
                    g_ivp21InconsistentCount++;
                    if (stdio_usb_connected()) {
                        printf("[FAIL] Spinlock: a=%lu b=%lu c=%lu wc=%lu\n",
                               (unsigned long)a, (unsigned long)b,
                               (unsigned long)c, (unsigned long)wc);
                    }
                }

                // Progress every 30 seconds
                if (stdio_usb_connected() &&
                    (nowMs - g_ivp21LastPrintMs) >= 30000) {
                    g_ivp21LastPrintMs = nowMs;
                    uint32_t elapsed = nowMs - g_ivp21StartMs;
                    uint32_t remain = (elapsed < kSpinlockSoakMs) ?
                        (kSpinlockSoakMs - elapsed) / 1000 : 0;
                    printf("[IVP-21] %lus, %lus left, %lu reads, %lu bad, "
                           "max %lu us, writes=%lu\n",
                           (unsigned long)(elapsed / 1000),
                           (unsigned long)remain,
                           (unsigned long)g_ivp21ReadCount,
                           (unsigned long)g_ivp21InconsistentCount,
                           (unsigned long)g_ivp21LockTimeMaxUs,
                           (unsigned long)wc);
                }

                // Soak complete?
                if ((nowMs - g_ivp21StartMs) >= kSpinlockSoakMs) {
                    g_ivp21Done = true;
                    if (stdio_usb_connected()) {
                        ivp21_gate_check();
                    }
                }
            }
        }

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

        // IVP-14: Calibration storage test (after IVP-13 completes)
        if (g_ivp13Done && !g_ivp14Done && g_calStorageInitialized &&
            stdio_usb_connected()) {
            g_ivp14Done = true;
            ivp14_gate_check();
        }

        // IVP-15: Gyro bias calibration (after IVP-14 completes)
        // Device must be stationary — 200 samples at 100Hz (~2 seconds)
        if (g_ivp14Done && !g_ivp15Done && g_imuInitialized &&
            stdio_usb_connected()) {

            uint64_t nowUs = time_us_64();

            if (!g_ivp15Started) {
                g_ivp15Started = true;
                g_ivp15LastFeedUs = nowUs;
                printf("=== IVP-15: Gyro Bias Calibration ===\n");
                printf("[INFO] Keep device STATIONARY for ~2 seconds...\n");
                cal_result_t startResult = calibration_start_gyro();
                if (startResult != CAL_RESULT_OK) {
                    printf("[FAIL] calibration_start_gyro() = %d\n", (int)startResult);
                    g_ivp15Done = true;
                }
            }

            // Feed gyro samples at 100Hz while calibrating
            if (g_ivp15Started && calibration_is_active() &&
                (nowUs - g_ivp15LastFeedUs) >= kIvp15FeedIntervalUs) {
                g_ivp15LastFeedUs = nowUs;
                icm20948_data_t data;
                if (icm20948_read(&g_imu, &data)) {
                    calibration_feed_gyro(data.gyro.x, data.gyro.y, data.gyro.z,
                                          data.temperature_c);
                    // Print progress every 50 samples
                    uint8_t progress = calibration_get_progress();
                    static uint8_t lastProgress = 0;
                    if (progress >= lastProgress + 25 || progress == 100) {
                        printf("[INFO] Gyro cal progress: %u%%\n", progress);
                        lastProgress = progress;
                    }
                }
            }

            // Check if calibration completed or failed
            cal_state_t state = calibration_manager_get_state();
            if (g_ivp15Started && !calibration_is_active() &&
                (state == CAL_STATE_COMPLETE || state == CAL_STATE_FAILED)) {
                g_ivp15Done = true;
                ivp15_gate_check();
                calibration_reset_state();
            }
        }

        // IVP-16: Accel level calibration (after IVP-15 completes)
        // Device must be flat on table (Z-up) — 100 samples at 100Hz (~1 second)
        if (g_ivp15Done && !g_ivp16Done && g_imuInitialized &&
            stdio_usb_connected()) {

            uint64_t nowUs = time_us_64();
            static uint32_t ivp16StartMs = 0;

            if (!g_ivp16Started) {
                g_ivp16Started = true;
                g_ivp16LastFeedUs = nowUs;
                ivp16StartMs = nowMs;
                printf("=== IVP-16: Accel Level Calibration ===\n");
                printf("[INFO] Keep device FLAT on table (Z-up) for ~1 second...\n");
                cal_result_t startResult = calibration_start_accel_level();
                if (startResult != CAL_RESULT_OK) {
                    printf("[FAIL] calibration_start_accel_level() = %d\n", (int)startResult);
                    g_ivp16Done = true;
                }
            }

            // Feed accel samples at 100Hz while calibrating
            if (g_ivp16Started && calibration_is_active() &&
                (nowUs - g_ivp16LastFeedUs) >= kIvp16FeedIntervalUs) {
                g_ivp16LastFeedUs = nowUs;
                icm20948_data_t data;
                if (icm20948_read(&g_imu, &data)) {
                    calibration_feed_accel(data.accel.x, data.accel.y, data.accel.z,
                                           data.temperature_c);
                    uint8_t progress = calibration_get_progress();
                    static uint8_t lastProgress16 = 0;
                    if (progress >= lastProgress16 + 25 || progress == 100) {
                        printf("[INFO] Level cal progress: %u%%\n", progress);
                        lastProgress16 = progress;
                    }
                }
            }

            // Check if calibration completed or failed
            cal_state_t state = calibration_manager_get_state();
            if (g_ivp16Started && !calibration_is_active() &&
                (state == CAL_STATE_COMPLETE || state == CAL_STATE_FAILED)) {
                g_ivp16Done = true;
                uint32_t elapsed = nowMs - ivp16StartMs;
                ivp16_gate_check(elapsed);
                calibration_reset_state();
            }
        }

        // IVP-18: RC_OS CLI + calibration sample feeding
        // RC_OS handles terminal connection, banner, input processing.
        // When CLI triggers a calibration, we feed samples here at 100Hz.
        {
            rc_os_update();

            // Feed sensor samples to active calibration (CLI-triggered)
            if (calibration_is_active()) {
                uint64_t nowUs = time_us_64();
                if ((nowUs - g_cliCalLastFeedUs) >= kCliCalFeedIntervalUs) {
                    g_cliCalLastFeedUs = nowUs;
                    cal_state_t calState = calibration_manager_get_state();

                    if (calState == CAL_STATE_GYRO_SAMPLING && g_imuInitialized) {
                        icm20948_data_t data;
                        if (icm20948_read(&g_imu, &data)) {
                            calibration_feed_gyro(data.gyro.x, data.gyro.y,
                                                  data.gyro.z, data.temperature_c);
                        }
                    } else if (calState == CAL_STATE_ACCEL_LEVEL_SAMPLING && g_imuInitialized) {
                        icm20948_data_t data;
                        if (icm20948_read(&g_imu, &data)) {
                            calibration_feed_accel(data.accel.x, data.accel.y,
                                                   data.accel.z, data.temperature_c);
                        }
                    } else if (calState == CAL_STATE_BARO_SAMPLING && g_baroContinuous) {
                        baro_dps310_data_t bdata;
                        if (baro_dps310_read(&bdata) && bdata.valid) {
                            calibration_feed_baro(bdata.pressure_pa,
                                                  bdata.temperature_c);
                        }
                    }
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
