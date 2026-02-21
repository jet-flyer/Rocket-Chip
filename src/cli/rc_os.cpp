/**
 * @file rc_os.c
 * @brief RocketChip OS - CLI menu system implementation
 *
 * Bare-metal adaptation of RC_OS v0.3 CLI.
 * Uses calibration_manager for calibration operations.
 */

#include "rc_os.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "calibration/calibration_manager.h"
#include "calibration/calibration_data.h"
#include "drivers/i2c_bus.h"
#include "rocketchip/config.h"
#include "hardware/watchdog.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// ============================================================================
// Configuration
// ============================================================================

constexpr const char* kRcOsVersion = "0.4.0";
constexpr uint32_t kRcOsPollMs     = 50;   // 20Hz polling

// CLI input constants
constexpr int      kEscChar              = 27;           // ASCII ESC
constexpr uint32_t kResetConfirmTimeoutUs = 10000000;    // 10 seconds for reset confirm
constexpr size_t   kResetConfirmBufSize  = 8;            // "YES\0" + margin
constexpr size_t   kResetConfirmMaxIdx   = 7;            // Buffer max index

// Verification constants
constexpr uint8_t  kVerifySampleCount    = 10;           // Live samples for post-cal check
constexpr uint32_t kUsbSettleMs          = 200;          // USB CDC settle time after connect

// INTERIM: NeoPixel calibration override values (Phase M.5).
// Must match kCalNeo* constants in main.cpp. Replace when AP_Notify state machine is implemented.
constexpr uint8_t kCalNeoOff         = 0;
constexpr uint8_t kCalNeoGyro        = 1;
constexpr uint8_t kCalNeoLevel       = 2;
constexpr uint8_t kCalNeoBaro        = 3;
constexpr uint8_t kCalNeoAccelWait   = 4;
constexpr uint8_t kCalNeoAccelSample = 5;
constexpr uint8_t kCalNeoMag         = 6;
constexpr uint8_t kCalNeoSuccess     = 7;
constexpr uint8_t kCalNeoFail        = 8;
constexpr uint32_t kNeoFlashDurationMs = 1000;           // Green/red flash after step

// 6-position calibration
constexpr uint8_t  kAccel6posNumPositions = 6;           // Orthogonal orientations for gravity fit

// Mag calibration (IVP-37)
constexpr uint32_t kMagCalPollMs          = 50;           // ~20Hz mag sample rate
constexpr uint16_t kMagCalProgressInterval = 25;          // Print every N accepted samples
constexpr uint16_t kMagCalMinSamples      = 50;           // Minimum for reliable fit
constexpr uint32_t kMagCalDiagIntervalMs  = 3000;         // Diagnostic print interval (no data)
constexpr uint32_t kMagCalSlowDiagMs      = 5000;         // Diagnostic print interval (slow progress)
constexpr uint16_t kMagCalMinForSlow      = 10;           // Minimum samples before slow diagnostic
constexpr uint32_t kCalFeedPollMs         = 10;           // Calibration feed poll interval
constexpr uint8_t  kProgressPercentStep   = 10;           // Calibration progress print granularity (%)

// ============================================================================
// State
// ============================================================================

static rc_os_menu_t g_menu = RC_OS_MENU_MAIN;
static bool g_wasConnected = false;
static bool g_bannerPrinted = false;
// (no extra state needed — reset confirmation is blocking)

// Callback for sensor status (set by main.cpp)
rc_os_sensor_status_fn rc_os_print_sensor_status = nullptr;

// Callback for boot summary reprint (set by main.cpp)
rc_os_boot_summary_fn rc_os_print_boot_summary = nullptr;

// Callback for full boot status on first connect (set by main.cpp)
rc_os_boot_status_fn rc_os_print_boot_status = nullptr;

// Sensor availability flags (set by main.cpp)
bool rc_os_imu_available = false;
bool rc_os_baro_available = false;
bool rc_os_i2c_scan_allowed = true;
volatile bool rc_os_mag_cal_active = false;

// Accel read callback for 6-pos calibration (set by main.cpp)
rc_os_read_accel_fn rc_os_read_accel = nullptr;
rc_os_cal_hook_fn rc_os_cal_pre_hook = nullptr;
rc_os_cal_hook_fn rc_os_cal_post_hook = nullptr;

// Mag read callback for compass calibration (set by main.cpp)
rc_os_read_mag_fn rc_os_read_mag = nullptr;

// Mag staleness reset callback (set by main.cpp)
rc_os_reset_mag_staleness_fn rc_os_reset_mag_staleness = nullptr;

// Unhandled key callback (set by main.cpp for extensible key handling)
rc_os_unhandled_key_fn rc_os_on_unhandled_key = nullptr;

// INTERIM: NeoPixel calibration override callback (Phase M.5)
rc_os_set_cal_neo_fn rc_os_set_cal_neo = nullptr;

// Calibration sensor feed callback (set by main.cpp)
rc_os_feed_cal_fn rc_os_feed_cal = nullptr;

// ESKF live output callback (set by main.cpp)
rc_os_eskf_live_fn rc_os_print_eskf_live = nullptr;

// Live ESKF mode state
static bool g_eskfLiveActive = false;
static uint32_t g_eskfLiveLastPrintUs = 0;
static constexpr uint32_t kEskfLivePeriodUs = 1000000;  // 1Hz

// ============================================================================
// Menu Printing
// ============================================================================

static void print_banner() {
    printf("\n");
    printf("========================================\n");
    printf("  RocketChip OS v%s\n", kRcOsVersion);
    printf("  Press 'h' for help\n");
    printf("========================================\n\n");
}

static void print_system_status() {
    printf("\n========================================\n");
    printf("  RocketChip System Status\n");
    printf("========================================\n");
    printf("  Version: %s\n", kVersionString);
    printf("  Board: Adafruit Feather RP2350 HSTX\n");
    printf("  Uptime: %lu ms\n", (unsigned long)to_ms_since_boot(get_absolute_time()));

    // Calibration status
    const calibration_store_t* cal = calibration_manager_get();
    printf("----------------------------------------\n");
    printf("Calibration Status:\n");
    printf("  Gyro:  %s\n", (cal->cal_flags & CAL_STATUS_GYRO) != 0 ? "OK" : "NOT DONE");
    if ((cal->cal_flags & CAL_STATUS_ACCEL_6POS) != 0) {
        printf("  Accel: 6POS\n");
    } else if ((cal->cal_flags & CAL_STATUS_LEVEL) != 0) {
        printf("  Accel: LEVEL\n");
    } else {
        printf("  Accel: NOT DONE\n");
    }
    printf("  Baro:  %s\n", (cal->cal_flags & CAL_STATUS_BARO) != 0 ? "OK" : "NOT DONE");
    printf("  Mag:   %s\n", (cal->cal_flags & CAL_STATUS_MAG) != 0 ? "OK" : "NOT DONE");

    printf("----------------------------------------\n");
    printf("Commands:\n");
    printf("  h - This help\n");
    printf("  s - Sensor status\n");
    printf("  e - ESKF live (1Hz, any key stops)\n");
    printf("  b - Boot summary (reprint)\n");
    printf("  i - I2C bus rescan\n");
    printf("  c - Calibration menu\n");
    printf("========================================\n\n");
}

static void print_calibration_menu() {
    printf("\n========================================\n");
    printf("  Calibration Menu\n");
    printf("========================================\n");
    printf("  g - Gyro calibration (keep still)\n");
    printf("  l - Level calibration (keep flat)\n");
    printf("  b - Baro calibration (ground ref)\n");
    printf("  a - 6-position accel calibration\n");
    printf("  m - Compass calibration\n");
    printf("  w - Full wizard (all in sequence)\n");
    printf("  r - Reset all calibration\n");
    printf("  v - Save calibration to flash\n");
    printf("  x - Return to main menu\n");
    printf("========================================\n\n");
}

// ============================================================================
// Forward Declarations
// ============================================================================

static void update_calibration_progress();
static bool wait_for_enter_or_esc();
static bool wait_for_async_cal();

// ============================================================================
// NeoPixel Calibration Helpers (INTERIM — Phase M.5)
// ============================================================================

static void cal_neo(uint8_t mode) {
    if (rc_os_set_cal_neo != nullptr) {
        rc_os_set_cal_neo(mode);
    }
}

static void cal_neo_flash_result(bool success) {
    cal_neo(success ? kCalNeoSuccess : kCalNeoFail);
    sleep_ms(kNeoFlashDurationMs);
    cal_neo(kCalNeoOff);
}

// ============================================================================
// Calibration Commands
// ============================================================================

static void cmd_gyro_cal() {
    if (!rc_os_imu_available) {
        printf("\nERROR: IMU not available\n");
        return;
    }

    printf("\nGyro Calibration — keep device STILL for ~2s.\n");
    printf("Press ENTER to start, 'x' to cancel.\n");
    cal_neo(kCalNeoGyro);
    if (!wait_for_enter_or_esc()) {
        cal_neo(kCalNeoOff);
        return;
    }

    cal_result_t result = calibration_start_gyro();
    if (result != CAL_RESULT_OK) {
        printf("ERROR: Failed to start gyro cal (%d)\n", result);
        cal_neo(kCalNeoOff);
        return;
    }

    printf("Sampling");
    (void)fflush(stdout);
    if (wait_for_async_cal()) {
        cal_neo_flash_result(true);
    } else {
        cal_neo_flash_result(false);
    }
}

static void cmd_level_cal() {
    if (!rc_os_imu_available) {
        printf("\nERROR: IMU not available\n");
        return;
    }

    printf("\nLevel Calibration — keep device FLAT and STILL for ~1s.\n");
    printf("Press ENTER to start, 'x' to cancel.\n");
    cal_neo(kCalNeoLevel);
    if (!wait_for_enter_or_esc()) {
        cal_neo(kCalNeoOff);
        return;
    }

    cal_result_t result = calibration_start_accel_level();
    if (result != CAL_RESULT_OK) {
        printf("ERROR: Failed to start level cal (%d)\n", result);
        cal_neo(kCalNeoOff);
        return;
    }

    printf("Sampling");
    (void)fflush(stdout);
    if (wait_for_async_cal()) {
        cal_neo_flash_result(true);
    } else {
        cal_neo_flash_result(false);
    }
}

static void cmd_baro_cal() {
    if (!rc_os_baro_available) {
        printf("\nERROR: Barometer not available\n");
        return;
    }

    printf("\nBaro Calibration — setting ground reference (~1s).\n");
    printf("Press ENTER to start, 'x' to cancel.\n");
    cal_neo(kCalNeoBaro);
    if (!wait_for_enter_or_esc()) {
        cal_neo(kCalNeoOff);
        return;
    }

    cal_result_t result = calibration_start_baro();
    if (result != CAL_RESULT_OK) {
        printf("ERROR: Failed to start baro cal (%d)\n", result);
        cal_neo(kCalNeoOff);
        return;
    }

    printf("Sampling");
    (void)fflush(stdout);
    if (wait_for_async_cal()) {
        cal_neo_flash_result(true);
    } else {
        cal_neo_flash_result(false);
    }
}

static void cmd_save_cal() {
    printf("\nSaving calibration to flash...");
    (void)fflush(stdout);

    cal_result_t result = calibration_save();
    if (result == CAL_RESULT_OK) {
        printf(" OK!\n");
        if (!i2c_bus_reset()) {  // Flash ops can disrupt I2C
            printf("[WARN] I2C bus reset failed after save\n");
        }
    } else {
        printf(" FAILED (%d)\n", result);
    }
}

static void cmd_reset_cal() {
    printf("\n*** RESET ALL CALIBRATION ***\n");
    printf("This will erase all calibration data!\n");
    printf("Type 'YES' + ENTER to confirm: ");
    (void)fflush(stdout);

    char confirm[kResetConfirmBufSize] = {0};
    int idx = 0;
    bool gotEnter = false;
    while (idx < (int)kResetConfirmMaxIdx) {
        int ch = getchar_timeout_us(kResetConfirmTimeoutUs);
        if (ch == PICO_ERROR_TIMEOUT) {
            printf("\nTimeout - cancelled.\n");
            return;
        }
        if (ch == '\r' || ch == '\n') {
            gotEnter = true;
            break;
        }
        confirm[idx++] = (char)ch;
        printf("%c", ch);
        (void)fflush(stdout);
    }
    printf("\n");

    if (gotEnter && strcmp(confirm, "YES") == 0) {
        printf("Resetting all calibration data...");
        (void)fflush(stdout);
        cal_result_t result = calibration_reset();
        if (result == CAL_RESULT_OK) {
            printf(" OK!\n");
        } else {
            printf(" FAILED (%d)\n", result);
        }
    } else {
        printf("Cancelled (need to type 'YES' then ENTER).\n");
    }
}

// Per-position instructions for the user
// Order matches calibration_manager.c kPositionNames (QGroundControl order)
static const char* const kPositionInstructions[kAccel6posNumPositions] = {
    "Place board FLAT on table, component side UP",
    "Stand board on its LEFT edge",
    "Stand board on its RIGHT edge",
    "Stand board on USB connector end (nose down)",
    "Stand board on opposite end from USB (nose up)",
    "Place board FLAT on table, component side DOWN (inverted)"
};

constexpr uint8_t  kAccel6posMaxRetries     = 3;
constexpr uint32_t kAccel6posWaitTimeoutUs  = 30000000;     // 30 seconds
constexpr float    kAccel6posGravityNominal = 9.80665F;     // m/s² (WGS-84 standard)
constexpr float    kAccel6posVerifyTolerance = 0.02F;       // m/s² (6-pos calibration gate)

// Helper: wait for ENTER key with timeout, ESC cancels. Returns true on ENTER.
static bool wait_for_enter_or_esc() {
    // Drain stale input (leftover \r\n from menu or prior prompt).
    // Two passes with a gap — USB CDC delivers in packets, so in-flight
    // bytes may not be available on the first drain.
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {}
    sleep_ms(100);
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {}

    uint32_t elapsedMs = 0;
    uint32_t timeoutMs = kAccel6posWaitTimeoutUs / 1000;

    while (elapsedMs < timeoutMs) {
        int ch = getchar_timeout_us(0);
        if (ch == kEscChar || ch == 'x' || ch == 'X') {
            printf("\nCancelled.\n");
            return false;
        }
        if (ch == '\r' || ch == '\n') {
            return true;
        }
        if (!stdio_usb_connected()) {
            return false;
        }
        watchdog_update();
        sleep_ms(kRcOsPollMs);
        elapsedMs += kRcOsPollMs;
    }

    printf("\nTimeout - calibration cancelled.\n");
    return false;
}

// Collect one position with retries. Returns true on success, false on abort/failure.
static bool collect_6pos_position(uint8_t pos) {
    printf("--- Position %d/%d: %s ---\n", pos + 1, kAccel6posNumPositions,
           calibration_get_6pos_name(pos));
    printf("  %s\n", kPositionInstructions[pos]);
    printf("  Press ENTER when ready...\n");
    (void)fflush(stdout);
    cal_neo(kCalNeoAccelWait);

    if (!wait_for_enter_or_esc()) {
        return false;
    }

    printf("  Sampling... hold still");
    (void)fflush(stdout);
    cal_neo(kCalNeoAccelSample);

    for (uint8_t retry = 0; retry < kAccel6posMaxRetries; retry++) {
        cal_result_t result = calibration_collect_6pos_position(pos, rc_os_read_accel);

        if (result == CAL_RESULT_OK) {
            const float* avg = calibration_get_6pos_avg(pos);
            printf(" OK!\n");
            printf("  Avg: X=%.2f Y=%.2f Z=%.2f\n",
                   (double)avg[0], (double)avg[1], (double)avg[2]);
            cal_neo_flash_result(true);
            return true;
        }

        const char* errMsg = nullptr;
        if (result == CAL_RESULT_MOTION_DETECTED) {
            errMsg = "Motion detected! Hold still and try again.";
        } else if (result == CAL_RESULT_INVALID_DATA) {
            printf("\n  Orientation doesn't match expected: %s\n",
                   calibration_get_6pos_name(pos));
            errMsg = "Please reposition the board correctly.";
        } else {
            printf("\n  ERROR: Failed to read sensor (%d)\n", result);
            cal_neo_flash_result(false);
            return false;
        }

        printf("\n  %s\n", errMsg);
        if (retry < kAccel6posMaxRetries - 1) {
            printf("  Press ENTER to retry (%d/%d)...\n",
                   retry + 2, kAccel6posMaxRetries);
            (void)fflush(stdout);
            cal_neo(kCalNeoAccelWait);
            if (!wait_for_enter_or_esc()) {
                return false;
            }
            printf("  Sampling... hold still");
            (void)fflush(stdout);
            cal_neo(kCalNeoAccelSample);
        }
    }

    printf("  Failed after %d attempts.\n", kAccel6posMaxRetries);
    cal_neo_flash_result(false);
    return false;
}

// Print 6-pos calibration results and verify with live samples.
static void print_6pos_results() {
    const calibration_store_t* cal = calibration_manager_get();
    printf("Results:\n");
    printf("  Offset: X=%.4f Y=%.4f Z=%.4f\n",
           (double)cal->accel.offset.x,
           (double)cal->accel.offset.y,
           (double)cal->accel.offset.z);
    printf("  Scale:  X=%.4f Y=%.4f Z=%.4f\n",
           (double)cal->accel.scale.x,
           (double)cal->accel.scale.y,
           (double)cal->accel.scale.z);
    printf("  Offdiag: XY=%.4f XZ=%.4f YZ=%.4f\n",
           (double)cal->accel.offdiag.x,
           (double)cal->accel.offdiag.y,
           (double)cal->accel.offdiag.z);

    printf("\nVerification (%d live samples):\n", kVerifySampleCount);
    float magSum = 0.0F;
    uint8_t goodReads = 0;
    for (uint8_t i = 0; i < kVerifySampleCount; i++) {
        float ax = 0.0F;
        float ay = 0.0F;
        float az = 0.0F;
        float temp = 0.0F;
        if (rc_os_read_accel(&ax, &ay, &az, &temp)) {
            float cx = 0.0F;
            float cy = 0.0F;
            float cz = 0.0F;
            calibration_apply_accel(ax, ay, az, &cx, &cy, &cz);
            float mag = sqrtf(cx*cx + cy*cy + cz*cz);
            magSum += mag;
            goodReads++;
        }
    }
    if (goodReads > 0) {
        float avgMag = magSum / static_cast<float>(goodReads);
        printf("  Avg gravity magnitude: %.4f m/s^2 (expected 9.8067)\n",
               (double)avgMag);
        float error = fabsf(avgMag - kAccel6posGravityNominal);
        if (error < kAccel6posVerifyTolerance) {
            printf("  PASS (error %.4f < %.2f)\n",
                   (double)error, (double)kAccel6posVerifyTolerance);
        } else {
            printf("  WARNING: error %.4f > %.2f threshold\n",
                   (double)error, (double)kAccel6posVerifyTolerance);
        }
    }
}

// Inner body of 6-position calibration. Returns on error or completion.
// Caller (cmd_accel_6pos_cal) guarantees pre/post hook execution.
static void cmd_accel_6pos_cal_inner() {
    calibration_reset_6pos();

    for (uint8_t pos = 0; pos < kAccel6posNumPositions; pos++) {
        if (!collect_6pos_position(pos)) {
            calibration_reset_6pos();
            printf("Calibration aborted.\n");
            return;
        }
        printf("\n");
    }

    // All 6 positions collected — run the fit
    printf("All 6 positions collected!\n");
    printf("Computing ellipsoid fit...");
    (void)fflush(stdout);

    cal_result_t fitResult = calibration_compute_6pos();
    if (fitResult != CAL_RESULT_OK) {
        printf(" FAILED (%d)\n", fitResult);
        printf("Fit failed — params out of range.\n");
        printf("Common cause: board not held at true cardinal orientations.\n");
        printf("Use a flat surface or box edge for precise positioning.\n");
        calibration_reset_6pos();
        return;
    }
    printf(" OK!\n\n");

    print_6pos_results();

    // Auto-save to flash
    printf("\nSaving to flash...");
    (void)fflush(stdout);
    cal_result_t saveResult = calibration_save();
    if (saveResult == CAL_RESULT_OK) {
        printf(" OK!\n");
        if (!i2c_bus_reset()) {
            printf("[WARN] I2C bus reset failed after save\n");
        }
    } else {
        printf(" FAILED (%d)\n", saveResult);
    }

    printf("\n6-position accel calibration complete.\n");
    calibration_reset_6pos();
}

static void cmd_accel_6pos_cal() {
    if (!rc_os_imu_available) {
        printf("\nERROR: IMU not available\n");
        return;
    }
    if (rc_os_read_accel == nullptr) {
        printf("\nERROR: Accel read callback not set\n");
        return;
    }

    printf("\n========================================\n");
    printf("  6-Position Accelerometer Calibration\n");
    printf("========================================\n");
    printf("Calibrates accel offset, scale, and\n");
    printf("cross-axis coupling by measuring gravity\n");
    printf("in 6 orientations.\n\n");
    printf("Hold the board still in each position,\n");
    printf("then press ENTER to sample.\n");
    printf("Press ENTER to start, 'x' to cancel.\n");
    printf("========================================\n\n");
    cal_neo(kCalNeoAccelWait);

    if (!wait_for_enter_or_esc()) {
        cal_neo(kCalNeoOff);
        return;
    }

    // Disable I2C master before rapid accel reads (prevents bank-switching race)
    if (rc_os_cal_pre_hook != nullptr) { rc_os_cal_pre_hook(); }

    cmd_accel_6pos_cal_inner();
    cal_neo(kCalNeoOff);

    // Re-enable I2C master for normal operation (mag reads)
    if (rc_os_cal_post_hook != nullptr) { rc_os_cal_post_hook(); }
}

// Print progress and diagnostics during mag sample collection.
// Called once per accepted or processed sample inside the collection loop.
static void mag_cal_print_progress(mag_feed_result_t result,
                                   uint16_t count,
                                   float mx, float my, float mz,
                                   uint32_t totalReads,
                                   uint32_t rejectClose,
                                   uint32_t rejectRange,
                                   uint16_t* lastPrintedCount,
                                   uint32_t* lastDiagMs) {
    // Print first accepted sample immediately for feedback
    if (result == mag_feed_result_t::ACCEPTED && count == 1) {
        float mag = sqrtf(mx*mx + my*my + mz*mz);
        printf("  First sample: |M|=%.1f uT (read %lu)\n",
               (double)mag, (unsigned long)totalReads);
        (void)fflush(stdout);
    }

    // Print progress at intervals
    if (result == mag_feed_result_t::ACCEPTED &&
        count >= *lastPrintedCount + kMagCalProgressInterval) {
        *lastPrintedCount = count;
        uint8_t coverage = calibration_get_mag_coverage_pct();
        printf("  Samples: %u  Coverage: %u%%  (read %lu, close:%lu range:%lu)\n",
               count, coverage, (unsigned long)totalReads,
               (unsigned long)rejectClose, (unsigned long)rejectRange);
        (void)fflush(stdout);
    }

    // Periodic diagnostic when collecting but no new acceptances
    uint32_t nowMs = to_ms_since_boot(get_absolute_time());
    if (count > 0 && count < kMagCalMinForSlow && (nowMs - *lastDiagMs) >= kMagCalSlowDiagMs) {
        *lastDiagMs = nowMs;
        printf("  [%u accepted, read %lu, close:%lu range:%lu — rotate device!]\n",
               count, (unsigned long)totalReads,
               (unsigned long)rejectClose, (unsigned long)rejectRange);
        (void)fflush(stdout);
    }
}

// Track rejection counts and print diagnostics for rejected samples.
static void mag_cal_track_reject(mag_feed_result_t result,
                                 float mx, float my, float mz,
                                 uint32_t* outRejectClose,
                                 uint32_t* outRejectRange) {
    if (result == mag_feed_result_t::REJECTED_CLOSE) {
        (*outRejectClose)++;
    } else if (result == mag_feed_result_t::REJECTED_RANGE) {
        (*outRejectRange)++;
        // Debug: print first 3 range rejects to diagnose out-of-range values
        if (*outRejectRange <= 3) {
            float rawMag = sqrtf(mx*mx + my*my + mz*mz);
            printf("  [range reject #%lu: raw=%.1f,%.1f,%.1f |M|=%.1f uT]\n",
                   (unsigned long)*outRejectRange, (double)mx, (double)my,
                   (double)mz, (double)rawMag);
            (void)fflush(stdout);
        }
    }
}

// Collect mag samples until buffer full or user cancels.
// Returns true if collection completed (buffer full), false if cancelled/disconnected.
static bool mag_cal_collect_samples(uint16_t* outFinalCount,
                                    uint8_t* outFinalCoverage) {
    uint16_t lastPrintedCount = 0;
    uint32_t totalReads = 0;
    uint32_t readFailCount = 0;
    uint32_t rejectClose = 0;
    uint32_t rejectRange = 0;
    uint32_t lastDiagMs = 0;
    while (true) {
        int ch = getchar_timeout_us(0);
        if (ch == 'x' || ch == 'X' || ch == kEscChar) {
            printf("\nCalibration cancelled. (accepted=%lu, read=%lu, readFail=%lu, close=%lu, range=%lu)\n",
                   (unsigned long)calibration_get_mag_sample_count(),
                   (unsigned long)totalReads,
                   (unsigned long)readFailCount,
                   (unsigned long)rejectClose,
                   (unsigned long)rejectRange);
            return false;
        }
        if (!stdio_usb_connected()) {
            printf("\nUSB disconnected — aborting.\n");
            return false;
        }
        watchdog_update();

        // Read mag sample from seqlock
        float mx = 0.0F;
        float my = 0.0F;
        float mz = 0.0F;
        if (!rc_os_read_mag(&mx, &my, &mz)) {
            readFailCount++;
            sleep_ms(kMagCalPollMs);
            uint32_t nowMs = to_ms_since_boot(get_absolute_time());
            if (totalReads == 0 && readFailCount > 0 &&
                (nowMs - lastDiagMs) >= kMagCalDiagIntervalMs) {
                lastDiagMs = nowMs;
                printf("  [waiting for mag data... %lu reads failed]\n",
                       (unsigned long)readFailCount);
                (void)fflush(stdout);
            }
            continue;
        }
        totalReads++;

        mag_feed_result_t result = calibration_feed_mag_sample(mx, my, mz);
        uint16_t count = calibration_get_mag_sample_count();

        mag_cal_track_reject(result, mx, my, mz, &rejectClose, &rejectRange);
        mag_cal_print_progress(result, count, mx, my, mz,
                               totalReads, rejectClose, rejectRange,
                               &lastPrintedCount, &lastDiagMs);

        if (result == mag_feed_result_t::BUFFER_FULL) {
            break;
        }
        sleep_ms(kMagCalPollMs);
    }

    *outFinalCount = calibration_get_mag_sample_count();
    *outFinalCoverage = calibration_get_mag_coverage_pct();
    return true;
}

// Verify calibration with live samples, save to flash, run post-hook.
// Returns true on successful save.
static bool mag_cal_verify_and_save() {
    // Verification: 10 live samples
    printf("\nVerification (%d live samples):\n", kVerifySampleCount);
    float magSum = 0.0F;
    uint8_t goodReads = 0;
    for (uint8_t i = 0; i < kVerifySampleCount; i++) {
        float mx = 0.0F;
        float my = 0.0F;
        float mz = 0.0F;
        if (rc_os_read_mag(&mx, &my, &mz)) {
            float cx = 0.0F;
            float cy = 0.0F;
            float cz = 0.0F;
            calibration_apply_mag(mx, my, mz, &cx, &cy, &cz);
            float mag = sqrtf(cx*cx + cy*cy + cz*cz);
            magSum += mag;
            goodReads++;
        }
        sleep_ms(kMagCalPollMs);
    }
    const calibration_store_t* cal = calibration_manager_get();
    if (goodReads > 0) {
        float avgMag = magSum / static_cast<float>(goodReads);
        printf("  Avg calibrated magnitude: %.2f uT (expected %.2f)\n",
               (double)avgMag, (double)cal->mag.expected_radius);
    } else {
        printf("  WARNING: No valid mag reads during verification\n");
    }

    // Auto-save to flash
    printf("\nSaving to flash...");
    (void)fflush(stdout);
    cal_result_t saveResult = calibration_save();
    if (saveResult == CAL_RESULT_OK) {
        printf(" OK!\n");
        if (!i2c_bus_reset()) {
            printf("[WARN] I2C bus reset failed after save\n");
        }
    } else {
        printf(" FAILED (%d)\n", saveResult);
    }

    // Signal Core 1 to reload calibration
    if (rc_os_cal_post_hook != nullptr) { rc_os_cal_post_hook(); }

    printf("\nCompass calibration complete.\n");
    return saveResult == CAL_RESULT_OK;
}

// Print ellipsoid fit results (offset, scale, offdiag, radius, fitness).
static void mag_cal_print_results() {
    const calibration_store_t* cal = calibration_manager_get();
    printf("Results:\n");
    printf("  Offset: X=%.2f Y=%.2f Z=%.2f uT\n",
           (double)cal->mag.offset.x,
           (double)cal->mag.offset.y,
           (double)cal->mag.offset.z);
    printf("  Scale:  X=%.4f Y=%.4f Z=%.4f\n",
           (double)cal->mag.scale.x,
           (double)cal->mag.scale.y,
           (double)cal->mag.scale.z);
    printf("  Offdiag: XY=%.4f XZ=%.4f YZ=%.4f\n",
           (double)cal->mag.offdiag.x,
           (double)cal->mag.offdiag.y,
           (double)cal->mag.offdiag.z);
    printf("  Expected radius: %.2f uT\n",
           (double)cal->mag.expected_radius);
    printf("  Fitness (RMS): %.3f uT\n",
           (double)calibration_get_mag_fitness());
}

// Inner body of compass calibration. Returns true on success.
// Shared by cmd_mag_cal() and cmd_wizard().
static bool mag_cal_inner() {
    calibration_reset_mag_cal();
    if (rc_os_reset_mag_staleness != nullptr) {
        rc_os_reset_mag_staleness();
    }
    cal_neo(kCalNeoMag);

    // Phase 1: Collect samples
    uint16_t finalCount = 0;
    uint8_t finalCoverage = 0;
    if (!mag_cal_collect_samples(&finalCount, &finalCoverage)) {
        calibration_reset_mag_cal();
        cal_neo(kCalNeoOff);
        return false;
    }

    printf("\nCollection complete: %u samples, %u%% coverage\n",
           finalCount, finalCoverage);

    // Phase 2: Validate sample count
    if (finalCount < kMagCalMinSamples) {
        printf("ERROR: Not enough samples (%u < %u)\n",
               finalCount, kMagCalMinSamples);
        calibration_reset_mag_cal();
        cal_neo_flash_result(false);
        return false;
    }

    // Phase 3: Compute ellipsoid fit
    printf("Computing ellipsoid fit...");
    (void)fflush(stdout);

    cal_result_t fitResult = calibration_compute_mag_cal();
    if (fitResult != CAL_RESULT_OK) {
        printf(" FAILED (%d)\n", fitResult);
        if (fitResult == CAL_RESULT_FIT_FAILED) {
            printf("Ellipsoid fit did not converge or params out of range.\n");
        }
        calibration_reset_mag_cal();
        cal_neo_flash_result(false);
        return false;
    }
    printf(" OK!\n\n");

    // Phase 4: Print results
    mag_cal_print_results();

    // Phase 5: Verify with live samples, save to flash
    mag_cal_verify_and_save();

    calibration_reset_mag_cal();
    cal_neo_flash_result(true);
    return true;
}

static void cmd_mag_cal() {
    if (rc_os_read_mag == nullptr) {
        printf("\nERROR: Mag read callback not set\n");
        return;
    }

    printf("\n========================================\n");
    printf("  Compass Calibration\n");
    printf("========================================\n");
    printf("Rotate the device slowly through all\n");
    printf("orientations to cover the full sphere.\n");
    printf("Press ENTER to start, 'x' to cancel.\n");
    printf("========================================\n\n");
    cal_neo(kCalNeoMag);

    if (!wait_for_enter_or_esc()) {
        cal_neo(kCalNeoOff);
        return;
    }

    rc_os_mag_cal_active = true;   // Pause GPS on Core 1
    mag_cal_inner();
    rc_os_mag_cal_active = false;  // Resume GPS
    cal_neo(kCalNeoOff);
}

// ============================================================================
// Wizard: Async calibration wait helper
// ============================================================================
// Blocks until the active async calibration (gyro/level/baro) completes.
// Feeds sensor samples via callback, kicks watchdog, checks ESC/USB.
// Returns true on successful completion, false on cancel/failure/disconnect.

static bool wait_for_async_cal() {
    while (calibration_is_active()) {
        // Check for cancel (x or ESC)
        int ch = getchar_timeout_us(0);
        if (ch == kEscChar || ch == 'x' || ch == 'X') {
            calibration_cancel();
            printf("\nCancelled.\n");
            return false;
        }

        // USB disconnect check
        if (!stdio_usb_connected()) {
            calibration_cancel();
            return false;
        }

        // Feed sensor samples to the active calibration
        if (rc_os_feed_cal != nullptr) {
            rc_os_feed_cal();
        }

        update_calibration_progress();
        watchdog_update();
        sleep_ms(kCalFeedPollMs);
    }

    // Check result
    cal_state_t state = calibration_manager_get_state();
    update_calibration_progress();  // Print final OK/FAIL
    calibration_reset_state();

    return (state == CAL_STATE_COMPLETE);
}

// ============================================================================
// Unified Calibration Wizard
// ============================================================================

// Run an async calibration step (gyro or level).
// Returns: 0=passed, 1=failed, 2=skipped
static uint8_t wizard_async_step(const char* stepLabel,
                                  const char* instructions,
                                  uint8_t calNeo,
                                  cal_result_t (*startFn)()) {
    printf("\n--- %s ---\n", stepLabel);
    if (!rc_os_imu_available) {
        printf("  SKIPPED (IMU not available)\n");
        return 2;
    }
    printf("%s\n", instructions);
    printf("ENTER to start, 'x' to skip.\n");
    cal_neo(calNeo);
    if (!wait_for_enter_or_esc()) {
        printf("  SKIPPED by user\n");
        cal_neo(kCalNeoOff);
        return 2;
    }
    cal_result_t result = startFn();
    if (result != CAL_RESULT_OK) {
        printf("ERROR: Failed to start cal (%d)\n", result);
        cal_neo_flash_result(false);
        return 1;
    }
    printf("Sampling");
    (void)fflush(stdout);
    if (wait_for_async_cal()) {
        calibration_save();
        i2c_bus_reset();
        cal_neo_flash_result(true);
        return 0;
    }
    cal_neo_flash_result(false);
    return 1;
}

// Run 6-position accel calibration step with pre/post hooks.
// Returns: 0=passed, 1=failed, 2=skipped.
static uint8_t wizard_6pos_step() {
    printf("\n--- Step 3/4: 6-Position Accel Calibration ---\n");
    if (!rc_os_imu_available || rc_os_read_accel == nullptr) {
        printf("  SKIPPED (IMU or accel callback not available)\n");
        return 2;
    }
    printf("Calibrates offset, scale, and cross-axis coupling.\n");
    printf("ENTER to start, 'x' to skip.\n");
    cal_neo(kCalNeoAccelWait);
    if (!wait_for_enter_or_esc()) {
        printf("  SKIPPED by user\n");
        cal_neo(kCalNeoOff);
        return 2;
    }
    if (rc_os_cal_pre_hook != nullptr) { rc_os_cal_pre_hook(); }
    cmd_accel_6pos_cal_inner();
    cal_neo(kCalNeoOff);
    if (rc_os_cal_post_hook != nullptr) { rc_os_cal_post_hook(); }
    const calibration_store_t* calCheck = calibration_manager_get();
    return ((calCheck->cal_flags & CAL_STATUS_ACCEL_6POS) != 0) ? 0 : 1;
}

// Run compass calibration step with GPS pause.
// Returns: 0=passed, 1=failed, 2=skipped.
static uint8_t wizard_compass_step() {
    printf("\n--- Step 4/4: Compass Calibration ---\n");
    if (rc_os_read_mag == nullptr) {
        printf("  SKIPPED (mag read callback not set)\n");
        return 2;
    }
    printf("Rotate device through all orientations.\n");
    printf("ENTER to start, 'x' to skip.\n");
    cal_neo(kCalNeoMag);
    if (!wait_for_enter_or_esc()) {
        printf("  SKIPPED by user\n");
        cal_neo(kCalNeoOff);
        return 2;
    }
    rc_os_mag_cal_active = true;
    uint8_t result = mag_cal_inner() ? 0 : 1;
    rc_os_mag_cal_active = false;
    return result;
}

static void wizard_print_summary(uint8_t passed, uint8_t failed, uint8_t skipped) {
    const calibration_store_t* calFinal = calibration_manager_get();
    printf("\n========================================\n");
    printf("  Wizard Complete\n");
    printf("========================================\n");
    printf("  Passed:  %d\n", passed);
    printf("  Failed:  %d\n", failed);
    printf("  Skipped: %d\n", skipped);
    printf("\nCalibration flags: 0x%02lX\n", (unsigned long)calFinal->cal_flags);
    printf("  Gyro:  %s\n", (calFinal->cal_flags & CAL_STATUS_GYRO) != 0 ? "OK" : "--");
    printf("  Level: %s\n", (calFinal->cal_flags & CAL_STATUS_LEVEL) != 0 ? "OK" : "--");
    printf("  Baro:  %s\n", (calFinal->cal_flags & CAL_STATUS_BARO) != 0 ? "OK" : "--");
    printf("  Accel: %s\n", (calFinal->cal_flags & CAL_STATUS_ACCEL_6POS) != 0 ? "6POS" : "--");
    printf("  Mag:   %s\n", (calFinal->cal_flags & CAL_STATUS_MAG) != 0 ? "OK" : "--");
    printf("========================================\n\n");
}

static void cmd_wizard() {
    printf("\n========================================\n");
    printf("  Full Calibration Wizard\n");
    printf("========================================\n");
    printf("This wizard runs all calibrations:\n");
    printf("  1. Gyro (keep still, ~2s)\n");
    printf("  2. Level (keep flat, ~1s)\n");
    printf("  3. 6-position accel\n");
    printf("  4. Compass calibration\n");
    printf("(Baro ground ref runs automatically at boot)\n");
    printf("Press ENTER to begin, 'x' to cancel.\n");
    printf("========================================\n\n");

    if (!wait_for_enter_or_esc()) {
        return;
    }

    uint8_t passed = 0;
    uint8_t failed = 0;
    uint8_t skipped = 0;

    // --- Step 1: Gyro ---
    { uint8_t r = wizard_async_step("Step 1/4: Gyro Calibration",
                                     "Keep device STILL for ~2s.",
                                     kCalNeoGyro, calibration_start_gyro);
      if (r == 0) { passed++; } else if (r == 1) { failed++; } else { skipped++; }
    }

    // --- Step 2: Level ---
    { uint8_t r = wizard_async_step("Step 2/4: Level Calibration",
                                     "Keep device FLAT and STILL for ~1s.",
                                     kCalNeoLevel, calibration_start_accel_level);
      if (r == 0) { passed++; } else if (r == 1) { failed++; } else { skipped++; }
    }

    // --- Step 3: 6-Position Accel ---
    { uint8_t r = wizard_6pos_step();
      if (r == 0) { passed++; } else if (r == 1) { failed++; } else { skipped++; }
    }

    // --- Step 4: Compass ---
    { uint8_t r = wizard_compass_step();
      if (r == 0) { passed++; } else if (r == 1) { failed++; } else { skipped++; }
    }

    cal_neo(kCalNeoOff);
    wizard_print_summary(passed, failed, skipped);
}

// ============================================================================
// Calibration Progress Monitoring
// ============================================================================

static void update_calibration_progress() {
    static uint8_t g_lastProgress = 0;
    static bool g_wasActive = false;

    cal_state_t state = calibration_manager_get_state();
    uint8_t progress = calibration_get_progress();
    bool isActive = calibration_is_active();

    // Print dots for progress (every 10%)
    if (isActive && progress / kProgressPercentStep > g_lastProgress / kProgressPercentStep) {
        printf(".");
        (void)fflush(stdout);
    }
    g_lastProgress = progress;

    // Detect completion: was active, now complete or failed
    if (g_wasActive && !isActive) {
        if (state == CAL_STATE_COMPLETE) {
            printf(" OK!\n");
            cal_neo_flash_result(true);
        } else if (state == CAL_STATE_FAILED) {
            cal_result_t result = calibration_get_result();
            if (result == CAL_RESULT_MOTION_DETECTED) {
                printf(" FAILED - motion detected!\n");
            } else {
                printf(" FAILED (%d)\n", result);
            }
            cal_neo_flash_result(false);
        }
        calibration_reset_state();
        g_lastProgress = 0;
    }

    g_wasActive = isActive;
}

// ============================================================================
// Main Menu Handler
// ============================================================================

// NOLINTNEXTLINE(readability-function-size) — pure key dispatcher, splitting adds indirection
static void handle_main_menu(int c) {
    switch (c) {
        case 'h':
        case 'H':
        case '?':
            print_system_status();
            break;

        case 's':
        case 'S':
            if (rc_os_print_sensor_status != nullptr) {
                rc_os_print_sensor_status();
            } else {
                printf("Sensor status not available.\n");
            }
            break;

        case 'i':
        case 'I':
            if (rc_os_i2c_scan_allowed) {
                printf("\nRescanning I2C bus...\n");
                i2c_bus_scan();
            } else {
                printf("\nI2C scan disabled (Core 1 owns bus)\n");
            }
            break;

        case 'b':
        case 'B':
            if (rc_os_print_boot_summary != nullptr) {
                rc_os_print_boot_summary();
            } else {
                printf("Boot summary not available.\n");
            }
            break;

        case 'c':
        case 'C':
            g_menu = RC_OS_MENU_CALIBRATION;
            print_calibration_menu();
            break;

        case 'e':
        case 'E':
            if (rc_os_print_eskf_live != nullptr) {
                g_eskfLiveActive = true;
                g_eskfLiveLastPrintUs = time_us_32();
                printf("\n--- ESKF live (1Hz) --- any key to stop ---\n");
                rc_os_print_eskf_live();
            } else {
                printf("ESKF not available.\n");
            }
            break;

        case '\r':
        case '\n':
            // Ignore line endings
            break;

        default:
            if (rc_os_on_unhandled_key != nullptr) {
                rc_os_on_unhandled_key(c);
            }
            break;
    }
}

// ============================================================================
// Calibration Menu Handler
// ============================================================================

// NOLINTNEXTLINE(readability-function-size) — pure key dispatcher, splitting adds indirection
static void handle_calibration_menu(int c) {
    switch (c) {
        case 'g':
        case 'G':
            cmd_gyro_cal();
            break;

        case 'l':
        case 'L':
            cmd_level_cal();
            break;

        case 'b':
        case 'B':
            cmd_baro_cal();
            break;

        case 'a':
        case 'A':
            cmd_accel_6pos_cal();
            break;

        case 'm':
        case 'M':
            cmd_mag_cal();
            break;

        case 'w':
        case 'W':
            cmd_wizard();
            break;

        case 'v':
        case 'V':
            cmd_save_cal();
            break;

        case 'r':
        case 'R':
            cmd_reset_cal();
            break;

        case 'x':
        case 'X':
        case kEscChar:
            if (calibration_is_active()) {
                calibration_cancel();
                printf("\nCalibration cancelled.\n");
            }
            printf("Returning to main menu.\n");
            g_menu = RC_OS_MENU_MAIN;
            break;

        case 'h':
        case 'H':
        case '?':
            print_calibration_menu();
            break;

        case '\r':
        case '\n':
        default:
            break;
    }
}

// ============================================================================
// Public API
// ============================================================================

void rc_os_init() {
    g_menu = RC_OS_MENU_MAIN;
    g_wasConnected = false;
    g_bannerPrinted = false;
}

// NOLINTNEXTLINE(readability-function-size) — USB state machine, splitting breaks state tracking
bool rc_os_update() {
    bool isConnected = stdio_usb_connected();

    // Not connected - do nothing (per LL Entry 15)
    if (!isConnected) {
        g_wasConnected = false;
        g_bannerPrinted = false;
        return false;
    }

    // Just connected - print boot status + banner
    if (!g_wasConnected) {
        g_wasConnected = true;

        // Brief settle time
        sleep_ms(kUsbSettleMs);

        // Drain any garbage from input buffer (per LL Entry 15)
        while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {
            // Discard
        }

        // Print full boot status on first-ever connect (non-blocking USB:
        // boot output deferred until terminal connects)
        if (!g_bannerPrinted && rc_os_print_boot_status != nullptr) {
            rc_os_print_boot_status();
        }

        // Print CLI banner
        print_banner();
        g_bannerPrinted = true;

        // Show initial status
        print_system_status();

        g_menu = RC_OS_MENU_MAIN;
    }

    // Update calibration progress display
    update_calibration_progress();

    // Live ESKF mode: periodic print at 1Hz, any key stops
    if (g_eskfLiveActive) {
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            g_eskfLiveActive = false;
            printf("\n--- ESKF live stopped ---\n");
            return true;
        }
        uint32_t nowUs = time_us_32();
        if (nowUs - g_eskfLiveLastPrintUs >= kEskfLivePeriodUs) {
            g_eskfLiveLastPrintUs = nowUs;
            if (rc_os_print_eskf_live != nullptr) {
                rc_os_print_eskf_live();
            }
        }
        return false;
    }

    // Check for input (non-blocking)
    int c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT) {
        return false;
    }

    // Route to appropriate handler
    if (g_menu == RC_OS_MENU_MAIN) {
        handle_main_menu(c);
    } else if (g_menu == RC_OS_MENU_CALIBRATION) {
        handle_calibration_menu(c);
    }

    return true;
}

bool rc_os_is_connected() {
    return stdio_usb_connected();
}

bool rc_os_is_calibrating() {
    return calibration_is_active();
}

rc_os_menu_t rc_os_get_menu() {
    return g_menu;
}
