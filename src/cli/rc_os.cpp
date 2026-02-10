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

// 6-position calibration
constexpr uint8_t  kAccel6posNumPositions = 6;           // Orthogonal orientations for gravity fit

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

// Accel read callback for 6-pos calibration (set by main.cpp)
rc_os_read_accel_fn rc_os_read_accel = nullptr;
rc_os_cal_hook_fn rc_os_cal_pre_hook = nullptr;
rc_os_cal_hook_fn rc_os_cal_post_hook = nullptr;

// Unhandled key callback (set by main.cpp for extensible key handling)
rc_os_unhandled_key_fn rc_os_on_unhandled_key = nullptr;

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
    printf("  a - 6-position accel (future)\n");
    printf("  m - Compass calibration (future)\n");
    printf("  w - Full wizard (all in sequence)\n");
    printf("  r - Reset all calibration\n");
    printf("  v - Save calibration to flash\n");
    printf("  x - Return to main menu\n");
    printf("========================================\n\n");
}

// ============================================================================
// Calibration Commands
// ============================================================================

static void cmd_gyro_cal() {
    if (!rc_os_imu_available) {
        printf("\nERROR: IMU not available\n");
        return;
    }

    printf("\nGyro Calibration - keep device STILL...\n");

    cal_result_t result = calibration_start_gyro();
    if (result == CAL_RESULT_BUSY) {
        printf("ERROR: Another calibration is in progress\n");
        return;
    }
    if (result != CAL_RESULT_OK) {
        printf("ERROR: Failed to start gyro cal (%d)\n", result);
        return;
    }

    printf("Sampling");
    fflush(stdout);
}

static void cmd_level_cal() {
    if (!rc_os_imu_available) {
        printf("\nERROR: IMU not available\n");
        return;
    }

    printf("\nLevel Calibration - keep device FLAT and STILL...\n");

    cal_result_t result = calibration_start_accel_level();
    if (result == CAL_RESULT_BUSY) {
        printf("ERROR: Another calibration is in progress\n");
        return;
    }
    if (result != CAL_RESULT_OK) {
        printf("ERROR: Failed to start level cal (%d)\n", result);
        return;
    }

    printf("Sampling");
    fflush(stdout);
}

static void cmd_baro_cal() {
    if (!rc_os_baro_available) {
        printf("\nERROR: Barometer not available\n");
        return;
    }

    printf("\nBaro Calibration - setting ground reference...\n");

    cal_result_t result = calibration_start_baro();
    if (result == CAL_RESULT_BUSY) {
        printf("ERROR: Another calibration is in progress\n");
        return;
    }
    if (result != CAL_RESULT_OK) {
        printf("ERROR: Failed to start baro cal (%d)\n", result);
        return;
    }

    printf("Sampling");
    fflush(stdout);
}

static void cmd_save_cal() {
    printf("\nSaving calibration to flash...");
    fflush(stdout);

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
    fflush(stdout);

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
        fflush(stdout);
    }
    printf("\n");

    if (gotEnter && strcmp(confirm, "YES") == 0) {
        printf("Resetting all calibration data...");
        fflush(stdout);
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
    while (true) {
        int ch = getchar_timeout_us(kAccel6posWaitTimeoutUs);
        if (ch == PICO_ERROR_TIMEOUT) {
            printf("\nTimeout - calibration cancelled.\n");
            return false;
        }
        if (ch == kEscChar) {
            printf("\nESC - calibration cancelled.\n");
            return false;
        }
        if (ch == '\r' || ch == '\n') {
            return true;
        }
    }
}

// Collect one position with retries. Returns true on success, false on abort/failure.
static bool collect_6pos_position(uint8_t pos) {
    printf("--- Position %d/%d: %s ---\n", pos + 1, kAccel6posNumPositions,
           calibration_get_6pos_name(pos));
    printf("  %s\n", kPositionInstructions[pos]);
    printf("  Press ENTER when ready...\n");
    fflush(stdout);

    if (!wait_for_enter_or_esc()) {
        return false;
    }

    printf("  Sampling... hold still");
    fflush(stdout);

    for (uint8_t retry = 0; retry < kAccel6posMaxRetries; retry++) {
        cal_result_t result = calibration_collect_6pos_position(pos, rc_os_read_accel);

        if (result == CAL_RESULT_OK) {
            const float* avg = calibration_get_6pos_avg(pos);
            printf(" OK!\n");
            printf("  Avg: X=%.2f Y=%.2f Z=%.2f\n",
                   (double)avg[0], (double)avg[1], (double)avg[2]);
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
            return false;
        }

        printf("\n  %s\n", errMsg);
        if (retry < kAccel6posMaxRetries - 1) {
            printf("  Press ENTER to retry (%d/%d)...\n",
                   retry + 2, kAccel6posMaxRetries);
            fflush(stdout);
            if (!wait_for_enter_or_esc()) {
                return false;
            }
            printf("  Sampling... hold still");
            fflush(stdout);
        }
    }

    printf("  Failed after %d attempts.\n", kAccel6posMaxRetries);
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
    fflush(stdout);

    cal_result_t fitResult = calibration_compute_6pos();
    if (fitResult != CAL_RESULT_OK) {
        printf(" FAILED (%d)\n", fitResult);
        printf("Ellipsoid fit did not converge or params out of range.\n");
        calibration_reset_6pos();
        return;
    }
    printf(" OK!\n\n");

    print_6pos_results();

    // Auto-save to flash
    printf("\nSaving to flash...");
    fflush(stdout);
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
    printf("Press ESC at any time to cancel.\n");
    printf("========================================\n\n");

    // Disable I2C master before rapid accel reads (prevents bank-switching race)
    if (rc_os_cal_pre_hook != nullptr) { rc_os_cal_pre_hook(); }

    cmd_accel_6pos_cal_inner();

    // Re-enable I2C master for normal operation (mag reads)
    if (rc_os_cal_post_hook != nullptr) { rc_os_cal_post_hook(); }
}

static void cmd_wizard() {
    // Wizard requires blocking waits which don't work with bare-metal polling.
    // Use individual calibrations instead (g, l, b) then save (v).
    printf("\n=== Calibration Wizard ===\n");
    printf("Wizard not available in bare-metal mode.\n");
    printf("Use individual calibrations instead:\n");
    printf("  1. Press 'g' - Gyro calibration (keep still, ~2s)\n");
    printf("  2. Press 'l' - Level calibration (keep flat, ~1s)\n");
    printf("  3. Press 'b' - Baro calibration (ground ref, ~1s)\n");
    printf("  4. Press 'v' - Save to flash\n");
    printf("Each calibration runs automatically once started.\n");
    printf("Press 'x' during calibration to cancel.\n\n");
}

// ============================================================================
// Calibration Progress Monitoring
// ============================================================================

static void update_calibration_progress() {
    static uint8_t lastProgress = 0;
    static bool wasActive = false;

    cal_state_t state = calibration_manager_get_state();
    uint8_t progress = calibration_get_progress();
    bool isActive = calibration_is_active();

    // Print dots for progress (every 10%)
    if (isActive && progress / 10 > lastProgress / 10) {
        printf(".");
        fflush(stdout);
    }
    lastProgress = progress;

    // Detect completion: was active, now complete or failed
    if (wasActive && !isActive) {
        if (state == CAL_STATE_COMPLETE) {
            printf(" OK!\n");
        } else if (state == CAL_STATE_FAILED) {
            cal_result_t result = calibration_get_result();
            if (result == CAL_RESULT_MOTION_DETECTED) {
                printf(" FAILED - motion detected!\n");
            } else {
                printf(" FAILED (%d)\n", result);
            }
        }
        calibration_reset_state();
        lastProgress = 0;
    }

    wasActive = isActive;
}

// ============================================================================
// Main Menu Handler
// ============================================================================

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
            printf("\nCompass calibration not yet implemented.\n");
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
            break;

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
