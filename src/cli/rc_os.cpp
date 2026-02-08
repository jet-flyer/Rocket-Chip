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

#define RC_OS_VERSION           "0.4.0"
#define RC_OS_POLL_MS           50      // 20Hz polling

// ============================================================================
// State
// ============================================================================

static rc_os_menu_t g_menu = RC_OS_MENU_MAIN;
static bool g_wasConnected = false;
static bool g_bannerPrinted = false;
// (no extra state needed — reset confirmation is blocking)

// Callback for sensor status (set by main.cpp)
rc_os_sensor_status_fn rc_os_print_sensor_status = NULL;

// Sensor availability flags (set by main.cpp)
bool rc_os_imu_available = false;
bool rc_os_baro_available = false;
bool rc_os_i2c_scan_allowed = true;

// Accel read callback for 6-pos calibration (set by main.cpp)
rc_os_read_accel_fn rc_os_read_accel = NULL;
rc_os_cal_hook_fn rc_os_cal_pre_hook = NULL;
rc_os_cal_hook_fn rc_os_cal_post_hook = NULL;

// Unhandled key callback (set by main.cpp, for IVP-29/30 test commands)
rc_os_unhandled_key_fn rc_os_on_unhandled_key = NULL;

// ============================================================================
// Menu Printing
// ============================================================================

static void print_banner(void) {
    printf("\n");
    printf("========================================\n");
    printf("  RocketChip OS v%s\n", RC_OS_VERSION);
    printf("  Press 'h' for help\n");
    printf("========================================\n\n");
}

static void print_system_status(void) {
    printf("\n========================================\n");
    printf("  RocketChip System Status\n");
    printf("========================================\n");
    printf("  Version: %s\n", ROCKETCHIP_VERSION_STRING);
    printf("  Board: Adafruit Feather RP2350 HSTX\n");
    printf("  Uptime: %lu ms\n", (unsigned long)to_ms_since_boot(get_absolute_time()));

    // Calibration status
    const calibration_store_t* cal = calibration_manager_get();
    printf("----------------------------------------\n");
    printf("Calibration Status:\n");
    printf("  Gyro:  %s\n", (cal->cal_flags & CAL_STATUS_GYRO) ? "OK" : "NOT DONE");
    if (cal->cal_flags & CAL_STATUS_ACCEL_6POS) {
        printf("  Accel: 6POS\n");
    } else if (cal->cal_flags & CAL_STATUS_LEVEL) {
        printf("  Accel: LEVEL\n");
    } else {
        printf("  Accel: NOT DONE\n");
    }
    printf("  Baro:  %s\n", (cal->cal_flags & CAL_STATUS_BARO) ? "OK" : "NOT DONE");
    printf("  Mag:   %s\n", (cal->cal_flags & CAL_STATUS_MAG) ? "OK" : "NOT DONE");

    printf("----------------------------------------\n");
    printf("Commands:\n");
    printf("  h - This help\n");
    printf("  s - Sensor status\n");
    printf("  i - I2C bus rescan\n");
    printf("  c - Calibration menu\n");
    printf("========================================\n\n");
}

static void print_calibration_menu(void) {
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

static void cmd_gyro_cal(void) {
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

static void cmd_level_cal(void) {
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

static void cmd_baro_cal(void) {
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

static void cmd_save_cal(void) {
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

static void cmd_reset_cal(void) {
    printf("\n*** RESET ALL CALIBRATION ***\n");
    printf("This will erase all calibration data!\n");
    printf("Type 'YES' + ENTER to confirm: ");
    fflush(stdout);

    char confirm[8] = {0};
    int idx = 0;
    bool gotEnter = false;
    while (idx < 7) {
        int ch = getchar_timeout_us(10000000);  // 10 second timeout
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
static const char* const kPositionInstructions[6] = {
    "Place board FLAT on table, component side UP",
    "Stand board on its LEFT edge",
    "Stand board on its RIGHT edge",
    "Stand board on USB connector end (nose down)",
    "Stand board on opposite end from USB (nose up)",
    "Place board FLAT on table, component side DOWN (inverted)"
};

#define ACCEL_6POS_MAX_RETRIES      3
#define ACCEL_6POS_WAIT_TIMEOUT_US  30000000    // 30 seconds
#define ACCEL_6POS_GRAVITY_NOMINAL  9.80665f    // m/s² (WGS-84 standard)
#define ACCEL_6POS_VERIFY_TOLERANCE 0.02f       // m/s² (IVP-17 gate requirement)

// Helper: wait for ENTER key with timeout, ESC cancels. Returns true on ENTER.
static bool wait_for_enter_or_esc(void) {
    while (true) {
        int ch = getchar_timeout_us(ACCEL_6POS_WAIT_TIMEOUT_US);
        if (ch == PICO_ERROR_TIMEOUT) {
            printf("\nTimeout - calibration cancelled.\n");
            return false;
        }
        if (ch == 27) {  // ESC
            printf("\nESC - calibration cancelled.\n");
            return false;
        }
        if (ch == '\r' || ch == '\n') {
            return true;
        }
    }
}

// Inner body of 6-position calibration. Returns on error or completion.
// Caller (cmd_accel_6pos_cal) guarantees pre/post hook execution.
static void cmd_accel_6pos_cal_inner(void) {
    calibration_reset_6pos();

    for (uint8_t pos = 0; pos < 6; pos++) {
        printf("--- Position %d/6: %s ---\n", pos + 1,
               calibration_get_6pos_name(pos));
        printf("  %s\n", kPositionInstructions[pos]);
        printf("  Press ENTER when ready...\n");
        fflush(stdout);

        if (!wait_for_enter_or_esc()) {
            calibration_reset_6pos();
            printf("Calibration aborted.\n");
            return;
        }

        printf("  Sampling... hold still");
        fflush(stdout);

        cal_result_t result = CAL_RESULT_INVALID_DATA;
        for (uint8_t retry = 0; retry < ACCEL_6POS_MAX_RETRIES; retry++) {
            result = calibration_collect_6pos_position(pos, rc_os_read_accel);

            if (result == CAL_RESULT_OK) {
                const float* avg = calibration_get_6pos_avg(pos);
                printf(" OK!\n");
                printf("  Avg: X=%.2f Y=%.2f Z=%.2f\n",
                       (double)avg[0], (double)avg[1], (double)avg[2]);
                break;
            }

            if (result == CAL_RESULT_MOTION_DETECTED) {
                printf("\n  Motion detected! Hold still and try again.\n");
                if (retry < ACCEL_6POS_MAX_RETRIES - 1) {
                    printf("  Press ENTER to retry (%d/%d)...\n",
                           retry + 2, ACCEL_6POS_MAX_RETRIES);
                    fflush(stdout);
                    if (!wait_for_enter_or_esc()) {
                        calibration_reset_6pos();
                        printf("Calibration aborted.\n");
                        return;
                    }
                    printf("  Sampling... hold still");
                    fflush(stdout);
                }
            } else if (result == CAL_RESULT_INVALID_DATA) {
                printf("\n  Orientation doesn't match expected: %s\n",
                       calibration_get_6pos_name(pos));
                printf("  Please reposition the board correctly.\n");
                if (retry < ACCEL_6POS_MAX_RETRIES - 1) {
                    printf("  Press ENTER to retry (%d/%d)...\n",
                           retry + 2, ACCEL_6POS_MAX_RETRIES);
                    fflush(stdout);
                    if (!wait_for_enter_or_esc()) {
                        calibration_reset_6pos();
                        printf("Calibration aborted.\n");
                        return;
                    }
                    printf("  Sampling... hold still");
                    fflush(stdout);
                }
            } else {
                printf("\n  ERROR: Failed to read sensor (%d)\n", result);
                calibration_reset_6pos();
                printf("Calibration aborted.\n");
                return;
            }
        }

        if (result != CAL_RESULT_OK) {
            printf("  Failed after %d attempts.\n", ACCEL_6POS_MAX_RETRIES);
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

    cal_result_t fit_result = calibration_compute_6pos();
    if (fit_result != CAL_RESULT_OK) {
        printf(" FAILED (%d)\n", fit_result);
        printf("Ellipsoid fit did not converge or params out of range.\n");
        calibration_reset_6pos();
        return;
    }
    printf(" OK!\n\n");

    // Print results
    {
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
    }

    // Verification: read a few live samples and show corrected gravity magnitude
    printf("\nVerification (10 live samples):\n");
    {
        float mag_sum = 0.0f;
        uint8_t good_reads = 0;
        for (uint8_t i = 0; i < 10; i++) {
            float ax, ay, az, temp;
            if (rc_os_read_accel(&ax, &ay, &az, &temp)) {
                float cx, cy, cz;
                calibration_apply_accel(ax, ay, az, &cx, &cy, &cz);
                float mag = sqrtf(cx*cx + cy*cy + cz*cz);
                mag_sum += mag;
                good_reads++;
            }
        }
        if (good_reads > 0) {
            float avg_mag = mag_sum / (float)good_reads;
            printf("  Avg gravity magnitude: %.4f m/s^2 (expected 9.8067)\n",
                   (double)avg_mag);
            float error = fabsf(avg_mag - ACCEL_6POS_GRAVITY_NOMINAL);
            if (error < ACCEL_6POS_VERIFY_TOLERANCE) {
                printf("  PASS (error %.4f < %.2f)\n",
                       (double)error, (double)ACCEL_6POS_VERIFY_TOLERANCE);
            } else {
                printf("  WARNING: error %.4f > %.2f threshold\n",
                       (double)error, (double)ACCEL_6POS_VERIFY_TOLERANCE);
            }
        }
    }

    // Auto-save to flash
    printf("\nSaving to flash...");
    fflush(stdout);
    {
        cal_result_t save_result = calibration_save();
        if (save_result == CAL_RESULT_OK) {
            printf(" OK!\n");
            // Flash ops can disrupt I2C bus state — recover so subsequent reads work
            if (!i2c_bus_reset()) {
                printf("[WARN] I2C bus reset failed after save\n");
            }
        } else {
            printf(" FAILED (%d)\n", save_result);
        }
    }

    printf("\n6-position accel calibration complete.\n");
    calibration_reset_6pos();
}

static void cmd_accel_6pos_cal(void) {
    if (!rc_os_imu_available) {
        printf("\nERROR: IMU not available\n");
        return;
    }
    if (rc_os_read_accel == NULL) {
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
    if (rc_os_cal_pre_hook) { rc_os_cal_pre_hook(); }

    cmd_accel_6pos_cal_inner();

    // Re-enable I2C master for normal operation (mag reads)
    if (rc_os_cal_post_hook) { rc_os_cal_post_hook(); }
}

static void cmd_wizard(void) {
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

static void update_calibration_progress(void) {
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
            if (rc_os_print_sensor_status) {
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
            if (rc_os_on_unhandled_key) {
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
        case 27:  // ESC
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

void rc_os_init(void) {
    g_menu = RC_OS_MENU_MAIN;
    g_wasConnected = false;
    g_bannerPrinted = false;
}

bool rc_os_update(void) {
    bool isConnected = stdio_usb_connected();

    // Not connected - do nothing (per LL Entry 15)
    if (!isConnected) {
        g_wasConnected = false;
        g_bannerPrinted = false;
        return false;
    }

    // Just connected - print banner
    if (!g_wasConnected) {
        g_wasConnected = true;

        // Brief settle time
        sleep_ms(200);

        // Drain any garbage from input buffer (per LL Entry 15)
        while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {
            // Discard
        }

        // Print banner
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

bool rc_os_is_connected(void) {
    return stdio_usb_connected();
}

bool rc_os_is_calibrating(void) {
    return calibration_is_active();
}

rc_os_menu_t rc_os_get_menu(void) {
    return g_menu;
}
