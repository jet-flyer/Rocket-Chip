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

// Callback for sensor status (set by main.cpp)
rc_os_sensor_status_fn rc_os_print_sensor_status = NULL;

// Sensor availability flags (set by main.cpp)
bool rc_os_imu_available = false;
bool rc_os_baro_available = false;

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
    printf("  Level: %s\n", (cal->cal_flags & CAL_STATUS_LEVEL) ? "OK" : "NOT DONE");
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
    } else {
        printf(" FAILED (%d)\n", result);
    }
}

static void cmd_reset_cal(void) {
    printf("\nResetting all calibration data...");
    fflush(stdout);

    cal_result_t result = calibration_reset();
    if (result == CAL_RESULT_OK) {
        printf(" OK!\n");
    } else {
        printf(" FAILED (%d)\n", result);
    }
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
            printf("\nRescanning I2C bus...\n");
            i2c_bus_scan();
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
            printf("\n6-position accel calibration not yet implemented.\n");
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
