// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
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
#include "flight_director/flight_director.h"
#include "flight_director/command_handler.h"
#include "cli/rc_os_commands.h"
#include "ao_flight_director.h"
#include "ao_rcos.h"
#include <stdio.h>
#include <string.h>

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

constexpr uint32_t kUsbSettleMs          = 200;          // USB CDC settle time after connect

// ============================================================================
// State
// ============================================================================

static rc_os_menu_t g_menu = RC_OS_MENU_MAIN;
static bool g_wasConnected = false;
static bool g_bannerPrinted = false;
// (no extra state needed — reset confirmation is blocking)

// Sensor availability flags (set by main.cpp)
bool rc_os_imu_available = false;
bool rc_os_baro_available = false;
bool rc_os_i2c_scan_allowed = true;
std::atomic<bool> rc_os_mag_cal_active{false};

// Accel read callback for 6-pos calibration (set by main.cpp)
rc_os_read_accel_fn rc_os_read_accel = nullptr;
rc_os_cal_hook_fn rc_os_cal_pre_hook = nullptr;
rc_os_cal_hook_fn rc_os_cal_post_hook = nullptr;

// Mag read callback for compass calibration (set by main.cpp)
rc_os_read_mag_fn rc_os_read_mag = nullptr;

// Mag staleness reset callback (set by main.cpp)
rc_os_reset_mag_staleness_fn rc_os_reset_mag_staleness = nullptr;



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

// Menu prompt — shows current context after each command
static void print_prompt() {
    switch (g_menu) {
        case RC_OS_MENU_MAIN:        printf("[main] "); break;
        case RC_OS_MENU_CALIBRATION: printf("[cal] "); break;
        case RC_OS_MENU_FLIGHT:      printf("[flight] "); break;
        default:                     printf("> "); break;
    }
}

static void print_system_status() {
    printf("\n========================================\n");
    printf("  RocketChip System Status\n");
    printf("========================================\n");
    printf("  Version: %s\n", kVersionString);
    printf("  Board: %s\n", board::kBoardName);
    printf("  Profile: %s\n", rc::kDefaultRocketProfile.name);
    printf("  Uptime: %lu ms\n", (unsigned long)to_ms_since_boot(get_absolute_time()));

    // Calibration status
    const calibration_store_t* cal = calibration_manager_get();
    printf("----------------------------------------\n");
    printf("Calibration:\n");
    printf("  Gyro:  %s\n", (cal->cal_flags & CAL_STATUS_GYRO) != 0 ? "OK" : "--");
    if ((cal->cal_flags & CAL_STATUS_ACCEL_6POS) != 0) {
        printf("  Accel: 6POS\n");
    } else if ((cal->cal_flags & CAL_STATUS_LEVEL) != 0) {
        printf("  Accel: LEVEL\n");
    } else {
        printf("  Accel: --\n");
    }
    printf("  Baro:  %s\n", (cal->cal_flags & CAL_STATUS_BARO) != 0 ? "OK" : "--");
    printf("  Mag:   %s\n", (cal->cal_flags & CAL_STATUS_MAG) != 0 ? "OK" : "--");

    printf("----------------------------------------\n");
    printf("Status:  h-Help  s-Sensor  e-ESKF  b-Boot\n");
    printf("Menus:   c-Calibration  f-Flight Director\n");
    printf("Data:    g-Flights  d-Download  l-Flush  x-Erase\n");
    printf("Radio:   t-Status  r-Rate  m-MAVLink  i-I2C\n");
    printf("========================================\n");
}

static void print_calibration_menu() {
    printf("\n========================================\n");
    printf("  Calibration Menu\n");
    printf("========================================\n");
    printf("  g-Gyro (keep still)   l-Level (keep flat)\n");
    printf("  b-Baro (ground ref)   a-Accel 6-position\n");
    printf("  m-Compass             w-Full wizard\n");
    printf("  r-Reset all           v-Save to flash\n");
    printf("  h-Help                x/ESC-Back\n");
    printf("========================================\n");
}

static void print_flight_menu() {
    printf("\n========================================\n");
    printf("  Flight Director Menu\n");
    printf("========================================\n");
    printf("  Commands:  a-ARM  d-DISARM  x-ABORT  r-RESET\n");
    printf("  Events:    l-LAUNCH  b-BURNOUT  p-APOGEE\n");
    printf("             m-MAIN    n-LANDING\n");
    printf("  Info:      s-Status  h-Help  z/ESC-Back\n");
    printf("========================================\n");
}

// ============================================================================
// Calibration Commands (blocking wizards removed — Phase D4)
// Calibration UI state machine now in AO_RCOS (ao_rcos.cpp).
// Retained: cmd_save_cal (fast sync), cmd_reset_cal (blocking confirm).
// ============================================================================

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

// Blocking calibration functions (cmd_gyro_cal, cmd_level_cal, cmd_baro_cal,
// cmd_accel_6pos_cal, cmd_mag_cal, cmd_wizard, wait_for_async_cal,
// wait_for_enter_or_esc, collect_6pos_position, mag_cal_*, wizard_*,
// update_calibration_progress) removed in Phase D4.
// All calibration UI now driven by AO_RCOS cal_ui_tick() state machine.


// [Phase D4: blocking functions removed — see ao_rcos.cpp cal_ui_tick()]
// Remaining here: only cmd_save_cal() and cmd_reset_cal() above.

// ============================================================================
// Main Menu Handler
// ============================================================================

// NOLINTNEXTLINE(readability-function-size) — pure key dispatcher, splitting adds indirection
static bool handle_main_menu(int c) {
    switch (c) {
        case 'h':
        case 'H':
        case '?':
            print_system_status();
            break;

        case 's':
        case 'S':
            if constexpr (kRadioModeRx) {
                cli_print_station_status();
            } else {
                cli_print_sensor_status();
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
            cli_print_hw_status();
            break;

        case 'c':
        case 'C':
            g_menu = RC_OS_MENU_CALIBRATION;
            print_calibration_menu();
            break;

        case 'e':
            g_eskfLiveActive = true;
            g_eskfLiveLastPrintUs = time_us_32();
            printf("\n--- ESKF live (1Hz) --- any key to stop ---\n");
            cli_print_eskf_live();
            break;

        case 'f':
        case 'F':
            g_menu = RC_OS_MENU_FLIGHT;
            print_flight_menu();
            break;

        default:
            cli_handle_unhandled_key(c);
            break;
    }
    return true;
}

// ============================================================================
// Calibration Menu Handler
// ============================================================================

// NOLINTNEXTLINE(readability-function-size) — pure key dispatcher, splitting adds indirection
static bool handle_calibration_menu(int c) {
    // Block menu keys while a calibration UI sequence is running
    if (AO_RCOS_cal_active() && c != 'x' && c != 'X' && c != kEscChar) {
        return false;
    }

    switch (c) {
        case 'g':
        case 'G':
            AO_RCOS_start_cal_gyro();
            break;

        case 'l':
        case 'L':
            AO_RCOS_start_cal_level();
            break;

        case 'b':
        case 'B':
            AO_RCOS_start_cal_baro();
            break;

        case 'a':
        case 'A':
            AO_RCOS_start_cal_6pos();
            break;

        case 'm':
        case 'M':
            AO_RCOS_start_cal_mag();
            break;

        case 'w':
        case 'W':
            AO_RCOS_start_cal_wizard();
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
            if (AO_RCOS_cal_active()) {
                // Cancel is handled by the cal UI state machine reading ESC
                // Just let it pass through
                break;
            }
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

        default:
            return false;
    }
    return true;
}

// ============================================================================
// Flight Director Menu Handler (IVP-68)
// ============================================================================

static void dispatch_flight_signal(int sig) {
    AO_FlightDirector_dispatch_signal(sig);
}

static void dispatch_flight_command(int cmd) {
    AO_FlightDirector_process_command(cmd);
}

// NOLINTNEXTLINE(readability-function-size) — pure key dispatcher
static bool handle_flight_menu(int c) {
    switch (c) {
        // Commands — routed through command handler (Go/No-Go for ARM)
        case 'a': case 'A': dispatch_flight_command(static_cast<int>(rc::CommandType::kArm)); break;
        case 'd': case 'D': dispatch_flight_command(static_cast<int>(rc::CommandType::kDisarm)); break;
        case 'x': case 'X': dispatch_flight_command(static_cast<int>(rc::CommandType::kAbort)); break;
        case 'r': case 'R': dispatch_flight_command(static_cast<int>(rc::CommandType::kReset)); break;

        // Sensor event injection (bench testing)
        case 'l': case 'L': dispatch_flight_signal(rc::SIG_LAUNCH); break;
        case 'b': case 'B': dispatch_flight_signal(rc::SIG_BURNOUT); break;
        case 'p': case 'P': dispatch_flight_signal(rc::SIG_APOGEE); break;
        case 'm': case 'M': dispatch_flight_signal(rc::SIG_MAIN_DEPLOY); break;
        case 'n': case 'N': dispatch_flight_signal(rc::SIG_LANDING); break;

        // Status
        case 's': case 'S':
            AO_FlightDirector_print_status();
            break;

        // Help
        case 'h': case 'H': case '?':
            print_flight_menu();
            break;

        // Return to main menu
        case 'z': case 'Z': case kEscChar:
            printf("Returning to main menu.\n");
            g_menu = RC_OS_MENU_MAIN;
            break;

        default:
            return false;
    }
    return true;
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
        if (!g_bannerPrinted) {
            cli_print_boot_status();
        }

        // Print CLI banner
        print_banner();
        g_bannerPrinted = true;

        // Show initial status
        g_menu = RC_OS_MENU_MAIN;
        print_system_status();
        print_prompt();
    }

    // Calibration progress now driven by AO_RCOS cal_ui_tick() (Phase D4).

    // Live ESKF mode: periodic print at 1Hz, any key stops
    if (g_eskfLiveActive) {
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            g_eskfLiveActive = false;
            printf("\n--- ESKF live stopped ---\n");
            print_prompt();
            return true;
        }
        uint32_t nowUs = time_us_32();
        if (nowUs - g_eskfLiveLastPrintUs >= kEskfLivePeriodUs) {
            g_eskfLiveLastPrintUs = nowUs;
            cli_print_eskf_live();
        }
        return false;
    }

    // Check for input (non-blocking)
    int c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT) {
        return false;
    }

    // Route to appropriate handler
    bool handled = false;
    if (g_menu == RC_OS_MENU_MAIN) {
        handled = handle_main_menu(c);
    } else if (g_menu == RC_OS_MENU_CALIBRATION) {
        handled = handle_calibration_menu(c);
    } else if (g_menu == RC_OS_MENU_FLIGHT) {
        handled = handle_flight_menu(c);
    }

    // Show context prompt only after recognized commands
    if (handled && !g_eskfLiveActive) {
        print_prompt();
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
