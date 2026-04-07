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
#include "active_objects/ao_telemetry.h"
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

// Menu prompt — shows current context after each command
static void print_prompt() {
    switch (g_menu) {
        case RC_OS_MENU_MAIN:        printf("[main] "); break;
        case RC_OS_MENU_CALIBRATION: printf("[cal] "); break;
        case RC_OS_MENU_FLIGHT:      printf("[flight] "); break;
        default:                     printf("> "); break;
    }
}

static void print_help_menu() {
    printf("h-Help  s-Status  e-ESKF  b-Boot Log\n");
    printf("c-Calibration  f-Flight Director\n");
    printf("g-Flights  d-Download  l-Flush  x-Erase\n");
    printf("t-Radio  r-Rate  m-MAVLink  i-I2C\n");
}

static void print_calibration_menu() {
    const calibration_store_t* cal = calibration_manager_get();
    printf("\n========================================\n");
    printf("  Calibration Menu\n");
    printf("----------------------------------------\n");
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

// Calibration UI driven by AO_RCOS cal_ui_tick() state machine (ao_rcos.cpp).
// Calibration save/reset triggered via AO_RCOS_start_cal_save() / AO_RCOS_start_cal_reset().

// ============================================================================
// Main Menu Handler
// ============================================================================

// NOLINTNEXTLINE(readability-function-size) — pure key dispatcher, splitting adds indirection
static bool handle_main_menu(int c) {
    switch (c) {
        case 'h':
        case 'H':
        case '?':
            printf("\n--- Help ---\n");
            print_help_menu();
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
            AO_RCOS_start_cal_save();
            break;

        case 'r':
        case 'R':
            AO_RCOS_start_cal_reset();
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
// Flight Director Menu Handler
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
// MAVLink mode input handler — returns true if byte was consumed
static bool handle_mavlink_input(int c) {
    if (AO_RCOS_get_output_mode() != StationOutputMode::kMavlink) { return false; }
    if (g_menu != RC_OS_MENU_MAIN) { return false; }
    if (c == 'm' || c == 'M') {
        AO_RCOS_set_output_mode(StationOutputMode::kMenu);
        printf("\nMAVLink mode off. CLI active.\n");
        print_prompt();
        return true;
    }
    AO_Telemetry_feed_usb_byte(static_cast<uint8_t>(c));
    return true;
}

// USB connect/disconnect + settle + banner state machine
static uint8_t s_settleCount = 0;

static bool handle_usb_connect() {
    if (!g_wasConnected) {
        s_settleCount = 1;
        g_wasConnected = true;
        return false;
    }
    if (s_settleCount > 0 && s_settleCount < 5) {
        s_settleCount++;
        return false;
    }
    if (s_settleCount == 5) {
        s_settleCount = 0;
        while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {}
        cli_print_boot_summary();
        if (!g_bannerPrinted) {
            printf("\n");
            print_help_menu();
            g_bannerPrinted = true;
        }
        g_menu = RC_OS_MENU_MAIN;
        print_prompt();
    }
    return true;  // Connected and settled
}

// ESKF live mode handler — returns true if in live mode (consumes input)
static bool handle_eskf_live() {
    if (!g_eskfLiveActive) { return false; }
    int c = getchar_timeout_us(0);
    if (c != PICO_ERROR_TIMEOUT) {
        g_eskfLiveActive = false;
        printf("\n--- ESKF live stopped ---\n");
        print_prompt();
    } else {
        uint32_t nowUs = time_us_32();
        if (nowUs - g_eskfLiveLastPrintUs >= kEskfLivePeriodUs) {
            g_eskfLiveLastPrintUs = nowUs;
            cli_print_eskf_live();
        }
    }
    return true;
}

bool rc_os_update() {
    if (!stdio_usb_connected()) {
        g_wasConnected = false;
        g_bannerPrinted = false;
        return false;
    }
    if (!handle_usb_connect()) { return false; }
    if (handle_eskf_live()) { return false; }

    // When a cal/input UI sequence is active, the AO_RCOS cal_ui_tick()
    // handles all key input. Don't read here or we steal characters.
    if (AO_RCOS_cal_active()) {
        return false;
    }

    // Check for input (non-blocking)
    int c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT) {
        return false;
    }

    // MAVLink mode: feed USB bytes to RX parser instead of CLI
    if (handle_mavlink_input(c)) { return true; }

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
