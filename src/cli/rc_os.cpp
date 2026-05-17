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
#include "rc_os_dashboard.h"
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
#include "ao_notify.h"              // Stage L — AO_Notify_post_prearm_fail
#include "ao_rcos.h"
#include "cli/rc_os_debug.h"   // R-25-exec step 2: was dev/dev_cli.h
#include "rocketchip/rc_log.h"
#include <string.h>

// ============================================================================
// Configuration
// ============================================================================

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
// R-17/R-18 (2026-05-07 audit): the rc_os_cal_pre_hook /
// rc_os_cal_post_hook function-pointer table was removed — both pointers
// were assigned at main.cpp:315 but never invoked anywhere. The actual
// cal_post_hook (Core 1 calibration-reload signal) is now called
// directly from ao_rcos.cpp; the I2C-pause mechanism the pre-hook used
// to wrap is now in src/safety/core1_i2c_pause.{h,cpp} (R-17).

// Mag read callback for compass calibration (set by main.cpp)
rc_os_read_mag_fn rc_os_read_mag = nullptr;

// Mag staleness reset callback (set by main.cpp)
rc_os_reset_mag_staleness_fn rc_os_reset_mag_staleness = nullptr;



// ESKF live mode state moved to src/dev/dev_cli.cpp (bench-only)

// ============================================================================
// Menu Printing
// ============================================================================

// Menu prompt — shows current context after each command
static void print_prompt() {
    switch (g_menu) {
        case RC_OS_MENU_MAIN:        rc::rc_log("[main] "); break;
        case RC_OS_MENU_CALIBRATION: rc::rc_log("[cal] "); break;
        case RC_OS_MENU_FLIGHT:      rc::rc_log("[flight] "); break;
        case RC_OS_MENU_DEBUG:       rc::rc_log("[debug] "); break;
        default:                     rc::rc_log("> "); break;
    }
}

static void print_help_menu() {
    rc::rc_log("h-Help  p-Preflight  c-Calibration  f-Flight\n");
    rc::rc_log("g-Flights  d-Download  l-Flush  x-Erase\n");
    rc::rc_log("t-Radio  r-Rate  m-MAVLink  b-Beacon  q-Debug\n");
}

static void print_calibration_menu() {
    const calibration_store_t* cal = calibration_manager_get();
    rc::rc_log("\n========================================\n");
    rc::rc_log("  Calibration Menu\n");
    rc::rc_log("----------------------------------------\n");
    rc::rc_log("  Gyro:  %s\n", (cal->cal_flags & CAL_STATUS_GYRO) != 0 ? "OK" : "--");
    if ((cal->cal_flags & CAL_STATUS_ACCEL_6POS) != 0) {
        rc::rc_log("  Accel: 6POS\n");
    } else if ((cal->cal_flags & CAL_STATUS_LEVEL) != 0) {
        rc::rc_log("  Accel: LEVEL\n");
    } else {
        rc::rc_log("  Accel: --\n");
    }
    rc::rc_log("  Baro:  %s\n", (cal->cal_flags & CAL_STATUS_BARO) != 0 ? "OK" : "--");
    rc::rc_log("  Mag:   %s\n", (cal->cal_flags & CAL_STATUS_MAG) != 0 ? "OK" : "--");
    rc::rc_log("----------------------------------------\n");
    rc::rc_log("  g-Gyro (keep still)   l-Level (keep flat)\n");
    rc::rc_log("  b-Baro (ground ref)   a-Accel 6-position\n");
    rc::rc_log("  m-Compass             w-Full wizard\n");
    rc::rc_log("  r-Reset all           v-Save to flash\n");
    rc::rc_log("  h-Help                x/ESC-Back\n");
    rc::rc_log("========================================\n");
}

static void print_flight_menu() {
    rc::rc_log("\n========================================\n");
    rc::rc_log("  Flight Director Menu\n");
    rc::rc_log("========================================\n");
    rc::rc_log("  Commands:  a-ARM  d-DISARM  x-ABORT  r-RESET\n");
    rc::rc_log("  Events:    l-LAUNCH  b-BURNOUT  p-APOGEE\n");
    rc::rc_log("             m-MAIN    n-LANDING\n");
    rc::rc_log("  Info:      s-Status  h-Help  z/ESC-Back\n");
    rc::rc_log("========================================\n");
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
            rc::rc_log("\n--- Help ---\n");
            print_help_menu();
            break;

        case 'c':
        case 'C':
            g_menu = RC_OS_MENU_CALIBRATION;
            print_calibration_menu();
            break;

        case 'f':
        case 'F':
            g_menu = RC_OS_MENU_FLIGHT;
            print_flight_menu();
            break;

        case 'p':
        case 'P':
            // IVP-/matrix Tier 5: station-flight host-script parity — preflight poll
            // is meaningful on RX (radio HW, MCU temp, RF Link, VERDICT) even when IMU/BARO/GPS primary
            // stations show ABSENT. Previously `if constexpr (!kRadioModeRx)` made 'p' a no-op on station.
            cli_print_preflight();
            break;

        case 'q':
        case 'Q':
            if (dev_debug_menu_enter()) {
                g_menu = RC_OS_MENU_DEBUG;
            }
            break;

        case 'b':
        case 'B':
            // Stage L — manual beacon (CLI find-me). Publishes SIG_BEACON_MANUAL
            // directly. AO_Notify sets NotifyState.beacon_manual; resolver forces
            // pure-white 2Hz regardless of state. Clears on SIG_PHASE_CHANGE out
            // of LANDED/ABORT.
            cmd_findme_beacon();
            break;

        default:
            cli_handle_unhandled_key(c);
            break;
    }
    return true;
}

// Debug Menu Handler — moved to src/dev/dev_cli.cpp (bench-only)

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
                rc::rc_log("\nCalibration cancelled.\n");
            }
            rc::rc_log("Returning to main menu.\n");
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
    bool accepted = AO_FlightDirector_process_command(cmd);
    // Stage L IVP-L3: on ARM rejection, post kPreArmFail so the LED shows
    // yellow double-flash for ~3s. Each repost refreshes the counter; a
    // successful arm (or any phase change) clears it via handle_phase_change.
    if (!accepted && cmd == static_cast<int>(rc::CommandType::kArm)) {
        AO_Notify_post_prearm_fail();
    }
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
            rc::rc_log("Returning to main menu.\n");
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
// MAVLink binary lockout: once 0xFD seen on USB, suppress all CLI routing.
// QGC sends binary frames containing arbitrary bytes (0x4C='L' triggers
// flash erase, crashing AO scheduler). ESC exits lockout.
static bool s_mavlinkDetected = false;

// IVP-122: ARM confirm state machine (file-scope for rc_os_start_arm_confirm)
static constexpr uint16_t kMavCmdArmDisarm = 400;
static bool s_arm_confirm_active = false;
static char s_arm_buf[4] = {};
static uint8_t s_arm_buf_pos = 0;
static uint32_t s_arm_start_ms = 0;

bool rc_os_arm_confirm_active() {
    return s_arm_confirm_active;
}

void rc_os_start_arm_confirm() {
#ifndef ROCKETCHIP_HOST_TEST
    s_arm_confirm_active = true;
    s_arm_buf_pos = 0;
    s_arm_start_ms = to_ms_since_boot(get_absolute_time());
    rc_os_dashboard_pause();
    rc::rc_log("Type ARM in caps then Enter to confirm (5s): ");
#endif
}

static bool handle_mavlink_lockout(int c) {
    if (static_cast<uint8_t>(c) == 0xFD) { s_mavlinkDetected = true; }

    if (s_mavlinkDetected ||
        AO_Telemetry_is_gcs_connected() ||
        AO_RCOS_get_output_mode() == StationOutputMode::kMavlink) {
        if (c == 27) {
            s_mavlinkDetected = false;
            AO_RCOS_set_output_mode(StationOutputMode::kMenu);
            rc::rc_log("\n[GCS mode exited]\n");
            print_prompt();
        }
        return true;
    }
    return false;
}

// MAVLink mode input handler — returns true if byte was consumed
static bool handle_mavlink_input(int c) {
    if (AO_RCOS_get_output_mode() != StationOutputMode::kMavlink) { return false; }
    if (g_menu != RC_OS_MENU_MAIN) { return false; }
    if (c == 'm' || c == 'M') {
        AO_RCOS_set_output_mode(StationOutputMode::kMenu);
        rc::rc_log("\nMAVLink mode off. CLI active.\n");
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
            rc::rc_log("\n");
            print_help_menu();
            g_bannerPrinted = true;
        }
        g_menu = RC_OS_MENU_MAIN;
        print_prompt();
    }
    return true;  // Connected and settled
}

// ESKF live mode — moved to src/dev/dev_cli.cpp (bench-only)

// IVP-122: ARM confirm state machine — returns -1 if not active,
// 0 if active but no input, 1 if active and consumed input.
static int handle_arm_confirm() {
    if (!s_arm_confirm_active) { return -1; }

    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - s_arm_start_ms > 5000) {
        rc::rc_log("ARM aborted (timeout)\n");
        s_arm_confirm_active = false;
        rc_os_dashboard_resume();
        print_prompt();
        return 1;
    }
    int ac = getchar_timeout_us(0);
    if (ac == PICO_ERROR_TIMEOUT) { return 0; }
    if (ac == '\r' || ac == '\n') {
        s_arm_buf[s_arm_buf_pos] = '\0';
        if (s_arm_buf_pos == 3 &&
            s_arm_buf[0] == 'A' && s_arm_buf[1] == 'R' && s_arm_buf[2] == 'M') {
            AO_Telemetry_send_tracked_command(kMavCmdArmDisarm, 1.0f);
            rc::rc_log("[CMD] ARM sent, waiting for ACK...\n");
        } else {
            rc::rc_log("ARM aborted (bad input: '%s')\n", s_arm_buf);
        }
        s_arm_confirm_active = false;
        rc_os_dashboard_resume();
        print_prompt();
        return 1;
    }
    if (s_arm_buf_pos < 3) {
        rc::rc_log("%c", ac);
        s_arm_buf[s_arm_buf_pos++] = static_cast<char>(ac);
    } else {
        rc::rc_log("ARM aborted (overflow)\n");
        s_arm_confirm_active = false;
        rc_os_dashboard_resume();
        print_prompt();
    }
    return 1;
}

bool rc_os_update() {
    if (!stdio_usb_connected()) {
        g_wasConnected = false;
        g_bannerPrinted = false;
        return false;
    }
    if (!handle_usb_connect()) { return false; }
    if (dev_eskf_live_poll()) { return false; }
    // R-25-exec steps 5+6: both replay polls removed (replay went host-side).

    int arm_result = handle_arm_confirm();
    if (arm_result == 0) { return false; }
    if (arm_result == 1) { return true; }

    if (AO_RCOS_cal_active()) { return false; }

    int c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT) { return false; }

    AO_Telemetry_feed_usb_byte(static_cast<uint8_t>(c));

    if (handle_mavlink_lockout(c)) { return true; }
    if (handle_mavlink_input(c)) { return true; }

    bool handled = false;
    if (g_menu == RC_OS_MENU_MAIN) {
        handled = handle_main_menu(c);
    } else if (g_menu == RC_OS_MENU_CALIBRATION) {
        handled = handle_calibration_menu(c);
    } else if (g_menu == RC_OS_MENU_FLIGHT) {
        handled = handle_flight_menu(c);
    } else if (g_menu == RC_OS_MENU_DEBUG) {
        // Stage L IVP-L1: if the LED-test submenu is waiting for a pick,
        // route this keystroke there instead of through the normal debug
        // dispatcher. Non-blocking (LL Entry 32).
        if (dev_led_test_pending()) {
            dev_led_test_feed(c);
        } else if (!dev_debug_menu_dispatch(c)) {
            g_menu = RC_OS_MENU_MAIN;
        }
        handled = true;
    }

    if (handled) {
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
