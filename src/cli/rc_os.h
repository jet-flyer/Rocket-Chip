// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// USB / lockout / ARM-confirm + engine kKey dispatch. Menu data is
// cli_menus.h. Station ANSI pad is not a menu (ao_rcos poll_dashboard_keys).

#ifndef ROCKETCHIP_RC_OS_H
#define ROCKETCHIP_RC_OS_H

#include <stdbool.h>
#include <stdint.h>

// ============================================================================
// Menu State
// ============================================================================

typedef enum {
    RC_OS_MENU_MAIN = 0,
    RC_OS_MENU_CALIBRATION,
    RC_OS_MENU_FLIGHT,
    RC_OS_MENU_DEBUG,
    RC_OS_MENU_SETTINGS,
} rc_os_menu_t;

// ============================================================================
// Initialization
// ============================================================================

// Call once after stdio_init_all() and before main loop.
void rc_os_init(void);
void rc_os_reset_to_main(void);
void rc_os_print_help(void);

// ============================================================================
// Main Loop Integration
// ============================================================================

// This function:
// - Checks if terminal is connected
// - Prints banner on first connection
// - Processes single-key commands
// - Runs calibration state machines
// Should be called at ~20Hz (every 50ms) from main loop.
// Does nothing if terminal not connected.
bool rc_os_update(void);

bool rc_os_is_connected(void);

bool rc_os_is_calibrating(void);

rc_os_menu_t rc_os_get_menu(void);

void rc_os_start_arm_confirm(void);

// True while ARM confirm is reading ARM+Enter. Pad key poll must not
// consume those chars — rc_os_update owns the buffer.
bool rc_os_arm_confirm_active(void);

// USB just connected; banner/pad keys wait. GCS STX still sniffed here.
bool rc_os_usb_settling(void);

// Runtime DEV_MODE. Compile-time ROCKETCHIP_DEV_MODE must be on for the
// toggle and inject/cal-reset rows to exist. Enable: USB + FD idle.
// Stays on across ARM so inject can run; USB unplug clears it.
// Probe test_mode_active() is a different gate (fault_force_*).
bool rc_os_dev_mode_runtime(void);
void rc_os_dev_mode_toggle(void);

// ============================================================================
// Sensor Availability Flags (set by main)
// ============================================================================

// Set these in main.cpp after sensor initialization.
extern bool rc_os_imu_available;
extern bool rc_os_baro_available;

// I2C scan / mag-cal GPS suppress: shared_state.h.

#endif // ROCKETCHIP_RC_OS_H
