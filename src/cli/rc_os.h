// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// RocketChip OS - CLI menu system (bare-metal)
// Provides terminal-based CLI with menu state machine.
// Adapted from v0.3 FreeRTOS implementation for bare-metal Pico SDK.
// Key patterns:
// - Single-key commands (no parsing)
// - Two-level menu: Main → Calibration
// - Non-blocking input via getchar_timeout_us(0)
// - Terminal-connected guard (no USB I/O when disconnected)

#ifndef ROCKETCHIP_RC_OS_H
#define ROCKETCHIP_RC_OS_H

#include <stdbool.h>
#include <stdint.h>
#include <atomic>

// ============================================================================
// Menu State
// ============================================================================

typedef enum {
    RC_OS_MENU_MAIN = 0,
    RC_OS_MENU_CALIBRATION,
    RC_OS_MENU_FLIGHT,
    RC_OS_MENU_DEBUG,           // IVP-109: debug sub-menu
} rc_os_menu_t;

// ============================================================================
// Initialization
// ============================================================================

// Call once after stdio_init_all() and before main loop.
void rc_os_init(void);

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

// IVP-122: ARM confirm state machine trigger
void rc_os_start_arm_confirm(void);

// IVP-T14d wrap-up: ARM confirm state machine is active. While true,
// callers eating raw input (e.g. station dashboard poll_dashboard_keys)
// must leave chars alone so rc_os_update() can feed the confirm state
// machine.
bool rc_os_arm_confirm_active(void);

// ============================================================================
// Sensor Availability Flags (set by main)
// ============================================================================

// Set these in main.cpp after sensor initialization.
extern bool rc_os_imu_available;
extern bool rc_os_baro_available;

// ============================================================================
// I2C Bus Scan Guard (set by main)
// ============================================================================

// Set to false when Core 1 owns the I2C bus (sensor phase).
// Prevents 'i' command from corrupting bus while Core 1 reads sensors.
// Defaults to true (scan allowed).
extern bool rc_os_i2c_scan_allowed;

// Set to true by cmd_mag_cal() to suppress GPS reads on Core 1.
// In bypass mode, GPS NMEA streaming (0x10) causes bus contention
// with AK09916 mag reads (0x0C). Core 1 checks this flag before
// calling core1_read_gps().
extern std::atomic<bool> rc_os_mag_cal_active;

// P10-9: rc_os_read_accel / rc_os_read_mag / rc_os_reset_mag_staleness
// function-pointer table removed. Accel 6-pos samples come from Core 1;
// mag cal calls cal_read_mag() / cal_reset_mag_staleness() directly
// from ao_rcos.cpp. R-17/R-18 already removed rc_os_cal_pre_hook /
// rc_os_cal_post_hook the same way (I2C-pause in core1_i2c_pause;
// cal_post_hook() called directly from ao_rcos.cpp).

#endif // ROCKETCHIP_RC_OS_H
