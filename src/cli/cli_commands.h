// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// CLI Commands — Display Functions and Key Handlers
//
// Pure command/display code extracted from main.cpp (Phase 7).
// These functions read state from AO public APIs and sensor seqlock —
// they own no state themselves.
//
// Wired into rc_os hooks by init_rc_os_hooks() in main.cpp.
//============================================================================
#ifndef ROCKETCHIP_CLI_COMMANDS_H
#define ROCKETCHIP_CLI_COMMANDS_H

/// Sensor status display (vehicle mode 's' key)
void cli_print_sensor_status();

/// Station RX telemetry display (station mode 's' key)
void cli_print_station_status();

/// Hardware status summary ('b' key reprint)
void cli_print_hw_status();

/// Full boot banner (first terminal connection)
void cli_print_boot_status();

/// Compact ESKF live output (1Hz, 'e' key)
void cli_print_eskf_live();

/// Unhandled key dispatcher (logging, radio, download commands)
void cli_handle_unhandled_key(int key);

#endif // ROCKETCHIP_CLI_COMMANDS_H
