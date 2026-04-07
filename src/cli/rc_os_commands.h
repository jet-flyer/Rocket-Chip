// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// RC_OS Commands — Display Functions and Key Handlers
//
// Part of the RC_OS CLI subsystem. Pure command/display code — reads
// state from AO public APIs and sensor seqlock, owns no state.
// Called directly from rc_os.cpp menu handlers.
//============================================================================
#ifndef ROCKETCHIP_RC_OS_COMMANDS_H
#define ROCKETCHIP_RC_OS_COMMANDS_H

/// Sensor status display (vehicle mode 's' key)
void cli_print_sensor_status();

/// Station RX telemetry display (station mode 's' key)
void cli_print_station_status();

/// Compact boot summary (auto-printed on connect)
void cli_print_boot_summary();

/// Full hardware init detail ('b' Boot Log key)
void cli_print_hw_status();

/// Full boot output (summary + HW detail, first connect only)
void cli_print_boot_status();

/// Compact ESKF live output (1Hz, 'e' key)
void cli_print_eskf_live();

/// Unhandled key dispatcher (logging, radio, download commands)
void cli_handle_unhandled_key(int key);

#endif // ROCKETCHIP_RC_OS_COMMANDS_H
