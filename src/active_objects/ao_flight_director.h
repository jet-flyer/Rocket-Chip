// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_FlightDirector — owns the FD QHsm, 100 Hz tick, CLI command dispatch.
//============================================================================
#ifndef ROCKETCHIP_AO_FLIGHT_DIRECTOR_H
#define ROCKETCHIP_AO_FLIGHT_DIRECTOR_H

extern "C" {
#include "qp_port.h"
}

// Forward declarations
namespace rc {
struct FlightDirector;
}

extern QActive * const AO_FlightDirector;

// Initialize and start the Flight Director AO.
// Constructs and inits the QHsm, starts the 100 Hz tick timer.
// Side effects are named fd_effect_* (including reset_subsystems).
void AO_FlightDirector_start(uint8_t prio);

// Dispatch a flight signal from CLI (ARM, DISARM, ABORT, RESET, etc.).
// Replaces cli_dispatch_flight_signal().
void AO_FlightDirector_dispatch_signal(int signal);

// Process a validated flight command (Go/No-Go + dispatch).
// Replaces cli_process_flight_command().
bool AO_FlightDirector_process_command(int cmd);

// Print flight director status to serial.
// Replaces cli_print_flight_status().
void AO_FlightDirector_print_status();

// Read-only access to the FlightDirector instance.
// Used by eskf_runner.cpp for ZUPT on-pad check and phase notification.
const rc::FlightDirector* AO_FlightDirector_get_director();

// Whether the FlightDirector has been initialized.
bool AO_FlightDirector_is_initialized();

// SET_RADIO_CONFIG ground gate. True if not yet initialized, or phase
// is kIdle. LANDED/ABORT need RESET (DISARM is ARMED-only).
bool AO_FlightDirector_is_ground_state();

#endif // ROCKETCHIP_AO_FLIGHT_DIRECTOR_H
