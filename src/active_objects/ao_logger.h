// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_Logger — Flight Data Logger Active Object
//
// Owns ring, decimator, flight table, FusedState builder, and event log.
// 50 Hz tick pushes one FusedState into the boxcar (ratio 4 PSRAM / 8 SRAM).
//============================================================================
#ifndef ROCKETCHIP_AO_LOGGER_H
#define ROCKETCHIP_AO_LOGGER_H

extern "C" {
#include "qp_port.h"
}

#include <stdint.h>
#include "rocketchip/pcm_frame.h"    // rc::LogEventId

// Forward declarations
namespace rc {
struct RingBuffer;
struct FlightTableState;
struct FusedState;
}
struct shared_sensor_data_t;

extern QActive * const AO_Logger;

// After PSRAM init and flash-safe test. Body calls flight_table_load().
// psram_ok: self-test AND flash-safe (else SRAM ring).
void AO_Logger_start(uint8_t prio, size_t psram_size, bool psram_ok);

// Read-only access to the ring buffer (Council A6).
// Used by CLI commands (cmd_flush_log, print_logging_status).
const rc::RingBuffer* AO_Logger_get_ring();

// Mutable access to the ring buffer — needed by CLI flush command
// (flush_ring_to_flash modifies head).
rc::RingBuffer* AO_Logger_get_ring_mut();

// Read-only access to the flight table (Council A6).
// Used by CLI commands and AO_FlightDirector Go/No-Go checks.
const rc::FlightTableState* AO_Logger_get_flight_table();

// Mutable access to the flight table — needed by CLI flush/erase commands
// that write to flash.
rc::FlightTableState* AO_Logger_get_flight_table_mut();

// Whether the logging ring buffer has been initialized.
bool AO_Logger_is_initialized();

// Log a discrete flight event to the ring buffer as a PCM event frame.
// Called from AO_FlightDirector (pyro fire, abort, etc.) and eskf_runner
// (via callback).
void AO_Logger_log_event(rc::LogEventId id,
                         uint8_t d0 = 0, uint8_t d1 = 0,
                         uint8_t d2 = 0, uint8_t d3 = 0);

// Build FusedState from ESKF + sensor snapshot.
// Shared utility: used by AO_Logger (logging_tick) and AO_FlightDirector
// (guard evaluation). Declared here so both can call it.
void AO_Logger_populate_fused_state(rc::FusedState& fused,
                                    const shared_sensor_data_t& snap);

#endif // ROCKETCHIP_AO_LOGGER_H
