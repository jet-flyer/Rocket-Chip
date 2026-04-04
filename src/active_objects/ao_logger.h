// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_Logger — Flight Data Logger Active Object (IVP-79, Phase 4)
//
// Owns the logging ring buffer, decimator, flight table, FusedState builder,
// and event logging. Publishes PCM frames to the ring buffer at decimated
// rate (50Hz PSRAM / 25Hz SRAM). CLI commands access state through
// read-only accessors (Council A6 pattern).
//
// Phase 4 migration: logging_tick(), populate_fused_state(),
// fused_copy_eskf_state(), log_flight_event(), init_logging_ring(),
// ring buffer globals, decimation constants all moved here from main.cpp.
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

/// Initialize ring buffer, decimator, flight table, and start 50Hz tick.
/// Must be called after PSRAM init and flight_table_load().
/// @param prio  QP/C priority level for this AO
/// @param psram_size  PSRAM size in bytes (0 if absent)
/// @param psram_self_test_passed  true if PSRAM self-test passed
void AO_Logger_start(uint8_t prio, size_t psram_size, bool psram_self_test_passed);

/// Read-only access to the ring buffer (Council A6).
/// Used by CLI commands (cmd_flush_log, print_logging_status).
const rc::RingBuffer* AO_Logger_get_ring();

/// Mutable access to the ring buffer — needed by CLI flush command
/// (flush_ring_to_flash modifies head). Phase 7 will encapsulate behind AO commands.
rc::RingBuffer* AO_Logger_get_ring_mut();

/// Read-only access to the flight table (Council A6).
/// Used by CLI commands and AO_FlightDirector Go/No-Go checks.
const rc::FlightTableState* AO_Logger_get_flight_table();

/// Mutable access to the flight table — needed by CLI flush/erase commands
/// that write to flash. Phase 7 will encapsulate these behind AO commands.
rc::FlightTableState* AO_Logger_get_flight_table_mut();

/// Whether the logging ring buffer has been initialized.
bool AO_Logger_is_initialized();

/// Log a discrete flight event to the ring buffer as a PCM event frame.
/// Called from AO_FlightDirector (pyro fire, abort, etc.) and eskf_runner
/// (via callback).
void AO_Logger_log_event(rc::LogEventId id,
                         uint8_t d0 = 0, uint8_t d1 = 0,
                         uint8_t d2 = 0, uint8_t d3 = 0);

/// Build FusedState from ESKF + sensor snapshot.
/// Shared utility: used by AO_Logger (logging_tick) and AO_FlightDirector
/// (guard evaluation). Declared here so both can call it.
void AO_Logger_populate_fused_state(rc::FusedState& fused,
                                    const shared_sensor_data_t& snap);

#endif // ROCKETCHIP_AO_LOGGER_H
