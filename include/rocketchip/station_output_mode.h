// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file station_output_mode.h
 * @brief Station output mode enum — shared between AO_RCOS and AO_Telemetry
 *
 * Extracted to its own header to avoid circular includes (Council A3).
 * AO_RCOS owns the write side, AO_Telemetry reads it.
 */

#ifndef ROCKETCHIP_STATION_OUTPUT_MODE_H
#define ROCKETCHIP_STATION_OUTPUT_MODE_H

#include <stdint.h>

// Station output mode — cycles with 'm' key.
// Future: boot default will be user-configurable via advanced settings menu.
enum class StationOutputMode : uint8_t {
    kAnsi,      // Live ANSI dashboard (boot default)
    kCsv,       // Machine-readable CSV lines
    kMavlink,   // Binary MAVLink v2
    kMenu,      // CLI menu — telemetry output suppressed
};

// Getter — implemented by AO_RCOS (ao_rcos.cpp)
StationOutputMode AO_RCOS_get_output_mode();
void AO_RCOS_set_output_mode(StationOutputMode mode);
void AO_RCOS_cycle_output_mode();

#endif // ROCKETCHIP_STATION_OUTPUT_MODE_H
