// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file mission.h
 * @brief Compile-time mission profile selector
 *
 * Selects behavioral configuration based on ROCKETCHIP_MISSION CMake define.
 * Board hardware (pins, peripherals) is handled separately by board.h.
 *
 * Supported profiles:
 *   VEHICLE  — Flight computer: TX telemetry, sensors active, logging active
 *   STATION  — Ground receiver: RX telemetry, sensors optional, CSV output
 *
 * Default (undefined): VEHICLE
 *
 * Usage in CMakeLists.txt:
 *   add_compile_definitions(ROCKETCHIP_MISSION_STATION=1)
 */

#ifndef ROCKETCHIP_MISSION_H
#define ROCKETCHIP_MISSION_H

#if defined(ROCKETCHIP_MISSION_STATION)
    #include "mission_station.h"
#else
    // Default: vehicle (flight computer)
    #include "mission_vehicle.h"
#endif

#endif // ROCKETCHIP_MISSION_H
