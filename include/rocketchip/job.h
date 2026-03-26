// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file job.h
 * @brief Compile-time device role selector
 *
 * Selects behavioral configuration based on ROCKETCHIP_JOB_STATION CMake define.
 * Board hardware (pins, peripherals) is handled separately by board.h.
 *
 * "Job" = device role (vehicle vs station). Distinct from "MissionProfile"
 * which is the flight profile data (guards, thresholds, abort actions)
 * used by the Flight Director at runtime.
 *
 * Supported roles:
 *   VEHICLE  — Flight computer: TX telemetry, sensors active, logging active
 *   STATION  — Ground receiver: RX telemetry, sensors optional, CSV output
 *
 * Default (undefined): VEHICLE
 *
 * Usage in CMakeLists.txt:
 *   add_compile_definitions(ROCKETCHIP_JOB_STATION=1)
 */

#ifndef ROCKETCHIP_JOB_H
#define ROCKETCHIP_JOB_H

#if defined(ROCKETCHIP_JOB_STATION)
    #include "job_station.h"
#else
    // Default: vehicle (flight computer)
    #include "job_vehicle.h"
#endif

#endif // ROCKETCHIP_JOB_H
