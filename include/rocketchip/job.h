// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file job.h
 * @brief Compile-time device role selector (IVP-95)
 *
 * Selects behavioral configuration based on CMake defines.
 * Board hardware (pins, peripherals) is handled separately by board.h.
 *
 * "Job" = device role. Distinct from "MissionProfile" which is the
 * flight profile data used by the Flight Director at runtime.
 *
 * Supported roles:
 *   VEHICLE  — Flight computer: TX telemetry, sensors, ESKF, logging, FD
 *   STATION  — Ground receiver: RX telemetry, MAVLink USB output, optional GPS
 *   RELAY    — Range extender: RX → validate → re-TX, no decode
 *
 * Default (no define): VEHICLE
 *
 * Usage in CMakeLists.txt:
 *   add_compile_definitions(ROCKETCHIP_JOB_STATION=1)
 *   add_compile_definitions(ROCKETCHIP_JOB_RELAY=1)
 */

#ifndef ROCKETCHIP_JOB_H
#define ROCKETCHIP_JOB_H

#include <stdint.h>

namespace job {

// Device role enum — runtime-queryable even though selection is compile-time
enum class DeviceRole : uint8_t {
    kVehicle = 0,   // Sensors + ESKF + FD + Logger + Telemetry + Radio (TX + RX)
    kStation = 1,   // LedEngine + Telemetry + Radio (RX continuous)
    kRelay   = 2,   // LedEngine + Radio (RX continuous + relay TX)
};

} // namespace job

#if defined(ROCKETCHIP_JOB_RELAY)
    #include "job_relay.h"
#elif defined(ROCKETCHIP_JOB_STATION)
    #include "job_station.h"
#else
    // Default: vehicle (flight computer)
    #include "job_vehicle.h"
#endif

#endif // ROCKETCHIP_JOB_H
