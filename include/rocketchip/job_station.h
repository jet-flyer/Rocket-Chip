// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file job_station.h
 * @brief Station role — ground receiver configuration
 *
 * RX telemetry reception, CCSDS decode, MAVLink re-encode over USB serial.
 * Sensors and ESKF are inert if hardware is absent (same binary,
 * just different behavioral defaults).
 *
 * Renamed from mission_station.h in IVP-68 to distinguish device role
 * ("job") from flight profile data ("MissionProfile").
 */

#ifndef ROCKETCHIP_JOB_STATION_H
#define ROCKETCHIP_JOB_STATION_H

namespace job {

// Radio mode: RX (telemetry reception)
inline constexpr bool kRadioModeRx = true;

// Default output: MAVLink binary for QGC/Mission Planner
inline constexpr bool kDefaultMavlinkOutput = true;

} // namespace job

#endif // ROCKETCHIP_JOB_STATION_H
