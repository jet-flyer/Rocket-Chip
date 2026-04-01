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

inline constexpr DeviceRole kRole = DeviceRole::kStation;

// Radio mode: RX (telemetry reception)
inline constexpr bool kRadioModeRx = true;

// Default output: CLI text (press 'm' to switch to MAVLink for QGC).
// MAVLink binary on startup floods serial with unparseable data when
// no QGC is connected, making CLI inaccessible.
inline constexpr bool kDefaultMavlinkOutput = false;

} // namespace job

#endif // ROCKETCHIP_JOB_STATION_H
