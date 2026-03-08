// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file mission_station.h
 * @brief Station mission profile — ground receiver configuration
 *
 * RX telemetry reception, CCSDS decode, MAVLink re-encode over USB serial.
 * Sensors and ESKF are inert if hardware is absent (same binary,
 * just different behavioral defaults).
 */

#ifndef ROCKETCHIP_MISSION_STATION_H
#define ROCKETCHIP_MISSION_STATION_H

namespace mission {

// Radio mode: RX (telemetry reception)
inline constexpr bool kRadioModeRx = true;

// Default output: MAVLink binary for QGC/Mission Planner
inline constexpr bool kDefaultMavlinkOutput = true;

} // namespace mission

#endif // ROCKETCHIP_MISSION_STATION_H
