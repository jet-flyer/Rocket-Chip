// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Station role — ground receiver configuration
// RX telemetry reception, Starcom COP-P decode, MAVLink re-encode over USB.
// Compile-time job pack (ROCKETCHIP_JOB_STATION), not a runtime switch
// in the vehicle binary. Sensors/ESKF are not started on this role.
// "Job" = device role, distinct from "MissionProfile" (flight profile data).

#ifndef ROCKETCHIP_JOB_STATION_H
#define ROCKETCHIP_JOB_STATION_H

namespace job {

inline constexpr DeviceRole kRole = DeviceRole::kStation;

// Radio mode: RX (telemetry reception)
inline constexpr bool kRadioModeRx = true;

// Default output: ANSI pad. First USB MAVLink STX (0xFD/0xFE) takes
// exclusive CDC; do not boot already in kMavlink.
inline constexpr bool kDefaultMavlinkOutput = false;

} // namespace job

#endif // ROCKETCHIP_JOB_STATION_H
