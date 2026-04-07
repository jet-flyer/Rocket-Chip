// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file job_vehicle.h
 * @brief Vehicle role — flight computer configuration
 *
 * TX telemetry downlink, sensors active, ESKF active, logging active.
 * This is the default role when no job is specified.
 *
 * "Job" = device role, distinct from "MissionProfile" (flight profile data).
 */

#ifndef ROCKETCHIP_JOB_VEHICLE_H
#define ROCKETCHIP_JOB_VEHICLE_H

namespace job {

inline constexpr DeviceRole kRole = DeviceRole::kVehicle;

// Radio mode: TX (telemetry downlink)
inline constexpr bool kRadioModeRx = false;

// No MAVLink output on vehicle (TX mode)
inline constexpr bool kDefaultMavlinkOutput = false;

} // namespace job

#endif // ROCKETCHIP_JOB_VEHICLE_H
