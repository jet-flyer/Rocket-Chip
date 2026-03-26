// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file job_vehicle.h
 * @brief Vehicle role — flight computer configuration
 *
 * TX telemetry downlink, sensors active, ESKF active, logging active.
 * This is the default role when no job is specified.
 *
 * Renamed from mission_vehicle.h in IVP-68 to distinguish device role
 * ("job") from flight profile data ("MissionProfile").
 */

#ifndef ROCKETCHIP_JOB_VEHICLE_H
#define ROCKETCHIP_JOB_VEHICLE_H

namespace job {

// Radio mode: TX (telemetry downlink)
inline constexpr bool kRadioModeRx = false;

// No MAVLink output on vehicle (TX mode)
inline constexpr bool kDefaultMavlinkOutput = false;

} // namespace job

#endif // ROCKETCHIP_JOB_VEHICLE_H
