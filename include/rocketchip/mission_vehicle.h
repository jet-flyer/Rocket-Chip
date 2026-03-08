// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file mission_vehicle.h
 * @brief Vehicle mission profile — flight computer configuration
 *
 * TX telemetry downlink, sensors active, ESKF active, logging active.
 * This is the default profile when no mission is specified.
 */

#ifndef ROCKETCHIP_MISSION_VEHICLE_H
#define ROCKETCHIP_MISSION_VEHICLE_H

namespace mission {

// Radio mode: TX (telemetry downlink)
inline constexpr bool kRadioModeRx = false;

// No MAVLink output on vehicle (TX mode)
inline constexpr bool kDefaultMavlinkOutput = false;

} // namespace mission

#endif // ROCKETCHIP_MISSION_VEHICLE_H
