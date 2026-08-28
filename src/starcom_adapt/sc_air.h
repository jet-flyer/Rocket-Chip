// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Compile-time air dialect for the Starcom consumer soak.
// ROCKETCHIP_USE_STARCOM links Starcom::starcom on host tests and Pico (IVP 21–22).
// Default OFF remains STOP-GAP air. kAirLoraCommandsEnabled is the STOP-GAP
// MAVLink-on-LoRa switch; ON uses COP-P instead.

#ifndef ROCKETCHIP_SC_AIR_H
#define ROCKETCHIP_SC_AIR_H

namespace rc {

#ifdef ROCKETCHIP_USE_STARCOM
inline constexpr bool kStarcomPrepBuild = true;
inline constexpr bool kAirLoraCommandsEnabled = false;
inline constexpr char const* kAirDialect = "starcom-prep";
#else
inline constexpr bool kStarcomPrepBuild = false;
inline constexpr bool kAirLoraCommandsEnabled = true;
inline constexpr char const* kAirDialect = "stop-gap";
#endif

}  // namespace rc

#endif  // ROCKETCHIP_SC_AIR_H
