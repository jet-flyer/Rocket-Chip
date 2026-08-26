// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Compile-time air dialect for the Starcom consumer soak.
// ROCKETCHIP_USE_STARCOM does not link Starcom::starcom yet.

#ifndef ROCKETCHIP_SC_AIR_H
#define ROCKETCHIP_SC_AIR_H

namespace rc {

#ifdef ROCKETCHIP_USE_STARCOM
inline constexpr bool kStarcomPrepBuild = true;
inline constexpr bool kAirLoraCommandsEnabled = false;
#else
inline constexpr bool kStarcomPrepBuild = false;
inline constexpr bool kAirLoraCommandsEnabled = true;
#endif

}  // namespace rc

#endif  // ROCKETCHIP_SC_AIR_H
