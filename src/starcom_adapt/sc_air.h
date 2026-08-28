// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Compile-time air dialect for the Starcom consumer soak.
// Host BUILD_TESTS + ROCKETCHIP_USE_STARCOM links Starcom::starcom (IVP 20).
// Pico link is IVP 21. Air path remains STOP-GAP until strip/cutover.

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
