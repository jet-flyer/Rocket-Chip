// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Air dialect identity for CLI / dashboard. LoRa air is Starcom COP-P.
// USB MAVLink is a separate path (QGC), not this string.

#ifndef ROCKETCHIP_SC_AIR_H
#define ROCKETCHIP_SC_AIR_H

namespace rc {

inline constexpr char const* kAirDialect = "starcom";

}  // namespace rc

#endif  // ROCKETCHIP_SC_AIR_H
