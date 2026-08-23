// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// ISA / hypsometric constants — not sensor-specific. Used by any
// pressure→altitude path (baro driver, calibration).

#ifndef ROCKETCHIP_ISA_ATMOSPHERE_H
#define ROCKETCHIP_ISA_ATMOSPHERE_H

namespace rc {

// ICAO ISA sea-level pressure.
inline constexpr float kStdAtmPressurePa = 101325.0F;
// Barometric formula: h = T0/L * (1 - (p/p0)^(R*L/g)) with T0=288.15 K,
// L=0.0065 K/m → 44330.77 m and exponent 0.190266.
inline constexpr float kHypsometricScale    = 44330.0F;
inline constexpr float kHypsometricExponent = 0.1903F;

} // namespace rc

#endif // ROCKETCHIP_ISA_ATMOSPHERE_H
