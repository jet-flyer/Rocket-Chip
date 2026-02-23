// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#ifndef ROCKETCHIP_FUSION_ESKF_STATE_H
#define ROCKETCHIP_FUSION_ESKF_STATE_H

// ESKF state vector indices.
// Pure C++ — no Pico SDK dependencies.
//
// 24-state error-state Kalman filter per Sola (2017) S5.
// Error state: dx = [d_theta(3), d_p(3), d_v(3), d_a_bias(3), d_g_bias(3),
//                     d_earth_mag(3), d_body_mag_bias(3), d_wind_NE(2), d_baro_bias(1)]
// Named indices prevent off-by-one errors in F_x, H, and K construction.
//
// States 15-23 are "extended" states with runtime inhibit flags (ArduPilot pattern).
// When inhibited, P diagonal = 0, cross-covariances = 0, no compute cost.
// ArduPilot EKF3 reference: statesArray[24] in NavEKF3_core.h.

#include <cstdint>

namespace rc::eskf {

// --- Core 15 states (always active) ---
constexpr int32_t kIdxAttitude     = 0;   // delta_theta     [0..2]
constexpr int32_t kIdxPosition     = 3;   // delta_p          [3..5]
constexpr int32_t kIdxVelocity     = 6;   // delta_v          [6..8]
constexpr int32_t kIdxAccelBias    = 9;   // delta_a_bias     [9..11]
constexpr int32_t kIdxGyroBias     = 12;  // delta_g_bias     [12..14]

// --- Extended states (runtime inhibit flags) ---
constexpr int32_t kIdxEarthMag     = 15;  // Earth field NED  [15..17]
constexpr int32_t kIdxBodyMagBias  = 18;  // Hard-iron body   [18..20]
constexpr int32_t kIdxWindNE       = 21;  // Horizontal wind  [21..22]
constexpr int32_t kIdxBaroBias     = 23;  // Baro alt bias    [23]

constexpr int32_t kStateSize       = 24;  // Total error state dimension

// Block sizes
constexpr int32_t kBlockSize = 3;

// Per-axis named indices (used in measurement update H-vector construction)
constexpr int32_t kIdxPosN  = kIdxPosition + 0;   // 3
constexpr int32_t kIdxPosE  = kIdxPosition + 1;   // 4
constexpr int32_t kIdxPosD  = kIdxPosition + 2;   // 5
constexpr int32_t kIdxVelN  = kIdxVelocity + 0;   // 6
constexpr int32_t kIdxVelE  = kIdxVelocity + 1;   // 7
constexpr int32_t kIdxVelD  = kIdxVelocity + 2;   // 8
constexpr int32_t kIdxYaw   = kIdxAttitude + 2;   // 2
constexpr int32_t kIdxWindN = kIdxWindNE + 0;      // 21
constexpr int32_t kIdxWindE = kIdxWindNE + 1;      // 22

} // namespace rc::eskf

#endif // ROCKETCHIP_FUSION_ESKF_STATE_H
