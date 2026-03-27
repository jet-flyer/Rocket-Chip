// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file data_convert.h
 * @brief Conversion functions between FusedState and TelemetryState
 *
 * SDK-independent — compiles on host and target.
 *
 * IVP-49: Data Model & ICD (Stage 6: Data Logging)
 */

#ifndef ROCKETCHIP_DATA_CONVERT_H
#define ROCKETCHIP_DATA_CONVERT_H

#include "rocketchip/fused_state.h"
#include "rocketchip/telemetry_state.h"

namespace rc {

/**
 * @brief Convert FusedState (float32) to TelemetryState (fixed-point)
 *
 * Quantization:
 *   Quaternion: Q15 (val * 32767), max error ±3.05e-5
 *   Velocity: cm/s (int16), max error ±0.005 m/s
 *   Altitude: mm (int32), max error ±0.001 m
 *   Temperature: rounded int8, max error ±0.5 C
 */
void fused_to_telemetry(const FusedState& fused, TelemetryState& out);

/**
 * @brief Approximate reverse conversion for roundtrip verification
 *
 * Not bit-exact — quantization is lossy. Used for test validation only.
 */
void telemetry_to_fused_approx(const TelemetryState& t, FusedState& f);

} // namespace rc

#endif // ROCKETCHIP_DATA_CONVERT_H
