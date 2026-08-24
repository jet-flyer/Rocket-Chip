// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Conversion functions between FusedState and TelemetryState
// SDK-independent — compiles on host and target.
// IVP-49: Data Model & ICD (Stage 6: Data Logging)

#ifndef ROCKETCHIP_DATA_CONVERT_H
#define ROCKETCHIP_DATA_CONVERT_H

#include "rocketchip/fused_state.h"
#include "rocketchip/telemetry_state.h"

namespace rc {

// Conversion scales (saturates via clamp_round_*): Q15 quat, cm/s vel, mm alt, int8 °C.
void fused_to_telemetry(const FusedState& fused, TelemetryState& out);

// Not bit-exact — quantization is lossy. Used for test validation only.
void telemetry_to_fused_approx(const TelemetryState& t, FusedState& f);

} // namespace rc

#endif // ROCKETCHIP_DATA_CONVERT_H
