// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Nav user-data pack for a future Starcom Space Packet SDU.
// 42 bytes: first 40 of TelemetryState + 2 pad. Not the STOP-GAP 54 B frame.

#ifndef ROCKETCHIP_NAV_SDU_H
#define ROCKETCHIP_NAV_SDU_H

#include <stdint.h>
#include "rocketchip/telemetry_state.h"

namespace rc {

constexpr uint8_t kNavSduUserBytes = 42;
constexpr uint8_t kNavSduTelemBytes = 40;

// Writes kNavSduUserBytes into out. Returns bytes written, or 0 if out_len
// is too small.
uint8_t pack_nav_sdu_user(uint8_t* out, uint8_t out_len,
                          const TelemetryState& telem);

// Copies the 40-byte telem prefix into *telem. Leaves met_ms and flags
// unchanged. Returns false if in_len != kNavSduUserBytes.
bool unpack_nav_sdu_user(const uint8_t* in, uint8_t in_len,
                         TelemetryState* telem);

}  // namespace rc

#endif  // ROCKETCHIP_NAV_SDU_H
