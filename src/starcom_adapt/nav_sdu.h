// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Nav user-data pack for the Starcom Space Packet SDU.
// Full packed TelemetryState (45 B), including met_ms and flags.
// Not the STOP-GAP 54 B CCSDS frame (primary + MET secondary + 42 B prefix).

#ifndef ROCKETCHIP_NAV_SDU_H
#define ROCKETCHIP_NAV_SDU_H

#include <stddef.h>
#include <stdint.h>
#include "rocketchip/telemetry_state.h"

namespace rc {

constexpr uint8_t kNavSduUserBytes = 45;
constexpr uint8_t kNavSduTelemBytes = 45;
static_assert(kNavSduUserBytes == sizeof(TelemetryState),
              "nav SDU is the whole packed TelemetryState");
static_assert(offsetof(TelemetryState, met_ms) == 40,
              "met_ms must stay at byte 40 on the nav SDU");
static_assert(offsetof(TelemetryState, flags) == 44,
              "flags must stay at byte 44 on the nav SDU");

// Writes kNavSduUserBytes into out. Returns bytes written, or 0 if out_len
// is too small.
uint8_t pack_nav_sdu_user(uint8_t* out, uint8_t out_len,
                          const TelemetryState& telem);

// Copies the full TelemetryState, including met_ms and flags.
// Returns false if in_len != kNavSduUserBytes.
bool unpack_nav_sdu_user(const uint8_t* in, uint8_t in_len,
                         TelemetryState* telem);

}  // namespace rc

#endif  // ROCKETCHIP_NAV_SDU_H
