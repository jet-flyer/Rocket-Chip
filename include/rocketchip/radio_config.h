// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// RadioConfig — Radio configuration (IVP-96)
//
// Sibling to MissionProfile — generated alongside it by generate_profile.py.
// MissionProfile is flight behavior. RadioConfig is radio parameters.
//
// User-facing .cfg has simple options (protocol, rate, power).
// Generator derives RF parameters (SF, BW, CR) from those.
// Advanced overrides available in V2 (not V1).
//============================================================================
#ifndef ROCKETCHIP_RADIO_CONFIG_H
#define ROCKETCHIP_RADIO_CONFIG_H

#include <stdint.h>
#include "rocketchip/telemetry_encoder.h"

namespace rc {

struct RadioConfig {
    EncoderType protocol;       // kCcsds or kMavlink
    uint8_t     nav_rate_hz;    // 2, 5, or 10
    uint8_t     power_dbm;      // 2-20
    uint8_t     spreading_factor;  // Derived: 6-12 (default 7)
    uint16_t    bandwidth_khz;     // Derived: 125, 250, or 500
    uint8_t     coding_rate;       // Derived: 5-8 (CR 4/x, default 5)
};

// Default radio config — used when no profile [radio] section exists
inline constexpr RadioConfig kDefaultRadioConfig = {
    .protocol         = EncoderType::kCcsds,
    .nav_rate_hz      = 2,
    .power_dbm        = 20,
    .spreading_factor = 7,
    .bandwidth_khz    = 125,
    .coding_rate      = 5,
};

} // namespace rc

#endif // ROCKETCHIP_RADIO_CONFIG_H
