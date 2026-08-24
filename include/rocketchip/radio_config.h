// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// RadioConfig — Radio configuration
//
// Sibling to MissionProfile — generated alongside it by generate_profile.py.
// MissionProfile is flight behavior. RadioConfig is radio parameters.
//
// User-facing .cfg has simple options (protocol, rate, power).
// Generator derives RF parameters (SF, BW, CR) from those.
//============================================================================
#ifndef ROCKETCHIP_RADIO_CONFIG_H
#define ROCKETCHIP_RADIO_CONFIG_H

#include <stdint.h>
#include "rocketchip/telemetry_encoder.h"

namespace rc {

// Radio role in config context — mirrors job::DeviceRole values.
// Currently compile-time (Job system).
enum class RadioRole : uint8_t {
    kTx    = 0,   // Vehicle: transmit telemetry
    kRx    = 1,   // Station: receive + decode
    kRelay = 2,   // Relay: receive + forward
};

struct RadioConfig {
    RadioRole   mode;           // TX, RX, or Relay
    EncoderType protocol;       // kCcsds or kMavlink
    uint8_t     nav_rate_hz;    // Hz; legal 1-50 (radio_config_sx1276_legal)
    uint8_t     power_dbm;      // 2-20
    uint8_t     spreading_factor;  // 7-12
    uint16_t    bandwidth_khz;     // 125, 250, or 500
    uint8_t     coding_rate;       // 5-8 (CR 4/x)
};

// Default radio config — used when no profile [radio] section exists
inline constexpr RadioConfig kDefaultRadioConfig = {
    .mode             = RadioRole::kTx,
    .protocol         = EncoderType::kCcsds,
    .nav_rate_hz      = 2,
    .power_dbm        = 20,
    .spreading_factor = 7,
    .bandwidth_khz    = 125,
    .coding_rate      = 5,
};

} // namespace rc

#endif // ROCKETCHIP_RADIO_CONFIG_H
