// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Station NeoPixel vs RF / COP-P. Header-only for host tests.
// Hold matches AO_Radio handle_link_quality kLinkGapMs (2 s).
// RF-on-band is raw LoRa RX (rx_count / last_rx gap), not Starcom decode.
// COP-P lock is FOP-P plcw_heard. After a live link, LOS stays Waiting
// (Cylon) until RX returns. Dim red is boot / never-heard only.
#ifndef ROCKETCHIP_STATION_BAR_MODE_H
#define ROCKETCHIP_STATION_BAR_MODE_H

#include <stdint.h>

enum class StationBarMode : uint8_t {
    NoSignal = 0,  // never heard RF — dim red
    Waiting  = 1,  // had RF, now quiet — Cylon
    RfHeard  = 2,  // RF in last 2 s, no COP-P lock — all 5 flash green
    Locked   = 3,  // COP-P lock and RF in last 2 s — solid RSSI
};

static constexpr uint32_t kStationBarHoldMs = 2000;

inline StationBarMode station_bar_mode(bool copp_lock,
                                       uint32_t rx_count, uint32_t gap_ms) {
    if (rx_count == 0) {
        return StationBarMode::NoSignal;
    }
    if (gap_ms >= kStationBarHoldMs) {
        return StationBarMode::Waiting;
    }
    if (copp_lock) {
        return StationBarMode::Locked;
    }
    return StationBarMode::RfHeard;
}

#endif
