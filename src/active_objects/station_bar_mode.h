// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Station NeoPixel vs RX freshness. Header-only for host tests.
// Hold matches AO_Radio handle_link_quality kLinkGapMs (2 s).
// After a live link, LOS stays Waiting (Cylon) until RX returns.
// Dim red is boot / never-heard only — Cylon already means LOS.
#ifndef ROCKETCHIP_STATION_BAR_MODE_H
#define ROCKETCHIP_STATION_BAR_MODE_H

#include <stdint.h>

enum class StationBarMode : uint8_t {
    NoSignal = 0,  // never heard — dim red
    Waiting  = 1,  // had signal, now quiet — Cylon
    Live     = 2,  // packets in last 2 s — solid RSSI
};

static constexpr uint32_t kStationBarHoldMs = 2000;

inline StationBarMode station_bar_mode(bool starcom_heard,
                                       uint32_t rx_count, uint32_t gap_ms) {
    if (!starcom_heard || rx_count == 0) {
        return StationBarMode::NoSignal;
    }
    if (gap_ms < kStationBarHoldMs) {
        return StationBarMode::Live;
    }
    return StationBarMode::Waiting;
}

#endif
