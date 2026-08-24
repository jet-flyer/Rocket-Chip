// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// RadioConfig presets (tested tuples) plus sx1276_legal() for SET.
// Preset membership is radio_config_in_whitelist. Production SET_RADIO_CONFIG
// uses radio_config_sx1276_legal(), not this table alone.
//============================================================================
#ifndef ROCKETCHIP_RADIO_CONFIG_TABLE_H
#define ROCKETCHIP_RADIO_CONFIG_TABLE_H

#include <stddef.h>
#include <stdint.h>

namespace rc {

struct RadioConfigEntry {
    uint16_t bw_khz;        // presets: 125, 250, 500
    uint8_t  nav_rate_hz;   // presets: 2, 5, 10
    uint8_t  sf;            // presets: 7
    uint8_t  cr;            // presets: 5 (CR 4/5)
    uint8_t  power_dbm;     // presets: 20 (legal 2-20)
};

// Tested runtime-SET tuples. Default first, then higher-rate / wider-BW,
// then lower-rate fallbacks.
inline constexpr RadioConfigEntry kRadioConfigTable[] = {
    { .bw_khz = 125, .nav_rate_hz = 5,  .sf = 7, .cr = 5, .power_dbm = 20 },
    { .bw_khz = 125, .nav_rate_hz = 10, .sf = 7, .cr = 5, .power_dbm = 20 },
    { .bw_khz = 250, .nav_rate_hz = 10, .sf = 7, .cr = 5, .power_dbm = 20 },
    { .bw_khz = 500, .nav_rate_hz = 10, .sf = 7, .cr = 5, .power_dbm = 20 },
    { .bw_khz = 125, .nav_rate_hz = 2,  .sf = 7, .cr = 5, .power_dbm = 20 },
    { .bw_khz = 250, .nav_rate_hz = 5,  .sf = 7, .cr = 5, .power_dbm = 20 },
};

inline constexpr size_t kRadioConfigTableSize =
    sizeof(kRadioConfigTable) / sizeof(kRadioConfigTable[0]);

// Exact match against kRadioConfigTable. Not the SET_RADIO_CONFIG gate.
inline constexpr bool radio_config_in_whitelist(uint16_t bw_khz,
                                                 uint8_t nav_rate_hz,
                                                 uint8_t sf,
                                                 uint8_t cr,
                                                 uint8_t power_dbm) {
    for (size_t i = 0; i < kRadioConfigTableSize; ++i) {
        const auto& e = kRadioConfigTable[i];
        if (e.bw_khz == bw_khz && e.nav_rate_hz == nav_rate_hz &&
            e.sf == sf && e.cr == cr && e.power_dbm == power_dbm) {
            return true;
        }
    }
    return false;
}

// SX1276 gate: BW 125/250/500, SF 7-12, CR 5-8, power 2-20 dBm, nav 1-50 Hz.
// No airtime-headroom check.
inline constexpr bool radio_config_sx1276_legal(uint16_t bw_khz,
                                                 uint8_t nav_rate_hz,
                                                 uint8_t sf,
                                                 uint8_t cr,
                                                 uint8_t power_dbm) {
    if (bw_khz != 125 && bw_khz != 250 && bw_khz != 500) { return false; }
    if (sf < 7 || sf > 12) { return false; }
    if (cr < 5 || cr > 8) { return false; }
    if (power_dbm < 2 || power_dbm > 20) { return false; }
    if (nav_rate_hz == 0 || nav_rate_hz > 50) { return false; }
    return true;
}

} // namespace rc

#endif // ROCKETCHIP_RADIO_CONFIG_TABLE_H
