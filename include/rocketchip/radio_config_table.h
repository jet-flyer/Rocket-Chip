// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// RadioConfig whitelist — SET_RADIO_CONFIG denies anything not listed.
// Only tuples this firmware has been tested to run. Code is canonical
// (docs/RADIO_TELEMETRY_STATUS.md points here, not the reverse).
//============================================================================
#ifndef ROCKETCHIP_RADIO_CONFIG_TABLE_H
#define ROCKETCHIP_RADIO_CONFIG_TABLE_H

#include <stddef.h>
#include <stdint.h>

namespace rc {

struct RadioConfigEntry {
    uint16_t bw_khz;        // 125, 250, 500
    uint8_t  nav_rate_hz;   // 2, 5, 10
    uint8_t  sf;            // 7 (only value currently supported)
    uint8_t  cr;            // 5 (CR 4/5, only value currently supported)
    uint8_t  power_dbm;     // 2-20 (SX1276 legal range)
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

// Preset-membership test. Returns true if `{bw, nav, sf, cr, power}`
// matches one of the entries in `kRadioConfigTable` exactly. Used by the
// debug-menu digit-key path (`q<digit>z`), channel-find scanner, and boot
// seed. Not the production gate for runtime SET_RADIO_CONFIG — that uses
// radio_config_sx1276_legal() below.
//
// (Name retained as `_in_whitelist` for existing call-site compatibility;
// see radio_config_sx1276_legal for the broader advanced-settings path.)
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

// Broader validator: accepts any {bw, nav, sf, cr, power} that is legal for
// SX1276 hardware AND that this firmware can plausibly operate at. User
// flagged 2026-04-21: "the preconfigured data rates are just presets... the
// user should really be able to change it to anything the radio can do."
//
// Per SX1276 datasheet §5.4:
//   BW: 125 / 250 / 500 kHz (lower values theoretically legal but rarely used;
//       accept 7.8 / 10.4 / 15.6 / 20.8 / 31.25 / 41.7 / 62.5 if extended later)
//   SF: 6-12 (SF6 is implicit-header only and not supported by this firmware)
//   CR: 5-8 (4/5 through 4/8)
//   Power (PA_BOOST): 2-20 dBm
// Nav rate: anything > 0 that leaves headroom above airtime. We sanity-cap at
// 50 Hz (20 ms period) — the TX interval check in AO_Telemetry would silently
// wedge if interval < airtime.
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
