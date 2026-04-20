// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// radio_config_table.h — Stage T IVP-T5.5: RadioConfig whitelist
//
// **Canonical** list of valid `{bw_khz, nav_rate_hz, sf, cr, power_dbm}`
// tuples. The SET_RADIO_CONFIG dispatcher rejects any incoming config not
// in this table with denied-ACK. The (deferred) channel-find scanner will
// also iterate this list in order.
//
// Code is authoritative; docs/RADIO_TELEMETRY_STATUS.md references THIS
// header (not vice versa) — prevents doc-vs-code drift.
//
// Design rules (correctness-council edit #2):
//   - Every tuple must be a combination the firmware is tested to
//     operate at without silent-RX regressions.
//   - Entries that are merely "legal on the SX1276 datasheet" but untested
//     in this project must NOT appear here.
//   - Keep the list small. ≤10 entries is the target so channel-find scan
//     stays under ~30 s (or ~1-2 s with CAD acceleration).
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

// Whitelist of valid runtime-SET config tuples.
// Ordered to match the future channel-find scanner's sweep order:
// most-common/default first, then the Stage T T6 sweep candidates.
inline constexpr RadioConfigEntry kRadioConfigTable[] = {
    // Default: the compile-time kDefaultRocketRadioConfig baseline.
    { .bw_khz = 125, .nav_rate_hz = 5,  .sf = 7, .cr = 5, .power_dbm = 20 },

    // Stage T IVP-T6 sweep candidates (in sweep order):
    { .bw_khz = 125, .nav_rate_hz = 10, .sf = 7, .cr = 5, .power_dbm = 20 },  // C0-equivalent at 10 Hz
    { .bw_khz = 250, .nav_rate_hz = 10, .sf = 7, .cr = 5, .power_dbm = 20 },  // C1
    { .bw_khz = 500, .nav_rate_hz = 10, .sf = 7, .cr = 5, .power_dbm = 20 },  // C2 primary candidate

    // Lower-rate fallbacks (useful for long-range or degraded-link recovery):
    { .bw_khz = 125, .nav_rate_hz = 2,  .sf = 7, .cr = 5, .power_dbm = 20 },
    { .bw_khz = 250, .nav_rate_hz = 5,  .sf = 7, .cr = 5, .power_dbm = 20 },
};

inline constexpr size_t kRadioConfigTableSize =
    sizeof(kRadioConfigTable) / sizeof(kRadioConfigTable[0]);

// Whitelist membership test. Returns true if `{bw, nav, sf, cr, power}`
// matches one of the entries in `kRadioConfigTable` exactly.
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

} // namespace rc

#endif // ROCKETCHIP_RADIO_CONFIG_TABLE_H
