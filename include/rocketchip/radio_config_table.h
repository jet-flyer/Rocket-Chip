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
