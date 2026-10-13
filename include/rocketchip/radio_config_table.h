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
#include "rocketchip/telemetry_state.h"

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
// idx 1 (125/10 SF7) is chip-legal leftover: nav PLTU ToA does not fit
// 10 Hz. COMM_CHANGE / NAV_PRESET / SET skip it via radio_config_next_fit
// / radio_config_nav_fits_hz. Do not reorder — catalog idx rides SET PL
// EXTENSIONS.
inline constexpr RadioConfigEntry kRadioConfigTable[] = {
    { .bw_khz = 125, .nav_rate_hz = 5,  .sf = 7, .cr = 5, .power_dbm = 2 },
    { .bw_khz = 125, .nav_rate_hz = 10, .sf = 7, .cr = 5, .power_dbm = 2 },
    { .bw_khz = 250, .nav_rate_hz = 10, .sf = 7, .cr = 5, .power_dbm = 2 },
    { .bw_khz = 500, .nav_rate_hz = 10, .sf = 7, .cr = 5, .power_dbm = 2 },
    { .bw_khz = 125, .nav_rate_hz = 2,  .sf = 7, .cr = 5, .power_dbm = 2 },
    { .bw_khz = 250, .nav_rate_hz = 5,  .sf = 7, .cr = 5, .power_dbm = 2 },
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
// Chip-legal only. Hops (COMM_CHANGE / NAV_PRESET / SET) also need
// radio_config_nav_fits_hz so commanded Hz can actually air.
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

// Nav PLTU on air: ASM+V-3+Space Packet+CRC = 18 + kNavSduUserBytes (51).
// Source: test_starcom_byte_pump encode_nav. Same formula as rfm95w_airtime_us
// (SX1276 §4.1.1.6, explicit header, CRC on, 8-sym preamble, CR 4/5).
inline constexpr uint8_t kRadioConfigNavPltuBytes = 69;
static_assert(18U + sizeof(TelemetryState) == kRadioConfigNavPltuBytes,
              "nav airtime byte count drifted from the SDU");
inline constexpr uint8_t kRadioConfigNoIndex = 0xFF;

inline constexpr uint32_t radio_config_nav_airtime_us(uint8_t sf, uint16_t bw_khz,
                                                      uint8_t payload_bytes) {
    if (bw_khz == 0) { bw_khz = 125; }
    if (sf < 7) { sf = 7; }
    if (sf > 12) { sf = 12; }
    const uint32_t t_sym_us =
        (static_cast<uint32_t>(1U) << sf) * 1000U / bw_khz;
    const uint32_t t_preamble_us = (t_sym_us * 49U) / 4U;
    const int32_t numerator = static_cast<int32_t>(8U * payload_bytes)
                              - static_cast<int32_t>(4U * sf) + 44;
    const int32_t denominator = static_cast<int32_t>(4U * sf);
    int32_t chunks = 0;
    if (numerator > 0) {
        chunks = (numerator + denominator - 1) / denominator;
    }
    const uint32_t n_payload = 8U + static_cast<uint32_t>(chunks) * 5U;
    return t_preamble_us + n_payload * t_sym_us;
}

inline constexpr bool radio_config_nav_fits_hz(uint16_t bw_khz, uint8_t nav_hz,
                                               uint8_t sf,
                                               uint8_t payload_bytes) {
    if (nav_hz == 0) { return false; }
    const uint32_t slot_us = 1000000U / static_cast<uint32_t>(nav_hz);
    return radio_config_nav_airtime_us(sf, bw_khz, payload_bytes) < slot_us;
}

// Next table row whose nav PLTU ToA fits commanded Hz. current_idx >=
// kRadioConfigTableSize means "not in table" — first fitting row (idx 0).
inline constexpr uint8_t radio_config_next_fit(size_t current_idx) {
    const size_t n = kRadioConfigTableSize;
    if (n == 0) { return kRadioConfigNoIndex; }
    const size_t start = (current_idx >= n) ? 0 : ((current_idx + 1) % n);
    for (size_t k = 0; k < n; ++k) {
        const size_t i = (start + k) % n;
        const auto& e = kRadioConfigTable[i];
        if (radio_config_nav_fits_hz(e.bw_khz, e.nav_rate_hz, e.sf,
                                     kRadioConfigNavPltuBytes)) {
            return static_cast<uint8_t>(i);
        }
    }
    return kRadioConfigNoIndex;
}

inline constexpr uint8_t radio_config_catalog_index(uint16_t bw_khz,
                                                    uint8_t nav_rate_hz,
                                                    uint8_t sf, uint8_t cr,
                                                    uint8_t power_dbm) {
    for (size_t i = 0; i < kRadioConfigTableSize; ++i) {
        const auto& e = kRadioConfigTable[i];
        if (e.bw_khz == bw_khz && e.nav_rate_hz == nav_rate_hz &&
            e.sf == sf && e.cr == cr && e.power_dbm == power_dbm) {
            return static_cast<uint8_t>(i);
        }
    }
    return kRadioConfigNoIndex;
}

static_assert(!radio_config_nav_fits_hz(125, 10, 7, kRadioConfigNavPltuBytes),
              "125/10 SF7 nav PLTU must not fit a 100 ms slot");
static_assert(radio_config_nav_fits_hz(250, 10, 7, kRadioConfigNavPltuBytes),
              "250/10 SF7 nav PLTU must fit a 100 ms slot");
static_assert(radio_config_next_fit(0) == 2, "NAV_PRESET skips 125/10");
static_assert(radio_config_next_fit(1) == 2, "from leftover 125/10 -> 250/10");
static_assert(radio_config_next_fit(2) == 3, "250/10 -> 500/10");
static_assert(radio_config_next_fit(5) == 0, "wrap to 125/5");
static_assert(radio_config_next_fit(kRadioConfigTableSize) == 0,
              "unknown current -> first fitting row");

} // namespace rc

#endif // ROCKETCHIP_RADIO_CONFIG_TABLE_H
