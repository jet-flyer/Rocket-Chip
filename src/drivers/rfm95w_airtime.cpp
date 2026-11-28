// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// RFM95W (SX1276) LoRa time on air, and the LoRa radio port for the
// modulation-agnostic airtime query (rocketchip/radio_config_airtime.h).
// Pure integer math, no SPI or SDK calls, so the host tests link it too.
// rfm95w_airtime_us moved from rfm95w.cpp without change (R-37).
// Prior Art: SX1276 datasheet Rev. 4, March 2015, section 4.1.1.6.

#include "rfm95w.h"
#include "rocketchip/radio_config_airtime.h"

#include <stddef.h>

uint32_t rfm95w_airtime_us(uint8_t sf, uint16_t bw_khz, uint8_t payload_bytes) {
    // SX1276 datasheet §4.1.1.6.
    // Assumes: explicit header (IH=0), CRC on (CRC=1), preamble 8 symbols,
    // low-data-rate-optimize OFF (valid for SF<=10 at BW>=125).
    // T_symbol = 2^SF / BW  (in microseconds if BW is Hz; use ratio below).
    // Keep integer math for host-test / deterministic build.
    if (bw_khz == 0) { bw_khz = 125; }  // sanity
    if (sf < 7)    { sf = 7; }
    if (sf > 12)   { sf = 12; }

    // T_symbol in microseconds = (1 << sf) * 1000 / bw_khz
    uint32_t t_sym_us = (static_cast<uint32_t>(1U) << sf) * 1000U / bw_khz;

    // Preamble: (n_preamble + 4.25) * T_sym ≈ 12.25 * T_sym for 8-symbol preamble.
    // Scale by 4 to keep integer: 4 * 12.25 = 49.
    uint32_t t_preamble_us = (t_sym_us * 49U) / 4U;

    // Payload-symbol count formula (explicit header, CRC on, LDRO off):
    //   n_payload = 8 + max(ceil((8*PL - 4*SF + 28 + 16*CRC - 20*IH) / (4*SF)) * (CR+4), 0)
    // With CRC=1, IH=0, CR=5 (4/5) here:
    //   n_payload = 8 + max(ceil((8*PL - 4*SF + 44) / (4*SF)) * 5, 0)
    int32_t numerator = static_cast<int32_t>(8U * payload_bytes)
                      - static_cast<int32_t>(4U * sf) + 44;
    int32_t denominator = static_cast<int32_t>(4U * sf);
    int32_t chunks = 0;
    if (numerator > 0) {
        chunks = (numerator + denominator - 1) / denominator;  // ceil
    }
    uint32_t n_payload_symbols = 8U + static_cast<uint32_t>(chunks) * 5U;

    uint32_t t_payload_us = n_payload_symbols * t_sym_us;
    return t_preamble_us + t_payload_us;
}

namespace rc {

namespace {

// rfm95w_airtime_us assumes CR 4/5. Every rate-table preset must use it.
constexpr bool all_presets_cr_4_5() {
    for (size_t i = 0; i < kRadioConfigTableSize; ++i) {
        if (kRadioConfigTable[i].cr != 5U) {
            return false;
        }
    }
    return true;
}
static_assert(all_presets_cr_4_5(),
              "rfm95w_airtime_us assumes CR 4/5: pass the preset CR first");

}  // namespace

uint32_t radio_preset_airtime_us(const RadioConfigEntry& preset,
                                 uint8_t frame_octets) {
    return rfm95w_airtime_us(preset.sf, preset.bw_khz, frame_octets);
}

}  // namespace rc
