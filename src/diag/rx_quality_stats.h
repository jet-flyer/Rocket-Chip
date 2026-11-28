// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// RX packet quality stats (FSK open item C). Running count, min, mean and
// max of packet RSSI and SNR. AO_Radio keeps one set for packets with the
// chip CRC good and one set for packets with the SX1276 PayloadCrcErr
// flag, so the two can be compared on 'w'.
// Pure code: no Pico SDK, so the host tests can use it.
#ifndef ROCKETCHIP_DIAG_RX_QUALITY_STATS_H
#define ROCKETCHIP_DIAG_RX_QUALITY_STATS_H

#include <stdint.h>

namespace rc {

struct RxQualStat {
    uint32_t n;
    int64_t sum;
    int16_t min;
    int16_t max;
};

struct RxQuality {
    RxQualStat rssi;  // dBm
    RxQualStat snr;   // dB
};

// Add one value. Stops at n == UINT32_MAX (the sum stays in range).
void rx_qual_add(RxQualStat& s, int16_t v);

// Mean rounded to the nearest integer (half away from zero). 0 if n is 0.
int16_t rx_qual_mean(const RxQualStat& s);

void rx_quality_note(RxQuality& q, int16_t rssi_dbm, int8_t snr_db);

} // namespace rc

#endif // ROCKETCHIP_DIAG_RX_QUALITY_STATS_H
