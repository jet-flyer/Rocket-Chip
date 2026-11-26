// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// RX packet quality stats (see rx_quality_stats.h).

#include "diag/rx_quality_stats.h"

namespace rc {

void rx_qual_add(RxQualStat& s, int16_t v) {
    if (s.n == UINT32_MAX) {
        return;
    }
    if ((s.n == 0U) || (v < s.min)) {
        s.min = v;
    }
    if ((s.n == 0U) || (v > s.max)) {
        s.max = v;
    }
    s.sum += static_cast<int64_t>(v);
    s.n++;
}

int16_t rx_qual_mean(const RxQualStat& s) {
    if (s.n == 0U) {
        return 0;
    }
    const int64_t n = static_cast<int64_t>(s.n);
    const int64_t half = n / 2;
    const int64_t q = (s.sum >= 0) ? ((s.sum + half) / n) : ((s.sum - half) / n);
    return static_cast<int16_t>(q);
}

void rx_quality_note(RxQuality& q, int16_t rssi_dbm, int8_t snr_db) {
    rx_qual_add(q.rssi, rssi_dbm);
    rx_qual_add(q.snr, static_cast<int16_t>(snr_db));
}

} // namespace rc
