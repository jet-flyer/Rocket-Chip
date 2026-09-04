// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// R-32 R0: increment-only radio rate counters. No TX policy, hold depth,
// or SF/BW change. Dump via CLI `t` and debug `d`.
#ifndef ROCKETCHIP_DIAG_RADIO_RATE_COUNTERS_H
#define ROCKETCHIP_DIAG_RADIO_RATE_COUNTERS_H

#include <stdint.h>

struct RadioRateCounters {
    uint32_t nav_submit_n;
    uint32_t pltu_post_n;
    uint32_t tx_start_n;
    uint32_t tx_done_n;
    uint32_t tx_busy_drop_n;
    uint32_t tx_hold_replace_n;
    uint32_t rx_crc_ok_n;
    uint32_t rx_crc_fail_n;
    uint32_t station_tx_n;
};

extern RadioRateCounters g_radioRateCounters;

const RadioRateCounters* radio_rate_counters();
void radio_rate_counters_dump();

inline void radio_rate_inc_nav_submit(void) {
    g_radioRateCounters.nav_submit_n++;
}
inline void radio_rate_inc_pltu_post(void) {
    g_radioRateCounters.pltu_post_n++;
}
inline void radio_rate_inc_tx_start(void) {
    g_radioRateCounters.tx_start_n++;
}
inline void radio_rate_inc_tx_done(void) {
    g_radioRateCounters.tx_done_n++;
}
inline void radio_rate_inc_tx_busy_drop(void) {
    g_radioRateCounters.tx_busy_drop_n++;
}
inline void radio_rate_inc_tx_hold_replace(void) {
    g_radioRateCounters.tx_hold_replace_n++;
}
inline void radio_rate_inc_rx_crc_ok(void) {
    g_radioRateCounters.rx_crc_ok_n++;
}
inline void radio_rate_inc_rx_crc_fail(void) {
    g_radioRateCounters.rx_crc_fail_n++;
}
inline void radio_rate_inc_station_tx(void) {
    g_radioRateCounters.station_tx_n++;
}

#endif  // ROCKETCHIP_DIAG_RADIO_RATE_COUNTERS_H
