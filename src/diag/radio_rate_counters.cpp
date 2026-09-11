// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// R-32 R0 dump. Increment-only; window is boot-relative.

#include "diag/radio_rate_counters.h"
#include "rocketchip/rc_log.h"

#include <cstdint>

#ifndef ROCKETCHIP_HOST_TEST
#include "pico/time.h"
#endif

RadioRateCounters g_radioRateCounters = {};

namespace {
constexpr uint32_t kRxWindowMs = 2000U;  // hop plan: not boot-lifetime
uint32_t g_rx_mark_n = 0;
uint32_t g_rx_mark_ms = 0;
}  // namespace

void radio_rate_rx_window_reset(uint32_t now_ms) {
    g_rx_mark_n = g_radioRateCounters.rx_crc_ok_n;
    g_rx_mark_ms = now_ms;
}

uint32_t radio_rate_rx_window_hz10(uint32_t now_ms, uint32_t window_ms) {
    if (window_ms == 0) {
        window_ms = kRxWindowMs;
    }
    if (g_rx_mark_ms == 0 && now_ms > 0) {
        radio_rate_rx_window_reset(now_ms);
    }
    if (now_ms > g_rx_mark_ms && (now_ms - g_rx_mark_ms) >= window_ms) {
        g_rx_mark_n = g_radioRateCounters.rx_crc_ok_n;
        g_rx_mark_ms = now_ms;
    }
    const uint32_t dt = (now_ms > g_rx_mark_ms) ? (now_ms - g_rx_mark_ms) : 0;
    if (dt < 1000U) {
        return 0xFFFFFFFFu;  // caller prints "--"
    }
    const uint32_t dn = g_radioRateCounters.rx_crc_ok_n - g_rx_mark_n;
    return static_cast<uint32_t>((static_cast<uint64_t>(dn) * 10000U) / dt);
}

const RadioRateCounters* radio_rate_counters() {
    return &g_radioRateCounters;
}

static uint32_t window_ms_now() {
#ifndef ROCKETCHIP_HOST_TEST
    return to_ms_since_boot(get_absolute_time());
#else
    return 0;
#endif
}

static void dump_hz_field(const char* name, uint32_t count, uint32_t window_s) {
    const uint32_t hz10 = static_cast<uint32_t>(
        (static_cast<uint64_t>(count) * 10U) / window_s);
    rc::rc_log(" %s=%lu.%lu", name,
               (unsigned long)(hz10 / 10U),
               (unsigned long)(hz10 % 10U));
}

void radio_rate_counters_dump() {
    const RadioRateCounters& c = g_radioRateCounters;
    const uint32_t window_ms = window_ms_now();
    rc::rc_log(
        "RATE: window_ms=%lu nav_submit=%lu pltu_post=%lu tx_start=%lu "
        "tx_done=%lu tx_busy_drop=%lu tx_hold_replace=%lu "
        "rx_crc_ok=%lu rx_crc_fail=%lu station_tx=%lu\n",
        (unsigned long)window_ms,
        (unsigned long)c.nav_submit_n,
        (unsigned long)c.pltu_post_n,
        (unsigned long)c.tx_start_n,
        (unsigned long)c.tx_done_n,
        (unsigned long)c.tx_busy_drop_n,
        (unsigned long)c.tx_hold_replace_n,
        (unsigned long)c.rx_crc_ok_n,
        (unsigned long)c.rx_crc_fail_n,
        (unsigned long)c.station_tx_n);
    if (window_ms < 1000U) {
        rc::rc_log("RATE_HZ: window_s<1 (counts only; Hz = n / window_s)\n");
        return;
    }
    const uint32_t window_s = window_ms / 1000U;
    rc::rc_log("RATE_HZ: window_s=%lu", (unsigned long)window_s);
    dump_hz_field("nav_submit", c.nav_submit_n, window_s);
    dump_hz_field("pltu_post", c.pltu_post_n, window_s);
    dump_hz_field("tx_start", c.tx_start_n, window_s);
    dump_hz_field("tx_done", c.tx_done_n, window_s);
    dump_hz_field("rx_crc_ok", c.rx_crc_ok_n, window_s);
    dump_hz_field("station_tx", c.station_tx_n, window_s);
    rc::rc_log("  (Hz = n / window_s since boot)\n");
}
