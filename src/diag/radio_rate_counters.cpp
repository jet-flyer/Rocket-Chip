// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// R-32 R0 dump. Increment-only; the RATE dump is boot-relative.
// Pad RX Hz is a separate 2 s sliding span.

#include "diag/radio_rate_counters.h"
#include "rocketchip/rc_log.h"

#include <cstdint>

#ifndef ROCKETCHIP_HOST_TEST
#include "pico/time.h"
#endif

RadioRateCounters g_radioRateCounters = {};

namespace {

// Pad sample step. Not an air slot. 20 steps cover the 2 s span.
constexpr uint32_t kRxWindowMs = 2000U;
constexpr uint32_t kRxMinSpanMs = 1000U;
constexpr uint32_t kRxSampleStepMs = 100U;
constexpr uint8_t kRxSampleCap = 32U;

struct RxSample {
    uint32_t ms;
    uint32_t count;
};

RxSample g_rx_samples[kRxSampleCap];
uint8_t g_rx_n = 0;

void rx_window_clear() {
    g_rx_n = 0;
}

void rx_window_drop_front(uint8_t drop) {
    if (drop == 0U) {
        return;
    }
    if (drop >= g_rx_n) {
        g_rx_n = 0;
        return;
    }
    const uint8_t keep = static_cast<uint8_t>(g_rx_n - drop);
    for (uint8_t i = 0; i < keep; ++i) {
        g_rx_samples[i] = g_rx_samples[static_cast<uint8_t>(i + drop)];
    }
    g_rx_n = keep;
}

void rx_window_push(uint32_t now_ms, uint32_t count) {
    if (g_rx_n > 0U) {
        const uint32_t last_ms = g_rx_samples[g_rx_n - 1U].ms;
        if (now_ms < last_ms) {
            rx_window_clear();
        } else if ((now_ms - last_ms) < kRxSampleStepMs) {
            return;
        }
    }
    if (g_rx_n == kRxSampleCap) {
        rx_window_drop_front(1U);
    }
    g_rx_samples[g_rx_n].ms = now_ms;
    g_rx_samples[g_rx_n].count = count;
    g_rx_n = static_cast<uint8_t>(g_rx_n + 1U);
}

// Newest sample at or before limit. None: 0, the oldest sample we still hold.
uint8_t rx_window_anchor(uint32_t limit_ms) {
    uint8_t anchor = 0;
    for (uint8_t i = 0; i < g_rx_n; ++i) {
        if (g_rx_samples[i].ms <= limit_ms) {
            anchor = i;
        }
    }
    return anchor;
}

uint32_t rx_window_hz10(uint32_t dn, uint32_t dt_ms) {
    return static_cast<uint32_t>((static_cast<uint64_t>(dn) * 10000U) / dt_ms);
}

}  // namespace

void radio_rate_rx_window_reset(uint32_t now_ms) {
    rx_window_clear();
    rx_window_push(now_ms, g_radioRateCounters.rx_crc_ok_n);
}

uint32_t radio_rate_rx_window_hz10(uint32_t now_ms) {
    const uint32_t count = g_radioRateCounters.rx_crc_ok_n;
    rx_window_push(now_ms, count);
    if (g_rx_n == 0U) {
        return kRadioRxWindowBlankHz10;
    }
    const uint32_t limit = (now_ms > kRxWindowMs) ? (now_ms - kRxWindowMs) : 0U;
    rx_window_drop_front(rx_window_anchor(limit));
    const uint32_t dt = now_ms - g_rx_samples[0].ms;
    if (dt < kRxMinSpanMs) {
        return kRadioRxWindowBlankHz10;
    }
    return rx_window_hz10(count - g_rx_samples[0].count, dt);
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
    rc::rc_log("RATE window_ms=%lu (boot-relative)\n",
               (unsigned long)window_ms);
    rc::rc_log("  nav_submit=%lu  pltu_post=%lu\n",
               (unsigned long)c.nav_submit_n,
               (unsigned long)c.pltu_post_n);
    rc::rc_log("  tx_start=%lu  tx_done=%lu\n",
               (unsigned long)c.tx_start_n,
               (unsigned long)c.tx_done_n);
    rc::rc_log("  tx_busy_drop=%lu  tx_hold_replace=%lu\n",
               (unsigned long)c.tx_busy_drop_n,
               (unsigned long)c.tx_hold_replace_n);
    rc::rc_log("  rx_crc_ok=%lu  rx_crc_fail=%lu  station_tx=%lu\n",
               (unsigned long)c.rx_crc_ok_n,
               (unsigned long)c.rx_crc_fail_n,
               (unsigned long)c.station_tx_n);
    if (window_ms < 1000U) {
        rc::rc_log("RATE_HZ window_s<1 (counts only)\n");
        return;
    }
    const uint32_t window_s = window_ms / 1000U;
    rc::rc_log("RATE_HZ window_s=%lu (Hz = n / window_s)\n",
               (unsigned long)window_s);
    rc::rc_log(" ");
    dump_hz_field("nav_submit", c.nav_submit_n, window_s);
    dump_hz_field("pltu_post", c.pltu_post_n, window_s);
    rc::rc_log("\n ");
    dump_hz_field("tx_start", c.tx_start_n, window_s);
    dump_hz_field("tx_done", c.tx_done_n, window_s);
    rc::rc_log("\n ");
    dump_hz_field("rx_crc_ok", c.rx_crc_ok_n, window_s);
    dump_hz_field("station_tx", c.station_tx_n, window_s);
    rc::rc_log("\n");
}
