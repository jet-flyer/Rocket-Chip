// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Debug sub-menu reads (ESKF live). LED test / digit-RF omitted from this
// sitting (ROCKETCHIP_DEV_MODE + probe test_mode later).

#include "cli/rc_os_debug.h"
#include "cli/rc_os_commands.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "rocketchip/rc_log.h"

static bool g_eskfLiveActive = false;
static uint32_t g_eskfLiveLastPrintUs = 0;
static constexpr uint32_t kEskfLivePeriodUs = 1000000;

void cli_debug_start_eskf_live() {
    g_eskfLiveActive = true;
    g_eskfLiveLastPrintUs = time_us_32();
    rc::rc_log("\n--- ESKF live (1Hz) --- any key to stop ---\n");
    cli_print_eskf_live();
}

bool dev_eskf_live_poll() {
    if (!g_eskfLiveActive) { return false; }
    int c = getchar_timeout_us(0);
    if (c != PICO_ERROR_TIMEOUT) {
        g_eskfLiveActive = false;
        rc::rc_log("\n--- ESKF live stopped ---\n");
        rc::rc_log("[debug] ");
    } else {
        uint32_t now_us = time_us_32();
        if (now_us - g_eskfLiveLastPrintUs >= kEskfLivePeriodUs) {
            g_eskfLiveLastPrintUs = now_us;
            cli_print_eskf_live();
        }
    }
    return true;
}

// R-25-exec step 5 (2026-05-13): dev_replay_poll() + parse_and_inject()
// + s_replayBuf DELETED per council amendment #4. Vehicle CSV-streamer
// replay retired; algorithmic coverage moves to host-side
// scripts/replay_harness_host.py (same ESKF code, host workstation,
// no on-target attack surface, no flash cost).

// R-25-exec step 6 (2026-05-13): dev_station_replay_poll() +
// station_parse_and_inject() + hex_nibble() + s_stReplayBuf DELETED
// per council amendment #4. Station hex-injection replay retired;
// coverage moves host-side (rewrite tracked in step 7).
