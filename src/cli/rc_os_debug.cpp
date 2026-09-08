// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Debug sub-menu reads (ESKF live).

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

bool debug_eskf_live_poll() {
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
