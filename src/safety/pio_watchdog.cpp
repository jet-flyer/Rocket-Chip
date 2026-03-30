// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#include "pio_watchdog.h"

#ifndef ROCKETCHIP_HOST_TEST

#include "hardware/pio.h"
#include "heartbeat_watchdog.pio.h"

namespace rc {

static PIO g_pio = nullptr;
static uint32_t g_sm = 0;
static uint32_t g_offset = 0;
static bool g_initialized = false;

bool pio_watchdog_init() {
    if (g_initialized) {
        return true;
    }

    // Use PIO2 — dedicated to safety (PIO0 = WS2812, PIO1 = reserved)
    g_pio = pio2;

    int sm = pio_claim_unused_sm(g_pio, false);
    if (sm < 0) {
        return false;
    }
    g_sm = static_cast<uint32_t>(sm);

    if (!pio_can_add_program(g_pio, &heartbeat_watchdog_program)) {
        pio_sm_unclaim(g_pio, g_sm);
        return false;
    }
    g_offset = pio_add_program(g_pio, &heartbeat_watchdog_program);

    // Initialize — no GPIO pins, uses IRQ flag 0
    heartbeat_watchdog_program_init(g_pio, g_sm, g_offset);

    // Send initial countdown to start the watchdog
    pio_sm_put_blocking(g_pio, g_sm, kPioWatchdogCountdown);

    g_initialized = true;
    return true;
}

void pio_watchdog_feed() {
    if (!g_initialized) {
        return;
    }
    // Non-blocking put — if FIFO is full, oldest value still valid
    pio_sm_put(g_pio, g_sm, kPioWatchdogCountdown);
}

bool pio_watchdog_fault_detected() {
    if (!g_initialized) {
        return false;
    }
    return pio_interrupt_get(g_pio, 0);
}

void pio_watchdog_deinit() {
    if (!g_initialized) {
        return;
    }
    pio_sm_set_enabled(g_pio, g_sm, false);
    pio_remove_program(g_pio, &heartbeat_watchdog_program, g_offset);
    pio_sm_unclaim(g_pio, g_sm);
    g_initialized = false;
}

}  // namespace rc

#else  // ROCKETCHIP_HOST_TEST

namespace rc {

static bool g_stub_fault = false;

bool pio_watchdog_init() { return true; }
void pio_watchdog_feed() {}
bool pio_watchdog_fault_detected() { return g_stub_fault; }
void pio_watchdog_deinit() {}

}  // namespace rc

#endif
