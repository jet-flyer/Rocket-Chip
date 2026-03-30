// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#ifndef ROCKETCHIP_SAFETY_PIO_WATCHDOG_H
#define ROCKETCHIP_SAFETY_PIO_WATCHDOG_H

// PIO-based heartbeat watchdog (IVP-88).
//
// Runs on PIO2, independent of both ARM cores. Monitors TX FIFO for
// heartbeat writes. If ARM cores stop feeding, PIO sets IRQ flag 0.
// No GPIO pins used — fault state read via PIO IRQ register.
//
// Three-layer safety architecture:
//   Layer 1: Smart Path (ESKF + FD)
//   Layer 2: PIO Heartbeat (this module)
//   Layer 3: PIO Backup Timers (IVP-89)

#include <cstdint>

namespace rc {

// Watchdog timeout: ~2 seconds at div=150 (1MHz), 3 cycles/iter
// countdown = timeout_s * 1000000 / 3
static constexpr uint32_t kPioWatchdogCountdown = 666666;

// Initialize the PIO heartbeat watchdog on PIO2.
// No GPIO pin needed — uses PIO IRQ flag 0 for fault signaling.
// Returns false if PIO resources unavailable.
bool pio_watchdog_init();

// Feed the watchdog — write countdown value to PIO TX FIFO.
// Call periodically from main loop.
void pio_watchdog_feed();

// Check if the PIO watchdog has detected a fault.
// Returns true if IRQ flag 0 is set (ARM stopped feeding).
bool pio_watchdog_fault_detected();

// De-initialize (stop PIO SM, release resources).
void pio_watchdog_deinit();

}  // namespace rc

#endif  // ROCKETCHIP_SAFETY_PIO_WATCHDOG_H
