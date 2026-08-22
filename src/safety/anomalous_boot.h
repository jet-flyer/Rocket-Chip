// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Boot gate: probably mid-flight vs on-pad (veto + 2-of-N). Bias false-positive
// — a pad false alarm is operator intervention; a miss re-zeros baro in flight.
// Brownout is a separate health latch (physical inspection), not this verdict.
// POWMAN_CHIP_RESET: RP2350 datasheet §6 / hardware/regs/powman.h.
//============================================================================
#ifndef ROCKETCHIP_SAFETY_ANOMALOUS_BOOT_H
#define ROCKETCHIP_SAFETY_ANOMALOUS_BOOT_H

#include <stdint.h>

namespace rc {

enum class BootVerdict : uint8_t {
    kProbablyOnPad     = 0,  // No mid-flight signals; normal boot proceeds.
    kProbablyMidFlight = 1,  // Sentinel and/or 2+ corroborating signals fired.
};

// Snapshot of the raw signals consulted by the gate, captured at boot before
// any sensor / driver / AO state is initialized. Used by health monitor and
// CLI to surface what the gate observed.
struct BootSignals {
    uint32_t powman_chip_reset;   // POWMAN_CHIP_RESET raw register value
    bool     had_por;             // Power-on reset (clean cold boot)
    bool     had_bor;             // Brown-out reset — sticky, gates health-monitor brownout latch
    bool     had_run_low;         // RUN pin pulled low (operator reset button or snagged cable)
    bool     had_any_non_por;     // Any non-POR reset cause bit set
    bool     sentinel_was_set;    // flight_in_progress sentinel survived
    uint32_t prior_uptime_ms;     // AON timer reading at boot (zero if timer reset)
    BootVerdict verdict;          // Computed verdict
};

// Read the POWMAN_CHIP_RESET register, AON timer, and sentinel; compute the
// verdict. Side-effect: clears the sentinel after read so subsequent boots
// don't see it.
//
// MUST be called exactly once, very early in main() — before any code that
// might side-effect the sentinel (e.g., FD init resets phase, AO_Logger
// would write to flash).
void anomalous_boot_init();

// Accessors for the snapshot. Valid after anomalous_boot_init() runs.
const BootSignals& anomalous_boot_signals();
BootVerdict anomalous_boot_verdict();
bool anomalous_boot_brownout_detected();

// Human-readable verdict for banner / log output. Returns a static string.
const char* anomalous_boot_verdict_name();

} // namespace rc

#endif // ROCKETCHIP_SAFETY_ANOMALOUS_BOOT_H
