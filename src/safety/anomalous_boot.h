// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// anomalous_boot — confidence gate at boot for "are we probably mid-flight?"
//
// Background: per the in-flight fault recovery architecture plan
// (council round 2 unanimous 2026-05-14), the firmware must detect at boot
// whether a reset happened mid-flight. Even though the firmware never
// internally issues a reset in flight (B.1: zero in-flight reset), external
// causes (brownout, snagged reset button, ESD, USB-power glitch) can still
// reboot the chip. A mid-flight reboot followed by the normal "fresh on the
// pad" boot sequence would re-zero the baro at altitude and re-init flight
// phase to kIdle — catastrophic when the rocket is actually descending under
// parachute at +500 m.
//
// Approach: read multiple signals at boot, classify "PROBABLY_MID_FLIGHT"
// vs "PROBABLY_ON_PAD" using veto + 2-of-N corroborator logic. Bias toward
// false-positive (refuse to act as fresh pad on ambiguous evidence) — cost
// of false-positive is one operator intervention; cost of false-negative is
// mission loss.
//
// Brownout has its own SEPARATE handling (kHealthCriticalPriorBrownout
// latch in health monitor) regardless of the mid-flight verdict: brownout
// can happen on the pad too (battery swap, ESD), and the right response is
// always "physical inspection required before continuing." That latch is
// not part of this module's PROBABLY_MID_FLIGHT verdict; it's surfaced via
// the brownout_detected() accessor for the health monitor's use.
//
// References:
//   - C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn.md  (plan B.4)
//   - RP2350 datasheet §6 Power Manager (POWMAN_CHIP_RESET register)
//   - SDK header: hardware/regs/powman.h
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
