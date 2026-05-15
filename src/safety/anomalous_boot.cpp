// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// anomalous_boot — confidence gate at boot for "are we probably mid-flight?"
// See anomalous_boot.h for design rationale.

#include "safety/anomalous_boot.h"
#include "safety/crash_record.h"
#include "hardware/structs/powman.h"
#include "hardware/regs/powman.h"

namespace rc {

// Threshold above which a non-zero AON-timer reading at boot is treated as
// evidence of "we'd been running for a while" rather than residual noise.
// 10 seconds chosen so a typical fresh-boot calibration cycle (~3-5s through
// banner + hardware status + first sample) cannot accidentally cross it.
static constexpr uint32_t kAonTimerPriorUptimeThresholdMs = 10000U;

// Singleton snapshot — captured once by anomalous_boot_init(), accessed by
// downstream consumers (health monitor, CLI, main.cpp baro-zero gate).
static BootSignals g_signals;
static bool        g_initialized = false;

// Local helpers ===============================================================

// POR is bit 16, BOR is bit 17. The full "any non-POR cause" mask is all the
// known cause-bits in POWMAN_CHIP_RESET except HAD_POR + the non-cause bits
// (DOUBLE_TAP at bit 0, RESCUE_FLAG at bit 4) — see RP2350 datasheet §6.4 +
// SDK hardware/regs/powman.h:486-735.
static constexpr uint32_t kAllCauseBitsMask =
      POWMAN_CHIP_RESET_HAD_WATCHDOG_RESET_RSM_BITS
    | POWMAN_CHIP_RESET_HAD_HZD_SYS_RESET_REQ_BITS
    | POWMAN_CHIP_RESET_HAD_GLITCH_DETECT_BITS
    | POWMAN_CHIP_RESET_HAD_SWCORE_PD_BITS
    | POWMAN_CHIP_RESET_HAD_WATCHDOG_RESET_SWCORE_BITS
    | POWMAN_CHIP_RESET_HAD_WATCHDOG_RESET_POWMAN_BITS
    | POWMAN_CHIP_RESET_HAD_WATCHDOG_RESET_POWMAN_ASYNC_BITS
    | POWMAN_CHIP_RESET_HAD_RESCUE_BITS
    | POWMAN_CHIP_RESET_HAD_DP_RESET_REQ_BITS
    | POWMAN_CHIP_RESET_HAD_RUN_LOW_BITS
    | POWMAN_CHIP_RESET_HAD_BOR_BITS
    | POWMAN_CHIP_RESET_HAD_POR_BITS;

static constexpr uint32_t kNonPorCauseBitsMask =
    kAllCauseBitsMask & ~static_cast<uint32_t>(POWMAN_CHIP_RESET_HAD_POR_BITS);

// AON timer prior-uptime read — DEFERRED to commit (b).
//
// The RP2350 POWMAN timer survives non-power-cycling resets (watchdog-RSM,
// hazard-DP, glitch-detect, SWcore-PD, watchdog-SWcore) but is reset by
// BOR/POR/watchdog-powman/rescue/DP-reset-req/RUN-low per SDK header
// hardware/regs/powman.h. Wiring it as a corroborator signal requires:
//   - Adding `hardware_powman` / `pico_aon_timer` to target_link_libraries
//   - Explicit timer-start at boot so the prior-uptime reading at next boot
//     has a meaningful zero (otherwise an uninitialized timer reading is
//     ambiguous with a reset-cleared timer)
// For commit (a) we leave prior_uptime_ms = 0 (signal absent); the verdict
// uses sentinel + reset-cause signals which are load-bearing on their own.
// Commit (b) wires the AON timer once the rest of the fault-recovery
// architecture is in place and the corroborator signal earns its keep.
static uint32_t read_prior_uptime_ms() {
    return 0U;
}

// Compute the verdict from the captured signals. Pure function for testability.
static BootVerdict compute_verdict(const BootSignals& s) {
    // Signal #1: sentinel was set. Alone triggers PROBABLY_MID_FLIGHT.
    if (s.sentinel_was_set) {
        return BootVerdict::kProbablyMidFlight;
    }
    // Signals #2-3-4 (non-POR cause, AON-timer prior-uptime, cut-off log).
    // Each alone needs one corroborator. Cut-off log signal is not yet
    // wired (deferred to commit (b)); for now we count what we have.
    int corroborators = 0;
    if (s.had_any_non_por) {
        corroborators++;
    }
    if (s.prior_uptime_ms >= kAonTimerPriorUptimeThresholdMs) {
        corroborators++;
    }
    if (corroborators >= 2) {
        return BootVerdict::kProbablyMidFlight;
    }
    return BootVerdict::kProbablyOnPad;
}

// Public API ==================================================================

void anomalous_boot_init() {
    if (g_initialized) {
        return;
    }

    // Snapshot the chip reset register before anything else touches it.
    g_signals.powman_chip_reset = powman_hw->chip_reset;

    g_signals.had_por =
        (g_signals.powman_chip_reset & POWMAN_CHIP_RESET_HAD_POR_BITS) != 0U;
    g_signals.had_bor =
        (g_signals.powman_chip_reset & POWMAN_CHIP_RESET_HAD_BOR_BITS) != 0U;
    g_signals.had_run_low =
        (g_signals.powman_chip_reset & POWMAN_CHIP_RESET_HAD_RUN_LOW_BITS) != 0U;
    g_signals.had_any_non_por =
        (g_signals.powman_chip_reset & kNonPorCauseBitsMask) != 0U;

    // Read sentinel BEFORE flight_in_progress_clear or any other consumer
    // can race. flight_in_progress_was_set() clears the sentinel on read.
    g_signals.sentinel_was_set = flight_in_progress_was_set();

    // AON timer reading (best-effort — may be zero on most reset types).
    g_signals.prior_uptime_ms = read_prior_uptime_ms();

    g_signals.verdict = compute_verdict(g_signals);

    g_initialized = true;
}

const BootSignals& anomalous_boot_signals() {
    return g_signals;
}

BootVerdict anomalous_boot_verdict() {
    return g_signals.verdict;
}

bool anomalous_boot_brownout_detected() {
    return g_signals.had_bor;
}

const char* anomalous_boot_verdict_name() {
    switch (g_signals.verdict) {
        case BootVerdict::kProbablyOnPad:     return "PROBABLY_ON_PAD";
        case BootVerdict::kProbablyMidFlight: return "PROBABLY_MID_FLIGHT";
    }
    return "UNKNOWN";
}

} // namespace rc
