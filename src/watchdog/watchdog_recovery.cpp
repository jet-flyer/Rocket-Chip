// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file watchdog_recovery.cpp
 * @brief Watchdog recovery policy implementation (IVP-66)
 */

#include "watchdog_recovery.h"

#ifdef ROCKETCHIP_HOST_TEST
// Host test: scratch registers are simulated in-memory
static uint32_t g_fake_scratch[8] = {};
static uint32_t scratch_read(uint32_t idx) { return g_fake_scratch[idx]; }
static void scratch_write(uint32_t idx, uint32_t val) { g_fake_scratch[idx] = val; }

// Test helper: direct access to fake scratch registers
extern "C" uint32_t* watchdog_recovery_test_scratch() { return g_fake_scratch; }
#else
// Target: real RP2350 scratch registers
#include "hardware/watchdog.h"
static uint32_t scratch_read(uint32_t idx) { return watchdog_hw->scratch[idx]; }
static void scratch_write(uint32_t idx, uint32_t val) { watchdog_hw->scratch[idx] = val; }
#endif

namespace rc {

void watchdog_recovery_init(WatchdogRecovery* recovery, bool watchdog_reboot) {
    // Zero the runtime struct
    *recovery = {};

    // Read scratch registers
    uint32_t s1 = scratch_read(1);
    uint32_t s2 = scratch_read(2);

    // Check if recovery data is valid (magic present in scratch[2])
    bool valid = recovery_validate_magic(s2);

    if (watchdog_reboot && valid) {
        // Genuine watchdog reboot with valid recovery data
        recovery->boot_state.flight_phase = recovery_unpack_flight_phase(s1);
        recovery->boot_state.last_tick_fn = recovery_unpack_tick_fn(s1);
        recovery->boot_state.reboot_count = recovery_unpack_reboot_count(s2);
        recovery->boot_state.valid = true;

        // Increment reboot counter
        uint16_t newCount = recovery->boot_state.reboot_count;
        if (newCount < UINT16_MAX) {
            newCount++;
        }
        recovery->boot_state.reboot_count = newCount;

        // Safe mode: too many rapid reboots
        if (newCount > kSafeModeRebootThreshold) {
            recovery->boot_state.safe_mode = true;
            recovery->boot_state.launch_abort = true;
            recovery->launch_abort = true;
        }
    } else if (watchdog_reboot && !valid) {
        // Watchdog reboot but no valid recovery data — first watchdog in session
        recovery->boot_state.valid = false;
        recovery->boot_state.reboot_count = 1;
    } else {
        // Clean boot (POR, SWD, picotool) — reset everything
        recovery->boot_state.valid = false;
        recovery->boot_state.reboot_count = 0;
    }

    // Initialize runtime state
    recovery->current_flight_phase = 0;  // IDLE
    recovery->current_tick_fn = TickFnId::kInit;
    recovery->eskf_fail_count = 0;
    recovery->eskf_disabled = false;

    // Write initial recovery data to scratch registers for this session
    watchdog_recovery_update_scratch(recovery);
}

void watchdog_recovery_update_scratch(const WatchdogRecovery* recovery) {
    scratch_write(1, recovery_pack_scratch1(
        recovery->current_flight_phase,
        recovery->current_tick_fn));
    scratch_write(2, recovery_pack_scratch2(
        recovery->boot_state.reboot_count));
}

bool watchdog_recovery_eskf_failed(WatchdogRecovery* recovery) {
    if (recovery->eskf_fail_count < UINT8_MAX) {
        recovery->eskf_fail_count++;
    }
    if (recovery->eskf_fail_count >= kEskfMaxFailCycles) {
        recovery->eskf_disabled = true;
        return true;
    }
    return false;
}

void watchdog_recovery_eskf_reenable(WatchdogRecovery* recovery) {
    recovery->eskf_fail_count = 0;
    recovery->eskf_disabled = false;
}

void watchdog_recovery_ack_launch_abort(WatchdogRecovery* recovery) {
    recovery->launch_abort = false;
    recovery->launch_abort_acked = true;
}

void watchdog_recovery_clear(WatchdogRecovery* recovery) {
    recovery->boot_state.reboot_count = 0;
    recovery->boot_state.safe_mode = false;
    recovery->boot_state.launch_abort = false;
    recovery->launch_abort = false;
    recovery->launch_abort_acked = false;
    recovery->eskf_fail_count = 0;
    recovery->eskf_disabled = false;

    scratch_write(1, 0);
    scratch_write(2, 0);
}

} // namespace rc
