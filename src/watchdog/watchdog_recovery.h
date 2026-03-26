// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file watchdog_recovery.h
 * @brief Watchdog recovery policy (IVP-66)
 *
 * Persists flight state and reboot history across watchdog resets using
 * RP2350 scratch registers. Implements safe-mode detection (>3 rapid
 * reboots), LAUNCH_ABORT flag, and ESKF failure backoff.
 *
 * Scratch register layout (RP2350):
 *   scratch[0] — watchdog sentinel (existing, 0x52435754 "RCWT")
 *   scratch[1] — recovery data word: flight_state[7:0] | tick_fn_id[15:8] | reserved[31:16]
 *   scratch[2] — reboot counter (lower 16 bits) | magic validator (upper 16 bits)
 *   scratch[3] — reserved for future use
 *   scratch[4-7] — used by bootrom (do not touch)
 *
 * The magic validator in scratch[2] upper half (0x5243 = "RC") ensures we
 * don't misinterpret random SRAM contents after a cold boot as valid
 * recovery data.
 */

#ifndef ROCKETCHIP_WATCHDOG_RECOVERY_H
#define ROCKETCHIP_WATCHDOG_RECOVERY_H

#include <cstdint>

namespace rc {

// Magic value in upper 16 bits of scratch[2] to validate recovery data
static constexpr uint16_t kRecoveryMagic = 0x5243;  // "RC"

// Reboot threshold for safe-mode lockout.
// Source: ArduPilot AP_BoardConfig boot_count check (3 rapid reboots → safe mode).
// If reboot counter exceeds this within a session (without a clean POR reset),
// the system enters safe mode and sets LAUNCH_ABORT.
static constexpr uint8_t kSafeModeRebootThreshold = 3;

// Maximum ESKF consecutive init→diverge cycles before disabling ESKF.
// Source: ArduPilot EKF3 `coreSetupRequired` pattern — after repeated
// failures the EKF stops trying until explicitly re-enabled.
static constexpr uint8_t kEskfMaxFailCycles = 5;

// Tick function IDs for crash diagnostics — identifies which tick was
// running when the watchdog fired.
enum class TickFnId : uint8_t {
    kInit           = 0,
    kHeartbeat      = 1,
    kWatchdog       = 2,
    kEskf           = 3,
    kFlightDirector = 4,
    kLogging        = 5,
    kRadio          = 6,
    kMavlink        = 7,
    kCli            = 8,
    kSleep          = 9,
};

// Recovery state read from scratch registers at boot.
struct RecoveryState {
    uint8_t  flight_phase;       // Flight phase at time of crash (0 = IDLE)
    TickFnId last_tick_fn;       // Which tick function was running
    uint16_t reboot_count;       // Cumulative watchdog reboot count
    bool     valid;              // True if scratch data had valid magic
    bool     safe_mode;          // True if reboot_count > threshold
    bool     launch_abort;       // True if safe_mode triggered LAUNCH_ABORT
};

// Runtime recovery state — call watchdog_recovery_init() at boot,
// then watchdog_recovery_update_scratch() periodically.
struct WatchdogRecovery {
    RecoveryState boot_state;    // State recovered from scratch at boot
    uint8_t  current_flight_phase;
    TickFnId current_tick_fn;
    uint8_t  eskf_fail_count;    // Consecutive ESKF init→diverge cycles
    bool     eskf_disabled;      // True when fail_count >= kEskfMaxFailCycles
    bool     launch_abort;       // Latched LAUNCH_ABORT flag
    bool     launch_abort_acked; // CLI acknowledge clears the flag
};

// Initialize recovery system. Reads scratch registers, detects safe mode,
// increments reboot counter, writes updated scratch back.
// Call early in init_hardware(), after check_watchdog_reboot().
//
// @param watchdog_reboot True if this boot was caused by a watchdog reset.
void watchdog_recovery_init(WatchdogRecovery* recovery, bool watchdog_reboot);

// Update scratch registers with current state. Call from watchdog_kick_tick()
// so the data is fresh if the next kick is missed and the watchdog fires.
void watchdog_recovery_update_scratch(const WatchdogRecovery* recovery);

// Record an ESKF init→diverge cycle. Returns true if ESKF should be disabled
// (fail count reached threshold).
bool watchdog_recovery_eskf_failed(WatchdogRecovery* recovery);

// Re-enable ESKF after CLI command. Resets fail counter.
void watchdog_recovery_eskf_reenable(WatchdogRecovery* recovery);

// Acknowledge LAUNCH_ABORT via CLI. Clears the flag for this session.
void watchdog_recovery_ack_launch_abort(WatchdogRecovery* recovery);

// Clear reboot counter (e.g., after a clean session with no watchdog resets).
// Writes zeros to scratch registers.
void watchdog_recovery_clear(WatchdogRecovery* recovery);

// ============================================================================
// Low-level scratch register helpers (also used by host tests)
// ============================================================================

// Pack recovery data into scratch[1] word.
inline uint32_t recovery_pack_scratch1(uint8_t flight_phase, TickFnId tick_fn) {
    return static_cast<uint32_t>(flight_phase) |
           (static_cast<uint32_t>(tick_fn) << 8);
}

// Pack reboot counter + magic into scratch[2] word.
inline uint32_t recovery_pack_scratch2(uint16_t reboot_count) {
    return static_cast<uint32_t>(reboot_count) |
           (static_cast<uint32_t>(kRecoveryMagic) << 16);
}

// Unpack flight_phase from scratch[1].
inline uint8_t recovery_unpack_flight_phase(uint32_t scratch1) {
    return static_cast<uint8_t>(scratch1 & 0xFF);
}

// Unpack tick_fn from scratch[1].
inline TickFnId recovery_unpack_tick_fn(uint32_t scratch1) {
    return static_cast<TickFnId>((scratch1 >> 8) & 0xFF);
}

// Unpack reboot_count from scratch[2].
inline uint16_t recovery_unpack_reboot_count(uint32_t scratch2) {
    return static_cast<uint16_t>(scratch2 & 0xFFFF);
}

// Validate magic in scratch[2].
inline bool recovery_validate_magic(uint32_t scratch2) {
    return (scratch2 >> 16) == kRecoveryMagic;
}

} // namespace rc

#endif // ROCKETCHIP_WATCHDOG_RECOVERY_H
