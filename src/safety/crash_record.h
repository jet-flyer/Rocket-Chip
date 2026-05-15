// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// crash_record — preserved-SRAM record for fault-handler capture-and-reset.
//
// R-3 (audit 2026-05-07): replaces the prior memmanage_fault_handler() halt-
// forever pattern with industry-standard capture-state-then-reset. The
// handler writes a record into the .uninitialized_data section (NOLOAD,
// survives reset), then triggers NVIC_SystemReset(). On the next boot,
// health_monitor_init() consumes the record and sets a HealthCritical
// bit so the existing safe-mode / FAULT-health-state pivot owns the
// recovery path.
//
// References:
//   - ArduPilot watchdog crash dump: https://ardupilot.org/copter/docs/common-watchdog.html
//   - Memfault firmware watchdog best practices:
//     https://interrupt.memfault.com/blog/firmware-watchdog-best-practices
//============================================================================
#ifndef ROCKETCHIP_SAFETY_CRASH_RECORD_H
#define ROCKETCHIP_SAFETY_CRASH_RECORD_H

#include <stdint.h>

namespace rc {

// Magic constants identifying record validity + reason. The handler writes
// kCrashRecordMagic + a reason code; the post-boot consumer clears the
// magic so a clean boot doesn't re-report.
//
// Magic is randomly-chosen but stable; any bit-flipped neighbor in
// .uninitialized_data won't match by accident. The reason code distinguishes
// hardfault types so future post-mortem tooling can categorize them.
static constexpr uint32_t kCrashRecordMagic = 0xC0DE'FA02U;  // "CODE FA02" mnemonic

enum CrashReason : uint32_t {
    kCrashReasonNone           = 0,
    kCrashReasonMemManage      = 1,  // MPU stack guard hit (memmanage_fault_handler)
    kCrashReasonMpuConfigFail  = 2,  // R-4: mpu_setup_stack_guard() couldn't configure
    kCrashReasonCore1BootWait  = 3,  // R-1: Core 1 boot-wait loop exceeded vehicle-path bound
    // Reserved for future fault categories (hardfault, busfault, usagefault).
};

// Crash record laid out in .uninitialized_data — survives NVIC_SystemReset().
//
// Layout is small + fixed so the handler (which must not use stack) can write
// it via direct stores. Fields populated by crash_record_capture(); fields
// consumed by crash_record_consume_prior().
struct CrashRecord {
    uint32_t magic;        // kCrashRecordMagic iff a record is present
    uint32_t reason;       // CrashReason enum
    uint32_t cfsr;         // ARMv8-M Configurable Fault Status Register (0xE000ED28)
    uint32_t hfsr;         // ARMv8-M HardFault Status Register (0xE000ED2C)
    uint32_t stacked_pc;   // PC at fault (from exception stack frame)
    uint32_t stacked_lr;   // LR at fault (from exception stack frame)
    uint32_t reserved[2];  // pad to 32 bytes for future expansion
};

// Capture fault state and trigger reset. Used in NON-handler contexts where
// a function call is safe (e.g., R-1 Core 1 boot-wait timeout, R-4 MPU
// config failure). For the actual fault handler, see memmanage_fault_handler
// in fault_protection.cpp — it inlines capture+reset to avoid the
// fault-on-fault → lockup risk a C-level call would create when the failing
// stack is barely above the MPU guard.
//
// Reads CFSR/HFSR from fixed SCB addresses; stacked_pc/stacked_lr come from
// caller. Triggers NVIC_SystemReset() and does not return.
[[noreturn]] void crash_record_capture(CrashReason reason,
                                       uint32_t stacked_pc,
                                       uint32_t stacked_lr);

// Direct extern access to the .uninitialized_data record storage. Used by
// memmanage_fault_handler() to bypass the function-call path entirely (see
// crash_record_capture comment above for why a function call is unsafe in
// the fault handler). The storage definition is in crash_record.cpp.
extern CrashRecord g_crash_record;

// Consume any prior-boot crash record. Called once from health_monitor_init().
// Returns true and fills *out if a valid record was present (clears the
// magic so the next clean boot doesn't re-report). Returns false otherwise.
bool crash_record_consume_prior(CrashRecord* out);

// ============================================================================
// Flight-in-progress sentinel
//
// Independent of the crash record above — a single magic word in
// .uninitialized_data that is set when the system transitions kIdle -> kArmed
// and cleared when the system reaches kLanded with no abort latched. If the
// sentinel is present at boot, the firmware was armed/airborne when something
// reset it (whether internally-issued or external like brownout / RUN-pin /
// snagged button). This is the primary "PROBABLY_MID_FLIGHT" signal for the
// anomalous-boot confidence gate in B.4 of the fault-recovery architecture
// plan (2026-05-14).
//
// SRAM retention across reset is guaranteed above the ~1.62V brownout
// threshold (RP2350 datasheet §6 Power Manager). A BOR reset (deeper power
// loss) clears .uninitialized_data — in that case the sentinel disappears
// but the brownout latch in health monitor catches it independently
// (POWMAN_CHIP_RESET.HAD_BOR is sticky across BOR specifically).
// ============================================================================
static constexpr uint32_t kFlightInProgressMagic = 0xF11617A0U;  // "FlIght AO" mnemonic

void flight_in_progress_set();    // Called on kIdle -> kArmed transition.
void flight_in_progress_clear();  // Called on safe LANDED entry.
bool flight_in_progress_was_set();  // Called once at boot. Clears on read.

} // namespace rc

#endif // ROCKETCHIP_SAFETY_CRASH_RECORD_H
