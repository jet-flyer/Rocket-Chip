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

// Capture fault state and trigger reset. Called from memmanage_fault_handler()
// (and R-4: from mpu_setup_stack_guard() failure path).
//
// Must not use stack — the caller is already in a no-stack-safe context.
// Reads CFSR/HFSR from fixed SCB addresses; stacked_pc/stacked_lr come from
// caller (which has the exception stack frame visible). Triggers
// NVIC_SystemReset() and does not return.
//
// IMPORTANT: this function never returns. Marked Q_NORETURN-equivalent via
// [[noreturn]] on the C++ declaration.
[[noreturn]] void crash_record_capture(CrashReason reason,
                                       uint32_t stacked_pc,
                                       uint32_t stacked_lr);

// Consume any prior-boot crash record. Called once from health_monitor_init().
// Returns true and fills *out if a valid record was present (clears the
// magic so the next clean boot doesn't re-report). Returns false otherwise.
bool crash_record_consume_prior(CrashRecord* out);

} // namespace rc

#endif // ROCKETCHIP_SAFETY_CRASH_RECORD_H
