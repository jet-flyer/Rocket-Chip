// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// crash_record — preserved-SRAM record for fault-handler capture-and-reset.
// See crash_record.h for design rationale.

#include "safety/crash_record.h"
#include "hardware/structs/scb.h"

namespace rc {

// The preserved record lives in .uninitialized_data — NOLOAD region per the
// Pico SDK linker script. SRAM contents survive NVIC_SystemReset() but not
// power-off. On clean boot the magic field is whatever garbage the SRAM
// happened to hold, so the consume side checks against kCrashRecordMagic.
__attribute__((section(".uninitialized_data"), used))
static CrashRecord g_crash_record;

[[noreturn]] void crash_record_capture(CrashReason reason,
                                       uint32_t stacked_pc,
                                       uint32_t stacked_lr) {
    // Read fault-status registers via the SDK's scb_hw pointer (resolves to
    // 0xE000ED00 + offsets). CFSR at 0x28, HFSR at 0x2C per ARMv8-M ARM.
    g_crash_record.cfsr       = scb_hw->cfsr;
    g_crash_record.hfsr       = scb_hw->hfsr;
    g_crash_record.stacked_pc = stacked_pc;
    g_crash_record.stacked_lr = stacked_lr;
    g_crash_record.reason     = static_cast<uint32_t>(reason);
    g_crash_record.reserved[0] = 0;
    g_crash_record.reserved[1] = 0;
    // Magic last so a torn write doesn't produce a false-positive on the
    // next boot. If reset fires mid-write, the consumer rejects the
    // record (no magic) and the boot proceeds without a phantom report.
    g_crash_record.magic = kCrashRecordMagic;

    // Memory barrier ensures the writes complete before reset.
    __asm volatile ("dsb" ::: "memory");

    // ARMv8-M software reset: AIRCR write with VECTKEY (0x05FA) and
    // SYSRESETREQ (bit 2). Per ARMv8-M ARM, this requests a system reset.
    scb_hw->aircr = (0x05FAU << 16) | (1U << 2);

    // Should not reach here, but guard against the reset not firing
    // immediately — spin without writing anything else.
    while (true) {
        __asm volatile ("wfi");
    }
}

bool crash_record_consume_prior(CrashRecord* out) {
    if (g_crash_record.magic != kCrashRecordMagic) {
        return false;
    }
    if (out != nullptr) {
        *out = g_crash_record;
    }
    // Clear the magic so a clean boot doesn't re-report. Use a barrier
    // to ensure the write commits before any subsequent code reads it.
    g_crash_record.magic = 0;
    __asm volatile ("dsb" ::: "memory");
    return true;
}

} // namespace rc
