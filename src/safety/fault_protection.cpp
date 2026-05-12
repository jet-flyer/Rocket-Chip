// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file fault_protection.cpp
 * @brief Implementation of shared fault protection.
 *
 * OPT-IVP-01 extraction. No-stack fault handler and MPU setup.
 * Registered early in init_early_hw(). Core 1 calls only the MPU setup.
 */

#include "safety/fault_protection.h"
#include "safety/crash_record.h"

#include "hardware/structs/mpu.h"
#include "hardware/structs/scb.h"  // for SHCSR.MEMFAULTENA write in stack-guard setup
#include "pico/stdio.h"  // for printf in Q_onError
#include <stdio.h>

// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" uint32_t __StackBottom;  // For Core 0 documentation (defined in linker)

// ============================================================================
// MemManage / HardFault Handler
// ============================================================================
// Fires when MPU stack guard is hit (stack overflow).
//
// R-3 (audit 2026-05-07): replaces the prior halt-forever LED-blink loop with
// the industry-standard capture-state-then-reset pattern (ArduPilot / PX4 /
// NASA cFS). On entry: read the stacked PC/LR from the exception frame at
// MSP, pass them with CFSR/HFSR into crash_record_capture(), which writes
// the preserved-SRAM record and triggers NVIC_SystemReset(). On the next
// boot, health_monitor_init() consumes the record and sets
// kHealthCriticalPriorHardfault so the existing safe-mode / FAULT-health
// pivot owns the recovery path.
//
// Must NOT use the failing stack: we read MSP via inline assembly, dereference
// the exception frame to get PC + LR, then call into crash_record_capture()
// which itself executes from .text and uses minimal stack (single function
// frame on the now-recovered space above the guard).

void memmanage_fault_handler(void) {
    __asm volatile ("cpsid i");  // Disable interrupts

    // Read the main stack pointer at exception entry. The exception frame
    // (ARMv8-M ARM Table B3-9) is:
    //   MSP+0  : R0
    //   MSP+4  : R1
    //   MSP+8  : R2
    //   MSP+12 : R3
    //   MSP+16 : R12
    //   MSP+20 : LR (return address)
    //   MSP+24 : PC (faulting instruction)
    //   MSP+28 : xPSR
    uint32_t msp = 0;
    __asm volatile ("mrs %0, msp" : "=r"(msp));

    uint32_t stacked_lr = 0;
    uint32_t stacked_pc = 0;
    if (msp != 0) {
        // Read via volatile to defeat any speculative optimization that
        // assumes the source is "uninitialized."
        stacked_lr = *reinterpret_cast<volatile uint32_t*>(msp + 20);
        stacked_pc = *reinterpret_cast<volatile uint32_t*>(msp + 24);
    }

    rc::crash_record_capture(rc::kCrashReasonMemManage, stacked_pc, stacked_lr);
    // crash_record_capture is [[noreturn]] — fires NVIC_SystemReset().
}

// ============================================================================
// QP/C Assertion Handler
// ============================================================================
// Called by QEP when a state machine invariant is violated (null state handler,
// nesting depth overflow, etc.). Logs the failure and halts — the watchdog
// will reset the device and the recovery policy will track it.

extern "C" Q_NORETURN Q_onError(
    char const * const module,
    int_t const id)
{
    __asm volatile("cpsid i");  // Disable interrupts
    printf("[QP ASSERT] module=%s, id=%d\n", module, id);
    // Spin until watchdog resets us — recovery scratch registers are fresh
    // from the last watchdog_kick_tick().
    while (true) {
        __asm volatile("nop");
    }
}

// ============================================================================
// MPU Stack Guard Setup (per-core, PMSAv8)
// ============================================================================
// Configures MPU region 0 as a read-only-privileged guard at the bottom of
// the stack. A stack overflow (PUSH/STR past __StackBottom) writes into the
// region and triggers a MemManage fault, captured by memmanage_fault_handler.
//
// Each core has its own MPU — call from the core being protected.
//
// **R-3 (audit 2026-05-07) — corrected AP encoding.** Pre-R-3 this code set
// AP=0b00 with comment "Privileged no-access," but per the RP2350 datasheet
// §3.7.4.7 + the ARMv8-M PMSAv8 / CMSIS armv8 header, AP[2:1] in PMSAv8
// encodes the **privilege model**, not a "no-access-for-anyone" mode:
//
//   AP[2:1] = 00 (RO=0, NP=0): RW, Privileged-Only  (← was set here; allowed
//                                                     all privileged writes,
//                                                     so the guard did NOT
//                                                     fault on stack overflow
//                                                     from Thread-privileged
//                                                     code — pre-existing bug
//                                                     surfaced by R-3 verify)
//   AP[2:1] = 01 (RO=0, NP=1): RW, Any-privilege
//   AP[2:1] = 10 (RO=1, NP=0): RO, Privileged-Only  ← CORRECT for guard
//   AP[2:1] = 11 (RO=1, NP=1): RO, Any-privilege
//
// PMSAv8 has no "no-access-for-anyone" encoding; that would prevent the
// kernel from accessing its own memory. The closest semantics for a stack
// guard is RO — a write fault on STR/PUSH past the boundary.
//
// **MEMFAULTENA enabled** so MPU permission violations invoke the dedicated
// MemManage handler instead of escalating to HardFault (per §3.7.4.7 "MPU
// mismatches and permission violations invoke the MemManage handler").

void mpu_setup_stack_guard(uint32_t stackBottom) {
    // Disable MPU during configuration
    mpu_hw->ctrl = 0;
    __dsb();
    __isb();

    // NOLINTBEGIN(readability-magic-numbers) — PMSAv8 MPU register bit fields per ARMv8-M Architecture Reference Manual
    // Region 0: Stack guard. RO-Privileged, Execute-Never. Stack-overflow
    // writes trip a MemManage fault.
    // PMSAv8 RBAR: [31:5]=BASE, [4:3]=SH(0=non-shareable),
    //              [2:1]=AP(10=RO-Privileged), [0]=XN(1)
    mpu_hw->rnr = 0;
    mpu_hw->rbar = (stackBottom & ~0x1FU)
                  | (0U << 3)   // SH: Non-shareable
                  | (2U << 1)   // AP: 0b10 = RO, Privileged-Only (per CMSIS armv8 header)
                  | (1U << 0);  // XN: Execute-never

    // PMSAv8 RLAR: [31:5]=LIMIT, [3:1]=ATTRINDX(0), [0]=EN(1)
    mpu_hw->rlar = ((stackBottom + kMpuGuardSizeBytes - 1) & ~0x1FU)
                  | (0U << 1)   // ATTRINDX: 0 (uses MAIR0 attr 0)
                  | (1U << 0);  // EN: Enable region

    // MAIR0 attr 0 = 0x00 = Device-nGnRnE (strictest, no caching)
    mpu_hw->mair[0] = 0;

    // Enable MPU with PRIVDEFENA=1 (default memory map for unprogrammed regions)
    mpu_hw->ctrl = (1U << 2)   // PRIVDEFENA
                 | (1U << 0);  // ENABLE
    __dsb();
    __isb();

    // Enable MemManage exception so MPU permission violations invoke
    // memmanage_fault_handler directly (rather than escalating to HardFault).
    // SHCSR bit 16 = MEMFAULTENA.
    scb_hw->shcsr |= (1U << 16);
    __dsb();
    __isb();
    // NOLINTEND(readability-magic-numbers)
}
