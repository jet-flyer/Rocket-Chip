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

#include "hardware/structs/sio.h"
#include "hardware/structs/mpu.h"
#include "pico/stdio.h"  // for printf in Q_onError
#include <stdio.h>

// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" uint32_t __StackBottom;  // For Core 0 documentation (defined in linker)

// ============================================================================
// MemManage / HardFault Handler
// ============================================================================
// Fires when MPU stack guard is hit (stack overflow).
// Must NOT use stack (it may be overflowed). Uses direct GPIO register writes.
// Blink pattern: 3 fast + 1 slow on red LED, forever.

void memmanage_fault_handler(void) {
    __asm volatile ("cpsid i");  // Disable interrupts

    // Direct GPIO register writes — no SDK calls that might use stack
    const uint32_t ledMask = 1U << board::kLedPin;

    // Active-low LED: SET register turns pin HIGH (off), CLR turns LOW (on)
    // Active-high LED: SET turns on, CLR turns off
    io_rw_32 *ledOn  = board::kLedActiveHigh ? &sio_hw->gpio_set : &sio_hw->gpio_clr;
    io_rw_32 *ledOff = board::kLedActiveHigh ? &sio_hw->gpio_clr : &sio_hw->gpio_set;

    while (true) {
        for (uint8_t i = 0; i < kFaultFastBlinks; i++) {
            *ledOn = ledMask;
            for (int32_t d = 0; d < kFaultBlinkFastLoops; d++) { __asm volatile(""); }
            *ledOff = ledMask;
            for (int32_t d = 0; d < kFaultBlinkFastLoops; d++) { __asm volatile(""); }
        }
        *ledOn = ledMask;
        for (int32_t d = 0; d < kFaultBlinkSlowLoops; d++) { __asm volatile(""); }
        *ledOff = ledMask;
        for (int32_t d = 0; d < kFaultBlinkSlowLoops; d++) { __asm volatile(""); }
    }
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
// Configures MPU region 0 as a no-access guard at the bottom of the stack.
// Each core has its own MPU — call from the core being protected.

void mpu_setup_stack_guard(uint32_t stackBottom) {
    // Disable MPU during configuration
    mpu_hw->ctrl = 0;
    __dsb();
    __isb();

    // NOLINTBEGIN(readability-magic-numbers) — PMSAv8 MPU register bit fields per ARMv8-M Architecture Reference Manual
    // Region 0: Stack guard (no access, execute-never)
    // PMSAv8 RBAR: [31:5]=BASE, [4:3]=SH(0=non-shareable), [2:1]=AP(0=priv no-access), [0]=XN(1)
    mpu_hw->rnr = 0;
    mpu_hw->rbar = (stackBottom & ~0x1FU)
                  | (0U << 3)   // SH: Non-shareable
                  | (0U << 1)   // AP: Privileged no-access
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
    // NOLINTEND(readability-magic-numbers)
    __dsb();
    __isb();
}
