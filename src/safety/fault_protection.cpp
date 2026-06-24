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
#include "flight_director/flight_state.h"  // FlightPhase + flight_phase_observable_get

#include "hardware/structs/mpu.h"
#include "hardware/structs/scb.h"  // for SHCSR.MEMFAULTENA write in stack-guard setup
#include "pico/time.h"   // for busy_wait_us in fault handler delay
#include "rocketchip/rc_log.h"  // R-5 Unit C: replaces printf in Q_onError

// ============================================================================
// Fault-handler dispatch internals (fault-recovery 2026-05-14, B.1-B.3, B.7)
// ============================================================================

// B.7 reentrance guard: if a second fault fires from inside the handler
// (e.g., during the busy-loop / LED-flash / status-print sequence), the
// second entry observes this flag set and routes directly to __WFE() halt.
// Prevents infinite recursion through the handler.
//
// One-shot: never cleared on successful exit. The only exits from the
// handler are (a) AIRCR-reset which wipes SRAM-to-the-power-domain (the
// flag's storage), or (b) busy-loop in kFault, where leaving via reset
// is the only path forward anyway. Either way, the flag's lifetime ends
// with the next chip reset.
static volatile bool s_in_fault_handler = false;

// Helper: distinct LED pattern emitted from the fault handler via direct
// GPIO writes. The full LED system depends on AO_LedEngine + PIO which
// may not be in a healthy state after the fault. The fault handler emits
// a minimal "I am in fault" pattern via the on-board status LED only, if
// one is available. For now, this is a placeholder — the visible signal
// from the fault handler is the serial banner. Future commit can wire a
// raw-GPIO LED toggle here once the safe-to-touch-from-fault GPIO list
// is verified.
static inline void fault_emit_visible_signal() {
    // No-op for now. Serial banner via printf below is the visible signal.
    // TODO: wire raw GPIO toggle for status LED once safe-from-fault-context
    //       pin write sequence is verified.
}

// Helper: trigger AIRCR.SYSRESETREQ. Used by the pad-reset branch of the
// fault handler only — in-flight branch never calls this.
[[noreturn]] static inline void fault_trigger_reset() {
    __asm volatile (
        "ldr r0, =0xE000ED0C\n"
        "ldr r1, =0x05FA0004\n"
        "str r1, [r0]\n"
        "dsb\n"
        ::: "r0", "r1", "memory"
    );
    while (true) {
        __asm volatile ("wfe");  // unreachable in practice; AIRCR resets first
    }
}

// Helper: in-flight degrade path. Set the fault-observable phase to kFault
// so any AOs that still get scheduled see the new phase, then busy-loop
// forever. PIO backup timers continue autonomously (they're independent of
// ARM execution); the rest of the firmware effectively stops. Beacon
// coverage during this state is the known gap — addressed by commit (c)'s
// last-gasp beacon (compile-time #ifdef, default off) and ultimately by a
// future dedicated PIO beacon session.
[[noreturn]] static inline void fault_degrade_in_place() {
    rc::flight_phase_observable_set(rc::FlightPhase::kFault);
    __asm volatile ("dsb" ::: "memory");
    while (true) {
        __asm volatile ("wfe");
    }
}

// Helper: pad reset path. Emit visible signal (serial banner already
// captured the crash details via stack-stored CFSR/HFSR/PC/LR; we add a
// brief delay so the operator sees the banner before AIRCR fires), then
// trigger AIRCR.
[[noreturn]] static inline void fault_reset_with_visible_signal() {
    fault_emit_visible_signal();
    // Brief delay so any in-progress USB CDC transmission of the serial
    // banner has time to drain before reset. busy_wait_us uses a loop
    // counter (not a hardware timer callback), so it's safe with
    // interrupts disabled in the fault handler context. ~50ms is
    // empirically enough to drain the last few CDC packets on the
    // RP2350's USB stack without being so long the operator thinks
    // the chip wedged. See:
    //   https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/
    //     group__hardware__sync.html (busy_wait_us semantics)
    busy_wait_us(50000U);
    fault_trigger_reset();
}

// MemManage / HardFault handler — phase-aware capture-then-dispatch.
// Design rationale: see plan B.1, B.2, B.3, B.7 +
// docs/decisions/FAULT_HANDLER_DESIGN.md (R-3 era). The no-stack-push
// constraint still holds for the capture portion. Function-size deviation
// logged in standards/ACCEPTED_STANDARDS_DEVIATIONS.md (FH-1).

__attribute__((used))
void memmanage_fault_handler(void) {
    __asm volatile ("cpsid i" ::: "memory");

    // B.7 reentrance guard
    if (s_in_fault_handler) {
        while (true) {
            __asm volatile ("wfe");
        }
    }
    s_in_fault_handler = true;

    rc::CrashRecord * const rec = &rc::g_crash_record;
    uint32_t cfsr;
    uint32_t hfsr;
    __asm volatile (
        "ldr %0, =0xE000ED28\n"
        "ldr %0, [%0]\n"
        "ldr %1, =0xE000ED2C\n"
        "ldr %1, [%1]\n"
        : "=&r"(cfsr), "=&r"(hfsr)
    );
    uint32_t msp;
    __asm volatile ("mrs %0, msp" : "=r"(msp));
    uint32_t stacked_pc = 0;
    uint32_t stacked_lr = 0;
    if (msp != 0U) {
        // Exception frame layout (ARMv8-M ARM Table B3-9): MSP+20=LR, MSP+24=PC.
        stacked_lr = *reinterpret_cast<volatile uint32_t*>(msp + 20U);
        stacked_pc = *reinterpret_cast<volatile uint32_t*>(msp + 24U);
    }
    rec->cfsr        = cfsr;
    rec->hfsr        = hfsr;
    rec->stacked_pc  = stacked_pc;
    rec->stacked_lr  = stacked_lr;
    rec->reason      = static_cast<uint32_t>(rc::kCrashReasonMemManage);
    rec->reserved[0] = 0U;
    rec->reserved[1] = 0U;
    __asm volatile ("dsb" ::: "memory");
    rec->magic = rc::kCrashRecordMagic;  // magic last so torn writes reject on consume
    __asm volatile ("dsb" ::: "memory");

    // B.1 / B.2 phase-aware dispatch. flight_phase_observable_get() returns
    // kFault on checksum mismatch (safe-by-default for corrupted phase byte).
    const rc::FlightPhase phase = rc::flight_phase_observable_get();
    if (phase == rc::FlightPhase::kIdle) {
        // B.2: pad fault — visible signal + reset (the post-reset latch
        // surfaces via kHealthCriticalPriorHardfault, gating pre-arm).
        fault_reset_with_visible_signal();
    }
    // B.1: any flight phase (kArmed/kBoost/kCoast/kDescent/kLanded/kAbort)
    // OR a corrupted phase (returns kFault) → degrade in place.
    fault_degrade_in_place();
}

// ============================================================================
// QP/C Assertion Handler
// ============================================================================
// Called by QEP when a state machine invariant is violated (null state handler,
// nesting depth overflow, etc.). Routes through the same phase-aware dispatch
// as memmanage_fault_handler — in flight, degrade in place; on pad, capture
// + visible signal + reset.
//
// Refactor 2026-05-14: removed stale "watchdog will reset us" comments that
// referenced a watchdog_kick_tick() / SDK hardware watchdog that does not
// exist in tree (IVP-90 removed it). With no auto-reset path, the previous
// halt-forever code would leave the chip dead until manual power cycle —
// inconsistent with the new phase-aware fault recovery architecture.

extern "C" Q_NORETURN Q_onError(
    char const * const module,
    int_t const id)
{
    __asm volatile("cpsid i" ::: "memory");

    // B.7 reentrance guard (shared with memmanage_fault_handler)
    if (s_in_fault_handler) {
        while (true) {
            __asm volatile ("wfe");
        }
    }
    s_in_fault_handler = true;

    // Capture into the crash record so the post-reset (or post-degrade
    // diagnostic readout) consumer sees a record. Reason code reuses
    // kCrashReasonNone since there's no dedicated QP-assert reason yet
    // (future enum addition); the module string + id are lost to the
    // crash record but printed live to serial below.
    rc::CrashRecord * const rec = &rc::g_crash_record;
    rec->cfsr        = 0U;
    rec->hfsr        = 0U;
    rec->stacked_pc  = 0U;
    rec->stacked_lr  = 0U;
    rec->reason      = static_cast<uint32_t>(rc::kCrashReasonNone);
    rec->reserved[0] = 0U;
    rec->reserved[1] = 0U;
    __asm volatile ("dsb" ::: "memory");
    rec->magic = rc::kCrashRecordMagic;
    __asm volatile ("dsb" ::: "memory");

    // Best-effort live print — USB CDC may or may not still drain after
    // interrupts-disabled, depending on whether the assertion came from
    // a context that already had USB infrastructure healthy. If it doesn't
    // make it out the wire, the captured crash record will surface on
    // next boot. rc_log writes to the ring buffer non-blocking; drain
    // happens later from Core 0's tud_task path or via the
    // visible-signal delay before AIRCR.
    rc::rc_log("[QP ASSERT] module=%s, id=%d\n", module, id);

    // Phase-aware dispatch — same as memmanage_fault_handler.
    const rc::FlightPhase phase = rc::flight_phase_observable_get();
    if (phase == rc::FlightPhase::kIdle) {
        fault_reset_with_visible_signal();
    }
    fault_degrade_in_place();
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

void mpu_setup_stack_guard(uintptr_t stackBottom) {
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
