// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Shared fault protection: no-stack handler and MPU stack guard.
// Registered early in init_early_hw(). Core 1 calls only the MPU setup.

#include "safety/fault_protection.h"
#include "safety/crash_record.h"
#include "flight_director/flight_state.h"  // FlightPhase + flight_phase_observable_get

#include "hardware/structs/mpu.h"
#include "hardware/structs/scb.h"  // for SHCSR.MEMFAULTENA write in stack-guard setup
#include "pico/time.h"   // for busy_wait_us in fault handler delay
#include "rocketchip/rc_log.h"  // Q_onError — no printf

// ============================================================================
// Fault-handler dispatch internals
// ============================================================================

// Reentrance guard: if a second fault fires from inside the handler
// (busy-loop / LED-flash / status-print), the second entry observes this
// flag and routes directly to __WFE() halt. Prevents handler recursion.
//
// One-shot: never cleared on successful exit. The only exits from the
// handler are (a) AIRCR-reset which wipes SRAM-to-the-power-domain (the
// flag's storage), or (b) busy-loop in kFault, where leaving via reset
// is the only path forward anyway. Either way, the flag's lifetime ends
// with the next chip reset.
static volatile bool g_inFaultHandler = false;

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
// coverage during this state is a known gap (optional last-gasp beacon
// is compile-time, default off).
[[noreturn]] static inline void fault_degrade_in_place() {
    rc::flight_phase_observable_set(rc::FlightPhase::kFault);
    __asm volatile ("dsb" ::: "memory");
    while (true) {
        __asm volatile ("wfe");
    }
}

// Pad reset: LED helper is a no-op. 50 ms wait with IRQs off does not drain CDC.
[[noreturn]] static inline void fault_reset_with_visible_signal() {
    fault_emit_visible_signal();
    busy_wait_us(50000U);
    fault_trigger_reset();
}

// MemManage / HardFault handler — phase-aware capture-then-dispatch.
// Capture is no-stack-push. Design: docs/decisions/FAULT_HANDLER_DESIGN.md.
// Function-size: capture-then-dispatch stays in one handler (no-stack-push).
// Design record: docs/decisions/FAULT_HANDLER_DESIGN.md. No FH-1 row in
// ACCEPTED_STANDARDS_DEVIATIONS.md — do not cite a missing register ID.

__attribute__((used))
void memmanage_fault_handler(void) {
    __asm volatile ("cpsid i" ::: "memory");

    // Reentrance guard
    if (g_inFaultHandler) {
        while (true) {
            __asm volatile ("wfe");
        }
    }
    g_inFaultHandler = true;

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

    // Phase-aware dispatch. Corrupted phase byte → kFault (fail closed).
    const rc::FlightPhase phase = rc::flight_phase_observable_get();
    if (phase == rc::FlightPhase::kIdle) {
        // Pad: visible signal + reset; latch gates pre-arm after reboot.
        fault_reset_with_visible_signal();
    }
    // Any flight phase, or corrupted phase (kFault) → degrade in place.
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
// No SDK watchdog auto-reset. Halt-forever on pad is wrong; this
// path uses the same phase-aware capture as memmanage_fault_handler.

extern "C" Q_NORETURN Q_onError(
    char const * const module,
    int_t const id)
{
    __asm volatile("cpsid i" ::: "memory");

    // Reentrance guard (shared with memmanage_fault_handler)
    if (g_inFaultHandler) {
        while (true) {
            __asm volatile ("wfe");
        }
    }
    g_inFaultHandler = true;

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
    // happens later from qv_idle_bridge drain or via the
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

// PMSAv8-M RBAR/RLAR/CTRL/SHCSR field encodings (ARMv8-M ARM; RP2350 §3.7.4.7).
constexpr uint32_t kMpuAddrAlignMask     = ~0x1FU;  // BASE/LIMIT [4:0] = 0 (32-byte)
constexpr uint32_t kMpuRbarShShift       = 3U;
constexpr uint32_t kMpuRbarApShift       = 1U;
constexpr uint32_t kMpuRbarXnBit         = 1U << 0;
constexpr uint32_t kMpuShareNonShareable = 0U;
constexpr uint32_t kMpuApRoPrivileged    = 2U;      // AP[2:1] = 0b10
constexpr uint32_t kMpuRlarAttrIdxShift  = 1U;
constexpr uint32_t kMpuRlarEnBit         = 1U << 0;
constexpr uint32_t kMpuCtrlPrivdefenaBit = 1U << 2;
constexpr uint32_t kMpuCtrlEnableBit     = 1U << 0;
constexpr uint32_t kScbShcsrMemFaultEna  = 1U << 16;

void mpu_setup_stack_guard(uintptr_t stack_bottom) {
    // Disable MPU during configuration
    mpu_hw->ctrl = 0;
    __dsb();
    __isb();

    // Region 0: Stack guard. RO-Privileged, Execute-Never. Stack-overflow
    // writes trip a MemManage fault.
    mpu_hw->rnr = 0;
    mpu_hw->rbar = (stack_bottom & kMpuAddrAlignMask)
                  | (kMpuShareNonShareable << kMpuRbarShShift)
                  | (kMpuApRoPrivileged << kMpuRbarApShift)
                  | kMpuRbarXnBit;

    mpu_hw->rlar = ((stack_bottom + kMpuGuardSizeBytes - 1) & kMpuAddrAlignMask)
                  | (0U << kMpuRlarAttrIdxShift)
                  | kMpuRlarEnBit;

    // MAIR0 attr 0 = Device-nGnRnE (strictest, no caching)
    mpu_hw->mair[0] = 0;

    mpu_hw->ctrl = kMpuCtrlPrivdefenaBit | kMpuCtrlEnableBit;
    __dsb();
    __isb();

    scb_hw->shcsr |= kScbShcsrMemFaultEna;
    __dsb();
    __isb();
}
