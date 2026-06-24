// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file fault_protection.h
 * @brief Shared fault protection and MPU stack guard for both cores.
 *
 * Extracted per OPT-IVP-01 to eliminate duplication between main.cpp and
 * sensor_core1.cpp. Provides no-stack fault handler and PMSAv8 MPU setup.
 */

#ifndef ROCKETCHIP_FAULT_PROTECTION_H
#define ROCKETCHIP_FAULT_PROTECTION_H

#include "pico.h"
#include "hardware/exception.h"
extern "C" {
#include "qp_port.h"
#include "qsafe.h"     // for Q_NORETURN
}
#include "rocketchip/board.h"

// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" uint32_t __StackOneBottom;  // Core 1 stack bottom (SCRATCH_X, linker-defined)

// ============================================================================
// Constants (moved from main.cpp)
// ============================================================================

static constexpr uint32_t kMpuGuardSizeBytes = 64;              // Guard region at bottom of stack

// R-3 (audit 2026-05-07): the kFaultBlink* / kFaultFastBlinks LED-blink
// constants that lived here were used by the pre-R-3 halt-forever handler.
// Fault-recovery 2026-05-14 (commit b/3): handler is now phase-aware —
// captures crash state, then either (a) triggers SYSRESETREQ via AIRCR
// after a brief visible-signal delay (only when phase==kIdle), or (b)
// transitions to kFault and busy-loops while PIO backup timers continue
// running (any flight phase). The blink loop is gone; the visible signal
// is the serial banner plus a future raw-GPIO LED toggle (placeholder
// pending safe-from-fault-context pin write sequence verification).
// See plan parsed-soaring-popcorn.md sections B.1/B.2/B.3/B.7 +
// standards/HW_GATE_DISCIPLINE.md Rule 6 for the safe-mode integration model.

// ============================================================================
// Public API
// ============================================================================

/**
 * MemManage / HardFault handler — phase-aware capture-then-dispatch.
 * Must not use stack for the capture portion. Registered for both cores via
 * exception_set_exclusive_handler().
 *
 * In kIdle: captures crash record, emits visible signal (serial banner via
 * prior printf path; future raw-GPIO LED), brief delay, then AIRCR reset
 * (operator sees the post-reset prior-hardfault latch and clears via CLI).
 *
 * Any flight phase (or corrupted phase byte): transitions observable phase
 * to kFault and busy-loops. PIO backup timers continue autonomously.
 *
 * Reentrance guard prevents recursive faults from looping the handler.
 */
void memmanage_fault_handler(void);

/**
 * QP/C assertion handler — same phase-aware dispatch as memmanage_fault_handler.
 * Declaration is plain to avoid macro expansion issues with Q_NORETURN.
 * The noreturn attribute is on the definition in fault_protection.cpp.
 *
 * Pre-2026-05-14 this halted forever expecting an SDK hardware watchdog
 * reset that does not exist in tree. Now routes through phase-aware
 * dispatch so the chip never gets stuck waiting for a watchdog that
 * will never fire.
 */
extern "C" void Q_onError(
    char const * const module,
    int_t const id);

/**
 * Configures MPU region 0 as stack guard (no-access, XN) at bottom of stack.
 * Call from each core with its own __Stack*Bottom symbol.
 * Per-core, PMSAv8. See ARMv8-M Architecture Reference Manual.
 */
// stackBottom: a linker-symbol address passed as uintptr_t (CAST-1, JSF AV-182 —
// hardware-interface conversion for the numeric MPU RBAR; provably lossless on the
// 32-bit target). See standards/ACCEPTED_STANDARDS_DEVIATIONS.md (CAST-1).
void mpu_setup_stack_guard(uintptr_t stackBottom);

#endif  // ROCKETCHIP_FAULT_PROTECTION_H
