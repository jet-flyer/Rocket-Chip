// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file fault_protection.h
 * @brief Shared fault protection and MPU stack guard for both cores.
 *
 * Extracted per OPT-IVP-01 to eliminate duplication between main.cpp and
 * sensor_core1.cpp. Provides no-stack fault handler and PMSAv8 MPU setup.
 * See standards/CODING_STANDARDS.md for Flight-Support classification.
 */

#pragma once

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
// The new handler captures crash state and triggers NVIC_SystemReset() via
// crash_record_capture(), so the blink loop is gone and these constants
// are no longer used. See standards/HW_GATE_DISCIPLINE.md Rule 6 for the
// safe-mode integration model.

// ============================================================================
// Public API
// ============================================================================

/**
 * MemManage / HardFault handler. Must not use stack. Blinks red LED forever.
 * Registered for both cores via exception_set_exclusive_handler().
 */
void memmanage_fault_handler(void);

/**
 * QP/C assertion handler. Logs and spins (watchdog or manual reset follows).
 * Declaration is plain to avoid macro expansion issues with Q_NORETURN.
 * The noreturn attribute is on the definition in fault_protection.cpp.
 */
extern "C" void Q_onError(
    char const * const module,
    int_t const id);

/**
 * Configures MPU region 0 as stack guard (no-access, XN) at bottom of stack.
 * Call from each core with its own __Stack*Bottom symbol.
 * Per-core, PMSAv8. See ARMv8-M Architecture Reference Manual.
 */
void mpu_setup_stack_guard(uint32_t stackBottom);
