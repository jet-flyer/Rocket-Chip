// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// QP/C Port for RocketChip
//
// Platform abstraction for QEP hierarchical state machine dispatch.
// Two modes:
//   - Target (RP2350 Cortex-M33): PRIMASK-based critical sections
//   - Host test (x86/x64): no-op critical sections (single-threaded tests)
//
// Reference: QP/C 8.1.3 ports/arm-cm/qv/gnu/qp_port.h
//============================================================================
#ifndef QP_PORT_H_
#define QP_PORT_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Load application-specific configuration first
#include "qp_config.h"

// ============================================================================
// C11 / C++ compatibility
// ============================================================================
#ifndef Q_NORETURN
    #ifdef __cplusplus
        #define Q_NORETURN  [[ noreturn ]] void
    #else
        #define Q_NORETURN   _Noreturn void
    #endif
#endif

// Static assert — use C11 _Static_assert or C++11 static_assert
#ifndef Q_ASSERT_STATIC
    #ifdef __cplusplus
        #define Q_ASSERT_STATIC(expr_) static_assert((expr_), "QP static assert")
    #else
        #define Q_ASSERT_STATIC(expr_) _Static_assert((expr_), "QP static assert")
    #endif
#endif

// ============================================================================
// Critical Section Macros
//
// QEP uses critical sections around assertion checks (FuSa subsystem).
// On target: PRIMASK-based interrupt disable (simplest, covers all priorities).
// On host: no-op (single-threaded test environment).
//
// These are NOT used in the QEP dispatch hot path — only in assertion guards.
// Performance impact is negligible.
// ============================================================================

#ifdef ROCKETCHIP_HOST_TEST
    // Host test build: no interrupts, no critical sections needed
    #define QF_INT_DISABLE() ((void)0)
    #define QF_INT_ENABLE()  ((void)0)
    #define QF_CRIT_STAT
    #define QF_CRIT_ENTRY()  ((void)0)
    #define QF_CRIT_EXIT()   ((void)0)
    #define QF_CRIT_EST()    ((void)0)
#else
    // Target build: RP2350 Cortex-M33 PRIMASK-based critical sections
    // Reference: ARM v8-M Architecture Reference Manual, B5.2.3
    //
    // PRIMASK disables all interrupts except NMI and HardFault.
    // We save and restore to support nested critical sections.
    #include "pico/critical_section.h"
    #include "hardware/sync.h"

    // QF interrupt disable/enable — PRIMASK-based (simplest for QV)
    #define QF_INT_DISABLE() __asm volatile ("cpsid i" ::: "memory")
    #define QF_INT_ENABLE()  __asm volatile ("cpsie i" ::: "memory")

    // Fast LOG2 via CLZ instruction (ARMv8-M Cortex-M33 has CLZ)
    #define QF_LOG2(n_) ((uint_fast8_t)(32U - __builtin_clz((unsigned)(n_))))

    // QF_CRIT_STAT: declares local variable for saving interrupt state.
    // QF_CRIT_ENTRY/EXIT: save/restore pair (used together with QF_CRIT_STAT).
    // QF_CRIT_EST: "enter and save together" — declares AND disables in one step.
    //   Used in Q_ERROR_ID and Q_ASSERT_LOCAL where QF_CRIT_STAT is not present.
    #define QF_CRIT_STAT     uint32_t qf_crit_stat_;
    #define QF_CRIT_ENTRY()  do { qf_crit_stat_ = save_and_disable_interrupts(); } while (0)
    #define QF_CRIT_EXIT()   do { restore_interrupts(qf_crit_stat_); } while (0)
    #define QF_CRIT_EST()    do { (void)save_and_disable_interrupts(); } while (0)
#endif

// ============================================================================
// QV Port — Active Object configuration for QV cooperative kernel
//
// IVP-75: Compile gate for QF+QV. Stage 9 (IVP-76) activates.
// Reference: QP/C 8.1.3 ports/arm-cm/qv/gnu/qp_port.h
// ============================================================================

// QV uses QEQueue as the built-in event queue for Active Objects.
// Must be defined before qp.h so QActive struct includes the eQueue member.
#define QACTIVE_EQUEUE_TYPE  QEQueue

// QV is cooperative (no preemption) — no thread or OS object needed
// QACTIVE_THREAD_TYPE not defined (single stack, run-to-completion)
// QACTIVE_OS_OBJ_TYPE not defined (no OS primitives)

// ============================================================================
// QP Framework Port — Load headers in dependency order
//
// Order matters: qequeue.h and qmpool.h define the types used by
// QACTIVE_EQUEUE_TYPE and QF_EPOOL_TYPE_ macros. These must be included
// BEFORE qp.h which uses them in the QActive struct definition.
// Reference: QP/C 8.1.3 ports/arm-cm/qv/gnu/qp_port.h
// ============================================================================
#include "qequeue.h"  // QEQueue — event queue (before qp.h)
#include "qmpool.h"   // QMPool — memory pool (before qp.h)
#include "qp.h"       // QP/C public API (QEvt, QHsm, QActive, etc.)
#include "qv.h"       // QV — cooperative scheduler

#endif // QP_PORT_H_
