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
// QP Framework Port — Load main QP header
// ============================================================================
#include "qp.h"  // QP/C public API (QEvt, QHsm, QState, etc.)

#endif // QP_PORT_H_
