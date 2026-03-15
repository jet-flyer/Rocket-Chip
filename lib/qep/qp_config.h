// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// QP/C Configuration for RocketChip
//
// QEP-only integration for Stage 8 (Flight Director HSM).
// QF Active Objects and QV scheduler added in Stage 9 (IVP-76+).
//
// Reference: QP/C 8.1.3 ports/config/qp_config.h
//============================================================================
#ifndef QP_CONFIG_H_
#define QP_CONFIG_H_

// ============================================================================
// API version (0 = maximum backwards compatibility)
// ============================================================================
#define QP_API_VERSION  0

// ============================================================================
// QF Framework Configuration
//
// QEP doesn't directly use these, but qp.h defines defaults that pull in
// unnecessary framework infrastructure. Setting them to minimal values
// avoids dead code in the build.
// ============================================================================

// Active objects: 1 minimum (QEP doesn't use AOs, but qp.h requires > 0)
// Will be increased in Stage 9 (IVP-76) when AO architecture is added.
#define QF_MAX_ACTIVE       1U

// Event pools: 0 (all events are stack-allocated or static in Stage 8)
// Will be increased in Stage 9 if dynamic event allocation is needed.
#define QF_MAX_EPOOL        0U

// Time events: 0 (Flight Director uses its own tick timing via superloop)
// Will be reconsidered in Stage 9 when QF time events may replace manual timers.
#define QF_MAX_TICK_RATE    0U

// ============================================================================
// Safety Configuration
//
// FuSa (Functional Safety) assertions are ENABLED (Q_UNSAFE is NOT defined).
// QEP assertions guard critical state machine invariants:
//   - State nesting depth overflow
//   - Null state handler detection
//   - Invalid transition targets
//   - Event signal range violations
//
// These assertions call Q_onError() which is implemented in the application
// (main.cpp) to log the failure and enter a safe halt state.
// ============================================================================
// Q_UNSAFE is intentionally NOT defined here.

// ============================================================================
// QS Software Tracing
//
// Disabled for now. QS tracing adds significant code size and requires a
// dedicated output channel. Can be enabled later for state machine debugging
// by defining Q_SPY and providing a qs_port.h.
// ============================================================================
// Q_SPY is intentionally NOT defined here.

#endif // QP_CONFIG_H_
