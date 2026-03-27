// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// QP/C Configuration for RocketChip
//
// Stage 8: QEP dispatch + QF/QV compile gate (IVP-75).
// Stage 9: QF Active Objects replace superloop (IVP-76+).
// Council-reviewed 2026-03-27 (amendments A1-A8 incorporated).
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

// Active objects: 8 (headroom for Stage 9 AOs + future expansion)
// IVP-76: FlightDirector(5), Logger(4), Telemetry(3), LedEngine(2), reserved(1)
#define QF_MAX_ACTIVE       8U

// Event pools: 1 (pre-wired for future pool readiness, Council A3)
// Pool memory NOT allocated and QF_poolInit() NOT called until needed.
// All events are currently stack-allocated (QV run-to-completion guarantee).
#define QF_MAX_EPOOL        1U

// Time events: 1 tick rate at 100Hz (Pico SDK repeating timer, IVP-76)
// AOs use QF time events with periods relative to this base:
//   FD: every 1 tick (100Hz), Logger: 2 (50Hz), Telem: 10 (10Hz), LED: 3 (~33Hz)
#define QF_MAX_TICK_RATE    1U

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
