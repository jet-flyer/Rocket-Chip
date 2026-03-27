// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// System-Wide Active Object Signal Catalog
//
// IVP-76: QF+QV BSP Integration (Stage 9: Active Object Architecture)
// Council-reviewed 2026-03-27 (Amendment A7: renamed from FlightSignal)
//
// All QP/C event signals for the RocketChip system. Signals are contiguous
// uint16_t values starting at Q_USER_SIG (4). Flight Director signals
// (SIG_TICK through SIG_RESET) retain their original values for backward
// compatibility.
//
// Convention:
//   - SIG_* prefix for all signals
//   - RcSignal enum for type safety
//   - SIG_AO_MAX sentinel sizes the pub-sub subscriber array
//
// Usage:
//   #include "rocketchip/ao_signals.h"
//   QEvt evt = QEVT_INITIALIZER(rc::SIG_PHASE_CHANGE);
//============================================================================
#ifndef ROCKETCHIP_AO_SIGNALS_H
#define ROCKETCHIP_AO_SIGNALS_H

#include <stdint.h>

extern "C" {
#include "qp_port.h"
}

namespace rc {

// ============================================================================
// RcSignal — system-wide event signal catalog
//
// Flight Director signals (4-13) are unchanged from IVP-68.
// System-wide AO signals start at SIG_MAX (14).
// ============================================================================
enum RcSignal : uint16_t {
    // --- Flight Director signals (values preserved from IVP-68) ---
    SIG_TICK        = Q_USER_SIG,   // 4: 100Hz periodic tick
    SIG_ARM,                         // 5: Arm command (CLI/radio)
    SIG_DISARM,                      // 6: Disarm command
    SIG_LAUNCH,                      // 7: Launch detected (guard)
    SIG_BURNOUT,                     // 8: Motor burnout (guard)
    SIG_APOGEE,                      // 9: Apogee detected (guard)
    SIG_MAIN_DEPLOY,                 // 10: Main chute altitude (guard)
    SIG_LANDING,                     // 11: Landing detected (guard)
    SIG_ABORT,                       // 12: Abort command/guard
    SIG_RESET,                       // 13: Reset to IDLE
    SIG_FD_MAX,                      // 14: FD sentinel (do not use as signal)

    // --- System-wide Active Object signals (IVP-76+) ---
    SIG_SENSOR_DATA = SIG_FD_MAX,   // 14: Sensor snapshot ready (eskf_tick → AOs)
    SIG_PHASE_CHANGE,                // 15: Flight phase transition (FD → Logger, Telem, LED)
    SIG_LED_PATTERN,                 // 16: LED pattern request (any → LedEngine)
    SIG_LOG_FRAME,                   // 17: Log data ready (Logger internal)
    SIG_TELEM_FRAME,                 // 18: Telemetry data ready (Logger → Telemetry)
    SIG_PYRO_INTENT,                 // 19: Pyro fire intent (FD → Logger for recording)
    SIG_LED_OVERRIDE,                // 20: Calibration/RX LED override (CLI → LedEngine)
    SIG_CLI_COMMAND,                 // 21: CLI command dispatch (CLI → FD)
    SIG_HEALTH_CHECK,                // 22: Periodic health ping (timer → ErrorHandler)
    SIG_AO_MAX                       // 23: Sentinel — pub-sub array size
};

// Backward compatibility aliases
using FlightSignal = RcSignal;

// SIG_MAX retained as enum member for code that assigns it to FlightSignal-typed
// variables (e.g., CommandResult.signal = SIG_MAX). Points to FD sentinel.
static constexpr RcSignal SIG_MAX = SIG_FD_MAX;

// ============================================================================
// Event Structures — typed payloads extending QEvt
//
// QV cooperative scheduling guarantees run-to-completion, so all events
// can be stack-allocated (QF_MAX_EPOOL pre-wired but pool not allocated).
// ============================================================================

// Phase transition event (published by Flight Director)
struct PhaseChangeEvt {
    QEvt super;
    uint8_t phase;      // rc::FlightPhase cast to uint8_t
    uint32_t timestamp_ms;
};

// LED pattern request (published by FD, calibration, RX overlay)
struct LedPatternEvt {
    QEvt super;
    uint8_t pattern;    // kCalNeo*, kRxNeo*, kFdNeo* values from ws2812_status.h
};

// Sensor data notification (published by eskf_tick bridge)
struct SensorDataEvt {
    QEvt super;
    uint32_t eskf_epoch;  // Which ESKF propagation epoch this represents
};

// Signal name lookup (extends flight_signal_name for system-wide signals)
const char* rc_signal_name(uint16_t sig);

} // namespace rc

#endif // ROCKETCHIP_AO_SIGNALS_H
