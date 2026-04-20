// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// System-Wide Active Object Signal Catalog
//
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
// Flight Director signals (4-13).
// System-wide AO signals start at SIG_MAX (14).
// ============================================================================
enum RcSignal : uint16_t {
    // --- Flight Director signals ---
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

    // --- System-wide Active Object signals ---
    //
    // Note on numbering: "removed" comments below are historical context
    // (which signals used to live at which slots). In C++ enums, removed
    // values do NOT create numeric gaps — the next enumerator just takes
    // the next value. Only explicit `= N` reassignments create gaps. The
    // numeric comments on each line reflect the ACTUAL runtime value.
    SIG_SENSOR_DATA = SIG_FD_MAX,   // 14: Sensor snapshot ready (eskf_tick → AOs)
    SIG_PHASE_CHANGE,                // 15: Flight phase transition (FD → Notify, Logger, HealthMon)
    SIG_LED_PATTERN,                 // 16: LED pattern request (Notify → LedEngine)
    SIG_BEACON_ACTIVE,               // 17: Post-landing beacon activated (FD ABORT timeout → Notify) [IVP-113]
                                     //     Sets NotifyState.beacon_auto — state-color + white overlay [Stage L]
    // Historically 17-18 held LOG_FRAME/TELEM_FRAME (removed — direct API used)
    SIG_PYRO_INTENT = 19,            // 19: Pyro fire intent (FD → Logger via callback)
    SIG_LED_OVERRIDE,                // 20: LEGACY — unused after IVP-116 (calibration now via AO_Notify_post_cal_intent)
    SIG_CLI_COMMAND,                 // 21: CLI command dispatch (CLI → FD, synchronous)
    // Historically 22 held HEALTH_CHECK (removed — HealthMonitor promoted to AO, Stage 13)

    // --- Radio Module ---
    SIG_RADIO_TX,                    // 22: Encoded packet ready for TX (Telem → Radio)
    SIG_RADIO_RX,                    // 23: Raw packet received (Radio → Telem)
    SIG_RADIO_STATUS,                // 24: Link quality update (Radio → Notify)
    SIG_GCS_CMD,                     // 25: Uplink command from GCS (Telem → FD) [C3-A4]
    SIG_HEALTH_STATUS,               // 26: Health flags changed (HealthMonitor → Notify, Logger, Telemetry)
    SIG_PYRO_FIRED,                  // 27: Pyro channel fired (FD/PIO → Logger)
    SIG_BEACON_MANUAL,               // 28: Manual CLI `findme` or GCS beacon command (RCOS/Telem → Notify) [Stage L]
                                     //     Sets NotifyState.beacon_manual — pure-white 2Hz (wins over beacon_auto)

    SIG_AO_MAX                       // 29: Sentinel — pub-sub array size
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

// Radio TX event — encoded packet ready to transmit
// Allocated from QP/C dynamic event pool [C3-A1]
struct RadioTxEvt {
    QEvt super;
    uint8_t buf[256];    // Encoded packet — CCSDS 54B, MAVLink multi-msg ~140B,
                         // future CCSDS+CLCW 58B. 256 matches EncodeResult.buf
                         // and SX1276 255-byte FIFO. (Stage T IVP-T5: was 128,
                         // overrunning on MAVLink encode_nav per findings.)
    uint8_t len;         // SX1276 FIFO is 255B — uint8_t is sufficient.
                         // Callers must clamp len <= sizeof(buf) before memcpy.
};

// Radio RX event — raw packet received from radio
// Allocated from QP/C dynamic event pool [C3-A1]
struct RadioRxEvt {
    QEvt super;
    uint8_t buf[256];    // Raw received bytes (matches TX for symmetry).
    uint8_t len;
    int16_t rssi;        // dBm
    int8_t  snr;         // dB
};

// Radio link quality status
struct RadioStatusEvt {
    QEvt super;
    uint8_t link_quality;   // 0=no radio, 1=lost, 2=gap, 3=receiving
};

// GCS uplink command [C3-A4]
struct GcsCmdEvt {
    QEvt super;
    uint8_t cmd_type;       // arm, disarm, abort, param, etc.
    uint32_t param;
};

// Health status change (health_monitor_tick detected flag change)
// Primary: 4 subsystems x 2-bit (IMU[1:0], Baro[3:2], ESKF[5:4], GPS[7:6])
// Secondary: 1-bit flags (radio, flash, watchdog, PIO)
struct HealthStatusEvt {
    QEvt super;
    uint8_t primary;        // 2-bit HealthLevel per subsystem
    uint8_t secondary;      // 1-bit HealthSecondary flags
};

// Pyro fired confirmation (FD smart path or PIO backup → Logger)
// [Council C-A1]: includes channel and source for diagnostics
struct PyroFiredEvt {
    QEvt super;
    uint8_t channel;        // 0=drogue, 1=main (matches PyroChannel enum)
    uint8_t source;         // 0=primary (FD-commanded), 1=PIO backup timeout
};

// Signal name lookup (extends flight_signal_name for system-wide signals)
const char* rc_signal_name(uint16_t sig);

} // namespace rc

#endif // ROCKETCHIP_AO_SIGNALS_H
