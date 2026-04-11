// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Notification Intent Enums and State (Stage 14, IVP-113)
//
// Per-category typed enums (Council A3: compile-time category enforcement).
// Callers set a typed intent; AO_Notify resolves priority and dispatches
// to output backends (LED, future audio, future OLED).
//
// Each enum's kNone = 0 so zero-init of NotifyState means "no intent active."
//
// Category priority order (highest to lowest):
//   Fault > Calibration > Flight > Radio > Sensor > Idle
//
// Calibration above Flight: when calibrating, the overlay must be visible
// regardless of flight phase (always IDLE during cal, but enforced structurally).
// See docs/decisions/NOTIFY_CONTRACT.md for rationale.
//============================================================================
#ifndef ROCKETCHIP_NOTIFY_INTENTS_H
#define ROCKETCHIP_NOTIFY_INTENTS_H

#include <stdint.h>

namespace rc {
namespace notify {

// ============================================================================
// Category: Flight Phase (from AO_FlightDirector via SIG_PHASE_CHANGE)
// Maps 1:1 to FlightPhase enum in flight_state.h, plus kBeacon for
// post-landing/abort recovery beacon (via SIG_BEACON_ACTIVE).
// ============================================================================
enum class PhaseIntent : uint8_t {
    kNone    = 0,
    kIdle    = 1,
    kArmed   = 2,
    kBoost   = 3,
    kCoast   = 4,
    kDrogue  = 5,
    kMain    = 6,
    kLanded  = 7,
    kAbort   = 8,
    kBeacon  = 9,   // Post-landing/abort recovery beacon (white blink)
};

// ============================================================================
// Category: Calibration overlay (from AO_RCOS via AO_Notify_post_cal_intent)
// ============================================================================
enum class CalIntent : uint8_t {
    kNone          = 0,
    kGyro          = 1,   // Blue breathe (keep still)
    kLevel         = 2,   // Blue breathe (keep flat)
    kBaro          = 3,   // Cyan breathe (sampling)
    kAccelWait     = 4,   // Yellow blink (position board)
    kAccelSample   = 5,   // Yellow solid (hold still)
    kMag           = 6,   // Rainbow (rotate freely)
    kSuccess       = 7,   // Green solid (step passed)
    kFail          = 8,   // Red blink fast (step failed)
};

// ============================================================================
// Category: Radio link quality (from AO_Radio via SIG_RADIO_STATUS)
// ============================================================================
enum class RadioIntent : uint8_t {
    kNone      = 0,
    kReceiving = 1,  // Packets arriving
    kGap       = 2,  // >2s gap
    kLost      = 3,  // >5s lost
};

// ============================================================================
// Category: Sensor status (evaluated by AO_Notify from seqlock reads)
// ============================================================================
enum class SensorIntent : uint8_t {
    kNone        = 0,
    kNoGps       = 1,  // ESKF running, no GPS module
    kGpsNoNmea   = 2,  // GPS init, no data yet
    kGpsSearch   = 3,  // NMEA received, no fix
    kGps2d       = 4,  // 2D fix
    kGps3d       = 5,  // 3D fix
    kEskfInit    = 6,  // ESKF not initialized (hold still)
    kTimeout     = 7,  // 5-min sensor phase timeout
};

// ============================================================================
// Category: Fault (from SIG_HEALTH_STATUS + Core1 vitality)
// Ascending value = ascending display priority. Resolver picks max.
// ============================================================================
enum class FaultIntent : uint8_t {
    kNone        = 0,
    kPioWdt      = 1,  // PIO watchdog fired
    kBaroFail    = 2,  // Baro fault
    kEskfFail    = 3,  // ESKF fault
    kImuFail     = 4,  // IMU fault
    kSafeMode    = 5,  // Watchdog safe mode
    kCore1Stall  = 6,  // Core 1 stalled (highest fault priority)
};

// ============================================================================
// NotifyState — complete intent snapshot owned by AO_Notify
//
// Zero-initialized = no active intents (safe default).
// One active intent per category. The resolver iterates categories in
// priority order and picks the first non-kNone.
// ============================================================================
struct NotifyState {
    PhaseIntent  phase;
    CalIntent    cal;
    RadioIntent  radio;
    SensorIntent sensor;
    FaultIntent  fault;
};

} // namespace notify
} // namespace rc

#endif // ROCKETCHIP_NOTIFY_INTENTS_H
