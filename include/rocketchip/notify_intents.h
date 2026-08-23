// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Notification intents — typed per category; kNone=0 so zero-init is idle.
// Priority: Fault > Calibration > Flight > Radio > Sensor > Idle.
// Cal above Flight so a cal overlay stays visible. See NOTIFY_CONTRACT.md.
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
    kNone       = 0,
    kIdle       = 1,
    kArmed      = 2,
    kBoost      = 3,
    kCoast      = 4,
    kDrogue     = 5,
    kMain       = 6,
    kLanded     = 7,
    kAbort      = 8,
    kBeacon     = 9,   // post-landing/abort recovery (white blink)
    kPreArmFail = 10,  // ARM rejected (yellow double-flash, ~3s auto-clear)
    kInit       = 11,  // boot warmup (rainbow; auto-clear on ESKF+IMU ready)
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

// One intent per category. Resolver takes the first non-kNone in priority order.
// beacon_auto: MAIN_DESCENT/LANDED/safe-mode overlay (composes with state color).
// beacon_manual: CLI findme / GCS — pure white 2 Hz; wins over auto.
// Both clear on SIG_PHASE_CHANGE out of LANDED/ABORT.
// vehicle_lost: AO_RfManager link-lost latch (not AO_Radio's per-packet RadioIntent).
struct NotifyState {
    PhaseIntent  phase;
    CalIntent    cal;
    RadioIntent  radio;
    SensorIntent sensor;
    FaultIntent  fault;
    bool         beacon_auto;
    bool         beacon_manual;
    bool         vehicle_lost;
};

} // namespace notify
} // namespace rc

#endif // ROCKETCHIP_NOTIFY_INTENTS_H
