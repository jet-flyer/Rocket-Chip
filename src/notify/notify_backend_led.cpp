// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Notification LED Backend (Stage 14, IVP-115/116)
//
// Priority resolver: maps NotifyState to a single rc::led::k* pattern code.
// Iterates categories in order: Fault > Cal > Flight > Radio > Sensor > Idle.
// First non-kNone category wins.
//
// notify_backend_led_update() calls the resolver then posts the resolved
// pattern to AO_LedEngine via the existing AO_LedEngine_post_pattern() API.
//
// In IVP-115 this is compiled but not called from AO_Notify tick. IVP-116
// wires it up atomically with removing the old direct-caller paths.
//============================================================================

#include "notify_resolver.h"
#include "rocketchip/notify_backend.h"
#include "rocketchip/led_patterns.h"

#ifndef ROCKETCHIP_HOST_TEST
#include "active_objects/ao_led_engine.h"
#endif

namespace rc {
namespace notify {

// ============================================================================
// Per-category pattern lookups
// Returns 0 (not a valid pattern code) if the intent is kNone.
// ============================================================================

static uint8_t fault_to_pattern(FaultIntent f) {
    switch (f) {
        case FaultIntent::kCore1Stall: return rc::led::kFaultCore1Stall;
        case FaultIntent::kSafeMode:   return rc::led::kFaultSafeMode;
        case FaultIntent::kImuFail:    return rc::led::kFaultImuFail;
        case FaultIntent::kEskfFail:   return rc::led::kFaultEskfFail;
        case FaultIntent::kBaroFail:   return rc::led::kFaultBaroFail;
        case FaultIntent::kPioWdt:     return rc::led::kFaultPioWdt;
        case FaultIntent::kNone:       return 0;
    }
    return 0;
}

static uint8_t cal_to_pattern(CalIntent c) {
    switch (c) {
        case CalIntent::kFail:         return rc::led::kCalFail;
        case CalIntent::kSuccess:      return rc::led::kCalSuccess;
        case CalIntent::kMag:          return rc::led::kCalMag;
        case CalIntent::kAccelSample:  return rc::led::kCalAccelSample;
        case CalIntent::kAccelWait:    return rc::led::kCalAccelWait;
        case CalIntent::kBaro:         return rc::led::kCalBaro;
        case CalIntent::kLevel:        return rc::led::kCalLevel;
        case CalIntent::kGyro:         return rc::led::kCalGyro;
        case CalIntent::kNone:         return 0;
    }
    return 0;
}

static uint8_t phase_to_pattern(PhaseIntent p) {
    switch (p) {
        case PhaseIntent::kBeacon: return rc::led::kFdBeacon;
        case PhaseIntent::kAbort:  return rc::led::kFdAbort;
        case PhaseIntent::kLanded: return rc::led::kFdLanded;
        case PhaseIntent::kMain:   return rc::led::kFdMain;
        case PhaseIntent::kDrogue: return rc::led::kFdDrogue;
        case PhaseIntent::kCoast:  return rc::led::kFdCoast;
        case PhaseIntent::kBoost:  return rc::led::kFdBoost;
        case PhaseIntent::kArmed:  return rc::led::kFdArmed;
        case PhaseIntent::kIdle:   return 0;  // Fall through
        case PhaseIntent::kNone:   return 0;
    }
    return 0;
}

static uint8_t radio_to_pattern(RadioIntent r) {
    switch (r) {
        case RadioIntent::kLost:      return rc::led::kRxLost;
        case RadioIntent::kGap:       return rc::led::kRxGap;
        case RadioIntent::kReceiving: return rc::led::kRxReceiving;
        case RadioIntent::kNone:      return 0;
    }
    return 0;
}

static uint8_t sensor_to_pattern(SensorIntent s) {
    switch (s) {
        case SensorIntent::kTimeout:    return rc::led::kSensorTimeout;
        case SensorIntent::kEskfInit:   return rc::led::kSensorEskfInit;
        case SensorIntent::kGps3d:      return rc::led::kSensorGps3d;
        case SensorIntent::kGps2d:      return rc::led::kSensorGps2d;
        case SensorIntent::kGpsSearch:  return rc::led::kSensorGpsSearch;
        case SensorIntent::kGpsNoNmea:  return rc::led::kSensorGpsNoNmea;
        case SensorIntent::kNoGps:      return rc::led::kSensorNoGps;
        case SensorIntent::kNone:       return 0;
    }
    return 0;
}

// ============================================================================
// Stage L: beacon overlay remap — applied to the base pattern after priority
// resolution. beacon_manual forces pure white (max visibility). beacon_auto
// preserves the state color so a recovery crew sees good/fault/safe-mode at
// a glance. Any base pattern not explicitly remapped (IDLE, BOOST/COAST,
// sensor states, calibration overlays) falls through to pure white when a
// beacon is active — beacon > any non-recovery-relevant state.
// ============================================================================
static uint8_t apply_beacon_overlay(uint8_t base, const NotifyState& s) {
    if (s.beacon_manual) {
        return rc::led::kFdBeacon;  // Pure white 2Hz — CLI findme / GCS beacon
    }
    if (!s.beacon_auto) {
        return base;
    }
    switch (base) {
        case rc::led::kFdLanded:        return rc::led::kFdLandedBeacon;
        case rc::led::kFdAbort:         return rc::led::kFdAbortBeacon;
        case rc::led::kFaultImuFail:    return rc::led::kFaultImuFailBeacon;
        case rc::led::kFaultEskfFail:   return rc::led::kFaultEskfFailBeacon;
        case rc::led::kFaultBaroFail:   return rc::led::kFaultBaroFailBeacon;
        case rc::led::kFaultPioWdt:     return rc::led::kFaultPioWdtBeacon;
        case rc::led::kFaultCore1Stall: return rc::led::kFaultCore1StallBeacon;
        case rc::led::kFaultSafeMode:   return rc::led::kFaultSafeMode;  // Already blue+white
        default:                        return rc::led::kFdBeacon;       // Pure white fallback
    }
}

// ============================================================================
// Priority resolver — pure function
// Iterates categories in order: Fault > Cal > Flight > Radio > Sensor > Idle.
// First non-zero result wins. Stage L: beacon overlay applied after resolution.
// ============================================================================
uint8_t resolve_led_pattern(const NotifyState& s) {
    uint8_t p = 0;
    if ((p = fault_to_pattern(s.fault))   != 0) { return apply_beacon_overlay(p, s); }
    if ((p = cal_to_pattern(s.cal))       != 0) { return apply_beacon_overlay(p, s); }
    if ((p = phase_to_pattern(s.phase))   != 0) { return apply_beacon_overlay(p, s); }
    if ((p = radio_to_pattern(s.radio))   != 0) { return apply_beacon_overlay(p, s); }
    if ((p = sensor_to_pattern(s.sensor)) != 0) { return apply_beacon_overlay(p, s); }
    return apply_beacon_overlay(rc::led::kSensorNoGps, s);  // Idle fallback
}

// ============================================================================
// Backend update function
// ============================================================================
void notify_backend_led_update(const NotifyState& state) {
    uint8_t pattern = resolve_led_pattern(state);
#ifndef ROCKETCHIP_HOST_TEST
    // Post resolved pattern to LedEngine. IVP-115: not yet called from
    // AO_Notify tick - wire-up in IVP-116.
    AO_LedEngine_post_pattern(pattern);
#else
    (void)pattern;  // Host test: resolver tested directly via resolve_led_pattern()
#endif
}

} // namespace notify
} // namespace rc
