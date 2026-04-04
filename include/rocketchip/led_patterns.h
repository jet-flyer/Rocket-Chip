// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// LED Pattern Constants — Single Source of Truth
//
// NeoPixel override values used by calibration, RX overlay, and flight
// phase subsystems. Replaces duplicate definitions in main.cpp and rc_os.cpp.
//
// Values are uint8_t pattern codes written to g_calNeoPixelOverride (interim)
// or posted as LedPatternEvt (target architecture). The LED engine maps
// each code to a (mode, color) pair.
//
// Value ranges:
//   0       = no override (normal status logic)
//   1-8     = calibration overlays
//   9-11    = RX link quality overlays
//   20-27   = flight phase overlays (match LedPhaseValue in action_executor.h)
//
// Stage 13 AO Architecture: Phase 0B extraction.
//============================================================================
#ifndef ROCKETCHIP_LED_PATTERNS_H
#define ROCKETCHIP_LED_PATTERNS_H

#include <stdint.h>

namespace rc {
namespace led {

// ============================================================================
// Calibration overlays (set by CLI calibration wizards)
// ============================================================================
static constexpr uint8_t kOff         = 0;  // Normal NeoPixel behavior
static constexpr uint8_t kCalGyro     = 1;  // Blue breathe (keep still)
static constexpr uint8_t kCalLevel    = 2;  // Blue breathe (keep flat)
static constexpr uint8_t kCalBaro     = 3;  // Cyan breathe (sampling)
static constexpr uint8_t kCalAccelWait   = 4;  // Yellow blink (position board)
static constexpr uint8_t kCalAccelSample = 5;  // Yellow solid (hold still)
static constexpr uint8_t kCalMag      = 6;  // Rainbow (rotate freely)
static constexpr uint8_t kCalSuccess  = 7;  // Green solid (step passed)
static constexpr uint8_t kCalFail     = 8;  // Red blink fast (step failed)

// ============================================================================
// RX mode overlays (set by radio RX status)
// ============================================================================
static constexpr uint8_t kRxReceiving = 9;   // Green solid (packets arriving)
static constexpr uint8_t kRxGap       = 10;  // Yellow blink (>1s gap)
static constexpr uint8_t kRxLost      = 11;  // Red blink fast (>5s gap)

// ============================================================================
// Flight phase overlays (set by Flight Director actions)
// Values match LedPhaseValue enum in action_executor.h
// ============================================================================
static constexpr uint8_t kFdArmed     = 20;  // Amber solid
static constexpr uint8_t kFdBoost     = 21;  // Red solid
static constexpr uint8_t kFdCoast     = 22;  // Yellow solid
static constexpr uint8_t kFdDrogue    = 23;  // Red blink
static constexpr uint8_t kFdMain      = 24;  // Red blink
static constexpr uint8_t kFdLanded    = 25;  // Green blink
static constexpr uint8_t kFdAbort     = 26;  // Red fast blink
static constexpr uint8_t kFdBeacon    = 27;  // White blink (post-landing locator)

} // namespace led
} // namespace rc

// ============================================================================
// Backward-compatibility aliases (non-namespaced)
//
// Used throughout main.cpp, rc_os.cpp, ao_led_engine.cpp until those files
// are migrated to use rc::led:: namespace directly. Remove after Phase 5.
// ============================================================================
static constexpr uint8_t kCalNeoOff         = rc::led::kOff;
static constexpr uint8_t kCalNeoGyro        = rc::led::kCalGyro;
static constexpr uint8_t kCalNeoLevel       = rc::led::kCalLevel;
static constexpr uint8_t kCalNeoBaro        = rc::led::kCalBaro;
static constexpr uint8_t kCalNeoAccelWait   = rc::led::kCalAccelWait;
static constexpr uint8_t kCalNeoAccelSample = rc::led::kCalAccelSample;
static constexpr uint8_t kCalNeoMag         = rc::led::kCalMag;
static constexpr uint8_t kCalNeoSuccess     = rc::led::kCalSuccess;
static constexpr uint8_t kCalNeoFail        = rc::led::kCalFail;

static constexpr uint8_t kRxNeoReceiving    = rc::led::kRxReceiving;
static constexpr uint8_t kRxNeoGap          = rc::led::kRxGap;
static constexpr uint8_t kRxNeoLost         = rc::led::kRxLost;

static constexpr uint8_t kFdNeoArmed        = rc::led::kFdArmed;
static constexpr uint8_t kFdNeoBoost        = rc::led::kFdBoost;
static constexpr uint8_t kFdNeoCoast        = rc::led::kFdCoast;
static constexpr uint8_t kFdNeoDrogue       = rc::led::kFdDrogue;
static constexpr uint8_t kFdNeoMain         = rc::led::kFdMain;
static constexpr uint8_t kFdNeoLanded       = rc::led::kFdLanded;
static constexpr uint8_t kFdNeoAbort        = rc::led::kFdAbort;
static constexpr uint8_t kFdNeoBeacon       = rc::led::kFdBeacon;

#endif // ROCKETCHIP_LED_PATTERNS_H
