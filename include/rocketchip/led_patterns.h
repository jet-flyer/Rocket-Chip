// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// LED pattern codes posted to AO_LedEngine (mode + color).
// 0 off; 1-8 cal; 9-11 RX; 12-18 beacon overlays (base + white);
// 20-27 flight phase; 28-29 pre-arm fail / boot; 30-36 sensors; 41-46 faults.
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
static constexpr uint8_t kCalGyro     = 1;  // Yellow blink
static constexpr uint8_t kCalLevel    = 2;  // Yellow blink
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
// Beacon overlays: base color + white, 2 Hz. Auto keeps the base color
// (good / fault / safe-mode). Manual (`findme` / GCS) is white 2 Hz.
// kFaultSafeMode (45) is already blue+white — no extra overlay code.
// ============================================================================
static constexpr uint8_t kFdLandedBeacon        = 12;  // Green   + White alt 2Hz
static constexpr uint8_t kFdAbortBeacon         = 13;  // Red     + White alt 2Hz
static constexpr uint8_t kFaultImuFailBeacon    = 14;  // Red     + White alt 2Hz (IMU fault)
static constexpr uint8_t kFaultEskfFailBeacon   = 15;  // Red     + White alt 2Hz (ESKF fault)
static constexpr uint8_t kFaultBaroFailBeacon   = 16;  // Orange  + White alt 2Hz (baro fault)
static constexpr uint8_t kFaultPioWdtBeacon     = 17;  // Orange  + White alt 2Hz (PIO wdt)
static constexpr uint8_t kFaultCore1StallBeacon = 18;  // Magenta + White alt 2Hz (Core 1 stall)

// ============================================================================
// Flight phase overlays (set by Flight Director actions).
// LedPhaseValue in action_executor.h aliases these codes.
// ============================================================================
static constexpr uint8_t kFdArmed     = 20;  // Red solid (AP parity, Stage L — was amber)
static constexpr uint8_t kFdBoost     = 21;  // Red solid
static constexpr uint8_t kFdCoast     = 22;  // Yellow solid
static constexpr uint8_t kFdDrogue    = 23;  // Red blink
static constexpr uint8_t kFdMain      = 24;  // Red blink
static constexpr uint8_t kFdLanded    = 25;  // Green blink
static constexpr uint8_t kFdAbort     = 26;  // Red fast blink
static constexpr uint8_t kFdBeacon    = 27;  // White blink (distress / find-me locator)

// ============================================================================
// Stage L additions — pre-arm fail and boot init
// ============================================================================
static constexpr uint8_t kFdPreArmFail = 28;  // Yellow double-flash (AP parity, Stage L)
static constexpr uint8_t kFdBootInit   = 29;  // Rainbow (AP parity, Stage L — boot/init warmup)

// ============================================================================
// Sensor status patterns (evaluated by AO_LedEngine priority compositor)
// ============================================================================
static constexpr uint8_t kSensorEskfInit  = 30;  // Red fast blink (hold still)
static constexpr uint8_t kSensorGps3d     = 31;  // Green solid (3D fix)
static constexpr uint8_t kSensorGps2d     = 32;  // Green blink (2D fix)
static constexpr uint8_t kSensorGpsSearch = 33;  // Yellow blink (NMEA, no fix)
static constexpr uint8_t kSensorGpsNoNmea = 34;  // Cyan fast blink (init, no data)
static constexpr uint8_t kSensorNoGps     = 35;  // Blue blink (ESKF, no GPS)
static constexpr uint8_t kSensorTimeout   = 36;  // Magenta solid (5min timeout)

// ============================================================================
// Fault patterns (highest priority — AO_LedEngine fault layer)
// Ascending code = ascending priority. Fault layer uses max() compositor.
// Council: Core1Stall > SafeMode > IMU > ESKF > Baro > PIO (IVP-106)
// ============================================================================
static constexpr uint8_t kFaultPioWdt     = 41;  // Orange solid (PIO watchdog fired)
static constexpr uint8_t kFaultBaroFail   = 42;  // Orange fast blink (baro fault)
static constexpr uint8_t kFaultEskfFail   = 43;  // Red blink (ESKF fault)
static constexpr uint8_t kFaultImuFail    = 44;  // Red fast blink (IMU fault)
static constexpr uint8_t kFaultSafeMode   = 45;  // Blue + White alt 2Hz
static constexpr uint8_t kFaultCore1Stall = 46;  // Magenta solid (Core 1 stalled)

} // namespace led
} // namespace rc

// ============================================================================
// Live aliases (non-namespaced). Canonical names are rc::led::k*.
// Callers: ao_rcos.cpp, ao_led_engine.cpp, action_executor.h.
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
static constexpr uint8_t kFdNeoLandedBeacon     = rc::led::kFdLandedBeacon;      // Stage L
static constexpr uint8_t kFdNeoAbortBeacon      = rc::led::kFdAbortBeacon;       // Stage L
static constexpr uint8_t kFdNeoPreArmFail       = rc::led::kFdPreArmFail;        // Stage L
static constexpr uint8_t kFdNeoBootInit         = rc::led::kFdBootInit;          // Stage L
static constexpr uint8_t kFaultNeoImuFailBeacon    = rc::led::kFaultImuFailBeacon;    // Stage L
static constexpr uint8_t kFaultNeoEskfFailBeacon   = rc::led::kFaultEskfFailBeacon;   // Stage L
static constexpr uint8_t kFaultNeoBaroFailBeacon   = rc::led::kFaultBaroFailBeacon;   // Stage L
static constexpr uint8_t kFaultNeoPioWdtBeacon     = rc::led::kFaultPioWdtBeacon;     // Stage L
static constexpr uint8_t kFaultNeoCore1StallBeacon = rc::led::kFaultCore1StallBeacon; // Stage L

static constexpr uint8_t kSensorNeoEskfInit  = rc::led::kSensorEskfInit;
static constexpr uint8_t kSensorNeoGps3d     = rc::led::kSensorGps3d;
static constexpr uint8_t kSensorNeoGps2d     = rc::led::kSensorGps2d;
static constexpr uint8_t kSensorNeoGpsSearch = rc::led::kSensorGpsSearch;
static constexpr uint8_t kSensorNeoGpsNoNmea = rc::led::kSensorGpsNoNmea;
static constexpr uint8_t kSensorNeoNoGps     = rc::led::kSensorNoGps;
static constexpr uint8_t kSensorNeoTimeout   = rc::led::kSensorTimeout;
static constexpr uint8_t kFaultNeoPioWdt     = rc::led::kFaultPioWdt;
static constexpr uint8_t kFaultNeoBaroFail   = rc::led::kFaultBaroFail;
static constexpr uint8_t kFaultNeoEskfFail   = rc::led::kFaultEskfFail;
static constexpr uint8_t kFaultNeoImuFail    = rc::led::kFaultImuFail;
static constexpr uint8_t kFaultNeoSafeMode   = rc::led::kFaultSafeMode;
static constexpr uint8_t kFaultNeoCore1Stall = rc::led::kFaultCore1Stall;

#endif // ROCKETCHIP_LED_PATTERNS_H
