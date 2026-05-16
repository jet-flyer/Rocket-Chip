// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file config.h
 * @brief RocketChip build configuration and pin definitions
 *
 * Per CODING_STANDARDS.md:
 * - Constants use k prefix: kSampleRate, kMaxRetries
 * - Global variables use g_ prefix: g_sensorData
 */

#ifndef ROCKETCHIP_CONFIG_H
#define ROCKETCHIP_CONFIG_H

#include "pico/stdlib.h"
#include "rocketchip/board.h"
#include "rocketchip/job.h"

// ============================================================================
// Runtime Assertions (P10-5 / LOC-3.1 / JSF AV Env 15)
// ============================================================================
//
// RC_ASSERT(expr) — runtime assertion for anomalous conditions.
//
// Debug build:  prints file:line + expression to USB, then spins until
//               watchdog resets the device (reboot-cause flag preserved).
// Release build: compiles to nothing (zero overhead).
//
// Usage:  RC_ASSERT(ptr != NULL);
//         RC_ASSERT(len > 0 && len <= MAX_BUF);
//
// Side-effect safety: expression is NOT evaluated in release builds.
// Do not put statements with side effects inside RC_ASSERT().

#ifdef DEBUG
    #include "rocketchip/rc_log.h"
    #define RC_ASSERT(expr) \
        do { \
            if (!(expr)) { \
                rc::rc_log("[ASSERT] %s:%d: %s\n", __FILE__, __LINE__, #expr); \
                while (true) { __asm volatile("nop"); } \
            } \
        } while (0)
#else
    #define RC_ASSERT(expr) ((void)0)
#endif

// ============================================================================
// Version Information
// ============================================================================

// Version constants — single source of truth in version.h
#include "rocketchip/version.h"
// Legacy alias (used by pcm_frame.cpp, telemetry, etc.)
constexpr const char* kVersionString = kFirmwareVersion;

// ============================================================================
// Feature Flags
// ============================================================================

// Tier selection (only one should be enabled)
// #define ROCKETCHIP_TIER_CORE
#define ROCKETCHIP_TIER_MAIN
// #define ROCKETCHIP_TIER_TITAN

// Feature toggles
#define ROCKETCHIP_FEATURE_USB_CDC      1   // USB serial console
#define ROCKETCHIP_FEATURE_NEOPIXEL     1   // NeoPixel status LED
#define ROCKETCHIP_FEATURE_I2C          1   // I2C bus (sensors, GPS)
#define ROCKETCHIP_FEATURE_SPI          1   // SPI bus (radio, high-rate sensors)
#define ROCKETCHIP_FEATURE_RADIO        1   // LoRa telemetry (RFM95W)

// Device role — behavioral configuration (radio mode, sensor policy, etc.)
// Selected by ROCKETCHIP_JOB_STATION define in CMakeLists.txt.
// Default: vehicle (TX telemetry downlink). See job.h for details.
// "Job" = device role. Distinct from "MissionProfile" (flight profile data).
using job::kRadioModeRx;
using job::kDefaultMavlinkOutput;

// ============================================================================
// Pin Definitions (from HARDWARE.md)
// ============================================================================

namespace rocketchip {
namespace pins {

// Board-abstracted pins — delegated to board:: namespace (see board.h)
constexpr uint8_t kLedRed       = board::kLedPin;
constexpr uint8_t kNeoPixel     = board::kNeoPixelPin;
constexpr uint8_t kPsramCs      = board::kPsramCsPin;

// I2C (STEMMA QT / Qwiic)
constexpr uint8_t kI2cSda       = board::kI2cSdaPin;
constexpr uint8_t kI2cScl       = board::kI2cSclPin;

// UART GPS
constexpr uint8_t kUart0Tx      = board::kUartGpsTxPin;
constexpr uint8_t kUart0Rx      = board::kUartGpsRxPin;

// SPI (radio bus)
constexpr uint8_t kSpiMiso      = board::kSpiMisoPin;
constexpr uint8_t kSpiSck       = board::kSpiSckPin;
constexpr uint8_t kSpiMosi      = board::kSpiMosiPin;

// Radio (RFM95W)
constexpr uint8_t kRadioCs      = board::kRadioCsPin;
constexpr uint8_t kRadioRst     = board::kRadioRstPin;
constexpr uint8_t kRadioIrq     = board::kRadioIrqPin;

} // namespace pins

// ============================================================================
// I2C Addresses (from HARDWARE.md)
// ============================================================================

namespace i2c {

constexpr uint8_t kIcm20948     = 0x69;     // Primary 9-axis IMU (AD0=HIGH default)
constexpr uint8_t kIcm20948Alt  = 0x68;     // ICM-20948 alternate (AD0=LOW)
constexpr uint8_t kDps310       = 0x77;     // Barometer
constexpr uint8_t kDps310Alt    = 0x76;     // DPS310 alternate
constexpr uint8_t kPa1010d      = 0x10;     // GPS module

} // namespace i2c

// ============================================================================
// Timing Configuration
// ============================================================================

namespace timing {

constexpr uint32_t kSensorPollMs    = 10;   // 100Hz sensor polling
constexpr uint32_t kBaroDivider     = 2;    // Baro every 2nd sensor poll = 50Hz
constexpr uint32_t kCliPollMs       = 50;   // 20Hz CLI input polling

} // namespace timing

} // namespace rocketchip

// ============================================================================
// Debug Macros (from DEBUG_OUTPUT.md)
// ============================================================================

// C++20 constexpr debug toggle — JSF AV Rule 26/28 compliant.
// Single #ifdef DEBUG bridge sets constexpr bool; all downstream code
// uses if constexpr (zero overhead when disabled, compiles to bx lr).
#ifdef DEBUG
inline constexpr bool kDebugEnabled = true;
#else
inline constexpr bool kDebugEnabled = false;
#endif

#include "pico/time.h"
#include "rocketchip/rc_log.h"

// R-5 Unit B step 4 (2026-05-16): backend repointed from <stdio.h>
// printf to rc::rc_log. DBG_PRINT/DBG_ERROR/DBG_STATE call surfaces
// stay identical for callers — this is the macro-repoint commit zero
// move (ArduPilot round 1 amendment: "cheap, high-leverage, runs the
// replacement under realistic load before the manual callsite
// migration starts"). config.h no longer includes <stdio.h> or
// <cstdio>, eliminating the transitive-leak path that the Unit A
// inventory surfaced (5 hidden-leak files via config.h's old DBG_PRINT
// macros pulling printf into translation units that didn't directly
// include <stdio.h>).
//
// Per the rc::rc_log contract (include/rocketchip/rc_log.h), each call
// emits at most 128 bytes to a non-blocking USB CDC ring; output that
// overflows is truncated with "...\n" marker. The "[%lu] " timestamp
// prefix + newline are baked into the format strings below.

template<typename... Args>
inline void dbg_print(const char* fmt, Args... args) {
    if constexpr (kDebugEnabled) {
        rc::rc_log("[%lu] ", (unsigned long)time_us_32());
        rc::rc_log(fmt, args...);
        rc::rc_log("\n");
    }
}

template<typename... Args>
inline void dbg_error(const char* fmt, Args... args) {
    if constexpr (kDebugEnabled) {
        rc::rc_log("[%lu] ERROR: ", (unsigned long)time_us_32());
        rc::rc_log(fmt, args...);
        rc::rc_log("\n");
    }
}

inline void dbg_state(const char* from, const char* to) {
    if constexpr (kDebugEnabled) {
        rc::rc_log("[%lu] State: %s -> %s\n", (unsigned long)time_us_32(), from, to);
    }
}

// Compatibility macros — map old names to C++20 template functions
#define DBG_PRINT(fmt, ...) dbg_print(fmt, ##__VA_ARGS__)
#define DBG_STATE(from, to) dbg_state(from, to)
#define DBG_ERROR(fmt, ...) dbg_error(fmt, ##__VA_ARGS__)

#endif // ROCKETCHIP_CONFIG_H
