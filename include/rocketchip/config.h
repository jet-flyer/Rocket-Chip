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

// ============================================================================
// Version Information
// ============================================================================

#define ROCKETCHIP_VERSION_MAJOR    0
#define ROCKETCHIP_VERSION_MINOR    1
#define ROCKETCHIP_VERSION_PATCH    0
#define ROCKETCHIP_VERSION_STRING   "0.1.0"

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
// #define ROCKETCHIP_FEATURE_SPI       1   // SPI bus (high-rate sensors)
// #define ROCKETCHIP_FEATURE_RADIO     1   // LoRa telemetry

// ============================================================================
// Pin Definitions (from HARDWARE.md) - C++ only
// ============================================================================

#ifdef __cplusplus
namespace rocketchip {
namespace pins {

// Built-in hardware (Feather RP2350 HSTX)
constexpr uint8_t kLedRed       = 7;        // Built-in red LED
constexpr uint8_t kNeoPixel     = 21;       // Built-in NeoPixel
constexpr uint8_t kPsramCs      = 8;        // DO NOT USE if PSRAM installed

// I2C1 (STEMMA QT / Qwiic connector)
constexpr uint8_t kI2c1Sda      = 2;        // I2C1 SDA
constexpr uint8_t kI2c1Scl      = 3;        // I2C1 SCL

// UART0 (Serial1)
constexpr uint8_t kUart0Tx      = 0;        // UART0 TX
constexpr uint8_t kUart0Rx      = 1;        // UART0 RX

// SPI0 (default SPI)
constexpr uint8_t kSpi0Miso     = 20;       // SPI0 MISO
constexpr uint8_t kSpi0Sck      = 22;       // SPI0 SCK
constexpr uint8_t kSpi0Mosi     = 23;       // SPI0 MOSI

// ADC inputs
constexpr uint8_t kAdcA0        = 26;       // ADC channel 0
constexpr uint8_t kAdcA1        = 27;       // ADC channel 1
constexpr uint8_t kAdcA2        = 28;       // ADC channel 2
constexpr uint8_t kAdcA3        = 29;       // ADC channel 3

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
// Task Configuration (from SCAFFOLDING.md)
// ============================================================================

namespace task {

// Task priorities (higher = more important)
constexpr uint8_t kPrioritySensor   = 5;    // SensorTask - highest
constexpr uint8_t kPriorityControl  = 5;    // ControlTask (Titan only)
constexpr uint8_t kPriorityFusion   = 4;    // FusionTask
constexpr uint8_t kPriorityMission  = 4;    // MissionTask
constexpr uint8_t kPriorityLogger   = 3;    // LoggerTask
constexpr uint8_t kPriorityTelemetry = 2;   // TelemetryTask
constexpr uint8_t kPriorityUi       = 1;    // UITask - lowest

// Stack sizes (in words, multiply by 4 for bytes)
constexpr uint16_t kStackSensor     = 256;  // 1KB
constexpr uint16_t kStackFusion     = 512;  // 2KB
constexpr uint16_t kStackMission    = 512;  // 2KB
constexpr uint16_t kStackLogger     = 512;  // 2KB
constexpr uint16_t kStackTelemetry  = 256;  // 1KB
constexpr uint16_t kStackUi         = 256;  // 1KB

// Core affinity
constexpr uint8_t kCoreRealtime     = 0;    // Sensor, Control tasks
constexpr uint8_t kCoreBackground   = 1;    // Fusion, Mission, Logger, UI, Telemetry

} // namespace task

} // namespace rocketchip
#endif // __cplusplus

// ============================================================================
// Debug Macros (from DEBUG_OUTPUT.md)
// ============================================================================
//
// Uses deferred logging via FreeRTOS Stream Buffer to avoid blocking
// high-priority tasks on USB CDC. Safe to call from any task/priority.
//
// IMPORTANT: debug_stream_init() must be called before scheduler starts,
// and debug_stream_flush() must be called periodically from UITask.
// ============================================================================

#ifdef DEBUG
    #include "pico/time.h"
    #include "debug/debug_stream.h"
    #define DBG_PRINT(fmt, ...) dbg_printf("[%lu] " fmt "\n", (unsigned long)time_us_32(), ##__VA_ARGS__)
    #define DBG_TASK(name) DBG_PRINT("[%s] tick", name)
    #define DBG_STATE(from, to) DBG_PRINT("State: %s -> %s", from, to)
    #define DBG_ERROR(fmt, ...) dbg_printf("[%lu] ERROR: " fmt "\n", (unsigned long)time_us_32(), ##__VA_ARGS__)
#else
    #define DBG_PRINT(fmt, ...) ((void)0)
    #define DBG_TASK(name) ((void)0)
    #define DBG_STATE(from, to) ((void)0)
    #define DBG_ERROR(fmt, ...) ((void)0)
#endif

#endif // ROCKETCHIP_CONFIG_H
