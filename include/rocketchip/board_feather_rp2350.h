// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file board_feather_rp2350.h
 * @brief Board constants for Adafruit Feather RP2350 HSTX (#6130)
 *
 * RP2350A, 8MB flash, 8MB PSRAM (APS6404L, GPIO 8 CS).
 * I2C1 on STEMMA QT, SPI0, UART0 for GPS.
 *
 * See docs/hardware/BOARD_COMPARISON.md for full pin comparison.
 */

#ifndef ROCKETCHIP_BOARD_FEATHER_RP2350_H
#define ROCKETCHIP_BOARD_FEATHER_RP2350_H

#include "hardware/i2c.h"
#include "hardware/spi.h"

namespace board {

// --- I2C (STEMMA QT / Qwiic) ---
inline constexpr uint8_t kI2cSdaPin       = 2;
inline constexpr uint8_t kI2cSclPin        = 3;
#define BOARD_I2C_INSTANCE i2c1

// --- SPI (radio) ---
inline constexpr uint8_t kSpiMisoPin       = 20;
inline constexpr uint8_t kSpiSckPin        = 22;
inline constexpr uint8_t kSpiMosiPin       = 23;
#define BOARD_SPI_INSTANCE spi0

// --- Radio (RFM95W LoRa FeatherWing) ---
inline constexpr uint8_t kRadioCsPin       = 10;
inline constexpr uint8_t kRadioRstPin      = 11;
inline constexpr uint8_t kRadioIrqPin      = 6;

// --- NeoPixel (WS2812) ---
inline constexpr uint     kNeoPixelPin     = 21;
inline constexpr uint8_t  kNeoPixelCount   = 1;
inline constexpr uint8_t  kNeoPixelGpioBase = 0;   // Default PIO gpiobase

// --- Onboard LED ---
inline constexpr uint8_t kLedPin           = 7;
inline constexpr bool    kLedActiveHigh    = true;

inline void board_led_set(bool on) {
    // Active-high: on=true → pin HIGH
    gpio_put(kLedPin, on);
}

// --- Shared peripheral RESET ---
// Feather has no shared peripheral RESET line like the Fruit Jam's
// GPIO 22 (ESP32-C6 + DAC). This is a no-op on this board so role-
// agnostic init code can call board::board_release_peripheral_reset()
// unconditionally.
inline void board_release_peripheral_reset() {
    // no-op on Feather
}

// --- PSRAM ---
inline constexpr uint8_t kPsramCsPin       = 8;

// --- UART GPS ---
// Feather has UART0 on GPIO 0/1 — available for GPS FeatherWing
inline constexpr bool    kUartGpsAvailable = true;
inline constexpr uint8_t kUartGpsTxPin     = 0;
inline constexpr uint8_t kUartGpsRxPin     = 1;

// --- Capability flags (Stage 16C IVP-143) ---
// Peripheral presence flags consumed by role-agnostic shared code
// (health monitor, CLI, telemetry). Let call sites branch on capability
// rather than board identity — makes future board ports (Tiny 2350,
// Pico 2) drop-in without touching shared code.
inline constexpr bool    kPsramAvailable       = true;   // 8 MB APS6404L
inline constexpr bool    kDvmAvailable         = false;  // HSTX not wired
inline constexpr bool    kSdCardAvailable      = false;  // no onboard SD
inline constexpr bool    kI2cStemmaAvailable   = true;   // STEMMA QT on I2C1

// --- Board identity ---
inline constexpr const char* kBoardName = "Adafruit Feather RP2350 HSTX";

} // namespace board

#endif // ROCKETCHIP_BOARD_FEATHER_RP2350_H
