// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file board_tiny_2350_common.h
 * @brief Shared board constants for Pimoroni Tiny 2350 family
 *
 * Tiny 2350 / Tiny 2350+ share ~95% of their pin map. Variant-specific
 * overrides (kPsramAvailable, kBoardName, flash size) live in
 * board_tiny_2350.h and board_tiny_2350_plus.h.
 *
 * Stage 16C IVP-143: scaffolding only. All pin assignments are
 * datasheet-sourced best-guesses and MUST be verified on hardware
 * before this file is used. The variant headers gate inclusion
 * behind the `TINY_2350_BRINGUP_OK` define which must be added in a
 * dedicated hardware bring-up IVP.
 *
 * Reference: Pimoroni Tiny 2350 schematic + RP2350A (QFN-60) datasheet.
 */

#ifndef ROCKETCHIP_BOARD_TINY_2350_COMMON_H
#define ROCKETCHIP_BOARD_TINY_2350_COMMON_H

#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

namespace board {

// --- I2C (best-guess broken-out pins) ---
// TODO(Tiny_2350): verify on hardware. Tiny has limited broken-out
// pins; user may need an external breakout for STEMMA QT.
inline constexpr uint8_t kI2cSdaPin        = 20;
inline constexpr uint8_t kI2cSclPin        = 21;
#define BOARD_I2C_INSTANCE i2c0

// --- SPI (radio) ---
// TODO(Tiny_2350): verify on hardware. Best-available 4-pin cluster
// for RFM95W or similar.
inline constexpr uint8_t kSpiMisoPin       = 4;
inline constexpr uint8_t kSpiSckPin        = 6;
inline constexpr uint8_t kSpiMosiPin       = 3;
#define BOARD_SPI_INSTANCE spi0

// --- Radio (RFM95W on breakout) ---
// TODO(Tiny_2350): verify on hardware.
inline constexpr uint8_t kRadioCsPin       = 5;
inline constexpr uint8_t kRadioRstPin      = 22;
inline constexpr uint8_t kRadioIrqPin      = 2;

// --- NeoPixel (on-board RGB LED per Pimoroni schematic) ---
// TODO(Tiny_2350): verify NeoPixel pin.
inline constexpr uint     kNeoPixelPin     = 23;
inline constexpr uint8_t  kNeoPixelCount   = 1;
inline constexpr uint8_t  kNeoPixelGpioBase = 0;

// --- Onboard LED ---
// TODO(Tiny_2350): verify active-high/low and pin.
inline constexpr uint8_t kLedPin           = 25;
inline constexpr bool    kLedActiveHigh    = true;

inline void board_led_set(bool on) {
    gpio_put(kLedPin, on);
}

// --- Shared peripheral RESET ---
// Tiny has no shared peripheral RESET line. No-op for API compatibility.
inline void board_release_peripheral_reset() {
    // no-op on Tiny 2350
}

// --- UART GPS ---
// TODO(Tiny_2350): verify pin availability. Base Tiny may lack free
// UART pins after radio + I2C consume the broken-out set.
inline constexpr bool    kUartGpsAvailable = false;
inline constexpr uint8_t kUartGpsTxPin     = 0;
inline constexpr uint8_t kUartGpsRxPin     = 1;

// --- Capability flags (Stage 16C IVP-143) ---
// kPsramAvailable is variant-specific — NOT defined here. Plus variant
// overrides to true; base variant keeps false. kBoardName is also
// variant-specific.
inline constexpr bool    kDvmAvailable       = false;
inline constexpr bool    kSdCardAvailable    = false;
inline constexpr bool    kI2cStemmaAvailable = false;  // external breakout needed

// --- PSRAM CS (only valid on Plus variant) ---
// TODO(Tiny_2350+): confirm PSRAM CS pin from Pimoroni schematic.
inline constexpr uint8_t kPsramCsPin = 21;

} // namespace board

#endif // ROCKETCHIP_BOARD_TINY_2350_COMMON_H
