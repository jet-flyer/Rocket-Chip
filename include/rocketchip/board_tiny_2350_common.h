// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// WIP shared pins for Pimoroni Tiny 2350 (unsupported until allowlisted)
// Datasheet-sourced guesses. There is no board_tiny_2350.h (base variant
// not packed). Pack merge (one Tiny file + PSRAM flag) is later.

#ifndef ROCKETCHIP_BOARD_TINY_2350_COMMON_H
#define ROCKETCHIP_BOARD_TINY_2350_COMMON_H

#ifndef TINY_2350_BRINGUP_OK
#error "Tiny 2350 is WIP/unsupported. Define TINY_2350_BRINGUP_OK only after a documented pin-map allowlist."
#endif

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

// --- Radio (expansion breakout defaults; not onboard LoRa) ---
// TODO(Tiny_2350): verify on hardware.
inline constexpr uint8_t kRadioCsPin       = 5;
inline constexpr uint8_t kRadioRstPin      = 22;
inline constexpr uint8_t kRadioIrqPin      = 2;
inline constexpr bool    kRadioTrustDio0   = false;

// --- NeoPixel (on-board RGB LED per Pimoroni schematic) ---
// TODO(Tiny_2350): verify NeoPixel pin.
inline constexpr uint     kNeoPixelPin     = 23;
inline constexpr uint8_t  kNeoPixelCount   = 1;
inline constexpr uint8_t  kNeoPixelGpioBase = 0;

// --- Pyro / PIO backup (bench GPIO; unverified on this pack) ---
inline constexpr uint8_t kPyroDroguePin    = 12;
inline constexpr uint8_t kPyroMainPin      = 13;

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

// --- UART GPS (expansion; not onboard — currently unavailable on this pack) ---
inline constexpr bool    kUartGpsAvailable = false;
inline constexpr uint8_t kUartGpsTxPin     = 0;
inline constexpr uint8_t kUartGpsRxPin     = 1;

// --- Capability flags ---
// kPsramAvailable and kBoardName are variant-specific — not defined here.
inline constexpr bool    kDvmAvailable       = false;
inline constexpr bool    kSdCardAvailable    = false;
inline constexpr bool    kI2cStemmaAvailable = false;  // external breakout needed

// Same ICM-20948 Z-up convention until this pack is bring-up verified (WN-124).
inline constexpr bool    kImuZUpNed          = true;
inline constexpr uint8_t kMcuTempAdcInput    = 4;  // RP2350A die sensor

// --- PSRAM CS (only valid on Plus variant) ---
// TODO(Tiny_2350+): confirm PSRAM CS pin from Pimoroni schematic.
inline constexpr uint8_t kPsramCsPin = 21;

} // namespace board

#endif // ROCKETCHIP_BOARD_TINY_2350_COMMON_H
