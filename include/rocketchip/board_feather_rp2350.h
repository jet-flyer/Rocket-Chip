// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Board constants for Adafruit Feather RP2350 HSTX (#6130)
// Onboard: RP2350A, 8 MB flash, 8 MB PSRAM (APS6404L CS GPIO 8),
// STEMMA QT I2C1, WS2812 GPIO 21, LED GPIO 7.
// Expansion (not soldered): RFM95W FeatherWing, UART GPS on GPIO 0/1.
// Store: https://www.adafruit.com/product/6130
// See docs/hardware/BOARD_COMPARISON.md.

#ifndef ROCKETCHIP_BOARD_FEATHER_RP2350_H
#define ROCKETCHIP_BOARD_FEATHER_RP2350_H

#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

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

// --- Radio (expansion / FeatherWing defaults; not onboard LoRa) ---
inline constexpr uint8_t kRadioCsPin       = 10;
inline constexpr uint8_t kRadioRstPin      = 11;
inline constexpr uint8_t kRadioIrqPin      = 6;
// Current path reads the SX1276 IRQ register (DIO0 not required).
inline constexpr bool    kRadioTrustDio0   = false;

// --- NeoPixel (WS2812) ---
inline constexpr uint     kNeoPixelPin     = 21;
inline constexpr uint8_t  kNeoPixelCount   = 1;
inline constexpr uint8_t  kNeoPixelGpioBase = 0;   // Default PIO gpiobase

// --- Pyro / PIO backup (bench pins on this pack) ---
inline constexpr uint8_t kPyroDroguePin    = 12;
inline constexpr uint8_t kPyroMainPin      = 13;

// --- Onboard LED ---
inline constexpr uint8_t kLedPin           = 7;
inline constexpr bool    kLedActiveHigh    = true;

inline void board_led_set(bool on) {
    gpio_put(kLedPin, kLedActiveHigh ? on : !on);
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

// --- UART GPS (expansion / FeatherWing; not an onboard GPS chip) ---
inline constexpr bool    kUartGpsAvailable = true;
inline constexpr uint8_t kUartGpsTxPin     = 0;
inline constexpr uint8_t kUartGpsRxPin     = 1;

// --- Capability flags ---
// Compile-time presence. Call sites branch on these (e.g. kPsramAvailable),
// not on board identity.
inline constexpr bool    kPsramAvailable       = true;   // 8 MB APS6404L
inline constexpr bool    kDvmAvailable         = false;  // HSTX not wired
inline constexpr bool    kSdCardAvailable      = false;  // no onboard SD
inline constexpr bool    kI2cStemmaAvailable   = true;   // STEMMA QT on I2C1

// ICM-20948 breakout is Z-up; ESKF is NED (Z-down). Sensor feed negates Z
// when this is true so a second mount cannot inherit the convention silently
// (WN-124). Not a full board_rotation matrix.
inline constexpr bool    kImuZUpNed            = true;
inline constexpr uint8_t kMcuTempAdcInput      = 4;  // RP2350A die sensor

// --- Board identity ---
inline constexpr const char* kBoardName = "Adafruit Feather RP2350 HSTX";

} // namespace board

#endif // ROCKETCHIP_BOARD_FEATHER_RP2350_H
