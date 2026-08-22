// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file board_pico2.h
 * @brief WIP / unsupported until allowlisted — Raspberry Pi Pico 2
 *
 * Onboard: RP2350A, 4 MB flash, GPIO LED. No PSRAM, DVI, or SD.
 * Expansion defaults: UART0 GPS GPIO 0/1, LoRa breakout pins.
 * Pin map from Pico 2 datasheet §2.1 — not HW-verified as a flight
 * board. Build fails unless PICO2_BRINGUP_OK is set (WN-027).
 */

#ifndef ROCKETCHIP_BOARD_PICO2_H
#define ROCKETCHIP_BOARD_PICO2_H

#ifndef PICO2_BRINGUP_OK
#error "Pico 2 is WIP/unsupported. Define PICO2_BRINGUP_OK only after a documented pin-map allowlist."
#endif

#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

namespace board {

// --- I2C (I2C0 on GPIO 4/5) ---
// Standard Pico pinout. STEMMA QT not onboard; external breakout needed.
inline constexpr uint8_t kI2cSdaPin        = 4;
inline constexpr uint8_t kI2cSclPin        = 5;
#define BOARD_I2C_INSTANCE i2c0

// --- SPI (SPI0 on GPIO 16-19) ---
inline constexpr uint8_t kSpiMisoPin       = 16;
inline constexpr uint8_t kSpiSckPin        = 18;
inline constexpr uint8_t kSpiMosiPin       = 19;
#define BOARD_SPI_INSTANCE spi0

// --- Radio (expansion breakout defaults; not onboard LoRa) ---
// TODO(Pico2): confirm CS / RST / IRQ during allowlisted bring-up.
inline constexpr uint8_t kRadioCsPin       = 17;
inline constexpr uint8_t kRadioRstPin      = 20;
inline constexpr uint8_t kRadioIrqPin      = 21;
inline constexpr bool    kRadioTrustDio0   = false;

// --- NeoPixel ---
// Pico 2 has only a standard GPIO-controlled LED, no RGB. Using 0 as a
// neutral "no NeoPixel chain" sentinel so WS2812 init skips.
inline constexpr uint     kNeoPixelPin     = 0;
inline constexpr uint8_t  kNeoPixelCount   = 0;
inline constexpr uint8_t  kNeoPixelGpioBase = 0;

// --- Pyro / PIO backup (bench GPIO; unverified on this pack) ---
inline constexpr uint8_t kPyroDroguePin    = 12;
inline constexpr uint8_t kPyroMainPin      = 13;

// --- Onboard LED ---
inline constexpr uint8_t kLedPin           = 25;  // Pico 2 onboard LED
inline constexpr bool    kLedActiveHigh    = true;

inline void board_led_set(bool on) {
    gpio_put(kLedPin, on);
}

// --- Shared peripheral RESET ---
inline void board_release_peripheral_reset() {
    // no-op on Pico 2
}

// --- UART GPS (expansion on UART0 GPIO 0/1; not an onboard GPS chip) ---
inline constexpr bool    kUartGpsAvailable = true;
inline constexpr uint8_t kUartGpsTxPin     = 0;
inline constexpr uint8_t kUartGpsRxPin     = 1;

// --- Capability flags (Stage 16C IVP-143) ---
inline constexpr bool    kPsramAvailable       = false;  // no onboard PSRAM
inline constexpr bool    kDvmAvailable         = false;  // no DVI
inline constexpr bool    kSdCardAvailable      = false;  // no onboard SD
inline constexpr bool    kI2cStemmaAvailable   = false;  // external breakout needed

// Same ICM-20948 Z-up convention until this pack is bring-up verified (WN-124).
inline constexpr bool    kImuZUpNed            = true;
inline constexpr uint8_t kMcuTempAdcInput      = 4;  // RP2350A die sensor

// --- Board identity ---
inline constexpr const char* kBoardName = "Raspberry Pi Pico 2";

} // namespace board

#endif // ROCKETCHIP_BOARD_PICO2_H
