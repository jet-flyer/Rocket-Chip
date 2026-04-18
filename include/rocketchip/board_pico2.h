// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file board_pico2.h
 * @brief Board constants for Raspberry Pi Pico 2 (RP2350A)
 *
 * Standard Pico form factor: 40 pins, RP2350A (QFN-60), 4 MB flash,
 * no onboard PSRAM, no DVI, no SD.
 *
 * Stage 16C IVP-143 scaffolding. Pin assignments sourced from the
 * Raspberry Pi Pico 2 datasheet (§2.1 Pinout). Bring-up is gated
 * behind the PICO2_BRINGUP_OK define — a future bring-up IVP must
 * define that symbol after physically verifying radio + I2C wiring
 * on the Pico 2.
 */

#ifndef ROCKETCHIP_BOARD_PICO2_H
#define ROCKETCHIP_BOARD_PICO2_H

#ifndef PICO2_BRINGUP_OK
#error "Pico 2 pin map not yet verified on hardware. Define PICO2_BRINGUP_OK after hardware bring-up."
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

// --- Radio (RFM95W on breakout) ---
// TODO(Pico2): confirm CS / RST / IRQ pin choices during bring-up.
inline constexpr uint8_t kRadioCsPin       = 17;
inline constexpr uint8_t kRadioRstPin      = 20;
inline constexpr uint8_t kRadioIrqPin      = 21;

// --- NeoPixel ---
// Pico 2 has only a standard GPIO-controlled LED, no RGB. Using 0 as a
// neutral "no NeoPixel chain" sentinel so WS2812 init skips.
inline constexpr uint     kNeoPixelPin     = 0;
inline constexpr uint8_t  kNeoPixelCount   = 0;
inline constexpr uint8_t  kNeoPixelGpioBase = 0;

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

// --- UART GPS ---
// GPIO 0/1 are UART0 on Pico 2 and are not multiplexed with USB Host
// or boot buttons (unlike the Fruit Jam). UART GPS is available.
inline constexpr bool    kUartGpsAvailable = true;
inline constexpr uint8_t kUartGpsTxPin     = 0;
inline constexpr uint8_t kUartGpsRxPin     = 1;

// --- Capability flags (Stage 16C IVP-143) ---
inline constexpr bool    kPsramAvailable       = false;  // no onboard PSRAM
inline constexpr bool    kDvmAvailable         = false;  // no DVI
inline constexpr bool    kSdCardAvailable      = false;  // no onboard SD
inline constexpr bool    kI2cStemmaAvailable   = false;  // external breakout needed

// --- Board identity ---
inline constexpr const char* kBoardName = "Raspberry Pi Pico 2";

} // namespace board

#endif // ROCKETCHIP_BOARD_PICO2_H
