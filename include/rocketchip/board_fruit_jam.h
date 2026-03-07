// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file board_fruit_jam.h
 * @brief Board constants for Adafruit Fruit Jam (#6200)
 *
 * RP2350B, 16MB flash, 8MB PSRAM.
 * I2C0 on STEMMA QT, SPI1 for radio (shared with ESP32-C6).
 *
 * See docs/hardware/BOARD_COMPARISON.md for full pin comparison.
 *
 * [M1] GPIO 5 conflict: shared between radio IRQ (DIO0) and Button3.
 *       When radio is present, Button3 must NOT be configured as GPIO input.
 *       Radio IRQ takes priority — polled in rfm95w_poll_irq().
 *
 * [N1] SPI1 bus sharing: radio CS=GPIO 10, ESP32-C6 CS=GPIO 46.
 *       Mutual exclusion required — only one device active at a time.
 *       Standard SPI multi-device CS arbitration.
 */

#ifndef ROCKETCHIP_BOARD_FRUIT_JAM_H
#define ROCKETCHIP_BOARD_FRUIT_JAM_H

#include "hardware/i2c.h"
#include "hardware/spi.h"

namespace board {

// --- I2C (STEMMA QT / Qwiic) ---
inline constexpr uint8_t kI2cSdaPin       = 20;
inline constexpr uint8_t kI2cSclPin        = 21;
#define BOARD_I2C_INSTANCE i2c0

// --- SPI (radio — shared with ESP32-C6 WiFi, different CS) ---
inline constexpr uint8_t kSpiMisoPin       = 28;
inline constexpr uint8_t kSpiSckPin        = 30;
inline constexpr uint8_t kSpiMosiPin       = 31;
#define BOARD_SPI_INSTANCE spi1

// --- Radio (RFM95W on breakout/FeatherWing adapter) ---
inline constexpr uint8_t kRadioCsPin       = 10;
inline constexpr uint8_t kRadioRstPin      = 6;
inline constexpr uint8_t kRadioIrqPin      = 5;   // [M1] Shared with Button3

// --- NeoPixel (WS2812) ---
// GPIO 32 requires PIO gpiobase=16 on RP2350B (pins 16-47 range)
inline constexpr uint     kNeoPixelPin     = 32;
inline constexpr uint8_t  kNeoPixelCount   = 5;
inline constexpr uint8_t  kNeoPixelGpioBase = 16;  // Required for GPIO 32+

// --- Onboard LED ---
// [M2] Active-low: writing HIGH turns LED off, LOW turns LED on
inline constexpr uint8_t kLedPin           = 29;
inline constexpr bool    kLedActiveHigh    = false;

inline void board_led_set(bool on) {
    // Active-low: on=true → pin LOW
    gpio_put(kLedPin, !on);
}

// --- PSRAM ---
// GPIO 47 — RP2350B standard PSRAM CS (from Adafruit schematic).
// Verify during J.2 parity gate: psram_init(47) must detect 8MB.
inline constexpr uint8_t kPsramCsPin       = 47;

// --- UART GPS ---
// [M3] GPIO 0 = Boot button, GPIO 1 = USB Host D+ on Fruit Jam.
// UART GPS is NOT available on this board.
inline constexpr bool    kUartGpsAvailable = false;
inline constexpr uint8_t kUartGpsTxPin     = 0;   // Unused — guard prevents init
inline constexpr uint8_t kUartGpsRxPin     = 0;   // Unused — guard prevents init

// --- Board identity ---
inline constexpr const char* kBoardName = "Adafruit Fruit Jam";

// --- Fruit Jam extras (not used by Stage J, documented for future) ---
// ESP32-C6 WiFi: CS=GPIO 46, ACK=GPIO 3, RESET=GPIO 22
// SD card: SPI0 (GPIO 34/35/36), CS=GPIO 39, Detect=GPIO 33
// Buttons: Boot=GPIO 0, Button2=GPIO 4, Button3=GPIO 5 (shared with radio IRQ)
// Audio I2S: DIN=GPIO 24, MCLK=GPIO 25, BCLK=GPIO 26, WS=GPIO 27

} // namespace board

#endif // ROCKETCHIP_BOARD_FRUIT_JAM_H
