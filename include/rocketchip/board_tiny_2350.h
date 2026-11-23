// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Board constants for Pimoroni Tiny 2350 (PIM721).
// RP2350A (QFN-60), 4 MB flash, no PSRAM. Onboard pins from Pico SDK 2.2.0
// boards/pimoroni_tiny2350.h and https://shop.pimoroni.com/products/tiny-2350.
// Broken-out IO is GP0-7 and GP26-29; GP12/13 are on the Qw/ST connector only.
// I2C and the onboard RGB LED were benched on PIM721 (Tiny 2350 bring-up,
// 2026-11). Radio SPI and pyro GPIOs are expansion defaults, not onboard,
// and are unverified. There is no Tiny 2350 Plus; PSRAM lives on other
// Pimoroni boards (Pico Plus 2), not this one.

#ifndef ROCKETCHIP_BOARD_TINY_2350_H
#define ROCKETCHIP_BOARD_TINY_2350_H

#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

namespace board {

// --- I2C (I2C0 on the Qw/ST connector; SDK PICO_DEFAULT_I2C_SDA/SCL_PIN) ---
inline constexpr uint8_t kI2cSdaPin        = 12;
inline constexpr uint8_t kI2cSclPin        = 13;
#define BOARD_I2C_INSTANCE i2c0

// --- SPI (radio) ---
// TODO(Tiny_2350): verify on hardware. Best-available 4-pin cluster
// for RFM95W or similar. No onboard radio.
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

// --- NeoPixel ---
// No WS2812 on this board (SDK: no PICO_DEFAULT_WS2812_PIN). Count 0 skips
// WS2812 init. Pin 0 is UART GPS TX; do not mux PIO.
inline constexpr uint     kNeoPixelPin     = 0;
inline constexpr uint8_t  kNeoPixelCount   = 0;
inline constexpr uint8_t  kNeoPixelGpioBase = 0;

// --- Pyro / PIO backup (bench GPIO; unverified on this pack) ---
// GP26/27 are free broken-out pins; GP12/13 are the Qw/ST I2C. No pyro
// hardware is fitted (kPyroAvailable false), so boot does not claim them:
// main.cpp skips pio_backup_timer_init on packs without pyro.
inline constexpr uint8_t kPyroDroguePin    = 26;
inline constexpr uint8_t kPyroMainPin      = 27;

// --- Onboard RGB LED (discrete, active low, product page) ---
inline constexpr uint8_t kLedRedPin        = 18;
inline constexpr uint8_t kLedGreenPin      = 19;
inline constexpr uint8_t kLedBluePin       = 20;
// Cross-check against the SDK board header (pimoroni_tiny2350.h) so a
// colour/pin swap cannot build: R=GP18 (PWM 1A), G=GP19 (1B), B=GP20 (2A).
#if defined(TINY2350_LED_R_PIN) && defined(TINY2350_LED_G_PIN) && defined(TINY2350_LED_B_PIN)
static_assert(kLedRedPin == TINY2350_LED_R_PIN, "Tiny 2350 red LED pin");
static_assert(kLedGreenPin == TINY2350_LED_G_PIN, "Tiny 2350 green LED pin");
static_assert(kLedBluePin == TINY2350_LED_B_PIN, "Tiny 2350 blue LED pin");
#endif

// --- Onboard LED (green channel; SDK PICO_DEFAULT_LED_PIN) ---
// This is the RGB LED's green channel, owned by the status LED as PWM
// (kOnboardLedIsRgbChannel); boot does not claim it as a GPIO.
inline constexpr uint8_t kLedPin           = kLedGreenPin;
inline constexpr bool    kLedActiveHigh    = false;

inline void board_led_set(bool on) {
    gpio_put(kLedPin, kLedActiveHigh ? on : !on);
}

// --- BOOT button (user switch, active low, product page) ---
inline constexpr uint8_t kBootSwPin        = 23;

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
inline constexpr bool        kPsramAvailable     = false;
inline constexpr const char* kBoardName          = "Pimoroni Tiny 2350";
inline constexpr bool        kDvmAvailable       = false;
inline constexpr bool        kSdCardAvailable    = false;
inline constexpr bool        kI2cStemmaAvailable = true;   // Qw/ST connector
inline constexpr bool        kPyroAvailable       = false;  // no pyro fitted
inline constexpr bool        kRadioAvailable      = false;  // local-logging tracker; no radio
inline constexpr bool        kRgbLedAvailable     = true;   // discrete RGB on GP18-20
inline constexpr bool        kLedRgbActiveLow     = true;

// Same ICM-20948 Z-up convention until this pack is bring-up verified (WN-124).
inline constexpr bool    kImuZUpNed          = true;
inline constexpr uint8_t kMcuTempAdcInput    = 4;  // RP2350A die sensor

// Required pack member: src/main.cpp passes kPsramCsPin to psram_init inside
// a runtime if (kPsramAvailable), so the symbol is still compiled. PIM721
// has no PSRAM; the value is unused.
inline constexpr uint8_t kPsramCsPin = 21;

} // namespace board

#endif // ROCKETCHIP_BOARD_TINY_2350_H
