// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Board constants for Adafruit Fruit Jam (#6200)
// Onboard: RP2350B, 16 MB flash, 8 MB PSRAM, STEMMA QT I2C0, HSTX DVI,
// SD slot, ESP32-C6, buttons, I2S DAC. ESP/SD/I2S/buttons are present
// on the SKU but not wired as RocketChip APIs (not implemented).
// Expansion: RFM95W on SPI1 adapter (GPIO 5 shared with Button3 — do
// not use Button3 as GPIO input when radio is present). SPI1 also has
// ESP32-C6 CS GPIO 46; keep CS arbitration if WiFi is ever revived.
// UART GPS is not available (GPIO 0/1 are boot / USB Host).
// Store: https://www.adafruit.com/product/6200
// See docs/hardware/BOARD_COMPARISON.md.

#ifndef ROCKETCHIP_BOARD_FRUIT_JAM_H
#define ROCKETCHIP_BOARD_FRUIT_JAM_H

#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/time.h"

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

// --- Radio (expansion adapter; not onboard LoRa) ---
inline constexpr uint8_t kRadioCsPin       = 10;
inline constexpr uint8_t kRadioRstPin      = 6;
inline constexpr uint8_t kRadioIrqPin      = 5;   // Shared with Button3
inline constexpr bool    kRadioTrustDio0   = false;  // DIO0 not trusted on this pack

// --- NeoPixel (WS2812) ---
// GPIO 32 requires PIO gpiobase=16 on RP2350B (pins 16-47 range)
inline constexpr uint     kNeoPixelPin     = 32;
inline constexpr uint8_t  kNeoPixelCount   = 5;
inline constexpr uint8_t  kNeoPixelGpioBase = 16;  // Required for GPIO 32+

// --- Onboard LED ---
// Active-low: HIGH = off, LOW = on.
inline constexpr uint8_t kLedPin           = 29;
inline constexpr bool    kLedActiveHigh    = false;

inline void board_led_set(bool on) {
    // Active-low: on=true → pin LOW
    gpio_put(kLedPin, !on);
}

// --- Shared peripheral RESET ---
// GPIO 22 is the shared active-low RESET line for both the ESP32-C6
// WiFi coprocessor and the TLV320DAC3100 audio DAC. Must be HIGH before
// any I2C scan or the DAC (0x18) and any onboard I2C devices sharing
// this reset will NACK. See BOARD_COMPARISON.md:117,150.
inline constexpr uint8_t kPeripheralResetPin = 22;

inline void board_release_peripheral_reset() {
    // Release shared DAC/ESP32-C6 RESET (active-low). Must be HIGH
    // before any I2C scan.
    gpio_init(kPeripheralResetPin);
    gpio_set_dir(kPeripheralResetPin, GPIO_OUT);
    gpio_put(kPeripheralResetPin, 1);   // release reset → DAC 0x18 appears
    sleep_ms(50);  // minimal stabilization before I2C devices see bus
}

// --- Pyro / PIO backup (same bench GPIO as Feather until a FJ harness exists) ---
inline constexpr uint8_t kPyroDroguePin    = 12;
inline constexpr uint8_t kPyroMainPin      = 13;

// --- PSRAM ---
// GPIO 47 — RP2350B standard PSRAM CS (from Adafruit schematic).
// RP2350B standard PSRAM CS. psram_init(47) must detect 8MB.
inline constexpr uint8_t kPsramCsPin       = 47;

// --- UART GPS (not available; GPIO 0/1 are boot / USB Host) ---
inline constexpr bool    kUartGpsAvailable = false;
inline constexpr uint8_t kUartGpsTxPin     = 0;   // Unused — guard prevents init
inline constexpr uint8_t kUartGpsRxPin     = 0;   // Unused — guard prevents init

// Peripheral presence — shared code branches on these, not board identity.
inline constexpr bool    kPsramAvailable       = true;   // 8 MB
inline constexpr bool    kDvmAvailable         = true;   // HSTX DVI output
inline constexpr bool    kSdCardAvailable      = true;   // SPI0 SD slot
inline constexpr bool    kI2cStemmaAvailable   = true;   // STEMMA QT on I2C0

// Same ICM-20948 Z-up breakout convention as Feather (WN-124).
inline constexpr bool    kImuZUpNed            = true;
inline constexpr uint8_t kMcuTempAdcInput      = 8;  // RP2350B die sensor

// --- Board identity ---
inline constexpr const char* kBoardName = "Adafruit Fruit Jam";

} // namespace board

#endif // ROCKETCHIP_BOARD_FRUIT_JAM_H
