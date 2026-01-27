# Phase 0: Pure ChibiOS Validation

These tests validate ChibiOS functionality on RP2350 WITHOUT ArduPilot.

## Prerequisites

```bash
# Clone ChibiOS
git clone https://github.com/ChibiOS/ChibiOS.git
cd ChibiOS && git checkout trunk

# ARM toolchain
# macOS: brew install --cask gcc-arm-embedded
# Linux: sudo apt install gcc-arm-none-eabi
```

## Test Order

1. test_led_blink - Kernel boot, PAL driver
2. test_uart - Serial driver, clocks
3. test_i2c_scan - I2C driver
4. test_spi - SPI driver

## Building

```bash
cd test_led_blink
make
# Flash: picotool load -f build/ch.uf2
```
