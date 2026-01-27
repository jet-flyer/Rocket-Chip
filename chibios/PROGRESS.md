# ChibiOS Integration Progress

## Phase 0: Pure ChibiOS Validation

| Test | Status | Notes |
|------|--------|-------|
| LED Blink | **PASS** | Verified on hardware 2026-01-27 |
| UART Console | **PASS** | Via debug probe UART, 115200 baud |
| USB (ChibiOS HAL) | **BLOCKED** | HAL crashes device - see CI1 |
| USB (TinyUSB) | **BLOCKED** | Pico SDK conflicts with ChibiOS - see CI1 |
| I2C Scan | **BUILD OK** | Using software I2C (fallback driver) - hardware I2C is stub |
| SPI Loopback | Not started | |

### LED Blink Build Notes

- **Date**: 2026-01-27
- **Build size**: 7,480 bytes text, 17KB UF2
- **Workarounds applied**:
  - `USE_SMART_BUILD=no` (Unix shell commands in hal.mk fail on Windows)
  - `USE_LTO=no` (LTO conflicts with portable make.exe on Windows)
  - `CH_CFG_SMP_MODE=FALSE` (ARMv8-M-ML port doesn't support kernel SMP)
  - `RP_CORE1_START=FALSE` (temporarily disabled, re-enable later)
  - Manually added `hal_safety.c` and `hal_buffered_serial.c` to CSRC (missing from hal.mk non-smart-build path)

### UART Console Build Notes

- **Date**: 2026-01-27
- **Build size**: 8,680 bytes text
- **Features**:
  - ChibiOS SIO driver for UART0
  - chprintf stream wrapper for debug output
  - Connects via debug probe UART pass-through (GPIO0 TX)
  - 115200 baud, 8N1

### I2C Scan Notes

- **Date**: 2026-01-27
- **Build size**: 11,540 bytes text
- **Driver**: ChibiOS fallback/software I2C (bit-bang via GPIO)
- **Reason**: Hardware I2C driver (I2Cv1) is a stub - empty function bodies (see CI13 in KNOWN_ISSUES.md)
- **Pins**: GPIO2 (SDA), GPIO3 (SCL) - STEMMA QT/QWIIC port
- **GPIO mode**: Push-pull with pull-ups (RP2350 PAL lacks open-drain mode definition)
- **Alternative found**: ChibiOS-Contrib has RP2040 I2C driver used by QMK, could potentially be adapted
- **Expected devices**:
  - 0x77: DPS310 barometer
  - 0x6A/0x6B: ISM330DHCX IMU
  - 0x1C/0x1E: LIS3MDL magnetometer (if connected)
- **To test**: Flash UF2, connect debug probe UART, open 115200 baud terminal

## Phase 1: hwdef Infrastructure

| File | Status | Notes |
|------|--------|-------|
| RP2350xx.py | Not started | |
| chibios_hwdef.py patch | Not started | |
| rp_mcuconf.h | Not started | |
| hwdef.dat | Not started | |

## Phase 2+: ArduPilot Integration

Not started - waiting on Phase 0 & 1.
