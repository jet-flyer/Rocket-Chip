# ChibiOS Integration Progress

## Phase 0: Pure ChibiOS Validation

| Test | Status | Notes |
|------|--------|-------|
| LED Blink | **PASS** | Verified on hardware 2026-01-27 |
| UART Console | **PASS** | Via debug probe UART, 115200 baud |
| I2C Scan | Not started | |
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

## Phase 1: hwdef Infrastructure

| File | Status | Notes |
|------|--------|-------|
| RP2350xx.py | Not started | |
| chibios_hwdef.py patch | Not started | |
| rp_mcuconf.h | Not started | |
| hwdef.dat | Not started | |

## Phase 2+: ArduPilot Integration

Not started - waiting on Phase 0 & 1.
