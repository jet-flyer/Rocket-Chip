# ChibiOS Migration

## Why ChibiOS?

RocketChip originally planned a custom FreeRTOS-based HAL for ArduPilot. In January 2026, ChibiOS merged upstream RP2350 support, making it the better path:

| Approach | Pros | Cons |
|----------|------|------|
| **FreeRTOS HAL** (original) | Full control | ~5000 lines of custom HAL code |
| **ChibiOS HAL** (new) | Reuse existing AP_HAL_ChibiOS | ~2000 lines of extensions |

ChibiOS reduces implementation effort by ~60% while providing a path to upstream contribution.

## What Changed

The FreeRTOS implementation is preserved in the `freertos` branch:
- Custom FreeRTOS HAL stubs
- CMake + Pico SDK build system
- ArduPilot math libraries integration

Main branch now targets ChibiOS:
- Extends existing AP_HAL_ChibiOS
- Uses ArduPilot's hwdef system
- Standard ArduPilot build (waf)

## Implementation Phases

### Phase 0: Pure ChibiOS Validation
Prove ChibiOS runs on hardware without ArduPilot complexity.
- LED blink, UART, I2C, SPI tests
- Location: `chibios/phase0_validation/`

### Phase 1: hwdef Infrastructure
Create ArduPilot board definition files:
- `RP2350xx.py` - Pin database
- `hwdef.dat` - Board configuration
- `chibios_hwdef.py` patches

### Phase 2: ArduPilot Compilation
```bash
./waf configure --board=RocketChip
./waf copter
```

### Phase 3: Boot & Sensor Validation
- Console output with ArduPilot banner
- All sensors detected

## Hardware (unchanged)

- **MCU:** Adafruit Feather RP2350 HSTX
- **IMU:** ICM20948 (I2C 0x68)
- **Mag:** LIS3MDL (I2C 0x1E)
- **Baro:** DPS310 (I2C 0x77)
- **GPS:** PA1010D (UART)

## Resources

- ChibiOS RP2350: `os/hal/ports/RP/RP2350/` (merged Jan 8, 2026)
- ArduPilot hwdef: `libraries/AP_HAL_ChibiOS/hwdef/`
- FreeRTOS branch: `git checkout freertos`
