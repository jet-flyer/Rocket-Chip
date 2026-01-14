# RocketChip Project Status

**Last Updated:** 2026-01-14 by Claude Code CLI

## Current Phase
Phase 2 (Sensor Integration) - Foundation complete, building sensor drivers.

## Recently Completed
- [x] HAL implemented and validated (Bus, GPIO, ADC, PWM, Timing, PIO, UART)
- [x] I2C sensors responding: ISM330DHCX, LIS3MDL, DPS310
- [x] FreeRTOS SMP dual-core operation validated
- [x] Hardware debugging workflow (Debug Probe + OpenOCD + GDB)

## Active Work
- [ ] Sensor driver implementation (IMU, Baro) using HAL
- [ ] Fix LED blinking in hal_validation smoke test (see Known Issues)
- [ ] Initialize TinyUSB git submodule in pico-sdk (optional)

## Blocked On
Nothing currently.

## Architecture Documents
- `docs/SAD.md` - Software Architecture Document
- `docs/SCAFFOLDING.md` - Directory structure and implementation status

## Back Burner
- Middle tier product name decision
- Ground station software evaluation
- Telemetry protocol final selection (CRSF/CCSDS vs raw MAVLink)

## Side Projects
- Pegasus flight controller (full FC for drones, glide-back boosters)
- Fruit Jam as potential ground station platform
- OpenMCT integration exploration

## Known Issues
- Legacy Arduino codebase archived in DEPRECATED/ (not carried forward)
- **hal_validation LED not blinking**: Red LED (GPIO 13) doesn't blink despite correct gpio calls. NeoPixel works. Suspect HAL GPIO class interaction. Debug probe recommended.

## Future Documentation Planned
- `docs/ICD.md` - Interface Control Document (Booster Pack connector)
- `docs/TEST_PLAN.md` - Test procedures and criteria
