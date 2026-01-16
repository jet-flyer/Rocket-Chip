# RocketChip Project Status

**Last Updated:** 2026-01-16 by Claude Code CLI

## Current Phase
Phase 2 (Sensor Integration) - All sensors complete. Radio telemetry pending test, then EKF3 fusion.

## Recently Completed
- [x] GPS driver (PA1010D) - NMEA parsing validated on hardware
- [x] Radio driver (RFM95W) - LoRa TX/RX for wireless debugging (pending hardware test)
- [x] Ground station receiver firmware created (ground_station/)
- [x] Council review: EKF3 sensor fusion architecture approved
- [x] ST platform-independent driver integration (council-approved)
  - ISM330DHCX IMU wrapper (accel + gyro)
  - LIS3MDL magnetometer wrapper
  - DPS310 barometer wrapper (ruuvi driver)
- [x] smoke_st_sensors test validated on hardware - all three sensors reading correctly
- [x] HAL implemented and validated (Bus, GPIO, ADC, PWM, Timing, PIO, UART)
- [x] FreeRTOS SMP dual-core operation validated
- [x] Hardware debugging workflow (Debug Probe + OpenOCD + GDB)

## Active Work
- [ ] Radio link hardware validation (two Feather RP2350 + RFM95W FeatherWings)
- [ ] Sensor fusion (EKF3-derived) - all sensors ready, implementation next

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

## Future Documentation Planned
- `docs/ICD.md` - Interface Control Document (Booster Pack connector)
- `docs/TEST_PLAN.md` - Test procedures and criteria
