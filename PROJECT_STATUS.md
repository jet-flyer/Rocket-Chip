# RocketChip Project Status

**Last Updated:** 2026-01-23 by Claude Code CLI

## Current Phase
Phase 2 (Sensor Integration) - All sensors complete. Radio telemetry pending test, then EKF3 fusion.

## Recently Completed
- [x] Production architecture migration (src/main.cpp + src/services/SensorTask) - **hardware verification pending**
- [x] GPS driver (PA1010D) - NMEA parsing validated on hardware
- [x] Radio driver (RFM95W) - LoRa TX/RX for wireless debugging - **hardware verification pending**
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
- [x] Hardware verification: AP_HAL_RP2350 Storage (smoke_storage test) - **VALIDATED 2026-01-24**
- [ ] Hardware verification: rocketchip firmware (production main.cpp + SensorTask)
- [ ] Hardware verification: radio link (two Feather RP2350 + RFM95W FeatherWings)
- [ ] Sensor fusion (EKF3-derived) - all sensors ready, implementation after hw verification

## Blocked On
Nothing currently.

## Research Tasks (AP_HAL_RP2350 Full Port)
Before implementing each AP_HAL component, check for known RP2040/RP2350 porting issues:

- [ ] Review ArduPilot RP2040 port discussions (Discord, forums, GitHub issues)
- [ ] Check ChibiOS RP2040 port for documented pitfalls
- [ ] Review Pico SDK "gotchas" documentation and GitHub issues
- [ ] Search for FreeRTOS + RP2040/RP2350 multicore issues

**Document findings in:** `docs/RP2350_FULL_AP_PORT.md`

**Key resources to check:**
- ArduPilot Discord #hardware channel
- ArduPilot Discourse forum (RP2040 search)
- GitHub: ardupilot/ardupilot issues (RP2040, Pico)
- GitHub: raspberrypi/pico-sdk issues
- Pico SDK documentation: https://www.raspberrypi.com/documentation/pico-sdk/

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
- `docs/icd/` - Interface Control Documents (expansion connector, protocols)
- `docs/TEST_PLAN.md` - Test procedures and criteria
