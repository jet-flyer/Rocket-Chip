# RocketChip Project Status

**Last Updated:** 2026-01-28 by Claude Code CLI

## Current Phase
Phase 2 (Sensor Integration) - FreeRTOS path validated with std::atomic fix. AP_Param integration next.

## Recently Completed
- [x] **PD12 Memory Visibility Fix (std::atomic)** - True dual-core operation achieved
  - AP_InertialSensor flags changed to `std::atomic<bool>` with acquire/release semantics
  - Core pinning removed - FreeRTOS SMP schedules freely across both cores
  - wait_for_sample() working, 1125Hz sample rate confirmed
  - See `docs/FREERTOS_PATH_EVALUATION.md` for full checkpoint status
- [x] **Accelerometer calibration working** - Full 6-position calibration with flash persistence
  - Fixed RP2350 flash page alignment (256-byte writes required)
  - AP_Param system integrated with AP_FlashStorage
  - Calibration survives power cycles
- [x] **AP_HAL_RP2350 Phase 2 complete** - All core peripherals implemented and hardware-validated (23/23 tests)
  - GPIO, AnalogIn, UARTDriver, I2CDevice, SPIDevice
  - Hardware validated: ISM330DHCX, LIS3MDL (I2C), RFM95W (SPI)
  - RCOutput deferred to Titan tier (TVC/servo work)
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
- [x] Hardware verification: AP_HAL_RP2350 Phase 1 (Storage) - **VALIDATED 2026-01-24**
- [x] Hardware verification: AP_HAL_RP2350 Phase 2 (GPIO, AnalogIn, UART, I2C, SPI) - **VALIDATED 2026-01-24**
- [x] Accelerometer calibration with persistence - **VALIDATED 2026-01-24**
- [x] PD12 std::atomic fix - **VALIDATED 2026-01-28**
- [x] Dual-core operation (no core pinning) - **VALIDATED 2026-01-28**
- [ ] **Phase 3: Enable AP_Param** - next up (calibration persistence path)
- [ ] Phase 4: Add AP_Baro/AP_GPS to sparse checkout
- [ ] Phase 5: EKF3 integration decision
- [ ] Magnetometer calibration
- [ ] Hardware verification: rocketchip firmware (production main.cpp + SensorTask)
- [ ] Hardware verification: radio link (two Feather RP2350 + RFM95W FeatherWings)

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

## Future Enhancements

### Pre-Crowdfunding
- **RC_OS** - Serial terminal-based configuration UI
  - Not a true OS, but an interactive terminal interface
  - Handles majority of setup and configuration tasks without needing a PC app
  - Calibration wizards (accel, compass, gyro)
  - Mission configuration editor
  - Firmware upload/update capability
  - System diagnostics and sensor readouts
  - Works with any serial terminal (PuTTY, screen, minicom, etc.)

### Post-Launch
- MCU temperature safety monitoring (like ArduPilot's HAL_WITH_MCU_MONITORING)
  - Track min/max MCU voltage
  - Overheat warning flag via AP_InternalError when temp > 85C
  - Report via MAVLink MCU_STATUS when telemetry available

## Side Projects
- Pegasus flight controller (full FC for drones, glide-back boosters)
- Fruit Jam as potential ground station platform
- OpenMCT integration exploration

## Known Issues
- Legacy Arduino codebase archived in DEPRECATED/ (not carried forward)

## Future Documentation Planned
- `docs/icd/` - Interface Control Documents (expansion connector, protocols)
- `docs/TEST_PLAN.md` - Test procedures and criteria
