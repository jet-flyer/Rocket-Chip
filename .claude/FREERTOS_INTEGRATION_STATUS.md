# FreeRTOS ArduPilot Integration Status

**Created:** 2026-01-26
**Last Updated:** 2026-02-02
**Status:** ACTIVE - FreeRTOS path confirmed as correct approach
**Branch:** `freertos`

## Overview

This document captures the state of the FreeRTOS-based ArduPilot integration effort.

**2026-02-02 Evaluation Result:** ChibiOS was re-evaluated after significant driver progress (USB, Flash, ADC landed in trunk Jan-Feb 2026). Conclusion: **Continue FreeRTOS path.** ChibiOS still has critical gaps (no XIP flash execution, untested RP2350 SMP). See "ChibiOS Evaluation" section below for details.

## What Was Accomplished

### 1. AP_HAL_RP2350 Implementation (Phase 1 & 2 Complete)
- **Scheduler** - FreeRTOS task-based scheduler with timer callbacks
- **Storage** - Flash storage using AP_FlashStorage with XIP-safe operations
- **Util** - System utilities including persistent data structures
- **Semaphores** - FreeRTOS mutex/semaphore wrappers
- **GPIO** - Digital I/O via Pico SDK
- **AnalogIn** - ADC via Pico SDK
- **I2CDevice** - I2C device manager
- **SPIDevice** - SPI device manager (polling mode per PD8)
- **UARTDriver** - UART and USB CDC serial

### 2. ArduPilot Library Integration
Libraries successfully integrated via sparse checkout:
- **AP_Math** - Vectors, matrices, quaternions
- **AP_FlashStorage** - Wear-leveled flash storage
- **Filter** - LowPass, Notch, Derivative filters
- **AP_AccelCal** - Accelerometer calibration (AccelCalibrator)
- **AP_Compass** - Compass calibration (CompassCalibrator available)

### 3. Compatibility Stubs Created
Stubs in `lib/ap_compat/` to satisfy ArduPilot header dependencies:
- AP_AHRS (bootstrap AHRS with complementary filter)
- AP_BoardConfig
- AP_Filesystem
- AP_GPS
- AP_GyroFFT
- AP_InternalError
- AP_Logger
- AP_Motors
- AP_MSP
- AP_Networking
- AP_Notify
- AP_Param (stub - full integration blocked)
- AP_RCProtocol
- AP_Rally
- AP_ROMFS
- AP_Scheduler
- AP_Scripting
- GCS_MAVLink (minimal STATUSTEXT implementation)
- AC_Fence

### 4. Calibration Infrastructure
- CalibrationStore for persistent calibration data
- Accelerometer calibration working with ArduPilot's AccelCalibrator
- Compass calibration infrastructure (paused due to sign convention issues)

## What Remains (FreeRTOS Path)

### Blocked: Full AP_Param Integration
ArduPilot's AP_Param has deep HAL dependencies:
- Requires `hal.util` and `hal.scheduler` pointers
- Expects AP_HAL::HAL class structure different from our RP2350::HAL_RP2350
- Options identified:
  1. Restructure AP_HAL_RP2350 to match ArduPilot's HAL hierarchy
  2. Keep stub approach with AP_PARAM_ENABLED=0
  3. Wait for ChibiOS support

### Blocked: AP_InertialSensor Integration
- Added to sparse checkout but not compiling
- Requires AP_Param, AP_Logger, AP_SerialManager
- ICM-20948 driver available (AP_InertialSensor_Invensensev2)
- Currently using ISM330DHCX with ST drivers instead

### Pending Tasks (if continuing FreeRTOS path)
1. Resolve AP_HAL class hierarchy conflicts
2. Complete AP_InertialSensor integration
3. Complete AP_Logger integration
4. Verify CompassCalibrator with corrected sign convention
5. Update HARDWARE.md for ICM-20948 as primary IMU

## Files Modified/Created This Session

### Modified
- CMakeLists.txt - Added ArduPilot library targets, global board defines
- lib/ap_compat/AP_HAL/*.h - Various HAL stubs expanded
- lib/ap_compat/AP_HAL_Compat.h - Added NEW_NOTHROW, DEV_PRINTF
- lib/ap_compat/AP_BoardConfig/AP_BoardConfig.h - Added allocation_error, etc.
- lib/ap_compat/AP_InternalError/*.cpp/h - Full implementation

### Deleted
- lib/ap_compat/AP_InertialSensor/AP_InertialSensor.cpp
- lib/ap_compat/AP_InertialSensor/AP_InertialSensor.h
- lib/ap_compat/AP_InertialSensor/AP_InertialSensor_config.h

### Created (New Stubs)
- lib/ap_compat/AP_Param/AP_Param.h
- lib/ap_compat/AP_Scheduler/AP_Scheduler.h
- lib/ap_compat/AP_Notify/AP_Notify.h
- lib/ap_compat/AP_GyroFFT/AP_GyroFFT.h
- lib/ap_compat/AP_Motors/AP_Motors_Class.h
- lib/ap_compat/AP_AHRS/AP_AHRS_View.h
- lib/ap_compat/AP_Filesystem/AP_Filesystem.h
- lib/ap_compat/AP_Filesystem/AP_Filesystem_config.h
- lib/ap_compat/AP_Scripting/AP_Scripting_config.h
- lib/ap_compat/AP_Networking/AP_Networking_Config.h
- lib/ap_compat/AP_RCProtocol/AP_RCProtocol.h
- lib/ap_compat/AP_RCProtocol/AP_RCProtocol_config.h
- lib/ap_compat/AP_MSP/AP_MSP.h
- lib/ap_compat/AP_ROMFS/AP_ROMFS.h
- lib/ap_compat/AC_Fence/AC_Fence_config.h
- lib/ap_compat/AP_Rally/AP_Rally_config.h
- lib/ap_compat/GCS_MAVLink/GCS_config.h
- standards/AP_DEPENDENCY_POLICY.md

## Sparse Checkout State

Current `lib/ardupilot/.git/info/sparse-checkout`:
```
/*
!/*/
/libraries/
!/libraries/*/
/libraries/AP_AccelCal/
/libraries/AP_Common/
/libraries/AP_Compass/
/libraries/AP_Declination/
/libraries/AP_FlashStorage/
/libraries/AP_HAL/
/libraries/AP_InertialSensor/
/libraries/AP_InternalError/
/libraries/AP_Logger/
/libraries/AP_Math/
/libraries/AP_Param/
/libraries/AP_SerialManager/
/libraries/Filter/
/libraries/StorageManager/
```

## Platform Differences Documented

See `docs/RP2350_FULL_AP_PORT.md` for:
- PD1: Flash XIP disable during writes
- PD2: SIO FIFO IRQ shared between cores
- PD3: TinyUSB expects Core 1
- PD4: PWM timer sharing
- PD5: Second-stage bootloader
- PD6: No official ChibiOS support (may be changing)
- PD7: I2C clock stretching issues
- PD8: SPI+DMA stops after ~253 cycles
- PD9: GPIO/ADC leakage erratum
- PD10: INS backend rate requirements
- PD11: Memory allocation must zero

## ChibiOS Evaluation (2026-02-02)

### Recent ChibiOS Progress
Significant driver work landed in ChibiOS trunk (Jan-Feb 2026) by contributor "emolitor":
- **USB LLD** - Fixed interrupt handling (Jan 31)
- **ADC registry** - RP2040/RP2350 ADC support (Feb 1)
- **EFL (Flash)** - Embedded Flash Layer drivers (Feb 1)
- **GPIO, I2C, SPI, UART** - Already working

RP2040 support merged to trunk, developer has commit access for RP2350 work.

### Critical Blockers Remaining

1. **No XIP Flash Execution** - ChibiOS RP2040/RP2350 runs from RAM only. XIP from external flash documented as "future priority" but not functional. This is a hard blocker for our 8MB flash firmware.

2. **RP2350 SMP Untested** - ChibiOS has SMP port for Cortex-M0 (RP2040), but Cortex-M33 (RP2350) is different architecture. No evidence of RP2350-specific SMP testing.

3. **ArduPilot Integration Missing** - ArduPilot's ChibiOS fork has no RP2040/RP2350 commits. No board definitions, no waf integration.

### ChibiOS SMP vs FreeRTOS SMP
| Aspect | ChibiOS SMP | FreeRTOS SMP (Ours) |
|--------|-------------|---------------------|
| Thread migration | No (static affinity) | Yes (dynamic) |
| RP2350 tested | No | Yes (PD12-14 fixes) |
| Memory visibility | Unclear | Solved |

### Decision: Continue FreeRTOS

**Rationale:**
- ChibiOS XIP blocker prevents real firmware
- Our FreeRTOS SMP is proven with extensive debugging
- ~60% of our HAL work transfers if ChibiOS becomes viable later
- ArduPilot integration is months away minimum

### What Transfers If We Switch Later
- **Preserves (~60%):** Device driver logic, flash storage, hwdef.h config, platform docs
- **Lost (~40%):** FreeRTOS Scheduler/Semaphores/DeviceBus (RTOS-specific)

### Next Evaluation
Re-check ChibiOS status mid-February 2026. Watch for:
- XIP flash execution support
- RP2350 Cortex-M33 SMP testing
- ArduPilot ChibiOS fork updates

Sources:
- [ChibiOS Forum: RP2350 Support](https://forum.chibios.org/viewtopic.php?f=3&t=6631)
- [ChibiOS GitHub](https://github.com/ChibiOS/ChibiOS/commits/master)
- [ChibiOS SMP Article](http://www.chibios.org/dokuwiki/doku.php?id=chibios:articles:smp_rt7)
