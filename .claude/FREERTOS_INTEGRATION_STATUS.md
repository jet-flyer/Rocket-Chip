# FreeRTOS ArduPilot Integration Status

**Created:** 2026-01-26
**Status:** PAUSED - Evaluating ChibiOS alternative
**Branch:** `freertos-integration`

## Overview

This document captures the state of the FreeRTOS-based ArduPilot integration effort. Work is paused pending evaluation of ChibiOS support for RP2350 which may provide a cleaner path to full ArduPilot compatibility.

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

## ChibiOS Alternative

If ChibiOS now supports RP2350 in ArduPilot trunk:
1. Could use ArduPilot's native build system (waf)
2. Would get AP_HAL_ChibiOS directly
3. All ArduPilot libraries would work without stubs
4. Sensor drivers, logging, params all native

Evaluation needed:
- Check ArduPilot trunk for RP2350/ChibiOS board definitions
- Verify ChibiOS port completeness (USB, flash, etc.)
- Compare effort: continue FreeRTOS stubs vs. adopt ChibiOS

## Decision Point

**If ChibiOS path is viable:** Archive FreeRTOS branch, start fresh with ArduPilot's native build

**If ChibiOS path not ready:** Continue FreeRTOS integration, resolve HAL hierarchy conflicts
