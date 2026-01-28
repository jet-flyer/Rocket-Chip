# Stub and Dependency Tracker

**Purpose:** Track all stubs, workarounds, and missing dependencies in the ArduPilot integration layer. This document grows as issues are discovered.

**Last Updated:** 2026-01-27

---

## Summary

| Category | Count | Status |
|----------|-------|--------|
| HAL Implementations | 8 | 2 adapted, 6 pending |
| ArduPilot Library Stubs | 15+ | Most need real impl |
| Config Stubs | 10+ | Many can remain stubs |

---

## Critical Priority (Blocks EKF/Flight)

### AP_Param (CRITICAL)

**Location:** `lib/ap_compat/AP_Param/AP_Param.h`

**Current State:** Stub with AP_PARAM_ENABLED=0

**What it stubs:**
- Parameter storage/retrieval system
- AP_Float, AP_Int8, AP_Int16, AP_Int32 types
- AP_ParamV for vector types
- GroupInfo structures
- load/save functionality

**Why it's critical:**
- AP_Param is used by virtually ALL ArduPilot components
- EKF3 parameters, sensor calibration, all settings go through AP_Param
- Without real AP_Param, no persistent configuration

**Dependencies to enable real AP_Param:**
1. `AP_HAL::Storage` - **DONE** (Storage.cpp adapted)
2. `StorageManager` - Need to add to sparse checkout
3. `AP_HAL::Scheduler` - Pending adaptation
4. Flash backend working - **DONE**

**Action:** Enable AP_PARAM_ENABLED=1 and document failures

---

### AP_HAL::Scheduler (HIGH)

**Location:** `lib/ap_compat/AP_HAL_RP2350/Scheduler.h/cpp`

**Current State:** Standalone class, NOT inheriting from AP_HAL::Scheduler

**What's needed:**
- Inherit from `AP_HAL::Scheduler`
- Implement all virtual methods with `override`
- Thread registration (main, timer, io, storage)
- Semaphore integration
- delay_microseconds_boost()

**ESP32 Reference:** `AP_HAL_ESP32/Scheduler.h`

**Blocked by:** Nothing - can adapt now

---

### AP_HAL::Util (HIGH)

**Location:** `lib/ap_compat/AP_HAL_RP2350/Util.h/cpp`

**Current State:** Standalone class, NOT inheriting from AP_HAL::Util

**What's needed:**
- Inherit from `AP_HAL::Util`
- get_soft_armed() / set_soft_armed()
- available_memory()
- get_system_id()
- Safety switch handling

**ESP32 Reference:** `AP_HAL_ESP32/Util.h`

**Blocked by:** Nothing - can adapt now

---

## HAL Layer Status

| Component | File | Inherits AP_HAL? | Status |
|-----------|------|------------------|--------|
| Semaphores | Semaphores.cpp | YES | **ADAPTED** |
| Storage | Storage.cpp | YES | **ADAPTED** |
| Scheduler | Scheduler.cpp | NO | Pending |
| Util | Util.cpp | NO | Pending |
| HAL_RP2350_Class | HAL_RP2350_Class.cpp | NO | Pending |
| GPIO | GPIO.cpp | NO | Pending |
| AnalogIn | AnalogIn.cpp | NO | Pending |
| UARTDriver | UARTDriver.cpp | NO | Pending |
| I2CDevice | I2CDevice.cpp | NO | Pending |
| SPIDevice | SPIDevice.cpp | NO | Pending |

---

## ArduPilot Library Stubs

### AP_AHRS (Stub)

**Location:** `lib/ap_compat/AP_AHRS/AP_AHRS.h`

**What it stubs:**
- get_rotation_body_to_ned()
- get_gyro()
- get_position()
- DCM/EKF attitude estimation

**Dependencies:** AP_InertialSensor, AP_Compass, AP_Baro, AP_GPS, AP_Param

**Status:** Custom stub - blocks full AHRS

---

### AP_GPS (Stub)

**Location:** `lib/ap_compat/AP_GPS/AP_GPS.h`

**What it stubs:**
- GPS state/location
- Status enums

**Real implementation in:** `lib/ardupilot/libraries/AP_GPS/`

**Dependencies:** AP_Param, AP_SerialManager

**Status:** Stub - GPS driver complete in hal_sensors, need AP_GPS wrapper

---

### AP_Compass (Config only)

**Location:** `lib/ap_compat/AP_Compass/AP_Compass_config.h`

**What it provides:**
- AP_COMPASS_ENABLED flag
- Backend enable/disable flags

**Status:** Config stub OK - real AP_Compass needs AP_Param

---

### AP_InternalError (REAL)

**Location:** `lib/ap_compat/AP_InternalError/AP_InternalError.cpp`

**Status:** **REAL IMPLEMENTATION** - not a stub

---

### GCS_MAVLink (Stub)

**Location:** `lib/ap_compat/GCS_MAVLink/GCS.h`

**What it stubs:**
- send_text()
- Message routing
- MAVLink protocol handling

**Dependencies:** AP_Param, AP_SerialManager, mavlink headers

**Status:** Minimal stub - needs real impl for telemetry

---

### AP_BoardConfig (Stub)

**Location:** `lib/ap_compat/AP_BoardConfig/AP_BoardConfig.h`

**What it stubs:**
- Board-specific initialization
- Safety switch handling

**Status:** Stub OK for now

---

### AP_Filesystem (Stub)

**Location:** `lib/ap_compat/AP_Filesystem/AP_Filesystem.h`

**What it stubs:**
- File operations
- ROMFS access

**Dependencies:** LittleFS or FatFS for real impl

**Status:** Stub - blocking AP_Logger

---

### AP_Logger (Stub/Disabled)

**Location:** `lib/ap_compat/AP_Logger/AP_Logger.h`

**What it stubs:**
- Flight logging
- Data recording

**Dependencies:** AP_Filesystem, AP_Param

**Status:** Not in sparse checkout - needs to be added

---

### AP_Scheduler (Stub)

**Location:** `lib/ap_compat/AP_Scheduler/AP_Scheduler.h`

**What it stubs:**
- Task scheduling
- Loop timing

**Dependencies:** AP_HAL::Scheduler, AP_Param

**Status:** Stub - real impl uses HAL scheduler

---

### AP_ExternalAHRS (Stub)

**Location:** `lib/ap_compat/AP_ExternalAHRS/AP_ExternalAHRS.h`

**What it stubs:** External AHRS support

**Status:** Disabled via config - OK

---

### AP_CustomRotations (Stub)

**Location:** `lib/ap_compat/AP_CustomRotations/AP_CustomRotations.h`

**Status:** Minimal stub - rarely needed

---

### AP_Motors_Class (Stub)

**Location:** `lib/ap_compat/AP_Motors/AP_Motors_Class.h`

**Status:** Stub - not needed until motor output

---

### AP_Notify (Stub)

**Location:** `lib/ap_compat/AP_Notify/AP_Notify.h`

**Status:** Stub - can use NeoPixel directly

---

### AP_ROMFS (Stub)

**Location:** `lib/ap_compat/AP_ROMFS/AP_ROMFS.h`

**Status:** Stub - needs filesystem support

---

### AP_RCProtocol (Stub/Config)

**Location:** `lib/ap_compat/AP_RCProtocol/AP_RCProtocol.h`

**Status:** Config stub - not needed yet

---

### AP_MSP (Stub)

**Location:** `lib/ap_compat/AP_MSP/AP_MSP.h`

**Status:** Stub - MSP protocol not used

---

### AP_GyroFFT (Stub)

**Location:** `lib/ap_compat/AP_GyroFFT/AP_GyroFFT.h`

**Status:** Disabled - advanced feature

---

## Config-Only Stubs (Can Remain)

These stubs only provide compile-time configuration and don't need real implementations:

- `AC_Fence/AC_Fence_config.h`
- `AP_AHRS/AP_AHRS_config.h`
- `AP_BoardConfig/AP_BoardConfig_config.h`
- `AP_Compass/AP_Compass_config.h`
- `AP_ExternalAHRS/AP_ExternalAHRS_config.h`
- `AP_Filesystem/AP_Filesystem_config.h`
- `AP_GPS/AP_GPS_config.h`
- `AP_Logger/AP_Logger_config.h`
- `AP_Networking/AP_Networking_Config.h`
- `AP_RCProtocol/AP_RCProtocol_config.h`
- `AP_Rally/AP_Rally_config.h`
- `AP_Scripting/AP_Scripting_config.h`
- `AP_Vehicle/AP_Vehicle_Type.h`
- `GCS_MAVLink/GCS_config.h`

---

## Sparse Checkout Additions Needed

Current sparse checkout may be missing:
- `libraries/StorageManager` - Required for AP_Param
- `libraries/AP_SerialManager` - Required for GPS, GCS
- `libraries/AP_Logger` - Required for logging

Check with:
```bash
cat lib/ardupilot/.git/info/sparse-checkout
```

---

## Build Error Log

### Error 1: xSemaphoreGetMutexHolder not declared

**Date:** 2026-01-27
**File:** Semaphores.cpp
**Fix:** Added `INCLUDE_xSemaphoreGetMutexHolder 1` to FreeRTOSConfig.h

---

## Dependency Chain for EKF3

To enable full EKF3, need (in order):

1. **AP_HAL Layer** (current work)
   - [x] Storage
   - [x] Semaphores
   - [ ] Scheduler
   - [ ] Util
   - [ ] HAL class

2. **AP_Param** (next)
   - [ ] Enable AP_PARAM_ENABLED=1
   - [ ] StorageManager

3. **Sensor Libraries**
   - [ ] AP_InertialSensor (partial)
   - [ ] AP_Compass (need AP_Param)
   - [ ] AP_Baro (need AP_Param)
   - [ ] AP_GPS (need AP_Param)

4. **AHRS**
   - [ ] AP_AHRS (need sensors)
   - [ ] AP_NavEKF3 (need AHRS)

---

## Notes

- Focus on getting AP_Param working first - it unlocks everything else
- ESP32 HAL is the reference implementation for FreeRTOS-based platforms
- Keep stubs minimal - real ArduPilot code is battle-tested
