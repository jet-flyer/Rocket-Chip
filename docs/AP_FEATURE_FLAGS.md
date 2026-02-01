# ArduPilot Feature Flags Reference

**Purpose:** Track all ArduPilot `_ENABLED` compile flags, their status in RocketChip, and rationale for each decision.

**Last Updated:** 2026-01-31

---

## Overview

ArduPilot uses `_ENABLED` compile flags to conditionally include features. This document tracks which features are enabled/disabled in RocketChip and provides guidance for enabling features in the future.

---

## Quick Reference

### Enabled Features

| Flag | Value | Rationale |
|------|-------|-----------|
| `AP_INERTIALSENSOR_ENABLED` | 1 | IMU support (ICM-20948) |
| `AP_COMPASS_ENABLED` | 1 | Magnetometer support (AK09916) |
| `AP_BARO_ENABLED` | 1 | Barometer support (DPS310) |
| `AP_PARAM_ENABLED` | 1 | Parameter storage - required for calibration persistence |
| `HAL_GCS_ENABLED` | 1 | MAVLink/GCS support for calibration commands |
| `HAL_INS_ACCELCAL_ENABLED` | 1 | Accelerometer calibration support |

### Disabled Features (Not Currently Needed)

| Flag | Value | Rationale |
|------|-------|-----------|
| `AP_VEHICLE_ENABLED` | 0 | We use our own Rocket class, not full AP_Vehicle |
| `AP_GPS_ENABLED` | 0 | Not yet integrated |
| `AP_AHRS_ENABLED` | 0 | Using stub, not full AHRS |
| `HAL_LOGGING_ENABLED` | 0 | Using custom logging |
| `AP_SCHEDULER_ENABLED` | 0 | Using FreeRTOS directly |
| `AP_SERIALMANAGER_ENABLED` | 0 | Using custom serial management |

### Disabled Features (Hardware Not Present)

| Flag | Value | Rationale |
|------|-------|-----------|
| `HAL_BUTTON_ENABLED` | 0 | No hardware buttons |
| `HAL_EFI_ENABLED` | 0 | No EFI system |
| `AP_RANGEFINDER_ENABLED` | 0 | No rangefinder |
| `AP_RELAY_ENABLED` | 0 | No relay outputs |
| `AP_RSSI_ENABLED` | 0 | No RSSI sensor |
| `AP_SERVORELAYEVENTS_ENABLED` | 0 | No servo relay |
| `AP_OPENDRONEID_ENABLED` | 0 | No Remote ID |
| `HAL_HOTT_TELEM_ENABLED` | 0 | No HoTT telemetry |
| `HAL_WITH_ESC_TELEM` | 0 | No ESC telemetry |
| `AP_ESC_TELEM_ENABLED` | 0 | No ESC telemetry |
| `AP_SERVO_TELEM_ENABLED` | 0 | No servo telemetry |
| `AP_VIDEOTX_ENABLED` | 0 | No video transmitter |
| `AP_SMARTAUDIO_ENABLED` | 0 | No SmartAudio |
| `AP_TRAMP_ENABLED` | 0 | No Tramp VTX |
| `HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL` | 0 | No FrSky telemetry |
| `AP_TEMPERATURE_SENSOR_ENABLED` | 0 | No temp sensors |
| `AP_STATS_ENABLED` | 0 | Stats not needed |
| `HAL_GENERATOR_ENABLED` | 0 | No generator |
| `HAL_VISUALODOM_ENABLED` | 0 | No visual odometry |
| `AP_AIS_ENABLED` | 0 | No AIS system |
| `HAL_NMEA_OUTPUT_ENABLED` | 0 | No NMEA output |
| `AP_KDECAN_ENABLED` | 0 | No KDE CAN |
| `AP_IBUS_TELEM_ENABLED` | 0 | No iBUS telemetry |
| `AP_GRIPPER_ENABLED` | 0 | No gripper |
| `AP_RPM_ENABLED` | 0 | No RPM sensor |
| `AP_SIM_ENABLED` | 0 | Not simulation |
| `AP_FILTER_ENABLED` | 0 | AP_Filter not needed |
| `AP_DDS_ENABLED` | 0 | No DDS/ROS2 |
| `AP_EXTERNAL_CONTROL_ENABLED` | 0 | No external control |
| `AP_NETWORKING_ENABLED` | 0 | No networking |
| `HAL_MSP_ENABLED` | 0 | No MSP telemetry |
| `AP_SCRIPTING_ENABLED` | 0 | No Lua scripting |
| `AP_CUSTOMROTATIONS_ENABLED` | 0 | No custom rotations |
| `AP_EXTERNAL_AHRS_ENABLED` | 0 | No external AHRS |
| `OSD_ENABLED` | 0 | No OSD |
| `AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED` | 0 | Notch filter not needed for rockets |

### Disabled Features (Compliance Review Required)

These features are intentionally disabled and require explicit review before enabling:

| Flag | Value | Rationale |
|------|-------|-----------|
| `AP_FENCE_ENABLED` | 0 | Geofencing - compliance review needed |
| `AP_RALLY_ENABLED` | 0 | Rally points - compliance review needed |
| `AP_MISSION_ENABLED` | 0 | Autonomous missions - compliance review needed |

---

## How to Enable a Feature

1. **Change flag value** in `CMakeLists.txt`:
   ```cmake
   AP_FEATURENAME_ENABLED=1  # Was 0
   ```

2. **Add library to sparse-checkout** if needed:
   ```bash
   cd lib/ardupilot
   git sparse-checkout add libraries/AP_FeatureName
   ```

3. **Remove stub** from `lib/ap_compat/AP_FeatureName/` if exists

4. **Add source files** to CMakeLists.txt if needed

5. **Update this document** with new status

6. **Test thoroughly** - build and verify functionality

---

## Stub Locations

Stubs for disabled features are in `lib/ap_compat/`:

| Feature | Stub Location |
|---------|---------------|
| AP_AHRS | `lib/ap_compat/AP_AHRS/` |
| AP_Scheduler | `lib/ap_compat/AP_Scheduler/` |
| AP_Logger | `lib/ap_compat/AP_Logger/` |
| AP_GPS | `lib/ap_compat/AP_GPS/` |
| GCS_MAVLink | `lib/ap_compat/GCS_MAVLink/` |
| Many others | `lib/ap_compat/<LibraryName>/` |

---

## Stub Audit TODO

After AP_Vehicle integration is stable, audit these stubs to determine if they can be replaced with proper `_ENABLED=0` flags:

- [ ] `lib/ap_compat/AP_AHRS/` - Full AHRS might be needed for EKF3
- [ ] `lib/ap_compat/AP_Scheduler/` - Using FreeRTOS, may not need
- [ ] `lib/ap_compat/AP_Logger/` - Custom logging, may not need
- [ ] `lib/ap_compat/AP_GPS/` - Will enable when GPS integrated
- [ ] `lib/ap_compat/AP_Notify/` - LED patterns, may want full version
- [ ] `lib/ap_compat/AP_BoardConfig/` - Board config stub
- [ ] `lib/ap_compat/AP_BattMonitor/` - Battery monitoring stub

---

## Flag Definition Locations

Flags are defined in CMakeLists.txt around line 54-130 in the global `add_compile_definitions()` block.

Some flags are also set per-library for specific backends (e.g., `ap_inertial_sensor`, `ap_compass`, `ap_baro`).

---

## Version History

| Date | Change |
|------|--------|
| 2026-01-31 | Initial document created during AP_Vehicle integration |
