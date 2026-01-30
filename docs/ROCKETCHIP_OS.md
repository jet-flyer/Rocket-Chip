# RocketChip OS

**Status:** Phase 2 - Sensor Integration
**Last Updated:** 2026-01-29

RocketChip OS is the command-line interface and user interaction system for RocketChip flight computers. It provides serial-based configuration, calibration, and status monitoring.

---

## Overview

RocketChip OS runs over USB CDC serial (115200 baud). It provides:
- Real-time sensor status monitoring
- Sensor calibration routines
- System health information
- Future: Mission configuration, parameter tuning, data download

The CLI is designed to work standalone (via terminal) or alongside MAVLink-based ground control stations (Mission Planner, QGroundControl).

---

## Current Commands (Phase 2)

### Main Menu

| Key | Command | Description |
|-----|---------|-------------|
| **h** | Help | Show system status and available commands |
| **s** | Sensor Status | Display current sensor readings and sample rates |
| **c** | Calibration | Enter calibration sub-menu |

### Calibration Menu (press 'c' to enter)

| Key | Command | Description |
|-----|---------|-------------|
| **w** | Wizard | Guided calibration of all sensors in sequence |
| **l** | Level Cal | Quick accelerometer level calibration (~2 sec) |
| **a** | Accel 6-Pos | Full 6-position accelerometer calibration |
| **m** | Mag Cal | Magnetometer/compass calibration |
| **b** | Baro Cal | Barometer ground pressure calibration |
| **x** | Exit | Return to main menu (also cancels active cal) |

---

## Calibration Wizard

The calibration wizard (`w`) guides users through all sensor calibrations in optimal order:

1. **Level Calibration** - Quick gravity reference (keep device flat)
2. **Barometer Calibration** - Sets ground pressure reference
3. **6-Position Accel Cal** - Full accelerometer calibration (optional)
4. **Compass Calibration** - Magnetometer hard/soft iron correction

At each step:
- Press **ENTER** to proceed
- Press **s** to skip
- Press **x** to cancel wizard

---

## MAVLink Coexistence

RocketChip OS detects MAVLink traffic (start bytes 0xFE/0xFD) and routes appropriately:
- MAVLink messages → GCS parser (for Mission Planner integration)
- ASCII characters → CLI parser

When a MAVLink GCS connects (sends heartbeats), binary telemetry is enabled. Without GCS, only human-readable ASCII output is sent.

---

## Planned Features

### Phase 3 - GPS & Telemetry
- **g** - GPS status and satellite info
- **t** - Telemetry radio status
- **r** - Radio test mode

### Phase 4 - Mission System
- **m** (redefined) - Mission menu
  - Mission selection
  - Parameter editing
  - Event configuration

### Phase 5 - Flight Operations
- **f** - Flight mode selection
- **a** (redefined) - Arm/disarm
- **d** - Data download menu

### Future
- Parameter system (like ArduPilot's PARAM_*)
- Scripting/macro support
- OTA firmware update
- WiFi/Bluetooth CLI access

---

## Implementation Details

### Task Architecture

CLI runs as `CLITask` in FreeRTOS:
- Priority: 5 (same as SensorTask for fair CPU time)
- Stack: 1KB
- Poll rate: 50ms (20Hz)

### Menu State Machine

```
MenuMode::Main
  ├── 'h' → printSystemStatus()
  ├── 's' → SensorTask_PrintStatus()
  └── 'c' → MenuMode::Calibration
              ├── 'w' → runCalibrationWizard()
              ├── 'l' → SensorTask_SimpleAccelCal()
              ├── 'a' → SensorTask_StartAccelCal()
              ├── 'm' → SensorTask_StartCompassCal()
              ├── 'b' → SensorTask_CalibrateBaro()
              └── 'x' → MenuMode::Main
```

### Source Files

| File | Purpose |
|------|---------|
| `src/main.cpp` | CLI task and menu handling |
| `src/services/SensorTask.cpp` | Calibration implementations |
| `lib/ap_compat/GCS_MAVLink/GCS.cpp` | MAVLink parsing and routing |

---

## Testing Checklist

### Main Menu
- [ ] **h** - Shows status with uptime, heap, calibration state
- [ ] **s** - Shows accel/gyro/mag/baro readings and rates
- [ ] **c** - Enters calibration menu

### Calibration Menu
- [ ] **w** - Wizard runs all calibrations in sequence
- [ ] **l** - Level cal completes in ~2 seconds
- [ ] **a** - 6-pos cal prompts for each orientation
- [ ] **m** - Compass cal starts and can be cancelled
- [ ] **b** - Baro cal completes quickly
- [ ] **x** - Returns to main menu

### Persistence
- [ ] Calibration data survives power cycle
- [ ] Status shows "calibrated" state after restart

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 0.2.0 | 2026-01-29 | Calibration sub-menu, wizard mode |
| 0.1.0 | 2026-01-29 | Initial CLI with basic commands |
