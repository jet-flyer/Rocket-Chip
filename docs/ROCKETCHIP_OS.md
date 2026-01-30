# RocketChip OS

**Status:** Phase 2 - Sensor Integration
**Last Updated:** 2026-01-30

RocketChip OS (RC_OS) is the command-line interface and user interaction system for RocketChip flight computers. It provides serial-based configuration, calibration, and status monitoring.

---

## Overview

RocketChip OS runs over USB CDC serial. It provides:
- Real-time sensor status monitoring
- Sensor calibration routines
- System health information
- Future: Mission configuration, parameter tuning, data download

The CLI is designed to work standalone (via terminal) or alongside MAVLink-based ground control stations (Mission Planner, QGroundControl).

### Design Philosophy

RC_OS follows patterns proven in the old APM CLI (ArduPilot ~2012-2014) while adapted for modern FreeRTOS architecture:

| Pattern | Implementation |
|---------|---------------|
| Single-key commands | No complex parsing, immediate response |
| Menu state machine | Two-level: Main → Calibration |
| Non-blocking input | `getchar_timeout_us(0)` polling |
| Terminal compatibility | Handles CR, LF, and CR+LF |
| Visual feedback | LED blink patterns during operations |

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
| **l** | Level Cal | Quick accelerometer level calibration |
| **a** | Accel 6-Pos | Full 6-position accelerometer calibration |
| **g** | Gyro Cal | Gyroscope calibration |
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

## MAVLink Integration (v0.3)

### Internal MAVLink Architecture

RC_OS v0.3 uses an **internal MAVLink** architecture where CLI commands are translated to MAVLink commands internally. This ensures CLI and external GCS (Mission Planner, QGroundControl) use identical code paths.

```
User types 'l' (level cal)
       ↓
CLITask (only runs when terminal connected)
       ↓
RC_OS::cmd_level_cal()
       ↓
GCS::send_local_command(MAV_CMD_PREFLIGHT_CALIBRATION, param5=1)
       ↓
GCS::handle_preflight_calibration()  ← SAME PATH as Mission Planner!
       ↓
Existing callbacks → SensorTask
```

### MAVLink Coexistence

RocketChip OS detects MAVLink traffic (start bytes 0xFE/0xFD) and routes appropriately:
- MAVLink messages → GCS parser (for Mission Planner integration)
- ASCII characters → CLI parser

When a MAVLink GCS connects (sends heartbeats), binary telemetry is enabled. Without GCS, only human-readable ASCII output is sent.

### Command Mapping

| CLI Key | MAVLink Command | Parameters |
|---------|-----------------|------------|
| `l` | MAV_CMD_PREFLIGHT_CALIBRATION | param5=1 (simple accel) |
| `a` | MAV_CMD_PREFLIGHT_CALIBRATION | param5=2 (6-pos accel) |
| `g` | MAV_CMD_PREFLIGHT_CALIBRATION | param1=1 (gyro) |
| `m` | MAV_CMD_PREFLIGHT_CALIBRATION | param2=1 (compass) |
| `b` | MAV_CMD_PREFLIGHT_CALIBRATION | param3=1 (baro) |

---

## Planned Features

### Phase 3 - GPS & Telemetry
- **g** - GPS status and satellite info
- **t** - Telemetry radio status
- **r** - Radio test mode
- **p** - Parameter view/dump (AP_Param)

### Phase 4 - Mission System
- **m** (redefined) - Mission menu
  - Mission selection
  - Parameter editing
  - Event configuration
- Flight state check before calibration (safety)

### Phase 5 - Flight Operations
- **f** - Flight mode selection
- **a** (redefined) - Arm/disarm
- **d** - Data download menu

### Future
- Priority boost during user interaction
- Ring buffer for pre-connect output capture
- Scripting/macro support
- OTA firmware update
- WiFi/Bluetooth CLI access

---

## Architecture

### FreeRTOS Task Model

| Task | Priority | Core | Purpose |
|------|----------|------|---------|
| CLITask | 1 (lowest) | 0 | User interface, menu handling |
| SensorTask | 5 | 1 | 1kHz IMU, 50Hz baro sampling |
| APM_I2C | 5 | 0 | ArduPilot I2C bus thread |

**Design rationale:** CLI at lowest priority ensures sensor sampling is never interrupted. This matches flight-critical requirements but may cause slight input lag during heavy sensor activity.

### Terminal-Connected Pattern (v0.3)

CLITask only executes when a terminal is connected (`stdio_usb_connected()`). This resolves Entry 15 where terminal connection was affecting program state.

```
CLITask Loop:
  if (!stdio_usb_connected()) {
      // DISCONNECTED: Slow LED blink (1Hz), NO USB I/O
      continue;
  }
  // CONNECTED: Process CLI commands via RC_OS::cmd_*()
```

**Benefits:**
- Terminal connection affects only output, not program behavior
- CLI completely decoupled from SensorTask internals
- No wasted cycles when no terminal connected

### Menu State Machine

```
MenuMode::Main
  ├── 'h' → printSystemStatus()
  ├── 's' → SensorTask_PrintStatus()  (read-only via mutex)
  └── 'c' → MenuMode::Calibration
              ├── 'w' → runCalibrationWizard()
              ├── 'l' → RC_OS::cmd_level_cal()      ← MAVLink path
              ├── 'a' → RC_OS::cmd_accel_cal_6pos() ← MAVLink path
              ├── 'g' → RC_OS::cmd_gyro_cal()       ← MAVLink path
              ├── 'm' → RC_OS::cmd_compass_cal()    ← MAVLink path
              ├── 'b' → RC_OS::cmd_baro_cal()       ← MAVLink path
              └── 'x' → MenuMode::Main
```

### Source Files

| File | Purpose |
|------|---------|
| `src/main.cpp` | CLI task and menu handling |
| `src/cli/RC_OS.h` | CLI→MAVLink command mappings |
| `src/services/SensorTask.cpp` | Calibration implementations |
| `lib/ap_compat/GCS_MAVLink/GCS.h` | MAVLink interface + send_local_command() |
| `lib/ap_compat/GCS_MAVLink/GCS.cpp` | MAVLink parsing, routing, command handling |

---

## Platform-Specific Considerations

RC_OS runs on RP2350 with FreeRTOS SMP - a fundamentally different environment from the original APM CLI (single-core ATmega, bare-metal). Key differences documented in `RP2350_FULL_AP_PORT.md`:

| Issue | Solution | Reference |
|-------|----------|-----------|
| Multi-core memory visibility | std::atomic flags, core affinity | PD12 |
| Priority inversion | vTaskDelay instead of busy-wait | PD13 |
| USB CDC init order | HAL init before stdio_init_all | PD1, PD3 |
| TinyUSB scheduler requirement | All USB I/O after vTaskStartScheduler | Entry 10 |

These issues are **not** present in traditional CLI implementations and required platform-specific solutions.

---

## Known Limitations

1. **Input lag** - CLI at priority 1 only runs when higher-priority tasks yield
2. **No parameter editing** - Can view AP_Param but not modify via CLI yet
3. **No flight state lockout** - Calibration can run even when armed (safety gap)
4. **Pre-connect output lost** - Messages before terminal connection are not buffered

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

### Terminal Compatibility
- [ ] PuTTY (Windows)
- [ ] screen (Linux/macOS)
- [ ] minicom (Linux)
- [ ] Arduino Serial Monitor

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 0.3.0 | 2026-01-30 | **Decoupled architecture**: CLI speaks MAVLink internally, terminal-connected pattern (Entry 15 fix) |
| 0.2.1 | 2026-01-30 | Architecture analysis, priority inversion fix, debug cleanup |
| 0.2.0 | 2026-01-29 | Calibration sub-menu, wizard mode |
| 0.1.0 | 2026-01-29 | Initial CLI with basic commands |
