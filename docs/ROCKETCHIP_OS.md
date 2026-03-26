# RocketChip OS

**Status:** Active
**Last Updated:** 2026-03-15

RocketChip OS (RC_OS) is the command-line interface and user interaction system for RocketChip flight computers. It provides serial-based configuration, calibration, flight operations, and status monitoring.

---

## Overview

RocketChip OS runs over USB CDC serial. It provides:
- Real-time sensor status monitoring
- Sensor calibration routines (gyro, accel, baro, mag)
- ESKF live telemetry display
- Flight Director control and bench testing
- Data management (flight list, download, flush, erase)
- Telemetry radio configuration (TX rate, MAVLink toggle)
- System health and boot diagnostics

The CLI is designed to work standalone (via terminal) or alongside MAVLink-based ground control stations (Mission Planner, QGroundControl).

### Design Philosophy

RC_OS follows patterns proven in the old APM CLI (ArduPilot ~2012-2014) adapted for bare-metal Pico SDK:

| Pattern | Implementation |
|---------|---------------|
| Single-key commands | No complex parsing, immediate response |
| Three-level menu | Main → Calibration, Flight Director |
| Non-blocking input | `getchar_timeout_us(0)` polling |
| Terminal compatibility | Handles CR, LF, and CR+LF |
| Visual feedback | LED blink patterns during operations |
| Context prompt | `[main]` `[cal]` `[flight]` shows current menu |

---

## Menu Structure

### Main Menu (`[main]`)

Displayed on terminal connect and on `h` keypress. Commands grouped by function:

| Group | Keys | Description |
|-------|------|-------------|
| **Status** | `h` `s` `e` `b` | Help, sensor status, ESKF live (1Hz), boot summary |
| **Menus** | `c` `f` | Calibration sub-menu, Flight Director sub-menu |
| **Data** | `g` `d` `l` `x` | List flights, download, flush to flash, erase all |
| **Radio** | `t` `r` `m` `i` | Telemetry status, cycle TX rate, MAVLink toggle, I2C scan |

### Calibration Menu (`[cal]`) — press `c` from main

| Key | Command | Description |
|-----|---------|-------------|
| `g` | Gyro Cal | Gyroscope calibration (keep still) |
| `l` | Level Cal | Quick accelerometer level calibration (keep flat) |
| `b` | Baro Cal | Barometer ground pressure reference |
| `a` | Accel 6-Pos | Full 6-position accelerometer calibration |
| `m` | Compass Cal | Magnetometer hard/soft iron correction |
| `w` | Wizard | Guided calibration of all sensors in sequence |
| `r` | Reset | Reset all calibration data |
| `v` | Save | Save calibration to flash |
| `h` | Help | Reprint calibration menu |
| `x`/ESC | Back | Return to main menu (cancels active cal) |

### Flight Director Menu (`[flight]`) — press `f` from main

| Group | Keys | Description |
|-------|------|-------------|
| **Commands** | `a` `d` `x` `r` | ARM, DISARM, ABORT, RESET |
| **Events** | `l` `b` `p` `m` `n` | Inject LAUNCH, BURNOUT, APOGEE, MAIN_DEPLOY, LANDING |
| **Info** | `s` `h` | Flight status (phase, markers, timing), help |
| **Nav** | `z`/ESC | Return to main menu |

Event injection keys simulate sensor guard signals for bench testing. In flight, these transitions are triggered automatically by guard functions (IVP-70).

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

## Architecture

### Execution Model

CLI runs as part of the polling main loop on Core 0, called at ~20Hz (`kCliPollMs = 50`). Sensor sampling runs on Core 1 at ~1kHz, so CLI polling never interferes with sensor timing.

### Terminal-Connected Pattern

CLI only executes when a terminal is connected (`stdio_usb_connected()`). This prevents USB I/O from affecting program behavior when no terminal is attached.

```
Main loop:
  if (!stdio_usb_connected()) {
      // DISCONNECTED: Slow LED blink (1Hz), NO USB I/O
  } else {
      // CONNECTED: Process CLI commands
  }
```

**Benefits:**
- Terminal connection affects only output, not program behavior
- CLI completely decoupled from sensor internals
- No wasted cycles when no terminal connected

### Menu State Machine

```
RC_OS_MENU_MAIN
  ├── 'h' → print_system_status()
  ├── 's' → print_sensor_status()    (callback from main.cpp)
  ├── 'e' → ESKF live mode (1Hz)     (any key stops)
  ├── 'b' → print_boot_summary()     (callback)
  ├── 'i' → i2c_bus_scan()           (guarded: Core 1 bus ownership)
  ├── 'c' → RC_OS_MENU_CALIBRATION
  │           ├── 'g'/'l'/'b'/'a'/'m' → calibration routines
  │           ├── 'w' → wizard (all in sequence)
  │           ├── 'r' → reset cal, 'v' → save to flash
  │           └── 'x'/ESC → RC_OS_MENU_MAIN
  ├── 'f' → RC_OS_MENU_FLIGHT
  │           ├── 'a'/'d'/'x'/'r' → ARM/DISARM/ABORT/RESET
  │           ├── 'l'/'b'/'p'/'m'/'n' → event injection
  │           ├── 's' → flight status
  │           └── 'z'/ESC → RC_OS_MENU_MAIN
  └── default → handle_unhandled_key()
                ├── 'g' → list flights
                ├── 'd' → download flight
                ├── 'l' → flush log
                ├── 'x' → erase all flights
                ├── 't' → radio status
                ├── 'r' → cycle TX rate
                └── 'm' → toggle MAVLink output
```

### Callback Architecture

RC_OS uses function pointer callbacks set by `main.cpp` to decouple CLI from application internals. All sensor reads, flight director signals, and status display go through callbacks — the CLI never calls hardware drivers directly.

### Source Files

| File | Purpose |
|------|---------|
| `src/cli/rc_os.h` | Public API, menu enum, callback typedefs |
| `src/cli/rc_os.cpp` | Menu handling, key dispatch, calibration routines |
| `src/main.cpp` | Callback implementations, CLI hook setup |

---

## Platform Considerations

| Concern | Approach | Reference |
|---------|----------|-----------|
| USB CDC guard | Only do I/O when `stdio_usb_connected()` | LL Entry 15 |
| Flash/USB conflict | Flash ops before `stdio_init_all()` | LL Entries 4, 12 |
| Input buffer garbage | Drain buffer on terminal connect | LL Entry 15 |
| I2C bus ownership | `rc_os_i2c_scan_allowed` flag | LL Entry 23 |
| MAVLink mode | Suppress CLI text, drain USB input | IVP-62 |

---

## Known Limitations

1. **Input lag** - CLI polled at 20Hz; up to 50ms latency on keypress
2. **No parameter editing** - View-only; parameter modification planned for Stage 11
3. **No flight state lockout** - Calibration can run even when armed (safety gap — IVP-69 will add pre-arm lockout)
4. **Pre-connect output lost** - Messages before terminal connection are not buffered
5. **Single-key only** - No arrow key or typed-command navigation (standard embedded CLI pattern)

---

## Terminal Compatibility

| Terminal | Input | Output | Notes |
|----------|-------|--------|-------|
| Python miniterm | OK | OK | **Recommended** |
| Python serial | OK | OK | Best for scripted tests |
| VSCode Serial Monitor | OK | OK | Use "Terminal" mode |
| PuTTY/plink | OK | Truncated banner | ~63 char cutoff on connect (host-side issue) |

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 0.4.0 | 2026-03-15 | **Flight Director sub-menu** (IVP-68), grouped main menu layout, context prompts `[main]`/`[cal]`/`[flight]`, `'f'` key reassigned from flight list to FD menu, flight list moved to `'g'` |
| 0.3.0 | 2026-01-30 | **Decoupled architecture**: CLI speaks MAVLink internally, terminal-connected pattern (LL Entry 15 fix) |
| 0.2.1 | 2026-01-30 | Architecture analysis, priority inversion fix, debug cleanup |
| 0.2.0 | 2026-01-29 | Calibration sub-menu, wizard mode |
| 0.1.0 | 2026-01-29 | Initial CLI with basic commands |
