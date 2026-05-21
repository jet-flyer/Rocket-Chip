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

Displayed on terminal connect and on `h` keypress. Commands grouped by function. **Source of truth: `src/cli/rc_os.cpp:94-98` (`print_help_menu()`) + `src/cli/rc_os.cpp:143-191` (`handle_main_menu()`).**

| Group | Keys | Description |
|-------|------|-------------|
| **Status** | `h` `p` | Help (reprint menu), Preflight Go/No-Go |
| **Menus** | `c` `f` `q` | Calibration sub-menu, Flight Director sub-menu, **Debug sub-menu (sensor status, HW status, ESKF live, etc. — bench-tier only)** |
| **Data** | `g` `d` `l` `x` | List flights, download, flush to flash, erase all |
| **Radio** | `t` `r` `m` `b` | Telemetry status, cycle TX rate, MAVLink toggle, **Beacon (manual find-me, Stage L)** |

**Drift fix 2026-05-13:** previously documented `s`/`e`/`b` as main-menu Status keys and `i` as a main-menu Radio key. Stage L (2026-04-18) reassigned `b` from "boot summary" to "Beacon (manual find-me)" in main menu, and bench-tier sensor/ESKF/HW status / I2C-scan keys moved to the `q`-Debug submenu via the bench-tier classification. The previous documentation rotted silently. See R-27 (this cycle) for the fix.

### Debug Sub-Menu (`[debug]`) — press `q` from main

| Key | Command | Test-mode gated? | Description |
|-----|---------|------------------|-------------|
| `s` | Sensors | No (read) | Print sensor status + IMU/baro read counters (`Reads: I=NNN M=NNN B=NNN G=NNN`) |
| `i` | I2C scan | No (read) | Rescan I2C bus (disabled if Core 1 owns bus) |
| `b` | Boot/HW | No (read) | Reprint boot banner + Hardware Status (`Hardware: N/N OK` + per-device PASS/FAIL) |
| `e` | ESKF live | No (read) | 1Hz ESKF state dump until any key |
| `y` | Pyro log | No (read) | Dump pyro edge logger CLI |
| `r` | Replay inject | retired | R-25-exec steps 5+6 deleted both vehicle and station on-MCU replay; see `scripts/replay_harness_host.py` (host-side, placeholder) |
| `d` | Diag stats | No (read) | Diagnostic counters dump (rate counters, MSP, etc.) |
| `l` | LED test | Yes | LED test pattern — requires `rc::test_mode_active()` |
| `0..5` | Radio cfg | Yes | Local radio configuration index — requires `rc::test_mode_active()` |
| `h` `H` `?` | Help | No | Reprint debug submenu |
| `z` `Z` ESC | Back | No | Return to main menu |

**R-25-exec (2026-05-13):** Debug submenu is always available in the flight binary. Read commands (s/i/b/e/y/d) always work; state-mutating commands (l, 0..5, retired r) check `rc::test_mode_active()` at entry and refuse if test mode is not armed. Test mode arms via debug probe (write `rc::kTestModeMagic` to `rc::g_test_mode_arm_magic` + reset; see `safety/test_mode.h`). Three-condition AND gate (magic AND state==IDLE AND boot-time-window) prevents accidental in-flight activation. See `docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md`.

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

## Station (Ground Station) UI Modes

Station firmware (`ROCKETCHIP_JOB_STATION=1`) has a different UI from the vehicle: a live ANSI dashboard is the default boot state, and only two keys exit it. This is enforced by `poll_dashboard_keys()` in `src/active_objects/ao_rcos.cpp`.

### Output modes (`StationOutputMode`)

| Mode | Description |
|------|-------------|
| `kAnsi` | **Default on clean boot.** Live dashboard with RSSI bar, MET, state, GPS, radio config, ARM indicator. Redraws on each RX packet or 1 Hz idle. |
| `kCsv`  | Plaintext CSV stream — one line per RX nav packet. For logging / scripted parsing. |
| `kMavlink` | MAVLink-over-USB passthrough for QGC / Mission Planner. |
| `kMenu` | Vehicle-style single-key CLI (main menu). Required for any debug-menu operations (`q…z`), calibration, flight-director injection. |

### Keys accepted while in `kAnsi` / `kMavlink` (dashboard active)

Stage T Batch B IVP-T14d wrap-up (2026-04-22) moved `m` / `M` out of the
dashboard and added `a` / `D` for station command-from-dashboard. Mode
cycling is kMenu-only now. ABORT is deliberately not bound from the
dashboard — NAR High Power Safety Code §6 treats the rocket as a
ballistic object post-ignition, so there is no in-flight ABORT channel
exposed in the fast-access UX. (An advanced / debug-menu ABORT path
remains for pre-launch safety.) STOP-GAP pending full CCSDS command
layer.

Only these keys are accepted; all others are silently dropped:

| Key | Action | Notes |
|-----|--------|-------|
| `x` / `X` | Exit to `kMenu` (enters single-key CLI mode). | — |
| `a` (station only) | Start ARM confirm flow. | Operator types `A`, `R`, `M` then `Enter` to confirm. ESC cancels. |
| `D` (station only, caps) | DISARM. | Single-key, ACK-tracked, no confirm. Lowercase `d` reserved for distance-to-vehicle. |

Mode cycle (`m`) is kMenu-only as of 2026-04-22.

### Keys accepted in `kMenu`

Full single-key CLI (see `enter_cli_menu()` in `ao_rcos.cpp`). Subset reachable only from kMenu:

Behaviors marked **(vehicle)** vs **(station)** below differ by build flavor (`kRadioModeRx`): vehicle and station share the same CLI source but dispatch some keys differently.

| Group | Keys | Description |
|-------|------|-------------|
| **Status** | `h` `s` `b` `p` | Help, sensor status, boot summary, preflight |
| **Radio** | `t` `r` `m` | Telemetry status; `r` = cycle TX rate **(vehicle)** / cycle CCSDS SET_RADIO_CONFIG to next entry in `kRadioConfigTable` **(station)**; cycle output mode (`m`) |
| **Station** | `g` `d` `p` | GPS status, distance-to-vehicle, GPS push |
| **Commands** | `a` `X` | ARM (with confirm), DISARM |
| **Flight logs** | `l` `x` | Flush to flash, erase |
| **Debug submenu** | `q` → `<digit>z` / other | Enter debug submenu, local radio config change, exit back to `[main]`. |

### Reaching the debug submenu from dashboard

Scripted example:

1. Assume station is in `kAnsi` (default boot state).
2. Send `x` → station switches to `kMenu`, prints banner + `[main] ` prompt.
3. Send `q<idx>z\r` → debug submenu applies local radio config index, exits back to `[main]`.
4. Send `m` → cycles back to `kAnsi` (dashboard resumes).

### Gotcha

If you send `q<idx>z` while station is still in `kAnsi`, **nothing happens** — the `q` is silently dropped. Sweep scripts and test harnesses must explicitly transition to `kMenu` first (`x`) before issuing debug-menu keys, and restore with `m` afterward if the test expects dashboard output.

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
2. **No parameter editing** - View-only; runtime parameter modification is a future feature
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
