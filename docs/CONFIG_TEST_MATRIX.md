# Configuration / Test Compatibility Matrix

**Purpose:** A single reference table mapping each firmware build
configuration to the host-side test scripts that work against it, plus the
underlying firmware features each configuration exposes.

**When to use:**
- Before running any host-side script — confirm the script's target matches
  the firmware on the bench.
- When adding a new test script — declare its target so the matrix stays
  in sync with reality.
- When changing a firmware menu binding (`'q'`, `'x'`, `'p'`, etc.) —
  cross-check the matrix to find every script that depends on that key.
- When promoting a script to CI / pre-commit gating — confirm it has a
  target guard, watchdog, and exit-code semantics that match the gate.

**Origin:** Captured during the 2026-04-27 incident triage (station_bench_sim
sending destructive `'x'` keystrokes to vehicle firmware because of stale
fallback-to-first-port behavior). The lesson generalised: tests written
against a single board/configuration silently rot as the project grows
multi-tier.

## The four firmware build configurations

Defined in `CMakePresets.json`. Two axes: **role** (vehicle vs station)
and **build flag** (`BUILD_FOR_FLIGHT=OFF` aka bench, vs `=ON` aka flight).

| Preset           | Role    | Board                          | `BUILD_FOR_FLIGHT` | Binary dir              |
|------------------|---------|--------------------------------|--------------------|-------------------------|
| `vehicle-bench`  | vehicle | Adafruit Feather RP2350 HSTX   | `OFF`              | `build/`                |
| `vehicle-flight` | vehicle | Adafruit Feather RP2350 HSTX   | `ON`               | `build_flight/`         |
| `station-bench`  | station | Adafruit Fruit Jam             | `OFF`              | `build_station/`        |
| `station-flight` | station | Adafruit Fruit Jam             | `ON`               | `build_station_flight/` |

USB CDC identification (used by host scripts):
- VID `0x2E8A`, PID `0x0009` for both roles.
- Role distinguished by **boot banner content**, not VID/PID.
- Vehicle banner contains `"Profile: Rocket"` and `"Adafruit Feather RP2350 HSTX"`.
- Station banner contains `"Ground Station"`, `"Fruit Jam"`, or `"Station RX"`.
- Bench vs flight is encoded in the build tag suffix
  (`bench-<sha>` vs `flight-<sha>`) on the boot banner line.

## Firmware feature matrix

Which CLI features / keystrokes exist in each configuration. Used by host
scripts to decide what they can rely on.

| Feature                                          | vehicle-bench | vehicle-flight | station-bench | station-flight |
|--------------------------------------------------|:-------------:|:--------------:|:-------------:|:--------------:|
| Boots into kMenu CLI                             |       ✓       |       ✓        |       —       |       —        |
| Boots into kAnsi dashboard                       |       —       |       —        |       ✓       |       ✓        |
| `'p'` print preflight (`[Health]` line)          |       ✓       |       ✓        |    no-op¹     |     no-op¹     |
| `'q'` enter debug menu                           |       ✓       |   ✗ (no-op)    |       ✓       | ✗ (sends QUERY)²|
| `'q'→'b'` cli_print_hw_status                    |       ✓       |       ✗        |       ✓       |       ✗        |
| `'q'→'d'` `diag_stats_dump()` `[Health]` line    |       ✓       |       ✗        |       ✓       |       ✗        |
| `'q'→'r'` replay inject                          |       ✓       |       ✗        | ✓ (RX-path)   |       ✗        |
| `'q'→'e'` ESKF live (1 Hz output)                |       ✓       |       ✗        |       —       |       —        |
| `'f'` flight menu (a/l/x=ABORT/etc.)             |       ✓       |       ✓        |       —       |       —        |
| `'x'` from main menu = Erase Flights confirm     |       ✓       |       ✓        |  ✓ (after 'x' from dashboard) |  ✓ (after 'x') |
| `'X'` (caps) = tracked-DISARM via radio          |       —       |       —        |       ✓       |       ✓        |
| `'a'` ARM (multi-char confirm flow)              |  flight menu  |  flight menu   |   ✓ via kAnsi |  ✓ via kAnsi   |
| `'m'` mode cycle (ANSI/CSV/MAVLink)              |       —       |       —        |       ✓       |       ✓        |
| Sensors (IMU, baro, GPS) installed               |       ✓       |       ✓        |       ✗       |       ✗        |
| Telemetry TX                                     |       ✓       |       ✓        |       ✗       |       ✗        |
| Telemetry RX                                     |       ✗       |       ✗        |       ✓       |       ✓        |
| Flight Director state machine                    |       ✓       |       ✓        |       ✗       |       ✗        |
| Stage T logging (`-DROCKETCHIP_STAGE_T_LOGGING=ON`)|     opt     |      opt       |      opt      |      opt       |

¹ Station 'p' falls through `if(!kRadioModeRx)` so it does nothing visible.
² On flight builds `dev_debug_menu_enter()` returns `false`, so 'q' falls
  through to `cli_handle_unhandled_key('q')`, which on station sends
  `MAV_CMD_USER_2 / QUERY_RADIO_CONFIG`. On vehicle-flight 'q' is silently
  unhandled (no debug menu, no station QUERY path).

### Implications for tests

- Any test that uses the **debug menu** (most diag/soak tests) requires
  `BUILD_FOR_FLIGHT=OFF`. Flight builds will silently mis-execute or hang.
- `'x'` is the only character bound to a **destructive confirmation prompt**
  on main menu. Scripts that send `'x'` MUST classify the firmware first.
- Station boots into the dashboard, not the CLI menu. Tests must send
  `'x'` (or another dashboard-recognised key) to enter `kMenu` before
  sending CLI keys.
- The `[Health]` line (5-field `primary=… secondary=… critical=… mcu=… go_nogo=…`)
  is **bench-only** on both roles. There is no production-build equivalent
  on the main menu. Closing this gap is open as a separate proposal.

## Test compatibility matrix (host scripts)

Each host script's required target configuration(s), based on what
keystrokes it sends and what output it parses. **Current state** column
reflects what the script will actually do today, not what it should do.

**Tier 2 (machine-readable targets):** As of Tier 2, each script listed below wraps
its `main()` with `@rc_test(target=...)` (`scripts/_rc_test_common.py`) so tools can
discover required role/build and enforce the shared 0/1/2 exit contract. Table
columns that say "Has guard: ✗ / no guard" still refer to **banner-based port
classification** --- migration to shared `find_target_port` / runtime guards are
planned in Tier 3+.

| Script                          | Required role            | Required build      | Current state                                    | Has guard | Has watchdog |
|---------------------------------|--------------------------|---------------------|--------------------------------------------------|:---------:|:------------:|
| `bench_sim.py`                  | vehicle                  | bench or flight     | ✓ correct, uses flight menu only                 |    ✓      |     ✓        |
| `station_bench_sim.py`          | station                  | bench (dev menu)    | ✓ correct (post 2026-04-27 hardening)            |    ✓      |     ✓        |
| `accel_cal_6pos.py`             | vehicle                  | bench or flight     | `--port` optional; VID:PID classify                    |    ✓      |     ✗        |
| `ack_stress_test.py`            | station                  | bench or flight     | hardcoded COM7, no guard                         |    ✗      |     ✗        |
| `cla_collect.py`                | vehicle                  | bench (uses 'e')    | `--port` optional; VID:PID classify (bench target)       | ✓              | ✗        |
| `cli_test.py`                   | vehicle                  | bench or flight     | `--port` optional; VID:PID classify                    |    ✓      |     ✗        |
| `codegen_soak_test.py`          | vehicle                  | bench or flight     | `--port` optional; VID:PID classify                    |    ✓      |     ✗        |
| `decode_flight_log.py`          | vehicle                  | bench or flight     | guard ✓ (rejects station)                        |    ✓      |     ✗        |
| `eskf_gps_soak.py`              | vehicle                  | bench (uses 'e')    | `--port` optional; VID:PID classify (bench target)       | ✓          | ✗ |
| `i2c_soak_test.py`              | vehicle                  | bench or flight     | `--port` optional; VID:PID classify                       |    ✓      |     ✗        |
| `mavlink_validate.py`           | station (passive)        | bench or flight     | `--port` optional; classify; no auto kMenu (MAVLink tap)  |    ✓      |     ✗        |
| `replay_gate_test.py`           | vehicle                  | bench (dev menu)    | `--port` optional; bench classify; replay UART stream           | ✓ | ✗ |
| `replay_harness.py`             | vehicle                  | bench (dev menu)    | --port=COM7, no guard, breaks on flight          |    ✗      |     ✗        |
| `soak_test.py`                  | vehicle                  | bench (dev menu)    | --port=COM7, no guard, breaks on flight          |    ✗      |     ✗        |
| `stage_t2_cheat.py`             | both (dual port)         | bench (Stage T log) | --vehicle-port + --station-port, no classify     |    ✗      |     ✗        |
| `stage_t4_ambient.py`           | station                  | bench or flight     | --port=COM9                                      |    ✓      |     ✗        |
| `stage_t6_sweep.py`             | both (dual port)         | bench (dev menu)    | dual port, weak classify                         |    ✓      |     ✗        |
| `stage_t_run.py`                | both (dual port)         | bench (Stage T log) | dual port, weak classify                         |    ✓      |     ✗        |
| `stage_t_wilson_ci.py`          | station                  | bench (dev menu)    | --port=COM9, no guard, breaks on flight          |    ✗      |     ✗        |
| `station_replay_harness.py`     | station                  | bench (dev menu)    | `--port` optional; STATION_BENCH classify; manual replay prep | ✓ | ✗ |

### Coverage gaps revealed by the matrix

- **No test covers `vehicle-flight` exclusively.** Flight-certified vehicle
  binaries get `bench_sim.py` coverage (good), but not the soak/ESKF/replay
  paths.
- **No test covers `station-flight` exclusively.** All station automated
  tests today depend on the debug menu (`'q'→...`), which is excluded from
  `BUILD_FOR_FLIGHT=ON` builds. There is no automated regression detector
  for the production station binary.
- **Three scripts use `'e'` ESKF live mode** (cla_collect, eskf_gps_soak,
  occasionally others). All require dev menu and therefore bench builds.
- **The COM-port hardcoding is inconsistent across 16+ scripts.** Defaults
  vary between COM6/COM7/COM9, set when the bench had different boards
  attached. Auto-classification is the durable answer.

## How to add a new test script

When creating a new script under `scripts/`:

1. **Declare the target** with the `@rc_test` decorator from
   `scripts/_rc_test_common.py`. Council-approved 2026-04-27 (decorator
   form chosen over comment header for refactor-safety + entry-point
   visibility).
2. **Use the shared helpers** from `scripts/_rc_test_common.py` for: port
   detection (`find_target_port`), banner classification
   (`classify_banner`, `peek_banner`), opening + post-open re-classify
   + auto-enter-kMenu (`open_classified_port`), wall-clock hard kill
   (`start_watchdog` is wired into `@rc_test`'s `watchdog_s` param).
3. **Choose exit codes** consistent with the rest of the suite (enforced
   by `@rc_test`):
   - `0` = pass
   - `1` = real failure (block in CI)
   - `2` = environment skip (no target present, wrong build, watchdog
     fired, KeyboardInterrupt). Pre-commit treats `2` as SKIP, not block.
4. **Update this file** when the script lands.

Minimum-template for a new vehicle-bench script:

```python
#!/usr/bin/env python3
"""My new test."""
from __future__ import annotations
import argparse
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _rc_test_common import (
    rc_test, find_target_port, open_classified_port,
    TARGET_VEHICLE_BENCH,
)

@rc_test(target=TARGET_VEHICLE_BENCH, watchdog_s=120)
def main():
    parser = argparse.ArgumentParser(description='...')
    parser.add_argument('--port', default=None)
    args = parser.parse_args()

    port, banner = find_target_port(TARGET_VEHICLE_BENCH,
                                    override=args.port, verbose=True)
    if port is None:
        print(f'INFO: skipping --- {banner}')
        return 2  # skip: no target present

    with open_classified_port(port, target=TARGET_VEHICLE_BENCH) as ser:
        # ...test body...
        return 0  # pass

if __name__ == '__main__':
    main()
```

For station scripts, replace `TARGET_VEHICLE_BENCH` with `TARGET_STATION_BENCH`
(or `TARGET_STATION_ANY` if the test works on flight builds too). The
`open_classified_port` helper auto-transitions station from kAnsi
dashboard to kMenu so `'q'`, `'h'`, etc. are honored.

## Open proposals (not yet executed)

- **Tier 1–4 — host-script hardening**: shared helper module + target
  declarations + watchdog + classification on every script. Plan drafted
  and council-reviewed (see `docs/plans/HOST_SCRIPT_HARDENING_PLAN.md`
  when written).
- **Tier 5 — firmware-side production health command**: add a station
  main-menu binding that prints the `[Health]` summary in flight builds,
  closing the `station-flight` test coverage gap. Bigger design call;
  separate proposal.
- **Tier 6 — CI integration**: have CI / pre-commit consume this matrix
  to choose which tests to run for which staged firmware change. Today
  the pre-commit hook hard-codes the trigger regexes.

## Maintenance

This file is the source of truth for cross-config compatibility. Update
it when:
- A new build configuration is added to `CMakePresets.json`.
- A CLI key binding changes in `src/cli/rc_os.cpp`,
  `src/cli/rc_os_commands.cpp`, or `src/dev/dev_cli.cpp`.
- A new `scripts/*.py` file is added (or existing one promoted to CI).
- A test starts or stops working on a configuration.
