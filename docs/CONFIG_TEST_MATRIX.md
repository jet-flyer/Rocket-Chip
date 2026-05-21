# Configuration / Test Compatibility Matrix

**Purpose:** A single reference table mapping each firmware build
configuration to the host-side test scripts that work against it, plus the
underlying firmware features each configuration exposes.

**Council review & roadmap:** `docs/council/HOST_SCRIPT_HARDENING_REVIEW_AND_ROADMAP.md`  
**Plan pointer:** `docs/plans/HOST_SCRIPT_HARDENING_PLAN.md`

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

## Firmware build configurations

Defined in `CMakePresets.json`. **Post-R-25-exec (2026-05-13):** one flight binary per role. Test affordances are runtime-gated by `rc::test_mode_active()` (probe-only three-condition AND gate: SRAM magic + state==IDLE + boot-time-window) — see [`docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md`](decisions/BENCH_TIER_DEPRECATION_2026-05-13.md).

| Preset           | Role    | Board                          | Binary dir              | Output         |
|------------------|---------|--------------------------------|-------------------------|----------------|
| `vehicle-flight` | vehicle | Adafruit Feather RP2350 HSTX   | `build_flight/`         | `rocketchip`   |
| `station-flight` | station | Adafruit Fruit Jam             | `build_station_flight/` | `rocketchip`   |

The legacy `vehicle-bench` and `station-bench` presets still exist in `CMakePresets.json` as backward-compat scaffolding (their `NOT_CERTIFIED_FOR_FLIGHT=ON` cache variable is parsed but no longer affects the build — every R-25-exec gate site in `src/` was either migrated test-mode-gated into the flight binary or deleted). All four presets produce feature-identical binaries per role; `kBuildConfig` is hardcoded to `"flight"` post-R-26 (2026-05-15).

USB CDC identification (used by host scripts):
- VID `0x2E8A`, PID `0x0009` for both roles.
- Role distinguished by **boot banner content**, not VID/PID.
- Vehicle banner contains `"Profile: Rocket"` and `"Adafruit Feather RP2350 HSTX"`.
- Station banner contains `"Ground Station"`, `"Fruit Jam"`, or `"Station RX"`.
- Build-tag suffix on the banner is always `flight-<sha>` post-R-26.

## Firmware feature matrix

Which CLI features / keystrokes exist on each role. Test-mode-gated affordances are marked **(test-mode)** — these require `arm_test_mode_via_probe()` at session start; the affordance is silently refused otherwise.

| Feature                                          | vehicle              | station              |
|--------------------------------------------------|:--------------------:|:--------------------:|
| Boots into kMenu CLI                             |          ✓           |          —           |
| Boots into kAnsi dashboard                       |          —           |          ✓           |
| `'p'` print preflight (`[Health]` ...)           |          ✓           |          ✓ ¹         |
| `'q'` enter debug submenu                        |          ✓           |          ✓           |
| `'q'→'b'` `cli_print_hw_status` (Hardware Status)|          ✓           |          ✓           |
| `'q'→'d'` `diag_stats_dump()` `[Health]` line    |          ✓           |          ✓           |
| `'q'→'s'` sensor status                          |          ✓           |          ✓           |
| `'q'→'e'` ESKF live (1 Hz output)                |          ✓           |          —           |
| `'q'→<digit>z` local radio config set            |   ✓ (test-mode)      |   ✓ (test-mode)      |
| `'q'→'l'` LED test                               |   ✓ (test-mode)      |   ✓ (test-mode)      |
| `'f'` flight menu (a/l/x=ABORT/etc.)             |          ✓           |          —           |
| `'x'` from main menu = Erase Flights confirm     |          ✓           |   ✓ (after 'x' from dashboard) |
| `'X'` (caps) = tracked-DISARM via radio          |          —           |          ✓           |
| `'a'` ARM (multi-char confirm flow)              |     flight menu      |     ✓ via kAnsi      |
| `'m'` mode cycle (ANSI/CSV/MAVLink)              |          —           |          ✓           |
| `fault_force_*` test-injection symbols           |   ✓ (test-mode)      |   ✓ (test-mode)      |
| Sensors (IMU, baro, GPS) installed               |          ✓           |          ✗           |
| Telemetry TX                                     |          ✓           |          ✗           |
| Telemetry RX                                     |          ✗           |          ✓           |
| Flight Director state machine                    |          ✓           |          ✗           |
| Stage T logging (`-DROCKETCHIP_STAGE_T_LOGGING=ON`)|       opt          |        opt           |

¹ Sensor lines on station preflight often **ABSENT** (no onboard IMU/baro/GPS RX path);
  HW radio / RF Link / MCU / **VERDICT** remain valid. Tier 5 parity: **`rc_os.cpp`**
  main-menu **`'p'`** calls **`cli_print_preflight()`** on RX as well as vehicle.

### Implications for tests

- **State-mutating debug-menu affordances** (LED test, local radio config set, `fault_force_*` entries) check `rc::test_mode_active()` at entry. Scripts that need them must arm test mode via probe at session start (`arm_test_mode_via_probe()` in `scripts/_rc_test_common.py`). Diagnostic *reads* (`q→s`, `q→b`, `q→d`, `q→e`) are always available with no gating.
- `'x'` is the only character bound to a **destructive confirmation prompt**
  on main menu. Scripts that send `'x'` MUST classify the firmware first.
- Station boots into the dashboard, not the CLI menu. Tests must send
  `'x'` (or another dashboard-recognised key) to enter `kMenu` before
  sending CLI keys.
- The **`[Health]` summary block (`cli_print_preflight`)** prints from main-menu **`'p'`**
  on **both roles** (**Tier 5**). Scripts parsing **comma-separated
  `[Health]` log fields** (`primary=...`) from `diag_stats_dump()` remain a **debug-menu** contract —
  not the same textual format as **`cli_print_preflight()`**.

## Test compatibility matrix (host scripts)

Each host script's required target configuration(s), based on what
keystrokes it sends and what output it parses. **Current state** column
reflects what the script will actually do today, not what it should do.

**Tier 2 (machine-readable targets):** Each script listed below that runs against
serial wraps `main()` with `@rc_test(target=...)` (`scripts/_rc_test_common.py`) where
applicable so tools can discover required role/build and enforce the shared 0/1/2
exit contract. **Tiers 3–4** (shared `find_target_port` / `open_classified_port` /
`find_vehicle_and_station_ports`) are **landed** for the rows marked accordingly;
see `docs/council/HOST_SCRIPT_HARDENING_REVIEW_AND_ROADMAP.md`. The **Has guard**
column means **banner-based classification** before destructive keys are sent.

| Script                          | Required role            | Current state                                                                | Has guard | Has watchdog |
|---------------------------------|--------------------------|------------------------------------------------------------------------------|:---------:|:------------:|
| `bench_sim.py`                  | vehicle                  | ✓ correct, uses flight menu only                                             |    ✓      |     ✓        |
| `station_bench_sim.py`          | station                  | ✓ correct (post 2026-04-27 hardening); uses always-available debug-menu reads |    ✓      |     ✓        |
| `accel_cal_6pos.py`             | vehicle                  | `--port` optional; VID:PID classify                                          |    ✓      |     ✗        |
| `cla_collect.py`                | vehicle                  | `--port` optional; VID:PID classify                                          |    ✓      |     ✗        |
| `cli_test.py`                   | vehicle                  | `--port` optional; VID:PID classify                                          |    ✓      |     ✗        |
| `decode_flight_log.py`          | vehicle                  | find_target_port + open_classified_port ✓                                    |    ✓      |     ✗        |
| `enhanced_fault_injection.py`   | vehicle                  | TARGET_VEHICLE_ANY + arm_test_mode_via_probe; ELF=build_flight               |    ✓      |     ✗        |
| `i2c_soak_test.py`              | vehicle                  | `--port` optional; VID:PID classify                                          |    ✓      |     ✗        |
| `mavlink_validate.py`           | station (passive)        | `--port` optional; classify; no auto kMenu (MAVLink tap)                     |    ✓      |     ✗        |
| `replay_harness_host.py`        | n/a (host-side)          | Placeholder skeleton; host-side ESKF replay implementation pending           |    n/a    |     n/a      |
| `soak_test.py`                  | vehicle                  | find_target_port + open_classified_port ✓                                    |    ✓      |     ✓        |
| `verify_boot_parity.sh`         | both                     | Build + flash + banner-classify per role (R-24, R-25-exec step 12)           |    ✓      |     n/a      |
| `warm_reboot_audit.py`          | both                     | TARGET_EITHER_ANY + arm_test_mode_via_probe; 4 gates (R-22, R-25-exec step 11) | ✓      |     n/a      |

### Coverage gaps revealed by the matrix

- **`replay_harness_host.py` is a placeholder.** Host-side ESKF replay against `tests/replay_profiles/*.csv` ground-truth has no implementation yet. Tracked on the WB ("Host-side replay harness implementation"). Blocks the IVP-131 verification model promised by R-25-exec amendment #4.
- **Legacy COM defaults (COM6/COM7/COM9) in docstrings** may still appear as
  examples; runtime selection uses **banner classification** with optional
  `--port` overrides. Prefer the council review doc for migration status.

## How to add a new test script

When creating a new script under `scripts/`:

1. **Declare the target** with the `@rc_test` decorator from
   `scripts/_rc_test_common.py`. Council-approved 2026-04-27 (decorator
   form chosen over comment header for refactor-safety + entry-point
   visibility).
2. **Use the shared helpers** from `scripts/_rc_test_common.py` for: port
   detection (`find_target_port`; **dual-board harnesses:**
   `find_vehicle_and_station_ports`), banner classification
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

Minimum-template for a new vehicle script:

```python
#!/usr/bin/env python3
"""My new test."""
from __future__ import annotations
import argparse
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _rc_test_common import (
    rc_test, find_target_port, open_classified_port,
    TARGET_VEHICLE_FLIGHT,
)

@rc_test(target=TARGET_VEHICLE_FLIGHT, watchdog_s=120)
def main():
    parser = argparse.ArgumentParser(description='...')
    parser.add_argument('--port', default=None)
    args = parser.parse_args()

    port, banner = find_target_port(TARGET_VEHICLE_FLIGHT,
                                    override=args.port, verbose=True)
    if port is None:
        print(f'INFO: skipping --- {banner}')
        return 2  # skip: no target present

    with open_classified_port(port, target=TARGET_VEHICLE_FLIGHT) as ser:
        # ...test body...
        return 0  # pass

if __name__ == '__main__':
    main()
```

For station scripts, replace `TARGET_VEHICLE_FLIGHT` with `TARGET_STATION_FLIGHT`
(or `TARGET_STATION_ANY` if the test doesn't need to distinguish dev-vs-flight banner
suffixes — irrelevant post-R-26 since the suffix is always `flight-<sha>`). The
`open_classified_port` helper auto-transitions station from kAnsi
dashboard to kMenu so `'q'`, `'h'`, etc. are honored.

## Roadmap history (completed)

- **Tier 5 — station `'p'` preflight parity:** **`src/cli/rc_os.cpp`** main-menu `'p'` now calls **`cli_print_preflight()`** on RX (station) as well as vehicle — **`[Health]` / VERDICT** visible without debug menu (`2026-04-30`).
- **Tier 6b — matrix-driven hooks:** Patterns live in **`scripts/ci/pre_commit_matrix.py`** (evaluated by **`scripts/hooks/pre-commit`**). One-time repo setup: `git config core.hooksPath scripts/hooks` — see **`scripts/hooks/README.md`**. **Categories not enumerations (2026-05-16, council unanimous, see LL Entry 40):** `FLIGHT_CRITICAL` matches any path that can change `rocketchip.elf` (`src/`, `include/`, `CMakeLists.txt`, `cmake/`, vendored libs we link, plus the gate scripts themselves for self-rot prevention). Pure-doc / pure-test / pure-tooling exempt by virtue of not matching the regex — never by explicit carve-out. The prior narrow enumeration (~7 hand-listed FD/AO paths) was the LL Entry 36 / 39 failure mode in waiting.
- **Tier 7 — watchdog ceilings:** **`@rc_test(..., watchdog_s=…)`** on **`soak_test`** (86400s), **`warm_reboot_audit`** (300s) — escapes hung USB open on Windows beyond intended wall times. (Post-R-25-exec 2026-05-13: ack_stress_test + replay_harness deleted in step 9.)

Older Tier 1–4 / 6a notes remain in **`docs/council/HOST_SCRIPT_HARDENING_REVIEW_AND_ROADMAP.md`**.

## Maintenance

This file is the source of truth for cross-config compatibility. Update
it when:
- A new build configuration is added to `CMakePresets.json`.
- A CLI key binding changes in `src/cli/rc_os.cpp`,
  `src/cli/rc_os_commands.cpp`, or `src/cli/rc_os_debug.cpp`.
- A new `scripts/*.py` file is added (or existing one promoted to CI).
- A test starts or stops working on a configuration.
