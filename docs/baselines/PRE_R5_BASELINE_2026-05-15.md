# Pre-R-5 Baseline — 2026-05-15

**Purpose:** Unit A deliverable of the R-5 stdio-removal plan (`C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn.md`). Captures "what working looks like" before any callsite migration starts, so each tier-end Level-2 regression diffs against this state.

**Captured against:** HEAD `c6195d1` (`[Claude] Stash Cycles 3 + 4 before R-5 plan-mode breakout`, 2026-05-15). Parent commit `8c0732e` is Cycle 1 close (R-26 dead-gate fix + 5 audit residual closures).

**Use:** every tier-end Level-2 regression run (Tiers 1-6 of the R-5 plan) re-checks these signals. Drift = a finding to investigate, not a free PASS.

---

## Build state

- **Vehicle tier:** `build_flight/` rebuilds clean. Last verified Cycle 1 commit `8c0732e`, both target tiers (16 TUs each) post-R-26 fix.
- **Station tier:** `build_station_flight/` rebuilds clean. Same commit.
- **Host tier:** `build_host/` rebuilds clean.

## Host ctest

- **Count: 800/800 PASS** at HEAD `c6195d1` (verified 2026-05-15).
- Test labels include `python` (2 tests for `scripts_rc_test_common` + `scripts_python_compileall`).
- Full test count per `ctest --test-dir build_host -N | tail`: `Total Tests: 800`.

## SPIN regression

- **Properties: 31** across 6 SPIN models (per `STANDARDS_AUDIT_2026-05-13.md` and `TOOLCHAIN_VERSION_AUDIT_2026-05-13.md`).
- **Errors: 0**. Master gate `SPIN_OK_31` per the 2026-05-13 audit cycle.
- Models: vehicle FD, station FD, radio scheduler, AO commandment IV, plus 2 others (re-verify at first SPIN run during migration).

## Vehicle bench banner

Captured 2026-05-15 via `python scripts/serial_reader.py COM7 6` (vehicle on COM7, OpenOCD attached via probe):

```
==============================================
  RocketChip v0.16.0  RCOS v0.5.0  flight-78d49ae
  Board: Adafruit Feather RP2350 HSTX
  Profile: Rocket  Uptime: 5s
==============================================
Hardware: 14/14 OK
Flash: 4 flights, 27% used

h-Help  p-Preflight  c-Calibration  f-Flight
g-Flights  d-Download  l-Flush  x-Erase
t-Radio  r-Rate  m-MAVLink  b-Beacon  q-Debug
[main]
```

Key invariants for tier-end regression diff:

- **Version line format:** `RocketChip v<X.Y.Z>  RCOS v<X.Y.Z>  flight-<git-short-hash>` (exact whitespace).
- **Build-config string:** `flight-` prefix (R-26 retired the `dev-` branch; should always be `flight-` now).
- **Board:** `Adafruit Feather RP2350 HSTX` (PICO_BOARD == `adafruit_feather_rp2350_hstx`).
- **Profile:** `Rocket`.
- **Hardware:** `14/14 OK` on vehicle. (Station equivalent: `11/11 OK` per Cycle 1 verification.)
- **Main menu format:** 13 entries, four lines, exact spacing per the capture above.

The 13-character format-spec inventory in `STDIO_FORMAT_SPEC_INVENTORY_2026-05-15.md` includes the specs that produce these banner strings (`%s`, `%lu`, `%d`, etc.). Migration of banner-emitting code (`main.cpp` Tier 6, parts of `rc_os.cpp` Tier 5) must produce byte-identical output to this capture.

## bench_sim

- **bench_sim 2/2 PASS** verified at last R-5-relevant HW gate (Cycle 1 close commit `8c0732e` + the Cycles 3+4 stash commit `c6195d1` is pure-doc so doesn't affect bench_sim).
- Bench_sim regex matches `[FD] PYRO FIRED: ...`, `[FD] ABORT`, `[FD]` state transitions emitted by `src/flight_director/flight_director.cpp` — a Unit-A-surfaced transitive-leak file. **Tier 4 byte-on-wire diff** must include these log lines verbatim.

## station bench (Fruit Jam)

- **station_bench_sim 3/3 PASS** verified at the same gate.
- Hardware: `11/11 OK` on warm-reboot. Cold-boot has the known `10/11 ok [fail] gps` quirk that is Cycle 3's investigation target (see `docs/plans/CYCLE_RESIDUALS_AFTER_R5.md`); for Pre-R5 baseline purposes, **warm-reboot is the reference state** because the cold-boot GPS failure is a Cycle-3 problem, not an R-5 problem.

## Reference scripts + files

These are the artifacts a tier-end regression script reads to confirm "still matches baseline":

| Signal | Source | Verification command |
|--------|--------|----------------------|
| Vehicle build | `build_flight/rocketchip.elf` | `cmake --build build_flight 2>&1 \| tail` (expect "Linking CXX executable rocketchip.elf") |
| Station build | `build_station_flight/rocketchip.elf` | `cmake --build build_station_flight 2>&1 \| tail` |
| Host ctest | `build_host/CTestTestfile.cmake` | `cd build_host && ctest --output-on-failure 2>&1 \| tail` (expect `100% tests passed, 0 tests failed out of 800`) |
| Vehicle banner | COM7 | `python /tmp/serial_reader.py COM7 6 <log>` (flash via probe first; expect Hardware 14/14 OK + `flight-` build-config prefix) |
| Vehicle bench_sim | `scripts/bench_sim.py` | `python scripts/bench_sim.py` (expect 2/2 PASS) |
| Station bench_sim | `scripts/station_bench_sim.py` | `python scripts/station_bench_sim.py` (expect 3/3 PASS) |
| SPIN | `tools/spin/run_stage_o_ao_spin.sh` (or equivalent) | (re-verify path at first SPIN run during migration) |

## What a regression looks like

At any Tier-N close, if the post-migration measurements show:

- ctest count != 800 (and the delta isn't accounted for by intentional test additions): investigate
- Banner byte-on-wire diff != captured above (and the delta isn't accounted for by intentional rc_log-emitted-string change): investigate
- bench_sim 2/2 reports a regex miss: this is the LL-Entry-36 / Council-amendment-#4 risk; investigate by capturing the specific log line that drifted
- station bench_sim 3/3 reports a failure: same investigation
- SPIN errors > 0: investigate (not expected to be affected by R-5 since SPIN models verify state machines, not log strings)

## Cross-references

- `docs/audits/STDIO_FORMAT_SPEC_INVENTORY_2026-05-15.md` — Unit A's format-spec inventory + transitive-leak finding (scope correction: 23 files / ~624 callsites / 13 format-spec patterns)
- `C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn.md` — R-5 plan (this baseline is Unit A's second deliverable)
- `standards/HW_GATE_DISCIPLINE.md` Rule 6 — Level-1 / Level-2 / Level-3 verification credit
- `.claude/LESSONS_LEARNED.md` Entry 36 — bench_sim regex rot pattern (the council amendment #4 defense is grounded in this)
