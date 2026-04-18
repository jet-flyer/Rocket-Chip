# Stage 16C — Status Snapshot

**Last updated:** 2026-04-18 — **STAGE COMPLETE**
**Plan:** `docs/plans/STAGE16C_STATION_DECOUPLING.md` (council-reviewed
2026-04-17: NASA/JPL, Professor, Cubesat; IVP-142c council 2026-04-18:
NASA/JPL, Professor, Rocketeer).

## Current state

**Stage 16C COMPLETE.** All IVPs closed:

- **IVP-139** `fd088df` — IVP.md Stage 16C table activation
- **IVP-140** `b54cb07` — station_idle_tick scaffolding + positive-control gate (redo after original `26b83d4` reverted)
- **IVP-141** — station GPS poll via idle bridge; `core1_update_best_gps_fix` + `core1_read_gps` promoted to shared helpers
- **IVP-142a** — MCU die-temp driver with package-aware AINSEL (RP2350A=4, RP2350B=8); seqlock field + static_assert size bump
- **IVP-142b-1** — MCU HealthLevel hysteresis FSM, pure `mcu_temp_classify()` for host tests
- **IVP-142b-2** — HealthCritical byte, visibility-only operator alerts (council: no dead safe-mode paths)
- **IVP-142b-3** `56a5834` — persistence + phase gate on `critical_fault()`
- **IVP-142c** `dadcbf1` — **station HealthMonitor parity via capability-masking**
- **IVP-143** — capability flags + Tiny 2350+ / Pico 2 scaffolding + CMakePresets.json
- **IVP-144** `753aa64` — `cmd_station_*` decoupling audit (clean)
- **IVP-145** — exit gate: 4 builds clean, 724/724 host tests, bench_sim 2/2 PASS, vehicle+station 5-min soaks PASS

## Session outcome (2026-04-17)

**B1 resolved (but not as expected).** Station I2C bus showed all sensors
`NOT FOUND` for the entire session, despite firmware appearing correct.
Root cause turned out to be a **marginal/broken STEMMA QT cable** — the
original cable had intact power/GND (GPS power LED solid) but broken
or marginal SDA/SCL data lines. Swapping in a known-good cable restored
I2C. Plugging the original cable back in afterwards also worked —
likely the swap-and-swap-back reseated the connectors enough to clear
whatever was wrong.

**Productive firmware changes committed** (in addition to the bad-cable
workaround):
- GPIO 22 (shared DAC + ESP32-C6 active-low RESET) now released in
  `board::board_release_peripheral_reset()`. Before this, the onboard
  TLV320DAC3100 at 0x18 was held silent and the bus appeared empty
  even with a good cable — it masked the positive-control signal that
  would have isolated the cable fault in 10 minutes instead of several
  hours.
- `gps_pa1010d_init()` relocated to `init_early_hw()`, blind PMTK
  configuration sequence + aggressive retry probe loop. The blind PMTK
  approach is what keeps the PA1010D in full-power I2C mode after
  cold boot. With the good cable, PMTK writes now return full byte
  counts `[51,18,51]` and `window_hit:1` / `init:1` (verified in
  Hardware Status `b` output).
- Instrumentation in `b` output (`[DBG ] GPS early-init:` line) so
  future regressions are self-diagnosing.

## B2 — IVP-140 false-positive gate — RESOLVED via revert

IVP-140 (`26b83d4`) was reverted as part of the Stage 16C reset. The
scaffolding itself (`src/station/station_idle_tick.{h,cpp}` + `main.cpp`
wiring + `CMakeLists.txt` source entries) is cleanly removed. Stage 16C
will pick back up from IVP-140 in a future session with a stricter gate
definition (see "HW gate discipline" follow-up).

## Follow-up work tracked for next session

1. **Redo IVP-140** with a real gate: a scaffolding commit should
   verify the no-op tick actually runs under `kRadioModeRx`, MSP + AO
   queue stability observed under at least one real operational
   scenario (radio packet receive + ACK retry), and no regression in
   station-bench 5-min soak.

2. **Continue Stage 16C** items IVP-141..145 from the committed plan.

3. **HW-gate discipline standard** (user request this session): define
   what counts as a "pass" for gates that involve external hardware.
   The IVP-140 false positive and the broader "bad cable masquerading
   as firmware bug" episode both point to the same gap —
   verification needs a positive-control signal that proves the
   hardware is delivering the expected behavior, not just that the
   firmware code path didn't crash. Same meta-pattern as LL Entry 36
   (bench-sim silent rot). Candidate deliverable:
   `standards/HW_GATE_DISCIPLINE.md` + amendment to
   `SESSION_CHECKLIST.md`.

4. **Bare-metal I2C probe test** (`tools/i2c_bare_test/`) created
   during debugging and used productively. Left uncommitted per plan
   scope, but the pattern is worth reusing — a single-file POR-flash
   I2C scan + DAC-control test would be valuable to keep for future
   HW triage. Decision deferred.

5. **Investigate whether bus-corruption state can persist across full
   power cycles on the Fruit Jam** (user hunch). During this session
   we observed one boot where the exact same binary that had worked
   minutes before suddenly NACK'd everything, and this pattern matches
   some behavior described in LL Entries 25/27/28/31. The cable
   theory explains most of today's observations but not quite all of
   them. Worth monitoring in future sessions — if a full POR-plus-
   reseat-all-cables procedure becomes necessary for reliable boots,
   that's a finding.

## Repo state at session end

- **On main:** IVP-140 reverted, plan+status committed at `10d78fe`,
  IVP-139 at `fd088df`. New commit landing today with the proven GPS
  fix (GPIO 22 release, PMTK blind-write early init, DBG line).
- **No stashes carried forward.** IVP-141 code from earlier in session
  was not preserved (will be redone from spec in future session).
- **`tools/i2c_bare_test/`** untracked (kept as local scratch).
- **`/tmp/stage-j-test`** worktree still present; can be removed with
  `git worktree remove /tmp/stage-j-test` when no longer needed.
