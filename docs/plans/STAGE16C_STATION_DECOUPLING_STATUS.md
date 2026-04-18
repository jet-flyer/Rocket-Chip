# Stage 16C — Status Snapshot

**Last updated:** 2026-04-17 ~20:30 local
**Plan:** `docs/plans/STAGE16C_STATION_DECOUPLING.md` (mirrored from
`.claude/plans/sunny-hugging-pumpkin.md`; council-reviewed 2026-04-17:
NASA/JPL, Professor, Cubesat).

## Scope in flight

Items 1–3 of original Stage 16C: station runtime decoupling, station/board
decoupling, station Core 1 sensor path — plus MCU die temp integrated into
AO_HealthMonitor, plus Tiny_2350+ / Pico 2 board scaffolding (populated
placeholders only, no HW bring-up this stage).

Deferred from original scope: item 4 (RadioScheduler ACK sync) and
item 5 full (Tiny 2350 full HW bring-up).

## IVP progression

| IVP  | Title | State | Commit | Notes |
|------|-------|-------|--------|-------|
| 139  | IVP.md Stage 16C table entry | Committed | `fd088df` | Source-of-truth alignment. |
| 140  | Station idle-tick adapter scaffolding (no-op) | **Commit present, gate claim SOFT** | `26b83d4` | Commit message claims "GATE PASS" but verification was shallow: build clean + MSP stable + AO queues stable. Station I2C sensors were already `NOT FOUND` at t=0 of that soak (all three sensor addresses NACK, `gps reads=0`) — agent did not catch this at the time and should not have marked pass. No regression introduced by the commit itself (no-op tick, gated by `kRadioModeRx`), but the gate claim overstates what was validated. **See "Blockers" below.** |
| 141  | Station GPS poll via idle bridge | **Code complete, HW-verify blocked, stashed** | — | Working-tree change stashed as `stash@{0}: WIP IVP-141 before Stage J checkout`. Contents: promote `core1_update_best_gps_fix` + `core1_read_gps` from file-static to public in `src/core1/sensor_core1.{h,cpp}`; implement `station_idle_tick()` body in `src/station/station_idle_tick.cpp` to reuse existing `g_gpsFn*` function pointers + seqlock publish. Builds clean on all 4 combos. **Not committed** because the HW gate (station `gps_read_count >= 2800` over 5 min) cannot be evaluated while station I2C sees zero devices. |
| 142a | MCU die temp driver + seqlock field | Not started | — | — |
| 142b | MCU over-temp in AO_HealthMonitor + safe-mode | Not started | — | — |
| 143  | Capability flags + Tiny_2350+/Pico 2 scaffolding + CMakePresets | Not started | — | — |
| 144  | `cmd_station_*` decoupling grep audit | Not started | — | — |
| 145  | Stage 16C exit gate + stage-end docs | Not started | — | — |

## Blockers

### B1 — Station I2C bus shows no devices at boot (open)

**Symptom.** On Fruit Jam station running any recent build (including Stage
16B final `f725b28` and current HEAD), the boot-time I2C scan reports
`NOT FOUND` at all three sensor addresses (0x0C / 0x69 / 0x77) and GPS
(0x10). Live GDB `i2c_bus_probe(0x10)` also returns 0. GPS module LED is
on (power confirmed). Radio works fine (SPI, `RegVersion=0x12`).

**What changed since last known-good.** Stage J (commit `7aeea79`,
2026-03-07) documented 15/15 HW parity items passing on Fruit Jam
including "GPS PA1010D on I2C0." Between that and today, no commit HW-
validated station I2C — every subsequent station soak (Stage 16B 132a
variants) was radio-only RX validation. Commits on the I2C path since
Stage J:

| Commit | Date | Effect |
|---|---|---|
| `f7c5cce` | 2026-03-31 | Added RP2350B pad-isolation fix to `i2c_bus_init()` (reason station GPS could work in the first place, per commit message). |
| `6f7efbf` | 2026-03-26 | Magic-number rename in `psram_init.cpp`, no logic change. |
| `1d2b683` | 2026-04-06 | Comments-only audit, 213 lines net reduction. |

**What has been ruled out as cause.**
- Initial misattribution to LL Entry 31 (`flash_safe_execute` corruption)
  — wrong, because `init_sensors()` runs before any `flash_safe_execute`
  in the boot sequence.
- Attempted fix: `reset_block(I2C0) + unreset` at top of `i2c_bus_init()`
  — did not change symptom; reverted.
- Pad-isolation fix is intact and SDK 2.2.0 `gpio_set_function()` does
  clear the `ISO` bit under `HAS_PADS_BANK0_ISOLATION` (confirmed via
  SDK source read).
- Stash `stash@{0}` contains IVP-141 WIP, does NOT touch I2C.
- My IVP-140 commit touches only `src/station/` + one line each in
  `main.cpp`'s idle bridge / init_application. Diff vs `f725b28` is
  clean in every I2C/GPS/boot-critical file.

**Open hypotheses.**
1. Accumulated SWD warm-reset state on I2C0 peripheral that even
   `reset_block` cannot clear without a POR (power cycle). Research
   agent's recommendation was explicit: power-cycle the Fruit Jam first
   to disambiguate code-vs-state.
2. Some commit between `7aeea79` and `f725b28` that affects boot timing
   for station but was never exercised against I2C sensors on station —
   to be found via bisect if POR does not resolve.

**Next action.** Enter plan mode. Full research session on this blocker
BEFORE any further Stage 16C coding. Not more guess-and-flash
iterations.

### B2 — IVP-140 commit has an over-broad GATE PASS claim (open)

The `26b83d4` commit message states the station 5-min HW soak passed,
citing MSP and AO queue high-water as evidence. Those are true facts,
but in the same diag dump `gps reads=0` and `station_gps: valid=NO
sats=0 fix_type=0` were already present — those were symptoms of a
pre-existing broken station I2C bus that agent did not investigate.
The IVP-140 *code* does not introduce any regression (no-op tick),
but the commit's gate-pass language is misleading.

**Next action.** After B1 is resolved, either:
- Amend the commit message to note partial verification (preferred —
  keeps the scaffolding commit), or
- Revert `26b83d4` and re-commit under a new SHA after B1 is fixed and
  a proper gate runs.

Decision deferred to post-B1. Flagged here so it isn't lost.

## Repo state

- **On main:** `26b83d4` (IVP-140 scaffolding) is HEAD. Two commits
  ahead of pre-Stage-16C (`2c4527e` Session wrap-up).
- **Stashed:** `stash@{0}: WIP IVP-141 before Stage J checkout` —
  shared-helper promotion in `src/core1/sensor_core1.{h,cpp}` + station
  tick body in `src/station/station_idle_tick.cpp`. 3 files, +87 / -10.
- **Worktree:** `/tmp/stage-j-test` checked out at Stage J (`7aeea79`)
  for comparison research. Still present; to be removed after research
  is done or kept as a reference checkout depending on the research
  plan.

## Memory / session notes

- User has corrected the agent repeatedly this session about:
  - Not asking hardware questions unless absolute last resort
  - Not committing code that isn't working
  - Not guessing at fixes without research
  - Doing proper HW verification, not shallow gates
- Research session entered 2026-04-17 after user directive: "store the
  current plan along with its current status in the repo and enter
  plan mode for a proper research session and fix."
