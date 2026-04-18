# Stage 16C (items 1-3) — Station Runtime Decoupling + Tiny 2350 Readiness

## Context

Stage 16B bench validation completed 2026-04-17 with a quantified gap: station Core 1 is idle on all non-vehicle roles (`core1_entry()` spins on `g_startSensorPhase` which is only set when `kRole == kVehicle`), so station's initialized GPS is never polled and `cmd_station_gps_push()` reads stale zeros from the seqlock. User also flagged during Stage 16B that the vehicle Feather ran warm after a 30-min battery soak and would like MCU die temperature captured from next soak forward. Upcoming Pimoroni Tiny 2350 hardware also needs to slot in cleanly as a third board without touching role code.

**Scope — items 1, 2, 3 of Stage 16C + Tiny_2350+ / Pico 2 scaffolding + MCU die temp integrated into AO_HealthMonitor.** Not included: RadioScheduler ACK sync (item 4), Tiny 2350 full port with HW bring-up (item 5 full). Battery ADC, non-blocking GPS refactor, station AO_FlightDirector/HealthMonitor/LedEngine enablement (station is telemetry-relay, not flight-director host) are all out of scope.

**Intended outcome.** Station role polls its own GPS at ~10 Hz on every board (Feather, Fruit Jam, Tiny_2350+, Pico 2) by reusing the existing vehicle GPS driver and function-pointer infrastructure unchanged — no second GPS code path. MCU die temp captured at ~1 Hz on both roles on every RP2350 board (on-die sensor, zero hardware work), shown in `diag_stats_dump`, AND wired into `AO_HealthMonitor` with datasheet-sourced warn/fault/safe-mode thresholds. Over-temp becomes a tracked health fault; at safe-mode threshold (≥105°C) AO_FlightDirector auto-ABORTs if ARMED. Four CMake presets cover the current build matrix. Tiny_2350+ and Pico 2 become ~1-file board-header drops when respective hardware arrives, with no role-code churn.

## Phase-1 Audit Findings (already done)

- Source is already role-gated cleanly via `kRadioModeRx` / `if constexpr(kRole == kVehicle)`. All ~10 role gates use role, not `PICO_BOARD`.
- Board abstraction via `board::` namespace in `include/rocketchip/board_{feather_rp2350,fruit_jam}.h` already exists. `board.h` dispatches on `ADAFRUIT_FRUIT_JAM` vs `ADAFRUIT_FEATHER_RP2350` macros.
- ~6 `PICO_BOARD` refs in source are all legitimate (data provenance in `pcm_frame.cpp`, debug banners in `ao_rcos.cpp`/`diag_stats.cpp`).
- `qv_idle_bridge()` in `src/main.cpp:529-567` already hosts non-AO tick work: `watchdog_kick_tick`, `eskf_runner_tick`, `AO_Telemetry_cmd_retry_tick` (station), `diag_stats_msp_tick`. This is the natural host for station periodic work — it is not an AO handler, so LL Entry 32's no-blocking-in-AO rule does not apply. `gps_pa1010d_update()` worst-case ~6 ms I2C is safe here.
- Seqlock `shared_sensor_data_t` (148 B) in `include/rocketchip/sensor_seqlock.h` already has full GPS fields. Same-core writer/reader (station Core 0 writes, Core 0 AOs read) is memory-order-correct; no cross-core sync needed when writer and readers share a core.
- No `adc_set_temp_sensor_enabled` anywhere — MCU temp is fresh work.

## Architectural Decision

**B-alt: idle-bridge extension, Core 1 stays idle on station.**

- Safe for ~6 ms GPS I2C (idle bridge is not an AO; LL Entry 32 does not apply).
- Minimal code. Reuses the `AO_Telemetry_cmd_retry_tick` pattern already in place.
- Station Core 0 remains sole I2C client (already the invariant today: `rc_os_i2c_scan_allowed = true` only on station).
- Vehicle completely unchanged. All new behavior gated by `if constexpr (kRadioModeRx)` or role.
- Future migration to an `AO_StationSensor` is a drop-in swap once GPS is refactored non-blocking. The adapter module is written so the swap is call-site-stable.

Rejected: (A) station Core 1 mirror loop — duplicates the lockout/I2C-own dance, pins Core 1 power cost for no sample-rate benefit. (B) new AO_StationSensor on Core 0 — blocked today by GPS 6 ms I2C would overrun AO handler budget.

## Additional Board Readiness Strategy (Tiny_2350 / Tiny_2350+ + Pico 2)

Two upcoming/additional RP2350 boards to scaffold alongside the existing Feather + Fruit Jam:

**Pimoroni Tiny_2350 (base) and Tiny_2350+ (Plus variant).** Per Pimoroni nomenclature: "Tiny 2350" is the base product, "Tiny 2350 Plus" is the higher-memory variant. Code uses `Tiny_2350` and `Tiny_2350+` as user-facing names in the `kBoardName` string; filenames use `board_tiny_2350.h` and `board_tiny_2350_plus.h` (the `+` is invalid in C++ identifiers and filenames). User has the `Tiny_2350+` (Plus) variant in hand. Both are very small RP2350A modules: no on-board I2C breakouts, no SD slot, no DVI/HSTX, limited GPIO count, one user LED, optional external NeoPixel.

**Raspberry Pi Pico 2.** Official RP2350 board. Standard Pico footprint, 4 MB flash, 520 KB SRAM, one user LED (GPIO 25), no NeoPixel, no PSRAM, exposed standard GPIO0-28. Once the capability-flag system is in place, Pico 2 scaffolding is trivial — it's just another board header listing its pin map and capability flags.

User's intent is these are "another board" in the same sense as Feather or Fruit Jam — NOT "another role." The capability-flag system (IVP-143a) lets any role run on any board that has the required capabilities.

**Approach:** add board feature flags that abstract capability gates, and land a `board_tiny2350.h` header as a populated-but-untested stub so the Tiny port is a board-header pin assignment + boot bring-up rather than a distributed hunt for `#ifdef ADAFRUIT_FRUIT_JAM`.

Specifically, in every existing `board_*.h`:
- `inline constexpr bool kUartGpsAvailable` — already present.
- `inline constexpr bool kPsramAvailable` — new flag. Today hardcoded by `psram_init()` CS pin; promote to feature flag.
- `inline constexpr bool kDvmAvailable` — new flag (DVI/HSTX display output). Today only Feather HSTX has it, not yet driven. Keep flag=false everywhere until a display driver lands.
- `inline constexpr bool kSdCardAvailable` — new flag. Today nothing uses it; adding now pays down future debt when logging ever writes SD instead of PSRAM/flash.
- `inline constexpr uint8_t kNeoPixelCount` — already present (Feather=1, Fruit Jam=5). Tiny 2350 = 0 or 1 depending on whether the external pixel is wired.
- `inline constexpr bool kI2cStemmaAvailable` — new flag. Tiny 2350 without a breakout has no on-board I2C sensors; for a Tiny station build the I2C GPS is wired externally, or GPS goes over UART if pins allow.

Any call site that previously said "if Fruit Jam..." because of a capability becomes "if board::kXAvailable...". No role code changes.

**Header landing plan (per council Amendment 3 — narrowed scope):**

1. `board_tiny_2350_common.h` — shared Tiny_2350 pin assignments (~95% of both variants), best-guess populated, all unverified pins tagged `// TODO(Tiny_2350): verify on hardware`.
2. `board_tiny_2350_plus.h` — populated placeholder for the Plus variant (user has this one). `kBoardName = "Pimoroni Tiny_2350+"`. Contains `#error` guard unless `TINY_2350_BRINGUP_OK` defined.
3. `board_tiny_2350.h` — base-variant header DEFERRED entirely until the Plus variant completes hardware bring-up.
4. `board_pico2.h` — populated best-guess from Raspberry Pi Pico 2 official pinout (standard, well-documented). Same `#error` guard unless `PICO2_BRINGUP_OK` defined.

No builds compile for these boards until the user explicitly passes `-DPICO_BOARD=<name>` AND defines the corresponding `*_BRINGUP_OK` macro, which forces the hardware bring-up conversation to happen at hardware arrival, not at plan approval.

## IVP Breakdown

Seven IVPs sized to "self-contained, buildable, committable, with a clear gate." Each lands independently. Numbering: Stage 16B used `IVP-132` / `IVP-132a`, Stage 17 owns `IVP-134` through `IVP-138`, so Stage 16C picks up at **IVP-139** for the IVP.md update (following the Stage 16B precedent of `NNNa` for a leading doc-alignment IVP), then **IVP-140 through IVP-145** for implementation.

**Gating conventions followed by every IVP in this stage** (per `SESSION_CHECKLIST.md` + project norms):
1. Build clean on **all 4 current combos** (`build/`, `build_flight/`, `build_station/`, `build_station_flight/`) — station + vehicle parity is a hard checklist item.
2. Host tests + SPIN models pass (counts called out per IVP where they change).
3. Pre-commit hook passes (includes the `bench_sim` canary when touching flight-critical paths, per LL Entry 36).
4. HW verify when the IVP changes runtime behavior — flash via **debug probe**, not picotool (LL Entries 25, 27).
5. CHANGELOG + PROJECT_STATUS updates per SESSION_CHECKLIST stage-end convention. Mid-stage IVPs use commit messages; aggregate docs update at stage exit (IVP-145).
6. No protected-file edits without explicit user approval (`CLAUDE.md` / `AK_GUIDELINES.md` / `CODING_STANDARDS.md` / `SAD.md` / `MULTICORE_RULES.md` / `SCAFFOLDING.md` / `COUNCIL_PROCESS.md`). `docs/IVP.md` is listed as protected — **IVP-139 is the explicit-approval grant** for Stage 16C rows only, and is the first step of the stage.

### IVP-139 — IVP.md Stage 16C Table Entry (first step, source-of-truth alignment)

**Purpose:** Replace the "DEFERRED" placeholder in `docs/IVP.md` Stage 16C section (lines 3455-3484) with an active table matching the Stage 16B format. Source-of-truth alignment before any implementation-numbered IVP begins — same pattern Stage 16B used with IVP-124a.

**What changes in `docs/IVP.md`:**
- Stage 16C status: "DEFERRED" → "In Progress — items 1-3 + MCU die temp + AO_HealthMonitor integration + Tiny_2350+/Pico 2 scaffolding"
- Items 4 and 5 of the original Stage 16C scope (RadioScheduler ACK sync + full Tiny_2350 HW bring-up) explicitly marked DEFERRED in a separate sub-section.
- Add scope narrative + plan file reference (`.claude/plans/sunny-hugging-pumpkin.md`) + council-reviewed date + NASA/JPL, Professor, Cubesat panelist list.
- Add `| Step | Title | Brief Description |` table (matching Stage 16B's column format at line 3442):

```
| IVP-139 | IVP.md Stage 16C Table Entry | Update IVP.md status + Stage 16C detail to match this plan. Source-of-truth alignment before implementation-numbered work begins. (This IVP.) |
| IVP-140  | Station Idle-Tick Adapter Scaffolding | Create src/station/station_idle_tick.{h,cpp} (no-op). Wire into qv_idle_bridge() under kRadioModeRx. Reuses pattern of AO_Telemetry_cmd_retry_tick. |
| IVP-141  | Station GPS Poll via Idle Bridge | Implement station_idle_tick() to reuse existing g_gpsFnUpdate/g_gpsFnGetData function pointers + seqlock_write. Zero new GPS code; transport-agnostic (UART where available, I2C on Fruit Jam). |
| IVP-142a | MCU Die Temp Driver + Seqlock Field | New src/drivers/mcu_temp.{h,cpp}. Add mcu_die_temp_c + mcu_temp_read_count to shared_sensor_data_t (size change with static_assert bump). Capture ~1 Hz both roles. diag_stats dump line. |
| IVP-142b | MCU Over-Temp in AO_HealthMonitor + Safe-Mode Trigger | Extend HealthState::primary to 16-bit (MCU slot). Datasheet-sourced thresholds (70/85/105°C) with 2°C hysteresis. SIG_MCU_OVERTEMP posted to AO_FlightDirector at safe-mode. ABORT if ARMED. |
| IVP-143  | Capability Flags + Tiny_2350+ / Pico 2 Scaffolding + CMakePresets | Add kPsramAvailable/kDvmAvailable/kSdCardAvailable/kI2cStemmaAvailable flags. Land board_tiny_2350_common.h + board_tiny_2350_plus.h + board_pico2.h with #error guards. 4 CMakePresets. Comment/string hygiene. |
| IVP-144  | cmd_station_* Decoupling Audit | Grep-validated audit: no hardcoded "Fruit Jam"/PICO_BOARD references in cmd_station_* code paths. Audit-only IVP; fix any drift found in same commit. |
| IVP-145  | Stage 16C Exit Gate + Stage-End Docs | Full verification matrix (all 4 builds, host tests, SPIN, 5-min + 30-min soaks both roles, over-temp force-test via GDB). Stage-end doc updates to CHANGELOG/PROJECT_STATUS/AGENT_WHITEBOARD/BENCH_TEST_PROCEDURE. |
```

**Files modified:** `docs/IVP.md` only (single-purpose commit).

**Gate:**
- User explicitly approves the edit to protected file `docs/IVP.md` (approval of this plan = approval for this single Stage 16C table entry).
- Grep in `docs/IVP.md` for "Stage 16C" shows the new table, not the old placeholder.
- No other files touched in this commit.

**Rationale for doing this first:** Stage 16B's IVP-124a set the precedent — update the authoritative plan doc before any implementation-numbered work, so `docs/IVP.md` remains source of truth throughout the stage. If the plan changes mid-stage, this IVP re-fires (edit then commit). Prevents drift.

### IVP-140 — Adapter scaffolding (no-op tick)

Create `src/station/station_idle_tick.{h,cpp}` exporting `station_idle_tick_init()` and `station_idle_tick()`. Name reflects actual execution context (Core 0 idle bridge, not Core 1) — per council Amendment 1. Tick body is a no-op. Wire from `qv_idle_bridge()` under `if constexpr (kRadioModeRx)`, adjacent to `AO_Telemetry_cmd_retry_tick`. Init called once from `init_application()` station branch.

Files:
- `src/station/station_idle_tick.h` (new)
- `src/station/station_idle_tick.cpp` (new)
- `src/main.cpp` — include + wire
- `CMakeLists.txt` — add source

Gate: 4 build combos clean, host tests unchanged, station 5-min CLI soak — MSP stable, no behavior change.

### IVP-141 — Station GPS poll + seqlock publish @ 10 Hz

**This IVP reuses existing vehicle GPS infrastructure unchanged** — it does NOT add a second GPS code path. The GPS driver, transport selection (UART vs I2C), parser, and seqlock machinery all exist today and work on vehicle. The only new code is a ~30-line tick wrapper that calls the existing machinery from station's idle bridge.

What's reused verbatim:
- `g_gpsFnUpdate` / `g_gpsFnGetData` function pointers populated by `init_sensors()` at boot. Whichever transport was bound (UART on Feather/Pico 2, I2C PA1010D on Fruit Jam) is the same transport station uses.
- `gps_uart.cpp` / `gps_pa1010d.cpp` drivers — no changes.
- `seqlock_write` / `shared_sensor_data_t` GPS fields — no changes to layout.
- `core1_update_best_gps_fix()` — promoted from file-static to public helper so vehicle and station share one implementation (no duplication).

`station_idle_tick()` new code (~30 lines):
- Rate-limit at ~10 Hz using `time_us_32()` and `kGpsMinIntervalUs = 100'000`.
- Early-return if `!g_gpsInitialized`.
- Call `g_gpsFnUpdate()` / `g_gpsFnGetData()` — same calls vehicle makes.
- Populate local `shared_sensor_data_t`, call `update_best_gps_fix`, `seqlock_write`.

Transport-agnostic by construction: on any board with UART GPS available (Feather, Pico 2, future boards), the tick runs the ~100µs UART drain path. On Fruit Jam (UART pins unavailable), the tick runs the I2C PA1010D path (~6ms worst case). Same pointer, same behavior as vehicle.

Files:
- `src/station/station_idle_tick.cpp` — implementation
- `src/core1/sensor_core1.{h,cpp}` — expose helper

Gate:
- Station 5-min HW soak: `gps_read_count >= 2800` in `diag_stats_dump` (station-bench binary on Fruit Jam).
- With GPS antenna + open sky: `gps_valid==true`, `gps_fix_type >= 2`, `cmd_station_gps_push` prints real lat/lon (not `0.00000,0.00000`).
- Vehicle 5-min soak unchanged: IMU ≥990 Hz, baro ≥30 Hz, 0 errors.
- Host tests + SPIN 11/11 unchanged.

FMEA: I2C hang → `gps_error_count++`, no reset. GPS never locks → graceful "No GPS 3D fix" message. GPS uninit → tick short-circuits.

**Worst-case idle-bridge latency budget (Amendment 2, NASA/JPL):** One idle pass executes in sequence `watchdog_kick_tick` + `eskf_runner_tick` + `AO_Telemetry_cmd_retry_tick` + `station_idle_tick` + `diag_stats_msp_tick` + `__wfi()`. Tail-latency budget written here for record:

| Tick | Typical | Worst case |
|---|---|---|
| `watchdog_kick_tick` | <1 µs | <1 µs (just a SIO write) |
| `eskf_runner_tick` | ~50 µs (codegen FPFT in SRAM) | ~120 µs |
| `AO_Telemetry_cmd_retry_tick` | <1 µs idle | ~50 µs when firing a resend (rare) |
| `station_idle_tick` (GPS, UART transport) | ~50 µs (byte drain from hw FIFO) | ~150 µs |
| `station_idle_tick` (GPS, I2C transport, Fruit Jam only) | ~2 ms (padding-only) | ~6 ms (full 255-byte MT3333 buffer) |
| `station_idle_tick` (MCU temp, 1-in-10 ticks) | ~10 µs (one ADC read) | ~10 µs |
| `diag_stats_msp_tick` | <1 µs | <1 µs |

**Stacked worst case per idle pass: ~6.2 ms.** Ceiling = watchdog timeout 5000 ms (CODING_STANDARDS.md). Margin = 800×. IVP-141 soak must verify `diag_stats` never reports a watchdog-recovery event. Measured latency will be captured via `diag_stats_msp_tick` MSP trace and reported in the commit body.

### IVP-142 — MCU die temp: driver + seqlock field + HealthMonitor integration

**Scope clarification:** MCU die temp sensor is on-die in every RP2350 (ADC input 4 per RP2350 datasheet §12.4.6). It is NOT board-specific — available on Feather, Fruit Jam, Tiny_2350+, Pico 2, and any future RP2350 board with zero hardware work. Captured on both vehicle and station roles. On the flight board in particular, it's a health-monitoring input: MCU over-temp becomes a tracked fault in AO_HealthMonitor alongside existing sensor-health signals.

**Split into two sub-IVPs for clean commit boundaries:**
- **IVP-142a:** driver + seqlock field + diag_stats dump (the data-capture layer).
- **IVP-142b:** AO_HealthMonitor integration — over-temp becomes a tracked health fault + safe-mode trigger (the health-contract layer).

**Thresholds — sourced from RP2350 datasheet (§1.4.3 Absolute Maximum Ratings + §12.4.6 Temperature Sensor).** No invented numbers.

| Level | Temp range | Health mapping | Action |
|---|---|---|---|
| Normal | < 70°C | `kHealthOk` | none |
| Warn (degraded) | 70-85°C | `kHealthDegraded` | health re-publishes at next tick; LED pattern + telemetry reflect degraded state |
| Fault | 85-105°C | `kHealthFault` | exceeded RP2350 industrial operating limit (85°C). Latched in secondary flags byte. Preflight GO/NO-GO fails. |
| Safe-mode | ≥ 105°C | `kHealthFault` (latched) + `SIG_MCU_OVERTEMP` posted to AO_FlightDirector | 20°C margin below 125°C absolute-max junction temp. If ARMED, FD executes DISARM with MCU_OVERTEMP cause. If pre-ARM, blocks ARM via existing go/no-go path. |

Thresholds k-prefixed in `mcu_temp.h`: `kMcuTempWarnC = 70.0F`, `kMcuTempFaultC = 85.0F`, `kMcuTempSafeModeC = 105.0F`. Datasheet citations in comments. These are VALIDATE values per `docs/UNIQUE_COMMENT_ITEMS.md` tracking — flight-data confirmation of rocket-airframe thermal profile in Stage 18 may refine the warn threshold downward, but the datasheet ceiling (85°C / 105°C / 125°C) is a hard physical limit that won't move.

**Hysteresis:** 2°C on every threshold crossing (e.g., enter warn at 70°C, return to OK at 68°C) to prevent flicker near the boundary. Standard pattern for thermal monitoring.

**IVP-142a — Driver + seqlock field + diag_stats dump**

New `src/drivers/mcu_temp.{h,cpp}`:
- `bool mcu_temp_init()` — `adc_init()`, `adc_set_temp_sensor_enabled(true)`, set `g_mcuTempInitialized`.
- `float mcu_temp_read_c()` — `adc_select_input(4)`, `adc_read()`, apply Pico SDK conversion. Returns NaN if not inited.
- Constants k-prefixed: `kAdcVref`, `kAdcMaxCount`, `kTempVbeRef`, `kTempSlope`, `kTempOffsetC` (sourced from Pico SDK `hardware/adc.h` examples and RP2350 datasheet §12.4.6).

Add to `shared_sensor_data_t`:
```cpp
float mcu_die_temp_c;          // degC; NaN when not captured
uint32_t mcu_temp_read_count;  // monotonicity for soak
```
New size 156 B, update `static_assert`.

Capture cadence ~1 Hz both roles:
- Vehicle: new `mcuTempCycle` counter in `core1_sensor_loop()` at `kMcuTempDivider = 1000` (1 kHz loop → 1 Hz capture).
- Station: every 10th `station_idle_tick()` (~1 Hz at 10 Hz GPS rate).

`diag_stats_dump` adds `MCU temp=XX.XC` line; prints `---` if NaN.

Files:
- `src/drivers/mcu_temp.{h,cpp}` (new)
- `include/rocketchip/sensor_seqlock.h` — field + static_assert
- `src/core1/sensor_core1.cpp` — vehicle capture
- `src/station/station_idle_tick.cpp` — station capture
- `src/main.cpp` — `mcu_temp_init()` in `init_hardware()` both roles
- `src/dev/diag_stats.cpp` — dump line
- `CMakeLists.txt` — add source, link `hardware_adc`

Gate:
- Both roles boot: MCU temp 20-60 °C at room temp.
- `mcu_temp_read_count` monotone at ~1 Hz.
- 30-min vehicle FLIGHT-binary soak: temp trend visible, bounded 20-85 °C.
- Host tests pass (mcu_temp.cpp guarded by host-test gate or excluded from host target in CMake).

**Seqlock size-change gate (Amendment 4, NASA/JPL — promoted from bullet to hard gate):** IVP-142 does not close until the commit body contains the literal output of:
```
Grep "sizeof(shared_sensor_data_t)|\\b148\\b" src/ include/ test/ scripts/
```
with every hit addressed. Zero unaddressed hits = gate pass. Soft-gate-as-hard-gate antipattern (LL Entry 36) explicitly avoided.

**Replay-file size guard (Amendment 5, Professor + NASA/JPL):** IVP-131 replay harness must reject replay files whose frame size doesn't match current `sizeof(shared_sensor_data_t)`. Cheapest implementation: four lines in the replay loader checking `file_size % sizeof(shared_sensor_data_t) == 0`, with a clear error message pointing the user to re-capture under post-16C firmware. Prevents silent garbage when replaying pre-16C captures under post-16C binaries.

**IVP-142b — AO_HealthMonitor integration + safe-mode trigger**

Reuses the existing 2-bit health-encoding infrastructure (`src/safety/health_monitor.{h,cpp}`, `HealthLevel` enum, `HealthSecondary` flags byte, `health_monitor_tick()`). MCU temp is a new health input; no new framework needed.

**Primary byte change — add MCU slot.** The primary byte today is 4 subsystems × 2 bits (IMU/Baro/ESKF/GPS), fully packed. Options:
- (a) Move MCU to the secondary byte as a 1-bit "MCU over-temp" flag — loses the warn/fault distinction.
- (b) Extend the primary byte from 8-bit to 16-bit — adds a slot for MCU + future subsystems (battery ADC slot already planned), preserves 2-bit encoding so warn/fault/safe-mode states all fit.

Recommend **(b)** — the 2-bit warn/fault/safe encoding matches the thermal threshold table above. New `kHealthShiftMcu = 8` (bits [9:8] in extended primary). This is a schema change to `HealthState::primary`; downstream readers (AO_Logger FusedState, AO_Telemetry health byte, AO_LedEngine fault layer, CLI preflight) must be audited for stale `uint8_t primary` assumptions. Council will likely call this out — promoting the audit to a hard-gate grep similar to IVP-142a's `sizeof(shared_sensor_data_t)` gate.

**Hysteresis and threshold evaluation** in `health_monitor_tick()`:
- Read `g_sensorSeqlock` snap, get `mcu_die_temp_c`.
- Evaluate against `kMcuTempWarnC` / `kMcuTempFaultC` / `kMcuTempSafeModeC` with 2°C hysteresis, using previous-state tracking already in `HealthState::prev_primary`.
- Write new MCU HealthLevel into `primary` byte via `health_set_subsystem`.
- At safe-mode threshold (≥105°C), additionally post `SIG_MCU_OVERTEMP` event to AO_FlightDirector (new signal in `ao_signals.h`). Once posted, latched until reset — same latching pattern as other health faults.

**AO_FlightDirector response to `SIG_MCU_OVERTEMP`:**
- If state == ARMED: execute ABORT path with `MCU_OVERTEMP` cause. Pyro policy follows Mission Profile (same rules as other ABORT sources).
- If state != ARMED (IDLE, LANDED, ERROR): record cause, block subsequent ARM via go/no-go check. No state transition from non-ARMED states.
- Logged via existing `[FD]` log format and telemetry phase-change packet.

**NASA/JPL FMEA concerns the plan pre-answers:**
- ADC read fails (NaN temp): `mcu_temp_available()` false → MCU slot stays `kHealthAbsent` (0b00). Does not trigger safe-mode on sensor failure. Logged as separate `kHealthAbsent` in preflight.
- Rapid-cycling noise near 70°C boundary: 2°C hysteresis prevents flicker.
- False trigger during pad warm-up on a sunny day: 105°C is a 20°C margin below absolute max. Sunlight on a black rocket body can push electronics into the 70-85°C warn range but 105°C requires a genuine failure (stuck LDO, bad capacitor leakage, etc.). Field Stage 18 will validate the warn threshold; the fault + safe-mode thresholds are datasheet limits and won't move.

**Files modified (IVP-142b):**
- `src/safety/health_monitor.h` — extend primary to `uint16_t`, add `kHealthShiftMcu`
- `src/safety/health_monitor.cpp` — evaluate MCU temp against thresholds in `health_monitor_tick()`
- `include/rocketchip/ao_signals.h` — new `SIG_MCU_OVERTEMP`
- `src/active_objects/ao_flight_director.cpp` — handle `SIG_MCU_OVERTEMP`, hook into go/no-go
- `src/active_objects/ao_health_monitor.cpp` — subscribe to MCU over-temp if posting from HM, or let HM post directly to FD (confirm at impl time which is cleaner)
- `src/flight_director/go_nogo_checks.h` — add MCU over-temp check to Tier-1 preflight
- `src/drivers/mcu_temp.h` — threshold constants with datasheet citations
- `test/` — host test for threshold + hysteresis logic

**Gate (IVP-142b):**
- Host test covers all 4 states (Ok/Warn/Fault/SafeMode) + hysteresis boundaries on both entry and exit.
- SPIN model adds `SIG_MCU_OVERTEMP` path; P6 (abort liveness) still passes. SPIN 12/12 if a new gate is added, or 11/11 unchanged if the path is folded into existing abort gate.
- HW soak: force high temp reading via debug probe override (gdb `set var` on `mcu_die_temp_c`) and verify (a) warn → degraded LED + telemetry, (b) fault → health fault + GO/NO-GO fails, (c) safe-mode → `SIG_MCU_OVERTEMP` posted, FD receives, ARM blocked (or ABORT if armed).
- 30-min vehicle FLIGHT-binary soak: real-world MCU temp stays in `kHealthOk` band (< 70°C) under baseline load. If it touches warn, that's a thermal-design finding worth acting on.
- `HealthState::primary` schema-change grep gate (analogous to Amendment 4 for seqlock): `Grep "HealthState|primary|health_get_subsystem"` output in commit body, all stale 8-bit assumptions addressed.

### IVP-143 — Board/role decoupling polish: capability flags + Tiny_2350 + Pico 2 scaffolding + CMakePresets

**(a) Capability flags** added to all `board_*.h`:
- `kPsramAvailable`, `kDvmAvailable`, `kSdCardAvailable`, `kI2cStemmaAvailable`.
- Existing `kUartGpsAvailable`, `kNeoPixelCount` stay.

Update call sites that previously branched on `PICO_BOARD` / `ADAFRUIT_FRUIT_JAM` to branch on the capability flag instead. Expected: `psram_init()` call path uses `kPsramAvailable`; `i2c_bus_scan` logging message uses `kBoardName` not hardcoded "Fruit Jam."

**(b) Additional-board scaffolding — narrowed per council Amendment 3 (Cubesat + Professor).** Original plan landed two populated variant headers against datasheet-only data. Revised approach lands scaffolding for TWO additional RP2350 boards alongside existing Feather + Fruit Jam:

**Tiny_2350 / Tiny_2350+ (Pimoroni):**
1. `board_tiny_2350_common.h` — shared ~95% of pin assignments, all `// TODO(Tiny_2350): verify on hardware` tagged.
2. `board_tiny_2350_plus.h` — **Tiny_2350+ variant** (the Plus; user has this one in hand). Structure:

```cpp
#pragma once
#include "board_tiny_2350_common.h"
#ifndef TINY_2350_BRINGUP_OK
#error "Tiny_2350 pin map not yet verified on hardware. Define TINY_2350_BRINGUP_OK after hardware bring-up in a future IVP."
#endif
// Tiny_2350+ specific overrides:
inline constexpr bool kPsramAvailable = true;                       // Plus has it
inline constexpr const char* kBoardName = "Pimoroni Tiny_2350+";
// inline constexpr uint32_t kFlashSizeMb = <confirm>;              // populated at bring-up
```

3. `board_tiny_2350.h` — **base variant** — DEFERRED entirely until the Plus variant completes hardware bring-up.

**Raspberry Pi Pico 2 (official RP2350 board):**

4. `board_pico2.h` — populated placeholder with standard Pico 2 pin assignments. Same `#error` guard pattern:

```cpp
#pragma once
#ifndef PICO2_BRINGUP_OK
#error "Pico 2 pin map not yet verified on hardware. Define PICO2_BRINGUP_OK after hardware bring-up in a future IVP."
#endif
// ... populated pin map ...
```

Pico 2 pinout is standard and well-documented in the Raspberry Pi datasheet, so `// TODO(Pico2): verify on hardware` tags are fewer and lower-risk than Tiny_2350. Once the capability-flag system is in place, Pico 2 should be trivial to bring up per user's note.

The `#error` guards force bring-up conversations to happen at hardware arrival, not plan approval.

**Tiny_2350 common-header best-guess values** (all `// TODO(Tiny_2350): verify on hardware`):
- I2C: GPIO 20/21 or similar (Tiny has limited broken-out pins; user may need external breakout).
- SPI for radio: best-available 4-pin cluster.
- `kUartGpsAvailable`: TBD based on pin availability.
- `kDvmAvailable = false`, `kSdCardAvailable = false`.
- `kNeoPixelCount = 1` (on-board RGB LED per Pimoroni schematic).
- `kBoardName`: NOT set in common — variant sets it ("Pimoroni Tiny_2350" vs "Pimoroni Tiny_2350+").
- `kPsramAvailable`: NOT set in common — required override per variant (Plus=true, base=false).

**Pico 2 best-guess values** (standard Pico footprint from Raspberry Pi datasheet):
- I2C: GPIO 4/5 (I2C0) or 6/7 (I2C1).
- SPI: GPIO 16-19 (SPI0).
- UART: GPIO 0/1 (UART0).
- `kUartGpsAvailable = true` (GPIO 0/1 free, unlike Fruit Jam's USB-host conflict).
- `kPsramAvailable = false`, `kDvmAvailable = false`, `kSdCardAvailable = false`.
- `kNeoPixelCount = 0` (Pico 2 has only a standard GPIO LED, no RGB).
- `kBoardName = "Raspberry Pi Pico 2"`.
- `kLedPin = 25`, `kLedActiveHigh = true`.

**(c) CMakePresets.json** encoding 4 current combos plus ready slots for Tiny variants:
```json
{
  "version": 3,
  "configurePresets": [
    {"name": "vehicle-bench",      "binaryDir": "build",                 "cacheVariables": {"BUILD_FOR_FLIGHT": "OFF"}},
    {"name": "vehicle-flight",     "binaryDir": "build_flight",          "cacheVariables": {"BUILD_FOR_FLIGHT": "ON"}},
    {"name": "station-bench",      "binaryDir": "build_station",         "cacheVariables": {"PICO_BOARD": "adafruit_fruit_jam", "ROCKETCHIP_JOB_STATION": "1", "BUILD_FOR_FLIGHT": "OFF"}},
    {"name": "station-flight",     "binaryDir": "build_station_flight",  "cacheVariables": {"PICO_BOARD": "adafruit_fruit_jam", "ROCKETCHIP_JOB_STATION": "1", "BUILD_FOR_FLIGHT": "ON"}}
  ],
  "buildPresets": [ /* corresponding 4 */ ]
}
```
Additional-board presets (e.g., `vehicle-bench-tiny-plus`, `station-bench-tiny-plus`, `vehicle-bench-pico2`, `station-bench-pico2`) are documented in `docs/BENCH_TEST_PROCEDURE.md` but not added to the presets file until respective hardware ships — avoids dangling configurations that fail CI without warning.

**(d) Comment/string hygiene:** replace hardcoded board names in comments (`i2c_bus.cpp:49`, `ws2812_status.cpp:109/117`, `rfm95w.cpp:312`, `rfm95w.h:148`, `main.cpp:287`, `notify_backend_audio.cpp:7`) with `board::kBoardName` refs or generic "RP2350B-class pads" language.

Files:
- `include/rocketchip/board_{feather_rp2350,fruit_jam}.h` — add capability flags
- `include/rocketchip/board_tiny_2350_common.h` + `board_tiny_2350_plus.h` placeholder for Plus variant user has in hand (base `board_tiny_2350.h` deferred until Plus completes HW bring-up per council Amendment 3)
- `include/rocketchip/board_pico2.h` placeholder (Raspberry Pi Pico 2 — trivial addition once capability-flag system lands)
- `include/rocketchip/board.h` — add Tiny_2350+ and Pico 2 selector branches
- `CMakePresets.json` (new)
- ~6 source files (comment edits only)
- Any call site currently branching on `PICO_BOARD` macro for a capability → switch to `board::kXAvailable`

Gate:
- `Grep -i "fruit jam|fruitjam"` in `src/**/*.{cpp,h}` returns 0 hits except legitimate data-provenance strings (allow-list in commit body).
- All 4 presets configure+build clean.
- `board_tiny2350.h` compiles as an unused header (`cmake --build` of existing presets unaffected).
- `#if defined(PIMORONI_TINY2350)` branch in `board.h` is syntactically valid (compile-test by temporarily defining it).

### IVP-144 — `cmd_station_*` decoupling audit

Grep-validated audit. Expected clean per Phase-1 but needs to be proven.

Approach:
1. `Grep -i "fruit jam|fruitjam|feather"` in `src/cli/rc_os_commands.cpp` (`cmd_station_*` bodies).
2. `Grep "PICO_BOARD|ADAFRUIT_FRUIT_JAM"` in `src/cli/`.
3. Validate board-dependent output uses `board::kBoardName`.
4. Fix any drift in the same commit.

Files: likely none (audit-only); if drift found, the relevant file.

Gate: audit table in commit body (command/file/line/disposition); zero hits for disallowed patterns.

### IVP-145 — Stage 16C items 1-3 exit gate + docs

Verification matrix:

| Gate | How |
|---|---|
| 4 build combos compile clean | `cmake --preset X && ninja -C build_X` all X |
| Host tests pass (incl. new MCU temp + threshold/hysteresis tests) | standard host build + `ctest` |
| SPIN pass (unchanged 11/11 or 12/12 if `SIG_MCU_OVERTEMP` path added) | project's SPIN invocation |
| Vehicle 5-min bench soak | IMU ≥990 Hz, baro ≥30 Hz, 0 errors, MCU temp populates, health byte shows MCU `kHealthOk` |
| Vehicle 30-min FLIGHT soak | Tier-1 criteria at 7 snapshots, MCU temp trend visible, health byte MCU stays `kHealthOk` under baseline load (touching warn at 70°C is a thermal finding, not a gate fail) |
| Station 5-min soak | `gps_read_count ≥ 2800`, `gps_valid=true` w/ antenna, `cmd_station_gps_push` real data, MCU temp sane |
| Station 30-min FLIGHT soak | monotone `gps_read_count`, `mcu_temp_read_count`, 0 watchdog resets |
| Over-temp force-test | GDB override of `mcu_die_temp_c` to 72/90/110°C verifies warn / fault / safe-mode transitions, ABORT from ARMED at safe-mode |
| T=0 preconditions both roles | `diag_stats_t0_preconditions` identity block unchanged |

Doc updates (allowed per SESSION_CHECKLIST stage-end convention):
- `docs/IVP.md` — IVP-140, 141, 142a, 142b, 143, 144, 145 rows under Stage 16C
- `docs/PROJECT_STATUS.md` — Stage 16C items 1-3 COMPLETE; items 4,5 DEFERRED
- `CHANGELOG.md` — stage summary
- `AGENT_WHITEBOARD.md` — close station GPS gap, Tiny 2350 readiness note, MCU temp completion
- `docs/BENCH_TEST_PROCEDURE.md` — reference CMakePresets; note future Tiny 2350 presets

Files NOT edited: CLAUDE.md, AK_GUIDELINES.md, CODING_STANDARDS.md, SAD.md, IVP.md structure outside stage rows, MULTICORE_RULES.md, SCAFFOLDING.md. If `SEQLOCK_DESIGN.md` needs the 148→156 byte update (from IVP-142), ask user first.

## Critical Files

- `src/main.cpp` — idle bridge wiring, MCU temp init both roles
- `src/core1/sensor_core1.{h,cpp}` — promote best-fix helper, add MCU temp capture
- `src/station/station_idle_tick.{h,cpp}` — new station periodic path
- `src/drivers/mcu_temp.{h,cpp}` — new
- `include/rocketchip/sensor_seqlock.h` — MCU temp field
- `include/rocketchip/board.h` + `board_tiny_2350_common.h` + `board_tiny_2350_plus.h` placeholder + `board_pico2.h` placeholder + existing board headers — capability flags + Tiny_2350+ + Pico 2 scaffolding (narrowed per council Amendment 3)
- `CMakePresets.json` — new
- `src/dev/diag_stats.cpp` — MCU temp dump line
- `CMakeLists.txt` — new sources, `hardware_adc` link

## Reused Existing Code

- `qv_idle_bridge()` pattern from `src/main.cpp:529-567`
- `AO_Telemetry_cmd_retry_tick` as the sibling station-only tick pattern
- `gps_pa1010d_update()` / `g_gpsFn*` function pointers from `init_sensors()`
- `seqlock_write` / `sensor_seqlock_t g_sensorSeqlock` from `include/rocketchip/sensor_seqlock.h`
- `board::k*Available` / `board::kBoardName` pattern already established in `board_{feather_rp2350,fruit_jam}.h`
- Pico SDK `hardware/adc.h` internal temp sensor, ADC input 4

## Risks

| Risk | Mitigation |
|---|---|
| Idle bridge adds ~6 ms @ 10 Hz | 6% CPU ceiling, matches vehicle Core 1 GPS. Documented in IVP-141 body. WFI still reached. |
| Seqlock struct size 148→156 B breaks readers | Pre-edit grep for `sizeof(shared_sensor_data_t)` / `148`. Fix static_assert. |
| Same-core seqlock writer on station (new pattern) | Memory-order correct regardless of writing core. Document in `SEQLOCK_DESIGN.md` (ask before edit). |
| CMakePresets on Windows + Ninja | Explicit `"generator": "Ninja"`; validate on user's box before marking IVP-143 done. |
| `board_tiny_2350_plus.h` or `board_pico2.h` pin assignments wrong | All unverified pins tagged `// TODO(Tiny_2350)` / `// TODO(Pico2)`. Neither board is built until user defines the `*_BRINGUP_OK` macro. Hardware bring-up is a separate future IVP. |
| Tiny_2350 vs Tiny_2350+ (memory-delta) diverge incorrectly | Variants share 95% of header; only `kPsramAvailable` + flash-size + `kBoardName` differ. Shared `board_tiny_2350_common.h` holds the common pins; variant headers include it. Base `Tiny_2350` header deferred entirely until `Tiny_2350+` HW bring-up completes. |
| Future battery ADC ADC-channel race | Document in `mcu_temp.h`: single-tick-owner rule for ADC consumers. |

## Verification End-to-End

1. Fresh checkout, run all four CMake presets. Each produces a clean `.elf`.
2. Host test suite via `ctest --test-dir build_host` — 709 pass.
3. SPIN suite passes 11/11.
4. Flash vehicle bench to Feather via probe. 5-min CLI soak: `diag_stats` shows IMU ≥990 Hz, MCU temp 20-60 °C increasing over warm-up period, no errors.
5. Flash vehicle FLIGHT to Feather via probe. 30-min battery soak under Tier-1 flight criteria. MCU temp trace available.
6. Flash station bench to Fruit Jam via probe. 5-min soak with GPS antenna: `gps_read_count` climbs at ~10 Hz, `cmd_station_gps_push` returns real lat/lon, MCU temp reads sanely.
7. Flash station FLIGHT to Fruit Jam via probe. 30-min soak. Monotone counters, zero watchdog.
8. Compile-test with `-DPICO_BOARD=pimoroni_tiny_2350_plus -DTINY_2350_BRINGUP_OK` temporarily — `board_tiny_2350_plus.h` compiles cleanly; revert. Repeat for `-DPICO_BOARD=pico2 -DPICO2_BRINGUP_OK` to verify Pico 2 header compiles. Base Tiny_2350 variant not compile-tested (header not yet written).

## Council Review

**Convened:** NASA/JPL Avionics Lead, Embedded Systems Professor, Cubesat Startup Engineer.

**Verdict: APPROVED with 5 amendments (now incorporated above).**

Amendments applied:
1. **Rename `src/core1/station_sensor_tick` → `src/station/station_idle_tick`** — reflects actual execution context (Core 0 idle bridge, not Core 1). Directory layout is documentation. (Professor)
2. **Worst-case idle-bridge latency budget** written into IVP-141 body as a table — tail latency ~6.2 ms per pass, 800× watchdog margin. (NASA/JPL)
3. **Tiny_2350 / Tiny_2350+ scaffolding narrowed** to one `board_tiny_2350_common.h` + one populated `board_tiny_2350_plus.h` for the Plus variant user has in hand, both `#error`-guarded unless `TINY_2350_BRINGUP_OK` defined. Base `board_tiny_2350.h` (non-Plus) deferred until Plus completes HW bring-up. Additionally (user follow-up): land `board_pico2.h` for Raspberry Pi Pico 2 with same guard pattern (`PICO2_BRINGUP_OK`) — trivial addition once the capability-flag system is in place. Forces bring-up conversations at hardware arrival, not plan approval. (Cubesat + Professor + user addition)
4. **Seqlock size-change grep promoted to hard gate** in IVP-142. Commit body must include `Grep "sizeof(shared_sensor_data_t)|148"` output with zero unaddressed hits. Avoids LL Entry 36 soft-gate antipattern. (NASA/JPL)
5. **Replay-file size guard added to IVP-142** — IVP-131 replay loader rejects files whose frame size doesn't match current `sizeof(shared_sensor_data_t)`. Four lines. Prevents silent garbage on pre-16C captures replayed under post-16C firmware. (Professor + NASA/JPL)

No dissents. Core architecture (idle-bridge B-alt, Core 1 stays idle on station, Tiny 2350 as board not role), IVP breakdown, and scope boundaries were unanimously endorsed.
