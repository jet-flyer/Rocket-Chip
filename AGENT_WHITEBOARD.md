# Agent Whiteboard

**Purpose:** Communication across context windows and between agents.

**Stages 1-14 COMPLETE.** 669 host tests, SPIN 11/11 (unchanged — AO_Notify is output-only). Tracking: `docs/AO_ARCHITECTURE.md`.

## Use Cases
1. **Cross-agent review** - Flag concerns about other agents' work (see `CROSS_AGENT_REVIEW.md`)
2. **Cross-context handoff** - Notes for future Claude sessions when context is lost
3. **Work-in-progress tracking** - Track incomplete tasks spanning multiple sessions
4. **Hardware decisions pending** - Flag items needing user input before code changes
5. **This is a whiteboard** - Erase completed items. Only keep active flags and deferred work.

---

## Open Flags

*Stage 13 (Health Monitor) COMPLETE. Config Wizard Phase A COMPLETE (prototype). Stage 14 (Notification Engine) COMPLETE — IVP-113 through IVP-118, all gates passed, HW verified via GDB memory inspection. SPIN safety analysis appended to NOTIFY_CONTRACT.md by user-approved one-time exception 2026-04-10.*

### Session Handoff (2026-04-15 → 2026-04-16)

**State:** Build compiles clean (bench + flight tiers). 709/709 host tests pass. OpenOCD may still be running in background (`taskkill //F //IM openocd.exe` to clean).

**Completed this session:** IVP-124a through IVP-131 (9 of 11 Stage 16B IVPs). Doc refresh, dev code audit, build-tier split, fault injection harness, PIO shakedown (5/5), sensor replay harness (5/5 profiles pass).

**Next:** IVP-132 (HW budgets + two-tier soak) then IVP-132a (station bench tests). Plan: `.claude/plans/stateless-hopping-allen.md`.

**Concerns:**
- Session-start canary was NOT run at the beginning of this session (missed checklist item 6). The pre-commit hook deferred HW verify on IVP-127a and IVP-127b. Run `python scripts/bench_sim.py` as first action next session.
- Serial replay via Python has USB CDC throughput issues — the `dev_replay_poll()` drain loop (256 bytes/poll) fixed it, but serial-based tests are slow (~150s per profile). GDB is more reliable for state verification.
- GDB locale warning (CP1252→UTF-32) on every `call` — cosmetic, Windows-only, doesn't affect functionality.
- `__pycache__` was briefly committed, now cleaned + `.gitignore` updated.

**Files being worked on:** `src/dev/`, `scripts/`, `tests/replay_profiles/`, `docs/FAULT_INJECTION.md`

### BENCH SIM RETIREMENT — RESOLVED (2026-04-12)

Old `bench_flight_sim.py` (479 lines, IVP-73) retired and replaced by `bench_sim.py` (~200 lines, 2 tests) in commit `5fbea19`. IVP-119 FusedState rename committed (`010f305`). Stage P7 + Stage 15 un-shelved. See LL Entry 36 for full root-cause analysis. Pre-commit hook now gates `ctest` + needs-based HW bench sim on flight-critical commits. Session-start canary added to SESSION_CHECKLIST.md item 6.

### ⚠️ RadioScheduler Sync — IVP-122 Command Delivery Unreliable (2026-04-12)

IVP-122 half-duplex ACK protocol is implemented and works when timing aligns (ARM ACK'd on first try during HW test), but command delivery is unreliable. DISARM failed 3 retries in the same test session. Root cause: the station sends commands at arbitrary times, but the vehicle only listens in brief RX windows between TX slots (~500ms cycle at 2Hz). The station needs to synchronize its TX to the vehicle's RX window.

**Research findings (LoRaWAN Class A, SiK TDMA, Altus Metrum poll-response):** The standard pattern is "listen-before-talk" — station sends a command immediately after receiving a vehicle packet, when the vehicle's RX window is guaranteed open. The SX1276 mode switch (TX→RX) takes ~100µs; the station's SPI FIFO write + TX start takes ~300-500µs. At SF7/BW125 this is well within the preamble detection window.

**Proposed fix:** In the station's `handle_rx_packet()`, when a vehicle nav packet is received AND a pending command exists, post `SIG_RADIO_TX` immediately (same tick). This uses the vehicle's natural post-TX RX window without new timing machinery. Requires a new IVP — this is a RadioScheduler architecture change, not an IVP-122 patch.

**Scope:** Affects all bidirectional communication (ACK, station GPS push, future config upload). Should be a dedicated RadioScheduler sync IVP before IVP-122 can be marked fully resolved.

### PIO Hardware Failure Gap — Gemini Tier (IVP-130 finding, 2026-04-15)

PIO backup timer shakedown (Scenario 5) confirmed: external PIO SM halt is undetectable by firmware. The PIO watchdog IRQ flag is only set by the PIO program itself — if the SM is disabled or corrupted, no flag is raised. ARM-side monitoring reintroduces the dependency chain the PIO was designed to avoid. The correct mitigation is physical redundancy: a second independent timer on a separate MCU. This is a Gemini-tier feature (dual-core carrier board for redundancy). Not a firmware defect — accepted gap for Core/Titan tiers.

### IVP-131 Replay Harness — Deferred to Grok (2026-04-15)

Profile generator `scripts/generate_replay_profiles.py` produces 5 sensor-rate CSVs. Sim now uses real F15-6 NAR thrust curve (ThrustCurve.org RASP data), correct Estes dry mass (70g, not .rkt's 160g), Cd=0.50. Output: **347m apogee at t=7.6s, chute at 13.6s, landed 83.4s.** Physically reasonable.

**Council improvements incorporated:** progressive thrust curve, chute deployment, early_burnout (not CATO), physical phase labels.

**Still needed for Grok:** firmware-side replay inject hook (`_replay_inject` CLI + Core 1 seqlock injection), `scripts/replay_harness.py` orchestration (CSV→serial→verify FD states), plot script (Space Camp Counselor suggestion), intermittent IMU fault variant (NASA/JPL suggestion).

**Files in repo:** `scripts/generate_replay_profiles.py`, `tests/replay_profiles/estes_big_daddy.rkt`, 5 CSV profiles (f15_nominal, early_burnout, imu_zero_fault, baro_dropout, gps_dropout_descent).

### ELRS on RP2350 — Research Item (2026-04-12)

ExpressLRS is fully open source ([github.com/ExpressLRS](https://github.com/ExpressLRS/ExpressLRS)). The 900MHz version uses SX1276 with custom frequency hopping and tight packet timing. Currently ELRS runs on ESP8285/STM32 paired with SX chips. Running ELRS natively on RP2350 with PIO-assisted modulation timing is a research question worth exploring — the RP2350 has plenty of headroom and PIO could handle the frequency hopping timing constraints.

**Not blocking any current work.** This is a future investigation item for potential radio protocol upgrade. The current RFM95W hardware (bare SX1276 on SPI) may be compatible if the ELRS packet format and hopping schedule can be implemented in RP2350 firmware. The Telstar Booster Pack (`docs/hardware/TELSTAR_BOOSTER_PACK.md`) already describes ELRS via CRSF/UART to a dedicated module as an alternative path.

### Landing Beacon LED — White Flash 2Hz (user request, 2026-04-15)

LANDED state LED pattern should be white flashing at ~2Hz. White is brightest/most visible for recovery. 2Hz is attention-getting without being excessive. Update the LANDED pattern in AO_LedEngine (color + period).

### SPIN Model Inaccuracy — Abort Pyro (discovered 2026-04-12)

`rocketchip_fd.pml` lines 89-92 and 105-108 unconditionally fire `drogue_fired = true` on ABORT from BOOST/COAST. The firmware gates this on `MissionProfile::abort_fires_drogue_from_boost/coast`, which default to `false` in `rocket.cfg`. The SPIN model is more aggressive than the firmware. Firing drogue at high speed (during BOOST) would be a shred event — the profile flag being `false` is physically correct for rockets. The SPIN model should be updated to gate drogue-on-abort behind a boolean. The code comment at `flight_director.cpp:317` ("fires drogue — Amendment #1") should say "fires drogue if profile flag is set."

**Action:** Update SPIN model + code comment when Stage P7 IVP-120/121 work touches the FD. Not urgent — firmware behavior is correct.

### QP Posted Events Must Be Static (LL Entry 35)

Discovered during IVP-117 HW verify: `AO_LedEngine_post_pattern()` had a latent use-after-free bug from Stage 7 (IVP-77). Declaring `QEvt` subclasses as stack-local before `QACTIVE_POST` is broken — QP stores the pointer, NOT a copy. When the caller returns, the event memory gets overwritten. Manifests as `Q_onError(qf_dyn, 750)` assertion.

Fixed in both `AO_LedEngine_post_pattern()` and `AO_Notify_post_cal_intent()` by switching to static event storage. Other existing posts (HealthMonitor, FD phase/pyro, Radio RX) were already correct. If any future AO adds a new post site, verify the event is static — see LL Entry 35 for the detection pattern via GDB.


### Launch Procedure Audit Findings — Future Safety Items

Items identified by comparing NASA/SpaceX/NAR launch procedures against RocketChip FD logic. All are future work requiring Mission Profile or hardware support.

1. **Angle-rate abort guard** — If vehicle exceeds bank angle threshold during BOOST (e.g., 2/3 engines lit, off-axis thrust), trigger ABORT. Real analog: range safety corridor violation / AFTS auto-destruct. Critical for multi-engine and air-dropped configurations. Needs IMU attitude estimate in BOOST phase.

2. **No-pyro-after-impact guard** — If stationary/impact detected while in BOOST or COAST (lawn dart), do NOT fire pyro charges. A rocket that hit the ground and then fires an ejection charge is a hazard. Guard: if landing guard fires before apogee guard, suppress pyro.

3. **Hung fire / ignition timeout** — Track time from ARM (or igniter command from station) to launch detection. If no launch within expected motor ignition window, throw HUNG_FIRE warning. NAR: wait 60s before approach after misfire. Station could enforce exclusion timer.

4. **Igniter continuity check** — Station-side check of igniter circuit continuity before arming. Standard practice in all commercial launch controllers and HPR altimeters. Could be built into station ↔ vehicle radio command path.

5. **Air-dropped vehicle profile** — Different abort timing: no pad, no "stay on ground" option. Engine failure after drop = glider or ballistic. Needs Mission Profile variant with altitude-aware abort logic.

6. **Multi-engine / staging support** — Partial engine light detection, inter-stage hold points, TRA Rule 13-9 (staging devices armed after recovery). Future Mission Profile work.

### MAIN_DESCENT Has No Timeout Fallback — SPIN P7 Fails

MAIN_DESCENT relies entirely on the stationary guard (`SIG_LANDING`) to exit. If ESKF is dead (no velocity estimate), the guard never fires and the system stays in MAIN_DESCENT forever. Also affects HAB profile (balloon float, no descent). Every other flight phase has a timeout fallback; this one doesn't.

**Fix:** Add `descent_timeout_ms` to `MissionProfile` (e.g., 600s rocket, longer for HAB). Add fallback in `state_main_descent` SIG_TICK handler: if elapsed > timeout → auto-LANDED. Update SPIN model with the timeout path — P7 liveness will then pass with weak fairness.

**SPIN status:** P7 (`p_liveness_flight_completes`) currently fails — this is the root cause, not a modeling limitation.

### CMake Source List Cleanup Needed

`CMakeLists.txt` lists `src/cli/rc_os.cpp` twice (once in the Stage 2 section at line 287, once in Stage 9 at line 416). CMake deduplicates silently so no build error, but it's sloppy. Same may apply to other source files. Should do a full dedup pass — not blocking any work, just housekeeping.

### Vehicle Board Running Warm During Long Soak (2026-04-16)

During IVP-132 Phase 4 (30-min battery soak on vehicle flight binary),
user noticed the Feather RP2350 HSTX was noticeably warm to the touch
after the soak (not hot, not concerning — but above ambient).

**Next long-duration soak should capture MCU die temperature.** The
RP2350 has an internal temperature sensor on ADC input 4 (per RP2350
datasheet §12.4.6). Access pattern:

```cpp
adc_init();
adc_set_temp_sensor_enabled(true);
adc_select_input(4);
uint16_t raw = adc_read();
// Conversion per SDK: T_c = 27.0f - (raw * 3.3f / 4096 - 0.706f) / 0.001721f
```

Add to `shared_sensor_data_t` (seqlock): `float mcu_temp_c`. Core 1
sensor loop reads it at ~1 Hz (slow-changing signal). Extend
`diag_stats_dump()` and the soak GDB scripts to include it.

**Why this matters:** Thermal drift could affect ESKF baseline noise,
XIP cache behavior, and crystal frequency (PPM drift). A warm baseline
at idle might not be concerning, but the data is free and catches
thermal runaway scenarios (e.g., stuck-on regulator, PIO thrashing).

**Not blocking IVP-132a.** Log this for the next full-system soak
(likely IVP-132a.4 station soak or the Stage 17 ground test).

### Station Role vs. Fruit Jam Board — Decoupling Audit (2026-04-16)

User flagged a latent coupling concern during IVP-132a work: "station mode"
(RX role, ground station behavior — `ROCKETCHIP_JOB_STATION=1`, `kRadioModeRx`)
must NOT be conflated with "Fruit Jam board" (Adafruit RP2350B-based SBC).
Station role should be runnable on any RP2350 variant — future Tiny 2350 port,
custom carrier boards, or even the flight Feather repurposed as a relay/GCS.

**Currently known conflations to audit:**
- `build_station/` cmake invocation may hardcode `PICO_BOARD=adafruit_fruit_jam`
  via environment. Board selection should be orthogonal to role selection.
- `rc_os_commands.cpp` station-specific CLI sometimes refers to "Fruit Jam"
  in print strings (`grep -n "Fruit Jam\|fruitjam" src/`).
- Display output assumes Fruit Jam's HSTX/DVI output. If a station-role
  device has no display, output should fall back to USB/serial gracefully.
- Pin allocations in `board_*.h` — station-role needs radio SPI and GPS UART
  wired, but other pins (DVI, SD card) are Fruit-Jam-only and should be
  optional.

**Action:** Post-IVP-132a audit task. Not blocking current work — IVP-132a
is validating station *behavior*, and it happens to run on Fruit Jam. The
decoupling cleanup is a separate pass where we grep for board-specific
references in station code paths and either gate with `#ifdef PICO_BOARD_*`
or move board-specifics to `board_*.h`.

**Related future work:** Tiny 2350 port (user mentioned upcoming) will
exercise the decoupling — same vehicle firmware, smaller board, no PSRAM.

### Protected File Updates Pending Approval

*None currently.*

---

## Upcoming Stages

**Stage 13: Health Monitor** — COMPLETE. AO_HealthMonitor, 2-bit encoding, fault patterns, preflight CLI, debug sub-menu, auto-DISARM, pre-launch latch. SPIN 11/11.

**Stage 14: Notification Engine** — COMPLETE. IVP-113 through IVP-118. AO_Notify intent layer, output backends, caller rewiring, sensor + Core1 vitality migration, static-event bug fix (LL Entry 35). Council-reviewed (NASA/JPL, ArduPilot, Professor, Rocketeer). 30 new host tests. HW verified via GDB memory inspection. Plan: `.claude/plans/encapsulated-pondering-sparkle.md`.

**Stage 15: Pre-Flight Polish** (was 14) — includes:
- **AO Responsibility Audit** — verify health/safety logic in correct AOs (Stage 13 Core1 gap)
- **Audio Output (I2S DAC)** — TLV320DAC3100, pico-extras, AP tone parser, ~10-12 IVPs. Fills Stage 14 audio backend stub.
- **User Guide** — operational procedures, CLI flowchart
- **Runtime Behavior Map** — update for AO architecture
- **Defense-in-depth evaluation** — Core1 stall checked in 3 places post-Stage-14. Evaluate justified vs. bloat.

**Stage 16: Field Tuning** (was 15) — All VALIDATE parameters. Needs flight data.

**Stage 16C (planned, post-Stage-17): Station Re-Work** — dedicated rebuild of the
station-role firmware. Surfaced during IVP-132a bench testing (2026-04-16).

Scope (must all land together — they're the same underlying problem):

1. **Station/vehicle runtime decoupling.** Today "station profile" is implemented
   as "vehicle profile with Core 1 idle" — Core 1 `core1_entry()` waits forever
   on `g_startSensorPhase` when `kRole != kVehicle`. This means any periodic
   work (GPS polling, MCU temp, battery ADC, future sensors) is vehicle-only
   by construction. Station GPS is initialized but never read. MCU die temp
   can't be added without a Core 0 path. See "Station Runtime Shape Is
   Fundamentally Different from Vehicle" flag below.

2. **Station role / board decoupling.** Today "station mode" is entangled with
   Adafruit Fruit Jam board specifics (hardcoded `PICO_BOARD`, HSTX/DVI
   display assumptions, Fruit Jam print strings). Station role should run on
   any RP2350 variant — flight Feather as relay, future Tiny 2350 port,
   custom carrier. See "Station Role vs. Fruit Jam Board — Decoupling Audit"
   flag below.

3. **Station Core 1 sensor path.** Give station its own minimal Core 1 loop
   (or Core 0 tick cycle) for its own periodic sensors: GPS update, MCU temp,
   eventual battery ADC. Keep seqlock layout shared — station populates
   fewer fields.

4. **MCU die temperature capture** (deferred from IVP-132a). Add to both
   roles once the sensor path exists. User noted 2026-04-16 that the vehicle
   Feather ran warm during the 30-min battery soak; worth monitoring long
   term. See "Vehicle Board Running Warm" flag below.

5. **Tiny 2350 port** (user mentioned upcoming) — validates the decoupling
   work. Different RP2350 variant, no PSRAM, same role capabilities.

Council review required before starting — this is an architectural shape
change, not a refactor.

**Blocking:** None for Stage 17 field testing (vehicle-side is complete).
Stage 16C can run in parallel or after Stage 17 flight data informs tuning.

See plan file for full breakdown.

### Deferred (near-term, post-Stage 15)

- **Battery ADC Monitoring** — Hardware not wired. ADC pin + driver + telemetry field.
- **CCSDS SDLS Command Authentication** — Telecommand auth for Rocket profile.
- ~~IVP-103 Station GPS Push~~ — **RESOLVED.** `cmd_station_gps_push()` implemented since Stage 7 Take 2 at `rc_os_commands.cpp:1218`. Bound to `p` key in station dispatcher.

### Far-future (moved to PROJECT_STATUS)

Mission Profile OTA, F' Evaluation, u-blox GPS, OTA drivers, GPS-free 3D reconstruction, FSK bitstream, MATLAB export — all moved to `docs/PROJECT_STATUS.md` future features.

### Recorded Elsewhere (Removed from Whiteboard)

These completed/resolved items are preserved in their canonical locations:

| Item | Recorded In |
|------|-------------|
| All Stage 1-12A completion details | `docs/PROJECT_STATUS.md` completed table |
| Stage 7 IVP-57-65 details | CHANGELOG 2026-04-07/08, PROJECT_STATUS |
| AO/State Engine Logging audit | `docs/ADVANCED_SETTINGS.md` (verbose/MP-configurable logging) |
| PCM frame expansion | `docs/ADVANCED_SETTINGS.md` Research Mode |
| DPS310 baro rate optimization | CHANGELOG, baro driver code |
| DPS310 baro read count fix | CHANGELOG, baro driver code |
| SRAM execution audit | `docs/benchmarks/` |
| Dense FPFT benchmark | `docs/benchmarks/`, whiteboard resolved (archived) |
| UD factorization benchmark | `docs/benchmarks/UD_BENCHMARK_RESULTS.md` |
| Bierman measurement update | CHANGELOG, ESKF code |
| MMAE/IMM pivot | `docs/decisions/ESKF/ESKF_RESEARCH_SUMMARY.md` |
| 24-state ESKF expansion | CHANGELOG, PROJECT_STATUS |
| Codegen FPFT (IVP-47) | CHANGELOG, PROJECT_STATUS |
| BSS/codegen sensitivity disproved | `LESSONS_LEARNED.md` Entry 27 |
| All strikethrough DONE items | CHANGELOG entries, PROJECT_STATUS |
| VALIDATE values inventory | `docs/UNIQUE_COMMENT_ITEMS.md` |
| Power optimization notes | Codegen is mandatory (benchmarked). Revisit if state count > 24 |
| clang-tidy status | Full audit clean. lizard CCN in tiered audit. `standards/STANDARDS_AUDIT_2026-03-26.md` |
| Job/Mission naming | Code uses `job.h` namespace. Settled. |
| ivp62-wip branch | Deleted 2026-04-09. All content ported to main. |

---

## Resolved

*Cleared 2026-04-09. All resolved items recorded in PROJECT_STATUS.md, CHANGELOG.md, and LESSONS_LEARNED.md.*
