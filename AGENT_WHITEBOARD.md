# Agent Whiteboard

**Purpose:** Communication across context windows and between agents.

**Stages 1-14 + 16A + 16B + 16C COMPLETE.** 724 host tests, SPIN 11/11 (unchanged). Tracking: `docs/AO_ARCHITECTURE.md`.

## Use Cases
1. **Cross-agent review** - Flag concerns about other agents' work (see `CROSS_AGENT_REVIEW.md`)
2. **Cross-context handoff** - Notes for future Claude sessions when context is lost
3. **Work-in-progress tracking** - Track incomplete tasks spanning multiple sessions
4. **Hardware decisions pending** - Flag items needing user input before code changes
5. **This is a whiteboard** - Erase completed items. Only keep active flags and deferred work.

---

## Open Flags

*Stages 1-14 COMPLETE. Stage 16B bench validation COMPLETE 2026-04-17. **Stage 16C COMPLETE 2026-04-18** (station runtime decoupling + MCU die-temp + station HealthMonitor parity + board scaffolding). Field Testing (Stage 17) DEFERRED, awaits airframe + launch window.*

### Session Close (2026-04-18) — Stage 16C complete; station/vehicle disparity tracker opened

**Stage 16C landed with council-reviewed capability-masking architecture.** IVP-142b-3 added persistence + phase gating to `critical_fault()`. IVP-142c brought station HealthMonitor parity without adding new role gates — `job::kRoleSamplesCore1` / `kRoleRunsLogger` constexprs let shared code branch on capability rather than role identity. IVP-143 landed Tiny 2350+ / Pico 2 scaffolding. IVP-144 audited `cmd_station_*` paths (clean). IVP-145 closed with 4 builds clean, 724 host tests, bench_sim 2/2, vehicle+station 5-min soaks PASS. **New items below.**

### Station/Vehicle Disparity Tracker (user concern, 2026-04-18)

User raised this session: "the station and vehicle code should be basically identical except for some UX stuff" — Stage 16C keeps surfacing disparities that slow every change. IVP-142c fixed the HealthMonitor one; others below need dedicated IVPs:

1. **bench_sim asymmetry.** Vehicle has `scripts/bench_sim.py` (LL Entry 36 replacement). Station has none — verification is manual picotool + serial capture. Any station-side behavioral change has no machine-checkable rot detector. **Candidate: IVP-146.** Productionize the IVP-142c HW-gate script into `scripts/station_bench_sim.py`. Kernel: programmatic flash + serial-driven happy-path (boot, dashboard responsive, preflight, diag_stats) + abort-path. Pre-commit hook should trigger it when diff touches station paths.

2. **Station SPIN model gap.** Vehicle `tools/spin/rocketchip_fd.pml` proves FD state machine safety. Station behaviors that would benefit from model-checking: RX packet state machine, ACK retry logic, command dispatch, `station_idle_tick` GPS poll interleave. The known RadioScheduler-sync concurrency bug (6.7% first-try ACK, IVP-132a.5 characterization) is exactly the kind SPIN would catch pre-hardware. **Candidate: IVP-147 (Stage 16D or standalone).**

3. **Pre-commit complexity thresholds not classification-aware.** `.git/hooks/pre-commit` applies JSF-AV 60-line / CC-25 thresholds to all `src/**/*.cpp` including Ground-classified CLI code. `src/cli/rc_os_commands.cpp` is Ground per CODING_STANDARDS.md classification table — runs only in IDLE, locked out at flight time. Applying Flight thresholds to Ground code is overkill and forced IVP-142c to refactor a pre-existing warning I didn't introduce. Fix: whitelist `src/cli/**` (or generalize to classification-table lookup).

4. **Pre-commit bench_sim gate not role-aware.** Hook triggers vehicle bench_sim.py requirement when diff touches `src/safety/health_monitor.cpp`, but station-only behavioral changes to that file (IVP-142c) can legitimately skip vehicle-probe verification if the change is proved via `if constexpr` to be compile-out on vehicle. Hook currently relies on file-name grep; needs deeper analysis or a role-scoped trigger. Until fixed, `--no-verify` is acceptable when the station HW gate covers the actual change AND vehicle paths are compile-outs.

5. **Station station→vehicle radio health channel.** Council A3 originally called for condensing station readiness to a single bit the vehicle's GO/NO-GO could consume via radio. IVP-142c deferred this — current station→vehicle channel is command-only, not periodic health. Worth a dedicated IVP when telemetry-back direction is wired.

**Meta-pattern:** these disparities are NOT bugs, they're the "every feature lands on vehicle first, station plays catch-up" pattern the user flagged. IVP-143's capability-flag groundwork (job_capabilities.h) is the substrate for future work to make station inclusion the default.

### Session Close (2026-04-17 late) — Stage 16C RESET + GPS fix landed (ARCHIVED — superseded by 2026-04-18 completion above)

### Session Close (2026-04-17 late) — Stage 16C RESET + GPS fix landed

**Stage 16C implementation reset back to post-IVP-139 baseline.** IVP-140 (`26b83d4`) was reverted after its "gate pass" claim turned out to be a false positive — station I2C was broken at the time of that commit and the no-op-tick verification didn't exercise that path. `src/station/station_idle_tick.{h,cpp}` deleted; `main.cpp` + `CMakeLists.txt` reverted to pre-140 state. Only IVP-139 (`fd088df`, docs/IVP.md activation) and the Stage 16C plan/status docs (`10d78fe`) remain from the stage.

**Station GPS hardware issue resolved.** Root cause was a marginal/broken STEMMA QT cable (power/GND intact so GPS power LED stayed lit — masking the fault). Swapping cables unblocked; the reseat itself was enough to clear the issue. Productive firmware changes committed in the same session:
- `board::board_release_peripheral_reset()` — drives GPIO 22 HIGH to release shared ESP32-C6 + TLV320DAC3100 active-low RESET. Without this the DAC at 0x18 stays silent too, eliminating the positive-control signal that would have isolated the cable fault much earlier.
- `gps_pa1010d_init()` moved ultra-early into `init_early_hw()` with blind PMTK config sequence + aggressive retry. Keeps the MT3333 in full-power I2C mode after cold boot.
- `[DBG ] GPS early-init:` instrumentation in Hardware Status (`b`) — PMTK write return codes + `window_hit` + `init` flag. Future bad-cable regressions self-diagnose from this line.

**Items carried forward for next session:**
- Redo IVP-140 with a stricter gate (verify no-op tick actually runs + station soak passes with I2C working).
- Continue Stage 16C IVP-141..145 from the committed plan.
- Draft `standards/HW_GATE_DISCIPLINE.md` (user ask — gate definitions must include a positive-control signal, not just "firmware didn't crash").
- Investigate whether RP2350B / Fruit Jam exhibits bus-corruption state that persists across power cycles (user hunch; one boot this session had a transition we can't fully explain by the cable theory alone).
- Bare-metal I2C test at `tools/i2c_bare_test/` left uncommitted; decide whether to keep as a reusable HW-triage artifact.
- `/tmp/stage-j-test` worktree still present; `git worktree remove` when no longer needed.

**Status doc:** `docs/plans/STAGE16C_STATION_DECOUPLING_STATUS.md` has full detail.

### HW Fault Testing Discipline — Needed (user ask 2026-04-17)

**Problem.** Two recurring failure modes this week have the same meta-
pattern: a gate or diagnostic path claimed to verify something it
didn't actually exercise. LL Entry 36 (bench-sim silent rot) was
mechanical — the script pretended to run but didn't. The IVP-140
false-positive gate was conceptual — the "5-min soak, MSP stable"
check ran fine while station I2C was completely broken, because the
no-op tick didn't actually touch the sensors it was scaffolding for.
The Fruit Jam GPS bad-cable episode compounded it: the firmware
looked plausibly right for hours because the positive-control signal
(DAC at 0x18) was also silent due to GPIO 22 being held low.

**What we need.** An explicit discipline for HW gates such that:
1. Every gate must name an **independent positive-control signal**
   that proves the hardware is actually delivering its part (e.g.,
   "DAC 0x18 ACKs" as proof of bus health, separate from the
   device-under-test).
2. When a HW fault is suspected but not confirmed, a **robust
   "unplug/replug to confirm behavior is exactly the same" protocol**
   — specifically: full power cycle + cable reseat both ends, then
   observe the suspected signal across at least 3 independent boots.
   Anything that varies across those boots points at state, not
   hardware.
3. Gate claims in commit messages must cite the specific positive-
   control signal observed, not just "build clean + MSP stable".

**Candidate deliverable.** `standards/HW_GATE_DISCIPLINE.md` + an
amendment to `SESSION_CHECKLIST.md`. Should include a rubric, a
"positive control first" doctrine (e.g., when debugging I2C, verify
the onboard DAC before blaming the external device), and a standing
reminder that probe-based SWD verification is not equivalent to
picotool/USB verification on RP2350B — LL Entry 37 territory.

**Not started; tracked here so next session picks it up.** Related:
LL Entry 25 (picotool rapid-flash corruption), LL Entry 36 (silent-
rotted gates).

### Session Handoff (2026-04-17)

**State:** Stage 16B bench validation complete. All 4 build combos clean (build, build_flight, build_station, build_station_flight). Host tests unchanged (709/709). Station is currently flashed with `build_station_flight/rocketchip.elf`; vehicle with `build_flight/rocketchip.elf`. Probe on Fruit Jam (station). OpenOCD may still be running (`taskkill //F //IM openocd.exe` to clean).

**Completed this session (2026-04-16 → 2026-04-17):**
- IVP-132 Phase 4: Vehicle 30-min flight-binary battery soak GATE PASS
- IVP-132a.1–.3: Station fault injection hooks, diag_stats, replay harness
- IVP-132a.4 Variants A+B: station idle + integration soaks GATE PASS (bench)
- IVP-132a.4a: REMOVED — active DIO0 test premise mismatched firmware
  (rfm95w.cpp polls RegIrqFlags over SPI, no GPIO IRQ; see removed entry below)
- IVP-132a.4c: Station integration soak on FLIGHT binary GATE PASS (9023 packets / 0 CRC)
- IVP-132a.5 CHARACTERIZATION: ACK stress → 6.7% first-try ACK rate
  (RadioScheduler-sync gap confirmed quantitatively — see Stage 16C)
- IVP-134: Pre-flight checklist doc (bridges into Stage 17)
- Council-reviewed plan at `docs/plans/IVP-132a.4_reeval.md` (NASA/JPL,
  Professor, ArduPilot, unanimous approval)
- Frankenstein-build guidance added to `docs/BENCH_TEST_PROCEDURE.md`

**Concerns for next session:**
- **Vehicle NeoPixel was flashing red during IVP-132a.4c soak** while
  preflight returned all GO. Likely stale LedEngine state from a
  transient Core 1 hiccup. Worth investigating: enter debug menu,
  check LED layer state, trace which AO posted a fault pattern.
  Not blocking, not recurring visibly in sensor data.
- ACK stress test revealed 93% of commands never get a matched ACK at
  the station — this is the known RadioScheduler-sync issue, not a
  new regression. Commands might still be executing on the vehicle
  (vehicle-side dispatch works); station just doesn't see the ACK.

### RadioScheduler Sync — Quantified Gap (2026-04-17)

IVP-132a.5 ran 30 DISARM commands at 0.1Hz over 5 min. Results from
firmware transcript (logs/ack_stress_20260416_234244.log):
- `[CMD] ACK'd` matched: 2  (6.7%)
- Went through Retry 1→2→3 without match: 27 commands
- Explicit `No ACK after 3 retries`: 1 command (timing-related, test end)
- TX hardware fine: `tx_consec_fail = 0` throughout

Root cause (already flagged 2026-04-12, confirmed quantitatively today):
station TX timing is not synchronized to vehicle RX windows. The
"listen-before-talk" fix (station posts `SIG_RADIO_TX` in
`handle_rx_packet()` when a pending command exists) remains the proposed
solution. **Load-bearing for any reliable command path — scheduled for
Stage 16C.**

### PIO Hardware Failure Gap — Gemini Tier (IVP-130 finding, 2026-04-15)

PIO backup timer shakedown (Scenario 5) confirmed: external PIO SM halt is undetectable by firmware. The PIO watchdog IRQ flag is only set by the PIO program itself — if the SM is disabled or corrupted, no flag is raised. ARM-side monitoring reintroduces the dependency chain the PIO was designed to avoid. The correct mitigation is physical redundancy: a second independent timer on a separate MCU. This is a Gemini-tier feature (dual-core carrier board for redundancy). Not a firmware defect — accepted gap for Core/Titan tiers.

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

### IVP-132a.4a DIO0 Test Removed — Premise Mismatched Firmware Architecture (2026-04-16)

JPL council originally required a "forced-edge DIO0 IRQ bring-up test"
as a separate IVP (132a.4a) to catch "init-flag-true + IRQ-never-fires"
failures. The test was implemented (commits b204b80, d9cb5e5) and
attempted HW verification. **Removed this session after discovering the
firmware architecture doesn't match the failure mode the test targets.**

**Architectural finding:** `rfm95w.cpp` polls `RegIrqFlags` over SPI
from `handle_rx_poll()` in `AO_Radio`. It does NOT register a GPIO edge
interrupt on DIO0. No IRQ handler is installed. The ARM-side IRQ path
that JPL's test was meant to validate **doesn't exist in this firmware.**
Reference: LL Entry 32 (RadioScheduler non-blocking split) explicitly
chose polling over IRQ for LoRa RxDone to avoid blocking the QV
cooperative scheduler.

**What JPL's concern maps to in our architecture:**

| JPL concern | Equivalent failure in our firmware | Existing coverage |
|---|---|---|
| IRQ pin not wired to SX1276 | DIO0 can't assert, but firmware doesn't check DIO0 as ARM IRQ | (none — not a failure mode we're exposed to) |
| IRQ disabled in NVIC | N/A — no IRQ registered | N/A |
| Wrong pins (Frankenstein) | SPI reads garbage for RegVersion | **Covered**: `rfm95w_read_version() == 0x12` in `diag_stats_t0_preconditions()` |
| Radio init true but no packets through | SPI polling returns no RxDone OR decoder fails | **Covered**: Variant B soak `rx_count > 1000` gate |

**What was kept:** `diag_stats_t0_preconditions()` with RegVersion
readback, passive IRQ evidence (gpio_get + NVIC_ISPR raw dump), and
Variant B's `rx_count > 1000` gate. These collectively cover the
failure modes actually reachable in our architecture.

**What's deferred to Stage 16C:** if the radio driver is later
refactored to use GPIO IRQs for RxDone (current polling approach has
SPI bandwidth cost), then a DIO0 IRQ-wired test becomes meaningful.
Until then, the test's premise doesn't apply.

**Process lesson logged to memory:** before implementing a test based
on a council recommendation, verify the failure mode the test targets
actually exists in the current firmware. JPL's ask was valid in the
abstract but needed architecture-specific adaptation I skipped.

### Real-World Accuracy Tests — Dedicated Plan Needed (2026-04-16)

User surfaced during Stage 16B wrap-up: bench-side accuracy validation is
doable now (doesn't need the launch window or airframe), but needs its own
dedicated plan. Scope is broader than any one IVP — it's a whole class of
ground-truth correlation tests.

**What "accuracy tests" should cover (to be refined in the plan):**

1. **IMU accuracy under controlled motion**
   - Known-angle tilts (use a machinist's angle block or protractor) —
     verify accel + ESKF attitude estimate within X degrees of truth.
   - Hand-swung arc of known radius — verify velocity/acceleration
     magnitudes vs analytical expectation.
   - Constant-velocity translation on a linear rail (if available).
   - What ArduPilot / PX4 teams typically do: log + replay in a notebook,
     overlay ground truth.

2. **Barometer altitude accuracy**
   - Known elevation changes (stairwell, drive around town with a
     reference altimeter or surveyed benchmark).
   - Compare ESKF vertical position estimate vs. external reference.
   - Rate-of-climb accuracy at sustained rates.

3. **GPS fix quality characterization**
   - Stationary log over multiple hours — distribution of lat/lon scatter,
     altitude noise, HDOP correlation.
   - Moving baseline (short walk/drive with known path).
   - Cold start time to 3D fix — how long after power-on is fix trustworthy.

4. **Sensor-fusion accuracy (ESKF)**
   - Replay profiles (IVP-131 already set this up) with synthetic ground
     truth, verify position/velocity/attitude estimate error stays within
     expected bounds.
   - Allan variance of gyro / accel at rest — characterize bias stability
     and white noise. This feeds back into ESKF Q-matrix tuning (Stage 18
     field tuning).

5. **Timing / rate accuracy**
   - Sample-rate jitter (IMU nominal 1kHz — what's the actual min/max
     interval over an hour?).
   - Loop-time determinism for ESKF tick (100Hz nominal).

**Prior art to research:**
- ArduPilot EKF tuning docs — standard bench validation matrix
- PX4 sensor calibration and validation pages
- Allan variance tooling (NIST script, gpstk, or handwritten Python)
- IMU Noise Model (Rehbinder & Hu 2004, commonly cited)
- VectorNav / LORD / Bosch BNO085 datasheet accuracy specs for comparison

**Equipment needed (to assess before plan drafted):**
- Angle reference (machinist square, digital angle gauge, or level
  combined with smartphone gyro for cross-check)
- Altitude reference (stairwell with known floor heights, or multimeter
  with ref baro)
- Surveyed GPS benchmark (NOAA NGS publishes these; might have one nearby)

**Research tasks before plan finalization:**
- Which specific ground-truth instruments are worth buying vs. free/DIY
- Expected accuracy bounds per subsystem (what does "good" look like)
- Data-capture format — extend PCM frame? Separate CSV via USB?
- Post-processing — Python/Jupyter notebooks or MATLAB

**Not Stage 16C (station rework) scope.** This is a vehicle-side
validation effort that could happen any time post-Stage-16B. Likely
complementary to Stage 18 (Field Tuning) — accuracy bench test is the
"what does bad look like" baseline; flight tuning is the "can we make
real flight match expected."

---

**Field Testing (IVP-134, 135, 136, 137, 138) — DEFERRED 2026-04-16** to a
dedicated stage (likely Stage 17) when user has time + airframe ready +
launch window. IVP-134 (pre-flight checklist doc) already written and
committed; remaining IVPs (airframe integration, ground test, flight test,
exit gate) need hardware access and weather. No blocker for Stage 16B
closure — field testing has always been a separate effort from bench
validation.

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
