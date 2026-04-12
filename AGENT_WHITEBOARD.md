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

### BENCH SIM RETIREMENT — RESOLVED (2026-04-12)

Old `bench_flight_sim.py` (479 lines, IVP-73) retired and replaced by `bench_sim.py` (~200 lines, 2 tests) in commit `5fbea19`. IVP-119 FusedState rename committed (`010f305`). Stage P7 + Stage 15 un-shelved. See LL Entry 36 for full root-cause analysis. Pre-commit hook now gates `ctest` + needs-based HW bench sim on flight-critical commits. Session-start canary added to SESSION_CHECKLIST.md item 6.

### ⚠️ RadioScheduler Sync — IVP-122 Command Delivery Unreliable (2026-04-12)

IVP-122 half-duplex ACK protocol is implemented and works when timing aligns (ARM ACK'd on first try during HW test), but command delivery is unreliable. DISARM failed 3 retries in the same test session. Root cause: the station sends commands at arbitrary times, but the vehicle only listens in brief RX windows between TX slots (~500ms cycle at 2Hz). The station needs to synchronize its TX to the vehicle's RX window.

**Research findings (LoRaWAN Class A, SiK TDMA, Altus Metrum poll-response):** The standard pattern is "listen-before-talk" — station sends a command immediately after receiving a vehicle packet, when the vehicle's RX window is guaranteed open. The SX1276 mode switch (TX→RX) takes ~100µs; the station's SPI FIFO write + TX start takes ~300-500µs. At SF7/BW125 this is well within the preamble detection window.

**Proposed fix:** In the station's `handle_rx_packet()`, when a vehicle nav packet is received AND a pending command exists, post `SIG_RADIO_TX` immediately (same tick). This uses the vehicle's natural post-TX RX window without new timing machinery. Requires a new IVP — this is a RadioScheduler architecture change, not an IVP-122 patch.

**Scope:** Affects all bidirectional communication (ACK, station GPS push, future config upload). Should be a dedicated RadioScheduler sync IVP before IVP-122 can be marked fully resolved.

### ELRS on RP2350 — Research Item (2026-04-12)

ExpressLRS is fully open source ([github.com/ExpressLRS](https://github.com/ExpressLRS/ExpressLRS)). The 900MHz version uses SX1276 with custom frequency hopping and tight packet timing. Currently ELRS runs on ESP8285/STM32 paired with SX chips. Running ELRS natively on RP2350 with PIO-assisted modulation timing is a research question worth exploring — the RP2350 has plenty of headroom and PIO could handle the frequency hopping timing constraints.

**Not blocking any current work.** This is a future investigation item for potential radio protocol upgrade. The current RFM95W hardware (bare SX1276 on SPI) may be compatible if the ELRS packet format and hopping schedule can be implemented in RP2350 firmware. The Telstar Booster Pack (`docs/hardware/TELSTAR_BOOSTER_PACK.md`) already describes ELRS via CRSF/UART to a dedicated module as an alternative path.

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

See plan file for full breakdown.

### Deferred (near-term, post-Stage 15)

- **Battery ADC Monitoring** — Hardware not wired. ADC pin + driver + telemetry field.
- **CCSDS SDLS Command Authentication** — Telecommand auth for Rocket profile.
- **IVP-103 Station GPS Push** — Needs radio command path.

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
