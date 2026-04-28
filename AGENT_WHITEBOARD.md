# Agent Whiteboard

**Purpose:** Cross-context / cross-agent communication channel for **active work only**.

> **Treat this like an IRL whiteboard — not a record of completed things.**
> When an item is done, **erase the row**. Don't add a "Resolved" section,
> don't strike it through, don't leave a "closed" marker. The CHANGELOG is
> the project's permanent record of what was done; this whiteboard's only
> job is to surface what's *still active*. A row's continued presence is
> the signal that it still needs attention. Stale "done" notes dilute that
> signal and bury the rows that actually matter.
>
> Before adding a row: check whether the work is already done elsewhere
> (CHANGELOG, git log, the relevant doc). Before acting on a row: spot-
> check it's still real — agent memory of "this needs doing" is exactly
> the failure mode that drives stale-row accumulation. If you reject an
> item after consideration, log the rejection rationale in CHANGELOG and
> erase the row, don't move it to a "rejected" section.

## Project status (one-line snapshot)

**Stages 1-14 + 16A + 16B + 16C + L + T COMPLETE.** **788** host `ctest` entries (786 C++ discovered + **2** Python `scripts/` gates), SPIN 11/11. Tracking: `docs/AO_ARCHITECTURE.md`. **Stage 17 (Field Testing & Avionics Airworthiness) restructured 2026-04-22** from 5-IVP direct-to-flight into 13-IVP tapered buildup (three council rounds, approved with amendments). First motor flight = step 13 of 14. Plan: `docs/plans/STAGE17_TAPERED_BUILDUP.md`. Execution awaits future session; starts with IVP-135a (pure-software log schema extension). **CCSDS TC-Layer + COP-1 rework deferred to post-Stage-17** (unanimous council) — field data will inform scoping.

## Use Cases
1. **Cross-agent review** — Flag concerns about other agents' work (see `CROSS_AGENT_REVIEW.md`)
2. **Cross-context handoff** — Notes for future Claude sessions when context is lost
3. **Work-in-progress tracking** — Track incomplete tasks spanning multiple sessions
4. **Hardware decisions pending** — Flag items needing user input before code changes
5. **Deferred items** — Active intent kept visible until acted on (and then erased — see header rule)

---

## High priority

- **IVP-T13 LQ-adaptive retry — deferred until after the CCSDS command-
  layer rework.** Original Stage T Batch C plan was to port the ELRS
  LQCALC pattern (retry aggressiveness scales U-shape with LQ: fewer
  retries when LQ is high, more when marginal, pause when LQ is near-
  dead) behind `ROCKETCHIP_LQ_ADAPTIVE_RETRY`, default OFF. Decision
  2026-04-22: don't polish parametric tuning on top of a retry
  architecture we've already marked as STOP-GAP pending proper CCSDS
  TC-Layer + COP-1. T13 reopens once the CCSDS command path lands —
  the adaptive algorithm likely maps onto whatever flow-control / FARM
  machinery the CCSDS layer provides, not onto naive retry counts.
  `AO_RfManager_ok_to_retry()` API already exists in-tree, ready for
  use when T13 returns.

- **Re-evaluate Stage T "95% first-try" gate with correct baseline.**
  User observation 2026-04-22: the Stage T diagnostics measured
  operator-burst ACK rate (10 Hz retry over ~300 ms), which treats "3rd
  retry succeeded at 300 ms" as a failure. But on a half-duplex LoRa
  link with sparse station TX (no heartbeat before IVP-T14d), a
  3rd-retry ACK latency of ~300 ms is within ABORT's 250 ms budget +
  reasonable operational margin. The 6.7% first-try number conflates
  "link broken" with "expected half-duplex latency for a burst into a
  sparse RX window." Once CCSDS station beacon lands, re-measure: (a)
  steady-state first-try success when station is continuously TXing;
  (b) actual ACK-latency distribution (p50 / p95 / p99); (c) whether
  the anchor-station-TX-to-vehicle-RxDone architectural fix is still
  needed or was over-engineered for our actual link conditions.

## Medium (session-scale, 4–12 hours)

Scope is clear but touches multiple files, needs verification, or has small design questions.

- **`standards/HW_GATE_DISCIPLINE.md` + checklist amendment.** User ask 2026-04-17. Gate definitions must name a positive-control signal (e.g., "DAC 0x18 ACKs" proves bus health separately from device-under-test), a 3-boot reseat protocol, and require commit messages to cite the observed control signal — not just "build clean + MSP stable". Motivated by IVP-140 false-positive + Fruit Jam GPS cable episode. Related: LL Entry 25, 36.
- **Station SPIN model extensions.** Scaffolding landed (IVP-147: P_TERMINATION + P_NO_DOUBLE_CLEAR, both PASS). Extend when corresponding firmware behavior lands: multi-pending-in-flight, RadioScheduler TX-window arbitration (needed for the sync-gap fix), MAVLink parser state, `station_idle_tick` GPS poll interleave.
- **Full AO audit against `docs/decisions/AO_COMMANDMENTS.md`.** Prelim for Stage T Batch B did only a spot-check sufficient to confirm no LL Entry 32/35-class violations block Batch B (findings: 3 documented blocking-in-handler deviations in `ao_rcos.cpp` / `ao_radio.cpp` with in-code rationale; all `QACTIVE_POST` use static events). A thorough audit across all 7 AOs — Commandments I-XII each checked, violations ranked by severity, remediation plan per violation, decision per violation whether to fix or document-as-deviation — hasn't been done. Should happen post-Stage-T, before any new AO is added (e.g., future dual-mode / Batch B+ additions). Promotion of `AO_COMMANDMENTS.md` to `standards/ACTIVE_OBJECT_RULES.md` (JSF-AV-grade normative) is a possible outcome.

## Large (multi-session, architectural)

Needs council review or planning doc before starting.

- **Station→vehicle radio health channel.** Council A3 asked for condensing station readiness to a single bit the vehicle's GO/NO-GO consumes via radio. Current channel is command-only, no periodic telemetry-back. Dedicated IVP when telemetry-back direction is wired.
- **Real-World Accuracy Tests plan.** Bench-side ground-truth validation — IMU known-angle tilts, baro altitude vs reference, GPS stationary/moving baseline characterization, ESKF replay vs synthetic truth, Allan variance for gyro/accel. Doesn't need launch window or airframe. Complements Stage 18 field tuning. Needs dedicated plan doc with prior-art research (ArduPilot EKF tuning, PX4 calibration) and equipment assessment.
- **Launch procedure audit items.** Six future safety items from NASA/SpaceX/NAR procedure comparison, all requiring Mission Profile or hardware support:
  1. Angle-rate abort guard (BOOST bank threshold → ABORT; needs IMU attitude in BOOST)
  2. No-pyro-after-impact guard (landing guard before apogee guard → suppress pyro)
  3. Hung-fire / ignition timeout (track time since ARM, station-side exclusion timer)
  4. Igniter continuity check (station-side pre-arm check)
  5. Air-dropped vehicle profile (altitude-aware abort, no "stay on ground")
  6. Multi-engine / staging support (partial engine light, inter-stage hold, TRA 13-9)

## Research / Deferred

No code changes planned — kept as context for future decisions.

- **ELRS on RP2350 — research item.** Running ExpressLRS natively on RP2350 with PIO-assisted frequency hopping. Current RFM95W (bare SX1276 on SPI) may be compatible if packet format + hopping schedule can be implemented in firmware. Telstar Booster Pack already describes CRSF/UART to a dedicated ELRS module as the alternative path. Future radio protocol investigation.
- **PIO hardware failure gap — Gemini tier only.** IVP-130 Scenario 5 confirmed: external PIO SM halt is undetectable by firmware (PIO watchdog IRQ only fires from PIO program itself; ARM-side monitoring defeats the independence point). Correct mitigation = physical redundancy (second independent timer on separate MCU). Gemini-tier feature (dual-core carrier board). Accepted gap for Core/Titan.
- **RP2350B/Fruit Jam persistent bus-corruption hypothesis.** User hunch 2026-04-17: one boot during the Fruit Jam GPS debug had a transition not fully explained by the cable theory alone. Investigate whether RP2350B exhibits bus-corruption state that survives power cycles. Low priority — may be a dead end, keep passive.

## Deferred (near-term, post-Stage 15)

- **Battery ADC monitoring.** Hardware not wired. ADC pin + driver + telemetry field.
- **CCSDS SDLS command authentication.** Telecommand auth for Rocket profile.

## Far-future

Mission Profile OTA, F' evaluation, u-blox GPS, OTA drivers, GPS-free 3D reconstruction, FSK bitstream, MATLAB export — all tracked in `docs/PROJECT_STATUS.md` future features.

## Upcoming Stages

**Stage 15: Pre-Flight Polish** — AO responsibility audit (Stage 13 Core1 gap), Audio Output (I2S DAC, ~10-12 IVPs, fills Stage 14 audio backend stub), User Guide, Runtime Behavior Map update for AO architecture, defense-in-depth evaluation (Core1 stall checked in 3 places post-Stage-14 — evaluate justified vs. bloat).

**Stage 16: Field Tuning** — All VALIDATE parameters. Needs flight data.

**Stage 17: Field Testing** — IVP-135, 136, 137, 138. Airframe integration, ground test, flight test, exit gate. Needs hardware access and weather. IVP-134 (pre-flight checklist) already committed.
