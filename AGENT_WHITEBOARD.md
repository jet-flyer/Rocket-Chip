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

- **R-25-exec audit-suite regression status (2026-05-14 bench session, updated 2026-05-15):** T0+T1+T2a passed at Level-2 in the 2026-05-14 session. T3a deprecated 2026-05-15 (rework eliminated the failure mode it was designed to catch — one-off sanity data captured + recorded in this row). **T2b is the remaining open gate** — armed `fault_force_*` sweep with the rework's new phase-aware expected outcomes (kFault transition + FAULT bit, not reset).

  **What ran tonight:**
  - **T0a — boot-parity:** vehicle (probe + GDB load) PASS, banner `vehicle flight v0.16.0 (kmenu)`, board `adafruit feather rp2350 hstx`, `hardware: 14/14 ok`. Station (picotool `--ser BEC71B8EDC6AEBD1` + manual `_boot_parity_check.py` invocation; `verify_boot_parity.sh station` only knows the probe-flash path which can't reach the station with probe permanently on the Feather) PASS, banner `station flight v0.16.0 (dashboard)`, board `adafruit fruit jam`.
  - **T1a — station_bench_sim:** 3/3 PASS in 16.9s. `TARGET_STATION_ANY` retargeting from R-25-exec step 7 working.
  - **T1b — vehicle warm-reboot (labeled-soft via runbook, Rule 4):** G-W2 3× operator-pressed reset button: banner identity stable across all 3 (same `flight-a101185`, `hardware: 14/14 ok`, fresh uptime each). G-W3 5× picotool `reboot -f --ser`: banner stable across all 5. Script path failed twice with Windows USB CDC race (script bugs `_probe_aircr_reset()` `monitor resume` after AIRCR; peek_banner default 2.0s peek-timeout too short for the firmware's post-reset banner cadence on this bench). Pivoted to runbook per the row's pre-declared pivot trigger.
  - **T1c — station warm-reboot (labeled-soft via runbook, Rule 4):** G-W2 3× reset button (note: first cycle was actually cold boot because operator power-cycled the Fruit Jam's USB switch by mistake; that cycle reported `10/11 ok [fail] gps`. Warm-reboot cycles 2 + 3 both reported `11/11 ok`, banner identity stable). G-W3 5× picotool: all 5 reported `11/11 ok`.
  - **T2a — negative-control gated-fault check:** PASS. With `rc::g_test_mode_enabled == false`, GDB `call fault_force_eskf_unhealthy()` produced `[FAULT] fault_force_eskf_unhealthy gated — arm test mode via probe ...` on serial AND `g_eskfInitialized` remained `true`. CODING_STANDARDS R-25-exec audit invariant verified empirically for this entry point. Grep coverage at `src/safety/fault_inject.cpp:57-60+` confirms every `fault_force_*` entry calls `fi_test_mode_gate()` as line 1 of its body (same pattern for `src/safety/station_fault_inject.cpp`).

  **T2b unblocked 2026-05-15** by the fault-recovery rework (commits `ed7c569` + `8baa18a`). T3a deprecated same date (see below). Updated expected outcomes:
  - **T2b — armed `fault_force_*` sweep across all 9 vehicle + 4 station entries.** Updated expected outcomes per commit (b): `fault_force_eskf_unhealthy` no longer triggers reset; produces FAULT bit + degrade-in-place state (kFault transition). `fault_force_launch_abort` produces auto-DISARM via existing 500ms persistence path (B.6 position i unchanged). Other `fault_force_*` entries unchanged. The `arm_test_mode_via_probe()` USB-CDC-re-enum-settle script-flake is still present and may require pivoting to a labeled-soft runbook procedure per HW_GATE_DISCIPLINE Rule 4 (same pivot trigger as T1b/T1c in 2026-05-14 session). Station-side fault entries (4× `fault_force_station_*`) remain grep-only coverage — no SWD on Fruit Jam.
  - **T3a — `warm_reboot_audit` G-W1 + G-W4** (AIRCR-only-Core-0 reset): **DEPRECATED 2026-05-15.** The failure mode T3a was designed to catch — the R-19 SIO_FIFO_IRQ wedge under a firmware-issued AIRCR — was eliminated by the rework (firmware no longer issues AIRCR in flight; only kIdle hardfault handler + Core 1 boot-wait timeout, both pre-flight). Probe-driven AIRCR is a debugger intervention that doesn't map to a real flight failure mode anymore. One-off sanity-check data captured 2026-05-15 confirming the rework didn't regress the silicon-level kIdle-reset path: pre-AIRCR `Reads: I=33166 M=3296 B=1036 G=331` (steady-state), AIRCR via probe (OpenOCD log: `[rp2350.cm0] external reset detected`), post-AIRCR `Reads: I=19947 M=1980 B=623 G=199` (fresh counters from new boot, IMU delta ≥ kIMU_DELTA_MIN=100 threshold from script, all error counters zero), `Hardware: 14/14 ok` maintained, GDB probe confirmed Core 1 in `i2c_read_blocking_internal` actively sampling post-reset. R-19 wedge would have manifested as Core 1 stuck in `isr_invalid` — did not occur. T3a removed from routine audit-suite regression scope. G-W2/G-W3 (chip-wide warm reboot via various paths) remain valid as routine cold-boot regression checks but were already verified in the 2026-05-14 session via labeled-soft runbook.

  **Closure state of PROBLEM_REPORTS rows:**
  - **R-24** — Level-2 CLOSED with T0a. Both roles built, flashed, banner-classified.
  - **R-23** — Level-2 CLOSED with T0 + T1. LM-solver refactor exercised end-to-end via bench_sim + warm-reboot stability.
  - **R-22** — partial Level-2 closure with T1b + T1c chip-wide warm-reboot verified. AIRCR-specific G-W1/G-W4 stays open with deferral note pointing at the architecture session.
  - **R-25-exec** — partial Level-2 closure with T0 + T1 + T2a (audit invariant verified for the negative-control case; grep coverage for the rest). Armed-sweep T2b deferral note pointing at the architecture session.

  **Findings to track separately (not blocking closures):**
  - **Script flake (warm_reboot_audit + enhanced_fault_injection):** both scripts trip on Windows USB CDC re-enum timing after probe-mediated chip reset. Specific bugs found: (a) `_probe_aircr_reset()` `monitor resume` after AIRCR write fails with "context restore failed" because AIRCR external-resets the chip out from under OpenOCD's halt-state (fix: drop the resume); (b) `peek_banner` default `peek_timeout=2.0s` too short for the post-reset banner-emit cadence (firmware's `handle_usb_connect` settle is 5 ticks @ 20Hz = 250ms + USB enum settling). Not patched in the script during this session per user direction to revert and pivot to runbook. Either fix in a dedicated script-hardening session, or accept the runbook for these gates going forward.
  - **Station GPS cold-boot slow-start:** on power-on, Fruit Jam reports `hardware: 10/11 ok [fail] gps` with PMTK writes returning -1 and `window_hit:0 init:0`. Warm-reboots immediately after show `11/11 ok`. Consistent with the I2C-GPS init-window timing pattern from LL Entry 31. Not blocking but worth tracking — does the station benefit from a longer GPS init window, or is one-shot retry on PMTK-fail the right fix?
  - **Station fault-inject probe-coverage gap:** no SWD on station. `fault_force_station_*` entries can only be verified by grep + reasoning about the gate mechanism. If we want armed exercise of those, options are: (1) move probe between sessions; (2) build station firmware on the Feather (`PICO_BOARD=adafruit_feather_rp2350 ROCKETCHIP_JOB_STATION=1`) as a probe-accessible Frankenstein-by-design for fault-inject testing only.

- **Next up (other deferred items from DC-2026-05-13 cycle, each its own session):**
  - **Host-side replay harness implementation** (`scripts/replay_harness_host.py` is a stub). Per R-25-exec amendment #4, IVP-131 verification model shifts from on-MCU CSV-streamer to host-side ESKF replay against `tests/replay_profiles/*.csv` ground-truth. Needs host-buildable ESKF driver + comparison harness against the oracle. Out of cycle scope; tracked here until implemented.
  - **Host ctest sweep over `fault_force_*` symbols** to mechanically verify every entry calls `rc::test_mode_active()` (audit invariant from CODING_STANDARDS R-25-exec section). Today it's a grep + manual walk; making it a ctest closes the audit gate. Companion to step 10's pre-commit-matrix path additions.
  - **L2-P2/P3/P4** from the 2026-05-07 cycle (sampling policy / citation inventory / scope language) — audit-policy doc edits, batch with whichever later cycle picks them up.
  - **R-5** (full stdio.h removal) + **R-2** (absorbed into R-5) — dedicated ETL session.
  - **R-20 / R-21 / L2-W1** dispositioned by the fault-recovery architecture rework (commits `ed7c569` + `8baa18a`, 2026-05-14/15). R-20 (Core 1 boot-wait AIRCR half-broken) — plan option C in modified form: in-flight Core 1 boot-wait timeout now transitions to kFault per B.1/B.8 instead of issuing AIRCR. R-21 (no auto chip-reset on PIO WDT) — endorsed unchanged: PIO watchdog signals IRQ flag, never auto-resets; plan B.1 confirms no auto-reset is the correct in-flight behavior. L2-W1 = R-20 (same finding, different IDs in audit doc vs PROBLEM_REPORTS). All three close when the rework's PROBLEM_REPORTS rows transition `verified → closed` after Level-2 audit-suite regression.
  - **CLA-RBM re-collection** triggers at the 90-day mark (~2026-06-06) per the Tier 4.5 threshold from the 2026-05-13 procedure refactor. Until then, watched-but-not-stale.

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

- **Station SPIN model extensions.** Scaffolding landed (IVP-147: P_TERMINATION + P_NO_DOUBLE_CLEAR, both PASS). Extend when corresponding firmware behavior lands: multi-pending-in-flight, RadioScheduler TX-window arbitration (needed for the sync-gap fix), MAVLink parser state, `station_idle_tick` GPS poll interleave.

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

- **In-flight fault recovery architecture rework — LANDED 2026-05-14/15** (commits `ed7c569` + `8baa18a`). The "design question" version of this row (the 5 questions surfaced 2026-05-12, the surfaced SIO_FIFO_IRQ wedge limitation, the multi-MCU framing) has been substantially executed by the rework. Plan + transcript: `C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn.md` (council rounds 1+2+3 unanimous). What landed:
  - Zero in-flight reset (B.1). `memmanage_fault_handler` + `Q_onError` are phase-aware: kIdle path captures + AIRCR resets after visible-signal delay; any flight phase transitions to kFault + busy-loops + PIO timers continue autonomously.
  - No silent reset on pad (B.2). Visible-signal delay before reset; operator sees prior-fault latch and clears via CLI `r` in IDLE.
  - Phase-aware dispatch via checksummed `phase + ~phase` pair (B.3) — corrupted byte falls back to kFault (safe-by-default).
  - Anomalous-boot confidence gate (B.4). Reads POWMAN_CHIP_RESET cause bits + `.uninitialized_data` flight-in-progress sentinel + (stub) AON timer; PROBABLY_MID_FLIGHT suppresses auto-zero-baro at `main.cpp:381`. Brownout-cause routes to independent `kHealthCriticalPriorBrownout` health-monitor latch regardless of mid-flight verdict.
  - ARM-state position (i) (B.6) — keep existing 500ms-persistence auto-DISARM. No code change required; matches ArduPilot/PX4 lived-experience for no-uplink vehicles.
  - Fault-handler reentrance guard (B.7) — `s_in_fault_handler` one-shot → `__WFE()` halt on second entry.
  - Explicit `FlightPhase::kFault` enum value + `kLedPhaseFault` magenta-blink (B.8). Distinct from `kAbort`.
  - GoNoGo Tier-1 stations for prior-hardfault + prior-brownout latches (newly load-bearing for ARM gating; pre-rework these were visibility-only).
  - Legacy stale comments cleaned up across `fault_protection.{h,cpp}`, `crash_record.h`, `fault_inject.cpp` removing references to the SDK hardware watchdog / `watchdog_kick_tick()` that don't exist in tree.

  **What's still open (becomes named future-session items rather than design-question subsections):**
  - **PIO beacon + SPI last-gasp combined session** (see dedicated row below).
  - **Mid-flight-reboot survivability beyond detection** is rejected unanimously per NASA Initialization-Safe-Mode trap framing + LL Entry 36; no future session planned.
  - **Multi-MCU Gemini/Titan-with-Core architecture** stays as a hardware-tier item; out of single-MCU scope.
  - **AON-timer prior-uptime signal** — stubbed to 0 in commit (a). Wiring it requires adding `pico_aon_timer` to target_link_libraries + explicit timer-start at boot. Marginal value (POWMAN reset register already carries the high-confidence signal for the most-critical brownout case; AON timer would corroborate for the watchdog-RSM / hazard-DP / glitch-detect / SWcore-PD reset classes only). Worth picking up if the auto-zero-baro suppression false-positive rate during bench testing turns out to need an extra corroborator. Otherwise deferred.

  **Audit-suite regression** for the rework's R-22 / R-23 / R-24 / R-25-exec closure path: T2b (armed fault-inject sweep) is the remaining active gate. T3a deprecated 2026-05-15 (probe-driven AIRCR no longer maps to a real flight failure mode post-rework; one-off sanity check captured before deprecation — see T3a row above). R-24 already closed at Level-2 last session via T0a; R-23 already closed via T0+T1. R-22 closes when T2b passes for the rework's expected outcomes + the WB chip-wide warm-reboot data from 2026-05-14 (G-W2/G-W3 runbook). R-25-exec closes when T2b's full armed sweep observes the new phase-aware fault dispatch.

- **PIO beacon + SPI last-gasp beacon (B.5) — combined dedicated future session.** Council round 3 (NASA/JPL + Cubesat, 2026-05-15) unanimously deferred the SPI-based last-gasp beacon (commit (c) of the rework was scoped for this and *not* implemented). User direction 2026-05-15: "merge with the future PIO beacon" — the two questions evaluate together rather than pre-committing to an interface (compile-time `ROCKETCHIP_LAST_GASP_BEACON` + `radio_init_confirmed` semantics) that would constrain the PIO design choice. Reasons for deferral, fully captured in plan B.5 + council-round-3 transcript at `C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn-agent-a355e8caee0717e0b.md`:
  - **SPI peripheral state corruption** if fault occurred mid-byte/mid-burst — recovery is NOT bounded-cost (FIFO drain + CS deassert via GPIO function override + SX1276 hardware reset pulse + full cold re-init; each step has its own hang potential).
  - **`#ifdef`-scaffolding-rot pattern** per LL Entry 36 — code-shaped-but-never-exercised artifact creates false-confidence for future contributors.
  - **JPL precedent: always architect beacons as independent silicon** (Cassini LGA+USO, SMAP transponder). Cubesats that share the radio rely on modem-level autonomous beacon modes (e.g., FSK Beacon Mode); SX1276 LoRa lacks this in long-range mode.
  - **PIO beacon is the architecturally-correct answer** (Pico SDK has working PIO-SPI in 2-3 instructions per primary-source verification 2026-05-14; DIO0 wired to GPIO 6 on Adafruit Feather RP2350; DIO5 not currently routed but solder-jumperable on the RFM95W FeatherWing #3231).
  - **In-flight ARM-dead beacon-coverage gap is the trade** — accepted as a known gap for Core/Titan single-MCU pending this session.

  Scope of the combined session: (1) evaluate whether SPI-from-fault-handler is ever the right stop-gap given the failure-mode inventory; (2) design the PIO-driven beacon program (target: PIO0 or PIO1 — PIO2 already shared between watchdog SM0 + backup-timer SM1-3); (3) decide whether the design requires soldering the DIO5 jumper on the FeatherWing; (4) bench-verify on a known-faulted chip state if any stop-gap is in scope. Likely outputs a dedicated decision doc under `docs/decisions/`. User direction will determine sequencing relative to other open work.
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
