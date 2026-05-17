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

- **rc_log drain rate-limiting — known-limitation followup** (LL Entry 39). Tier 5 (~430 callsites across rc_os/rc_os_debug/rc_os_commands/ao_rcos/dashboard) shipped 2026-05-17 with the existing ring-empty fast-path, no drain-rate regressions observed in normal CLI use. The known limitation remains: under sustained heavy CLI output (e.g. an operator dumping the full sensor-status block in a tight loop) the ring isn't empty for sustained periods and the drain runs on every idle tick. Council deferral stands — address when actual sustained-output regression surfaces. Candidate fixes still apply: rate-limit drain to every-Nth-idle-tick, OR drain via dedicated timer-tick instead of qv_idle_bridge.

- **Pre-existing 30%+ comment-density functions to evaluate.** Surfaced 2026-05-17 during R-5 session-end comment-density audit (per `feedback_comment_density.md` — Polyspace 20% floor, healthy 15-25% band). Session-touched files are fine in aggregate (TOTAL 23.1%). These per-function hot spots exceed 30% in a single function with comments that pre-date this session:
  - `src/drivers/gps_uart.cpp` `gps_uart_init` 44.1%, `gps_uart_reinit` 37.1%, `negotiate_baud_to_57600` 33.3%
  - `src/drivers/i2c_bus.cpp` `i2c_bus_init` 38.7%, `i2c_bus_recover` 34.6%
  - `src/flight_director/flight_director.cpp` `enter_phase` 46.2%
  - `src/active_objects/ao_telemetry.cpp` `encode_and_send` 30.4%
  Decision per future session: for each, check if the multi-paragraph rationale is captured in a decision doc / LL entry; if yes, trim the inline to a 1-line pointer; if no, write the rationale to a doc first, then trim. Don't blindly trim — information preservation per user direction 2026-05-17.

- **Dead-code CCSDS `q`→QUERY_RADIO_CONFIG mapping in `cli_handle_unhandled_key`.** Surfaced 2026-05-16 during Unit F tier-end L2 wire-verification. `src/cli/rc_os_commands.cpp:1509` maps `q` to QUERY_RADIO_CONFIG, gated by `kRadioModeRx`. But `src/cli/rc_os.cpp:172-177` in `handle_main_menu` unconditionally consumes `q` → debug menu BEFORE the unhandled-key dispatcher is reached. So the QUERY path is unreachable on the current dispatch. ROCKETCHIP_OS.md updated to note the dead-code path; the code itself either needs to (a) move QUERY to a different key, (b) gate the debug-menu enter behind `!kRadioModeRx`, or (c) just delete the dead QUERY path. Low priority — SET_RADIO via `r` already covers the station's CCSDS exchange needs.

- **CONFIG_TEST_MATRIX + Code Classification doc lag.** `scripts/ci/pre_commit_matrix.py` gate widened 2026-05-16 to "categories not enumerations" but `docs/CONFIG_TEST_MATRIX.md` Tier 6b section + `standards/CODING_STANDARDS.md` Code Classification table still describe the old narrower scope. State-of-system trigger applies — these docs are now contradicted by the matrix script. Doc-only follow-up; no code change needed.

- **R-25-exec audit-suite regression: all gates CLOSED at Level-2 (2026-05-14 + 2026-05-15 sessions).** T0a + T1a + T1b/T1c (labeled-soft runbook per HW_GATE_DISCIPLINE Rule 4) + T2a (negative-control gate) verified 2026-05-14. T2b positive-path verified 2026-05-15 via probe-only combined-session runbook (now documented in `docs/FAULT_INJECTION.md`): test mode armed via probe magic + reset, `g_test_mode_enabled=true` confirmed at kIdle, `fault_force_eskf_unhealthy()` called → `g_eskfInitialized: true→false` observed (state-mutation positive-control signal). Grep coverage at `src/safety/fault_inject.cpp` + `station_fault_inject.cpp` confirms 8/8 + 3/3 gated entries call `fi_test_mode_gate()` / `fis_test_mode_gate()` at line 1 of body (recovery actions `fault_force_core0_stall_clear()` + `fault_force_station_gps_restore()` intentionally exempt per documented design). T3a deprecated 2026-05-15 — the R-19 SIO_FIFO_IRQ wedge it was designed to catch was eliminated by the fault-recovery rework (firmware no longer issues AIRCR in flight); one-off sanity data captured (pre/post AIRCR sensor counters reset cleanly, hardware 14/14 ok, GDB-observed Core 1 active in `i2c_read_blocking_internal` post-reset, no wedge). PROBLEM_REPORTS closures: R-22, R-23, R-24, R-25-exec all Level-2 CLOSED. R-20/R-21/L2-W1 dispositioned by rework per plan B.1/B.8.

  **Findings tracked separately (not blocking):**
  - **Station GPS cold-boot slow-start:** Fruit Jam on power-on reports `hardware: 10/11 ok [fail] gps`, warm-reboot immediately after = `11/11 ok`. Consistent with LL Entry 31 I2C-GPS init-window pattern. Question for future session: longer GPS init window vs. one-shot retry on PMTK-fail.
  - **Station fault-inject probe-coverage gap:** no SWD on Fruit Jam. `fault_force_station_*` entries grep-only. Options when full positive-path is needed: move probe between sessions, or build station firmware on Feather (`PICO_BOARD=adafruit_feather_rp2350 ROCKETCHIP_JOB_STATION=1`) as a probe-accessible test bed.

- **Four-cycle plan status (council-approved 2026-05-15):**
  - **Cycle 1 CLOSED 2026-05-15** — quick decoupled audit closures. Five rows L2-P6, L2-P7, L2-P8, L2-P9, L2-P11 advanced `analyzed→closed` via `docs/audits/AUDIT_COVERAGE_QUICK_CLOSURES_2026-05-15.md`. R-26 (stale `#ifdef ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS` in version.h) found + closed inline. R-27 opened as observation-only (RfManager XII via accessor + events). L2-P5 + L2-P10 deferred to Cycle 4 (coupled to upcoming refactors).
  - **Cycle 2 (R-5 stdio removal) CLOSED at Level-3 2026-05-17.** Units A-K all shipped; allowlist deleted; pre-commit Gate 1 upgraded to unconditional stdio rejection. See PROBLEM_REPORTS R-5 row for evidence + commit chain. Side-effects: R-2 closed by absorption.
  - **Cycle 3 — STASHED** to [`docs/plans/CYCLE_RESIDUALS_AFTER_R5.md`](docs/plans/CYCLE_RESIDUALS_AFTER_R5.md). Station GPS cold-boot fix. R-5 prereq cleared; ready when next session picks it up.
  - **Cycle 4 — STASHED** (L2-P5 JSF AV walk + L2-P10 CLA-RBM re-collection). Runs after Cycle 3 closes at Level-2.

- **Next up (deferred from DC-2026-05-13 cycle, not part of four-cycle plan):**
  - **Host-side replay harness implementation** (`scripts/replay_harness_host.py` is a stub). Per R-25-exec amendment #4, IVP-131 verification model shifts from on-MCU CSV-streamer to host-side ESKF replay against `tests/replay_profiles/*.csv` ground-truth. Needs host-buildable ESKF driver + comparison harness against the oracle. Out of cycle scope; tracked here until implemented.
  - **Host ctest sweep over `fault_force_*` symbols** to mechanically verify every entry calls `rc::test_mode_active()` (audit invariant from CODING_STANDARDS R-25-exec section). Today it's a grep + manual walk; making it a ctest closes the audit gate. Companion to step 10's pre-commit-matrix path additions.
  - **L2-P2/P3/P4** from the 2026-05-07 cycle (sampling policy / citation inventory / scope language) — audit-policy doc edits, batch with whichever later cycle picks them up.

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

- **CONFIG_TEST_MATRIX doc lag (post pre-commit-matrix widening).**
  Council 2026-05-16 widened `scripts/ci/pre_commit_matrix.py`'s
  FLIGHT_CRITICAL regex from a narrow path enumeration to "any
  firmware-affecting change." The hook change shipped. CODING_STANDARDS.md
  Code Classification table partially addressed by R-5 Unit J 2026-05-17
  (stdio column retired, but the prescriptive vs descriptive framing
  question for the three-tier structure remains an open design
  discussion — defer to next session that touches the table).
  `docs/CONFIG_TEST_MATRIX.md` Tier 6b section still describes the
  former narrow matrix; needs to reflect the widened regex + the
  "categories not enumerations" rationale. Doc-only edit.
  Per LL Entry 39: this is the meta-pattern that LL36/LL39 keep
  surfacing — categories drift behind the code they police. Doc
  realignment is the documentation-side of the same lesson.

## Medium (session-scale, 4–12 hours)

Scope is clear but touches multiple files, needs verification, or has small design questions.

- **Station SPIN model extensions.** Scaffolding landed (IVP-147: P_TERMINATION + P_NO_DOUBLE_CLEAR, both PASS). Extend when corresponding firmware behavior lands: multi-pending-in-flight, RadioScheduler TX-window arbitration (needed for the sync-gap fix), MAVLink parser state, `station_idle_tick` GPS poll interleave.

- **Systematic dead-code / orphaned-TU detector.** No mechanical check currently exists for dead-but-still-compiled artifacts. The R-5 Unit D part 1 verification on 2026-05-16 surfaced one instance — `src/telemetry/telemetry_service.cpp` had been orphaned at IVP-94 commit `71e4816` (2026-03-31) when AO_Telemetry took over the protocol layer directly, and the now-unreferenced TU sat compiled-but-DCE'd for ~6 weeks before being noticed. That instance is being cleaned up inline. The general question this raises and which the WB row tracks: how many other orphans are sitting in `src/`, and what's the standing check that would catch the next one. The instance R-5 caught was easy mode (entire TU DCE'd, found via `nm` comparison). The harder cases are: (a) **partial dead code** — a TU where some exports are live but `static` helpers or unused-but-not-DCE'd functions are dead; (b) **dead branches** in live functions (compile-time-constant `if (false)`, vestigial `#ifdef` blocks left enabled, unreachable `case` arms); (c) **dead headers** declaring functions whose definitions were removed; (d) **dead globals** (variables defined and live-referenced but never read meaningfully, e.g., flags that are written but no longer queried); (e) **dead CLI commands** (handlers still in the switch but no user-facing documentation or path to invoke them). Candidate tools to evaluate when a session takes this up: clang's `-Wunused-function`/`-Wunused-variable` for `static` helpers (compile-time, already enabled with `-Wall -Wextra` per CODING_STANDARDS, but `static`-only); GCC's `-flto` + `-Wlto-type-mismatch` for cross-TU dead detection; `nm --undefined-only`/`nm --extern-only` set difference (what R-5 used by hand); cppcheck `--enable=unusedFunction` (covers cross-TU but slow); a custom diff harness that compares the source-file source list (CMakeLists `ROCKETCHIP_SOURCES`) against the linker's actually-emitted-symbol set (the IVP-94 incident, exactly). Probably needs to be **multiple gates layered** — fast incremental check in pre-commit (catches some), thorough sweep at milestone (catches the rest). Companion to `BUILD_SYSTEM_AUDIT.md` items P1-A/P3/P4 (self-flagged dead code, ROCKETCHIP_SOURCES coverage, host/target split) which already cover *some* of this but didn't catch `telemetry_service.cpp`. Out-of-scope to design here — needs a dedicated session with research into IRL practice (LLVM/Clang community has standing patterns; ArduPilot has `wscript` heuristics).

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
