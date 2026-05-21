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

**Stages 1-14 + 16A + 16B + 16C + L + T COMPLETE.** **871** host `ctest` entries (867 C++ discovered + **4** Python `scripts/` gates), SPIN 11/11. Tracking: `docs/AO_ARCHITECTURE.md`. **Stage 17 (Field Testing & Avionics Airworthiness) restructured 2026-04-22** from 5-IVP direct-to-flight into 13-IVP tapered buildup (three council rounds, approved with amendments). First motor flight = step 13 of 14. Plan: `docs/plans/STAGE17_TAPERED_BUILDUP.md`. Execution awaits future session; starts with IVP-135a (pure-software log schema extension). **CCSDS TC-Layer + COP-1 rework deferred to post-Stage-17** (unanimous council) — field data will inform scoping.

## Use Cases
1. **Cross-agent review** — Flag concerns about other agents' work (see `CROSS_AGENT_REVIEW.md`)
2. **Cross-context handoff** — Notes for future Claude sessions when context is lost
3. **Work-in-progress tracking** — Track incomplete tasks spanning multiple sessions
4. **Hardware decisions pending** — Flag items needing user input before code changes
5. **Deferred items** — Active intent kept visible until acted on (and then erased — see header rule)

---

## High priority

- **rc_log drain rate-limiting — known-limitation followup** (LL Entry 39). Tier 5 (~430 callsites across rc_os/rc_os_debug/rc_os_commands/ao_rcos/dashboard) shipped 2026-05-17 with the existing ring-empty fast-path, no drain-rate regressions observed in normal CLI use. The known limitation remains: under sustained heavy CLI output (e.g. an operator dumping the full sensor-status block in a tight loop) the ring isn't empty for sustained periods and the drain runs on every idle tick. Council deferral stands — address when actual sustained-output regression surfaces. Candidate fixes still apply: rate-limit drain to every-Nth-idle-tick, OR drain via dedicated timer-tick instead of qv_idle_bridge.

- **Station fault-inject probe-coverage gap.** No SWD on Fruit Jam. `fault_force_station_*` entries grep-verified only (no live probe-driven positive-control). Options when full positive-path is needed: move probe between sessions, or build station firmware on Feather (`PICO_BOARD=adafruit_feather_rp2350 ROCKETCHIP_JOB_STATION=1`) as a probe-accessible test bed.

- **Four-cycle plan — Cycle 4 stashed.** L2-P5 JSF AV walk + L2-P10 CLA-RBM re-collection. Cycles 1-3 closed; gate now open. See CHANGELOG for cycle-by-cycle history.

- **Next up (deferred from DC-2026-05-13 cycle, not part of four-cycle plan):**
  - **Host-side replay harness implementation** (`scripts/replay_harness_host.py` is a stub). Per R-25-exec amendment #4, IVP-131 verification model shifts from on-MCU CSV-streamer to host-side ESKF replay against `tests/replay_profiles/*.csv` ground-truth. Needs host-buildable ESKF driver + comparison harness against the oracle. Out of cycle scope; tracked here until implemented.

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

- **Station→vehicle radio health channel — deferred to CCSDS rework batch.**
  Council A3 asked for condensing station readiness to a single bit the
  vehicle's GO/NO-GO consumes via radio. Current channel is command-only,
  no periodic telemetry-back. Moved into the CCSDS rework batch (with
  IVP-T13 + Stage T re-baseline) on 2026-05-21 — the telemetry-back
  direction wiring depends on what TC-Layer / TM-Layer split the CCSDS
  rework lands, so designing it now would commit to an interface that
  the CCSDS work will likely change.

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

- **Systematic dead-code / orphaned-TU detector.** No mechanical check currently exists for dead-but-still-compiled artifacts. The R-5 Unit D part 1 verification on 2026-05-16 surfaced one instance — `src/telemetry/telemetry_service.cpp` had been orphaned at IVP-94 commit `71e4816` (2026-03-31) when AO_Telemetry took over the protocol layer directly, and the now-unreferenced TU sat compiled-but-DCE'd for ~6 weeks before being noticed. That instance is being cleaned up inline. The general question this raises and which the WB row tracks: how many other orphans are sitting in `src/`, and what's the standing check that would catch the next one. The instance R-5 caught was easy mode (entire TU DCE'd, found via `nm` comparison). The harder cases are: (a) **partial dead code** — a TU where some exports are live but `static` helpers or unused-but-not-DCE'd functions are dead; (b) **dead branches** in live functions (compile-time-constant `if (false)`, vestigial `#ifdef` blocks left enabled, unreachable `case` arms); (c) **dead headers** declaring functions whose definitions were removed; (d) **dead globals** (variables defined and live-referenced but never read meaningfully, e.g., flags that are written but no longer queried); (e) **dead CLI commands** (handlers still in the switch but no user-facing documentation or path to invoke them). Candidate tools to evaluate when a session takes this up: clang's `-Wunused-function`/`-Wunused-variable` for `static` helpers (compile-time, already enabled with `-Wall -Wextra` per CODING_STANDARDS, but `static`-only); GCC's `-flto` + `-Wlto-type-mismatch` for cross-TU dead detection; `nm --undefined-only`/`nm --extern-only` set difference (what R-5 used by hand); cppcheck `--enable=unusedFunction` (covers cross-TU but slow); a custom diff harness that compares the source-file source list (CMakeLists `ROCKETCHIP_SOURCES`) against the linker's actually-emitted-symbol set (the IVP-94 incident, exactly). Probably needs to be **multiple gates layered** — fast incremental check in pre-commit (catches some), thorough sweep at milestone (catches the rest). Companion to `BUILD_SYSTEM_AUDIT.md` items P1-A/P3/P4 (self-flagged dead code, ROCKETCHIP_SOURCES coverage, host/target split) which already cover *some* of this but didn't catch `telemetry_service.cpp`. Out-of-scope to design here — needs a dedicated session with research into IRL practice (LLVM/Clang community has standing patterns; ArduPilot has `wscript` heuristics).

## Large (multi-session, architectural)

Needs council review or planning doc before starting.

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

- **Fault-recovery rework follow-ups (commits `ed7c569` + `8baa18a` landed 2026-05-14/15; see `docs/decisions/FAULT_RECOVERY_2026-05-14.md` for the design):**
  - **PIO beacon + SPI last-gasp combined session** — see dedicated row below.
  - **AON-timer prior-uptime signal** — stubbed to 0 in the anomalous-boot confidence gate. Wiring it requires adding `pico_aon_timer` to target_link_libraries + explicit timer-start at boot. Marginal value (POWMAN reset register already carries the high-confidence signal for brownout; AON timer would corroborate for the watchdog-RSM / hazard-DP / glitch-detect / SWcore-PD reset classes only). Worth picking up if auto-zero-baro suppression false-positive rate during bench testing needs an extra corroborator. Otherwise deferred.

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
