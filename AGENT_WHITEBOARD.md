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

**Stages 1-14 + 16A + 16B + 16C + L + T COMPLETE.** **856** host `ctest` entries (852 C++ discovered + **4** Python `scripts/` gates), SPIN 11/11. Tracking: `docs/AO_ARCHITECTURE.md`. **Stage 17 (Field Testing & Avionics Airworthiness) restructured 2026-04-22** from 5-IVP direct-to-flight into 13-IVP tapered buildup (three council rounds, approved with amendments). First motor flight = step 13 of 14. Plan: `docs/plans/STAGE17_TAPERED_BUILDUP.md`. Execution awaits future session; starts with IVP-135a (pure-software log schema extension). **CCSDS TC-Layer + COP-1 rework deferred to post-Stage-17** (unanimous council) — field data will inform scoping.

---

## Session Handoff Notes (2026-05-26, Grok)

**User requested explicit session handoff + pause.**

**Work completed this session:**
- Deep dive into current CCSDS status (TM side implemented as pruned Space Packet; TC + COP-1 is the missing STOP-GAP).
- Created thorough current-state analysis of the command/retry/ACK reliability layer (the main area the CCSDS command rework would replace).
- Produced two primary artifacts for CCSDS prep:
  - `docs/decisions/CURRENT_COMMAND_RETRY_ACK_DATA_FLOW.md` (detailed textual map with file:line citations)
  - `docs/decisions/CURRENT_COMMAND_RETRY_ACK_DATA_FLOW.dot` + rendered `.svg` (Graphviz diagram matching project conventions from `docs/audits/cla_rbm/dot/`)
- Installed Graphviz via winget so the visualizer can be used going forward.
- Graphviz is now present on the machine (`C:\Program Files\Graphviz\bin\dot.exe`).

**Files touched (all new, untracked):**
- docs/decisions/CURRENT_COMMAND_RETRY_ACK_DATA_FLOW.md
- docs/decisions/CURRENT_COMMAND_RETRY_ACK_DATA_FLOW.dot
- docs/decisions/CURRENT_COMMAND_RETRY_ACK_DATA_FLOW.svg

**Current state of CCSDS prep work:**
- We now have a high-quality, citable "as-built" map of the existing command delivery + retry mechanism.
- This directly supports scoping, interface design, and migration planning for any future CCSDS TC-layer work (including the standalone library idea the user mentioned).
- Multiple items remain explicitly parked pending this rework (see "High priority" section below: IVP-T13, station radio health channel, first-try metric re-baseline).

**What was discussed but not yet executed:**
- Next actions with the data flow artifacts (scoping doc, library contract definition, target-state diagram, design doc for council, etc.).
- User indicated they want to pause for the night.

**Handoff instructions from user:**
- **Check with user before any commit or push.** Do not commit these files or push without explicit approval in the next session.
- Working tree currently has only the three new decision files above as untracked changes.

**Open questions / suggested next steps for next session:**
- Which direction to take the data flow work (scoping options, library behavioral contract, target-state diagram, full design doc)?
- Any specific refinements needed to the .dot / MD before using them in a council or design review?
- Whether to create a dedicated CCSDS command layer planning document.

**Build / verification state:** N/A — pure documentation work. No src/ changes. No builds or tests affected.

---

**End of handoff note.** (Erase or update this section when work resumes.)

## Session Summary (2026-05-28, Grok) – WSL Soft Pivot

**Work delivered:**
- Full WSL_SOFT_PIVOT plan executed (Phases 0–6).
- Linux FS worktree (`~/Rocket-Chip`) created; clean vehicle + station firmware builds + both `bench_sim.py` (2/2) and `station_bench_sim.py` (3/3) proven from it with positive controls.
- Documentation delivered at docs root: `WSL_SETUP.md`, `WSL_QUICKSTART.md`, `WSL_ROLLBACK_CHECKLIST.md`.
- Steady-state policy ("try until it breaks") reviewed via council; recorded in `docs/decisions/WSL_STEADY_STATE_POLICY_2026-05-28.md`.
- Dual-toolchain exercise requirement added to milestone checklist (item 17c).

**State at close:**
- WSL (Linux FS) is now a fully functional primary environment.
- Windows remains supported fallback with explicit, lightweight rollback checklist.
- Repo clean. All plan deliverables delivered.

---

## Session Handoff Notes (2026-05-30, Grok) – Starcom CCSDS Library External Research

**Context:** User requested deep external research on building a standalone CCSDS library ("Starcom"), with emphasis on the official Physical Layer (CCSDS 211.1-B-4) vs current Rocket-Chip hardware (RP2350 + SX1276), PIO possibilities, commercial radio options, practical tradeoffs for <50 km use, and existing prior art. Session was explicitly handed off for comparison with another agent's research tomorrow, followed by initial planning.

**Work completed this session:**
- Comprehensive external research on CCSDS Physical Layer requirements (211.1-B-4) and what can realistically be achieved with the current SX1276/RFM95W + RP2350 PIO.
- Analysis of PIO opportunities even with the existing radio (Continuous Mode + bit synchronizer + synchronous sampling, custom Manchester handling, improved lock detection, etc.).
- Survey of commercially available radios/IP cores for closer or full compliance (ComBlock 211.1 IP core at ~$2.5k, AX5043, Si4463, ADALM-Pluto SDR, etc.), with price and practicality notes.
- Practical in-field impact assessment for the user's actual use cases (<50 km rocketry, post-landing recovery, log offload).
- Data Link Layer side (COP-1 / COP-P / USLP requirements from 232.1-B-2, 211.0-B-6, 732.1-B-3).
- Prior-art review of existing libraries (OSDLP, NASA cFS, CCSDSPack, etc.).
- Light reference to Rocket-Chip AO integration patterns for future wrapper design.
- Maintained `docs/research/STARCOM_CCSDS_LIBRARY_RESEARCH.md` as the living artifact with clear per-section authorship attribution for multi-agent use.

**Key artifacts updated:**
- `docs/research/STARCOM_CCSDS_LIBRARY_RESEARCH.md` (multiple new sections added with proper attribution)
- `AGENT_WHITEBOARD.md` (this handoff note)

**Current state of the research:**
- All items from the original research todo list are complete.
- The document provides a solid foundation covering official requirements, hardware limitations, practical tradeoffs, prior art, and integration considerations.
- No code changes were made. This was pure external research + documentation.

**Handoff for next session:**
- User plans to review **both agents' research outputs** tomorrow (this document + the other agent's independent research).
- Goal: Compare findings, then begin formulating an initial plan/scope for actually starting the Starcom library.

**Open questions / suggested next steps:**
- After comparing both research documents, what gaps or differing conclusions exist?
- What should the initial library scope/phasing look like (e.g., Data Link Layer first, best-effort PHY adapter, full PHY later)?
- Any specific areas that need deeper research before planning begins?
- Should we create a dedicated planning/decision document for the library architecture?

**Build / verification state:** N/A — pure research and documentation work. No src/ changes, no builds, no tests, no hardware involved.

---

**End of handoff note.** (Erase or update this section when the next session resumes the work.)

---

## Corrections (2026-05-30, Grok) – Research docs fact-check

Factual/staleness errors fixed in both Grok research docs + duplicate structure cleaned in CCSDS doc. CHANGELOG 2026-05-30-001 amended with details. All other citations audited for recency.

See the two research .md files for the corrections.

**End of note.** (Erase after review.)

---

## Use Cases
1. **Cross-agent review** — Flag concerns about other agents' work (see `CROSS_AGENT_REVIEW.md`)
2. **Cross-context handoff** — Notes for future Claude sessions when context is lost
3. **Work-in-progress tracking** — Track incomplete tasks spanning multiple sessions
4. **Hardware decisions pending** — Flag items needing user input before code changes
5. **Deferred items** — Active intent kept visible until acted on (and then erased — see header rule)

---

## High priority

- **AO Commandments source-citation audit.** Investigating R-27 (RfManager Commandment XII observation) surfaced that Commandment XII's `Source:` line cites LL Entry 36, but LL 36 is about test-tool rot (bench_flight_sim.py going stale), not AO state-transition logging or runtime observability. A research agent walked the doc's stated sources (Samek PSiCC2 Ch. 11, state-machine.com Active Object/RTEF/QP/C SRS pages, NASA F´ Code Style + State Machines doc) and confirmed **no clean substitute citation exists in any of those** — the rule is project-internal invention generalized from folklore, not inherited from external authority. This is an [LL Entry 37](docs/agents/LESSONS_LEARNED.md)-class citation-rot finding. Per Entry 37 discipline ("if one citation was wrong, check the rest"), audit all 12 Commandment `Source:` lines in `docs/decisions/AO_COMMANDMENTS.md` against their cited sources; fix XII's citation (either reframe as project-internal "Rationale:" or cite PSiCC2 Ch. 11 honestly as topical-but-tool-framing); reassess R-27's disposition once the rule's authority is correctly understood. Est. ~1-2 hrs. Block on this is open per user direction 2026-05-22 — address before closing R-27.

- **Four-cycle plan — Cycle 4 stashed.** L2-P5 JSF AV walk + L2-P10 CLA-RBM re-collection. Cycles 1-3 closed; gate now open. See CHANGELOG for cycle-by-cycle history.

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

- **PIO I²C master reference implementation available (Flipper One MCU firmware).** If station I²C ever needs to leave DW_apb_i2c hardware (candidate direction for the Fruit Jam GPS cold-boot intermittency tracked in `docs/plans/CYCLE_RESIDUALS_AFTER_R5.md`), Flipper Devices published their RP2350 co-processor firmware on 2026-05-21 with a working PIO I²C master driver at `lib/drivers/i2c_master_pio/pio_i2c.c` in https://github.com/flipperdevices/flipperone-mcu-firmware. Pairs `pio_claim_free_sm_and_add_program_for_gpio_range` at init with `pio_remove_program_and_unclaim_sm` at deinit (LL Entry 42 discipline as a working pattern). Mid-cycle error recovery uses `pio_sm_drain_tx_fifo` + `pio_sm_exec` (jump-to-wrap) + `pio_interrupt_clear` — never touches program memory. They also use the acquire/release pad-mux discipline (LL Entry 28) as a first-class per-handle pattern (`Activate`/`Deactivate` callbacks in `targets/f100/furi_hal/furi_hal_i2c_config.c`: `i2c_init` + pad config on activate, `i2c_deinit` + pads-to-input on deactivate). Useful as **reading material** before any PIO-I²C migration evaluation; not actionable today. Same SDK (2.2.0) and toolchain (14_2_Rel1) as us. License check needed before any code import.

## Deferred (near-term, post-Stage 15)

- **Battery ADC monitoring.** Hardware not wired. ADC pin + driver + telemetry field.
- **CCSDS SDLS command authentication.** Telecommand auth for Rocket profile.

## Far-future

Mission Profile OTA, F' evaluation, u-blox GPS, OTA drivers, GPS-free 3D reconstruction, FSK bitstream, MATLAB export — all tracked in `docs/PROJECT_STATUS.md` future features.

## Upcoming Stages

**Stage 15: Pre-Flight Polish** — AO responsibility audit (Stage 13 Core1 gap), Audio Output (I2S DAC, ~10-12 IVPs, fills Stage 14 audio backend stub), User Guide, Runtime Behavior Map update for AO architecture, defense-in-depth evaluation (Core1 stall checked in 3 places post-Stage-14 — evaluate justified vs. bloat).

**Stage 16: Field Tuning** — All VALIDATE parameters. Needs flight data.

**Stage 17: Field Testing** — IVP-135, 136, 137, 138. Airframe integration, ground test, flight test, exit gate. Needs hardware access and weather. IVP-134 (pre-flight checklist) already committed.
