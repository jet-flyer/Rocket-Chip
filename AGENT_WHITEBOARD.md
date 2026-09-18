# Agent Whiteboard

**Purpose:** Cross-context / cross-agent communication channel for **active work only**.

> **Treat this like an IRL whiteboard - not a record of completed things.**
> When an item is done, **erase the row**. Don't add a "Resolved" section,
> don't strike it through, don't leave a "closed" marker. The CHANGELOG is
> the project's permanent record of what was done; this whiteboard's only
> job is to surface what's *still active*. A row's continued presence is
> the signal that it still needs attention. Stale "done" notes dilute that
> signal and bury the rows that actually matter.
>
> Before adding a row: check whether the work is already done elsewhere
> (CHANGELOG, git log, the relevant doc). Before acting on a row: spot-
> check it's still real - agent memory of "this needs doing" is exactly
> the failure mode that drives stale-row accumulation. If you reject an
> item after consideration, log the rejection rationale in CHANGELOG and
> erase the row, don't move it to a "rejected" section.

## GCS glass: live board -> Master Dashboard (NEXT) (2026-09-05)

Desk **Master Dashboard MVP** is up on facsimile (Flexible Layout + on-demand feeder). **Not verified:** piping a live board (station USB/`m` or flight) into the same glass path. That is the next glass sitting - confirm RSSI/baro/phase with real RF, not only Big Daddy CSV. Prefer MET + GPS on both ends for time-sync.

Deferred polish (not blocking board-val): Condition Set + alphanumeric caution tiles (digits + color), Zero static-nginx smoke. LCARS is its own row.


---

## Passive Estes first flight: chute detect (WANTED) (2026-09-05)

Nathan (2026-09-05): **first flight is a passive motor Estes-style rocket** (single chute, **no pyro**). FD today is dual-deploy pyro (drogue/main fire on phase transitions). Need a **passive chute-detect / recovery-phase path** before that flight - post-apogee drag jump / |Vvel| collapse (baro + fusion), not pyro events. oMCT glance CSV uses layout-only synthetic chute_detected until this lands.

Not a license to implement this sitting. Owner: FD / fusion sitting when scheduled.

---

## GCS glass: LCARS skin (WANTED) (2026-09-05)

Desk Open MCT hello-world is on Espresso with stacked RSSI/SNR/Baro + radio overlay. **LCARS skin** (louh/lcars or frame-wrapper) is the next aesthetic pass after more field testing / layout tweaks. Do not start unprompted; themes after ingest was the prior lock and ingest is now desk-proven.

---

## FPV-style scan / find (WANTED) (2026-09-02, restated 2026-09-03)

**Coupled with LoRa Hail** (Prox-1 §6 / Best-effort hail - `starcom/AGENT_WHITEBOARD.md`). Same first-impl slot: station has to acquire a vehicle that is already on some PHY. **Either/or for first implementation; FPV scan won.** Hail is not cancelled - it stays the book/MAC path after find, or if scan is not enough.

Stashed as `wip phy-scan leds docs` with both in one Starcom row - split: **scan/find is RC (this board)**; hail write-up is Starcom.

**Find axis = SF×BW at 915.0 / sync `0x12`.** Firmware-legal **6 SF × 3 BW = 18** cells (not hundreds; do not add US915 64-freq hop). CR/power/nav are not find axes. 18 is FPV-goggle sized (Fatshark ~40 ch). Lock/apply uses `radio_config_nav_fits_hz` (125/10 SF7 refused) — same gate as COMM_CHANGE / NAV_PRESET.

**Dual-use (list stays small):** station tool = (1) find *our* vehicle (valid PLTU) (2) show which cells are occupied (RSSI/CAD) like goggles. Prior art: Hertz-Hunter, MikyM0use OLED-scanner / JAFaR (RX5808 RSSI sweep + autoscan), PortaPack FPV Detect (40-ch AutoScan). Pattern: finite table, RSSI bar per cell, lock strongest / first match. Extra vs analog FPV: CRC’d PLTU vs raw energy. **RX-only while sweeping.**

WIP untracked: `src/safety/station_phy_scan.h`, `test/test_station_phy_scan.cpp` (stash also has scan-bar LED + `ao_radio` tick - do not `git stash pop` into a soak wrap). Not a license to implement this sitting.

---

## Class D outdoor BW rank (WANTED) (2026-09-03)

125 vs 250 vs 500 at +20 dBm, then step down. Only BW-for-range rank. Desk stays 2 dBm. Procedure: `starcom/docs/integration/TWO_BOARD_SOAK.md`. Report: `docs/RADIO_SOAK_PASS_AB_2026-09-03.md`. Not a license this sitting.

---

## Adaptive TX power (WANTED) (2026-09-03)

Owner-wanted after LoRa Pass A. Not hail. Sit after station 2 dBm ELF is on the Jam so SET is not a +20 dBm desk shot. FSK bearer lives on the Starcom board.

---

## Fault beacon last-gasp (HELD)

Held until FPGA PHY work is done. Then **PIO beacon + SPI fallback in one session** — do not SPI-from-fault-handler alone. PIO2 = safety; PIO0 = WS2812; **PIO1 empty on purpose** until this sitting (or a later FSK timing need). `docs/hardware/PIO_BUDGET.md`. Council B.5 / round 3: `docs/decisions/FAULT_RECOVERY_2026-05-14.md`. Not a license this sitting.

---

## Skills to add (OPEN)

Wanted skills - not written yet. Not a license to author them until scheduled.

- **Session-checklist skill** - grant add-only cadence writes (`CHANGELOG.md`, `PROJECT_STATUS.md`) only when that checklist scope is actually running (commit / push / wrap). Stops jumping the gun on a changelog because the list was *read*. If the protected-file hook is revived on Claude Code, this skill drives that gating. (Moved here from the graphify/hook note.)
- **Council skill** - when the user says “council review” / “panel check,” load `COUNCIL_PROCESS.md` (panel, stop conditions, incomplete-review offer) instead of relying on the agent to remember the file.

---

## Starcom

Library flags: [`starcom/AGENT_WHITEBOARD.md`](starcom/AGENT_WHITEBOARD.md). Sequence: [`starcom/docs/IVP.md`](starcom/docs/IVP.md). Product `starcom-v0.2.25`. RC air is always Starcom COP-P. Worktree: `C:\Users\pow-w\Documents\starcom_dev` (`grok/sc-dev`). Nested `starcom/` on `main` matches that tree as of 2026-09-17.

---

## `rp400` git remote = Pi 400 keyboard clone (DEFER) (2026-08-20)

Not a radio chip and not WSL. Git remote `rp400` (`npow@192.168.1.233:~/Rocket-Chip.git`) is an early clone onto the **Raspberry Pi 400** keyboard computer (CYBERDECK HAT/Bonnet on hand - `docs/hardware/HARDWARE.md` Ground Station). Host was off/unreachable 2026-08-20. Local tracking of `claude/tender-banach` was dropped; that branch may still exist on the Pi (Feb 2026 SAD/ESKF, already an ancestor of `main`). **Do not chase it now.** Next time that machine is used - likely Stage 12B Yamcs / OpenMCT / advanced GCS - if the clone has not been fully redone, delete leftover branches there (at least `claude/tender-banach`). CHANGELOG `2026-08-20-004` is the land-time note.

---

## First-flight prod strip of test/inject (OPEN) (2026-08-23)

Current `build_flight` ELF **is still development firmware.** Approach A (inject/debug linked, `test_mode_active()` no-op) is acceptable until first flight.

**Before first flight:** a dedicated production image that **omits** test/inject TUs from the link - not “compiled in and gated,” not “ifdef in the tree but we pinky-swear DEBUG is off.” `fault_force_*`, debug mutators, station inject, and the arm gate must not be in that ELF (`nm` / `strings` check). Mechanism (prod CMake preset vs stripped tree) is picked in that sitting, not now.

Does not reopen sitting 11. Does not strip on `main` until that sitting.

**Concerns:** Probe residual power (E2) if the board looks dead after SWD.

---

## Notify / LED system overhaul (OPEN) (2026-08-24)

AO_Notify + `led_patterns.h` + AO_LedEngine need a dedicated sitting, not more overlay nits.

**Known split to keep:** Stage L ARMED is **red solid** (APM2 LED A / traffic-light “motors live”). Pixhawk RGB standard is **solid green** with GPS 3D / **solid blue** without. Do not flip ARMED to green in overlay remediates.

Also in that sitting: `kLedPhaseFault` dropped (`347f0a4`) - 28 is AP pre-arm yellow double-flash only; failsafe/EKF stay Notify `FaultIntent`. USER_GUIDE ARMED is red solid (R-30).

---

## Safety/ops criticality inventory (OPEN) (landed from walk WB W-15)

Optional project-wide map of **things the system does** (Go/No-Go, launch abort, pyro intent, confidence gate, ESKF healthy, FD HSM, …) -> owning files/APIs. Review priority / gate scope / doc SSOT. Not a C++ or build tier.

**WN tie is weak.** **WN-182**’s real claim is Go/No-Go SSOT; the inventory is an explicit owner tangent. **WN-184** is a load-bearing comment-vs-type contract and only points at W-15 as safety-adjacent. **Do not block** disposing those WNs on creating this list. Seeds if/when built: WN-182, WN-142, WN-172, WN-176, ESKF brake, fault recovery.

---


## Local-LLM try-later shortlist (OPEN)

Not adopted. Detail: `docs/tools/LOCAL_LLM_COMPANION_RESEARCH.md` §5. WSL `.wslconfig` is **48GB** (Cookbook ~47 GB after refresh).

Cookbook scan-row Download for Devstral hits **official** `mistralai/Devstral-Small-2-24B-Instruct-2512` (no GGUF). Use Direct Download: `unsloth/Devstral-Small-2-24B-Instruct-2512-GGUF`.

- **Devstral Small 2 24B** - owner trying **Q8_0** first (`…-Q8_0.gguf`, ~23 GB). Q4 later for A/B. Do not pull the whole Unsloth repo.
- Qwen3.6-35B-A3B
- Gemma 4 31B QAT-Q4_0 (`google/gemma-4-31B-it-qat-q4_0-gguf`, not Cookbook’s Q4_K_M row)
- Nemotron 3.5 Lightning 30B

---

## IEEE 1028 review-level -> decision-table mapping (PROPOSED / DEFERRED) (2026-07-04, Claude/Opus)

IEEE Std 1028-2008 recorded as a **review/audit-process** reference in `standards/AUDIT_GUIDANCE.md` Appendix B.5, kept deliberately distinct from the JSF/P10/JPL **coding** standards (how we review ≠ how we write). **Provisional, not a sole standard:** useful but broad, lightly vetted so far - complementary review standards may join it later; this is a starting point, not a settled adoption. **Open rework:** the "When to Do What" decision table in AUDIT_GUIDANCE.md sets review *scope* per trigger but leaves review *depth* implicit. IEEE 1028 names five review levels - management review, technical review, **inspection**, walk-through, audit. Map those onto the 7-tier procedure so each trigger states which depth applies (e.g. the L2-P5 manual walk = an **inspection** per Appendix B.4 "Phase 9"; a small change = a walk-through). Deferred - do when the audit procedure is next reworked. Owner decision on priority.

---

## Use Cases
1. **Cross-agent review** - Flag concerns about other agents' work (see `CROSS_AGENT_REVIEW.md`)
2. **Cross-context handoff** - Notes for future Claude sessions when context is lost
3. **Work-in-progress tracking** - Track incomplete tasks spanning multiple sessions
4. **Hardware decisions pending** - Flag items needing user input before code changes
5. **Deferred items** - Active intent kept visible until acted on (and then erased - see header rule)

---

## Medium (session-scale, 4-12 hours)

Scope is clear but touches multiple files, needs verification, or has small design questions.

- **QP/C naming-convention divergence - TRACKED (2026-06-24, Claude).** QP/C-vs-QP/C++ eval closed: stay on QP/C (`docs/decisions/QP_C_VS_QP_CPP_2026-09-07.md`). The L2-P5 naming pass renamed the project's AO/QP code from Samek/QP house conventions to the project's JSF house standard: QP **`l_`** module-static prefix -> `g_` (JSF-209/CODING_STANDARDS:469 static convention); QP state-handler **`Xxx_initial`/`Xxx_running`** CamelCase -> `lower_case` (JSF AV Rule 51); QP **`s_evt`/`tx_evt`** event statics -> `g_`-prefixed. **Why this is safe (researched 2026-06-24, primary sources):** (1) QP/C consumes these as *function pointers* (`Q_STATE_CAST(&FdAo_initial)`) and *variable identifiers* (`&l_fdAo.super`) - names are arbitrary to the framework, only signatures + registration matter; (2) the AO code is **hand-written** (no `.qm` model, no QM-generated banners) so nothing regenerates QP names back; (3) Quantum Leaps publishes their own coding style (QL-C/C++:2022) as **editable markdown explicitly meant to be forked/customized** to a project's house standard. So QP naming is *guidance, not a hard line*, and JSF governs (no accepted-deviation needed). **Watch-items down the line (the reason this is tracked):** (a) QP forum/book examples + `docs/decisions/AO_COMMANDMENTS.md` use Samek naming - a QP-veteran reading our AO code sees house naming instead; (b) **if QM (the QP modeling tool) is ever adopted**, its generated code reimposes QP conventions -> the divergence would resurface as a generated-vs-house conflict (revisit then); (c) stay-on-QP/C is recorded in `docs/decisions/QP_C_VS_QP_CPP_2026-09-07.md`. **Formalization TODO:** record this as an "Identifier naming (QP/Samek vs JSF)" bullet in `CODING_STANDARDS.md` -> "Worked consolidation decisions" (alongside the function-pointer P10-vs-JSF and nullptr-vs-JSF-175 resolutions) - that file is PROTECTED, so needs repo-owner to name it for editing.

- **Station SPIN model extensions.** Scaffolding landed (IVP-147: P_TERMINATION + P_NO_DOUBLE_CLEAR, both PASS). RadioScheduler is gone (always-on COP-P). Extend when firmware lands: multi-pending-in-flight, MAVLink parser state, `station_idle_tick` GPS poll interleave.

## Large (multi-session, architectural)

Needs council review or planning doc before starting.

- **Real-World Accuracy Tests plan.** Bench-side ground-truth validation - IMU known-angle tilts, baro altitude vs reference, GPS stationary/moving baseline characterization, ESKF replay vs synthetic truth, Allan variance for gyro/accel. Doesn't need launch window or airframe. Complements Stage 18 field tuning. Needs dedicated plan doc with prior-art research (ArduPilot EKF tuning, PX4 calibration) and equipment assessment.
- **Launch procedure audit items.** Six future safety items from NASA/SpaceX/NAR procedure comparison, all requiring Mission Profile or hardware support:
  1. Angle-rate abort guard (BOOST bank threshold -> ABORT; needs IMU attitude in BOOST)
  2. No-pyro-after-impact guard (landing guard before apogee guard -> suppress pyro)
  3. Hung-fire / ignition timeout (track time since ARM, station-side exclusion timer)
  4. Igniter continuity check (station-side pre-arm check)
  5. Air-dropped vehicle profile (altitude-aware abort, no "stay on ground")
  6. Multi-engine / staging support (partial engine light, inter-stage hold, TRA 13-9)

## Research / Deferred

No code this sitting. Unique leftovers only — last-gasp is **Fault beacon last-gasp (HELD)**; FSK/SDLS/SET OTA are the Starcom board.

- **Datasheet RAG.** Index the PDFs we already use (RP2350, SX1276, Pico SDK, CCSDS books) so register lookups are a query. Pointer not authority (LL 37/38). Independent of local vs cloud LLM (see Local-LLM shortlist). Starcom wire codecs already landed; remaining is datasheet/register RAG, not framing.
- **ELRS on RP2350.** Native ExpressLRS / PIO hop vs Telstar CRSF module. Future radio protocol, not this PHY.
- **PIO SM halt is undetectable** (IVP-130 Scenario 5). Accepted Core/Titan gap; mitigation is a second MCU. Gemini-tier.
- **AON-timer prior-uptime** — stubbed to 0 in the anomalous-boot confidence gate. POWMAN already covers brownout; wire `pico_aon_timer` only if auto-zero-baro false-positives need a second corroborator. `docs/decisions/FAULT_RECOVERY_2026-05-14.md`.
- **Battery ADC.** Hardware not wired. ADC pin + driver + telemetry field.

## Far-future

Mission Profile OTA, F' evaluation, u-blox GPS, OTA drivers, GPS-free 3D reconstruction, MATLAB export - all tracked in `docs/PROJECT_STATUS.md` future features. FSK bitstream is on the Starcom board.

## Upcoming Stages

**Stage 15: Pre-Flight Polish** - AO responsibility audit (Stage 13 Core1 gap), Audio Output (I2S DAC, ~10-12 IVPs, fills Stage 14 audio backend stub), User Guide, Runtime Behavior Map update for AO architecture, defense-in-depth evaluation (Core1 stall checked in 3 places post-Stage-14 - evaluate justified vs. bloat).

**Stage 16: Field Tuning** - All VALIDATE parameters. Needs flight data.

**Stage 17: Field Testing** - IVP-135, 136, 137, 138. Airframe integration, ground test, flight test, exit gate. Needs hardware access and weather. IVP-134 (pre-flight checklist) already committed.
