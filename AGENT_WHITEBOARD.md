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

- **Audit-infrastructure follow-up session (post-2026-05-13 cycle) — NEXT SESSION.** The 2026-05-13 audit cycle closed (CHANGELOG entries `2026-05-13-003` + `2026-05-13-004` — Grok independent verification CLOSED Tier 5/6/7). Next session fills the audit-tooling script gaps. Bounded scope per user direction 2026-05-13 end-of-session.

  **Scope for next session (script-tooling only):**
  - **F-2026-05-13-001** — Build `scripts/audit/pre_commit_fixture_test.sh` (or equivalent). Synthetic staged-diff fixture exercises `scripts/hooks/pre-commit` against both known-good and known-bad scenarios; asserts the gate matrix produces the expected verdict for each. Wire into Tier 1.3.
  - **F-2026-05-13-002** — Build `scripts/audit/list_bench_sim_pass_tokens.sh` (or equivalent). Greps firmware source for emitted `[FD]` / positive-control tokens; cross-checks against bench_sim's regex constants. Output is an inventory file (`logs/bench_sim_pass_tokens.txt` or similar). Wire into Tier 1.4(b).
  - **F-2026-05-13-003** — Build `scripts/audit/check_toolchain_drift.sh`. Mechanically pulls upstream versions for Pico SDK, picotool, GCC ARM, OpenOCD (Pi fork), Pico Probe firmware, CMake. Compares against locally installed + project-pinned versions. Wire into Tier 2.3.
  - All three are Minor DEFER findings; runtime enforcement unaffected. Out of scope for a fresh session: any production-source firmware changes, audit re-walks, or surfaced-bug expansion (HW_GATE Rule 7 does not apply here — these are script-tooling gaps the audit cycle already disposition'd).

  **Other items (separate, not for next session):**
  - L2-P2/P3/P4 from the 2026-05-07 cycle (sampling policy / citation inventory / scope language) — audit-policy doc edits, batch with whichever later cycle picks them up.
  - R-22 (warm-reboot audit script), R-24 (boot-parity gate flash + verify-banner extension) — both require probe + HW; separate session.
  - CLA-RBM re-collection: triggers at the 90-day mark (~2026-06-06) per the Tier 4.5 threshold introduced in the 2026-05-13 procedure refactor. Until then, watched-but-not-stale.

  **Exact state for handoff (2026-05-13 session close):**
  - `origin/main` HEAD = `1a32103` ([Claude] Merge Grok Tier 5/6/7 independent verification: cycle CLOSED).
  - All 4 build tiers compile clean (`-Wpedantic` on full `ROCKETCHIP_SOURCES` post-F-2026-05-13-004 fix).
  - Host ctest 794/794 PASS.
  - SPIN_OK_31 (6 models, 31 LTL properties).
  - Master audit `docs/audits/MASTER_STANDARDS_AUDIT_2026-05-13.md` CLOSED — Tier 7 disposition complete via Grok independent verification; no audit re-open required.
  - 3 script-tooling findings (F-2026-05-13-001/002/003) DEFERRED to the next session named above.
  - No HW work was attempted this session (pure-doc + 1 CMakeLists.txt edit + Grok merge).

- **Other deferred audit-cycle items (separate sessions):**
  - R-5 (full stdio.h removal) + R-2 (absorbed into R-5) deferred for dedicated ETL session.
  - R-23 (vehicle bench tier HardFault) blocks future Tier 5.4 + 4.3 vehicle scenarios until R-25 deprecation evaluation chooses path.
  - Architecture session: R-20 (Core 1 boot-wait AIRCR), R-21 (PIO WDT no auto-reset), L2-W1 — all feed "in-flight fault recovery architecture" thinking already on this whiteboard.

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

- **In-flight fault recovery architecture — dedicated future session.** Surfaced 2026-05-12 during R-3 (capture-then-reset hardfault handler) verification on the bench. R-3 ships partially validated (single-cycle bench evidence; 3-boot reliability blocked on the Core 1 SIO_FIFO_IRQ post-AIRCR-reset wedge — see below). The deeper architectural questions to evaluate together in a focused session:

  **1. Is "reset" even the right recovery primitive for in-flight faults?**

  User insight 2026-05-12: an in-flight reboot loses the ESKF datum (gyro bias, baro reference, pressure→altitude integration history, attitude-quaternion state). Without datum the rest of the flight is useless — the ESKF would have to re-converge against a sensor stream that's already mid-flight, with no ground reference, no still period for gyro bias estimation, etc. **A degraded safe-mode (keep running with FAULT health bit, GO/NO-GO blocks ARM, but ESKF/baro/IMU keep going) is likely better than reset for in-flight faults.**

  This means R-3's "capture → reset → safe-mode-on-next-boot" pattern is **right for ground/bench debugging** (where reset is fine and the FAULT latch is useful information for the operator) but possibly **wrong for in-flight faults** (where reset destroys the data we need to land safely). The fault handler should ideally distinguish the two and pick the right primitive — capture-and-safe-mode-in-place for in-flight, capture-and-reset for ground.

  **2. Persistence of recovery-critical state across reboots.**

  If reset IS the right primitive sometimes, the state needed to keep the flight useful (gyro bias, baro datum, ESKF state, current flight phase) needs to survive the reset. Options:
  - **`.uninitialized_data` SRAM** — survives AIRCR reset (already used for crash record), wiped at power-off. Could hold the ESKF state + sensor calibration. Risk: requires checksum/version-tagging so a corrupted reset doesn't load garbage.
  - **Flash** — survives power-off, but flash writes during flight risk LL Entry 4/12/31 patterns (USB/I2C breakage). Only safe in narrow non-flight windows.
  - **PSRAM with battery backup** — out of scope for current hardware.

  **3. PIO watchdog as the independent check + safe-mode trigger.**

  User insight 2026-05-12: the existing PIO watchdog (`src/safety/pio_watchdog.cpp`) might be adequate as the independent check for faults the ARM can't detect itself. The PIO sets IRQ flag 0 if not fed; ARM observes this and **transitions to safe-mode IN PLACE — no reset required.** This sidesteps the AIRCR/SIO mess entirely AND preserves in-flight datum. Worth evaluating whether PIO+safe-mode covers the threats we care about (vs needing a true reset path for some specific scenarios).

  **4. Per IEC 61508 HFT=0 framing**, single-channel single-MCU systems "by definition have no ability to tolerate faults" — what we have is fault detection + safe-mode pivot, not true fault tolerance. This is the right safety class for hobbyist/educational rocketry (matches Featherweight, Altus Metrum, MissileWorks practice). Internal MCU watchdog has limited fault-coverage value (*"the watchdog is part of the possible defected microcontroller"*); the PIO watchdog is closer to the IEC 61508 ideal because it's a separate silicon block with different failure modes.

  **5. Proper aerospace-grade fault handling is multi-MCU board work (Rocket Chip line).** User insight 2026-05-13:

  - **Gemini** (dual-MCU carrier board, project's existing concept for the Rocket Chip line) is the canonical multi-MCU answer for rocketry.
  - **Titan** could also adopt multi-MCU if wired in — e.g., onboard pads for a Core Rocket Chip module to plug into as a dedicated GPIO + watchdog + safety co-processor. Reuses an existing module (Core), gives Titan its independent observer per IEC 61508 (separate silicon, separate clock domain, independent of the main flight ARM), and keeps the design modular.
  - **Pegasus FC** (separate side-project per `docs/PROJECT_STATUS.md` "Side Projects & Future Product Lines" — full FC for drones, glide-back boosters) is independent of the Rocket Chip safety design and chooses its own architecture. The Core-as-co-processor pattern *could* be reused on Pegasus if useful, but that's Pegasus's call and outside this audit's scope.

  When such Rocket Chip multi-MCU hardware exists, that's where cross-checking + voting + redundant-reset architecture gets implemented + validated against the IEC 61508 HFT≥1 framing. Until then, Core/Titan single-MCU stays at HFT=0 and uses PIO+safe-mode as the best-available single-MCU pattern. Don't try to graft multi-MCU semantics onto single-MCU Core — wait for the hardware actually designed for it.

  **Surfaced limitation — Core 1 SIO_FIFO_IRQ post-AIRCR-reset wedge.** When `crash_record_capture()` triggers AIRCR.SYSRESETREQ on Core 0, Core 1's NVIC SIO FIFO IRQ pending state survives the reset (AIRCR is processor-only on RP2350 per datasheet §7.3.1; not chip-level). Post-reset Core 0 reboots and calls `multicore_launch_core1()` which DOES NOT reset Core 1's NVIC (it only manages Core 0's). When Core 1 starts, the pending SIO FIFO IRQ fires before any handler is installed (window between bootrom handoff and our `core1_entry()` first lines), vectoring to `isr_invalid` → wedge. Tried fixes that didn't work: drain SIO from handler (only drains Core 0's inbound), drain SIO + clear NVIC pending in `core1_entry` (too late — IRQ already fired), no-op handler installation (conflicts with `multicore_lockout_victim_init`'s exclusive-handler assertion). The clean fix likely requires either: (a) modify the SDK Core 1 boot path to clear NVIC pending earlier, (b) use a chip-level reset mechanism instead of AIRCR (PSM-based — but Core 0 can't trigger that for itself + Core 1 atomically), or (c) approach via the PIO watchdog → safe-mode-in-place path that doesn't require reset at all.

  **Action when this session opens:** evaluate (1)-(5) above as a coherent design question. The right output is probably "Core/Titan uses PIO watchdog + safe-mode-in-place as the primary fault recovery; the capture-then-reset path stays as a ground/bench debugging tool with explicit operator awareness; persistent state in `.uninitialized_data` is opt-in for specific subsystems that benefit." Multi-MCU evaluation is Gemini-tier scope.
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
