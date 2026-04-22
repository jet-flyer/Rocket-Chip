# Agent Whiteboard

**Purpose:** Communication across context windows and between agents.

**Stages 1-14 + 16A + 16B + 16C + L COMPLETE.** 755 host tests, SPIN 11/11. Tracking: `docs/AO_ARCHITECTURE.md`. Field Testing (Stage 17) deferred — awaits airframe + launch window. **Stage T in progress** (RadioScheduler timing diagnostics — 4 IVPs of data collection before fix design; plan: `docs/plans/STAGE_T_RADIO_DIAGNOSTICS.md`).

## Use Cases
1. **Cross-agent review** — Flag concerns about other agents' work (see `CROSS_AGENT_REVIEW.md`)
2. **Cross-context handoff** — Notes for future Claude sessions when context is lost
3. **Work-in-progress tracking** — Track incomplete tasks spanning multiple sessions
4. **Hardware decisions pending** — Flag items needing user input before code changes
5. **This is a whiteboard** — Erase completed items. Only keep active flags and deferred work.

---

## Housekeeping (Trivial — do now or on next touch)

*(cleared — `tools/i2c_bare_test/` committed 2026-04-18 as `485260b`; landing-beacon design became Stage L.)*

## High priority

- **build_station/ CMake should auto-pick `adafruit_fruit_jam` when
  `ROCKETCHIP_JOB_STATION=1` is set.** 2026-04-22 session I Frankenstein-
  flashed the station twice because my clean rebuild of `build_station/`
  only passed `-DROCKETCHIP_JOB_STATION=1` and the root CMakeLists.txt
  line 185 defaulted `PICO_BOARD` to `adafruit_feather_rp2350`. Station
  role + Feather pin config = firmware that looks valid via picotool
  but has wrong LED/I2C/SPI pins for the physical board. Fix in root
  CMakeLists.txt: when `ROCKETCHIP_JOB_STATION=1` is set and
  `PICO_BOARD` is not explicitly specified, default to
  `adafruit_fruit_jam`. Also applies to `build_station_flight/`.
  Diagnostic hint for this footgun documented in
  `docs/TROUBLESHOOTING.md`.

- **Deep RP2350 errata sweep + codified watchlist.** 2026-04-22 we tripped
  RP2350-E2 (SIO spinlock mirror writes) as a boot-time deadlock — fix
  was a one-line `PICO_SW_SPIN_LOCKS_NO_EXTEXCLALL=1` that we *hadn't
  applied* because the erratum fell through the cracks during initial
  Pico-2 / RP2350 onboarding. Adafruit warns about E9 (GPIO pad leakage,
  A2-only) on product page but E2 slipped past. We need (a) full pass
  through the RP2350 datasheet errata appendix, (b) a tracked
  `docs/hardware/RP2350_ERRATA.md` listing every erratum with status
  (affected? workaround applied? how?), (c) a reference from
  CMakeLists.txt and CODING_STANDARDS.md so future flag changes are
  audited against the list, and (d) subscribe to RP2350 errata updates
  (datasheet revisions bump on new findings). Our chip is A2 — check
  whether E9 workarounds are needed for any of our GPIO pins (floating
  inputs with pull-downs = leakage).

  **Related SWD-debug quirk (may be upstream-worth):** after SDK
  runtime_init completes on RP2350 with `PICO_USE_SW_SPIN_LOCKS=1`,
  halting the target via `monitor halt` during a `spin_lock_blocking`
  wait appears to lose the exclusive-monitor reservation. On resume,
  STREXB fails forever, the spinlock never acquires, and Core 0
  eventually HardFaults (same PC=0xeffffffe / LR=0xffffffe9 lockup
  signature). Workflow workaround: always follow `monitor halt` with
  `monitor reset halt` + `load` + `monitor resume` rather than bare
  `monitor resume` when debugging past a spinlock wait. File as part
  of the errata sweep — may warrant upstream pico-sdk issue citing
  #2495 / #2706 line of investigation.

- **Remove watchdog reboot entirely in favor of safe-mode.** 2026-04-22
  bench test on vehicle after Stage T Batch B surfaced `[WARN] WATCHDOG
  REBOOT` in the boot banner — the reboot path is still live. Design
  intent (from prior stages) was to replace MCU reboot with safe-mode
  entry so the vehicle fails *in place* with state preserved for
  diagnosis, not rebooted silently. Audit `src/safety/watchdog_*`,
  `watchdog_recovery.*`, any `watchdog_enable*` callers, and the
  boot-reason decode in `main.cpp`. Remove every `watchdog_reboot`
  code path; replace with `watchdog_recovery_enter_safe_mode()` or
  equivalent. Update LL / docs / SPIN model if reboot was referenced.
  **Why high priority:** silent reboots mid-test destroy diagnostic
  state and hide the root-cause bug.

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

## Easy (1–3 hours, clear scope)

*(cleared — the IVP-132a.4c red-flash was called a transient 2026-04-18 after a 31-min idle soak showed zero recurrence: 1.86M IMU reads, 0 errors, 0 baro errors, Core 1 healthy, MCU temp 35.6°C stable. Purple at 5+ min is `kSensorNeoTimeout` as designed.)*

## Medium (session-scale, 4–12 hours)

Scope is clear but touches multiple files, needs verification, or has small design questions.

- **Portable pre-commit hooks.** Port the bench_sim gates (vehicle + station) from `.git/hooks/pre-commit` into a tracked `scripts/hooks/pre-commit` + install script so new clones get the gates automatically. Currently hooks are per-clone — fresh LL-Entry-36-class rot gap.
- **Role-aware bench_sim gate.** The pre-commit bench_sim trigger is file-name grep based. Station-only changes to e.g. `src/safety/health_monitor.cpp` that are `if constexpr` compile-outs on vehicle shouldn't require vehicle-probe verification. Needs classification/role-scoped trigger logic.
- **MAIN_DESCENT timeout fallback (SPIN P7 fix).** Add `descent_timeout_ms` to `MissionProfile` (e.g., 600s rocket, longer for HAB). Fallback in `state_main_descent` SIG_TICK handler: elapsed > timeout → auto-LANDED. Update SPIN model — P7 liveness will pass with weak fairness. Currently the only flight phase with no timeout fallback; HAB float profile also affected.
- **`standards/HW_GATE_DISCIPLINE.md` + checklist amendment.** User ask 2026-04-17. Gate definitions must name a positive-control signal (e.g., "DAC 0x18 ACKs" proves bus health separately from device-under-test), a 3-boot reseat protocol, and require commit messages to cite the observed control signal — not just "build clean + MSP stable". Motivated by IVP-140 false-positive + Fruit Jam GPS cable episode. Related: LL Entry 25, 36.
- **Station SPIN model extensions.** Scaffolding landed (IVP-147: P_TERMINATION + P_NO_DOUBLE_CLEAR, both PASS). Extend when corresponding firmware behavior lands: multi-pending-in-flight, RadioScheduler TX-window arbitration (needed for the sync-gap fix), MAVLink parser state, `station_idle_tick` GPS poll interleave.
- **Station role / board decoupling.** "Station mode" (`ROCKETCHIP_JOB_STATION=1`) is still entangled with Fruit Jam specifics: `PICO_BOARD` hardcoded via environment in `build_station/`, "Fruit Jam" in print strings, HSTX/DVI output assumptions, Fruit-Jam-only pin allocations not gated. Grep `"Fruit Jam\|fruitjam"` in `src/`, gate with `#ifdef PICO_BOARD_*` or move to `board_*.h`. Tiny 2350 port will exercise this.
- **Full AO audit against `docs/decisions/AO_COMMANDMENTS.md`.** Prelim for Stage T Batch B did only a spot-check sufficient to confirm no LL Entry 32/35-class violations block Batch B (findings: 3 documented blocking-in-handler deviations in `ao_rcos.cpp` / `ao_radio.cpp` with in-code rationale; all `QACTIVE_POST` use static events). A thorough audit across all 7 AOs — Commandments I-XII each checked, violations ranked by severity, remediation plan per violation, decision per violation whether to fix or document-as-deviation — hasn't been done. Should happen post-Stage-T, before any new AO is added (e.g., future dual-mode / Batch B+ additions). Promotion of `AO_COMMANDMENTS.md` to `standards/ACTIVE_OBJECT_RULES.md` (JSF-AV-grade normative) is a possible outcome.

## Large (multi-session, architectural)

Needs council review or planning doc before starting.

- **RadioScheduler TX-window sync — Stage T (plan: `docs/plans/STAGE_T_RADIO_DIAGNOSTICS.md`).** Renamed from M → T on 2026-04-18. Stage T is **diagnostics only** (4 IVPs: T1 baseline + failure classification, T2 cheat-mode upper-bound, T3 protocol A/B, T4 CAD feasibility + ambient RSSI). Fix plan comes in a second council session once T1/T2/T4 CSVs land in `logs/stage_t/`. Station commands only match ACKs 6.7% of the time (IVP-132a.5: 2/30 first-try, 27 went through all 3 retries). TX hardware fine (`tx_consec_fail = 0`). Candidate fixes being evaluated: CCSDS COP-1 + CLCW (leading), SX1276 CAD listen-before-talk, vehicle-side RX-window announcement, hybrid. Load-bearing for reliable station→vehicle command path.
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
- **Station bench_sim + SPIN model rot detectors.** Both landed this session (IVP-146, IVP-147). Extensions tracked above under Medium (Station SPIN) and Medium (portable hooks).

## Deferred (near-term, post-Stage 15)

- **Battery ADC monitoring.** Hardware not wired. ADC pin + driver + telemetry field.
- **CCSDS SDLS command authentication.** Telecommand auth for Rocket profile.

## Far-future

Mission Profile OTA, F' evaluation, u-blox GPS, OTA drivers, GPS-free 3D reconstruction, FSK bitstream, MATLAB export — all tracked in `docs/PROJECT_STATUS.md` future features.

## Upcoming Stages

**Stage 15: Pre-Flight Polish** — AO responsibility audit (Stage 13 Core1 gap), Audio Output (I2S DAC, ~10-12 IVPs, fills Stage 14 audio backend stub), User Guide, Runtime Behavior Map update for AO architecture, defense-in-depth evaluation (Core1 stall checked in 3 places post-Stage-14 — evaluate justified vs. bloat).

**Stage 16: Field Tuning** — All VALIDATE parameters. Needs flight data.

**Stage 17: Field Testing** — IVP-135, 136, 137, 138. Airframe integration, ground test, flight test, exit gate. Needs hardware access and weather. IVP-134 (pre-flight checklist) already committed.

---

## Resolved This Session (2026-04-18)

Short-lived items cleared from the live board — full detail lives in CHANGELOG / PROJECT_STATUS / docs.

- **Stage 16C COMPLETE.** IVP-139 through IVP-145. Station runtime decoupling via capability-masking (`job::kRoleSamplesCore1` / `kRoleRunsLogger` constexprs), Tiny 2350+ / Pico 2 scaffolding, station HealthMonitor parity, `cmd_station_*` audit (clean). 4 builds clean, 724 host tests, bench_sim 2/2, vehicle+station 5-min soaks PASS.
- **IVP-146 Station bench_sim landed.** `scripts/station_bench_sim.py` — 3 tests (boot/N-A/health) + QP assertion scan + VID:PID port detection. Pre-commit hook extended locally (portability tracked above under Medium).
- **IVP-147 Station SPIN scaffolding landed.** `tools/spin/rocketchip_station.pml` with P_TERMINATION (liveness) + P_NO_DOUBLE_CLEAR (safety), both PASS. Extensions tracked above.
- **IVP-142c HealthMonitor parity done.** Shared code branches on capability, not role identity.
- **MCU die temperature.** Already landed — `src/drivers/mcu_temp.cpp`, wired into `health_monitor.{cpp,h}` and host tests. Flagged in whiteboard as pending but was already done.
- **SPIN drogue-on-abort gate + comment fix.** `rocketchip_fd.pml` BOOST and COAST abort branches now model both `MissionProfile::abort_fires_drogue_from_boost/coast` settings non-deterministically. All 7 FD safety LTL properties still pass (errors: 0). Comments at `flight_director.cpp:350,377` updated to "fires drogue if profile flag set".
- **Pre-commit Ground-code whitelist.** `.git/hooks/pre-commit` now skips `src/cli/**` in the function-size / complexity gate. JSF-AV thresholds applied only to Flight-Critical and Flight-Support code per CODING_STANDARDS.md classification table.
- **CMake `src/cli/rc_os.cpp` "duplicate" — false alarm.** Both appearances (lines 290, 434) are intentional: one in `add_executable(rocketchip …)` source list, one in the `ROCKETCHIP_SOURCES` variable that scopes `-Wpedantic` to our sources (line 471). Not a dedup bug. Whiteboard was wrong.
- **Host test audit — clean.** 35 test files (~12.6K LOC). No disabled tests, no GTEST_SKIP, no TODO/FIXME markers, no commented-out cases. The 9 ESKF test files (bierman, gps_update, mag_3axis, mag_update, propagation, reset, update, zupt) each cover distinct APIs — not merge candidates. Test infrastructure (replay harness, synthetic data, CSV reference logs) healthy. No cleanup needed.
- **`/tmp/stage-j-test` worktree removed.** `git worktree remove --force` — only had untracked `build_fruitjam/` artifacts.

## Resolved (Historical — Recorded Elsewhere)

| Item | Recorded In |
|------|-------------|
| All Stage 1-12A completion details | `docs/PROJECT_STATUS.md` completed table |
| Stage 7 IVP-57-65 details | CHANGELOG 2026-04-07/08, PROJECT_STATUS |
| AO/State Engine Logging audit | `docs/ADVANCED_SETTINGS.md` |
| PCM frame expansion | `docs/ADVANCED_SETTINGS.md` Research Mode |
| DPS310 baro rate + read-count fixes | CHANGELOG, baro driver code |
| SRAM execution audit, Dense FPFT, UD factorization benchmarks | `docs/benchmarks/` |
| Bierman measurement update | CHANGELOG, ESKF code |
| MMAE/IMM pivot | `docs/decisions/ESKF/ESKF_RESEARCH_SUMMARY.md` |
| 24-state ESKF expansion + Codegen FPFT (IVP-47) | CHANGELOG, PROJECT_STATUS |
| BSS/codegen sensitivity disproved | `LESSONS_LEARNED.md` Entry 27 |
| VALIDATE values inventory | `docs/UNIQUE_COMMENT_ITEMS.md` |
| Power optimization notes | Codegen mandatory (benchmarked). Revisit if state count > 24 |
| clang-tidy + lizard CCN | `standards/STANDARDS_AUDIT_2026-03-26.md` |
| Job/Mission naming | Code uses `job.h` namespace. Settled. |
| ivp62-wip branch | Deleted 2026-04-09 |
| IVP-103 Station GPS Push | `cmd_station_gps_push()` at `rc_os_commands.cpp:1218`, bound to `p` |
| IVP-132a.4a DIO0 IRQ test | Removed 2026-04-16 — firmware polls RegIrqFlags, no GPIO IRQ |
| QP static events (LL Entry 35) | LedEngine + Notify fixed, detection pattern in LL |
| RP2350 XIP cache vs SRAM | Codegen `.time_critical` section, LL Entry 30 |
| Station GPS hardware (STEMMA QT cable) | Cable swapped; `board_release_peripheral_reset()` + ultra-early GPS init + `[DBG ] GPS early-init:` instrumentation landed |
