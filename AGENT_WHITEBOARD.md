# Agent Whiteboard

**Purpose:** Communication across context windows and between agents.

**Stages 1-14 + 16A + 16B + 16C + L COMPLETE.** 755 host tests, SPIN 11/11. Tracking: `docs/AO_ARCHITECTURE.md`. Field Testing (Stage 17) deferred — awaits airframe + launch window. **Stage M planned** (RadioScheduler TX-window sync, load-bearing for reliable station→vehicle commands — see `docs/IVP.md`).

## Use Cases
1. **Cross-agent review** — Flag concerns about other agents' work (see `CROSS_AGENT_REVIEW.md`)
2. **Cross-context handoff** — Notes for future Claude sessions when context is lost
3. **Work-in-progress tracking** — Track incomplete tasks spanning multiple sessions
4. **Hardware decisions pending** — Flag items needing user input before code changes
5. **This is a whiteboard** — Erase completed items. Only keep active flags and deferred work.

---

## Housekeeping (Trivial — do now or on next touch)

*(cleared — `tools/i2c_bare_test/` committed 2026-04-18 as `485260b`; landing-beacon design became Stage L.)*

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

## Large (multi-session, architectural)

Needs council review or planning doc before starting.

- **RadioScheduler TX-window sync fix — now Stage M.** Promoted to dedicated stage in `docs/IVP.md` 2026-04-18 during Stage L exit. Station commands only match ACKs 6.7% of the time (IVP-132a.5: 2/30 first-try, 27 went through all 3 retries). TX hardware fine (`tx_consec_fail = 0`) — root cause is station TX timing not synchronized to vehicle RX windows. Proposed fix: "listen-before-talk" — station posts `SIG_RADIO_TX` in `handle_rx_packet()` when a pending command exists. Load-bearing for any reliable command path; blocks end-to-end verification of Stage L beacon roundtrip and any future GCS-initiated command. Needs plan + council review.
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
