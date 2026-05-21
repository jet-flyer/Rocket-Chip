# Changelog

## Format
`### YYYY-MM-DD-NNN | Author | tags`

Files affected in parentheses if relevant.

**Frequency:** Typically one entry per session, not per individual change. Log when a task is completed or work transitions to a new focus. However, if multiple *significant* changes occur in one session (e.g., refactoring logging system AND redesigning state engine), create separate entries for each.

**Conciseness is the default.** Most entries should be 1-3 sentences. The entry itself should state *what* changed. If additional context is needed, a brief parenthetical or second sentence suffices.

**Rationale sections are rare.** Only add an italicized rationale block when:
- An unconventional approach was chosen (e.g., experimental driver, workaround for a known issue)
- A decision would appear wrong without context (e.g., why we avoided the "obvious" solution)
- Architectural trade-offs need to be preserved for future contributors

Routine work—even if complex—does not warrant rationale. Bugfixes, documentation updates, configuration changes, and hardware corrections rarely need explanation. When in doubt, omit the rationale.

**Tags:** bugfix, feature, architecture, tooling, hardware, council, documentation, refactor

**History vs drafting:** Prefer not to revise **older unrelated** dated entries (`### YYYY-MM-DD-NNN`) to fix mistakes—that belongs in **a newer entry** unless the change is a trivial typo. **Drafting:** it’s fine to **edit/refine the entry you’re composing in this commit** until it reads right before you ship it—that isn’t rewriting project history.

---

### 2026-05-20-001 | Grok Build 0.1 | bugfix, tooling, hardware

**bench_sim post-run board state hygiene (pre-commit gate).** Added synchronous, time-bounded final reset (using existing `reset_target(to_main=True)`) + post-condition banner check after tests. Goal: reliably leave the attached vehicle in MAIN + IDLE and banner-peekable for the next invocation. (scripts/bench_sim.py)

Earlier version of this entry overstated back-to-back success. Re-verification (same hardware) showed Run 1 cleanup still capable of hitting watchdog after ABORT sequence and subsequent runs failing banner peek. Per 4-person council (JPL Avionics Lead, Embedded Systems Professor, ArduPilot Core Contributor, Cubesat Startup Engineer 2026-05-20): replaced best-effort + daemon cleanup with explicit synchronous post-condition. Further tuning on ABORT path still required.

Verified so far: single runs 2/2 PASS + clean exit on this hardware. Multi-run state hygiene not yet fully closed. (scripts/bench_sim.py)

---

### 2026-05-20-002 | Claude | bugfix, tooling, council

**Bug A: ESKF/Mahony stays UNHEALTHY across FD RESET-to-IDLE — fixed.** The post-RESULT cleanup work in [2026-05-20-001] above was chasing the wrong target. Council 2026-05-20 (NASA/JPL + Professor + ArduPilot + Cubesat, unanimous) traced the failing back-to-back repro to a defect in FD RESET semantics: `state_idle` Q_ENTRY did no subsystem-reset action, so ESKF stayed in its post-flight state (diverged or brake-tripped) and the operator could not re-ARM without power-cycle. Worse silent path: pad-abort auto-IDLE timeout lands in the same state with no operator input that caused it. Fix is a new `reset_subsystems_cb` callback on `FlightDirector`, wired in `ao_flight_director.cpp` to `eskf_runner_request_reinit()` (clears `g_eskfInitialized` + `g_mahonyInitialized` + `eskf_reenable()`). Covers four IDLE-entry paths: RESET-from-LANDED, RESET-from-ABORT, DISARM-from-ARMED, pad-abort auto-IDLE timeout. The bench_sim cleanup machinery from [2026-05-20-001] reverted in commit c3fc03d — bench_sim returns directly after RESULT without trying to navigate the chip state, because tests themselves work fine in single-invocation and the back-to-back failure was always Bug A + Bug B in firmware. Council split Bug B (CDC wedge during sustained NO-GO output) as separate WB-tracked work; see AGENT_WHITEBOARD.md "Bug B" row. (src/flight_director/flight_director.{h,cpp}, src/active_objects/ao_flight_director.cpp, src/fusion/eskf_runner.{h,cpp}, test/test_flight_director.cpp, scripts/bench_sim.py, scripts/hooks/pre-commit, docs/decisions/FAULT_RECOVERY_2026-05-14.md)

Also in this window: pre-commit hook had a Gate 2 early-exit bug eating Gates 3 + 4 (host ctest + bench_sim HW gate) — fix wraps Gate 2 in `if` instead of `exit 0` so the verification gates run unconditionally. This is the LL Entry 36 / 40 "fail-open when prerequisites missing" pattern; it had been masking the chain that led to Bug A's surfacing. (scripts/hooks/pre-commit)

Comment-density follow-up from R-5 audit (WB row 2026-05-17) shipped: five hot-spot functions trimmed from 30-48% per-function density into the Polyspace healthy band (≤20% per user direction). Multi-paragraph rationale for the kAbort invariant + B.3 checksummed-pair design extracted to `docs/decisions/FAULT_RECOVERY_2026-05-14.md` (retroactive capture). Sister commit earlier this window: LL Entry 41 captured the RP2350B GPIO pad-isolation gotcha + `i2c_bus.cpp` trim (commit 9e74713). (src/active_objects/ao_telemetry.cpp, src/drivers/gps_uart.cpp, src/flight_director/flight_director.cpp, src/drivers/i2c_bus.cpp, .claude/LESSONS_LEARNED.md)

---

### 2026-05-20-003 | Claude | bugfix, council, safety

**Bug B was NOT a CDC drain issue — was a Critical PIO program lifecycle asymmetry in `pio_backup_timer.cpp`. Latent since IVP-89 (commit `e1f0438`).** Backup pyro deployment silently dead on any second-or-later ARM cycle, followed by Core 0 wedge on second DISARM. Tier-1 pyro safety regression for the pad-abort-then-re-arm operational path. Fixed (commit `860c3c9`). The [2026-05-20-002] entry above named "Bug B" as a deferred CDC-drain issue per the 4-person council's initial framing — that framing was wrong; GDB backtrace revealed the actual root cause within minutes. Two coupled defects: (1) `pio_backup_timer_disarm()` called `pio_remove_program` (paired with init, not arm/disarm), tripping the SDK's metadata-bitmap assertion on second disarm; (2) `disarm()` reverted pins to SIO LOW but `arm()` didn't restore PIO function on re-arm, silently no-op'ing pin drives. Fix removes the disarm `pio_remove_program` (program stays loaded for chip lifetime, idiomatic pico-examples `hello_pio` pattern) and re-calls `backup_timer_program_init` in `arm()` to restore pin function + SM state. Three-cycle bench verification (probe-attached, vehicle flight v0.16.0): `a/l/x/r` repeats three times, all transitions log cleanly, no SDK assertion, USB CDC stays live throughout. (src/safety/pio_backup_timer.cpp, .claude/LESSONS_LEARNED.md Entry 42, docs/MULTICORE_RULES.md "PIO State Machines" section)

Council micro-review (ArduPilot + NASA/JPL + Cubesat, unanimous, 2026-05-20) identified both defects together; lone-Claude review missed the secondary pin-function bug. Web research against primary sources (RP2350 datasheet §11.7, pico-sdk `hardware_pio/pio.c`, pico-examples `hello_pio`) corrected an additional mental-model error: `pio_remove_program` is metadata-only — it does NOT zero PIO instruction memory; the `_used_instruction_space` bitmap is what asserts. `docs/MULTICORE_RULES.md` "PIO State Machines" section expanded with PIO program lifecycle rule + pin function lifecycle rule + resuming-a-halted-SM guidance + primary source links.

Sub-lesson (LL Entry 42 + memory `feedback_check_gdb_first`): when a USB CDC wedge symptom matches the LL 22/28/31/39 family, check GDB for `__assert_func` / `_exit` / `__breakpoint` on Core 0 *before* building a hypothesis on prior LL entries. The SDK's assertion trap mimics CDC failure closely.

Also in this window: WB cleanup (erased the resolved Bug B row, trimmed the in-flight fault recovery row to just open follow-ups), `docs/CONFIG_TEST_MATRIX.md` Tier 6b updated with the categories-not-enumerations note from LL Entry 40 (closes the doc lag tracked since 2026-05-16), and verified by scan that `ws2812_status.cpp` + `pio_watchdog.cpp` do NOT have the same lifecycle asymmetry (both use correct add-at-init / remove-at-deinit pairing). (AGENT_WHITEBOARD.md, docs/CONFIG_TEST_MATRIX.md)

Verified locally (commit `860c3c9` + `24afa68`): host ctest 869/869 PASS, vehicle + station target builds clean, bench 3 ABORT-RESET cycles all complete cleanly with no SDK assertion and no CDC wedge across all cycles.

Verified locally: host ctest 869/869 PASS (was 863, +6 new Bug A contract tests covering RESET-from-LANDED, RESET-from-ABORT, DISARM-from-ARMED, ARMED-timeout, pad-abort-auto-IDLE, and startup-doesn't-fire-cb negative case); vehicle + station target builds clean; bench (post-flash) cycle-1 ABORT-RESET → wait 6s → cycle-2 ARM accepted with `Platform: 8/8 GO` (was `7/8 NO-GO ESKF UNHEALTHY`). Banner: vehicle flight v0.16.0 (kmenu). All commits in window passed the (now-fixed) pre-commit hook with bench_sim 2/2 PASS.

---

### 2026-05-17-001 | Claude | refactor, architecture, council

**R-5 stdio removal CLOSED at Level-3 — Units F through K shipped.** R-5 plan ([`docs/plans/R5_STDIO_REMOVAL.md`](docs/plans/R5_STDIO_REMOVAL.md)) closed at firmware level (Units F-J) + audit-suite Level-3 regression (Unit K). All `src/*.cpp` are now stdio-free; pre-commit gate upgraded to unconditional rejection of `<stdio.h>`; allowlist file deleted. Council 2026-05-17 (unanimous, parser-extract framing): new `rc::rc_snprintf` + `rc::strbuf` siblings to `rc::rc_log` share one parser, three sinks. Side fixups along the way: cross-checked Grok 4.3's ICM-20948 stuck-slave recovery (`dfabcd4`) with a minor correctness tweak to the post-clock blind writes (4× 1-byte → 2× `[reg,value]` 2-byte — the 27-pulse SCL clocking that actually does the recovery work was unchanged); GPS UART sticky-baud handler (CR1220 backup battery makes the MT3339 baud survive host reset); dashboard hint string + RCOS doc drift; HW_GATE_DISCIPLINE Rule 2 cable-reseat language retired; comment-density audit (aggregate 22.8%, in 15-25% band). Level-3 regression: 6/6 cold-restart bench_sim, 863/863 host ctest, SPIN_OK_31, station dashboard live + RF Link TRACK 100%, vehicle preflight 9/9 GO. PROBLEM_REPORTS R-5 + R-2 both closed; L2-P5 + L2-P10 R-5 dependency cleared. PROJECT_STATUS Future Features gains GPS 10Hz experimental-mode + eyalroz/printf exit-ramp candidate entries.

### 2026-05-16-003 | Grok 4.3 (via OpenCode) | bugfix, hardware

**ICM-20948 stuck-slave recovery.** Added `i2c_bus_imu_recovery()` (extended 27-pulse SCL clocking + blind Bank-0 + device-reset writes) invoked on first probe failure at boot. Allows partial recovery from post-flash no-ACK latch without full Vdd power cycle. Verified: clean build + live GDB-reset + COM7 serial capture (Hardware 13/14 OK, IMU now passes). Complements the documented long power-cycle procedure. (src/drivers/i2c_bus.{h,cpp}, src/main.cpp)

### 2026-05-16-002 | Claude | refactor, bugfix, architecture, council

**R-5 stdio removal Units D + E shipped; meta-policy gate widening; rc_log idle-drain IMU regression fixed.** 8 commits this session.

R-5 Tier 2 (Unit D, 3 sub-commits: `c71090e` gps_pa1010d, `89c1571` gps_uart, `caa9b42` i2c_bus) and Tier 3 (Unit E, 1 commit: `857b573` safety/diag) closed. ~125 callsites migrated, 5 dead-export public functions deleted (FreeRTOS-era), 3 files dropped from `scripts/hooks/stdio_allowlist.txt`. Unit B-fixup #2 (`2749a99`) corrected drain semantics: hold-on-disconnect ring + drop-oldest + `tud_cdc_write_flush()` per Pico SDK `stdio_usb_out_chars` pattern + ring 1KB→8KB per council. Verified via byte-on-wire diff on station for `q→d` diag_stats dump + 3D GPS lock confirmed on vehicle UART path (gps_uart Unit D part 2b also fixed a latent baud-bug — `gps_uart_send_command` early-return-on-`!g_initialized` was silently dropping PMTK251 baud-change, ride-along per HW_GATE Rule 7).

`8cd6368` **rc_log drain ring-empty fast-path** — bisect-confirmed `8adab2d` (R-5 Unit B-fixup wiring `rc_log_drain_to_cdc()` into `qv_idle_bridge`) was disrupting Core 1's ICM-20948 I2C reads via continuous TinyUSB calls contending with the SDK's stdio_usb mutex/IRQ machinery. Fix: 3-line early-return when ring is empty (most idle ticks). Restored `IMU reads=41694 errs=0` + bench_sim 2/2 PASS. LL Entry 39 added.

`a40e7f5` **pre-commit gate widening: "categories not enumerations" (LL Entry 40)** — council unanimous (NASA/JPL + Prof + ArduPilot + Cubesat, 5 questions). FLIGHT_CRITICAL regex in `scripts/ci/pre_commit_matrix.py` widened from ~7 narrow path patterns to any change that can affect rocketchip.elf (`src/`, `include/`, `CMakeLists.txt`, `cmake/`, `EXTERNAL/etl-`, `lib/`) plus the gate self-rot vector (`scripts/hooks/`, `scripts/ci/`, `scripts/bench_sim.py`, `scripts/station_bench_sim.py`). Pure-doc/test/script exempt by virtue of not matching, NOT explicit carve-out. Direct meta-application of R-25-exec's "all code uploaded = flight code" principle to gate scope. `7bcecdb` realigned the host test that encoded the prior narrow policy and added codified anti-regression assertions for the 8adab2d-class path (`main.cpp + rc_log.h`), the three gate-self-rot vectors, and pure-doc exemption.

**WB followups tracked** (`AGENT_WHITEBOARD.md`): doc realignment of `standards/CODING_STANDARDS.md` Code Classification table + `docs/CONFIG_TEST_MATRIX.md` Tier 6b section (descriptive docs lag shipped gate policy per AK Rule 3 surgical scope). Tier-5-drain rate-limiting (LL 39 known-limitation) for when sustained rc_log output enters the picture during CLI tier migration.

**Open issue at session end**: vehicle ICM-20948 (chip serial `02FBDDB8E1CA1281`) entered a stuck-slave state mid-session that survives USB power cycle + survives the shipped `8cd6368` fix. Earlier in the same session the same hardware worked cleanly post-`8cd6368` flash (`IMU reads=41694`); after ~5-10 subsequent flash cycles for the gate-widening work, chip stopped ACKing reads AND writes at 0x69 (DPS310 baro at 0x77 on the same wire continues to ACK, confirming bus electrical health). Diagnostic report prepared for offline analysis; defensive pre-probe write to PWR_MGMT_1 (ArduPilot _hardware_init pattern) prepared but rolled back unverified pending offline analysis. Unit F migration paused at the bench_sim baseline gate; resumes once IMU recovers.

**Verified locally**: vehicle + station + host builds clean at HEAD `7bcecdb`. Host ctest 827/827. Defensive write rolled back; HEAD matches the working state from earlier in the session. The `8cd6368` fix is shipped and was verified working empirically before the bench drifted; current bench state is independent of fix correctness.

**Active plan**: `docs/plans/R5_STDIO_REMOVAL.md`. Tier 4 (Unit F) is the next migration unit; pre-Unit-F council session 2026-05-16 ratified the verification plan (bench_sim 2/2 + SPIN 31 + live `[FD]` byte-on-wire diff + 5-min soak with transition exercised + `rc_log_dropped_bytes==0` + `high_water<4096` assertions). Plan ready for resume once bench is back.

---

### 2026-05-16-001 | Claude | refactor, architecture, council

**R-5 stdio removal Units A + B closed.** Unit A produced the format-spec inventory (`docs/audits/STDIO_FORMAT_SPEC_INVENTORY_2026-05-15.md`) and pre-R5 baseline (`docs/baselines/PRE_R5_BASELINE_2026-05-15.md`). Inventory surfaced a scope correction: actual migration target is 23 files / ~624 callsites (not 18/600 as originally framed) because `include/rocketchip/config.h`'s old DBG_PRINT macros pulled `<stdio.h>` transitively into 5 files (most importantly `flight_director.cpp`'s 10 `[FD]` log lines that bench_sim regex parses).

Unit B landed `rc_log` infrastructure: vendored ETL 20.47.1 at `EXTERNAL/etl-20.47.1/` (acceptance criterion verified — no `<stdio.h>`/`vsnprintf` in format+to_string subtree), built `include/rocketchip/rc_log.h` + `src/log/rc_log.cpp` (~500 LOC hand-rolled printf-subset formatter dispatching to `etl::to_string`), 27 host ctests covering the 13 format-spec patterns + boundary cases (`-0.0`, halfway-rounding, truncation marker, bench_sim-monitored log lines). Repointed `DBG_PRINT`/`RC_ASSERT` macros in `config.h` from `printf` to `rc::rc_log` and retired the `<stdio.h>`/`<cstdio>` transitive-leak includes from `config.h`. HW smoke verified: vehicle banner reads `flight-<hash>` Hardware 14/14 OK byte-identical to pre-Unit-B baseline across 3 probe-driven boots. Binary-size cost +6072 bytes / +2.7% (re-measured at Unit J close per council amendment #3).

**Pivot from plan, council 3-persona Approach A focused review (2026-05-15):** float formatter is hand-rolled, NOT ETL's. Council acceptance criterion #1 surfaced two real ETL divergences from libc — strips sign of `-0.0`, uses round-half-away-from-zero instead of IEEE 754 round-half-to-even. Pathfinder-class systematic divergence risk for ground-side parsers; ETL has no build flag to fix. Pivoted to hand-rolled formatter using C99 `nearbyint()` (FE_TONEAREST = round-half-to-even = matches libc). Float-to-string research (`C:\Users\pow-w\.claude\plans\float-to-string-research-agent-a7c2e9d318f6b40a1.md`) confirmed the scale-and-round approach matches ArduPilot/PX4/F'/Cubesat flight-software practice and is sub-noise-floor for IMU/ESKF data; one documented within-1-ULP edge case (synthetic decimal-halfway literals like `3.785`).

10 commits this session: `8872c14` (T2b Level-2 closure), `6006785` (T3a deprecation), `8c0732e` (Cycle 1 audit catchup), `c6195d1` (Cycle 3+4 stash), `0b02e4b` (Unit A inventory + baseline), `0fe851c` (ETL vendor), `5f2a805` (rc_log infra), `16bc36b` (rc_log brace-fix), `a97c6bf` (migration guide API correction), `7eda5e3` (DBG_PRINT macro repoint).

**Active plan:** [`docs/plans/R5_STDIO_REMOVAL.md`](docs/plans/R5_STDIO_REMOVAL.md). Units C-J (per-tier callsite migration: proving ground → drivers → safety/diag → telemetry/AO → CLI/dashboard → cleanup) remain. Unit K = Level-3 independent regression at allowlist-empty.

---

### 2026-05-15-002 | Claude | architecture, safety, council

**In-flight fault recovery architecture rework (B.1-B.8).** Fault handlers now phase-aware: `kIdle` captures + visible-signal + AIRCR-reset; any flight phase transitions to `kFault` and busy-loops while PIO backup timers continue autonomously. Anomalous-boot gate suppresses auto-zero-baro when POWMAN_CHIP_RESET + flight-in-progress sentinel indicate mid-flight reboot — prevents council-flagged silent-reset-re-zeros-baro failure mode.

Three commits: `ed7c569` (anomalous-boot gate + kFault enum + brownout latch), `8baa18a` (phase-aware dispatch + checksummed pair + reentrance guard + GoNoGo prior-fault stations), `5566050` (doc close-out: WB + LL Entry 38). Commit-(c) SPI last-gasp beacon deferred per council round 3 — merged with future PIO-beacon session.

See: plan `C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn.md` (3 council rounds, 4 research rounds, all unanimous); `AGENT_WHITEBOARD.md` PIO-beacon-and-SPI-last-gasp combined future-session row.

Verified locally: host ctest 800/800, both flight tiers compile clean, HW probe-flash + banner read identical to pre-commit, probe-verified `kIdle` baseline. Level-2 audit regression (T2b + T3a) is the closure path for R-22/R-23/R-24/R-25-exec; expected outcomes locked in by the rework but not yet bench-run.

---

### 2026-05-14-002 | Claude | audit, verification

**R-25-exec audit-suite regression — Tier 0 + Tier 1 + T2a complete at Level-2.** Re-ordered the flat 6-step audit checklist into 4 dependency tiers. T0a (verify_boot_parity), T1a (station_bench_sim 3/3), T1b/T1c (warm-reboot G-W2 + G-W3 via labeled-soft runbook after script flaked on Windows USB CDC re-enum), and T2a (negative-control gated-fault check) all PASS.

Closes R-24 + R-23 at Level-2. R-22 + R-25-exec at partial Level-2 pending T2b/T3a (now unblocked by the 2026-05-15-002 rework).

See: `AGENT_WHITEBOARD.md` "R-25-exec audit-suite regression status" row for full per-tier results + side-findings (script flakiness, station GPS cold-boot slow-start, station fault-inject grep-only coverage).

---

### 2026-05-14-001 | Claude | refactor, architecture, cleanup

**R-25-exec COMPLETE — bench tier collapsed 4→2.** All `src/dev/*` modules either migrated test-mode-gated into the single flight binary (fault_inject, station_fault_inject → `src/safety/`; diag_stats → `src/diag/`; dev_cli → `src/cli/rc_os_debug.cpp`) or deleted (replay_inject, station_replay per amendment #4 — host-side replay TBD). 12 commits, 12 numbered steps + doc sweep, plus 10 stage-archived host scripts and 3 replay scripts deleted. Closes R-22, R-23, R-24, R-25-exec as side-effects.

See: `docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md` for the council-approved plan + 7 amendments; `docs/PROBLEM_REPORTS.md` for R-22/R-23/R-24/R-25-exec closure rows (each cites its commit SHA); `standards/CODING_STANDARDS.md` Code Classification table for the new test-mode-gated entries + the audit invariant grep targets.

Verified: 2 firmware tiers compile clean (-Wpedantic + -Werror=cmse), host ctest 800/800 PASS at every step. HW gate (bench_sim + warm_reboot_audit + verify_boot_parity) runs at cycle-close commit. Flash-budget impact: ~1.7 KB on the flight binary (0.04% of RP2350 4 MB).

---

### 2026-05-13-007 | Claude | audit, cleanup, decision

**Deferred-Cleanup Cycle DC-2026-05-13 Sessions 2 + 3 — R-22 deferred, R-27 fixed, R-25-eval council-APPROVED.** Session 2 (R-22 warm-reboot audit script) hit a flight-tier-firmware design wall after 4 iterations + council redesign; deferred to R-25-exec. Session 3 (R-25-eval bench-tier deprecation evaluation) closed with unanimous council approval of Approach A (single-binary runtime test-mode, probe-only gated — PX4 `SYS_FAILURE_EN` pattern). R-25-exec inherits the council-amended plan as a separate execution session.

**R-27 REMEDIATED:** `docs/ROCKETCHIP_OS.md` main-menu key table stale (claimed `s`/`e`/`b` as Status keys + `i` as Radio key; Stage L 2026-04-18 reassigned `b` to Beacon and moved status keys to q-Debug submenu). Same drift class as R-26. Surfaced during R-22 iteration 4 when the script's main-menu `b` keystroke activated Beacon instead of Hardware Status.

**R-25-eval decision (council unanimous, NASA/JPL + Prof + ArduPilot + Cubesat):** Approach A with 6 amendments + 4 R-25-exec checklist items. Decision grounded in 3 evidence-of-real-bugs from the dual-binary failure mode this cycle (R-23 bench-tier INVPC HardFault, F-2026-05-13-004 pedantic-gate drift, R-22 design wall). User-preferred original direction (Approach B patch-based) rejected after IRL aerospace dev-code-sequestration research showed patch-rot is a known failure mode. Approach C (minimal bench tier) rejected because it retains the same structural problem at smaller scope.

**`CODING_STANDARDS.md` amended (state-of-system trigger-driven edit):** NASA Software Engineering Handbook reference broadened from "§8.11 + §8.5" to handbook-as-whole; specific section citations live at the point where applied.

**R-25-exec inherits:** 13-item plan in `docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md` § "R-25-exec follow-on session." Closes R-22, R-23, R-24 as side-effects. Council-required `peek_banner`-based redesign for R-22 preserved at `C:\Users\pow-w\.claude\plans\snoopy-wibbling-noodle.md`.

Verified: pure-software / doc-only this session. End-of-session sanity check passed (host ctest 794/794, bench_sim 2/2 PASS, SPIN_OK_31). No HW reseat required.

Files: `docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md` (new), `docs/ROCKETCHIP_OS.md`, `docs/PROBLEM_REPORTS.md` (R-22/R-25/R-25-exec/R-27 rows), `docs/plans/DEFERRED_CLEANUP_PLAN_2026-05-13.md` (Session 2 post-mortem + Session 3 close), `standards/CODING_STANDARDS.md`, `CHANGELOG.md`.

---

### 2026-05-13-006 | Claude | audit, cleanup

**Deferred-Cleanup Cycle DC-2026-05-13 Session 1 COMPLETE.** Cat-4 closeout of accumulated DEFER queue from the 2026-05-07 + 2026-05-13 master audit cycles. 5 commits, no source changes, no HW reseat required.

Session ran per council-approved plan in `docs/plans/DEFERRED_CLEANUP_PLAN_2026-05-13.md`:

- **A1 self-check** (open): F-001/002/003 against `origin/main` — 30/30 fixture rows PASS, inventory generated, drift report clean.
- **C4-2 (L2-P9)**: Toolchain version audit 2026-05-13 produced from F-003 mechanical pull. `docs/audits/TOOLCHAIN_VERSION_AUDIT_2026-05-13.md` (new). No new drift vs 2026-04-27 baseline.
- **C4-5 (L2-P2/P3/P4)**: Audit-policy doc edits in `AUDIT_GUIDANCE.md` Tier 3.8 (exhaustive-coverage rule, citation-inventory tooling-gap, scope-language reporting rule) + `STANDARDS_AUDIT.md` template "How to Use" section.
- **C4-3 (R-10b)**: Stack-usage SDK gap re-eval decision recorded in PROBLEM_REPORTS — no new evidence, defer to next milestone close.
- **C4-1**: Archive 16 verified rows (R-1, R-3, R-4, R-6c, R-7, R-9c, R-11, R-12, R-13, R-15, R-16, R-17, R-18, R-19, R-26, P8-FMEA-Pyro) from Active → Archive section in PROBLEM_REPORTS.md. Each cites Grok Tier 5/6/7 merge `1a32103` as the audit-suite-regression that closed it.
- **End-of-session sanity check**: host ctest 794/794, bench_sim 2/2 PASS (vehicle flight v0.16.0 (kmenu) COM7, 6.5s), SPIN_OK_31 across 6 models.

Active PROBLEM_REPORTS table now contains only DEFER rows (R-2, R-5, R-10b, R-20, R-21, R-22, R-23, R-24, R-25 + L2-P5..L2-P11) — 9 R-* + 7 L2-P*.

Next: Session 2 (R-22 warm-reboot audit script + R-24 boot-parity extension, ~3-4 hr HW work) and Session 3 (R-25-eval bench-tier deprecation evaluation, ~2 hr decision-doc).

Files: `scripts/audit/*` (unchanged this commit), `docs/audits/TOOLCHAIN_VERSION_AUDIT_2026-05-13.md` (new), `standards/AUDIT_GUIDANCE.md`, `standards/STANDARDS_AUDIT.md`, `docs/PROBLEM_REPORTS.md`, `CHANGELOG.md`.

Verified: pure-software / doc-only session. Host ctest 794/794, bench_sim 2/2 PASS, SPIN_OK_31. No HW reseat required (Session 1 was no-HW; bench_sim end-of-session check observed all positive-control signals).

---

### 2026-05-13-005 | Claude | audit, tooling

**Audit-infrastructure follow-up: F-001/002/003 REMEDIATED.** Three audit-tooling scripts shipped to close the DEFERRED findings from the 2026-05-13 master cycle:

- `scripts/audit/pre_commit_fixture_test.py` (F-001) — 30-row fixture exercises `scripts/ci/pre_commit_matrix.py`.
- `scripts/audit/list_bench_sim_pass_tokens.py` (F-002) — bidirectional regex-vs-emission inventory for `bench_sim.py` and `station_bench_sim.py`.
- `scripts/audit/check_toolchain_drift.py` (F-003) — mechanical upstream-version pulls vs CMakeLists.txt pins + local installs.

`standards/AUDIT_GUIDANCE.md` Tier 1.3 / 1.4(b) / 2.3 now name the three scripts as the audit-time mechanisms. `docs/audits/MASTER_STANDARDS_AUDIT_2026-05-13.md` findings table retroactively amended (DEFER → REMEDIATED) per `.claude/SESSION_CHECKLIST.md` retroactive-amendment rule (cycle explicitly deferred with expectation of follow-up).

Verified locally: F-001 30/30 fixture rows PASS; F-002 10/11 token classes match firmware emissions ([ESKF] empty + documented as doc-vs-firmware drift); F-003 confirms no NEW toolchain drift vs `TOOLCHAIN_VERSION_AUDIT_2026-04-27.md` baseline. Host ctest 794/794 (no source changes).

Files: `scripts/audit/pre_commit_fixture_test.py` (new), `scripts/audit/list_bench_sim_pass_tokens.py` (new), `scripts/audit/check_toolchain_drift.py` (new), `standards/AUDIT_GUIDANCE.md`, `docs/audits/MASTER_STANDARDS_AUDIT_2026-05-13.md`, `AGENT_WHITEBOARD.md`, `CHANGELOG.md`.

---

### 2026-05-13-004 | Grok | audit, verification, independence

**Tiers 5/6/7 Master Standards Audit complete (independent verification).** Fresh session by Grok (structurally different LLM). Walked all Tier 5 sub-items (pre-flight gate, RP2350 errata, bench/replay, requirements traceability, regression), Tier 6 (F-2 drift, G/H, protected-doc, CHANGELOG), Tier 7 (independent disposition review of 4 open findings).

All verdicts: PASS (or documented PARTIAL for replay gate). 0 new findings. Prior F-2026-05-13-001/002/003 confirmed DEFER (Minor, audit-time only); F-004 confirmed REMEDIATED/CLOSED (Major). Severity gate observed; no Catastrophic/Critical. DO-178C level-3 independence credit achieved per HW_GATE Rule 6 + council amendment #7.

See `docs/audits/MASTER_STANDARDS_AUDIT_2026-05-13.md` (Grok Tier 5/6/7 sections) for full walk + findings table confirmation. Cycle closes. No follow-up session required from this work.

Verified: pure-documentation update (audit report + CHANGELOG), no source changes, no HW reseat required, host ctest baseline unchanged.

Files: `docs/audits/MASTER_STANDARDS_AUDIT_2026-05-13.md` (appended), `CHANGELOG.md`.

---

### 2026-05-13-003 | Claude | audit, architecture, documentation, council

**Audit-coverage gap-fill cycle (steps 0, 1a, 1b, Tiers 1-4).** Procedure refactored 8-step flat → 7-tier dependency-ordered (council-approved with 8 amendments). Tiers 5-7 deferred to a different agent for DO-178C verification-independence. See `docs/audits/MASTER_STANDARDS_AUDIT_2026-05-13.md` for the cycle's findings + handoff to next agent; `AUDIT_GUIDANCE_REWRITE_PROPOSAL_2026-05-13.md` for the council verdict; `AUDIT_COVERAGE_INVENTORY_2026-05-13.md` for the gap-list inventory; `STANDARDS_AUDIT_2026-05-13.md` for the Tier 3 dated companion.

F-2026-05-13-004 (Major) remediated in this commit: 3 production .cpp files restored to the `-Wpedantic` gate. All 4 build tiers rebuilt clean.

Verified: pure-software (CMakeLists.txt + doc edits), host ctest 794/794 PASS, no HW reseat required.

Files: `CMakeLists.txt`, `standards/AUDIT_GUIDANCE.md`, `standards/STANDARDS_AUDIT.md`, `standards/CODING_STANDARDS.md`, `docs/PROJECT_STATUS.md`, `AGENT_WHITEBOARD.md`, `docs/audits/AUDIT_COVERAGE_INVENTORY_2026-05-13.md` (new), `docs/audits/AUDIT_GUIDANCE_REWRITE_PROPOSAL_2026-05-13.md` (new), `docs/audits/MASTER_STANDARDS_AUDIT_2026-05-13.md` (new), `docs/audits/STANDARDS_AUDIT_2026-05-13.md` (new), `CHANGELOG.md`.

---

### 2026-05-13-002 | Claude | audit, architecture, documentation

**Master Standards Audit 2026-05-07 — Phase 8L2 (updated audit) wrapped with findings.** Per-finding record in `docs/audits/MASTER_STANDARDS_AUDIT_2026-05-07.md` § Phase 8L2; PR tracker `docs/PROBLEM_REPORTS.md` updated.

Two source-code fixes implemented + verified (R-19 + R-26). Multiple findings DEFER to future cycles (R-20/R-21/R-22/R-23/R-24/R-25 + audit-policy L2-P2/P3/P4 + post-L2 remediation queue L2-1..L2-20 from cppcheck/lizard/coverage). R-19 (`multicore_reset_core1()` before `multicore_launch_core1()`) closes the AGENT_WHITEBOARD-tracked SIO_FIFO_IRQ wedge from R-3 verification — verified via probe-driven AIRCR test, 3-power-cycle baseline, 5× rapid picotool warm-reboot. R-26 fixes a stale RP2350 datasheet citation (§1.4.3 → §14.9.1) surfaced by Phase 7 exhaustive citation walk.

**Coverage gap acknowledged at L2 close:** L2 (and yesterday's original) ran tool-driven sweeps + a P10 per-rule applicability snapshot (10 rules, all CONFIRMED-status at Phase 2.3) but did NOT walk the **221 JSF AV rules + JPL C LOC-1..4 + project-specific + agent-behavioral** that the original 2026-02-07 audit covered. AUDIT_GUIDANCE.md Step 2 calls for a full `STANDARDS_AUDIT.md` template walk producing a dated `STANDARDS_AUDIT_YYYY-MM-DD.md` companion; yesterday substituted a partial P10 sub-walk for that. Logged as L2-P5 + sibling re-audit gaps (L2-P6 DEV_CODE, L2-P7 VERSION_STRING, L2-P8 AO_COMMANDMENTS, L2-P9 TOOLCHAIN) for a focused audit-coverage-catchup cycle.

**L2 wrap regression:** 4 target tiers compile clean, host ctest 794/794, master SPIN gate SPIN_OK_31, vehicle bench_sim 2/2 PASS, station_bench_sim 3/3 PASS.

---

### 2026-05-13-001 | Claude | audit, architecture, documentation

**Master Standards Audit 2026-05-07 — Phase 8 wrapped (L1 only; L2 deferred).** Continuation of `2026-05-12-001` audit work. Per-step record in `docs/audits/MASTER_STANDARDS_AUDIT_2026-05-07.md`; project-wide PR tracker in `docs/PROBLEM_REPORTS.md`. Disposition table populated in audit report `## Remediation` section.

This session ran the remaining Cat 3 / Cat 4 work + audit-cycle-surfaced cascade + sub-checks + wrap:

- **R-6c — FP-1 template-dispatch (verified).** Extracted LM solver to `src/calibration/lm_solver.{h,cpp}` as pure-function module; replaced function-pointer dispatch with C++ templates; added `test/test_calibration_lm.cpp` host coverage (host ctest 788 → 794). FP-1 moved Active → Resolved in ACCEPTED_STANDARDS_DEVIATIONS.md.

- **R-11 + R-17 — SPIN flash-protocol model + cooperative pause (verified).** R-11 SPIN model (`tools/spin/rocketchip_flash_protocol.pml`, 3 LTL properties) initially failed `p_no_i2c_during_flash` against pre-R-17 firmware — counterexample matched the LL Entry 31 race in micro-detail. R-17 extracts general `core1_i2c_pause()` / `core1_i2c_resume()` primitives in new `src/safety/core1_i2c_pause.{h,cpp}` and wraps every reachable runtime `flash_safe_execute()` callsite. Model + firmware now both pass (master gate SPIN_OK_31, was 28). The `core1_i2c_resume()` clears BOTH atomics so back-to-back flash ops can't observe a stale paused-ack — surfaced by SPIN counterexample, fixed within the R-11+R-17 commit.

- **R-12 — SPIN boot-handshake model (verified).** New `tools/spin/rocketchip_boot.pml` covers vehicle (bounded wait + timeout) and station/relay (Holzmann scheduler exemption) cases non-deterministically. 2 LTL properties verify clean (97 + 164 states, errors: 0). Master gate SPIN_OK_28.

- **R-15 — i2c_bus_reset after CLI flush + erase (verified).** Surfaced during R-11 prep. CLI handlers `cmd_flush_log()` + `cli_do_erase_flights()` were missing the LL Entry 31 prescribed `i2c_bus_reset()` after their flash_safe_execute chains. Added to match the ao_rcos.cpp:338 pattern.

- **R-18 — dead cal_pre_hook cleanup (verified).** Surfaced during R-17 implementation. The `cal_pre_hook()` function in cal_hooks.cpp was DEFINED but never called from anywhere (function-pointer table `rc_os_cal_pre_hook` was assigned at main.cpp:315 but never invoked). R-17 made it fully redundant. R-18 removes the dead function + function-pointer table + simplifies cal_post_hook.

- **R-7 — Holzmann inverted-rule exemption (verified).** Already documented in CODING_STANDARDS.md Foundation section; added single-sentence cross-reference to ACCEPTED_STANDARDS_DEVIATIONS.md "Note on Power-of-10 Rule 2" where the project's compliant non-terminating loops are enumerated.

- **R-13 — SESSION_CHECKLIST trigger row for SPIN model ride-along (verified).** Added one row to the Per-doc trigger map for `tools/spin/*.pml` — when firmware behavior matching a candidate SPIN extension lands, the corresponding `.pml` edit rides in the same commit.

- **R-16 — Systematic LL-entry freshness audit (verified, no fix needed).** Audited 23 Critical/High LL entries. Zero stale entries detected. LL 31 specifically reinforced by R-15/R-17/R-18. LL 25 already SUPERSEDED 2026-04-22. LL 22 (USB reconnect Core 1 IMU rate) remains open observation per its own framing (no flight concern). Recommendation: cadence-driven milestone-close discipline.

- **P8-FMEA-Pyro — pyro state-machine FMEA (verified, no fix needed).** Reviewed 9 multi-transition failure modes (double-drogue, main-before-drogue, pyro-from-IDLE, DISARM-after-fire, ABORT-double-fire ordering, RESET-mid-descent, auto-DISARM-race, hardware-path boundary, double-APOGEE). All mitigated by HSM dispatch semantics. R-14 unified pyro protocol SPIN model NOT queued — not justified given existing rocketchip_fd.pml coverage + single-actor pyro hardware.

- **Comment-density audit (verified, no fix needed).** End-of-cycle audit + NASA/Polyspace research. src/ overall .cpp = 21.8% (within 15-25% target band). CODING_STANDARDS updated with formal target band, research basis (Polyspace 20% lower limit; Arafati & Riehle 2009 mean 19%/median 17%; Elish & Offutt mean 15.2%), headers-excluded measurement method, per-context guidance.

**Cascade chain surfaced this session (Rule 7 in-scope):** R-11 → R-15 → R-17 → R-18 + firmware fix in R-17 commit. Each surfacing was the previous PR's verification path failing to produce its claimed positive-control signal until the underlying issue was fixed.

**Phase 8 wrap L1 regression at commit:** 4 target tiers compile clean, host ctest 794/794, master SPIN gate SPIN_OK_31 (6 models, 31 LTL properties), vehicle bench_sim 2/2 PASS. L2 audit-suite regression (re-run Phases 1/3/4/5/7) deferred to a future dedicated session per user direction ("finish this up first; we need a solid foundation and need to clear known issues so the new audit isn't as cluttered").

Disposition totals: 5 Cat 1 closed + 14 verified-pending-L2 + 3 deferred + 1 audit-only finding = 23. No new permanent deviations added. ACCEPTED_STANDARDS_DEVIATIONS edits: FP-1 moved Active → Resolved by R-6c.

Verified: pure-software / pure-doc commits this cycle. Each commit cites per-commit positive-control per HW_GATE Rule 3. Host ctest 794/794 PASS at session end. Target-build parity (4 tiers) verified at every source-touching commit. Vehicle bench_sim 2/2 PASS on flight + dev binaries at multiple gate points.

### 2026-05-12-001 | Claude | audit, architecture, documentation, tooling

**Master Standards Audit 2026-05-07 — Phases A through 8 partial.** Per-step record in `docs/audits/MASTER_STANDARDS_AUDIT_2026-05-07.md`; project-wide PR tracker (established this cycle) in `docs/PROBLEM_REPORTS.md`. Resume point next session: R-6c.

Outcomes by phase / category:
- **Phase A.2 + R-6 + R-6b** — renamed `STANDARDS_DEVIATIONS.md` → `ACCEPTED_STANDARDS_DEVIATIONS.md`, re-evaluated every Active deviation row, corrected a JSF Rule 170 misreading proliferation and re-attributed FP-1 to its actual governing standard (Power of 10 Rule 9). New standards-precedence rule in `CODING_STANDARDS.md`. LL Entry 37 added.
- **Phases B / 0 / 1-7** — `AUDIT_GUIDANCE.md` Appendix B inlined; scripted sweeps + manual phases ran end-to-end (Phase 4 PARTIAL became R-9a/b/c; Phases 5/6/7 PASS).
- **Phase 8 ordering framework** — `AUDIT_GUIDANCE.md` Appendix C (4-category remediation ordering), `PROBLEM_REPORTS.md` established, `SESSION_CHECKLIST.md` Per-Commit 5a (Change Impact Analysis), `HW_GATE_DISCIPLINE.md` Rule 6 (regression-suite credit) + Rule 7 (surfaced-bugs in-scope).
- **Phase 8 Cat 1 closed** — P8-SPIN-A/B, R-10a, R-9a, R-9b.
- **Phase 8 Cat 2 closed** — R-3 (capture-state-then-reset hardfault handler; R-4 MPU AP encoding + MEMFAULTENA folded in per surfaced-bug rule; later amended to inline-only handler after fault-on-fault lockup surfaced during verification). `docs/decisions/FAULT_HANDLER_DESIGN.md` captures rationale.
- **Phase 8 Cat 3 partial** — R-1 verified (bounded Core 1 boot-wait), R-9c verified (log-on-change for secondary health bits), **R-5 deferred** to a dedicated session with proliferation gate active NOW (`scripts/hooks/pre-commit` + `stdio_allowlist.txt` block new `<stdio.h>` includes; `docs/decisions/STDIO_REPLACEMENT_PLAN.md` + `docs/STDIO_TO_RC_LOG_MIGRATION_GUIDE.md` describe the design), **R-2 absorbed** into the R-5 session (council review preserved). R-6c, R-11, R-12, P8-FMEA-Pyro remain.

Architecture work captured this cycle: AGENT_WHITEBOARD "In-flight fault recovery architecture" 5-question framing for a dedicated future session (including the surfaced Core 1 SIO_FIFO_IRQ post-AIRCR-reset wedge); PROJECT_STATUS "Side Projects & Future Product Lines" re-added; TP-1 deviation row for a Pico SDK `spi_set_format` cognitive-complexity finding.

Verified: pure-software / pure-doc commits this cycle (each commit message cites per-commit positive-control per HW_GATE Rule 3). Host ctest 788/788 PASS at session end. Target-build parity (4 tiers) verified at the R-3 commit boundary (`e4d222a`); no source changes since.

### 2026-05-07-001 | Grok 4.3 | tooling, documentation

**Category 2 audit scripts + master guidance updates.** Added five new scripted audit tools with self-verification modes and positive-control verdicts: `analyze_stack_usage.sh`, `run_cppcheck.sh`, `verify_build_parity.sh`, `generate_coverage_report.sh`, and `enhanced_fault_injection.py`. Created `scripts/README.md` with Required Tools section. Updated `AUDIT_GUIDANCE.md` with Hardware Validation Scripts classification and external references. Minor hardening of `run_clang_tidy.sh` for positive-control output.

Verified: pure-software change, host ctest 788/788 PASS, no HW reseat required.

Files: `scripts/`, `standards/AUDIT_GUIDANCE.md`, `scripts/run_clang_tidy.sh`, `CHANGELOG.md`

### 2026-04-27-007 | Claude | documentation, refactor

**`.claude/SESSION_CHECKLIST.md` restructured: four-scope hierarchy + Trigger-Driven Documentation Edits.** Surfaced by user observation that the previous structure conflated multiple events — "Session End (Normal Completion)" mixed per-commit hygiene (verification, message format, no-orphans) with per-push gates (CHANGELOG, WB cleanup, build parity), and the absence of an explicit COMMIT layer meant "ready to push" got read as "ready to end the session." Triggered-driven doc edits (CHANGELOG, WB, PROJECT_STATUS, drift checks) were inline as cadence rules rather than as content-driven triggers, which is how they actually work.

**New hierarchy:** `COMMIT ⊂ PUSH ⊂ SESSION_END ⊂ MILESTONE`. Each outer scope inherits inner-scope rules and adds its own. Per-Commit covers build verification, no orphans, no unintended deletions, commit message format + HW gate citation per `HW_GATE_DISCIPLINE.md` Rule 3. Per-Push adds station/vehicle 4-tier parity rebuild, "triggered-doc-edits-are-committed" gate, and CHANGELOG-covers-the-push-window. Session End adds no-broken-code-on-main, push-to-remote, handoff notes (folding what was previously the separate "Session End (Handoff)" section into the default Session-End mode). Session-End: Milestone keeps its existing rot-detection layer (protected-doc drift check, build-system audit, full-tree clang-tidy, lessons-learned consideration) and is now explicitly named as a structurally-authorized retroactive-fix moment.

**New section: Trigger-Driven Documentation Edits.** Captures the principle that a protected/architecture document is edited only when the session's work directly contradicts its current contents — not on cadence, not on stage transitions, not "at session end" by default. The edit rides with the trigger (same commit as the change), so `git log --follow` on the doc and `git bisect` see code + doc atomically consistent.

**Two-category framing for protected docs:**

- **State-of-system docs** (SAD, SCAFFOLDING, AO_ARCHITECTURE, MULTICORE_RULES, CODING_STANDARDS, RP2350_ERRATA, HW_GATE_DISCIPLINE, COUNCIL_PROCESS, .claude/CLAUDE.md, AK_GUIDELINES, PROTECTED_FILES, SESSION_CHECKLIST, README, PROJECT_STATUS, AGENT_WHITEBOARD): forward-going edit applies. When the diff makes the doc wrong, fix it in the same commit that triggered the wrongness.
- **Historical-record docs** (`docs/decisions/*`, `docs/plans/*`, `docs/audits/*`, `docs/baselines/*`, CHANGELOG, LESSONS_LEARNED, STANDARDS_DEVIATIONS Resolved section): forward-going edit does NOT apply. New entries are new files / new entries. Edits to existing content are typo-correction or supersession-header marking only (e.g., the legitimate "SUPERSEDED 2026-04-22" header on LL Entry 25).
- **Mixed-mode** (`docs/IVP.md`): per-IVP entries are historical (frozen on IVP commit), stage-list / next-stage sections are state-of-system. Two trigger modes — planning-entry (first step of stage planning) and closing-entry (after divergence from plan).

**Retroactive amendment rule:** drift caught after the trigger event is legitimate to fix **with explicit user approval**, or as part of the milestone-authorized drift-check audit. Otherwise default is to flag the drift on `AGENT_WHITEBOARD.md` and continue. The approval gate exists because retroactive fixes look identical to "decided to start editing protected files" from a diff perspective.

Verified: pure-software change (single doc edit), host ctest 788/788 PASS, no HW reseat required.

Files: `.claude/SESSION_CHECKLIST.md`, `CHANGELOG.md`

### 2026-04-27-006 | Claude | documentation, council, refactor

**`standards/HW_GATE_DISCIPLINE.md` authored + AO Commandments full-sweep audit.** Two related sweeps closing out the long-standing WB items.

**(a) HW gate discipline (new normative doc).** `standards/HW_GATE_DISCIPLINE.md` consolidates two previously-scattered concerns: (1) what a HW gate must *observe* to pass (Rules 1-4 — positive-control signal, 3-boot reseat protocol, commit-message citation, hard-vs-soft classification — addressing the IVP-140 false-positive + Fruit Jam GPS cable episode that motivated the user's 2026-04-17 ask), and (2) how a HW gate stays *unskippable* (Rule 5 — pre-commit hook runs the gate, session-start canary catches prior-session rot, `--no-verify` requires explicit user approval — addressing the `STAGE_P7_15_SHELVED_2026-04-11.md` "claimed but not performed" / LL Entry 36 bench_sim-rot framing). `.claude/SESSION_CHECKLIST.md` Item 9 amended to require commit messages cite the observed control signal per Rule 3 (with exempt form for pure-software changes). Wired into `.claude/CLAUDE.md` auto-load + README.md "Read First" so it's intaken on every session alongside the other standards docs. Unblocks `docs/plans/STAGE_T_T14_DESIGN.md` Q7 convergence which explicitly waited on this doc to land.

**(b) AO Commandments full sweep.** New `docs/audits/AO_COMMANDMENTS_AUDIT_2026-04-27.md`. Audited 9 active AOs (Radio, FlightDirector, HealthMonitor, RfManager, Notify, Logger, Telemetry, LedEngine, RCOS) against all 12 commandments. Zero unaddressed violations. Two documented Commandment IV deviations (blocking `flash_safe_execute` in `ao_radio` config persistence and `ao_rcos` calibration save/erase), both with in-code rationale, both bounded by queue-depth math (32 events × 10 ms tick = 320 ms vs ~100-500 ms worst-case block). LL Entry 35 fix is universal — every `QACTIVE_POST` and `Q_PUBLISH` site uses `static …Evt s_evt;` storage. Three observations not rising to violation: Logger has mixed module-style and event-style API surface (naming clarity), `AO_FlightDirector` calls `AO_Logger_log_event` synchronously (it's a module-function ring-buffer push, not cross-AO synchronous call), RCOS↔FlightDirector interactions follow the documented Commandment V/VII pattern. AO_COMMANDMENTS.md remains advisory; elevation to `standards/` deferred.

**(c) Dead-code cleanup (audit-driven).** Deleted `src/active_objects/ao_blinker.cpp/.h` and `src/active_objects/ao_counter.cpp/.h` (4 files, 283 lines). Created 2026-03-27 in `a32e309` as the IVP-76 "trivial 2-AO demo" to validate QF time events + QV dispatch + AO lifecycle on the path to real AOs. They served their proof-of-concept role and were never started after Stage 9 graduated to actual AOs (FlightDirector, HealthMonitor, etc.). CMakeLists.txt source-list entries removed from both the `add_executable(rocketchip ...)` block and the `ROCKETCHIP_SOURCES` `-Wpedantic` scope. Stale `#include "ao_blinker.h"` / `#include "ao_counter.h"` removed from `src/main.cpp`. `docs/AO_ARCHITECTURE.md` "(disabled)" footnote removed. `docs/SCAFFOLDING.md` directory tree updated. `docs/VERIFICATION_OVERVIEW.md` soak-check guidance no longer references `AO_Counter` jitter. `docs/IVP.md` historical IVP-76 record left intact (per protected-file historical-record convention).

Verified: pure-software changes, all 4 firmware targets build clean (vehicle bench 174 ninja targets, vehicle flight 169, station bench 174, station flight 169 — same -2 delta as the source removals), host ctest 788/788 PASS across all six commits in this batch, no HW reseat required.

Files: `standards/HW_GATE_DISCIPLINE.md` (new), `docs/audits/AO_COMMANDMENTS_AUDIT_2026-04-27.md` (new), `.claude/CLAUDE.md`, `.claude/SESSION_CHECKLIST.md`, `README.md`, `src/active_objects/ao_blinker.{cpp,h}` (deleted), `src/active_objects/ao_counter.{cpp,h}` (deleted), `src/main.cpp`, `CMakeLists.txt`, `docs/AO_ARCHITECTURE.md`, `docs/SCAFFOLDING.md`, `docs/VERIFICATION_OVERVIEW.md`, `CHANGELOG.md`

### 2026-04-27-005 | Claude | refactor, documentation

**Test-harness build-tag follow-up to the polarity rename + WB cleanup.** The post-rename firmware emits `dev-<sha>` in the boot banner (vs the old `bench-<sha>`), but `scripts/_rc_test_common.py`'s `_RE_BUILD_TAG` regex still required `(bench|flight)`. Tests passed because the matcher's `Build.UNKNOWN` fallback is permissive, but build-tier classification was silently degraded for every test run since the rename. Fixed: regex now matches `(dev|bench|flight)` (`bench` retained for backward-compat with old transcripts), `Build.BENCH` enum member renamed to `Build.DEV`, `TARGET_*_BENCH` constants kept as aliases for callers (8 scripts depend on the name). Test fixture `VEHICLE_BENCH_BANNER` updated to `dev-...`, host-test assertions updated to `Build.DEV` / `'vehicle-dev'`. 788/788 ctest pass + `scripts/test__rc_test_common.py` direct run shows ALL CHECKS PASS. Whiteboard: cross-check confirmed two more WB rows were stale and erased — Role-aware bench_sim gate (already implemented in `scripts/ci/pre_commit_matrix.py` with separate `TRIGGER_FLIGHT_BENCH` / `TRIGGER_STATION_BENCH` triggers), Station role/board decoupling (already implemented via `include/rocketchip/board.h` `#if defined(ADAFRUIT_FRUIT_JAM)` selector + `board_*.h` per-board headers + auto-pick in CMakeLists.txt). Remaining "Fruit Jam" mentions in `src/` are explanatory comments, not coupling.

Files: `scripts/_rc_test_common.py`, `scripts/test__rc_test_common.py`, `AGENT_WHITEBOARD.md`, `CHANGELOG.md`

### 2026-04-27-003 | Claude | refactor, documentation

**One-time scaffolding cleanup.** Deleted `src/tools/mat_benchmark.cpp` + its CMake target and removed the empty `src/tools/` directory — the IVP-40 24-state-ESKF gate decision is closed and no documentation calls for periodic re-runs (results preserved in `docs/benchmarks/`). Deleted `docs/TOOLCHAIN_VALIDATION.md` (Jan-2026 FreeRTOS SMP + Pico SDK 2.1.0 scaffolding, last touched 2026-02-02 and wrong on every axis since the QP/C bare-metal pivot; current build/debug guidance lives in `DEBUG_PROBE_NOTES.md`, `FLASHING.md`, `BENCH_TEST_PROCEDURE.md`, `BOARD_FIRMWARE_VERIFICATION.md`). Updated CMakeLists.txt parent-gate comment (`ROCKETCHIP_BUILD_DEV_TOOLS` now exposes only the stage archive — no standing dev-tool targets). Removed mat_benchmark and ud_benchmark exclusions from `scripts/run_clang_tidy.sh`. SCAFFOLDING.md, SAD.md, BUILD_SYSTEM_AUDIT.md updated to match. Erased the `Resolved` sections from `AGENT_WHITEBOARD.md` per its IRL-whiteboard rule (CHANGELOG is the source of truth). Audit confirmed `src/dev/` (6 modules, 1084 lines, all properly `BUILD_FOR_FLIGHT`-gated) has no dead code. Verified: 788/788 host ctest pass, vehicle bench builds clean.

Files: `src/tools/mat_benchmark.cpp` (deleted), `docs/TOOLCHAIN_VALIDATION.md` (deleted), `CMakeLists.txt`, `docs/SCAFFOLDING.md`, `docs/SAD.md`, `docs/BUILD_SYSTEM_AUDIT.md`, `scripts/run_clang_tidy.sh`, `AGENT_WHITEBOARD.md`, `CHANGELOG.md`

### 2026-04-27-004 | Claude | refactor, council, documentation, tooling

**Polarity rename + first toolchain audit + WB hygiene.** Three independent build-system audit follow-ups closed in one session.

**1. Operator-facing flag rename + source-side mirror.** Renamed CMake flag `BUILD_FOR_FLIGHT` (default OFF = dev) → `NOT_CERTIFIED_FOR_FLIGHT` (default OFF = flight-ready) so the safe state matches the unflagged default. New positive source-side mirror `ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS=1` is defined when the operator flag is ON; all `src/` and `include/` `#ifdef`/`#ifndef` sites use the positive mirror so the source code reads in the affirmative even though the operator flag is negative-polarity. CMake `message(WARNING ...)` fires when the dev flag is set, output binary becomes `rocketchip-dev.uf2` (vs `rocketchip.uf2` for flight-ready), and `kBuildConfig` reports `"dev"` vs `"flight"` so the boot banner carries the bit. CMakePresets.json, BENCH_TEST_PROCEDURE.md, BUILD_SYSTEM_AUDIT.md, CONFIG_TEST_MATRIX.md, FAULT_INJECTION.md, FLASHING.md, BOARD_FIRMWARE_VERIFICATION.md, .claude/SESSION_CHECKLIST.md updated to match. All 4 builds verified clean (vehicle/station × bench/flight); 788/788 host ctest pass; ELF-size delta ~+170KB confirms dev code is genuinely excluded from flight binaries. *Council review (NASA/JPL, ArduPilot Core, Embedded Prof, Cubesat Startup) converged on this expanded scope: pure mechanical rename was the original WB ask; the two-name pattern + WARNING + OUTPUT_NAME + banner-config additions came from the council's defense-in-depth additions.*

**2. Toolchain-version audit (P6 first run).** New `docs/audits/TOOLCHAIN_VERSION_AUDIT_2026-04-27.md`. All pinned components within P6 "clean" criterion. No upgrades required: Pico SDK 2.2.0 current, picotool 2.2.0-a4 current, GCC ARM 14.2.Rel1 explicitly stays pinned (15.2.Rel1 available but Pico SDK validated against 13/14, codegen risk per LL Entry 30), OpenOCD 0.12.0+dev pinned (Pi fork uses rolling `rpi-common` branch with no release cadence — chasing tip introduces probe-flake risk), CMake 4.2.1 local vs 3.25 project minimum — both fine. Single residual action item documented: read installed Pico Probe firmware version against debugprobe-v2.3.0 to check for the upstream-#201 "fails to start" regression. Future-watch options surfaced (quarterly cadence, pre-commit drift check, errata-coupling).

**3. Tier consolidation rejected.** WB item asked whether `src/dev/*.cpp` could become runtime-gated (flight-state checks) instead of compile-excluded, eliminating the 4-target build matrix. *Rejected after consideration:* the current 2-tier compile-time exclusion gives a physical-layer safety property (`strings rocketchip.elf | grep "Debug Menu"` returns zero) that runtime gating cannot match. A runtime gate is one bug, glitch, command-injection, or memory corruption away from being bypassable; compile-time absence cannot be bypassed. The 4-build matrix cost is real but acceptable; the absence-as-proof property is non-negotiable. Same reasoning that drives the carve-out distinction in CODING_STANDARDS.md (CLI uses runtime lockout, dev tools use compile-time exclusion). The flag-count concern that motivated the WB row was largely dissolved by the 2026-04-23 hierarchical-gate cleanup + this morning's `mat_benchmark` deletion + the rename above — `cmake -B build` now sees 3 surface-level flags with documented purpose.

**4. WB hygiene.** Rewrote `AGENT_WHITEBOARD.md` header with explicit "treat like an IRL whiteboard, not a record of completed things" rule + rationale (stale "done" notes dilute the active-row signal). Erased rows resolved-elsewhere or already-done: RP2350 errata sweep (done 27068cb + 276228a, content fully migrated to standards/RP2350_ERRATA.md), MAIN_DESCENT timeout / SPIN P7 (done f005899 / IVP-121), the polarity rename + toolchain audit + tier consolidation closed by this commit, host-script-hardening "closed" marker (Tiers 1-7 done in -001, .git/hooks/pre-commit removed, core.hooksPath set), Stage T COMPLETE marker (forward-looking follow-up already its own row), Station bench_sim/SPIN landed marker (forward-looking extensions already in Medium section). WB now 108 lines (was ~206), all rows are active deferrals or genuinely pending work.

**5. Date typo fix on prior 2026-04-NN entries.** The four most-recent CHANGELOG entries (`-001`/`-002`/`-003` from prior agents and the `-004` initially drafted in this session) were all dated `2026-04-30` despite being committed `2026-04-28` per `git log`. User-granted exception (the format guide's "trivial typo" carve-out, which normally protects older entries from cross-agent revision): retroactively renumbered `2026-04-30-{001,002,003,004}` → `2026-04-27-{001,002,003,004}` (yesterday before-midnight, the actual authorship date). Internal cross-references updated (e.g., the `-002` body's reference to `-001` follows the new date). Also renamed `docs/audits/TOOLCHAIN_VERSION_AUDIT_2026-04-30.md` → `..._2026-04-27.md` and fixed the in-doc title + "Latest upstream pulled" attribution date to match.

Files: `CMakeLists.txt`, `CMakePresets.json`, `include/rocketchip/version.h`, `src/active_objects/ao_telemetry.cpp`, `src/cli/rc_os_commands.cpp`, `src/dev/*.cpp`, `src/dev/*.h`, `src/fusion/eskf_runner.{cpp,h}`, `src/main.cpp`, `docs/BOARD_FIRMWARE_VERIFICATION.md`, `docs/BUILD_SYSTEM_AUDIT.md`, `docs/CONFIG_TEST_MATRIX.md`, `docs/FAULT_INJECTION.md`, `docs/FLASHING.md`, `docs/PROJECT_STATUS.md`, `docs/audits/TOOLCHAIN_VERSION_AUDIT_2026-04-27.md` (new), `.claude/SESSION_CHECKLIST.md`, `AGENT_WHITEBOARD.md`, `CHANGELOG.md`

### 2026-04-27-001 | Cursor | tooling, council, documentation

**Host script hardening closure (Tiers 1–7):** `scripts/_rc_test_common.py` gains `find_vehicle_and_station_ports`; Tier 4 Stage T / decode / bench paths use `find_target_port` + `open_classified_port`; `CMakeLists.txt` registers `ctest` targets `scripts_rc_test_common` + `scripts_python_compileall` (`-DBUILD_TESTS=ON`). `.github/workflows/python-scripts-ci.yml` + `scripts/ci/pre_commit_matrix.py`; tracked **`git config core.hooksPath scripts/hooks`** + `scripts/hooks/pre-commit` (prior `.git/hooks/pre-commit` should be removed/renamed to avoid duplication). **`@rc_test(watchdog_s=)`** ceilings on `ack_stress_test`, `soak_test`, `replay_harness`. **Firmware Tier 5:** `src/cli/rc_os.cpp` — main-menu **`p`** runs `cli_print_preflight()` on station (RX). Council review + roadmap: `docs/council/HOST_SCRIPT_HARDENING_REVIEW_AND_ROADMAP.md`; matrix + plan stubs updated.

Files: `src/cli/rc_os.cpp`, `test/CMakeLists.txt`, `scripts/**/*.py`, `scripts/ci/pre_commit_matrix.py`, `scripts/hooks/*`, `.github/workflows/python-scripts-ci.yml`, `docs/council/HOST_SCRIPT_HARDENING_REVIEW_AND_ROADMAP.md`, `docs/plans/HOST_SCRIPT_HARDENING_PLAN.md`, `docs/CONFIG_TEST_MATRIX.md`, `docs/PROJECT_STATUS.md`, `docs/SCAFFOLDING.md`, `AGENT_WHITEBOARD.md`, `.gitignore`, `CHANGELOG.md`

### 2026-04-27-002 | Cursor | documentation

Changelog hygiene: restored the **`### 2026-04-29-003`** section header immediately below; its body text had stayed in the file but was left without a dated heading after the **`2026-04-27-001`** insertion. No retrospective edits to **`2026-04-29-003`** body copy beyond reattaching the header.

Files: CHANGELOG.md

### 2026-04-29-003 | Grok via Cursor | documentation, refactor

**Protected files review:** Reverted unauthorized edits to `.claude/SESSION_CHECKLIST.md` (Stage O policy block), deleted incremental `standards/STANDARDS_AUDIT_2026-04-28.md`, and trimmed overly-specific MPU section from `docs/MULTICORE_RULES.md`. Preserved general single-source fault protection rules, dual-core MPU guard setup, and GDB naming note in MULTICORE_RULES.md (now under Memory section). Kept practical pre-commit guidance in DEBUG_PROBE_NOTES.md. Non-ASCII (mostly em-dashes in comments) has no functional impact.

Files: `.claude/SESSION_CHECKLIST.md`, `docs/MULTICORE_RULES.md`, `standards/STANDARDS_AUDIT.md`, CHANGELOG.md

### 2026-04-29-002 | Cursor | architecture, refactor, documentation, tooling, hardware

**OPT‑IVP‑01/02/05 backlog — combined changes in `origin/main..f142901` (four commits).**

`src/safety/fault_protection.{h,cpp}` (MPU guard, MemManage, `Q_onError`), `shared_state` + `main.cpp` / `sensor_core1` / calibration / CLI cleanup, `eskf_runner` fusion-cycle + bench updates, baseline/runbook/multicore/fault docs, `.claude/DEBUG_PROBE_NOTES.md`, SPIN helper + `scripts/start_openocd_pico_sdk.ps1` + dual-core GDB watch (Windows-safe delay); `.gitattributes` / `.gitignore` housekeeping. Ordinary IVP/session batch—not a milestone sign-off.

**Policies / cleanup:** **`CHANGELOG.md` Format** (see above)—don’t refactor past dated sections for substantive fixes; drafts of the active entry stay editable.

**Revert:** misplaced **Milestone item 19** (Stage O soak wording) struck from **`.claude/SESSION_CHECKLIST.md`** so that file stays **owner‑approved‑edits‑only** per **`PROTECTED_FILES.md`**; same material lives under **`docs/baselines/stage_o_hw_verification_2026-04-28.md`** and **`.claude/DEBUG_PROBE_NOTES.md`**.

Files: `.claude/DEBUG_PROBE_NOTES.md`, `.claude/SESSION_CHECKLIST.md`, `.gitattributes`, `.gitignore`, `CHANGELOG.md`, `CMakeLists.txt`, `docs/baselines/stage_o_hw_verification_2026-04-28.md`, `docs/benchmarks/UD_BENCHMARK_RESULTS.md`, `docs/FAULT_INJECTION.md`, `docs/MULTICORE_RULES.md`, `docs/PROJECT_STATUS.md`, `include/rocketchip/shared_state.h`, `scripts/opt_ivp01_row10_dualcore_watch.gdb`, `scripts/start_openocd_pico_sdk.ps1`, `src/calibration/cal_hooks.cpp`, `src/cli/rc_os_commands.cpp`, `src/core1/sensor_core1.cpp`, `src/core1/sensor_core1.h`, `src/fusion/eskf_runner.cpp`, `src/fusion/eskf_runner.h`, `src/main.cpp`, `src/safety/fault_protection.cpp`, `src/safety/fault_protection.h`, `src/shared_state.cpp`, `standards/STANDARDS_AUDIT.md`, `standards/STANDARDS_AUDIT_2026-04-28.md`, `tools/spin/README.md`, `tools/spin/run_stage_o_ao_spin.sh`

### 2026-04-29-001 | Cursor | documentation, hardware

**Stage O OPT‑IVP‑01/02/05 plan closed — HW verified.**

Closure recorded in **`docs/baselines/stage_o_hw_verification_2026-04-28.md`**: **`bench_sim` 2/2 COM7** (2026-04-29 ~6.5 s), Row 10 evidence `build/row10_ivp01_watch_2026-04-27.log`, prior SPIN / Tier‑1 soak citations unchanged. **`scripts/opt_ivp01_row10_dualcore_watch.gdb`** passive delay uses **`python -c`** so Windows GDB isn’t blocked on POSIX `sleep`. **`docs/PROJECT_STATUS.md`** Stage O table set to **CLOSED**.

Files: `docs/PROJECT_STATUS.md`, `docs/baselines/stage_o_hw_verification_2026-04-28.md`, `scripts/opt_ivp01_row10_dualcore_watch.gdb`, `CHANGELOG.md`

### 2026-04-28-001 | Cursor | documentation, testing, council

**Stage O gate traceability: HW runbook + standards audit (OPT-IVP-01/02/05).**

Added `docs/baselines/stage_o_hw_verification_2026-04-28.md` with explicit commands for 5 min GDB soak (`soak_gdb.gdb`), station `station_bench_sim`, SPIN (Cygwin), and MPU notes. Added `standards/STANDARDS_AUDIT_2026-04-28.md` and linked it from `standards/STANDARDS_AUDIT.md`. Updated `docs/PROJECT_STATUS.md` Stage O table. Confirmed 786/786 `build_host` ctest; vehicle `bench_sim` green; station/SPIN/soak require local OpenOCD + hardware.

Files: `docs/baselines/stage_o_hw_verification_2026-04-28.md`, `standards/STANDARDS_AUDIT_2026-04-28.md`, `standards/STANDARDS_AUDIT.md`, `docs/PROJECT_STATUS.md`

### 2026-04-27-001 | Cursor agent | optimization, architecture, documentation

**Stage O (OPT-IVP-02, OPT-IVP-05): Centralized global definitions + ESKF full-tick bench.**

`src/shared_state.cpp` now defines the globals declared in `include/rocketchip/shared_state.h` (init flags, GPS function pointers, seqlock, atomics). `main.cpp` uses small `bind_gps_*_backend()` helpers to avoid three-way duplication of I2C/UART pointer wiring. `sensor_core1.h`, `cal_hooks.cpp`, and `rc_os_commands.cpp` include the shared header and drop redundant `extern` blocks. `eskf_runner` gains dev-only `eskf_runner_get_bench_full_tick()` (min/max/avg, predict through confidence) alongside the existing predict-only bench; CLI `s` + `docs/benchmarks/UD_BENCHMARK_RESULTS.md` document both. Removed stale `g_watchdogReboot` extern (recovery module removed; symbol was never defined).

Files: `src/shared_state.cpp`, `src/main.cpp`, `src/core1/sensor_core1.h`, `src/calibration/cal_hooks.cpp`, `src/fusion/eskf_runner.{h,cpp}`, `src/cli/rc_os_commands.cpp`, `CMakeLists.txt`, `docs/benchmarks/UD_BENCHMARK_RESULTS.md`

### 2026-04-26-002 | Grok via Cursor | refactor, optimization

**Stage O (OPT-IVP-01): Extract shared fault protection module.**

Moved duplicated MPU stack guard (`mpu_setup_stack_guard`, `kMpuGuardSizeBytes`), `memmanage_fault_handler()`, and `Q_onError()` to `safety/fault_protection.{h,cpp}`. Updated `src/main.cpp`, `src/core1/sensor_core1.cpp`, CMakeLists.txt, and includes. Eliminates ~80 LOC duplication while preserving exact behavior, stack-safety, and both-core MPU setup. No linter errors. Full gates passed (ctest, build, manual MPU verification).

Files: `safety/fault_protection.h`, `safety/fault_protection.cpp`, `src/main.cpp`, `src/core1/sensor_core1.cpp`, `CMakeLists.txt`

### 2026-04-26-001 | Codex 5.3 via Cursor | bugfix, refactor

**Startup init cleanup: avoid redundant second I2C bus init on healthy boots.**

`src/main.cpp` now calls `i2c_bus_init()` in `init_hardware()` only if early I2C bring-up did not already succeed. This preserves startup behavior and required baro startup calibration while removing unnecessary duplicate initialization work in the normal path.

Files: `src/main.cpp`

---

### 2026-04-22-004 | Claude Opus 4.7 | tooling, refactor, documentation, council

**Whiteboard triage session — 4 of 6 High-priority items closed.** CMake auto-pick Fruit Jam board when `ROCKETCHIP_JOB_STATION=1` (prevents Frankenstein station-on-Feather builds); centralized flashing + board-firmware verification into new `docs/FLASHING.md` replacing scattered references across `BENCH_TEST_PROCEDURE.md` / `DEBUG_PROBE_NOTES.md` / deleted memory files; full RP2350 silicon errata sweep producing `standards/RP2350_ERRATA.md` (28 errata, 4 active-attention triaged against our silicon blocks + Adafruit board pulls, 24 not-applicable with reasons); Rescue-DP referenced in FLASHING.md as a secondary recovery option. Four commits: `0a6aae5` (CMake auto-pick), `5685f26` (docs centralization), `27068cb` (errata matrix), `276228a` (E12 re-read + E2 incident tracker), `7902b81` (Rescue-DP note).

Memory purged of six HW-identity files that had stored a swapped chip-serial ↔ board mapping — root cause of repeated Frankenstein-flash incidents. Repo docs (e.g., `docs/plans/STAGE_T_FIX_PLAN.md:339` cites station serial) are now the authoritative source; memory holds only patterns, not identity. Added memory note `feedback_clarifications_in_docs.md` enforcing "conceptual content lives in docs, code comments point to docs."

A4 silicon (Feather HSTX post-2026-03-20) fixes E9 and others — captured as a procurement note for future hardware orders.

R-1 / R-2 E2 trigger paths (`picotool -f` warm reboot, SWD halt mid-`spin_lock_blocking`) have an **Incident log** table in the errata doc for structured recurrence tracking. Online search confirmed no exact-pattern match for our intersection — collect data before filing upstream.

Files: `CMakeLists.txt`, `docs/FLASHING.md` (new), `docs/BOARD_FIRMWARE_VERIFICATION.md` (new, intermediate — kept per user direction for later cleanup), `docs/BENCH_TEST_PROCEDURE.md` (Frankenstein section → pointer), `standards/RP2350_ERRATA.md` (new), `standards/CODING_STANDARDS.md` (one-line errata pointer), `AGENT_WHITEBOARD.md`.

---

### 2026-04-23-001 | Claude Opus 4.7 | refactor, tooling, documentation, audit

**Build-system audit — CMake + toolchain cleanup with `docs/BUILD_SYSTEM_AUDIT.md` as deliverable.**

User flagged CMake accretion as overdue after a broken auxiliary target (`ud_benchmark`) went undetected for 2+ weeks. Session scope expanded from "delete dead targets" to full end-to-end audit. Six phases, seven commits (`5bc463b` through `95de0a4` + `d6bccd2` whiteboard).

**Removed (code):**
- `ud_benchmark` target + `src/benchmark/ud_benchmark.cpp` (946 lines) — Phase-1 gate decision delivered 2026-02-24, results preserved in `docs/benchmarks/UD_BENCHMARK_RESULTS.md`, Bierman adopted.
- `src/benchmark/` directory (now empty).

**Gated behind new hierarchical flags (not deleted — preserved for replay):**
- `ROCKETCHIP_BUILD_DEV_TOOLS` (parent, default OFF) — regular dev-tool targets. `mat_benchmark` now lives here.
- `ROCKETCHIP_STAGE_ARCHIVE` (child of DEV_TOOLS, default OFF) — stage-bounded diagnostic flags: `STAGE_T_LOGGING`, `STAGE_T2_CHEAT`, `STAGE_T3_MAVLINK`. Each flag still opt-in per-flag after both parents are on. Nested structure prevents accidental activation of archived stage flags during regular dev work.

**Fixed (build coverage):**
- 14 of our `.cpp` files had drifted outside `ROCKETCHIP_SOURCES`, missing `-Wpedantic` coverage silently for months (active_objects, dev/diag_stats, fusion/{confidence_gate,innovation_monitor}, notify backends, safety/{pio_watchdog,pio_backup_timer,pyro_edge_logger}). All 14 now in the list.
- Vendored third-party includes (`lib/mavlink`, `lib/qep`, `lib/ruuvi.dps310.c`, `lib/lwgps`) reclassified from `-I` to `-isystem` (SYSTEM PRIVATE target_include_directories).
- Pico SDK + TinyUSB interface targets marked `SYSTEM TRUE` so transitive includes don't surface vendor-header pedantic errors in our code.
- `cmake_minimum_required` 3.13 → 3.25 (3.13 was already fictional — CMakePresets.json required 3.21 to parse; 3.25 enables native SYSTEM target property).

**Documented (flight_director host/target split):**
- `rc_flight_director` library (host build) and `add_executable(rocketchip)` (target build) both list the same flight_director source files. Looks like duplication; actually intentional — two `if(BUILD_TESTS)` branches compile those files in different preprocessor contexts (host vs target). Comment block at `rc_flight_director` added to prevent future audits from collapsing the split.

**Removed (narrative comments):**
- "Source files added as IVP-39/40 / IVP-41+ are implemented" stage-scaffolding comments.
- "migrated from watchdog_recovery 2026-04-22" note on `test_eskf_disable_brake` (migration history is in git log, not live tree).

**Deliverable:**
- `docs/BUILD_SYSTEM_AUDIT.md` (296 lines) — structured audit checklist with 7 check categories (P1-A dev-tool gating, P1-B self-flagged dead code, P2 ROCKETCHIP_SOURCES coverage, P3 host/target split, P4 SYSTEM classification, P5 scaffolding comments, P6 toolchain-version check), guardrails list (what NOT to delete — PICO_SW_SPIN_LOCKS_NO_EXTEXCLALL, diag_stats unconditional inclusion, target_link_options --undefined, per-file -O2 on eskf codegen), and append-only incident log.
- `.claude/SESSION_CHECKLIST.md` item 16 added cross-referencing the new doc at milestone close.

**Side effect (verified benign, documented in audit doc):** Firmware `.text` ~740 bytes smaller across all 4 configs — GCC optimizes SDK code slightly more aggressively under `-isystem` semantics. Our code `.obj` byte-identical to baseline. Runtime verified.

**Verification gate at each phase:** host tests (786/786 PASS), 4 firmware configs clean link, `nm` symbol diff against Phase 0 baseline at `docs/baselines/build_audit_2026-04-23/`. HW boot-clean gate on vehicle: `Board: Adafruit Feather RP2350 HSTX`, `Hardware: 14/14 OK`, preflight `VERDICT: GO`. Vehicle `bench_sim.py`: 2/2 PASS.

Plan file: `.claude/plans/i-just-put-you-glimmering-elephant.md` (retained).

Open items captured in `AGENT_WHITEBOARD.md` Easy section:
- `BUILD_FOR_FLIGHT` → `NOT_CERTIFIED_FOR_FLIGHT` polarity rename (reads correct against safety default).
- First run of toolchain-version audit (P6 — Pico SDK, Pico Probe firmware, GCC ARM, OpenOCD, picotool, CMake).
- Tier consolidation evaluation (whether `src/dev/*.cpp` could be runtime-gated instead of compile-excluded).

Files: `CMakeLists.txt`, `test/CMakeLists.txt`, `test/test_go_nogo.cpp`, deleted `src/benchmark/ud_benchmark.cpp` + directory, new `docs/BUILD_SYSTEM_AUDIT.md` + `docs/baselines/build_audit_2026-04-23/*`, `.claude/SESSION_CHECKLIST.md`, `AGENT_WHITEBOARD.md`.

---

### 2026-04-22-005 | Claude Opus 4.7 | refactor, safety, documentation

**Watchdog recovery machinery removed — dead code after IVP-90, producing false-positive `[WARN] WATCHDOG REBOOT` banners on warm reboots.**

IVP-90 (2026-03-29) removed the SDK hardware watchdog with intent "no automatic MCU reset, ever, without user command." The recovery machinery that depended on those resets was never cleaned up, so whenever the bootrom set `watchdog_hw->reason` on a warm reboot (picotool `-f`, SWD reset), `check_watchdog_reboot()` returned true and the CLI printed a `[WARN] WATCHDOG REBOOT` line that had no real meaning. Surfaced on the station after a picotool cycle 2026-04-22; council review (NASA/JPL, ArduPilot, Rocketeer) agreed: remove outright, not rename.

**Removed:** `src/watchdog/` directory, scratch-register persistence (`kRecoveryMagic`, `recovery_pack_*/unpack_*/validate_magic`), reboot counter + `kSafeModeRebootThreshold` (safe-mode-on-3-reboots can't fire without reboot detection), `TickFnId` crash-tickfn diagnostics (only-written-never-read), `check_watchdog_reboot()`, `g_watchdogReboot`, `g_recovery` global, `kWatchdogSentinel` scratch write, `g_lastTickFunction`, `launch_abort_acked` field + `watchdog_recovery_ack_launch_abort()` (aspirational CLI ack, never wired), `rc_watchdog` CMake library, `test_watchdog_recovery.cpp`.

**Migrated (live behaviors preserved):** `launch_abort` → file-local static in `flight_director.cpp` with 2-function API (set + read). Level-3 safety posture per new `docs/USER_GUIDE.md` "Safety State Model" section: power-cycle-only clear. **No CLI command to clear, by design** — user veto on keystroke-clear for safety-critical launch abort (mirrors pad abort doctrine: stop, physically inspect, reset). ESKF runaway-restart brake → new `src/fusion/eskf_brake.cpp` (separate file so host tests link without SDK), 3-function API (`eskf_is_disabled / eskf_reenable / eskf_note_divergence`). HealthMonitor reads `eskf_is_disabled()` behind `job::kRoleSamplesCore1` constexpr guard for station builds.

**User Guide:** added three-level Safety State Model section — flight hold (auto-clears), safe mode (reserved, not implemented), launch abort (physical-intervention-required, power-cycle-only clear). Clarification content lives in docs; code comments cite the doc.

**In-commit cleanup:** `test_go_nogo.cpp` had pre-existing rot from Stage T Batch B (commit 3159173) — `AllGoReturnsAllGo` expected tier2_total=4 but "RF Link" added a 5th check. Fixed here rather than deferred: updated `all_go_input()` to populate `rf_anchor_valid`/`rf_link_state`/`rf_lq_pct`, corrected expected counts in 4 tests.

**Verification:** 786/786 host tests pass (8 new: 3 `LaunchAbortFixture`, 5 `EskfBrakeFixture`). 4 builds clean (vehicle bench, vehicle flight, station bench, station flight). HW boot-clean gate on vehicle via SWD probe: banner shows `Board: Adafruit Feather RP2350 HSTX`, build-hash matches HEAD, `Hardware: 14/14 OK`, no `[WARN] WATCHDOG REBOOT`, no `[WARN] LAUNCH ABORT`, preflight VERDICT GO.

**Reconstruction pointer:** if a future engineer needs the deleted safe-mode-on-rapid-reboot / TickFnId crash diagnostics / scratch persistence machinery, see this commit's parent and earlier. Deletion was deliberate (detect-what-we-can't-cause dead code), not refactoring accident.

`AGENT_WHITEBOARD.md` "Remove watchdog reboot entirely in favor of safe-mode" high-priority item closed.

Files: `src/watchdog/*` (deleted), `src/fusion/eskf_brake.cpp` (new), `test/test_flight_director_launch_abort.cpp` (new), `test/test_eskf_disable_brake.cpp` (new), `test/test_watchdog_recovery.cpp` (deleted), `CMakeLists.txt`, `test/CMakeLists.txt`, `test/test_go_nogo.cpp`, `src/main.cpp`, `src/active_objects/ao_flight_director.cpp`, `src/safety/health_monitor.cpp`, `src/cli/rc_os_commands.cpp`, `src/flight_director/flight_director.{h,cpp}`, `src/fusion/eskf_runner.{h,cpp}`, `docs/USER_GUIDE.md`, `AGENT_WHITEBOARD.md`.

---

### 2026-04-22-003 | Claude Opus 4.7 | docs, council, planning

**Stage 17 restructured from 5-IVP direct-to-flight to 13-IVP tapered buildup.**

User direction: first motor flight must be the **last** step, not the third — the equivalent of high-speed taxi testing and a runway hop before full test flights. Main validation targets: ESKF performance/accuracy, RCOS robustness, telemetry quality, all in progressively harder real-world conditions. Three council rounds (NASA/JPL, ArduPilot, Rocketeer, Cubesat) shaped the plan; final round approved with 6 amendments folded in.

New IVP list: IVP-135a (log schema extension + Tier 2 diagnostic payload + runtime log rate + Validation profile), IVP-135b (instrument self-test), IVP-136 (static gravity-vector accuracy with $40 angle-gauge fixture — addresses user's unresolved QGC return-to-level observation), IVP-137 (static heading accuracy), IVP-138 (parking-lot walk), IVP-139 (car-top ESKF shakedown), IVP-140 (drop/pendulum/arm-swing), IVP-141 (thermal + vibration), IVP-142 (RF pattern characterization), IVP-143 (bungee/tethered — default skip), IVP-144 (airframe integration), IVP-145 (static ground test at pad), IVP-146 (first motor flight), IVP-147 (exit gate).

Ground rules: three-band PASS/MARGINAL/FAIL acceptance, retrogression-on-FAIL with a 3-cycle architectural-review escape hatch, log-only data capture (telemetry is not pass/fail evidence), expected-to-balloon stage budget.

CCSDS rework unanimously deferred to post-Stage-17 — don't stack an unknown command layer with unknown real-world behavior on the same flight day; STOP-GAP retry is characterized (755 host tests, SPIN 11/11) and field-usable. Cubesat Engineer standing watch: promote CCSDS-first if field data shows STOP-GAP is actively hurting data capture.

`docs/ADVANCED_SETTINGS.md` gained the wizard opt-in principle block (single `Enable advanced settings? [y/N]` gate with tiered warnings; direct `.cfg` editing preserved for discoverability), two new Research Mode rows (`LOG_DIAGNOSTICS`, `LOG_RATE_HZ`), and a post-Stage-17 placeholder IVP for full advanced-settings flesh-out.

Files: `docs/IVP.md` (Stage 17 section rewritten + status table updated + stage preamble updated), `docs/ADVANCED_SETTINGS.md` (wizard principle + new rows), `docs/plans/STAGE17_TAPERED_BUILDUP.md` (new — plan stored in repo alongside other stage plans). Working copy of plan in `.claude/plans/tapered-buildup-airworthy.md` preserved. No code changes. Execution deferred to future sessions.

---

### 2026-04-22-002 | Claude Opus 4.7 | feature, firmware, radio, docs

**Stage T Batch C + close-out.**

IVP-T14c live retry indicator: new `CMD:` row on station dashboard with four display states (idle / `Try N/9` / `ACK Nms` / FAILED) and auto-clear hold windows. IVP-T13 LQ-adaptive retry deferred to post-CCSDS-rework (parametric tuning on top of a STOP-GAP command path is not worth it). ABORT paper-budget addendum added to T14 design doc covering single-shot, retry-fallback, and worst-case round-trips.

N=100 Wilson 95% CI attempted three ways — all instrumentation-bound non-results. Deferred to post-CCSDS-rework where "first-try" becomes meaningful under flow-controlled delivery. Record at `logs/stage_t/t14_wilson_ci_attempts.md`. Scope-bench rigor items (ACK-path timing, LDO rail scope session, RX-window vs Batch A binary) dropped from plan — council-approved beyond what's practical for the current bench setup.

Close-out: PROJECT_STATUS.md + AGENT_WHITEBOARD Stage T block marked COMPLETE; AO_ARCHITECTURE.md table updated (FD prio 9, new AO_RfManager row at prio 7); `docs/IVP.md` Stage M mislabel corrected (the actual Stage M is mag-cal, done long ago; the RF work has always been Stage T) with original framing preserved.

Commits: 59b5192, 9f0e04a, 8fdf951, 9426954, f0552a7, 7138fc0, plus this changelog commit.

---

### 2026-04-21/22-001 | Claude Opus 4.7 | feature, firmware, radio, hardware, council, docs

**Stage T Batch B implementation — complete. Work spanned 2026-04-21 evening → 2026-04-22 afternoon.**

*One-time format exception per user request: single entry, two-date header, covering two calendar days since Batch B landed as one continuous push.*

IVP-T14 landed per Round 2 council: station-TX now anchored to vehicle RxDone via `AO_RfManager`, a new AO owning learned-link state (LinkState kAcq/kTentative/kTrack/kTrackDegraded, sliding-window LQ%, anchor filter, deadman, forced-ACQ). Pure state-machine helpers in `src/safety/rf_link_health.h` with 32 host tests; SPIN model with 5 LTL properties (all pass, errors: 0).

IVP-T14a: operator-mashed-command dedupe (newest-wins, ARM excluded per Round 2 #5). IVP-T14b: retry instrumentation always on, surfaced in `q → s` diag dump. IVP-T14d: aggressive retry bumped to 8 × 250 ms for all tracked commands (was 3 × 500 ms); safety-class round-trip under ~2 s worst case.

FlightDirector pre-arm aggregator extended with RF link-health via `AO_RfManager_get_state()` (cooperative-dispatch-only invariant, Commandment V). New "RF Link" Tier-2 warn station; existing "Radio" renamed "Radio HW". Dashboard gains a single-line RF Link row with `[OK]/[--]/[!!]` glance indicator matching pre-arm thresholds.

Vehicle-lost notification wiring (Round 2 #10): `SIG_NOTIFY_VEHICLE_LOST/_FOUND` posted by AO_RfManager on transition edges; AO_Notify latches `NotifyState.vehicle_lost`. Audio backend picks it up automatically when TLV320DAC3100 driver lands.

Dashboard keys: `a` (ARM-confirm) and `D` (DISARM, single-key) accepted in `kAnsi` now, not just `kMenu`. `m`/`M` removed from dashboard. ABORT deliberately not bound from dashboard (NAR High Power Safety Code §6: ballistic post-ignition).

*Rationale — CCSDS station beacon parked rather than built:* planned for T14d as 1 Hz uplink heartbeat, endorsed by a blind 3-panelist council. On re-read, the council's reasoning implicitly assumed separate channels we don't have — continuous uplink collides with vehicle's 5 Hz nav on our single shared half-duplex radio. Correct CCSDS / smallsat practice is downlink-inferred liveness + on-demand operator checks. APID reservation parked with a STOP-GAP marker. All Stage T command-path code flagged as stop-gap pending full CCSDS layer (COP-1 still not-pursued, `docs/decisions/COP1_NOT_PURSUED.md`).

*Rationale — RP2350-E2 silicon erratum fix:* a trivial diff in `ao_rcos.cpp` deterministically triggered a boot-time HardFault. Research identified **RP2350-E2** (SIO spinlock mirror writes, datasheet p.1373) interacting with the SDK's software-spinlock EXTEXCLALL bit. Reporter-confirmed fix `PICO_SW_SPIN_LOCKS_NO_EXTEXCLALL=1` applied, `PICO_PLATFORM`-gated. Affects both RP2350A (Feather) and RP2350B (Fruit Jam) — shared SIO logic. References: pico-sdk #2495, #2706, #1812.

Also: LL Entry 25 marked SUPERSEDED (picotool-corrupts-I2C diagnosis no longer accurate; fixes in LL Entries 28 + 31); content preserved. Long-standing host test rot fixed (`FlashLayout.SectorCount`, `FlightTable.CapacityEmpty` — 1915 → 1913 after Stage T IVP-T5.5 added radio-config sectors). Host tests 791/791.

High-priority whiteboard items added: deep RP2350 errata sweep + `docs/hardware/RP2350_ERRATA.md` (E2 slipped through onboarding); remove watchdog reboot in favor of safe-mode; re-evaluate Stage T 95% gate with correct baseline (the 6.7% number may have been operator-burst latency, not link failure).

Stage T Batch B code is complete at this commit. Remaining Batch B plan items are bench-equipment sessions. Field gate deferred to pre-flight tuning stage per user direction. Batch C (LQ-adaptive retry, live retry indicator) code-implementable; flag-default-off.

Commits in this chain (chronological, `git log origin/main..HEAD`): 1748fd9, 35f9591, e78df5e, 494dba0, 774a164, c888743, b2a20d3, ecd90d9, b7c0ae4, 093e29f, 6d6eab2, a4908ab, 64fd99d, a267ff4, e1e5c7f, 031d776, 3159173, 00c173a, 1d9e16f, 63d8a1b, 8d2ed74, e5fd105, 45ab8a7, 5bbe743, 5adb878.

Files: `src/active_objects/{ao_rf_manager,ao_notify,ao_telemetry,ao_radio,ao_rcos,ao_flight_director}.{h,cpp}`, `src/safety/{rf_link_health.h,health_monitor.cpp}`, `src/flight_director/go_nogo_checks.{h,cpp}`, `src/cli/{rc_os.{h,cpp},rc_os_dashboard.cpp}`, `include/rocketchip/{notify_intents.h,telemetry_encoder.h}`, `test/{test_rf_link_health.cpp,test_flight_table.cpp}`, `tools/spin/rocketchip_rf_manager.pml`, `CMakeLists.txt`, `docs/ROCKETCHIP_OS.md`, `docs/plans/STAGE_T_T14_DESIGN.md`, `AGENT_WHITEBOARD.md`, `.claude/LESSONS_LEARNED.md`, `logs/stage_t/t12_summary.csv`.

---

### 2026-04-21-003 | Claude Opus 4.7 | bugfix, firmware, radio, testing

**Stage T Batch B prelim — runtime radio config actually adapts now. Three hardcoded-to-default values fixed.**

Discovered during T14 pre-Batch-B RX-window instrumentation: **vehicle at "BW500 10Hz" config was actually TXing at 5 Hz.** Dashboard reported 10 Hz on both halves; station inter-arrival PDF showed 5 Hz (4.44 Hz observed, 1 seq per RX, 0 drops). Root cause: `AO_Telemetry::interval_ms` was hardcoded to 200 ms in `TelemAo_initial` and only updated by the CLI `r` (cycle-rate) key — SET_RADIO_CONFIG's apply path updated the `RadioConfig` struct and the dashboard string but never touched the TX cadence gate. Bug predates Stage T; made every previous test's `nav_rate_hz` change cosmetic. Pre-fix T12 C2 "BW500/10Hz" data was BW500 but ~5 Hz.

User direction 2026-04-21: "since the configurable radio settings during runtime were fairly new we need to be sure the things that were previously hard coded under assumptions actually adapt to the settings naturally." Audit found 3 offenders; all fixed:

1. **`AO_Telemetry::interval_ms` now scales with `nav_rate_hz`.** New `AO_Telemetry_set_rate()` setter called from `ao_radio_apply_runtime_config()`. Verified on bench: vehicle at idx 3 (BW500/10Hz) TXes at 9.80 Hz; at idx 4 (BW125/2Hz) TXes at 2.10 Hz. Pre-fix was 5.00 Hz regardless.

2. **`rfm95w::kTxTimeoutUs` (150 ms hardcoded) now airtime-scaled.** New `rfm95w_airtime_us()` implements SX1276 datasheet §4.1.1.6 (T_preamble + T_payload). `rfm95w_set_tx_timeout_us()` sets a per-device threshold = `2 × max-payload airtime`. `ao_radio_apply_runtime_config()` pushes the value whenever config changes. SF7/BW500 worst case ≈140 ms timeout; SF12/BW125 worst case scales appropriately.

3. **`kAckRetryTimeoutMs` (500 ms hardcoded by IVP-T7) now airtime-scaled.** Replaced static constexpr with mutable `s_ack_retry_timeout_ms`. `AO_Telemetry_set_ack_retry_timeout_ms()` setter called from the same apply path; formula is `4 × airtime + 50 ms`, clamped to [200, 1000]. Suits both fast configs (BW500/SF7 ≈ 330 ms) and long-range presets (BW125/SF12 hits the 1 s ceiling).

**Validation boundary also broadened** per user: "the preconfigured data rates are just presets... the user should really be able to change it to anything the radio can do." `radio_config_table.h` now exports two validators:
- `radio_config_in_whitelist()`: preset-membership check (unchanged). Still used for digit-key debug menu + channel-find scanner + boot seed.
- `radio_config_sx1276_legal()` (new): broader hardware-legal check for advanced-settings path. `dispatch_set_radio_config` and the flash-storage `validate_entry` now use this.

**Gotcha flagged throughout**: all α-filter / window-width / jitter numbers measured on this prelim are **static bench at room temperature**. Real-rocket thermal swing, vibration, Doppler, antenna motion will shift them. Every numeric value captured here must be re-validated against field data. Explicit marker in design doc; will carry through Round 2 council + Batch B.

Files: `rfm95w.h/cpp` (tx_timeout_us field + `rfm95w_set_tx_timeout_us()` + `rfm95w_airtime_us()`), `ao_telemetry.h/cpp` (set_rate + set_ack_retry_timeout_ms), `ao_radio.cpp` (calls both setters in apply path; T14 instrumentation: `stage_t_log_tx_start/tx_done`), `radio_config_table.h` (sx1276_legal validator), `radio_config_storage.cpp` (broader validator on flash read). 4-tier + 2-tier-instrumented builds clean. Host tests 757/759 (same pre-existing). Bug captures: `logs/stage_t/t14_rxwindow_C0.log`, `t14_rxwindow_C2.log` — C2 shows the 5 Hz bug pre-fix.

### 2026-04-21-002 | Claude Opus 4.7 | docs, council

**Stage T Batch B prelim — `docs/decisions/AO_COMMANDMENTS.md` + retroactive scan of existing AOs.**

Research pass over Samek PSiCC2, state-machine.com (QP framework), Douglass, NASA F´, and NASA cFS produced the conclusion that no external JSF-AV-grade *application-level* AO ruleset exists — Samek gives 3 principles, F´ adopts JPL Power-of-10 (already our standard), state-machine.com's SRS_QP_AO_* specs cover framework internals not application design. Correct deliverable tier is advisory, not normative.

`AO_COMMANDMENTS.md` codifies 12 rules, each sourced to Samek/QP (with URLs) or to LL Entries 32/35 where external sources are silent. Rules cover data isolation, async messaging, run-to-completion, no-blocking, the `const*` read-only-accessor cooperative-dispatch-only invariant (critical for `AO_RfManager` coming in Batch B), static-event discipline (LL Entry 35), AO boundary / one-responsibility (the rule behind `ok_to_arm()` being in FlightDirector not AO_RfManager), AO promotion criteria, priorities-set-at-boot, boot-time subscriptions, timer-event locality, and AO-decision instrumentation.

Retroactive scan of the 7 existing AOs (AGENT_WHITEBOARD.md): Commandment VI clean (every `QACTIVE_POST` uses `static` events, LL Entry 35 discipline universally applied); Commandment IV has 3 documented deviations (`calibration_save()`, `cli_do_erase_flights()`, `tick_persist_debounce()` when `ROCKETCHIP_RADIO_PERSIST` is defined) — all interactive/infrequent with in-code rationale, none escalated to an IVP. Batch B is not blocked by any pre-existing violation.

Elevation path documented: if the project grows or a Commandment violation is traced to a bug, the file moves to `standards/ACTIVE_OBJECT_RULES.md` and becomes normative.

### 2026-04-21-001 | Claude Opus 4.7 | feature, firmware, testing, council, docs

**Stage T continuation Batch A — IVP-T11 SX1276 register hygiene + IVP-T12 manual regression sweep.**

Supersedes the prior T5-T10 fix plan. Council review (NASA/JPL, ArduPilot, Cubesat, Rocketeer) concluded retry-timer changes are not the load-bearing fix; architectural RxDone-anchored TX (T14, Batch B) is. Batch A lands driver hygiene only; Batch B (T14 `AO_LinkHealth` state machine) comes next.

**IVP-T11 driver hygiene.** `src/drivers/rfm95w.{h,cpp}` now writes `RegLna = 0x23` (LnaGain=001 G1 max, LnaBoostHf=11 for +3 dB on 915 MHz HF port — SX1276 datasheet §5.5.3) and `RegModemConfig3 = 0x04` (LowDataRateOptimize=0 for SF≤10 at BW≥125, AgcAutoOn=1 adaptive gain — §5.4.3) in `configure_modem()`. Matches ArduPilot AP_Radio default since 2018; long-standing driver omission on our side. New `rfm95w_read_audit()` reads back RegInvertIQ (0x33), RegModemConfig2, RegLna, RegModemConfig3 and snapshots into `RadioAoState::boot_audit` post-init. `cmd_radio_status()` (the `t` CLI key) displays the audit every call so it's verifiable from any post-boot serial session.

**IVP-T12 regression sweep.** Manual runbook (`docs/plans/STAGE_T_IVP_T12_RUNBOOK.md`) executed 2026-04-21 with both boards on `bench-1591794`. Four configs covering single-variable changes between rows:

| Config | BW | Nav | first-try | eventual | Variable vs previous |
|--------|---:|----:|----------:|---------:|----------------------|
| C0  | 125 | 5  | 13.3% | 36.7% | baseline |
| C0P | 125 | 10 | 16.7% | 30.0% | nav rate only |
| C1  | 250 | 10 | 53.3% | 96.7% | BW only |
| C2  | 500 | 10 | 83.3% | 96.7% | BW only |

Clean attribution: bandwidth dominates. Nav rate +5 Hz at BW125 = +3.4 pp first-try; BW doubling = +30-37 pp. T1's "TX airtime > RX window" diagnosis confirmed. No T7-style link brittleness recurred across any config; the earlier "link won't reform at BW250/500" was a procedural bug — the harness wasn't transitioning the station from kAnsi to kMenu before sending `q<idx>z`. Cross-session variance is large (T6 saw C2 at 100%, T12 at 83.3%); not yet explained. Batch A gate C2 ≥ 95% NOT MET (83.3%), as expected per council — T14's RxDone-anchored TX is the architectural fix, not T11.

**Supporting work.** `docs/ROCKETCHIP_OS.md` extended with a station UI-modes section (kAnsi / kCsv / kMavlink / kMenu) documenting the 2-key rule (`m` and `x` only accepted in kAnsi), which is the rule behind several debugging dead-ends this session. Council transcript landed at `docs/decisions/STAGE_T_CONTINUATION_COUNCIL.md`. 4-tier builds clean (vehicle bench, vehicle flight, station bench, station flight). Host tests 757/759 (same pre-existing failures; no new failures).

Batch A lands to main but does NOT fly until Batch B's 500 m field gate passes (per NASA/JPL council direction).

### 2026-04-20-001 | Claude Opus 4.7 | feature, architecture, refactor

**Stage T IVP-T5.5 sub 2e/2f/2g — runtime radio config push completes: QUERY echo, APID 0x004 nav-with-config, dashboard row, station auto-revert, WS2812 KITT sweep.**

Closes out the T5.5 feature block that started with the sub 2a-2d apply/revert machinery and sub-persist flash write. All load-bearing pieces of the "station commands vehicle to change LoRa config at runtime" path are now in tree.

**sub 2e — QUERY_RADIO_CONFIG** extends `CommandAckPayload` from 5→10 bytes with cfg echo (bw/nav/sf/cr). Populated only on accepted USER_3 responses, zeroed otherwise. `kCmdAckPacketLen` bumped 17→22. Pattern is flight-heritage (CCSDS CLCW, MAVLink COMMAND_ACK progress, CFDP FIN PDU) — ACK stays the same APID, payload grows with metadata. Station prints `[CMD] vehicle config: BW=.. nav=.. SF=.. CR=..` on matched ACK.

**sub 2f — APID 0x004 nav-with-config + dashboard + station revert.** New APID `kApidNavWithConfig = 0x004` (vehicle always emits it now; legacy 0x001 decoder path kept for cross-version graceful reject). Payload grows 42→46 B (packet 54→58 B). 4-byte config tail: `[bw_hi bw_lo] [sf<<4|nav] [cr<<4|flags]` with `kCfgFlagJustChanged` bit 3. New `s_config_just_changed` latch — set by commit and revert, consumed once by encoder via `AO_Radio_consume_just_changed()`. Dashboard gets "Radio: ..." row showing both station and vehicle configs with yellow-on-mismatch and cyan-`[CHANGED]` for the single-frame transition marker. Station auto-revert threshold is role-aware: vehicle keeps `max(15, 3×nav_hz)`, station/relay uses `max(6, ceil(1.5×nav_hz))` so the operator sees failure first visually while vehicle is the final fallback. Fixed latent rot in `validate_ccsds_crc` (hardcoded CRC offset 52 → length-aware `len-2`, works for both packet sizes).

**sub 2g — WS2812 KITT sweep during LOS watchdog.** New `ws2812_set_sweep_bar(color)` — single pixel walks back-and-forth at caller-throttled cadence, handles 1-LED strip + 0-LED edge. `handle_rssi_bar()` dispatches dim-yellow sweep at 20 Hz while `apply_in_progress`, falls back to 2 Hz RSSI bar once watchdog clears.

JSF-AV rule-1 decomposition landed as part of the same commit: `handle_radio_tick` split into 3 tick helpers; `try_handle_cmd_ack` early-returns on non-match with 2 station-side extractions; `try_mavlink_rx` split into `dispatch_set_radio_config` / `dispatch_command` / `stage_cmd_ack` / `handle_parsed_mavlink`. `scripts/bench_sim.py` + `scripts/station_bench_sim.py` taught to distinguish vehicle vs station by banner text (fixes spurious pre-commit gate failures when both boards are plugged in — the Stage T dev pattern).

Live-verified over RF: APID 0x004 round-trip (station Pkts climbing 3265→3269, zero CRC errors after the validate_ccsds_crc fix), dashboard Radio row renders both configs with mismatch highlight, `[CHANGED]` marker appears for exactly 1 frame (50 captured → 1 showed CHANGED) when `s_config_just_changed` flipped on vehicle via GDB, vehicle apply+commit path verified BW125→BW250 via break in `ao_radio_commit_pending_config`. USB-deferred (pending T6/T7 reliable SET delivery): station-side auto-revert triggered end-to-end, WS2812 KITT sweep eyeball confirmation on Fruit Jam. Host tests 757/759 (2 pre-existing unrelated failures). 4 new APID 0x004 round-trip tests.

Tracking detail in `logs/stage_t/t5.5_revalidation_list.md`. Phase T continues — T6 (BW sweep via runtime push instead of compile-flag rebuilds) and T7 (retry-timer compression) next.

### 2026-04-18-002 | Claude Opus 4.7 | feature, architecture, council, refactor

**Stage L COMPLETE — LED engine + notification polish (AP parity, beacon overlay, pre-arm fail, boot init rainbow, GCS beacon command).**

Seven IVPs + one full-tree JSF-AV sweep. Plan `.claude/plans/shimmering-twirling-thimble.md` council-reviewed before implementation (NASA/JPL, ArduPilot, Rocketeer, Cubesat — unanimous).

Two new driver modes (`WS2812_MODE_ALTERNATE` for beacon overlays, `WS2812_MODE_DOUBLE_FLASH` for pre-arm fail, AP parity). Five new pattern codes for fault-beacon composition (kFault{Imu,Eskf,Baro,PioWdt,Core1Stall}Beacon). Two orthogonal flags on `NotifyState`: `beacon_manual` forces pure-white 2Hz (CLI `b` key + `MAV_CMD_USER_1` over radio); `beacon_auto` preserves state color via +white alternate (automatic triggers). Both clear on `SIG_PHASE_CHANGE` out of {LANDED, ABORT}. New `PhaseIntent::kPreArmFail` (yellow double-flash, 3s auto-clear via pure helper `include/rocketchip/prearm_fail_ticks.h`). New `PhaseIntent::kInit` (boot rainbow with min-visibility gate essential for warm resets). AP-parity color swaps: ARMED amber→red, cal gyro/level blue-breathe→yellow-blink. Station/vehicle LED role divergence preserved and documented (station RSSI bar vs vehicle flight-state display, LL Entry 32).

Full-tree clang-tidy sweep (LL Entry 36 discipline) surfaced 5 latent JSF-AV rule-1 function-size violations — decomposed `flight_director_evaluate_guards`, `AO_Logger_populate_fused_state`, `handle_rx_packet`, `core1_sensor_loop`, `guard_evaluator_tick`. Added `src/dev/**` to pre-commit hook whitelist alongside `src/cli/**`. `SESSION_CHECKLIST.md` item 17 added to run full-tree sweep at milestone close so the next accumulation gets caught on the way in, not after it's compounded.

755/755 host tests (up from 724; 31 new — 9 prearm-helper, 14 beacon overlay, 5 pre-arm resolver, 3 boot-init). 4 builds clean, SPIN FD 7/7 errors=0, bench_sim 2/2 + 3/3 (station requires `--port COM9`; auto-detect script limitation tracked in whiteboard), vehicle 5-min soak PASS (1.04M IMU reads, 1 transient I2C error, 0 baro errors, MCU 36.5 °C stable).

Station→vehicle end-to-end beacon roundtrip not verified: vehicle-side resolver chain proven via GDB `set var beacon_manual=true` → pure-white 2Hz confirmed, but live radio test showed vehicle `rx_count=0` from station TX attempts. Consistent with the pre-existing RadioScheduler TX-window sync issue (IVP-132a.5 quantified 6.7% first-try ACK rate in Stage 16B). Tracked as new **Stage M** — "RadioScheduler TX-Window Synchronization" — with proposed "listen-before-talk" fix requiring plan + council review.

Documentation: `docs/AO_ARCHITECTURE.md` LED section + SIG_BEACON_MANUAL row; `docs/SCAFFOLDING.md` module rows for ws2812_status + ao_notify; `docs/decisions/NOTIFY_CONTRACT.md` full Stage-L extensions section (composition table, AP-parity table, signal catalog); `docs/IVP.md` new Stage L + Stage M stub sections.

Commits: `1d231b1` (IVP-L1), `a9f538a`+`9fe8061`+`58575e8` (sweep), `7a26beb` (IVP-L2), `d559703` (IVP-L3), `57d266a` (IVP-L4), `c449e6b` (IVP-L5), and the L7 exit commit (this changelog entry).

---

### 2026-04-18-001 | Claude Opus 4.7 | feature, architecture, council

**Stage 16C COMPLETE — station runtime decoupling + MCU die-temp + station HealthMonitor parity + board scaffolding.**

IVP-142b-3 refactored `health_monitor_critical_fault()` with per-subsystem 5-tick persistence counters + phase gate. Primary-byte faults count as critical for auto-action only after 500 ms consecutive fault AND phase != IDLE; prevents false auto-DISARM from transient noise (dust in baro vent, transient I2C NACK). Threshold-bound critical byte bits (MCU over-temp) propagate regardless of phase. 6 new host tests. Council-reviewed (NASA/JPL + Professor + ArduPilot + Rocketeer).

IVP-142c added **station HealthMonitor parity** via capability-masking (not role-gating). Council-reviewed (NASA/JPL + Professor + Rocketeer). New `include/rocketchip/job_capabilities.h` with `kRoleSamplesCore1` / `kRoleRunsLogger` / `kRoleHasFullGoNogo` predicates. `check_core1_vitality()` short-circuits to true on roles that don't sample Core 1 (station, relay). `evaluate_secondary()` masks `kHealthFlashOk` when Logger isn't expected. `AO_HealthMonitor` now starts unconditionally on every non-Relay role. New `g_imuInitAttempted`/`baroInitAttempted`/`gpsInitAttempted` flags drive `[FAIL]` vs `[N/A]` presentation for uninstalled-sensors case. `evaluate_gps()` startup race fixed (read_count==0 now `kHealthAbsent`, was `kHealthFault`). Station HW gate PASS (no QP assertions over 5 min, correct health state).

IVP-143 landed capability flags (`kPsramAvailable` / `kDvmAvailable` / `kSdCardAvailable` / `kI2cStemmaAvailable`) on existing boards + scaffolding for Pimoroni Tiny 2350+ (`board_tiny_2350_common.h` + `board_tiny_2350_plus.h`) and Raspberry Pi Pico 2 (`board_pico2.h`) with `#error` bring-up guards (`TINY_2350_BRINGUP_OK` / `PICO2_BRINGUP_OK`). `CMakePresets.json` encoding the 4 current build combos. Comment/string hygiene in `ud_benchmark.cpp` (uses `board::kBoardName`) and `rc_os_commands.cpp` (generic "module unpopulated?" vs FeatherWing-specific).

IVP-144 audit-only: grep-validated `cmd_station_*` paths clean of hardcoded board names.

IVP-145 exit gate: 4 builds clean, 724/724 host tests, bench_sim.py 2/2 PASS, vehicle 5-min soak PASS (IMU 1153/s, baro 36/s, 0 errors, health primary=0xaf secondary=0x1f critical=0x00 mcu=OK go_nogo=READY), station 5-min soak PASS (covered in IVP-142c HW gate).

Station/vehicle disparities tracked for follow-up in `AGENT_WHITEBOARD.md`: bench_sim asymmetry (new IVP-146 to land `station_bench_sim.py`), station SPIN model gap (new IVP-147), pre-commit complexity thresholds not classification-aware (Ground CLI held to Flight standard), pre-commit bench_sim gate not role-aware (station-only diffs trigger vehicle probe requirement).

Files: `src/safety/health_monitor.{h,cpp}`, `src/main.cpp`, `src/cli/rc_os_commands.cpp`, `src/core1/sensor_core1.h`, `src/benchmark/ud_benchmark.cpp`, `test/test_health_monitor.cpp`, `include/rocketchip/job_capabilities.h` (new), `include/rocketchip/board.h`, `include/rocketchip/board_feather_rp2350.h`, `include/rocketchip/board_fruit_jam.h`, `include/rocketchip/board_tiny_2350_common.h` (new), `include/rocketchip/board_tiny_2350_plus.h` (new), `include/rocketchip/board_pico2.h` (new), `CMakePresets.json` (new), `docs/IVP.md`, `docs/PROJECT_STATUS.md`, `AGENT_WHITEBOARD.md`.

---

### 2026-04-17-002 | Grok 4.2 (dev) Claude CLI (QA) | bugfix, hardware

**Stage 16C Fruit Jam PA1010D GPS I2C detection restored.**

- Fixed regression on adafruit_fruit_jam: PA1010D (0x10) never ACK'd on STEMMA QT I2C0 (GPIO 20/21) while DAC (0x18) was also silent.
- Root cause: original STEMMA QT cable had broken SDA/SCL (power/GND intact → GPS power LED solid, masking fault).
- Added `kPeripheralResetPin = 22` + `board_release_peripheral_reset()` in `include/rocketchip/board_fruit_jam.h` (releases shared DAC/ESP32-C6 RESET active-low).
- `gps_pa1010d_init()` moved ultra-early into `init_early_hw()` (`src/main.cpp`) with blind PMTK314/PMTK220 sequence, 20 ms settle, aggressive 8× retry probe loop, and static PMTK write-result buffer (shown in `b` Hardware Status).
- Reverted Stage 16C IVP-140 scaffolding commit (`26b83d4`) — gate claim was false-positive (station I2C broken at time of that commit, issue not surfaced). Stage 16C restart to follow with more robust HW-gate verification.
- Verified: `[PASS] GPS init (I2C at 0x10, 500 µs settling delay)`, PMTK writes return full byte counts `[51,18,51]`, `window_hit:1`, NMEA streaming, stable alongside DAC. i2c scan still skips 0x10 per LL Entry 20.

(`include/rocketchip/board_fruit_jam.h`, `include/rocketchip/board_feather_rp2350.h`, `src/drivers/gps_pa1010d.cpp`, `src/drivers/gps_pa1010d.h`, `src/cli/rc_os_commands.cpp`, `src/main.cpp`, `CMakeLists.txt`, `src/station/station_idle_tick.{h,cpp}` deleted)

### 2026-04-17-001 | Claude Code CLI | testing, architecture, tooling, council

**Stage 16B bench validation COMPLETE (IVP-132, 132a).** Vehicle 30-min
flight-binary battery soak GATE PASS (IMU 997 Hz, 0 errors across 2.25M
reads, MSP 0 drift). Station (Fruit Jam) bench test suite: fault
injection hooks, diag_stats extensions, replay harness, idle + integration
soaks. IVP-132a.4c 30-min station flight-binary integration soak GATE
PASS (9023 packets / 0 CRC errors / 0 SPI errors / MSP 0 drift, 5.01 Hz
sustained). IVP-134 pre-flight checklist document written. Council
review (NASA/JPL, Professor, ArduPilot) added council-required
preconditions: `diag_stats_t0_preconditions()` (RegVersion readback,
identity strings, NVIC ISPR), `g_spi_error_count` hot-path counter,
per-AO ring-index activity proxy. Frankenstein-build guidance added to
`docs/BENCH_TEST_PROCEDURE.md`.

**Process lessons committed to memory:** (1) verify council test premises
against current firmware architecture before implementing — IVP-132a.4a
DIO0 IRQ test premise didn't match our polling radio driver and was
removed after implementation; (2) "absence of signal is not proof of
health" — a test that watches derivative metrics (queue watermarks)
without verifying the peripheral is actually engaged can claim PASS on a
non-functional binary.

**Known gap flagged:** IVP-132a.5 ACK stress characterized the
RadioScheduler-sync issue — 6.7% first-try ACK rate because station TX
isn't aligned with vehicle RX windows. Fix scheduled for Stage 16C.

**Deferred:** Stage 16C (station rework — runtime decoupling, board
decoupling, RadioScheduler sync, MCU die temp sensor), Stage 17 (field
testing — airframe + launch window dependent), real-world accuracy tests
(dedicated plan TBD). All recorded in `AGENT_WHITEBOARD.md` and
`docs/IVP.md`.

---

### 2026-04-15-002 | Claude Code CLI | testing, safety, tooling

**Stage 16B bench testing (IVP-129, 130, 131).** GDB fault injection harness with 7 hook functions (`src/dev/fault_inject.{h,cpp}`), procedure guide (`docs/FAULT_INJECTION.md`), 5 GDB scripts. PIO backup timer shakedown — all 5 scenarios HW verified (Core 0 stall proven independent of PIO timers, PIO SM halt gap documented for Gemini tier). Pyro edge logger in flight binary (`src/safety/pyro_edge_logger.{h,cpp}`). Sensor replay harness — 5 Big Daddy F15-6 profiles (nominal, early burnout, IMU fault, baro dropout, GPS dropout) all reach kLanded via full FD state sequence. Real NAR thrust curve from ThrustCurve.org. Council reviewed (NASA/JPL, Rocketeer, Space Camp Counselor, Professor, ArduPilot).

---

### 2026-04-15-001 | Claude Code CLI | documentation, refactor, tooling

**Stage 16B front-loaded cleanup (IVP-124a, 125, 126, 127, 127a, 127b).** IVP.md restructured into Stage 16B/17/18. SAD + SCAFFOLDING superloop purge (AO-first rewrite). Doc-drift checkpoint in SESSION_CHECKLIST. Dev code audit + build-tier split (`BUILD_FOR_FLIGHT` CMake option, `src/dev/` directory). Version string consolidation into `version.h` (firmware 0.16.0, RCOS 0.5.0, git hash). Council-reviewed plan: `.claude/plans/stateless-hopping-allen.md`. 709/709 host tests.

---

### 2026-04-12-003 | Claude Code CLI | documentation, architecture

**Stage 15 complete (IVP-122–124) + Stage 16A pre-bench (IVP-127–130).** Station radio hardening and documentation refresh.

**Stage 15 (IVP-122–124):** Half-duplex ACK protocol (CCSDS APID 0x003 CommandAckPayload, station ARM confirm UX: `a` → type ARM → Enter, `X`-DISARM with ACK tracking, 3 retries × 3s). Distance-to-rocket (haversine + bearing, freshness check). Station help refresh. Radio command delivery is unreliable due to RX window timing — RadioScheduler sync IVP needed. 709/709 host tests.

**Stage 16A pre-bench (IVP-127–130):** AO architecture audit — removed dead Core1 vitality fields from AO_Notify (IVP-117 leftovers never wired), documented 2-layer model in AO_ARCHITECTURE.md. TBD placeholder purge — filled memory budget (flash 152KB, SRAM 134KB/520KB), power estimates from datasheets, diagnostics packet candidates. Runtime Behavior Map complete rewrite for QV architecture (superloop→8 AOs, idle bridge, event flow, health escalation, multi-channel landing, station CLI). User Guide quick-reference card (LED states, ARM/DISARM, troubleshooting). IVP-125/126 (SAD/SCAFFOLDING superloop purge) deferred to post-bench per council sequencing.

**5 Graphviz architecture diagrams** in `docs/audits/cla_rbm/dot/`: boot sequence (updated for QF_run), AO event flow (new — all 8 AOs + signals + seqlock + idle bridge), Flight Director HSM (new — 9-state machine with descent superstate + P7 multi-channel landing), cross-core comms (updated for HealthMonitor/LedEngine vitality reads), error recovery (updated with health escalation + radio ACK + landing paths). References added to AO_ARCHITECTURE.md and RUNTIME_BEHAVIOR_MAP.md.

---

### 2026-04-12-002 | Claude Code CLI | architecture, safety, verification

**Stage P7 complete: MAIN_DESCENT Liveness Fix (IVP-119 through IVP-121).** Out-of-sequence safety pass closing SPIN property P7 (flight-completes liveness), which had been known-failing since Stage 9.

**IVP-120: Baro-stationary guard.** New `guard_baro_stationary()` fires `SIG_LANDING` when raw baro altitude rate stays below 0.3 m/s for 5 seconds. ESKF-independent — reads `baro_alt_rate_mps` computed from raw DPS310 pressure delta via hydrostatic approximation in `AO_Logger_populate_fused_state()`. Three new MissionProfile fields: `baro_landing_rate_threshold_mps`, `baro_landing_sustain_ms`, `descent_max_duration_ms`. 8 new guard tests, 694/694 total. HW verified: GDB `baro_alt_rate_mps = -0.208 m/s` stationary (DPS310 noise floor). bench_sim 2/2 PASS.

**IVP-121: Multi-channel landing detection + SPIN P7 resolved.** Two new paths in `flight_director_evaluate_guards()`: (1) ESKF-fault + baro-stationary conjunction — ESKF must be confirmed dead AND raw baro must agree we're stationary. Conjunction, not disjunction. (2) Last-resort backstop — `descent_max_duration_ms` elapsed, all physical channels silent, fires `SIG_BEACON_ACTIVE` for recovery. Beacon callback (`beacon_cb`) wired through `ao_flight_director.cpp` using static `QEvt` (LL Entry 35 safe).

**SPIN model rework: Discrete-Time Promela (Tripakis & Courcoubetis, 1996).** Bounded tick counters added to ALL flight phases (BOOST, COAST, DROGUE_DESCENT, MAIN_DESCENT) modeling physical inevitability: motors burn out (finite propellant), PIO timers fire (hardware countdown), backstops expire (wall-clock). When counter >= limit, exit transition is forced with no skip option. This is the accepted pattern for timed liveness in SPIN, consistent with NASA JPL DS1 flight software verification (Gluck & Holzmann, 2001). **All 8 SPIN properties pass (0 errors) including P7 liveness.** SPIN README updated with DT-Promela references, P7 documentation, and Cygwin gcc DLL dependency note.

5 new host tests for MAIN_DESCENT paths (conjunction, backstop, conjunction-alone-stays, backstop-before-timeout, baro-stationary-nominal). 699/699 total. HW verified: bench_sim 2/2 PASS.

**SPIN abort-pyro model inaccuracy discovered.** `rocketchip_fd.pml` unconditionally fires drogue on ABORT from BOOST/COAST, but firmware gates this on `MissionProfile::abort_fires_drogue_from_boost/coast` (default `false`). Firing drogue at high speed is a shred event — the profile default is physically correct. Flagged on whiteboard for future model update.

---

### 2026-04-12-001 | Claude Code CLI | tooling, architecture, bugfix

**Bench sim retirement + IVP-119 FusedState baro field rename.** Two council-reviewed plans executed across two sessions.

**Bench sim retirement:** Old `bench_flight_sim.py` (479 lines, IVP-73) retired after discovery of 5-day silent bit-rot (2026-04-06 to 2026-04-11). Replaced by `scripts/bench_sim.py` (~200 lines, 2 tests: happy path + abort-from-BOOST). SPIN-provable state machine safety not re-tested (redundant). New script auto-detects serial port (threaded probe with 3s timeout per port), connect retry (5 attempts), 120s deadline, health wait before test. New `test/test_command_handler.cpp` (8 host tests) fills the accept/reject matrix gap. LL Entry 36 documents root-cause pattern: "soft gate presented as hard gate." Pre-commit hook extended with `ctest` gate + needs-based HW bench sim gate (runs bench_sim.py when staged diff touches flight-critical paths and probe is available; loud prompt when probe unavailable). Session-start canary added to SESSION_CHECKLIST.md item 6.

**IVP-119 (Stage P7):** `FusedState::baro_vvel` renamed to `vert_vel_eskf` (was actually `g_eskf.v.z`, not a raw baro derivative). New `baro_pressure_pa` field added for ESKF-independent baro sensing (foundation for IVP-120 `guard_baro_stationary`). Mechanical rename across 12 files. `guard_baro_peak()` parameter renamed; no body/logic change. Wire format `TelemetryState::baro_vvel_cms` deliberately preserved. HW verified: bench_sim 2/2 PASS, GDB confirmed `baro_pressure_pa` = 100398 Pa (live DPS310).

**SPIN model inaccuracy discovered:** `rocketchip_fd.pml` unconditionally fires drogue on ABORT from BOOST/COAST, but firmware gates this on `MissionProfile::abort_fires_drogue_from_boost/coast` (default `false`). Firing drogue at high speed is a shred event — the profile default is physically correct. SPIN model + code comment update tracked on whiteboard for IVP-120/121.

---

### 2026-04-10-003 | Claude Code CLI | documentation

**Stage 15/16/17 reorganization in IVP.md.** Split old Stage 15 "Pre-Flight Polish" into two stages after scope re-evaluation: new Stage 15 "Pre-Flight Radio + Station" (MAIN_DESCENT P7 timeout fix, half-duplex ACK + ARM confirmation UX, distance-to-rocket finish, station help/whiteboard cleanup) runs before the polish pass. Old Stage 15 becomes Stage 16 "Pre-Flight Polish" with three phases: 16A Documentation & Cleanup, 16B Bench Testing, 16C Field Testing. Old Stage 16 "Field Tuning & Validation" renumbered to Stage 17. Audio output (I2S DAC, ~10-12 IVPs) and battery ADC monitoring explicitly deferred — audio is ground-station only, battery ADC pending custom hardware. Removed "(was Stage X)" suffixes throughout. Updated three Stage 10 cross-references from "Stage 15 (Field Tuning)" to "Stage 17". No IVP numbers assigned yet per reorg-only scope.

---

### 2026-04-10-002 | Claude Code CLI | architecture, feature, bugfix

**Stage 14: Notification Engine complete (IVP-113 through IVP-118).** New AO_Notify intent layer sits between state producers and display consumers. Subsystems report typed intents; AO_Notify's priority resolver produces a single LED pattern posted to AO_LedEngine. Rewires 5 direct LED callers through the intent API. Also fixes a Stage 13 gap by moving Core 1 vitality check from AO_LedEngine to AO_HealthMonitor.

Architecture: `include/rocketchip/notify_intents.h` (per-category typed enums — Phase, Cal, Radio, Sensor, Fault — for compile-time category enforcement), `src/active_objects/ao_notify.{h,cpp}` (33Hz AO, priority 5, queue depth 16), `src/notify/notify_backend_led.cpp` (pure-function priority resolver, no vtable per JSF AV Rule 170), `src/notify/notify_backend_audio.cpp` (I2S DAC stub for future Fruit Jam audio stage), `docs/decisions/NOTIFY_CONTRACT.md` (decision doc + IVP-118 verification amendment). Priority reshuffle: all AOs shifted +1 to insert Notify at 5 (Radio=8, FD=7, HealthMon=6, Notify=5, Logger=4, Telem=3, LedEngine=2, RCOS=1). LedEngine simplified from 6-layer compositor to 3 layers (Fault/Notify/Idle) — pure display driver. SIG_BEACON_ACTIVE (slot 17) added for FD ABORT timeout beacon one-shot. SIG_HEALTH_STATUS now carries kHealthCore1Ok secondary bit.

Council-reviewed (NASA/JPL, ArduPilot, Professor, Rocketeer) with 4 amendments (A1-A4) + 2 plan-review additions (P1-P2). 669 host tests (30 new in `test/test_notify.cpp`). HW verified via GDB memory inspection: subscriber bitmaps match plan, Core 1 alive, resolver output visible in LedEngine layers. SPIN model unchanged (11/11 passing) — AO_Notify cannot cause flight-safety failures. User-approved one-time exception to append IVP-118 verification record to NOTIFY_CONTRACT.md.

**Latent bug found and fixed (LL Entry 35):** `AO_LedEngine_post_pattern()` had been using stack-local `QEvt` subclasses passed to `QACTIVE_POST` since Stage 7 (IVP-77). QP stores the pointer, NOT a copy — when the caller returned, the stack frame was reclaimed and subsequent calls overwrote the event memory before QV could dispatch it. Worked for months by luck; IVP-117's added stack usage in AO_Notify's tick handler finally exposed it as a `qf_dyn` id=750 assertion. Fixed by using static event storage in both `AO_LedEngine_post_pattern()` and new `AO_Notify_post_cal_intent()`. Audited existing posts — HealthMonitor, FD phase/pyro/beacon, and Radio RX all already use static. LedEngine was the odd one out. Documented in LESSONS_LEARNED Entry 35, feedback memory, code comments.

(include/rocketchip/{notify_intents,notify_backend,ao_signals}.h, src/active_objects/{ao_notify,ao_led_engine,ao_flight_director,ao_rcos,ao_health_monitor}.{h,cpp}, src/notify/*, src/safety/health_monitor.{h,cpp}, src/main.cpp, test/test_notify.cpp, test/CMakeLists.txt, CMakeLists.txt, docs/decisions/NOTIFY_CONTRACT.md, docs/AO_ARCHITECTURE.md, docs/IVP.md, .claude/LESSONS_LEARNED.md, AGENT_WHITEBOARD.md)

---

### 2026-04-10-001 | Claude Code CLI | feature, tooling

**Configuration Wizard prototype (Phase A).** Python-based terminal wizard that generates mission profile .cfg files from human-understandable questions. Council-reviewed (5 personas, unanimous GO).

Architecture: `scripts/config_wizard/` — core logic (taxonomy, derivation, validation, emitter) separated from UI (terminal, future GUI API). Wizard generates .cfg files that feed into existing `generate_profile.py` pipeline.

Features: 6 vehicle categories + custom (rocket/balloon/ground/aircraft/station/passive), sub-vehicle types, recovery types + methods, peripherals by bus (FeatherWing/I2C/GPIO), motor impulse-to-threshold derivation, certification checks (NAR/TRA L1-L3, HAM) with inline advisories, YOLO mode to bypass gates, back navigation, profile naming, auto code generation. 22 wizard tests + 648 host tests passing.

(scripts/config_wizard/*, AGENT_WHITEBOARD.md, CHANGELOG.md)

---

### 2026-04-09-003 | Claude Code CLI | bugfix, architecture

**Launch procedure audit: ABORT rework + baro critical fault.** Compared FD state machine against NASA/SpaceX/NAR launch procedures. Three safety fixes:

1. **ABORT is now a sink state** — no transition to LANDED. Pad abort (never launched): timeout → IDLE. In-flight abort: beacon activates after timeout, stays in ABORT. Landing moment determined in post-processing. Fixes bug where a pad-aborted rocket could enter LANDED (clearing fault latches, signaling "mission complete").

2. **Baro added to critical fault check** — `health_monitor_critical_fault()` now includes baro. A baro fault while ARMED triggers auto-DISARM + safe mode, same as IMU/ESKF. Without baro: no altitude-gated main deploy, ESKF vertical drift.

3. **SPIN P9: `p_no_landed_without_launch`** — new property verifying LANDED requires prior launch. `has_launched` variable tracks ARMED→BOOST. 7/7 safety properties pass. P7 liveness is a known pre-existing failure (MAIN_DESCENT needs timeout fallback — tracked on whiteboard).

648 host tests (was 647: +1 new in-flight abort test, 1 renamed). Device build clean.

(AGENT_WHITEBOARD.md, flight_director.cpp, health_monitor.cpp/h, test_flight_director.cpp, rocketchip_fd.pml, spin/README.md)

---

### 2026-04-09-002 | Claude Code CLI | feature, architecture, council

**Stage 13: Health Monitor complete (IVP-104–112).** New standalone AO_HealthMonitor with 2-bit per-subsystem encoding (absent/fault/degraded/healthy), sliding window degraded detection, fault latch. SIG_HEALTH_STATUS wired to LED/Logger/Telemetry (was orphaned). LED engine: 6 fault patterns with max() priority compositor. Telemetry health byte expanded to 4x2-bit, MAVLink SYS_STATUS fixed (was all-or-nothing). CLI restructured: debug sub-menu (`q`), preflight Go/No-Go (`p`). Vehicle output mode fix (kAnsi default was eating CLI input). Version bump v0.3.0, RC_OS v0.5.0.

Health-gated safety: auto-DISARM + safe mode on critical sensor fault while ARMED. Pre-launch fault latch in IDLE (persists until manual clear or reboot — loose wire safety). Council-reviewed (5 personas, unanimous). SPIN 11/11 properties (3 new health: ARM guard, fault latch, auto-DISARM). 647 host tests. HW verified: Qwiic unplug fault propagation confirmed (IMU FAULT → red blink → NO-GO → recovery on reconnect).

---

### 2026-04-09-001 | Claude Code CLI | documentation, refactor

**Whiteboard cleanup + stage restructuring.** Full audit of all tracking documents — no orphaned items. Deleted `ivp62-wip` branch (fully superseded, content ported to main). Cleared whiteboard resolved section (14 DONE items, all recorded in canonical docs). Fixed stale references across 8 files: AO_ARCHITECTURE (SIG_HEALTH_STATUS marked orphaned — zero subscribers despite doc claiming two), IVP.md (IVP-62/64/65 marked complete, Stage 13 inserted as Health Monitor), RADIO_TELEMETRY_STATUS (IVP-62 updated to complete), profiles/README (boot-load → future with lockouts). Restructured stages: 13=Health Monitor, 14=Pre-Flight Polish (14A/14B/14C), 15=Field Tuning. Far-future items moved to PROJECT_STATUS. Council review (5 personas, unanimous) with 24 amendments attached to plan file.

---

### 2026-04-08-001 | Claude Code CLI | bugfix, feature

**IVP-62d: QGC direct USB connection fixed.** Root cause: QGC sends binary MAVLink frames containing arbitrary byte values — bytes like 0x4C ('L') triggered CLI flash erase command, blocking for >1s and crashing the AO scheduler (queue overflow). Fix: CLI lockout on first 0xFD byte detection (ESC exits). Supporting fixes: CDC TX buffer 64→1024B, CRLF translation disabled for binary MAVLink, stdout timeout 500ms→10ms, direct tud_cdc_write bypassing stdio for MAVLink output, COMM_0 parser for USB input. QGC connects <10s, HSI responsive, recovers from occasional jitter. Remaining USB jitter deferred to Stage 13 polish.

---

### 2026-04-07-003 | Claude Code CLI | feature, council

**Stage 7 Take 2: Complete deferred radio/telemetry IVPs.** Council-reviewed (NASA, ArduPilot, Professor, Cubesat).

IVP-62a: GCS connection state machine — heartbeat-only until QGC detected, 5s timeout. IVP-62b: MAVLink RX parser ported from ivp62-wip branch (14 tests, 624 total). USB + LoRa input paths with separate parser channels (COMM_1/COMM_2). IVP-62c: Station→vehicle ARM command over LoRa — full path verified on hardware (FD IDLE→ARMED). MAVLink COMMAND_LONG parsed, dispatched via Flight Director. Fixed: vehicle RX after TX, CCSDS/MAVLink packet discrimination, RadioScheduler rx_continuous_ restore. IVP-62d: QGC direct USB deferred to Stage 13 — causes QGC freeze (USB CDC transport issue). IVP-64: RadioConfig SF/BW/CR wired to RFM95W driver. IVP-65: Native MAVLink TX via protocol selection. Stage 13 items added: half-duplex ACK, CCSDS SDLS auth, QGC USB CDC investigation.

---

### 2026-04-07-002 | Claude Code CLI | feature, council

**Stage 3D: 3-Axis Magnetometer Model (IVP-99 through IVP-102).** Council-reviewed (JPL, ArduPilot, Cubesat, Rocketeer, unanimous).

IVP-99: `update_mag_3axis()` — 6 sequential scalar updates (3 earth_mag + 3 body_mag_bias), same pattern as GPS position. Magnitude pre-gate ±25%, per-axis 5σ gate. `kRMag3dPerAxis = 0.36` µT² (AK09916 datasheet). IVP-100: Auto-enable when mag calibrated + WMM field available (GPS 3D fix or stored or profile default). IVP-101: WMM info in sensor status ('s' key) with source tracking (GPS/stored/default), 12 new host tests (610/610 total). IVP-102: WMM position persisted in cal storage v4, auto-upgrades from stored/default to GPS on first fix. IVP-103 (station GPS push) deferred — needs radio command path.

Boot banner slimmed from 45→12 lines. Advanced settings tracking doc created. WMM2025 tables generated from NOAA via BGS API.

---

### 2026-04-07-001 | Claude Code CLI | feature

**WMM2025 geomagnetic tables.** Replaced expired ArduPilot IGRF13 declination-only table with WMM2025 three-component tables (declination, inclination, total intensity) generated directly from NOAA coefficients via BGS API. Removes ArduPilot dependency. New `wmm_get_field()` / `wmm_get_earth_field_ned()` API. Default location (Dallas TX 33N 97W) in Mission Profile for Core tier no-GPS WMM lookup. Stage 3D (3-axis mag model) defined as dedicated IVP stage for future implementation.

---

### 2026-04-06-001 | Claude Code CLI | architecture, refactor, council

**Post-Stage 13 side items.** 6 commits. Council review (4 personas, unanimous with 5 amendments).

AO signal audit: wired SIG_PHASE_CHANGE (FD→Logger with exact timestamp), SIG_RADIO_STATUS (AO_Radio→LedEngine with change detection), added SIG_PYRO_FIRED (channel + source, primary + PIO paths). Removed 3 dead signals (LOG_FRAME, TELEM_FRAME, HEALTH_CHECK) with enum stability. AO_ARCHITECTURE.md updated to match reality.

Flash layout portability: new `flash_layout.h` derives all flash addresses from `PICO_FLASH_SIZE_BYTES`. Enables Tiny2350 (4MB) and other boards. Compile-time static_assert validates no overlap.

Flight log metadata header: 64-byte `FlightLogHeader` (magic 0x52434C47, version, firmware/board/profile info) written at start of each flight's flash region.

Passive ejection mission profile: `profiles/passive.cfg` for motor-eject rockets (HAS_PYRO=0, no PIO timers).

Code comments audit: 76 files, 213 lines net reduction. Stripped all IVP ticket numbers, migration history, future-tense promises. Preserved council refs, VALIDATE markers, numerical justifications. Created `docs/UNIQUE_COMMENT_ITEMS.md` tracking 6 buried action items.

Buried TODO resolution: PIO timer values wired from MissionProfile (was hardcoded 15/45s). Mahony ARM termination (force_end_startup on ARM). Radio ownership transferred from main.cpp externs to AO_Radio (owns init lifecycle). VALIDATE values audited against ArduPilot/PX4 norms. Removed unused config.h compatibility aliases.

---

### 2026-04-04-001 | Claude Code CLI | architecture, refactor, council

**Stage 13: AO Architecture Completion.** 10 commits across 8 phases. Council review (5 personas, unanimous with amendments).

main.cpp reduced from 3,384 to 706 lines (79%). Every subsystem extracted into its own module or AO with explicit interfaces. New files: sensor_seqlock.h (cross-core data types), led_patterns.h (single source of truth for LED constants), sensor_core1.cpp (Core 1 sensor loop), eskf_runner.cpp (ESKF fusion module, idle bridge), health_monitor.cpp (centralized health state), cli_commands.cpp (display/command handlers), cal_hooks.cpp (calibration cross-core protocol).

Existing AOs completed: AO_FlightDirector now owns FlightDirector HSM + guard eval + Go/No-Go. AO_Logger now owns ring buffer + flight table + FusedState builder + event logging. AO_LedEngine now has 6-layer priority compositor (fault > flight > cal > radio > sensor > idle) with Core 1 vitality check.

ESKF stays in idle bridge (Council A1: 100Hz QF tick can't match 200Hz rate). HealthMonitor is a module called from AO_FD at 10Hz (Council A2: doesn't justify own AO). SIG_HEALTH_STATUS (27) added, SIG_AO_MAX bumped to 28. New doc: docs/AO_ARCHITECTURE.md.

598/598 host tests pass on every phase. All phases HW verified on Feather RP2350.

---

### 2026-04-03-001 | Claude Code CLI | feature, architecture, council

**Stage 12B: ANSI Terminal Dashboard + AO_RCOS extraction.** Two commits. Two council reviews (both unanimous).

**ANSI Dashboard:** Firmware-rendered live dashboard over USB serial. Color-coded telemetry (flight state, RSSI bar, signal age). Cursor-home + clear-to-EOL rendering (no flicker). ANSI-by-default, `m` cycles modes (ANSI→CSV→MAVLink), `x` enters CLI menu. No companion app needed. HW verified on Fruit Jam + Feather TX.

**AO_RCOS:** Extracted CLI output mode management and ANSI rendering from qv_idle_bridge into dedicated Active Object (20Hz). StationOutputMode enum in shared header (council A3). Cal bridge pattern for blocking wizards (council A2). rc_os_update() stays in idle bridge. 598/598 SPIN tests pass.

**WiFi dashboard deferred:** NINA SPI TCP server write bug unresolved. Code preserved on `claude/stage-12b-wifi-dashboard` branch + stash.

---

### 2026-03-31-001 | Claude Code CLI | feature, architecture, council

**Stage 12A COMPLETE: Radio Module + Fruit Jam GCS (IVP-92–98).** Seven IVPs. Three council reviews (radio universality, RadioScheduler, final plan — all unanimous). HW verified on 3 boards: Vehicle TX (Feather RP2350), Station RX (Fruit Jam), Relay (Feather RP2350).

**IVP-92:** Non-blocking TX split (`send_start`/`send_poll`). Runtime config setters for SF, BW, CR.
**IVP-93:** AO_Radio extraction with RadioScheduler half-duplex state machine (100Hz, protocol-agnostic).
**IVP-94:** AO_Telemetry refactored to protocol-only. SIG_RADIO_TX/RX event flow. ~120 lines removed from main.cpp.
**IVP-95:** Three-Job DeviceRole system (Vehicle/Station/Relay) with conditional AO startup.
**IVP-96:** RadioConfig in Mission Profile `.cfg` with auto-derived RF parameters.
**IVP-97:** Fruit Jam station: 5-NeoPixel RSSI bar, station CLI (GPS status, distance-to-rocket stubs).
**IVP-98:** Link-layer relay in AO_Radio (CCSDS CRC validation, seq dedup, re-TX).

**Bugs found during HW verification:**
- RP2350B I2C pad isolation: GPIO pads start isolated on RP2350B, breaks bus recovery (LL Entry 35 candidate)
- QP/C static event pattern: stack-allocated events crash on refCtr assert
- PIO contention: AO_LedEngine + RSSI bar on same PIO SM → queue overflow. Fix: disable LedEngine in station mode.

**HW test results:** TX→RX 296+ pkts, 0 CRC errors, continuous seq. Relay confirmed via duplicate seq at different RSSI. RSSI bar gradient working. GPS detected on FJ after I2C fix.

---

### 2026-03-29-003 | Claude Code CLI | feature

**Event logging framework.** PCM event frame (type=3, 15 bytes) for discrete flight events: pyro fire, abort, confidence gate transitions. Logged to ring buffer with MET timestamp alongside periodic telemetry frames. Designed for future Mission Profile toggles. Persistent pyro-fired flags added to FlightState.

---

### 2026-03-29-002 | Claude Code CLI | feature, architecture, council

**Stage 11 COMPLETE: PIO Safety Architecture (IVP-87–91).** Five IVPs. Two council reviews (watchdog architecture + plan review, 5 panelists each). SPIN model updated — 6/6 safety properties pass. 598/598 host tests, 65s HW soak clean.

**IVP-87 (Baseline Benchmark):** Recorded pre-change performance numbers for regression comparison.

**IVP-88 (PIO Heartbeat Watchdog):** PIO2 heartbeat watchdog replaces SDK software watchdog concept. IRQ-based, no GPIO pin interference with I2C/SPI. LL Entry 33: PIO GPIO/I2C interference discovery.

**IVP-89 (PIO Backup Deployment Timers):** Independent PIO-driven drogue + main backup timers. HW verified via GPIO read. LL Entry 34: baro fan turbulence during bench testing.

**IVP-90 (SDK Watchdog Removal):** SDK `watchdog_enable()` removed — PIO is sole health monitor. No auto MCU reset; phase-dependent degrade instead.

**IVP-91 (Post-Change Benchmark):** No regression: +2.5% predict time, +1.8KB text. Within acceptable margins.

Persistent pyro-fired flags added to FlightState. Watchdog safety architecture decision doc + research sources archived (`docs/decisions/WATCHDOG_SAFETY_ARCHITECTURE.md`).

**IVP renumbering:** Stage 12: Ground Station (IVP-92–97). Stage 13: Pre-Flight Polish (IVP-98–102). Stage 14: Field Tuning & Validation (IVP-103+).

(`src/pio/pio_heartbeat.cpp`, `src/pio/pio_backup_timer.cpp`, `src/main.cpp`, `src/flight_director/flight_state.h`, `tools/spin/rocketchip_ao.pml`, `CMakeLists.txt`)

### 2026-03-29-001 | Claude Code CLI | feature, architecture, council

**Stage 10 COMPLETE: Adaptive Estimation & Safety (IVP-83–85).** Three IVPs, IVP-86 retired. Council-reviewed (unanimous, 7 amendments incorporated). 598/598 host tests, 65s HW soak clean (85K IMU reads, 0 errors, conf=Y).

**IVP-83 (Phase-Scheduled Q/R + Innovation Monitor):** Per-phase Q scaling and R values sourced from Mission Profile `.cfg`. Additive Q delta applied post-codegen FPFT. Per-channel sliding-window NIS tracker with 10x cap. NIS push-after-gate fix: gated (rejected) readings were corrupting innovation monitor windows. 28 new tests.

**IVP-84 (Confidence Gate):** Binary ESKF trust flag with hysteresis (500ms loss, 2s recovery debounce). Platform safety, not profile-configurable. Wired into `eskf_tick()`, `FusedState`, CLI display. 14 new tests.

**IVP-85 (Confidence-Gated Actions):** Wired into SafetyLockout/guard combinator. Pyro locked when ESKF uncertain, no fallback. SPIN model updated with confidence property. Abort action no longer fires pyro by default. 4 new tests.

**IVP-86 (retired):** Originally planned cycle-performance benchmark; deferred to post-stage hardware validation.

**IVP.md restructured:** Stage 12 renamed "Pre-Flight Polish", Stage 13 added (Field Tuning placeholder).

(`src/fusion/eskf.h`, `src/fusion/eskf.cpp`, `src/fusion/innovation_monitor.h`, `src/fusion/innovation_monitor.cpp`, `src/flight_director/safety_lockout.cpp`, `src/flight_director/guard_functions.cpp`, `tools/spin/rocketchip_ao.pml`, `profiles/rocket.cfg`, `profiles/hab.cfg`)

### 2026-03-27-002 | Claude Code CLI | architecture, council

**IVP-82a/82b: SPIN Formal Verification (Stage 9 completion).** SPIN 6.5.2 + MinGW GCC + Cygwin toolchain installed. Two Promela models: FD-only (73 states) and full AO topology (107,818 states). 8 properties verified exhaustively in 37ms each — 5 safety (pyro-never-in-IDLE, drogue-before-main, requires-ARMED, drogue-once, main-once) + 3 mission-critical (event delivery to Logger/Telem/LED). Council-reviewed (4 panelists, 5 amendments). Key optimization: `atomic{}` blocks simulate QV run-to-completion, channel depth [1] (safety depends on ordering not buffering). Verification overview doc added (`docs/VERIFICATION_OVERVIEW.md`) unifying all 5 verification layers.

(`tools/spin/rocketchip_fd.pml`, `tools/spin/rocketchip_ao.pml`, `tools/spin/README.md`, `docs/VERIFICATION_OVERVIEW.md`)

### 2026-03-27-001 | Claude Code CLI | architecture, council

**IVP-76 through IVP-81: Active Object Architecture (Stage 9).** Full superloop-to-AO migration. Two council reviews (8 + 5 amendments). 6 Active Objects running under QV cooperative scheduler.

**IVP-76 (BSP):** QF_run() replaces while(true). 100Hz tick timer, system-wide RcSignal catalog (`ao_signals.h`), pub-sub infrastructure. QS tracing deferred (source not vendored, no spare UART). Git tag `pre-qv-main`.

**IVP-77 (LED Engine):** AO_LedEngine owns NeoPixel pattern state. ws2812 calls from Core 0 (33Hz tick). Static event race fixed (Council C5 — stack-local instead of static).

**IVP-78 (Flight Director):** 100Hz time event wraps flight_director_tick(). Queue depth 32 to handle LoRa TX blocking (LL Entry 32).

**IVP-79 (Logger):** 50Hz time event wraps logging_tick(). Queue depth 32.

**IVP-80 (Telemetry):** 10Hz time event wraps telemetry_radio_tick() + mavlink_direct_tick(). Queue depth 32. Blocking rfm95w_send() (50-150ms LoRa airtime) inside AO handler is the root cause of queue overflow — pragmatic fix via depth 32, architectural fix (non-blocking driver) deferred.

**IVP-81 (Superloop removal):** QV_onIdle reduced to watchdog (permanent, Council A2), ESKF (seqlock bridge), CLI (polled). sleep_ms(1) for USB CDC yield.

**Blocking driver lesson (LL Entry 32):** Cooperative schedulers amplify blocking. `rfm95w_send()` was invisible in the superloop but caused `qf_actq id=130` assertion under QV — timer ISR posts events to all AO queues during the block. Incremental AO addition isolated AO_FlightDirector (100Hz = fastest queue fill) as the first victim. Queue depth 32 handles 320ms worst-case. Non-blocking driver (`send_start`/`send_poll`) is the proper long-term fix.

*Rationale: QV cooperative scheduler over FreeRTOS/ChibiOS — the dual-core architecture isolates deterministic sensor sampling on Core 1, diminishing the primary RTOS advantage. QV gives decoupled modules, typed events, and priority scheduling without per-task stacks, mutexes, or context-switch overhead.*

552/552 host tests. 60s HW soak: 288K IMU reads, 2 errors (boot-only), ESKF healthy, AO_Counter avg=100ms.

*Rationale: QV cooperative scheduler over FreeRTOS/ChibiOS — the dual-core architecture already isolates deterministic sensor sampling on Core 1, diminishing the primary advantage of a preemptive RTOS. QV provides decoupled modules, typed events, and priority-based scheduling without per-task stacks, mutexes, or context-switch overhead. Worst-case AO dispatch latency (~850µs from eskf_tick) is well within the 10ms budget at 100Hz.*

(`lib/qep/qp_config.h`, `lib/qep/bsp_qv.c`, `src/main.cpp`, `src/flight_director/flight_director.h`, `CMakeLists.txt`)
(**New:** `include/rocketchip/ao_signals.h`, `src/active_objects/ao_blinker.cpp`, `src/active_objects/ao_counter.cpp`)

### 2026-03-26-005 | Claude Code CLI | feature

**State-aware ZUPT.** When Flight Director phase is IDLE or ARMED, skip IMU stationarity check and use tighter R (0.01 m/s^2 vs 0.25 m/s^2) for stronger velocity constraint on pad. ArduPilot EKF3 `onGround` / PX4 ECL `vehicle_at_rest` pattern. HW verified: zNIS=0.01, vh=0.01 m/s. Resolves whiteboard deferred item.

(`src/fusion/eskf.h`, `src/fusion/eskf.cpp`, `src/main.cpp`)

### 2026-03-26-004 | Claude Code CLI | documentation

**IVP restructure: Stage 9 expanded 5→7 IVPs.** Split Logger+Telemetry IVP into separate AOs (IVP-79, IVP-80). Added SPIN formal verification (IVP-82). Clarified stack-allocated events in IVP-76, noted LED consolidation in IVP-77, noted audit printf deferral in IVP-78. Stages 10-12 renumbered (IVP-83–97). Total: 97 IVPs across 12 stages.

(`docs/IVP.md`)

### 2026-03-26-003 | Claude Code CLI | audit, standards, refactor

**Full standards audit + remediation.** Ran tiered audit across 40 production files (2,315 clang-tidy warnings, 26 lizard, 25 Tier 3, 8 Tier 4). Audit report in `standards/STANDARDS_AUDIT_2026-03-26.md`.

**Remediated:**
- 1 potential bug: `ring_buffer.cpp` uninitialized `pos` (false positive, added `= 0`)
- 8 driver files: Prior Art comment blocks added (3 retagged, 5 new)
- ~60 magic numbers: replaced with named `constexpr` constants across 13 files (telemetry, data_convert, rfm95w, spi_bus, eskf, ud_factor, psram_init, log_decimator, main)
- `psram_configure_qmi` decomposed (timing calc extracted to stay under 60-line threshold)
- Audit script: excluded auto-generated + benchmark files from PRODUCTION_FILES

**Deferred:** Flight Director diagnostic `printf` calls (10 hits) — deferred to Stage 9 IVP-78 (AO migration, diagnostic output moves to event-based logging).

**Accepted:** 2 CCN>20 CLI menu handlers (Ground code), QEP C-style casts (framework), ~40 snake_case parameter names (matches project convention).

**Whiteboard:** Added deferred note — flash layout should derive from `PICO_FLASH_SIZE_BYTES` instead of hardcoding 8MB. Currently non-portable to boards with different flash sizes (e.g., Pimoroni Tiny2350, 4MB). Side project, not blocking.

Session wrap-up: updated `docs/PROJECT_STATUS.md` (Stage 8 complete → Stage 9 next), `docs/SCAFFOLDING.md` (added profiles/, flight_director/, logging/, telemetry/, watchdog/, lib/qep/, new scripts), `.claude/SESSION_CHECKLIST.md` (fixed PROJECT_STATUS.md path reference).

(`standards/STANDARDS_AUDIT_2026-03-26.md`, `docs/PROJECT_STATUS.md`, `docs/SCAFFOLDING.md`, `.claude/SESSION_CHECKLIST.md`, 20 source files)

### 2026-03-26-002B | Claude Code CLI | tooling, audit, standards

**Implemented and validated Grok's tiered audit blocks.** Adapted for Windows/Git Bash (lizard via `python -m` fallback, portable grep patterns, `set -e` safe exit handling). Dropped magic-numbers grep (869 false positives — unusable at grep level, clang-tidy's relaxed check + manual review is more effective). Changed Tier 3/4 from hard `exit 1` to warnings on first run to allow full triage. Tier 4 scoped to `src/drivers/` only (flight_director files have design-level prior art in docs, not per-file).

First-run results: Tier 2 found 2 CCN>20 (CLI menu switches, Ground code — accepted). Tier 3 found 25 stdio hits (14 in Ground-only `i2c_bus_scan`, 11 in flight_director diagnostic logging on Core 0 CLI thread — no flight-safety risk). Tier 4 found 8 driver files missing Prior Art blocks.

(`scripts/run_clang_tidy.sh`)

### 2026-03-26-002A | Grok | tooling, audit, standards

**Added tiered audit blocks to scripts/run_clang_tidy.sh for full standards coverage.**

**Changes:**
- Dynamic PRODUCTION_FILES sync from CMakeLists.txt (portable, fixes stale list missing Stage 8 files)
- Tier 2: lizard cyclomatic + refined strict magic numbers (with line numbers + stronger exclusions)
- Tier 3: RP2350 platform guards (scoped to Flight-Critical files only, excludes debug macros)
- Tier 4: Prior Art checks (scoped to drivers/hardware/flight_director only)
- Automatic exit 1 on violations
Keeps fast abbreviated clang-tidy unchanged. Closes gaps identified in systematic review of CODING_STANDARDS.md vs current script/.clang-tidy.

(`scripts/run_clang_tidy.sh`)

### 2026-03-26-001 | Claude Code CLI | feature, architecture, council, tooling

**Stage 8 Flight Director — IVP-72 through IVP-75 complete. Stage 8 done.**

**IVP-72:** Action Executor. `ActionType` enum (SET_LED, MARK_EVENT, REPORT_STATE, FIRE_PYRO, SET_BEACON), constexpr action arrays per phase in `flight_actions.h`, callbacks wired into QEP state handlers. NeoPixel colors per flight state (ARMED=amber, BOOST=red, COAST=yellow, DESCENT=red blink, LANDED=green blink, ABORT=red fast blink). Pyro intent logging via callbacks. Council Amendment #2 (pyro negative tests) enforced. NeoPixel override switch extracted to `neo_apply_override()`. 32 tests, 529/529 total.

**IVP-73:** Bench Flight Simulation. Python pyserial script (`scripts/bench_flight_sim.py`) automates Flight Director HW gate — 9 test cases (happy path, 4 abort paths, 3 rejection cases). Data-driven `TestCase` architecture (council A3), prompt-sync serial (A1), rejection tests first (A2). 9/9 PASS in 10s. Council-reviewed.

**IVP-74:** Mission Profile Configuration. User-editable `.cfg` files (`profiles/rocket.cfg`, `profiles/hab.cfg`) with ArduPilot `.param`-inspired `NAME VALUE` format. Python generator (`scripts/generate_profile.py`) produces C++ header with `static_assert` validation (council A3) and source hash traceability (A2). Field guide README with safe ranges (A4). Profile name shown in boot banner, system status, and FD status. HAB profile swap HW-verified (lower launch threshold caused behavioral change). Compile-time only; boot-load deferred to Stage 11. Council-reviewed (unanimous). 23 tests, 552/552 total.

**IVP-75:** Active Object Migration Planning. QP/C 8.1.3 QF framework (10 source files) and QV cooperative scheduler vendored. BSP shim with `QF_onStartup()`, `QV_onIdle()` (WFI), port header with QEQueue/QMPool/QV includes. Compile gate passes — QF+QV links alongside existing superloop with no runtime behavior change. Preliminary migration doc in `docs/flight_director/ACTIVE_OBJECT_MIGRATION.md` (needs full plan + council review before Stage 9 implementation).

(`src/flight_director/`, `src/main.cpp`, `src/cli/rc_os.cpp`, `lib/qep/`, `profiles/`, `scripts/`, `test/`, `docs/flight_director/`)

---

### 2026-03-25-002 | Claude Code CLI | feature, architecture, council

**Stage 8 Flight Director — IVP-69 through IVP-71 complete.** Continued same session after IVP-66–68 changelog entry.

**IVP-69:** Go/No-Go pre-arm checks + command handler. NASA-style readiness poll: Tier 1 (6 platform stations: IMU, Baro, ESKF, Flash, Safety, Watchdog — all must GO) + Tier 2 (4 profile stations: GPS, Mag, Radio, Battery — warn only). CLI ARM/DISARM/ABORT/RESET routed through command handler with phase validation. Sensor event keys bypass for bench testing. 32 tests, 444/444 total.

**IVP-70:** Guard functions + evaluator. 6 guard functions (launch accel, burnout accel, apogee velocity, baro peak, main deploy altitude, stationary) with sustain timers and phase-validity bitmasks. All thresholds from MissionProfile. Wired into main.cpp at 100Hz via seqlock snapshot. Hand shake triggered automatic ARMED→BOOST. 31 tests, 475/475 total.

**IVP-71:** Guard combinators + three-layer safety architecture. Council-reviewed (NASA/JPL, ArduPilot, Professor, Rocketeer — unanimous APPROVE, 6 amendments). Industry research: timer backups standard at every level (NASA primary for sounding rockets), velocity lockout universal. Three layers: (1) lockout gates (velocity 80 m/s + 3s min-time), (2) sensor combinators (AND for apogee: velocity + baro must agree), (3) timer backup (gated by lockouts). ESKF-unhealthy bypasses velocity lockout for timer only (Council A2). Managed/unmanaged guard split via constexpr array (Council A4). Per-profile `emergency_deploy_anytime` for HAB. MissionProfile expanded with lockout, backup, combinator fields. 22 combinator tests, 497/497 total.

(`src/flight_director/`, `src/main.cpp`, `src/cli/rc_os.{cpp,h}`, `include/rocketchip/config.h`, `test/test_go_nogo.cpp`, `test/test_guards.cpp`, `test/test_guard_combinator.cpp`)

---

### 2026-03-25-001 | Claude Code CLI | feature, architecture

**Stage 8 Flight Director — IVP-66 through IVP-68 complete.** ~10 day gap between sessions (machine powered off and relocated). HW gate re-verified from clean boot — all sensors init OK, 60s soak 0 new errors, ESKF stable. No regressions from the gap.

**IVP-66:** Watchdog recovery policy — `WatchdogRecoveryState` struct with boot state preservation, `TickFnId` enum for identifying which tick function was running when watchdog fired, flight phase saved across reboot.

**IVP-67:** QP/C 8.1.3 QEP vendored (646 lines portable C99 dispatch engine). STARS/QM/SPIN toolchain evaluation documented in `docs/flight_director/TOOLCHAIN_EVALUATION.md` — STARS not adopted (commercial), QM deferred (statechart too small), SPIN deferred to Stage 9 AOs. 12 QEP smoke tests passing.

**IVP-68:** Flight Director QHsm skeleton — 9 state handlers (idle, armed, boost, coast, descent superstate, drogue_descent, main_descent, landed, abort). Source-specific ABORT per Council Amendment #1 (armed=no pyro, boost/coast=drogue intent, descent=ignored). MissionProfile struct with `kDefaultRocketProfile` (guard thresholds, sustain times, timeouts). `mission.h` renamed to `job.h` ("job"=device role, "MissionProfile"=flight config). CLI `RC_OS_MENU_FLIGHT` sub-menu with bench signal injection, grouped main menu layout, `[main]`/`[cal]`/`[flight]` context prompts. Fixed prompt spam on unrecognized keys and double prompt at boot. 29 FD tests, 412/412 total. HW gate: all 9 phases reachable, all 4 abort paths correct, coast timeout verified.

(`src/flight_director/`, `include/rocketchip/job*.h`, `src/main.cpp`, `src/cli/rc_os.{cpp,h}`, `src/watchdog/watchdog_recovery.h`, `test/test_flight_director.cpp`, `docs/ROCKETCHIP_OS.md`)

---

### 2026-03-14-002 | Claude Code CLI | hardware, council

**DPS310 baro: independent P/T config, 8x OS, R correction.** Council-reviewed DPS310 measurement rate and oversampling selection. Key findings: (1) previous 16x OS / 32 SPS config ran at 88% duty cycle in CONT_BOTH mode (P+T interleaved, sharing measurement budget); (2) 64 SPS attempts failed HW verification at both 8x and 4x OS — DPS310 enforces MaxRate limits stricter than measurement-time math suggests; (3) P and T channels can be configured independently. New config: pressure at 8x OS / 32 SPS, temperature at 1x OS / 2 Hz (compensation only). Duty cycle drops from 88% to 48% with better noise match. Updated `kSigmaBaro` from 0.029m (matched 16x OS) to 0.033m (matches actual 8x OS) in ESKF and BaroKF — the R value was previously too tight for the hardware config. HW verified: 60s soak, 2029 baro reads, 0 errors, bNIS=0.01. (`src/drivers/baro_dps310.{h,cpp}`, `src/fusion/eskf.{h,cpp}`, `src/fusion/baro_kf.h`, `src/main.cpp`, `docs/hardware/HARDWARE.md`)

---

### 2026-03-14-001 | Claude Code CLI | architecture, council, documentation

**Stage 8 restructure: UML statecharts + QEP adoption.** Integrated three council-reviewed documents (formalism research, council decisions, QP/C application guide) into `docs/flight_director/` and `docs/decisions/flight_director/`. Deprecated old `FLIGHT_DIRECTOR_DESIGN.md` and `RESEARCH.md`. Restructured IVP Stage 8 from 5 to 10 steps (IVP-66–75) reflecting council decisions: UML statecharts as formalism, QEP dispatch engine, STARS toolchain gate, apogee reclassified as event, transition-gated pyro safety architecture. Added new Stage 9: Active Object Architecture (IVP-76–80) for QF+QV migration. Renumbered Stages 10–12, total 95 IVP steps across 12 stages. (`docs/IVP.md`, `docs/flight_director/`, `docs/decisions/flight_director/`)

---

### 2026-03-08-002 | Claude Code CLI | documentation, tooling

**CLA + RBM pre-Stage 8 audit.** Computational Load Analysis from 270s HW soak (zero firmware changes — serial CLI data collection only). Runtime Behavior Map covering boot sequence, Core 0/Core 1 dispatch, CLI state machine, NeoPixel priority, error recovery (6 gaps documented). Graphviz `.dot` diagrams for boot sequence, cross-core timeline, error recovery. Repeatable via `scripts/cla_collect.py`, `scripts/rbm_check.py`, `scripts/render_dot.sh`. Cross-reference added to `HARDWARE_BUDGETS.md`. (`docs/audits/cla_rbm/`, `scripts/`)

---

### 2026-03-08-001 | Claude Code CLI | feature, bugfix

**Stage 7 Radio & Telemetry — IVP-58 through IVP-61 complete. IVP-62 deferred.** End-to-end telemetry pipeline verified: CCSDS encoding over LoRa (vehicle→station), MAVLink re-encoding on station, QGC High Latency mode connected via Fruit Jam bridge with live attitude data.

**IVP-58:** CCSDS space packet encoder — 42-byte PCM nav frames with CRC-16, 6-byte CCSDS primary header, MET timestamps.
**IVP-59:** Telemetry service — replaces test TX with production pipeline. SX1276 DIO0 mapping fix.
**IVP-60:** Station RX mode + CCSDS decode + Mission Profile infrastructure. Compile-time vehicle/station behavioral selection via `mission.h`. 5-min soak: 607 pkts, 0 CRC err, 98.7% delivery.
**IVP-61:** MAVLink v2 encoder using official c_library_v2 (header-only submodule). Station re-encodes CCSDS→MAVLink for QGC. Vehicle direct USB output at 10Hz. QGC High Latency mode verified on both paths.

**IVP-62 (bidirectional MAVLink commands) deferred.** Full implementation complete (mavlink_rx handler, 14 host tests, command dispatch for params/capabilities/arm/mode/missions) but QGC direct USB connection unstable — "Communication Lost" after initial connect due to USB CDC buffer timing. Work preserved on `ivp62-wip` branch. Main reverted to IVP-61 where FJ bridge path is confirmed working.

**QGC connection status:** Works via Fruit Jam LoRa bridge (COM9, High Latency mode). Direct USB (COM7) deferred — protocol is correct (verified via pymavlink/Python) but QGC's 3.5s heartbeat timeout + USB CDC buffering behavior causes disconnects.

### 2026-03-07-001 | Claude Code CLI | feature, architecture

**Stage J: Fruit Jam HAL — Board abstraction + parity verification complete.** Compile-time board support package (BSP) via `board::` namespace constants so the same `main.cpp` builds and runs on both Feather RP2350 HSTX (#6130) and Fruit Jam (#6200). Zero runtime overhead — all `constexpr` pin constants resolved at compile time.

**New files:** `board.h` (selector), `board_feather_rp2350.h`, `board_fruit_jam.h`, `BOARD_COMPARISON.md`. **Modified:** i2c_bus, spi_bus, ws2812, rfm95w, gps_uart, config.h, main.cpp, rc_os.cpp, both CMakeLists.txt. **Deleted:** `gs_spi.cpp`, `radio_rx.cpp` (replaced by board-abstracted main.cpp).

**Council amendments implemented:** [M1] GPIO 5 conflict guard, [M2] LED active-low inversion with `board_led_set()` helper, [M3] UART GPS unavailable guard, [M4] PSRAM CS from SDK, [S1] hardcoded I2C leak grep, [S3] multi-LED NeoPixel clone, [S4] ESKF clean no-op verification, [N1] SPI1 bus sharing comment, [N2] full codebase inclusion rationale.

**J.2 HW verified on Fruit Jam:** All 15 parity gate items passed. PSRAM 8MB on GPIO 47, radio RFM95W on SPI1, GPS PA1010D on I2C0, 5 NeoPixels on GPIO 32 (gpiobase=16), absent IMU/baro graceful no-op, 60s soak with 0 errors and no watchdog resets.

**Bug fixed:** Core 1 was reading absent IMU/baro ~1000/s, generating unnecessary I2C errors. Added `g_imuInitialized`/`g_baroInitialized` guards in Core 1 sensor loop.

### 2026-03-04-003 | Claude Code CLI | feature, bugfix

**Stage 6 Data Logging — IVP-54a/54b completed. Full USB download pipeline HW verified.** IVP-54a: CLI flight list (`f`) and binary download (`d`) commands. Sector-aware frame streaming skips flash padding. CRLF translation disabled during binary output (`stdio_set_translate_crlf`). End-to-end CRC-32 in RCEND footer. IVP-54b: Python decoder script (`scripts/decode_flight_log.py`) — serial download, offline decode, CSV output, auto-detect port, interactive mode (no-args lists flights and prompts for selection).

**HW verified:** 3733 frames downloaded, 0 corrupt, transport CRC-32 OK. CSV output with plausible sensor values, monotonic timestamps.

**Bugs fixed:** (1) Python struct format `TELEM_FORMAT` was 44 bytes — missing `temperature_c` field, should be 45. (2) Pico SDK `stdio_usb` CRLF translation converted `0x0A` bytes in binary data to `0x0D 0x0A`, corrupting frame alignment. Fixed with `stdio_set_translate_crlf(&stdio_usb, false)` around binary write. (3) Download streamed contiguous bytes including sector padding (26B of 0xFF per 4KB sector). Fixed with sector-aware frame-by-frame streaming. (4) Unicode box-drawing chars in flight table header caused Windows console codec errors. Fixed with ASCII dashes.

**UX improvements:** Erase moved from `E` to `x` (too close to `e`/ESKF live). Erase confirmation changed from single `Y` keypress to typing "yes" + Enter. Flush accepts both `l` and `L`. Flight list/download accept both cases. Interactive mode: running script with no args lists flights and prompts for selection.

### 2026-03-04-002 | Claude Code CLI | feature, bugfix

**Stage 6 Data Logging — IVP-52c, IVP-53b completed. Full flash storage pipeline HW verified.** IVP-52c (decimation + main loop integration + SRAM fallback) and IVP-53b (flash flush + watchdog + CLI capacity) verified on hardware. Flush: 5,807 frames → 78 sectors → Flight #1, second flush 5,018 frames → Flight #2. Power cycle persistence: 2 flights survived reboot (7.7% used). Erase: 'E'+'Y' confirmation clears table and used sectors. Boot banner shows flash capacity. CLI help shows L/E commands.

**Bugs fixed:** (1) `case 'E':` in `rc_os.cpp` shadowed the erase command — both `'e'` and `'E'` mapped to ESKF live mode, preventing the unhandled-key callback from reaching `cmd_erase_all_flights()` in main.cpp. Fixed: only `'e'` triggers ESKF live. (2) `flight_log_erase_all()` erased all 1912 flight log sectors regardless of usage — at ~30ms/sector = 57 seconds, causing USB timeout and device crash. Fixed: only erases used sectors (based on `next_free_sector`). (3) GDB `monitor reset run` doesn't reliably resume both cores on dual-core RP2350 — Core 1 appears stuck at bootrom. Fixed: use `monitor resume` instead. Updated DEBUG_PROBE_NOTES.md.

**Observed:** DPS310 baro read count freezes after ~800 reads (error count keeps climbing). Baro data valid for initial reads but sensor appears to stop continuous measurement mode. Pre-existing issue, not caused by IVP-53b changes. Needs recovery mechanism (future IVP).

(`src/main.cpp`, `src/cli/rc_os.cpp`, `src/logging/flash_flush.h`, `src/logging/flash_flush.cpp`, `.claude/DEBUG_PROBE_NOTES.md`, `AGENT_WHITEBOARD.md`)

---

### 2026-03-04-001 | Claude Code CLI | feature

**Stage 6 Data Logging — IVP-52b, IVP-53a, IVP-52a completed.** Sub-steps executed out of plan order (plan: 52b→53a→52a→52c→53b→54a→54b) to front-load host-testable work before hardware-dependent steps. IVP-52b (ring buffer, host-testable) and IVP-53a (CRC-32 + flight table, host-testable) completed first with 299/299 host tests. IVP-52a (PSRAM init + self-test) completed next — 8MB APS6404L detected via QMI direct-mode SPI ID read, QPI mode configured, self-test passes at 3 addresses, council req. #2 flash-safe hard gate passes (write→flash_safe_execute erase→readback verify). PSRAM init runs before Core 1 launch (QMI manipulation unsafe while Core 1 executes from flash); flash-safe test deferred to after Core 1 calls multicore_lockout_victim_init(). Flight table sector count corrected from plan estimate 1912 to actual 1916 ((0x7FC000-0x080000)/4096). 60s HW soak: 88,896 IMU reads, 2 startup errors, ESKF stable.

(`src/logging/psram_init.h`, `src/logging/psram_init.cpp`, `src/logging/ring_buffer.h`, `src/logging/ring_buffer.cpp`, `src/logging/flight_table.h`, `src/logging/flight_table.cpp`, `src/logging/crc32.h`, `test/test_ring_buffer.cpp`, `test/test_flight_table.cpp`, `CMakeLists.txt`, `test/CMakeLists.txt`, `src/main.cpp`)

---

### 2026-03-03-001 | Claude Code CLI | architecture, documentation

**IVP resequencing — Data Logging pulled forward to Stage 6.** Two council reviews (telemetry protocol selection + data logging architecture) established that the telemetry encoder depends on data structures defined by the logging architecture. Data Logging moved from Stage 9 to Stage 6 (IVP-49–56), Radio & Telemetry becomes Stage 7 (IVP-57–65), new Ground Station Stage 10 (IVP-75–80) added. All IVP numbers from 49 onward renumbered. Total IVP count: 72→85. Logging format changed from MAVLink .bin to PCM fixed frames. Telemetry encoder now uses CCSDS/MAVLink strategy pattern selected by Mission Profile. Three-struct data model defined: FusedState (float32 ESKF-internal), TelemetryState (fixed-point wire-ready), SensorSnapshot (raw pre-calibration).

(`docs/IVP.md`, `docs/SAD.md`, `docs/PROJECT_STATUS.md`, `AGENT_WHITEBOARD.md`, `src/main.cpp`, `src/fusion/eskf.h`, `src/fusion/mahony_ahrs.h`, `test/test_eskf_mag_update.cpp`, `CMakeLists.txt`, `docs/flight_director/FLIGHT_DIRECTOR_DESIGN.md`, `docs/mission_profiles/MISSION_PROFILES.md`, `docs/DYNAMIC_VALIDATION.md`, `docs/ESKF_TESTING_GUIDE.md`, `docs/PHASE5_ESKF_PLAN.md`, `docs/hardware/GEMINI_CARRIER_BOARD.md`)

---

### 2026-02-25-001 | Claude Code CLI | feature, hardware

**Ground station LoRa RX bridge — Fruit Jam + RFM95W breakout, end-to-end link verified.** Standalone Pico SDK build for Adafruit Fruit Jam (#6200, RP2350B) with RFM95W breakout (#3072) wired via jumper cables on SPI1. Receives LoRa packets and prints to USB serial with RSSI/SNR per packet and 30-second link quality summaries (min/avg/max RSSI+SNR, packet count, CRC errors). 5 onboard NeoPixels show RSSI bar graph (red→green gradient, 0-5 bars based on signal strength, 2-second timeout to off). Button 3 (GPIO5) toggles NeoPixels on/off. `gs_spi.cpp` provides the same `spi_bus_*()` API as the flight `spi_bus.cpp` but targets SPI1 (GPIO 28/30/31), allowing `rfm95w.cpp` to compile unchanged for both builds. Temporary 1 Hz test TX heartbeat added to flight firmware for link verification. TX power reduced to +5 dBm for bench testing. HW verified: RSSI -54 to -58 dBm, SNR 8-10 dB, <1% CRC errors at desk range. RP2350B PIO gpiobase=16 required for GPIO32 NeoPixels (SDK issue #2030).

(`ground_station/radio_rx.cpp`, `ground_station/gs_spi.cpp`, `ground_station/CMakeLists.txt`, `src/main.cpp`, `src/drivers/rfm95w.cpp`, `ground_station/lora_rx_simple/lora_rx_simple.ino`)

---

### 2026-02-24-004 | Claude Code CLI | hardware, documentation

**Hardware inventory update from Adafruit order.** Added new components to HARDWARE.md: ADXL375 high-G accelerometer and LoRa FeatherWings (×2) moved to on-hand inventory, Ground Station section expanded with Fruit Jam (#6200), HyperPixel 4.0" display, Cyberdeck HAT/Bonnet, LoRa Radio Bonnet with OLED, and Pi Zero 2W. Teensy 3.x Feather Adapter and Pico-to-Pi HAT X Converter added to accessories. ADXL375 datasheet (Analog Devices Rev. B) downloaded to `docs/hardware/datasheets/` and added to VENDOR_GUIDELINES.md inventory. SRAM audit closure, SCAFFOLDING.md fix, and 10 datasheets from prior session folded in per whiteboard note.

---

### 2026-02-24-003 | Claude Code CLI | feature, architecture

**Bierman measurement update adoption + SRAM DCP benchmark.** Replaced Joseph scalar measurement updates with Bierman on UD-factored covariance (43% faster per epoch: 486µs vs 851µs). Compile-time switch via `ESKF_USE_BIERMAN=1` (target only; host tests keep Joseph path unchanged). P representation state machine (`PRepr` enum + `ensure_dense()`/`ensure_ud()`) handles lazy factorize/reconstruct around codegen predict. SRAM DCP micro-benchmark confirmed DCP overhead is intrinsic register shuffling (~58 cyc/op from SRAM vs ~63 from flash), not XIP cache. Alpha precision canary: f32 relErr=1.37e-08 vs f64 reference — DCP Phase 2 deferred (not blocking). 207/207 host tests (199 original + 8 new Bierman). Target build clean.

**Net change summary:**

| Metric | Before | After | Delta |
|--------|--------|-------|-------|
| Measurement epoch | 851 µs | 486 µs | **-43%** (365 µs saved) |
| Scalar update (per) | 81 µs (Joseph) | 43 µs (Bierman) | **-47%** |
| CPU headroom @ 200Hz | 3,149 µs free | 3,514 µs free | **+365 µs WFI/cycle** |
| text (flash) | 137,732 B | 139,100 B | +1,368 B (+1.0%) |
| BSS (SRAM) | 88,268 B | 90,960 B | +2,692 B (+3.0%) |
| Host tests | 199 | 207 | +8 (Bierman suite) |
| f32 alpha precision | — | relErr 1.37e-08 | No f64 DCP needed |
| P stability | Symmetric, positive | Symmetric, positive | No regression (1000-cycle stress test) |
| Joseph form | Active | Retained behind `#ifdef` | Zero-risk A/B switch |

(`CMakeLists.txt`, `src/fusion/eskf.h`, `src/fusion/eskf.cpp`, `src/fusion/ud_factor.cpp`, `src/benchmark/ud_benchmark.cpp`, `test/test_eskf_bierman.cpp`, `test/CMakeLists.txt`)

---

### 2026-02-24-002 | Claude Code CLI | documentation

**MMAE pivot documentation + IVP restructuring.** Updated project documentation to reflect ESKF research findings: MMAE/IMM replaced by phase-scheduled Q/R (IVP-54, Stage 7). Stage 5 marked complete (IVP-39–48). New Stage 6: Flight Director (IVP-49–53). New Stage 7: Adaptive Estimation & Safety (IVP-54–57). Downstream stages renumbered 8–10 (IVP-58–72, total 72). Audited and fixed stale IVP references across 8 docs + 3 source files (comment-only: IVP-52→IVP-50 in eskf.h, main.cpp, mahony_ahrs.h). Added superseded banner to PHASE5_ESKF_PLAN.md.

(`docs/IVP.md`, `docs/PROJECT_STATUS.md`, `AGENT_WHITEBOARD.md`, `docs/SAD.md`, `docs/SCAFFOLDING.md`, `docs/PHASE5_ESKF_PLAN.md`, `docs/ESKF_TESTING_GUIDE.md`, `docs/flight_director/FLIGHT_DIRECTOR_DESIGN.md`, `docs/DYNAMIC_VALIDATION.md`, `src/main.cpp`, `src/fusion/eskf.h`, `src/fusion/mahony_ahrs.h`)

---

### 2026-02-24-001 | Claude Code CLI | architecture

**UD factorization + DCP float64 benchmark.** Implemented and ran 5-test benchmark suite comparing UD (Thornton WMGS + Bierman) vs current codegen FPFT + Joseph architecture. Phase 1 gate **PASS**: P is rock-stable at 100K steps with codegen — no negative diagonals, zero asymmetry, condition number bounded. UD not needed for numerical stability. DCP float64 is 7.8× slower than f32 FPU per MAC. Thornton f32 predict is 29.6× slower than codegen (1,420µs vs 48µs). Bierman scalar update is 2× faster than Joseph (43µs vs 81µs). Fixed Thornton D-array in-place corruption bug (algorithm requires old D values during WMGS sweep — added snapshot). Fixed NaN detection in `ud_all_positive()`.

(`src/fusion/ud_factor.h`, `src/fusion/ud_factor.cpp`, `src/benchmark/ud_benchmark.cpp`, `docs/benchmarks/UD_BENCHMARK_RESULTS.md`, `CMakeLists.txt`, `src/fusion/eskf.h`)

---

### 2026-02-23-002 | Claude Code CLI | documentation

**Dynamic validation methods document.** Created `docs/DYNAMIC_VALIDATION.md` — six repeatable physical test methods for verifying ESKF accuracy beyond host-side unit tests and stationary soaks: Allan variance (Q tuning validation), turntable rotation test, pendulum test, elevator test (baro fusion), data logging + replay infrastructure, and vehicle GPS-vs-INS comparison. Includes truth references, pass/fail criteria, and Python Allan variance implementation.

(`docs/DYNAMIC_VALIDATION.md`)

---

### 2026-02-23-001 | Claude Code CLI | architecture

**Dense FPFT + SRAM feasibility benchmark.** Tested dense O(N³) F*P*F^T at 24 states with SRAM placement (`.time_critical` section) to evaluate eliminating codegen maintenance. Result: **NOT VIABLE** — 1,747µs avg vs codegen 111µs (15.7× slower, 34.9% CPU at 200Hz). SRAM eliminated XIP cache thrashing (device runs, tight min/max) but O(24³)=13,824 MACs is the fundamental bottleneck. Codegen stays. SRAM audit of remaining hot-path functions found all <640 bytes — no further `.time_critical` placements needed. No code changes on main.

---

### 2026-02-22-002 | Claude Code CLI | documentation, tooling

**Prior art audit & license compliance.** Rewrote `LICENSE` (removed stale Adafruit/Arduino/LittleFS refs from FreeRTOS era). Created `THIRD_PARTY_LICENSES.md` with full attribution for all actual dependencies (Pico SDK BSD-3, ruuvi MIT, lwGPS MIT, ICM-20948 MIT, GoogleTest BSD-3, WMM table from ArduPilot GPL-3). Added SPDX headers (`GPL-3.0-or-later`) to all 54 project-owned source files. Removed unused `ws2812b-animation` submodule. Added explicit GPL-3.0 attribution to WMM declination table.

(`LICENSE`, `THIRD_PARTY_LICENSES.md`, `.gitmodules`, 54 source/test files)

---

### 2026-02-22-001 | Claude Code CLI | refactor

**Post-Stage 5 code audit cleanup.** Updated stale build tag (`ivp45-4` → `stage5-complete`), fixed outdated "not yet implemented" comment for 6-pos accel cal in `calibration_manager.h`, clarified baro_kf rejection comment. Deleted 6 untracked `.uf2` files (old debug probe firmware + test binary) from repo root. Full codebase audit found no debugging artifacts, commented-out code, or dead code — codebase is clean.

---

### 2026-02-21-003 | Claude Code CLI | feature, architecture

**24-State ESKF Expansion with Runtime Inhibit Flags**

Expanded error-state vector from 15 to 24 states: earth_mag NED (3), body_mag_bias (3), wind_NE (2), baro_bias (1). ArduPilot EKF3-pattern runtime inhibit flags (`inhibit_mag_states_`, `inhibit_wind_states_`, `inhibit_baro_bias_`) — all inhibited by default, P zeroed for inhibited blocks. When inhibited, codegen propagates all 24 states (identity F for new states, near-zero cost) but `clamp_covariance()` zeros inhibited P blocks. `set_inhibit_*()` methods handle enable/disable with proper P initialization and cross-covariance zeroing.

Codegen regenerated for 24 states via `generate_fpft.py` (N=24). Benchmark: **111µs avg, 101µs min, 156µs max** (was 59µs at 15-state). Critical runtime fix: O(N²) rank-1 Joseph form replaced O(N³) dense triple product in `scalar_kalman_update()` — device was unresponsive with dense 24×24 matrix multiplies at measurement update rates. Sparse `reset()` exploits G=I structure except 3×3 attitude block (~450 MACs vs ~27,648 dense). `healthy()` made inhibit-aware (skips P diagonal check for inhibited indices).

Baro update subtracts `baro_bias_` when enabled. Mag states (15-20) are **unobservable** with current yaw-only H — require 3-axis mag model (Titan tier). 5 new inhibit tests, Mat15→Mat24 across all test files. All 5 replay reference CSVs regenerated. CLI shows `inhib: mag=Y wind=Y bbias=Y` + conditional extended state display when states enabled. 199/199 host tests pass. HW verified: 0 sensor errors.

(`src/fusion/eskf.cpp`, `src/fusion/eskf.h`, `src/fusion/eskf_state.h`, `src/fusion/eskf_codegen.cpp`, `src/fusion/eskf_codegen.h`, `scripts/generate_fpft.py`, `src/math/mat.h`, `src/main.cpp`, `test/test_eskf_propagation.cpp`, `test/test_eskf_update.cpp`, `test/test_eskf_mag_update.cpp`, `test/test_eskf_zupt.cpp`, `test/test_mat.cpp`, `test/data/reference/*.csv`)

---

### 2026-02-21-002 | Claude Code CLI | feature, architecture

**IVP-47: Codegen FPFT — 9.1× speedup, predict() at 59µs (was 538µs)**

SymPy codegen script (`scripts/generate_fpft.py`) generates flat scalar C++ for F*P*F^T + Q_d covariance propagation. 199 CSE intermediates, 120 upper-triangle outputs, Q_d constants baked in. Read-after-write hazard fixed by snapshotting all 120 P inputs to locals before computation. Running from SRAM (`.time_critical` section) — XIP cache (2KB) thrashes on 10KB function from flash (398µs), SRAM eliminates cache entirely (59µs). Three-layer verification: SymPy self-test (2.6e-18 vs NumPy), Test 8 CodegenVsDenseFPFT (100 steps, 1e-4), Test 15 CodegenSingleStep (1e-6). `static_assert` guards sync codegen constants with eskf.h. 194/194 host tests pass. Binary: 239,616 bytes (+21KB codegen, +10KB .data from SRAM placement). Benchmark: 59µs avg, 50µs min, 113µs max (17,883 calls), 0 sensor errors. Standards deviation CG-1 logged for auto-generated function exceeding 200 L-SLOC.

(`scripts/generate_fpft.py`, `src/fusion/eskf_codegen.cpp`, `src/fusion/eskf_codegen.h`, `src/fusion/eskf.cpp`, `CMakeLists.txt`, `test/test_eskf_propagation.cpp`, `test/data/reference/*.csv`, `standards/STANDARDS_DEVIATIONS.md`)

---

### 2026-02-21-001 | Claude Code CLI | feature, architecture

**IVP-47: Block-sparse FPFT experiment — reverted to dense after benchmark**

Implemented and verified block-sparse FPFT covariance propagation exploiting F_x block structure (15 of 25 blocks are zero). Algebraically correct: 193/193 host tests pass, 1e-6 single-step tolerance vs dense reference. However, on-target benchmark showed block-sparse was **31% slower** than dense (712µs avg vs 542µs avg). Root cause: ~30+ `block3()` Mat3 copies per step, poor cache locality jumping around 15×15 P matrix, and dense inner loops autovectorize better on Cortex-M33. Theoretical 10× FMA reduction doesn't offset memory access overhead.

**Outcome:** Reverted predict() to dense. Retained `block3()`/`set_block3()`/`add_block3()` helpers in mat.h (useful for measurement updates). Retained PSymmetry test. predict() at 538µs avg confirmed on target (matches pre-experiment baseline). **Codegen (SymPy/SymForce element-wise scalar expansion) is the correct optimization path** — scheduled for 24-state wind estimation expansion, where dense becomes O(24³×3) ≈ 41K FMA vs codegen O(N²).

*Block-sparse and codegen are fundamentally different approaches. ArduPilot/PX4 use codegen (SymPy/SymForce), not block-sparse. Codegen emits flat scalar C++ with CSE — zero function calls, zero copies, zero temporaries at runtime.*

(`src/fusion/eskf.cpp`, `src/math/mat.h`, `test/test_eskf_propagation.cpp`, `test/data/reference/*.csv`)

---

### 2026-02-20-005 | Claude Code CLI | feature

**IVP-47: ESKF health tuning — mag heading fix + diagnostics**

Fixed mNIS=124.99 death spiral (mag heading completely non-functional). Three-part fix: (1) tilt-conditional R inflation at 30-60° with hard reject above 60° (ArduPilot `fuseEulerYaw` pattern), (2) public `reset_mag_heading()` API for state machine use (IVP-52), (3) mag gate widened to 300σ matching ArduPilot EKF3 (interference detection handles bad data, not the gate). Added per-sensor accept/reject counters for baro, mag, GPS pos/vel, ZUPT. Q constants reviewed — all correct per ICM-20948 datasheet. CLI `s` command shows gate counters + P velocity/bias diagonals. CLI `e` command shows mag accept ratio + ZUPT NIS. 192/192 host tests pass. HW verified: 65s soak, mNIS 0.00–0.52 (was 124.99), bNIS 0–4, zero sensor errors.

(`src/fusion/eskf.h`, `src/fusion/eskf.cpp`, `src/main.cpp`, `test/test_eskf_mag_update.cpp`)

---

### 2026-02-20-004 | Claude Code CLI | tooling

**Pre-commit hook: function size + cognitive complexity gate**

Added `.git/hooks/pre-commit` that runs `readability-function-size` and `readability-function-cognitive-complexity` checks on staged `.cpp` files in `src/`. Blocks commit if any function exceeds thresholds (60 lines, 200 statements, nesting 6, CC 25). Uses same clang-tidy path and ARM cross-compilation args as `scripts/run_clang_tidy.sh`. Skippable with `--no-verify`.

---

### 2026-02-20-003 | Claude Code CLI | refactor

**P5c: decompose 24 function-size/CC warnings to zero**

Per plan: `.claude/plans/quirky-squishing-clarke.md`. Four batches executed in council-mandated order (lowest risk first): Batch 3 `rc_os.cpp` (wizard + mag cal extraction), Batch 4 `calibration_manager.cpp` (LM solver dedup via function pointers), Batch 2 `main.cpp` (eskf_tick + sensor print decomposition), Batch 1 `eskf.cpp` (Joseph-form deduplication into `scalar_kalman_update()`). Additional borderline findings caught in final audit: `core1_read_imu`, `core1_read_gps`, `core1_sensor_loop`, `eskf_tick_gps`, `lm_solve` — all decomposed. Binary: text=114,144 (-2,248 from P5b baseline), BSS=96,524 (-11,408, from consolidating 5 sets of static Mat15 into 1). Full clang-tidy audit: 0 function-size/CC findings in production code. HW verified: 60s soak, 86K IMU reads, 0 errors, ESKF stable (qnorm=1.000000, vel<0.03 m/s).

(`src/fusion/eskf.cpp`, `src/fusion/eskf.h`, `src/main.cpp`, `src/calibration/calibration_manager.cpp`, `src/cli/rc_os.cpp`)

---

### 2026-02-20-002 | Claude Code CLI | documentation

**Pre-audit documentation cleanup: 10 files updated for consistency**

Cross-document review and reconciliation ahead of full clang-tidy audit. Major updates: SCAFFOLDING.md complete rewrite to match actual filesystem (was 18 days stale with fictional directories); SAD.md converted from living-document style to high-level architecture reference with current implementation status; VENDOR_GUIDELINES.md updated for ICM-20948 bypass mode (implemented 2026-02-10) and UART GPS transport (preferred since IVP-46); GPS UART driver added to CODING_STANDARDS.md file classification and STANDARDS_DEVIATIONS.md IO-2 deviation. Removed empty `src/debug/` and `src/tasks/` directories and 6 stale temp files.

(`docs/SCAFFOLDING.md`, `docs/SAD.md`, `docs/PROJECT_STATUS.md`, `docs/IVP.md`, `standards/VENDOR_GUIDELINES.md`, `standards/CODING_STANDARDS.md`, `standards/AUDIT_REMEDIATION.md`, `standards/STANDARDS_AUDIT.md`, `standards/STANDARDS_DEVIATIONS.md`, `AGENT_WHITEBOARD.md`)

---

### 2026-02-20-001 | Claude Code CLI | feature

**GPS upgraded to 57600 baud + 10Hz update rate**

`gps_uart_init()` now negotiates the MT3339 from factory-default 9600 baud to 57600 baud via PMTK251, then sets 10Hz update rate via PMTK220. Cold-start safe: always detects presence at 9600 then negotiates. 9600 baud saturates at ~4.8 NMEA bursts/sec and cannot sustain 10Hz; 57600 gives 2.8× headroom. ESKF GPS measurement updates (`update_gps_position()` / `update_gps_velocity()`) automatically benefit — they fire on each new `gps_read_count` increment with no further code changes needed.

HW verified: ~127 GPS reads/10s, rxOvf=0, IMUerr=0. (`src/drivers/gps_uart.cpp`, `src/drivers/gps_uart.h`)

---

### 2026-02-19-001 | Claude Code CLI | bugfix, architecture

**ESKF velocity divergence: root-cause fix for silent ICM-20948 zero-output fault**

Fixed catastrophic ESKF divergence (`vel=1688 m/s`, `bNIS=23,000,000`) caused by ICM-20948 entering sleep/reset state mid-session. The device ACKs I2C reads in sleep mode and returns all-zero output registers — invisible to the existing consecutive-fail counter which only triggers on NACKs. With raw accel ≈ (0,0,0), the ESKF propagated a constant +9.8 m/s² specific force in NED-Z, accumulating 1688 m/s after ~3 min. ZUPT and ESKF init guards both failed because `|A|=0.285 m/s²` (calibration offset residual) is nowhere near 9.8 m/s².

Three-part fix: (1) `core1_read_imu()` now validates raw accel magnitude against `kAccelMinHealthyMag = 3.0f` m/s² (gravity floor at 72° tilt: 9.8×cos(72°)) after every successful I2C read, routing zero-output reads to the consecutive-fail path so bus-recover and device-reinit fire. (2) `healthy()` in `eskf.cpp` now checks `v.norm() >= kMaxHealthyVelocity (500 m/s)` as a velocity-divergence sentinel — above max hobby rocket burnout, below any slow divergence trajectory. (3) `eskf_tick()` reset-on-unhealthy path already correct (verified, no change needed).

HW verified: 60s indoor soak, max bNIS=3.81, max vel=0.077 m/s, 0 IMU errors. 187/187 host tests pass.

*Root cause established by backward math: stored cal `off=[0.0235, 0.1632, -0.2364]` implies raw_x≈0, raw_y≈0, raw_z≈0 — physically impossible from any real sensor orientation. The calibration was correct; the sensor was in sleep state.*

(`src/main.cpp`, `src/fusion/eskf.h`, `src/fusion/eskf.cpp`, `src/drivers/gps_uart.cpp`)

---

### 2026-02-18-003 | Claude Code CLI | bugfix, feature

**IVP-46 Step 9 fix: interrupt-driven UART GPS + ESKF init NeoPixel**

Fixed UART FIFO overflow that caused Step 9 outdoor validation failure — GPS GGA/RMC sentences lost due to 32-byte hardware FIFO filling between 10Hz polls. Added interrupt-driven 512-byte SPSC ring buffer (ISR on Core 0, consumer on Core 1). Also added fast red NeoPixel blink during ESKF stationary init as "hold still" cue.

Outdoor verified: Fix=3 Sats=12 HDOP=0.90, GPS feeding ESKF (G=Y). 60s soak: bNIS 0.00–2.13, zero UNHEALTHY, rxOvf=0.

(`src/drivers/gps_uart.cpp`, `src/drivers/gps_uart.h`, `src/main.cpp`, `AGENT_WHITEBOARD.md`)

---

### 2026-02-18-002 | Claude Code CLI | feature, architecture

**Stage 4 GPS: IVP-46 complete (8-step incremental plan)**

ESKF GPS position + velocity measurement updates (IVP-46). 8 individually HW-verified commits:
1. Transport-neutral `gps_data_t` types extracted to `gps.h`
2. UART GPS driver (`gps_uart.cpp`) for FeatherWing on GPIO0/1
3. ESKF GPS methods + **P covariance reset fix** in `set_origin()` (council-identified root cause of previous bNIS explosion) + 18 host tests including PostOriginBaroStability
4. HDOP/VDOP in shared sensor data + CLI display
5. Transport detection (UART-first, I2C fallback) + function pointer dispatch + hold-on-valid pattern
6. Best-fix diagnostic capture with atomic cross-core visibility
7. ESKF GPS wiring in `eskf_tick()` — origin establishment, geodetic-to-NED, position/velocity updates, 10km origin re-centering
8. SENSOR_ARCHITECTURE doc + soak script + CHANGELOG

**Critical fix:** `set_origin()` now resets P[3..8] diagonal to GPS-derived uncertainty and zeros all cross-covariance terms. Without this, stale correlations from pre-origin predict cycles corrupted baro Kalman gain → bNIS explosion.

Host tests: 173/173 pass. 5-minute indoor soak: bNIS max=4.09, 0 UNHEALTHY, ZUPT 100% active.

(`src/fusion/eskf.h`, `src/fusion/eskf.cpp`, `src/fusion/eskf_state.h`, `src/drivers/gps.h`, `src/drivers/gps_uart.h`, `src/drivers/gps_uart.cpp`, `src/drivers/gps_pa1010d.h`, `src/drivers/gps_pa1010d.cpp`, `src/main.cpp`, `test/test_eskf_gps_update.cpp`, `docs/SENSOR_ARCHITECTURE.md`, `scripts/eskf_gps_soak.py`)

---

### 2026-02-18-001 | Claude Code CLI | documentation

**Session handoff: Stage 4 GPS reboot deferred — bNIS regression unresolved**

Attempted Stage 4 GPS reboot (protocol-agnostic/UART-first approach per plan `purrfect-pondering-hopcroft.md`). Commits A/B/C built clean and passed 172/172 host tests, but HW validation revealed ESKF bNIS explosion (~125K→589K after ~12 baro updates, stationary). Root cause not diagnosed — may predate Stage 4. All Stage 4 commits reverted; repo returned to `17c4111`. Next session must diagnose bNIS regression before resuming Stage 4. See AGENT_WHITEBOARD.md for full handoff notes.

(`AGENT_WHITEBOARD.md`)

---

### 2026-02-14-001 | Claude Code CLI | documentation, architecture

**Flight Director & Mission Profile: naming, docs, cross-reference sync**

Established council-decided naming convention: **Flight Director** (runtime engine) and **Mission Profile** (configuration data), replacing the umbrella term "Mission Engine." Created `docs/flight_director/` with comprehensive design spec (`FLIGHT_DIRECTOR_DESIGN.md`, all details PRELIMINARY) and moved research doc there (`RESEARCH.md`, historical — unchanged). Created `docs/mission_profiles/` with stub (`MISSION_PROFILES.md`, to be fleshed out in future session). Updated 32 cross-references across 11 files to use correct terminology. Historical research doc preserved with original naming.

(`docs/flight_director/FLIGHT_DIRECTOR_DESIGN.md`, `docs/flight_director/RESEARCH.md`, `docs/mission_profiles/MISSION_PROFILES.md`, `docs/SAD.md`, `docs/IVP.md`, `docs/SCAFFOLDING.md`, `standards/CODING_STANDARDS.md`, `docs/decisions/SENSOR_FUSION_TIERS.md`, `docs/decisions/TITAN_BOARD_ANALYSIS.md`, `docs/decisions/ESKF/FUSION_ARCHITECTURE_DECISION.md`, `docs/ESKF_TESTING_GUIDE.md`, `docs/PHASE5_ESKF_PLAN.md`, `docs/PROJECT_OVERVIEW.md`, `tools/state_to_dot.py`)

---

### 2026-02-13-004 | Claude Code CLI | architecture, feature

**Modular GPS refactor: transport-neutral types + UART backend**

Refactored GPS subsystem into transport-neutral data types + transport-specific backends, establishing the pattern for migrating all base sensors off I2C (IMU→SPI, baro→SPI in future). Created `gps.h` with shared `gps_data_t`, `gps_fix_t`, `gps_transport_t` (zero transport dependencies). Updated `gps_pa1010d.h/cpp` to use shared types. Created UART GPS backend (`gps_uart.h/cpp`) for Adafruit Ultimate GPS FeatherWing (#3133) on GPIO0/1 (9600 baud NMEA, lwGPS parser, 2-second presence detection timeout). Added function pointer dispatch in main.cpp — auto-detects UART first (production), falls back to I2C (Qwiic dev). Boot banner and CLI status display show transport type. Created `docs/SENSOR_ARCHITECTURE.md` documenting the modular sensor pattern.

Host tests: 172/172 pass. Target build: 0 errors, 0 warnings. Binary: 216,576 bytes UF2. HW verification deferred to UART GPS wiring.

(`src/drivers/gps.h`, `src/drivers/gps_uart.h`, `src/drivers/gps_uart.cpp`, `src/drivers/gps_pa1010d.h`, `src/drivers/gps_pa1010d.cpp`, `src/main.cpp`, `CMakeLists.txt`, `docs/SENSOR_ARCHITECTURE.md`)

---

### 2026-02-13-003 | Claude Code CLI | feature

**IVP-46: GPS position & velocity measurement update for ESKF**

Added GPS position (3 sequential scalar updates) and velocity (2 sequential scalar updates, N/E only) measurement updates to the ESKF. Flat-earth NED frame with double-precision geodetic origin, moving-origin mechanism for HAB-range flights (>10km), HDOP-scaled position noise (σ=3.5m base, MT3333 CEP50), innovation gating (5σ), per-axis NIS tracking.

New files: `test/test_eskf_gps_update.cpp` (17 tests: origin management, geodetic conversion, position/velocity updates, gating, convergence, full pipeline), `scripts/eskf_gps_soak.py` (indoor/outdoor/walk modes). Added HDOP/VDOP to seqlock struct, named state index constants (`kIdxPosN` etc.) to `eskf_state.h`, GPS constants + NED origin state + 5 methods to ESKF. Wired into `eskf_tick()` between mag update and ZUPT. Updated `s` and `e` CLI displays with position, GPS NIS, and origin flag.

Council-approved (3 panelists, unanimous). 8 conditions integrated: C-1 MT3333 noise, C-2 velocity NE-only, C-3 named indices, C-4 HDOP origin gate, C-5 double-precision subtraction, C-6 ZUPT stays IMU-only, C-7 origin reset continuity, C-8 course-to-velocity formula.

Host tests: 172/172 pass (155 existing + 17 new). Target build: 0 errors, 0 warnings. Binary: 213,504 bytes UF2 (+6,144 from GPS methods). Outdoor gate tests deferred to LoRa serial bridge setup.

(`src/fusion/eskf.h`, `src/fusion/eskf.cpp`, `src/fusion/eskf_state.h`, `src/main.cpp`, `test/test_eskf_gps_update.cpp`, `test/CMakeLists.txt`, `scripts/eskf_gps_soak.py`)

---

### 2026-02-13-002 | Claude Code CLI | refactor, documentation

**Pre-IVP-46 cleanup: remove standalone BaroKF from firmware, wire WMM declination, doc sweep**

Removed standalone BaroKF from main firmware — superseded by ESKF `update_baro()` (IVP-43). Removed `fusion_tick()`, BaroKF globals, KF display line from `print_seqlock_sensors`, CMakeLists target build entry. Host tests + mat_benchmark retain baro_kf for unit testing and benchmarking.

Wired WMM declination into mag heading update: uses `wmm_get_declination(lat, lon)` when GPS has valid fix (fix_type >= 2), falls back to 0 (magnetic heading) without GPS. Removed unused `g_loopCounter`. Updated stale GPS comment in `i2c_bus.cpp`.

Living document cleanup: updated PROJECT_STATUS.md (cleared stale baro NIS blocker, fixed IVP-46 as next step, noted standalone BaroKF removal), HARDWARE.md (baro cross-reference now points to eskf.h), ESKF_TESTING_GUIDE.md (removed false "gitignored" claim), AGENT_WHITEBOARD.md (DPS310 datasheet priority adjusted). CODING_STANDARDS.md and SEQLOCK_DESIGN.md cosmetic edits from earlier in session.

Host tests: 155/155 pass. Target build: 0 errors, 0 warnings. Binary: 207,360 bytes UF2.

(`src/main.cpp`, `CMakeLists.txt`, `src/drivers/i2c_bus.cpp`, `docs/PROJECT_STATUS.md`, `docs/hardware/HARDWARE.md`, `docs/ESKF_TESTING_GUIDE.md`, `AGENT_WHITEBOARD.md`, `standards/CODING_STANDARDS.md`, `docs/decisions/SEQLOCK_DESIGN.md`)

---

### 2026-02-13-001 | Claude Code CLI | feature

**IVP-44 + IVP-44b: ESKF magnetometer heading update + zero-velocity (ZUPT)**

Implemented mag heading measurement update (IVP-44): tilt-compensated heading via zero-yaw rotation (ArduPilot fuseEulerYaw approach), wrap_pi innovation, two-tier interference detection (25% inflates R 10x, 50% hard rejects, council), declination_rad parameter for WMM true heading. 16 new host tests. Mag-based initial yaw at ESKF init prevents gate rejection when heading far from 0°.

Added ZUPT pseudo-measurement (IVP-44b): prevents horizontal velocity divergence in GPS-denied operation (previously ~48 m/s in 30s, now <0.03 m/s for 60s+). Stationarity detection from accel/gyro, three sequential scalar velocity updates with Joseph form, kSigmaZupt=0.5 matching ArduPilot EKF3. 13 new host tests.

Target integration: sensor_to_ned INTERIM Z-negate helpers (ICM-20948 Z-up → NED Z-down), mag update at ~10Hz via seqlock, ZUPT at 200Hz, live display shows yaw/mNIS/ZUPT flag. Watchdog sentinel refinement (scratch[0] custom sentinel replaces broken SDK functions). Synthetic data + replay harness updated with mag columns.

Host tests: 135/135 (29 new). Binary: 202,752 bytes UF2. HW verified: 60s stationary, vh<0.03, Z=Y, mNIS<1, yaw drift <1°, 0 IMU errors.

(`src/fusion/eskf.h`, `src/fusion/eskf.cpp`, `src/main.cpp`, `test/test_eskf_mag_update.cpp`, `test/test_eskf_zupt.cpp`, `test/CMakeLists.txt`, `test/scripts/generate_synthetic.py`, `test/replay/replay_harness.cpp`, `test/test_replay_regression.cpp`, `test/data/*.csv`)

---

### 2026-02-12-003 | Claude Code CLI | feature, bugfix

**IVP-43: ESKF barometric altitude measurement update + 6-param accel cal fix**

Implemented ESKF baro measurement update (IVP-43): scalar update with Joseph form P update (inline statics for stack safety), 3σ innovation gating, NIS diagnostic, `isfinite`/S guards per council conditions. Added live ESKF mode ('e' key for 1Hz compact status: alt, velocity, Patt, bNIS). 11 new host tests (106/106 total pass). Fixed 6-pos accel cal: reduced Gauss-Newton solver from 9 to 6 parameters — 6 axis-aligned positions provide only 6 constraints, offdiag sits in Jacobian null space. Board rotation matrix added to status output. HW verified: |A|=9.770, bNIS stable, 0 I2C errors.

(`src/fusion/eskf.cpp`, `src/fusion/eskf.h`, `src/main.cpp`, `src/cli/rc_os.cpp`, `src/cli/rc_os.h`, `src/calibration/calibration_manager.cpp`, `test/test_eskf_update.cpp`, `test/CMakeLists.txt`)

---

### 2026-02-12-002 | Claude Code CLI | bugfix

**Fix calibration data flow: 6-pos accel zeros, mag cal zeros, wizard persistence**

Three bugs fixed during HW calibration testing: (1) 6-pos accel cal returned zeros after ~200 reads — `read_accel_for_cal()` used 6-byte read that doesn't clear ICM-20948 data-ready flag, switched to 14-byte `icm20948_read()`. (2) Mag cal fed zero samples — `icm20948_read()` mag divider skipped 9/10 calls without setting `mag_valid=false`, uninitialized stack value bypassed staleness gate, added else clause. (3) Wizard gyro/level results lost after Core 1 resume — wizard stored in RAM but never called `calibration_save()`, Core 1 reload from flash overwrote them, now saves to flash after each step. HW verified: mag cal 300 samples, 90% coverage, RMS 0.927 uT.

(`src/main.cpp`, `src/drivers/icm20948.cpp`, `src/cli/rc_os.cpp`)

---

### 2026-02-12-001 | Claude Code CLI | bugfix, architecture

**Watchdog FMEA analysis, IVP-51 Watchdog Recovery Policy, reboot detection bugfix**

Fixed `watchdog_enable_caused_reboot()` → `watchdog_caused_reboot()` in `init_hardware()`. The `_enable_` variant checks scratch[4] magic that persists across picotool flashes, causing false "PREVIOUS REBOOT WAS CAUSED BY WATCHDOG" warnings on clean boots. The base function uses RP2350-specific `rom_get_last_boot_type()` for correct POR discrimination.

Added IVP-51 (Watchdog Recovery Policy) as first step of Stage 6, prerequisite to state machine. Covers: scratch register persistence for reboot diagnostics, reboot counting with safe-mode lockout, ESKF failure backoff, recovery boot path, ground-side launch abort on any WDT reset. All downstream IVP numbers shifted +1 (Stage 7: IVP-57-61, Stage 8: IVP-62-66, Stage 9: IVP-67-71). Updated Gemini carrier board doc with cross-board watchdog handoff and per-board status LEDs. Updated SAD Section 14.

(`src/main.cpp`, `docs/IVP.md`, `AGENT_WHITEBOARD.md`, `docs/hardware/GEMINI_CARRIER_BOARD.md`, `docs/SAD.md`)

---

### 2026-02-10-008 | Claude Code CLI | council, documentation

**Council decision: Sensor fusion tier architecture (MMAE + sensor affinity)**

Added council-approved architectural decision document for hybrid MMAE + sensor affinity across hardware tiers (Core: single ESKF, Titan: MMAE regime switching, Gemini: full MMAE + affinity with dual-MCU). Unanimous approval with staged implementation. Added MATLAB .mat v5 export to PROJECT_STATUS.md future features roadmap.

(`docs/decisions/SENSOR_FUSION_TIERS.md`, `docs/PROJECT_STATUS.md`)

---

### 2026-02-10-007 | Claude Code CLI | feature, bugfix

**ICM-20948: Migrate from I2C master mode to bypass mode**

Replaced ICM-20948's internal I2C master with bypass mode (`INT_PIN_CFG.BYPASS_EN=1`) for AK09916 magnetometer access. AK09916 now reads directly at 0x0C on the external I2C bus, eliminating the bank-switching race (LL Entry 21), progressive master stall, and unrecoverable disable/enable cycle that caused mag cal failures. Removed ~120 lines of I2C master plumbing (Bank 3 registers, SLV0 config, master clock setup). Added mag read divider (100Hz vs 1kHz), two-level device recovery (bus recover at 10 fails, full device reset at 50), lazy mag re-init after device reset, and GPS pause during mag cal (`rc_os_mag_cal_active` flag). Reordered `init_sensors()` so GPS probe/init happens after IMU bypass mode is established. Council-approved (unanimous). HW verified: mag cal 300 samples/72% coverage/RMS 0.878 uT with GPS on bus, system self-recovers from lockups.

*ArduPilot uses the same approach (`AP_InertialSensor_Invensensev2.cpp`). The I2C master creates an entire class of bugs that bypass mode eliminates entirely.*

(`src/drivers/icm20948.cpp`, `src/drivers/icm20948.h`, `src/main.cpp`, `src/cli/rc_os.cpp`, `src/cli/rc_os.h`)

---

### 2026-02-10-006 | Claude Code CLI | feature, bugfix

**Phase M.5 complete: Full calibration wizard + mag cal HW verified**

NeoPixel now shows mode color during ENTER wait (before user presses ENTER, not after). All standalone cal commands match wizard UX with ENTER prompt + NeoPixel feedback. Added `cmd_accel_6pos_cal()` initial ENTER prompt (was missing). Added mag cal diagnostics: cancel prints accepted/read/readFail/close/range breakdown, sensor status shows `M=` (mag_read_count), periodic `mag_valid=false` diagnostic prints. Full mag cal HW verified: 300 samples, 81% coverage, ellipsoid fit RMS 2.499 uT, save to flash OK. Build tag: wizard-12.

(`src/main.cpp`, `src/cli/rc_os.cpp`)

---

### 2026-02-10-005 | Claude Code CLI | feature

**Phase M.5: Unified calibration wizard with NeoPixel feedback**

Implemented full 5-step calibration wizard (gyro, level, baro, 6-pos accel, compass) accessible from CLI `c` → `w`. Each step waits for ENTER to start, 'x' to skip/cancel. NeoPixel shows calibration state via cross-core atomic override (blue breathe = IMU sampling, cyan = baro, yellow = accel positioning, rainbow = mag rotation, green = success, red = failure). Moved async calibration sensor feeds from Core 0 to Core 1 to eliminate I2C bus contention — Core 0 no longer touches I2C during gyro/level/baro calibrations. Extracted `mag_cal_inner()` for reuse between standalone command and wizard. Fixed watchdog crash in blocking prompts (polling loop with `watchdog_update()` instead of 30s `getchar_timeout_us()`). Fixed auto-progress bug from stale USB input buffer bytes (double-drain with 100ms gap). Fixed mag cal collecting only 1 sample: seqlock now carries raw mag data (`mag_raw_x/y/z`) alongside calibrated — ellipsoid solver needs uncorrected data. Struct grew 128→140 bytes. HW verified: wizard-6, gyro/level/baro/6-pos accel all pass.

(`src/main.cpp`, `src/cli/rc_os.cpp`, `src/cli/rc_os.h`)

---

### 2026-02-10-004 | Claude Code CLI | feature

**Phase M complete: Core 1 live mag apply + heading display (IVP-38)**

Applied mag calibration on Core 1 via `calibration_apply_mag_with()`, matching existing accel/gyro pattern. Sensor status (`s`) now shows calibrated magnitude `|M|` and tilt-uncorrected heading (0-360 deg). Both seqlock display and pre-sensor-phase direct-read paths updated. HW verified: 0 IMU errors, `|M|` ~60 µT stable, heading tracks smoothly. Phase M (IVP-34 through IVP-38) is now complete — all 4 commits merged.

(`src/main.cpp`)

---

### 2026-02-10-003 | Claude Code CLI | feature

**Non-blocking USB init — firmware runs without terminal**

Replaced blocking `wait_for_usb_connection()` with non-blocking `stdio_init_all()`. Boot banner and HW status deferred to first terminal connection via `rc_os_print_boot_status` callback. Core 1 sensor phase and watchdog start immediately with no USB dependency. Qwiic chain order documented: GPS must be first (closest to board) — at end of chain, probe detection is intermittent. Soak verified: 536K IMU reads, 0 errors (6 min, all 3 sensors).

(`src/main.cpp`, `src/cli/rc_os.cpp`, `src/cli/rc_os.h`, `docs/hardware/HARDWARE.md`)

---

### 2026-02-10-002 | Claude Code CLI | architecture, documentation

**SAD Section 13: I2C peripheral detection and driver management architecture**

Expanded SAD Section 13 (Extensibility) from placeholder to full architecture for I2C peripheral detection. Boot-time probe-first detection (already implemented). Runtime hot-plug detection, device registry with WHO_AM_I disambiguation, and OTA driver downloads for WiFi/BT models (crowdfunding stretch goal for Core/Middle tiers). Resolved open questions #1 (Booster Pack detection) and #2 (OTA updates). Established ESKF gate: foundational features (mag cal wizard, non-blocking USB, unified calibration) must complete before Stage 5.

(`docs/SAD.md`, `AGENT_WHITEBOARD.md`, `docs/PROJECT_STATUS.md`)

---

### 2026-02-10-001 | Claude Code CLI | bugfix, feature

**Fix i2c_bus_recover() peripheral corruption + probe-first detection**

Fixed critical bug: `i2c_bus_recover()` corrupted the DW_apb_i2c peripheral by switching GPIO functions (SIO↔I2C) while the peripheral was enabled. Now properly deinits before GPIO switch and reinits after (LL Entry 28). Added probe-first peripheral detection in `init_sensors()` — only inits drivers for devices physically present on the bus. GPS drain no longer triggers recovery for absent devices. Recovery improved with SCL-stuck-low check and per-pulse SDA early exit (Linux kernel pattern). New soak test script with early-fail detection and SWD reset support. Soak verified: 386K IMU reads, 0 errors.

(`src/drivers/i2c_bus.cpp`, `src/main.cpp`, `scripts/i2c_soak_test.py`)

---

### 2026-02-09-008 | Claude Code CLI | bugfix

**Revert non-blocking USB init (6de6245) — soak failures across 4 build variants**

Reverted premature commit 6de6245 which removed `wait_for_usb_connection()`. Deep research confirmed Pico SDK has no I2C bus recovery function (custom recovery is correct per I2C spec NXP UM10204 Section 3.1.16). Attempted 4 build variants (prod-13 through prod-16) with robust bus recovery (retry loop, 100µs GPIO settling delay, `i2c_deinit()` before bit-bang) and non-blocking USB. All failed soak testing at 40-90 seconds with cascading I2C errors (IMU first, then baro/GPS). Phase M stash@{0} (CLI mag cal) preserved. **[CORRECTION 2026-02-09]:** The "codegen sensitivity" attribution was wrong — all testing used picotool rapid flash cycles, which corrupt the I2C bus (LL Entry 25). Disproved by three soak tests via debug probe with i2c_bus.cpp modifications, all 0 errors (LL Entry 27). Non-blocking USB should be retested via probe.

(`src/main.cpp`, `src/cli/rc_os.cpp`, `src/cli/rc_os.h`, `AGENT_WHITEBOARD.md`)

---

### 2026-02-09-007 | Claude Code CLI | documentation

**IVP plan expansion: Phase M magnetometer calibration + Stage 5 sensor fusion flesh-out**

Added Phase M (IVP-34 through IVP-38) — 5 magnetometer calibration IVPs with full ArduPilot CompassCalibrator parity (sphere-coverage, two-step Levenberg-Marquardt, ellipsoid fit). Renumbered all downstream IVPs +5 (Stage 5: IVP-39-50, Stage 6: IVP-51-56, Stage 7: IVP-57-61, Stage 8: IVP-62-66, Stage 9: IVP-67-71). Fleshed out all 10 Stage 5 IVPs with full implementation specs: Vec3/Quaternion library, matrix operations, 1D baro KF, ESKF propagation (Sola 2017), baro/mag/GPS measurement updates, Mahony AHRS, MMAE bank manager, confidence gate. Updated cross-references in AGENT_WHITEBOARD.md, PROJECT_STATUS.md, VENDOR_GUIDELINES.md.

(`docs/IVP.md`, `AGENT_WHITEBOARD.md`, `docs/PROJECT_STATUS.md`, `standards/VENDOR_GUIDELINES.md`)

---

### 2026-02-09-006 | Claude Code CLI | refactor

**Strip IVP test code from production codebase**

Removed all Stage 1-4 IVP test code from `main.cpp` after hardware verification: 3,623→1,073 lines (**2,550 lines removed, 70% reduction**). 33 functions, ~80 state variables, ~70 constants deleted including 4KB jitter timestamp array. Binary **198,144→155,648 bytes (42,496 bytes saved, 21.4% smaller)**. Replaced `g_ivp25Active` with production `g_sensorPhaseActive` (plain bool). Unconditional watchdog enable (council critical fix). Renamed `hw_validate_stage1()` to `print_hw_status()`. Resolved 2 standards deviations: RC-1 (recursion), BM-6 (unbounded test loop). Updated IO-1 printf count (428→212). Changed main.cpp classification from "IVP Test (mixed)" to "Ground (mixed)". Deleted 4 IVP files (2 test scripts, 2 log files). All IVP code preserved in git history.

(`src/main.cpp`, `src/cli/rc_os.cpp`, `src/cli/rc_os.h`, `standards/STANDARDS_DEVIATIONS.md`, `standards/CODING_STANDARDS.md`, `standards/AUDIT_REMEDIATION.md`, `AGENT_WHITEBOARD.md`)

---

### 2026-02-09-005 | Claude Code CLI | refactor

**Clang-tidy P5b-f: identifier naming, bool conversions, init vars, function decomposition**

Completed remaining clang-tidy remediation for all production code. P5b: 162 identifier renames to camelCase. P5c: 41 implicit bool conversions made explicit. P5d: 51 uninitialized variables initialized at declaration. P5e: 9 production functions decomposed under 60-line JSF AV limit. P5f: JSF AV Rule 213 math parentheses check disabled (LL Entry 26). Binary 198,656 bytes (+512 from function call overhead). HW verified: 0 errors across all sensors. All production code now fully remediated — only IVP test code remains deferred.

(18 files across src/, standards/, docs/, .claude/)

---

### 2026-02-09-004 | Claude Code CLI | refactor

**Clang-tidy audit remediation — 5 phases, 911 findings resolved**

P1: Safety-critical fixes (widening multiplication overflow, narrowing conversions, missing default clause) — HW verified 0 errors/117K reads. P2: 354 auto-fixable cosmetic warnings (uppercase suffixes, void args, nullptr, bool literals, using). P3: 275 magic numbers extracted to named constexpr (production code); IVP test code NOLINT'd. P4: 170 missing braces added per JSF AV Rule 59. P5: ~80 C-style casts converted to static_cast/reinterpret_cast per JSF AV Rule 185. Binary size unchanged throughout (198,144 bytes). Remaining ~370 cosmetic findings (naming, parens, implicit bool, uninit vars) documented as deferred in `standards/AUDIT_REMEDIATION.md`.

(all 10 source files, `standards/AUDIT_REMEDIATION.md`)

---

### 2026-02-09-003 | Claude Code CLI | tooling

**Comprehensive clang-tidy standards audit — 127 checks mapped to JSF AV / P10 / JPL C**

Rewrote `.clang-tidy` from 78-line function-size-only config to 376-line comprehensive standards audit config covering bugprone (34), cert (14), cppcoreguidelines (11), google (6), hicpp (3), misc (11), modernize (11), performance (11), readability (26), and clang-analyzer checks. Ran full audit across all 10 source files: 1,251 warnings in our code. Key finding: `misc-no-recursion` caught a recursive call chain that the manual audit missed (P10-1 was marked PASS). 5 safety-critical findings documented. Results in `docs/audits/CLANG_TIDY_AUDIT_2026-02-09.md`.

(`.clang-tidy`, `docs/audits/CLANG_TIDY_AUDIT_2026-02-09.md`)

---

### 2026-02-09-002 | Claude Code CLI | refactor

**Decompose main() and core1_entry() — P10-4 function length compliance**

Extracted `main()` (992→65 lines) into `init_hardware()`, `print_boot_status()`, `init_application()`, and 15 tick-function dispatchers per council-approved plan. Extracted `core1_entry()` (367→15 lines) into `core1_test_dispatcher()`, `core1_spinlock_soak()`, `core1_sensor_loop()`. Added `g_lastTickFunction` watchdog tracking (council recommendation). Binary size unchanged (198,144 bytes). Hardware-verified: all behavior identical.

(`src/main.cpp`)

---

### 2026-02-09-001 | Claude Code CLI | documentation

**Trim tracking documents — collapse completed sections**

Cleaned up PROJECT_STATUS.md (completed stages → summary table), AGENT_WHITEBOARD.md (erased resolved flags, compacted deferred items into bullet list), AUDIT_REMEDIATION.md (line-by-line fix tables → summary). Net -365 lines. No information lost — per-IVP detail remains in IVP.md, audit detail in STANDARDS_AUDIT_2026-02-07.md.

(`docs/PROJECT_STATUS.md`, `AGENT_WHITEBOARD.md`, `standards/AUDIT_REMEDIATION.md`)

---

### 2026-02-08-006 | Claude Code CLI | feature

**IVP-32/33: GPS outdoor validation + CLI integration (Phase 4 complete)**

GPS fix confirmed outdoors (PPS lock, lat/lon accuracy verified). CLI `s` command format polished to match IVP-33 gate wording (`GPS: no fix (N sats)`). BSS layout regression investigated — `alignas(64)` + flag grouping caused new errors. Minimal-change build (`ivp32-33-1`) passes with 68K+ reads, 0 errors. **[CORRECTION 2026-02-09]:** "BSS layout regression" was actually picotool bus corruption from rapid flash cycles (LL Entry 25/27). Disproved — see LL Entry 27.

(`src/main.cpp`)

---

### 2026-02-08-005 | Claude Code CLI | feature

**IVP-31: PA1010D GPS integration on Core 1 with I2C contention fix**

GPS reads at 10Hz on Core 1 via seqlock. Full NMEA parsing (lwGPS), GGA+RMC+GSA sentence filter, Adafruit-style 0x0A padding filter, cold-boot bus recovery. I2C contention (8.4% IMU error rate at gps-10) resolved with 500us post-read settling delay — controlled isolation tests (gps-12a/b/c) proved delay alone is sufficient at 10Hz. Auto-detection: delay only active when I2C GPS at 0x10 is detected; UART GPS skips it. `kCore1ConsecFailMax` lowered from 50 to 10 for GPS-induced burst detection.

*Rationale: 500us delay vs 5Hz rate reduction — delay preserves 10Hz GPS data with only 0.5% CPU overhead. Isolation testing documented in LL Entry 24. ArduPilot never shares GPS with high-rate sensors on I2C; production migrates to UART FeatherWing (Adafruit 3133).*

(`src/main.cpp`, `src/drivers/gps_pa1010d.cpp`, `src/drivers/gps_pa1010d.h`, `.claude/LESSONS_LEARNED.md`)

---

### 2026-02-08-004 | Claude Code CLI | documentation

**Titan doc: H7 board candidates section**

Added Section 13 to `TITAN_BOARD_ANALYSIS.md` — STM32H7 board landscape research. No maker H7 boards exist in Feather/Thing Plus form factor. Recommended Matek H743-Wing V3 flight controller for Titan Path A prototyping (ArduPilot-validated, dual IMU, DPS310, CAN, 13 PWM, ~$55-65).

(`docs/decisions/TITAN_BOARD_ANALYSIS.md`)

---

### 2026-02-08-003 | Claude Code CLI | refactor

**C++20 conversion: .c→.cpp rename + #define→constexpr (PP-1 resolved)**

Two-commit migration of all 9 C source files to C++20. Commit 1: renamed files via `git mv`, fixed C99 compound literals, added `extern "C"` wrap for ruuvi DPS310 library, removed internal `extern "C"` guards from headers, fixed `const` cast in GPS driver. Commit 2: converted 90+ `#define` macros to `constexpr` across all drivers/calibration/CLI — ICM-20948 registers organized into bank-scoped namespaces, calibration/storage constants use `k` prefix. Resolved PP-1 deviation (JSF 29/30/31). Binary size delta: -1404 bytes (97988 vs 99392 baseline). Third-party libs (ruuvi, lwGPS) remain C.

(`src/**/*.cpp`, `src/**/*.h`, `include/rocketchip/config.h`, `CMakeLists.txt`, `standards/STANDARDS_DEVIATIONS.md`)

---

### 2026-02-08-002 | Claude Code CLI | refactor, documentation

**Standards audit remediation — Tier 4: RC_ASSERT, goto elimination, deviation docs**

Completed Phase D of the audit remediation plan. D1: Added `RC_ASSERT()` macro to `config.h` (debug = printf + watchdog spin, release = no-op). D2: Eliminated all 6 `goto` statements in `cmd_accel_6pos_cal()` by extracting body into `cmd_accel_6pos_cal_inner()` helper — wrapper guarantees I2C master pre/post hook execution. D4: Documented bare-metal loop deviations (BM-1 through BM-6), stdio usage (IO-1/IO-2), and preprocessor defines (PP-1) in `STANDARDS_DEVIATIONS.md`. Updated audit dashboard: 220 PASS / 25 PARTIAL/FAIL (90% compliance, up from 82%).

(`include/rocketchip/config.h`, `src/cli/rc_os.c`, `standards/STANDARDS_DEVIATIONS.md`, `standards/AUDIT_REMEDIATION.md`, `standards/STANDARDS_AUDIT_2026-02-07.md`)

---

### 2026-02-08-001 | Claude Code CLI | architecture, council

**F' (F Prime) comprehensive evaluation for Titan**

Expanded the F' addendum in `TITAN_BOARD_ANALYSIS.md` from a brief note into a full 14-section evaluation. Covers: F' architecture and component model, coding standards alignment with JSF AV (near-complete overlap), platform support matrix (RP2350 HSTX is officially supported via fprime-arduino), the critical multicore limitation (Zephyr has no Cortex-M SMP — F' multicore only works on Linux via pthreads), Pi Zero 2 W vs STM32H7 hardware comparison, a hybrid architecture proposal (Pi Zero 2 W running F'/Linux as mission CPU + RP2350 as real-time safety CPU), MAVLink incompatibility (F' uses its own protocol), F' flight heritage (Ingenuity, ASTERIA, RapidScat), and a "cherry-pick" alternative for adopting F' patterns without the framework.

(`docs/decisions/TITAN_BOARD_ANALYSIS.md`)

---

### 2026-02-07-001 | Claude Code CLI | documentation

**New standard: Vendor & OEM Guidelines**

Created `standards/VENDOR_GUIDELINES.md` — centralized reference for vendor-specific constraints, datasheet-sourced values, and OEM recommendations. Consolidates knowledge previously scattered across LL entries, driver comments, IVP notes, and whiteboard flags. Covers ICM-20948 (bank-switching, I2C master race), DPS310 (config, noise spec gaps), PA1010D (255-byte full-buffer reads per vendor app note, bus interference behavior, PMTK commands), RP2350 (errata E2, USB/flash ordering, memory constraints), and Feather board pin assignments. Includes datasheet inventory with gap analysis — DPS310 and PA1010D datasheets are missing locally.

(`standards/VENDOR_GUIDELINES.md`)

---

### 2026-02-06-003 | Claude Code CLI | architecture, documentation

**Stage 4 GPS IVP revision — restructured for dual-core architecture**

Rewrote IVP-31 through IVP-34 (now IVP-31 through IVP-33) after Stage 3 established that Core 1 owns the I2C bus exclusively. Original IVPs assumed GPS could run on Core 0 — this causes bus collisions (LL Entry 20). Key changes: GPS init before Core 1 launch, GPS reads on Core 1 sensor loop (full 255-byte reads at 10Hz per vendor recommendation), seqlock for Core 0 access, old IVP-33 (Core 1 migration) merged into IVP-31 (it's now a prerequisite, not a follow-on). Updated `gps_pa1010d.c` with full-buffer reads and PMTK314 sentence filter. Renumbered IVP-35+ down by 1 to close the gap (now IVP-34 through IVP-71). Reverted partial IVP-31 implementation from main.cpp.

(`docs/IVP.md`, `src/drivers/gps_pa1010d.c`, `src/main.cpp`)

*Rationale: The original Stage 4 IVPs were written before Stage 3's dual-core work revealed that I2C bus access is single-core only (no mutual exclusion in Pico SDK). Attempting Core 0 GPS reads with Core 1 running IMU/baro caused ICM-20948 init failures. Council review confirmed GPS on Core 1 is the only correct approach. The 32-byte read size was an Arduino Wire.h software limitation — vendor app notes and Pico SDK examples both use full 255-byte reads.*

---

### 2026-02-06-002 | Claude Code CLI | architecture, council, documentation

**Stage 3 prep: Seqlock cross-core design — research, council review, IVP corrections**

Created `docs/decisions/SEQLOCK_DESIGN.md` — council-reviewed decision document for cross-core data sharing via seqlock. Four parallel research agents investigated struct layout, prior art (ArduPilot/Betaflight/PX4), RP2350 memory model, and notification mechanisms. Council (3 personas) unanimously approved with 7 required modifications (all incorporated): bounded retry loop, `core1_loop_count`, `mag_read_count`, `_Static_assert` guards, DMB rationale comments, `_Atomic bool cal_reload_pending` signaling. Corrected RP2350 errata reference from E17 to E2 across IVP and whiteboard. Audited SAD Section 4.3 seqlock code — found 5 issues (missing DMB barriers, unnecessary double-buffer, wrong errata ID, nonexistent types, missing timestamps). Updated `AGENT_WHITEBOARD.md` with Stage 3 session plan and research findings.

(`docs/decisions/SEQLOCK_DESIGN.md`, `docs/IVP.md`, `AGENT_WHITEBOARD.md`)

---

### 2026-02-06-001 | Claude Code CLI | feature, bugfix

**IVP-15/16/17/18: Calibration suite + CLI integration — Minimum Viable Demo milestone**

Fixed ICM-20948 returning all zeros after ~150 rapid accel reads during 6-position calibration. Root cause: internal I2C master (for AK09916 mag) races with external reads on shared bank-select register (0x7F). Added `icm20948_set_i2c_master_enable()` API and pre/post calibration hooks to disable I2C master during sampling. Removed motion check and orientation pre-check (ArduPilot doesn't use either — Gauss-Newton solver handles bad data). Reordered positions to QGroundControl standard. Added `i2c_bus_reset()` after flash saves. Version bumped to 0.1.1. HW-verified: 3/3 consecutive runs pass across USB replug cycles.

(`src/drivers/icm20948.c/h`, `src/calibration/calibration_manager.c`, `src/cli/rc_os.c/h`, `src/main.cpp`, `scripts/accel_cal_6pos.py`, `include/rocketchip/config.h`)

---

### 2026-02-05-005 | Claude Code CLI | feature

**IVP-14: Calibration storage (flash persistence)**

Re-enabled calibration system (`calibration_data`, `calibration_manager`, `calibration_storage`) in CMakeLists.txt with `hardware_flash` and `pico_flash` libs. Added storage init before USB (per LL Entry 4/12), manager init after sensors. IVP-14 self-test verifies all 4 gates: load/defaults, save/readback match, power cycle persistence, 10 consecutive saves (wear leveling). Added `kSkipVerifiedGates` flag to skip IVP-10/12/13 at boot — set false to re-run. All gates pass across power cycles.

(`CMakeLists.txt`, `src/main.cpp`)

---

### 2026-02-05-004 | Claude Code CLI | feature

**IVP-13a: I2C bus recovery under fault**

Implemented I2C bus recovery for sensor disconnect/reconnect scenarios. Recovery fires after 50 consecutive errors (~333ms), performs bus reset (deinit + 9-clock bit-bang + STOP + reinit). IMU self-recovers from reads; baro gets lazy reinit after 100 consecutive failures when IMU confirms bus is healthy. Hardware-verified: Qwiic cable disconnect during 100Hz/50Hz polling — no hang, both sensors resume at full rate after reconnect, error counter visible in status output. All 4 IVP-13a gates pass.

(`src/main.cpp`)

---

### 2026-02-05-003 | Claude Code CLI | feature

**IVP-09 through IVP-13: IMU, barometer, and multi-sensor polling**

Re-enabled ICM-20948 and DPS310 drivers incrementally with verification at each gate. IMU: fixed 6 magnetometer init issues, added 3-attempt retry, set ±4g/±500dps. DPS310: ruuvi library with 8Hz/8x oversampling continuous mode. IVP-10 validation (50 IMU samples, magnitude-based gate checks), IVP-12 validation (100 baro samples, pressure/noise/stuck-value gates), IVP-13 multi-sensor polling (IMU 100Hz, baro 50Hz, 60s, zero I2C errors). Measured I2C timing: IMU avg 774us, baro avg 251us. All gates pass across reboot cycles.

(`CMakeLists.txt`, `src/main.cpp`, `src/drivers/icm20948.c`, `docs/PROJECT_STATUS.md`)

---

### 2026-02-05-002 | Claude Code CLI | bugfix, refactor

**Reset to clean Stage 1 baseline, fix I2C bus recovery**

Stripped all Stage 2 code from build (drivers, calibration, CLI) to resolve I2C bus reliability issues. Root cause: two compounding issues — (1) no bus recovery on boot after picotool `--force` reboots left sensors mid-transaction with SDA held low, (2) Stage 2 init code (IMU bank switching, mag I2C master, flash ops) was corrupting bus state. Added `i2c_bus_recover()` call before `i2c_init()` in `i2c_bus_init()`. Restored 400kHz (100kHz was a red herring). All 3 devices (0x69, 0x77, 0x10) now detected reliably. Stage 2 source files remain on disk for incremental re-enablement.

(`CMakeLists.txt`, `src/main.cpp`, `src/drivers/i2c_bus.c`, `src/drivers/i2c_bus.h`, `AGENT_WHITEBOARD.md`)

---

### 2026-02-05-001 | Claude Code CLI | documentation, architecture

**Telstar Booster Pack, docs/hardware/ reorganization, Gemini ELRS section**

Created `docs/hardware/` subdirectory and moved hardware design documents into it (`HARDWARE.md`, `GEMINI_CARRIER_BOARD.md`, `STATUS_INDICATORS.md`) using `git mv` to preserve history. Created `docs/hardware/TELSTAR_BOOSTER_PACK.md` — Telstar Booster Pack design document covering ELRS RC link, CRSF protocol, FPV video transmitter, standalone product potential, FAA Remote ID module support, and updated Booster Pack lineup. Added Section 8.4 (Dedicated ELRS Communications Core) to Gemini doc. Updated all cross-references across repo (README, SAD, SCAFFOLDING, PROJECT_OVERVIEW, HARDWARE, ICDs, SpaceWire-Lite).

---

### 2026-02-04-003 | Claude Code CLI | bugfix

**I2C debug: drop to 100kHz, add verbose scan output**

Cross-referenced current I2C code against working `AP_FreeRTOS` branch. Code is functionally identical (same pins, same instance, same pull-ups) except the working `i2c_scan.c` test used 100kHz, not 400kHz. Changed `I2C_BUS_FREQ_HZ` to 100kHz. Added verbose scan output showing I2C instance number, GPIO pin states, and configured frequency. Build ready, not yet flashed.

(`src/drivers/i2c_bus.c`, `src/drivers/i2c_bus.h`)

---

### 2026-02-04-002 | Claude Code CLI | feature

**IVP Stage 2: RC_OS CLI + Calibration Integration**

Implemented bare-metal RC_OS CLI with calibration integration (IVP-15, IVP-16, IVP-18). CLI provides single-key command interface with calibration menu, sensor status, and I2C rescan capability. Sensor availability checks prevent calibration commands when sensors not initialized. Non-blocking calibration progress monitoring with dots and OK/FAIL output.

**Status:** Calibration logic complete but blocked on I2C issue — see AGENT_WHITEBOARD.md for details.

---

### 2026-02-04-001 | Claude Code CLI | feature, hardware

**IVP Stage 1 Complete: Foundation (IVP-01 through IVP-08)**

Created `src/main.cpp` implementing bare-metal firmware foundation: red LED heartbeat, NeoPixel rainbow via PIO, USB CDC serial with terminal reconnect handling, I2C bus init and scan, structured HW validation output. All gates hardware-verified (ICM-20948, DPS310, PA1010D detected).

Deleted stale files with broken dependencies (`accel_calibrator.c/h`, `debug.h`). Updated CMakeLists.txt to build only Stage 1 sources; other drivers commented with IVP stage markers for incremental re-enablement.

---

### 2026-02-03-007 | Claude Code CLI | tooling

Added plan mode council review instructions to `.claude/CLAUDE.md`. Before ExitPlanMode, agent now asks which council personas to use, spawns a Task agent to run the review, and attaches the verdict to the plan.

---

### 2026-02-03-006 | Claude Code CLI | documentation, architecture

Added Section 8.2 (Dual-IMU Fusion and EKF Lane Switching) to `docs/GEMINI_CARRIER_BOARD.md`. Covers EKF3-style lane switching concept using Gemini's dual independent sensor suites, bandwidth analysis for high-rate bidirectional sensor exchange over SpaceWire-Lite, and protocol implications including new SENSOR_RAW and EKF_HEALTH message types.

---

### 2026-02-03-005 | Claude Code CLI | documentation, architecture, council

**Integration and Verification Plan (IVP) + SAD Updates**

Created `docs/IVP.md` — 64-step development roadmap across 9 stages with pass/fail verification gates. Council review (4 personas) produced 12 findings, all implemented: `⚠️ VALIDATE` convention for numerical values, RP2350 inter-core primitives section (spinlocks, FIFO, doorbells as IVP-21 through IVP-23), GPS as Stage 4 before fusion, I2C bus recovery, MPU details, milestone markers.

SAD.md updates: per-sensor validity flags, seqlock implementation replacing TODO, inter-core primitives table, PIO allocation table, AMP architecture corrections, Section 14 numbering fix. SCAFFOLDING.md and PROJECT_STATUS.md updated with PIO_ALLOCATION.md placeholders.

---

### 2026-02-03-004 | Claude Code CLI | documentation, architecture

Added PIO hardware watchdog design concepts document (`docs/PIO/PIO_WATCHDOG.md`). Covers heartbeat watchdog, dual-core cross-check, and pyro channel lockout using PIO state machines as CPU-independent safety monitors. Concept stage — not committed to IVP.

---

### 2026-02-03-003 | Claude Code CLI | documentation, refactor

**Bare-Metal Pivot — Documentation Cleanup (continued)**

Systematic cleanup of FreeRTOS/ArduPilot references across all documentation.

- LESSONS_LEARNED.md — archived 8 FreeRTOS-specific entries (7-10, 14, 17-19), minor rewording on entries 3, 4, 12, 15
- DEBUG_OUTPUT.md — replaced deferred logging with direct printf macros, renamed per-task to per-module, updated build config table
- CODING_STANDARDS.md — removed RTOS tier table, ArduPilot Library Integration, Dependency Bypassing Policy, HAL Adaptation Policy sections; demoted ArduPilot from "check first" to "useful reference" in Prior Art Research; removed archived LL entry references
- PROJECT_OVERVIEW.md — replaced "RTOS" with "deterministic control loops" in Titan tier, "FreeRTOS" with "Bare-metal Pico SDK" in Technical Foundation
- PROJECT_STATUS.md — added ChibiOS upstream note, added full ArduPilot integration as back burner goal
- ROCKETCHIP_OS.md — replaced FreeRTOS Task Model with bare-metal Execution Model, updated source files table, replaced platform-specific FreeRTOS issues with USB CDC concerns, removed priority-based references
- Deleted GETTING_STARTED.md (thoroughly outdated, not needed until development matures)

---

### 2026-02-03-002 | Claude Code CLI | architecture, refactor

**Pivot from FreeRTOS to Bare-Metal Pico SDK**

Removed all FreeRTOS dependencies, pivoting to bare-metal Pico SDK with polling main loop.

**Deleted:**
- FreeRTOS-Kernel submodule, FreeRTOSConfig.h, FreeRTOS_Kernel_import.cmake
- docs/FreeRTOS/TASK_PRIORITIES.md
- src/main.cpp, src/tasks/sensor_task.c/h, src/debug/debug_stream.c/h (RTOS glue — will be rewritten)

**Edited:**
- CMakeLists.txt — removed FreeRTOS imports, heap link, commented out deleted sources
- config.h — replaced task priorities/stacks with polling timing constants, simplified debug macros to direct printf
- CODING_STANDARDS.md — replaced FreeRTOS platform constraints with bare-metal rules
- SAD.md, PROJECT_STATUS.md, SCAFFOLDING.md, MULTICORE_RULES.md — updated for bare-metal architecture
- DEBUG_PROBE_NOTES.md, SESSION_CHECKLIST.md — minor RTOS reference removal
- ws2812_status.h — comment update

**Deferred for later evaluation (>40% rewrite needed):**
- DEBUG_OUTPUT.md — deferred logging architecture references deleted code
- LESSONS_LEARNED.md — ~50% of entries are FreeRTOS-specific

**Preserved unchanged:** src/calibration/*, src/drivers/* (sensor drivers, calibration, LED — no RTOS dependency)

---

### 2026-02-03-001 | Claude Code CLI | documentation

**Platform Constraints, Multi-Core Rules, Session Checklist, Decisions Folder**

Added critical documentation capturing RP2350 + FreeRTOS SMP platform constraints and session management procedures.

- Added `standards/CODING_STANDARDS.md` "RP2350 + FreeRTOS SMP Platform Constraints" section (~98 lines) - non-negotiable rules derived from LESSONS_LEARNED entries
- Created `docs/MULTICORE_RULES.md` - core assignment rules, cross-core communication, memory barriers
- Created `.claude/SESSION_CHECKLIST.md` - session start/end procedures, handoff protocol
- Created `docs/decisions/` folder structure for council review outputs
- Moved `docs/ESKF/` to `docs/decisions/ESKF/`
- Updated `docs/PROJECT_STATUS.md` to "Reboot: Validate & Rebuild" phase with validation checklist
- Updated `.claude/PROTECTED_FILES.md` with new protected entries
- Updated `.claude/CLAUDE.md` with new @ references

**Files:** CODING_STANDARDS.md, MULTICORE_RULES.md, SESSION_CHECKLIST.md, docs/decisions/README.md, PROJECT_STATUS.md, PROTECTED_FILES.md, CLAUDE.md

---

### 2026-02-02-004 | Claude Code CLI | bugfix, architecture

**FreeRTOS SMP + USB CDC Fix + Calibration Stack Fix**

Fixed USB CDC enumeration failure on RP2350 with FreeRTOS SMP, plus stack overflow during 6-position accel calibration.

**USB CDC Root cause:** Using main FreeRTOS repo instead of Raspberry Pi's fork, plus incorrect FreeRTOSConfig.h settings.

**Calibration Root cause:** Sensor task stack (256 words) too small for ellipsoid fit local matrices. Increased to 512 words.

**Changes:**
- Replaced FreeRTOS-Kernel submodule with raspberrypi/FreeRTOS-Kernel (has RP2350_ARM_NTZ port)
- Rewrote FreeRTOSConfig.h to match pico-examples/freertos/hello_freertos
- Removed custom USB task - SDK's IRQ-based handling is correct approach
- Pinned sensor sampling to Core 1 (Core 0 hosts USB IRQ handlers)
- Simplified CMakeLists.txt USB configuration to use SDK defaults
- Increased SENSOR_TASK_STACK from 256 to 512 words for calibration

**Files:** FreeRTOS-Kernel (submodule), FreeRTOSConfig.h, FreeRTOS_Kernel_import.cmake, src/main.cpp, CMakeLists.txt, src/tasks/sensor_task.c

**Reference:** LESSONS_LEARNED.md Entry 18 (USB), Entry 19 (stack overflow)

**Status:** Calibration runs to completion but ellipsoid fit diverges - needs algorithm tuning.

*Rationale: pico-examples/freertos/hello_freertos is the canonical reference for FreeRTOS on RP2350. Previous attempts used custom configurations that conflicted with SDK's USB stack. The Raspberry Pi FreeRTOS fork contains tested, working SMP support that the main FreeRTOS repo lacks.*

---

### 2026-02-02-003 | Claude Code CLI | documentation

**Documentation Cleanup for Fresh Start**

Reviewed all docs folder files to ensure consistency with bespoke FreeRTOS approach after branch reorganization.

- Updated SAD.md: Rewrote directory structure, module dependencies, driver interfaces, storage architecture, and reset all development phases
- Updated SCAFFOLDING.md: Aligned with SAD.md, reset implementation status
- Updated TOOLCHAIN_VALIDATION.md: Removed stale sensor references, added Phase 2 note
- Updated HARDWARE.md: Removed ArduPilot driver references from IMU and GPS sections
- Updated .claude/CLAUDE.md: Removed reference to deleted RP2350_FULL_AP_PORT.md
- Deleted: docs/RP2350_FULL_AP_PORT.md, docs/AP_HAL_RP2350_PLAN.md (archived in AP_FreeRTOS branch)
- Moved: PROJECT_STATUS.md, PROJECT_OVERVIEW.md to docs/ folder

ESKF docs and ROCKETCHIP_OS.md left as-is (historical reference for design decisions).

---

### 2026-02-02-002 | Nathan (via Claude Opus council review) | architecture

**Sensor Fusion Architecture: ESKF + MMAE**

Changed sensor fusion approach from ArduPilot EKF3 extraction to custom Error-State Kalman Filter (ESKF) with Multiple Model Adaptive Estimation (MMAE) bank for anomaly resilience.

- Added `docs/ESKF/FUSION_ARCHITECTURE_DECISION.md` - full rationale and design
- Added `docs/ESKF/FUSION_ARCHITECTURE.md` - reference document
- Updated SAD.md §5.4 (fusion), §10 (Phase 4), key decisions table, open questions
- Updated SCAFFOLDING.md FusionTask descriptions
- AP_HAL_RP2350 still used for calibration/math/storage - NOT for fusion
- Added `.claude/PROTECTED_FILES.md` - files requiring explicit permission to edit

**Note:** Specific numerical parameters (state counts, filter counts, latencies) pending systematic review before implementation.

---

### 2026-02-02-001 | Claude Code CLI | architecture

**Branch Reorganization - Fresh Start with Bespoke FreeRTOS**

Major pivot: Archived ArduPilot integration attempts, starting fresh with bespoke FreeRTOS approach.

- Created `AP_ChibiOS` branch (archived ChibiOS exploration - blocked by XIP flash issues)
- Created `AP_FreeRTOS` branch (archived FreeRTOS + ArduPilot HAL work - mature but complex)
- Fresh `main` branch for new bespoke implementation

Cherry-picked universal reference documentation (LESSONS_LEARNED, DEBUG_PROBE_NOTES, HARDWARE, SAD, SCAFFOLDING, ROCKETCHIP_OS, coding standards).

**Reference:** Full reorganization plan at `C:\Users\pow-w\.claude\plans\spicy-toasting-breeze.md`

---
