# Problem Reports (Project-Wide Finding Tracker)

**Status:** State-of-system. Living document.
**Established:** 2026-05-12 (as part of the 2026-05-07 master standards audit cycle).
**Purpose:** Single project-wide log of open Problem Reports (PRs) — audit findings, bench-debug findings, agent-flagged code defects, and any other discovered issue that warrants tracking through to closure.

---

## Why this file exists

Adapted from the DO-178C "problem report" concept: every discovered issue is recorded, analyzed for impact, tracked through fix and verification, and explicitly closed. Without a single living log, the project's open findings end up scattered across audit reports, whiteboard rows, LL entries, commit messages, and agent memory — making it impossible to answer "what's open" at a glance and easy to forget items that aren't actively in front of someone.

**This file is NOT:**

- `AGENT_WHITEBOARD.md` — that's for short-lived irregular flags between sessions. PRs are tracked items with a defined lifecycle, not whiteboard notes.
- Per-audit `## Remediation` sections — those are audit-scoped and frozen on commit. This file is the project-wide superset that audit cycles draw from and contribute back to.
- `LESSONS_LEARNED.md` — that's the historical record of *how problems were debugged*. This file is the *current open queue* of problems being worked on.
- `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` — that's the log of *user-accepted permanent deviations*. PRs in this file are open work; once accepted as a permanent deviation, a PR row migrates out of this file and into ACCEPTED_STANDARDS_DEVIATIONS.md.

**This file IS:** the central state-of-system list of every open Problem Report the project knows about, regardless of where the PR originated.

---

## PR lifecycle (states)

Every row carries one of the following states. The state column is the most important column — it's how a reader knows whether the row is waiting for analysis, waiting for code work, waiting for verification, or waiting for the close gate.

| State | What it means | What's needed to advance |
|---|---|---|
| `open` | Discovered and recorded. No analysis yet. | Run impact analysis (what files / mechanisms does the fix touch? what other PRs share those? what audit gates does it interact with?). Record edges in the dependency-graph column. |
| `analyzed` | Impact analysis complete. Disposition recorded (REMEDIATE / ACCEPT / DEFER). Ready for sequencing. | Schedule into a remediation cycle per `standards/AUDIT_GUIDANCE.md` Appendix C category ordering. |
| `in-progress` | Code change in flight in a focused commit (or set of commits). | Land the commit(s). Per-commit verification observes its own positive-control signal per `standards/HW_GATE_DISCIPLINE.md` Rule 3. |
| `verified` | Local commit verification passed. The fix does what it claims. | Run the audit-suite regression (or, for non-audit PRs, the project's normal scripted-check suite) at the next category transition. Independent verification (a re-audit or a next-cycle review) is the strongest form. |
| `closed` | Audit-suite regression passed end-to-end. The row signed and locked. | Nothing — row stays in this file for historical reference, or is archived per `## Archived (Closed) Problem Reports` below. |
| `accepted` | Migrated to `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` as a permanent user-signed deviation. | Nothing — row stays here as a stub pointer to the deviation entry. |
| `deferred` | Scheduled for a future cycle. Requires a one-line safety-impact rationale (per `standards/AUDIT_GUIDANCE.md` Step 8). | Re-evaluate on each milestone close; promote to `open` / `analyzed` / `in-progress` when scheduled. |

A PR can move backward only on a halted remediation cycle (per `AUDIT_GUIDANCE.md` Appendix C.7). The recorded state always reflects the current truth.

---

## How a PR is born

Three intake paths:

1. **Audit finding** — a dated audit report's `## Remediation` section names a finding. The audit's Phase 8 wrap copies each non-closed finding into this file as an `open` (or `analyzed`, if the audit already did impact analysis) PR row with a back-reference to the dated audit report.
2. **Bench-debug finding** — a debug session surfaces an issue that isn't fixed in-session. The session-end note adds an `open` PR row here rather than (or in addition to) writing to the whiteboard.
3. **Agent-flagged defect** — an agent reading code notices something. If the issue is non-trivial and the agent isn't fixing it in the current task, it gets logged here as `open`.

The whiteboard rule still applies: if it's a short-lived irregular flag (a one-paragraph note about a specific recent observation), `AGENT_WHITEBOARD.md` is correct. If it's a tracked defect that will work through the lifecycle, this file is correct.

---

## How a PR is closed

A PR is `closed` when:

1. The fix landed in a focused commit (state was `in-progress`, advanced to `verified`).
2. The audit-suite regression (or equivalent scripted-check pass) ran cleanly post-fix (state was `verified`, advanced to `closed`).
3. The row is signed in the dated audit report's `## Remediation` section if the PR was audit-originated, OR in a focused commit if the PR was non-audit-originated.

For audit-originated PRs, "closed" maps onto the dated audit report's row also being signed. For non-audit PRs, "closed" is recorded here directly with the commit SHA that completed the verification gate.

---

## How a PR is accepted (becomes a permanent deviation)

When the user explicitly signs off on accepting a PR as a permanent deviation rather than fixing it, the row moves out of this file and into `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` Active section. A stub row stays here pointing at the deviation entry, with state `accepted`.

This is the only path by which a PR leaves this file other than `closed`.

---

## Active PRs

Each row: `ID`, `Title`, `Origin`, `State`, `Disposition`, `Files / mechanism`, `Depends on`, `Owner / next action`, `Safety-impact note (required for DEFER)`.

Inline notes after each table entry are fine when the row needs more context than the columns can hold; large discussions belong in the dated audit report or a focused decision doc instead.

| ID | Title | Origin | State | Disposition | Files / mechanism | Depends on | Owner / next action | Safety-impact (if DEFER) |
|----|-------|--------|-------|-------------|-------------------|------------|---------------------|--------------------------|
| R-5 | Full `<stdio.h>` removal from `src/` (replaces R-5a/b/c/d) | Audit 2026-05-07 (IO-1) | deferred | DEFER | All 18 currently-using `src/*.cpp` files; replacement = `rc_log` + ETL per `docs/decisions/STDIO_REPLACEMENT_PLAN.md` | none (gate active now) | Dedicated future session per plan doc | Proliferation prevention is active NOW via `scripts/hooks/pre-commit` Gate 1 + `scripts/hooks/stdio_allowlist.txt` (commit `9f7d924`). No new file can join the allowlist without explicit user-cosigned justification + PR entry. Existing callsites are all behind ground-classification runtime lockouts (CLI / dev / driver-init printfs run pre-arm only) — no flight-time `<stdio.h>` execution observable from the audit. Risk of remaining at status quo: medium (implementation-defined behavior + unbounded format-string surface in pre-arm paths). Risk of partial migration: higher (mixed-pattern codebase). Full migration in dedicated session is the safer path. Re-evaluate at next milestone. |
| R-2 | GPS PMTK `snprintf` → const arrays (absorbed into R-5 dedicated session 2026-05-13) | Audit 2026-05-07 (IO-2) | deferred | DEFER | `src/drivers/gps_pa1010d.cpp`, `gps_uart.cpp` | R-5 (ETL vendoring dependency surfaced during Phase 8 Cat 3 entry) | R-5 dedicated session | Scope-investigation 2026-05-13 surfaced that the const-array refactor handles 8 of 9 `snprintf` callsites cleanly, but `gps_pa1010d_get_debug_status()` writes to a caller-supplied buffer (the MISRA-C 2012 accepted safe subset). Eliminating that one `snprintf` to take `gps_pa1010d.cpp` off the allowlist requires either (a) vendoring ETL now (premature infrastructure work — out of R-2 scope), or (b) hand-rolling a one-off int-formatter (re-inventing what R-5's ETL plan already covers). User direction: bump R-2 onto the R-5 cycle so ETL is vendored once and all `<stdio.h>` sites migrate consistently. PMTK path (8 callsites) becomes the proving ground for the const-array pattern; debug-status (1 callsite) becomes a proving ground for `etl::format`. Council Approach-B + 3 amendments (2026-05-13) carry forward to that session: `constexpr` checksum verification via `static_assert`, byte-on-wire HW gate, CHANGELOG documentation of header-API removal. Council review notes saved for re-use in R-5 dedicated session. Risk: status quo carries `<stdio.h>` for these two files; both files are already on the allowlist (gate active) so no proliferation possible. Both files are flight-critical drivers (R-2's scope was real); migrating them carefully under the unified R-5 plan is safer than partial migration. |
| R-10b | Stack-usage SDK coverage gap | Audit 2026-05-07 Phase 5 | analyzed | DEFER | CMake `-fstack-usage` global vs runtime watermark | none | Re-eval at next milestone | Tooling completeness gap, not safety gap. Project-side max stack 200 bytes vs 1024 threshold — empirically verified well within margin. Re-evaluate if stack usage grows above 50% of threshold. **2026-05-13 DC-cycle review (C4-3):** decision confirmed — defer to next milestone close. No new stack-usage evidence in the 6 days since R-10b was opened (2026-05-07). Project-side max stack reading unchanged at ~200 bytes vs 1024 threshold (20% utilization, far below the 50%-of-threshold re-eval trigger). SDK side (the `-fstack-usage` coverage gap itself) is unchanged — it's an upstream tooling matter. Next re-eval lands at next master audit cycle's Tier 5.2 stack-deep-review walk. No code change required this cycle. |
| R-20 | `kCrashReasonCore1BootWait` AIRCR path is half-broken on RP2350 — Core 1 calling AIRCR resets only Core 1, not Core 0 | Audit 2026-05-07 Phase 8L2 — surfaced by user "warm reboot" question | analyzed | DEFER | `src/core1/sensor_core1.cpp:520-524` → `src/safety/crash_record.cpp:46` | R-3 (introduced the path), R-21 (related architecture) | DEFER to in-flight fault recovery architecture session (`AGENT_WHITEBOARD.md` "In-flight fault recovery architecture"). Three options to evaluate: (A) doorbell + Core 0 wakeup-and-reset, (B) POWMAN chip-level reset (verify SRAM survival), (C) downgrade to log+halt with status LED. R-20 is one of the "what IS the right reset primitive" inputs to that session. |
| R-21 | No automatic chip-reset on PIO watchdog timeout — `Q_onError()` halts forever | Audit 2026-05-07 Phase 8L2 — surfaced by user "warm reboot" question | analyzed | DEFER | `src/main.cpp:391-393` (per IVP-90 SDK watchdog removal); `src/safety/fault_protection.cpp` `Q_onError()` | R-19 (R-19 may unblock SDK watchdog re-evaluation); R-20 (related architecture) | DEFER to in-flight fault recovery architecture session. Investigate whether SDK hardware watchdog can be re-enabled now that R-19 fixes the post-watchdog-reset Core 1 wedge. If IVP-90 was specifically about the SIO_FIFO_IRQ wedge, R-19 should close that; if it was about a different RP2350 watchdog issue (E13/E15 errata?), retain the IVP-90 decision. |
| R-22 | Add warm-reboot audit script (G-W1 AIRCR via probe + G-W2/G-W3/G-W4 picotool variants) | Audit 2026-05-07 Phase 8L2 — proposed by L2's first-time exercise of these gates | analyzed | DEFER | new `scripts/warm_reboot_audit.py` | R-25-exec (tier model depends on bench-tier deprecation outcome) | DEFER to R-25-exec session. **Scope-change 2026-05-13 (DC-cycle Session 2 mid-execution):** R-22 was originally Session 2's primary item. Four iterations of the script wedged on Windows USB CDC edge cases (blind-sleep re-enum loops, then chip-serial plumbing bugs, then peek_banner integration via the council redesign). The redesign's hard-gate positive-control for G-W1/G-W4 (Core-1 IMU/baro read-counter increment) requires the q-Debug submenu, which is **bench-tier-only** (`ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS` gate per `src/dev/dev_cli.h:9-19`). Flight-tier firmware (what's flashed on the bench today) has no equivalent Core-1 liveness signal accessible via main-menu CLI. Since R-25-exec will decide bench-tier's fate (deprecation under approach B will remove the Debug submenu entirely), shipping R-22 against the soon-to-be-deprecated tier is churn. R-22 merges into R-25-exec so it's built once against the final tier model — banner-only gates if approach B (no bench tier), or full counter-increment gates if approach C (bench tier retained). The 4-iteration redesign + council review + research memos all preserved at `C:\\Users\\pow-w\\.claude\\plans\\snoopy-wibbling-noodle.md`; R-25-exec session inherits that work. Safety-impact: zero — R-22 is audit infrastructure; runtime behavior unaffected. |
| R-23 | Vehicle bench tier (`build/rocketchip-dev.elf`) HardFaults on cold boot — `INVPC` UsageFault → forced HardFault | Audit 2026-05-07 Phase 8L2 — first-ever attempt to exercise Phase 4 against bench tier in-cycle | analyzed | DEFER | `build/rocketchip-dev.elf` (NOT_CERTIFIED_FOR_FLIGHT=ON, vehicle-board) | R-25 (subsumes — if bench tier is deprecated, R-23 may not need fixing) | DEFER to dedicated debug session OR rolled into R-25 deprecation evaluation. CFSR=0x100 (UFSR.INVPC = invalid EXC_RETURN), HFSR=0x40000000 (forced from UsageFault escalation because UsageFault is not enabled). Independent of R-19 (verified by reverting R-19 — bench tier still crashes). Station bench tier with same R-19 boots cleanly — narrows R-23 to vehicle-bench-tier-specific. **Blocks Phase 4 vehicle scenarios + replay_gate_test in this audit cycle.** Pre-existing — not caught earlier because original Phase 1 ran against flight tier (whatever was on COM7); 4-tier build parity gate only checks compilation, not boot. |
| R-24 | Extend 4-tier build parity gate to include flash + verify-banner per tier | Audit 2026-05-07 Phase 8L2 — surfaced as L2-P1 audit-policy finding (R-23 would have been caught if this existed) | analyzed | DEFER | new `verify_boot_parity.sh` (or extend `verify_build_parity.sh`); requires probe + COM port for each tier | R-25-exec (tier count depends on bench-tier deprecation outcome) | DEFER to R-25-exec session. **Scope-change 2026-05-13 (DC-cycle Session 2 mid-execution):** R-24 was originally bundled with R-22 in Session 2 of this cleanup cycle. Mid-Session 2, the question came up: dev tiers are headed for deprecation in R-25-eval / R-25-exec under user-preferred approach B. Shipping a 4-tier boot-parity gate now and then re-scoping it to 2 tiers in R-25-exec is churn. R-24 merges into R-25-exec so it gets built once with the right tier count. Safety-impact: zero — boot-parity is audit-time confidence; runtime behavior unaffected. The known R-23 INVPC HardFault on vehicle dev tier is the existence proof that the gap exists today; R-25-exec closes both R-23 and R-24 together. |
| R-25 | Evaluate deprecating bench tier in favor of patch-based testing | Audit 2026-05-07 Phase 8L2 — user observation: "if it's not maintained and just an amalgamation of single-use testing features then it should be deprecated" | analyzed | DEFER | All `src/dev/*.cpp` modules + 40 `#ifdef ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS` gates across `src/`; 12 host-side scripts depend on bench-tier symbols (3 recurring: `enhanced_fault_injection.py`, `replay_gate_test.py`, `station_bench_sim.py`; 9 stage-archived) | R-23 (subsumed — if bench tier is removed, R-23 doesn't need fixing) | DEFER to dedicated bench-tier-deprecation evaluation session. Three options: (A) flight-tier runtime test-mode flag, (B) patch-based testing (user preferred direction), (C) tightened bench tier. R-23 prerequisite if A or C chosen. Inputs: inventory of 23 distinct citations (already done in audit § L2.6f), per-script migration plan, replay_inject.cpp's fate (the largest surface), cleanup plan for the 9 archived host scripts. |
| L2-P5 | AUDIT_GUIDANCE.md Step 2 (Coding Standards Deep Compliance) was only partially executed in the master audit. Yesterday's audit Phase 2.3 walked the 10 P10 rules with status; the **221 JSF AV rules + JPL C LOC-1..4 + project-specific + agent-behavioral** were NOT walked. Step 2 calls for a full `STANDARDS_AUDIT.md` template walk producing a dated `STANDARDS_AUDIT_YYYY-MM-DD.md` companion file; yesterday substituted the partial P10 walk. | Audit 2026-05-07 Phase 8L2 — surfaced at L2 close by user "look up other audits" inventory | analyzed | DEFER | new dated `STANDARDS_AUDIT_YYYY-MM-DD.md` companion file walking the un-walked rule sets; tool sweeps caught the function-size / complexity / format-string / dead-code subset but missed header discipline, naming conventions, parameter validation, return-value checking, single-exit, no-recursion, etc. | none (rule-set didn't change since 2026-02-07; needs fresh PASS/PARTIAL/FAIL per file) | DEFER to **focused audit-coverage-catchup cycle**. Most rows expected to PASS because intervening remediation kept the project current, but L2 cannot claim that without actually walking the rules. Estimated 2-4 hours of focused reading. |
| L2-P6 | DEV_CODE re-audit (`#ifndef BUILD_FOR_FLIGHT` exhaustive inventory) not re-run since IVP-127a | Audit 2026-05-07 Phase 8L2 — surfaced at L2 close | analyzed | DEFER | All `src/dev/` + 40 `#ifdef ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS` gates | R-25 (subsumes — if bench tier deprecated, this re-audit may not need running) | DEFER to Phase 8L3 catchup cycle OR rolled into R-25 evaluation. |
| L2-P7 | VERSION_STRING re-audit (single-source-of-truth print-site inventory) not re-run since IVP-127b | Audit 2026-05-07 Phase 8L2 — surfaced at L2 close | analyzed | DEFER | All version-string print sites; `__version__`, `kBuildTag`, banner constants | none | DEFER to Phase 8L3 catchup cycle. Small surface — ~15 minutes. |
| L2-P8 | AO_COMMANDMENTS re-audit (Commandment IV "no blocking" + observations) not re-run since 2026-04-27 | Audit 2026-05-07 Phase 8L2 — surfaced at L2 close | analyzed | DEFER | All AO event handlers in `src/active_objects/` | none | DEFER to Phase 8L3 catchup cycle. ~30 minutes. |
| L2-P9 | TOOLCHAIN_VERSION re-audit (per-component cmake/clang/ninja/etc. version + watch mechanism) not re-run since 2026-04-27 | Audit 2026-05-07 Phase 8L2 — surfaced at L2 close | analyzed | DEFER | clang-tidy, lizard, cppcheck, spin, gcc, cmake, ninja, picotool versions | none | DEFER to Phase 8L3 catchup cycle. ~15 minutes. |
| L2-P10 | CLA-RBM (Computational Load Analysis + Runtime Behavior Map) not re-run since 2026-03-08 | Audit 2026-05-07 Phase 8L2 — post-session-close check of `docs/audits/` recursively found `cla_rbm/` sub-folder that L2-P5..P9 inventory missed | analyzed | DEFER | `docs/audits/cla_rbm/CLA_RBM_PLAN.md`, `COMPUTATIONAL_LOAD_ANALYSIS.md`, `RUNTIME_BEHAVIOR_MAP.md`, `cla_2026-03-08.md`, `cla_2026-03-08_5min.md` | none (could surface architectural items if AO topology changed materially since 2026-03-08, which it has — Stages L/T/16C added new AOs and changed Core 0 idle-bridge behavior) | DEFER to audit-coverage-catchup cycle. RBM in particular is worth re-validating — Stage 16C added `station_idle_tick`, Stage T added RadioScheduler, and the AO_HealthMonitor split happened mid-cycle. |
| L2-P11 | Inventory-completeness check itself was not exhaustive — L2-P5..P10 found by spot checks only; the meta-question "are there more gaps?" is unproven | User direction at session close: "possibility of more" | analyzed | DEFER | All `docs/` for audit-shaped artifacts; all `AUDIT_GUIDANCE.md` step + appendix sub-items; all `CODING_STANDARDS.md` listed-but-unaudited standards (e.g., MISRA C); all stage plans + decision docs + IVP entries that may have run their own checks; pre-commit-hook coverage matrix | none | DEFER to start of audit-coverage-catchup cycle as **step 0** (inventory completeness check) before the gap-fill walk. WB note has the full sub-checklist. The L2-P10 finding (CLA-RBM missed in initial inventory) is the existence proof that L2-P11 is real. |

---

## Archived (Closed) Problem Reports

Closed PRs are archived to keep the Active table readable. Archive entry retains: ID, title, origin, file:line of fix, verification gate observed, closure commit SHA, and date.

**Phase 8 Category 1 — Gate Integrity (closed 2026-05-12):**

| ID | Title | Fix location | Verification | Commit | Date |
|----|-------|--------------|--------------|--------|------|
| P8-SPIN-B | Delete `rocketchip_fd_pp.pml` zero-byte stub | `tools/spin/rocketchip_fd_pp.pml` (was untracked) | `ls tools/spin/*.pml` shows only the 4 active models | (no commit — file was untracked; documented in P8-SPIN-A commit) | 2026-05-12 |
| P8-SPIN-A | Extend SPIN gate to all 4 active models | `tools/spin/run_stage_o_ao_spin.sh` | `SPIN_OK_26` across 4 models (26 LTL properties total); log: `logs/audit-2026-05-07/08_spin_all_4_models.log` | `86b07ca` | 2026-05-12 |
| R-10a | Fix `analyze_stack_usage.sh` parser bug | `scripts/analyze_stack_usage.sh` | Self-test PASS; real `build_stack/` reports `5 KiB` (was `0 KiB`); log: `logs/audit-2026-05-07/09_stack_usage_post_R10a.log` | `3551231` | 2026-05-12 |
| R-9a | Add `fault_force_launch_abort()` | `src/dev/fault_inject.cpp`, `CMakeLists.txt` | `enhanced_fault_injection.py --scenario launch-abort`: `SCENARIO_LAUNCH-ABORT_COMPLETE`, observed `[FD] ABORT*` | `678f82f` | 2026-05-12 |
| R-9b | Add `fault_force_radio_dropout()` + test-only RfManager setter | `src/dev/fault_inject.cpp`, `src/active_objects/ao_rf_manager.{h,cpp}`, `CMakeLists.txt` | `enhanced_fault_injection.py --scenario radio-dropout`: `SCENARIO_RADIO-DROPOUT_COMPLETE`, observed `link lost` | `1fab314` | 2026-05-12 |

**Category 1 → 2 transition gate signed 2026-05-12:** SPIN `SPIN_OK_26`, stack analyzer reports honest `5 KiB`, host ctest 788/788 PASS (gated by pre-commit hook on `1fab314`), launch-abort + radio-dropout HW scenarios observe firmware-side positive-control signals. Category 2 (R-3) ready to begin.

**Phase 8 Categories 2 + 3 + Cycle Cleanup — closed via Grok independent verification (2026-05-13):**

The 16 rows below all advanced from `verified` to `closed` via the same audit-suite-regression event: Grok's Tier 5/6/7 independent walk of the 2026-05-13 master audit cycle (merge commit `1a32103` — fresh-session, structurally-different LLM, DO-178C level-3 independence credit per `HW_GATE_DISCIPLINE.md` Rule 6). Grok's Tier 7 disposition section explicitly re-verified the 4 cycle-originated findings and confirmed the prior 2026-05-07 cycle's REMEDIATE rows by transitivity through Tier 5.6 regression-on-closed-findings.

These rows formally close in this cleanup-cycle commit (DC-2026-05-13 C4-1), per Appendix C.5 — the chronology is "Grok regression THEN bookkeeping" rather than the strict-letter "cleanup-cycle commits THEN regression" because all 16 rows were code-clean before the audit cycle started; the cleanup commit is pure bookkeeping (PROBLEM_REPORTS.md row-state advance) with zero risk of invalidating Grok's verification.

| ID | Title | Fix location | Verification | Commit | Date |
|----|-------|--------------|--------------|--------|------|
| R-1 | Bound Core 1 boot-wait loop on vehicle | `src/core1/sensor_core1.cpp` vehicle-path boot-wait | L1 commit + Grok Tier 5/6/7 PASS (1a32103); related to R-3 Category 2 root | `273a2f9` | 2026-05-13 |
| R-3 | Capture-state-then-reset hardfault handler | `src/safety/fault_protection.cpp`, `src/safety/crash_record.{h,cpp}`, `src/safety/health_monitor.cpp` | Category 2 root. Surfaced + folded in R-4 (MPU AP=0b10 + MEMFAULTENA). Grok Tier 5/6/7 PASS (1a32103) | `e4d222a` | 2026-05-13 |
| R-4 | MPU-config failure handling (AP=0b10 + MEMFAULTENA) | `src/safety/fault_protection.cpp` `mpu_setup_stack_guard()` | Folded into R-3 per surfaced-bug rule. Grok Tier 5/6/7 PASS (1a32103) | `e4d222a` (with R-3) | 2026-05-13 |
| R-6c | FP-1 template-dispatch (function-pointer deviation retired) | `src/calibration/calibration_manager.cpp`, new module `src/calibration/lm_solver.{h,cpp}`, host coverage `test/test_calibration_lm.cpp` | bench_sim 2/2 PASS, host coverage 788 → 794. FP-1 moved Active → Resolved in `ACCEPTED_STANDARDS_DEVIATIONS.md`. Grok Tier 5/6/7 PASS (1a32103) | (commits in 2026-05-13 cycle; see audit report § R-6c) | 2026-05-13 |
| R-7 | Holzmann inverted-rule exemption doc cross-reference | `standards/CODING_STANDARDS.md` ↔ `ACCEPTED_STANDARDS_DEVIATIONS.md` | Doc-only edit verified. Grok Tier 5/6/7 PASS (1a32103) | (commit in 2026-05-13 cycle) | 2026-05-13 |
| R-9c | Log-on-change emit in health_monitor.cpp | `src/safety/health_monitor.cpp` `log_secondary_transitions()` | L1 verified + Grok Tier 5.4 PASS (1a32103) | `9a47ba6` | 2026-05-13 |
| R-11 | SPIN model: flash_safe_execute protocol | new `tools/spin/rocketchip_flash_protocol.pml` | 3 LTL properties PASS (61-66 states each, 0 errors). Master gate `SPIN_OK_31`. Grok Tier 5/6/7 PASS (1a32103) | (commit in 2026-05-13 cycle) | 2026-05-13 |
| R-12 | SPIN model: cross-core boot handshake | new `tools/spin/rocketchip_boot.pml` | 2 LTL properties PASS (safety + liveness with weak fairness, errors: 0). Master gate `SPIN_OK_28` → `SPIN_OK_31`. Grok Tier 5/6/7 PASS (1a32103) | (commit in 2026-05-13 cycle) | 2026-05-13 |
| R-13 | SESSION_CHECKLIST trigger-map row for SPIN ride-along | `.claude/SESSION_CHECKLIST.md` | Doc-only edit verified. Master gate auto-discovers LTL properties for new claims. Grok Tier 6 PASS (1a32103) | (commit in 2026-05-13 cycle) | 2026-05-13 |
| R-15 | `i2c_bus_reset()` after CLI log-flush + erase (LL Entry 31 application) | `src/cli/rc_os_commands.cpp` `cmd_flush_log()` + `cli_do_erase_flights()` | Code edit verified; matches the ao_rcos.cpp:338 proven pattern. R-11 SPIN model verifies the corrected protocol. Grok Tier 5/6/7 PASS (1a32103) | (commit in 2026-05-13 cycle) | 2026-05-13 |
| R-16 | Systematic LL-entry freshness audit (Critical/High entries) | `.claude/LESSONS_LEARNED.md` Critical/High entries | 23 entries audited 2026-05-13; zero stale entries detected. Recommendation captured: milestone-close discipline. Grok Tier 4.6 PASS (1a32103) | (commit in 2026-05-13 cycle) | 2026-05-13 |
| R-17 | Cooperative pause around runtime `flash_safe_execute()` (closes LL-31 race) | new `src/safety/core1_i2c_pause.{h,cpp}`; wraps at `ao_rcos.cpp` `cal_save_to_flash()`, `rc_os_commands.cpp` `cmd_flush_log()` + `cli_do_erase_flights()` | All 4 target tiers compile clean; host ctest 794/794; SPIN_OK_31; bench_sim 2/2 PASS vehicle flight + dev tiers. R-11 SPIN model surfaced the race-class; R-17 prevents it. Grok Tier 5/6/7 PASS (1a32103) | (commit in 2026-05-13 cycle) | 2026-05-13 |
| R-18 | Clean up dead `cal_pre_hook` + function-pointer table | `src/calibration/cal_hooks.{h,cpp}`, `src/cli/rc_os.{h,cpp}`, `src/main.cpp` | All 4 target tiers compile clean; host ctest 794/794. Net delta: −31 lines. Grok Tier 5/6/7 PASS (1a32103) | (commit in 2026-05-13 cycle) | 2026-05-13 |
| R-19 | Unconditional `multicore_reset_core1()` before `multicore_launch_core1()` (post-AIRCR SIO_FIFO_IRQ wedge fix) | `src/main.cpp` `init_hardware()` (4-line addition); `docs/decisions/CORE1_PSM_RESET_BEFORE_LAUNCH.md` (new) | L1 verified: 4-tier build clean, AIRCR-driven probe test PASS, 3-power-cycle baseline + 5x rapid picotool warm-reboot all PASS. Diagnosis: AIRCR per-processor on RP2350; PSM toggles the NVIC pending. Grok Tier 5/6/7 PASS (1a32103) | (commit in 2026-05-13 cycle) | 2026-05-13 |
| R-26 | Fix RP2350 datasheet citation §1.4.3 → §14.9.1 in `health_monitor.{h,cpp}` | `src/safety/health_monitor.h:183`, `src/safety/health_monitor.cpp:463-464` (2-line comment edit) | Comment-only edit, no rebuild required. Surfaced by L2 Phase 7 exhaustive citation walk (motivated the L2-P2 exhaustive-coverage rule). Grok Tier 5/6/7 PASS (1a32103) | (commit in 2026-05-13 cycle) | 2026-05-13 |
| P8-FMEA-Pyro | Pyro intent/arm/disarm FMEA sub-check (no code fix needed) | review + log | F1–F9 multi-transition failure-mode review 2026-05-13: all mitigated by HSM run-to-completion + state ownership. HSM-level coverage already provided by `tools/spin/rocketchip_fd.pml` (8 LTL properties). R-14 SPIN unified-pyro model NOT queued — not justified given existing coverage + single-actor pyro hardware. Grok Tier 5/6/7 PASS (1a32103) | (review-only; see audit report § 8.P8-FMEA-Pyro) | 2026-05-13 |
| R-27 | `docs/ROCKETCHIP_OS.md` main-menu key table stale (claimed `s`/`e`/`b` as Status keys + `i` as Radio key; Stage L 2026-04-18 reassigned `b` to Beacon and moved `s`/`e`/`b`/`i` to the `q`-Debug submenu; doc never updated) | `docs/ROCKETCHIP_OS.md` Main Menu table + new Debug Sub-Menu section | Doc edit verified against firmware `print_help_menu()` at `src/cli/rc_os.cpp:94-98` (the live runtime-emitted help text is the source of truth) + `handle_main_menu()` at `src/cli/rc_os.cpp:143-191` + `dev_debug_menu_dispatch()` at `src/dev/dev_cli.cpp:92-178`. Surfaced 2026-05-13 during R-22 (warm-reboot audit) implementation when my script tried to send `b` from main menu expecting Hardware Status and got Beacon activation instead. Same drift class as R-26 (datasheet citation rot). | (this commit, DC-2026-05-13 Session 2) | 2026-05-13 |

**DC-2026-05-13 Session 1 C4-1 archive sign-off (this commit):** 16 rows advanced from `verified` to `closed`. Verification basis: Grok Tier 5/6/7 independent walk (merge `1a32103`) ran the audit-suite regression — host ctest, SPIN, bench_sim 2/2 — and confirmed every Tier 1-4 finding by transitivity (Tier 5.6 regression-on-closed-findings). End-of-session sanity check (host ctest 794/794 + bench_sim 2/2 PASS + SPIN_OK_31) on the cleanup-cycle commits confirms no bookkeeping-side regression.

---

## Relationship to other docs

- `standards/AUDIT_GUIDANCE.md` Appendix C — defines the order in which PRs are processed during an audit-driven remediation cycle.
- `standards/HW_GATE_DISCIPLINE.md` — defines what a PR's "verified" state requires (positive-control signal, regression-suite credit).
- `.claude/SESSION_CHECKLIST.md` Per-Commit — change impact analysis runs against this file's `Files / mechanism` and `Depends on` columns when a change is in flight.
- `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` — receives PRs that the user explicitly accepts as permanent deviations.
- `AGENT_WHITEBOARD.md` — receives short-lived irregular flags; if a flag becomes a tracked defect, migrate it to this file.

---

*Update only when a PR's state changes or a new PR is recorded. State-of-system protected per `.claude/SESSION_CHECKLIST.md` trigger map.*
