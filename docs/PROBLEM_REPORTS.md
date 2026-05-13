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
| R-3 | Capture-state-then-reset hardfault handler | Audit 2026-05-07 (BM-4) | verified | REMEDIATE | `src/safety/fault_protection.cpp`, `src/safety/crash_record.{h,cpp}`, `src/safety/health_monitor.cpp` | none (Category 2 root) | Awaiting Phase 8 wrap regression for close | — |
| R-4 | MPU-config failure handling | Audit 2026-05-07 (BM-5) | verified | REMEDIATE | `src/safety/fault_protection.cpp` `mpu_setup_stack_guard()` AP=0b10 + MEMFAULTENA | R-3 | Folded into R-3 commit `e4d222a` (surfaced-bug rule); awaiting Phase 8 wrap regression for close | — |
| R-1 | Bound Core 1 boot-wait loop on vehicle | Audit 2026-05-07 (BM-2) | verified | REMEDIATE | `src/core1/sensor_core1.cpp` vehicle-path boot-wait | R-3 | Awaiting Phase 8 wrap regression for close (commit `273a2f9`) | — |
| R-9c | Log-on-change emit in health_monitor.cpp | Audit 2026-05-07 Phase 4.3 | verified | REMEDIATE | `src/safety/health_monitor.cpp` `log_secondary_transitions()` | none | Awaiting Phase 8 wrap regression for close (commit `9a47ba6`) | — |
| R-5 | Full `<stdio.h>` removal from `src/` (replaces R-5a/b/c/d) | Audit 2026-05-07 (IO-1) | deferred | DEFER | All 18 currently-using `src/*.cpp` files; replacement = `rc_log` + ETL per `docs/decisions/STDIO_REPLACEMENT_PLAN.md` | none (gate active now) | Dedicated future session per plan doc | Proliferation prevention is active NOW via `scripts/hooks/pre-commit` Gate 1 + `scripts/hooks/stdio_allowlist.txt` (commit `9f7d924`). No new file can join the allowlist without explicit user-cosigned justification + PR entry. Existing callsites are all behind ground-classification runtime lockouts (CLI / dev / driver-init printfs run pre-arm only) — no flight-time `<stdio.h>` execution observable from the audit. Risk of remaining at status quo: medium (implementation-defined behavior + unbounded format-string surface in pre-arm paths). Risk of partial migration: higher (mixed-pattern codebase). Full migration in dedicated session is the safer path. Re-evaluate at next milestone. |
| R-2 | GPS PMTK `snprintf` → const arrays (absorbed into R-5 dedicated session 2026-05-13) | Audit 2026-05-07 (IO-2) | deferred | DEFER | `src/drivers/gps_pa1010d.cpp`, `gps_uart.cpp` | R-5 (ETL vendoring dependency surfaced during Phase 8 Cat 3 entry) | R-5 dedicated session | Scope-investigation 2026-05-13 surfaced that the const-array refactor handles 8 of 9 `snprintf` callsites cleanly, but `gps_pa1010d_get_debug_status()` writes to a caller-supplied buffer (the MISRA-C 2012 accepted safe subset). Eliminating that one `snprintf` to take `gps_pa1010d.cpp` off the allowlist requires either (a) vendoring ETL now (premature infrastructure work — out of R-2 scope), or (b) hand-rolling a one-off int-formatter (re-inventing what R-5's ETL plan already covers). User direction: bump R-2 onto the R-5 cycle so ETL is vendored once and all `<stdio.h>` sites migrate consistently. PMTK path (8 callsites) becomes the proving ground for the const-array pattern; debug-status (1 callsite) becomes a proving ground for `etl::format`. Council Approach-B + 3 amendments (2026-05-13) carry forward to that session: `constexpr` checksum verification via `static_assert`, byte-on-wire HW gate, CHANGELOG documentation of header-API removal. Council review notes saved for re-use in R-5 dedicated session. Risk: status quo carries `<stdio.h>` for these two files; both files are already on the allowlist (gate active) so no proliferation possible. Both files are flight-critical drivers (R-2's scope was real); migrating them carefully under the unified R-5 plan is safer than partial migration. |
| R-6c | FP-1 template-dispatch | Audit 2026-05-07 | verified | REMEDIATE | `src/calibration/calibration_manager.cpp` `lm_solve()`, new module `src/calibration/lm_solver.{h,cpp}`, new host coverage `test/test_calibration_lm.cpp` | none | Awaiting Phase 8 wrap regression for close. FP-1 moved from Active to Resolved in ACCEPTED_STANDARDS_DEVIATIONS.md. Bench positive control: vehicle flight + dev `bench_sim 2/2 PASS`. Host coverage 788 → 794. | — |
| R-11 | SPIN model: flash_safe_execute protocol | Audit 2026-05-07 Phase 6 council | verified | REMEDIATE | new `tools/spin/rocketchip_flash_protocol.pml` | R-17 (model verifies the corrected protocol) | Awaiting Phase 8 wrap regression for close. 3 LTL properties: `p_core1_locked_during_flash` (SDK contract), `p_no_i2c_during_flash` (R-17 cooperative pause), `p_i2c_reset_after_flash` (R-15 recovery). All PASS, errors: 0, 61-66 states each. Master gate `SPIN_OK_31` (was 28). The model originally surfaced the LL-31 race against pre-R-17 firmware (counterexample matched LL Entry 31 root cause); R-17 closes the gap; R-11 verifies the corrected protocol. | — |
| R-12 | SPIN model: cross-core boot handshake | Audit 2026-05-07 Phase 6 council | verified | REMEDIATE | new `tools/spin/rocketchip_boot.pml` | R-1 (code change) | Awaiting Phase 8 wrap regression for close. Model covers vehicle (bounded wait with timeout) + station/relay (Holzmann exemption) roles non-deterministically. 2 LTL properties: `p_no_premature_sensor_read` (safety, 97 states / 147 transitions / errors: 0), `p_vehicle_core1_eventually_proceeds` (liveness, weak fairness, 164 states / 548 transitions / errors: 0). Master gate `SPIN_OK_28` (was 26). | — |
| P8-FMEA-Pyro | Pyro intent/arm/disarm FMEA sub-check | Audit 2026-05-07 Phase 6 council | analyzed | REMEDIATE | review + log | none | Phase 8 Cat 3 | — |
| R-7 | Holzmann inverted-rule exemption doc | Audit 2026-05-07 | analyzed | REMEDIATE | `standards/CODING_STANDARDS.md` | none | Phase 8 Cat 4 | — |
| R-13 | SESSION_CHECKLIST trigger-map row for SPIN ride-along | Audit 2026-05-07 Phase 6 council | analyzed | REMEDIATE | `.claude/SESSION_CHECKLIST.md` | none | Phase 8 Cat 4 | — |
| R-10b | Stack-usage SDK coverage gap | Audit 2026-05-07 Phase 5 | analyzed | DEFER | CMake `-fstack-usage` global vs runtime watermark | none | Re-eval at next milestone | Tooling completeness gap, not safety gap. Project-side max stack 200 bytes vs 1024 threshold — empirically verified well within margin. Re-evaluate if stack usage grows above 50% of threshold. |
| R-15 | Add `i2c_bus_reset()` after CLI log-flush + erase per LL Entry 31 | Surfaced 2026-05-13 during R-11 prep | verified | REMEDIATE | `src/cli/rc_os_commands.cpp` `cmd_flush_log()`, `cli_do_erase_flights()` | none (precedes R-11) | Awaiting Phase 8 wrap regression for close. Code-state argument confirmed LL 31 still applies: the protocol is documented + observed in code (ao_rcos.cpp:338, the proven callsite); reachable unprotected paths existed at the two CLI handlers; both now match the ao_rcos pattern. R-11 will model the corrected protocol. | — |
| R-17 | Cooperative pause around runtime `flash_safe_execute()` (closes LL-31 race) | Surfaced 2026-05-13 by R-11 SPIN model | verified | REMEDIATE | new `src/safety/core1_i2c_pause.{h,cpp}` + wraps at `ao_rcos.cpp` `cal_save_to_flash()`, `rc_os_commands.cpp` `cmd_flush_log()` + `cli_do_erase_flights()` | none | Awaiting Phase 8 wrap regression for close. R-11 SPIN model surfaced the LL-31 race-class: SDK's `multicore_lockout` halts Core 1's CPU but does NOT drain in-flight DW_apb_i2c transactions; the APB bridge times out the stalled transfer (datasheet §2.1.4, 65535-cycle ceiling), corrupting the peripheral. R-15's post-flash reset was a recovery; R-17 is the prevention. New module extracts general `core1_i2c_pause()` / `core1_i2c_resume()` primitives (the project already had the atomics + Core 1's honor loop; only `cal_pre_hook` had ever used them, and `cal_pre_hook` itself was DEAD CODE — defined but never called from anywhere). All three reachable runtime flash callsites now wrap their flash op chain in pause/resume. Verified: all 4 target tiers compile clean, host ctest 794/794, SPIN_OK_31, bench_sim 2/2 PASS on vehicle flight + dev tiers. | — |
| R-18 | Clean up dead `cal_pre_hook` + `rc_os_cal_pre_hook` / `rc_os_cal_post_hook` function-pointer table | Surfaced 2026-05-13 during R-17 implementation | analyzed | REMEDIATE | `src/calibration/cal_hooks.{h,cpp}`, `src/cli/rc_os.{h,cpp}`, `src/main.cpp` | none | Phase 8 wrap or post-audit cleanup cycle. The `cal_pre_hook()` function in cal_hooks.cpp is defined but never called from anywhere (function-pointer `rc_os_cal_pre_hook` is assigned at `main.cpp:315` but never invoked). The mechanism it implements (Core 1 I2C pause via `g_core1PauseI2C` / `g_core1I2CPaused` atomics) has been extracted to `src/safety/core1_i2c_pause.{h,cpp}` by R-17, making the cal_hooks version dead. Cleanup: delete `cal_pre_hook()` (and possibly `cal_post_hook()`'s no-longer-needed pause-clear, leaving only the `g_calReloadPending` signal); remove the function-pointer table entries. Risk: low (the dead code has no behavior to preserve). | — |
| R-16 | Systematic LL-entry freshness audit (Critical/High entries) | Surfaced 2026-05-13 during R-11 prep — user observation that LL entries can become outdated | analyzed | REMEDIATE | All `.claude/LESSONS_LEARNED.md` Critical/High entries | none | Future post-Phase-8 cycle. Scope: enumerate Critical/High LL entries (~10-15 candidates), characterize current behavior for the failure mode each describes, mark stale ones SUPERSEDED. Reference: LL Entry 25's existing SUPERSEDED 2026-04-22 header sets the precedent. Should be cadence-driven (every milestone close) rather than one-off. | — |

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

---

## Relationship to other docs

- `standards/AUDIT_GUIDANCE.md` Appendix C — defines the order in which PRs are processed during an audit-driven remediation cycle.
- `standards/HW_GATE_DISCIPLINE.md` — defines what a PR's "verified" state requires (positive-control signal, regression-suite credit).
- `.claude/SESSION_CHECKLIST.md` Per-Commit — change impact analysis runs against this file's `Files / mechanism` and `Depends on` columns when a change is in flight.
- `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` — receives PRs that the user explicitly accepts as permanent deviations.
- `AGENT_WHITEBOARD.md` — receives short-lived irregular flags; if a flag becomes a tracked defect, migrate it to this file.

---

*Update only when a PR's state changes or a new PR is recorded. State-of-system protected per `.claude/SESSION_CHECKLIST.md` trigger map.*
