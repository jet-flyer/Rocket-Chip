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
| R-3 | Capture-state-then-reset hardfault handler | Audit 2026-05-07 (BM-4) | analyzed | REMEDIATE | `src/safety/fault_protection.cpp`, preserved-SRAM record format | none (Category 2 root) | Phase 8 Cat 2 next | — |
| R-4 | MPU-config failure handling | Audit 2026-05-07 (BM-5) | analyzed | REMEDIATE | `src/safety/fault_protection.cpp` `mpu_setup_stack_guard()` | R-3 | Phase 8 Cat 3 after R-3 | — |
| R-1 | Bound Core 1 boot-wait loop on vehicle | Audit 2026-05-07 (BM-2) | analyzed | REMEDIATE | `src/core1/sensor_core1.cpp:490` | R-3 | Phase 8 Cat 3 after R-3 | — |
| R-9c | Log-on-change emit in health_monitor.cpp | Audit 2026-05-07 Phase 4.3 | analyzed | REMEDIATE | `src/safety/health_monitor.cpp` (~40-60 lines) | none | Phase 8 Cat 3 | — |
| R-5a | Inventory + classify stdio.h callsites | Audit 2026-05-07 (IO-1) | analyzed | REMEDIATE | scratch table in dated report | none | Phase 8 Cat 3 (precedes R-5b/c/d) | — |
| R-5b | Move dev/diagnostic printfs to src/dev/ | Audit 2026-05-07 (IO-1) | analyzed | REMEDIATE | many files (per-file commits) | R-5a inventory | Phase 8 Cat 3 | — |
| R-5c | Custom bounded writer `rc_write()` | Audit 2026-05-07 (IO-1) | analyzed | REMEDIATE | new `src/util/rc_write.{h,cpp}` | R-5b | Phase 8 Cat 3 | — |
| R-5d | Final cleanup: zero `<stdio.h>` in flight binary | Audit 2026-05-07 (IO-1) | analyzed | REMEDIATE | grep verification | R-5c | Phase 8 Cat 3 | — |
| R-2 | GPS PMTK `snprintf` → const arrays | Audit 2026-05-07 (IO-2) | analyzed | REMEDIATE | `src/drivers/gps_pa1010d.cpp`, `gps_uart.cpp` | none | Phase 8 Cat 3 | — |
| R-6c | FP-1 template-dispatch | Audit 2026-05-07 | analyzed | REMEDIATE | `src/calibration/calibration_manager.cpp` `lm_solve()` | none | Phase 8 Cat 3 | — |
| R-11 | SPIN model: flash_safe_execute protocol | Audit 2026-05-07 Phase 6 council | analyzed | REMEDIATE | new `tools/spin/rocketchip_flash_protocol.pml` | none | Phase 8 Cat 3 | — |
| R-12 | SPIN model: cross-core boot handshake | Audit 2026-05-07 Phase 6 council | analyzed | REMEDIATE | new `tools/spin/rocketchip_boot.pml` | R-1 (code change) | Phase 8 Cat 3 | — |
| P8-FMEA-Pyro | Pyro intent/arm/disarm FMEA sub-check | Audit 2026-05-07 Phase 6 council | analyzed | REMEDIATE | review + log | none | Phase 8 Cat 3 | — |
| R-7 | Holzmann inverted-rule exemption doc | Audit 2026-05-07 | analyzed | REMEDIATE | `standards/CODING_STANDARDS.md` | none | Phase 8 Cat 4 | — |
| R-13 | SESSION_CHECKLIST trigger-map row for SPIN ride-along | Audit 2026-05-07 Phase 6 council | analyzed | REMEDIATE | `.claude/SESSION_CHECKLIST.md` | none | Phase 8 Cat 4 | — |
| R-10b | Stack-usage SDK coverage gap | Audit 2026-05-07 Phase 5 | analyzed | DEFER | CMake `-fstack-usage` global vs runtime watermark | none | Re-eval at next milestone | Tooling completeness gap, not safety gap. Project-side max stack 200 bytes vs 1024 threshold — empirically verified well within margin. Re-evaluate if stack usage grows above 50% of threshold. |

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
