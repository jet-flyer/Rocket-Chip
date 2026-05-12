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
| _no active PRs yet — the 2026-05-07 audit cycle's PRs will be backfilled into this table as Phase 8 progresses_ | | | | | | | | |

---

## Archived (Closed) Problem Reports

Closed PRs are archived to keep the Active table readable. Archive entry retains: ID, title, origin, file:line of fix, verification gate observed, closure commit SHA, and date.

_(empty — first PRs to close will land here during the 2026-05-07 audit cycle's Phase 8 wrap)_

---

## Relationship to other docs

- `standards/AUDIT_GUIDANCE.md` Appendix C — defines the order in which PRs are processed during an audit-driven remediation cycle.
- `standards/HW_GATE_DISCIPLINE.md` — defines what a PR's "verified" state requires (positive-control signal, regression-suite credit).
- `.claude/SESSION_CHECKLIST.md` Per-Commit — change impact analysis runs against this file's `Files / mechanism` and `Depends on` columns when a change is in flight.
- `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` — receives PRs that the user explicitly accepts as permanent deviations.
- `AGENT_WHITEBOARD.md` — receives short-lived irregular flags; if a flag becomes a tracked defect, migrate it to this file.

---

*Update only when a PR's state changes or a new PR is recorded. State-of-system protected per `.claude/SESSION_CHECKLIST.md` trigger map.*
