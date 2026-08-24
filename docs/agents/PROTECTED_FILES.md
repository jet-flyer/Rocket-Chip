# Protected Files

**Purpose:** Files that require explicit user permission before editing.

## Rule

**DO NOT edit any file listed below without the user explicitly stating:**
- "Edit [filename]"
- "Update [filename]"
- "Modify [filename]"
- Or similar direct instruction naming the specific file

General requests like "update the docs" or "fix the instructions" do NOT grant permission to edit protected files. Ask for clarification if a task might require editing a protected file.

---

## Protected File List

> **Categories:**
> - **Hard-Protected** — do not edit without explicit per-file permission (the Rule above).
> - **Historical (frozen)** — snapshot / append-only-by-policy records that must not be
>   silently amended or deleted; new entries are new files / new sections, not edits to
>   existing ones. Editing existing content needs explicit permission.
> - **Checklist-cadence** — session-tracking files. Reading or self-checking
>   the checklist is **not** a write grant. If the user **started** a checklist
>   scope (commit, push, wrap/end session, milestone), the writes that belong
>   to that scope are allowed (see `SESSION_CHECKLIST.md` items 7–8, 13).
>
> A file appears under only one category. Anything not listed here is unprotected.

### Hard-Protected

#### Agent Instructions & Behavioral Guidelines
- `AGENTS.md` - Primary agent instruction index (required every session)
- `.claude/CLAUDE.md` - Agent context includes (thin shim to AGENTS.md)
- `standards/AK_GUIDELINES.md` - Behavioral guidelines (Andrej Karpathy rules)
- `docs/agents/PROTECTED_FILES.md` - This file
- `README.md` - Primary human-facing project documentation

#### Standards Documents
- `standards/CODING_STANDARDS.md` - Project coding standards
- `standards/STANDARDS_AUDIT.md` - Comprehensive compliance audit
- `standards/RP2350_ERRATA.md` - RP2350 errata tracker and E2 incident log
- `COUNCIL_PROCESS.md` - Council review protocol

#### Architecture Documents Pending Review
- `docs/decisions/ESKF/*.md` - Fusion architecture (numerical parameters pending systematic review)

#### Architecture Documents
- `docs/SAD.md` - Software Architecture Document
- `docs/IVP.md` - Integration and Verification Plan (64-step development roadmap)
- `docs/MULTICORE_RULES.md` - Multi-core programming rules
- `docs/SCAFFOLDING.md` - Directory structure (update when structure changes, not speculatively)

#### Session Management
- `docs/agents/SESSION_CHECKLIST.md` - Session handoff/end procedures

### Historical (frozen)

Snapshot / append-only records. Existing content is frozen except for rare
corrections; new entries are new files or new sections. Any edit → `ask`.
(See SESSION_CHECKLIST.md "Historical-record docs".)
- `docs/decisions/*` - Council review outputs and architectural decisions (individual files are snapshots — almost never added to or changed)
- `docs/agents/LESSONS_LEARNED.md` - Debugging journal (append-only)
- `docs/audits/*` - Dated, frozen audit reports
- `docs/baselines/*` - Frozen snapshots
- `docs/plans/*` - Per-stage plans (frozen on commit)

### Checklist-cadence

Session-tracking files. Walking `SESSION_CHECKLIST.md` on your own is not
permission to edit them. If the user started Commit, Push, Session End
(wrap), or Milestone, the cadence writes for **that** scope are in play.

- `CHANGELOG.md` — Append-only event log. **Push:** should usually have an
  entry for a significant unit; skip is allowed if you **say so** (no silent
  skip). **Session wrap / end session:** always write an entry unless the
  user says skip. **Handoff** (pick up later): no entry unless the user
  asks. Deleting or amending an old entry → ask. Finishing a sub-task or
  merely reading the checklist does not create an entry.
- `docs/PROJECT_STATUS.md` — Only **milestone item 13** (or a user-named
  edit). A clean Wrap / Push / Handoff does **not** update this file.
  Phase history is going stale vs `IVP.md`; do not refresh it “because
  the sitting ended.”

`AGENT_WHITEBOARD.md` is **not** on this list. Adding a short active row is
ok without a prompt. When a row is done, erase it (IRL rule on that file).
Do not park session essays there.

---

## Adding Files to This List

The user may add files to this list at any time by stating:
- "Add [filename] to protected files"
- "Protect [filename]"

---

## Rationale

Some documents define agent behavior, project standards, or contain values requiring human review before implementation. Unintended modifications could:
1. Change agent behavior in undesirable ways
2. Introduce unapproved standards deviations
3. Lock in numerical parameters before proper analysis
