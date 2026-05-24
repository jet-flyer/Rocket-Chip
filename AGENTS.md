# AGENTS.md — RocketChip Agent Instructions

## Behavioral Guidelines

@standards/AK_GUIDELINES.md

---

## Session Discipline — Required on Every Session

@docs/agents/SESSION_CHECKLIST.md

---

## Protected Files & Permission Rules

@docs/agents/PROTECTED_FILES.md

---

## Read First (one-time on new context or major changes)

- **docs/SAD.md** — Software Architecture Document (system design, modules, interfaces, task model)
- **docs/SCAFFOLDING.md** — Directory structure and implementation status
- **standards/CODING_STANDARDS.md** — Coding standards, communication protocols, data formats, safety requirements
- **standards/DEBUG_OUTPUT.md** — Debug output conventions and macros
- **standards/HW_GATE_DISCIPLINE.md** — HW gate rules (positive-control signal, 3-boot reseat, commit citation)
- **docs/hardware/HARDWARE.md** — Prototype hardware, GPIO usage, I2C conflicts, product naming
- **COUNCIL_PROCESS.md** — How to run panel reviews and persona behavior
- **CROSS_AGENT_REVIEW.md** — Protocol for flagging concerns when multiple agents work on the codebase

## Each Session (check at the start of every session)

- **PROJECT_STATUS.md** — Current phase, blockers, planned work
- **CHANGELOG.md** — When, who, what, why
- **AGENT_WHITEBOARD.md** — Flags and quirks noticed by other agents

## Key Project Rules

- Adafruit components preferred unless a notably better alternative exists.
- Log all significant changes in CHANGELOG.md.
- Never silently modify another agent's work.
- Coding standards are mandatory — no deviations without explicit approval (see `standards/ACCEPTED_STANDARDS_DEVIATIONS.md`).
- **Prior art research required** before implementing hardware interfaces or novel functionality (see CODING_STANDARDS.md).
- Repository files are authoritative. Flag any conflict between agent memory and repo content.

---

## Core Standards

@standards/CODING_STANDARDS.md
@standards/DEBUG_OUTPUT.md
@standards/GIT_WORKFLOW.md
@standards/HW_GATE_DISCIPLINE.md
@standards/ACCEPTED_STANDARDS_DEVIATIONS.md

---

## Architecture & Cross-Agent Coordination

@docs/MULTICORE_RULES.md
@AGENT_WHITEBOARD.md
@COUNCIL_PROCESS.md
@CROSS_AGENT_REVIEW.md

---

## Persistent Memory & Lessons Learned

@docs/agents/LESSONS_LEARNED.md
@docs/agents/DEBUG_PROBE_NOTES.md
