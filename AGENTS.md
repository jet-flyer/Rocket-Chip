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

- Never silently modify another agent's work.
- Coding standards are mandatory — no deviations without explicit approval (see `standards/ACCEPTED_STANDARDS_DEVIATIONS.md`).
- **Prior art research required** before implementing hardware interfaces or novel functionality (see CODING_STANDARDS.md).
- **CCSDS Blue Books first** for Starcom / MAC / COP-P / air: open the cited PDF in `starcom/standards/ccsds/` and `starcom/docs/COVERAGE.md` neighbors **before** writing a workaround. Do not invent a dwell, token substitute, or retry layer when the book names the event. Detail: `starcom/docs/WORKING_HERE.md`.
- Repository files are authoritative. When a discrepancy between agent memory and repo content is flagged, the repository is almost always the correct, double check with user to update your memory.
- Write in ASD-STE100 Simplified Technical English (the current issue, https://www.asd-ste100.org/). This covers chat replies, new commit messages, and new CHANGELOG entries. Use short sentences. Put one topic in each sentence. Keep technical names, book cites, and operational numbers. `starcom/docs/USER_GUIDE.md` is the wording example. It is not the limit of the rule.
- Content rules for commit messages still apply (for example `standards/HW_GATE_DISCIPLINE.md` Rule 3).
- Do not rewrite existing documents or old commits from this rule, aside from re-evaluation passes explicitly triggered by a human user.
- Code, comments, and firmware log strings stay under `standards/CODING_STANDARDS.md`.
- **Hardware-facing changes need a real-board check before merge to main.** Scope: `src/drivers/` (I2C, SPI, UART, GPS, IMU, baro, RFM95), `src/active_objects/ao_radio*`, `src/diag/radio_*`, `src/logging/radio_config_storage*`, `src/safety/` (boot, watchdog), `src/main.cpp`, `src/cli/` (CLI `k` restart path), `pio/*.pio`, and any ISR, boot or reset path. You can commit before the check. Until then the commit's `Verified:` line must say `UNBENCHED`. The check is a run on real boards by Buzz or Grok Build. It must cover warm restarts (CLI `k` and `picotool reboot`), a power-on boot, and a short soak with the radio link live. Hook and pre-push checks alone do not count. Why: `c4278fb3` (cloud agent, host-tested only) caused the station PA1010D warm-restart hang. Also in `.cursor/rules/hw-verify.mdc`.
- The bench assumes the board console is USB serial (USB CDC); a board with a UART-only console needs a USB-UART adapter.

---

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
