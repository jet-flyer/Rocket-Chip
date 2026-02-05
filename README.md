# RocketChip Agent Instructions

## Read First
Read these fully on first session. Re-read only if changelog notes updates.

- **docs/SAD.md** - Software Architecture Document (system design, modules, interfaces, task model)
- **docs/SCAFFOLDING.md** - Directory structure and implementation status
- **standards/CODING_STANDARDS.md** - Coding standards, communication protocols, data formats, safety requirements
- **standards/DEBUG_OUTPUT.md** - Debug output conventions and macros
- **docs/hardware/HARDWARE.md** - Prototype hardware, GPIO usage, I2C conflicts, product naming
- **COUNCIL_PROCESS.md** - How to run panel reviews and persona behavior
- **CROSS_AGENT_REVIEW.md** - Protocol for flagging concerns in other agents' work

## Each Session
These change often. Check at the start of every session.

- **PROJECT_STATUS.md** - Current phase, blockers, planned work
- **CHANGELOG.md** - When, who, what, why
- **AGENT_WHITEBOARD.md** - Flags and quirks noticed in other agents' work

## Key Rules
- Adafruit components preferred unless notably better alternative exists
- Log all significant changes in CHANGELOG.md
- Never silently modify another agent's work
- Coding standards are mandatory—no deviations without explicit approval (see exceptions table in standards/STANDARDS_DEVIATIONS.md)
- **Prior art research required** - Before implementing hardware interfaces or novel functionality, check how ArduPilot, Adafruit, and SparkFun handle it (see CODING_STANDARDS.md "Prior Art Research" section)
- Repository files are authoritative. If conflicts exist between agent memory and repo content, flag the discrepancy for review—don't silently overwrite either.
- If caching or access issues pop up, prefer curl-equivalent methods like direct raw.githubusercontent.com URLs or appending timestamps to query parameters.
