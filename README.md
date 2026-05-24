# RocketChip

RocketChip is a modular, open-source flight computer platform targeted at model rockets, high-power rocketry, drones, and experimental aerospace applications. It is designed to be the "Flipper Zero of motion tracking" — highly capable while remaining accessible to serious hobbyists and student teams, with clear upgrade paths toward professional-grade performance.

## Development Approach

This repository follows strict engineering discipline. Major architectural decisions, safety-related changes, and significant process or tooling shifts are normally run through a **Council Review** (see below). Coding standards are treated as mandatory. Prior art research (ArduPilot, Adafruit, SparkFun, etc.) is expected before implementing hardware interfaces or novel functionality.

## Working with AI Agents

This project is designed to be worked on collaboratively by humans and AI agents (Claude, Grok, Cursor, etc.).

- **Primary instructions for agents**: See [AGENTS.md](AGENTS.md)
- **Operational discipline** (session checklists, protected files, lessons learned): See `docs/agents/`
- **Progress & status tracking**: Agents are directed to update `PROJECT_STATUS.md`, `CHANGELOG.md`, and `AGENT_WHITEBOARD.md` at the end of sessions to record progress, decisions, and current state. Humans can read these same files to quickly understand recent work and where things stand.

Humans who collaborate with agents are encouraged to read `AGENTS.md` to understand how agents are directed in this repository.

### For AI Agents

When working in this repository, please follow these reinforced expectations:

- **Repository files are authoritative.** If anything in your context or memory conflicts with files on disk, treat the repository as the source of truth and flag the discrepancy.
- Never silently modify another agent's work.
- Update `PROJECT_STATUS.md`, `CHANGELOG.md`, and `AGENT_WHITEBOARD.md` at the end of sessions with meaningful status and progress information.
- Surface assumptions and ask clarifying questions when something is unclear instead of guessing or over-engineering.

## Council Review Process

The Council is a structured, multi-persona technical review process used for major architectural decisions, safety-critical changes, broad policy or tooling changes, and significant pivots.

It brings together different perspectives through defined personas rather than relying on a single agent's (or single person's) judgment:

- **Core Personas**:
  - ArduPilot Core Contributor
  - Retired NASA/JPL Avionics Lead
  - Embedded Systems Professor
  - Senior Aerospace Student (capstone/thesis perspective)

- **Auxiliary Personas** are brought in as needed (Cubesat Startup Engineer, Advanced Hobbyist Rocketeer, etc.).

Councils are typically run when the path forward is non-obvious, when multiple valid approaches exist with real tradeoffs, or when a decision has long-term implications for safety, maintainability, or flightworthiness. They often go multiple rounds. Outcomes are usually either unanimous consensus or approval with specific numbered amendments that are then folded into the plan or implementation.

In practice, the Council has been used for things like:
- Major architecture shifts (e.g. in-flight fault recovery strategy)
- Large-scale refactors and policy changes (e.g. stdio removal approach, dead-code handling, pre-commit gate scope)
- Verification strategy pivots
- Tooling and process improvements that affect reliability

Detailed instructions for running a council review live in [COUNCIL_PROCESS.md](COUNCIL_PROCESS.md). Records of past councils are typically attached to the relevant decision documents in `docs/decisions/` or plans.

## Key Rules

- Adafruit components are preferred unless a notably better alternative exists (see `docs/hardware/HARDWARE.md` for the current policy and any amendments).
- Never silently modify another agent's work.
- All significant changes are logged in `CHANGELOG.md`.
- Coding standards are mandatory — no deviations without explicit approval (see `standards/ACCEPTED_STANDARDS_DEVIATIONS.md`).
- **Prior art research required** before implementing hardware interfaces or novel functionality (see CODING_STANDARDS.md).

## Important Documents

- [AGENTS.md](AGENTS.md) — Instructions for AI agents
- `standards/CODING_STANDARDS.md` — Mandatory coding and engineering standards
- `docs/SAD.md` — Software Architecture Document
- `docs/SCAFFOLDING.md` — Directory structure and implementation status
- `COUNCIL_PROCESS.md` — How to run Council reviews
- `CROSS_AGENT_REVIEW.md` — Protocol for multi-agent collaboration

---

For build instructions, hardware details, and contribution workflow, see the documents linked above.
