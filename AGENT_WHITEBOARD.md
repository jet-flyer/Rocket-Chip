# Agent Whiteboard

**Purpose:** Communication across context windows and between agents.

**Stages 1-12A COMPLETE.** 610+ host tests, SPIN 6/6. **Stage 13 (Health Monitor) IN PROGRESS.** Tracking: `docs/AO_ARCHITECTURE.md`.

## Use Cases
1. **Cross-agent review** - Flag concerns about other agents' work (see `CROSS_AGENT_REVIEW.md`)
2. **Cross-context handoff** - Notes for future Claude sessions when context is lost
3. **Work-in-progress tracking** - Track incomplete tasks spanning multiple sessions
4. **Hardware decisions pending** - Flag items needing user input before code changes
5. **This is a whiteboard** - Erase completed items. Only keep active flags and deferred work.

---

## Open Flags

*Stage 13 (Health Monitor) in progress. IVP-104 through IVP-112.*

### Protected File Updates Pending Approval

*None currently.*

---

## Upcoming Stages

**Stage 13: Health Monitor** (IN PROGRESS) — AO_HealthMonitor, 2-bit encoding, fault patterns, preflight CLI, debug sub-menu. Council-reviewed plan in `.claude/plans/wild-wishing-pinwheel.md`.

**Stage 14: Notification Engine** (NEW) — AP_Notify-style intent→display routing. Scope doc produced at end of Stage 13.

**Stage 15: Pre-Flight Polish** (was 14) — 15A Telemetry Polish, 15B System Polish, 15C Verification

**Stage 16: Field Tuning** (was 15) — All VALIDATE parameters. Needs flight data.

See plan file for full breakdown.

### Deferred (near-term, post-Stage 15)

- **Battery ADC Monitoring** — Hardware not wired. ADC pin + driver + telemetry field.
- **CCSDS SDLS Command Authentication** — Telecommand auth for Rocket profile.
- **IVP-103 Station GPS Push** — Needs radio command path.

### Far-future (moved to PROJECT_STATUS)

Mission Profile OTA, F' Evaluation, u-blox GPS, OTA drivers, GPS-free 3D reconstruction, FSK bitstream, MATLAB export — all moved to `docs/PROJECT_STATUS.md` future features.

### Recorded Elsewhere (Removed from Whiteboard)

These completed/resolved items are preserved in their canonical locations:

| Item | Recorded In |
|------|-------------|
| All Stage 1-12A completion details | `docs/PROJECT_STATUS.md` completed table |
| Stage 7 IVP-57-65 details | CHANGELOG 2026-04-07/08, PROJECT_STATUS |
| AO/State Engine Logging audit | `docs/ADVANCED_SETTINGS.md` (verbose/MP-configurable logging) |
| PCM frame expansion | `docs/ADVANCED_SETTINGS.md` Research Mode |
| DPS310 baro rate optimization | CHANGELOG, baro driver code |
| DPS310 baro read count fix | CHANGELOG, baro driver code |
| SRAM execution audit | `docs/benchmarks/` |
| Dense FPFT benchmark | `docs/benchmarks/`, whiteboard resolved (archived) |
| UD factorization benchmark | `docs/benchmarks/UD_BENCHMARK_RESULTS.md` |
| Bierman measurement update | CHANGELOG, ESKF code |
| MMAE/IMM pivot | `docs/decisions/ESKF/ESKF_RESEARCH_SUMMARY.md` |
| 24-state ESKF expansion | CHANGELOG, PROJECT_STATUS |
| Codegen FPFT (IVP-47) | CHANGELOG, PROJECT_STATUS |
| BSS/codegen sensitivity disproved | `LESSONS_LEARNED.md` Entry 27 |
| All strikethrough DONE items | CHANGELOG entries, PROJECT_STATUS |
| VALIDATE values inventory | `docs/UNIQUE_COMMENT_ITEMS.md` |
| Power optimization notes | Codegen is mandatory (benchmarked). Revisit if state count > 24 |
| clang-tidy status | Full audit clean. lizard CCN in tiered audit. `standards/STANDARDS_AUDIT_2026-03-26.md` |
| Job/Mission naming | Code uses `job.h` namespace. Settled. |
| ivp62-wip branch | Deleted 2026-04-09. All content ported to main. |

---

## Resolved

*Cleared 2026-04-09. All resolved items recorded in PROJECT_STATUS.md, CHANGELOG.md, and LESSONS_LEARNED.md.*
