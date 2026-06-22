# Starcom status

**Placeholder.** Library-scoped phase and blockers (lighter than Rocket-Chip `docs/PROJECT_STATUS.md`).

> **Note:** The design record and research corpus under `docs/` were produced **before** this folder existed (originally at `docs/research/`). Relocated unchanged 2026-06-18 — see `docs/README.md` for mapping.

Update when implementation work begins. Until then, design status lives in `docs/design_record_claude.md` and open forks in `docs/comparison.md`.

**Condensation complete (2026-06-22, feat branch):** `docs/DESIGN.md` is now the canonical condensed record. Historical sources preserved. See DESIGN.md header for table + no-loss summary.

## Phase

**Research + folder scaffold** (2026-06-18). No code, no CMake targets, not wired into root build.

## Next

1. ~~Condensation session → `docs/DESIGN.md`~~ **DONE 2026-06-22** (see DESIGN.md)
2. Phase 0 CMake skeleton (see `docs/research/library_craft_claude.md` §7)
3. Standards ref update: CCSDS 131.0-B-3 → B-5 before coding definitions lock (see root `AGENT_WHITEBOARD.md`)

## Blockers

- Half-duplex telemetry flight blocker drives *when* RC integrates; library scaffold is not blocked on hardware.