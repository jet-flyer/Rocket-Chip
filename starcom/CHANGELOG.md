# Starcom Changelog

Format: [Keep a Changelog](https://keepachangelog.com/) + SemVer once releases begin. See `VERSIONING.md` (placeholder).

> **Scope (while incubating inside Rocket-Chip):** This file logs **Starcom library** changes only — folder layout, library docs, `starcom/` CMake/tests/code. **Rocket-Chip-only changes** (firmware, AO integration, IVP, stage plans, `src/` STOP-GAP telemetry, root `docs/decisions/`, etc.) belong in the **repo-root** [`CHANGELOG.md`](../CHANGELOG.md), not here.

## Unreleased

### Added
- `starcom/` incubation folder: scaffold placeholders (`include/`, `src/ccsds/`, `adapters/`, `tests/`, `examples/`, `cmake/`), `docs/WORKING_HERE.md` (dos/don'ts), tracking placeholders (`STATUS.md`, `VERSIONING.md`, `CONTRIBUTING.md`, `LICENSE`, `docs/DESIGN.md`).

### Changed
- Six pre-existing research/design documents relocated from `docs/research/` to `starcom/docs/` **without content edits** (renamed on move; internal cross-references still cite original `docs/research/STARCOM_*` paths). Mapping: `docs/README.md`.