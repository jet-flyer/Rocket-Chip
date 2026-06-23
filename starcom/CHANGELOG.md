# Starcom Changelog

Format: [Keep a Changelog](https://keepachangelog.com/) + SemVer once releases begin. See `VERSIONING.md` (placeholder).

> **Scope (while incubating inside Rocket-Chip):** This file logs **Starcom library** changes only — folder layout, library docs, `starcom/` CMake/tests/code. **Rocket-Chip-only changes** (firmware, AO integration, IVP, stage plans, `src/` STOP-GAP telemetry, root `docs/decisions/`, etc.) belong in the **repo-root** [`CHANGELOG.md`](../CHANGELOG.md), not here.

## Unreleased

### Added
- `starcom/` incubation folder: scaffold placeholders (`include/`, `src/ccsds/`, `adapters/`, `tests/`, `examples/`, `cmake/`), `docs/WORKING_HERE.md` (dos/don'ts), tracking placeholders (`STATUS.md`, `VERSIONING.md`, `CONTRIBUTING.md`, `LICENSE`, `docs/DESIGN.md`).

### Changed
- Six pre-existing research/design documents relocated from `docs/research/` to `starcom/docs/` **without content edits** (renamed on move; internal cross-references still cite original `docs/research/STARCOM_*` paths). Mapping: `docs/README.md`.

### Documentation
- **Condensation of all CCSDS preliminary documents (2026-06-22).** Produced canonical `docs/DESIGN.md` replacing placeholder. Includes: full agreement/conflict/gaps table (sourced from comparison.md Entries 1/2 + design_record), §0 scope (lifted), D-1..D-5 decisions with adjudications, FOP-1/FARM-1 state tables, PLCW 16-bit 7-field layout, USLP/V-3 notes, sans-I/O + conformance + PHY tiers, unique Grok PIO prior-art + <50km data, Claude bit tables + FCC/Part97, generator test idea, "architecturally complete + feature-incremental", MIB, no-heap gates, etc. Zero substantive loss — all load-bearing items attributed back to sources. Work on `feat/condense-starcom-ccsds-prelim-20260622` only. Historical docs untouched. Also updated `docs/README.md` + `STATUS.md` pointers + this entry. (starcom/docs/DESIGN.md, starcom/docs/README.md, starcom/STATUS.md, starcom/CHANGELOG.md)
- Added `docs/DATA_PRESERVATION_MANIFEST.md` (new) and `docs/scripts/capture_condensation_evidence.ps1` (new) to produce clean UTF8 no-BOM evidence files and mechanically validate no substantive loss (per strategy to satisfy verif plan and acceptance #3). No historical docs edited. (starcom/docs/DATA_PRESERVATION_MANIFEST.md, starcom/docs/scripts/capture_condensation_evidence.ps1, starcom/CHANGELOG.md)