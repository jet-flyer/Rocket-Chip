# Starcom Changelog

Format: [Keep a Changelog](https://keepachangelog.com/) + SemVer. See [`VERSIONING.md`](VERSIONING.md).

> **Scope (while incubating inside Rocket-Chip):** This file logs **Starcom library** changes only, and only on **major pushes** (first tagged cut, public extract, supported-API change). Ordinary codec sittings and graph snapshots do not mint a row. Work that stays entirely under `starcom/` is recorded **here only** when a row is warranted. Do not add a repo-root [`CHANGELOG.md`](../CHANGELOG.md) row for a Starcom-only sitting. **Rocket-Chip firmware and integration** belong in the root changelog, not here.

## Unreleased

### 2026-08-27-003 | Grok 4.6 (Build CLI) | feature, architecture

**Sans-I/O data-link core cut (IVP 0–12).** Product `0.12.0-dev`. Codecs: Annex C CRC-32, PLTU, V-3, Space Packet, PLCW, CLCW, USLP (truncated / Insert / FECF), COP-P (incl. `copp_init_uslp`), COP-1 subset (FARM-1 + FOP-1 S4/S5). Host loopback + `RadioPort`. `repeat_pltu`, `hunt_pltu`, caller-owned `PltuRepeatQ`. Versioning SSOT (`STARCOM_VERSION`; same scheme as RC 2026-08-26; humans edit that file only; CMake writes `generated/starcom/version.hpp`; git is the live discriminant; incubating tags `starcom-v*`). Starcom-only graph snapshot in `starcom/graphify-out/` (query `--graph starcom/graphify-out/graph.json`; repo-root graph still excludes this tree). Prefix smoke + `STARCOM_SANITIZE` option (not run — this MinGW has no libasan). Consumer map: `docs/integration/CONSUMERS.md`. CFDP (727.0) is wanted post-mission offload, not 0–25. Next is increment 13 — owner §6 pick; do not stub the unchosen cut. No RC `add_subdirectory`. No tag. No SC-NNN. Detail: `STATUS.md`, `docs/IVP.md`, `VERSIONING.md`.

Verified: pure-software change, `starcom.unit` 1/1 PASS, host ctest 862/862 PASS, no HW reseat required.

### 2026-08-27-002 | Grok 4.6 (Build CLI) | documentation

**Docs cut before first codec.** Glossary with Blue Book section cites; README key-concept table. IVP work sequence is the increment numbers; Purpose / Methods / Closed are plan parts, not steps to run first. T/A/R/I from ECSS-E-ST-10-02; T is default; not all four on every increment. ICD codec handshake locked (`crc32`, `decode_pltu`, `encode_pltu`). Public docs state what the system is; hung-up corrections live on WORKING_HERE / `starcom/AGENT_WHITEBOARD.md`. Repeater walked back from the increment-0+1 lock in 2026-08-27-001 — early RC capability after codecs, grade still open. PIO/FPGA called out as later port seams. Blue Books 232.0-B-4, 232.1-B-2, 732.1-B-3 on the shelf. No library code in this commit.

Verified: documentation only, no firmware path, no HW reseat required.

### 2026-08-27-001 | Grok 4.6 (Build CLI) | documentation, architecture

**PLTU repeater locked into MVP; buffered grade deferred.** Bent-pipe: ASM + CRC-32, bit-exact octets out, V-3 FSN dedup — one unit, dual-use board or pole/aerostat. Not COP-P, not Space Packet parse, not a Prox-1/long-haul gateway. Buffered repeater (caller-owned queue; 133.0 §2.4 storage/forwarding; RC relay profile may use PSRAM instead of IMU RAM) is deferred, not DTN. Half-duplex TX-ready stays with the consumer. Living map: CONFORMANCE, SAD, ICD, IVP, STATUS, WORKING_HERE, DESIGN note. No library code. Verified: docs only, no firmware path, no HW reseat required.

### 2026-08-25-002 | Grok Hamilton (Grok Bot) | documentation, architecture

**Starcom docs cut + living IVP.** SAD, ICD, CONFORMANCE, identity README, folder READMEs, DESIGN pointer (D-4 closed as PLTU wraps V-3 XOR USLP). IVP at `docs/IVP.md` (IEEE 1012 + ECSS methods; Closed log IDs when gates pass, not a projected SC-01 list). Next: first codec, CMake with it. No library code. Detail: `docs/IVP.md`, `STATUS.md`.

### 2026-08-25-001 | Grok Researcher (Grok Bot) | documentation

Collected public CCSDS Blue Books and NASA/ESA/GSFC FPGA handbooks under `../../standards/starcom/` (index README). DESIGN.md gained a 2026-08-25 note on layered Prox-1 compliance, T8/LDPC, and Pluto-as-lab-PHY. Identity README / architecture map still pending. No CMake or core code this sitting.

### 2026-08-21-001 | Grok Researcher (Grok Bot) | documentation, architecture

Named the stack-vs-library distinction in `docs/DESIGN.md` (dated note) and `docs/WORKING_HERE.md`. Starcom = stack; core = sans-I/O library; ports are first-party; RC is integration. Detail lives in the DESIGN note, not here.

### 2026-06-22-001 | Grok | docs, starcom

**Condensation of all CCSDS preliminary documents.** Produced canonical `docs/DESIGN.md` replacing placeholder. Includes: full agreement/conflict/gaps table (sourced from comparison.md Entries 1/2 + design_record), §0 scope (lifted), D-1..D-5 decisions with adjudications, FOP-1/FARM-1 state tables, PLCW 16-bit 7-field layout, USLP/V-3 notes, sans-I/O + conformance + PHY tiers, unique Grok PIO prior-art + <50km data, Claude bit tables + FCC/Part97, generator test idea, "architecturally complete + feature-incremental", MIB, no-heap gates, etc. Zero substantive loss — all load-bearing items attributed back to sources. Finalized with modern naming convention (skip archaic I-prefix), added explicit pointer to detailed Claude council Round 2 verdict in design_record_claude.md for D-2 sans-I/O. Merged feat branch to main. Work only on feat/condense-starcom-ccsds-prelim-20260622. Historical docs untouched. Updated `docs/README.md`, `STATUS.md`, manifests, scripts + this entry. Verified: pure-software change, host ctest 857/857. (starcom/docs/DESIGN.md, starcom/docs/README.md, starcom/STATUS.md, starcom/CHANGELOG.md)

### Added
- `starcom/` incubation folder: scaffold placeholders (`include/`, `src/ccsds/`, `adapters/`, `tests/`, `examples/`, `cmake/`), `docs/WORKING_HERE.md` (dos/don'ts), tracking placeholders (`STATUS.md`, `VERSIONING.md`, `CONTRIBUTING.md`, `LICENSE`, `docs/DESIGN.md`).

### Changed
- Six pre-existing research/design documents relocated from `docs/research/` to `starcom/docs/` **without content edits** (renamed on move; internal cross-references still cite original `docs/research/STARCOM_*` paths). Mapping: `docs/README.md`.

### Documentation
- Added `docs/DATA_PRESERVATION_MANIFEST.md` (new) and `docs/scripts/capture_condensation_evidence.ps1` (new) to produce clean UTF8 no-BOM evidence files and mechanically validate no substantive loss (per strategy to satisfy verif plan and acceptance #3). No historical docs edited. (starcom/docs/DATA_PRESERVATION_MANIFEST.md, starcom/docs/scripts/capture_condensation_evidence.ps1, starcom/CHANGELOG.md)