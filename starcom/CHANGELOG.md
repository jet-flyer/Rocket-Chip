# Starcom Changelog

Format: [Keep a Changelog](https://keepachangelog.com/) + SemVer. See [`VERSIONING.md`](VERSIONING.md).

> **Scope (while incubating inside Rocket-Chip):** This file logs **Starcom library** changes only, and only on **major pushes** (first tagged cut, public extract, supported-API change). Ordinary codec sittings and graph snapshots do not mint a row. Work that stays entirely under `starcom/` is recorded **here only** when a row is warranted. Do not add a repo-root [`CHANGELOG.md`](../CHANGELOG.md) row for a Starcom-only sitting. **Rocket-Chip firmware and integration** belong in the root changelog, not here.

## Unreleased

### 2026-09-03-001 | Grok 4.6 (Build CLI) | hardware, documentation

**Two-board ON Pass A pre-soak: A1/A2 scored; A3–A5 blocked on station image.** Consumer `ROCKETCHIP_USE_STARCOM=ON`, 45 B nav SDU, desk Class A, 2 dBm. A1: COP-P lock, CRC 0, ~4.6 Hz, ARM leftover FAIL (seq-nav+PLCW TX-busy, paper ~82% of 200 ms). A2 10 Hz FAIL as expected. Live Jam still `flight-2f7096d` SET pwr=20 vs vehicle 2 dBm; SET ±6 dB gate + no leftover → PHY stayed BW125. 2 dBm station UF2 built, not flashed. Dual-board picotool targeting parked (owner: solved months ago; flash with the other board unplugged). Resume: `logs/soak/2026-09-03_HANDOFF.md`. Completed A/B scores are RC `docs/RADIO_SOAK_PASS_AB_2026-09-03.md` (root CHANGELOG `2026-09-03-002`). Verified: COM5 CFG BW=125/5/2 dBm RegVersion 0x12 Hardware 13/13; COM7 banner `flight-2f7096d`; no new Starcom library code in this row.

### 2026-09-02-001 | Grok 4.6 (Build CLI) | documentation, tooling, architecture

**One Starcom tree + PICS catch-up + no library deviation.** Worktree `C:\Users\pow-w\Documents\starcom_dev` (`grok/sc-dev`). PICS levels 0 / Best effort / Full in CONFORMANCE; FSK bitstream is a future Best-effort bearer, not 211.1. House identifiers follow NASA F´ / cFS practice (C++ tokens are house camelBack; Blue Book names stay in comments). No Starcom row in `ACCEPTED_STANDARDS_DEVIATIONS.md`. `Error` enumerators and JSF 151 names remain open on this tree. Starcom-owned clang-tidy + Grey report under `docs/audits/`. FPGA hub already on `main`. No version bump. No tag.

Verified: documentation + tidy tooling, no firmware path, no HW reseat required.

### 2026-08-28-003 | Grok 4.6 (Build CLI) | architecture, bugfix, documentation

**Starcom wrap through IVP 25.** MCU: COP encode scratch in BSS; `fop1Init`/`fopPInit` memset in place; GNU `-Wstack-usage=1024`. IVP 24: WSL ASan+UBSan, book-max fuzz, measured size. IVP 10 remainder: FOP-1 Resume/setup/LLIF, TT=1 suspend (232.1 Table 5-1). Sent copies off 256-FSN tables (`CoppEndpoint` 19544→10136). Consumer `docs/USER_GUIDE.md`. Product **`0.2.25`** (owner: `0.2.N`, N = increment), EXTRA empty, tag `starcom-v0.2.25`. FPGA PHY/decode still held; CFDP wanted, not 0–25. No SC-NNN.

Verified: pure-software change, `starcom.unit` PASS, no HW reseat required.

### 2026-08-28-002 | Grok 4.6 (Build CLI) | bugfix

**`copp_init` / `cop1_init` memset in place.** `e = CoppEndpoint{}` (and the COP-1 twin) compiled to an ~18 KiB stack temporary; Pico Core 0 stack is 4 KiB (`PICO_STACK_SIZE=0x1000`). First ON UF2 reset-looped at `AO_Telemetry_start`. `payload_by_fsn` / `payload_by_ns` stay in BSS. Product still `0.19.0-dev`. No tag. No SC-NNN.

RC Pico+AO (21) and COP-P air path (22) are the consumer sitting — log them in the repo-root `CHANGELOG.md` `2026-08-28-002`, not here. Branch `grok/sc-dev` (`1090959`).

Verified: `StarcomBytePump` + `CmdSdu` host tests PASS; vehicle ON after the fix `Air: starcom-prep`, `bench_sim` 2/2 PASS on COM5. Detail of the RC wiring is the root entry.

### 2026-08-28-001 | Grok Hamilton (Grok Bot) | feature

**IVP 13–19 cut.** Product `0.19.0-dev`. Full 211.0 §6 MAC + SET V(R) (13). V-3 DFC 11 user-defined octets, not bitstream (14). Host file replay + UDP; sockets only on `Starcom::adapters_host` (15). Generic SPI/GPIO `BusOps`, host fake bus, no Pico/RFM in the core (16). PIO bit pipe, not 211.1 (17). PHY adapter tiers; uncoded host path; `PhyTier::compliant` not offered (18). Conv K=7 r=1/2 with G2 inversion + LDPC (2048,1024) encode, CSM `0347 76C7 2728 95B0`, codeword-only randomize; decode later GCS/Pi (19). Next is increment 20 (RC host `add_subdirectory`). FPGA/decode hold is on `AGENT_WHITEBOARD.md` (Forgix first, then Snickerdoodle). No merge to main. No tag. No SC-NNN. Detail: `STATUS.md`, `docs/IVP.md`.

Verified: pure-software change, `starcom.unit` 1/1 PASS, host ctest 862/862 PASS, no HW reseat required.

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