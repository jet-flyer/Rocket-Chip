# Code Trimming / Source Size Reduction Targets Audit

**Date:** 2026-07-03  
**Author:** Grok 4.3 (Build CLI) analysis session  
**Scope:** Main Rocket Chip authored source only (`src/` + `include/rocketchip/`). Excludes vendored (lib/, EXTERNAL/, pico-sdk/), tests/, scripts/, starcom/, docs/, ground_station/, tools/.  
**Status:** Open — identification only. No source modifications performed. Targets for future discussion/implementation passes. Saved per user request to pause and return later.  
**Related prior work:**  
- Initial survey targets documented in scratch `ROCKET_CHIP_SIZE_REDUCTION_TARGETS_ANALYSIS.md` (session temp).  
- Dead-code policy per `standards/CODING_STANDARDS.md` (2026-05-22 council).  
- Previous `docs/audits/DEAD_CODE_INVENTORY_2026-05-22.md` (clean at the time).  
- History cross-checks via git, CHANGELOG.md, SAD.md, SCAFFOLDING.md, ROCKETCHIP_OS.md, AO_ARCHITECTURE.md, LESSONS_LEARNED.md, ESKF docs, etc.

## Executive Summary

The main codebase (~186 files, ~33k LOC) has accreted over months of incremental work (AO extractions, protocol additions, role/station-vehicle branching, cal wizards, radio config, etc.). This creates opportunities for reliable, functionality-preserving source reduction focused on:

- Dead/unused abstractions and guarded fallback paths.
- Duplication and structural bloat (e.g., glue code, boilerplate).
- Piecemeal growth in large modules (CLI/RC_OS morphed from simple menu; custom utilities grown without later consolidation).
- Scaffolding and legacy remnants from bring-up/retirement efforts.

**Key principle (reaffirmed):** Efficiency + reliability (e.g., fewer branches to audit, single source for fixes, reduced cognitive load in hot paths like ESKF/CLI). Not arbitrary cuts. All suggestions preserve supported vehicle/station configs, main runtime paths, and documented architecture.

Total potential (conservative net estimates from prior analysis + history): 400–1500+ LOC across focused efforts, depending on depth of refactors. Individual items range from small cleanups to higher-impact architectural cleanups in largest modules.

**Next steps (when resumed):** Re-review this audit + scratch analysis. User to approve targets for implementation. Any future pass must follow full session checklist, HW gates where relevant, protected-doc triggers, and dead-code discipline (history trace before deletion).

## Identified Targets (Prioritized by Potential Impact)

### 1. ESKF Alternative Covariance Paths / Bierman Fallback (Larger Impact)
- **Files:** `src/fusion/eskf.cpp` (multiple `#ifndef ESKF_USE_BIERMAN` blocks + `clamp_covariance`/`clamp_*` + calls in baro/mag/zupt/gps paths), `src/fusion/eskf.h` (decls/conditionals), related in `eskf_codegen.cpp`.
- **Description:** Bierman/UD chosen (commit `cbdeb60`, 2026-02) for ~43% measurement speedup (O(N²) Joseph form -> UD). Joseph/scalar fallback + clamps retained behind `#ifdef` "for A/B". 
- **Est. net reduction:** 100–200+ LOC (dead alternative impl in 1625 LOC file; keep active Bierman + predict path).
- **Why high value:** Concentrated in largest module. Reduces conditional source in safety-critical fusion. Fits "guarded by now-unneeded defines".
- **History check (2026-07-03 session):** Reasoning holds (performance primary on RP2350-class + ICM-20948; numerical conditioning bonus). No supersession in CHANGELOG/docs (still documented in ADVANCED_SETTINGS.md as "Bierman vs Joseph"). Recent June renames only. `ESKF_USE_BIERMAN=1` always-on in main CMake target.
- **Preserves:** Current Bierman path, 24-state ESKF, inhibits, latency budgets, f32 precision (canaries passed).
- **Risks/Notes:** Confirm test suite (separate bierman target) doesn't require fallback. Cross-check SAD fusion section + ESKF plans.
- **References:** `docs/decisions/ESKF/FUSION_ARCHITECTURE.md`, CHANGELOG Bierman entries, eskf.cpp commit history.

### 2. RC_OS / CLI as Whole — Menu/Navigation + Command Bloat (Higher-Impact Refactor Candidate)
- **Files:** `src/cli/rc_os_commands.cpp` (1455 LOC), `src/cli/rc_os.cpp` + debug/dashboard, `src/active_objects/ao_rcos.cpp` + .h (1193 LOC), related dispatch.
- **Description:** Started simple (APM-inspired single-key menus per ROCKETCHIP_OS.md). Piecemeal accretion: async cal wizards (full `CalUiState` machine), radio config cycle, tracked commands, flight injects, debug submenu, dashboard, confirmations, output mode cycling. AO_RCOS at 20Hz now drives significant UX state (see below).
- **Est. net reduction:** 300–500+ LOC (boilerplate handlers + dispatch; data-driven table or improved navigation state machine).
- **Why high value:** One of two largest files. "Morphed almost into a pseudo-OS". More efficient menu navigation likely highest impact.
- **History check:** Extraction from main.cpp (SAD: 3384→706 lines). ROCKETCHIP_OS.md (updated 2026-03 with 2026-05 drift fixes for bench deprecation). AO_ARCHITECTURE.md (2026-04 council). June renames only (no structural change).
- **UX State Owned by AO_RCOS (clarification from audit discussion):** Per `ao_rcos.h` header: "Owns all serial I/O: key dispatch, output mode cycling, ANSI dashboard rendering, USB connection state machine, and calibration UI state machine." `calibration_manager` owns async execution. `rc_os.cpp` is "pure menu dispatcher".
  - In `RcosAo` struct: ANSI dashboard render state (`last_ansi_*`), full `CalUiState` machine + wizard/6pos/mag tracking, confirmation buffers (reset/erase), wizard counters, `g_outputMode`.
  - Does **not** own core data (sensors, ESKF, cal coefficients — those in `calibration_manager`).
  - Intent: UX layer that triggers (posts intents/signals). Has grown with non-blocking cal (Phase D).
- **Preserves:** All current commands/behaviors, station/vehicle gating, test-mode gates (per 2026-05-13 bench deprecation), menus.
- **Risks/Notes:** Table-driven or better navigation state machine would reduce per-item code while keeping single-key UX.
- **References:** `docs/ROCKETCHIP_OS.md`, `docs/AO_ARCHITECTURE.md`, CHANGELOG (extractions, deprecations), `src/active_objects/ao_rcos.cpp` (struct + comments).

### 3. Custom rc_log Formatter (Hindsight Refactor Candidate)
- **Files:** `src/log/rc_log.cpp` (680 LOC), `include/rocketchip/rc_log.h`, uses across AOs/drivers/logging/.
- **Description:** Built during R-5 stdio removal (~2026-05). Hand-rolled mini-printf (to `etl::to_string`) after council pivot (ETL float divergences: `-0.0` sign, rounding mode vs IEEE/libc). ~500 LOC formatter + drain logic. Idle drain contention issues documented/fixed (LESSONS_LEARNED Entry 39).
- **Est. net reduction:** 200–400+ LOC (slim to actual used specs from inventory; or lighter alternative).
- **Why high value:** Large utility. With hindsight (stdio excised, usage patterns known, project direction clearer), better/more efficient/maintainable option likely exists.
- **History check:** Council (2026-05-15/17) on hand-rolled for correctness (ground parsers). CHANGELOG details R-5 units, binary size cost accepted. Recent June rename only. Still referenced in AO_ARCH, standards audits.
- **Preserves:** All used log formats in main flight paths (per `docs/audits/STDIO_FORMAT_SPEC_INVENTORY_2026-05-15.md`), ring/drain behavior.
- **Risks/Notes:** Alternatives considered historically (etl subset, eyalroz/printf exit-ramp candidate). Inventory-driven slimming or refactor recommended.
- **References:** CHANGELOG R-5 entries, `docs/agents/LESSONS_LEARNED.md` (Entry 39), `docs/audits/STDIO_FORMAT_SPEC_INVENTORY_2026-05-15.md`, `src/log/rc_log.cpp`.

### 4. Earlier Survey Targets (Baseline from Initial Pass; Lower Individual Impact but Cumulative)
From src/include survey + LOC/grep (186 files, top: eskf.cpp 1625, rc_os_commands 1455, ao_rcos 1193, etc.):
- Dead `TelemetryEncoderState` + legacy `CcsdsEncoder::encode_nav` (wrapper + 42B path; ~70–110 LOC; ao_telemetry already uses direct `encode_nav_with_config` + MAVLink; decoder/legacy APID compat stays for RX).
- GPS backend glue duplication (`gps_pa1010d.*` + `gps_uart.*`; ~120–180 net after extracting common lwGPS/parse; both transports must stay for runtime selection).
- Board scaffolding branches/headers (`board.h` tiny/pico2 elifs + `board_tiny_2350_*`, `board_pico2.h`; ~150–190 LOC; explicit "scaffolding" comments, 4TIER retirement).
- Scattered from largest modules: `kApidDiag` (unused), parked beacon, `DEPRECATED` blocking cal API (`calibration_collect_6pos_position`), gated "Stage T2 cheat-mode" (~55–70 LOC), legacy `kCalNeo*` compat in ao_rcos (~15–25 LOC).

**Est. net:** 400–600+ LOC total. History: align with dead-code policy, no contradictions in prior inventory (clean 2026-05-22). Recent renames/refactors (June 2026) but core still present.

**History/Supersession Notes (all items):** 2026-07-03 re-check via git/CHANGELOG/docs showed reasoning holds (no major reversals). Recent activity mostly style/standards (e.g., `1bfa602` renames). Piecemeal accretion (AO extractions, T5.5 radio, Stage L/L cal async) explains bloat. Starcom still scaffold (no supersession of telemetry paths). SAD/SCAFFOLDING slightly lag implementation details.

## Recommendations for Future Pass
- Prioritize by impact + risk (dead code first, then refactors with tests + bench_sim).
- Full verification: src/-limited greps for callers, read key files, cross-check SAD/AO_ARCH/ROCKETCHIP_OS, run verification plan from original goal.
- Metrics: re-count LOC pre/post, binary size (if relevant), but focus source as per goal.
- Document in CHANGELOG + new audit entry on implementation.
- Protected files: no direct edits without approval.

**End of current identification.** Resume when ready for deeper drill or implementation discussion.

---
*This file is a historical-record audit (new file per policy). Appended for traceability. No state-of-system protected docs contradicted.*