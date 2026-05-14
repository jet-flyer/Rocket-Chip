# Master Standards Audit — 2026-05-13 (Tiered Re-Audit)

**Audit Date:** 2026-05-13 (Tiers 1-4 in this session); Tier 5-7 deferred to fresh session per council amendment #7 (DO-178C verification-independence).
**Audited By:** Claude Opus 4.7 (1M context).

---

## Handoff to next agent — Tiers 5, 6, 7

**This handoff is to Grok via Cursor.** Grok is a structurally different LLM from the Claude Opus 4.7 instance that walked Tiers 1-4, which is what gives this verification substantive DO-178C-style independence (not just fresh-session-same-agent structural independence). Cursor's tooling reads files directly, doesn't load Claude's `MEMORY.md` or `.claude/` auto-load, and doesn't share my conversation history — exactly what's wanted.

**You (Grok) are picking up this audit after Tiers 1-4 are complete. Tiers 5-7 are your job.**

### What was done (Tiers 1-4 — read the report below for detail)

- **Tier 1** (gate integrity): PASS. 2 minor findings about missing audit-tooling fixtures, both DEFERRED to audit-infrastructure work.
- **Tier 2** (foundation): PASS. 1 minor finding about toolchain-drift script gap (DEFERRED). 1 major finding (F-2026-05-13-004 — three production .cpp files had drifted out of the pedantic-warning gate) — REMEDIATED in this commit. All 4 build tiers rebuilt clean.
- **Tier 3** (standards catalog): PASS. 0 new FAIL or PARTIAL. Walked exhaustively with a one-time brevity exception (FAILs/PARTIALs documented in detail, PASSes covered by a coverage statement at the top of `STANDARDS_AUDIT_2026-05-13.md`).
- **Tier 4** (runtime behavior): PASS. R-17 strengthens prior AO Commandments deviations. CLA-RBM watched but within 90-day staleness threshold. Tier 4.3 fault-injection remains PARTIAL pending R-23 (vehicle-bench-tier HardFault, blocked on R-25 bench-tier deprecation evaluation).

### What you need to do

Walk Tiers 5, 6, and 7 of `standards/AUDIT_GUIDANCE.md`. The procedure tells you what each sub-item is. Per the council amendments (already applied), stop conditions are severity-gated (only NASA SWE §8.5 Catastrophic/Critical halt; Major and below flow forward to Tier 7 disposition).

**Tier 5** — pre-flight gate (`docs/PRE_FLIGHT_CHECKLIST.md`), RP2350 errata (Appendix A.2), bench_sim end-to-end, replay-gate (blocked on R-23 — documented gap), exhaustive requirements traceability spot-check, regression check on prior-cycle closed findings.

**Tier 6** — F-2 audit-cycle citations in non-standards docs (PROJECT_STATUS, SAD, SCAFFOLDING, AO_ARCHITECTURE), Sections G/H of `STANDARDS_AUDIT.md` template, protected-doc drift check, CHANGELOG rollup.

**Tier 7** — disposition the findings from Tiers 1-4 (4 are open: F-2026-05-13-001/002/003 DEFER, F-2026-05-13-004 REMEDIATED-pending-close). Per Appendix C: 4-category ordering (Gate Integrity → Shared Foundations → Behavior Changes → Cleanup), local-vs-audit-suite verification cadence.

### How the verdicts in Tiers 1-4 should affect your work

Treat them as CLAIMS by the prior agent, not as facts. If you walk the same sub-item and get a different verdict, your verdict is the audit-of-record one. Document where you agree and where you don't.

### Inputs to read

1. `standards/AUDIT_GUIDANCE.md` (the 7-tier procedure).
2. `docs/audits/AUDIT_GUIDANCE_REWRITE_PROPOSAL_2026-05-13.md` (proposal + council verdict with 8 amendments — context for why the procedure looks the way it does).
3. The rest of this report below (Tier 1-4 work).
4. `docs/audits/STANDARDS_AUDIT_2026-05-13.md` (Tier 3 dated companion).
5. `docs/audits/AUDIT_COVERAGE_INVENTORY_2026-05-13.md` (inventory deliverables).
6. The 4 protected standards files: `CODING_STANDARDS.md`, `HW_GATE_DISCIPLINE.md`, `ACCEPTED_STANDARDS_DEVIATIONS.md`, `RP2350_ERRATA.md`.
7. For Tier 5: `docs/PRE_FLIGHT_CHECKLIST.md`.
8. Source code as needed.

### How to deliver

Append your Tier 5/6/7 walks as new sections in this same report (`MASTER_STANDARDS_AUDIT_2026-05-13.md`). Note your agent identity at the top of your Tier 5 section so future readers can trace the independence claim. Update the findings table with any new entries. Update the user-facing CHANGELOG with a brief session-end entry that points to this report.

---
**Procedure:** `standards/AUDIT_GUIDANCE.md` 7-tier dependency-ordered structure (refactored 2026-05-13 in this session — see `docs/audits/AUDIT_GUIDANCE_REWRITE_PROPOSAL_2026-05-13.md` for the proposal + council verdict).
**Prior cycle:** `docs/audits/MASTER_STANDARDS_AUDIT_2026-05-07.md` (L1 + L2 wraps). This cycle is the first run of the post-refactor procedure.
**Step-1b inventory deliverable:** `docs/audits/AUDIT_COVERAGE_INVENTORY_2026-05-13.md` (proves the procedure is gap-free at procedural level; 3 tooling-class gaps documented).
**Codebase snapshot at audit start:** to be captured at the start of Tier 1 below.

---

## Cycle scope split (strict, council-approved)

Per user direction 2026-05-13 (informed by council amendment #7 — DO-178C verification-independence):

- **This session (2026-05-13):** Tiers 1, 2, 3, 4.
- **Fresh session (next):** Tiers 5, 6, 7. Held for independent-session walk to get level-3 verification credit per `HW_GATE_DISCIPLINE.md` Rule 6. Stage 17 (first motor flight) is the upcoming milestone — level-3 verification credit is load-bearing.

**Pacing per user direction:** Per-finding conversational (`AUDIT_GUIDANCE.md` Appendix B.4 default). When a finding surfaces during a tier walk, the agent surfaces it for user disposition (REMEDIATE / ACCEPT / DEFER) before continuing.

---

## Tier 1 — Verify the Audit's Own Gates

Status: **PASS with 2 Minor tooling-gap findings (DEFERRED).** No severity-gate halt.

**Snapshot:** working tree at `fcd3496` + uncommitted step-1a doc edits (no source changes since `c9c8585`).

### 1.1 Tool self-checks — PASS
- clang-tidy 21.1.8 (`/c/Program Files/LLVM/bin/clang-tidy`)
- cppcheck 2.20.0 (`/c/Users/pow-w/bin/cppcheck`)
- lizard 1.21.2 (`/c/Users/pow-w/bin/lizard`)
- Matches 2026-05-07 cycle fingerprint exactly.

### 1.2 SPIN model compile + load — PASS
- SPIN 6.5.2 Cygwin (`/c/tools/cygwin/bin/spin.exe`)
- 6 .pml models present (`rocketchip_ao`, `rocketchip_boot`, `rocketchip_fd`, `rocketchip_flash_protocol`, `rocketchip_rf_manager`, `rocketchip_station`)
- Full re-verification happens at Tier 2.2.

### 1.3 Pre-commit hook integrity — PARTIAL (F-2026-05-13-001)
- Hook present at `scripts/hooks/pre-commit` (212 lines), `core.hooksPath = scripts/hooks` (set per CHANGELOG 2026-04-27-005).
- `bash -n` syntax: OK.
- 3 references to `pre_commit_matrix.py` — matrix-driven gate structure intact.
- **F-2026-05-13-001 (Minor):** no synthetic staged-diff fixture exists to validate the hook's gate matrix at audit time. Hook itself is exercised by every real commit and observed working; the gap is in audit-time validation only.
- **Disposition: DEFER** to audit-infrastructure queue (L2-P3 / R-22 / R-24 family). Safety-impact: hook's runtime enforcement is unaffected; only audit-time confidence in hook correctness is weakened.

### 1.4 bench_sim.py bidirectional regex audit — PASS (F-2026-05-13-002)
- (a) **Rotted-regex check: PASS by transitivity.** `c9c8585` ran bench_sim 2/2 PASS; no source changes since; therefore all 4 regexes (`RE_TRANSITION`, `RE_PYRO`, `RE_PHASE`, `RE_PROMPT`) still match live log lines.
- (b) **Deleted-regex check: PASS by manual walk.** 18 firmware `[FD]` emissions in `src/`; bench_sim's 2-scenario documented scope (happy-path + abort-from-BOOST) covers the subset its 4 regexes target. Other `[FD]` emissions (ABORT-different-states, CRITICAL_FAULT, timeout) are by-design out of bench_sim's scope and are covered by host `test_command_handler.cpp` rejection-path tests instead.
- **F-2026-05-13-002 (Minor):** no maintained PASS-token inventory exists, so (b) was a manual walk this cycle rather than a mechanical check.
- **Disposition: DEFER** to audit-infrastructure queue. Safety-impact: bench_sim's documented scope is fully covered today; the missing inventory only weakens audit-time confidence that the script's scope hasn't silently shrunk in a future refactor.

### 1.5 Host ctest fixture-suite — PASS
- 794/794 tests passed, 0 failed, 6.04 sec.
- Matches `c9c8585` recorded count exactly. **Positive-control signal observed.**

### 1.6 Build-parity (4 tiers) — PASS
- `build/` (vehicle bench, `NOT_CERTIFIED_FOR_FLIGHT=ON`) — built clean, 12 jobs.
- `build_flight/` (vehicle flight) — built clean, 11 jobs.
- `build_station/` (station bench) — already up to date.
- `build_station_flight/` (station flight) — built clean, 11 jobs.
- All 4 link successfully. No warnings.

**Tier 1 verdict:** PASS. No Catastrophic/Critical findings. Severity gate: not triggered. Proceeding to Tier 2.

---

## Tier 2 — Establish the Foundation

Status: **PASS with 4 findings (F-2026-05-13-003 DEFER, F-2026-05-13-004 REMEDIATED, all others by transitivity from L2 baseline).**

### 2.1 Baseline scripted sweeps — PASS (by transitivity)
- Zero source-file changes since L2 baseline `c9c8585` (verified by `git diff --stat c9c8585 HEAD -- 'src/**' 'include/**' 'lib/**'` returning empty).
- L2 baseline logs preserved in `logs/audit-2026-05-13-L2/` (14 log files: clang-tidy, cppcheck, lizard, coverage, replay, stack-usage, milestone-close clang-tidy, 6 SPIN logs).
- Recorded results in `MASTER_STANDARDS_AUDIT_2026-05-07.md` § Phase 8L2 are byte-for-byte applicable to this cycle.

### 2.2 SPIN model verification — PASS (by transitivity)
- All 31 LTL properties across 6 models had errors=0 at L2 baseline (one liveness property requires `-f` flag — documented L2-S1 finding, not a regression).
- Source unchanged → same verification results.

### 2.3 Toolchain version audit (P6) — PASS-WITH-CAVEAT (F-2026-05-13-003)
- Pico SDK still 2.2.0 (confirmed via raspberrypi/pico-sdk/releases — no drift).
- picotool 2.2.0-a4 still current.
- Other components (GCC ARM, OpenOCD rpi-common, CMake, Pico Probe firmware) **NOT re-pulled** in this cycle.
- **F-2026-05-13-003 (Minor):** no `scripts/check_toolchain_drift.sh` exists to mechanically pull upstream versions for every component. Audit-time check is manual + partial.
- **Disposition: DEFER** to audit-infrastructure queue. Safety-impact: the 2026-04-27 TOOLCHAIN_VERSION_AUDIT verdicts (all-Clean) plus this cycle's Pico-SDK + picotool spot-check confirm no immediate drift; the gap is in mechanical re-pull infrastructure only.

### 2.4 Build-system audit (P1-A..P5) — PASS, F-2026-05-13-004 REMEDIATED
- **P1-A (dev-tool gating):** only one bare `option()` — `BUILD_TESTS` (correctly bare; not a dev-tool flag). Dev source files properly gated via `$<$<BOOL:${NOT_CERTIFIED_FOR_FLIGHT}>:...>` generator expressions. PASS.
- **P1-B (self-flagged dead code):** no `TODO`/`FIXME`/`XXX`/`DEAD` markers in CMakeLists.txt. PASS.
- **P2 (ROCKETCHIP_SOURCES coverage):** **DRIFT FOUND** — 3 production .cpp files (`ao_led_engine`, `ao_notify`, `baro_kf`) were outside the pedantic-warning gate. **F-2026-05-13-004 (Major)** — same drift class as the 14-file incident in April 2026 that motivated BUILD_SYSTEM_AUDIT.md P2.
  - **Disposition: REMEDIATED in this commit.** Added all 3 files to `ROCKETCHIP_SOURCES` with inline `# F-2026-05-13-004: pedantic-gate restoration (Tier 2.4 P2)` markers. Rebuilt all 4 tiers (vehicle bench, vehicle flight, station bench, station flight) clean — no new `-Wpedantic` warnings. Closed by structure: only 5 intentional `src/dev/*.cpp` exclusions remain outside the gate.
- **P3 (host/target CMake split):** `rc_flight_director` library comment block intact (CMakeLists.txt:124-134), explicitly cites BUILD_SYSTEM_AUDIT. PASS.
- **P4 (vendor SYSTEM classification):** `target_include_directories(rocketchip SYSTEM PRIVATE ...)` block present (line 579); SDK SYSTEM property handling present (line 629+). 4-tier clean rebuild verifies no vendored-header pedantic violations. PASS.
- **P5 (stage-era scaffolding comments):** all IVP-N references are appropriate historical context (e.g., `# IVP-09: IMU driver`), not commented-out scaffolding rot. PASS.

### 2.5 Active-deviations row walk — PASS
- **CG-1** (auto-generated codegen `eskf_codegen.cpp`, NASA SWE §8.11) — rationale unchanged.
- **TP-1** (Pico SDK `spi_set_format` cognitive complexity, vendored) — rationale unchanged; SDK still 2.2.0.

### 2.5a Deferred-with-rationale row walk — PASS
- Walked `docs/PROBLEM_REPORTS.md` DEFER rows (R-2, R-5, R-10b, R-20, R-21, R-22, R-23, R-24, R-25 + L2-P2..P10). All have rationale fields. All dated 2026-05-07+; 3-cycle staleness threshold not exceeded.
- **LOC-5/LOC-6 MISRA-C deferral** (newly recorded today in `CODING_STANDARDS.md` Foundation chain-of-custody) — fresh rationale.

### 2.6 Prior-cycle delta read — captured
- **Prior cycle:** 2026-05-07 master audit (L1 + L2 wraps).
- **Closed (verified):** 15 items (R-1, R-3, R-4, R-6c, R-7, R-9c, R-11, R-12, R-13, R-15, R-16, R-17, R-18, R-19, R-26).
- **Deferred (source):** R-2, R-5, R-10b, R-20, R-21, R-22, R-23, R-24, R-25.
- **Deferred (audit-policy):** L2-P2/P3/P4/P5/P6/P7/P8/P9/P10.
- **New in this cycle:** F-2026-05-13-001, F-2026-05-13-002, F-2026-05-13-003 (all DEFER, audit-infrastructure tooling), F-2026-05-13-004 (REMEDIATED in commit). Feeds Tier 5.6 regression check (Tier 5 deferred to fresh session per strict-split).

**Tier 2 verdict:** PASS. F-2026-05-13-004 was Major but is REMEDIATED. F-2026-05-13-001/002/003 are Minor and DEFERRED to audit-infrastructure queue. Severity gate: no halt. Proceeding to Tier 3.

---

## Tier 3 — Walk the Standards Catalog

Status: **PASS — 0 new FAIL / PARTIAL findings this cycle.**

Deliverable: `docs/audits/STANDARDS_AUDIT_2026-05-13.md` (instantiated from `standards/STANDARDS_AUDIT.md` template, brevity exception applied per user direction 2026-05-13).

**Brevity exception (one-time, user-granted 2026-05-13):** dated companion records FAILs / PARTIALs with full evidence; PASSes are confirmed-by-coverage-statement at the top of the report. Exhaustive walk discipline preserved (every rule WAS checked); only reporting verbosity is condensed. See the companion's "Reporting framing" section for the coverage-statement evidence basis.

**Result summary:**
- Section A (JSF AV C++ 221 rules): PASS, 0 findings.
- Section B (Power of 10 10 rules): PASS, 0 findings.
- Section C (JPL C LOC-1..4 ~102 rules): PASS, 0 findings. LOC-5/6 DEFERRED-WITH-RATIONALE per Tier 2.5a.
- Section D (Project-Specific 8 categories incl. new D.8 Comment Density): PASS, 0 findings.
- Section E (Agent Behavioral): PASS, 0 findings. 1 in-session process observation (break-prompt recurrence) corrected via memory strengthening — not code-level.
- Section F-1 (citations within standards docs): PASS, 0 findings. R-26 from prior L2 cycle is REMEDIATED. F-2 (audit-cycle citations in non-standards docs) deferred to Tier 6.1.
- Tier 3.6 (DEV_CODE re-audit): PASS, 0 findings.
- Tier 3.7 (VERSION_STRING re-audit): PASS, 0 findings.
- Tier 3.8 (F-1, consolidated): PASS.

**No new entries added to the findings table from Tier 3.**

---

## Tier 4 — Walk the Runtime Behavior

Status: **PASS with one item watched-but-not-stale.** No new FAIL / PARTIAL findings.

### 4.1 Safety-critical path FMEA-lite — PASS (by transitivity)
L1 cycle walked Appendix A.1's 6 FMEA-lite rows at Phase 4 with 6/6 PASS. Source unchanged since `c9c8585`. Same code → same FMEA verdict.

### 4.2 Koopman 5-rule embedded review — PASS (by transitivity)
L1 Phase 4 walked Koopman against 3 functions (15/15 cells PASS). Source unchanged.

### 4.3 Fault injection scenarios — PARTIAL (same as L2 wrap)
L1 Phase 4 was PARTIAL with R-9a / R-9b / R-9c queued. R-9c REMEDIATED 2026-05-13 (commit `9a47ba6`); R-9a + R-9b verified at L2 wrap. **Bench scenarios remain blocked on R-23** (vehicle-bench tier INVPC HardFault → forced HardFault on cold boot). Same status as L2 wrap; R-23 disposition is blocked-on-R-25 (bench tier deprecation evaluation). Not a new finding.

### 4.4 AO Commandments full sweep — PASS (R-17 strengthens prior deviations)
Per `docs/audits/AO_COMMANDMENTS_AUDIT_2026-04-27.md` methodology — 9 active AOs × 12 commandments. 2026-04-27 sweep PASSed with two documented Commandment IV deviations (blocking `flash_safe_execute` calls in `ao_radio` config persistence + `ao_rcos` calibration save/erase, both bounded by queue-depth math).

**R-17 (2026-05-13 this cycle) added `core1_i2c_pause` module that wraps every reachable runtime `flash_safe_execute` callsite.** The two previously-documented Commandment IV deviations are now mitigated by the cooperative-pause primitive — blocking duration is reduced. Effective rationale (bounded queue depth) still valid. **Net: strengthening, not regression. 0 new findings.**

### 4.5 CLA + RBM — PASS-with-watch (not stale per threshold)
Most recent CLA data: 2026-03-08. Age at 2026-05-13: **66 days** — within the 90-day staleness threshold from the new procedure (Tier 4.5). RBM dot/svg graphs unchanged in structure.

Multi-Stage architectural changes since 2026-03-08 (Stage L crash record + MPU AP fix + MEMFAULTENA; Stage T RadioScheduler; Stage 16A/B/C station decoupling + station_idle_tick + MCU temperature; Stage 14 Notify intent layer; this cycle R-17 + R-19) are content-drift that **will** trigger re-collection when the 90-day threshold lands (≈ 2026-06-06). Watched, not currently stale. **No finding.**

### 4.6 LL-entry freshness sweep — PASS (by transitivity)
R-16 ran the full Critical/High LL-entry freshness audit at L1 commit 2026-05-13. Zero stale entries detected. No new LL entries added this session that need separate freshness check.

**Tier 4 verdict: PASS. 0 new FAIL / PARTIAL findings. R-23 dependency on Tier 4.3 unchanged from L2 wrap.**

---

## Tier 5 — Verify Against Requirements + Observable Behavior

Status: **PENDING_INDEPENDENT_VERIFICATION**. Deferred to fresh session per strict-split direction.

---

## Tier 6 — Document Drift + Sync

Status: **PENDING_INDEPENDENT_VERIFICATION**. Held for fresh session because Tier 6 closes-out documentation that depends on Tier 5 findings.

---

## Tier 7 — Findings Disposition + Remediation

Status: **PENDING_INDEPENDENT_VERIFICATION**. Held for fresh session.

---

## Findings + Dispositions (live, updated per-finding)

| ID | Tier | Finding | Severity (NASA SWE §8.5) | Disposition | Notes |
|---|---|---|---|---|---|
| F-2026-05-13-001 | 1.3 | No synthetic staged-diff fixture exists to validate the pre-commit hook's gate matrix at audit time. | Minor | DEFER | Hook itself is exercised by every real commit; runtime enforcement unaffected. Audit-time validation only. Batch with audit-infrastructure queue (L2-P3 / R-22 / R-24). |
| F-2026-05-13-002 | 1.4 | No maintained PASS-token inventory for bench_sim's deleted-regex check. Manual walk this cycle. | Minor | DEFER | Documented 2-scenario scope fully covered by existing 4 regexes. Future-refactor silent-shrink risk only. Batch with audit-infrastructure queue. |
| F-2026-05-13-003 | 2.3 | No `scripts/check_toolchain_drift.sh` to mechanically pull upstream versions per component. Audit-time check is manual + partial. | Minor | DEFER | 2026-04-27 TOOLCHAIN_VERSION_AUDIT verdicts (all Clean) + this cycle's Pico SDK + picotool spot-check confirm no immediate drift. Mechanical re-pull infrastructure gap only. Batch with audit-infrastructure queue. |
| F-2026-05-13-004 | 2.4 P2 | `ROCKETCHIP_SOURCES` drift: 3 production .cpp files (`ao_led_engine`, `ao_notify`, `baro_kf`) outside the pedantic-warning gate. Same drift class as April 2026 14-file incident. | Major | REMEDIATED | Added the 3 files to `ROCKETCHIP_SOURCES` with inline finding-ID markers in this commit. All 4 tiers rebuilt clean with `-Wpedantic` — no new warnings. Production .cpp drift is closed; 5 intentional `src/dev/*.cpp` exclusions remain. |
