# AUDIT_GUIDANCE.md Rewrite Proposal — 2026-05-13

**Status:** Council-reviewed 2026-05-13. CONSENSUS APPROVE WITH AMENDMENTS. Awaiting protected-file edit.

**Council:** NASA/JPL Avionics Lead, ArduPilot Core Contributor, Embedded Systems Professor, Cubesat Startup Engineer (Cubesat substituted for Senior Aerospace Student per user direction 2026-05-13).

**Council amendments (8) incorporated below.** See "Council Amendments" section at end of doc for the verbatim verdict.

**Source:** Whiteboard "Master Standards Audit 2026-05-07 — audit-coverage gap re-evaluation" step 1a. Inventory completeness check in `AUDIT_COVERAGE_INVENTORY_2026-05-13.md` surfaced 15 gap items requiring either (a) folding into the master audit procedure or (b) documented-deferral.

**Premise:** The current 8-step `AUDIT_GUIDANCE.md` procedure reads as a flat checklist of audit *types* with no dependency ordering between steps. The Appendix C remediation triage (4-category PR ordering, established 2026-05-12, sourced from DO-178C problem-report lifecycle + static-analysis-tool community / SonarQube merge-blocker-vs-backlog distinction + refactoring research on dependency cycles + NASA SWE §8.5 severity scale + project-internal LL Entry 36 gate-integrity-first lived experience) introduced explicit dependency ordering for remediation work but did not refactor the audit-procedure steps themselves. This proposal extends the same triage logic from remediation (Appendix C) into the audit procedure (the main body of `AUDIT_GUIDANCE.md`).

---

## Why reorder the audit procedure?

The current step ordering has three concrete problems surfaced by the 2026-05-07 audit cycle:

1. **Tools-honesty assumption is implicit, not gated.** Current Step 1 ("Baseline Scripted Sweeps") runs tools and reports verdicts. There is no explicit step that asks "are the tools themselves honest?" before running them. LL Entry 36 (bench_sim regex rot — gate had been silently broken for 5 days while 4+ commits claimed it as PASS) is the documented lived-experience case for why this matters. Appendix C category 1 ("Gate Integrity") addresses this for *remediation*, but the audit procedure itself doesn't have a parallel gate.

2. **Static analysis and runtime analysis sit at equal depth without dependency.** Current Step 2 (Coding Standards Deep Compliance) and Step 4 (Safety-Critical Path Review / FMEA-lite + Koopman) operate at the same level. In practice, the runtime-behavior findings should be informed by what static analysis surfaces (a Koopman review of an unbounded-loop case is more focused once Section A of the standards walk has flagged the parent function); and the standards walk should not run on code paths that runtime work has already invalidated.

3. **Sibling audits have no home.** `BUILD_SYSTEM_AUDIT.md`, `DEV_CODE_AUDIT.md`, `VERSION_STRING_AUDIT.md`, `AO_COMMANDMENTS_AUDIT_2026-04-27.md`, `TOOLCHAIN_VERSION_AUDIT_2026-04-27.md`, and `cla_rbm/` exist as fully-realized periodic audits but are not enumerated as phases in `AUDIT_GUIDANCE.md`. They are referenced only from `SESSION_CHECKLIST.md` (item 15) or from CHANGELOG narrative. This is the L2-P6 / L2-P8 / L2-P9 / L2-P10 / L2-P13 gap cluster.

The proposed reorder is a **structural rewrite**, not a sub-bullet edit. It applies Appendix C's triage logic (gate integrity → shared foundations → behavior changes → cleanup) to the audit steps themselves, with three additions specific to *audit work* (vs. remediation work): a verification-against-running-system tier between behavior-change findings and final doc-sync; an explicit standards-catalog walk tier (the JSF/JPL/P10 catalog work that Appendix C doesn't address because remediation isn't catalog-driven); and a runtime-behavior tier (FMEA + Koopman + CLA-RBM + AO Commandments) that aggregates the runtime-shaped sibling audits.

---

## Proposed 7-tier structure

The new audit procedure has seven tiers. Each tier closes before the next begins (same discipline as Appendix C's category transitions). Within a tier, items can run in parallel.

### Tier 1: Verify the Audit's Own Gates

**Purpose:** Before measuring anything, confirm the measurement instruments are honest. Source: LL Entry 36 + Appendix C category 1.

- 1.1 Tool self-checks: clang-tidy, cppcheck, lizard, coverage harness produce expected output on known-good fixture inputs.
- 1.2 SPIN models compile + load on the current build of `spin.exe` (Cygwin environment per [SPIN must run via Cygwin bash](file:///c:/Users/pow-w/.claude/projects/c--Users-pow-w-Documents-Rocket-Chip/memory/debugging_spin_cygwin_environment.md) memory).
- 1.3 Pre-commit hook integrity: `scripts/hooks/pre-commit` runs on a synthetic staged diff and produces the expected gate matrix output.
- 1.4 `bench_sim.py` regex audit (**bidirectional** per council amendment #1):
  - **(a) Rotted-regex check:** every regex constant in the script matches at least one live firmware log line from a freshly-flashed banner read. Catches the LL Entry 36 regex-rot pattern *before* the script is trusted.
  - **(b) Deleted-regex check:** every PASS-token / positive-control signal the firmware emits has at least one regex in the script looking for it. Catches the inverse case where a regex was silently removed during a refactor — the script passes because nothing fails, but it's no longer actually verifying the path.
- 1.5 Host ctest fixture-suite passes (788 → 794 → current count, all PASS).
- 1.6 Build-parity script (`verify_build_parity.sh`) produces clean output for all 4 tiers.

**Stop condition:** any Tier 1 failure halts the cycle until the tool/gate is repaired. Subsequent tiers cannot produce reliable findings on broken instruments.

**Positive-control expectation:** every tool produces its expected self-test output AND the output's positive-control signal is observable in serial / file / GDB.

---

### Tier 2: Establish the Foundation

**Purpose:** With instruments verified honest in Tier 1, capture the baseline data the rest of the audit reasons from. Source: Appendix C category 2 ("Shared Foundations") + traditional "baseline scripted sweeps" framing.

- 2.1 Baseline scripted sweeps: `run_clang_tidy.sh`, `run_cppcheck.sh`, `analyze_stack_usage.sh`, `generate_coverage_report.sh`, lizard cyclomatic complexity sweep. Record verdicts + raw output paths.
- 2.2 SPIN model verification: re-run all `tools/spin/*.pml` models against current firmware behavior. Master gate `run_stage_o_ao_spin.sh` should produce SPIN_OK_N for the current N (was 31 as of 2026-05-13).
- 2.3 Toolchain version audit: walk `BUILD_SYSTEM_AUDIT.md` P6 (current versions vs latest upstream, drift assessment). Absorbs `TOOLCHAIN_VERSION_AUDIT_*.md` sibling audit.
- 2.4 Build-system audit: walk `BUILD_SYSTEM_AUDIT.md` P1-A through P5 (dev-tool gating, self-flagged dead code, ROCKETCHIP_SOURCES coverage, host/target split, vendor SYSTEM classification, IVP-era scaffolding comments). Absorbs the L2-P13 gap.
- 2.5 Active-deviations row walk: walk `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` Active section row-by-row, confirm each row's justification still holds (the work formerly done at Phase A.2 of the 2026-05-07 cycle).
- 2.5a **Deferred-with-rationale row walk** (council amendment #4): in parallel to 2.5, walk every deferred-with-rationale row across the audit machinery (LOC-5 / LOC-6 MISRA-C deferral; any DEFER row in active dated audit reports' `## Remediation` sections; deferred-with-safety-impact rows in `PROBLEM_REPORTS.md`). Confirm each row's rationale still holds. Three cycles without re-evaluation is the stale-rationale threshold; flag for explicit user re-disposition if exceeded.
- 2.6 **Prior-cycle delta read** (council amendment #5, sourced from JPL F' Mars sustaining-engineering pattern): open the most recent prior dated audit report. List which findings closed, which remained open, which were re-opened, which are new. This output feeds Tier 5.6 (regression check) and informs Tier 7 disposition.

**Stop condition:** Tier 2 findings that invalidate Tier 1's verified-gates assumption (e.g., toolchain version audit reveals the static analyzer is from an unsupported branch) AND the finding's severity is Catastrophic or Critical per NASA SWE §8.5 → return to Tier 1 to re-verify. Major/Minor/Trivial findings flow forward to Tier 7 disposition without re-walking Tier 1 (council amendment #3 — severity gate prevents tar-pit pattern).

**Positive-control expectation:** complete baseline-data set captured + ACCEPTED_DEVIATIONS row walk complete.

---

### Tier 3: Walk the Standards Catalog

**Purpose:** Mechanical catalog work. The big static-analysis pass against the published rule sets the project has adopted. Source: traditional `STANDARDS_AUDIT.md` template walk (which the 2026-02-07 audit did fully but the 2026-05-07 cycle skipped Sections A/C/D/E/F/G/H of, surfaced as the L2-P5/P12 gap).

- 3.1 **Section A — JSF AV C++ Standards (221 rules).** Per-rule applicability + PASS/PARTIAL/FAIL + file citations. A.1 (Not-Applicable rules — C files) acknowledges absent .c files. A.2 (SDK Interface Constraints) documents which rules are bypassed by SDK boundary calls. A.3 (Applicable Rules Audit Tables) is the bulk of the work.
- 3.2 **Section B — Power of 10 (Holzmann/JPL 2006).** Per-rule applicability. 10 rules total; mostly mechanical. This is the only Section the 2026-05-07 cycle walked.
- 3.3 **Section C — JPL Institutional Coding Standard.**
  - LOC-1 through LOC-4: per-rule walk (102 rules total = 10 + 7 + 12 + 73). These are JPL-original tiers.
  - LOC-5 and LOC-6: **DEFERRED-WITH-RATIONALE.** These are the MISRA-C absorption tiers (LOC-5 = MISRA "shall" rules not in LOC-1..4; LOC-6 = MISRA "should" rules). The rule text is MIRA Ltd. copyrighted and not in the public JPL PDF. Project policy: defer until a formally-certified-code variant is in scope. Source: 2026-05-13 research + user-direction.
- 3.4 **Section D — Project-Specific Rules.** RP2350 platform constraints, multicore rules, debug output rules, prior-art-research rules, safety-and-regulatory rules, git-workflow rules, session-management rules, **comment-density measurement (target band 15-25% per CODING_STANDARDS.md)**. Absorbs L2-P14 comment-density audit gap.
- 3.5 **Section E — Agent Behavioral Guidelines.** Walk `standards/AK_GUIDELINES.md` per-rule. Audit deliverable is whether the agent's recent commits demonstrate compliance.
- 3.6 **DEV_CODE audit** (dev-tier code in flight binary): re-run the `DEV_CODE_AUDIT.md` methodology. Absorbs L2-P6 gap.
- 3.7 **VERSION_STRING audit**: walk `VERSION_STRING_AUDIT.md` for stale version-like values in serial output / banner. Absorbs L2-P7 gap.
- 3.8 **Section F-1 — Citations within standards documents** (council amendment #2): walk file:line references and standards-citations inside the standards documents themselves (`CODING_STANDARDS.md`, `HW_GATE_DISCIPLINE.md`, `AUDIT_GUIDANCE.md`, `RP2350_ERRATA.md`, `ACCEPTED_STANDARDS_DEVIATIONS.md`). Catalog-shaped catch — citation rot inside the rules themselves. (F-2, the audit-cycle citations in PROJECT_STATUS / SAD / CHANGELOG, stays in Tier 6.)

**Stop condition:** Tier 3 finds a FAIL on a rule that invalidates Tier 2's baseline assumptions AND the finding's severity is Catastrophic or Critical per NASA SWE §8.5 → return to Tier 2. Major/Minor/Trivial flow forward to Tier 7 disposition.

**Positive-control expectation:** dated `STANDARDS_AUDIT_YYYY-MM-DD.md` produced with Sections A through E populated. F-G-H are Tier 6.

---

### Tier 4: Walk the Runtime Behavior

**Purpose:** Now examine what the code *does* at runtime, not just its static structure. Source: existing Step 4 (FMEA-lite + Koopman) + AO Commandments audit + CLA-RBM + LL-entry freshness.

- 4.1 **Safety-critical path FMEA-lite** (existing `AUDIT_GUIDANCE.md` Appendix A.1). Maps to `docs/SAD.md` state machine. Positive-control column required per `HW_GATE_DISCIPLINE.md` Rule 1.
- 4.2 **Koopman 5-rule embedded review** on 3-5 flight-critical functions of agent's choice (auditor selects which, agent surfaces conversationally for user disposition per Appendix B.4 conversational workflow).
- 4.3 **Fault injection scenarios**: run `enhanced_fault_injection.py` end-to-end (this was Phase 4 PARTIAL in the 2026-05-07 cycle; remediation R-9a/b/c queued).
- 4.4 **AO Commandments full sweep**: walk 9 active AOs (Radio, FlightDirector, HealthMonitor, RfManager, Notify, Logger, Telemetry, LedEngine, RCOS) against all 12 commandments in `docs/decisions/AO_COMMANDMENTS.md`. Absorbs L2-P8 gap.
- 4.5 **CLA + RBM (Computational Load Analysis + Runtime Behavior Map)**: walk `docs/audits/cla_rbm/CLA_RBM_PLAN.md` deliverables. If the most-recent CLA data is >3 months old, the analysis section flags re-collection as a remediation item (CLA re-collection requires HW soak, deferred to a separate session if needed). Absorbs L2-P10 gap.
- 4.6 **LL-entry freshness sweep**: walk `docs/agents/LESSONS_LEARNED.md` Critical/High entries. Each entry's claim ("X is now fixed in tree", "Y workaround is in place") must be verified against current code. Stale entries get supersession headers per `LESSONS_LEARNED.md` format (e.g., LL Entry 25's "SUPERSEDED 2026-04-22" header is the pattern). Absorbs L2-P15 / R-16 gap.

**Stop condition:** Tier 4 finds a runtime-behavior FAIL that contradicts Tier 3's static-analysis PASS for the same code path AND the finding's severity is Catastrophic or Critical per NASA SWE §8.5 → re-open Tier 3 for that path. Major/Minor/Trivial findings feed forward to Tier 7 with the contradiction noted.

**Positive-control expectation:** every safety-critical-path row in Appendix A.1 shows a clear positive-control signal in log/p-output. AO Commandments sweep produces a complete matrix. CLA-RBM analysis is current (or re-collection is flagged as a queued remediation).

---

### Tier 5: Verify Against Requirements + Observable Behavior

**Purpose:** Now that the static work (Tier 3) and runtime work (Tier 4) are clean, prove the running system does what its requirements say it does. Source: existing Steps 3 + 6 + 7.

**Independence amendment (council #7):** Tier 5 should run in a **fresh session** distinct from Tiers 1-4 where feasible. Gets toward DO-178C verification-independence at zero tooling cost. Especially relevant given Stage 17 (first motor flight) is the upcoming milestone — the same agent walking the verification it just performed is level-2 credit at best per HW_GATE_DISCIPLINE Rule 6. A separate session pass is level-3 ("Verified independently").

- 5.1 **Pre-flight gate execution**: walk `PRE_FLIGHT_CHECKLIST.md` in full. Every GO verdict + positive-control signal observed.
- 5.2 **Stack / Memory / RP2350 Errata Deep Review** (existing `AUDIT_GUIDANCE.md` Appendix A.2). E2 / E9 / E11 / E12 re-verification.
- 5.3 **Bench end-to-end**: `bench_sim.py` 2/2 PASS for vehicle (flight + dev tiers); `station_bench_sim.py` for station (when station bench HW available).
- 5.4 **Replay gate test**: `replay_gate_test.py` end-to-end. NOTE: BLOCKED on R-23 (vehicle bench tier INVPC HardFault) until R-25 disposition. This is the L2-P19 gap; documented but cannot be cleared this cycle.
- 5.5 **Requirements traceability spot-check**: state machine, ESKF outputs, pyro commands, telemetry fields, safety flags all have traceable requirements. **Exhaustive coverage required** against the distinct-cited-source population (the 2026-05-07 L2 audit demonstrated exhaustive is feasible and that sampling would miss findings like R-26). Per L2-P2 disposition.
- 5.6 **Regression check on prior-cycle closed findings** (council amendment #6, sourced from ArduPilot sustaining-engineering pattern): every finding closed in a prior audit cycle (drawn from Tier 2.6's prior-cycle delta read) gets a one-line regression check. If R-19 closed by removing the FAULT timestamp from boot banner in cycle N, then in cycle N+1 Tier 5.6 confirms the banner still doesn't have it. Catches re-do work and silent regression in subsequent refactors.

**Stop condition:** Tier 5 finds a verification FAIL that contradicts Tier 3/4 PASS AND the finding's severity is Catastrophic or Critical per NASA SWE §8.5 → return to whichever tier first claimed PASS. Major/Minor/Trivial findings feed forward to Tier 7.

**Positive-control expectation:** every pre-flight GO confirmed, errata still mitigated, bench scripts PASS, requirements traceability table CONFIRMED for all rows.

---

### Tier 6: Document Drift + Sync

**Purpose:** Catch documentation that the earlier tiers' work has invalidated. Last because earlier tiers may surface drift findings (e.g., this cycle's I-1 stale citation in `PROJECT_STATUS.md`).

- 6.1 **Section F-2 — Audit-cycle citations in non-standards docs** (per council amendment #2): Walk cited file paths, line numbers, audit citations in protected state-of-system docs OUTSIDE the standards documents themselves — `PROJECT_STATUS.md`, `SAD.md`, `CHANGELOG.md`, `SCAFFOLDING.md`, `AO_ARCHITECTURE.md`. Catches I-1-class findings (stale citations to deleted dated audits). F-1 (citations *within* standards docs) is Tier 3.8.
- 6.2 **Section G — Audit History** (template). Roll up this cycle's findings into the audit-history table.
- 6.3 **Section H — Ongoing Compliance Verification** (template). Walk pre-commit hook coverage, SESSION_CHECKLIST trigger coverage, milestone-close discipline coverage.
- 6.4 **Protected-doc drift check** (existing SESSION_CHECKLIST.md item 14). Grep state-of-system docs for symbols / module paths that this audit's findings have deleted, renamed, or re-homed.
- 6.5 **CHANGELOG audit-history rollup**: confirm this cycle's CHANGELOG entry references the dated audit report + names every disposition (REMEDIATE / ACCEPT / DEFER) without ambiguity.

**Stop condition:** Tier 6 surfaces drift that invalidates a Tier 5 PASS AND the finding's severity is Catastrophic or Critical per NASA SWE §8.5 (rare; would mean a doc claimed a verification that didn't happen) → return to Tier 5 for the affected row. Major/Minor/Trivial drift findings feed forward to Tier 7 disposition.

**Positive-control expectation:** dated audit report Sections F + G + H populated. CHANGELOG entry written. Protected-doc drift check complete.

---

### Tier 7: Findings Disposition + Remediation

**Purpose:** Existing Phase 8 structure preserved verbatim. Source: Appendix C (4-category remediation triage, unchanged).

- Same as current Step 8. Findings disposition (REMEDIATE / ACCEPT / DEFER) per finding. Per `Appendix C`, remediation execution order is: Cat 1 (Gate Integrity) → Cat 2 (Shared Foundations) → Cat 3 (Behavior Changes) → Cat 4 (Cleanup). DEFER rows require safety-impact one-liners. ACCEPTED deviations require user sign-off and land in `ACCEPTED_STANDARDS_DEVIATIONS.md`. Verification cadence per Appendix C.5 (local per-commit vs. category-transition regression vs. cycle-close regression).

---

## Decision-table mapping (current → proposed)

| Trigger | Current procedure says | Proposed procedure says |
|---|---|---|
| Every commit (small change) | Step 2 (coding standards) on changed files only | Tier 3 sub-walks (A/B/C/D as applicable to changed files); pre-commit hook is Tier 1.3 mechanism |
| Before push | Build parity + targeted | Tier 2.6 (build-parity) + relevant Tier 3 sub-walks |
| Before field test / launch | Step 3 + Step 4 | Tier 5.1 (pre-flight) + Tier 4 (full runtime walk if scope warrants) |
| Milestone / stage close | Full master procedure | **Full 7-tier walk** |
| Safety-critical or architecture change | Steps 2, 4, 5, 6, 7 scoped to affected modules | Tiers 3 + 4 + 5 scoped to affected modules; Tier 1 mandatory regardless |
| After failed audit or remediation | Re-execute failed steps | Re-enter at the tier the failure originated; later tiers re-run if affected |

---

## Sibling-audit absorption matrix

| Sibling audit | Currently lives in | Folded into proposed tier | Notes |
|---|---|---|---|
| `BUILD_SYSTEM_AUDIT.md` | docs/, referenced by SESSION_CHECKLIST.md item 15 | Tier 2.4 (P1-A..P5) + Tier 2.3 (P6 = toolchain) | Stays as standalone doc; AUDIT_GUIDANCE links to it as Tier 2 sub-step |
| `TOOLCHAIN_VERSION_AUDIT_2026-04-27.md` | docs/audits/, P6 of BUILD_SYSTEM_AUDIT | Tier 2.3 | Methodology stays in BUILD_SYSTEM_AUDIT; dated reports continue in docs/audits/ |
| `DEV_CODE_AUDIT.md` | docs/audits/ | Tier 3.6 | Methodology stays in DEV_CODE_AUDIT; per-cycle pass produces dated companion if findings change |
| `VERSION_STRING_AUDIT.md` | docs/audits/ | Tier 3.7 | Same pattern |
| `AO_COMMANDMENTS_AUDIT_2026-04-27.md` | docs/audits/ | Tier 4.4 | Methodology grounded in `docs/decisions/AO_COMMANDMENTS.md`; dated reports in docs/audits/ |
| `cla_rbm/` subfolder | docs/audits/cla_rbm/ | Tier 4.5 | Plan + analysis + dot/ stay in cla_rbm/; AUDIT_GUIDANCE links Tier 4.5 to the plan |
| Comment-density audit | CHANGELOG narrative + CODING_STANDARDS section | Tier 3.4 (Section D measurement row) | New row in STANDARDS_AUDIT.md template Section D |
| LL-entry freshness audit (R-16) | CHANGELOG narrative only | Tier 4.6 | Becomes part of every milestone audit |

---

## CHANGELOG strategy (council amendment #8 — supersedes earlier draft)

**Per council unanimous (Cubesat + Professor specifically): DO NOT edit past CHANGELOG entries.** Editing history erodes the bisectability of `git log` over historical-record files and sets a precedent against the project's existing `SESSION_CHECKLIST.md` trigger map (CHANGELOG is historical-record / append-only).

**Pattern instead:** single forward-going CHANGELOG entry dated 2026-05-13 titled "Audit procedure refactored: 8-step → 7-tier structure with severity-gated stop conditions." The entry includes a **Step → Tier mapping table** so future readers walking old CHANGELOG entries can translate the "Step N" references into the current tier structure. This matches the inline-reference pattern that LL Entry 25's "SUPERSEDED 2026-04-22" header uses — the historical entry is not edited; a new entry references it and supersedes.

**Step → Tier mapping table** (for the new CHANGELOG entry):

| Old "Step" reference | New tier reference |
|---|---|
| Step 1 (Baseline Scripted Sweeps) | Tier 2 (Establish the Foundation) |
| Step 2 (Coding Standards Deep Compliance) | Tier 3 (Walk the Standards Catalog) |
| Step 3 (Pre-Flight Gate Execution) | Tier 5.1 |
| Step 4 (Safety-Critical Path Review) | Tier 4 (Walk the Runtime Behavior) |
| Step 5 (Stack / Memory / Errata) | Tier 5.2 |
| Step 6 (Formal Verification + Simulation Coverage) | Tier 5.3 + 5.4 (and Tier 1.2 for the SPIN self-check) |
| Step 7 (Requirements Traceability) | Tier 5.5 |
| Step 8 (Remediation & Historical Logging) | Tier 7 (unchanged in structure; Appendix C 4-category triage preserved) |
| (Phase A.2 — Accepted Deviations review) | Tier 2.5 (active) + Tier 2.5a (deferred-with-rationale) |
| (Pre-2026-05-07 inline "verify the gates" assumption) | Tier 1 (now explicit) |
| (No prior equivalent) | Tier 2.6 (prior-cycle delta read — new per council #5) |
| (No prior equivalent) | Tier 5.6 (regression check on prior closed findings — new per council #6) |

**Non-record references (forward-going only):** `SESSION_CHECKLIST.md` items 6 / 14 / 15 / 16 / 17 references to "Steps", `README.md`, `AGENT_WHITEBOARD.md` audit-coverage row — all updated to "Tier N" wording in the same commit that lands the AUDIT_GUIDANCE rewrite. Historical-record files (CHANGELOG, LESSONS_LEARNED, dated audit reports in `docs/audits/`) are NOT updated; the new forward-going CHANGELOG entry above is the supersession reference.

---

## Questions for council review (answered)

Council personas: NASA/JPL Avionics Lead, ArduPilot Core Contributor, Embedded Systems Professor, Cubesat Startup Engineer (Cubesat substituted for Senior Aerospace Student per user direction 2026-05-13).

1. **Real dependency ordering** — stop-conditions are topological back-edges. Confirmed by JPL Lead (DO-178C objective-table-backtrace analog) + Professor (build-system DAG analog).
2. **Tier 1 genuinely separable** — instruments verify against fixtures, not baselines. (JPL Lead)
3. **Section F splits** — F-1 (citations *within* standards docs) → Tier 3.8; F-2 (audit-cycle citations in non-standards docs) → Tier 6.1. (Professor)
4. **Tier 4.6 stays in Tier 4** — LL freshness is code-shaped (verify Z is in tree), not doc-shaped. (Professor)
5. **Tier 4 / Tier 5 stay split** — Tier 4 = behavior model (FMEA, AO commandments, what *should* happen); Tier 5 = actual binary on actual hardware observed. Different observational stances. (ArduPilot)
6. **Missing items, now added:** Tier 2.5a (deferred-rationale walk), Tier 2.6 (prior-cycle delta), Tier 5.6 (regression check on prior closed findings), Tier 5 independence amendment. (Professor + JPL + ArduPilot)
7. **Stop-conditions tight enough with severity gate** (Catastrophic/Critical only halt; Major/Minor/Trivial flow forward); without the gate the procedure becomes the tar pit it's meant to prevent. (Cubesat)

---

## Council Amendments — Final

Council session 2026-05-13 produced the following 8 amendments, all incorporated above:

1. **Tier 1.4 bidirectional regex check** (ArduPilot) — verify both directions: regexes match live log lines AND every PASS-token has at least one regex looking for it.
2. **Tier 3 split for Section F** (Professor) — F-1 → Tier 3.8 alongside A-E; F-2 → Tier 6.1.
3. **Severity-gated stop conditions** (Cubesat + JPL) — only NASA SWE §8.5 Catastrophic/Critical findings halt forward progress. Major/Minor/Trivial flow forward.
4. **Tier 2.5a deferred-with-rationale row walk** (Professor) — parallel to active-deviations walk; 3-cycle stale-rationale threshold.
5. **Tier 2.6 prior-cycle delta read** (JPL — F' Mars sustaining-engineering pattern).
6. **Tier 5.6 regression check on prior-cycle closed findings** (ArduPilot sustaining-engineering pattern).
7. **Tier 5 independence amendment** (JPL) — Tier 5 should run in a fresh session distinct from Tiers 1-4 where feasible. DO-178C verification-independence at zero tooling cost.
8. **CHANGELOG: no edits to past entries** (Cubesat + Professor) — one forward-going entry with Step→Tier mapping table.

**Consensus disposition:** APPROVE WITH AMENDMENTS. All amendments incorporated.

**Sources cited by council** (load-bearing only):
- DO-178C objective tables + tool-qualification discipline (DO-330) — JPL Lead
- ArduPilot AP_HAL_ChibiOS CI-rot incident (analogous to LL Entry 36) — ArduPilot
- Refactoring research / topological-cut dependency-DAG framing — Professor
- NASA SWE §8.5 four-grade severity scale — Cubesat + JPL
- F' Mars sustaining-engineering pattern (prior-cycle delta) — JPL
- ArduPilot sustaining-engineering pattern (regression check on closed findings) — ArduPilot
- DO-178C verification-independence framing — JPL
- Project-internal: LL Entry 36 (gate rot), LL Entry 25 (supersession header pattern), `HW_GATE_DISCIPLINE.md` Rule 6 (verified-locally vs verified-at-audit vs verified-independently), `feedback_triage_not_aviation.md` memory entry (sourcing reminder).
