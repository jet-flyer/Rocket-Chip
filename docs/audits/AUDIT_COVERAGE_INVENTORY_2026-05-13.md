# Audit Coverage Inventory — 2026-05-13

**Purpose:** Whiteboard step 0 + step 1b combined deliverable.

- **Step 0 (closed 2026-05-13 earlier in session):** prove the inventory of audit-coverage gaps is itself exhaustive *before* filling any of them. Surface every audit-shaped artifact in the repo, classify whether it is currently absorbed by `standards/AUDIT_GUIDANCE.md` master phases, and produce a final gap list that step 1 (fill) will work against.
- **Step 1b (closed 2026-05-13 later in session):** second inventory pass against the *refactored* procedure (after step 1a landed the 7-tier rewrite). Prove the restructure itself didn't introduce new gaps.

**User direction (this session):** the master suite should include not only the new/updated work but **anything we've done a full audit on in the past**. Past one-off cycles are absorbed into the master suite unless there's a clear specialized-cycle reason. Nothing comes to mind as a clear exception.

**Authored:** 2026-05-13. Step 1b extension added 2026-05-13 same session.

---

## Method

1. Enumerated every file named `*AUDIT*` or `*REVIEW*` repo-wide (excluding build dirs, vendored libs, `.git`).
2. Walked `standards/AUDIT_GUIDANCE.md` Steps 1–8 + Appendix A.1 / A.2 + Appendix B / C to identify every claimed audit phase.
3. Walked `standards/STANDARDS_AUDIT.md` template Sections A–H (the file Step 2 says to instantiate dated).
4. Cross-checked `CHANGELOG.md` for the strings "audit", "sweep", "comment.density", "signal.audit" to surface scattered audit-style work that landed without a centralized record.
5. Inspected `docs/PROJECT_STATUS.md` for audit citations (caught one broken link).
6. Inspected `docs/BUILD_SYSTEM_AUDIT.md` for stand-alone audit machinery not in `docs/audits/`.

---

## Repo inventory — every audit-shaped artifact

### A. Dated / explicit audit reports in `docs/audits/`

| File | Type | Date | Absorbed by master suite? (post-1a refactor) |
|---|---|---|---|
| `MASTER_STANDARDS_AUDIT_2026-05-07.md` | Master audit (with L2 regression) | 2026-05-07 + 2026-05-13 | ✅ Is the master |
| `MASTER_STANDARDS_AUDIT_2026-05-07_PHASE8_ORDERING.md` | Companion (remediation ordering) | 2026-05-12 | ✅ Companion to master |
| `MASTER_STANDARDS_AUDIT_2026-05-07_REMEDIATION_PRELIMINARY.md` | Companion (preliminary queue) | 2026-05-12 | ✅ Companion to master |
| `STANDARDS_AUDIT_2026-02-07.md` | Full Sections A–H walk (canonical) | 2026-02-07 | ✅ Tier 3 instantiates next cycle |
| `STANDARDS_AUDIT_2026-03-26.md` | clang-tidy + lizard tiered pass | 2026-03-26 | ✅ Tier 2.1 |
| `CLANG_TIDY_AUDIT_2026-02-09.md` | clang-tidy 19.1.7 sweep | 2026-02-09 | ✅ Tier 2.1 |
| `CLANG_TIDY_AUDIT_2026-02-20.md` | clang-tidy 21.1.8 sweep | 2026-02-20 | ✅ Tier 2.1 |
| `AO_COMMANDMENTS_AUDIT_2026-04-27.md` | 9 AOs × 12 commandments | 2026-04-27 | ✅ Tier 4.4 |
| `DEV_CODE_AUDIT.md` | Dev-tier code in flight binary | 2026-04-15 | ✅ Tier 3.6 |
| `VERSION_STRING_AUDIT.md` | 4 stale version-like values | 2026-04-15 | ✅ Tier 3.7 |
| `TOOLCHAIN_VERSION_AUDIT_2026-04-27.md` | P6 of BUILD_SYSTEM_AUDIT (versions) | 2026-04-27 | ✅ Tier 2.3 |
| `cla_rbm/CLA_RBM_PLAN.md` + COMPUTATIONAL_LOAD_ANALYSIS + RUNTIME_BEHAVIOR_MAP + dated CLA reports + dot/ graphs | CLA + RBM | 2026-03-08 (most recent) | ✅ Tier 4.5 |

### B. Audit machinery outside `docs/audits/`

| File | Type | Absorbed? (post-1a refactor) |
|---|---|---|
| `standards/AUDIT_GUIDANCE.md` | Master procedure (7-tier dependency-ordered) | ✅ Is the procedure |
| `standards/STANDARDS_AUDIT.md` | Template for Tier 3 dated companion | ✅ Tier 3 instantiates |
| `standards/AUDIT_REMEDIATION.md` | LEGACY (closed 2026-05-07; remediation now in dated reports) | ✅ Marked legacy |
| `docs/BUILD_SYSTEM_AUDIT.md` | CMake / toolchain / dev-tool gating / coverage rot audit (P1-A..P6) | ✅ Tier 2.3 (P6) + Tier 2.4 (P1-A..P5) |

### C. Audit-style work in CHANGELOG without dedicated audit reports

| What was audited | When | Documented in | Absorbed? (post-1a refactor) |
|---|---|---|---|
| Comment-density audit | 2026-05-13 | CHANGELOG, CODING_STANDARDS.md "Comment Density" | ✅ Tier 3.4 / Section D.8 |
| LL-entry freshness audit (R-16) | 2026-05-13 | CHANGELOG | ✅ Tier 4.6 |
| AO signal audit | 2026-04-06 | CHANGELOG, `docs/AO_ARCHITECTURE.md` | ✅ Tier 4.4 absorbs (per user 2026-05-13: "AO comment audit is one case of a bespoke audit but things like that should be covered in the overall comment check anyways") |
| Code-comments audit (76 files, content review) | 2026-04-06 | CHANGELOG | ✅ Tier 3.4 / Section D.8 (comment-density measurement is the regular cadence; content review folds under Tier 3) |
| Dead-code cleanup (audit-driven) | 2026-04-27 | CHANGELOG | ✅ Tier 2.4 (BUILD_SYSTEM_AUDIT P2 self-flagged dead code) + Tier 3.6 (DEV_CODE) |
| Polarity rename + tier consolidation audit | 2026-04-27 | CHANGELOG | ✅ Tier 2.4 (BUILD_SYSTEM_AUDIT P1-A) |
| Protected-files review | 2026-04-29 | CHANGELOG | ⚠️ Reactive on user observation; no periodic absorption. Tier 6.4 (protected-doc drift check) is adjacent but reactive-style; not the same as a proactive review. Documented as deliberate non-absorption. |
| Stage-close build-system audit | Every stage close since 2026-04-23 | `BUILD_SYSTEM_AUDIT.md` | ✅ Tier 2.4 (every milestone audit walks it) |
| Full-tree clang-tidy sweep | Every milestone close | `SESSION_CHECKLIST.md` item 17 | ✅ Tier 2.1 (master); SESSION_CHECKLIST item 17 is the milestone-close mechanism between master cycles |

### D. Audit-style work in standards docs (live-running)

| Mechanism | Where defined | Absorbed? |
|---|---|---|
| `ACCEPTED_STANDARDS_DEVIATIONS.md` Active section walk | `ACCEPTED_STANDARDS_DEVIATIONS.md` | ✅ Tier 2.5 |
| `RP2350_ERRATA.md` E2/E9/E11/E12 re-verification | `AUDIT_GUIDANCE.md` Appendix A.2 | ✅ Tier 5.2 |
| `PROBLEM_REPORTS.md` PR-lifecycle tracker | Established 2026-05-12 | ✅ Tier 7 (Appendix C 4-category) + Tier 2.5a (deferred rows) |
| Pre-flight gate (`PRE_FLIGHT_CHECKLIST.md`) | Tier 5.1 | ✅ Tier 5.1 |
| Deferred-with-rationale rows (NEW post-council #4) | All audit machinery | ✅ Tier 2.5a |
| Prior-cycle delta (NEW post-council #5) | Most recent dated audit | ✅ Tier 2.6 |
| Regression check on prior closed findings (NEW post-council #6) | Tier 2.6 output | ✅ Tier 5.6 |

### E. Council reviews (semi-audit-shaped)

Council reviews are decisional / one-shot — distinct from periodic audit cycles. **No master-suite absorption needed**; they're already linked from the WB / decision-doc structure.

---

## Cross-check: pre-1a "Step N" vs post-1a "Tier N"

| Old Step | Old scope | Tier mapping | Coverage post-1a |
|---|---|---|---|
| Step 1 (Baseline Scripted Sweeps) | clang-tidy, bench_sim, ctest, SPIN, soak | Tier 2 | ✅ Expanded (Tier 1 gate-check added before Tier 2) |
| Step 2 (Coding Standards Deep Compliance) | `STANDARDS_AUDIT.md` template walk | Tier 3 | ✅ Full Sections A-E + F-1 + DEV_CODE + VERSION_STRING |
| Step 3 (Pre-Flight Gate Execution) | `PRE_FLIGHT_CHECKLIST.md` | Tier 5.1 | ✅ Same scope |
| Step 4 (Safety-Critical Path Review) | FMEA-lite + Koopman | Tier 4.1-4.2 | ✅ Same scope + 4.4 AO + 4.5 CLA + 4.6 LL |
| Step 5 (Stack / Memory / Errata) | Appendix A.2 | Tier 5.2 | ✅ Same scope |
| Step 6 (Formal Verification + Simulation) | SPIN, bench_sim | Tier 5.3 + 5.4 (SPIN self-check now Tier 1.2 / verification at Tier 2.2 / re-verification at Tier 5.3) | ✅ Same coverage, split across appropriate tiers |
| Step 7 (Requirements Traceability) | Spot-check | Tier 5.5 (now exhaustive per L2-P2 disposition) | ✅ Stronger than before |
| Step 8 (Remediation & Historical) | `## Remediation` + Appendix C | Tier 7 | ✅ Unchanged in structure |

**Post-1a additions beyond the old Step structure:**
- Tier 1 (gate integrity) — entirely new, addresses LL Entry 36 pattern
- Tier 2.5a (deferred-rationale walk) — new per council #4
- Tier 2.6 (prior-cycle delta) — new per council #5
- Tier 5.6 (regression check on prior closed findings) — new per council #6
- Tier 1.4 bidirectional regex audit — new per council #1
- Severity-gated stop conditions — new per council #3
- Tier 5 independence guidance — new per council #7
- Section F split into F-1 (Tier 3.8) and F-2 (Tier 6.1) — new per council #2

---

## Step-1b verification: every step-0 gap item has a home

Walking the 15 gap items from step 0 against the post-1a procedure:

| Gap ID | What gap | Post-1a home | Status |
|---|---|---|---|
| L2-P5 (JSF AV 221 + JPL C LOC + Project + Agent walk) | Standards-walk catchup | Tier 3.1 + 3.3 + 3.4 + 3.5 | ✅ Closed by structure |
| L2-P12 (Sections F + G + H of template) | Standards-walk catchup | F-1 → Tier 3.8; F-2 → Tier 6.1; G → Tier 6.2; H → Tier 6.3 | ✅ Closed by structure |
| L2-P6 (DEV_CODE re-audit) | Sibling re-audit | Tier 3.6 | ✅ Absorbed |
| L2-P7 (VERSION_STRING re-audit) | Sibling re-audit | Tier 3.7 | ✅ Absorbed |
| L2-P8 (AO_COMMANDMENTS re-audit) | Sibling re-audit | Tier 4.4 | ✅ Absorbed |
| L2-P9 (TOOLCHAIN re-audit) | Sibling re-audit | Tier 2.3 | ✅ Absorbed |
| L2-P10 (CLA-RBM as audit type) | New phase absorption | Tier 4.5 | ✅ Absorbed |
| L2-P13 (BUILD_SYSTEM_AUDIT absorption) | New phase absorption | Tier 2.3 (P6) + Tier 2.4 (P1-A..P5) | ✅ Absorbed |
| L2-P14 (comment-density into Section D) | Template extension | Section D.8 added in STANDARDS_AUDIT.md template | ✅ Closed by template edit |
| L2-P15 (LL-entry freshness procedural placement) | Procedural placement | Tier 4.6 | ✅ Absorbed |
| L2-P16 (PROJECT_STATUS stale citation I-1) | Doc-drift | Removed in step 1a | ✅ Closed |
| L2-P17 (MISRA-C governance) | User-review item | DEFERRED-WITH-RATIONALE; chain-of-custody in CODING_STANDARDS.md Foundation; re-evaluated at Tier 2.5a; forward-pointer in PROJECT_STATUS | ✅ Closed by structure + protected-file edits |
| L2-P18 (AO signal + code-comments audit absorption) | Audit-procedural | Folded into Tier 3.4 (Section D.8 comment-density) + Tier 4.4 (AO Commandments) per user 2026-05-13 | ✅ Closed |
| L2-P19 (replay_gate_test gap blocked on R-23) | Blocked dependency | Tier 5.4 — explicitly documented as blocked-on-R-23 | ⚠️ Documented in procedure; cannot clear until R-25 disposition |

**All 15 gap items have homes in the updated procedure.**

L2-P19 is the only one with a forward-going block, and that block is **explicit in Tier 5.4 of the procedure** — not a hidden gap. Future audit cycles see "this is blocked on R-23 / R-25" as part of the procedure, not as a missing-step surprise.

---

## Step-1b verification: every council amendment is reflected in protected files

| # | Amendment | Reflected where |
|---|---|---|
| 1 | Tier 1.4 bidirectional regex check | `AUDIT_GUIDANCE.md` Tier 1.4 (a) + (b) |
| 2 | Section F split: F-1 → Tier 3.8, F-2 → Tier 6.1 | `AUDIT_GUIDANCE.md` Tier 3.8 + Tier 6.1 |
| 3 | Severity-gated stop conditions | `AUDIT_GUIDANCE.md` severity gate note + every Tier stop-condition |
| 4 | Tier 2.5a deferred-with-rationale walk | `AUDIT_GUIDANCE.md` Tier 2.5a |
| 5 | Tier 2.6 prior-cycle delta read | `AUDIT_GUIDANCE.md` Tier 2.6 |
| 6 | Tier 5.6 regression check on prior closed findings | `AUDIT_GUIDANCE.md` Tier 5.6 |
| 7 | Tier 5 independence amendment | `AUDIT_GUIDANCE.md` Tier 5 preamble |
| 8 | CHANGELOG: no edits to past entries; single forward-going entry with mapping | Deferred to session-end CHANGELOG entry per user feedback 2026-05-13. The Step→Tier mapping table will live there. |

All 8 amendments reflected in protected files (with amendment #8 deliberately deferred to session-end CHANGELOG entry).

---

## Step-1b verification: cross-reference consistency

| Doc | Cites which other doc | Consistent? |
|---|---|---|
| `AUDIT_GUIDANCE.md` Tier 3 | `STANDARDS_AUDIT.md` template Sections A-E + F-1 | ✅ Yes (template has all those Sections) |
| `AUDIT_GUIDANCE.md` Tier 2.4 | `docs/BUILD_SYSTEM_AUDIT.md` P1-A..P5 | ✅ Yes |
| `AUDIT_GUIDANCE.md` Tier 3.3 LOC-5/6 | `CODING_STANDARDS.md` Foundation MISRA chain-of-custody | ✅ Yes |
| `STANDARDS_AUDIT.md` Section C | `CODING_STANDARDS.md` Foundation MISRA chain-of-custody | ✅ Yes |
| `STANDARDS_AUDIT.md` Standards Hierarchy | `CODING_STANDARDS.md` standards-precedence rule | ✅ Yes |
| `CODING_STANDARDS.md` Foundation | `PROJECT_STATUS.md` "Side Projects" MISRA forward-pointer | ✅ Yes |
| `PROJECT_STATUS.md` (post-edit) | `docs/baselines/stage_o_hw_verification_2026-04-28.md` (Stage O runbook) | ✅ Yes (file exists) |
| `AGENT_WHITEBOARD.md` audit-coverage row | `AUDIT_GUIDANCE.md` (Tier 2.4, Tier 4 vehicle scenarios, Tier 5.5 sampling policy) | ✅ Yes |

All cross-references consistent.

---

## Step-1b verification: new gaps from the restructure itself?

Walked the new procedure tier-by-tier looking for items that have no defined source or no clear methodology:

- **Tier 1.1** — tool self-checks. Defined: each tool's `--version` + a known-good fixture input. Pattern is mechanical. No gap.
- **Tier 1.2** — SPIN models compile + load. Defined: `tools/spin/README.md` invocation. No gap.
- **Tier 1.3** — Pre-commit hook integrity. Defined: synthetic staged-diff fixture. **POTENTIAL GAP:** no existing fixture script. Surface as new item for tooling backlog.
- **Tier 1.4** — bidirectional regex audit. **POTENTIAL GAP:** the (b) deleted-regex check requires a list of every PASS-token the firmware emits; no maintained inventory exists. Same gap as L2-P3 (citation inventory) — surface for tooling backlog.
- **Tier 1.5** — Host ctest. Defined. No gap.
- **Tier 1.6** — Build-parity script. Defined: `verify_build_parity.sh`. No gap.
- **Tier 2.1** — Baseline sweeps. All scripts exist. No gap.
- **Tier 2.2** — SPIN re-run. Defined. No gap.
- **Tier 2.3** — Toolchain. Defined by `BUILD_SYSTEM_AUDIT.md` P6. No gap.
- **Tier 2.4** — Build-system audit P1-A..P5. Defined by `BUILD_SYSTEM_AUDIT.md`. No gap.
- **Tier 2.5** — Active deviations walk. Defined: `ACCEPTED_STANDARDS_DEVIATIONS.md` Active section. No gap.
- **Tier 2.5a** — Deferred-with-rationale walk. **POTENTIAL GAP:** no existing inventory of every deferred-with-rationale row across the audit machinery; spread across `ACCEPTED_STANDARDS_DEVIATIONS.md`, `PROBLEM_REPORTS.md`, dated audit reports' `## Remediation` sections, `CODING_STANDARDS.md` (the new LOC-5/6 entry). Surface as new item for tooling backlog: need a `scripts/list_deferred_rationale.sh` or equivalent.
- **Tier 2.6** — Prior-cycle delta. Defined: open most recent dated audit. Manual walk. No gap.
- **Tier 3.1-3.5** — Sections A-E of template walk. Methodology defined by `STANDARDS_AUDIT.md` template. No gap.
- **Tier 3.6** — DEV_CODE. Methodology defined by `DEV_CODE_AUDIT.md`. No gap.
- **Tier 3.7** — VERSION_STRING. Methodology defined by `VERSION_STRING_AUDIT.md`. No gap.
- **Tier 3.8** — F-1 standards-doc citations. **POTENTIAL GAP:** same as L2-P3 — no maintained citation inventory; the 2026-05-07 L2 audit's grep produced one (R-26 surfaced). Codify as `scripts/list_citations.sh` per L2-P3 disposition. Already on tooling backlog.
- **Tier 4.1-4.2** — FMEA + Koopman. Defined by Appendix A.1. No gap.
- **Tier 4.3** — Fault injection. Defined: `enhanced_fault_injection.py`. **POTENTIAL GAP:** R-9a/b/c queued from 2026-05-07 PARTIAL — scenarios deferred. Acceptable: the deferred-rationale walk (Tier 2.5a) covers this.
- **Tier 4.4** — AO Commandments. Defined by `AO_COMMANDMENTS_AUDIT_*.md` methodology. No gap.
- **Tier 4.5** — CLA-RBM. Defined by `cla_rbm/CLA_RBM_PLAN.md`. No gap.
- **Tier 4.6** — LL-entry freshness. Defined: walk Critical/High entries against current code. No gap.
- **Tier 5.1-5.5** — pre-flight, errata, bench, replay, traceability. All defined. Tier 5.4 explicitly blocked on R-23.
- **Tier 5.6** — Regression check. Defined: takes Tier 2.6 output as input, one-line check per row. No gap.
- **Tier 6.1-6.5** — F-2 + G + H + drift check + CHANGELOG rollup. Defined by `STANDARDS_AUDIT.md` template + `SESSION_CHECKLIST.md`. No gap.
- **Tier 7** — disposition + Appendix C. Defined verbatim by Appendix C. No gap.

**Step-1b conclusion:** the restructure surfaced 3 tooling gaps (Tier 1.3 fixture script, Tier 1.4(b) PASS-token inventory, Tier 2.5a / 3.8 citation inventory). All 3 are **tooling** gaps, not procedural gaps — the procedure itself is complete, but it asks for tools that don't fully exist yet. These tooling gaps are tracked together (they're the same class as the existing L2-P3 / R-22 / R-24 audit-infrastructure remediation queue).

**The procedure is gap-free at the procedural level.** The tooling required to fully execute Tiers 1.3, 1.4(b), 2.5a, and 3.8 partially overlaps with existing audit-infrastructure backlog and should be batched with that queue when it gets a dedicated session.

---

## Sign-off

This inventory closes L2-P11 (step 0) and L2-P12 (step 1b).

Step 0 inventory list: exhaustive within scope walked (audit-shaped artifacts in repo + CHANGELOG strings + AUDIT_GUIDANCE phase mapping + STANDARDS_AUDIT template sections + PROJECT_STATUS / SESSION_CHECKLIST cross-refs). Specifically out of scope: council reviews (one-shot decisional), plan/decision/baseline docs (historical-record), LESSONS_LEARNED entries (post-mortems).

Step 1b verification result: the 7-tier refactored procedure has homes for all 15 step-0 gap items. The 8 council amendments are reflected in protected files (amendment #8 deferred to session-end CHANGELOG per user feedback). Cross-references are consistent across `AUDIT_GUIDANCE.md` / `STANDARDS_AUDIT.md` / `CODING_STANDARDS.md` / `PROJECT_STATUS.md` / `AGENT_WHITEBOARD.md`. Three tooling gaps surfaced (Tier 1.3 fixture, Tier 1.4(b) PASS-token inventory, citation inventory) — all are tooling-class, not procedural, and batch with the existing L2-P3 / R-22 / R-24 audit-infrastructure backlog.

**Step 2 (run the updated master audit) can proceed.**
