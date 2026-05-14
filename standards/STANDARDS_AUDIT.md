# RocketChip Standards Audit — Template

**To start a new audit:** Copy this file to `STANDARDS_AUDIT_YYYY-MM-DD.md` and populate findings.

**Previous audits:**

- [STANDARDS_AUDIT_2026-02-07.md](../docs/audits/STANDARDS_AUDIT_2026-02-07.md) — Phase 1-3 (P10, flow control, portable code, platform, C++, functions, style, JPL C)
- [STANDARDS_AUDIT_2026-03-26.md](../docs/audits/STANDARDS_AUDIT_2026-03-26.md) — clang-tidy / lizard tiered pass (post-Stage 8)

---



## Metadata

- **Audit Date:** YYYY-MM-DD
- **Audited By:** [agent or person]
- **Codebase Snapshot:** [commit hash]
- **Lines Audited:** [count]

---

## How to Use This Document

**Status codes:**


| Code        | Meaning                         |
| ----------- | ------------------------------- |
| PASS        | Compliant with the rule         |
| FAIL        | Violation found — see notes     |
| PARTIAL     | Some files comply, others don't |
| NOT CHECKED | Not yet audited                 |
| N/A         | Not applicable to this codebase |


**Applicability codes:**


| Code | Meaning                                                                                                              |
| ---- | -------------------------------------------------------------------------------------------------------------------- |
| M    | Mandatory — applies directly, must be enforced                                                                       |
| R    | Recommended — applies in principle, aspirational                                                                     |
| N/A  | Not Applicable — wrong language feature, platform, etc.                                                              |
| D    | Deferred — applies but not relevant until a future feature is implemented                                            |
| SDK  | SDK Interface Constraint — violation occurs at Pico SDK boundary. Documented per-function with safety justification. |


**Relationship to other documents:**

- `CODING_STANDARDS.md` — Defines our rules. This document checks compliance.
- `ACCEPTED_STANDARDS_DEVIATIONS.md` — Accepted deviations are logged there, referenced here.
- `LESSONS_LEARNED.md` — Debugging knowledge that informed our platform rules.

---

## Standards Hierarchy

Chronological ordering by publication date; newer overrides older per `CODING_STANDARDS.md` Foundation section standards-precedence rule:

```
JSF AV C++ (December 2005)    ← Foundational catalog: 221 C++ rules (Lockheed Martin)
  └── Power of 10 (2006)      ← Distillation: 10 most critical safety rules (Holzmann/JPL)
        └── JPL C (March 2009) ← C-language refinement, LOC-1..6 tiers
              ├── LOC-1..4    ← JPL-original tiers (102 rules); audited at Tier 3.3.
              └── LOC-5..6    ← MISRA-C absorption tiers; DEFERRED-WITH-RATIONALE.
                                 MISRA C (1998/2004) underpins via absorption here.
                                 Rule text MIRA Ltd. copyrighted; not in public JPL PDF.
                                 Re-evaluate when formally-certified-code variant is in scope.
```

**Precedence rule:** newer overrides older unless the older standard's text explicitly governs the specific case. Silent conflict → default to the more recent OR more restrictive (IRL aviation-engineering practice). Per `CODING_STANDARDS.md` Foundation section + LL Entry 37 (rule-citation discipline).

**MISRA-C chain-of-custody:** MISRA-C is not directly audited as a separate standard. It is **absorbed by JPL C at LOC-5 and LOC-6** (LOC-5 = MISRA "shall" rules not covered by LOC-1..4; LOC-6 = MISRA "should" rules). LOC-5/6 are DEFERRED-WITH-RATIONALE per project policy 2026-05-13. MISRA-C remains a candidate for full inclusion in any future formally-certified-code variant (e.g., a hypothetical Pro-tier / Gemini-class certification path). See `docs/PROJECT_STATUS.md` "Side Projects & Future Product Lines" for the forward-looking pointer.

---

## Summary Dashboard


| Standard          | Total | Applicable | Compliant | Deviations | Not Checked |
| ----------------- | ----- | ---------- | --------- | ---------- | ----------- |
| Power of 10       | 10    | 10         |           |            |             |
| JSF AV C++        | 221   | ~150       |           |            |             |
| JPL C (LOC-1-6)   | ~31   | ~25        |           |            |             |
| Platform Rules    | ~12   | ~12        |           |            |             |
| Multicore         | ~5    | ~5         |           |            |             |
| Debug Output      | ~20   | ~20        |           |            |             |
| Git Workflow      | ~14   | ~14        |           |            |             |
| Session Checklist | ~12   | ~12        |           |            |             |


---

## Section A: JSF AV C++ Standards (221 Rules)

### A.1: Not Applicable Rules — C Files

*Copy from previous audit or re-assess. ~60 rules N/A for .c files (classes, namespaces, templates, C++ casts, exceptions).*

### A.2: SDK Interface Constraints

*Catalog Pico SDK functions at API boundary. Document per-function justification.*

### A.3: Applicable Rules — Audit Tables

*One table per JSF AV category. Copy tables from previous audit and reset status to NOT CHECKED.*

### A.4: main.cpp C++ Assessment

*Audit main.cpp against full C++ ruleset (classes, namespaces, templates, casts, etc.).*

---

## Section B: Power of 10 Rules

*10 rules, each with: rule statement, rationale, cross-references, verification method, compliance status with file:line citations, recommendation.*

---

## Section C: JPL C Standard

*LOC-1 through LOC-4 mandatory (JPL-original tiers, ~102 rules total: LOC-1 has 10, LOC-2 has 7, LOC-3 has 12, LOC-4 has 73). Per-rule walk required at every milestone audit.*

*LOC-5 and LOC-6 are **DEFERRED-WITH-RATIONALE**.* These are the MISRA-C absorption tiers (LOC-5 covers MISRA "shall" rules not in LOC-1..4 = 73 rules; LOC-6 covers MISRA "should" rules = 16 rules). Rule text is MIRA Ltd. copyrighted and not in the public JPL PDF. **Project policy 2026-05-13:** defer until a formally-certified-code variant is in scope (e.g., Pro-tier / Gemini-class certification). The deferral is re-evaluated at every cycle's Tier 2.5a (deferred-with-rationale row walk) — 3-cycle stale-rationale threshold for explicit re-disposition. See `standards/CODING_STANDARDS.md` Foundation section for the MISRA-C chain-of-custody.

---

## Section D: Project-Specific Rules

*D.1: Platform Constraints (RP2350, see `CODING_STANDARDS.md` "RP2350 Bare-Metal Platform Constraints"), D.2: Multicore Rules (see `docs/MULTICORE_RULES.md`), D.3: Debug Output (see `standards/DEBUG_OUTPUT.md`), D.4: Prior Art Research (see `CODING_STANDARDS.md` "Prior Art Research"), D.5: Safety & Regulatory (see `CODING_STANDARDS.md` "Safety and Regulatory"), D.6: Git Workflow (see `standards/GIT_WORKFLOW.md`), D.7: Session Management (see `.claude/SESSION_CHECKLIST.md`), D.8: Comment Density (target band 15-25% per `CODING_STANDARDS.md` "Comment Density"; measurement script: `scripts/audit/comment_density.sh` or equivalent; target band sourced from Polyspace 20% lower limit + Arafati & Riehle 2009 mean 19% / median 17% + Elish & Offutt mean 15.2%).*

---

## Section E: Agent Behavioral Guidelines

*Reference only — AK_GUIDELINES.md is behavioral, not code-auditable.*

---

## Section F: Documentation Sync Issues

*Issues found during audit that affect docs, not code.*

---

## Section G: Audit History

| Date | Phase | Scope | Auditor | Commit | Notes |
| 2026-02-07 | Manual | 249 rules (P10, JSF AV, JPL C, platform) | Claude | `5a8...` | 90% compliant, 25 accepted deviations. See `STANDARDS_AUDIT_2026-02-07.md` |


| 2026-02-09 | Automated | clang-tidy 127 checks, 10 files | Claude | `2c3b5e6` | 1,251 findings, all remediated across 6 phases. See `docs/audits/CLANG_TIDY_AUDIT_2026-02-09.md` |
| ---------- | --------- | ------------------------------- | ------ | --------- | ------------------------------------------------------------------------------------------------ |


---

## Section H: Ongoing Compliance Verification

*Pre-commit checklist, static analysis plan, triggered rechecks, maintenance cadence.*