# RocketChip Standards Audit — Template

**To start a new audit:** Copy this file to `STANDARDS_AUDIT_YYYY-MM-DD.md` and populate findings.

**Previous audits:**
- [STANDARDS_AUDIT_2026-02-07.md](STANDARDS_AUDIT_2026-02-07.md) — Phase 1-3 (P10, flow control, portable code, platform, C++, functions, style, JPL C)

---

## Metadata

- **Audit Date:** YYYY-MM-DD
- **Audited By:** [agent or person]
- **Codebase Snapshot:** [commit hash]
- **Lines Audited:** [count]

---

## How to Use This Document

**Status codes:**

| Code | Meaning |
|------|---------|
| PASS | Compliant with the rule |
| FAIL | Violation found — see notes |
| PARTIAL | Some files comply, others don't |
| NOT CHECKED | Not yet audited |
| N/A | Not applicable to this codebase |

**Applicability codes:**

| Code | Meaning |
|------|---------|
| M | Mandatory — applies directly, must be enforced |
| R | Recommended — applies in principle, aspirational |
| N/A | Not Applicable — wrong language feature, platform, etc. |
| D | Deferred — applies but not relevant until a future feature is implemented |
| SDK | SDK Interface Constraint — violation occurs at Pico SDK boundary. Documented per-function with safety justification. |

**Relationship to other documents:**
- `CODING_STANDARDS.md` — Defines our rules. This document checks compliance.
- `STANDARDS_DEVIATIONS.md` — Accepted deviations are logged there, referenced here.
- `LESSONS_LEARNED.md` — Debugging knowledge that informed our platform rules.

---

## Standards Hierarchy

```
MISRA C (1998/2004)          ← Foundation: automotive safety C rules
  └── JSF AV C++ (2005)      ← Extension: C++ for flight-critical systems (Lockheed Martin)
        └── JPL C (2009)      ← Refinement: JPL institutional standard, adds LOC levels
              └── Power of 10 ← Distillation: 10 most critical safety rules (Holzmann/JPL)
```

Where JSF AV and JPL C conflict, JPL C takes precedence.

---

## Summary Dashboard

| Standard | Total | Applicable | Compliant | Deviations | Not Checked |
|----------|-------|------------|-----------|------------|-------------|
| Power of 10 | 10 | 10 | | | |
| JSF AV C++ | 221 | ~150 | | | |
| JPL C (LOC-1-6) | ~31 | ~25 | | | |
| Platform Rules | ~12 | ~12 | | | |
| Multicore | ~5 | ~5 | | | |
| Debug Output | ~20 | ~20 | | | |
| Git Workflow | ~14 | ~14 | | | |
| Session Checklist | ~12 | ~12 | | | |

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

*LOC-1 through LOC-4 mandatory. LOC-5/6 deferred.*

---

## Section D: Project-Specific Rules

*D.1: Platform Constraints, D.2: Multicore Rules, D.3: Debug Output, D.4: Prior Art, D.5: Safety, D.6: Git Workflow, D.7: Session Management.*

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
|------|-------|-------|---------|--------|-------|

---

## Section H: Ongoing Compliance Verification

*Pre-commit checklist, static analysis plan, triggered rechecks, maintenance cadence.*
