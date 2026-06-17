# L2-P5 Manual Standards-Walk Guide

**Status:** WORKING DOC — in progress. This is a *human-driven* walk guide, not a finished audit.
**Owner:** Nathan (manual walk). Agents may add rule-class research sections; the walk itself + dispositions are Nathan's.
**Established:** 2026-06-06.
**Closes:** PR **L2-P5** in `docs/PROBLEM_REPORTS.md` (and feeds the Cycle-4 remediation doc per `docs/plans/CYCLE_RESIDUALS_AFTER_R5.md`).

---

## What this doc is (and is not)

**Is:** a field guide for manually walking the *semantic* coding-standards rules — the ones a human has to
eyeball because no script can judge them. For each rule class: the verbatim governing rule (primary-source
verified), which standard governs and why, a concrete "what to look for," a grep recipe to *find candidates*
(not to judge them), judging criteria, and a findings table you fill in as you go.

**Is not:** a re-run of the mechanically-covered rules (function size, cognitive complexity, format-string,
dead-code). Those are PASS-BY-SCRIPT — cite the script + last clean run, don't eyeball them. See the
"Mechanically covered" section at the bottom.

**Why a guide instead of just doing it:** the walk spans multiple sittings. Without per-rule-class
criteria + a place to record progress, you lose your place and re-derive "what am I even looking for"
each time. This doc holds both.

---

## How to use this doc

1. Work one **rule class** at a time (the `## Class:` sections below). They're independent — any order.
2. For each class: read the verbatim rule, run the grep recipe to pull candidate sites, walk each candidate
   against the judging criteria, record PASS/FAIL/PARTIAL in the class's findings table.
3. Tick the class's checkbox in the **Progress** table when its findings table is complete.
4. A `FAIL` that you decide to fix → it becomes (or links to) a new `R-NN` row in `docs/PROBLEM_REPORTS.md`.
   A `FAIL` you decide to accept → it migrates to `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` with your
   sign-off. A `PASS` needs no action. (This mirrors the project's normal disposition flow —
   `AUDIT_GUIDANCE.md` Step 8.)
5. When all classes are done, write the Cycle-4 remediation doc
   (`docs/audits/AUDIT_COVERAGE_CATCHUP_YYYY-MM-DD.md`) and flip L2-P5 → `closed`.

**Status vocabulary** (same as `standards/STANDARDS_AUDIT.md`): `PASS` / `FAIL` / `PARTIAL` /
`NOT CHECKED` / `N/A`.

**Scope of files to walk** (per `CYCLE_RESIDUALS_AFTER_R5.md` — the code that changed since the last full
audit at snapshot `fcd3496`):
- `src/log/rc_log.{h,cpp}` (new from R-5)
- the R-5-migrated files (no longer using `<stdio.h>`)
- `src/drivers/gps_pa1010d.cpp` (Cycle-3 touch)
- `src/safety/anomalous_boot.{h,cpp}`, `src/safety/flight_in_progress.cpp`, the `fault_protection.cpp`
  phase-aware refactor, the GoNoGo prior-fault/prior-brownout additions in `health_monitor.cpp`
> ⚠️ **Open scope question (resolve before walking):** a full `STANDARDS_AUDIT_2026-05-13.md` already claims
> an *exhaustive* walk of every applicable rule at snapshot `fcd3496`, using a "brevity exception" (only
> FAILs/PARTIALs were written up; PASSes covered by a blanket coverage statement). So L2-P5 may legitimately
> be just a **delta-walk** over the files above (what landed *after* `fcd3496`), rather than from-scratch.
> Decide which before investing the time. If delta-walk: the file list above *is* the scope. If from-scratch:
> widen to all of `src/`.

---

## Progress

| # | Rule class | Governing standard | Mode | Done? |
|---|------------|--------------------|------|-------|
| 1 | Return-value checking | JPL C Rule 14 (LOC-3) | manual | ☐ (template ready; walk not started) |
| 2 | Parameter validation | JPL C Rule 15 (LOC-3) | manual | ☐ not researched |
| 3 | Naming conventions | JSF AV 50–58 | manual | ☐ not researched |
| 4 | Header discipline | JSF AV 33–39 / JPL Rule 31 | manual | ☐ not researched |
| 5 | Single point of exit | JSF AV 113 / 114 | manual | ☐ not researched |
| 6 | No recursion | P10 Rule 1 / JPL LOC-2 | manual | ☐ not researched |
| 7 | JPL LOC-1..4 manual-only rules | JPL C | manual | ☐ not researched |
| 8 | Project-specific D.1–D.8 | RocketChip docs | manual | ☐ not researched |
| 9 | Agent-behavioral E | AK_GUIDELINES | reference-only | ☐ not researched |

> Classes 2–9 are stubs until the pilot (class 1) shape is approved. Class 1 below is the **template** —
> every other class gets built to match it.

---

## Class 1 — Return-value checking  *(PILOT / TEMPLATE)*

### Governing rule (primary-source verified)

**Governs: JPL C Rule 14 (LOC-3, Defensive Coding), 2009 — newest in the chain, so it wins per
`CODING_STANDARDS.md` precedence rule.** It's also the one that spells out the escape hatch, which is the
operationally important part for a walker.

> **JPL C Rule 14 (checking return values):** *"The return value of non-void functions shall be checked or
> used by each calling function, or explicitly cast to (void) if irrelevant."* `[MISRA-C:2004 Rule 16.10;
> Power of Ten Rule 7]`
> — *JPL Institutional Coding Standard for C (D-60411), §LOC-3, p.13.*

Supporting / older rules in the precedence chain (they agree; the newer ones cite the older inline, so there
is **no conflict** — JPL Rule 14 just states it most completely):

> **Power of 10, Rule 7:** *"The return value of non-void functions must be checked by each calling function,
> and the validity of parameters must be checked inside each function."*
> Rationale (verbatim highlights): *"…even the return value of printf statements and file close statements
> must be checked. One can make a case, though, that if the response to an error would rightfully be no
> different than the response to success, there is little point in explicitly checking a return value. This is
> often the case with calls to printf and close. In cases like these, it can be acceptable to explicitly cast
> the function return value to (void) – thereby indicating that the programmer explicitly and not accidentally
> decides to ignore a return value. In more dubious cases, a comment should be present to explain why a return
> value is irrelevant."*
> — *Holzmann, "The Power of Ten," IEEE Computer, June 2006.*

> **JSF AV Rule 115 (= MISRA Rule 86):** *"If a function returns error information, then that error
> information will be tested."*
> Rationale: *"Ignoring return values could lead to a situation in which an application continues processing
> under the false assumption that the context in which it is operating (or the item on which it is operating)
> is valid."*
> — *JSF Air Vehicle C++ Coding Standards, Rev C (2RDU00001), Dec 2005.*

**Verification provenance:** JSF 115 + P10 Rule 7 + JPL Rule 14/15 were read verbatim from the canonical
primary-source PDFs on 2026-06-06 (JSF via pypdf text extraction; P10 + JPL via direct PDF read). This
satisfies LL Entry 37 step 1 (verify wording) + step 2 (verify the right standard in the precedence chain).

### What to look for

A call to a non-`void` function whose return value is **silently dropped** — not assigned, not tested in a
condition, not passed onward, and **not** explicitly `(void)`-cast. The cast (or a `// why-ignored` comment)
is what turns "accidentally ignored" into "deliberately ignored" — the former is a finding, the latter is
compliant.

Highest-value targets on this codebase (where a dropped error genuinely bites):
- **I2C / bus calls** — `i2c_bus_*`, `*_read_reg`, `*_write_reg` returning a bool/status that's dropped.
  The whole LL-28/31/41 family is "a bus op silently failed." A dropped I2C return is exactly that class.
- **Flash ops** — `flash_safe_execute(...)` return dropped (LL-31 territory).
- **`rc_log` / CDC drain** — new R-5 surface; check the `rc_log` API and `tud_cdc_*` call sites.
- **GPS PMTK writes** — `gps_pa1010d.cpp` PMTK write helpers return -1 on I2C error (per Cycle-3 notes);
  a dropped return there hides the cold-boot failure mode.
- **QP post / publish** — `QACTIVE_POST` is `void`, so N/A; but any wrapper returning a bool counts.

### Grep recipe (finds *candidates*, does not judge them)

These surface call sites to eyeball. They over-report by design — a hit is a candidate, not a finding.

```bash
# 1) Bus/flash/log calls that appear as a bare statement (line starts with the call, ends in ';')
#    — i.e. return value not obviously consumed. Hand-walk each.
grep -rnE '^\s*(i2c_bus_[a-z_]+|flash_safe_execute|gps_[a-z0-9_]+|rc_log[a-z_]*)\s*\(' src/ \
  | grep -vE '=\s*|if\s*\(|while\s*\(|return |\(void\)'

# 2) Any non-void-looking call used as a bare statement that is NOT already (void)-cast.
#    Noisy; best run per-file on the scope list.
grep -rnE '^\s*[a-zA-Z_][a-zA-Z0-9_]*\s*\([^;]*\)\s*;\s*$' src/log/rc_log.cpp src/drivers/gps_pa1010d.cpp \
  | grep -vE '\(void\)'

# 3) Sanity counter-check: where ARE we already casting to (void)? (these are the compliant ones)
grep -rnE '\(void\)\s*[a-zA-Z_]' src/
```

> The grep cannot tell whether the callee returns `void`. You confirm that by checking the declaration
> (jump to the header). A bare `void` call is automatically compliant.

### Judging criteria (apply to each candidate)

| Situation | Verdict |
|---|---|
| Callee returns `void` | **PASS** (rule N/A to this call) |
| Return assigned, tested in a condition, or passed onward | **PASS** |
| Return dropped, but call is `(void)`-cast (or has an explicit `// ignored because …` comment) | **PASS** — deliberate-ignore is compliant per JPL Rule 14 |
| Return dropped silently, **error response would matter** (bus/flash/GPS/log path) | **FAIL** |
| Return dropped silently, but error response genuinely wouldn't differ from success (rare; e.g. a best-effort debug print) | **PARTIAL** → fix is cheap: add the `(void)` cast to make the intent explicit. Note it; don't agonize. |

Tie-break / philosophy: per P10 Rule 7's own rationale, *"it will be easier to comply with the rule than to
explain why non-compliance might be acceptable."* When unsure, the cheap compliant move is the `(void)` cast +
a one-line why. Default toward that rather than a long deliberation.

### Findings (fill in as you walk)

| File:line | Call | Callee non-void? | Consumed / cast? | Verdict | Note / disposition |
|-----------|------|------------------|------------------|---------|--------------------|
| _e.g._ `src/drivers/gps_pa1010d.cpp:240` | `pmtk_write(...)` | yes (returns int -1 on err) | dropped, not cast | _TBD_ | _walk it_ |
| | | | | | |

**Class 1 disposition summary:** _(write once table is complete: N walked, X PASS / Y FAIL / Z PARTIAL;
which became R-NN rows, which were accepted)_

---

## Mechanically covered (do NOT hand-walk — cite the script)

These rule classes are enforced by tooling. For the L2-P5 writeup, record the script + its last clean run;
do not eyeball them. This is the "NOT MECHANICALLY COVERED" inverse that council amendment #1 requires the
remediation doc to make explicit, so the green checks don't imply the manual rules were covered too.

| Rule class | Script / gate | Notes |
|---|---|---|
| Function size (JSF 1 / P10 4 / JPL Rule 25) | clang-tidy `readability-function-size` (pre-commit + milestone sweep) | exemptions: `src/cli/**`, `eskf_codegen.cpp` |
| Cognitive complexity | clang-tidy `readability-function-cognitive-complexity` | threshold 25 |
| `<stdio.h>` / format-string (JSF 22) | pre-commit Gate 1 (unconditional since R-5 closure) | allowlist deleted |
| Dead code | `scripts/audit/find_dead_code.py` | high false-positive by design; hand-walk *findings* |
| Fault-force test-mode gate | ctest `scripts_fault_force_gate_audit` | `scripts/audit/check_fault_force_gates.py` |
| Toolchain drift | `scripts/audit/check_toolchain_drift.py` | |

---

## Source PDFs (canonical, for the next rule class's research)

- JSF AV C++ Rev C: https://www.stroustrup.com/JSF-AV-rules.pdf
  *(terse rule statements are in the early summary section; the later appendix only has rules needing
  examples and SKIPS others — extract text with pypdf and grep "AV Rule N" rather than paging.)*
- Power of 10: https://spinroot.com/gerard/pdf/P10.pdf
- JPL C (D-60411): https://yurichev.com/mirrors/C/JPL_Coding_Standard_C.pdf
- Helper: `scripts/audit/pdf_section_lookup.py <pdf> <section>` (confirms a section exists; not ideal for
  terse JSF rule statements).
