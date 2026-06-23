# L2-P5 Manual Standards-Walk Guide

**Status:** WORKING DOC — in progress. Human-driven walk guide, not a finished audit.
**This is the FIELD MANUAL (what to look for *during* the walk).** The *work plan* — the ordered prep/walk/close-out tasks, including §CM gating and §RP research — is a separate doc: `docs/plans/L2P5_WALK_PLAN.md`. Don't conflate building the guide with using it.
**Governing principle: COMPLETENESS over brevity.** Every in-scope file gets walked and its coverage recorded (PASS files too, not only FAILs); full criteria per class. Thoroughness is the goal — not a terse summary.
**Owner:** Nathan (manual walk). Agents may build/refine rule-class sections; the walk itself + dispositions are Nathan's.
**Established:** 2026-06-06. **Scope re-set:** 2026-06-20 (triage-driven, see Scope below).
**Closes:** PR **L2-P5** in `docs/PROBLEM_REPORTS.md` (feeds the Cycle-4 remediation doc per `docs/plans/CYCLE_RESIDUALS_AFTER_R5.md`).
**Driver:** `docs/audits/RULE_VERIFIABILITY_TRIAGE.md` — the frozen, primary-source-verified per-rule classification. This guide is the *operational layer* over that triage; it does not re-quote rule text, it points to the triage's detail rows for verbatim + "why."
**Sibling, sequenced AFTER this walk (do not lose):** **L2-P10** (CLA-RBM re-collection, `PROBLEM_REPORTS.md`) is the other half of Cycle 4. It is intentionally **downstream** of this walk: CLA-RBM measures *runtime* behavior, so if the walk surfaces FAILs that get remediated, those code changes alter what CLA-RBM would measure. Close L2-P5, action its remediations, then run L2-P10.

---

## What this doc is (and is not)

**Is:** a field guide for manually walking every coding-standard rule that the tooling **cannot fully and correctly evaluate** — the semantic rules a human has to eyeball, the rules where a green check is misleading, and the mechanically-decidable rules that simply aren't gated yet. For each walk-class: the in-scope rules (with triage cross-refs), the anchor rule(s) verbatim, a concrete "what to look for," a grep recipe to *find candidates* (not to judge them), judging criteria, and a findings table you fill in as you go.

**Is not:** a re-run of rules a trustworthy script already evaluates in full. Those are PASS-BY-SCRIPT — cite the gate + last clean run, don't eyeball them. See "Genuinely script-covered" at the bottom. **But** "a script exists" ≠ "script-covered" — several gates over-claim (enforce the inverse, or cover only a curated subset). Those move *into* the walk. The triage's enforcement diagnostic is the arbiter.

**Why a guide instead of just doing it:** the walk spans many sittings across a large rule set. Without per-class criteria + a progress record you lose your place and re-derive "what am I even looking for" each time. This doc holds both, and the triage holds the verbatim rule text + the reason each rule lands where it does.

---

## The primary lens: what an AI agent waves through

**This codebase was built working with AI agents, and the load-bearing reason for this walk is to catch what an
agent tends to skip, ignore, or not even register as wrong — *regardless of whether the code compiles, runs, and
passes its tests mechanically.*** A green check, a passing ctest, and a clean clang-tidy run are exactly the
conditions under which an agent (and a tool) will declare victory. This guide exists to look *past* that surface.

The failure mode is real and already lived on this project — these are not hypotheticals:
- **Plausible-but-wrong citations that proliferate.** [LL Entry 37](../agents/LESSONS_LEARNED.md): an agent cited
  "JSF Rule 170 prohibits function pointers" (it doesn't), and that wrong citation spread across six docs as
  architectural rationale before anyone checked the primary source. The fix passed every test the whole time.
- **Fabricated authority.** A WebFetch summarizer invented P10 "quotes" (claimed Rule 1 = single-exit — false).
  Nothing mechanical catches a confident fabrication.
- **Over-claim — a gate asserting coverage it doesn't have.** The return-value CANARY (Class 1): clang-tidy reads
  "clean," but `CheckedFunctions` is empty, so the project's own bus/flash/GPS returns are *uncovered*. The green
  check is the lie. (This walk itself found two more: the gates ignore `.clang-tidy`, and the triage mis-listed
  `void` functions as return-checkable.)
- **Code that works but is improper.** Comments that restate code or paraphrase a datasheet (Class 3), a magic
  number with no source (Class 13), `volatile` used as if it synchronized across cores (Class 10) — all compile,
  all pass tests, all are wrong by the standard the project adopted.

So every walk-class carries an explicit **▸ Agent-tendency watch** — the specific way an agent is likely to
produce passing-but-improper code in that rule area. Weight your attention there. The unifying question:
*the tool sees the **what**; is it blind to the **why**?* An agent has the same blind spot, and it doesn't flag
what it can't see.

---

## How to use this doc

1. Work one **walk-class** at a time (the `## Class:` sections). They're independent — any order. The Class Index below shows which are built vs. pending.
2. For each class: read the anchor rule(s) + open the triage detail rows it cites, run the grep recipe to pull candidate sites, walk each candidate against the judging criteria, record PASS/FAIL/PARTIAL in the class's findings table.
3. Tick the class in the **Class Index** when its findings table is complete.
4. Disposition each `FAIL`: a fix → new `R-NN` row in `docs/PROBLEM_REPORTS.md`; an accept → migrates to `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` with your sign-off; a `PASS` needs no action. (Mirrors `AUDIT_GUIDANCE.md` Step 8.)
5. When all classes + the two action sections are done, write the Cycle-4 remediation doc (`docs/audits/AUDIT_COVERAGE_CATCHUP_YYYY-MM-DD.md`) and flip L2-P5 → `closed`.

**Status vocabulary** (same as `standards/STANDARDS_AUDIT.md`): `PASS` / `FAIL` / `PARTIAL` / `NOT CHECKED` / `N/A`.

---

## Scope (re-set 2026-06-20)

**This walk is NOT bounded by the post-`fcd3496` file delta.** It descends from the master "touch-everything" audit lineage, and the governing instruction is: **go over everything we cannot verify the scripts fully and correctly evaluate.** That set is exactly what the triage classified. Files: **all of `src/`** (+ `include/`), not a recency-delta subset.

The inclusion test for a rule, taken straight from the triage's **enforcement diagnostic**:

| Triage says | In the walk? | Which tier |
|---|---|---|
| **over-claiming (critical)** — gate presented as full coverage but enforces a subset, the inverse, or nothing | **YES** — highest priority | Tier 3 semantic |
| **Grey / Manual / Split (the non-deterministic half)** | **YES** | Tier 3 semantic |
| **honest-narrow (low)** — curated/partial tool; residual uncovered | **YES**, walk the residual | Tier 3 semantic |
| **Det-direct / Det-by-reference but UNGATED** (greppable/decidable, but no check actually runs) | **YES**, but cheaply | Tier 2 one-shot mechanical |
| **Det-direct/by-reference WITH a gate that actually runs and fully covers** | **NO** — cite gate + last run | Tier 1 script-covered |
| **Compliant-by-construction** | **NO** — one-line confirmation only | Tier 1 |

### The three tiers (this is the heart of the re-scope)

- **Tier 3 — Semantic walk.** Genuine human judgment, no shortcut. The bulk of the classes below. This is where the time goes.
- **Tier 2 — One-shot mechanical.** The rule is decidable but no script runs it. You don't "walk" these every sitting — you run the grep once, or flip the check on once (the triage's "conversion move"). Collected in **§ One-shot mechanical checks** so they're knocked out in a batch, not re-judged per file.
- **Tier 1 — Script-covered.** Don't hand-walk. Listed in **§ Genuinely script-covered** *after* confirming the gate runs and covers the whole rule. Anything that fails that confirmation is demoted to Tier 2 or Tier 3.

> The old ⚠️ open scope-question (delta vs from-scratch, trust the 2026-05-13 brevity-exception or not) is **resolved**: from-scratch over all of `src/`, scoped by rule-verifiability rather than file recency. The 2026-05-13 audit's blanket-PASS coverage is *not* relied on for the rules in this walk — that's the whole point of walking them.

---

## Class Index

Tier-3 semantic walk-classes (each pulls its in-scope rules from the triage; open the cited triage rows for verbatim + why):

| # | Walk-class | In-scope rules (triage §) | Built? |
|---|------------|---------------------------|--------|
| 1 | Return values & parameter validation | P10-7, JPL 14/15, JSF 114/115 (§2, §3, §4c) | ☑ **built** (template) |
| 2 | Naming & the JSF supersession | JSF 45/49/50/51/52/55/56 (§4a); finding §7.1 | ☑ **built** |
| 3 | Comments & documentation quality | JSF 127/128/129/130/131/132/133/134 (§4c) | ☑ **built** |
| 4 | Assertions | P10-5, JPL 16 (§2, §3) | ☑ **built** |
| 5 | Declaration scope & object lifetime | P10-6, JPL 13, JSF 136/142/143, 70.1/71 (§2,§3,§4b,§4c) | ☑ **built** |
| 6 | Pointers, casts & conversions | P10-9, JPL 26/27/28/30, JSF 169/171/173/174/177–184/215 (§4d); over-claims 175/182 §7.1 | ☑ **built** |
| 7 | Class & interface design | JSF 64–97 cluster (§4b) | ☑ **built** |
| 8 | Templates | JSF 101/102/103/105/106 (§4b) | ☑ **built** |
| 9 | Control-flow discipline | JSF 113/114/191/192/198/199/201/204/205 (§4c,§4d) | ☑ **built** |
| 10 | Concurrency & shared-data ownership | JPL 6/7/8/9 (§3) | ☑ **built** |
| 11 | Preprocessor judgment residual | JPL 20, JSF 8/10/20/29/30/31 (§3,§4a) | ☑ **built** |
| 12 | Header organization | JSF 34/35/36/37/38/40 (§4a) | ☑ **built** |
| 13 | Magic numbers & literal discipline | JSF 147/151/151.1/210/210.1 (§4c,§4d) | ☑ **built** |
| 14 | Expressions & evaluation order | JPL 18, JSF 157/162/163/164/166/187/203/204.1/213 (§3,§4c,§4d) | ☑ **built** |

Action sections (not per-file judgment walks):

| § | Section | Built? |
|---|---------|--------|
| LV | Live unlogged violations to disposition (JSF 18, 27, 190, 202) | ☑ **built** (walk-content) |
| CM | One-shot mechanical checks / conversion moves | ↗ **prep — owned by the work plan** (`docs/plans/L2P5_WALK_PLAN.md` Phase B) |
| RP | Research pass — fill the Manual classes with externally-sourced criteria | ↗ **prep — owned by the work plan** (Phase D) |
| IT | **Walk Itinerary** — ordered file-coverage map (the traversal spine) | ☑ **built** (walk-content) |
| SC | Genuinely script-covered (revalidated; do NOT walk) | ☑ **built** (walk-content) |

> **Build order rationale:** Class 1 is the worked template. Classes 2 and 3 are built next because 2 is a *critical over-claim* (the gate actively encodes a non-JSF scheme while docs cite JSF conformance) and 3 is the "comments restating code / commented-out code / documenting external sources" smell-cluster that motivated the separate agent-walk idea. The remaining Tier-3 classes (4–14) are now built to the Class-1 template, leaning on the triage for verbatim/why; the genuinely-Manual ones get their concrete criteria from **§RP** (work plan, Phase D) before they're walked.

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
- **Flash ops** — `flash_safe_execute(...)` returns `int` (PICO_OK / error) per the SDK — a dropped return is
  LL-31 territory. *(Verify the exact SDK signature before adding to `CheckedFunctions`; if it were `void` it
  would be N/A.)*
- **`rc_log` / CDC drain** — **correction:** `rc_log()` itself returns `void` → **N/A** to this rule. The
  return-checkable log fns are `rc_snprintf` (`size_t` written) and `strbuf_overflowed` (`bool`). The triage's
  "rc_log returns uncovered" framing was wrong — don't hand-walk `rc_log()` for a dropped return.
- **GPS PMTK writes** — `gps_pa1010d.cpp` PMTK write helpers return -1 on I2C error (per Cycle-3 notes);
  a dropped return there hides the cold-boot failure mode.
- **QP post / publish** — `QACTIVE_POST` is `void`, so N/A; but any wrapper returning a bool counts.

> **Why this is in the walk, not script-covered (triage §7.4, THE CANARY):** `bugprone-unused-return-value`
> has **no project `CheckedFunctions`** configured (measured) and `cert-err33-c` covers only the C stdlib — so
> the **`i2c_bus_*` / `gps_*`** returns above are **uncovered** (and `flash_safe_execute`'s `int`, pending sig
> verify). A clang-tidy-clean run does NOT mean Rule 7/14/115 passed. This is the audit's designated canary; it's
> being converted to Det-by-reference by populating an **honest** `CheckedFunctions` list pre-walk (work plan
> Phase B) — honest meaning complete-for-the-family, or a partial list re-creates the very over-claim it fixes.

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

## Class 2 — Naming conventions & the JSF supersession

### Anchor rules (primary-source verified — triage §4a, finding §7.1)

This is a **critical over-claim class**, not an ordinary walk. The project's naming gate
(`readability-identifier-naming`) enforces a **camelBack / `k`-prefixed-constant / CamelCase-type** scheme.
The JSF rules it is cited against say the *opposite*:

> **JSF AV Rule 50:** type names — first letter upper, rest lower (`CamelCase`-ish).
> **JSF AV Rule 51:** *function and variable names shall be entirely lowercase.*
> **JSF AV Rule 52:** *constant and enumerator values shall be lowercase* (project uses `k`-prefix → inverts this).
> **JSF AV Rule 45:** identifiers use underscore word-separation (project uses camelBack → contradicts).
> — *JSF AV C++ Rev C; verbatim properties per triage §4a, primary-source-verified 2026-06-06. Quote the exact
> wording from the JSF PDF (`stroustrup.com/JSF-AV-rules.pdf`) into the finding if you want the literal text on record.*

### What to look for — and the actual finding

The **code is internally consistent and fine** (camelBack functions/vars, `k`-prefixed constants, `g_`-prefixed
globals, CamelCase types — the RocketChip convention). The problem is **not in the source**; it's the
**documentation over-claim**:

- `standards/CODING_STANDARDS.md` (Pre-Commit Checklist → Standards Compliance, line 433) cites the naming scheme as
  *"JSF AV Rule 50-53"* compliance. It is **not** JSF-compliant — it's a deliberate, reasonable project
  convention that **supersedes** JSF 45/51/52. The over-claim is the doc text asserting JSF conformance.

So this "walk" is really a **disposition**, not a per-file hunt:

1. Confirm the source is internally consistent with the project convention (spot-check; the gate already enforces it).
2. Confirm the doc over-claim exists (it does — `CODING_STANDARDS.md:433`).
3. **Disposition:** record the supersession explicitly. Per the project precedence rule a documented project
   convention can override an older standard — but it must be *written down as a supersession*, not mis-labeled
   as "JSF compliance." This is the honest-disposition fix the triage §7.1 calls for.

### Grep recipe (consistency spot-check only)

```bash
# Constants should be k-prefixed; globals g_-prefixed. These find any that AREN'T (convention drift).
grep -rnE '\bconst(expr)?\b[^=]*\b[a-z][a-zA-Z0-9]*\s*=' src/ | grep -vE '\bk[A-Z]'   # candidate non-k constants
grep -rnE '^\s*[A-Za-z_][A-Za-z0-9_<>:]*\s+g[^_a-zA-Z]' src/                            # candidate non-g_ globals
# The gate (readability-identifier-naming) already enforces camelBack fns/vars; drift here is rare.
```

### Judging criteria

| Situation | Verdict |
|---|---|
| Source follows the project convention (camelBack / `k` / `g_` / CamelCase) consistently | **PASS** (against the *project convention*, which governs) |
| `CODING_STANDARDS.md` cites this as "JSF AV 50-53 compliance" | **FAIL (over-claim)** — doc finding, not a code finding |
| A genuine convention-drift site (e.g. a `snake_case` function, an un-prefixed global) | **PARTIAL/FAIL** — small remediation; rare |

### Findings

| Item | Location | Verdict | Disposition |
|------|----------|---------|-------------|
| Doc over-claim "JSF AV 50-53" | `standards/CODING_STANDARDS.md:433` | _TBD_ | record supersession (project convention overrides JSF 45/51/52); reword the citation |
| Source convention consistency | `src/` (spot-check) | _TBD_ | expect PASS |
| | | | |

> **Note:** `CODING_STANDARDS.md` is protected. The supersession reword is a separate authorized edit — flag it
> for Nathan, don't fold it into a code commit. (The cross-ref table amendment already landed there 2026-06-17;
> this is a distinct change.)

---

## Class 3 — Comments & documentation quality

### In-scope rules (triage §4c — almost all **Manual**; this is the smell-cluster)

These are the rules a tool fundamentally cannot judge, and the ones that motivated wanting an outside-eyes
pass. Properties per triage §4c (primary-source-verified); quote the exact JSF wording into a finding if a
dispute arises:

- **JSF 127** — delete commented-out code. *(Grey — deadcode tools ≠ commented-out code; a human confirms.)*
- **JSF 128** — no comments that document *external* sources (cite a doc/pointer instead of restating it). **← the specific smell you flagged.**
- **JSF 129** — header comments describe *external behavior/abstraction*, not implementation.
- **JSF 130** — every executable line is purpose-commented *(Manual — density ≠ purpose).*
- **JSF 131** — no comment that merely restates the code. **← the other smell you flagged.**
- **JSF 132** — every declaration/member commented *(presence is mechanical; non-redundancy is Manual).*
- **JSF 133** — every file has an intro comment *(presence mechanical; accuracy Manual).*
- **JSF 134** — document a function's assumptions & limitations.
- (JSF 126 — only `//`-style comments — is Det-direct/lexer; lives in §SC, not here.)

These also intersect the project's **Comment-Density policy** in `CODING_STANDARDS.md` (15-25% band in `.cpp`;
"comments answer *why*, not *what*"; pull multi-paragraph rationale to a decision doc + a 1-line pointer).

### What to look for

The two highest-value smells (your framing):
1. **Restatement (131):** a comment that says what the next line already says in well-named code
   (`// increment i` over `++i`; `// loop over sensors` over `for (auto& s : sensors)`). Noise; delete.
2. **Restating an external doc (128/129):** a comment block that paraphrases a datasheet, an LL entry, a
   decision doc, or a standard — instead of *pointing* to it. These rot (the source moves on; the paraphrase
   doesn't) and bloat density. Replace with a one-line pointer (`// see LL Entry 31` / `// DPS310 datasheet §7.6`).

Secondary:
3. **Missing "why" on a load-bearing line (130/134):** a hardware quirk, a safety invariant, a magic constant,
   a surprising ordering — uncommented. These are the comments you *keep/add*.
4. **Stale file/header intro (133/129):** intro comment describes a module that has since been refactored.

> Net principle (matches the density policy): **delete what restates code or external docs; keep/add what
> captures why, hardware quirks, invariants, and LL-lessons.** A pointer beats a paraphrase every time.

### Grep recipe (candidates only — these are weak signals; judgment is the whole job)

```bash
# 1) Comment lines that look like they restate the immediately-following code token (very rough).
#    Eyeball: does the comment add 'why', or just echo the 'what'?
grep -rnB0 -A1 -E '^\s*//' src/ | less   # walk per-file; no regex substitutes for reading

# 2) Multi-line comment blocks (>= 4 consecutive // lines) — candidates for "paraphrasing a doc that
#    should be a pointer." Find dense comment runs:
awk '/^\s*\/\//{c++; if(c==4) print FILENAME":"NR" : >=4-line comment block"} !/^\s*\/\//{c=0}' \
  $(git ls-files 'src/*.cpp' 'src/*.h')

# 3) Commented-out code (JSF 127): lines that are // followed by something that looks like a statement.
grep -rnE '^\s*//\s*[a-zA-Z_].*[;{}]\s*$' src/ | grep -vE '//\s*(TODO|NOTE|see|per|ref)'
```

### Judging criteria

| Situation | Verdict |
|---|---|
| Comment explains *why* (hardware quirk, invariant, safety rationale, LL ref, surprising ordering) | **PASS** — keep |
| Comment restates what well-named code already says (131) | **FAIL** — delete |
| Comment paraphrases an external doc/datasheet/LL/decision instead of pointing to it (128/129) | **FAIL** — replace with a 1-line pointer |
| Commented-out code (127) | **FAIL** — delete (git has it) |
| Load-bearing line with no "why" and it's non-obvious (130/134) | **PARTIAL** — add the missing rationale |
| Decl/member/file with no comment where the contract isn't self-evident (132/133) | **PARTIAL** — add a one-liner |
| `.cpp` function over ~30% density driven by paragraph rationale | **FAIL (density)** — pull to a decision doc + pointer (per `CODING_STANDARDS.md`) |

### Findings

| File:line | Smell (127/128/129/130/131/132/133/134) | What it restates / lacks | Verdict | Disposition |
|-----------|------------------------------------------|--------------------------|---------|-------------|
| | | | | |

**Class 3 disposition summary:** _(N walked; deletions vs pointer-replacements vs added-why; any density refactors)_

---

## Classes 4–14

Built to the Class-1 template, **leaning on the triage for verbatim rule text + the "why"** (open the cited
`§` rows) rather than re-quoting — that's the two-docs-don't-drift design. Each class is the *operational* layer:
what to look for, the agent blind spot, a candidate-finding grep, judging criteria, a findings table. The
genuinely-Manual classes (3, 7, 8, 10) get their concrete criteria filled in by **§RP** (work plan, Phase D)
before they're walked — where one currently says "use judgment," that's a placeholder, not the final instruction.

---

## Class 4 — Assertions

### In-scope rules (triage §2 P10-5, §3 JPL 16)

P10-5 == JPL 16. **P10-5 is Split/critical:** density (≥2 assertions/fn) + non-vacuity have **no tool**;
side-effect-freedom is Det (`bugprone-assert-side-effect`). On this project assertions are a real safety
mechanism — `Q_ASSERT`/`Q_REQUIRE`/`Q_ENSURE` route to `Q_onError` (the fault path), not a debug no-op.

### What to look for

Functions doing real work with **no precondition assertions**; assertions that are **vacuous** (`assert(true)`,
`assert(1)`, asserting a fact the type system or the previous line already guarantees); assertions with
**side effects** (`assert(x = init())`); the density target on the longer functions.

### ▸ Agent-tendency watch

Agents pad assertion density with **decorative** asserts (`assert(ptr != nullptr)` immediately after
`ptr = &thing;`) that count toward "≥2/fn" but check nothing real — and **under-assert the load-bearing
preconditions** (JPL 15 param-validity) because the code runs fine without them. Both pass every gate; neither
satisfies the *intent* of P10-5. A wall of assertions is not the same as meaningful ones.

### Grep recipe (candidates)

```bash
grep -rnE '\b(assert|Q_ASSERT|Q_REQUIRE|Q_ENSURE|Q_INVARIANT)\s*\(' src/   # all assertion sites; eyeball density
grep -rnE '\bassert\s*\(\s*(true|false|1|0)\s*\)' src/                      # vacuous candidates
```

### Judging criteria

| Situation | Verdict |
|---|---|
| Side effect inside the assert expression | **FAIL** (`bugprone-assert-side-effect` should catch; if it slips, that's a finding) |
| Vacuous / tautological Boolean | **FAIL** — delete or replace with a real invariant |
| Long, load-bearing fn with no precondition assertions | **PARTIAL** — add the missing precondition assert |
| Meaningful invariant/precondition assertions present | **PASS** |

### Findings

| File:line | Assertion | Issue (vacuous / side-effect / missing / OK) | Verdict | Disposition |
|---|---|---|---|---|
| | | | | |

---

## Class 5 — Declaration scope & object lifetime

### In-scope rules (triage §2 P10-6, §3 JPL 13, §4b/§4c JSF 136/142/143/70.1/71)

P10-6 == JPL 13 == JSF 136 (smallest scope). JPL 13 also bundles **shadowing** (Det via `-Wshadow`, gated by
the work plan's §CM). JSF 142 init-before-use (Det), 143 no-var-until-meaningful-init (Grey/Manual), 70.1
use-before/after-lifetime (Grey, LL 35), 71 no-external-op-before-full-init (Grey, two-phase init, LL 6).

### What to look for

Variables declared wider/earlier than first use; use-before-init or use-after-lifetime; two-phase-init objects
used before `begin()`/`init()`; **stack-local `QEvt` posted to QP** (LL 35 use-after-free — the canonical case).

**The mandated-static override (read this first):** [LL Entry 1](../agents/LESSONS_LEARNED.md) + the
Memory-Allocation rules **REQUIRE** `static` file-scope for objects >1KB (stack-overflow prevention). A
mandatory file-scope static is **PASS by-reference, not a scope violation** — the *why* (stack limit) is fixed
elsewhere. Look it up; don't judge it.

### ▸ Agent-tendency watch

The dangerous one: an agent "tidying for smaller scope" moves a **mandated** `static g_calibrator;` into a
function-local — reintroducing the exact LL Entry 1 silent stack-overflow crash, while the code still compiles.
Agents also miss **two-phase-init** contracts (object used before `begin()` — compiles, only bites at runtime,
LL 6) and the **stack-local-event** use-after-free (LL 35 shipped for months passing tests). Lifetime/scope bugs
are precisely the class that passes host tests and most hardware runs.

### Grep recipe (candidates)

```bash
grep -rnE '\bstatic\b.*\bg_[a-z]' src/                  # confirm large objects ARE static (LL1) — these are PASS
grep -rnE '\.begin\(\)|_init\(' src/                    # two-phase-init sites: is begin()/init() called before use?
# Shadowing is gated by the work plan (§CM / -Wshadow); cite the gate, don't hand-walk it once enabled.
```

### Judging criteria

| Situation | Verdict |
|---|---|
| `static` file-scope on a >1KB object (LL 1 mandate) | **PASS** — by-reference, NOT a violation |
| Scope wider/earlier than needed, no mandate | **PARTIAL** — narrow it / declare at first meaningful init |
| Use-before-init, use-after-lifetime, or stack-local posted event (LL 35) | **FAIL** |
| Two-phase object used before its `begin()`/`init()` (LL 6) | **FAIL** |
| Declared at smallest feasible scope, initialized meaningfully | **PASS** |

### Findings

| File:line | Declaration | Scope/lifetime issue | Mandated-static? | Verdict | Disposition |
|---|---|---|---|---|---|
| | | | | | |

---

## Class 6 — Pointers, casts & conversions  *(CRITICAL over-claims)*

### In-scope rules (triage §4d JSF 169/171/173/174/177–184/215, §3 JPL 26/27/28/30, §2 P10-9; findings §7)

Two **critical over-claims** live here — the finding is the *gate/doc*, not (mostly) the source:

- **JSF 175 (use plain `0`, not `NULL`):** the project **enables `modernize-use-nullptr`**, which enforces the
  *inverse*. Any doc/audit citing "JSF 175 enforced/passed" **over-claims**. Disposition = record the supersession
  (deliberate C++20 `nullptr`) in the deviation log → cleared pre-walk (work plan Phase C).
- **JSF 182 (no casting to/from pointers):** the RP2350 bare-metal target **structurally requires** pointer casts
  (MPU setup, HW registers, CFSR reads); `pro-type-reinterpret-cast` is **SKIPPED for exactly this**. So nothing
  flags pointer casts — neither the mandated ones nor an avoidable one. Standing deviation that must be **logged,
  not silently passed** → accepted-pointer-cast list (Phase C).

Grey cluster (tool surfaces a candidate subset, residual is human): 177 user-defined conversions, 178/179
downcasts, 180/184 lossy/float→int, 181 redundant casts, 171/173/174 pointer provenance/escape/null, 215 pointer
arithmetic. JPL 26 (≤2 indirection) + 27 (≤2 deref) are Det but ungated. JSF 185 (named casts) IS gated.

### What to look for

Each `reinterpret_cast` / C-style pointer cast: **is it the mandated HW/MPU/CFSR kind, or avoidable?** Implicit
**lossy/narrowing** conversions; **pointer arithmetic** (215, gate SKIPPED); >2 levels of indirection.

### ▸ Agent-tendency watch

This is where "compiles + passes" diverges *most* from "correct," because the gate is **deliberately blind**
(reinterpret/pointer-cast checks SKIPPED for HW reasons). An agent reaches for `reinterpret_cast` or a C-style
cast to "make it compile," and **nothing flags it** — so an avoidable cast looks identical to a mandated one. An
agent also cites "`modernize-use-nullptr` clean" as JSF-175 compliance (the inverse), and introduces silent
narrowing conversions the compiler permits. Treat every pointer cast as a thing to *justify*, not assume.

### Grep recipe (candidates)

```bash
grep -rnE 'reinterpret_cast<|\(\s*[A-Za-z_][A-Za-z0-9_:<> ]*\*\s*\)' src/   # pointer casts: classify each HW vs avoidable
grep -rnE '\bNULL\b|=\s*0\s*;' src/                                         # NULL/0-as-pointer (175 over-claim disposition)
grep -rnE '\*\*+' src/                                                      # >1 level indirection candidates (JPL 26)
```

### Judging criteria

| Situation | Verdict |
|---|---|
| Pointer cast that IS the mandated HW/MPU/CFSR kind, and on the accepted-cast list | **PASS** by-reference |
| Same, but no accepted-pointer-cast list exists yet | **PARTIAL** — log it (Phase C authors the list) |
| Pointer cast that is **avoidable** (not HW-mandated) | **FAIL** — redesign to avoid |
| `nullptr` usage cited anywhere as "JSF 175 compliant" | **FAIL (over-claim)** — record supersession |
| Implicit lossy/narrowing conversion | **PARTIAL/FAIL** — make explicit or fix |

### Findings

| File:line | Cast / conversion | HW-mandated? | On accepted-list? | Verdict | Disposition |
|---|---|---|---|---|---|
| | | | | | |

---

## Class 7 — Class & interface design  *(Manual — criteria via §RP)*

### In-scope rules (triage §4b JSF 64–97 cluster)

Almost all **Manual** (design judgment) or **Grey**: interface complete-and-minimal (64), struct-vs-class by
invariant (65/66), **no public/protected data** (67, AST-decidable but ungated), Rule-of-X special members
(68/76 — `cppcoreguidelines-special-member-functions` ENABLED, surfaces the candidate set; "needed?" is human),
const member fns (69), friend justification (70), RAII (79), self-assignment (81), virtual dtor when virtual fn
(78, `-Wnon-virtual-dtor`), is-a/has-a/LSP (91/92/93). Most "classes" here are QP/C AOs or POD-ish structs.

> **§RP fills this class** with concrete C++-Core-Guidelines-sourced criteria before the walk. Until then, treat
> the below as the skeleton.

### What to look for

Classes with **public/protected data** + getters that add nothing; pointer-owning classes missing copy/assign or
`=delete` (76); non-RAII resource handling (79); virtual function with **no virtual dtor** (78); public
inheritance that isn't is-a (91).

### ▸ Agent-tendency watch

Agents generate **public-data classes wrapped in pass-through getters/setters** (violates 67 + 123 and adds
nothing), bolt on the **full Rule-of-5 boilerplate even when the class owns nothing** (speculative — violates
AK_GUIDELINES §2 simplicity), and **forget the virtual dtor** when adding a virtual function. All compile, all
pass; all are design smells a test never sees.

### Grep recipe (candidates — then eyeball the class defs in `include/`)

```bash
grep -rnE '^\s*(public|protected):' include/ src/   # then check for raw data members under public/protected
# special-member-functions + -Wnon-virtual-dtor already surface candidates; this class is mostly eyeball.
```

### Judging criteria

| Situation | Verdict |
|---|---|
| Interface minimal+complete, invariants guarded, RAII clean | **PASS** |
| Public/protected data member (not a deliberate POD struct, 65) | **FAIL/PARTIAL** — encapsulate |
| Pointer-owning class missing copy/assign decl or `=delete` (76) | **PARTIAL** |
| Spurious Rule-of-5 on a class that owns nothing | **PARTIAL** — remove (simplicity) |
| Virtual fn without virtual dtor (78) | **FAIL** |
| Public inheritance that isn't is-a (91) | **FAIL** — remodel as has-a |

### Findings

| File:line | Type | Smell (64–97) | Verdict | Disposition |
|---|---|---|---|---|
| | | | | |

---

## Class 8 — Templates  *(Manual/Grey — criteria via §RP)*

### In-scope rules (triage §4b JSF 101/102/103/105/106)

101 templates reviewed (Manual — mandated activity), 102 tests cover instantiations (Grey), 103 constrain
template args (Grey), 105 minimize instantiation-context dependency (Grey), 106 pointer-type specializations
where apt (Grey). **Low population** — template use is limited (e.g. `lm_solver.h` dispatches residual/jacobian
via deduced callables, the FP-1 resolution). Find the templates, check each.

### What to look for

**Unconstrained** templates (no concept / `static_assert` / `enable_if` guarding the type); template
**instantiations not covered by a test** (102); templates added for **single-use** code.

### ▸ Agent-tendency watch

Agents write **unconstrained** templates that accept wrong types and explode with cryptic errors deep in
instantiation, and add template "flexibility" for **single-use** code (AK §2 — speculative generality). Cheap to
catch here because there are few templates; don't skip it for being small.

### Grep recipe (candidates)

```bash
grep -rnE 'template\s*<' src/ include/   # every template; check constraint + test coverage per hit
```

### Judging criteria

| Situation | Verdict |
|---|---|
| Constrained (concept/static_assert) + each instantiation tested + single clear purpose | **PASS** |
| Unconstrained but used narrowly with safe types | **PARTIAL** — add a constraint |
| Speculative generality for single-use code | **PARTIAL** — simplify |
| Instantiation with no covering test (102) | **PARTIAL** — add the test |

### Findings

| File:line | Template | Constrained? | Instantiations tested? | Verdict | Disposition |
|---|---|---|---|---|---|
| | | | | | |

---

## Class 9 — Control-flow discipline

### In-scope rules (triage §4c/§4d JSF 113/114/191/192/198/199/201/204/205)

113 single function exit (**JSF-only — P10/JPL do NOT subsume it; they tolerate early returns**), 114 single
return/exit-discipline (Grey), 191 break-only-to-terminate-switch (Split), 192 if/else-if final-else-or-comment
(Manual), 198/199 for-init/increment single-param (Grey), 201 loop-counter-not-modified-in-body (Grey), **205
`volatile` only for hardware** (Grey). (`continue` ban is JSF 190 → §LV.)

### What to look for

**JSF 205 is the high-value one.** True MMIO (NVIC `ISPR`, inline asm) is compliant; but **borderline
cross-core/ISR-sync `volatile`** — `gps` ring head/tail, the `rc_log` ring, `g_t2_*` flags,
`g_phase_observable_pair` — needs judgment against the "hardware interface" clause **and** against
[MULTICORE_RULES.md](../MULTICORE_RULES.md) ("`volatile` is NOT a memory barrier" / LL 8). Also: 113 single-exit
(confirm whether the project enforces it or deliberately uses early-return style → disposition, not 200 rewrites);
201 loop counters modified in the body.

### ▸ Agent-tendency watch

The dangerous agent habit: using **`volatile` as if it synchronized across cores** — it does **not** on ARM
(needs `atomic`/spinlock/barrier per MULTICORE_RULES + LL 8). This compiles, usually "works" in testing, and is a
real cross-core corruption bug. Agents also scatter **early returns** freely (fine in modern style — but it means
JSF 113 must be *dispositioned as not-enforced*, not silently marked FAIL across the tree).

### Grep recipe (candidates)

```bash
grep -rnE '\bvolatile\b' src/ include/   # classify each: true MMIO/asm (PASS) vs cross-core/ISR flag (judge vs MULTICORE_RULES)
grep -rnE '\breturn\b' src/              # single-exit (113): count per fn only to DECIDE the project-wide disposition, not per-site
```

### Judging criteria

| Situation | Verdict |
|---|---|
| `volatile` on true MMIO / inline-asm register | **PASS** |
| `volatile` used for cross-core/ISR data sharing as if it were a barrier | **FAIL** — use atomic/spinlock/FIFO (LL 8) |
| Multiple `return`s via modern early-return style | **PASS** under a recorded "113 single-exit not enforced" deviation |
| Loop counter modified inside the body (201) | **PARTIAL** — restructure |
| `if/else-if` chain with no final `else`/comment (192) | **PARTIAL** — add the explicit final branch |

### Findings

| File:line | Construct | Issue (volatile-sync / early-exit / counter / else) | Verdict | Disposition |
|---|---|---|---|---|
| | | | | |

---

## Class 10 — Concurrency & shared-data ownership  *(highest consequence — criteria via §RP)*

### In-scope rules (triage §3 JPL 6/7/8/9 — the canonical Manual class)

6 IPC discipline, 7 thread safety (**no delay-as-synchronization**), 8 shared data (**single owner / owner-only
mutation** — `avoid-non-const-global` deliberately SKIPPED because `g_` globals are by design), 9
semaphores/locking (avoid locks; documented order; **unlock in the same fn as lock**). **No tool decides any of
these.** SPIN is *adjacent* (LTL on AO state machines — proves specific properties, not these rules).

### What to look for

Walk **every cross-core shared object**: who **owns** it, who **mutates** it, what **barrier** protects it.
Project model: Core 0 = main/USB/AO, Core 1 = sensor loop; sharing via spinlock / FIFO / double-buffer
(MULTICORE_RULES). Lock/unlock split across functions (JPL 9 C); `sleep_ms`/`busy_wait` used to "wait for"
another core (JPL 7).

### ▸ Agent-tendency watch

**This is the class AI agents are worst at, by construction.** Concurrency defects (a) compile, (b) pass
single-threaded host tests, (c) pass most hardware runs, and (d) surface only as rare timing-dependent
corruption — so an agent gets *zero* mechanical signal that anything is wrong. Agents share data via plain
`volatile` or a plain global with **no barrier**; use `sleep_ms` as a **synchronization** mechanism; **mutate
shared state from both cores** with no single-owner discipline; and **split lock/unlock** across functions. The
project's entire cross-core LL history (8, 22, 28, 31, 39, 42) is exactly this — bugs that passed every gate.
Spend the most eyeball time here.

### Grep recipe (candidates)

```bash
grep -rnE '\bg_[a-z][A-Za-z0-9_]*' src/                # shared globals: for each, name the owner + the barrier
grep -rn  'spin_lock_blocking\|spin_unlock' src/        # confirm lock & unlock are in the SAME function (JPL 9C)
grep -rn  'sleep_ms\|busy_wait' src/                    # each: real timing requirement, or delay-as-sync (JPL 7 FAIL)?
```

### Judging criteria

| Situation | Verdict |
|---|---|
| Cross-core shared data, single owner, proper barrier (spinlock/atomic/FIFO/double-buffer) | **PASS** |
| Shared mutable state via plain `volatile` or plain global, no barrier | **FAIL** (LL 8 class) |
| `sleep_ms`/`busy_wait` used to synchronize with another thread/core | **FAIL** (JPL 7) |
| Single-owner object mutated from a non-owner core | **FAIL** (JPL 8) |
| `lock` and `unlock` in different functions | **PARTIAL** (JPL 9C) — pair them in one function |

### Findings

| File:line | Shared object | Owner | Barrier | Mutated by | Verdict | Disposition |
|---|---|---|---|---|---|---|
| | | | | | | |

---

## Class 11 — Preprocessor judgment residual

### In-scope rules (triage §3 JPL 20, §4a JSF 8/10/20/29/30/31)

JPL 20 (limit preprocessor; "simple macro" boundary is human), JSF 8 (ISO C++ conformance — C++20 + asm/reinterpret
carve-outs), JSF 10 (documented ISO-10646 subset — **artifact not authored**), JSF 20 (four-directive allowlist —
`version.h` legitimately uses `#ifdef`/`#elif` for job-role select), 29/30/31 (no inline/function-like/constant
`#define` — JPL 20-21 govern; the **PP-1 closed set** + the variadic `DBG_*` carve-out are the resolvers).

### What to look for

Each surviving `#define`: **guard / feature-flag / build-identity / documented carve-out** (PASS), or a
**constant** that should be `constexpr` / a **function-like** macro with hazards (FAIL, PP-1 pattern)? Each
`#ifdef` branch: is it **live in some build config**?

### ▸ Agent-tendency watch

Agents reach for `#define` where `constexpr`/`enum`/`inline` fits (PP-1 is the standing cleanup of exactly this),
write **function-like macros with unparenthesized args / side effects**, and add `#ifdef` branches that are
**dead**. Caution: **verify against the build configs before calling an `#ifdef` dead** — the "R-26 dead
conditional in `version.h`" claim was itself **FALSE** (`ROCKETCHIP_JOB_RELAY` is compile-defined at
`CMakeLists.txt:740`). An agent confidently deleting a "dead" branch is its own failure mode.

### Grep recipe (candidates)

```bash
grep -rnE '^\s*#\s*define' src/ include/                       # classify each survivor (guard / flag / DBG_* / build-id / SDK)
grep -rnE '^\s*#\s*(if|ifdef|ifndef|elif|else)' src/ include/  # each branch: live in some config? (check CMake job flags first)
```

### Judging criteria

| Situation | Verdict |
|---|---|
| Survivor is an include guard / feature flag / documented carve-out (`DBG_*`) / build identity | **PASS** |
| Constant `#define` that should be `constexpr`/`enum` | **FAIL** — migrate (PP-1) |
| Function-like macro with unparenthesized args or side effects | **FAIL** |
| `#ifdef` branch dead in **all** build configs (verified against CMake) | **FAIL** — remove |

### Findings

| File:line | Macro / directive | Category | Live in a config? | Verdict | Disposition |
|---|---|---|---|---|---|
| | | | | | |

---

## Class 12 — Header organization

### In-scope rules (triage §4a JSF 34/35/36/37/38/40)

34 header = logically related decls (Manual cohesion), 35 every header has an inclusion guard (Det, ungated —
`llvm-header-guard` not enabled), 36 minimize compilation dependencies (Manual), 37 headers include only what
they need (Grey — IWYU available, **unused**), 38 forward-decl for ptr/ref deps (Grey — IWYU), 40 impl file
includes what it uses (Grey — `misc-include-cleaner` **SKIPPED**). Cross-refs: §LV JSF 27 (`#pragma once`),
§SC JSF 39 (`misc-definitions-in-headers` ENABLED).

### What to look for

Every `.h` has an `#ifndef`/`#define`/`#endif` guard (and **none uses `#pragma once`** — cross-ref §LV); headers
carrying **unrelated** decls (34); **transitive-include reliance** (a file compiles only because some *other*
header pulled the dependency in) and **unused includes** (37/40 — nothing flags these with IWYU off).

### ▸ Agent-tendency watch

Agents **accumulate includes** — they add what they need and never remove what they stopped needing — and **lean
on transitive includes** (the file compiles today only by luck of include order). With IWYU off and
`misc-include-cleaner` skipped, **nothing mechanical flags either**, so the header dependency graph silently rots
while every build stays green. They also drop unrelated decls into a convenient header (34).

### Grep recipe (candidates)

```bash
grep -rLE '#\s*ifndef|#\s*define' include/    # .h files MISSING a classic guard (candidates incl. #pragma once)
grep -rn  '#pragma once' include/ src/         # cross-ref §LV (3 project headers)
# Include hygiene (37/40): IWYU is the tool and it is OFF — spot-check headers for unused / missing-forward-decl.
```

### Judging criteria

| Situation | Verdict |
|---|---|
| Header cohesive, `#ifndef`-guarded, minimal includes, forward-decls where possible | **PASS** |
| Uses `#pragma once` | **FAIL** → route to §LV (JSF 27) |
| Relies on a transitive include / carries unused includes | **PARTIAL** — add the direct include / drop the unused |
| Unrelated decls bundled in one header (34) | **PARTIAL** — split |

### Findings

| File:line | Header | Issue (guard / cohesion / includes) | Verdict | Disposition |
|---|---|---|---|---|
| | | | | |

---

## Class 13 — Magic numbers & literal discipline

### In-scope rules (triage §4c/§4d JSF 147/151/151.1/210/210.1)

151 no magic numbers (**Split:** literal-vs-ignore-list is Det-by-reference against the curated set
`{0,1,2,3,4,8,16,32,64,100,255,256,1000}` + floats + powers-of-2 + bitfield widths; whether a flagged literal
*should* be named is Grey), 151.1 no string-literal modification (Grey), 147 no FP bit-representation use (Grey),
210 no memory-representation assumptions (Grey), 210.1 no access-specifier member-order assumption (Grey).
**Sibling project rule (stricter):** "**No arbitrary numerical values**" (CODING_STANDARDS + SESSION_CHECKLIST)
— every numeric value needs a **sourced** justification (datasheet / SDK / reference impl / confirmed forum post).

### What to look for

Unnamed numeric literals — especially **timeouts, buffer sizes, thresholds, sample rates** — and for each named
*and* unnamed one: **is there a source** for the value? Literals **hiding inside the curated ignore set**
(e.g. `256`) that are contextually magic. Memory-layout assumptions in serialization code (210).

### ▸ Agent-tendency watch

**High blind-spot class.** An agent's single most common habit the project guards against is introducing a
**number with no source** — a timeout, a size, a threshold that "works" but cites no datasheet/SDK basis (the
SESSION_CHECKLIST lists "picking numerical values without a source" as a **red flag**). Agents also **hide magic
numbers in the ignore-list range** (the gate passes `256` silently even when it's contextually a magic buffer
size), and **assume memory layout** in (de)serialization. A number passing the literal gate ≠ a *justified*
number.

### Grep recipe (candidates)

```bash
grep -rnE '[^A-Za-z0-9_."]([0-9]{2,}|[0-9]+\.[0-9]+)' src/   # multi-digit / float literals; filter the curated set, then ask: named? sourced?
```

### Judging criteria

| Situation | Verdict |
|---|---|
| In the curated ignore set, used in an obvious idiom | **PASS** |
| Named constant **with a sourced justification** (datasheet/SDK/comment) | **PASS** |
| Unnamed magic number (threshold/timeout/size) with **no source** | **FAIL** — name it + cite the source (no-arbitrary-numbers rule) |
| A value in the ignore set that is contextually magic | **PARTIAL** — name it for intent |
| Memory-layout / representation assumption (210/210.1) | **PARTIAL/FAIL** |

### Findings

| File:line | Literal | Named? | Sourced? | Verdict | Disposition |
|---|---|---|---|---|---|
| | | | | | |

---

## Class 14 — Expressions & evaluation order

### In-scope rules (triage §3 JPL 18, §4c/§4d JSF 157/162/163/164/166/187/203/204.1/213)

JPL 18 == JSF 204.1 (eval-order invariance), JSF 157 (no side effects in RHS of `&&`/`||`), 162 (no mixed
signed/unsigned arithmetic — `-Wsign-compare` partial, value-sensitive cases slip), **163 (no unsigned
arithmetic — correctly UN-enforced)**, 164/164.1 (shift bounds — Grey), 166 (no `sizeof` on side-effecting expr —
Split), 187 (non-null statements have a side effect — volatile/HW-read residual), 203 (no over/underflow
dependence), **213 (no sub-arithmetic-precedence dependence — math-parens DISABLED per LL 26)**.

**Two deliberate non-findings (do not "fix"):** (1) JSF 163 is intentionally not enforced — a "no unsigned
arithmetic" gate would **over-report** the project's MANDATED fixed-width `uintN_t` / HW-register / bitmask code.
(2) JSF 213 math-parens are DISABLED ([LL 26](../agents/LESSONS_LEARNED.md)): `a*b+c` needs no parens; reserve
parens for bitwise/ternary/logical mixing.

### What to look for

**Signed/unsigned mixing** in comparisons and arithmetic (162 — the genuine bug source: wraparound, off-by-one);
**eval-order dependence** across sequence points (204.1); side effects in `&&`/`||` RHS (157); shift RHS out of
`[0, width-1]` or right-shift of a negative (164/164.1).

### ▸ Agent-tendency watch

Two opposite failure modes, both real: (1) agents **mix signed and unsigned** (the partial gate misses
value-sensitive cases) — a classic source of wraparound bugs that pass tests on the happy path; and (2) agents
**over-correct** the project's deliberate exemptions — sprinkling redundant parens everywhere (against the
disabled 213) or "fixing" mandated `uintN_t` bitmask code to signed (against 163), which is a **real regression**.
Know the two non-findings cold so you neither miss the signed/unsigned bug nor flag the exempt code.

### Grep recipe (candidates)

```bash
grep -rnE '<<|>>' src/                       # shift sites: RHS in [0,width-1]? LHS non-negative? (164/164.1)
# Signed/unsigned mixing: -Wsign-compare catches a subset; eyeball comparisons between int and uintN_t.
```

### Judging criteria

| Situation | Verdict |
|---|---|
| Signed/unsigned mix that can wrap or misbehave | **FAIL/PARTIAL** — make types consistent |
| Eval-order dependence across a sequence point (204.1) | **FAIL** |
| Side effect in the RHS of `&&`/`||` (157) | **PARTIAL/FAIL** |
| Mandated unsigned `uintN_t`/HW/bitmask arithmetic (163) | **PASS** — do NOT convert to signed |
| Plain arithmetic missing "extra" parens (213) | **PASS** — exemption, not a finding |

### Findings

| File:line | Expression | Issue (sign-mix / eval-order / shift) | Verdict | Disposition |
|---|---|---|---|---|
| | | | | |

---

## § LV — Live unlogged violations to disposition

These are **factual present findings** from the triage §7.3 / §4 — not "go hunt," but "confirm the cited site,
then decide log-as-deviation vs remediate." Knock these out directly; each is a known location.

| Rule | Site(s) | Finding | Disposition options |
|------|---------|---------|---------------------|
| **JSF 18** (no `offsetof`) | `src/calibration/calibration_data.cpp:106,119` | `offsetof` used for CRC offset computation, **no deviation row** | analyze the two sites, then (a) log accepted deviation or (b) remediate to a non-`offsetof` form — disposition is yours |
| **JSF 27** (include guards, no other technique) | **3 project headers** (measured): `src/safety/fault_protection.h`, `src/flight_director/mission_profile_data.h`, `include/rocketchip/shared_state.h` | uses `#pragma once` (triage said 1 — **corrected to 3**); the rest use `#ifndef` guards | (a) convert the 3 to `#ifndef`/`#define`/`#endif` to match the convention; or (b) accept `#pragma once` as the convention and log it — but 3 is enough that this is a real "which way is the convention?" call, not one stray outlier |
| **JSF 190** (no `continue`) | **28 sites across 12+ files** (e.g. `ao_telemetry.cpp:348` in a real for-loop) | `continue` used; no gate flags it, no deviation row | JSF-only rule (P10/NASA don't ban `continue`). Options: (a) one blanket accepted-deviation covering it project-wide, or (b) remediate per-site. Disposition is yours |
| **JSF 202** (no float exact `==`/`!=`) | `src/fusion/eskf_runner.cpp:292-295` (real `!= 0.0f` tests) | float equality used; `-Wfloat-equal` not in build, `cert-flp30-c` covers only loop counters | walk the specific sites — confirm each is a deliberate sentinel/guard (often legitimate vs a known-exact 0.0f) → log; or rewrite to an epsilon test if it's a true comparison |

> For 190: a JSF-only rule the project de-facto deviates from (28 sites) with no closed exemption. The two
> dispositions — one project-wide accepted-deviation row vs. per-site remediation — are genuinely a judgment
> call; weigh them at disposition, don't assume either.

---

## § CM — One-shot mechanical checks / conversion moves  *(prep — owned by the work plan)*

> **This is a pre-walk PREP step, not walk-content.** The full conversion-moves catalogue, the council decision
> (measure-first, 3-bin sort, the centralize-into-a-script gating fix, keep `CheckedFunctions` honest), and its
> status live in the **work plan**: `docs/plans/L2P5_WALK_PLAN.md` § Phase B. Making gates/scripts is plan work,
> not part of the eyeball walk.

What the walker needs to know: several decidable-but-ungated rules are converted to gates **before** the walk so
the walk shrinks. Once a rule is gated it moves to **§SC** (cite the gate, don't walk). The known violations
those gates surface are **cleared pre-walk** (work plan Phase C), so you don't re-handle them during the
semantic walk.

---

## § RP — Research pass (fills the Manual-class criteria)  *(prep — owned by the work plan)*

> **This is a pre-walk PREP step, not walk-content.** The research scope, external sources, anti-fabrication
> discipline (LL 37), and status live in the **work plan**: `docs/plans/L2P5_WALK_PLAN.md` § Phase D.

Where a class above says "use judgment / design judgment," that is a **placeholder** — the concrete "what good
looks like / what a finding looks like" criteria for the genuinely-Manual classes (3, 7, 8, 10, + Manual rules in
9/13/14) are produced by the §RP research pass and **folded back into those classes here** before they're walked.
Until that's done, those classes are scaffolds: do **§RP first** for them. A note to look is not knowing what to
look for.

---

## § IT — Walk Itinerary (the file-coverage map / traversal spine)

**Why:** the rule-class sections above are the *lens library*. This itinerary is the *route* — an ordered pass
over **every** in-scope file so coverage is trackable and you never skip around. Per the completeness principle,
record a verdict for **every** file, PASS included (not just FAILs).

**Two-pass model.** (1) **Cross-repo sweeps** — the grep-able / mechanical rules — are done *once* in the work
plan's §CM/§LV (Phases B/C), *before* this walk. (2) **This file-by-file semantic pass** — for each file, read it
whole and apply the **per-file lenses** below with the file in front of you.

**Per-file lens checklist** (apply each to every file; jump to the cited class for criteria):
- **Class 3** — comments restate code / paraphrase a doc / missing "why" / commented-out code.
- **Class 4** — meaningful vs vacuous/decorative assertions; missing preconditions.
- **Class 5** — declaration scope & lifetime; mandated-static respected; two-phase-init; stack-local events.
- **Class 7** — class/interface design (public data, RAII, special members, virtual dtor). *(headers esp.)*
- **Class 9** — control-flow; **`volatile`-as-cross-core-sync** (high value); early-exit style.
- **Class 10** — concurrency: every shared object's owner + barrier; delay-as-sync; lock/unlock pairing. *(highest consequence)*
- **Class 13** — magic numbers / unsourced values.
- **Class 14** — signed/unsigned mixing; eval-order; (don't "fix" the 163/213 exemptions).
- (Classes 2/11/12 are largely cross-repo/over-claim — handled in their sections, not per file.)

**The itinerary itself is the companion checklist** → **`docs/audits/L2P5_WALK_ITINERARY.md`** — all **185**
in-scope files, ordered criticality + execution-path first (safety → fusion → flight_director → active_objects →
core1 → drivers → calibration → logging → diag/notify/telemetry/station → math → top-level → cli last → headers),
each with a checkbox + coverage note + which §LV/class hot-spots live in it. Tick files there as you walk; record
findings in the relevant class's table here. Kept as a separate doc because it's the *heavily-edited progress
tracker*, distinct from this reference manual.

---

## § SC — Genuinely script-covered (revalidated; do NOT hand-walk)

Cite the gate + its last clean run in the remediation writeup; do not eyeball. **Each row was confirmed against
the triage to (a) have a gate that actually runs and (b) fully cover the rule.** Rules that looked covered but
aren't (param-count half of size, return values, shadowing, etc.) were demoted to §CM/Tier-3 above — that
demotion is the council-mandated "don't let green checks imply the semantic rules were covered."

| Rule(s) | Gate | Note |
|---|---|---|
| Recursion (P10-1 / JPL 4 / JSF 119) | clang-tidy `misc-no-recursion` | full |
| No dynamic allocation (P10-3 / JPL 5 / JSF 206) | no-malloc by construction + review | full (CbC) |
| `goto` / labels (P10-1 / JPL 11 / JSF 188/189) | `avoid-goto` + GT-1 closed | full |
| `<stdio.h>` ban (JSF 22) | pre-commit include-ban (R-5, unconditional) | full |
| Fixed-width types (JPL 17 / JSF 209) | `google-runtime-int` | full |
| Function **length** ≤60/≤200 (P10-4 / JPL 25 / JSF 1) | `readability-function-size` LineThreshold + full-tree sweep (checklist 17) | length half only — **param half is §CM** |
| Braces on control bodies (JSF 59) | `braces-around-statements` (SSL=0) | full |
| Switch default / fallthrough (JSF 193/194/196) | `-Wimplicit-fallthrough` + `switch-missing-default` + `-Wswitch` | full |
| Float loop counters (JSF 197) | `cert-flp30-c` | full |
| Definitions in headers (JSF 39) | `misc-definitions-in-headers` ENABLED | full |
| `//`-only comments (JSF 126) | lexer/token check | full |
| Cyclomatic/cognitive (JSF 3) | `readability-function-cognitive-complexity` (T=25) + lizard | **honest-narrow** — proxy, self-disclaimed; treat as low, not full |

> Full PASS-BY-SCRIPT list lives implicitly in the triage's Det-direct/by-reference rows that have an enabled,
> full-coverage gate. When writing the remediation doc, cite the specific gate + last clean run per the council
> amendment that requires the "NOT mechanically covered" inverse to be explicit, so green checks don't imply
> the manual rules were walked too.

---

## Source PDFs (canonical, for verbatim when a finding needs the exact wording)

- JSF AV C++ Rev C: https://www.stroustrup.com/JSF-AV-rules.pdf
  *(terse rule statements are in the early summary section; the later appendix only has rules needing
  examples and SKIPS others — extract text with pypdf and grep "AV Rule N" rather than paging.)*
- Power of 10: https://spinroot.com/gerard/pdf/P10.pdf
- JPL C (D-60411): https://yurichev.com/mirrors/C/JPL_Coding_Standard_C.pdf
- Helper: `scripts/audit/pdf_section_lookup.py <pdf> <section>` (confirms a section exists; not ideal for
  terse JSF rule statements).
- **The triage is the first stop for rule text + "why":** `docs/audits/RULE_VERIFIABILITY_TRIAGE.md`
  §2 (P10), §3 (JPL), §4a–d (JSF), §7 (findings). Primary-source-verified; this guide leans on it rather than
  re-quoting.
