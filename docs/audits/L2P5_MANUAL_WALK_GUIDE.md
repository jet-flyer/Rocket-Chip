# L2-P5 Manual Standards-Walk Guide

**Status:** WORKING DOC — in progress. Human-driven walk guide, not a finished audit.
**Owner:** Nathan (manual walk). Agents may build/refine rule-class sections; the walk itself + dispositions are Nathan's.
**Established:** 2026-06-06. **Scope re-set:** 2026-06-20 (triage-driven, see Scope below).
**Closes:** PR **L2-P5** in `docs/PROBLEM_REPORTS.md` (feeds the Cycle-4 remediation doc per `docs/plans/CYCLE_RESIDUALS_AFTER_R5.md`).
**Driver:** `docs/audits/RULE_VERIFIABILITY_TRIAGE.md` — the frozen, primary-source-verified per-rule classification. This guide is the *operational layer* over that triage; it does not re-quote rule text, it points to the triage's detail rows for verbatim + "why."

---

## What this doc is (and is not)

**Is:** a field guide for manually walking every coding-standard rule that the tooling **cannot fully and correctly evaluate** — the semantic rules a human has to eyeball, the rules where a green check is misleading, and the mechanically-decidable rules that simply aren't gated yet. For each walk-class: the in-scope rules (with triage cross-refs), the anchor rule(s) verbatim, a concrete "what to look for," a grep recipe to *find candidates* (not to judge them), judging criteria, and a findings table you fill in as you go.

**Is not:** a re-run of rules a trustworthy script already evaluates in full. Those are PASS-BY-SCRIPT — cite the gate + last clean run, don't eyeball them. See "Genuinely script-covered" at the bottom. **But** "a script exists" ≠ "script-covered" — several gates over-claim (enforce the inverse, or cover only a curated subset). Those move *into* the walk. The triage's enforcement diagnostic is the arbiter.

**Why a guide instead of just doing it:** the walk spans many sittings across a large rule set. Without per-class criteria + a progress record you lose your place and re-derive "what am I even looking for" each time. This doc holds both, and the triage holds the verbatim rule text + the reason each rule lands where it does.

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
| 4 | Assertions | P10-5, JPL 16 (§2, §3) | ☐ indexed |
| 5 | Declaration scope & object lifetime | P10-6, JPL 13, JSF 136/142/143, 70.1/71 (§2,§3,§4b,§4c) | ☐ indexed |
| 6 | Pointers, casts & conversions | P10-9, JPL 26/27/28/30, JSF 169/171/173/174/177–184/215 (§4d); over-claims 175/182 §7.1 | ☐ indexed |
| 7 | Class & interface design | JSF 64–97 cluster (§4b) | ☐ indexed |
| 8 | Templates | JSF 101/102/103/105/106 (§4b) | ☐ indexed |
| 9 | Control-flow discipline | JSF 113/114/191/192/198/199/201/204/205 (§4c,§4d) | ☐ indexed |
| 10 | Concurrency & shared-data ownership | JPL 6/7/8/9 (§3) | ☐ indexed |
| 11 | Preprocessor judgment residual | JPL 20, JSF 8/10/20/29/30/31 (§3,§4a) | ☐ indexed |
| 12 | Header organization | JSF 34/35/36/37/38/40 (§4a) | ☐ indexed |
| 13 | Magic numbers & literal discipline | JSF 147/151/151.1/210/210.1 (§4c,§4d) | ☐ indexed |
| 14 | Expressions & evaluation order | JPL 18, JSF 157/162/163/164/166/187/203/204.1/213 (§3,§4c,§4d) | ☐ indexed |

Action sections (not per-file judgment walks):

| § | Section | Built? |
|---|---------|--------|
| LV | Live unlogged violations to disposition (JSF 18, 27, 190, 202) | ☑ **built** |
| CM | One-shot mechanical checks / conversion moves | ☑ **built** |
| SC | Genuinely script-covered (revalidated; do NOT walk) | ☑ **built** |

> **Build order rationale:** Class 1 is the worked template. Classes 2 and 3 are built next because 2 is a *critical over-claim* (the gate actively encodes a non-JSF scheme while docs cite JSF conformance) and 3 is the "comments restating code / commented-out code / documenting external sources" smell-cluster that motivated the separate agent-walk idea. The remaining Tier-3 classes are indexed (rules + triage refs assigned) and get built to the Class-1 template on the next pass.

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

> **Why this is in the walk, not script-covered (triage §7.4, THE CANARY):** `bugprone-unused-return-value`
> has **no project `CheckedFunctions`** configured and `cert-err33-c` covers only the C stdlib — so exactly
> the bus/flash/GPS/log functions above are **uncovered**. A clang-tidy-clean run does NOT mean Rule 7/14/115
> passed. This is the audit's designated canary; walk it by hand until `CheckedFunctions` is populated (see
> §CM, which converts this to Det-by-reference).

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
2. Confirm the doc over-claim exists (it does — `CODING_STANDARDS.md:403`).
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

## Classes 4–14 — indexed, build pending

Each is assigned its in-scope rules + triage cross-refs in the Class Index above. They get built to the Class-1
template (anchor rule verbatim → what-to-look-for → grep recipe → judging criteria → findings table) on the next
pass. Highest-value-next, in order:

- **Class 6 (Pointers/casts/conversions)** — carries two more *critical over-claims*: **JSF 175** (gate enforces
  `nullptr`, the literal inverse of "use plain 0" — disposition = deliberately-not-followed) and **JSF 182**
  (pointer casts structurally required by the bare-metal target; `pro-type-reinterpret-cast` SKIPPED; standing
  deviation must be logged). Triage §4d + §7.1.
- **Class 5 (Declaration scope & lifetime)** — P10-6/JPL 13/JSF 136/143, with the **LL Entry 1 mandated-static
  override** (a tool flagging mandatory file-scope statics for >1KB objects is a *false positive* — that subset
  is by-reference, the rest is Grey). Triage §2/§3/§4c.
- **Class 10 (Concurrency & shared-data ownership)** — JPL 6/7/8/9, the canonical Manual class; SPIN is adjacent,
  not a verifier. Triage §3.
- **Class 4 (Assertions)** — P10-5 over-claim (density + non-vacuity have no tool). Triage §2.
- Then 7 (class design), 8 (templates), 9 (control-flow), 11 (preprocessor), 12 (headers), 13 (magic numbers),
  14 (expressions/eval-order).

---

## § LV — Live unlogged violations to disposition

These are **factual present findings** from the triage §7.3 / §4 — not "go hunt," but "confirm the cited site,
then decide log-as-deviation vs remediate." Knock these out directly; each is a known location.

| Rule | Site(s) | Finding | Disposition options |
|------|---------|---------|---------------------|
| **JSF 18** (no `offsetof`) | `src/calibration/calibration_data.cpp:106,119` | `offsetof` used for CRC offset computation, **no deviation row** | (a) log accepted deviation (legitimate CRC-offset idiom) — likely; or (b) remediate to a non-`offsetof` form |
| **JSF 27** (include guards, no other technique) | `include/rocketchip/shared_state.h:~16` | uses `#pragma once`; 32/33 other headers use `#ifndef` guards | (a) convert to `#ifndef`/`#define`/`#endif` to match the rest — likely trivial fix; or (b) log deviation if `#pragma once` is intended project-wide (then it's the *other* 32 that are the convention) |
| **JSF 190** (no `continue`) | **28 sites across 12+ files** (e.g. `ao_telemetry.cpp:348` in a real for-loop) | `continue` used; no gate flags it, no deviation row | (a) blanket accepted-deviation (modern style tolerates `continue`; P10/NASA don't ban it — JSF-only rule) — likely; or (b) remediate per-site (heavy, low value) |
| **JSF 202** (no float exact `==`/`!=`) | `src/fusion/eskf_runner.cpp:292-295` (real `!= 0.0f` tests) | float equality used; `-Wfloat-equal` not in build, `cert-flp30-c` covers only loop counters | walk the specific sites — confirm each is a deliberate sentinel/guard (often legitimate vs a known-exact 0.0f) → log; or rewrite to an epsilon test if it's a true comparison |

> For 190 especially: this is the classic "JSF-only rule the project de-facto deviates from without a closed
> exemption." The cheap honest move is one accepted-deviation row covering `continue` project-wide, not 28
> rewrites — but that's Nathan's call at disposition.

---

## § CM — One-shot mechanical checks / conversion moves

These rules are **decidable but currently ungated** — the triage's "conversion moves." Don't re-judge them per
file; do each **once** (run the grep, or flip the check on), and several stop being walk items forever. Batch them.

| Rule(s) | One-shot action | Effect |
|---------|-----------------|--------|
| **P10-7 / JSF 115 / JPL 14** (return values) | Populate `.clang-tidy` `bugprone-unused-return-value.CheckedFunctions` with the project API (`i2c_bus_*`, `flash_safe_execute`, `gps_*`, `rc_log*`) | Converts THE CANARY from a hand-walk (Class 1) to **Det-by-reference**. Biggest single win. |
| **JSF 135 / JPL 13** (shadowing) | Add `-Wshadow` to the build (it is NOT in `-Wall`/`-Wextra`) | Fully decidable, currently un-enabled → becomes script-covered |
| **JSF 48** (no confusable identifiers) | Enable `misc-confusable-identifiers` (currently SKIPPED) | becomes script-covered |
| **JSF 175** (use plain `0`) | Write the supersession to the deviation log: project deliberately uses `nullptr` (C++20), `modernize-use-nullptr` enforces it | Converts over-claim → Det-by-reference (honest "deliberately-not-followed") |
| **JSF 182** (no pointer casts) | Author an accepted-pointer-cast list (MPU setup, HW registers, CFSR reads) + log the deviation | Converts over-claim → Det-by-reference |
| **JSF 45/51/52** (naming) | Record the project-convention supersession (see Class 2) | Removes the doc over-claim |
| **JPL 25 param-half** (≤6 params) | Configure `readability-function-size` `ParameterThreshold` (currently unset) | param-count half becomes gated |
| **JPL 22** (`#undef` ban) | One grep (`grep -rn '#undef' src/ include/`) → expect zero; add a cheap check or confirm CbC | closes the gap |
| **JSF 10** (documented ISO-10646 subset) | Author the subset artifact (effectively ASCII) once | makes the rule checkable |
| **JSF 6** (deviation annotated in-file) | Add a grep cross-referencing the central deviation log vs in-file annotations | Converts Grey → Det |

> Doing the top two (CheckedFunctions + `-Wshadow`) alone removes a real chunk of Tier-3 walking. Recommend
> running §CM **before** the heavy semantic classes so the walk shrinks first.

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
