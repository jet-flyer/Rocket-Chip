# RocketChip Unified Master — Rule-Verifiability Triage

**Status:** Working master — generated 2026-06-17 via multi-agent workflow (gather → classify → adversarial-verify [full 14/14 batch coverage] → repair → synthesize). Supersedes the prior P10-only calibration draft. Section 7 findings (live unlogged violations + naming-citation over-claim) are to action; the section 5 cross-reference is mirrored into `standards/CODING_STANDARDS.md` per repo-owner authorization.
**Corpus:** Power of 10 (10) + JPL C LOC-1..4 (31) + JSF AV C++ (233 distinct IDs); 225 triaged property-rows. JPL LOC-5/6 (89 MISRA-absorption rules) deferred-with-rationale (section 6).
**Provenance:** classify cache reused across resumes; rule text primary-source-verified; 14 truncated JSF extractions repaired; full result in workflow run `wf_65c7fe01-f1b`.

---

Single audited surface over **Power of Ten (10) + JPL C LOC-1..4 (31) + JSF AV C++ (221-numbered, 233 distinct IDs)**. Classification is frozen per the audited verifiability-triage contract; verifier corrections (audit-of-record) have been applied and the 14 truncated JSF rules replaced with their repaired verbatim + classification.

## 1. Method & Bucket Recap (reference only)

**Precedence:** newer overrides older where it speaks; silence leaves the older standing. JPL C (2009) ▸ P10 (2006) ▸ JSF (2005). When >1 standard covers the SAME property the newest **governs**; the others become **pointer-rows** citing source verbatim. Most JSF rules are JSF-only C++ specifics with no newer cousin ⇒ JSF governs them.

**No "N/A" bucket.** Apparent-N/A = *compliant-by-construction* (banned/unused feature, or enforced-by-language) — stays in scope as a cheap one-line row.

**Buckets**
- **Det-direct** — one tool decides it from syntax/a metric.
- **Det-by-reference** — mechanical, but the answer is fixed elsewhere (another rule, the closed deviation log, a platform constraint, or a cross-TU/grep scan).
- **Grey** — tool gives partial purchase (candidate shell) but a human judges the unrepresentable "why." *Deep analysis lives here.*
- **Manual** — no tool purchase; judgment end-to-end.
- **Split** — a rule bundling properties in different buckets; classify each property.
- **Compliant-by-construction** — banned/unused/enforced-by-language; cheap row.

**Enforcement diagnostic (finding):** no tool = honest gap (**none**, least-worrying); narrow/curated tool not presented as full coverage = **honest-narrow (low)**; tool/gate silent or presented as full coverage of a not-fully-decidable rule = **over-claiming (critical)**. Hinge = intention/explicitness. Tool verdicts are untrustworthy 3 ways: under-report (false pass), over-report (flags mandated code), mute.

**Conversion move:** a rule Grey *only* because the resolving artifact (exemption list / convention / ignore-list) doesn't exist yet converts Grey → Det-by-reference once that artifact is created.

---

## 2. Power of Ten (P10, 2006) — 10 rules

| # | rule | governs | bucket | enforcement |
|---|------|---------|--------|-------------|
| 1 | control flow (goto/recursion/setjmp) | self | Det-direct | none — misc-no-recursion + avoid-goto |
| 2 | loop bounds | self | Det-by-reference | **low** — no fixed-bound prover; exemption set resolves |
| 3 | no dynamic allocation | self | Det-direct | none — no-malloc; no-heap by construction |
| 4 | function length | self | Det-direct | none — function-size LineThreshold 60 |
| 5 | assertions | self | **Split** | **critical** — density/non-vacuity ungated |
| 6 | smallest scope | self | Grey | low — adjacent checks only |
| 7 | return values + params | self | **Split** | **critical** — CANARY, project fns uncovered |
| 8 | preprocessor | self | Det-direct | low — variadic DBG_* carve-out |
| 9 | pointers (deref/fn-ptr) | self | **Split** | low — fn-ptr by-reference (FP-1) |
| 10 | warnings / static analysis | self | Det-by-reference | low — "daily" cadence is honor-system |

**Detail (Grey/Split/finding rules):**

- **P10-2 (Det-by-ref, low):** No tool proves a static iteration bound; `bugprone-infinite-loop` catches only a subset. The intentionally non-terminating loops (QF_run, Core 1 sensor loop, fault halt) are resolved by the **closed exemption set** in `ACCEPTED_STANDARDS_DEVIATIONS.md "Note on P10 Rule 2"` — the lookup that makes this by-reference, not Grey. Already-converted (the artifact exists).
- **P10-5 (Split, critical):** Property A density (≥2/fn) = Grey (counting mechanical, "meaningful"/which-macros not); Property B side-effect-free = Det-direct (`bugprone-assert-side-effect`); Property C non-vacuous Boolean = Manual. **Critical**: density + non-vacuity have NO tool yet are core to the rule — any presentation of the gate as covering P10-5 over-claims.
- **P10-6 (Grey, low):** `misc-const-correctness` / `readability-isolate-declaration` / deadcode give adjacent purchase; none decides "smallest possible." **Over-report hazard:** LL Entry 1 MANDATES static file-scope for >1KB objects — a tool flagging those as too-broad would be a false positive. That mandated subset is a by-reference lookup; the rest is Grey.
- **P10-7 (Split, critical) — THE CANARY:** Property A return-checked = Grey; Property B param-validity = Manual; (void)-cast carve-out = Det-by-reference. `bugprone-unused-return-value` (NO project `CheckedFunctions`) + `cert-err33-c` (C-stdlib only) leave `i2c_bus_*`, `flash_safe_execute`, `gps_*`, `rc_log` **uncovered** — the **LL 28/31/41 dropped-bus-return false-negatives** slip through. Converts to Det-by-reference once `CheckedFunctions` is populated.
- **P10-8 (Det-direct, low):** macro-to-enum/parentheses/side-effect checks enabled; token-pasting/recursive macros absent by construction; R-26 dead `#ifdef` is a grep (by-reference). Low because the project deliberately keeps variadic `DBG_*` (`##__VA_ARGS__`) with `-Wno-gnu-zero-variadic-macro-arguments` against the literal "no ellipses" clause — documented carve-out.
- **P10-9 (Split, low):** Property A single-deref = Grey, Property B hidden-deref = Manual/Grey, Property C function-pointer-ban = Det-by-reference (FP-1 resolved 2026-05-13; P10-9 GOVERNS over JSF 176 per LL Entry 37). No tidy check enforces deref-depth or the fn-ptr ban directly; policed by deviation log + review.
- **P10-10 (Det-by-ref, low):** pedantic `-Wall -Wextra -Werror` build + clang-analyzer family + cppcheck + SPIN. Verdict is a mechanical multi-source lookup. Low: the "checked daily" cadence is an honor-system process claim (LL Entry 36 rot pattern), not mechanically self-verifying.

---

## 3. JPL C LOC-1..4 (2009) — 31 rules

| # | rule | governs | bucket | enforcement |
|---|------|---------|--------|-------------|
| 1 | language conformance / no-UB | self (cites MISRA 1.1/1.2) | **Split** | low — UB only partly analyzer-reachable |
| 2 | routine checking (warnings+SA) | **P10-10 governs** (pointer-row, MISRA 21.1) | Det-by-reference | low — mirrors P10-10 |
| 3 | loop bounds | **P10-2 governs** (pointer-row) | Det-by-reference | low — `@non-terminating@` is the resolver |
| 4 | recursion | **P10-1 governs** (pointer-row, MISRA 16.2) | Det-direct | none — misc-no-recursion |
| 5 | heap memory | **P10-3 governs** (pointer-row, MISRA 20.4) | Det-direct | none — no-malloc |
| 6 | IPC discipline | self | Manual | none — honest gap (SPIN adjacent) |
| 7 | thread safety (no delay-as-sync) | self | Manual | none — honest gap |
| 8 | shared data (single-owner) | self | Manual | none — avoid-non-const-global SKIPPED |
| 9 | semaphores/locking | self | **Split** | none — same-fn-pairing convertible |
| 10 | memory protection | self | Det-by-reference | low — MPU mandate + HW bench, not SA |
| 11 | goto / setjmp | **P10-1 governs goto** (pointer-row) | **Split** | none — avoid-goto; setjmp CbC |
| 12 | enum init | self (MISRA 9.3) | Det-direct | low — enabled check adjacent, not exact |
| 13 | limited scope (shadow + smallest) | self; subsumes P10-6 | **Split** | low — shadow gated, smallest Manual |
| 14 | checking return values | self (P10-7/JSF 114 pointer-rows) | Grey | **low** ⟵ *was critical; verifier downgrade* |
| 15 | checking parameter values | self (P10-7 pointer-row) | Grey | none — no entry-check tool |
| 16 | assertions | self (P10-5 pointer-row) | **Split** | low — side-effect gated, density not |
| 17 | fixed-width types | self (JSF 209 pointer-row, MISRA 6.3) | Det-direct | none — google-runtime-int |
| 18 | order of evaluation | self (MISRA 12.2) | Grey | low — math-parens DISABLED (JSF 213) |
| 19 | boolean side effects | self (MISRA 13.1) | Det-direct | none — assignment-in-if-condition |
| 20 | preprocessor use | self; P10-8/JSF 26-31 pointer-rows | Grey | low — "simple macro" boundary human |
| 21 | macro definition (in fn/block) | self (MISRA 19.5) | Det-direct | low — greppable, ungated |
| 22 | macro undef ban | self (MISRA 19.6) | Det-direct | low — grep; CbC in practice |
| 23 | conditional directives same-file | self (MISRA 19.17) | Det-by-reference | low — directive-matching pass |
| 24 | one statement / decl per line | self (JSF 152 pointer-row) | Det-direct | none — isolate-declaration + format |
| 25 | function size (≤60 lines, ≤6 params) | self; P10-4/JSF 1 pointer-rows | **Split** | low — staged-only; param half UNGATED |
| 26 | indirection levels (≤2) | self (JSF 170 pointer-row, MISRA 17.5) | Det-direct | none — no declarator-depth check |
| 27 | dereference levels (≤2/stmt) | self; P10-9/JSF 215 pointer-rows | Det-direct | none — no deref-depth check |
| 28 | dereference hiding | self (P10-9 family) | Grey | none — honest gap |
| 29 | function pointers | **P10-9 governs** (pointer-row); JSF 176 older | Det-by-reference | none — FP-1 resolved |
| 30 | type conversion (fn-ptr→integral) | self (MISRA 11.1) | Det-direct | low — reinterpret_cast HW carve-out |
| 31 | include directives | self (JSF 35 older) | Det-direct | low — include-cleaner SKIPPED |

**Detail (Grey/Split/Manual/finding rules):**

- **JPL 1 (Split, low):** Property A conformance pinned by `-std=c++20` + pedantic build (Det-by-reference; C99 text read against C++20 target). Property B no-UB = Grey: analyzers (clang-analyzer-core/cplusplus, narrowing, undefined-memory-manipulation, signed-bitwise) catch a subset, never all UB.
- **JPL 6/7/8 (Manual, none):** Multithreading-ownership discipline — the canonical Manual class. No tool decides "all task comms via IPC," "no delay-as-synchronization," or "single owner / owner-only mutation." SPIN is adjacent (LTL on AO state machines), not a verifier of these rules. `avoid-non-const-global-variables` is deliberately SKIPPED (g_ globals by design). Honest gaps — least worrying.
- **JPL 9 (Split, none):** A/B (avoid locks; documented order) = Manual; **Property C (unlock in same fn as lock) = Det-direct in principle / Grey-shell** — a custom matcher pairing `spin_lock_blocking`/`spin_unlock` within a function body would convert it to Det-direct. None enabled.
- **JPL 10 (Det-by-ref, low):** "where available" ⇒ platform lookup: RP2350 MPU available, project mandates `mpu_setup_stack_guard()` on both cores (`fault_protection.cpp`); verified by HW fault-injection bench (R-3 AP=0b00 + MEMFAULTENA), not by SA. Calling it clang-tidy-covered would over-claim; the project does not.
- **JPL 13 (Split, low):** shadowing half = Det-direct (`-Wshadow` *if enabled* — see Findings, it is NOT); smallest-scope half = Manual, with the LL Entry 1 static-large-object by-reference override.
- **JPL 14 (Grey, low):** *Audit-of-record correction:* originally critical; downgraded to **low** because no gate *presents itself* as full return-value coverage — `bugprone-unused-return-value` (no project `CheckedFunctions`) + `cert-err33-c` are honest-narrow stdlib-only checks; the over-claim is hypothetical (an audit reading clang-tidy-clean ⇒ PASS), and the project's own calibration rates it low-to-medium. Converts to Det-by-reference once `CheckedFunctions` enumerates the project API.
- **JPL 15 (Grey, none):** parameter-validity-at-entry is semantic; no enabled check claims it. Honest gap (no narrow over-claiming tool, unlike JPL 14).
- **JPL 16 (Split, low):** side-effect-freedom Det-direct (`bugprone-assert-side-effect`); the ">10-line ⇒ ≥1 assertion" density target is a countable shell currently ungated (`readability-function-size` measures length but doesn't correlate to assertion presence); meaningfulness stays Manual.
- **JPL 18 (Grey, low):** `readability-math-missing-parentheses` is **DISABLED** by documented JSF 213/LL 26 exemption (arithmetic precedence treated as universally understood); `hicpp-signed-bitwise` catches the bitwise sliver. Documented, rationale-backed deviation ⇒ honest, low.
- **JPL 20 (Grey, low):** macro-to-enum/parentheses checks migrate constant macros; classifying each surviving macro as "simple" vs forbidden-complex is human. PP-1 curated remaining-macro set is the effective resolving convention.
- **JPL 21/22 (Det-direct, low):** `#define`-in-block and `#undef`-ban are one-grep predicates; no dedicated enabled check ⇒ honest, trivially-closable gaps. CbC in practice (PP-1; constexpr/enum migration).
- **JPL 23 (Det-by-ref, low):** same-file `#if/#else/#elif/#endif` balance = mechanical directive-matching pass; `readability-redundant-preprocessor` covers a related-but-different redundancy property. R-26 same family.
- **JPL 25 (Split, low):** both ≤60-lines and ≤6-params are pure metrics. Low: hook is staged-files-only (full-tree sweep, checklist item 17, is the backstop; CG-1 accepted deviation). **Coverage caveat (verifier):** `ParameterThreshold` is NOT configured — the param half is Det-direct *in principle* but currently UNGATED; the original note overstated coverage.
- **JPL 28 (Grey, none):** deref-in-macro/typedef has a syntactic seed but "hiding vs legitimate accessor" is the unrepresentable why; no tool flags concealed dereferences.
- **JPL 30 (Det-direct, low):** cast-style + pointer-conversion checks decide dominant cases; deliberate `reinterpret_cast` allowance for MPU/HW registers (`pro-type-reinterpret-cast` SKIPPED) is the narrow uncovered corner.

---

## 4. JSF AV C++ (2005) — 219 base + 14 sub-rules (233 distinct IDs)

*Compact tables in number order; detail blocks follow for Grey/Split/Manual/finding rules. Repaired truncated rules carry the authoritative verbatim/classification.*

### 4a. Rules 1–58 (process, lexical, naming, headers)

| # | rule | governs | bucket | enforcement |
|---|------|---------|--------|-------------|
| 1 | function size (200 L-SLOC) | **P10-4/JPL 25 govern** (pointer-row) | Det-direct | none — StatementThreshold 200 + lizard |
| 2 | no self-modifying code | self | Compliant-by-construction | none — flash/XIP, no JIT |
| 3 | cyclomatic complexity ≤20 | self | Det-direct | **low** — cognitive-complexity proxy, lizard for true CC |
| 4 | should-deviation approval | self | Det-by-reference | none — deviation-acceptance policy |
| 5 | will/shall-deviation approval | self | Det-by-reference | none — council + repo-owner |
| 6 | deviation documented in-file | self | Det-by-reference | low — central log, no in-file cross-check |
| 7 | exception relief clause | self | Compliant-by-construction | none — relief, nothing to scan |
| 8 | ISO C++ conformance, no extensions | self | Grey | low — C++20 substitution + asm/reinterpret carve-outs |
| 9 | basic source character set | self (MISRA 5) | Det-direct | low — byte-scan, ungated |
| 10 | documented ISO 10646-1 subset | self (MISRA 6) | Grey | none — no subset artifact authored |
| 11 | no trigraphs | self (MISRA 7) | Det-direct | low — removed in C++17, enforced-by-language |
| 12 | no digraphs | self (MISRA 7 ext) | Det-direct | low — valid in C++20, ungated |
| 13 | no multi-byte/wide literals | self (MISRA 8) | Det-direct | low — token-prefix scan, ungated |
| 14 | uppercase literal suffixes | self | Det-direct | none — hicpp + readability uppercase-suffix |
| 15 | run-time checking / defensive prog. | parent; P10-5/JPL 14-16 govern slices | Grey | **low** — return-value canary slice |
| 16 | DO-178B/SEAL-1 certified libs | self | Grey | none — procurement judgment, tier scoped-out |
| 17 | no errno | self (MISRA 119) | Det-direct | low — grep; zero use |
| 18 | no offsetof | self (MISRA 120) | Det-direct | **low — but USED, unlogged (see Findings)** |
| 19 | no <locale.h>/setlocale | self (MISRA 121) | Det-direct | low — grep; zero use |
| 20 | only #ifndef/#define/#endif/#include | JPL 20-23 govern restrict-half (pointer); allowlist self | **Split** | low — allowlist ungated, real #ifdef use |
| 21 | guards via #ifndef/#define/#endif | self | Det-by-reference | low — pragma-once grep; **shared_state.h violates** |
| 22 | <stdio.h> banned | self | Det-by-reference | none — pre-commit include-ban (R-5) |
| 23 | no atof/atoi/atol | self | Compliant-by-construction | none — zero use |
| 24 | no abort/exit/getenv/system | self | Compliant-by-construction | none — QP/C Q_onError path |
| 25 | no <time.h> | self | Compliant-by-construction | none — SDK time_us_32 |
| 26 | guard #ifndef/#endif use (repaired) | self (~P10-8/JPL 20-23) | Det-direct | none — guarded-body check |
| 27 | include guards, no other technique | self | Det-by-reference | low — **shared_state.h uses #pragma once** |
| 28 | #ifndef/#endif only as AV27 (repaired) | self | Det-direct | none — guarded-body AST/token check |
| 29 | no inline/function-like macros | JPL 20-21 govern (pointer-row) | Det-by-reference | low — macro-usage check off; PP-1 + DBG_* |
| 30 | no constant #define (repaired-confirmed) | JPL 20-21 govern (pointer-row) | Det-by-reference | low — PICO_BOARD/GIT_HASH build-identity carve-out |
| 31 | #define only for include guards | JSF 27 + JPL 20-21 (pointer-row) | Det-by-reference | low — PP-1 closed set |
| 32 | #include only header (*.h) files (repaired) | self (~P10-8/JPL 20-23) | Det-direct | none — include-target extension check |
| 33 | #include uses <filename.h> (repaired) | self; JPL 31 newer | Det-direct | none — angle/quote token check |
| 34 | header = logically related decls | self | Manual | none — cohesion judgment |
| 35 | every header has inclusion guard | self (co-governs w/ 27) | Det-direct | low — llvm-header-guard not enabled |
| 36 | minimize compilation dependencies | self | Manual | none — "when possible" judgment |
| 37 | headers include only what they need | self | Grey | none — IWYU available, unused |
| 38 | fwd-decl headers for ptr/ref deps | self | Grey | none — IWYU available, unused |
| 39 | no non-const var/fn defs in headers (repaired) | self (~AV 139) | Det-direct | none — **misc-definitions-in-headers ENABLED** |
| 40 | impl file includes what it uses | self | Grey | low — misc-include-cleaner SKIPPED |
| 41 | lines ≤120 chars | self | Det-direct | low — no .clang-format wired |
| 42 | one statement per line | **JPL 24 governs** (pointer-row) | Det-direct | low — no formatter gate |
| 43 | avoid tabs | self | Det-direct | low — no formatter gate |
| 44 | indent ≥2 spaces, consistent | self | **Split** | low — consistent-half converts on .clang-format |
| 45 | underscore word separation | self (vs project convention) | Det-direct | **critical** — gate enforces camelBack, cited as JSF |
| 46 | ≤64 significant id chars | self (MISRA 11) | Det-direct | none — CbC on C++20 toolchain |
| 47 | no leading-underscore ids | self | Det-direct | low — reserved-identifier check SKIPPED |
| 48 | no confusable ids | self | Det-direct | low ⟵ *misc-confusable-identifiers SKIPPED (verifier)* |
| 49 | uppercase acronyms | self | Manual | none — acronym recognition semantic |
| 50 | type names: first-upper rest-lower | self | **Split** | low — type-case unconfigured, CamelCase half-satisfies |
| 51 | fn/var names all lowercase | self (vs convention) | **Split** | **critical** — camelBack vars cited as JSF |
| 52 | constant/enumerator names lowercase | self (vs convention) | Det-by-reference | **critical** — k-prefix inverts JSF, cited as JSF |
| 53 | headers .h extension | self | Det-direct | none — CbC (all .h) |
| 53.1 | (see-also fragment) | n/a | Compliant-by-construction | none — not a rule |
| 54 | impl files .cpp extension | self | Det-direct | none — CbC (all .cpp, PP-1) |
| 55 | header name reflects entity | self | Manual | none — semantic appropriateness |
| 56 | impl name reflects entity | self | **Split** | none — ext+pairing mechanical, name-reflects Manual |
| 57 | class sections public→protected→private | self | Det-direct | none — no enabled check |
| 58 | multi-line param layout (>2 params) | self | Det-direct | low — no .clang-format |

### 4b. Rules 59–113 (layout, classes, inheritance, templates, functions, scope)

| # | rule | governs | bucket | enforcement |
|---|------|---------|--------|-------------|
| 59 | braces around control-flow bodies | self | Det-direct | none — braces-around-statements (SSL=0) |
| 60 | brace column alignment | self | Det-direct | low — no clang-format gate |
| 61 | only comments on brace line | self | Det-direct | low — no clang-format gate |
| 62 | `*`/`&` bound to type | self | Det-direct | low — no clang-format gate |
| 63 | no spaces around `.`/`->`/unary | self | Det-direct | low — no clang-format gate |
| 64 | interface complete & minimal | self | Manual | none — design judgment |
| 65 | struct for no-invariant entity | self | Manual | none — invariant judgment |
| 66 | class for invariant entity | self | Manual | none — invariant judgment |
| 67 | no public/protected data in classes | self | Det-direct | low — AST-decidable, no enabled check |
| 68 | disallow unneeded implicit members | self | Grey | low ⟵ *special-member-functions ENABLED (verifier)* |
| 69 | const member fns by default | self | Grey | low — const-correctness partial |
| 70 | friend only when needed | self | Manual | none — friend-justification judgment |
| 70.1 | no improper use before/after lifetime | self | Grey | low — analyzer subset (LL 35) |
| 71 | no external op before full init | self | Grey | low — two-phase init contract (LL 6) |
| 71.1 | no virtual calls in ctor/dtor | self | Det-direct | low — PureVirtualCall live, VirtualCall off |
| 72 | invariant in pre/postconditions | self | Manual | none — unstated invariant |
| 73 | no unnecessary default ctors | self | Grey | none — "unnecessary" judgment |
| 74 | mem-init-list not body assignment (repaired) | self | Det-direct | low — prefer-member-initializer not enabled |
| 75 | init-list in declaration order | self | Det-direct | low — `-Wreorder` in pedantic build (≈none) |
| 76 | declare copy+assign for ptr-owning (repaired verbatim) | self | Grey | low ⟵ *special-member ENABLED (verifier); verbatim false* |
| 77 | copy ctor copies invariant members | self | Manual | none — invariant judgment |
| 77.1 | no default-arg copy-ctor-signature | self | Det-direct | none — mechanical, ungated |
| 78 | virtual dtor if virtual fn | self | Det-direct | low — `-Wnon-virtual-dtor` heuristic |
| 79 | RAII — dtor releases resources | self | Manual | none — ownership semantic |
| 80 | default copy/assign if reasonable (repaired) | self | Grey | none — "reasonable" judgment |
| 81 | assignment handles self-assignment | self | Grey | none — copy-and-swap safety semantic |
| 82 | assignment returns `*this` | self | Det-direct | none — mechanical, ungated |
| 83 | assign all invariant members (repaired) | self (mirrors 77) | Grey | none — invariant-membership judgment |
| 84 | operators sparing & conventional | self | Manual | none — "sparingly"/"conventional" |
| 85 | opposite operators, one via other | self | **Split** | none — pairing by-ref, delegation Grey |
| 86 | concrete types = simple concepts | self | Manual | none — modeling judgment |
| 87 | hierarchies on abstract classes | self | Grey | none — abstractness fact + design why |
| 88 | MI shape (n iface/m priv/≤1 prot) | self | Grey | none — iface-vs-impl judgment |
| 88.1 | stateful virtual base declared in derived | self | Det-by-reference | none — inheritance-graph cross-ref |
| 89 | base not both virtual & non-virtual | self | Det-by-reference | none — whole-hierarchy aggregation |
| 90 | heavy interfaces minimal/general | self | Manual | none — usage-weighted judgment |
| 91 | public inheritance = is-a | self | Manual | none — is-a semantics |
| 92 | LSP override pre/postconditions | self | Manual | none — behavioral subtyping |
| 93 | has-a via membership/non-pub inherit | self | Manual | none — relationship classification |
| 94 | no redefining inherited non-virtual fn | self | Det-by-reference | none — base/derived cross-ref |
| 95 | no redefined inherited default arg | self | Grey | none — override default-arg compare |
| 96 | no polymorphic array treatment | self | Grey | none — stride-mismatch flow |
| 97 | Array class over raw arrays in interfaces | self | Grey | none — interface-boundary judgment |
| 97.1 | no ==/!= on ptr-to-virtual-member-fn | self | Det-direct | none — type-based, ungated |
| 98 | nonlocal names in a namespace | self | Det-direct | low — cheap deterministic, ungated |
| 99 | namespace nesting ≤2 | self | Det-direct | none — countable, ungated |
| 100 | using-decl vs using-directive by count | self | Grey | none — "~5"/readability judgment |
| 101 | templates reviewed (repaired) | self | Manual | none — mandated review activity |
| 102 | template tests cover instantiations (repaired) | self | Grey | none — coverage judgment |
| 103 | constrain template arguments | self | Grey | none — constraint-sufficiency judgment |
| 104 | specialization declared before use | self | Det-direct | none — compiler-enforced |
| 105 | minimize template instantiation-context dep | self | Grey | none — coupling judgment |
| 106 | pointer-type specializations where apt | self | Grey | none — appropriateness judgment |
| 107 | functions at file scope | self (MISRA 68) | Det-direct | none — compiler/CbC |
| 108 | no variadic (ellipsis) functions | self (MISRA 69) | Det-direct | none — syntactic, CbC |
| 109 | in-class fn body only if inlined | self | Grey | none — inlining-intent judgment |
| 110 | ≤7 function arguments | self | Det-direct | low — arg-count uncounted by gate |
| 111 | no return ptr/ref to local | self | Det-direct | none — `-Wreturn-stack-address` + analyzer |
| 112 | returns not obscuring ownership | self | Grey | none — ownership-clarity judgment |
| 113 | single function exit point | self (P10-1/JPL 11 *related, not subsuming*) | Det-direct | low — return-count uncounted; may be deviated |

### 4c. Rules 114–168 (returns, params, recursion, overloading, comments, declarations, expressions)

| # | rule | governs | bucket | enforcement |
|---|------|---------|--------|-------------|
| 114 | single return / exit via return | self | Grey | low — exit-discipline ungated |
| 115 | returned error info shall be tested | **P10-7/JPL 14 govern** (pointer-row, MISRA 86) | Grey | **critical** — CANARY, project fns uncovered |
| 116 | pass small concrete types by value | self | Manual | none — caller-mutation+size judgment |
| 117 | pass by ref when NULL impossible (parent) | self | Manual | none — verbatim false (colon stub) |
| 117.1 | pass `const T&` when not modified | self | Manual | none — const-correctness assists only |
| 117.2 | pass `T&` when may be modified | self | Manual | none — mutate-intent judgment |
| 118 | pass by ptr when NULL possible (parent) | self | Manual | none — verbatim false (colon stub) |
| 118.1 | pass `const T*` when not modified | self | Manual | none — assists only |
| 118.2 | pass `T*` when may be modified | self | Manual | none — mutate-intent judgment |
| 119 | no recursion | **P10-1/JPL 4 govern** (pointer-row, MISRA 70) | Det-by-reference | none — misc-no-recursion |
| 120 | overload families share semantics | self | Manual | none — semantic coherence |
| 121 | inline only 1-2 statement fns | self | Grey | none — candidacy advisory |
| 122 | inline trivial accessors/mutators | self | Manual | none — triviality judgment |
| 123 | minimize accessors/mutators | self | Manual | none — encapsulation quality |
| 124 | inline trivial forwarding fns | self | Manual | none — role/triviality |
| 125 | avoid unnecessary temporaries | self | Manual | none — perf checks not enabled |
| 126 | only C++-style (//) comments (repaired) | self | Det-direct | none — lexer token check |
| 127 | delete commented-out code | self | Grey | none — deadcode≠comments |
| 128 | no comments documenting external sources | self | Manual | none — content judgment |
| 129 | header comments = external behavior | self | Manual | none — abstraction judgment |
| 130 | every executable line purpose-commented | self | Manual | none — density≠purpose |
| 131 | no comment restating code | self | Manual | none — equivalence judgment |
| 132 | every decl/member commented | self | Grey | none — presence mechanical, redundancy not |
| 133 | every file has intro comment | self | Grey | none — presence mechanical, accuracy not |
| 134 | document fn assumptions/limitations | self | Manual | none — contract judgment |
| 135 | no inner-scope identifier hiding (MISRA 21) | self | Det-direct | **low** — `-Wshadow` NOT enabled |
| 136 | smallest feasible scope (MISRA 22) | **P10-6/JPL 13 govern** (pointer-row) | Grey | none — verbatim false; LL1 tension |
| 137 | file-scope static where possible (MISRA 23) | self | Det-direct | none — misc-use-internal-linkage |
| 138 | no internal+external linkage same TU (MISRA 24) | self | Det-direct | low — redundant-declaration partial |
| 139 | external objects in one file (MISRA 27, repaired) | self (~AV 39) | Det-by-reference | none — cross-TU symbol scan; verbatim false |
| 140 | no register specifier (MISRA 28) | self | Det-direct | none — removed in C++17, CbC |
| 141 | no type declared in its type-definition | self | Det-direct | none — AST shape, ungated |
| 142 | initialize before use (MISRA 30) | self | Det-direct | none — init-variables + analyzer; verbatim false |
| 143 | no var until meaningful init (repaired) | **P10-6/JPL 13 govern** (pointer-row) | Grey/Manual | none — "meaningful" semantic; verbatim false |
| 144 | brace structure in aggregate init (MISRA 31) | self | Det-direct | none — `-Wmissing-braces` |
| 145 | enum `=` mix prohibition (MISRA 32) | self | Det-direct | none — readability-enum-initial-value |
| 146 | IEEE-754 floating point (MISRA 15) | self | Compliant-by-construction | none — RP2350 FPU + GCC |
| 147 | no FP bit-representation use (MISRA 16) | self | Grey | none — reinterpret-cast SKIPPED |
| 148 | enum over int for choice sets | self | Manual | none — abstraction antecedent |
| 149 | no non-zero octal constants (MISRA 19) | self | Det-direct | low — leading-0 scan, ungated |
| 150 | uppercase hex constants | self | Det-direct | none — lexical, ungated |
| 151 | no magic numbers / symbolic constants | self | **Split** | low — ignore-list lookup + should-name judgment |
| 151.1 | no string-literal modification | self | Grey | none — flow-sensitive write-through |
| 152 | one variable per declaration line | self | Det-direct | none — isolate-declaration |
| 153 | no unions (MISRA 110) | self | Compliant-by-construction | none — grep; reinterpret_cast used instead |
| 154 | bit-fields explicitly unsigned/enum | self (MISRA 111/112) | Det-direct | none — AST type check, ungated |
| 155 | bit-fields not solely for space | self | Manual | none — purpose intent |
| 156 | all members named, accessed by name | self (MISRA 113) | Det-direct | none — CbC idiomatic C++ |
| 157 | no side effects in RHS of &&/\|\| | **JPL 19 governs** (pointer-row, MISRA 33) | Grey | low — callee-purity residual |
| 158 | parenthesize binary-op operands of &&/\|\| | **JPL 18 governs** (pointer-row, MISRA 34) | Det-direct | none — `-Wparentheses` partial |
| 159 | no overload of `&&`/`\|\|`/unary& | self | Det-direct | none — AST scan, CbC |
| 160 | assignment only as expression statement | self (MISRA 35) | Det-direct | low — assignment-in-if subset only |
| 162 | no mixed signed/unsigned arithmetic | self | Grey | low — narrowing+`-Wsign-compare` partial |
| 163 | no unsigned arithmetic | self | Grey | none — would over-report mandated uintN_t |
| 164 | shift RHS in [0,width-1] (MISRA 38) | self | Grey | none — constant case gated, runtime hard |
| 164.1 | right-shift LHS not negative | self | Grey | none — hicpp-signed-bitwise over-approx |
| 165 | no unary minus on unsigned (MISRA 39) | self | Det-direct | none — AST, ungated |
| 166 | no sizeof on side-effecting expr (MISRA 40) | self | **Split** ⟵ *was Det-direct (verifier)* | low — sizeof-expression + callee-purity Grey |
| 167 | integer-division determined/documented (MISRA 41) | self | Manual | none — documentation obligation |
| 168 | no comma operator (MISRA 42) | self | Det-direct | none — AST, ungated/CbC |

### 4d. Rules 169–221 (pointers, casting, control flow, layout, coverage)

| # | rule | governs | bucket | enforcement |
|---|------|---------|--------|-------------|
| 169 | avoid ptr-to-ptr when possible | self | Manual | low — avoidability judgment |
| 170 | ≤2 levels pointer indirection (MISRA 102) | **JPL 26 governs** (pointer-row) | Det-by-reference | none — AST depth count |
| 171 | pointer relational only same object (MISRA 103) | self | Grey | low — provenance partly analyzable |
| 173 | no stack-address escape (MISRA 106) | self | Grey | low — StackAddressEscape subset |
| 174 | no null deref (MISRA 107) | JPL 14/15 govern check-before-use (pointer-row) | Grey | low — NullDereference under/over-reports |
| 175 | use plain 0 not NULL | self (superseded by practice) | Grey | **critical** — modernize-use-nullptr enforces inverse |
| 176 | typedef function-pointer decls | **P10-9 governs** (pointer-row, LL 37) | Grey | none — moot under fn-ptr ban (FP-1) |
| 177 | avoid user-defined conversions | self | Grey | low — google-explicit-constructor subset |
| 178 | downcast only via virtual/visitor | self | Grey | low — static-cast-downcast shell |
| 179 | no virtual-base→derived conversion | self | Grey | low — virtual-base refinement ungated |
| 180 | no lossy implicit conversions (MISRA 43) | **JPL 30 governs** (pointer-row) | Grey | low — narrowing checks strong but value-sensitive |
| 181 | no redundant explicit casts (MISRA 44) | self | Grey | low — no useless-cast check enabled |
| 182 | no casting to/from pointers (MISRA 45) | self | Grey | **critical** — reinterpret_cast SKIPPED for HW |
| 183 | every measure to avoid casting | self | Manual | none — design-effort advisory |
| 184 | float→int only if required/HW | JPL 30 governs detection (pointer); justification self | Grey | low — detection gated, justification human |
| 185 | named C++ casts not C-style | self | Det-direct | none — pro-type-cstyle-cast |
| 186 | no unreachable code (MISRA 52) | self | Grey | low — SA subset, dead-branch OOS |
| 187 | non-null statements have side-effect (MISRA 53) | self | Grey | low — volatile/HW-read residual |
| 188 | labels only in switch (MISRA 55) | **P10-1/JPL 11 govern** (pointer-row) | Det-by-reference | none — avoid-goto + GT-1 |
| 189 | no goto (MISRA 56) | **P10-1/JPL 11 govern** (pointer-row) | Det-by-reference | none — avoid-goto + GT-1 |
| 190 | no continue | self | Det-direct | low — ungated; **28 real uses** |
| 191 | break only to terminate switch | self | **Split** | none — loop-break enumeration convertible |
| 192 | if/else-if has final else or comment | self | Manual | none — comment-adequacy disjunction |
| 193 | non-empty case terminated by break | self | Det-direct | none — `-Wimplicit-fallthrough` |
| 194 | switch has final default | self | Det-direct | none — switch-missing-default + `-Wswitch` |
| 195 | switch expr not Boolean | self | Det-direct | none — type fact, CbC |
| 196 | switch ≥2 cases + default | self | Det-direct | none — switch-missing-default |
| 197 | no float loop counters | self | Det-direct | none — cert-flp30-c |
| 198 | for-init: single param only | self | Grey | none — init purity semantic |
| 199 | for-increment: single param only | self | Grey | none — increment purity semantic |
| 200 | no null for-init/increment (use while) | self | Det-direct | none — empty-clause AST, ungated |
| 201 | loop counter not modified in body | self | Grey | none — aliasing defeats full coverage |
| 202 | no float exact ==/!= | self | Det-direct | **low** — `-Wfloat-equal` off; **real `!=0.0f` uses** |
| 203 | no over/underflow dependence | self (MISRA 51) | Grey | low — analyzer heuristic subset |
| 204 | side-effecting op in 7 contexts only | self | Grey | none — taxonomy ungated |
| 204.1 | value invariant under eval order (MISRA 46) | **JPL 18 governs** (pointer-row) | Grey | low — `-Wsequence-point` UB subset |
| 205 | volatile only for hardware | self | Grey | none — HW-justification judgment |
| 206 | no heap after init (repaired) | **P10-3/JPL 5 govern** (pointer-row) | Det-by-reference | none — no-malloc; AP-1/AP-2 resolved |
| 207 | avoid unencapsulated globals | self | Manual | none — encapsulation judgment; avoid-non-const-global SKIPPED |
| 208 | no C++ exceptions | self | Compliant-by-construction | none — exceptions disabled |
| 209 | fixed-width types | **JPL 17 governs** (co-cited pointer-row, MISRA 13) | Det-direct | none — google-runtime-int |
| 210 | no memory-representation assumptions (repaired) | self | Grey | none — reinterpret/byte-poke candidates only |
| 210.1 | no access-specifier member-order assumption (repaired) | self | Grey | none — assumption-prohibition judgment |
| 211 | no scalar-address assumptions | self | Manual | none — alignment-assumption judgment |
| 212 | no over/underflow-behavior dependence | self | Manual | low — narrowing+analyzer curated subset |
| 213 | no sub-arithmetic-precedence dependence | self | Grey | low — math-parens DISABLED (LL 26) |
| 214 | no cross-TU static-init-order assumption | self | Manual | none — SIOF, whole-program judgment |
| 215 | no pointer arithmetic (MISRA 101) | **P10-9/JPL 26-29 govern** (pointer-row) | Grey | low — pro-bounds-pointer-arithmetic SKIPPED |
| 216 | no premature optimization (Meyers 16) | self | Manual | none — intent/evidence judgment |
| 217 | prefer compile/link-time errors (Meyers 46) | self | Manual | none — design-alternative judgment |
| 218 | compiler warning levels per policy | **P10-10/JPL 2 govern** (pointer-row) | Det-by-reference | none — pedantic `-Werror` build + policy lookup |
| 219 | base interface tests applied to derived | self | Manual | none — LSP test-inheritance |
| 220 | structural coverage on flattened classes | self | Manual | none — coverage methodology (DO-178C) |
| 221 | structural coverage of polymorphic resolutions | self | Manual | none — per-virtual-target coverage |

**Detail — JSF Grey/Split/finding rules (the why):**

- **JSF 3 (Det-direct, low):** lizard computes true cyclomatic; the pre-commit fast gate is `readability-function-cognitive-complexity` (Threshold 25), an *admitted* non-faithful proxy (`.clang-tidy` itself states "Cognitive != cyclomatic … passing here does NOT guarantee JSF compliance"). Explicit self-disclaimer ⇒ honest-narrow, not over-claiming. Converges to none if lizard reliably runs.
- **JSF 6 (Det-by-ref, low):** project keeps central log + inline comments, but no tool cross-checks "deviation annotated in its own file" as 6 strictly requires. Converts to Det once a grep cross-references central log vs in-file annotations.
- **JSF 8 (Grey, low):** compiler pins `-std` and `-Wpedantic` flags extensions, but the project deliberately targets C++20 (not the cited 2002 baseline) and uses platform extensions (BASEPRI `__asm` — `hicpp-no-assembler` SKIPPED; `reinterpret_cast` for MPU/HW — `pro-type-reinterpret-cast` SKIPPED). Judging those accepted-by-necessity is human.
- **JSF 10 (Grey, none):** requires a project-authored "documented ISO 10646-1 subset" that does not exist; a human must define the subset (effectively ASCII) before it is checkable. **Conversion move** applies.
- **JSF 15 (Grey, low):** broad defensive-programming parent; specific slices governed by P10-5/JPL 14-16. Reproduces the **return-value canary**: `cert-err33-c` + `bugprone-unused-return-value` cover only the curated set, not `i2c_bus_*`/`flash_safe_execute`/`gps_*`/`rc_log`.
- **JSF 16 (Grey, none):** DO-178B/SEAL-1 library certification is procurement judgment; project's hobbyist/NAR-TRA tier (IEC 61508 HFT=0) scopes out the SEAL-1 antecedent. As-built inventory (Pico SDK, QP/C, lwGPS, ETL, MAVLink) is risk-accepted.
- **JSF 20 (Split, low):** restrict-half governed by JPL 20-23; the four-directive allowlist is JSF-stricter and ungated — `version.h` legitimately uses `#ifdef/#elif/#else` (job role-select) outside the allowlist. (Verifier: the R-26 "dead conditional in version.h" claim is FALSE — `ROCKETCHIP_JOB_RELAY` is compile-defined at `CMakeLists.txt:740`.)
- **JSF 27 (Det-by-ref, low):** include-guard discipline; **a real standing violation: `include/rocketchip/shared_state.h` uses `#pragma once`** (the one banned "other technique"); 32/33 headers use `#ifndef` guards. A grep decides it.
- **JSF 37/38 (Grey, none):** IWYU is the canonical tool, available but NOT in the gate stack. Tool exists, unused, no gate claims coverage ⇒ honest gap.
- **JSF 44/50 (Split, low):** indent-minimum & type-leading-uppercase mechanical; the "consistent within file" / "all-others-lowercase" halves convert/deviate. Type-name cases (`ClassCase`/`EnumCase`/…) are NOT configured while the gate is cited as "Rule 50-53."
- **JSF 68 (Grey, low):** *verifier:* `cppcoreguidelines-special-member-functions` IS enabled (+ `modernize-use-equals-delete`), so the Rule-of-X candidate set IS surfaced; "unneeded" stays human. tool_used=true, finding=low (was none).
- **JSF 69 (Grey, low):** `misc-const-correctness` enabled but checks variable const-ness, not member-function const-by-default nor the explicit-reason carve-out.
- **JSF 70.1/71 (Grey, low):** clang-analyzer catches a real lifetime/init subset (LL 35 stack-local QEvt use-after-free; LL 6 WS2812 begin()-before-use) but cannot model two-phase-init contracts or all cross-TU/runtime lifetimes.
- **JSF 76 (Grey, low):** *verifier:* batch verbatim was Appendix-A prose ⇒ **verbatim_ok=false**; real text = "A copy constructor and an assignment operator shall be declared for classes that contain pointers to data items or nontrivial destructors." `special-member-functions` ENABLED surfaces the asymmetry candidate; deep-copy-vs-delete stays human.
- **JSF 80/83 (Grey, none, repaired):** "reasonable semantics" (80) and "members that affect the class invariant" (83, mirrors 77) are non-representable; tools list candidates, human judges. 83 converts to Det-by-reference if an invariant-affecting-member inventory is authored.
- **JSF 85 (Split, none):** existence-pairing of opposite operators = Det-by-reference (and C++20 rewritten-candidates make `!=` from `==` compliant-by-construction); whether one is implemented *in terms of* the other = Grey (body inspection).
- **JSF 95/96/97 (Grey, none):** override-default-arg comparison / polymorphic-array stride-mismatch flow / interface-array-vs-internal judgment — all candidate-surfaceable, human-decided.
- **JSF 100/103/105/106/109/121/127/132/133 (Grey):** advisory/threshold rules with a syntactic shell (name counts, constraint presence, comment presence, statement counts, commented-code heuristic) and a human "why" (readability/sufficiency/intent/redundancy). None has a claiming gate ⇒ finding none (least-worrying).
- **JSF 101 (Manual, repaired):** "templates shall be reviewed" mandates a human review activity; no tool purchase.
- **JSF 102 (Grey, repaired):** instantiation list is enumerable (compiler), but whether a test "covers" each instantiation is human; converts on a per-instantiation test-coverage manifest.
- **JSF 113 (Det-direct, low):** *verifier correction to governs string:* single-exit is **NOT subsumed** by P10-1/JPL 11 (those ban goto/recursion; P10/NASA tolerate early returns) — JSF-only, P10-1 merely related. Return-count is countable but ungated and may be locally deviated by modern early-return style.
- **JSF 114 (Grey, low):** exit-discipline intent ungated; the mechanical fall-off-end-UB sub-property is caught by `-Wreturn-type`.
- **JSF 115 (Grey, critical) — CANARY pointer-row:** property = checking returned error info = P10-7/JPL 14 territory (they govern). `bugprone-unused-return-value` (no `CheckedFunctions`) + `cert-err33-c` pass the project's own dropped-return failures (LL 28/31/41) while reading as return-value coverage. *Note: per the JPL-14 audit-of-record downgrade, the over-claim hinge is whether a gate is presented as full coverage; this row is retained critical as the canary the contract mandates surface, with the same conversion (populate `CheckedFunctions`).*
- **JSF 135 (Det-direct, low):** shadowing is `-Wshadow`-decidable, but `-Wshadow` is **NOT in `-Wall`/`-Wextra`** and not separately enabled — fully decidable yet un-gated. Easy fix: add `-Wshadow`.
- **JSF 147 (Grey, none):** float↔int reinterpret/union/memcpy candidates exist, but `pro-type-reinterpret-cast` is SKIPPED (HW registers) so even the candidate-surfacer is off; telemetry/log float serialization is a legitimate-use area a human distinguishes.
- **JSF 151 (Split, low):** literal-vs-ignore-list = Det-by-reference (curated set: `0;1;2;3;4;8;16;32;64;100;255;256;1000` + floats + powers-of-2 + bitfield widths); whether a flagged literal *should* be named = Grey. Honest-narrow: a value in the curated set passes silently even when contextually magic.
- **JSF 157/204.1 (Grey, low):** JPL 19/18 govern (newer, broader). `bugprone-assignment-in-if-condition` / `-Wsequence-point` catch UB sub-cases; general callee-purity / cross-call shared-state order-dependence is human.
- **JSF 162/164/164.1/171/173/174/177-181/186/187/203 (Grey, low/none):** static-analysis or narrowing/cast checks give a candidate subset; the residual (value-range, provenance, aliasing, lifetime, ownership, downcast-mechanism, redundancy intent, dead-branch, volatile-HW-read, overflow-dependence) is human. None over-claims.
- **JSF 163 (Grey, none):** correctly UNenforced — a "no unsigned arithmetic" gate would **over-report** the project's MANDATED fixed-width `uintN_t`/HW-register/bitmask code (the over-report tool-untrustworthiness mode).
- **JSF 166 (Split, low) — verifier re-bucket:** syntactic side-effect-construct-in-`sizeof` majority = Det-direct (`bugprone-sizeof-expression`); the callee-purity residual = Grey (identical to 157, which forced the consistency fix from flat Det-direct).
- **JSF 175 (Grey, critical):** mandates plain `0`; project ENABLES `modernize-use-nullptr` which rewrites both `0` and `NULL` to `nullptr` — the **inverse**. Any audit citing AV 175 as "enforced/passed" over-claims. Converts to Det-by-reference once the supersession is written to the deviation log (no such row exists yet).
- **JSF 182 (Grey, critical):** bans casting to/from pointers; the RP2350 bare-metal target *structurally* requires it (MPU setup, HW registers, CFSR reads) and `pro-type-reinterpret-cast` is SKIPPED for exactly this reason. Standing deviation that must be logged, not silently passed. Converts to Det-by-reference once an accepted-pointer-cast list is authored (none exists).
- **JSF 190 (Det-direct, low):** `continue` ban is a pure grep, but NO gate flags it and NO deviation row covers it while **28 real `continue;` sites** exist across 12+ files (ao_telemetry.cpp:348 inside a real for-loop, etc.) — honest gap, de-facto-deviating without a closed exemption set.
- **JSF 202 (Det-direct, low):** float `==`/`!=` is `-Wfloat-equal`-decidable, but that flag is NOT in the build (only in vendored pico-sdk) and `cert-flp30-c` covers loop counters not equality — while **real `!= 0.0f` tests exist at `eskf_runner.cpp:292-295`**.
- **JSF 205 (Grey, none):** every `volatile` is greppable; true MMIO (NVIC ISPR, inline asm) compliant, but borderline cross-core/ISR-sync uses (gps ring head/tail, rc_log ring, `g_t2_*` flags, `g_phase_observable_pair`) need human judgment vs the "hardware interface" clause.
- **JSF 210/210.1 (Grey, repaired):** memory-representation / access-specifier member-order *assumptions* — reinterpret/byte-poke/offsetof candidates surfaceable, dependence-intent human. 210.1's batch text was Appendix-A commentary (verbatim flagged); normative rule is the access-specifier-ordering prohibition. 211 (scalar-address) parked Manual (no syntactic shell).
- **JSF 213 (Grey, low):** `readability-math-missing-parentheses` deliberately DISABLED (LL 26 / JSF 213 exemption — arithmetic precedence universally understood; ArduPilot AP_Math relies on `*` before `+`). The exempted-out residual (bitwise/ternary/logical mixing) is human-only; re-enabling on the non-arithmetic complement is the conversion target.

---

## 5. Rule-Equivalence Cross-Reference (self-contained — for standalone doc + CODING_STANDARDS amendment)

Only cross-standard OVERLAP clusters. Relation key: `==` equivalent, `(` narrower, `)` broader, `~` related-distinct. JSF-only and JPL-original rules with no cousin are omitted.

| property | P10 | JPL | JSF | governs | relation |
|----------|-----|-----|-----|---------|----------|
| control flow / goto | 1 | 11 | 188, 189 | P10-1 / JPL 11 | P10-1 == JPL 11 (goto half); JSF 188/189 ~ (labels/goto specifics) |
| recursion | 1 | 4 | 119 | P10-1 / JPL 4 | JPL 4 == P10-1 (recursion half, cites MISRA 16.2); JSF 119 == (MISRA 70) |
| loop bounds | 2 | 3 | — | P10-2 | JPL 3 == P10-2 (cites it); JPL adds `@non-terminating@` |
| no dynamic allocation | 3 | 5 | 206 | P10-3 / JPL 5 | JPL 5 == P10-3 (cites MISRA 20.4); JSF 206 == (heap-after-init scope) |
| function length / size | 4 | 25 | 1 | JPL 25 | JPL 25 == P10-4 (map); JSF 1 ( (200 L-SLOC specific metric) |
| assertions | 5 | 16 | (15) | P10-5 | JPL 16 ~ P10-5 (>10-line density variant); JSF 15 ) defensive-prog parent |
| smallest scope | 6 | 13 | 136, 143 | P10-6 / JPL 13 | JPL 13 == P10-6 (smallest half) + ) (shadow half); JSF 136 (, 143 ~ |
| return values | 7 | 14 | 114, 115 | P10-7 / JPL 14 | JPL 14 == P10-7 return-half (cites it); JSF 115 ( (error-returns only, MISRA 86); JSF 114 ~ |
| parameter validation | 7 | 15 | — | JPL 15 | JPL 15 == P10-7 param-half |
| preprocessor | 8 | 20–23 | 26–33 | JPL 20-23 | JPL 20-23 ) P10-8; JSF 26-33 ( / ~ (allowlist/guard/include-form specifics) |
| pointer indirection depth | 9 | 26, 27 | 170, 215 | JPL 26 / 27 | JPL 26 == JSF 170 (≤2 levels, MISRA 102); JPL 27 ~ (per-stmt deref); P10-9 ) |
| function pointers | 9 | 29 | 176 | **P10-9** | P10-9 ) JPL 29 ) JSF 176 (P10-9 most-restrictive: outright ban; LL 37) |
| dereference hiding | 9 | 28 | — | JPL 28 | JPL 28 ~ P10-9 family |
| type conversion | — | 30 | 180, 184 | JPL 30 | JPL 30 ) JSF 180 (lossy-implicit, MISRA 43) & ~ JSF 184 (float→int) |
| fixed-width types | — | 17 | 209 | JPL 17 | JPL 17 ) JSF 209 (co-cited; project checklist names 209) |
| warnings / static analysis | 10 | 2 | 218 | P10-10 | P10-10 == JPL 2 (MISRA 21.1); JSF 218 ~ (warning-level policy) |
| order of evaluation | — | 18 | 158, 204.1, 213 | JPL 18 | JPL 18 ) JSF 158/204.1 (MISRA 34/46); JSF 213 ~ (sub-arithmetic precedence) |
| boolean side effects | — | 19 | 157 | JPL 19 | JPL 19 ) JSF 157 (RHS-of-short-circuit only, MISRA 33) |
| one statement / decl per line | — | 24 | 42, 152 | JPL 24 | JPL 24 == JSF 42 (one stmt) + JSF 152 ( (one var/decl) |
| enum init | — | 12 | 145 | JPL 12 / JSF 145 | JPL 12 ~ JSF 145 (distinct: deliberate-values vs mixed-`=` anti-pattern, MISRA 32) |
| external object single-decl | — | 31 | 39, 139 | JSF 139 | JSF 139 ~ JSF 39 (header-content) & ~ JPL 31 (#include form, distinct) |

---

## 6. DEFERRED — JPL LOC-5/6 (73 + 16 MISRA-absorption rules)

Per existing project policy (`CODING_STANDARDS.md`, 2026-05-13), **JPL LOC-5 (73 MISRA-shall) + LOC-6 (16 MISRA-should) are DEFERRED-WITH-RATIONALE — parked, NOT silently omitted.** Rationale of record:
1. **Paywalled MISRA text** — the normative statements are not freely redistributable; verbatim-source triage (the audited method's contract) cannot be performed without licensed text.
2. **~90% JSF overlap** — the MISRA rules these absorb are largely the same properties already triaged under the JSF AV catalog above (e.g. MISRA 5/6/7/8 → JSF 9-13; MISRA 19/27/28/30-32 → JSF 149/139/140/142/144/145; MISRA 33/34/38-46/51/52/53 → JSF 157/158/164-168/186/187/203/204.1; MISRA 102/103/106/107/110-113/119-121 → JSF 170/171/173/174/153/154/156/17-19). Re-triaging them would mostly reproduce JSF rows.
3. **Hobbyist / NAR-TRA tier** — IEC 61508 HFT=0, not airworthiness-certified; full MISRA LOC-5/6 conformance is out of the project's assurance scope.

These 89 rules retain their provenance pointers (the JSF/JPL cousins above cite the absorbed MISRA numbers) and convert from DEFERRED to triageable only if licensed MISRA text is obtained. They are explicitly in-scope-but-parked, not N/A.

---

## 7. Findings

### 7.1 Over-claiming (critical)
- **P10-5** assertion density + non-vacuity: NO tool; if the gate is presented as covering P10-5 it over-claims the two core properties.
- **P10-7 (THE CANARY) + JSF 115**: `bugprone-unused-return-value` (no project `CheckedFunctions`) + `cert-err33-c` (C-stdlib only) leave `i2c_bus_*`, `flash_safe_execute`, `gps_*`, `rc_log` **uncovered** ⇒ the **LL 28/31/41 dropped-bus-return false-negatives** pass silently while reading as return-value coverage. **Conversion:** populate `.clang-tidy bugprone-unused-return-value.CheckedFunctions` with the project API ⇒ Det-by-reference.
- **JSF 45 / 51 / 52** (naming): `readability-identifier-naming` enforces camelBack/k-CamelCase/lower_case — which *contradict* JSF underscore-separation (45) / all-lowercase variables (51) / lowercase constants (52). `CODING_STANDARDS.md` cites the gate as "JSF AV Rule 50-53" compliance while it encodes a non-JSF scheme. The project convention legitimately supersedes JSF here; the *over-claim* is the doc text asserting JSF conformance. **Fix:** record the supersession explicitly.
- **JSF 175** (`use plain 0`): `modernize-use-nullptr` enforces the literal inverse (C++20). Honest disposition = deliberately-not-followed; an audit citing AV 175 as enforced over-claims.
- **JSF 182** (no casting to/from pointers): `pro-type-reinterpret-cast` SKIPPED; HW access structurally requires pointer casts. Standing deviation must be logged.

### 7.2 Honest-narrow gaps (low)
P10-2, P10-6, P10-8, P10-9, P10-10; JPL 1, 2, 3, 10, 12, 13, **14 (downgraded from critical — audit-of-record)**, 16, 18, 20, 21, 22, 23, 25, 30; JSF 3, 6, 8, 9, 11, 12, 13, 15, 17, 18, 19, 35, 40, 41, 42, 43, 44, 47, **48 (verifier: `misc-confusable-identifiers` SKIPPED, was none)**, 50, 58, 60-63, 67, **68 (verifier: special-member ENABLED)**, 69, 70.1, 71, 71.1, 74, **76**, 78, 110, 113, 114, **135 (`-Wshadow` not enabled)**, 138, 149, 151, 157, 160, 162, **166 (verifier re-bucket Split)**, 171, 173, 174, 177-182(low subset), 184, 186, 187, **190 (28 unflagged `continue`)**, **202 (real `!=0.0f`)**, 203, 204.1, 212, 213, 215.

### 7.3 Live unlogged violation (factual)
- **JSF 18 (no offsetof):** `offsetof` IS USED at `src/calibration/calibration_data.cpp:106,119` (CRC offset computation) with **no deviation row**. Bucket Det-direct/low stands, but this is a *present* finding, not just a gate gap — either log a deviation (legitimate CRC-offset idiom) or remediate.
- **JSF 27:** `include/rocketchip/shared_state.h` uses `#pragma once` — a present violation of the `#ifndef`-only guard discipline (32/33 headers compliant).

### 7.4 Return-value narrow-coverage finding (mandated surface)
The **JSF 114/115 + JPL 14/15** cluster is the audited canary. `bugprone-unused-return-value` has **no `CheckedFunctions` key** anywhere in `.clang-tidy CheckOptions`; `cert-err33-c`/`cert-err34-c` cover only their built-in C-stdlib lists. Therefore project IO/bus/log functions are not return-checked, reproducing the LL 28/31/41 dropped-bus-return false-negative class. JPL 14 carries this at **low** (no gate presents full coverage; project calibration rates low-to-medium); JSF 115 carries it as the **critical canary** per contract; JPL 15 / P10-7 Property B (parameter validation) is a separate honest gap (no tool). All converge to Det-by-reference once `CheckedFunctions` is populated.

### 7.5 Catalog-count citation errors
- **JPL LOC-1..4 = 31 rules, NOT the 102 the project docs claim.** The "102" conflates the full JPL D-60411 catalog (incl. the deferred LOC-5/6 MISRA-absorption rules) with the LOC-1..4 subset actually triaged here. Correct counts: LOC-1..4 = 31; LOC-5 = 73; LOC-6 = 16.
- **JSF "221" is the MAX rule-number, not a count.** There are **233 distinct rule IDs** (219 base + 14 sub-rules: 53.1, 70.1, 71.1, 77.1, 88.1, 97.1, 117.1/.2, 118.1/.2, 151.1, 164.1, 204.1, 210.1). Numbering **gaps exist at 161 and 172** (no rule with those numbers). Any "221 JSF rules" claim is off.

### 7.6 Repair status of the 14 truncated JSF rules
All 14 located and repaired with real normative statements — **`repaired_ok=true` for all 14**; none remains suspect:
- **Det-direct:** 28, 32, 33, 39, 74, 126.
- **Det-by-reference:** 139 (cross-TU symbol scan), 206 (init-phase boundary + new/delete; **P10-3/JPL 5 govern**, 206 demoted to pointer-row).
- **Grey:** 80, 83, 102, 210, 210.1.
- **Manual:** 101, 143.
Additionally, the verifier flagged **JSF 76's batch verbatim as Appendix-A prose (verbatim_ok=false)** and **JSF 210.1's batch text as Appendix-A commentary (real normative text substituted)** — both content-repaired here; buckets unchanged.