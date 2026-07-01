# L2-P5 Manual Code-Review Guide

**What this is:** the human-followable bridge for the L2-P5 standards walk — the eyes-on review the tools and gates can't do. You read it while walking the firmware file-by-file, judging the *semantic* rules a human has to eyeball, where a green build, a passing test suite, and a clean linter are exactly the conditions under which improper code slips through. It connects the depth (verified criteria) to the route (the file list). Companions:

- **ROUTE — which files, in what order:** `docs/audits/L2P5_WALK_ITINERARY.md`
- **FULL sourced depth — every criterion + its primary source + the AI-tendency it catches:** `docs/audits/L2P5_RP_SOURCES_2026-06-25.md`
- **Rule text + why each rule lands where it does:** `docs/audits/RULE_VERIFIABILITY_TRIAGE.md`
- **Work plan / gates / mechanical checks:** `docs/plans/L2P5_WALK_PLAN.md`

**Owner:** Nathan (the walk + all dispositions). **Status vocabulary:** `PASS` / `FAIL` / `PARTIAL` / `NOT CHECKED` / `N/A`.

---

## How to use this guide

1. **Take the next file from the itinerary** (`L2P5_WALK_ITINERARY.md`). Order is criticality + execution-path first, so a partial walk still covers what matters most. Walk a `.cpp` together with its `.h` — read each file whole.
2. **Run The spine on each function** — the gestalt human-eye review: judge the whole function/file, not the line. Can you name it without "and"? Can you eyeball-verify every path? Is the same idea expressed once? Is intent readable from the code's shape rather than its comments?
3. **Apply the per-file lenses** the Class index below names for that file (the itinerary tags each file with its relevant classes).
4. **Record a verdict in the itinerary** (tick the box, one-line note), and put any `FAIL`/`PARTIAL` in the relevant class's findings table below. **Completeness: record every file, `PASS` included** — full coverage is the deliverable, not just the defect list.
5. **Disposition each `FAIL`:** a fix → a new `R-NN` row in `docs/PROBLEM_REPORTS.md`; an accept → migrates to `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` with sign-off. A `PASS` needs no action; a `PARTIAL` notes the cheap compliant move.

---

## Class index — the lenses actually walked

| Lens | When you are walking |
|------|----------------------|
| **The spine** (gestalt / function-shape: does-one-thing, duplication, altitude, AI-code blind spots, embedded applicability) | **Every function, every file** |
| **Comments & documentation quality** | every file (comment-bearing) — heaviest on drivers, fusion, flight_director |
| **Assertions** | functions doing real work — safety/, fusion/, flight_director/, math/ |
| **Declaration scope & object lifetime** | every file; the canonical sites are `active_objects/ao_led_engine` (stack-local event), AOs, and large file-scope statics |
| **Class & interface design** | type definitions — `include/rocketchip/` public headers, QP/C AOs, driver classes |
| **Templates** | the few template sites — `calibration/lm_solver`, `math/`, header utilities |
| **volatile / control-flow discipline** | `volatile`-bearing code — `fusion/eskf_runner`, `core1/`, `logging/ring_buffer`, AOs, `include/rocketchip/sensor_seqlock` |
| **Concurrency & shared-data ownership** | every cross-core shared object — `active_objects/`, `core1/sensor_core1`, `safety/core1_i2c_pause`, `logging/ring_buffer`, `main.cpp`, `shared_state.cpp`, `include/rocketchip/sensor_seqlock` · `sensor_snapshot` |

---

## The spine — read the whole function, not the line

This is the core of the walk. Everything else in the guide is a rule-class lens you point at a clause; the spine is the lens you keep on for **every function you read**. The job it does that no gate and no line-by-line pass can: catch code that **compiles, lints clean, and passes its tests, yet is improper** — wrong on a path no test exercises, structured so a human can't hold it, or confidently commented into something it doesn't actually do. AI-generated code is fluent and plausible by construction, so this is exactly the surface it slips through. You read the function as a whole, ask what it is *for*, and judge whether its shape, its error paths, and its claims survive that reading.

Three blocks. **A** is the timeless function-shape eye (does-one-thing, altitude, nesting, scope, comments, duplication, whole-function gestalt). **B** is the distrust layer — the durable error *types* documented in AI-generated code, the confidence signals to specifically refuse. **C** is the embedded recast — what to add because this is bare-metal dual-core firmware, and what web-centric findings to drop. Each entry is `ID: title` → **good** → **finding**, with an agent-tendency note where one is recorded. Every criterion rests on a primary outside source (C++ Core Guidelines, Fowler's refactoring catalog, Holzmann's Power of Ten, NIST, OWASP, SEI CERT) — preserve the IDs and wording exactly.

> **On the numbers below.** Where block B cites a rate, it is stamped `[model, year]` — read it as an *illustration of the type*, never as the rate for whatever you are reviewing. The durable, citable thing is the **error TYPE**; the frequencies are volatile across model generations. Lead with the type.

---

### A. Function shape & altitude

Read the whole function before you judge any line of it. The questions here are all whole-function questions — they are invisible to a per-line pass because no single line is wrong.

**The name test (does-one-thing).** Before anything else, try to name the function in one verb phrase with **no "and."** If the honest name needs "and" (`read_and_print`, `init_and_publish`), or collapses to a vague `handle()` / `process()` / `run()`, it is doing more than one thing — even when every line is correct and the size gate is green.

- **CppCoreGuidelines F.1: "Package" meaningful operations as carefully named functions**
  - good: Every well-specified action is factored out and named; non-trivial repeated logic — including a lambda used in more than one place — is named, not copy-pasted inline. (CCG's own *"Flag identical and very similar lambdas used in different places"* is a clang-tidy-style cue, not a hand-walk; exact/near-exact clones are caught mechanically — see ES.3. The manual catch is the *nameable-action-left-inline* judgment, which no tool makes.)
  - finding: An identifiable, nameable action is left inline and unnamed, or the same several-line idiom (a CRC step, a register-poke sequence, a queue-publish dance) appears twice.
  - agent-tendency: LLM code writes the same multi-line idiom inline at each call site instead of extracting it once, and copy-pastes non-trivial lambdas across call sites — each instance locally correct, the duplication drifting out of sync later.

- **CppCoreGuidelines F.2: A function should perform a single logical operation**
  - good: Walking the whole body, you can state what it does in one short "and"-free verb phrase, and that phrase covers every line; inputs arrive as parameters and the result leaves as a return value, so the operation is reusable at a second call site.
  - finding: Two or more logical operations sitting next to each other (reads input AND formats AND writes errors AND mutates state). The size gate can be **green** here — the function is short yet still does two things. That is precisely the ungated "one logical operation" judgment.
  - agent-tendency: LLM code is generated in prompt order, so one function inlines setup + input read + core computation + side effects + cleanup as one straight-line body. It compiles, passes tests built against that same shape, trips no linter, and forces a name like `read_and_print`.

- **CppCoreGuidelines F.3: Keep functions short and simple**
  - good: Fits on one screen with few logical paths; complex sub-steps are pulled out into smaller, well-named functions. (The line/statement/nesting and cyclomatic counts are **gated** — `readability-function-size` at LineThreshold 60 + `readability-function-cognitive-complexity` at 25 — so don't hand-count; the manual half is the gestalt judgment below, not the metric.)
  - finding: Control flow deep or branchy enough that you cannot convince yourself every path is handled — CCG's own challenge: *"How would you know if all possible alternatives have been correctly handled?"* The length may be concealing an extractable operation, not just verbosity.
  - agent-tendency: Agents avoid the raw line-count gate by writing **dense rather than long** — nested if/else-if and switch ladders over flag parameters, packed under the size limit, so the size linter is green while several alternatives go silently unhandled.

**One level of abstraction (altitude).** A body should read at **one altitude** — a sequence of peer-level named steps, with the mechanics pushed down into callees. This is the Single Level of Abstraction / stepdown idea (a named principle; cite by name). Its citable enablers are below.

- **CppCoreGuidelines P.1: Express ideas directly in code**
  - good: Each layer expresses its idea with a construct at its own level — a named algorithm says "find/sort/transform," a typed `Point` says "coordinate." The layout itself, not an adjacent comment, carries the meaning.
  - finding: Intent encoded indirectly — a hand-rolled index loop where a named algorithm was meant, an inline bit-loop for an operation that has a name; an open-coded fragment drags the surrounding body down to its altitude.
  - agent-tendency: A line-by-line reviewer confirms each statement is valid and approves. "Could be expressed more directly at this altitude" is a design judgment, not a line defect — the agent has no trigger to raise it, so correct-but-low-altitude code is exactly what it waves through.

- **CppCoreGuidelines P.3: Express intent**
  - good: The top of the function names the operation; iteration mechanism, loop-variable scope, `const`-ness, and argument meaning are visible (named types where bare ints would be ambiguous), so a reader can judge correctness from shape and names alone.
  - finding: The "how" is exposed but the "what" is missing — a `while (i < v.size())` exposing the index mechanism, a `draw_line(int,int,int,int)` where you can't tell `(x1,y1,x2,y2)` from `(x,y,h,w)`. Whether the code is correct becomes undecidable from the code alone.

- **CppCoreGuidelines F.2 (altitude reading):** the body should not juxtapose a high-level call with raw bit-twiddling — `dispatch(evt); reg |= (1u << pin); recompute_crc();` performs one operation but at jarringly mixed altitudes and reads as two functions fused. Line-by-line review validates the high-level call and the low-level poke independently, finds both correct, and never notices the abstraction-level seam.

- **Fowler — Extract Function** (the mechanical enabler): an inline block that clearly does one nameable sub-task, left embedded beside higher-level calls, is the raw material that should have been extracted so the parent reads top-down. Extraction changes no behavior, so an agent has no correctness trigger to propose it — inlined logic that works is accepted as complete.

**Guard clauses & shallow nesting.**

- **CppCoreGuidelines F.56: Avoid unnecessary condition nesting**
  - good: Guard clauses handle exceptional cases up front and return early, putting the essential code at the outermost scope; the guideline states shallow nesting *"makes the intent clearer."* (Early return / guard-clause style is sound here — Holzmann's Power of Ten explicitly does **not** require single-exit, calling an early error return "the simpler solution.")
  - finding: The real work buried inside nested ifs (or behind a redundant `else`) so the body is *"simply a conditional statement enclosing a block."* Intent is recoverable only by tracing indentation inward.
  - agent-tendency: Nesting is behavior-preserving, so the agent never flags it — the code produces identical results — and it doesn't apply the guideline's own enforcement (flag a redundant else; flag a body that is just a conditional wrapping a block), so deep arrow-shaped code passes review.

**Declare near use, keep scopes tight.**

- **CppCoreGuidelines ES.5: Keep scopes small** — variables, temporaries, and resources live in the smallest enclosing block, so the function's shape is a series of tight scopes whose boundaries delimit each phase. finding: names held at full-function scope when needed only inside one branch (common after incremental edits) — correct, but flattens the visual structure and obscures resource lifetimes on no-heap code.
- **CppCoreGuidelines ES.6: Declare names in for-statement initializers and conditions to limit scope** — loop indices and condition-only names declared inside the for/if-init so they don't leak. finding: a loop counter declared above the loop and left dangling after it — runs fine, but the lingering name can be silently reused by a later block.
- **CppCoreGuidelines ES.21: Don't introduce a variable (or constant) before you need to use it** — each declaration sits immediately before first use. (Verbatim enforcement: *"Flag declarations that are distant from their first use."*) finding: a C-style block of declarations hoisted to the top, first touched many lines later; the reader must scan forward and hold them all in working memory.
- **CppCoreGuidelines ES.22: Don't declare a variable until you have a value to initialize it with** — declared at the point it gets its real initial value, no default-then-assign-later seams. finding: an uninitialized/zero-init variable parked early (`string s;`) then conditionally assigned far below — the *declaration distance* smears one logical step across the whole function. (The used-before-read / read-of-indeterminate-value half is gate-covered — `cppcoreguidelines-init-variables` + the `clang-analyzer-core.*`/`clang-analyzer-cplusplus.*` family answer "definitely assigned before read"; the manual half here is only the working-memory cost of the distance, not the UB.)

For all four: declaration distance and scope width are invisible to a correctness pass — the agent sees a declaration and a later valid use and confirms; it doesn't model reader working-memory cost, so front-loaded declaration dumps and over-wide scopes sail through.

**Say *why*, not *what* (comments).**

- **CppCoreGuidelines NL.1: Don't say in comments what can be clearly stated in code**
  - good: Where a comment would restate mechanics, the code is written — or the block extracted and named — so names and structure say it. Reason (verbatim): *"Compilers do not read comments."*
  - finding: A comment narrating what the next line literally does, or paraphrasing a block of raw detail that should have been extracted. A load-bearing mechanics-comment is evidence the structure dropped altitude.
  - agent-tendency: The agent treats explanatory comments as a positive signal ("well documented") and may add more; it doesn't recognize a mechanics-restating comment as a signal to Extract Function, so it preserves or multiplies the crutch that hides weak structure.

- **CppCoreGuidelines NL.2: State intent in comments**
  - good: Comments reserved for what code cannot say — the intended contract — because *"code says what is done, not what is supposed to be done."* The comment states the supposed-to, giving a reader something to check the body against.
  - finding: The only comments present restate mechanics (an NL.1 violation) while the intended behaviour/invariant is never stated. The guideline's warning bites: *"If the comment and the code disagree, both are likely to be wrong"* — and here there's no intent comment at all.

**Duplication.**

- **CppCoreGuidelines ES.3: Don't repeat yourself, avoid redundant code**
  - good: The same shape of code is hoisted once or replaced with a standard algorithm; each piece of knowledge lives in exactly one place.
  - finding: The same computation, validation pattern, or magic-constant sequence expressed as the same *knowledge* in two or more places — the semantic duplication only a whole-file read sees and a line-by-line pass misses. (Fowler's catalog names this the **Duplicated Code** smell; cite by name.) Exact / near-identical branch bodies are gate-caught by `bugprone-branch-clone`, so the walk targets the conceptual duplicates the tool can't see, not literal clones.
  - agent-tendency: Each turn/edit is generated semi-independently and the model favors locally-complete code, so it re-emits a block it already wrote elsewhere — duplicates individually correct and tested, until one requirement change must be made in N places.

**Encapsulation theater.**

- **CppCoreGuidelines C.131: Avoid trivial getters and setters**
  - good: Getters/setters add semantic value (maintain an invariant, convert internal→interface type) or they don't exist; where there is nothing to encapsulate, the type is a struct of public data. (Companion: **C.2** — use `class` if the type has an invariant, `struct` if members vary independently.)
  - finding: A class wrapping plain data in mechanical `get_x()/set_x()` pairs with no behavioral members and no invariant. (CCG's *"Flag multiple get and set member functions that simply access a member without additional semantics"* reads as a tool-style cue; the manual judgment is whether a real invariant sits behind the accessors.) Ceremony without semantic value — Fowler's **Data Class** smell.
  - agent-tendency: LLMs default to "proper OO" boilerplate — private fields plus full getter/setter pairs — even for passive value aggregates, producing encapsulation theater with no invariant behind it.

**The one-page gestalt.** Step back and look at the function as a single unit.

- **Power of Ten, Rule 4 (Holzmann): no function longer than a single sheet of paper (≈60 lines)**
  - good: Each function *"is a logical unit in the code that is understandable and verifiable as a unit"* (verbatim rationale) — it fits on one page so a reviewer can hold and verify the whole unit at once. (The ≈60-line count is gated by `readability-function-size`; the judgment here is whether the unit is *holdable and verifiable as a whole*, beyond the line count.)
  - finding: A function spanning *"multiple screens on a computer display or multiple pages when printed,"* which the rule calls *"often a sign of poorly structured code."* A pure whole-function judgment — length and structural integrity as a unit, independent of any line being wrong.
  - agent-tendency: Agents grow a function past the one-page bound by repeatedly adding in-place handling for each new case; the no-heap/no-recursion constraints push logic into straight-line code, so generated functions sprawl and become un-verifiable-by-eye while building, running, and testing clean.

> A code smell is *"a surface indication that usually corresponds to a deeper problem in the system"* (Fowler) — and *"by definition something that's quick to spot."* Treat comment-segmented length, duplication, and trivial accessors as surface signals to **investigate**, not automatic defects. The whole point of block A is that these are quick to spot on a holistic read and invisible line-by-line. Smells don't always indicate a problem, but they always warrant the look.

---

### B. Distrust the AI confidence signals — durable error TYPES

Block A asks "is this well-shaped?" Block B asks "is the confidence this code projects *earned*?" The binding premise: AI-generated code routinely **compiles, looks plausible, and passes the limited tests/linters applied to it, yet is functionally wrong, non-self-contained, insecure, or fabricated** — and human reviewers systematically *under*-scrutinize it. The signals you must specifically refuse are: a green test run, a confident comment, a heavily-documented surface, a defensive-looking error branch, and sheer volume. Each is below as a durable type.

**Passes-tests-yet-wrong (the headline).** This is the single most common failure type and the reason the spine exists. A green test/lint run is **necessary, not sufficient.** The canonical demonstration: expanding a benchmark's sparse bundled tests ~80× cut measured correctness by **19.3–28.9%** and even mis-ranked models `[EvalPlus / HumanEval+, NeurIPS 2023]`; under differential testing, roughly **30%** of "passing" agentic patches diverge from the oracle `[SWE-bench family, 2025]`. The type held across model generations even as rates moved.
- finding: Code that passes the given tests but fails on unexercised inputs — corner cases, null/edge inputs, unstated ordering or uniqueness assumptions the body silently breaks. **Do this:** exercise corner/edge/null/unexercised inputs by hand; probe unstated ordering/uniqueness/boundary assumptions; verify algorithmic efficiency, not just output (LLM output is often correct-but-non-optimal).

**Spec-noncompliance.** finding: The code silently drops an explicit *semantic* requirement — a type or boundary check, an ordering direction, a required step — while looking complete and passing normal-usage tests. Latent bugs labeled "correct" then snow-ball on later edits `[Copilot / JSS 2023]`. **Do this:** re-read the spec and confirm *every* requirement (types, boundaries, ordering, structure); test beyond "normal usage." (Project *token* bans — `goto`, heap-after-init, recursion, C-style casts, `<stdio.h>`, variadics — are greppable/gated, not this lens; don't re-hunt them by eye.)

**Confabulation.** This is an institutionally-named generative-AI risk: **NIST AI 600-1 (Generative AI Profile, 2024)** defines *confabulation* as confidently-presented false content accompanied by **fabricated justifying logic or citations**. finding: A confident comment, docstring, or rationale that the body does not actually implement — or a cited "fact"/reference that doesn't exist. The fabricated justification is what disarms reviewers. **Do this:** treat confident justifications and citations as potentially confabulated; if deleting the comments would leave a reader unable to reconstruct what the code is *supposed* to do, the intent was living in prose, not code (a P.1 failure too). On this codebase, a comment paraphrasing a datasheet/standard is doubly suspect — verify against the source or replace with a pointer.

**Non-self-contained / hallucinated symbols.** finding: Undefined helpers, undefined variables, or references to APIs/packages/registers that don't exist — code that *looks* complete but isn't. Package/API hallucination is a *reproducible* type: ~**19.7%** of generated package references were non-existent and 43% recurred across re-runs `[USENIX Security 2025]`, compressing to ~**4.62–6.10%** on the 2026 frontier cohort but with names invented *identically* across models — *the range shrinks, the threat remains*. **Do this:** confirm every symbol/helper/variable is actually defined; cross-check unfamiliar or rare API calls against current official docs (hallucination concentrates in low-frequency APIs).

**Unchecked returns / happy-path-only error handling.**
- **Power of Ten, Rule 7 (Holzmann): the return value of non-void functions must be checked by each calling function; parameter validity must be checked inside each function.** *(The "explicitly cast to `(void)`" escape is JPL-C Rule 14's phrasing, not P10's.)* On this codebase the **must-be-checked half is gated** — project APIs carry `[[nodiscard]]` under `-Werror`, with the one vendored exception in `.clang-tidy`'s `CheckedFunctions` — so the manual residual is the shape the gate can't judge.
  - finding: The code calls an API and uses the result as if it always succeeds — or, worse, assumes an *optimistic return shape* (treats a status return as if it were the data), a wrong-API-contract the `[[nodiscard]]` gate does not catch. The other manual residual: confirm a deliberately dropped-and-`(void)`-cast return was a *justified* ignore, with a reason. Memory-safety and **unchecked-return** CWEs (CWE-252/253) are the dominant types documented in LLM-generated C/C++.
  - agent-tendency: LLMs write to the success path and silently drop error returns, and combined with hallucinated API contracts they assume a return shape the real API does not have. **Do this:** walk *every* exit path, not the happy path the tests exercise.
- **Related:** code that *adds* a defensive-looking error branch whose cleanup is incomplete on the path no test covers — acquire-then-return-early without the release (ties to the **Declaration scope & object lifetime** lens (leak-on-unhappy-path)). (The "introduces exception-based handling" variant is moot here — exceptions are disabled project-wide with `-fno-exceptions`, so a `try`/`catch`/`throw` fails to compile; it is not a manual-review surface.)

**Automation bias (the reviewer-side type).** The failure isn't only in the code — it's in how the code is reviewed. AI-assisted authors wrote *less* secure code yet believed it was *more* secure `[CCS 2023]`; in the wild, the majority of AI-authored changes receive no genuine human review `[2026]`. **OWASP LLM09:2025 (Misinformation / Overreliance)** names overreliance as the harm amplifier. **Do this:** independently evaluate the change rather than steering the agent; never let an AI review tool substitute for human review; decompose large changes for incremental review; and counter your own bias — the fluency and confident comments are *designed* to earn trust you should withhold until the body has earned it.

---

### C. Embedded applicability (firmware, not web)

The block-B evidence base is overwhelmingly Python/JavaScript and web/cloud. Two recasts are needed for RP2350 dual-core Cortex-M33 / Pico-SDK / QP/C bare-metal firmware: **ADD** the bare-metal failure modes the web corpus never tests, and **DROP/deprioritize** the web-centric findings that don't transfer (each leaves a thin residual worth naming). The transferable core — passes-tests-but-wrong, spec-noncompliance, confabulation, unchecked returns, automation bias — carries over unchanged and **gains** weight (no MMU, no sanitizer, no GC safety net underneath).

**ADD — review criteria specific to firmware:**

- **Volatile-as-cross-core-barrier / weak-memory ordering.** finding: `volatile` used as if it were a cross-core synchronization barrier; missing acquire/release reasoning on a shared flag, FIFO, spinlock, or double-buffer; sequential-consistency assumed on a weakly-ordered ARM core. `volatile` prevents compiler reordering but issues no hardware memory barrier — data written on one core may not be visible on the other. LLMs reason poorly about relaxed memory models (40–72% accuracy on relaxed-model tasks; ARM litmus tests). This is the highest-stakes ADD on a dual-core split: ask "who else reads/writes this, on which core, and what orders it?" for every shared mutable. (This is the same surface as the **Control-flow discipline** (volatile = MMIO only; JSF 205 / CP.8) and **Concurrency** (volatile-as-sync, ownership) lenses — walk the detailed criteria there, not twice; this spine entry is the heads-up.)

- **HW-register / MMIO + SDK-symbol hallucination.** finding: the *silent, resolving* half of confabulation — a wrong register offset, vendor-layout confusion, or a mis-typed symbol that *does* resolve to something valid. Verify register addresses/offsets against the datasheet and SDK symbols against the actual headers. (The invented-symbol / nonexistent-register half fails *closed* at compile/link for vendored SDK code — a build error, not a manual-review surface — so it is not what you read for here.)

- **Peripheral init SEQUENCE / lifecycle.** finding: code that compiles but leaves the hardware in the wrong state — mis-ordered init, a missing enable/begin step, a bad protocol state transition, a resource added at init but never paired with teardown (or removed twice). The order and the lifecycle pairing are correctness, not style, and tests that exercise the steady state never see the wrong transition. Walk init→use→teardown as a sequence.

- **Blocking-in-cooperative-scheduler / ISR timing.** finding: a blocking call (busy-wait, poll-to-completion, slow peripheral op) inside a handler that runs under a cooperative active-object scheduler, or work too heavy for ISR/handler context. Under run-to-completion scheduling, a handler that blocks longer than its budget starves every other object — **blocking is itself the defect**, not a performance nit. The agent validates the blocking call as individually correct and never weighs it against the scheduler's run-to-completion contract.

- **Functionally-correct-but-safety-blind hardware code.** finding: code that does the functional job but ignores the hardware's safety surface — a DMA setup that omits the lock/status check, a write that bypasses a write-once protection, a path that's correct in the happy case but blind to the peripheral's failure modes. The functional-vs-safety gap is the documented firmware face of the security-doesn't-improve-with-scale type. Correct output is not the same as safe hardware handling.

**DROP / deprioritize — web-centric, with residual:**

- **Injection (XSS / SQLi / SSRF / CSRF / path traversal).** DROP — there is no DOM, SQL engine, HTTP server, or filesystem in the firmware control loop. **OWASP LLM05:2025 (Improper Output Handling)** still applies to the residual: any **CLI / UART / USB byte-stream parser** is an untrusted-input sink — treat LLM-written parsing of a serial frame or command token with zero trust (check conversion failure and out-of-range before use; LLMs assume well-formed input from example-rich training data and omit the unhappy-input handling real byte streams require).

- **Package hallucination / slopsquatting.** DROP for firmware (fails closed at link against the vendored SDK) — but the type **re-homes** as HW-register/SDK-symbol confabulation (ADDed above), and stays **HIGH** for any **host-side Python** tooling/CI in the repo, where a hallucinated import is a live supply-chain risk.

- **Cloud-IAM / secret-store / hardcoded credentials / JWT / S3.** DEPRIORITIZE — no cloud or auth substrate in the control loop. Residual: **flash-stored device keys, secure-boot material, and host-side deploy credentials** still warrant the secrets discipline.

---

_Full sourced criteria + IDs: L2P5_RP_SOURCES_2026-06-25.md Part 1 (Spine 1-4) + Passes 2-4._

---

## Comments & documentation quality

**When you are walking:** every group in the itinerary — comments live in all of them — but spend the most judgment on the dense-comment subsystems: `drivers/` (datasheet paraphrase), `fusion/` (algorithm/math narration), `safety/` and `flight_director/` (preconditions and "why" that must not be lost), and the `include/rocketchip/` public headers (function preambles and contracts). Walk each `.cpp` together with its `.h`.

**Governing rule.** Two JSF AV anchors bound this class. Rule 131: *"One should avoid stating in comments what is better stated in code (i.e. do not simply repeat what is in the code)."* Rule 134: *"Assumptions (limitations) made by functions should be documented in the function's preamble."* Together they set the test for every comment you read — delete what merely restates the code; keep and demand what records intent, assumptions, and limitations the code cannot express.

**What to look for**

This is a judgment class, not a counting class. Read each comment and ask one question: if I deleted it, would any information be lost? Then ask the inverse on the silent lines: is there a "why" here that the code can't say and a future reader will need?

1. **Restatement of code (JSF 131 / CCG NL.1 — "Don't say in comments what can be clearly stated in code").** Where the language can already express a fact — a named variable, a typed constant, an enum, a `const`/`constexpr`, a well-named function — the code should express it and there should be NO redundant comment. The canonical bad example is `auto x = m * v1 + vv; // multiply m with v1 and add the result to vv`. The tell: deleting the comment loses zero information. That comment is noise that will rot out of sync on the next edit. Verdict: delete. (Find-hint: scan the line directly after each `//`.)

2. **State intent, not mechanics (CCG NL.2 — "State intent in comments").** A good comment on a non-trivial function or block says WHY it exists or WHAT it must achieve — the contract — and stays true even if the implementation were rewritten. Preconditions, units, invariants, and the reason for a non-obvious choice belong here. The tell of a bad one: it narrates mechanics the code already shows, OR it disagrees with the code. NL.2's own warning is load-bearing — *"If the comment and the code disagree, both are likely to be wrong."* A comment that says "milliseconds" over code that stores microseconds is worse than no comment. Verdict: rewrite to intent, or correct the disagreement (and treat the disagreement itself as a latent bug).

3. **Point to the spec; do not paraphrase it (CCG NL.3 — "Keep comments crisp").** Comments are concise. Documentation that belongs in a specification POINTS to the spec — cite the datasheet section, the standard clause, the ICD section — rather than re-paraphrasing it inline. The tell: a bloated multi-paragraph block that is essentially a re-paraphrase of a datasheet, RFC, or standards section, pushing code off-screen. On a standards-fidelity / sensor-driver codebase this is doubly dangerous: the paraphrase can be subtly wrong AND it duplicates an authoritative source it will inevitably drift from. Proper form is a short pointer (for example `// see RP2350 datasheet §11.7` or `// DPS310 datasheet §7.6`). Verdict: replace the paragraph with the one-line pointer.

4. **Document the function's assumptions and limitations (JSF 134).** A function that carries a precondition, a unit expectation, an ordering requirement, a "must be called after X", or a range it does not validate must say so in its preamble. The tell: a load-bearing line or a function with a non-obvious contract and no statement of it — a hardware quirk, a safety invariant, a surprising ordering, a magic constant whose meaning isn't self-evident. These are the comments you KEEP and ADD, not delete. Verdict: add the missing rationale / preamble assumption.

5. **Commented-out code (CERT MSC04-C — "Use comments consistently and in a readable fashion").** No real code is disabled by wrapping it in `/* … */` or `//`. Any block of genuine C++ sitting inside a comment is a finding: delete it (version control remembers it) or, if it is a deliberate build/platform variant, use conditional compilation (`#if 0 … #endif`, `#ifdef`) with a stated reason. Watch also for a `/*` nested inside a comment that could swallow a trailing `*/` and silently disable real code — MSC04-C calls out non-nested delimiters for exactly this reason. Verdict: delete, or `#if 0` with a reason.

6. **Comments that describe dead or unreachable code (CERT MSC12-C — "Detect and remove code that has no effect or is never executed").** Disabled/dead code and the comments describing it should be removed once obsolete; documentation must describe code that actually runs. If a comment describes a path the code can no longer take — a guard that is always false, a function never called — the comment is documenting a lie, a false map of the system. (Exception: code intentionally `#ifdef`'d out for another platform/build, with a stated reason, is fine.) Verdict: remove the stale doc-comment along with the dead code it maps.

> Net principle: delete what restates code or paraphrases an external doc; keep and add what captures *why* — intent, assumptions, hardware quirks, invariants, and units. A pointer beats a paraphrase every time.

**▸ How an AI agent gets this wrong**

- **Over-commenting the *what* to look thorough (NL.1).** Agents restate self-evident code by reflex, manufacturing comment debt that diverges from the code on the next edit and buries the few comments that actually carry intent.
- **Paraphrasing the token stream instead of the domain (NL.2).** LLM comments narrate what was just typed (`// initialize the mutex`) rather than encoding the domain intent, the units, or the safety reason. They restate WHAT and almost never WHY — and go stale once the line is later edited.
- **Confidently paraphrasing an external standard inline (NL.3).** LLMs produce verbose, essay-style block comments that paraphrase a datasheet or standard (doc-paraphrase instead of doc-pointer). On a standards-fidelity codebase the paraphrase can be subtly wrong and it duplicates the spec, so it rots.
- **Leaving prior/alternative implementations commented out (MSC04-C).** Agents keep "the old version just in case," or comment out a block to make tests pass and never remove it — dead, misleading clutter on a safety-critical build.
- **Attaching doc-comments to code they later neutralize (MSC12-C).** Because agents edit locally and don't reason about whole-program reachability, they leave doc-comments and headers on code that is never reached — a green build that documents behavior the system can't actually produce.

### Judging criteria

| Situation | Verdict |
|---|---|
| Comment explains *why* (hardware quirk, invariant, safety rationale, surprising ordering, function assumption per JSF 134) | **PASS** — keep |
| Comment restates what well-named code already says (JSF 131 / NL.1) | **FAIL** — delete |
| Comment paraphrases an external doc/datasheet/standard instead of pointing to it (NL.3) | **FAIL** — replace with a one-line pointer |
| Comment and code disagree, e.g. wrong units (NL.2) | **FAIL** — correct the disagreement (and check for a latent bug) |
| Commented-out code (MSC04-C) | **FAIL** — delete (git has it) or `#if 0` with a reason |
| Doc-comment describing dead / never-executed code (MSC12-C) | **FAIL** — remove with the dead code |
| Load-bearing line with no "why", or function with an undocumented assumption/limitation (JSF 134 / NL.2) | **PARTIAL** — add the rationale / preamble |
| Multi-paragraph rationale bloating a `.cpp` function instead of a crisp pointer (NL.3) | **FAIL** — move the rationale out, leave a one-line pointer |

### Findings

| File:line | Smell (131/134, NL.1/2/3, MSC04/12) | What it restates / lacks | Verdict | Disposition |
|-----------|-------------------------------------|--------------------------|---------|-------------|
| | | | | |

_Full sourced criteria + IDs: L2P5_RP_SOURCES_2026-06-25.md Support A._

---

## Assertions

**When you are walking:** every group on the itinerary — assertion density is a whole-codebase lens — but it bites hardest in the longer, load-bearing functions of safety/, fusion/, flight_director/, core1/, and drivers/ (the work-doing files, not the data tables or vendored headers).

> **Governing rule.** Power of Ten Rule 5: *"The assertion density of the code should average to a minimum of two assertions per function."* JPL-C Rule 16: *"Assertions shall be used to perform basic sanity checks throughout the code. All functions of more than 10 lines should have at least one assertion... Assertions are used to check for anomalous conditions that should never happen in real-life executions."* Two clauses live in that text: a **density** floor (mechanical) and a **meaning** requirement — an assertion must check an anomalous condition that should never happen, and (P10-5) must be **side-effect free**. The mechanical half is countable; the meaning half is the manual job.

On this firmware assertions are a genuine safety mechanism, not a debug-only no-op — they route to the project fault path, so a tripped assertion is a real runtime event. That raises the stakes both ways: a *missing* load-bearing precondition is an un-caught anomalous condition, and a *vacuous* one is fault-path noise that proves nothing.

### What to look for

1. **A low-density function flags the candidate; you judge the missing invariant (P10-5 / JPL-16).** The count itself is mechanical — the ~2/function average and JPL-16's *"any function over 10 lines should carry at least one"* are a countable/length-measurable metric (a one-shot grep; function length is already measured by `readability-function-size`). Let that surface the candidates. The manual job is the part no count can do: for a flagged low-density function (a 40-line guard evaluator or sensor-decode routine with no entry checks), name the load-bearing precondition it is actually *missing*, and judge whether any present checks are real invariants or padding. Don't hand-count — judge the gap.

2. **Meaningful vs vacuous (JPL-16's "anomalous conditions that should never happen").** A good assertion states an invariant that, if false, means the program is already broken — a bound that must hold, a pointer that must be non-null *by contract* (not one the line above just assigned), a state the machine must be in. Vacuous tells: `assert(true)`, asserting a constant, or re-asserting what the type system, an `enum`, or the immediately-preceding statement already guarantees. These count toward density and check nothing. Treat density-padding as a failure of the *intent* of P10-5, not a pass.

3. **Load-bearing preconditions that are missing (JPL-16 sanity checks at entry).** The high-value manual catch: a function that consumes a parameter assuming a range, a non-null pointer, a prior-init state, or an index in-bounds — with no assertion asserting it. The code "runs fine" because callers happen to behave; the contract is checkable only by reading. Demand the entry-point sanity check the rule calls for. (Find-hint: read each function's first few lines and ask "what does this body assume about its inputs that nothing here verifies?")

4. **Side-effect-free is gate-covered — cite, don't hand-walk (P10-5 side-effect clause).** That the assertion expression must not change state (`assert(x = init())`, `assert(buf.pop())`, `assert(++n < max)`) is caught mechanically by `bugprone-assert-side-effect` (enabled in `.clang-tidy`). It is a deterministic check, not an eyeball one — cite the gate in any remediation writeup rather than walking for it.

5. **Prefer compile-time over run-time where the check is static (CCG P.5: "Prefer compile-time checking to run-time checking").** Some "sanity checks" are constraints the type system, `static_assert`, or `constexpr` could enforce — an out-of-contract call then fails to *compile* rather than tripping a fault at run time on a flight core. Where a precondition is statically knowable (a fixed array size, a layout assumption, a numeric range on a constant), the strongest form is a compile-time check, not a runtime assertion. Note any runtime assert that re-checks something the types already guarantee, or that could be lifted to `static_assert`.

### ▸ How an AI agent gets this wrong

- **Decorative density-padding.** Asked to "add assertions" or to hit ~2/function, agents insert checks that satisfy the count but verify nothing — `assert(ptr != nullptr)` immediately after `ptr = &thing;`, or asserting a value the line above set. A wall of assertions reads as diligence and passes every gate; it does not satisfy JPL-16's "anomalous conditions that should never happen."
- **Under-asserting the real preconditions (JPL-16 param-validity).** The load-bearing entry checks — range, non-null-by-contract, valid-state, in-bounds-index — are exactly the ones agents omit, because the code works without them on the happy path the tests exercise. The missing precondition is invisible per-line; only reading the function's *assumptions* surfaces it.
- **Reproducing runtime guards where the standard wants compile-time (CCG P.5).** Agents reproduce training-data idioms that re-check at run time what the types already guarantee, instead of lifting the constraint to `static_assert`/`constexpr`. On a bare-metal target a runtime-only guard for a statically-decidable fact is a weaker, later-failing form of the check.
Both the padding and the under-asserting compile and pass; neither matches the rule's intent. The manual judgment — meaningful-vs-vacuous, and the missing load-bearing precondition — is the whole job here.

### Judging criteria

| Situation | Verdict |
|---|---|
| Vacuous / tautological Boolean, or re-asserting what the type/prior line guarantees | **FAIL** — delete or replace with a real invariant |
| Long, load-bearing fn (>10 lines, JPL-16) with no precondition assertions | **PARTIAL** — add the missing entry-point sanity check |
| Runtime assert re-checking a statically-decidable fact (CCG P.5) | **PARTIAL** — lift to `static_assert`/`constexpr` where feasible |
| Meaningful invariant / precondition assertions present, density adequate | **PASS** |

### Findings

| File:line | Assertion | Issue (vacuous / side-effect / missing / OK) | Verdict | Disposition |
|---|---|---|---|---|
| | | | | |

_Full sourced criteria + IDs: L2P5_RP_SOURCES_2026-06-25.md Spine P.5 + P10 Rule 5 / JPL Rule 16._

---

## Declaration scope & object lifetime

**When you are walking:** primary on `active_objects/` (QP/C AOs — stack-local event posting), `core1/` and the cross-core boundary, the boot/init path in `main.cpp` and `shared_state.cpp`, and any two-phase-init driver in `drivers/`, `safety/`, and `fusion/`. Touch it anywhere a large object, a shared object, or an object-with-a-`begin()` is declared.

**Governing rule (verbatim):** *"Declarations should be at the smallest feasible scope."* (JSF AV 136; the same rule appears in Power of Ten Rule 6 — *"Data objects must be declared at the smallest possible level of scope"* — and JPL-C Rule 13 — *"Data objects shall be declared at the smallest possible level of scope"*). And on the lifetime/init half: *"All variables shall be initialized before use"* (JSF AV 142), with *"Variables will not be introduced until they can be initialized with meaningful values"* (JSF AV 143). Two halves are **gate-covered — cite, don't re-walk**: smallest-scope / no-shadow (`-Wshadow`), and read-of-an-indeterminate-value, i.e. flat used-before-init (`cppcoreguidelines-init-variables` + the `clang-analyzer-core.*`/`clang-analyzer-cplusplus.*` family = CERT EXP53-CPP — the analyzer reasons per-path). **The manual residual is what no gate sees: ownership and lifetime reasoning (use-after-free, two-phase-init, leak-on-unhappy-path, cross-core lifetime) plus JSF 143's "introduced before a meaningful value exists" — the declaration-distance / value-not-yet-meaningful judgment.**

**What to look for**

- **Introduced before there is a value to give it (JSF 143 / CCG ES.21 / ES.22).** A name declared, then default- or zero-initialized, then conditionally assigned far below (`string s;` ... fifty lines later `s = ...`). What good looks like: each declaration sits at the point it can be given its real initial value, so its scope is minimal and the paragraph is self-contained. The tell: a declare-then-fill split that smears one logical step across the whole function and hides whether *every* path actually writes it before the read. CCG ES.22's own reason: "Don't risk used-before-set." This is the half of the class a human carries — the gates check "definitely assigned before read," not "introduced too early to mean anything."

- **Flat used-before-init is gate-covered — cite, don't hand-walk (JSF 142 / CCG ES.20 / CERT EXP53-CPP).** A read of an indeterminate value (a field read on a path where it was never set) is caught by `cppcoreguidelines-init-variables` + the `clang-analyzer` uninitialized-read checks — the analyzer reasons per-path, so this is not the human's pass. What stays manual is the *value-not-yet-meaningful* and *lifetime* cases the analyzer can't judge — the two-phase-init, use-after-free, and ownership bullets below, not "is it assigned before read."

- **Two-phase-init objects used before their second phase.** An object whose constructor does not fully initialize it — it needs a later `begin()` / `init()` before any real operation — and a call reaches it before that second phase runs. Compiles, links, passes most host runs; bites only at runtime when the half-constructed object is operated on. What good looks like: the contract "construct then begin then use" is visibly satisfied on every path that reaches a use. (A find-hint: scan for `.begin()` / `_init(` and confirm it precedes first use.)

- **Used after its lifetime ends — use-after-free / dangling (CCG P.8 ownership).** A reference, pointer, or stored address that outlives the object it names. What good looks like: every acquired resource has a single owner and a deterministic release on **every** exit path, including early returns, and ownership is visible within one function's scope (RAII). The tell: a value escapes (stored, posted, returned by reference) while the storage it points into is about to be reclaimed.
  - **The canonical case on a QP/C codebase:** a stack-local event subclass posted to an active object. QP/C stores the raw pointer — it does **not** copy the event — so the event memory must outlive the post; a function-local event is reclaimed when the poster returns and the receiver later dispatches a dangling pointer. What good looks like: posted events use static (or pool-allocated) storage so the memory survives until dispatch. This is a textbook P.8 ownership/lifetime defect that compiles, passes tests, and can run "fine" for a long time by luck before the stack slot is reused.

- **Leaked / non-released resources on the unhappy path (CCG P.8 — "Don't leak any resources").** Walk **every** exit, not the happy path the tests exercise. The tell: an acquire-then-return-early on an error branch that skips the release; or code that assumes a garbage-collector / destructor that does not exist in this no-heap-after-init build. What good looks like: a resource's release path is reached on every return, including the invented defensive error branches.

- **Data-race lifetime on shared state (CCG CP.2 — "Avoid data races").** Where this class meets concurrency: an object's *effective* lifetime spans two cores, and one context reads while another writes (or frees) without synchronization. What good looks like: cross-core shared writable state is synchronized or eliminated — no two contexts access it where at least one writes. The tell is invisible per-line; it surfaces only when you ask "who else touches this, and when does it stop being valid?" across the whole file.

- **Mandated wide scope is not a violation.** Some objects are *required* to be file-scope `static` (large objects that would overflow a small stack if stack-allocated). A mandatory file-scope static is correct **by reference** — it is not a smallest-scope finding, and "tidying" it into a function-local reintroduces a silent stack-overflow. When you see a large object held at file scope, confirm whether the wide scope is the mandate before flagging it. (Smallest-scope tightening applies only where no such constraint exists.)

**▸ How an AI agent gets this wrong**

- **Reproduces managed-language cleanup idioms.** It assumes implicit cleanup / defer-by-convention / a GC that this no-heap-after-init build does not have, and adds defensive-looking error branches that *leak on the path no test covers* — acquire-then-return-early without the release.
- **Posts stack-local events.** It writes an event on the stack and posts the pointer, assuming the framework copies it (it does not). Locally correct, tests pass, runs for a long time before the reused stack slot turns the dangling pointer into corruption.
- **Misses two-phase-init contracts.** It operates an object before its `begin()`/`init()` because the construction *compiles* and there is no test that reaches the half-initialized path; the defect only appears at runtime on hardware.
- **Declaration distance is invisible to it (ES.21/ES.22).** The agent sees the declaration, sees a later valid use, and confirms — it does not model reader working-memory cost or measure the gap, so front-loaded "declare everything at the top" dumps and default-then-assign-far-below splits sail through. Narrowing scope or sliding a declaration to first use is a structural improvement, not a bug fix, so it never proposes it.
- **Introduces unguarded cross-core shared state (CP.2).** It adds a mutable static/global "cache" or "flag" that compiles and passes single-core tests but is touched by both cores with no barrier — a lifetime/ownership defect that surfaces only as rare timing-dependent corruption, giving the agent zero mechanical signal.

### Judging criteria

| Situation | Verdict |
|---|---|
| Large object held at mandated file-scope `static` (stack-overflow prevention) | **PASS** — by-reference, NOT a scope violation |
| Scope wider/earlier than needed, no mandate | **PARTIAL** — narrow it / declare at first meaningful init (JSF 136/143) |
| Use-after-lifetime / dangling, or a stack-local event posted to QP | **FAIL** (P.8) — *(flat use-before-init is gate-covered: `init-variables` + analyzer / EXP53)* |
| Two-phase object operated on before its `begin()`/`init()` | **FAIL** (JSF 143) |
| Resource acquired but not released on some exit path | **FAIL** (CCG P.8) |
| Object's lifetime spans two cores with unsynchronized read-while-write | **FAIL** (CCG CP.2) |
| Declared at smallest feasible scope, initialized with a meaningful value | **PASS** |

### Findings

| File:line | Declaration | Scope/lifetime issue | Mandated-static? | Verdict | Disposition |
|---|---|---|---|---|---|
| | | | | | |

_Full sourced criteria + IDs: L2P5_RP_SOURCES_2026-06-25.md Spine ES.5/6/20/21/22, EXP53-CPP, CP.2, P.8._

---

## Class & interface design

**When you are walking:** primary lens for `include/rocketchip/` public headers (Section 13 of the itinerary) and for every class/struct definition you meet along the way — the QP/C AOs in `active_objects/`, the driver classes in `drivers/`, the calibration and logging types, the small value types in `math/`. Most "classes" here are AOs or POD-ish structs; the headers section is where this lens earns its keep.

The binding standard for this lens is the house standard's class rules — **JSF AV C++**, verbatim: **AV 67:** *"Public and protected data should only be used in structs—not classes."* **AV 68:** *"Unneeded implicitly generated member functions shall be explicitly disallowed."* **AV 72:** the class invariant is *"a part of the postcondition of every class constructor."* **AV 76:** *"A copy constructor and an assignment operator shall be declared for classes that contain pointers to data items or nontrivial destructors."* **AV 78:** *"All base classes with a virtual function shall define a virtual destructor."* **AV 79:** *"All resources acquired by a class shall be released by the class's destructor."* **AV 87/88:** hierarchies *"based on abstract classes,"* multiple inheritance only as *"n interfaces plus m private implementations."* **AV 177:** *"User-defined conversion functions should be avoided."* These are the rules a class either honors or quietly fails while compiling green. The **C++ Core Guidelines (C.2/C.21/C.35/C.46/C.131/I.25) and CERT (OOP50/52/58) cited in the criteria below are supporting elaboration — modern restatements and the gate names — not the governing standard** (CCG/CERT are references, not adopted house sources). Mappings: class-vs-struct & trivial accessors (C.2/C.8/C.131) → **AV 67 + AV 72**; rule-of-five (C.21) → **AV 68 + AV 76**; virtual dtor (C.35 / OOP52) → **AV 78**; resource release (P.8 / C.31) → **AV 79**; explicit/implicit conversion (C.46) → **AV 177**; thin interfaces (I.25) → **AV 87/88**; no-virtual-call-before-fully-constructed (OOP50) → **AV 71**.

> **Gate-covered here — cite, don't hand-walk (the same move as in **Declaration scope & object lifetime**).** Three of the detections below are mechanical and already wired in the build: a polymorphic base with a public non-virtual destructor is a `-Wnon-virtual-dtor` `-Werror` compile failure (C.35 / OOP52-CPP); the rule-of-five *asymmetry* — declared one of {copy/move ctor, copy/move assign, dtor} but not all — is surfaced by `cppcoreguidelines-special-member-functions`; a non-`explicit` single-argument constructor is flagged by `google-explicit-constructor`. The gate finds those candidates; the walk judges only the *disposition* below. Don't re-hunt them by eye.

### What to look for

The dominant passing-but-improper pattern here is **encapsulation theater**: private fields plus trivial accessors wrapping an aggregate that has no invariant, alongside special-member and polymorphic-base defects that compile and pass every test. Walk each type and judge against these:

- **Class-vs-struct by invariant (CG C.2).** A `class` is justified when its constructor establishes a relationship its members must maintain — a real invariant, private data, validating construction. A bag of values that vary independently should be a `struct` with public members. *Finding:* a `class` with private members and accessors but no enforced invariant (a struct masquerading as a class); or the reverse — a public `struct` for a type that genuinely has a cross-member constraint (e.g. a `count` that must track a `len`), leaving the invariant enforceable only by convention.

- **Interface vs implementation (CG C.3).** The public section should present a coherent, minimal contract; implementation detail stays private. *Finding:* a class whose public section interleaves implementation helpers, internal scratch state, or implementation-only members — callers can reach into the mechanism, so the "interface" is not a contract.

- **Keyword honesty (CG C.8): "Use class rather than struct if any member is non-public."** A type with any non-public member should say `class`, signalling that it controls access and likely maintains an invariant. *Finding:* a `struct` carrying a private or protected member — legal, compiles, but misleads a reader who assumes a transparent aggregate. Watch for a `struct` that accreted a private helper during a multi-edit session and never changed its keyword.

- **Rule of five — disposition only (CG C.21).** `cppcoreguidelines-special-member-functions` surfaces the candidate (declared one special member, not all); your job is the disposition the gate can't make. *Finding (high value on bare-metal):* a class with a resource-freeing destructor — releasing a hardware handle, a DMA channel, a claimed lock — that left copy/move implicitly available. The compiler-generated copy shallow-copies the owned handle; the destructor then runs twice, double-releasing the peripheral. Decide per case: define the deep copy, or `=delete` copy/move because the resource is unique. The mirror disposition — a spurious Rule-of-5 on a class that owns nothing — strip it.

- **No virtual dispatch in ctor/dtor (CERT OOP50-CPP: "Do not invoke virtual functions from constructors or destructors").** During base construction/destruction the most-derived override is not called (and a pure virtual is UB). *Finding:* a ctor/dtor that calls a virtual member — directly or via a helper — expecting the derived override to run. The classic shape is a virtual `init()` called from the base constructor; it binds to the base version, and any test that exercises the fully-constructed object passes anyway.

- **Explicit single-arg constructors — confirm intent on the gate's hits (CG C.46).** `google-explicit-constructor` flags every non-`explicit` one-argument constructor; the only manual residual is the rare deliberate case. *Finding:* on a flagged `Timeout(int ms)` / `RegisterAddr(uint32_t)`, confirm whether the implicit conversion was genuinely intended and documented — if not, it lets an `int` silently become a `Timeout`, defeating the strong typing this codebase relies on for typed units and register wrappers. The bulk detection is the gate's; the intent call is yours.

- **Trivial accessors (CG C.131) / data classes (Fowler "Data Class" smell).** A member with no invariant should just be public (or the type should be a struct); accessors should exist only where they add semantic value — validation, conversion, locking, invariant maintenance. *Finding:* private fields shadowed one-for-one by `getX()/setX()` that only read/write the member — encapsulation theater that mis-signals an invariant. Its sibling is the data class: a type that is only fields plus getters/setters, with all behavior scattered into free functions or callers. Either it has no invariant (make it a struct) or the invariant is unprotected and reconstructed by every caller.

- **Thin interfaces (CG I.25): "Prefer empty abstract classes as interfaces to class hierarchies."** A polymorphic interface should be an abstract class of pure virtual functions only (plus the right destructor per C.35) — no data, no implementation. *Finding:* an "interface" base carrying data members or concrete implementation, forcing derived classes to inherit state and couple to base internals.

- **Const-correct copy (CERT OOP58-CPP).** A copy constructor and copy assignment take the source by reference-to-const and leave it unchanged. *Finding:* a "copy" whose source parameter is non-const and is mutated — transferring ownership or resetting a flag on the source under the name of a copy (a move wearing a copy's signature).

### ▸ How an AI agent gets this wrong

- It wraps a plain value aggregate in a class with private members plus trivial `get_`/`set_` for every field and calls it "good encapsulation" — producing data classes dressed as classes (the signature tell for C.131). The reverse also happens: a public struct for a type that has a real cross-member constraint, leaving the invariant enforceable only by convention.
- It promotes helper methods and intermediate state to `public` while iterating, because making something public is the fastest way to get a call to compile — leaking implementation through the "interface."
- It writes a resource-freeing destructor and stops, never declaring copy/move; the implicit shallow copy then double-frees the owned handle. The test never copies, so it passes.
- It writes a virtual `init()` and calls it from the base constructor assuming derived behavior runs; it compiles and binds to the base version.
- It routinely omits `explicit` on single-arg constructors — clean-reading code that opens silent conversions undermining typed units and register wrappers.
- It folds convenience state and default implementations into an interface base "to avoid duplication," yielding interface bases with data members — a design defect each line of which is locally reasonable.
- It conflates copy and move, writing a "copy" that steals from or edits the source via a non-const source parameter.
- It produces data classes by default — entity-like types of pure fields plus accessors with logic spread across callers.

### Judging criteria

| Situation | Verdict |
|---|---|
| Interface minimal+complete, invariants guarded, RAII clean | **PASS** |
| Public/protected data member (not a deliberate POD struct) | **FAIL/PARTIAL** — encapsulate, or make it an honest `struct` |
| Pointer-owning class missing copy/assign decl or `=delete` | **PARTIAL** |
| Spurious Rule-of-5 on a class that owns nothing | **PARTIAL** — remove |
| Public inheritance that isn't is-a | **FAIL** — remodel as has-a |

### Findings

| File:line | Type | Smell | Verdict | Disposition |
|---|---|---|---|---|
| | | | | |

_Full sourced criteria + IDs: L2P5_RP_SOURCES_2026-06-25.md Support B._

---

## Templates

**When you are walking:** the few files that actually define templates — most concentrated in `calibration/lm_solver.{cpp,h}` (residual/jacobian dispatch via deduced callables) and `math/mat.h`; spot-check `include/rocketchip/` headers as you pass them. Template use across the tree is low, so this lens is cheap — find every template, judge each one. Don't skip it for being small.

**Governing rules (house standard — JSF AV C++, verbatim):** templates are governed by the JSF AV template rules — **AV 101:** *"Templates shall be reviewed as follows: 1. with respect to the template in isolation considering assumptions or requirements placed on its arguments…"* (the manual-review mandate); **AV 102:** *"Template tests shall be created to cover all actual template instantiations."*; **AV 103:** *"Constraint checks should be applied to template arguments."*; **AV 105:** *"A template definition's dependence on its instantiation contexts should be minimized."*; **AV 106:** *"Specializations for pointer types should be made where appropriate."* (AV 104 — *"a template specialization shall be declared before its use"* — is compiler-enforced, gated not walked.) The durable defect is *unconstrained or speculative generality* — code that compiles and passes against the one type it is instantiated with while saying nothing about what it requires. The **C++ Core Guidelines T-series below is the modern (C++20-concepts) review technique for carrying out AV 101's mandate — supporting elaboration, not the governing standard** (CCG is a reference, not an adopted house source). Mappings: T.10/T.11/T.20/T.41 (concepts) elaborate **AV 103**; T.47/T.61/T.69 (ADL / coupling / bloat) elaborate **AV 105**; T.102 *is* **AV 102**; T.150 pins **AV 103** at the type.

**What to look for**

The signature template failure is **unrequested/speculative generality**: asked for one concrete thing, a fully templated, policy-parameterized abstraction is emitted, with the type contract living only in the body. The single instantiation compiles, runs, and lints clean. Walk each template against these, in plain terms:

- **Unconstrained type parameters (T.10).** A `template<typename T>` whose body silently assumes operations (`t.tick()`, `a < b`, `c[i]`, `<< t`) with no concept or `requires` guarding them. *Good:* every parameter that is more than "merely a type" carries a concept naming the semantic role it must satisfy; bare `typename`/`auto` appears only where nothing but "it is a type" can be assumed, and is commented. *Tell:* it compiles for the one type instantiated today; a second instantiation fails deep inside with a wall-of-text error.
- **Reinvented standard concepts (T.11).** *Good:* where a standard-library concept expresses the requirement (`std::integral`, `std::copyable`, `std::input_iterator`, `std::invocable`), the template uses it directly; locally defined concepts appear only for domain notions the standard cannot name. *Tell:* a bespoke concept that duplicates — imperfectly — a standard one (`sortable` vs `std::totally_ordered`/`std::regular`), usually weaker or subtly wrong, letting unintended types through.
- **Meaningless syntactic "concepts" (T.20).** *Good:* a concept expresses a coherent semantic notion and demands a complete operation set (a `Number` requires `+ - * /` together); single-operation predicates are building blocks, never the public guard. *Tell:* a syntactic probe used as the actual interface constraint — the canonical trap is `concept Addable = requires(T a, T b){ a+b; }` matching `std::string`, so a numeric routine silently concatenates `"7"+"9"=="79"`.
- **Over-constrained concepts (T.41).** *Good:* the concept lists only the properties essential to what the template fundamentally does; incidental needs (debug streaming, logging) are not promoted into the public constraint. *Tell:* a `requires`-clause bundling a non-essential requirement — a sort-like routine demanding `Streamable<S>` only for one `cerr` debug line — over-constraining callers and churning the interface when instrumentation changes.
- **Highly visible unconstrained templates with common names (T.47).** *Good:* unconstrained function templates named like `operator==`, `begin`, `swap`, `size` are constrained and/or kept out of namespaces that also define concrete types. *Tell:* an unconstrained `template<class T1,class T2> bool operator==(T1,T2)` sitting beside concrete types — needing no conversions, it out-competes the intended overload via ADL and silently mis-dispatches.
- **Over-parameterized members / SCARY (T.61).** *Good:* a member type/function depends on every template parameter it is nested under; anything that doesn't is hoisted into its own minimally-parameterized type. *Tell:* a nested member formally parameterized on arguments it never references (the classic `List<T,A>::Link` depending on `A` it never uses) — every distinct argument combination spawns a redundant identical instantiation, bloating code size. That matters directly on a flash-constrained RP2350.
- **Accidental customization points (T.69).** *Good:* an unqualified non-member call on a dependent argument (`f(t)`) is made only where ADL-based customization is the deliberate design (like `swap`); internal helpers live in a `detail` namespace and are called qualified. *Tell:* a template body calling an internal helper unqualified — turning a private detail into an unintended ADL hook that mis-dispatches when a new type appears.
- **Gratuitous metaprogramming (T.120).** *Good:* TMP is used only where genuinely needed — value computations use `constexpr` functions, type constraints use concepts rather than `enable_if`, and metaprogramming is never hidden in macros. *Tell:* elaborate `enable_if`/SFINAE/recursive TMP where a `constexpr` function, concept, or plain overload would do. On a maintainable safety-critical codebase the gratuitous TMP is itself the defect.
- **Unintentionally non-generic code (T.143).** *Good:* generic code commits to the most general facility that does the job — iterators compared with `!=` not `<`, and parameters typed to the least-derived class/concept actually used. *Tell:* `for (i = first; i < last; ++i)`, or a function taking `Derived&` while only calling `Base`'s members. (The `.size()==0`→`.empty()` sub-case is gate-covered by `readability-container-size-empty` — cite it, don't hand-walk it.)
- **Specialized function templates (T.144) — mechanical, not walked.** An explicit function-template specialization is the literal token `template<>` on a function definition — its presence *is* the finding (function templates are never explicitly specialized; use overloading or a specialized class template instead). This is a §CM one-shot grep, not a judgment call — handle it there rather than as an eyeball criterion.
- **Concept membership left unverified (T.150).** The manual half is naming *which* types are intended to model a concept (e.g. a sensor-sample POD meant to be `std::trivially_copyable`) — a design judgment; once a type is named, confirming a `static_assert(Concept<X>)` guards it is a grep, not an eyeball pass. *Good:* every intended-to-model type pins its intent with a `static_assert` near the type, so a regression fails the build at the type with a clear message. *Tell:* a type you judge is *meant* to model a concept with no `static_assert` verifying it — so when someone adds a triviality- or copyability-breaking member, nothing fails until a distant instantiation does. (The presence check itself is mechanical; the intent call is the manual part.)

**▸ How an AI agent gets this wrong**

- Defaults to `template<typename T>` boilerplate and lets the body imply the contract — the generated call site (the only test) instantiates with one concrete type that happens to satisfy the unstated requirements, so it compiles, runs, and lints clean (T.10).
- Hand-rolls syntactic "concepts" that accidentally match the wrong types — the `Addable`-matches-`std::string` failure (T.20).
- Adds template "flexibility" for single-use code: speculative generality where one concrete function was asked for.

### Judging criteria

| Situation | Verdict |
|---|---|
| Constrained (concept/static_assert) + each instantiation tested + single clear purpose | **PASS** |
| Unconstrained but used narrowly with safe types | **PARTIAL** — add a constraint |
| Speculative generality for single-use code | **PARTIAL** — simplify |
| Instantiation with no covering test (JSF 102) | **coverage gap, not a manual call** — mechanical: the compiler emits the instantiation list (JSF 102's own note) and "is it tested?" is a test-inventory lookup; currently *ungated*, so flag it to wire/grep — don't eyeball |

### Findings

| File:line | Template | Constrained? | Instantiations tested? | Verdict | Disposition |
|---|---|---|---|---|---|
| | | | | | |

_Full sourced criteria + IDs: L2P5_RP_SOURCES_2026-06-25.md Support C._

---

## Control-flow discipline

**When you are walking:** this lens is primary on the concurrency-boundary and ISR-adjacent files — `core1/sensor_core1.{cpp,h}`, `include/rocketchip/sensor_seqlock.h` + `sensor_snapshot.h`, the `logging/ring_buffer.{cpp,h}` and `log/rc_log.cpp` channels, `safety/core1_i2c_pause.{cpp,h}`, `safety/pio_*` timers, the QP/C active objects in `active_objects/`, `main.cpp`, `shared_state.cpp` — and, for the eval-order half, every `.cpp` carrying terse arithmetic or multi-argument call sites (fusion math, drivers, `math/`).

**Governing rules (house standard — quote exactly):**
- **JSF AV Rule 205:** *"The volatile keyword shall not be used unless directly interfacing with hardware."* (the volatile half)
- **JSF AV Rule 204.1:** *"The value of an expression shall be the same under any order of evaluation that the standard permits."* — with **JSF AV Rule 204:** *"A single operation with side-effects shall only be used in the following contexts: 1. by itself…"* and **JPL-C Rule 18:** *"In compound expressions with multiple sub-expressions the intended order of evaluation shall be made explicit with parentheses."* (the eval-order half)

The **C++ Core Guidelines (CP.8, ES.43, ES.44) and SEI CERT (EXP50-CPP) cited below are supporting elaboration, not the governing standard** (both are references, not adopted house sources): CP.8 expands JSF 205 — *"volatile does not provide atomicity, does not synchronize between threads, and does not prevent instruction reordering"*; ES.43 / ES.44 / EXP50 expand AV 204.1 / JPL 18. (Side-effects-in-`sizeof` — JSF AV 166, the canon behind CERT EXP52 — is a bright-line *mechanical* check, not a manual criterion; it is tracked as a §CM to-wire item, not walked here.)

The semantic core of this class is two-sided: `volatile` must mean *true MMIO*, not *cross-core/ISR sync intent* (JSF 205); and a single full-expression must not depend on an order of evaluation the language leaves unspecified (JSF 204.1 / JPL 18).

**What to look for**

- **`volatile` = hardware only (JSF 205).** Good: `volatile` qualifies *only* objects that change outside the program — memory-mapped registers (NVIC `ISPR`, peripheral control regs), inline-asm register access. These are PASS. Finding: a `volatile` flag, ring head/tail index, or handshake variable shared between the two cores or between an ISR and main, declared `volatile` *to make sharing work*. Walk each `volatile` in the sensor-loop, ring-buffer, `rc_log`, PIO-timer, and shared-state files and classify it true-MMIO vs cross-context-sync.

- **`volatile` does not synchronize (CP.8).** Good: cross-core / ISR↔main communication uses `std::atomic`, a spinlock, the hardware FIFO, or a documented double-buffer — *not* `volatile`. Finding (the high-value tell): a `volatile` object used *as if* it provided atomicity or ordering. Per CP.8 the qualifier *"does not provide atomicity, does not synchronize between threads, and does not prevent instruction reordering"* — so the data written on one core may never become visible, correctly, on the other. The dual pairing to flag: **synchronization-by-volatile = finding**, and its mirror, **a hardware register / MMIO pointer that is NOT volatile-qualified = finding**.

- **Undefined order of evaluation (JSF AV 204.1 / JPL 18; cf. CCG ES.43).** Good: no object is read *and* modified more than once in a single full-expression without an intervening sequence point; side-effecting subexpressions are split into separate statements. The Guideline's own warning: *"You have no idea what such code does... it might do something different on another compiler... or with a different optimizer setting."* The single-scalar, same-object UB cases (`v[i] = i++;`, `a = a++ + ...`, `i = ++i + 1;`) are **gate-covered** — `-Wall` under `-Werror` enables `-Wsequence-point` on the project's arm-none-eabi-gcc, so those fail the build today; cite the gate, don't hand-walk them. The genuinely manual residual is what `-Wsequence-point` does NOT catch: a single full-expression (or sibling call arguments) that both reads and writes a shared object the *other core* touches — cross-call / cross-core order-dependence. That is the eval-order finding to read for here.

- **Order of evaluation of function arguments (JSF AV 204.1; cf. CCG ES.44).** Good: argument expressions are independent — no two arguments to the same call read-and-modify the same object, and no argument's correctness depends on left-to-right vs right-to-left evaluation. Finding: a call site whose arguments share a mutated object or an ordering assumption — `f(g(), h())` where both touch shared state, or two arguments each advancing the same stream/index. Such code "works" only because of the current compiler's incidental argument order.

- **Unsequenced side effects are UB, not style (JSF AV 204.1; cf. CERT-EXP50-CPP).** Good: side effects on the same scalar object are separated into distinct, sequenced statements (or captured in intermediates). Per CERT: *"In C++, modifying an object, calling a library I/O function, accessing a volatile-qualified value, or calling a function that performs one of these actions are ways to modify the state of the execution environment. These actions are called side effects."* Finding: UB from unsequenced side effects on one scalar — `i = ++i + 1;`, `func(i++, i);`, or any expression that modifies and separately reads the same object with no sequencing. Call this out as **undefined behavior**, not a style nit — it is the CERT-graded sibling of ES.43/ES.44. (The elementary single-scalar cases are `-Wsequence-point`-gated per ES.43 above; EXP50's manual weight is the severity framing and the unsequenced cases that span a helper call the gate can't see.)

**▸ How an AI agent gets this wrong**

- **`volatile` ≙ "shared across threads/interrupts."** LLMs strongly associate `volatile` with concurrency (a Java/C#/older-C carryover), so dual-core and ISR-shared flags get declared `volatile`. The missing atomicity/ordering only bites under optimization or a specific interleaving — it compiles, usually "works" in testing, and is a real cross-core corruption bug.
- **Terse "clever" mutate-and-read one-liners.** Agents emit compact expressions that fold a mutation and a use of the same variable into one expression. It compiles and gives the expected answer on the dev toolchain, so tests pass; the defect surfaces only under a different optimizer/compiler. Because the construct is UB (EXP50-CPP), a passing test suite is *actively misleading*.
- **Two interacting side effects in sibling arguments.** When an agent compresses setup into a single call to look concise, it can place two side effects in sibling arguments with no sequencing between them — tests pass only because of the build compiler's chosen evaluation order.

**Judging criteria**

| Situation | Verdict |
|---|---|
| `volatile` on true MMIO / inline-asm register | **PASS** |
| `volatile` used for cross-core/ISR data sharing as if it were a barrier | **FAIL** — use atomic / spinlock / FIFO (CP.8) |
| Hardware register / MMIO pointer that is NOT volatile-qualified | **FAIL** — qualify as `volatile` (JSF 205) |
| Cross-call / cross-core full-expression whose result depends on the evaluation order of a shared object (ES.43; the single-scalar UB cases are `-Wsequence-point`-gated) | **FAIL** — undefined/unspecified behavior; sequence it explicitly |
| Call whose arguments share a mutated object or ordering assumption (ES.44) | **FAIL** — make argument expressions independent |

**Findings**

| File:line | Construct | Issue (volatile-sync / eval-order) | Verdict | Disposition |
|---|---|---|---|---|
| | | | | |

_Sourced criteria (house standard): JSF AV Rule 205, 204, 204.1; JPL-C Rule 18. Supporting elaboration: C++ Core Guidelines CP.8 / ES.43 / ES.44; SEI CERT EXP50-CPP._

---

## Concurrency & shared-data ownership  *(highest consequence)*

**When you are walking:** primary on `active_objects/` (QP/C AOs — heavy for this lens), `core1/sensor_core1.{cpp,h}` (the Core 0 ↔ Core 1 boundary), `safety/core1_i2c_pause.{cpp,h}`, `logging/ring_buffer.{cpp,h}`, `include/rocketchip/sensor_seqlock.h` + `sensor_snapshot.h`, `main.cpp` (concurrency launch), and `shared_state.cpp`. Touch it anywhere a `g_`-prefixed object, an AO event, a spinlock, or a `multicore_*` call appears.

**Governing rules (binding standard text — quote exactly):** JPL-C Rule 8 (shared data): *"Data objects in shared memory should have a single owning task. Only the owner of a data object should be able to modify the object. Ownership should be passed between tasks explicitly, preferably via IPC messages."* JPL-C Rule 7 (thread safety): *"Task synchronization shall not be performed through the use of task delays."* JPL-C Rule 9 (semaphores and locking): nested use should be avoided; if unavoidable, lock acquisitions *"shall always occur in a single predetermined, and documented, order. Unlock operations shall always appear"* in the same function as the lock. JPL-C Rule 6 (IPC): *"No task should directly execute code or access data that belongs to a different task. All IPC messages shall be received at a single point in a task."*

This is the highest-consequence class because the defects are invisible to your normal evidence. A concurrency bug (a) compiles, (b) passes single-threaded host tests, (c) passes most hardware runs, and (d) surfaces only as rare, timing-dependent corruption — so neither the agent nor the gate gets any signal that anything is wrong. The through-line the research records: an LLM writes code correct for ONE execution context, reaches for the shortest sharing mechanism (a mutable global, a captured-by-reference object, a `volatile` flag), and structures locks for the happy path only. Spend the most eyeball time here.

### What to look for

Walk **every cross-core / cross-context shared object** and answer three questions out loud: who **owns** it, who **mutates** it, what **barrier** protects it. If you cannot name all three for an object, that ambiguity is itself the finding.

> **By-reference scope note (shrinks the live surface).** `std::mutex` / `std::condition_variable` / `std::scoped_lock` do not exist in authored `src/` — cross-core sync is `std::atomic<bool>` flags + SDK spinlocks / `multicore_lockout` + the one interrupts save/restore region. So the CV-predicate-loop and notify-discipline criteria (CON54 / CON55), the mutex-lifetime criterion (CON50), and the `std::mutex`-specific guard wording below are dispositioned **by reference** ("no `std::mutex`/CV in this codebase"), not hand-walked per file. The live manual surface is: `std::atomic` flags, the one interrupts-disabled region, `volatile` classification, and AO / cross-core ownership.

- **Minimize the shared writable surface (CP.3).** Good: the set of mutable objects reachable from more than one context (Core 0 vs Core 1, ISR vs main, two QP actors) is small, explicit, and each has ONE named owner; cross-boundary data is const/immutable or passed by value/message. The tell: a mutable global/static/singleton read by one core/ISR and written by another with no single owner and no documented guard — a non-const reference or raw pointer to shared state captured into a Core-1 callback or QP handler, a "temporary" shared scratch buffer, or a whole struct passed when one POD field would do.

- **Single owner, owner-only mutation (JPL 8).** Good: each shared object has one owning context that alone writes it; ownership is handed off explicitly via IPC/message, not by two contexts both reaching in. The tell: a single-owner object mutated from a non-owner core — the most consequential FAIL in this class.

- **No `volatile`-as-synchronization (CP.8).** Good: `volatile` appears ONLY for memory-mapped hardware registers / peripheral I/O; every cross-core or cross-ISR signaling variable is `std::atomic<T>` with a justified memory order, or is mutex/critical-section protected. The tell: a `volatile` flag/counter used to communicate between cores or ISR↔main, treated as if it gave atomicity or ordering — per the rule it *"does not provide atomicity, does not synchronize between threads, and does not prevent instruction reordering."* Also flag a plain non-atomic read-modify-write (`free_slots--`) shared across contexts. (Find-hint: every `volatile` is greppable — grep them first, then classify; the triage already pre-enumerates the candidate sites. Locating them is mechanical; the true-MMIO-vs-cross-core-sync classification is the manual part.)

- **No delay-as-synchronization (JPL 7).** Good: a `sleep_ms`/`busy_wait` exists because of a real timing requirement (settling time, datasheet delay). The tell: a delay used to "wait for" another core/task to finish — synchronization smuggled in as a timing assumption.

- **RAII over bare lock/unlock (CP.20).** Good: every critical section is entered via a scoped guard so the lock releases on EVERY exit path; no bare `mtx.lock()`/`mtx.unlock()` pairs, no manual save/restore-interrupts a later edit could skip. The tell: a manual lock/unlock — or `disable_irq()`/`restore_irq()`, `taskENTER/EXIT_CRITICAL`, `spin_lock`/`spin_unlock` — where some path (early `return`, `break`/`goto`, a later-added branch) can skip the unlock. *"Sooner or later, someone will forget the mtx.unlock(), place a return... or something."* On this no-exceptions codebase that removes one escape path but not early returns/breaks.

- **Every abnormal exit releases the lock (JPL 9 / CON51).** The manual half is reading control flow: in this no-exceptions codebase, does every early-return / error-status / `break`/`goto` / later-added branch out of a manually locked region still release the lock? Good: the lock is released on ALL non-normal exits. The tell: a manually locked region where an abnormal exit (early error-code return) skips the unlock — *"the mutex will be left in the locked state."* (The simpler `lock`-in-function-A / `unlock`-in-function-B mismatch is a greppable token/scope check, not eyeball work — and the live surface is one `save_and_disable_interrupts`/`restore_interrupts` pair in `psram_init.cpp`, both in the same function, plus zero spinlock pairs. So this is a control-flow knock-out check, not a from-scratch hand-walk.)

- **Acquire multiple locks safely (CP.21 / CON53).** Good: any path holding two or more locks acquires them atomically via `std::scoped_lock`/`std::lock`, or in one globally-agreed, documented order, so acquisition order can never form a cycle. The tell: two locks (or a mutex plus interrupt-disable, or two spinlocks) acquired in one order on path A and the reverse on path B — the textbook deadlock; runs fine until the exact interleaving occurs.

- **Never call unknown code while holding a lock (CP.22 / CON56).** Good: inside a critical section the code calls only short, known, non-blocking, non-reentrant leaf operations on the protected data; callbacks, virtual dispatch, function pointers, QP event posts, and logging happen AFTER the lock is released. The tell: a callback, virtual function, function pointer, `std::function`, event post, or any call into uncontrolled code invoked while a lock / critical section / interrupts-disabled region is held — risking re-entry/self-deadlock (CON56: re-locking a non-recursive mutex the calling context already holds) or blocking everyone waiting. On bare-metal this also appears as work done with interrupts disabled too long.

- **Mutex outlives its users (CON50).** Good: every mutex is static/file-scope or owned by a long-lived object whose lifetime strictly contains all contexts that can lock it. *"If a mutex object is destroyed while a thread is blocked waiting for the lock, critical sections and shared data are no longer protected."* The tell: a stack-local mutex in a function that spawns a worker / registers an ISR and returns, or a mutex inside an object torn down during shutdown while the other core still runs — UB that compiles and usually "works" on the happy path.

- **Bit-fields and adjacent members across contexts (CON52).** Good: when distinct bit-fields (or small adjacent members) are written by different contexts, all writers hold a common lock, or the fields are split into independently-addressable atomic objects. The tell: two contexts each updating "their own" bit-field of the same struct without synchronization — *"when accessing a bit-field, a thread may inadvertently access a separate bit-field in adjacent memory,"* so the compiler's read-modify-write of the shared storage unit clobbers one update; *"difficult to detect because the shared memory access isn't obvious from code inspection alone."*

- **Condition-variable predicate loops and notify discipline (CON54 / CON55).** Good: every `wait()`/`wait_for()`/`wait_until()` re-checks its predicate in a loop (explicit `while` or the predicate-lambda overload) while holding the associated mutex; where several threads wait on one CV for distinct conditions, the code uses `notify_all()` rather than `notify_one()`. The tell: a bare `cv.wait(lock);` (or `if` instead of `while`) with no predicate re-check, so a spurious/stale wakeup lets the consumer run on data that isn't ready; or `notify_one()` on a CV with multiple waiters of different predicates, so the wakeup reaches a thread whose predicate is false while the one that could proceed stays asleep (a lost-wakeup/liveness failure).

### ▸ How an AI agent gets this wrong

- Reaches for a shared mutable global or capture-by-reference to "pass data" between producer and consumer — the shortest code that compiles and passes single-threaded tests — without reasoning about the two contexts, so it neither minimizes the shared surface nor marks it const (CP.3).
- Confidently writes `volatile bool data_ready` for a Core1→Core0 handshake, because LLM training data is full of pre-C++11/embedded idioms where `volatile` is the "shared-flag" tool; it compiles, lints silent, single-threaded tests pass, and there is zero synchronization on two cores (CP.8).
- Models each named bit-field as an independent variable and writes them from different contexts without a lock, with no notion that the hardware writes the whole byte/word (CON52).
- Writes symmetric `lock(); ...; unlock();`, then later adds an early-return guard clause inside the critical section without connecting that the early return bypasses the trailing unlock (CP.20). The error-path variant: guards the normal path correctly but, when adding error handling, returns an error status from the middle of a hand-locked section without unlocking (CON51).
- Edits two functions in separate turns and locks the mutexes in whatever order is locally convenient in each, never seeing both paths together — an order cycle invisible to single-threaded tests and linters (CP.21 / CON53).
- Keeps the lock guard at function scope and does ALL the work — including invoking a user callback or posting a QP event — inside the guarded scope, because the callback "is just another call" (CP.22). Or factors a guarded operation into a helper that locks, then calls that helper from another already-locked region, because each function "should protect its own data" — self-deadlock (CON56).
- Declares a mutex "where it's used" (smallest local scope) without reasoning about the lifetime of the OTHER context that captures it; with workers/ISRs that outlive the spawning function, the stack-local mutex is destroyed too early (CON50).
- Writes the intuitive "wait, then use the data" with a single `wait()` and no loop because the recheck looks redundant and the one-producer/one-consumer test passes (CON54); and picks `notify_one()` as the "efficient" default across waiters with differing conditions (CON55).

### Judging criteria

| Situation | Verdict |
|---|---|
| Cross-core shared data, single owner, proper barrier (spinlock/atomic/FIFO/double-buffer) | **PASS** |
| Shared mutable state via plain `volatile` or plain global, no barrier (CP.8) | **FAIL** |
| `sleep_ms`/`busy_wait` used to synchronize with another thread/core (JPL 7) | **FAIL** |
| Single-owner object mutated from a non-owner core (JPL 8) | **FAIL** |
| `lock` and `unlock` in different functions, or an exit path skips unlock (JPL 9 / CON51) | **PARTIAL** — pair them in one function / make every exit release |
| Two locks acquired in opposite orders on different paths (CP.21 / CON53) | **FAIL** |
| Callback / event post / unknown call made while a lock or critical section is held (CP.22 / CON56) | **FAIL** |

### Findings

| File:line | Shared object | Owner | Barrier | Mutated by | Verdict | Disposition |
|---|---|---|---|---|---|---|
| | | | | | | |

_Full sourced criteria + IDs: L2P5_RP_SOURCES_2026-06-25.md Support D._

---

## Source docs

The review criteria in this guide rest on three primary published safety-critical C/C++ coding standards — **all three are freely available** at the URLs below. The verbatim rule statements quoted in the Class sections were extracted and checked directly from these PDFs (2026-06-27); the C++ Core Guidelines / SEI CERT / Fowler / NIST / OWASP review criteria are verified in the resources doc. Open a standard for fuller context or surrounding rules; the per-rule "why it lands where it does" lives in the triage (see pointers below).

- **JSF AV C++ (Rev C)** — the adopted C++ source: https://www.stroustrup.com/JSF-AV-rules.pdf
- **The Power of Ten** — Holzmann, *Rules for Developing Safety Critical Code*: https://spinroot.com/gerard/pdf/P10.pdf
- **JPL-C** — JPL Institutional Coding Standard for C (D-60411): https://yurichev.com/mirrors/C/JPL_Coding_Standard_C.pdf

**Pointers (this section does not duplicate them):**

- **Full verified corpus + provenance** (every cited ID/title/quote adversarially verified against a primary source; UNVERIFIED items quarantined): `docs/audits/L2P5_RP_SOURCES_2026-06-25.md`.
- **Exact rule wording — first stop for lookups** (primary-source-verified where a primary was reachable): `docs/audits/RULE_VERIFIABILITY_TRIAGE.md` — §2 (P10), §3 (JPL), §4a–d (JSF), §7 (findings).
- **Route — which files each lens applies to:** `docs/audits/L2P5_WALK_ITINERARY.md`.
- **Work plan — prep / gate-wiring / sequencing:** `docs/plans/L2P5_WALK_PLAN.md`.
