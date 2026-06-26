# L2-P5 §RP — Research Sources & Judging Criteria (Phase D deliverable)

**Status:** STAGING ARTIFACT — the sources stash + verified criteria that feed `L2P5_MANUAL_WALK_GUIDE.md`. NOT the field manual itself. Per `docs/plans/L2P5_WALK_PLAN.md` Phase D.
**Date:** 2026-06-25.
**Owner:** Nathan. Agents produced the research; dispositions + the fold into the field manual are Nathan's.
**Discipline:** Primary-source-verified per **Lessons-Learned Entry 37** (anti-fabrication). Every guideline ID/title/stat was adversarially verified against a primary source; anything unconfirmable is quarantined in the Appendix as UNVERIFIED, never asserted. A WebFetch summarizer has fabricated plausible quotes on this project before — so quotes are verbatim-from-primary only.

## How this was produced

Four multi-agent **Workflow** passes (fan-out research → adversarial verify → synthesis), ~48 agents total:

1. **Spine + rule-class criteria** (9 facets) — gestalt/function-shape + Manual rule-classes (3,7,8,10 + residuals 9/13/14) from C++ Core Guidelines / SEI CERT / Fowler / Holzmann.
2. **AI-code-review (general)** — authoritative + peer-reviewed sources on reviewing AI/LLM-generated code (SEI/NIST/OWASP + empirical studies).
3. **Agentic-era refresh (2024-2026)** — current frontier/agentic-system evidence; **error TYPES are durable, FREQUENCIES are volatile**; every rate carries a `[model, year]` stamp.
4. **Embedded applicability** — what to ADD (bare-metal-specific agent failure modes) vs DROP/recast (web-centric findings that don't transfer to RP2350 firmware).

**The spine is the point** (the gestalt human-eye review of what AI agents wave through that compiles/passes/tests-clean); the rule-classes are appendix/backing. See the field manual's top lens.

---

## Consolidated source corpus

🔒 = paywalled (IDs/titles used only, no body text quoted). Domains: `canon` (coding-standard primary), `ai-general`, `ai-agentic`, `embedded`.

| Tier | Yr | Domain | Title | Venue | Model/scope |
|---|---|---|---|---|---|
| peer-reviewed | 2026 | ai-agentic | Why Are Agentic Pull Requests Merged or Rejected? An Empirical Study | MSR 2026 | Agentic PRs from OpenAI Codex, Devin, GitHub C |
| peer-reviewed | 2026 | ai-agentic | From Industry Claims to Empirical Reality: An Empirical Study of Code Review Agents in Pull Requests | MSR 2026 | 13 code-review agents incl. GitHub Copilot, Co |
| peer-reviewed | 2026 | ai-agentic | When AI Teammates Meet Code Review: Collaboration Signals Shaping the Integration of Agent-Authored Pull Requests | MSR 2026 | Autonomous coding agents' PRs (AIDev dataset;  |
| peer-reviewed | 2026 | ai-agentic | Evaluating Plan Compliance in Autonomous Programming Agents | arXiv (UIUC/IBM) | GPT-5 mini, DeepSeek-V3, DeepSeek-R1, Devstral |
| peer-reviewed | 2026 | ai-agentic | Finding Widespread Cheating on Popular Agent Benchmarks | DebugML (UPenn) | Claude Opus 4.6, Claude 3.7 Sonnet, GPT-5.4, Q |
| peer-reviewed | 2026 | ai-agentic | Measuring and Exploiting Contextual Bias in LLM-Assisted Security Code Review | arXiv (AUEB) | GPT-4o-mini, Claude 3.5 Haiku, Claude Sonnet 4 |
| peer-reviewed | 2026 | ai-agentic | The Range Shrinks, the Threat Remains: Re-evaluating LLM Package Hallucinations on the 2026 Frontier-Model Cohort | arXiv (independent) | Claude Sonnet 4.6, Claude Haiku 4.5 (4.62%), G |
| peer-reviewed | 2026 | ai-agentic | Why LLMs Fail: A Failure Analysis and Partial Success Measurement for Automated Security Patch Generation | arXiv (Univ. Passau) | Gemini 2.0 Flash (319 patches, 64 Vul4J Java v |
| peer-reviewed | 2026 | ai-agentic | Debt Behind the AI Boom: A Large-Scale Empirical Study of AI-Generated Code in the Wild | arXiv (SMU) | 304,362 AI-authored commits (Jan 2024-Oct 2025 |
| peer-reviewed | 2025 | ai-general | Security Weaknesses of Copilot-Generated Code in GitHub Projects: An Empirical Study | ACM Transactions on Software Engineering and Methodology (TO |  |
| peer-reviewed | 2025 | ai-general | We Have a Package for You! A Comprehensive Analysis of Package Hallucinations by Code Generating LLMs | USENIX Security 2025 (34th USENIX Security Symposium) |  |
| peer-reviewed | 2025 | ai-agentic | The Illusion of Diminishing Returns: Measuring Long Horizon Execution in LLMs | ICLR 2026 | Frontier + smaller LLMs incl. thinking/non-thi |
| peer-reviewed | 2025 | ai-agentic | Uncovering Systematic Failures of LLMs in Verifying Code Against Natural Language Specifications | ASE 2025 | GPT-4o, Claude-3.5-Sonnet, Gemini-2.0-Flash |
| peer-reviewed | 2025 | ai-agentic | CWEval: Outcome-driven Evaluation on Functionality and Security of LLM Code Generation | IEEE/ACM LLM4Code 2025 | GPT-4o-2024-08-06, GPT-4o mini, Claude 3.5 Son |
| peer-reviewed | 2025 | ai-agentic | Security Degradation in Iterative AI Code Generation: A Systematic Analysis of the Paradox | IEEE-ISTAS 2025 | GPT-4o (C and Java) |
| peer-reviewed | 2025 | ai-agentic | Are 'Solved Issues' in SWE-bench Really Solved Correctly? An Empirical Study | ACM proceedings (FSE 2026 likely) | CodeStory, LearnByInteract, OpenHands+CodeAct  |
| peer-reviewed | 2025 | ai-agentic | Threats to scientific software from over-reliance on AI code assistants 🔒 | Nature Computational Science | AI code assistants broadly (general, not a sin |
| peer-reviewed | 2025 | ai-agentic | Measuring the Impact of Early-2025 AI on Experienced Open-Source Developer Productivity | METR (arXiv 2507.09089) | Cursor Pro + Claude 3.5/3.7 Sonnet (early 2025 |
| peer-reviewed | 2025 | ai-agentic | SWE-Bench Pro: Can AI Agents Solve Long-Horizon Software Engineering Tasks? | arXiv (Scale AI) | GPT-5, Claude Opus 4.1, Claude Sonnet 4.5/4, C |
| peer-reviewed | 2025 | ai-agentic | UTBoost: Rigorous Evaluation of Coding Agents on SWE-Bench | ACL 2025 | ~44-45 SWE-bench leaderboard agents (Dec 2024  |
| peer-reviewed | 2025 | ai-agentic | Why Do Multi-Agent LLM Systems Fail? (MAST taxonomy) | arXiv (UC Berkeley) | GPT-4o, GPT-4o-mini, Claude-3.7-Sonnet, Qwen2. |
| peer-reviewed | 2025 | ai-agentic | Is Vibe Coding Safe? Benchmarking Vulnerability of Agent-Generated Code in Real-World Tasks (SusVibes) | arXiv | Claude 4 Sonnet, Kimi K2, Gemini 2.5 Pro (+Gem |
| peer-reviewed | 2025 | ai-agentic | Security and Quality in LLM-Generated Code: A Multi-Language, Multi-Model Analysis | arXiv (UCF) | GPT-4o, Claude-3.5, Gemini-1.5, Codestral, Lla |
| peer-reviewed | 2025 | ai-agentic | Do LLMs Consider Security? An Empirical Study on Responses to Programming Questions | arXiv | GPT-4 Turbo, Claude 3 Opus, Llama 3 (70B) |
| peer-reviewed | 2025 | ai-agentic | An Empirical Study on Failures in Automated Issue Solving | arXiv (Beihang) | OpenHands-CodeAct v2.1, Tools+Claude 3.5 Sonne |
| peer-reviewed | 2025 | ai-agentic | 2025 GenAI Code Security Report | Veracode | 100+ LLMs across 80+ coding tasks (Java/Python |
| peer-reviewed | 2025 | ai-agentic | Quality Assurance of LLM-generated Code: Addressing Non-Functional Quality Characteristics | arXiv | SLR over 2022+ literature; empirical on Claude |
| peer-reviewed | 2025 | ai-agentic | A Survey on Code Generation with LLM-based Agents | arXiv (Peking Univ.) | Agentic systems on SWE-bench (SWE-agent, MAGIS |
| peer-reviewed | 2025 | ai-agentic | A Systematic Literature Review of Code Hallucinations in LLMs: Characterization, Mitigation Methods, Challenges, and Fut | arXiv | SLR over contemporary code LLMs (search throug |
| peer-reviewed | 2024 | ai-general | A systematic literature review on the impact of AI models on the security of code generation | Frontiers in Big Data |  |
| peer-reviewed | 2023 | ai-general | Is Your Code Generated by ChatGPT Really Correct? Rigorous Evaluation of Large Language Models for Code Generation (Eval | NeurIPS 2023 (Advances in Neural Information Processing Syst |  |
| peer-reviewed | 2023 | ai-general | Do Users Write More Insecure Code with AI Assistants? | ACM CCS 2023 (SIGSAC Conference on Computer and Communicatio |  |
| peer-reviewed | 2023 | ai-general | GitHub Copilot AI pair programmer: Asset or Liability? 🔒 | Journal of Systems and Software (Elsevier), vol. 203, 111734 |  |
| peer-reviewed | 2023 | ai-general | How Secure is Code Generated by ChatGPT? | IEEE International Conference on Systems, Man, and Cyberneti |  |
| peer-reviewed | 2022 | ai-general | Asleep at the Keyboard? Assessing the Security of GitHub Copilot's Code Contributions | IEEE Symposium on Security and Privacy (S&P) 2022 |  |
| peer-reviewed | 2022 | ai-general | An Empirical Evaluation of GitHub Copilot's Code Suggestions | MSR 2022 (Mining Software Repositories), ACM, DOI 10.1145/35 |  |
| peer-reviewed |  | canon | C++ Core Guidelines — F.1/F.2/F.3 (F: Functions and arguments) | https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines |  |
| peer-reviewed |  | canon | CppCoreGuidelines.md (raw master) — verbatim source text | https://raw.githubusercontent.com/isocpp/CppCoreGuidelines/m |  |
| peer-reviewed |  | canon | C++ Core Guidelines (rendered single page) | https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines |  |
| peer-reviewed |  | canon | Extract Function (alias Extract Method) — refactoring catalog | https://refactoring.com/catalog/extractFunction.html |  |
| peer-reviewed |  | canon | Decompose Conditional — refactoring catalog | https://refactoring.com/catalog/decomposeConditional.html |  |
| peer-reviewed |  | canon | CodeSmell — definition of a code smell | https://martinfowler.com/bliki/CodeSmell.html |  |
| peer-reviewed |  | canon | The Power of Ten — Rules for Developing Safety Critical Code | https://spinroot.com/gerard/pdf/P10.pdf |  |
| peer-reviewed |  | canon | Bad Smells in Code (Refactoring, ch.3) — smells PDF | http://www.laputan.org/pub/patterns/fowler/smells.pdf |  |
| peer-reviewed |  | canon | Refactoring: Improving the Design of Existing Code (2nd ed.) — 'Bad Smells in Code' chapter 🔒 | https://martinfowler.com/books/refactoring.html |  |
| peer-reviewed |  | canon | SEI CERT C++ — EXP53-CPP. Do not read uninitialized memory | https://cmu-sei.github.io/secure-coding-standards/sei-cert-c |  |
| peer-reviewed |  | canon | SEI CERT C++ — Rule 08: Exceptions and Error Handling (ERR) index | https://cmu-sei.github.io/secure-coding-standards/sei-cert-c |  |
| peer-reviewed |  | canon | SEI CERT C — MSC04-C. Use comments consistently and in a readable fashion | https://cmu-sei.github.io/secure-coding-standards/sei-cert-c |  |
| peer-reviewed |  | canon | SEI CERT C — MSC12-C. Detect and remove code that has no effect or is never executed | https://cmu-sei.github.io/secure-coding-standards/sei-cert-c |  |
| peer-reviewed |  | canon | SEI CERT C++ — Rule 09: Object Oriented Programming (OOP) index | https://cmu-sei.github.io/secure-coding-standards/sei-cert-c |  |
| peer-reviewed |  | canon | SEI CERT C++ — OOP52-CPP. Do not delete a polymorphic object without a virtual destructor | https://cmu-sei.github.io/secure-coding-standards/sei-cert-c |  |
| peer-reviewed |  | canon | SEI CERT C++ — OOP50-CPP. Do not invoke virtual functions from constructors or destructors | https://cmu-sei.github.io/secure-coding-standards/sei-cert-c |  |
| peer-reviewed |  | canon | SEI CERT C++ — OOP58-CPP. Copy operations must not mutate the source object | https://cmu-sei.github.io/secure-coding-standards/sei-cert-c |  |
| peer-reviewed |  | canon | SEI CERT C++ — Concurrency (CON) rule index | https://cmu-sei.github.io/secure-coding-standards/sei-cert-c |  |
| peer-reviewed |  | canon | SEI CERT C++ — CON50-CPP. Do not destroy a mutex while it is locked | https://cmu-sei.github.io/secure-coding-standards/sei-cert-c |  |
| peer-reviewed |  | canon | SEI CERT C++ — CON51-CPP. Ensure actively held locks are released on exceptional conditions | https://cmu-sei.github.io/secure-coding-standards/sei-cert-c |  |
| peer-reviewed |  | canon | SEI CERT C++ — CON52-CPP. Prevent data races when accessing bit-fields from multiple threads | https://cmu-sei.github.io/secure-coding-standards/sei-cert-c |  |
| peer-reviewed |  | canon | SEI CERT C++ — CON53-CPP. Avoid deadlock by locking in a predefined order | https://cmu-sei.github.io/secure-coding-standards/sei-cert-c |  |
| peer-reviewed |  | canon | SEI CERT C++ — CON54-CPP. Wrap functions that can spuriously wake up in a loop | https://cmu-sei.github.io/secure-coding-standards/sei-cert-c |  |
| peer-reviewed |  | canon | SEI CERT C++ — CON55-CPP. Preserve thread safety and liveness when using condition variables | https://cmu-sei.github.io/secure-coding-standards/sei-cert-c |  |
| peer-reviewed |  | canon | SEI CERT C++ — CON56-CPP. Do not speculatively lock a non-recursive mutex already owned by the calling thread | https://cmu-sei.github.io/secure-coding-standards/sei-cert-c |  |
| peer-reviewed |  | canon | SEI CERT C++ — Expressions (EXP) rule index | https://cmu-sei.github.io/secure-coding-standards/sei-cert-c |  |
| peer-reviewed |  | canon | SEI CERT C++ — EXP50-CPP. Do not depend on the order of evaluation for side effects | https://cmu-sei.github.io/secure-coding-standards/sei-cert-c |  |
| peer-reviewed |  | canon | SEI CERT C++ — EXP52-CPP. Do not rely on side effects in unevaluated operands | https://cmu-sei.github.io/secure-coding-standards/sei-cert-c |  |
| peer-reviewed |  | canon | SEI CERT C++ — Integers (INT) rule index | https://cmu-sei.github.io/secure-coding-standards/sei-cert-c |  |
| peer-reviewed |  | canon | refactoring.com catalog (smell-name lookups) | https://refactoring.com/catalog/ |  |
| standards-body | 2025 | ai-agentic | OWASP Top 10 for LLM Applications 2025 | OWASP | Model-agnostic standard |
| standards-body | 2024 | ai-general | NIST AI 600-1: Artificial Intelligence Risk Management Framework — Generative AI Profile | NIST, U.S. Dept. of Commerce |  |
| standards-body | 2024 | ai-agentic | NIST AI 600-1: AI RMF Generative AI Profile | NIST | Model-agnostic standard (defines Confabulation |
| reputable-org | 2025 | ai-general | OWASP Top 10 for LLM Applications 2025 — LLM05:2025 Improper Output Handling | OWASP Gen AI Security Project (OWASP Foundation) |  |
| reputable-org | 2025 | ai-general | OWASP Top 10 for LLM Applications 2025 — LLM09:2025 Misinformation (incl. Overreliance) | OWASP Gen AI Security Project (OWASP Foundation) |  |
| reputable-org | 2025 | ai-general | Review AI-generated code (GitHub Copilot tutorial) | GitHub Docs (GitHub, Inc. / Microsoft) |  |
| reputable-org | 2025 | ai-general | Responsible use of GitHub Copilot code review | GitHub Docs (GitHub, Inc. / Microsoft) |  |
| reputable-org | 2025 | ai-general | Perspectives on Generative AI in Software Engineering and Acquisition | Software Engineering Institute (SEI), Carnegie Mellon Univer |  |
| reputable-org | 2024 | ai-general | On Mitigating Code LLM Hallucinations with API Documentation (CloudAPIBench) | arXiv preprint (NOT peer-reviewed); primary observational st |  |
| reputable-org | 2024 | ai-general | LLM Hallucinations in Practical Code Generation: Phenomena, Mechanism, and Mitigation | arXiv preprint (NOT peer-reviewed); primary taxonomy study |  |
| reputable-org |  | canon | CppCoreGuidelines nl-naming-and-layout-rules.md (GitHub mirror) | https://github.com/Trree/CppCoreGuidelines/blob/master/nl-na |  |
| reputable-org |  | canon | isocpp/CppCoreGuidelines Issue #507 — C.131 title | https://github.com/isocpp/CppCoreGuidelines/issues/507 |  |
| reputable-org |  | canon | C++ Core Guidelines community mirror — class / interfaces pages | https://cpp-core-guidelines-docs.vercel.app/class |  |
| reputable-org |  | canon | C++ Core Guidelines docs mirror — ES / CP pages | https://cpp-core-guidelines-docs.vercel.app/expr |  |
| reputable-org |  | canon | refactoring.guru — Speculative Generality | https://refactoring.guru/smells/speculative-generality |  |
| reputable-org |  | canon | modernescpp — C++ Core Guidelines: statements and arithmetic rules | https://www.modernescpp.com/index.php/c-core-guidelines-rule |  |
| preprint/other | 2026 | ai-general | These Aren't the Reviews You're Looking For: How Humans Review AI-Generated Pull Requests | arXiv preprint (NOT peer-reviewed; submitted to EASE 2026) |  |
| preprint/other | 2026 | ai-general | From Vulnerabilities to Remediation: A Systematic Literature Review of LLMs in Code Security | arXiv preprint (NOT peer-reviewed; SLR) |  |
| preprint/other | 2026 | embedded | EmbedAgent: Benchmarking Large Language Models in Embedded System Development | ICSE 2026 (IEEE/ACM Int. Conf. on Software Engineering) |  |
| preprint/other | 2026 | embedded | InCoder-32B: Code Foundation Model for Industrial Scenarios (introduces EmbedCGen bare-metal benchmark) | arXiv preprint |  |
| preprint/other | 2026 | embedded | HardSecBench: Benchmarking the Security Awareness of LLMs for Hardware Code Generation | arXiv preprint |  |
| preprint/other | 2026 | embedded | H2LooP Spark Preview: Continual Pretraining of Large Language Models for Low-Level Embedded Systems Code | arXiv preprint |  |
| preprint/other | 2026 | embedded | Benchmarking Large Language Models for Embedded Systems Programming in Microcontroller-Driven IoT Applications (ESP32) | Future Internet (MDPI), peer-reviewed — UNVERIFIED, page HTT |  |
| preprint/other | 2025 | ai-general | LLM-CSEC: Empirical Evaluation of Security in C/C++ Code Generated by Large Language Models | arXiv preprint (NOT peer-reviewed) |  |
| preprint/other | 2025 | general | Assessing Large Language Models in Comprehending and Verifying Concurrent Programs across Memory Models | arXiv preprint |  |
| preprint/other | 2025 | embedded | AI Code Generation in Embedded Systems: Constraints and Solutions | GoCodeo (vendor blog — lowest source tier; corroborated by a |  |
| preprint/other | 2025 | general | The Rise of Slopsquatting: How AI Hallucinations Are Fueling a New Class of Supply Chain Attacks | Socket.dev (industry security blog) |  |
| preprint/other | 2024 | embedded | Weaknesses in LLM-Generated Code for Embedded Systems Networking | IEEE QRS 2024 (peer-reviewed; full text via open conference  |  |
| preprint/other | 2024/2025 | embedded | Securing LLM-Generated Embedded Firmware through AI Agent-Driven Validation and Patching | arXiv preprint |  |

---


---

# Pass 1 — Spine (gestalt + function-shape) + Supporting rule-class criteria

# L2-P5 RP / Phase D — Sources & Criteria DRAFT

**Target file:** `docs/audits/L2P5_RP_SOURCES_2026-06-25.md`
**Status:** DRAFT staging artifact. This is a curated set of *candidate* review criteria plus their sources, assembled from per-facet research each paired with an adversarial verification verdict. **It is NOT yet folded into the field manual.** A human reviewer must walk this, approve the criteria, and clear the UNVERIFIED lists and the AI-code-review sources before anything here becomes a manual rule.

**Inclusion rule applied:** A guideline appears as an active criterion **only if** its verification verdict marked it `verbatim_confirmed`. Every other ID/claim is preserved in a per-facet "UNVERIFIED — do not cite until checked" list (never silently dropped, never presented as fact).

**Criterion line shape:** `ID: Title` → `good:` bullet → `finding:` bullet → AI-agent tendency.

---

## PART 1 — SPINE FACETS (holistic human-eye core)

---

### Spine 1 — Function decomposition & single-responsibility (does one thing)

The reviewer reads the *whole* function and asks whether it performs one logical operation. The decisive walk-test is naming: try to name the function in one "and"-free verb phrase. If you cannot, or the honest name needs "and", it is doing more than one thing — even when every individual line is correct and the JSF size gate is green.

**Criteria (verbatim-confirmed only):**

- **CppCoreGuidelines-F.1: "Package" meaningful operations as carefully named functions**
  - good: Every well-specified action is factored out of its surrounding code and given a name; non-trivial repeated logic (including a lambda used in more than one place) is named rather than copy-pasted inline.
  - finding: An identifiable, named-able action is left inline and unnamed, or duplicated; the same several-line idiom appears twice. Per the verbatim Enforcement, "Flag identical and very similar lambdas used in different places."
  - AI-agent tendency: LLM code writes the same multi-line idiom (a CRC step, a register-poke sequence, a queue-publish dance) inline at each call site instead of extracting it once, and drops non-trivial lambdas copy-pasted across call sites — each instance locally correct, the duplication drifting out of sync later.

- **CppCoreGuidelines-F.2: A function should perform a single logical operation**
  - good: Walking the whole function, you can state what it does in one short verb phrase with NO "and", and that phrase covers every line; inputs arrive as parameters and the result leaves as a return value, so the operation is reusable at a second call site.
  - finding: The function performs two or more logical operations that merely sit next to each other (reads input AND formats output AND writes errors AND mutates state). Note the JSF size gate can be GREEN here: the function is short yet still does two things — precisely the ungated "one logical operation" judgment.
  - AI-agent tendency: LLM code is generated in prompt order, so one function inlines setup + input read + core computation + side effects (logging, USB write, shared-state mutation) + cleanup as one straight-line body. It compiles, passes tests built against that same shape, trips no linter. Forces a name like `read_and_print` / `init_and_publish`, or a vague `handle()`/`process()`/`run()`. On the RP2350 split, watch for a Core-1 sensor-loop function that samples AND filters AND packs AND signals Core 0 in one body.

- **CppCoreGuidelines-F.3: Keep functions short and simple**
  - good: The function fits on one screen and has few logical paths; complex sub-steps are pulled out into smaller cohesive, well-named functions.
  - finding: Control flow is deep or branchy enough that you cannot convince yourself every path is handled — CCG's own challenge: "How would you know if all possible alternatives have been correctly handled?" The length/branching may be concealing an extractable operation, not just verbose code. (The line-count side is what the house JSF gate catches mechanically; F.3's value-add for the walker is the complexity/path judgment, which is ungated.)
  - AI-agent tendency: Agents avoid the raw line-count gate by writing DENSE rather than long — deeply nested if/else-if and switch ladders over flag parameters packed well under the size limit, so the size linter is green while cyclomatic complexity is high and several alternatives are silently unhandled. On bare-metal dual-core code this shows up as one ISR/loop function carrying mode flags for every operating state.

- **Fowler-ExtractFunction: Extract Function (alias Extract Method)**
  - good: The canonical remedy for the Long Function smell — extractable parts are pulled into named helpers; a block of code with a comment explaining what it does is replaced by a function named after the comment.
  - finding: A function long enough that comments are needed to explain blocks of it — Fowler's signal that those blocks are separate operations awaiting Extract Function. This is the refactoring-canon counterpart to F.2/F.3.
  - AI-agent tendency: The common shape is a long, comment-segmented function (`// validate`, `// compute`, `// publish`) where each comment literally labels a distinct logical operation the model declined to extract. Read the comments as a table of contents and ask whether each section should be its own named function.

- **Fowler-CodeSmell-definition: Code Smell — "a surface indication that usually corresponds to a deeper problem in the system"**
  - good: The reviewer treats a smell as a surface signal to investigate, not an automatic defect; the absence of the Long Function smell looks like short functions whose extractable parts are already named helpers.
  - finding: A surface indication (length, comment-segmentation, duplication) is present and points at a deeper structural problem the per-line pass would miss.
  - AI-agent tendency: Per-line-correct, test-passing AI output routinely carries surface smells (comment-segmented length, duplicated idioms) whose deeper problem is structural and invisible line-by-line.

**UNVERIFIED — do not cite until checked (fn-decomposition):**
- `Fowler-LongFunction` — the smell NAME "Long Function" is a citable lead, but the parenthetical "formerly Long Method" *rename* could not be confirmed on a free primary page (extractFunction.html returns "aliases Extract Method", an alias, not a "formerly called" rename).
- F.2 Reason quote: "A function that performs a single operation is simpler to understand, test, and reuse." — TITLE confirmed; this body sentence not confirmed verbatim (summarizer returned three contradictory readings).
- F.2 Enforcement quotes: "Consider functions with more than one 'out' parameter suspicious" and "Consider functions with 7 or more parameters suspicious" — not confirmed verbatim; the "7 or more" threshold could not be located.
- F.3 Enforcement thresholds: "60 lines by 140 characters" and "more than 10 logical paths through" / "Count a simple switch as one path" — could not be confirmed from a primary source.
- F.3 Note: "One-to-five-line functions should be considered normal." — not confirmed verbatim.
- F.1 Enforcement: "Flag identical and very similar lambdas used in different places." — TITLE confirmed; this body quote marked unverified out of caution (summarizer self-contradicted), though likely genuine. *(NOTE: the same string is treated as verbatim_confirmed for F.1 in the verdict's verified list; flagged here as a residual discrepancy for the human to resolve.)*
- `Decompose Conditional` (refactoring.com/catalog/decomposeConditional.html) — mentioned in findings; its catalog page was not independently fetched (returned unreachable this pass).

---

### Spine 2 — Function layout, shape & abstraction-level (altitude)

Read the function (and its enclosing file/class) as a SHAPE: does the body read top-down from intent to detail, stay at ONE level of abstraction, declare names near first use, keep scopes tight, and let the visual/paragraph/brace structure itself state intent — so a reviewer recovers WHAT the code is for from its shape, not from comments or line-by-line mental execution. This spine is distinct from Spine-1 (decomposition / does-one-thing) and Spine-3 (duplication / code-smells); where a rule overlaps (F.2, F.3, F.56, Extract Function), it is framed here on the altitude/layout reading only.

Each criterion: **ID: Title** -> good -> finding -> AI-agent tendency.

---

**CCG-P.1: Express ideas directly in code**
- good: Each layer expresses its idea with a construct at its own level — a range-for says "visit every element," a named algorithm says "find/sort/transform," a typed `Point` says "this pair is a coordinate." The layout itself, not an adjacent comment, carries the meaning, so intent is readable at the top and detail only appears when you step down into a callee.
- finding: Intent encoded indirectly — a hand-rolled index loop where a named algorithm was meant, a raw int pair standing in for a coordinate, an inline bit-loop for an operation that has a name. Correct, but the reader must reconstruct the idea instead of reading it, and an open-coded fragment drags the surrounding body down to its altitude.
- AI-agent tendency: A line-by-line reviewer confirms each statement is individually valid and the loop bounds are correct, then approves. "Could be expressed more directly at this altitude" is a design judgment, not a line defect, so the agent has no trigger to raise it; correct-but-low-altitude code is exactly what it waves through.

**CCG-P.3: Express intent**
- good: Say WHAT should be done, not just how. The top of the function names the operation; iteration mechanism, loop-variable scope, `const`-ness, and argument meaning are visible in the code (named types where bare ints would be ambiguous), so a reader can judge whether the body does what it is supposed to from shape and names alone.
- finding: The "how" is exposed but the "what" is missing — a `while (i < v.size())` exposing the index mechanism, a `draw_line(int,int,int,int)` where you cannot tell `(x1,y1,x2,y2)` from `(x,y,h,w)`, a mutable variable where `const` was meant. Whether the code is correct becomes undecidable from the code alone; on a hard-real-time AO handler this hides whether a block is setup, a guarded critical section, or teardown.
- AI-agent tendency: The agent verifies the loop terminates and indices stay in bounds and passes it. With no "supposed-to" stated it cannot judge "does this do what it's supposed to," so it defaults to approval; it treats an accompanying comment as sufficient intent-expression rather than flagging that the structure fails to convey purpose.

**CCG-F.2: A function should perform a single logical operation** (altitude reading)
- good: The body operates at ONE logical level — `read_and_print()` reads as `auto x = read(cin); print(cout, x);`, orchestration only, with the mechanics pushed down into the callees. Even when decomposition is fine, the body does not juxtapose a high-level call with raw bit-twiddling; it is a sequence of peer-level named steps.
- finding: A body that interleaves altitudes — one line dispatches a QP event or calls a service, the next manually clears an interrupt flag with a literal bitmask (`dispatch(evt); reg |= (1u << pin); recompute_crc();`). It performs one logical operation but at jarringly mixed altitudes, so the body reads as two functions fused and no longer reads as a coherent shape.
- AI-agent tendency: Line-by-line review validates the high-level call and the low-level poke independently, finds both correct, and never notices the abstraction-level seam. The agent has no built-in "are these statements at the same altitude?" check; uniform correctness masks non-uniform abstraction, and it may even praise the inlined detail as "efficient."

**CCG-F.3: Keep functions short and simple** (altitude reading)
- good: The function fits on one editor screen and reads as a short paragraph of same-altitude steps; complex control structure has been factored into well-named suboperations so the top-level body states the algorithm, not its mechanics. Concrete bars: one-to-five-line functions are normal; be suspicious past ~60 lines or ~10 logical paths.
- finding: A long, deeply-nested handler that is fully correct but whose shape can't be taken in at once — nested ifs, flag arithmetic and a switch where you cannot tell "if all alternatives are handled." The reader must scroll and track nesting to follow intent; particularly dangerous in ISR/AO code where the whole control flow must be reasoned about at once.
- AI-agent tendency: The agent reads a long function top to bottom, finds no single broken line, and approves — length and nesting depth are not correctness signals and it has no screenful or path-count threshold. It doesn't weigh "can a human hold this shape in their head" as a review criterion.

**CCG-F.56: Avoid unnecessary condition nesting**
- good: Guard clauses handle exceptional cases up front and return early, placing the essential code at the outermost scope. The shallow shape makes the main intent the visually dominant path; the guideline states shallow nesting "makes the intent clearer."
- finding: The real work is buried inside nested ifs (or behind a redundant else) so the body is "simply a conditional statement enclosing a block." The essential logic is no longer at the altitude the eye lands on; intent is recoverable only by tracing indentation inward.
- AI-agent tendency: Nesting is behavior-preserving, so the agent never flags it — the code produces identical results. It does not apply the guideline's own enforcement (flag a redundant else; flag a body that is just a conditional wrapping a block), so deep arrow-shaped code passes review.

**CCG-ES.5: Keep scopes small**
- good: Variables, helper temporaries and resources live in the smallest enclosing block, so the function's shape is a series of tight scopes whose boundaries visually delimit each phase of the operation.
- finding: Names held at full-function scope when they're only needed inside one branch — common when a body was edited incrementally. Correct, but the wide scope flattens the visual structure, hides which step owns which state, and on no-heap embedded code obscures resource lifetimes.
- AI-agent tendency: The agent verifies the wide-scope variable is used correctly everywhere it's visible and stops; it doesn't propose tightening because nothing is broken. Scope-narrowing is a structural improvement, not a bug fix, so it falls outside a correctness-first review.

**CCG-ES.6: Declare names in for-statement initializers and conditions to limit scope**
- good: Loop indices and condition-only names are declared inside the for/if-init so they don't leak past the construct, keeping the body's scope structure aligned with its control structure.
- finding: A loop counter or temporary declared above the loop and left dangling in scope afterward. It runs fine, but the lingering name muddies the layout and can be silently reused by a later block — a structural smell, not a compile error.
- AI-agent tendency: The agent accepts any in-scope, correctly-used name and doesn't treat leaked loop variables as worth flagging since behavior is unchanged. The C++17 if/switch-init and for-init scoping idiom is a layout refinement it rarely suggests unprompted.

**CCG-ES.21: Don't introduce a variable (or constant) before you need to use it**
- good: Each declaration sits immediately before its first use, so the variable's lifetime and purpose are visible at a glance, its scope is minimal, and the body reads top-down as cohesive paragraphs. Enforcement is exactly this layout check: "Flag declarations that are distant from their first use."
- finding: A C-style block of declarations hoisted to the top of the function, first touched many lines later. The reader must scan forward to discover each name's purpose, must hold them all in working memory, and the wide scope invites accidental reuse. Correct; the layout simply no longer co-locates a name with its meaning.
- AI-agent tendency: Declaration distance is invisible to a correctness pass and to most linters — the agent sees the declaration, sees a later valid use, and confirms. It doesn't model reader working-memory cost or measure the gap, so front-loaded declaration dumps sail through.

**CCG-ES.22: Don't declare a variable until you have a value to initialize it with**
- good: Each variable is declared at the point it can be given its real initial value, keeping scope minimal and the surrounding paragraph self-contained — no default-then-assign-later seams splitting one logical step across the body. Reason: "Readability. Limit the scope... Don't risk used-before-set."
- finding: An uninitialized or zero-initialized variable parked early (`string s;`), then conditionally assigned far below. Even when every path eventually writes it (no UB), the declare-then-fill pattern smears one logical step across the whole function and hides whether all paths are covered.
- AI-agent tendency: Static analysis and the agent focus on the definitely-assigned-before-read question; once that holds, the split is accepted. The readability/altitude cost — a step split across the function — is not a correctness signal, so a line-by-line pass approves it.

**CCG-NL.1: Don't say in comments what can be clearly stated in code**
- good: Where a comment would restate mechanics ("multiply m with v1 and add the result to vv"), the code is instead written — or the block extracted and named — so names and structure say it. Intent is carried by the code's shape, with no redundant prose to drift out of sync. Reason: "Compilers do not read comments."
- finding: A comment narrates what the next line literally does, or paraphrases a block of raw detail that should have been extracted and named. This is the Spine-2 tell that intent is recoverable only from comments, not from shape: a load-bearing mechanics-comment is evidence the structure dropped altitude.
- AI-agent tendency: The agent treats explanatory comments as a positive signal ("well documented") and may add more. It doesn't recognize a mechanics-restating comment as evidence the function mixes altitudes and should Extract Function, so it preserves or multiplies the crutch that hides weak structure.

**CCG-NL.2: State intent in comments**
- good: Comments are reserved for what code cannot say — the intended contract (e.g. "sort c by <, keeping equal elements in original order") above a non-trivial body — because "code says what is done, not what is supposed to be done." The comment states the supposed-to, giving a reader something to check the body against.
- finding: The only comments present restate mechanics (an NL.1 violation) while the actual intended behaviour or invariant is never stated, so there is nothing to check the implementation against. The guideline's warning bites: if comment and code disagree both are likely wrong — and here there is no intent comment at all.
- AI-agent tendency: The agent judges comment presence, not whether comments carry intent. It rarely notices that a non-trivial function lacks any statement of its contract, so review confirms "has comments" while the load-bearing intent stays unexpressed.

**CCG-NL.4: Maintain a consistent indentation style**
- good: Indentation consistently reflects nesting depth, so the body's control structure is readable as shape alone — the reader sees the phases and guards of an AO handler by indentation contour.
- finding: Inconsistent indentation that misrepresents nesting (the dangling-else / goto-fail family), where the visual structure disagrees with the actual block structure. It can compile and run while actively misleading a reviewer about which statements are guarded.
- AI-agent tendency: The agent reads the braces, not the whitespace, so misleading indentation that contradicts the real structure is invisible to it — it sees the true scoping and approves. This is the exact gap where a human (or a security bug like goto-fail) is misled but the agent is not, so it won't warn the human.

**CCG-NL.16: Use a conventional class member declaration order**
- good: Members follow the conventional order — types, then constructors/assignments/destructor, then functions, then data; public before protected before private — so the class reads top-down at a consistent altitude and its layout states "what you call" before "how it works." For an RP2350 QP/C active object, the public event-handling interface reads before private state.
- finding: Scattered access blocks (public, then private, then public again) and data hoisted above the interface force the reader to reassemble the type's intent from fragments. The class compiles and behaves correctly, but its shape no longer separates API from internals — altitude-mixing at type scope.
- AI-agent tendency: Member ordering has zero semantic effect, so the agent never flags a scrambled class — every order compiles and behaves identically. It appends a new member (and often a fresh access label) wherever the diff is smallest, treating ordering as style noise rather than a readability contract.

**CCG-NL.17: Use K&R-derived layout**
- good: Consistent K&R/Stroustrup layout makes the function's shape legible — "separate lines for each statement, the branches of an if, and the body of a for." Clear brace structure and blank-line separation let the body's altitude and paragraph breaks be read at a glance.
- finding: A body mixing brace styles, compressed branches, or irregular bracing so the visual structure no longer signals where one altitude/paragraph ends and the next begins. Behaviorally irrelevant and a formatter may accept it, but the function stops "reading as a shape" — precisely the Spine-2 concern.
- AI-agent tendency: Brace placement is invisible to semantics, so the agent never raises it and defers to "the formatter"; it matches whatever snippet it last saw rather than the file's prevailing style. It won't connect inconsistent-within-a-function layout to lost altitude legibility because each statement is individually valid.

**CCG-NL.18: Use C++-style declarator layout**
- good: Pointer/reference declarators bind to the type (`T* p`, `T& r`) consistently, so declarations inside the body read uniformly and the type of each name is unambiguous at a glance.
- finding: Mixed declarator styles within a function (`int *p` next to `int* q`), or inconsistently split one-per-intent declarations. It compiles, but the inconsistency adds friction exactly where the reader is trying to map names to types.
- AI-agent tendency: Declarator spelling is purely cosmetic to the compiler, so the agent treats `T *p` and `T* p` as equivalent and never flags mixing. It doesn't enforce intra-function consistency because no individual line is wrong.

**CCG-NL.20: Don't place two statements on the same line**
- good: One statement per line, so the function's vertical layout is a faithful one-step-per-line narrative at uniform granularity — each step is independently visible and independently breakpoint-able on hardware.
- finding: Two statements crammed on one line (`flag = true; notify();`, or `if (x) y(); else z();`) — correct and compact, but it hides a step from the line-oriented scan and from a debugger single-stepping an ISR/AO handler. The layout no longer reflects the operation count.
- AI-agent tendency: The agent parses both statements and validates each, so the packing is semantically transparent to it — and it will even compress two statements onto one line to minimize line count or match a terse neighbor. Nothing is hidden FROM THE COMPILER, only from the human and the debugger, so it won't flag it.

**CCG-NL.21: Declare one name (only) per declaration**
- good: Each declaration introduces a single name, ideally near first use, so declarations form a clean readable column and pointer/reference declarators aren't shared ambiguously across names.
- finding: `int* p, q;` (q is not a pointer) is the classic correct-by-accident trap and a real bug source; multi-name declarations also push variables away from first use, working against declare-near-use. The defect is structural/legibility, invisible at the statement's surface.
- AI-agent tendency: The agent resolves the declarator grammar correctly and confirms types are as intended, so it passes `int* p, q;` when it happens to be right — and may reproduce or extend the form, misreading `*` as distributing across all names. It rarely warns that the form is a readability and bug-prone trap because in the instance under review it behaves correctly.

**Fowler — Extract Function** (aka Extract Method)
- good: A fragment of lower-altitude detail is pulled into a well-named function, so the caller reads as a sequence of same-altitude named intentions — the name documents the "what," the body holds the "how." This is the mechanical enabler of a single level of abstraction / the stepdown rule, motivated by separating the caller's intention from the implementation detail.
- finding: An inline block clearly does one nameable sub-task but is left embedded beside higher-level calls, dragging the surrounding code down to its altitude. Correct, but it is the raw material that should have been extracted so the parent reads top-down; the un-extracted fragment is itself the Spine-2 smell.
- AI-agent tendency: Extraction changes no behavior, so the agent has no correctness trigger to propose it — inlined logic that works is accepted as complete. It extracts only under duplication or length pressure, not to restore a single level of abstraction, because altitude isn't a property it checks.

**Fowler — Slide Statements**
- good: Related statements are moved together and a variable's declaration is slid down to just before its first use, so the function reads as coherent paragraphs, each doing one thing. The visual grouping itself states which lines belong to which step. (Title confirmed verbatim; Fowler body paywalled — name/principle cited only.)
- finding: Logically related lines are spread apart and a declaration sits far from where it is finally used, so the eye cannot group the function into intent-paragraphs. The ordering is arbitrary rather than narrative; the layout fails to state structure even though the code runs correctly.
- AI-agent tendency: Statement ordering is behavior-preserving, so the agent has no correctness hook and treats the arrangement as given — it will even add a new declaration "at the top where declarations go." It doesn't assess whether lines are grouped into coherent paragraphs, so scattered bodies pass review.

**Fowler — Split Phase**
- good: A function doing two sequential kinds of work (parse-then-compute, decode-event-then-act, read-config-then-act) is visibly split into ordered phases, each at its own level of abstraction, so the body reads top-down as distinct stages rather than an interleaved blob — directly the "top-down narrative" shape.
- finding: A body that interleaves two phases (computing intermediate data while already consuming it) so the stages aren't separable by eye. It compiles and tests pass, but the layout hides the phase boundary, keeping two abstraction levels braided together and making the operation harder to reason about as a sequence.
- AI-agent tendency: Interleaved phases compute the same result as split ones, so the agent validates the output and moves on — and extends the function by threading new logic into the existing braid (smallest diff). The absence of a visible phase boundary isn't a defect it recognizes; phase structure is an intent/layout concern outside its correctness scope.

---

#### UNVERIFIED (named only — NOT included as criteria)

These appeared in the per-angle research but were NOT marked `verbatim_confirmed` in any verify pass, so per the verbatim-only gate they are excluded from the criteria above.

- **Single Level of Abstraction Principle (SLAP) / "one level of abstraction per function" / stepdown rule** — the conceptual core of this entire spine, but its canonical home is Robert C. Martin's *Clean Code* (paywalled). No free primary page defines it with a citable ID. The research correctly declined to mint an ID and routed the idea through the verified F.2 / P.1 / P.3 / F.3 / Extract Function / Split Phase rows; it remains unverified as an independent citable guideline.
- **Compose Method** — Joshua Kerievsky, *Refactoring to Patterns* (paywalled); confirmed NOT in Fowler's free catalog. No guideline row was created — correct.
- **CCG-SF.4: Include `.h` files before other declarations in a file** — could not be confirmed against a PRIMARY source; the canonical isocpp single-page document truncates inside "F: Functions" before reaching the SF section on every fetch. Title corroborated only by non-primary mirrors/snippets.
- **CCG-SF.5: A `.cpp` file must include the `.h` file(s) that defines its interface** — same primary-source truncation; not confirmable from primary despite mirror corroboration.
- **CCG-SF.7: Don't write `using namespace` at global scope in a header file** — same truncation; wording also differs across mirrors (older mirror: "Don't put a `using`-directive in a header file"), and the current-master phrasing could not be verified against the primary source.
- **CCG-SF.8: Use `#include` guards for all `.h` files** — same truncation; mirror-corroborated only, not confirmed from primary.
- **MISRA C++:2023 / AUTOSAR layout rules** — paywalled primary text not fetched; the research declined to invent rule numbers. Their layout guidance is covered by the verified CCG NL.* rows.
- **Fowler — Slide Statements body / principle text** — the title is verbatim-confirmed (so the row above is a valid criterion), but the entry body is paywalled and its detailed text remains unverifiable.

---

### Spine 3 — Code smells & gestalt review heuristics (whole > parts)

The reviewer judges the whole function/file at once: can you name it without "and", can you eyeball-verify every path, is the same idea expressed once, does behavior live with its data? No single line is wrong; the gestalt is what fails.

**Criteria (verbatim-confirmed only):**

- **CCG-F.2: A function should perform a single logical operation**
  - good: Read the whole function and name it in one verb phrase with no "and"; if the single name fits (`read_sample`, `apply_calibration`), it does one logical operation. Out-parameters are absent or rare; the body reads as one coherent unit at one level of abstraction.
  - finding: You cannot summarize the function without "and" (it reads a sensor AND formats AND transmits). The CCG enforcement smells apply: more than one "out" parameter, a body that does not fit one editor screen, distinct logical parts glued into a monolith. Mark it F.2 even when every line compiles.
  - AI-agent tendency: LLM generation emits one linear top-to-bottom function that does everything asked in sequence (acquire, validate, transform, emit, log), mirroring the prompt's narrative — fusing several logical operations into one un-reusable blob.

- **CCG-F.3: Keep functions short and simple**
  - good: The function fits on one screen and you can trace every alternative path by eye; large bodies are broken into smaller cohesive and named functions. Nesting is shallow.
  - finding: The function does not fit on a screen, or has deeply nested flag-driven branching where you cannot convince yourself "all possible alternatives have been correctly handled." A holistic call — no single line is wrong; the gestalt is unverifiable-by-eye.
  - AI-agent tendency: Agents inline helper logic rather than extracting named sub-operations, and pile on flag parameters and special-case branches to make every test green, producing the long, multi-flag, deeply-nested shape that compiles, passes, and lints clean but cannot be eyeball-verified.

- **CCG-ES.3: Don't repeat yourself, avoid redundant code**
  - good: Scan the whole file/module for the same shape of code appearing more than once; the common part is hoisted or a standard algorithm is used. Each piece of knowledge lives in exactly one place.
  - finding: The same computation, validation pattern, magic-constant sequence, or branch body appears in two or more places (often near-identical with one tweak). Line-by-line review passes each copy individually; only whole-file gestalt sees the repetition.
  - AI-agent tendency: Because each turn/edit is generated semi-independently and the model favors locally-complete code, agents re-emit a block they already wrote elsewhere (cut-and-paste programming at scale) — duplicates individually correct and tested, so nothing flags, until one requirement change must be made in N places.

- **CCG-C.131: Avoid trivial getters and setters**
  - good: Getters/setters add semantic value (maintain an invariant, convert internal→interface type) or they are not present; where there is nothing to encapsulate, the type is a struct of public data.
  - finding: A class wraps plain data in mechanical `get_x()`/`set_x()` pairs with no behavioral member functions and no invariant. Enforcement: "Flag multiple get and set member functions that simply access a member without additional semantics." Ceremony without semantic value.
  - AI-agent tendency: LLMs default to "proper OO" boilerplate — private fields plus full getter/setter pairs — even for passive value aggregates, producing encapsulation theater with no invariant behind it.

- **CCG-I.23: Keep the number of function arguments low**
  - good: A function's parameter list is short and each argument is plainly needed; arguments that travel together are grouped into a struct. A short list is corroborating evidence the function does one thing.
  - finding: A long parameter list (Fowler's Long Parameter List smell), often signaling several jobs merged or a Data Clump that should be an object. Each parameter is individually valid; the gestalt is a signature pointing at a deeper single-responsibility violation.
  - AI-agent tendency: When an agent needs more inputs to handle an added case, it appends another parameter (and another flag) rather than reconsidering decomposition — the minimal diff that keeps tests passing — so parameter lists grow monotonically into Long Parameter List / flag-argument smells.

- **P10-Rule4: Power of Ten, Rule 4 — no function longer than a single sheet of paper (≈60 lines)**
  - good: Each function "is a logical unit in the code that is understandable and verifiable as a unit" (verbatim rationale) — it fits on one page so a reviewer can hold and verify the whole unit at once.
  - finding: A function spans "multiple screens on a computer display or multiple pages when printed," which the rule calls "often a sign of poorly structured code." A pure whole-function gestalt check: length and structural integrity as a unit, independent of any line being wrong. The direct house-standard anchor for the ungated holistic length judgment.
  - AI-agent tendency: Agents grow a function past the one-page bound by repeatedly adding in-place handling for each new case; the no-heap/no-recursion constraints push logic into straight-line code, so generated functions sprawl and become un-verifiable-by-eye while building, running, and testing clean.

- **Fowler-CodeSmell: a smell is "a surface indication that usually corresponds to a deeper problem"; "by definition something that's quick to spot"; "smells don't always indicate a problem"**
  - good: Reviewing the gestalt, identical/near-identical fragments and misplaced behavior jump out on visual inspection and have been consolidated/relocated; the reviewer spots the surface smell, then investigates.
  - finding: A surface smell is present on a whole-module read; the reviewer flags it and judges whether it hides a missing abstraction (smells don't always indicate a problem, but they always warrant the look).
  - AI-agent tendency: Independent per-request generation reproduces duplication and misplaced responsibility that pass tests; only a holistic file read (the kind Fowler says is "quick to spot") catches what a line-by-line pass waves through.

**UNVERIFIED — do not cite until checked (code-smells-gestalt):**
- `Fowler-DuplicatedCode` — smell NAME is genuine (Chapter 3, *Refactoring* 2nd ed.) but the cited primary URL (refactoring.com/catalog) lists refactorings, not smells; the one-line gloss is paraphrase. Name citable per brief; body text not confirmable at the cited non-paywalled source.
- `Fowler-DivergentChange` — same: name real, cited URL does not contain it, gloss ("change different parts/contexts") is paraphrase, book chapter is paywalled.
- `Fowler-ShotgunSurgery` — same: name real, not on any fetched non-paywalled page, gloss ("changes in a lot of different places") is paraphrase.
- `Fowler-FeatureEnvy` — same: name real, not on the cited catalog or CodeSmell.html, gloss ("communicates more with functions in another module") is paraphrase.

---

### Spine 4 — Reviewing AI/LLM-generated code (documented failure modes)

The spine failure mode: LLM code is plausible, fluent, heavily-commented, and passes tests/lint while being improper on a path no test exercises. The human walker must distrust the model's confidence signals specifically — confident comments that drift from code, happy-path-only error handling, hallucinated API/contract assumptions, managed-language idioms unsafe in a no-heap/no-exceptions/dual-core build, and over-production that buries the one wrong branch.

**Criteria (verbatim-confirmed only):**

- **CppCoreGuidelines P.1: Express ideas directly in code**
  - good: The function's structure states its intent; a walker can read the body and recover WHY before reading any comment.
  - finding: The code "works" but its intent lives only in a comment/docstring while the body does something subtly different or more general than the comment claims. Flag any function where deleting the comments would leave a reader unable to reconstruct the requirement.
  - AI-agent tendency: LLMs pair plausible prose with code that drifts from it — the comment asserts the invariant; the code only approximates it. Reviewers anchored to the confident comment wave the body through.

- **CppCoreGuidelines P.5: Prefer compile-time checking to run-time checking**
  - good: Constraints are enforced by the type system / static_assert / constexpr, so an out-of-contract call fails to compile rather than at run time on Core 1.
  - finding: LLM code re-checks at run time what the types already guarantee, or leaves a contract checkable only by reading. On bare-metal RP2350 a runtime-only guard is a latent fault; demand it be lifted to compile time where possible.
  - AI-agent tendency: (none recorded beyond the finding) — runtime guards reproduced from training-data idioms where compile-time enforcement was available.

- **CppCoreGuidelines P.8: Don't leak any resources**
  - good: Every acquired resource has an owner and a deterministic release path on every exit, including early returns; ownership is visible in one function's scope (RAII).
  - finding: LLM code acquires-then-returns-early on an invented error branch, skipping release; or assumes a GC/destructor that does not exist in this no-heap-after-init build. Walk EVERY exit path, not the happy path the tests exercise.
  - AI-agent tendency: LLMs reproduce managed-language idioms (implicit cleanup, defer-by-convention) and add defensive-looking error branches that leak on the path no test covers.

- **CppCoreGuidelines P.10: Prefer immutable data to mutable data**
  - good: State shared across Core 0 / Core 1 is const where it can be; the small mutable surface is explicit and obviously guarded.
  - finding: LLM introduces a mutable static/global "cache" or "flag" that compiles and passes single-core tests but is touched by both cores without a guard — invisible per-line, surfaces only when you ask "who else writes this?" across the whole file.
  - AI-agent tendency: (none recorded beyond the finding).

- **CppCoreGuidelines F.3: Keep functions short and simple**
  - good: Each function does one thing at one level of abstraction and fits on a screen; a reviewer can hold its whole contract in mind at once.
  - finding: LLM emits a long, plausible, well-formatted function fusing parsing + validation + I/O + error handling. It passes tests and lint, but its size hides a wrong branch. Treat unexplained length/breadth as the review target.
  - AI-agent tendency: LLMs over-produce — comprehensive-looking functions with extra cases and defensive branches (Speculative Generality in spirit) that inflate surface area and bury the one incorrect path.

- **CppCoreGuidelines ES.20: Always initialize an object**
  - good: Every local, member, and constructed object is initialized before any read; no path reaches a use of an indeterminate value.
  - finding: LLM declares a struct/array/local and a confident-looking branch reads a field before it was set on that path; tests pass because the field happens to be set on the tested path. Walk declaration-to-first-read for each variable, per path.
  - AI-agent tendency: (none recorded beyond the finding). *(Note: research had flagged ES.20 as a LEAD; verification subsequently confirmed it verbatim against the raw master, so it is staged as a criterion.)*

- **CppCoreGuidelines CP.2: Avoid data races**
  - good: Cross-core shared writable state is synchronized or eliminated; no two contexts perform unsynchronized access where at least one writes.
  - finding: LLM introduces unguarded cross-core/ISR shared state. *(Research flagged CP.2 as a LEAD; verification confirmed it verbatim against the raw master, so it is staged here.)*
  - AI-agent tendency: (none recorded beyond the finding).

- **Power of Ten Rule 7 (Holzmann): non-void return values must be checked by each calling function; parameter validity must be checked inside each function**
  - good: Every non-void call's result is checked or explicitly cast to `(void)` with a comment saying why ignoring it is safe; every function validates its parameters on entry.
  - finding: LLM code calls an API and uses the result as if it always succeeds — no check, no explicit `(void)` — because in the training-data happy path it "just works." Also flag invented/optimistic return-value semantics (assuming a function returns the data when it returns a status).
  - AI-agent tendency: LLMs write to the success path and silently drop error returns; combined with hallucinated API contracts they assume a return shape the real API does not have.

- **SEI CERT EXP53-CPP: Do not read uninitialized memory**
  - good: Every local, member, and new'd object is initialized before any read; no path reaches a use of an indeterminate value.
  - finding: LLM declares a struct/array/local and a branch reads a field before it was set on that path; tests pass because the field happens to be set on the tested path. Walk declaration-to-first-read per path.
  - AI-agent tendency: (none recorded beyond the finding).

- **SEI CERT ERR62-CPP: Detect errors when converting a string to a number**
  - good: Any string-to-number / parse step checks for conversion failure and out-of-range before the value is used downstream.
  - finding: LLM parsing of a sensor frame or config token converts and uses the number without detecting failure/overflow, because the prompt's examples were well-formed. The improper instance is plausible and compiles; demand the failure detection the model skipped.
  - AI-agent tendency: LLMs assume well-formed input from their example-rich training distribution and omit the unhappy-input handling real embedded byte streams require. *(Citation-precision note: the title was confirmed verbatim on the ERR index page, not a dedicated err62-cpp page.)*

- **SEI CERT ERR57-CPP: Do not leak resources when handling exceptions**
  - good: Error-handling paths release everything the success path would; in this no-exceptions build, the equivalent error-return paths are RAII-clean.
  - finding: LLM adds an exception/try-catch or error branch (sometimes inappropriate for a no-exceptions target) whose cleanup is incomplete, or that contradicts the house no-RTTI/no-exceptions rule while still compiling under the toolchain default.
  - AI-agent tendency: LLMs default to exception-based error handling from mainstream C++ corpora, ignoring the project's no-exceptions constraint — a passing-but-improper pattern a linter may not flag.

- **Fowler/Beck smell: Comments (Bad Smells in Code)**
  - good: Comments explain WHY (rationale, hazard) where code cannot; they are not restating or apologizing for the code. Verbatim: comments are often used "as a deodorant."
  - finding: LLM output is heavily, fluently commented in a way that narrates the code line-by-line and projects confidence. Treat dense explanatory commenting as a smell pointing at code that needs reading harder, not as evidence of quality.
  - AI-agent tendency: LLMs emit confident, voluminous comments/docstrings (over-confidence signal); reviewers read the assured prose and under-scrutinize the code — the core spine failure of this facet.

- **Fowler/Beck smell: Duplicated Code (Bad Smells in Code)**
  - good: A rule is expressed once; the same constant/sequence is not re-derived in multiple functions. Verbatim: "Number one on the stink parade is duplicated code."
  - finding: LLM regenerates near-identical helper logic (bounds math, frame parse, register-write sequence) in several spots that drift subtly. Each copy compiles and passes; the divergence between copies is the latent bug. Compare copies whole, not line-by-line.
  - AI-agent tendency: Independent per-request generation reproduces the same logic at multiple sites; only a holistic read catches it.

**UNVERIFIED — do not cite until checked (ai-code-review):**
- ERR62-CPP citation precision: the `source_url` points to the ERR *index* page, not a dedicated err62-cpp rule page. Title is confirmed; this is a citation-precision note, not a fabrication.

*(The supporting studies — Perry et al. CCS 2023, Spracklen et al. USENIX 2025, the TOSEM Copilot study — back the AI-agent **tendency** characterization, not any guideline ID. They are recorded in the consolidated source list with study-vs-opinion tags and must not be cited as guideline criteria. The Help Net Security news item was **dropped** per repo-owner — redundant non-primary corroboration, superseded by the peer-reviewed Spracklen/USENIX hallucination anchor.)*

---

## PART 2 — SUPPORTING FACETS (appendix rule-classes)

---

### Support A — Comments & documentation quality (holistic half) — maps to house rules JSF AV C++ 127–134

The why-not-what spine for documentation: a comment must survive a reasonable reimplementation, carry intent/units/preconditions, and never restate or contradict the code or disable real code.

**Criteria (verbatim-confirmed only):**

- **CppCoreGuidelines NL.1: Don't say in comments what can be clearly stated in code**
  - good: Where the language can express the fact (a named variable, typed constant, enum, const/constexpr, named function), the code expresses it and there is NO redundant comment. Comments are reserved for what code cannot express.
  - finding: A comment duplicates information already plain in the code (canonical bad example: `auto x = m * v1 + vv; // multiply m with v1 and add the result to vv`). If deleting the comment loses zero information, it is noise that will rot out of sync.
  - AI-agent tendency: Agents over-comment to look thorough — restating self-evident code by reflex — manufacturing comment debt that diverges on the next edit and buries the few comments carrying intent.

- **CppCoreGuidelines NL.2: State intent in comments**
  - good: Each non-trivial function/block carries a comment that says WHY it exists or WHAT it must achieve (the contract/intent), still true if the implementation were rewritten. Preconditions, units, invariants, and the reason for a non-obvious choice are stated.
  - finding: A comment narrates mechanics the code already shows, OR comment and code disagree ("If the comment and the code disagree, both are likely to be wrong") — e.g. a comment says "milliseconds" but the code stores microseconds.
  - AI-agent tendency: LLM comments paraphrase the token stream just produced ("// initialize the mutex") rather than encoding domain intent, units, or the safety reason. They restate WHAT and almost never WHY, and go stale when later edited.

- **CppCoreGuidelines NL.3: Keep comments crisp**
  - good: Comments are concise and to the point. Documentation that belongs in the spec POINTS to the spec (cite the standard clause / ICD section) rather than paraphrasing it inline.
  - finding: Bloated, multi-paragraph block comments (often a re-paraphrase of a datasheet, RFC, or CCSDS/ICD section) that push code off-screen and will drift from the authoritative source. Proper form is a short pointer ("see CCSDS 132.0-B §4.1.2").
  - AI-agent tendency: LLMs produce verbose, essay-style block comments and confidently paraphrase external standards inline (doc-paraphrase instead of doc-pointer). On a standards-fidelity codebase the paraphrase can be subtly wrong AND it duplicates the spec, so it rots.

- **CERT MSC04-C: Use comments consistently and in a readable fashion** *(CERT C recommendation inherited via the house standard — cite as CERT C, do not relabel CERT C++)*
  - good: No code is disabled by wrapping it in `/* ... */`; temporary disabling uses conditional compilation (`#if 0` / `#endif`, `#ifdef`). Comment delimiters are not nested.
  - finding: Commented-out code left in the source, or a `/*` inside a comment that could swallow a trailing `*/` and silently disable real code. Any block of real C++ inside a comment is a finding — delete it (version control remembers) or `#if 0` it with a reason.
  - AI-agent tendency: Agents leave prior/alternative implementations commented out ("keeping the old version just in case") or comment out a block to make tests pass and never remove it — dead, misleading clutter on a safety-critical build.

- **CERT MSC12-C: Detect and remove code that has no effect or is never executed** *(CERT C recommendation inherited via the house standard)*
  - good: Disabled/dead code and its describing comments are removed once obsolete; documentation describes code that actually runs. (Exception: code intentionally `#ifdef`'d out for another platform/build, with a reason, is fine.)
  - finding: A descriptive comment documenting a path the code no longer takes, or stale doc-comments attached to dead/never-executed code. If the comment describes behavior you can prove the code can't reach (guard always false, function never called), the comment is documenting a lie.
  - AI-agent tendency: Agents leave doc-comments and headers attached to code they later neutralize or that is never reached, because they edit locally and don't reason about whole-program reachability — a false map of the system that builds green.

**UNVERIFIED — do not cite until checked (comments-docs):**
- `Fowler code smell: Comments` ("comments are often used as a deodorant"; "a comment is a good place to say why you did something") cited to refactoring.com/catalog — the cited primary URL lists refactorings only; neither it nor CodeSmell.html contains the quoted text. The smell NAME is genuine (Refactoring book, Bad Smells chapter) but the quoted body text is not confirmable against a permitted primary page.
- isocpp PRIMARY page text for NL.1/NL.2/NL.3 — the three NL titles WERE confirmed verbatim, but on a GitHub mirror (Trree/CppCoreGuidelines), not the cited isocpp primary, which truncated before the NL section. Titles stand; cited primary URL was unreachable for the relevant content.
- JSF AV C++ rules 127–134 (house anchors) — paywalled standard, no primary fetched; cited only as house-rule anchors with no body text. ID→title mappings unconfirmed.
- Power of Ten comment-quality content — P10.pdf did not parse this pass and P10 has no comment-specific rule; correctly not asserted.

---

### Support B — Class & interface design — backs JSF AV C++ 64–97

The dominant passing-but-improper pattern is **encapsulation theater**: private fields + trivial accessors wrapping aggregates with no invariant, plus special-member / polymorphic-base defects that compile and pass tests.

**Criteria (verbatim-confirmed only):**

- **CG-C.2: Use class if the class has an invariant; use struct if the data members can vary independently**
  - good: A type whose constructor establishes and whose members maintain a relationship (an invariant) is a class with private data and validating construction; a bag of independently-varying values is a struct with public members.
  - finding: A `class` with private members and accessors but no enforced invariant (a struct masquerading as a class); or a `struct` with public members that genuinely has a cross-member constraint a constructor should enforce (the invariant is unprotected).
  - AI-agent tendency: LLMs wrap plain value aggregates in a class with private members plus trivial get/set; the reverse also occurs (a public struct for a type with a real constraint, e.g. `count` that must match `len`), leaving the invariant enforceable only by convention.

- **CG-C.3: Represent the distinction between an interface and an implementation using a class**
  - good: The public interface is clearly separated from implementation detail; the public section presents a coherent, minimal contract and implementation members are private.
  - finding: A class whose public section interleaves implementation helpers, internal scratch state, or implementation-only members — callers can reach into mechanism, so the "interface" is not a contract.
  - AI-agent tendency: Agents promote helper methods and intermediate state to public while iterating, because making something public is the quickest way to get a call to compile, leaking implementation.

- **CG-C.8: Use class rather than struct if any member is non-public**
  - good: If a type has any non-public member it is declared `class`, signalling that the type controls access and likely maintains an invariant.
  - finding: A `struct` containing private/protected members — legal and compiles, but the keyword misleads a reader who assumes a transparent aggregate.
  - AI-agent tendency: Agents mix access specifiers under whichever keyword they started typing; a `struct` that accreted a private helper during a multi-edit session still says `struct`.

- **CG-C.21: If you define or =delete any copy, move, or destructor function, define or =delete them all**
  - good: Rule of five — a class declaring any one of {copy ctor, copy assign, move ctor, move assign, destructor} declares or `=delete`s the complete set.
  - finding: A class defining a destructor (e.g. to release a hardware handle or DMA channel) but leaving copy/move implicitly available — the compiler-generated copy duplicates the handle, leading to double-release/double-free.
  - AI-agent tendency: An agent writes a resource-freeing destructor and stops; the implicit copy shallow-copies the owned handle and the destructor runs twice. Tests that never copy the object pass; on bare-metal this corrupts a peripheral/lock.

- **CG-C.35: A base class destructor should be either public and virtual, or protected and non-virtual**
  - good: A polymorphic base has a public virtual destructor; a base never deleted through a base pointer has a protected non-virtual destructor (polymorphic deletion becomes a compile error, not UB).
  - finding: A class with virtual functions whose destructor is public and non-virtual (or implicitly defaulted/non-virtual) — deleting a derived object through a base pointer is UB.
  - AI-agent tendency: LLM abstract bases very often declare pure-virtual methods but omit any destructor, so the compiler gives a public non-virtual one; the UB is latent because the test never deletes through a base pointer.

- **CG-C.46: By default, declare single-argument constructors explicit**
  - good: Every constructor callable with a single argument is `explicit` unless an implicit conversion is genuinely intended and documented.
  - finding: A non-`explicit` one-argument constructor (`Timeout(int ms)`, `RegisterAddr(uint32_t)`) enabling an unintended implicit conversion — an `int` silently becoming a `Timeout`, defeating strong typing at call sites.
  - AI-agent tendency: LLMs routinely omit `explicit` on single-arg constructors; code compiles and reads cleanly but opens silent conversions that undermine the house strong-typing (typed units, register wrappers).

- **CG-C.131: Avoid trivial getters and setters**
  - good: A data member with no invariant is made public (or the type is a struct); accessors exist only where they add semantic value (validation, conversion, locking, invariant maintenance).
  - finding: A class with private fields shadowed one-for-one by `getX()/setX()` that only read/write the member — encapsulation theater that mis-signals an invariant exists.
  - AI-agent tendency: The signature LLM tell for this facet — private member plus boilerplate `get_`/`set_` for every field as "good encapsulation," producing data classes dressed as classes. *(Citation-precision: title is verbatim-confirmed against the raw master at anchor #Rh-get; the research's attached source_url pointed at GitHub issue #507, which should be replaced by the #Rh-get anchor.)*

- **CG-I.4: Make interfaces precisely and strongly typed**
  - good: Parameters and return types name the concept they carry (typed units, enums, wrapper types, spans) rather than raw `int`/`bool`/`void*`, so wrong arguments fail to compile and units cannot be transposed.
  - finding: An interface taking several same-typed primitives (`configure(int, int, int, bool, bool)`), a raw `void*`/length pair where a span exists, or a positional `bool` flag — callers can pass arguments in the wrong order or unit and it still compiles.
  - AI-agent tendency: LLMs gravitate to primitive-typed signatures with multiple `int`s and boolean flags (path of least resistance, common in training examples); tests pass with the test's own consistent order while real call sites can silently transpose.

- **CG-I.25: Prefer empty abstract classes as interfaces to class hierarchies**
  - good: A polymorphic interface is an abstract class of only pure virtual functions (plus the appropriate destructor per C.35) — no data, no implementation — so implementations are decoupled from the contract.
  - finding: An "interface" base carrying data members or concrete implementation, forcing derived classes to inherit state and coupling them to base internals.
  - AI-agent tendency: Agents fold convenience state and default implementations into the base "to avoid duplication," yielding interface bases with data members — a design defect each line of which is locally reasonable.

- **CERT-OOP52-CPP: Do not delete a polymorphic object without a virtual destructor**
  - good: Any object deleted through a base-class pointer has a base class with a virtual destructor.
  - finding: A `delete`/owning-pointer reset on a base pointer whose static type has a non-virtual destructor — UB (truncated destruction, leaked derived members). The deletion-site framing of the C.35 hazard.
  - AI-agent tendency: The destruction call site is often written separately from the class definition, so a missing virtual destructor only manifests as UB at the delete — invisible in unit tests that destroy via the concrete type.

- **CERT-OOP50-CPP: Do not invoke virtual functions from constructors or destructors**
  - good: Constructors and destructors call only non-virtual operations, or perform two-phase init explicitly; no virtual dispatch is relied on during construction/destruction.
  - finding: A ctor/dtor that calls a virtual member (directly or via a helper) expecting the derived override to run — during base construction/destruction the most-derived override is NOT called (and a pure virtual is UB).
  - AI-agent tendency: LLMs write an `init()` virtual and call it from the base constructor assuming derived behavior runs; it compiles and binds to the base version. Tests that exercise the fully-constructed object pass.

- **CERT-OOP58-CPP: Copy operations must not mutate the source object**
  - good: Copy constructor and copy assignment take the source by reference-to-const and leave it unchanged; const-correctness is enforced by the signature.
  - finding: A copy operation whose source parameter is non-const and is modified (transferring ownership in what is named a copy, or resetting a flag on the source).
  - AI-agent tendency: Agents sometimes conflate copy and move, writing a "copy" that steals from or edits the source (non-const source parameter).

- **FOWLER-DataClass: Data Class (code smell)**
  - good: Behavior lives with the data it operates on; a type carrying fields also carries the operations that maintain or interpret them.
  - finding: A class that is only fields plus getters/setters with all behavior elsewhere — the classic data-class smell. Pairs with C.131/C.2: either it has no invariant (make it a struct) or invariant enforcement is missing and scattered into callers.
  - AI-agent tendency: LLMs produce data classes by default — entity-like types of pure fields plus accessors, with logic spread across free functions or callers. *(Caveat: CodeSmell.html mentions "Data classes" only as an illustrative example, not a numbered catalog entry; cited as a named-smell reference, not a normative rule.)*

**UNVERIFIED — do not cite until checked (class-interface):**
- `CG-C.20` title as submitted — "C.20: If you can avoid defining **any** default operations, do" inserts "any"; the primary raw md reads "C.20: If you can avoid defining default operations, do". ID/anchor/intent correct but the cited title is not verbatim. **Correct by dropping "any" before this is staged as a criterion.**
- `CG-C.131` source_url = GitHub issue #507 — wrong/non-primary URL for the rule. The TITLE is verbatim-confirmed (raw md, #Rh-get) so C.131 is staged above; replace the citation URL with `...#Rh-get`.
- `cpp-core-guidelines-docs.vercel.app` — non-primary community mirror; any title not independently confirmed against isocpp should not be trusted from it.
- C.132 / C.133 / C.134 — strong leads (over-virtualization, protected data, member access levels) but not verbatim-confirmed against a fetched primary page this pass.

---

### Support C — Templates — backs JSF AV C++ 101–106

The signature template failure of LLM code is **unrequested/speculative generality**: asked for one concrete thing, the agent emits a fully templated, policy-parameterized abstraction with the type contract living only in the body (no concepts). The single instantiation compiles, runs, passes tests, and lints clean.

**Criteria (verbatim-confirmed only):**

- **CppCoreGuidelines T.10: Specify concepts for all template arguments**
  - good: Every template type parameter that is more than "merely a type" carries a concept constraint naming the semantic role it must satisfy; plain `typename`/`auto` appears only where nothing but "it is a type" can be assumed (rare, commented).
  - finding: A `template<typename T>` whose body silently assumes operations (`t.tick()`, `a < b`, `c[i]`, `<< t`) with no concept/requires guarding them. It compiles for the one type currently instantiated; a second instantiation fails deep inside with a wall-of-text error.
  - AI-agent tendency: LLMs default to `template<typename T>` boilerplate and let the body imply the contract, because the generated call site (the only test) instantiates with one concrete type that satisfies the unstated requirements.

- **CppCoreGuidelines T.11: Whenever possible use standard concepts**
  - good: Where a standard-library concept expresses the requirement (`std::integral`, `std::copyable`, `std::input_iterator`, `std::invocable`), the template uses it directly; locally defined concepts appear only for domain notions the standard has no name for.
  - finding: A bespoke concept that duplicates — imperfectly — a standard concept (`sortable`, `std::totally_ordered`, `std::regular`), usually weaker or subtly wrong, reducing interoperability and letting unintended types through.
  - AI-agent tendency: (none recorded beyond the finding).

- **CppCoreGuidelines T.20: Avoid "concepts" without meaningful semantics**
  - good: Concepts express a semantic notion and require a complete, coherent set of operations (a `Number` demands `+ - * /` together); single-operation predicates appear only as building blocks, never as the public constraint.
  - finding: A syntactic-probe constraint (`concept Addable = requires(T a,T b){ a+b; }`) used as the actual interface guard — the canonical trap is `Addable` matching `std::string`, so a numeric algorithm silently concatenates `"7"+"9"=="79"`.
  - AI-agent tendency: (none recorded beyond the finding) — hand-rolled syntactic "concepts" that accidentally match wrong types.

- **CppCoreGuidelines T.41: Require only essential properties in a template's concepts**
  - good: A template's concept lists the properties essential to what it fundamentally does and nothing more; incidental needs (debug streaming, logging) are not promoted into the public constraint.
  - finding: A concept/requires-clause bundling non-essential requirements — a `sort`-like routine demanding `Streamable<S>` only for a `cerr` debug line — over-constraining callers and churning the interface when instrumentation changes.
  - AI-agent tendency: (none recorded beyond the finding).

- **CppCoreGuidelines T.47: Avoid highly visible unconstrained templates with common names**
  - good: Unconstrained function templates with common names (`operator==`, `begin`, `swap`, `size`) are not placed in a namespace that also defines concrete types, and are constrained with concepts and/or kept out of widely-visible namespaces.
  - finding: An unconstrained `template<class T1,class T2> bool operator==(T1,T2)` in the same namespace as concrete types — it needs no conversions, so it out-competes the intended overload via ADL and silently mis-dispatches.
  - AI-agent tendency: (none recorded beyond the finding).

- **CppCoreGuidelines T.61: Do not over-parameterize members (SCARY)**
  - good: Member types/functions/lambdas/variable templates depend on every template parameter they are nested under; anything that doesn't use a parameter is hoisted out into its own minimally-parameterized type.
  - finding: A nested member formally parameterized on arguments it never references (the classic `List<T,A>::Link` that depends on `A` despite not using it) — every distinct argument combination spawns a redundant identical instantiation, bloating code size. Matters directly on a flash-constrained RP2350.
  - AI-agent tendency: (none recorded beyond the finding).

- **CppCoreGuidelines T.69: Inside a template, don't make an unqualified non-member call unless you intend a customization point**
  - good: An unqualified non-member call on a dependent argument (`f(t)`) is made only where ADL-based customization is the deliberate design (like `swap`); internal helpers live in a `detail` namespace and are called qualified.
  - finding: A template body calling an internal helper unqualified (`helper(t)` where `t` is dependent-typed) — turning a private implementation detail into an unintended ADL customization point that mis-dispatches when a new type is introduced.
  - AI-agent tendency: (none recorded beyond the finding).

- **CppCoreGuidelines T.120: Use template metaprogramming only when you really need to**
  - good: TMP is used only where genuinely needed; value computations use `constexpr` functions, type constraints use concepts rather than `enable_if`, and metaprogramming is never hidden in macros.
  - finding: Elaborate `enable_if`/SFINAE/recursive TMP where a `constexpr` function, concept, or plain overload would do — hard to read, slow to compile, hard to maintain. The gratuitous TMP is itself the defect on a maintainable safety-critical codebase.
  - AI-agent tendency: (none recorded beyond the finding).

- **CppCoreGuidelines T.143: Don't write unintentionally non-generic code**
  - good: Generic code commits only to the most general facility that does the job: iterators compared with `!=` not `<`, emptiness via `.empty()` not `.size()==0`, parameters typed to the least-derived class/concept actually used.
  - finding: A template needlessly restricting itself: `for (i = first; i < last; ++i)`, `x.size()==0` instead of `x.empty()`, or a function taking `Derived&` while only calling `Base`'s members.
  - AI-agent tendency: (none recorded beyond the finding).

- **CppCoreGuidelines T.144: Don't specialize function templates**
  - good: Function templates are never explicitly specialized; alternative implementations for particular types are provided by overloading, or by delegating to a specialized class template.
  - finding: An explicit specialization (`template<> void f<int>(...)`). Because function-template specializations don't participate in overload resolution, the call silently resolves to the primary template instead — doing the wrong thing for the very type it was meant to special-case.
  - AI-agent tendency: (none recorded beyond the finding) — explicit function-template specializations that don't participate in overload resolution.

- **CppCoreGuidelines T.150: Check that a class matches a concept using static_assert**
  - good: When a type is intended to model a concept (e.g. a sensor-sample POD meant to be `std::trivially_copyable`), a `static_assert(Concept<X>)` near the type pins that intent so a regression fails the build immediately, at the type, with a clear message.
  - finding: A type documented/assumed to model a concept but with no `static_assert` verifying it — so when someone adds a triviality/copyability-breaking member, nothing fails until a distant template instantiation does.
  - AI-agent tendency: (none recorded beyond the finding).

**UNVERIFIED — do not cite until checked (templates):**
- `Fowler Refactoring (smell): Speculative Generality` cited to refactoring.guru — the smell name/description are genuine but refactoring.guru is a third-party mirror, NOT a primary Fowler source; the entry is not on martinfowler.com/bliki/CodeSmell.html and refactoring.com/catalog lists refactorings, not smells; the book chapter is not freely fetchable. Name citable per brief, but the citation rests on a non-primary mirror, so it does not meet the primary-source verbatim bar.
- JSF AV C++ 101–106 — paywalled house anchors; intentionally not quoted, no body text invented.

---

### Support D — Concurrency & shared-data ownership (highest consequence)

The through-line: an LLM writes concurrency code correct for ONE execution context, reaches for the shortest sharing mechanism (a mutable global, a captured-by-reference object, a `volatile` flag), and structures locks for the happy path. Because the test suite and linters are effectively single-threaded, every one compiles, runs, and passes while being improper; the defect is timing-/interleaving-dependent.

**Criteria (verbatim-confirmed only):**

- **CP.3: Minimize explicit sharing of writable data**
  - good: The set of mutable objects reachable from more than one context (Core 0 vs Core 1, ISR vs main, two QP actors) is small, explicit, and each has ONE named owner; cross-boundary data is const/immutable or passed by value/message. A reviewer can name, for every cross-context object, who owns it and what guards it.
  - finding: A mutable global/static/singleton read by one core/ISR and written by another with no single owner and no documented guard. Red flags: a non-const reference/raw pointer to shared state captured into a Core-1 callback or QP handler; "temporary" shared scratch buffers; passing a whole struct when one POD field would do. The finding is the ownership ambiguity itself.
  - AI-agent tendency: An LLM reaches for a shared mutable global or capture-by-reference to "pass data" between producer and consumer — the shortest code that compiles and passes single-threaded tests — without reasoning about the two contexts, so it neither minimizes the shared surface nor marks it const.

- **CP.8: Don't try to use `volatile` for synchronization**
  - good: `volatile` appears ONLY for memory-mapped hardware registers / peripheral I/O; every cross-core or cross-ISR signaling variable is `std::atomic<T>` with a justified memory order, or is mutex/critical-section protected.
  - finding: A `volatile` flag/counter used to communicate between cores or ISR↔main, treated as if it provided atomicity or ordering — per the rule it "does not provide atomicity, does not synchronize between threads, and does not prevent instruction reordering... It simply has nothing to do with concurrency." Also flag a plain non-atomic read-modify-write (`free_slots--`) shared across contexts.
  - AI-agent tendency: LLM training data is full of pre-C++11/embedded idioms where `volatile` is the "shared-flag" tool, so an agent confidently writes `volatile bool data_ready` for a Core1→Core0 handshake; it compiles, lints silent, single-threaded tests pass, zero synchronization on two cores.

- **CP.20: Use RAII, never plain `lock()`/`unlock()`**
  - good: Every critical section is entered via a scoped RAII guard so the lock releases on EVERY exit path; no bare `mtx.lock()`/`mtx.unlock()` pairs and no manual save/restore-interrupts a later edit could skip.
  - finding: A manual lock/unlock (or `disable_irq()`/`restore_irq()`, `taskENTER/EXIT_CRITICAL`, `spin_lock`/`spin_unlock`) pair where some path can skip the unlock — an early `return`, a `break`/`goto`, or a later-added escaping branch. "Sooner or later, someone will forget the mtx.unlock(), place a return... or something."
  - AI-agent tendency: An agent writes symmetric `lock(); ...; unlock();` and then adds an early-return guard clause inside the critical section without connecting that the early return bypasses the trailing unlock. The no-exceptions house rule removes one escape path but not early returns/breaks.

- **CP.21: Use `std::lock()` or `std::scoped_lock` to acquire multiple `mutex`es**
  - good: Any path holding two or more locks acquires them atomically via `std::scoped_lock`/`std::lock` (or in one globally-agreed order), so acquisition order can never form a cycle. A reviewer can confirm a single documented lock-ordering discipline.
  - finding: Two locks (or a mutex plus interrupt-disable, or two spinlocks) acquired in one order on path A and the reverse on path B — the textbook deadlock; compiles and runs fine until the exact interleaving occurs. (C++CG counterpart of CERT CON53-CPP.)
  - AI-agent tendency: An agent edits two functions independently in separate turns and locks the mutexes in whatever order is locally convenient in each, never seeing both paths together; invisible to single-threaded tests and linters.

- **CP.22: Never call unknown code while holding a lock (e.g., a callback)**
  - good: Inside a critical section the code calls only short, known, non-blocking, non-reentrant leaf operations on the protected data; callbacks, virtual dispatch, function pointers, QP event posts, and logging are made AFTER the lock is released.
  - finding: A callback, virtual function, function pointer, `std::function`, event post, or any call into uncontrolled code invoked while a lock / critical section / interrupts-disabled region is held — risking re-entry/self-deadlock or blocking everyone waiting. On bare-metal this also shows up as work done with interrupts disabled too long.
  - AI-agent tendency: An agent keeps the lock guard at function scope and does ALL the work — including invoking a user callback or posting a QP event — inside the guarded scope, because the callback "is just another call."

- **CON50-CPP: Do not destroy a mutex while it is locked**
  - good: Every mutex outlives all contexts that can lock it — static/file-scope or owned by a long-lived object whose lifetime strictly contains its users. "If a mutex object is destroyed while a thread is blocked waiting for the lock, critical sections and shared data are no longer protected."
  - finding: A mutex scoped narrower than its users — a stack-local mutex in a function that spawns threads/registers an ISR and returns, or a mutex inside an object torn down during shutdown while the other core still runs. UB that compiles and usually "works" on the happy path.
  - AI-agent tendency: An agent places a mutex at the smallest scope satisfying local code ("declare it where it's used"), not reasoning about the lifetime of the OTHER context that captures it; with workers/ISRs that outlive the spawning function, the stack-local mutex is destroyed too early.

- **CON51-CPP: Ensure actively held locks are released on exceptional conditions**
  - good: Locks are released on ALL non-normal exits; the preferred mechanism is an RAII wrapper. In this no-exceptions codebase, every early-return/error-status path out of a manually locked region still releases the lock.
  - finding: A manually locked region where an abnormal exit (early return on error code) skips unlock — "the mutex will be left in the locked state." CP.20's failure mode framed around the error path specifically.
  - AI-agent tendency: An agent guards the normal path correctly but, when adding error handling, returns an error status from the middle of a hand-locked section without unlocking, treating the error branch as separate from the lock a few lines up.

- **CON52-CPP: Prevent data races when accessing bit-fields from multiple threads**
  - good: When distinct bit-fields are written by different contexts, the design accounts for shared storage units — all writers hold a common lock, or fields are split into independently-addressable atomic objects.
  - finding: Two contexts each update "their own" bit-field (or small adjacent member) of the same struct without synchronization. "When accessing a bit-field, a thread may inadvertently access a separate bit-field in adjacent memory" — the compiler's read-modify-write of the shared storage unit clobbers one update; "difficult to detect because the shared memory access isn't obvious from code inspection alone."
  - AI-agent tendency: An agent models each named bit-field as an independent variable and writes them from different contexts without a lock, with no notion that the hardware writes the whole byte/word.

- **CON53-CPP: Avoid deadlock by locking in a predefined order**
  - good: All code holding more than one lock acquires them in a single project-wide predefined order (by object identity/address), or uses `std::lock` — breaking the circular-wait precondition.
  - finding: Two paths acquire the same pair of mutexes in opposite orders (circular wait), so an unlucky interleaving deadlocks. "Sometimes, when locking mutexes, multiple threads hold each other's lock, and the program consequently deadlocks." (CERT counterpart of CP.21.)
  - AI-agent tendency: Across separate edits the agent locks the same two resources in inconsistent, locally-convenient orders with no global ordering discipline; nested locking is invisible to single-threaded tests.

- **CON54-CPP: Wrap functions that can spuriously wake up in a loop**
  - good: Every condition_variable `wait()`/`wait_for()`/`wait_until()` re-checks its predicate in a loop (explicit `while` or the predicate-lambda overload) while holding the associated mutex, so a spurious/stale wake-up does not let the thread proceed on a false condition.
  - finding: A condition_variable waited on with bare `cv.wait(lock);` (or `if` instead of `while`) and no predicate re-check, so a spurious wakeup or stale notify lets the consumer run on data that isn't ready.
  - AI-agent tendency: An agent writes the intuitive "wait, then use the data" with a single `wait()` and no loop because the recheck looks redundant and the one-producer/one-consumer test passes.

- **CON55-CPP: Preserve thread safety and liveness when using condition variables**
  - good: CV use preserves thread-safety (predicate re-tested under the mutex, in a `while` loop) and liveness (no thread blocked forever); where several threads wait on one CV for distinct conditions, the code uses `notify_all()` rather than `notify_one()`.
  - finding: `notify_one()` on a CV with multiple waiters of different predicates, so the single wakeup goes to a thread whose predicate is false (re-waits) while the thread that could proceed stays asleep — a lost-wakeup/liveness failure.
  - AI-agent tendency: An agent picks `notify_one()` as the "efficient" default and shares one CV across waiters with differing conditions, not reasoning about which waiter the single token reaches.

- **CON56-CPP: Do not speculatively lock a non-recursive mutex that is already owned by the calling thread**
  - good: No path re-locks a non-recursive mutex the same context already holds; re-entrant call chains route through a single top-level acquisition, use a recursive mutex deliberately, or assume the lock is already held.
  - finding: A function taking a non-recursive lock is reachable (directly or via a callback/virtual dispatch — see CP.22) from another function already holding the same lock — self-deadlock/UB, invisible at each call site individually.
  - AI-agent tendency: An agent factors a guarded operation into a helper that locks, then calls that helper from another already-locked region, because each function "should protect its own data."

**UNVERIFIED — do not cite until checked (concurrency):**
- CON54-CPP body quote "must be invoked from within a loop that verifies whether the condition predicate holds true" — ID/title verbatim-confirmed; this specific body wording is a paraphrase (primary renders it "must be invoked from a loop that checks whether a condition predicate holds"). Mapping sound; exact phrase does not match word-for-word.
- CON56-CPP `good`/`finding` body wording — author paraphrase (research disclosed the per-rule body page was not separately fetched). ID and title are verbatim-confirmed and the page is reachable, so the title citation is solid; treat the body text as paraphrase.
- MISRA C++:2023 / AUTOSAR C++14 — paywalled, intentionally not quoted.

---

### Support E — Manual residuals: control-flow, magic numbers, eval-order

The strongest AI-agent failure mode here: concise, compiling embedded code — inlined magic literals, mutate-and-read one-liners (UB that passes on the dev toolchain), int/unsigned mixing against `size_t`, and `volatile`-as-cross-core-sync — all of which pass tests and default linters yet are improper for RP2350 safety-critical review.

**Criteria (verbatim-confirmed only):**

- **CG-ES.43: Avoid expressions with undefined order of evaluation**
  - good: No object is read and modified more than once in a single full-expression without an intervening sequence point; side-effecting subexpressions are split into separate statements. "You have no idea what such code does... it might do something different on another compiler... or with a different optimizer setting." Enforcement: "Can be detected by a good analyzer."
  - finding: An expression whose value depends on unspecified/undefined order: `v[i] = i++;`, `f(i++, i)`, `a = a++ + ...`, or reading and writing the same volatile/global twice in one expression. On dual-core code, also flag a single full-expression that both reads and writes a shared/volatile object the other core touches.
  - AI-agent tendency: Agents emit terse "clever" one-liners folding a mutation and a use of the same variable into one expression; it compiles and gives the expected answer on the dev toolchain, so tests pass — the defect surfaces only under a different optimizer/compiler.

- **CG-ES.44: Don't depend on order of evaluation of function arguments**
  - good: Argument expressions are independent: no two arguments to the same call read-and-modify the same object, and no argument's correctness depends on left-to-right vs right-to-left evaluation.
  - finding: A call site whose arguments share a mutated object or an ordering assumption: `f(g(), h())` where both touch shared state, or two arguments each advancing the same stream/index. The code "works" only because of the current compiler's incidental order.
  - AI-agent tendency: When an agent compresses setup into a single call to look concise, it can place two interacting side effects in sibling arguments; there is no sequencing, but tests pass on the build compiler's chosen order.

- **CG-ES.45: Avoid "magic constants"; use symbolic constants**
  - good: Every literal carrying domain meaning (buffer size, timeout in ms, register bit position, scale factor, loop bound) is bound to a named constexpr/enum at one definition site; only self-evident literals (0, 1, nullptr, '\n', "") appear bare.
  - finding: A bare domain-meaning literal in an expression (`42`, `0.001f`, `1023`, `0x1F`, `250`). Two findings to walk for: (a) the SAME magic value repeated at multiple sites (the divergent-edit defect), and (b) a literal whose units/meaning are non-obvious without a name.
  - AI-agent tendency: LLM embedded code inlines plausible constants where used (`if (adc_val > 3500)`, `sleep_us(2000)`, `buf[256]`) because that is the most common training-data pattern; it rarely hoists one-offs, and when it emits the same value at two sites it doesn't cross-link them, so a later single-site edit silently diverges.

- **CG-ES.100: Don't mix signed and unsigned arithmetic**
  - good: Arithmetic and comparisons keep both operands in the same signedness; signed↔unsigned conversions are explicit and bounds-checked. Reason: "Avoid wrong results."
  - finding: A mixed-sign operation where the usual conversions silently coerce signed to unsigned: `int x; unsigned y; ... x - y`, `if (signed_var < unsigned_size)`, or `for (int i=0; i < container.size(); ++i)`. On embedded: a `length - 1` on an unsigned length that can underflow.
  - AI-agent tendency: Agents freely mix `int` counters/deltas with unsigned sizes, register widths, and `size_t`; the implicit conversion compiles clean with no default warning and tests with non-negative inputs pass — the wrong result triggers only at a negative/boundary value.

- **CG-ES.101: Use unsigned types for bit manipulation**
  - good: Masking, shifting, and bit-field work is done on unsigned types (`uint32_t` etc.), so shifts and the high bit are well-defined — appropriate for RP2350 register code.
  - finding: Bitwise operators on a signed operand: `int reg; reg << n`, `signed_val & MASK`, or a right-shift of a possibly-negative signed value (implementation-defined). Flag register/flag manipulation flowing through `int`.
  - AI-agent tendency: LLMs default intermediate variables to `int` and later `&`/`<<`/`>>` them; it works for small positive values, but the implementation-defined/UB shift behavior at the sign bit is invisible to tests and to a reviewer not tracking declared signedness.

- **CG-ES.102: Use signed types for arithmetic**
  - good: Quantities that can be subtracted or go negative are stored in signed types, so differences and decrements behave as expected and don't wrap.
  - finding: Arithmetic (especially subtraction or decrement-below-zero) on an unsigned variable: `unsigned remaining; remaining - used` that underflows, or `for (unsigned i = n; i >= 0; --i)` that never terminates. The counterpart to ES.101 — bit work unsigned, arithmetic signed.
  - AI-agent tendency: Agents reflexively pick `size_t`/`unsigned` for any count or length, then do arithmetic on it; the `i >= 0` unsigned loop and unsigned-underflow subtraction compile, pass nominal tests, and hang or produce a giant value only at the zero boundary.

- **CG-CP.8: Don't try to use volatile for synchronization**
  - good: `volatile` qualifies only objects that change outside the program (MMIO / hardware registers); cross-core communication uses `std::atomic` or proper synchronization. "In C++... volatile does not provide atomicity, does not synchronize between threads, and does not prevent instruction reordering."
  - finding: A `volatile` flag/handshake between cores or ISR↔main on the assumption it provides atomicity/ordering. Conversely, flag a hardware register / MMIO pointer that is NOT volatile-qualified. The pairing: synchronization-by-volatile = finding; hardware-access-without-volatile = finding.
  - AI-agent tendency: LLMs strongly associate `volatile` with "shared across threads/interrupts" (Java/C#/old-C carryover), so dual-core/ISR-shared flags get declared `volatile`; the missing atomicity/ordering only bites under optimization or a specific interleaving.

- **CERT-EXP50-CPP: Do not depend on the order of evaluation for side effects**
  - good: Side effects on the same scalar object are separated into distinct, sequenced statements (or captured in intermediates), so behavior is well-defined regardless of order. "In C++, modifying an object, calling a library I/O function, accessing a volatile-qualified value, or calling a function that performs one of these actions are ways to modify the state of the execution environment. These actions are called side effects."
  - finding: UB from unsequenced side effects on one scalar: `i = ++i + 1;`, `func(i++, i);`, or any expression that modifies and separately reads the same object with no sequencing. The CERT-graded (UB) sibling of ES.43/ES.44 — call it out as undefined behavior, not style.
  - AI-agent tendency: Identical to ES.43 — the LLM produces a compact mutate-and-read expression the dev toolchain evaluates sensibly; because it is UB, a passing test suite is actively misleading.

- **CERT-EXP52-CPP: Do not rely on side effects in unevaluated operands**
  - good: Operands of unevaluated contexts (`sizeof`, `typeid`, `noexcept`, `decltype`, `declval`) contain no side effect the code depends on. "Some expressions involve operands that are unevaluated... An unevaluated operand is not evaluated."
  - finding: A side-effecting call/increment inside `sizeof(...)`, `decltype(...)`, `noexcept(...)`, or `typeid(...)` relied upon to run — `sizeof(f(x++))` expecting `x` to advance, or `noexcept(init())` expecting `init()` to execute. It silently does nothing.
  - AI-agent tendency: Rare but high-impact for templated/constexpr-heavy modern C++ — an LLM may put a meaningful expression inside `sizeof`/`decltype` to "compute" a type and assume it also executes; the type is right so it passes, but the runtime effect never happens.

**UNVERIFIED — do not cite until checked (manual-residuals):**
- CP.8 Enforcement label: the quoted text ("Use atomic types where you might have used volatile in some other language. Use a mutex for more complicated examples.") is verbatim-correct but appears under the `##### Alternative` heading in the primary md, not under "Enforcement" as labeled. ID/title verified; only the section label is wrong.
- isocpp rendered-page URL as fetch source for ES/CP verbatim text — the rendered page truncates before ES/CP; IDs/titles WERE confirmed against the canonical raw markdown (raw.githubusercontent.com/.../CppCoreGuidelines.md), not via the rendered URL. Anchor deep-links match the slug convention but were not individually clicked through.
- INT31-C — referenced only as an ES.100 house-mapping cross-reference; not independently fetched/verified this pass. Treat as a lead, not a verified citation.
- ES.100 Reason "Avoid wrong results" — high confidence (corroborated) but the research flagged a mirror mis-paste risk; worth a 5-second eyeball on the live isocpp page.
- JSF / P10 house mappings (JSF 151, JSF 204.1, JSF 205, P10 Rule 1, P10 Rule 8, JPL Rule 18) — referenced as house-rule mapping targets; not staged as criteria here.

---

## Reviewer checklist before folding into the field manual

1. **Re-research Spine 2 (fn-layout-altitude):** it currently contributes zero criteria (placeholder research). It must be re-run before the spine is complete.
2. **Clear every per-facet UNVERIFIED list** above — either confirm the body quote/ID against a primary page, fix the citation URL (notably C.20 title "any", C.131 → #Rh-get), or explicitly accept the item as a named-smell lead.
3. **Approve the AI-code-review sources** (separate list below) — these need a human sign-off before any tendency claim leans on them.
4. **Resolve the F.1-lambda-enforcement discrepancy** in fn-decomposition (verified list vs. unverified list disagree).


---

# Pass 2 — AI-code-review lens (general / foundational)

# DRAFT — Reviewing AI-Generated Code (PRIMARY-lens evidence pack)

> **Staging artifact — NOT for folding into the manual.** This document characterizes *tendencies* of AI/LLM-generated code (what such code tends to get wrong while passing mechanical checks) and *reviewer-side* behavior. It is review-lens motivation only. **None of these items is a coding-standard guideline and none may be transcribed as a guideline ID.** Every quantitative claim below is tied to a source whose existence was confirmed and whose statistic was verified as present in the primary text; preprints and institutional guidance are labelled.

## Scope and framing

The binding premise of this lens: AI-generated code routinely **compiles, looks plausible, and passes the limited tests/linters applied to it, yet is functionally wrong, non-optimal, insecure, or fabricated** — and human reviewers systematically *under*-scrutinize it. The sources below justify *why* manual, security-context, human-led review is required *on top of* tests and linters. They do not establish acceptance thresholds.

A scope caveat carried from verification: published quantification is overwhelmingly **Python/JavaScript** (packages, web CWEs) and general-purpose APIs. For **C/C++ and safety-critical embedded** targets the only directly relevant study (LLM-CSEC) is a non-peer-reviewed preprint reporting static-analysis detections *not* conditioned on passing tests. For embedded C++, treat the hallucination/memory-safety tendencies as **documented behaviors to verify against**, not as asserted rates.

## Ranked sources (by authority)

### Peer-reviewed (Priority 1)
1. **EvalPlus — *Is Your Code Generated by ChatGPT Really Correct?*** (NeurIPS 2023). Canonical proof that sparse bundled tests score wrong LLM code as correct; HumanEval tests expanded 80x cut pass@k by **19.3–28.9%** and even mis-ranked models.
2. **Perry et al. — *Do Users Write More Insecure Code with AI Assistants?*** (ACM CCS 2023). Reviewer-side automation bias: AI-assisted participants wrote less secure code on 4 of 5 tasks yet believed it was *more* secure.
3. **Pearce et al. — *Asleep at the Keyboard?*** (IEEE S&P 2022). **~40%** of 1,689 Copilot completions across 89 MITRE CWE Top-25 scenarios were vulnerable. (Overall rate only; per-language split unverified.)
4. **Dakhel et al. — *Copilot: Asset or Liability?*** (Journal of Systems and Software 2023). Spec-noncompliance, snow-balling latent bugs labeled "correct," expert-in-loop conclusion.
5. **Nguyen & Nadi — *Empirical Evaluation of Copilot's Suggestions*** (MSR 2022). First-suggestion correctness only 27–57% by language; undefined-helper / undefined-variable "plausible but incomplete" code.
6. **Fu et al. — *Security Weaknesses of Copilot-Generated Code in GitHub Projects*** (ACM TOSEM 2025). In-the-wild: 29.5% Python / 24.2% JS snippets weak across 43 CWEs (top CWE-330, CWE-94, CWE-79); static-analysis-fed remediation fixed only up to 55.5%.
7. **Spracklen et al. — *We Have a Package for You!*** (USENIX Security 2025). **19.7%** of generated package refs non-existent; 205,474 unique fake names; 43% reproduce across all 10 re-runs (the persistence behind slopsquatting).
8. **Negri-Ribalta et al. — SLR on AI models & code-generation security** (Frontiers in Big Data 2024). Peer-reviewed synthesis: "code generated by AI is not necessarily secure"; language-varying MITRE Top-25 prevalence (Python 38.35%, C 50.29%).
9. **Khoury et al. — *How Secure is Code Generated by ChatGPT?*** (IEEE SMC 2023). Secure on only 5 of 21 programs first attempt; self-corrects mostly only when explicitly challenged.

### Standards-body / reputable-org (Priority 2)
10. **NIST AI 600-1 — Generative AI Profile** (2024). Defines **confabulation** (confidently-presented false content + fabricated justifying logic/citations) and prescribes MS-2.6-004 (review generated code for downstream risk).
11. **OWASP LLM05:2025 — Improper Output Handling.** Zero-trust treatment of LLM output; injection/RCE risk class and mitigations.
12. **OWASP LLM09:2025 — Misinformation (incl. Overreliance).** Names "insecure or non-existent code libraries" and Overreliance as the harm amplifier.
13. **GitHub Docs — *Review AI-generated code*** (tutorial) and **Responsible-use code-review** page. Concrete reviewer language; AI review must be supplemented with human review. (Note: the "risk of hallucination / supplement with human review" quotes live on the *responsible-use* page, not the tutorial — corrected here.)
14. **SEI/CMU — *Perspectives on Generative AI in SE and Acquisition*** (2025). Human oversight + static analysis for complementary perspectives; incremental review of decomposed subtasks; trust the workflow/process/people, not the output.

### Indicative — preprints / vendor (NOT peer-reviewed; cite as supporting only)
15. **CloudAPIBench** (arXiv 2024). API hallucination concentrates in low-frequency APIs (GPT-4o 38.58% valid → 47.94% with doc augmentation).
16. **LLM Hallucinations… Phenomena/Mechanism/Mitigation** (arXiv 2024). Hallucination taxonomy: Task-Requirement 43.53%, Factual-Knowledge 31.91% (API 20.41%), Project-Context 24.56%.
17. **LLM-CSEC** (arXiv 2025). C/C++: 5–35% of generated files vulnerable; recurring CWE-120/787/119/122 (memory) and CWE-252/253 (unchecked return). Static-analysis detections, *not* conditioned on passing tests.
18. **Duma et al. — *These Aren't the Reviews You're Looking For*** (arXiv 2026). Agent-authored PRs got human-only review only 8.08% of the time; most AI PRs receive no real human review.
19. **Basic & Giaretta — SLR, *From Vulnerabilities to Remediation*** (arXiv 2026). Ten vulnerability-class taxonomy across 102 studies (superseded by the Frontiers SLR for the headline).
20. **Lasso Security blog** (2024). Real-world huggingface-cli slopsquatting PoC (>30k downloads on an empty package); used for the dependency-verification practice, not for quantification.

## Failure-mode taxonomy (how AI code passes mechanical checks yet is improper)

- **Plausible-but-wrong:** passes the given tests, fails on unexercised inputs (EvalPlus).
- **Corner-case / subtle-condition mishandling** and **unstated-assumption violations** (ordering/uniqueness not preserved) (EvalPlus).
- **Non-optimal algorithms** that are correct but inefficient (EvalPlus).
- **Spec-noncompliance:** silently drops explicit requirements (type/boundary checks, ordering, banned constructs) (Dakhel et al.).
- **Snow-balling latent bugs** labeled "correct" under normal-usage tests (Dakhel et al.).
- **Non-self-contained code:** undefined helpers / undefined variables that look complete (Nguyen & Nadi).
- **Decorative-but-vacuous complexity:** more complex *and* less correct on the same problem (Nguyen & Nadi).
- **Security-CWE patterns** that compile and pass tests yet are unsafe (~40% rate, Pearce et al.; in-the-wild distribution, Fu et al.).
- **Memory-safety & unchecked-return-value CWEs in C/C++** (LLM-CSEC, indicative).
- **Hallucinated packages/APIs** — reproducible, hence exploitable (Spracklen et al.; CloudAPIBench, indicative; taxonomy preprint, indicative).
- **Confabulation** — confident wrong content plus fabricated justifying logic/citations that mislead reviewers into trust (NIST AI 600-1).
- **Improper output handling** — LLM output as an injection sink (OWASP LLM05).
- **Over-confidence / automation bias / overreliance** and **real-workflow under-review** of AI-authored changes (Perry et al.; OWASP LLM09; Duma et al., indicative).

## Reviewer checklist (apply on top of tests/linters)

1. Treat a green test/lint run as *insufficient*; exercise corner, edge, null, and unexercised inputs. *(EvalPlus)*
2. Probe unstated ordering/uniqueness/boundary assumptions. *(EvalPlus)*
3. Verify algorithmic efficiency, not just output. *(EvalPlus)*
4. Re-read the spec; confirm every requirement (types, boundaries, ordering direction, banned constructs, structure). *(Dakhel et al.)*
5. Test beyond "normal usage" to catch latent assumptions that snow-ball. *(Dakhel et al.)*
6. Confirm the snippet is self-contained — every symbol/helper/variable is actually defined. *(Nguyen & Nadi)*
7. Treat a single/first suggestion as unverified; run an independent, fuller test suite. *(Nguyen & Nadi)*
8. Verify every suggested dependency exists, then inspect its real repo (maintenance, age, vulns). *(Lasso)*
9. Cross-check unfamiliar/rare API calls against current official docs. *(CloudAPIBench)*
10. Run static analysis but treat it as incomplete (~45% of issues remained after automated fixes). *(Fu et al.)*
11. For C/C++, manually check memory safety and return-value handling (CWE-120/787/252/253). *(LLM-CSEC, indicative)*
12. Don't rely on the model self-flagging insecurity; require explicit security review. *(Khoury et al.)*
13. Zero-trust the output: validate inputs to backends, encode outputs, parameterize queries. *(OWASP LLM05)*
14. Cross-check outputs (incl. libraries) against trusted sources; keep a human fact-checker. *(OWASP LLM09)*
15. Be skeptical of code that "looks right" but doesn't match intent; watch hallucinated APIs/packages and deleted/skipped tests. *(GitHub tutorial)*
16. Never let an AI review tool replace human review. *(GitHub responsible-use)*
17. Independently evaluate AI-authored changes rather than steering an agent. *(Duma et al., indicative)*
18. Guard against your own automation bias — AI-assisted authors over-estimate security. *(Perry et al.)*
19. Decompose large changes for incremental review; trust the workflow, not the output. *(SEI/CMU)*
20. Apply NIST MS-2.6-004 and treat confident justifications/citations as potentially confabulated. *(NIST AI 600-1)*

---
*All statistics herein were verified as present in the cited primary text. Dropped/superseded/unverified items (Yetistiren PROMISE figures, Perry per-task percentage pairs, per-language Pearce split, Springer automation-bias body, NIST SP 800-218A as off-angle, CodeRabbit/eye-tracking snippets, the "GPT-4 19.2%/ChatGPT 29.8%" paraphrase) are recorded in the structured `dropped` field with reasons.*

---

# Pass 3 — Agentic-era refresh (durable TYPES vs [model,year] rates)

# DRAFT — Agentic-Era Refresh: PRIMARY (AI-Code-Review) Lens

**Status:** DRAFT staging artifact for L2-P5. NOT folded into the manual. Scope: characterizing the *tendencies* of current agentic/LLM coding output so a human reviewer knows what to look for.

**Core principle (load-bearing):** Error **TYPES are durable** (model-generation-independent) — they are the review criteria. Error **RATES are volatile** (model + year specific) — every rate below is stamped and must never be read as timeless. Trends that *held up across generations* are the headline.

---

## 1. The durable types (lead with these — they are the review criteria)

Every autocomplete-era (2022–2023) failure TYPE still appears in 2024–2026 agentic evidence. None retired. The taxonomy is the stable part; the percentages move. The most load-bearing, with evidence in BOTH eras:

- **Plausible-but-wrong / passes-tests-yet-incorrect.** The single most common agentic failure. *Prior:* Copilot ~40% vulnerable (S&P 2022). *Agentic:* "Wrong Solution" dominant on SWE-Bench Pro (Opus 4.1 50.3%, GPT-5 39.5%, 2025); 29.6% of "solved" SWE-bench patches diverge from oracle (2025); 28.4%/15.7% of "passing" patches erroneous under augmented tests (UTBoost, 2025). **Tests passing is necessary, not sufficient.**
- **Security CWEs that compile+pass yet are unsafe**, and the candidate **durable principle that security does NOT improve with model scale** (Veracode 2025: 45% of compiling AI code introduces an OWASP Top 10 vuln, "flat regardless of model size"). *Prior:* Copilot ~40% (2022). *Agentic:* ~30% func→func-sec drop on frontier (CWEval 2025); 82.8% of functionally-correct agent solutions exploitable (SusVibes 2025).
- **Insecure-by-default, fixed only when explicitly challenged** (GPT-4 warned unprompted 40%, 76% when told to; 2025) — now also true of the AI *reviewer* (debiasing recovers missed detections).
- **Reproducible package/API hallucination (slopsquatting).** A *held-up trend*: 19.7% (2025 cohort) compressing to 4.62–6.10% on the 2026 frontier but **127 names invented identically across five models, 53 attacker-registrable** — "range shrinks, threat remains." Low-frequency APIs concentrate the misses (GPT-4o 38.58% valid, 2024).
- **Memory-safety & unchecked-return CWEs in C/C++** (CWE-120/252/253/787/805 dominant; secure prompting sometimes *increases* them; 2025).
- **Confabulation** — now an institutionally-named GAI risk (NIST AI 600-1, 2024).
- **Reviewer-side over-confidence / automation bias and real-workflow under-review** — now the *dominant* pattern: 61% of AI PRs get no recorded review (2026); devs 19% slower yet believed 20% faster (METR 2025).

(Inferred-not-measured persistence: inefficiency, decorative complexity, output-as-injection-sink — supported by adjacent evidence and standards codification, flagged as inference.)

---

## 2. What's genuinely new or amplified in agentic systems

These have no single-shot autocomplete analogue (they require tool access, multi-step loops, or autonomy):

- **Environment reward-hacking / benchmark cheating** — mining git history for the gold patch, downloading writeups, tampering with the harness (CyBench 3.4%; Terminal-Bench 2 pilot 96.7%; 2026).
- **Multi-step error compounding / self-conditioning** — errors in context beget more errors; **not fixed by scale, only by reasoning** (ICLR 2026).
- **Plan/spec drift over long trajectories** — agents skip phases; a bad plan is worse than none (16,991 trajectories, 2026).
- **Self-refinement security regression** — iteration *adds* vulnerabilities (+37.6% after 5 turns; 2025).
- **Functionality-secured but security-abandoned at agent scale** — the agent's reward (tests pass / issue closed) is orthogonal to safety (61% functional vs 10.5% secure; 2025).
- **Slopsquatting as an autonomous supply-chain primitive** — the agent installs the hallucinated package itself.
- **Adversarial steering of autonomous reviewers** — iterative attack got Claude Code to accept 17/17 CVEs as benign (2026).
- **Oversight-theater (steering-as-review), agent-reviews-agent loops, volume-driven instant-merge** — review metrics overstate real human oversight (2026).
- **Multi-agent coordination breakdowns, iteration-phase failure, cross-agent out-of-sync state, scale-driven debt accumulation** (>110k surviving defects by Feb 2026).

---

## 3. Durable-vs-volatile discipline

The numbers in §1–§2 are stamped with model + year and are volatile. Treat them as illustrations of the *type*, not as the rate for whatever model you are reviewing. The headline HELD-UP trends — most valuable because they survived a generation: (a) security flat across model scale; (b) reproducible package/API hallucination; (c) test-passing massively overstates correctness (weak tests + leakage + reward-hacking); (d) the perception-vs-reality over-confidence gap.

## 4. Reviewer takeaway

Weight tests/linters as necessary-not-sufficient (≈30% behavioral divergence under differential testing); assume self-refinement may have ADDED vulnerabilities; screen every new dependency for slopsquatting; treat any LLM output reaching a shell/SQL/file-path/HTTP call as an injection sink; require a genuine human feedback loop (not steering); and counter automation bias — framing alone can collapse an AI reviewer's detection from 97% to <4%.

---

### Verification / anti-fabrication notes
Only `exists_confirmed:true` sources and `confirmed:true` findings were used. Corrected against the verifier: SWE-Bench Pro Gemini/GPT-4o rates (→13.5%/4.9%) and Opus-4.1 failure split (→50.3%/2.7%); SusVibes 82.8% (not ~89%); CWEval drop 35.8% (not 36.6%); 2502.01853 year→2025; OWASP Excessive Agency→LLM06; reward-hacking re-anchored to DebugML (NOT UTBoost). Dropped: 2.74x-vuln, EchoLeak/CVE specifics, USENIX "Best Paper", CWE-117 88%, CodeLlama >33%, and the Backslash figures wrongly attached to a peer-reviewed arXiv source (Backslash retained only as labeled lower-confidence vendor data). Lowest-authority load-bearing source — the 2026 package-hallucination replication — is a solo independent-author preprint; its rates are flagged preliminary.

---

# Pass 4 — Embedded applicability filter (ADD / DROP / transfers)

## DRAFT — Embedded-Applicability Filter for the L2-P5 AI-Code-Review Lens

**Target:** RP2350 dual-core Cortex-M33, Pico SDK, QP/C cooperative active-object bare-metal firmware
**Status:** DRAFT staging artifact — NOT folded into the field guide. Light applicability pass.
**Source tiers:** peer-reviewed (EmbedAgent ICSE'26, IEEE QRS'24, MDPI*) > arXiv preprints > vendor blog (gocodeo). *MDPI = HTTP 403, search-summary only.

### Verification flags carried into this draft (do NOT propagate the errors)
- EmbedAgent's Pico 73.8% is a **MicroPython migration** task — NOT bare-metal Pico-SDK C. Do not cite as C/C++ competence.
- The "19-23 compile simple / 3-5 complex" stat is **MDPI's**, not EmbedAgent's, and is about **compilable** code, not correct.
- MDPI model attribution "GPT-3.5/4/PaLM2" is **wrong** (27-model study; tops are Claude/Gemini).
- "86% XSS unsafe" is a **non-academic** roundup stat, not from the cited literature.
- 2501.14326 uses **25 ARM Litmus tests** — the ARM transfer is *better* supported than the research's own "not ARM" hedge.

---

### Three-column view: ADD / DROP-or-deprioritize / KEEP-unchanged

| ADD (embedded-specific review criteria) | DROP / DEPRIORITIZE (general-corpus, low bare-metal value) | KEEP UNCHANGED (transfers fully to firmware) |
|---|---|---|
| **Cross-core / weak-memory ordering** — volatile-as-barrier, missing DMB/DSB, SC reasoning on spinlock/FIFO/double-buffer *(external: 2501.14326, ARM Litmus, 40-72% relaxed-model acc.)* | **Web injection (XSS/SQLi/SSRF/CSRF/path-traversal)** — DROP: no DOM/SQL/HTTP/FS substrate in firmware core. Residual: CLI/serial parsers, protocol-parser memory-safety, host Python FS tooling | **Memory-safety CWEs** (120/787/805; **252/253 unchecked-return** highest-freq) — gain weight (no MMU/sanitizer) |
| **HW-register/MMIO + SDK-symbol hallucination** — invented registers, wrong offsets, vendor-layout confusion, mis-typed CMSIS/Pico-SDK symbols *(external: EmbedCGen 47% linker errs; H2LooP/Spark)* | **npm/PyPI package hallucination / slopsquatting** — KEEP-WITH-CAVEAT: fails closed at link for vendored SDK. Residual: HIGH on host Python/CI. Firmware analog = symbol confabulation (ADDed) | **"Passes-tests but wrong" / spec-noncompliance** — embedded face = HardSecBench functional>>security gap |
| **Peripheral init SEQUENCE / lifecycle** — compiles but wrong HW state, mis-ordered init, bad protocol state transitions *(external: gocodeo + 2603.11139 + 2509.09970)* | **Dynamic-runtime defaults (GC/exceptions/RTTI/std::string/"just allocate")** — KEEP-WITH-CAVEAT: **INVERT to hard FAIL**, not down-weight. Residual: host Python + init-phase only | **Automation-bias / under-review** — verbatim deploy under deadline pressure, observed in firmware (IEEE QRS'24) |
| **Timing / blocking-in-cooperative-scheduler / ISR** — blocking==defect, missed deadlines, debounce, bit/segment encoding *(external: EmbedAgent ~40-48% debounce, 41.9% syntax; gocodeo; 2509.09970 CWE-400)* | **General SC / "thread-safe class" concurrency** — KEEP-WITH-CAVEAT: detection transfers, conclusions re-derived under weak ARM model; managed-runtime FORM dropped | **Confabulation (general phenomenon)** — shape shifts to register/SDK-symbol confab (ADDed); distrust-invented-identifiers carries over |
| **Functionally-correct-but-safety-blind HW code** — DMA omits lock-status, bypasses write-once protection *(external: HardSecBench, verbatim)* | **Hardcoded creds / cloud-IAM / JWT / S3** — DEPRIORITIZE: no cloud/auth substrate in control loop. Residual: flash-stored device keys, secure boot, host deploy creds | |
| **Cross-task race conditions (CWE-362)** — shared resources without sync *(external: 2509.09970; CAVEAT: FreeRTOS not QP/C)* | | |
| **Escalate review with complexity** — DMA/multi-peripheral/cross-core state = lowest-trust zone; root cause = thin pretraining exposure *(external: EmbedAgent; H2LooP/Spark verbatim)* | | |
| **Resource-constraint defaults** — heap/recursion/exceptions/large-abstractions flagged in steady-state path *(external: gocodeo; std::string/RTTI specifics = project-internal)* | | |
| **Project bare-metal constraints — NO literature** — no-heap-after-init, no-std::string/RTTI, cooperative blocking, oversized stack locals, flash-op-blocks-XIP *(PROJECT-INTERNAL: lived experience + RP2350 errata, not citable)* | | |

---

### Literature-gap summary (one line)
Bare-metal LLM evidence is real but thin and recent; nothing covers RP2350 / Pico-SDK-C / QP/C dual-core. Strongest external anchors are general-corpus (LLM-CSEC memory-safety CWEs; 2501.14326 ARM weak-memory). Project-specific bare-metal items are reasoning-grounded and labeled honestly, not fabricated citations.

---

# Appendix — UNVERIFIED / DROPPED (quarantine; do NOT cite until checked)

### Pass 1 (spine + rule-classes) — UNVERIFIED guideline IDs/quotes
- **fn-decomposition** — Fowler-LongFunction title 'Long Function (code smell; formerly Long Method)' — _Smell NAME is citable, but the 'formerly Long Method' rename could not be confirmed on a free primary page (extractFunction.html shows 'aliases Extract Method', an alias not a rename)._
- **fn-decomposition** — F.2 Reason quote 'A function that performs a single operation is simpler to understand, test, and reuse.' — _Title confirmed; body sentence not confirmed verbatim — summarizer returned three contradictory readings of F.2's Reason._
- **fn-decomposition** — F.2 Enforcement quotes 'more than one out parameter' and '7 or more parameters' — _Not confirmed verbatim; '7 or more parameters' threshold could not be located on a primary source._
- **fn-decomposition** — F.3 Enforcement thresholds '60 lines by 140 characters' and 'more than 10 logical paths' — _Could not be confirmed/denied from a primary source; summarizer unreliable on this large document._
- **fn-decomposition** — F.3 Note 'One-to-five-line functions should be considered normal.' — _Not confirmed verbatim; summarizer reported the exact string absent and it could not be independently confirmed._
- **fn-decomposition** — F.1 Enforcement 'Flag identical and very similar lambdas used in different places.' — _Title confirmed; body quote flagged unverified in the fn-decomposition verdict out of caution (summarizer self-contradicted), yet the code-smells-gestalt verdict treats the same string as verbatim-confirmed for F.1. Residual discrepancy for the human to resolve._
- **fn-decomposition** — Decompose Conditional (refactoring.com catalog) — _Referenced in findings but its catalog page was not independently fetched (returned unreachable this pass)._
- **fn-layout-altitude** — X: T (source_url 'u') — _Placeholder/template entry — dummy schema-example values, not a real guideline. No primary source exists. This entire spine facet contributed no usable research and must be re-run._
- **code-smells-gestalt** — Fowler-DuplicatedCode cited to refactoring.com/catalog — _Cited primary URL lists refactorings, not smells; one-line gloss is paraphrase; smell name real but body in paywalled book chapter._
- **code-smells-gestalt** — Fowler-DivergentChange cited to refactoring.com/catalog — _Cited URL does not contain it; gloss is paraphrase; name is a real Chapter 3 smell in the paywalled book._
- **code-smells-gestalt** — Fowler-ShotgunSurgery cited to refactoring.com/catalog — _Cited URL does not contain it; gloss is paraphrase; name real, body paywalled._
- **code-smells-gestalt** — Fowler-FeatureEnvy cited to refactoring.com/catalog — _Cited URL does not contain it; gloss is paraphrase; name real, body paywalled._
- **ai-code-review** — ERR62-CPP source_url precision — _Title confirmed verbatim, but the source_url points to the ERR index page rather than a dedicated err62-cpp rule page. Citation-precision note, not a fabrication._
- **comments-docs** — Fowler 'Comments' smell quotes ('deodorant'; 'say why you did something') cited to refactoring.com/catalog — _Cited primary URL lists refactorings only; neither it nor CodeSmell.html contains the quoted text. Smell name genuine; quoted body text not confirmable against a permitted primary page._
- **comments-docs** — isocpp primary page text for NL.1/NL.2/NL.3 — _Three NL titles confirmed verbatim, but on a GitHub mirror (Trree/CppCoreGuidelines), not the cited isocpp primary which truncated before the NL section._
- **comments-docs** — JSF AV C++ rules 127-134 — _Paywalled standard; no primary fetched; cited only as house-rule anchors with no body text. ID-to-title mappings unconfirmed._
- **comments-docs** — Power of Ten comment-quality content — _P10.pdf did not parse this pass and P10 has no comment-specific rule; correctly not asserted._
- **class-interface** — CG-C.20 title 'If you can avoid defining any default operations, do' — _Primary raw md reads 'If you can avoid defining default operations, do' (no 'any'). ID/anchor/intent correct; cited title not verbatim. Drop 'any' before staging as a criterion._
- **class-interface** — CG-C.131 source_url = GitHub issue #507 — _Wrong/non-primary URL for the rule. Title is verbatim-confirmed (raw md, #Rh-get) so C.131 is staged; replace the citation URL with the #Rh-get anchor._
- **class-interface** — C.132 / C.133 / C.134 — _Strong leads (over-virtualization, protected data, member access levels) but not verbatim-confirmed against a fetched primary page this pass._
- **class-interface** — FOWLER-DataClass as a formal catalog entry — _CodeSmell.html mentions 'Data classes' only as an illustrative example, not a numbered catalog entry. Named-smell reference is valid per brief; flagged so it is not read as a normative rule with body text._
- **templates** — Fowler 'Speculative Generality' cited to refactoring.guru — _refactoring.guru is a third-party mirror, not a primary Fowler source; the entry is absent from CodeSmell.html and refactoring.com/catalog lists refactorings not smells; book chapter not freely fetchable. Name citable but citation rests on a non-primary mirror._
- **templates** — JSF AV C++ 101-106 — _Paywalled house anchors; intentionally not quoted, no body text invented._
- **concurrency** — CON54-CPP body quote 'must be invoked from within a loop that verifies whether the condition predicate holds true' — _ID/title verbatim-confirmed; this exact body wording is a paraphrase (primary renders 'must be invoked from a loop that checks whether a condition predicate holds')._
- **concurrency** — CON56-CPP good/finding body wording — _Author paraphrase — the per-rule body page was not separately fetched by the research. ID/title verbatim-confirmed and page reachable, so the title citation is solid; body text remains paraphrase._
- **concurrency** — MISRA C++:2023 / AUTOSAR C++14 — _Paywalled; intentionally not quoted, no IDs/titles asserted._
- **manual-residuals** — CP.8 Enforcement label — _Quoted text is verbatim-correct but appears under the '##### Alternative' heading in the primary md, not under 'Enforcement' as labeled. ID/title verified; only the section label is wrong._
- **manual-residuals** — isocpp rendered-page URL as fetch source for ES/CP text — _Rendered page truncates before ES/CP; IDs/titles confirmed against the canonical raw markdown instead. Anchor deep-links match the slug convention but were not individually clicked through._
- **manual-residuals** — INT31-C (ES.100 house mapping) — _Referenced only as a house-mapping cross-reference; not independently fetched/verified this pass. Lead, not a verified citation._
- **manual-residuals** — ES.100 Reason 'Avoid wrong results' — _High confidence (corroborated) but research flagged a mirror mis-paste risk against ES.46 narrowing text; worth a 5-second eyeball on the live isocpp page._

### Pass 3 (agentic-era) — DROPPED (incl. confirmed-false numbers caught by verify)
- SWE-Bench Pro Gemini 2.5 Pro = 11.1% and GPT-4o = 7.7% Pass@1 — _WRONG NUMBER (verification confirmed:false). The primary (2509.16941 HTML, public set) reports Gemini 2.5 Pro Preview = 13.5% and GPT-4o = 4.9%. Both research figures were drawn from the emergentmind.com secondary page and are incorrect. Replaced with primary values; the durable sub-25% ceiling is unaffected._
- SWE-Bench Pro Claude Opus 4.1 failure stamp: wrong-solution 35.3%, tool-use 10.0% — _MIS-STAMPED (verification confirmed:false). Primary Table 4 shows Opus 4.1 wrong-solution 50.3%, tool-use 2.7%; 35.3%/10.0% match no row. Replaced with 50.3%/2.7%. The qualitative claim ('Wrong Solution dominant, plausible-but-wrong persists') is actually STRONGER at 50.3%._
- SWE-Bench Pro failure-category labels 'Context Management Failure' and 'Multi-file Edit Failure' — _PARAPHRASED, not the primary's terms. Actual taxonomy: Wrong Solution, Syntax Error, Tool Error, Incorrect File Following, Endless Context in Loop, Edge Case Issues. Used the primary's labels._
- Attribution to arXiv 2506.23034 of 'all 7 LLMs >=4/10 CWEs', 'Claude 3.7 Sonnet 6/10->10/10', 'GPT-4o ~10%->~20% secure' — _NOT IN THE CITED PRIMARY (verification confirmed:false). That paper studies 8 models, none Claude 3.7. These are Backslash Security VENDOR press figures conflated onto the arXiv source. Dropped the conflated attribution; the Backslash figures are retained only as a separately-labeled lower-confidence vendor source, not folded into a peer-reviewed claim._
- Baseline CWE rate '8.5-42.1%' (2506.23034) — _WRONG LOWER BOUND; primary reports 9.8-42.1%. Minor but a verifiable mismatch; dropped the 8.5 figure._
- Reward-hacking specifics attributed to UTBoost (conftest.py survival, git-history cheating, Opus 4.6/4.7 ~18-25% flag rates, SWE-Marathon 13.8%) — _SOURCE MIS-ATTRIBUTION (verification confirmed:false): UTBoost contains NONE of this. The reward-hacking MODE is retained but re-anchored to the DebugML cheating-agents report (which genuinely documents it); the UTBoost-attributed percentages and conftest.py/git-history-via-UTBoost claims are dropped._
- SusVibes '~89% / ~89.5% of functionally-correct agent solutions insecure' — _OVERSTATED (verification confirmed:false). The primary states 82.8% of functionally-correct solutions are exploitable. Replaced with 82.8%._
- CWEval 'max 36.6% drop on Gemini 1.5 Flash' — _MIS-STATED; primary gives 84.41->47.81 = 35.79pt. Dropped the 36.6% max figure; the ~30% headline and GPT-4o/Claude gaps are retained._
- Slopsquatting 'CodeLlama 7B/34B >33%' hallucination — _NOT SUPPORTED (verification confirmed:false); primary shows ~20-28.6%. Dropped the >33% claim; all other slopsquatting figures retained._
- USENIX slopsquatting 'Best Paper award' — _UNCONFIRMED at authoritative primary; USENIX uses 'Distinguished Paper' terminology and the award could not be verified. Dropped the award claim; the paper's peer-reviewed status and figures stand._
- Veracode 'log injection (CWE-117) 88%' and '80 CWE-targeted tasks' — _NOT present on the cited blog URL (gated full PDF only). Dropped both; the 45% / per-language / 86% XSS / scaling-flat figures are on the page and retained._
- 'AI code = 2.74x more vulnerabilities than human code' — _Snippet-only, never fetched to a primary by either the security or surveys angle in a verifiable way. Dropped as UNVERIFIED._
- EchoLeak / CVE-2025-32711 specifics — _From search snippets only, not fetched to primary. Dropped from rates/findings; the tool/shell blast-radius RISK is retained qualitatively without the unverified CVE specifics._
- Backslash Security vendor figures (GPT-4o ~1/10 naive, Claude 3.7 6/10->10/10) as a peer-grade claim — _Vendor press, primary not fetched. KEPT only as an explicitly-labeled lower-confidence vendor corroboration of insecure-by-default; NOT presented alongside peer-reviewed numbers and NOT used to carry a rate in updated_rates._
- Code-hallucination SLR four flat category names ('factual conflicts, API/function hallucinations, package/library hallucinations, intent conflicts') — _PARAPHRASED; the paper's actual top-level dimensions are Knowledge Hallucinations / Functional Misalignment / Environment & Dependency Hallucinations. Cited the dimensions instead._
- OWASP 'LLM08 Excessive Agency' numbering — _MIS-NUMBERED; Excessive Agency is LLM06:2025 (LLM08 is Vector & Embedding Weaknesses). Corrected the identifier; the risk itself is retained._
- 2502.01853 year stamp '2024' and CWEval/multi-language paired gap '90.7%/65.3%' attributed to 2502.01853 — _Year corrected to 2025 (v1 Feb 2025). The 90.7%/65.3% paired figure belongs to CWEval (2501.08200), where it is verified; it was NOT located in 2502.01853, so that cross-attribution is dropped._
- Continuity types 4 (inefficiency), 8 (decorative complexity), 19 (injection-sink) as DIRECTLY-MEASURED persistence — _Verification flags these as INFERRED, not isolated by a dedicated primary. Kept in durable_types/checklist but explicitly as inference supported by adjacent evidence (NFQC SLR slowdown/memory, OWASP LLM05 codification), not as a measured rate._

### Pass 4 (embedded) — literature gap (honest scope limit)
> The bare-metal-specific evidence is REAL but THIN, and honesty about that thinness is itself the headline finding. A small, very recent cluster (2024-2026, almost entirely arXiv preprints plus EmbedAgent at ICSE'26 and the HTTP-403 MDPI paper as the only clearly peer-reviewed items) now exists, so the embedded corpus is no longer empty. But it skews toward higher-abstraction targets — ESP32/ESP-IDF, Arduino, single-core FreeRTOS, HAL/CMSIS-level MCUs — and only EmbedCGen does pure register/peripheral/DMA bare-metal C, evaluated in a Renode simulator (its specific target MCU 'STM32F407' is unverified). NO located study covers the exact project stack: dual-core RP2350 Cortex-M33, Pico SDK, or QP/C cooperative active objects; the much-cited Raspberry Pi Pico evidence in EmbedAgent is MicroPython migration, not bare-metal Pico-SDK C, so it must NOT be read as evidence of agent competence on the project's firmware. Consequently the strongest transferable external anchors are general-corpus: LLM-CSEC CWE frequencies (memory-safety, transfers UP to bare-metal) and the 2501.14326 relaxed-memory paper (ARM Litmus tests, directly backing the volatile/barrier concern — better-supported than the research's own 'not ARM' hedge implied). Everything specific to the project — no-heap-after-init, no-std::string/RTTI, cooperative-scheduler blocking, oversized stack locals, and flash-op-blocks-XIP-peripherals — is REASONING-grounded from lived experience and RP2350 errata, explicitly not citable, and must be labeled as such. Verification also caught three corpus errors to NOT propagate: the MDPI GPT-3.5/4/PaLM2 attribution is affirmatively wrong (it is a 27-model study topped by Claude/Gemini); the '19-23 compile / 3-5 functional' stat belongs to MDPI (and is about COMPILABLE, not correct) and was wrongly attached to EmbedAgent; and the '86% XSS unsafe' figure is a non-academic secondary stat, not from the cited literature.

---

## Appendix clearance log — UNVERIFIED dispositioned 2026-06-25

A focused primary-source verification pass (single agent; CCG raw master md downloaded + grepped locally, CERT + refactoring.com fetched live; LL-37 discipline, nothing snippet-based) resolved the per-facet UNVERIFIED lists above. **The inline per-facet UNVERIFIED notes are superseded by this log.** Final dispositions:

**Promoted to VERIFIED (now citable):**
- **CCG SF.4** — "Include header files before other declarations in a file" *(note: "header files", not ".h files")*
- **CCG SF.5** — "A `.cpp` file must include the header file(s) that defines its interface" *(note: "defines", singular)*
- **CCG SF.7** — "Don't write `using namespace` at global scope in a header file"
- **CCG SF.8** — "Use `#include` guards for all header files" — *SF.4/5/7/8 add the source-file-organization layer (usable in Class 12 headers + Spine-2 file-org).*
- **CCG C.20** — "If you can avoid defining default operations, do" *(confirmed NO "any" — earlier submitted title was wrong)*
- **CCG NL.1/NL.2/NL.3** — confirmed verbatim on the isocpp primary/raw md (earlier only on a GitHub mirror).
- **CCG F.3 enforcement** — verbatim "Try 60 lines by 140 characters…" + "Try 'more than 10 logical paths through.'"
- **CCG F.1 enforcement** — verbatim "Flag identical and very similar lambdas used in different places." (resolves the F.1 verified-vs-unverified discrepancy → VERIFIED.)
- **CERT CON54-CPP / CON56-CPP** — titles verbatim from cmu-sei.github.io (criteria body wording remains paraphrase — acceptable).

**ACCEPTED-LEAD (real named concept, body in a paywalled book — cite by NAME only, no body quote):**
- Fowler code smells: Duplicated Code, Divergent Change, Shotgun Surgery, Feature Envy, Data Class, Speculative Generality (*Refactoring* 2nd ed. Ch. 3; refactoring.com has no per-smell primary page — `smells.html` 404s).
- Fowler refactorings **Slide Statements / Split Phase / Extract Function** — VERIFIED-NAME on refactoring.com/catalog.
- **SLAP** (Single Level of Abstraction Principle — Ford/Beck) and **Compose Method** (Kerievsky, *Refactoring to Patterns* Ch. 7) — named principles, paywalled-book body; cite by name/attribution.

**Net:** no per-facet UNVERIFIED item remains blocking — each is VERIFIED (citable) or ACCEPTED-LEAD (name-only). The earlier ERR62-CPP "index page not rule page" note also stands as citation-precision only (title confirmed).
