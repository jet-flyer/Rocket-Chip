# RocketChip Audit Guidance — Master Procedure

**Status:** Normative master audit procedure.
**Purpose:** Single authoritative document that sequences every audit type into one lifecycle. The core checklist is concise (≤1 page). Manual audit guidance lives in concise appendices.

**Procedure history:** Refactored 2026-05-13 from 8-step flat checklist to 7-tier dependency-ordered structure. Reasons: (a) gates-honest discipline was implicit, leaving the LL Entry 36 pattern (gate rot accepted as PASS) without a procedural defense; (b) standards-catalog and runtime-behavior tiers sat at equal depth without dependency ordering; (c) sibling audits (BUILD_SYSTEM_AUDIT, DEV_CODE, VERSION_STRING, AO_COMMANDMENTS, TOOLCHAIN, CLA-RBM) had no enumerated home. Council-reviewed 2026-05-13 (NASA/JPL Avionics Lead, ArduPilot Core Contributor, Embedded Systems Professor, Cubesat Startup Engineer); CONSENSUS APPROVE WITH AMENDMENTS — 8 amendments incorporated. Step-to-Tier mapping in CHANGELOG entry dated 2026-05-13. Source attribution per Appendix C.9 (DO-178C problem-report lifecycle + static-analysis-tool community / SonarQube merge-blocker-vs-backlog + refactoring research on dependency cycles + NASA SWE §8.5 (Software FMEA) severity scale + project-internal LL Entry 36 gate-integrity-first lived experience).

---

## Master vs Targeted Audits

- **Master Procedure (`AUDIT_GUIDANCE.md`)**: The unified lifecycle checklist. Defines *when* to run what scope. Not a daily checklist itself.
- **Targeted / Operational Audits**: The actual focused work (coding standards, pre-flight gate, FMEA, stack review, etc.). These are invoked by tiers in the master procedure.

Use the decision table below to choose the right scope. Use the targeted documents to perform the work.

---

## When to Do What – Audit Scope Decision Table

| Trigger                                | Scope                          | Specific Tiers / Targeted Audits to Run                                       | Positive-Control Expectation                     | Notes |
|----------------------------------------|--------------------------------|-------------------------------------------------------------------------------|--------------------------------------------------|-------|
| Every commit (small change)            | Incremental / targeted         | Tier 3 (coding standards) sub-walks on **changed files only**                 | clang-tidy clean on staged files                 | Pre-commit hook is Tier 1.3 mechanism |
| Before push                            | Build parity + relevant targeted | Tier 2.6 (build-parity) + Tier 3 sub-walks on modified modules              | All 4 build tiers compile + standards pass       | See `SESSION_CHECKLIST.md` item 6 |
| Before field test / launch             | Pre-flight gate + safety review | Tier 5.1 (`PRE_FLIGHT_CHECKLIST.md`) + Tier 4 if scope warrants              | All GO verdicts + positive-control signals       | Highest safety bar |
| Milestone / stage close                | **Full master procedure**      | All 7 tiers                                                                   | Complete set of positive-control outputs         | Super audit |
| Safety-critical or architecture change | Targeted deep review           | Tiers 3 + 4 + 5 **scoped to affected modules**; Tier 1 mandatory regardless    | Positive-control signals on changed paths        | Do not re-audit untouched areas; Tier 1 always runs |
| After failed audit or remediation      | Affected portions + regression | Re-enter at the tier the failure originated; later tiers re-run if affected   | Previously failed items now PASS                 | Severity gate per stop conditions |

**Principle:** Use the smallest sufficient scope. The full master procedure is reserved for milestone/stage-close events.

**Severity gate (council amendment #3, applies to all stop conditions below):** stop-conditions fire on NASA SWE §8.5 (Software FMEA) **Catastrophic or Critical** findings only. Major / Minor / Trivial findings flow forward to Tier 7 disposition without re-walking earlier tiers. Without this gate, the procedure becomes a tar pit — every minor finding triggers re-walks and the audit never closes.

---

## Master Audit Procedure — 7 Tiers

Run the full procedure only at milestone / stage close. For all other triggers, use the decision table above.

Each tier closes before the next begins. Within a tier, items can run in parallel. Stop conditions are topological back-edges, severity-gated per the rule above.

### Tier 1 — Verify the Audit's Own Gates

Before measuring anything, confirm the measurement instruments are honest. Source: LL Entry 36 (bench_sim rot — gate had been silently broken for 5 days while 4+ commits claimed it as PASS) + Appendix C category 1 + DO-330 tool-qualification.

1.1 Tool self-checks: `run_clang_tidy.sh`, `run_cppcheck.sh`, lizard, coverage harness produce expected output on known-good fixture inputs.
1.2 SPIN models compile + load on current `spin.exe` (Cygwin per `tools/spin/README.md`).
1.3 Pre-commit hook integrity: `scripts/hooks/pre-commit` runs on a synthetic staged diff and produces the expected gate matrix output. Mechanism: `scripts/audit/pre_commit_fixture_test.py` (exercises `scripts/ci/pre_commit_matrix.py` against a synthetic-staged-diff fixture table covering known-good / known-bad / no-fire / edge cases — exit 0 = all rows pass).
1.4 `bench_sim.py` **bidirectional regex audit** (council amendment #1):
    - (a) Rotted-regex check: every regex constant matches at least one live firmware log line from a freshly-flashed banner read.
    - (b) Deleted-regex check: every PASS-token / positive-control signal the firmware emits has at least one regex in the script looking for it. Mechanism: `scripts/audit/list_bench_sim_pass_tokens.py` (greps src/ for emitted positive-control tokens, groups by class, surfaces the inventory at `logs/bench_sim_pass_tokens.txt` for diff-against-prior-cycle comparison; empty token classes appear as NOTE-level signals for review).
1.5 Host ctest fixture-suite passes.
1.6 Build-parity script (`verify_build_parity.sh`) produces clean output for all 4 tiers.

**Stop condition:** any Tier 1 failure halts the cycle until the tool/gate is repaired (no severity gate — broken instruments mean every downstream finding is unreliable by construction).

### Tier 2 — Establish the Foundation

With instruments verified honest in Tier 1, capture the baseline data the rest of the audit reasons from. Source: Appendix C category 2 + traditional "baseline scripted sweeps" framing.

2.1 Baseline scripted sweeps: `run_clang_tidy.sh`, `run_cppcheck.sh`, `analyze_stack_usage.sh`, `generate_coverage_report.sh`, lizard cyclomatic complexity sweep. Record verdicts + raw output paths.
2.2 SPIN model verification: re-run all `tools/spin/*.pml` against current firmware. Master gate `run_stage_o_ao_spin.sh` produces SPIN_OK_N.
2.3 Toolchain version audit: walk `docs/BUILD_SYSTEM_AUDIT.md` P6 (current versions vs latest upstream, drift assessment). Absorbs `TOOLCHAIN_VERSION_AUDIT_*.md` sibling audit. Mechanism: `scripts/audit/check_toolchain_drift.py` (mechanically pulls upstream-latest versions for Pico SDK, picotool, OpenOCD Pi-fork branch HEAD, Pico Probe firmware, CMake; reads project-pinned versions from `CMakeLists.txt`; reads local installed versions from `~/.pico-sdk/<component>/`; produces drift table at `logs/toolchain_drift.txt`. Graceful degradation on network failure — does not block the cycle).
2.4 Build-system audit: walk `docs/BUILD_SYSTEM_AUDIT.md` P1-A through P5 (dev-tool gating, self-flagged dead code, ROCKETCHIP_SOURCES coverage, host/target split, vendor SYSTEM classification, IVP-era scaffolding comments).
2.5 Active-deviations row walk: walk `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` Active section row-by-row, confirm each row's justification still holds.
2.5a **Deferred-with-rationale row walk** (council amendment #4): walk every deferred-with-rationale row across the audit machinery (LOC-5/LOC-6 MISRA-C deferral; any DEFER row in active dated audit reports' `## Remediation` sections; deferred-with-safety-impact rows in `docs/PROBLEM_REPORTS.md`). Three cycles without re-evaluation = stale-rationale threshold; flag for explicit user re-disposition if exceeded.
2.6 **Prior-cycle delta read** (council amendment #5, sourced from JPL F' Mars sustaining-engineering pattern): open most-recent prior dated audit report. List which findings closed, remained, were re-opened, are new. Feeds Tier 5.6 and Tier 7.

**Stop condition:** Tier 2 finding that invalidates Tier 1's verified-gates assumption AND severity Catastrophic/Critical → return to Tier 1.

### Tier 3 — Walk the Standards Catalog

Mechanical catalog work. The big static pass against published rule sets the project has adopted. Source: traditional `STANDARDS_AUDIT.md` template walk (Sections A through E + F-1).

3.1 **Section A — JSF AV C++ Standards (221 rules).** Per-rule applicability + PASS/PARTIAL/FAIL + file citations. A.1 acknowledges absent .c files. A.2 documents SDK-boundary bypasses. A.3 is the bulk of the work.
3.2 **Section B — Power of 10 (Holzmann/JPL 2006).** Per-rule applicability. 10 rules.
3.3 **Section C — JPL Institutional Coding Standard.**
    - LOC-1 through LOC-4 (102 rules — JPL-original tiers): per-rule walk.
    - **LOC-5 and LOC-6: DEFERRED-WITH-RATIONALE.** These are the MISRA-C absorption tiers (LOC-5 = MISRA "shall" rules not in LOC-1..4; LOC-6 = MISRA "should" rules). Rule text is MIRA Ltd. copyrighted, not in public JPL PDF. Project policy 2026-05-13: defer until a formally-certified-code variant is in scope. See `standards/CODING_STANDARDS.md` Foundation section for the chain-of-custody (MISRA-C → JPL C LOC-5/6 → JSF → P10 distill). Re-evaluated at Tier 2.5a each cycle.
3.4 **Section D — Project-Specific Rules.** RP2350 platform constraints, multicore rules, debug output, prior-art-research, safety-and-regulatory, git-workflow, session-management, **comment-density measurement (target band 15-25% per `CODING_STANDARDS.md`)**.
3.5 **Section E — Agent Behavioral Guidelines.** Walk `.claude/AK_GUIDELINES.md` per-rule. Deliverable: do recent commits demonstrate compliance?
3.6 **DEV_CODE audit** (per `docs/audits/DEV_CODE_AUDIT.md` methodology): dev-tier code in flight binary.
3.7 **VERSION_STRING audit** (per `docs/audits/VERSION_STRING_AUDIT.md` methodology): stale version-like values in serial output / banner.
3.8 **Section F-1 — Citations within standards documents** (council amendment #2): walk file:line references and standards-citations inside the standards documents themselves (`CODING_STANDARDS.md`, `HW_GATE_DISCIPLINE.md`, `AUDIT_GUIDANCE.md`, `RP2350_ERRATA.md`, `ACCEPTED_STANDARDS_DEVIATIONS.md`). F-2 (audit-cycle citations in non-standards docs) is Tier 6.1. **Exhaustive coverage** against the distinct-cited-source population (sampling-based walks missed R-26 in the 2026-05-07 L2 audit — see L2-P2 below). **Tooling gap:** no maintained citation inventory script exists yet (L2-P3 — could become a future audit-infrastructure F-2026-MM-DD-NNN follow-up alongside the F-001/002/003 family); the audit's own grep is the current method.

**Stop condition:** Tier 3 finding that invalidates Tier 2 baseline AND severity Catastrophic/Critical → return to Tier 2.

**Positive-control expectation:** dated `STANDARDS_AUDIT_YYYY-MM-DD.md` produced with Sections A through E + F-1 populated.

**Sampling-policy rule (L2-P2 disposition, 2026-05-13):** wherever this procedure asks for a walk against a defined population (cited sources, AO inventory, ifdef-gated dev code, version-string print sites), the default is **exhaustive coverage** against the distinct-item population, not a sample. The 2026-05-07 L2 audit demonstrated that exhaustive coverage is feasible (~23 distinct cited sources reducible from 78 cite sites) and load-bearing (R-26 was missed by the original 12-item sample). If sampling is used (e.g., because the population is too large), the report's scope-language must follow the L2-P4 disposition below.

**Report scope-language rule (L2-P4 disposition, 2026-05-13):** PASS results in dated audit reports must name the population walked. Acceptable wording: "23/23 distinct cited sources CONFIRMED" (exhaustive); "12/23 sampled cited sources CONFIRMED — 11 not sampled this cycle" (acknowledged-incomplete). Unacceptable wording: "Citations PASS" (no scope), "All citations checked" (over-claim). The 2026-05-13 master cycle's Tier 3 brevity exception specifically calls out scope language in its coverage statement.

### Tier 4 — Walk the Runtime Behavior

Examine what the code *does* at runtime, not just its static structure. Source: pre-2026-05-13 Step 4 (FMEA-lite + Koopman, now Tier 4.1-4.2) + AO Commandments audit (now Tier 4.4) + CLA-RBM (now Tier 4.5) + LL-entry freshness (now Tier 4.6).

4.1 **Safety-critical path FMEA-lite** (Appendix A.1). Maps to `docs/SAD.md` state machine. Positive-control column required per `HW_GATE_DISCIPLINE.md` Rule 1.
4.2 **Koopman 5-rule embedded review** on 3-5 flight-critical functions of agent's choice (auditor selects; surfaces conversationally per Appendix B.4 workflow).
4.3 **Fault injection scenarios**: run `enhanced_fault_injection.py` end-to-end.
4.4 **AO Commandments full sweep** (per `docs/audits/AO_COMMANDMENTS_AUDIT_*.md` methodology): walk 9 active AOs × 12 commandments in `docs/decisions/AO_COMMANDMENTS.md`.
4.5 **CLA + RBM** (per `docs/audits/cla_rbm/CLA_RBM_PLAN.md` deliverables): Computational Load Analysis + Runtime Behavior Map. If most-recent CLA data >3 months old, flag re-collection as remediation item (HW soak; can be queued to a separate session).
4.6 **LL-entry freshness sweep**: walk `.claude/LESSONS_LEARNED.md` Critical/High entries. Each entry's claim ("X is fixed in tree", "Y workaround in place") verified against current code. Stale entries get supersession headers (LL Entry 25 SUPERSEDED 2026-04-22 is the pattern).

**Stop condition:** Tier 4 finding that contradicts Tier 3 PASS for the same code path AND severity Catastrophic/Critical → re-open Tier 3 for that path.

### Tier 5 — Verify Against Requirements + Observable Behavior

Prove the running system does what its requirements say. Source: existing Steps 3 + 6 + 7.

**Independence amendment (council #7):** Tier 5 should run in a **fresh session** distinct from Tiers 1-4 where feasible. Per `HW_GATE_DISCIPLINE.md` Rule 6, same-agent verification is level-2 credit at best; a separate session is level-3 ("Verified independently"). Especially relevant before any Stage 17 (field flight) milestone.

5.1 **Pre-flight gate execution**: walk `docs/PRE_FLIGHT_CHECKLIST.md` in full. Every GO verdict + positive-control signal observed.
5.2 **Stack / Memory / RP2350 Errata Deep Review** (Appendix A.2). E2 / E9 / E11 / E12 re-verification.
5.3 **End-to-end gates**: `bench_sim.py` 2/2 PASS for vehicle (single flight binary post-R-25-exec 2026-05-13); `station_bench_sim.py` for station; `warm_reboot_audit.py` (R-22, R-25-exec step 11) for warm-reboot coverage; `verify_boot_parity.sh` (R-24, R-25-exec step 12) for cross-tier banner verification.
5.4 **Host-side replay**: `scripts/replay_harness_host.py` end-to-end. (Currently a placeholder skeleton — R-25-exec council amendment #4 deleted the on-MCU CSV-streamer in steps 5+6; host-side ESKF replay implementation is follow-on work, IVP-131 verification model shifted. Documented gap; clears when the host-side ESKF replay implementation lands.)
5.5 **Requirements traceability spot-check**: state machine, ESKF outputs, pyro commands, telemetry fields, safety flags. **Exhaustive coverage** against distinct-cited-source population (sampling missed R-26 in the 2026-05-07 L2 audit).
5.6 **Regression check on prior-cycle closed findings** (council amendment #6): every finding closed in prior cycles (from Tier 2.6 delta read) gets a one-line regression check. If R-19 closed in cycle N by removing X, Tier 5.6 in cycle N+1 confirms X still gone.

**Stop condition:** Tier 5 verification FAIL that contradicts Tier 3/4 PASS AND severity Catastrophic/Critical → return to whichever tier first claimed PASS.

### Tier 6 — Document Drift + Sync

Catch documentation that earlier tiers' work has invalidated. Last because earlier tiers may surface drift findings.

6.1 **Section F-2 — Audit-cycle citations in non-standards docs** (council amendment #2): walk file:line references + audit-cycle citations in protected state-of-system docs OUTSIDE the standards documents — `PROJECT_STATUS.md`, `docs/SAD.md`, `CHANGELOG.md` (current-cycle entry), `docs/SCAFFOLDING.md`, `docs/AO_ARCHITECTURE.md`.
6.2 **Section G — Audit History** (`STANDARDS_AUDIT.md` template). Roll up cycle's findings into audit-history table.
6.3 **Section H — Ongoing Compliance Verification** (template). Walk pre-commit hook coverage, SESSION_CHECKLIST trigger coverage, milestone-close discipline coverage.
6.4 **Protected-doc drift check** (SESSION_CHECKLIST.md item 14): grep state-of-system docs for symbols / module paths that this audit's findings have deleted, renamed, or re-homed.
6.5 **CHANGELOG audit-history rollup**: cycle's CHANGELOG entry references dated audit report + names every disposition (REMEDIATE / ACCEPT / DEFER) without ambiguity.

**Stop condition:** Tier 6 drift that invalidates a Tier 5 PASS AND severity Catastrophic/Critical → return to Tier 5.

### Tier 7 — Findings Disposition + Remediation

Existing structure preserved verbatim. Source: Appendix C (4-category remediation triage, unchanged).

Findings disposition (REMEDIATE / ACCEPT / DEFER) per finding. Per **Appendix C**, remediation execution order is: Cat 1 (Gate Integrity) → Cat 2 (Shared Foundations) → Cat 3 (Behavior Changes) → Cat 4 (Cleanup). **DEFER disposition requires a safety-impact one-liner.** A finding can be deferred only if the dated report names — explicitly — why the deferral is acceptable from a safety perspective (typically: mission-phase exposure is bounded; doc-only with no behavior change; risk dominated by an unrelated dependency that must land first). Plain "DEFER — not blocking" is not sufficient. Source: DO-178C "open problem reports at certification must be evaluated for their safety impact." Equivalent rule in `docs/PROBLEM_REPORTS.md` for non-audit deferrals.

ACCEPTED deviations require user sign-off and land in `ACCEPTED_STANDARDS_DEVIATIONS.md`. Verification cadence per Appendix C.5 (local per-commit vs. category-transition regression vs. cycle-close regression).

---

## Appendix A: Manual Audit Guides (Concise)

### A.1 FMEA-lite & Koopman Embedded Review

> **Reference material is inlined in Appendix B.** Before working this section, read **B.2 (Koopman 5-rule embedded review)** and **B.3 (FMEA worksheet columns, per NASA SWE §8.5 (Software FMEA))** for the rule wording, grep patterns, and PASS-vs-FAIL examples. **B.4 describes the agent-driven audit workflow** including how FMEA-lite results surface conversationally for user disposition.

**Scope:** Pyro arming/firing, launch-abort, disarm timeout, hung-fire, radio command validation, ESKF divergence brake. Map every item to `docs/SAD.md` states.

**Files to inspect:**
- `src/active_objects/ao_flight_director.cpp` (state machine + guards)
- `src/eskf/` (divergence detection + brake)
- `src/drivers/rfm95w.cpp` + radio scheduler (command path)
- `src/cli/rc_os.cpp` (p, a, X commands)
- `src/logging/` (pyro intent and disarm logs)

**Koopman Embedded Review (adapted, one-page focus):**
- Every safety-critical function has documented preconditions and postconditions.
- No recursion or unbounded loops in interrupt or high-priority AO paths.
- Every state transition has an explicit guard + observable positive-control signal.
- Error paths either reach a safe terminal state or are formally unreachable.
- Backup paths (watchdog, PIO WDT) are exercised in every safety-critical sequence.

**FMEA-lite Table (positive-control column required):**

| Failure Mode | Files / Sections | What to Verify | Positive-Control Signal | Status |
|--------------|------------------|----------------|-------------------------|--------|
| Pyro fires without ARMED | ao_flight_director, SAD states | No pyro command accepted in IDLE/BOOST/COAST | `[FD] PYRO FIRED: DROGUE/MAIN` only after `ARMED` log | |
| Launch-abort not triggered | ao_flight_director + mission profile | Abort guard fires on high-G or timeout | `[FD] ABORT` logged with reason | |
| Disarm timeout fails | ao_flight_director | Timeout clears pyro intent after configured window | `[FD] DISARM TIMEOUT` + pyro intent cleared | |
| Radio command accepted without valid link | rfm95w + radio scheduler | Command only processed after `RegVersion=0x12` + recent nav frame | Valid `[CMD] ACK` only after link health shown | |
| ESKF divergence not braked | eskf/ + ao_flight_director | Velocity/position divergence triggers brake or abort | `[ESKF] DIVERGENCE` + state → ABORT or DISARM | |
| Watchdog / PIO WDT not armed | HW_GATE_DISCIPLINE + ao_flight_director | Both timers armed before ARMED state | `Watchdog: GO` and `PIO WDT: GO` in `p` output | |

**Pass criterion:** Every row shows a clear positive-control signal in the log or `p` output. Any missing signal = FAIL.

### A.2 Stack / Memory / RP2350 Errata Deep Review

> **Reference material is inlined in Appendix B.** Before working this section, read **B.1 (Power of 10 rule list with project-specific application notes)** for the rules that govern stack/memory hygiene. **B.4 describes the agent-driven audit workflow** including how errata-checklist results surface conversationally for user disposition.

**Stack & Memory Usage**
- Build with `-fstack-usage` and run `analyze_stack_usage.sh` (or equivalent).
- Flag any function >1 KB locals or any build exceeding 70 % SRAM.
- Review PSRAM ring buffers (`src/logging/`) and flash log buffers for head/tail overflow under worst-case logging rate.

**RP2350 Errata Re-verification** (from `RP2350_ERRATA.md`)
- **E2 (spinlock mirror writes)**: Re-check any change touching SIO spinlocks, boot paths, or picotool/SWD flows. Confirm `PICO_SW_SPIN_LOCKS_NO_EXTEXCLALL=1` is still set.
- **E9 (GPIO leakage)**: Re-audit any new `gpio_init()` or `gpio_set_function()` call. Verify no new floating inputs with IE=1 and weak pull.
- **E11 (XIP cache clean)**: Re-verify only if `xip_cache_clean_all()` or manual cache maintenance is added.
- **E12 (USB sync)**: Re-verify only if clock configuration changes `clk_sys` vs `clk_usb` margin.

**Manual Checklist (one-page focus):**
- Large stack allocations inside ISRs or Core-1 sensor loop.
- Flash erase/write operations during flight-critical windows (BOOST–APOGEE).
- Core 1 polling budget under maximum sensor load (IMU + baro + GPS).
- PSRAM ring buffer wrap handling and log flush safety.
- Any new compile flag affecting RP2350 platform constraints.

**Positive-control expectation:** Analyzer report shows zero flags above threshold + explicit statement in audit log: “Stack/memory/errata review complete – no violations above limits.” Any new E2/E9 exposure = FAIL.

### References for Manual Audit Guidance

**Internal (RocketChip):**
- `RP2350_ERRATA.md` — E2 (spinlock), E9 (GPIO leakage), E11 (XIP cache), E12 (USB sync) active errata, status, and workaround references
- `HW_GATE_DISCIPLINE.md` — Positive-control signal definition, examples (`RegVersion=0x12`, `DAC 0x18 ACK`, watchdog/PIO WDT GO), and anti-examples
- `docs/SAD.md` — State machine, flight phases, and safety-critical path definitions
- `PRE_FLIGHT_CHECKLIST.md` — `p` command output format, GO/FAULT verdicts, and pre-arm sequence
- `standards/AUDIT_GUIDANCE.md` (this document) — Master procedure, scope decision table, and positive-control requirement

**External (industry standards & guidance) — see Appendix B.5 for the demoted external-link list.** Substantive content from these sources is inlined in Appendix B (B.1 Power of 10, B.2 Koopman embedded review, B.3 FMEA worksheet columns). External URLs remain available as source material for going beyond the inline guidance, but day-to-day audit work should not require fetching them.

---

## Appendix B: Inline Reference Material

**Purpose:** Make the manual-audit sections self-contained. Previous audit attempts hit external-URL fetch failures (embedded.com PDF extractor returning binary garbage; PDF text-layer extraction returning unreadable streams). Inlining substantive content here means the manual reviewer can complete A.1, A.2, Tier 3 (standards catalog), and Tier 5.5 (requirements traceability) walks without leaving this document. Source URLs remain in B.5 for going beyond the inline material.

**Discipline:** All rule wording is quoted verbatim from the cited primary or authoritative secondary source. Per LL Entry 37, do not paraphrase rule text; the cleanup cost of an embedded misreading vastly exceeds the cost of accurate quoting. Project-application notes are clearly separated from the verbatim rule text.

### B.1 Power of 10 (Holzmann/JPL, 2006)

The ten rules, quoted verbatim from Perforce's reproduction of Holzmann's IEEE Computer 2006 article (cross-checked against the HardwareTeams summary; the original PDF at spinroot.com is the primary source but resists text-layer extraction). Each rule is followed by project-applicability notes.

**Rule 1.** *"Restrict all code to very simple control flow constructs—do not use goto statements, setjmp or longjmp constructs, or direct or indirect recursion."*

Project: Complies. Recursion-free codebase. `goto` resolved by GT-1 remediation (commit history). Maps to JSF AV Rules 188-189.

**Rule 2.** *"Give all loops a fixed upper bound."*

Project: Complies via Holzmann's own scheduler exemption (his original paper carves out non-terminating iterations — process schedulers, fault halts — and inverts the rule: it must be statically provable that the loop *cannot* terminate). Our exempt loops: `QF_run()` (QP/C cooperative scheduler — vendored), `core1_sensor_loop()` (Core 1 device-lifetime polling), `memmanage_fault_handler()` (intentional halt, R-3 remediation queued to switch to capture-state-then-reset), station/relay Core 1 idle wait. **Not deviations** — they satisfy the inverted form of the rule. See `standards/CODING_STANDARDS.md` Foundation section for the project's scheduler-exemption write-up.

**Rule 3.** *"Do not use dynamic memory allocation after initialization."*

Project: Complies. No heap allocations in production code after `init_application()`. Per LL Entry 1, large objects must be `static` (not stack) to avoid stack-overflow at function entry. Maps to JSF AV Rule 206.

**Rule 4.** *"No function should be longer than what can be printed on a single sheet of paper in a standard format with one line per statement and one line per declaration."*

Project: Enforced at ≤60 lines via JSF AV Rule 1 and the milestone full-tree clang-tidy sweep (SESSION_CHECKLIST item 17). Sole accepted deviation is CG-1 (auto-generated `codegen_fpft()`, ~1130 lines, per NASA SWE §8.11 (Auto-Generated Code) assurance pattern).

**Rule 5.** *"The code's assertion density should average to minimally two assertions per function."*

Project: Aspirational. Pico SDK and QP/C provide some assertion infrastructure (`Q_ASSERT`, `assert()` macros). Density not currently measured. Out-of-scope for this audit cycle; may be added to remediation queue if a future audit measures density and finds it lacking.

**Rule 6.** *"Declare all data objects at the smallest possible level of scope."*

Project: Complies. Globals are file-scope `static` where possible; minimal use of namespace-scope mutable globals. Cross-core globals (e.g., `g_bestGpsFix`) are intentional and documented in `docs/MULTICORE_RULES.md`.

**Rule 7.** *"Each calling function must check the return value of nonvoid functions, and each called function must check the validity of all parameters provided by the caller."*

Project: Mostly complies. Sensor-driver return checks documented in `LESSONS_LEARNED.md` (zero-output validation, consecutive-fail counters). Clang-tidy enforces return-value checks where the function is annotated `[[nodiscard]]`. Audit cycles should grep for `(void)` casts that discard return values without justification.

**Rule 8.** *"The use of the preprocessor must be limited to the inclusion of header files and simple macro definitions."*

Project: Complies. `#define` macros reduced to feature flags (`ROCKETCHIP_TIER_*`, `ROCKETCHIP_JOB_STATION`, `ROCKETCHIP_STAGE_T_LOGGING`/`T2_CHEAT`/`T3_MAVLINK`), SDK macros (`I2C_BUS_INSTANCE`), and the lwGPS config. Constants are `constexpr` per PP-1 resolution.

**Rule 9.** *"Limit pointer use to a single dereference, and do not use function pointers."*

Project: One accepted deviation (FP-1: `lm_solve()` uses `ResidualFn`/`JacobianFn` function pointers for LM-solver deduplication in Ground-classified calibration code, never in flight loop). Per the standards-precedence rule in `CODING_STANDARDS.md`, P10 Rule 9 (2006) supersedes JSF Rule 176 (2005) for function-pointer governance — JSF Rule 176 only requires typedef discipline, P10 Rule 9 explicitly prohibits. Template-based dispatch remediation queued as R-6c. JSF Rule 170 (≤2 levels of pointer indirection) also satisfied.

**Rule 10.** *"Compile with all possible warnings active; all warnings should then be addressed before the release of the software."*

Project: Complies. `-Wall -Wextra -Wpedantic -Werror` set for production builds (P10-10 resolution). Clang-tidy 21.1.8 enforces project-specific rule subsets in pre-commit. Auto-generated codegen file is exempt from the warning-as-error gate per CG-1.

### B.2 Koopman Embedded Review — 5-Rule One-Pager

The five rules already listed in A.1, with grep patterns and PASS-vs-FAIL examples added.

The original "Koopman 10 Rules" article (Phil Koopman, *Embedded.com*) is the source of these as a safety-critical-software discipline; A.1 reproduces five of the ten as the project's most-applicable subset.

**Rule K1: Every safety-critical function has documented preconditions and postconditions.**

Grep: comments containing `pre:` / `post:` / `requires:` / `ensures:` adjacent to flight-critical function definitions. Functions in `src/active_objects/`, `src/eskf/`, `src/safety/`, `src/drivers/icm20948.cpp`, `src/drivers/baro_dps310.cpp` should have these.

PASS: `core1_read_imu()` has clear comments stating "writes localData iff sensor read succeeds; on failure invalidates accel_valid and gyro_valid."

FAIL: a safety-critical function with no comment block stating its contract; reviewer can't tell what invariants the caller must maintain.

**Rule K2: No recursion or unbounded loops in interrupt or high-priority AO paths.**

Grep: `\\brecursion\\b` (no hits expected — codebase is recursion-free per P10 Rule 1 compliance); `while.*true` or `for.*;.*;` inside ISR handlers or AO event handlers (handlers are run-to-completion under QV cooperative scheduling; any unbounded loop blocks the scheduler). LL Entry 32 documents the original incident.

PASS: AO handlers return within one tick period (10ms at 100Hz). Verified by QP/C queue-depth analysis.

FAIL: any AO handler with a polling loop that depends on an external event (peripheral DRDY, etc.) without a bounded timeout.

**Rule K3: Every state transition has an explicit guard + observable positive-control signal.**

Grep: state transitions in `src/active_objects/ao_flight_director.cpp` and matching `[FD]` log lines in `src/logging/`. Per HW_GATE_DISCIPLINE.md Rule 1, every safety-relevant transition must emit a serial signal that proves the transition fired for the right reason.

PASS: `[FD] PYRO FIRED: DROGUE (primary)` log emitted only after the guard `is_armed && is_at_apogee` evaluates true. The log line proves the transition path executed.

FAIL: transition condition that updates state without a log line; agent can't observe whether the transition fired or was bypassed.

**Rule K4: Error paths either reach a safe terminal state or are formally unreachable.**

Grep: `return false`, `goto error`, exception throws (none — exceptions are disabled). Every error return path must lead to either (a) recoverable retry, (b) a documented degraded mode, or (c) an unrecoverable halt that the watchdog can observe.

PASS: `icm20948_init()` failure path sets `m_initialized = false` and the caller treats the device as absent — degraded mode.

FAIL: an error return that just propagates upward with no caller actually handling it; eventual default behavior is undefined.

**Rule K5: Backup paths (watchdog, PIO WDT) are exercised in every safety-critical sequence.**

Grep: `watchdog_update()` calls in main loop and Core 1 loop. PIO watchdog (if active per `docs/MULTICORE_RULES.md`) should be kicked by the same paths.

PASS: `watchdog_update()` called every iteration of Core 1's sensor loop at ~1kHz; documented 5s timeout per IVP-30.

FAIL: a long-running task that doesn't kick the watchdog; potential silent hang.

### B.3 FMEA Worksheet Columns (per NASA SWE §8.5 (Software FMEA))

NASA's Software FMEA guidance recognizes three worksheet forms — an **SFMEA Worksheet** (master), a **Data Table** (for corrupted-information failures), and an **Events Table** (for action-failures). Software FMEA differs from hardware FMEA in fault-mode taxonomy, but the column structure for failure-effect tracking is consistent.

**SFMEA master worksheet columns** (NASA SWE §8.5 (Software FMEA)):
- **ID number** — "drawing number, work break down structure number, CSCI identification, or other identification value."
- **Item / failure mode** — the failure mode being analyzed.
- **Failure cause(s) / mechanism(s)** — what could trigger this failure mode.
- **Mission phase / operational mode** — when in flight does this matter (IDLE, ARMED, BOOST, COAST, DESCENT, LANDED, ABORT).
- **Local effect** — at the component (the failing module).
- **Next-higher-level effect** — at the subsystem containing the component.
- **End effect** — system-level (mission outcome).
- **Severity classification** — one of: catastrophic, critical, marginal, negligible (NASA SWE §8.5 (Software FMEA) four-grade scale).
- **Likelihood** — one of: probable, occasional, remote, improbable.
- **Risk level** — 1 (prohibitive — must redesign) through 5 (acceptable). NASA SWE §8.5 (Software FMEA) 5-level scale.
- **Failure detection method** — software signals/mechanisms that observe the fault (positive-control signal in our terminology; per HW_GATE_DISCIPLINE.md Rule 1).
- **Compensating provisions** — what mitigates if the failure occurs (redundant component, degraded mode, reset, abort).
- **Proposed action** — remediation if the risk is unacceptable.

**Data Table columns** (for failures of *data*): the data item (input/output/stored), how it can be corrupted (out of range, missing, out of sequence, overwritten, wrong), what the consequences are. Useful for sensor pipelines (ICM-20948 raw → calibrated → ESKF input), ring buffers, telemetry packets.

**Events Table columns** (for failures of *actions*): the event item (action triggered by software, or by software on hardware), categorized as halt (abnormal termination), omission (event fails to occur), incorrect logic/event, or timing/order (wrong time, out of sequence). Useful for state machine transitions, pyro fire commands, calibration sequence steps.

**How RocketChip's A.1 FMEA-lite maps to this:**

The A.1 table columns ("Failure Mode," "Files / Sections," "What to Verify," "Positive-Control Signal," "Status") collapse the full NASA SWE §8.5 (Software FMEA) worksheet into a tight 5-column form suitable for a single-page review:

- **Failure Mode** = SFMEA's "Item / failure mode."
- **Files / Sections** = SFMEA's "ID number" expressed as source-file location.
- **What to Verify** = SFMEA's "Failure cause(s)" + "Local / Next-higher / End effect" + "Mission phase" combined narratively.
- **Positive-Control Signal** = SFMEA's "Failure detection method" — explicitly the project's discipline per HW_GATE_DISCIPLINE.md.
- **Status** = PASS / FAIL / PARTIAL replaces the severity/likelihood/risk-level triplet.

This is a deliberate simplification — we don't compute risk priority numbers because the binary "signal exists / signal doesn't" is what actually drives an audit-row decision. NASA SWE §8.5 (Software FMEA) acknowledges this is acceptable: "The handbook's emphasis on hardware-software interface analysis and fault propagation across component boundaries remains broadly applicable" regardless of column-count tailoring.

**Note on RPN:** If anyone ever extends this to a full quantitative FMEA, **do not compute risk priority numbers by multiplying ordinal scales** (the AIAG/VDA 2019 update replaced RPN with Action Priority for exactly this reason — multiplying ordinal rankings produces mathematically suspect numbers). Use the Action Priority approach instead.

### B.4 Audit Workflow (agent-driven with conversational user dispositions)

The audit's user-review interactions are **conversational**, not table-driven. The agent does the scripted/automated work, surfaces findings with evidence (file:line citations, raw quotes, suggested disposition with rationale), and the user dispositions each finding in conversation as it emerges. The audit report records the agreed dispositions afterward.

This is the pattern used during Phase A.2 (re-evaluating the 13 accepted deviations row-by-row in conversation) and during Phase 1 reporting (10 candidate findings surfaced with disposition recommendations, user gave REMEDIATE/DEFER/NEEDS REVIEW per row in conversation).

Three disposition labels:

- **REMEDIATE** — fix as part of Phase 8 in this audit cycle. The agent does the fix as a focused commit during Phase 8.
- **ACCEPT** — user signs off as a permanent deviation; row added to `standards/ACCEPTED_STANDARDS_DEVIATIONS.md`.
- **DEFER** — queue for a future post-audit remediation session. Logged in the dated report's `## Remediation` section. **Per Tier 7: every DEFER row requires a one-line safety-impact rationale** explaining why the deferral is acceptable; plain "not blocking" doesn't satisfy the rule.

**Per-phase notes:**

- **Phase 2** — no separate user walk. Records the dispositions for Phase 1 findings that the user gave conversationally during Phase 1 reporting. Includes agent-produced reference snapshots (P10 per-rule applicability from Appendix B.1, doc-sync issues, currently-accepted deviations entering the audit).
- **Phase 4** — agent walks the FMEA-lite table from Appendix A.1 itself, applies the Koopman 5-rule one-pager (B.2) to 3–5 flight-critical functions of agent's choice, runs `enhanced_fault_injection.py` scenarios. Records PASS/FAIL/PARTIAL per row with evidence. Any FAIL/PARTIAL surface conversationally for user disposition.
- **Phase 5** — agent walks the A.2 errata checklist itself (E2/E9/E11/E12 greps + ISR allocs / flash ops in BOOST / Core 1 polling budget / PSRAM wrap / new compile flags). Records PASS/FAIL with evidence. Any FAIL/PARTIAL surface conversationally.
- **Phase 7** — agent walks the requirements traceability spot-check itself. Per the master-audit plan's amendment 8, records raw quote + verification per row (CONFIRMED / STALE / MISSING). STALE/MISSING surface conversationally.

**Phase 9 — Post-audit guided code review (separate from the scripted audit cycle):**

After Phase 8 commits land and after some/all of the queued remediations have been applied, the user's eventual line-by-line read of the safety-critical code is supported by two agent-staged deliverables:

1. **Reading-discipline tips** — practical advice tailored to this codebase: what to grep first, how to prioritize, how to spot comment-vs-implementation drift, what's auto-detectable (already caught by the scripted audit) vs. what only a human catches, realistic time estimates for the read.
2. **Guided source tour** — agent pre-reads flight-critical paths and stages an annotated walking order (15–25 waypoints) with audit-finding annotations alongside surrounding-context narration. User walks the agent's tour rather than the code raw.

Both deliverables live in `docs/audits/<AUDIT>_MANUAL_REVIEW_GUIDE.md` (or as appendices to the dated audit report, at user preference). The agent's role during the user's read is responding to questions, not driving the read.

The scripted audit informs the manual review by showing where to focus; the manual review catches what scripted checks can't (semantic correctness, design-intent drift, integration issues across files).

### B.5 External References (Demoted)

Use only when the inline material above is insufficient. Per LL Entry 37 step 1, verify rule wording against the primary source before citing.

**Primary safety-critical-coding standards:**
- JSF AV C++ Coding Standards (Lockheed Martin, December 2005) — https://www.stroustrup.com/JSF-AV-rules.pdf (Stroustrup's mirror; the F-35 baseline document)
- Power of 10 (Holzmann/JPL, 2006) — https://spinroot.com/gerard/pdf/P10.pdf (original IEEE Computer article; the foundational 10-rule distillation)
- JPL Institutional Coding Standard for C (NASA/JPL, March 2009) — https://yurichev.com/mirrors/C/JPL_Coding_Standard_C.pdf (builds on P10, C-language refinement)

**NASA institutional guidance:**
- NASA Software Engineering Handbook — https://swehb.nasa.gov/ (handbook root; publicly accessible NASA-wide guidance)
  - §8.5 — Software Failure Modes and Effects Analysis (source for B.3 above)
  - §8.11 — Auto-Generated Code (source for CG-1 deviation rationale)

**Safety-critical FMEA primary sources** (MIL-STD-1629A and SAE ARP4761 — both predate NASA SWE §8.5 (Software FMEA) which now serves as the project-applicable digest):
- MIL-STD-1629A (U.S. DoD, 1980) — "Procedures for Performing a Failure Mode, Effects and Criticality Analysis." Hosted via NASA at https://extapps.ksc.nasa.gov/Reliability/Documents/milstd1629_FMEA.pdf (PDF text-layer extraction has been unreliable; consult NASA SWE §8.5 (Software FMEA) first).
- SAE ARP4761 — Aerospace Recommended Practice for safety assessment process on civil airborne systems. SAE-paywalled; for our project's needs, NASA SWE §8.5 (Software FMEA) (which references both upstream) is the project-applicable digest.

**Koopman safety-critical resources:**
- Phil Koopman's writing on safety-critical embedded software — https://users.ece.cmu.edu/~koopman/ (faculty page at CMU; better-extractable than the embedded.com versions of his articles).

All external references are public, widely cited sources in safety-critical embedded systems. No proprietary or paywalled material is required for routine audit work.

---

## Appendix C: Remediation Execution Ordering

**Purpose:** Define the canonical order in which audit findings are remediated, and the verification cadence between groups. Applies to Tier 7 of every master audit cycle (was Step 8 pre-2026-05-13) and to any focused post-audit remediation session that draws from a dated audit's `## Remediation` queue.

**Why this appendix exists.** When an audit produces a remediation queue of more than ~3 findings, the order in which they are executed is itself a quality decision. Fixing in arbitrary order has three failure modes specific to this project: (a) early fixes get invalidated by later refactors that touch the same code, (b) "fixed" findings regress because a finding they depend on wasn't fixed first, (c) verification fatigue where every focused commit re-runs the 6-hour audit suite, so the audit suite ends up skipped. The DO-178C "problem report" lifecycle (record → impact-analyze → fix → verify → close) and the static-analysis-tool community's distinction between "what blocks a merge" and "what becomes backlog" both speak to the same problem and converge on the framework below.

### C.1 Treat each finding as a Problem Report

Adopt the DO-178C framing: every audit finding is a Problem Report (PR) with a lifecycle. The audit's dated `## Remediation` section is the PR log. Each PR's state is one of:

- **Open** — finding recorded, no analysis yet.
- **Analyzed** — impact analysis complete (see C.2). Dependency edges identified. Disposition (REMEDIATE / ACCEPT / DEFER) recorded.
- **In progress** — code change in flight. Local verification (per-commit ctest + the change's own positive-control signal) underway.
- **Verified** — local commit verification passed; awaiting the audit-suite regression at C.5's gate.
- **Closed** — audit-suite regression passed end-to-end and the dated report's `## Remediation` row signed.

The lifecycle is **mandatory in concept, lightweight in practice** — for our project a PR's state is a column in the `## Remediation` table, not a separate ticketing system. The point is that "fixed locally" and "closed" are different states, which keeps a tail-end audit-suite regression from being skipped.

### C.2 Impact analysis before sequencing

Before deciding the order, run impact analysis on each open PR. Three questions per PR:

1. **What files and modules does the fix touch?** (Build a quick dependency footprint.)
2. **What other PRs share those files or rely on a mechanism this PR creates?** (Identifies edges in the dependency graph.)
3. **What audit gates does the fix interact with?** (Pre-commit hook, ctest, SPIN, lizard, coverage. Identifies "gate-integrity" PRs — see C.3 category 1.)

The output is a small dependency graph over the PR set. For audits with ≤10 PRs, the graph fits on a napkin; for larger queues it's still bounded enough to draw by hand or as a markdown table column. The graph is captured in the dated report's `## Remediation` section so subsequent sessions can follow the same plan.

### C.3 Order by category, then by dependency, then by risk

Process the PR set in four categories, in this order. Within each category, dependencies (from C.2) come before dependents; within a dependency level, higher safety risk comes before lower.

**Category 1 — Gate Integrity.** Any PR that fixes a tool, harness, gate, or pre-commit check that the audit itself depended on. Includes script bugs, broken regex, missing test coverage, stub files that imply coverage that doesn't exist (LL Entry 36 pattern). These come first because the rest of the verification cycle relies on them being honest. If a gate is reporting green for the wrong reason, every subsequent "verified" claim inherits the same defect.

**Category 2 — Shared Foundations.** PRs that introduce or refactor a mechanism that multiple later PRs reuse. Identified from C.2's graph as a node with high out-degree (many PRs depend on it). Examples in this project's history: a preserved-SRAM crash-record format used by multiple fault-handler PRs, a custom bounded writer used by multiple printf-elimination PRs. Land these next because the dependent PRs ride on them.

**Category 3 — Behavior Changes.** Code that closes an open finding by changing observable program behavior. Within this category, order by **mission-phase exposure × intrinsic severity**, not by intrinsic severity alone:

- Flight-critical paths (state machine, ESKF, pyro, watchdog, fault handler) before flight-support paths (LED engine, telemetry, logging) before ground-only paths (calibration, CLI, dev/diagnostic). A "critical" defect in code that can't execute during flight is less urgent than a "marginal" defect in flight-critical code, because runtime lockout removes the in-flight failure mode for the ground-only case.
- Within a tier, prefer PRs that touch the same file (one focused commit per file) over PRs scattered across many files — re-touching a file across many commits has a small but real chance of conflict if active development overlaps with the audit cycle.

**Category 4 — Cleanup & Documentation.** PRs that are doc-only (CHANGELOG, SAD updates triggered by category-2/3 work, decision-doc supersession notes), comment-only, or stylistic. Defer to the end of the cycle. The audit's `## Remediation` section is itself state-of-system documentation, so per-doc trigger-driven edits per `.claude/SESSION_CHECKLIST.md` still apply at the appropriate trigger point — category 4 is for cleanup that wasn't already covered by per-commit trigger discipline.

### C.4 Commit cadence — atomic per PR, with named clustering exceptions

The default is **one focused commit per PR**, citing the PR ID and the observed verification signal per `HW_GATE_DISCIPLINE.md` Rule 3. This is the "atomic commits" pattern the static-analysis-tool community names as test-supported remediation, and it makes each PR independently bisectable and revertable.

Two clustering exceptions where one commit per PR is wrong:

- **Pattern-shared findings.** Multiple PRs that are the same defect class in different files (e.g., 30 `snprintf` callsites that all migrate to const arrays). One commit per file, not per finding.
- **Single-mechanism refactor.** Multiple PRs that all wire into one new shared mechanism, where landing them piecewise would force commits that are syntactically incomplete. Land the mechanism in one commit (category 2), then each dependent PR in its own focused commit.

The clustering decision is a judgment call; the default is split.

### C.4a Surfaced-bug scope rule

**When verifying PR-X reveals bug-Y that PR-X's verification path depends on, expand PR-X's scope to also fix bug-Y. Both fixes ship in the same commit (or back-to-back commits with the same PR ID).** Splitting bug-Y off as a separate PR produces a verification gap: PR-X claims `closed` but its own verification couldn't actually run, and the gap looks identical to "we verified locally only" from a reader's perspective.

The test is: **did fixing PR-X's primary issue *reveal* bug-Y, or was bug-Y just coincidentally nearby?** Revealed bugs are in-scope; coincident bugs split into separate PRs as normal.

Indicators that a bug was revealed (not coincident):
- PR-X's primary defect was masking bug-Y (e.g., a halt-forever handler hid the fact that the MPU guard region wasn't actually catching writes, because the handler was never exercised). Fixing X exposes Y.
- PR-X's verification recipe (named in the audit's preliminary remediation queue) cannot produce its positive-control signal until bug-Y is also fixed.
- Bug-Y is in code PR-X is touching, and a future reader following PR-X's verification trail will hit the same issue.

Indicators that a bug is coincident (split into a new PR):
- Bug-Y is in a different subsystem, unrelated mechanism, and PR-X's verification doesn't touch it.
- Bug-Y existed independently and PR-X just happened to be in the same file.
- Fixing bug-Y doesn't change PR-X's verification outcome.

**Documenting the expanded scope:** the PR-X commit message names the surfaced bug, states why it was in-scope (one of the indicators above), and cites the verification that proves both fixes work together. The dated audit report's `## Remediation` section captures the same in the PR-X row.

Source: project policy 2026-05-12 — surfaced bugs are in-scope because they were hidden by the same defect PR-X exists to fix; treating them as separate PRs pretends an independence that wasn't there. Lived-experience case: R-3 (audit 2026-05-07) surfaced the MPU `AP=0b00` encoding bug + the missing MEMFAULTENA — both hidden by the pre-R-3 halt-forever handler that never exercised the guard. Both rode in the R-3 commit.

### C.5 Verification cadence — per commit local, per category regression

The static-analysis-tool community names this distinction clearly: **what blocks a merge ≠ what gets the full audit-suite regression**.

- **Per commit (merge-blocker level):** the change does what it claims, host ctest passes, the change's own positive-control signal is observed (per `HW_GATE_DISCIPLINE.md` Rule 3). This is what the pre-commit hook enforces and what each commit message cites.
- **Per category transition:** when category 1 closes and category 2 begins, when category 2 closes and category 3 begins, etc., run the **original audit's scripted check suite** end-to-end. This is the same Phase 1 + Phase 6 work that produced the audit's baseline — clang-tidy, cppcheck, ctest, SPIN, lizard, coverage, and any HW gates the original audit required. Compare against the baseline. Per DO-178C, this is the change-impact regression — verification that the fixes haven't broken what the audit confirmed was working.
- **At cycle close:** before signing the dated report's `## Remediation` section as complete, re-run the audit-suite regression one final time *after* the category-4 cleanup commits, so the verification covers the as-shipped state including doc-sync edits.

The regression suite is selective in the sense that it's the original audit's already-defined check suite, not an open-ended exploration. If category-2 or category-3 PRs changed code in a way the original audit's checks don't cover (e.g., a new SPIN model is needed for a refactor), the new check is staged before the category transition that introduced the gap.

### C.6 Re-audit credibility — independent vs author verification

The DO-178C principle of independence applies: the author of the fix is not the most credible verifier of the fix. In a single-developer project this is impractical to enforce per PR, but it shows up at audit boundaries. The fix author runs the per-commit verification; the next audit cycle (or a focused re-audit per the AUDIT_GUIDANCE decision table row 6) is what provides independent verification credit. The dated report's `## Remediation` section is the audit trail that lets a later session validate the prior session's claims without re-doing the fix author's work.

### C.7 Stop conditions

Halt the remediation cycle (do not advance categories, do not close the audit) on any of:

- **Category 1 fails.** The audit's own gates can't be made honest. Everything downstream is unverified by construction. Reopen the audit's verification surface before continuing.
- **Category 2 produces unexpected scripted-check regressions.** The audit's impact-analysis was wrong about what depends on what. Re-do C.2 before continuing category 3.
- **User escalates ≥3 findings to ACCEPT (permanent deviation) within a single cycle.** The audit's findings frame doesn't match the project's actual constraints. Reopen audit scope or rework the findings before continuing.
- **Hardware 3-boot reseat per `HW_GATE_DISCIPLINE.md` Rule 2 produces different positive-control signals across boots.** This is a flight-safety stop independent of remediation state.

Halts are recorded as `HALT` rows in the dated report's `## Remediation` section, with the category that halted and why. Subsequent sessions resume from the halt rather than restarting.

### C.8 Generally applicable to future audits

This appendix is project-wide guidance, not specific to any single audit cycle. Every future master audit's Tier 7 follows the same PR-lifecycle framing with the same four categories and the same verification cadence. Each dated report's `## Remediation` section opens with a one-paragraph preamble naming any deviations from the canonical order (rare — e.g., an emergency safety hotfix that bypasses category 1 because the safety issue is more urgent than gate-hygiene).

If the project's verification surface area changes meaningfully (new hardware tier, new audit phase, new gating mechanism), update this appendix as a state-of-system trigger — same discipline as `.claude/SESSION_CHECKLIST.md` Trigger-Driven Doc Edits.

### C.9 Source notes (load-bearing only)

- **DO-178C problem-report lifecycle** is the source of C.1's PR-states framing. DO-178C requires that all problems be recorded in problem reports, analyzed for impact, and tracked to closure; open PRs at certification require safety-impact evaluation. We adopt the lifecycle concept; we don't adopt the formal Change Control Board apparatus, which is sized for multi-team certified-aircraft projects.
- **Static-analysis-tool community** (SonarQube and similar) is the source of C.5's "merge-blocker vs backlog" distinction. The named failure mode — "teams fail because nobody makes clear decisions about what blocks a merge and what becomes backlog work" — is the trap C.5 is designed to avoid.
- **Refactoring research on dependency cycles and hotspots** is the source of C.3 category 2's "shared foundations" framing. Components in dependency cycles are defect-prone and unblock the most downstream work; fix them early.
- **NASA SWE §8.5 (Software FMEA) severity scale** is the project's framing for C.3 category 3's within-tier ordering. NASA's catastrophic/critical/marginal/negligible scale combines with mission-phase exposure (flight-critical vs ground-only) to produce the actual risk rank.
- **Project-internal precedent.** LL Entry 36 (bench_sim rot — gate had been silently broken) is the lived-experience driver for C.3 category 1 (gate integrity first). Without category 1, audit cycles would inherit the same defect that LL Entry 36 documents.

---

## Three-Category Plan Reference

Full details of scripted vs manual work items are in `AUDIT_CLEANUP_2026-05-06.md` (Category 1–3 tables). This document only sequences their execution and defines scope.

---

## Hardware Validation Scripts – Usage Classification

**Regular / Periodic Use** (run frequently — pre-flight, milestone, or integration):
- `soak_test.py` — General passive soak (watchdog, queues, error counts, MSP depth)
- `i2c_soak_test.py` — Focused I2C/sensor error monitoring (IMU, baro, GPS)
- `bench_sim.py` + `station_bench_sim.py` — End-to-end flight director + CLI path test
- `enhanced_fault_injection.py` — Fault-injection scenarios via probe (arms test mode at session start)
- `warm_reboot_audit.py` — R-22 warm-reboot 4-gate audit (R-25-exec step 11, 2026-05-13)
- `verify_boot_parity.sh` — R-24 build + flash + banner-classify per role (R-25-exec step 12, 2026-05-13)
- `cli_test.py` — Non-destructive CLI command tests

**Specialized / Feature-specific** (run when the relevant subsystem changes):
- `accel_cal_6pos.py` — 6-position accelerometer calibration
- `replay_harness_host.py` — Host-side ESKF replay (placeholder skeleton; implementation pending per amendment #4)

**Retired** (deleted by R-25-exec step 9 per council amendment #6, 2026-05-13):
- All `stage_t_*` scripts, `ack_stress_test.py`, `codegen_soak_test.py`, `eskf_gps_soak.py`, `replay_gate_test.py`, `replay_harness.py`, `station_replay_harness.py`. Coverage moved host-side or absorbed by retained gates.

New scripted audit tools will only enforce rules already present in `CODING_STANDARDS.md`, `HW_GATE_DISCIPLINE.md`, and platform constraints. No new rules will be invented without explicit council approval.

---

## File Location Policy & Consolidation

- Master procedure + `STANDARDS_AUDIT.md` template → `standards/`
- Historical audit reports → `docs/audits/`
- Operational checklists (`PRE_FLIGHT_CHECKLIST.md`, `BENCH_TEST_PROCEDURE.md`) → `docs/`
- Category 3 manual guides live inside this file as Appendix A

---

## Cross-References

- `CODING_STANDARDS.md` — Rule definitions
- `HW_GATE_DISCIPLINE.md` — Positive-control requirement
- `PRE_FLIGHT_CHECKLIST.md` — Tier 5.1 detail
- `VERIFICATION_OVERVIEW.md` — Complementary verification layers
- `AUDIT_REMEDIATION.md` — Historical fixes
- `AUDIT_CLEANUP_2026-05-06.md` — Three-category plan
- `RP2350_ERRATA.md` — Appendix A.2 reference
- `docs/SAD.md` — State machine for FMEA mapping

---

*End of master audit procedure. Update only when overall audit strategy changes.*