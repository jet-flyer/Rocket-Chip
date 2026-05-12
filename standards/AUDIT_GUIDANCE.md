# RocketChip Audit Guidance — Master Procedure

**Status:** Normative master audit procedure.  
**Purpose:** Single authoritative document that sequences every audit type into one lifecycle. The core checklist is concise (≤1 page). Manual audit guidance lives in concise appendices.

---

## Master vs Targeted Audits

- **Master Procedure (`AUDIT_GUIDANCE.md`)**: The unified lifecycle checklist. Defines *when* to run what scope. Not a daily checklist itself.
- **Targeted / Operational Audits**: The actual focused work (coding standards, pre-flight gate, FMEA, stack review, etc.). These are invoked by steps in the master procedure.

Use the decision table below to choose the right scope. Use the targeted documents to perform the work.

---

## When to Do What – Audit Scope Decision Table

| Trigger                                | Scope                          | Specific Steps / Targeted Audits to Run                                      | Positive-Control Expectation                     | Notes |
|----------------------------------------|--------------------------------|------------------------------------------------------------------------------|--------------------------------------------------|-------|
| Every commit (small change)            | Incremental / targeted         | Step 2 (coding standards) on **changed files only**                          | clang-tidy clean on staged files                 | Pre-commit hook enforces |
| Before push                            | Build parity + relevant targeted | Station/vehicle parity + re-run Step 2 on modified modules                 | Both build tiers compile + standards pass        | See `SESSION_CHECKLIST.md` item 6 |
| Before field test / launch             | Pre-flight gate + safety review | Step 3 (full `PRE_FLIGHT_CHECKLIST.md`) + Step 4 (FMEA-lite)               | All GO verdicts + positive-control signals       | Highest safety bar |
| Milestone / stage close                | **Full master procedure**      | All 8 steps                                                                  | Complete set of positive-control outputs         | Super audit |
| Safety-critical or architecture change | Targeted deep review           | Steps 2, 4, 5, 6, 7 **scoped to affected modules**                           | Positive-control signals on changed paths        | Do not re-audit untouched areas |
| After failed audit or remediation      | Affected portions + regression | Re-execute failed step(s) + any dependent steps                              | Previously failed items now PASS                 | Log in `AUDIT_REMEDIATION.md` |

**Principle:** Use the smallest sufficient scope. The full master procedure is reserved for milestone/stage-close events.

---

## Master Audit Procedure (Concise Checklist)

Run the full procedure only at milestone / stage close. For all other triggers, use the decision table above.

1. **Baseline Scripted Sweeps (Category 1)**  
   Run `run_clang_tidy.sh`, `bench_sim.py`, `replay_gate_test.py`, `ctest`, SPIN models, hardware soak scripts. Record verdicts + positive-control lines.

2. **Coding Standards Deep Compliance**  
   Execute full `STANDARDS_AUDIT.md` template (JSF AV / JPL / Power of 10 / platform rules). Log in dated `STANDARDS_AUDIT_YYYY-MM-DD.md`.

3. **Pre-Flight Gate Execution**  
   Follow `PRE_FLIGHT_CHECKLIST.md` in full (firmware verification, 5-min soak, calibration, mission profile, station link, `p` command, arm sequence). Confirm every GO + positive-control signal.

4. **Safety-Critical Path Review**  
   See **Appendix A.1** (FMEA-lite & Koopman Embedded Review). Map to `docs/SAD.md` state machine. Use positive-control signal column.

5. **Stack / Memory / RP2350 Errata Deep Review**  
   See **Appendix A.2**. Run `analyze_stack_usage.sh` + manual checklist. Flag >1 KB locals or >70 % SRAM. Cross-reference `RP2350_ERRATA.md`.

6. **Formal Verification + Simulation Coverage**  
   Re-run SPIN models on current `ao_flight_director.cpp`. Execute `bench_flight_sim.py` 9-scenario suite.

7. **Requirements Traceability Spot-Check (flight-critical paths only)**  
   Verify state machine, ESKF outputs, pyro commands, telemetry fields, safety flags have traceable requirements. Lightweight markdown table or script output.

8. **Remediation & Historical Logging**  
   Record user-accepted deviations in `ACCEPTED_STANDARDS_DEVIATIONS.md`. Append fixes to the dated audit report's `## Remediation` section (per-audit, self-contained — see file-location policy below). Update this document only for strategy changes (state-of-system trigger).

---

## Appendix A: Manual Audit Guides (Concise)

### A.1 FMEA-lite & Koopman Embedded Review

> **Reference material is inlined in Appendix B.** Before walking this section, read **B.2 (Koopman 5-rule embedded review)** and **B.3 (FMEA worksheet columns, per NASA SWE Handbook §8.5)** for the rule wording, grep patterns, and PASS-vs-FAIL examples. **B.4 (walking procedure for A.1)** gives the step-by-step.

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

> **Reference material is inlined in Appendix B.** Before walking this section, read **B.1 (Power of 10 rule list with project-specific application notes)** for the rules that govern stack/memory hygiene, and **B.4 (walking procedure for A.2)** for the step-by-step.

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

**Purpose:** Make the manual-audit sections self-contained. Previous audit attempts hit external-URL fetch failures (embedded.com PDF extractor returning binary garbage; PDF text-layer extraction returning unreadable streams). Inlining substantive content here means the manual reviewer can complete A.1, A.2, Step 2, and Step 7 walks without leaving this document. Source URLs remain in B.5 for going beyond the inline material.

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

Project: Enforced at ≤60 lines via JSF AV Rule 1 and the milestone full-tree clang-tidy sweep (SESSION_CHECKLIST item 17). Sole accepted deviation is CG-1 (auto-generated `codegen_fpft()`, ~1130 lines, per NASA SWE Handbook §8.11 auto-generated-code assurance pattern).

**Rule 5.** *"The code's assertion density should average to minimally two assertions per function."*

Project: Aspirational. Pico SDK and QP/C provide some assertion infrastructure (`Q_ASSERT`, `assert()` macros). Density not currently measured. Out-of-scope for this audit cycle; may be added to remediation queue if a future audit measures density and finds it lacking.

**Rule 6.** *"Declare all data objects at the smallest possible level of scope."*

Project: Complies. Globals are file-scope `static` where possible; minimal use of namespace-scope mutable globals. Cross-core globals (e.g., `g_bestGpsFix`) are intentional and documented in `docs/MULTICORE_RULES.md`.

**Rule 7.** *"Each calling function must check the return value of nonvoid functions, and each called function must check the validity of all parameters provided by the caller."*

Project: Mostly complies. Sensor-driver return checks documented in `LESSONS_LEARNED.md` (zero-output validation, consecutive-fail counters). Clang-tidy enforces return-value checks where the function is annotated `[[nodiscard]]`. Audit cycles should grep for `(void)` casts that discard return values without justification.

**Rule 8.** *"The use of the preprocessor must be limited to the inclusion of header files and simple macro definitions."*

Project: Complies. `#define` macros reduced to feature flags (`ROCKETCHIP_TIER_*`, `NOT_CERTIFIED_FOR_FLIGHT`), SDK macros (`I2C_BUS_INSTANCE`), and the lwGPS config. Constants are `constexpr` per PP-1 resolution.

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

### B.3 FMEA Worksheet Columns (per NASA SWE Handbook §8.5)

NASA's Software FMEA guidance recognizes three worksheet forms — an **SFMEA Worksheet** (master), a **Data Table** (for corrupted-information failures), and an **Events Table** (for action-failures). Software FMEA differs from hardware FMEA in fault-mode taxonomy, but the column structure for failure-effect tracking is consistent.

**SFMEA master worksheet columns** (NASA SWE Handbook §8.5):
- **ID number** — "drawing number, work break down structure number, CSCI identification, or other identification value."
- **Item / failure mode** — the failure mode being analyzed.
- **Failure cause(s) / mechanism(s)** — what could trigger this failure mode.
- **Mission phase / operational mode** — when in flight does this matter (IDLE, ARMED, BOOST, COAST, DESCENT, LANDED, ABORT).
- **Local effect** — at the component (the failing module).
- **Next-higher-level effect** — at the subsystem containing the component.
- **End effect** — system-level (mission outcome).
- **Severity classification** — one of: catastrophic, critical, marginal, negligible (NASA SWE §8.5 four-grade scale).
- **Likelihood** — one of: probable, occasional, remote, improbable.
- **Risk level** — 1 (prohibitive — must redesign) through 5 (acceptable). NASA SWE §8.5 5-level scale.
- **Failure detection method** — software signals/mechanisms that observe the fault (positive-control signal in our terminology; per HW_GATE_DISCIPLINE.md Rule 1).
- **Compensating provisions** — what mitigates if the failure occurs (redundant component, degraded mode, reset, abort).
- **Proposed action** — remediation if the risk is unacceptable.

**Data Table columns** (for failures of *data*): the data item (input/output/stored), how it can be corrupted (out of range, missing, out of sequence, overwritten, wrong), what the consequences are. Useful for sensor pipelines (ICM-20948 raw → calibrated → ESKF input), ring buffers, telemetry packets.

**Events Table columns** (for failures of *actions*): the event item (action triggered by software, or by software on hardware), categorized as halt (abnormal termination), omission (event fails to occur), incorrect logic/event, or timing/order (wrong time, out of sequence). Useful for state machine transitions, pyro fire commands, calibration sequence steps.

**How RocketChip's A.1 FMEA-lite maps to this:**

The A.1 table columns ("Failure Mode," "Files / Sections," "What to Verify," "Positive-Control Signal," "Status") collapse the full NASA SWE §8.5 worksheet into a tight 5-column form suitable for a single-page review:

- **Failure Mode** = SFMEA's "Item / failure mode."
- **Files / Sections** = SFMEA's "ID number" expressed as source-file location.
- **What to Verify** = SFMEA's "Failure cause(s)" + "Local / Next-higher / End effect" + "Mission phase" combined narratively.
- **Positive-Control Signal** = SFMEA's "Failure detection method" — explicitly the project's discipline per HW_GATE_DISCIPLINE.md.
- **Status** = PASS / FAIL / PARTIAL replaces the severity/likelihood/risk-level triplet.

This is a deliberate simplification — we don't compute risk priority numbers because the binary "signal exists / signal doesn't" is what actually drives an audit-row decision. NASA SWE §8.5 acknowledges this is acceptable: "The handbook's emphasis on hardware-software interface analysis and fault propagation across component boundaries remains broadly applicable" regardless of column-count tailoring.

**Note on RPN:** If anyone ever extends this to a full quantitative FMEA, **do not compute risk priority numbers by multiplying ordinal scales** (the AIAG/VDA 2019 update replaced RPN with Action Priority for exactly this reason — multiplying ordinal rankings produces mathematically suspect numbers). Use the Action Priority approach instead.

### B.4 Manual-Audit Walking Procedures

For each of the four user-owned manual phases of the master audit, here is how to walk it.

#### Walking Phase 2 — Step 2 STANDARDS_AUDIT.md walk (~45 min)

**Read first:** the agent-staged section in the dated audit report (rule-category table from `standards/STANDARDS_AUDIT.md` with Status columns blank, plus the diff of clang-tidy/cppcheck findings against current `ACCEPTED_STANDARDS_DEVIATIONS.md` Active rows).

**Walk order:**
1. Top-to-bottom through the rule-category dashboard (Power of 10, JSF AV, JPL C, Platform Rules, Multicore, Debug Output).
2. For each row, decide PASS / FAIL / PARTIAL / NOT CHECKED / N/A per the existing template legend.
3. For any FAIL or PARTIAL row, note: (a) the file(s) affected, (b) whether it's a known accepted deviation (cross-reference `ACCEPTED_STANDARDS_DEVIATIONS.md`), (c) whether it's a candidate new deviation requiring user sign-off, (d) whether it's a candidate remediation.

**Grep patterns to apply per row:** the rule's `grep -rn <pattern> src/` should match expected occurrences (if zero hits where there should be many, the rule isn't being applied; if many hits where there should be zero, deviation present).

**Time budget:** ~45 minutes for a thorough walk. If running shorter, focus on Power of 10 (10 rules) + Platform Rules + Multicore. JSF AV's 221 rules can be sampled — pick the high-impact ones (assertion density, fixed-width types, exception handling, RTTI).

**Done looks like:** every row has a status code; FAIL/PARTIAL rows have at least a one-line note explaining what to do.

#### Walking Phase 4 — A.1 FMEA-lite (~30 min)

**Read first:** Appendix A.1 FMEA-lite table as staged by the agent (file references hot-linked to current source line ranges; positive-control signals pre-loaded from Phase 1 bench_sim runs). Also read B.2 (Koopman 5-rule) and B.3 (FMEA column structure) above.

**Walk order:**
1. For each row (Pyro-fires-without-ARMED, Launch-abort-not-triggered, Disarm-timeout-fails, Radio-command-without-valid-link, ESKF-divergence-not-braked, Watchdog/PIO-WDT-not-armed): open the cited source file, find the guard or detection logic, verify it matches the stated "Positive-Control Signal" column.
2. Mark Status: PASS if guard exists, signal observable; FAIL if guard missing or signal not observable; PARTIAL if guard exists but signal is anti-evidence (per HW_GATE_DISCIPLINE.md Rule 1).
3. After the FMEA table, apply the Koopman 5-rule one-pager (B.2) to 3–5 flight-critical functions of your choice (`ao_flight_director.cpp`, `eskf.cpp`, `rfm95w.cpp` are good candidates).

**Time budget:** ~30 minutes. FMEA table is 6 rows × 5 min each. Koopman walk is 3–5 functions × 3 min each.

**Done looks like:** FMEA table has Status filled per row, with positive-control signals confirmed observed (not just expected). Koopman walk has a 5×N table where N is the number of functions reviewed, with PASS/FAIL per cell.

#### Walking Phase 5 — A.2 Manual Errata Checklist (~20 min)

**Read first:** agent-staged A.2 section with errata-grep results pre-loaded (E2 spinlock callsites, E9 GPIO calls, E11 XIP cache references, E12 clk_sys/clk_usb config). Also read `standards/RP2350_ERRATA.md` to refresh on each erratum's workaround status.

**Walk order:** Verify per erratum row that the agent's grep results match the expected state (e.g., E2 should show `PICO_SW_SPIN_LOCKS_NO_EXTEXCLALL=1` still in CMakeLists; E9 should show no new floating-input GPIO calls). Then walk the manual checklist: ISR large allocations, flash ops during BOOST/COAST/DESCENT, Core 1 polling budget under max sensor load, PSRAM ring buffer wrap behavior, new compile flags introduced since the prior audit.

**Time budget:** ~20 minutes if the agent's grep pre-load is clean. Longer if grep surfaces unexpected callsites that need investigation.

**Done looks like:** every errata row has a PASS or note explaining the new callsite/state. Manual checklist items have PASS or remediation note.

#### Walking Phase 7 — Requirements Traceability Spot-Check (~20 min)

**Read first:** agent-staged 10–15 row candidate table of flight-critical items (state machine transitions, ESKF outputs, pyro commands, telemetry fields, safety flags). Per the council's amendment 8 to the master audit plan, the agent stages the *raw quote* found in the source (e.g., `claimed: ao_flight_director.cpp:142 says "// per SAD §4.3"`), not a clean citation.

**Walk order:** For each row, verify the cited section in the referenced doc (e.g., open SAD.md §4.3, confirm the section actually describes the transition being claimed). The failure mode being prevented is rubber-stamping comments that point at refactored-away sections.

**Time budget:** ~20 minutes for 10–15 rows. Most rows resolve in <1 minute (citation still accurate); 1–2 rows per audit cycle typically surface a stale citation that wants a follow-up.

**Done looks like:** each row marked CONFIRMED / STALE / MISSING. STALE rows go to the dated audit report's Remediation section as cleanup items.

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

**Safety-critical FMEA primary sources** (MIL-STD-1629A and SAE ARP4761 — both predate NASA SWE §8.5 which now serves as the project-applicable digest):
- MIL-STD-1629A (U.S. DoD, 1980) — "Procedures for Performing a Failure Mode, Effects and Criticality Analysis." Hosted via NASA at https://extapps.ksc.nasa.gov/Reliability/Documents/milstd1629_FMEA.pdf (PDF text-layer extraction has been unreliable; consult NASA SWE §8.5 first).
- SAE ARP4761 — Aerospace Recommended Practice for safety assessment process on civil airborne systems. SAE-paywalled; for our project's needs, NASA SWE §8.5 (which references both upstream) is the project-applicable digest.

**Koopman safety-critical resources:**
- Phil Koopman's writing on safety-critical embedded software — https://users.ece.cmu.edu/~koopman/ (faculty page at CMU; better-extractable than the embedded.com versions of his articles).

All external references are public, widely cited sources in safety-critical embedded systems. No proprietary or paywalled material is required for routine audit work.

---

## Three-Category Plan Reference

Full details of scripted vs manual work items are in `AUDIT_CLEANUP_2026-05-06.md` (Category 1–3 tables). This document only sequences their execution and defines scope.

---

## Hardware Validation Scripts – Usage Classification

**Regular / Periodic Use** (run frequently — pre-flight, milestone, or integration):
- `soak_test.py` — General passive soak (watchdog, queues, error counts, MSP depth)
- `i2c_soak_test.py` — Focused I2C/sensor error monitoring (IMU, baro, GPS)
- `bench_sim.py` + `station_bench_sim.py` — End-to-end flight director + CLI path test
- `replay_gate_test.py` — Replay-based gate validation
- `cli_test.py` — Non-destructive CLI command tests

**Specialized / Feature-specific** (run when the relevant subsystem changes):
- `eskf_gps_soak.py` — ESKF + GPS fusion validation (indoor/outdoor/walk modes)
- `accel_cal_6pos.py` — 6-position accelerometer calibration

**One-off / Retired** (Stage T specific, experiment-specific, or no longer maintained):
- All `stage_t_*` scripts
- `ack_stress_test.py`, `codegen_soak_test.py`, most `generate_*` scripts

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
- `PRE_FLIGHT_CHECKLIST.md` — Step 3 detail
- `VERIFICATION_OVERVIEW.md` — Complementary verification layers
- `AUDIT_REMEDIATION.md` — Historical fixes
- `AUDIT_CLEANUP_2026-05-06.md` — Three-category plan
- `RP2350_ERRATA.md` — Appendix A.2 reference
- `docs/SAD.md` — State machine for FMEA mapping

---

*End of master audit procedure. Update only when overall audit strategy changes.*