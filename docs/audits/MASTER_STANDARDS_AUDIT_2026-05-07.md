# Master Standards Audit — 2026-05-07

**Audit Date:** 2026-05-07
**Audited By:** Claude Opus 4.7 (scripted phases) + user (manual review phases)
**Codebase Snapshot:** `e0a307e` (post Phase A/A.2/R-6/R-6b/Phase B; flight-e0a307e on bench)
**Sources Audited:** 72 `.cpp` + 76 `.h` files = 25,571 LOC of production source
**Plan:** `.claude/plans/streamed-launching-lagoon.md`
**Procedure:** `standards/AUDIT_GUIDANCE.md` (8-step master procedure + Appendix A.1/A.2/B)

**Tools:**
- clang-tidy 21.1.8 (LLVM)
- lizard 1.21.2 (Python — cyclomatic complexity)
- cppcheck 2.20.0 (C/C++ static analysis)
- gcovr 8.6 + llvm-cov + llvm-profdata (coverage)
- spin 6.5.2 (formal verification)
- Pico SDK GDB 14_2_Rel1 + OpenOCD 0.12.0+dev (HW probe)

---

## Audit Series (six setup commits before Phase 0)

This audit's setup phase produced six commits to clean up the standards documentation and re-evaluate accepted deviations before the master procedure ran. They are part of this audit's record:

| Commit | Phase | Summary |
|---|---|---|
| `663cd0e` | A | Rename `STANDARDS_DEVIATIONS.md` → `ACCEPTED_STANDARDS_DEVIATIONS.md` + ripple to 11 protected docs |
| `bf9d393` | A.2 | Re-evaluate 13 accepted deviations against current code; preliminary remediation queue created |
| `d186fc9` | R-6 | Correct JSF AV Rule 170 misreading proliferation across 6 files |
| `122b31e` | R-6b | Reattribute FP-1 to P10 Rule 9 + codify standards-precedence rule in `CODING_STANDARDS.md` |
| `e0a307e` | B | Append Appendix B to `AUDIT_GUIDANCE.md` (P10/Koopman/FMEA/walking procedure inline) |

The R-6 → R-6b near-miss (initial misreading correction nearly erased a real deviation) is documented in `LESSONS_LEARNED.md` Entry 37 as a procedural defense.

---

## Executive Summary

| Step | Section | Status | Headline |
|---|---|---|---|
| 1 — Build parity | Phase 1.5 | ✅ PASS | All 4 build tiers compile clean |
| 1 — Host ctest | Phase 1.6 | ✅ PASS | 788/788 in 7.69s |
| 1 — SPIN AO | Phase 1.7 | ✅ PASS (soft) | `SPIN_OK_ALL_11` (1.08s); model self-consistency only — see Phase 6 model/source diff |
| 1 — clang-tidy + lizard | Phase 1.8 | ⚠️ FAIL | 3,288 warnings + multiple CCN > 20 violations + Tier 3 unguarded printf in flight-critical drivers + Tier 4 missing Prior Art comment |
| 1 — cppcheck | Phase 1.9 | ⚠️ FAIL | 46 issues (1 cppcheck-config error, 5 real warnings incl. icm20948 nullptr deref, 33 style, 7 portability) |
| 1 — coverage | Phase 1.10 | ⚠️ PARTIAL | ESKF main file 65.27% (under pre-declared 80% threshold); AOs/drivers/calibration not in host scope |
| 1 — bench_sim vehicle | Phase 1.11 | ✅ PASS | 2/2 in 6.5s, vehicle flight v0.16.0 (kmenu) |
| 1 — bench_sim station | Phase 1.11b | N/A | Station hardware not on bench |
| 1 — replay_gate | Phase 1.12 | ⚠️ PARTIAL | Script ran but produced anti-evidence (None for ESKF/FD/pyro state) — soft gate per HW_GATE_DISCIPLINE.md Rule 4 |
| 2 — Standards walk | Phase 2 | ⏸ AWAITING USER | Step 2 STANDARDS_AUDIT walk staged below |
| 3 — Pre-flight gate | Phase 3 | ⏸ pending | HW gate including 3-boot reseat |
| 4 — FMEA-lite | Phase 4 | ⏸ AWAITING USER | A.1 FMEA staged for user review; agent runs enhanced_fault_injection scenarios |
| 5 — Stack/errata | Phase 5 | ⏸ pending | Agent runs stack-usage + errata grep; user walks A.2 manual checklist |
| 6 — SPIN + model/source diff | Phase 6 | ⏸ AWAITING USER | Coverage-evaluation pre-step + SPIN re-run + model/source diff |
| 7 — Traceability spot-check | Phase 7 | ⏸ AWAITING USER | 10–15 row table staged with raw-quote evidence per amendment 8 |
| 8 — Remediation + CHANGELOG | Phase 8 | ⏸ pending | Findings disposition; embedded `## Remediation` section |

---

## Phase 0 — Pre-flight setup

**Tool PATH check:** clang-tidy 21.1.8, lizard 1.21.2, cppcheck 2.20.0, spin 6.5.2, python 3.13.5, gcovr 8.6 (installed during audit), llvm-cov + llvm-profdata. All present.

**Cat 2 self-tests:** 5/5 PASS (analyze_stack_usage, run_cppcheck, verify_build_parity, generate_coverage_report, enhanced_fault_injection). Re-verified at audit start.

**OpenOCD:** Started on `127.0.0.1:3333`. CMSIS-DAP probe serial `E663AC91D3487137`, SWD speed 5000 kHz.

**Banner captured (post-rebuild with current git hash):**
```
==============================================
  RocketChip v0.16.0  RCOS v0.5.0  flight-e0a307e
  Board: Adafruit Feather RP2350 HSTX
  Profile: Rocket  Uptime: 5s
==============================================
Hardware: 14/14 OK
Flash: 4 flights, 27% used
```

Build tag matches HEAD (`e0a307e`). Hardware status 14/14 OK.

---

## Phase 1 — Baseline Scripted Sweeps

All raw outputs in `logs/audit-2026-05-07/`. Each verdict cited per HW_GATE_DISCIPLINE.md Rule 3.

### 1.5 — verify_build_parity.sh ✅ PASS

`logs/audit-2026-05-07/01_build_parity.log`

All 4 build variants compiled cleanly:
- `build/` (vehicle bench, NOT_CERTIFIED_FOR_FLIGHT=ON)
- `build_flight/` (vehicle flight)
- `build_station/` (station bench)
- `build_station_flight/` (station flight)

Verdict: `VERDICT: PASS — All available build variants compiled successfully`

### 1.6 — Host ctest 788/788 ✅ PASS

`logs/audit-2026-05-07/02_ctest.log`

`100% tests passed, 0 tests failed out of 788` in 7.69s. Including 2 Python script gates and 786 C++ tests.

### 1.7 — SPIN AO 11/11 ✅ PASS (soft gate)

`logs/audit-2026-05-07/03_spin_ao.log`

`SPIN_OK_ALL_11` emitted. 1.08s wall, 280 MB peak memory, 1,684,540 states/second. All 11 LTL properties verified:

- `p_no_pyro_idle` — pyro never fires in IDLE
- `p_drogue_before_main` — drogue must precede main
- `p_pyro_requires_armed` — no pyro without ARMED
- `p_drogue_once` — drogue fires at most once
- `p_main_once` — main fires at most once
- `p_logger_gets_all` — logger sees every signal
- `p_telem_gets_all` — telemetry sees every signal
- `p_led_gets_all` — LED engine sees every signal
- `p_fault_blocks_arm` — FAULT state blocks arming
- `p_fault_latch_holds` — FAULT latch is sticky
- `p_armed_fault_safe_mode` — armed→fault→safe-mode liveness

**Soft gate per HW_GATE_DISCIPLINE.md Rule 4 + amendment 4.** This proves model self-consistency — that the 11 LTL properties hold over the SPIN model — but it does NOT prove the C++ in `src/active_objects/ao_flight_director.cpp` matches the model. The model/source correspondence is verified by Phase 6 step 24b (manual diff staged for user review).

### 1.8 — clang-tidy + lizard ⚠️ FAIL

`logs/audit-2026-05-07/04_clang_tidy.log` + `logs/clang-tidy-2026-05-11/`

**Verdict: `VERDICT: FAIL — clang-tidy: 3288 warnings across 71 files`**

Comparison to March 2026 baseline (`STANDARDS_AUDIT_2026-03-26.md`):
- March: 2,315 raw warnings → ~497 actionable after excluding codegen (1,529) + benchmarks (289) + remediated tier-1 fixes
- May: 3,288 raw warnings — apples-to-apples comparison requires same exclusion set; net change unclear from raw count alone. Detailed comparison deferred to Phase 8.

**Tier 2 (lizard cyclomatic complexity):** Multiple functions exceed CCN > 20 across AOs and CLI. Notable:
- `src/active_objects/ao_led_engine.cpp` `led_apply_pattern` — CCN 42, 46 NLOC
- `src/cli/rc_os.cpp` `handle_calibration_menu` — CCN 29, 60 NLOC
- `src/cli/rc_os.cpp` `handle_flight_menu` — CCN 27, 26 NLOC
- `src/cli/rc_os_commands.cpp` `cli_handle_unhandled_key` — CCN 31, 61 NLOC
- `src/dev/dev_cli.cpp` `dev_debug_menu_dispatch` — CCN 32, 84 NLOC

These are pre-existing — the same files surfaced in March 2026 audit. Most are CLI dispatch tables (large but inherently cyclomatic). Per `standards/CODING_STANDARDS.md` file-classification, the CLI files are Ground-classified.

**Tier 3 (RP2350 platform guards — Flight-Critical files only):** Unguarded `printf` in flight-critical paths:

- `src/drivers/i2c_bus.cpp:104-155` — 16 printf calls in I2C bus scan/diag paths. Per file-classification table in `CODING_STANDARDS.md`, `i2c_bus_scan()` is explicitly Ground-only; the read/write/probe path is Flight-Critical. Need per-callsite review to confirm Ground-only callers.
- `src/flight_director/flight_director.cpp:66-591` — 12 printf calls in `[FD]` log lines. These are the positive-control signals per HW_GATE_DISCIPLINE.md Rule 1. They emit during state transitions. Need review: are these guarded by `stdio_usb_connected()` or do they violate the rule?
- `src/flight_director/go_nogo_checks.cpp:100,112` — 2 printf in `[GO/NO-GO]` output. Same question as above.
- `src/flight_director/guard_combinator.cpp:154` — 1 printf in `[FD] WARN`.

**Tier 4 (Prior Art):** `src/drivers/mcu_temp.cpp` missing the Prior Art comment block required by `standards/CODING_STANDARDS.md` "Prior Art Research" section. Single small fix.

### 1.9 — cppcheck ⚠️ FAIL

`logs/audit-2026-05-07/05_cppcheck.log` + `logs/cppcheck-2026-05-11/cppcheck.txt`

**Verdict: `VERDICT: FAIL — cppcheck: 46 issues across 71 files`**

Severity breakdown:
- **error: 1** — `src/flight_director/flight_director.cpp:33` "unknown macro `Q_DEFINE_THIS_MODULE`" — cppcheck doesn't know the QP/C macro definition. **This is a cppcheck-config issue, not a code bug.** Suppress in cppcheck config or add `--inline-suppr` per QP/C macro.
- **warning: 5** — REAL FINDINGS:
  - `src/active_objects/ao_telemetry.cpp:1039` — `%u` for signed int (format-string mismatch). Easy fix.
  - `src/dev/dev_cli.cpp:216` — 3× `%ld *` for `int *` (pointer-format mismatch in scanf-like call). Easy fix.
  - `src/drivers/icm20948.cpp:540` — **possible nullptr dereference** in `icm20948_read(dev, data)`: the early-return path checks `data == nullptr` then immediately calls `memset(data, 0, ...)`, dereferencing the pointer the check just confirmed could be null. **Real bug candidate.** Recommended fix: only memset if `data != nullptr`, OR remove the memset (the function returns false anyway).
- **portability: 7** — `memset()` on structs containing floats. Across calibration_data, calibration_manager, gps drivers, icm20948. Mostly pre-existing pattern; documented as MISRA-C 2012 accepted use.
- **style: 33** — Redundant initializations, always-true/false conditions, etc. Most pre-existing; need triage.

### 1.10 — Coverage report ⚠️ PARTIAL

`logs/audit-2026-05-07/06_coverage.txt` + `build_host_cov/coverage.profdata`

Built with `clang -fprofile-instr-generate -fcoverage-mapping` (Debug, -O0). 470 .profraw files merged via llvm-profdata. Per-source-file report via llvm-cov over 41 test executables.

**Pre-declared threshold (per audit plan):** any file under `src/active_objects/` or `src/fusion/` (ESKF) below 80% line coverage = FAIL row.

**Results:**

| Area | Files | Coverage |
|---|---|---|
| `src/fusion/eskf.cpp` | 1 | **65.27% line — UNDER THRESHOLD** |
| `src/fusion/eskf_brake.cpp` | 1 | 100.00% line |
| `src/fusion/eskf_codegen.cpp` | 1 | 100.00% line |
| `src/flight_director/*.cpp` | 6 | 87.78% – 100% line, all above threshold |
| `src/active_objects/*.cpp` | 0 | **Not in host test scope** — target-only, not coverage-instrumentable from host |
| `src/drivers/*.cpp` | 0 | **Not in host test scope** — target-only |
| `src/calibration/*.cpp` | 0 | **Not in host test scope** — target-only |

**Two findings for Phase 8:**

1. **`eskf.cpp` 65.27% — under the 80% threshold.** Identify which specific functions/branches aren't tested and decide: extend tests, accept the gap, or document a planned remediation IVP.

2. **AO/driver/calibration coverage is structurally unmeasurable from host tests** because those modules link only on the target build. The threshold check as written can't apply to them without on-device coverage instrumentation (a much larger lift). Two options for next audit cycle: (a) revise the threshold to reference only host-buildable files, or (b) add target-side coverage as a separate audit pass.

### 1.11 — bench_sim vehicle ✅ PASS

`logs/audit-2026-05-07/07_bench_sim_vehicle.log`

```
=== RocketChip Bench Sim ===
  Port: COM7
  Classified: vehicle flight v0.16.0 (kmenu)
  Max runtime: 120s

  [PASS]  1. Happy path (nominal flight)
  [PASS]  2. Abort from BOOST (no pyro under default profile)

  RESULT: 2/2 PASS  (6.5s)
```

Positive-control signals observed: `[FD] PYRO FIRED: DROGUE (primary)` (happy path), `[FD] ABORT from BOOST — drogue pyro intent` (abort path).

### 1.11b — bench_sim station N/A

Station hardware not present on bench during this audit cycle. Single-vehicle bench setup. Documented as gap; future audits with full vehicle+station bench should run station_bench_sim.

### 1.12 — replay_gate_test ⚠️ PARTIAL

`logs/audit-2026-05-07/08_replay_gate.log`

Required reflashing vehicle to **bench tier** (`build/rocketchip-dev.elf`) — flight tier doesn't include the replay-injection dev hooks. Bench banner: `dev-ba52d38` (note: bench tier ELF was built before this audit's git-hash refresh; doesn't affect the test functionality).

Replay streamed 13,660 samples (90s nominal-flight profile) in 23.3s wall. Script exited 0, but the GDB-based gate signal inspection returned `None` for ESKF init / FD phase / pyro edges. **Per HW_GATE_DISCIPLINE.md Rule 1, exit-zero alone is anti-evidence**; the gate is structurally soft per Rule 4 because the positive-control signal mechanism didn't deliver.

Two interpretations possible: (a) GDB-state inspection raced with the running OpenOCD session (the script's own `monitor halt` calls collide with our existing connection), or (b) the replay-injection hooks didn't register the way the script expects in the dev tier. Phase 8 disposition: investigate as a remediation candidate (strengthen the gate's positive-control signal per HW_GATE_DISCIPLINE Rule 4 strengthening clause).

---

## Phase 2 — Step 2 STANDARDS_AUDIT walk

> **⏸ AWAITING USER REVIEW.** Per `standards/AUDIT_GUIDANCE.md` Appendix B.4 walking procedure for Phase 2 (~45 min). Read first: this section's pre-staged tables + Phase 1 findings above + `standards/STANDARDS_AUDIT.md` template + `standards/CODING_STANDARDS.md` Foundation section (the standards-precedence rule installed in R-6b).

**Walking instructions** (from `AUDIT_GUIDANCE.md` Appendix B.4):

1. Top-to-bottom through the dashboard below.
2. For each row, decide **PASS / FAIL / PARTIAL / NOT CHECKED / N/A** per the existing template legend.
3. For any FAIL or PARTIAL row, note: (a) file(s) affected, (b) whether it's a known accepted deviation (cross-reference the deviations diff below), (c) whether it's a candidate new deviation requiring user sign-off, (d) whether it's a candidate remediation.
4. Default disposition is REMEDIATE per the policy in `ACCEPTED_STANDARDS_DEVIATIONS.md` — only mark something as a candidate accepted deviation if remediation is genuinely a major stumbling block.

**Done looks like:** every row has a status code; FAIL/PARTIAL rows have at least a one-line note.

### 2.1 — Summary Dashboard (reproduced from `standards/STANDARDS_AUDIT.md` line 70-82)

| Standard          | Total | Applicable | Compliant | Deviations | Not Checked | Status | Notes |
| ----------------- | ----- | ---------- | --------- | ---------- | ----------- | ------ | ----- |
| Power of 10       | 10    | 10         |           |            |             |        |       |
| JSF AV C++        | 221   | ~150       |           |            |             |        |       |
| JPL C (LOC-1-6)   | ~31   | ~25        |           |            |             |        |       |
| Platform Rules    | ~12   | ~12        |           |            |             |        |       |
| Multicore         | ~5    | ~5         |           |            |             |        |       |
| Debug Output      | ~20   | ~20        |           |            |             |        |       |
| Git Workflow      | ~14   | ~14        |           |            |             |        |       |
| Session Checklist | ~12   | ~12        |           |            |             |        |       |

Status legend: **PASS** = all applicable rules in the category compliant; **FAIL** = one or more violations not yet covered by an accepted deviation; **PARTIAL** = some compliant + some deviations; **NOT CHECKED** = category not yet sampled this audit; **N/A** = entire category doesn't apply (rare).

### 2.2 — Phase 1 findings vs. existing accepted deviations (diff)

**Currently-accepted deviations** (per `standards/ACCEPTED_STANDARDS_DEVIATIONS.md`, post R-6b):

| ID | Rule | Location | Standard | Status |
|---|---|---|---|---|
| CG-1 | JSF AV Rule 1 (≤200 L-SLOC) | `src/fusion/eskf_codegen.cpp codegen_fpft()` | JSF AV (1) | Accepted via NASA SWE §8.11 auto-generated-code adjustment |
| FP-1 | P10 Rule 9 (no function pointers) | `src/calibration/calibration_manager.cpp lm_solve()` | P10 (newer; supersedes JSF 176 per precedence rule) | Accepted; template-dispatch remediation queued as R-6c |

**Phase 1 candidate findings to disposition** (each row asks: known deviation, new acceptance candidate, or remediation?):

| Source | Finding | Likely Standard | Suggested disposition (USER decides) |
|---|---|---|---|
| cppcheck warning | `src/drivers/icm20948.cpp:540` — `if (data == nullptr) memset(data, 0, ...)` dereferences `data` after null-check confirms it could be null. **Real bug candidate.** | Defensive coding / general C++ correctness | **REMEDIATE NOW** (recommended) — single-line fix, real nullptr-deref bug |
| cppcheck warning | `src/active_objects/ao_telemetry.cpp:1039` — `%u` format specifier for signed int argument | C/C++ format-string correctness | REMEDIATE — trivial cast or format change |
| cppcheck warning | `src/dev/dev_cli.cpp:216` — 3× `%ld *` for `int *` (scanf-like format-pointer mismatch) | C/C++ format-string correctness | REMEDIATE — trivial fix; dev tier (compile-excluded from flight binary, lower urgency) |
| cppcheck error (config) | `src/flight_director/flight_director.cpp:33` — cppcheck doesn't know `Q_DEFINE_THIS_MODULE` macro | Tool config, not code | REMEDIATE cppcheck config — add `--inline-suppr` or QP/C macro definition list |
| cppcheck portability ×7 | `memset()` on structs containing floats (calibration, gps, icm20948) | MISRA-C 2012 portability | KNOWN PATTERN — ACCEPTED across drivers per existing precedent. New deviation row needed if you want to formalize |
| clang-tidy (3,288 raw) | Per-tier breakdown deferred; baseline comparison vs March 2026 (~497 actionable) needs same exclusion set applied | JSF AV / P10 / JPL C mixed | **DEFER** to dedicated triage commit — too many to disposition row-by-row in this audit |
| lizard CCN > 20 | `ao_led_engine.cpp led_apply_pattern` (CCN 42), `rc_os.cpp handle_calibration_menu` (CCN 29), `rc_os_commands.cpp cli_handle_unhandled_key` (CCN 31), `dev_cli.cpp dev_debug_menu_dispatch` (CCN 32), and others | JSF AV Rule 3 / P10 Rule 4 | KNOWN PATTERN (same files as March 2026) — most are CLI dispatch tables (inherently cyclomatic). Decide per-row: accept-as-deviation, refactor, or defer |
| Tier 3 unguarded printf | `src/drivers/i2c_bus.cpp` (16 calls in scan/diag), `src/flight_director/flight_director.cpp` (12 calls in `[FD]` log lines), `src/flight_director/go_nogo_checks.cpp` (2 in `[GO/NO-GO]`), `src/flight_director/guard_combinator.cpp` (1) | JSF Rule 22 / Platform Rule (USB CDC guard) | **NEEDS PER-CALLSITE REVIEW.** Per-file classification: i2c_bus_scan is Ground (per CODING_STANDARDS table); flight_director `[FD]` log lines ARE the positive-control signals per HW_GATE_DISCIPLINE.md Rule 1 — guarding them with `stdio_usb_connected()` would suppress the very signal that proves a transition fired. Resolution may be re-classifying or documenting this exception. |
| Tier 4 missing Prior Art | `src/drivers/mcu_temp.cpp` missing required Prior Art comment block | CODING_STANDARDS.md "Prior Art Research" section | REMEDIATE — single comment block addition |
| coverage | `src/fusion/eskf.cpp` at 65.27% line coverage (under 80% pre-declared threshold) | Test-coverage target | Identify uncovered functions/branches; decide: extend tests OR document tested-via-replay-harness gap OR queue specific extension IVPs |
| replay_gate (PARTIAL) | Script exited 0 but produced anti-evidence (None for ESKF/FD/pyro state from GDB) per HW_GATE_DISCIPLINE Rule 1 | HW gate discipline | Strengthen the gate's positive-control signal mechanism (Rule 4 strengthening clause). Investigate GDB-state inspection race with our existing OpenOCD session. Queue as R-8 in remediation. |

### 2.3 — Section A (JSF AV C++) — sub-section status

These are sub-sections within the Section A audit table from `standards/STANDARDS_AUDIT.md` line 87-103. Status columns blank for user fill-in.

| Sub-section | Description | Status | Notes |
|---|---|---|---|
| A.1 — N/A rules for C files | ~60 JSF rules N/A to .c files (classes, namespaces, templates, exceptions). Codebase is now all .cpp per PP-1 resolution — recheck applicability count |  |  |
| A.2 — SDK Interface Constraints | Pico SDK functions at API boundary; documented per-function justification |  |  |
| A.3 — Applicable rules audit tables | Rule-by-rule status table (~150 rules). Sample the high-impact rules (assertion density, fixed-width types, exception handling, RTTI) per AUDIT_GUIDANCE Appendix B.4 |  |  |
| A.4 — main.cpp C++ assessment | main.cpp full C++ ruleset audit |  |  |

### 2.4 — Section B (Power of 10) — see Appendix B.1 in `AUDIT_GUIDANCE.md`

Each P10 rule with project-applicability noted in `standards/AUDIT_GUIDANCE.md` Appendix B.1. Reproduce the per-rule status here:

| Rule | Statement (verbatim) | Project status (from B.1) | User confirms? |
|---|---|---|---|
| 1 | Restrict all code to very simple control flow constructs—no goto, setjmp/longjmp, or recursion | Complies (recursion-free; goto resolved by GT-1) |  |
| 2 | Give all loops a fixed upper bound | Complies via Holzmann scheduler exemption (QF_run, core1_sensor_loop, fault halt — inverted-rule form) |  |
| 3 | Do not use dynamic memory allocation after initialization | Complies (no heap after init_application; LL Entry 1) |  |
| 4 | Functions ≤ printable on one page | Enforced ≤60 lines via JSF Rule 1 + milestone clang-tidy sweep. Sole accepted deviation: CG-1 (auto-generated codegen, NASA SWE §8.11) |  |
| 5 | Assertion density ≥2 per function | Aspirational. Density not currently measured; out-of-scope for this audit |  |
| 6 | Declare data objects at smallest scope | Complies (file-scope statics; cross-core globals documented in MULTICORE_RULES.md) |  |
| 7 | Caller checks return values; callee validates parameters | Mostly complies. Sensor-driver return checks per LL Entry 29; clang-tidy enforces `[[nodiscard]]`. Audit cycles should grep for `(void)`-cast discards |  |
| 8 | Limit preprocessor to file inclusion + simple macros | Complies (constants are constexpr per PP-1; remaining `#define` are feature flags + SDK macros) |  |
| 9 | Limit pointer use to single dereference + no function pointers | One accepted deviation: FP-1 (lm_solve, Ground-classified). All other code complies. Remediation R-6c queued |  |
| 10 | Compile with all warnings active; address before release | Complies (`-Wall -Wextra -Wpedantic -Werror` in production). CG-1 exempt from warning-as-error gate |  |

### 2.5 — Section C (JPL C) — sub-section status

Per AUDIT_GUIDANCE template line 113-115: LOC-1 through LOC-4 mandatory, LOC-5/6 deferred.

| LOC level | Description | Status | Notes |
|---|---|---|---|
| LOC-1 (Language Compliance) | Mandatory C99/C11 conformance |  |  |
| LOC-2 (Predictable Execution) | Bounded loops, no recursion, no dynamic memory after init |  |  |
| LOC-3 (Defensive Coding) | Fixed-width types, asserts, return-value checks |  |  |
| LOC-4 (Code Clarity) | Single-purpose functions, named constants |  |  |
| LOC-5 (Required Tools) | Deferred — tooling-investment scope |  |  |
| LOC-6 (Verification) | Deferred — formal-verification scope |  |  |

### 2.6 — Section D (Project-Specific Rules) — sub-section status

Per AUDIT_GUIDANCE template line 119-121.

| Sub-section | Description | Status | Notes |
|---|---|---|---|
| D.1 — Platform Constraints | RP2350 bare-metal rules (LL Entry 1, 4, 12, 15, 28, 31). See CODING_STANDARDS.md Platform Constraints section |  |  |
| D.2 — Multicore Rules | docs/MULTICORE_RULES.md compliance |  |  |
| D.3 — Debug Output | standards/DEBUG_OUTPUT.md compliance (USB CDC guard pattern, debug macros, build iteration tags) |  |  |
| D.4 — Prior Art | Driver files have Prior Art comment block (CODING_STANDARDS.md "Prior Art Research" section). **Phase 1 Tier 4 found 1 missing: `src/drivers/mcu_temp.cpp`** |  |  |
| D.5 — Safety | Pyro channel safety, watchdog requirements (CODING_STANDARDS.md Safety section) |  |  |
| D.6 — Git Workflow | standards/GIT_WORKFLOW.md compliance |  |  |
| D.7 — Session Management | .claude/SESSION_CHECKLIST.md compliance |  |  |

### 2.7 — Section F (Documentation Sync Issues)

Issues found during audit that affect docs, not code. Pre-loaded from Phase 1 + this audit's setup phase:

| Issue | Source | Severity | Resolution |
|---|---|---|---|
| `standards/AUDIT_GUIDANCE.md` line 138 references `AUDIT_CLEANUP_2026-05-06.md` (3-category plan) — file does not exist | Phase 0 grep | Low | Either create the referenced file or update the reference. Defer to Phase 8 |
| `docs/IVP.md:3047` (IVP-113 entry) still cites JSF Rule 170 for the no-vtable design choice. Per R-6/R-6b correction, the right citation is P10 Rule 9. NOTIFY_CONTRACT.md supersession note covers the chain | R-6b cleanup | Low | Historical-record IVP entry; covered by NOTIFY_CONTRACT.md supersession note. No further edit needed |
| (User: add any new sync issues you find during the walk) |  |  |  |

### 2.8 — User sign-off

When done walking 2.1–2.7:
- [ ] All Status columns filled in (no blank cells)
- [ ] FAIL/PARTIAL rows annotated with disposition recommendation
- [ ] Any new accepted-deviation candidates flagged for Phase 8
- [ ] Any new remediation items flagged for Phase 8 Remediation section

**User signature line:**
- Walked by: ____________________
- Date: 2026-05-__
- Outcome: [PASS / FAIL / PARTIAL]

---

## Phase 3 — Pre-Flight Gate Execution (HW)

⏸ Pending. Will walk `docs/PRE_FLIGHT_CHECKLIST.md` items 1–13 sequentially with 3-boot reseat protocol per HW_GATE_DISCIPLINE.md Rule 2.

---

## Phase 4 — Safety-Critical Path Review (FMEA-lite + Koopman)

⏸ Pending. Agent stages FMEA-lite table + runs enhanced_fault_injection scenarios. User walks the FMEA + Koopman 5-rule per Appendix B.4.

---

## Phase 5 — Stack / Memory / RP2350 Errata Deep Review

⏸ Pending. Agent runs stack-usage build + errata grep. User walks A.2 manual checklist.

---

## Phase 6 — Formal Verification + Simulation Coverage

⏸ Pending. SPIN coverage-evaluation pre-step (per user-added 23b — boot handshake R-1/BM-2 candidate), then re-run SPIN, then model/source diff staging.

---

## Phase 7 — Requirements Traceability Spot-Check

⏸ Pending. Agent stages 10–15 row table with raw-quote evidence per amendment 8.

---

## Phase 8 — Findings Disposition + Remediation Section

⏸ Pending. User reviews each FAIL/PARTIAL row from Phase 1 + manual phases. Per-row decision: remediate-now / accept-as-deviation (signs into ACCEPTED_STANDARDS_DEVIATIONS.md) / defer (queued in `## Remediation` section below).

## Remediation

This section is populated during Phase 8. Pre-staged remediation queue from Phase A.2 lives in `MASTER_STANDARDS_AUDIT_2026-05-07_REMEDIATION_PRELIMINARY.md` (R-1 through R-7c — R-6 and R-6b already DONE during audit setup).

---

## Appendices

### Appendix A — PRE_FLIGHT_CHECKLIST walk

⏸ Filled during Phase 3.

### Appendix B — FMEA-lite table

⏸ Filled during Phase 4.

### Appendix C — Stack analyzer + manual errata checklist

⏸ Filled during Phase 5.

### Appendix D — Bench-sim and 3-boot reseat banner captures

Phase 0 banner already captured above. Phase 3 will add per-boot banners from the 3-boot reseat.
