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
| 1 — SPIN station | Phase 1.7b | ✅ PASS (soft) | 2/2 (p_termination, p_no_double_clear); errors:0 each. Added mid-audit per user direction (SPIN doesn't need HW) |
| 1 — clang-tidy + lizard | Phase 1.8 | ⚠️ FAIL | 3,288 warnings + multiple CCN > 20 violations + Tier 3 unguarded printf in flight-critical drivers + Tier 4 missing Prior Art comment |
| 1 — cppcheck | Phase 1.9 | ⚠️ FAIL | 46 issues (1 cppcheck-config error, 5 real warnings incl. icm20948 nullptr deref, 33 style, 7 portability) |
| 1 — coverage | Phase 1.10 | ⚠️ PARTIAL | ESKF main file 65.27% (under pre-declared 80% threshold); AOs/drivers/calibration not in host scope |
| 1 — bench_sim vehicle | Phase 1.11 | ✅ PASS | 2/2 in 6.5s, vehicle flight v0.16.0 (kmenu) |
| 1 — bench_sim station | Phase 1.11b | N/A | Station hardware not on bench |
| 1 — replay_gate | Phase 1.12 | ⚠️ PARTIAL | Script ran but produced anti-evidence (None for ESKF/FD/pyro state) — soft gate per HW_GATE_DISCIPLINE.md Rule 4 |
| 2 — Phase 1 findings dispositions | Phase 2 | ✅ DONE (conversational) | All 11 Phase 1 findings dispositioned (REMEDIATE/DEFER/NEEDS REVIEW); recorded in Phase 2 below |
| 3 — Pre-flight gate | Phase 3 | ⏸ pending | HW gate including 3-boot reseat |
| 4 — FMEA-lite | Phase 4 | ⏸ pending | Agent walks FMEA-lite + Koopman + fault-injection; any FAIL/PARTIAL surfaces conversationally for user disposition |
| 5 — Stack/errata | Phase 5 | ⏸ pending | Agent runs stack-usage + errata grep + walks A.2 manual checklist; conversational dispositions |
| 6 — SPIN + model/source diff | Phase 6 | ⏸ pending | Coverage-evaluation pre-step + SPIN re-run + agent-staged model/source diff for user review |
| 7 — Traceability spot-check | Phase 7 | ⏸ pending | Agent walks traceability with raw-quote evidence per amendment 8; STALE/MISSING rows surface conversationally |
| 8 — Remediation + CHANGELOG | Phase 8 | ⏸ pending | Per-finding focused commits + final disposition wrap commit + CHANGELOG |
| 9 — Post-audit guided code review | Phase 9 | ⏸ pending (separate session) | Reading-discipline tips + guided source tour |

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

### 1.7b — SPIN station 2/2 ✅ PASS (soft gate)

`logs/audit-2026-05-07/03b_spin_station.log`

Station-side SPIN model `rocketchip_station.pml` verified against 2 LTL properties:

- `p_termination` — station eventually terminates (liveness): `errors: 0`, depth 44, state-vector 52 byte.
- `p_no_double_clear` — clear command at most once (safety): `errors: 0`, depth 63, state-vector 52 byte.

Added after user clarified that SPIN doesn't need HW — pure model verification. Per IVP-147 scaffolding (`AGENT_WHITEBOARD.md` notes). Same soft-gate classification as Phase 1.7 — proves model self-consistency, not C++/model correspondence (deferred to Phase 6 along with the vehicle-side AO model/source diff).

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

### 1.11b — bench_sim station N/A (deliberate scope)

Station hardware (Fruit Jam) not on the bench during this audit cycle by user direction: "anything that needs dedicated FJ testing we can do after all this has been gone through and remediated. There shouldn't be any major differences aside from small GPIO bugs from our experience so far." Single-vehicle Feather bench setup for this audit.

**Coverage of station code through other Phase 1 checks (what IS covered):**

- ✅ `verify_build_parity.sh` (Phase 1.5) — compiled `build_station/` and `build_station_flight/` clean; station code passes build gate.
- ✅ clang-tidy + lizard full-tree sweep (Phase 1.8) — covered station source files alongside vehicle.
- ✅ cppcheck (Phase 1.9) — covered station source files alongside vehicle.
- ✅ Host ctest (Phase 1.6) — station-side logic that has host tests gets exercised.
- ✅ **SPIN station model `rocketchip_station.pml` 2/2 PASS** (Phase 1.7b) — added mid-audit after user clarified SPIN doesn't need HW.

**On test-coverage for station-side code:**

The Phase 1.10 coverage report is host-test scope, which links `rc_flight_director` + `qep` libraries only. Same scope applies to vehicle-side AOs/drivers/CLI as to station-side: none are in the host test scope today, regardless of role. The 65.27% line coverage on `eskf.cpp` is the one threshold finding, and it's role-neutral. There is no specific "station has less coverage than vehicle" finding here — both have the same shape of gap, which is *anything in `src/active_objects/`, `src/drivers/`, `src/cli/`, `src/calibration/` is target-only and not in host scope.*

Extending host-test scope to cover station-side AOs (or vehicle-side AOs) is a substantial host-test-infrastructure project, not in this audit's scope. Target-side coverage instrumentation (running instrumented firmware on the FJ or Feather and collecting .profraw via SWD) is a larger lift, deferred.

**What remains deferred to post-remediation FJ pass:**

- ❌ Station-side runtime (no live FJ on bench): station_bench_sim, FJ banner capture, FJ 3-boot reseat, FJ HW status command output.
- ❌ FJ-specific code paths in Phase 4-7 that need observed firmware behavior (FMEA station-side runtime, station SPIN model/source diff, station traceability rows).

**Rationale for deferral:** Station and vehicle share the same source tree gated by `ROCKETCHIP_JOB_STATION=1` and `kRadioModeRx`. Build-time and source-level checks (build_parity, clang-tidy, cppcheck, ctest, SPIN) already exercised both roles. Runtime divergence in this project's history has been small GPIO/wiring bugs, not architectural deltas. A focused FJ-only pass after remediation is the right granularity.

### 1.12 — replay_gate_test ⚠️ PARTIAL

`logs/audit-2026-05-07/08_replay_gate.log`

Required reflashing vehicle to **bench tier** (`build/rocketchip-dev.elf`) — flight tier doesn't include the replay-injection dev hooks. Bench banner: `dev-ba52d38` (note: bench tier ELF was built before this audit's git-hash refresh; doesn't affect the test functionality).

Replay streamed 13,660 samples (90s nominal-flight profile) in 23.3s wall. Script exited 0, but the GDB-based gate signal inspection returned `None` for ESKF init / FD phase / pyro edges. **Per HW_GATE_DISCIPLINE.md Rule 1, exit-zero alone is anti-evidence**; the gate is structurally soft per Rule 4 because the positive-control signal mechanism didn't deliver.

Two interpretations possible: (a) GDB-state inspection raced with the running OpenOCD session (the script's own `monitor halt` calls collide with our existing connection), or (b) the replay-injection hooks didn't register the way the script expects in the dev tier. Phase 8 disposition: investigate as a remediation candidate (strengthen the gate's positive-control signal per HW_GATE_DISCIPLINE Rule 4 strengthening clause).

---

## Phase 2 — Phase 1 findings dispositions (conversational)

Phase 2 captures the user's per-finding dispositions for the Phase 1 results. These dispositions were given conversationally during this audit session when Phase 1 surfaced each finding — not via a separate "user walks blank tables" workflow. Disposition labels:

- **REMEDIATE** — fix as part of Phase 8 in this audit cycle.
- **ACCEPT** — user signs off as a permanent deviation; row added to `ACCEPTED_STANDARDS_DEVIATIONS.md`.
- **DEFER** — queue for a future post-audit remediation session.

### 2.1 — Currently-accepted deviations (entering this audit)

Per `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` post R-6b:

| ID | Rule | Location | Standard | Status |
|---|---|---|---|---|
| CG-1 | JSF AV Rule 1 (≤200 L-SLOC) | `src/fusion/eskf_codegen.cpp codegen_fpft()` | JSF AV (1) | Accepted via NASA SWE §8.11 auto-generated-code adjustment |
| FP-1 | P10 Rule 9 (no function pointers) | `src/calibration/calibration_manager.cpp lm_solve()` | P10 (newer; supersedes JSF 176 per precedence rule) | Accepted; template-dispatch remediation queued as R-6c |

### 2.2 — Phase 1 findings + dispositions

| Finding | Source | Disposition | Notes |
|---|---|---|---|
| `src/drivers/icm20948.cpp:540` — possible nullptr deref. `if (data == nullptr) memset(data, 0, ...)` dereferences `data` after null-check confirms it could be null. Real bug. | cppcheck | **REMEDIATE** | Per Phase 8 workflow; single-line fix. |
| `src/active_objects/ao_telemetry.cpp:1039` — `%u` format specifier for signed int argument. | cppcheck | **REMEDIATE** | Trivial cast or format change. |
| `src/dev/dev_cli.cpp:216` — 3× `%ld *` for `int *` (scanf-like format-pointer mismatch). | cppcheck | **REMEDIATE** | Trivial fix; dev-tier file is compile-excluded from flight binary, but fix anyway per project standards-rigor policy. |
| `src/flight_director/flight_director.cpp:33` — cppcheck doesn't know `Q_DEFINE_THIS_MODULE` macro. | cppcheck tool config | **REMEDIATE** | Add `--inline-suppr` or QP/C macro definition list to cppcheck config. Not a code bug. |
| `memset()` on structs containing floats — 7 sites across calibration, gps, icm20948. | cppcheck portability | KNOWN PATTERN; **DEFER** decision to Phase 8 with user | If user accepts the pattern, add a single accepted-deviation row covering all 7 sites. If not, remediate (rewrite as zero-initializer). Existing precedent leans accept. |
| clang-tidy: 3,288 raw warnings. | clang-tidy full sweep | **DEFER** | Per-tier breakdown deferred to a dedicated triage commit. Too many to disposition row-by-row in this audit. Compared against March 2026 baseline (~497 actionable after same exclusion set) requires applying the same exclusions before drawing conclusions. |
| lizard CCN > 20: `ao_led_engine.cpp led_apply_pattern` (CCN 42), `rc_os.cpp handle_calibration_menu` (CCN 29), `rc_os_commands.cpp cli_handle_unhandled_key` (CCN 31), `dev_cli.cpp dev_debug_menu_dispatch` (CCN 32), plus others. | lizard | **DEFER** | Known pattern (same files as March 2026 audit). Most are CLI dispatch tables (inherently cyclomatic). Per-row decision (accept / refactor / defer) deferred to Phase 8. |
| Tier 3 unguarded printf in flight-critical files: `src/drivers/i2c_bus.cpp` (16 in scan/diag), `src/flight_director/flight_director.cpp` (12 in `[FD]` log lines), `src/flight_director/go_nogo_checks.cpp` (2 in `[GO/NO-GO]`), `src/flight_director/guard_combinator.cpp` (1). | clang-tidy tier 3 | **NEEDS REVIEW at Phase 8** | The `[FD]` log lines ARE the positive-control signals per HW_GATE_DISCIPLINE.md Rule 1. Guarding them with `stdio_usb_connected()` would suppress the very signals that prove transitions fired. Resolution likely involves re-classifying the call sites OR documenting this exception. Not a simple remediation. |
| `src/drivers/mcu_temp.cpp` missing required Prior Art comment block. | clang-tidy tier 4 | **REMEDIATE** | Single comment-block addition. |
| `src/fusion/eskf.cpp` at 65.27% line coverage (under pre-declared 80% threshold). | gcovr coverage | **DEFER** | Identify uncovered functions/branches in a follow-up; decide tests / harness / IVPs at that time. Real coverage gap to address but not in this audit cycle. |
| replay_gate exited 0 but produced anti-evidence (None for ESKF/FD/pyro state from GDB). | replay_gate_test | **DEFER** | Strengthen the gate's positive-control signal mechanism per HW_GATE_DISCIPLINE Rule 4 strengthening clause. Queue as a new R-8 entry in remediation queue. |

**User dispositions captured conversationally during Phase 1 reporting** in this session. Any disposition change before Phase 8 commits is noted as a subsequent edit to this table.

### 2.3 — Power of 10 per-rule applicability snapshot

Reproduced from `standards/AUDIT_GUIDANCE.md` Appendix B.1 for at-a-glance reference. This snapshot is agent-produced from R-6b's standards-precedence work; no user fill-in needed.

| Rule | Statement (verbatim) | Project status |
|---|---|---|
| 1 | Restrict all code to very simple control flow constructs—no goto, setjmp/longjmp, or recursion | Complies (recursion-free; goto resolved by GT-1) |
| 2 | Give all loops a fixed upper bound | Complies via Holzmann scheduler exemption (QF_run, core1_sensor_loop, fault halt — inverted-rule form) |
| 3 | Do not use dynamic memory allocation after initialization | Complies (no heap after init_application; LL Entry 1) |
| 4 | Functions ≤ printable on one page | Enforced ≤60 lines via JSF Rule 1 + milestone clang-tidy sweep. Sole accepted deviation: CG-1 (auto-generated codegen, NASA SWE §8.11) |
| 5 | Assertion density ≥2 per function | Aspirational. Density not currently measured; out-of-scope for this audit |
| 6 | Declare data objects at smallest scope | Complies (file-scope statics; cross-core globals documented in `docs/MULTICORE_RULES.md`) |
| 7 | Caller checks return values; callee validates parameters | Mostly complies. Sensor-driver return checks per LL Entry 29; clang-tidy enforces `[[nodiscard]]` where annotated |
| 8 | Limit preprocessor to file inclusion + simple macros | Complies (constants are constexpr per PP-1; remaining `#define` are feature flags + SDK macros) |
| 9 | Limit pointer use to single dereference + no function pointers | One accepted deviation: FP-1 (lm_solve, Ground-classified). All other code complies. Remediation R-6c queued for future cycle |
| 10 | Compile with all warnings active; address before release | Complies (`-Wall -Wextra -Wpedantic -Werror` in production). CG-1 exempt from warning-as-error gate |

### 2.4 — Documentation sync issues surfaced during this audit

Issues affecting documentation, not code. Agent-produced from Phase 1 + audit setup.

| Issue | Source | Severity | Resolution |
|---|---|---|---|
| `standards/AUDIT_GUIDANCE.md` line 138 references `AUDIT_CLEANUP_2026-05-06.md` (3-category plan) — file does not exist | Phase 0 grep | Low | Either create the referenced file or update the reference. **DEFER** to Phase 8. |
| `docs/IVP.md:3047` (IVP-113 entry) still cites JSF Rule 170 for the no-vtable design choice. Per R-6/R-6b correction, the right citation is P10 Rule 9. | R-6b cleanup | Low | Historical-record IVP entry; covered by NOTIFY_CONTRACT.md supersession note that R-6b applied. No further edit needed. |

---

## Phase 3 — Pre-Flight Gate Execution (HW)

⏸ Pending. Will walk `docs/PRE_FLIGHT_CHECKLIST.md` items 1–13 sequentially with 3-boot reseat protocol per HW_GATE_DISCIPLINE.md Rule 2.

---

## Phase 4 — Safety-Critical Path Review (FMEA-lite + Koopman)

⏸ Pending. Agent walks the FMEA-lite table from Appendix A.1, applies the Koopman 5-rule one-pager (Appendix B.2) to 3–5 flight-critical functions, and runs `enhanced_fault_injection.py` scenarios. Records PASS/FAIL/PARTIAL per row with evidence. Any FAIL/PARTIAL rows surface conversationally for user disposition (REMEDIATE / ACCEPT / DEFER).

---

## Phase 5 — Stack / Memory / RP2350 Errata Deep Review

⏸ Pending. Agent runs stack-usage build + errata grep (E2/E9/E11/E12), walks the A.2 manual checklist (ISR allocs, flash ops in BOOST window, Core 1 polling budget, PSRAM ring buffer wrap, new compile flags). Records PASS/FAIL per row with evidence. Any FAIL/PARTIAL rows surface conversationally for user disposition.

---

## Phase 6 — Formal Verification + Simulation Coverage

⏸ Pending. SPIN coverage-evaluation pre-step (per user-added 23b) considers candidate model extensions for BOTH vehicle and station roles:

**Vehicle-side candidate:** Cross-core boot handshake (R-1 / BM-2 in remediation queue) — Core 0 sets `g_startSensorPhase` while Core 1 waits. Textbook SPIN-shaped (two-process model + shared atomic). Not currently modeled in `rocketchip_ao.pml`.

**Station-side candidates** (per `AGENT_WHITEBOARD.md` "Station SPIN model extensions" item — extend when corresponding firmware behavior lands):
- multi-pending-in-flight (multiple radio commands in flight at once)
- RadioScheduler TX-window arbitration (needed for the sync-gap fix)
- MAVLink parser state
- `station_idle_tick` GPS poll interleave

Re-run existing models (vehicle AO 11/11 baseline + station 2/2 baseline). Then agent stages model/source diff between current `ao_flight_director.cpp` HSM ↔ `rocketchip_ao.pml` AND current station logic ↔ `rocketchip_station.pml` for user review of correspondence (the hard claim Phase 6 makes about firmware correctness — the SPIN re-runs alone are soft gates per amendment 4).

---

## Phase 7 — Requirements Traceability Spot-Check

⏸ Pending. Agent walks the requirements traceability check for 10–15 flight-critical items. Per amendment 8, raw quote + verification recorded per row (CONFIRMED / STALE / MISSING). STALE/MISSING rows surface conversationally for user disposition.

---

## Phase 8 — Findings Disposition + Remediation Section

⏸ Pending. Per-remediation focused commits (user-stated preference): icm20948 nullptr fix → commit; ao_telemetry/dev_cli format-string fixes → commit; mcu_temp.cpp Prior Art comment block → commit; etc. Final Phase 8 wrap commit records the disposition table in the `## Remediation` section below + the CHANGELOG entry.

## Remediation

This section is populated during Phase 8. Pre-staged remediation queue from Phase A.2 lives in `MASTER_STANDARDS_AUDIT_2026-05-07_REMEDIATION_PRELIMINARY.md` (R-1 through R-7c — R-6 and R-6b already DONE during audit setup).

---

## Phase 9 — Post-Audit Guided Code Review

⏸ Pending; happens after Phase 8 completes and after some/all of the queued remediations have been applied (timing user-chosen). This is NOT part of the scripted master audit cycle — it's a post-audit handoff supporting the user's eventual line-by-line read of the safety-critical code.

Two deliverables the agent stages for the user's read:

1. **Reading-discipline tips** — practical advice tailored to this codebase: what to grep first, how to prioritize, how to spot comment-vs-implementation drift, leveraging the agent-produced findings as a starting point, what's auto-detectable vs. what only a human catches, realistic time estimates.

2. **Guided source tour** — agent pre-reads flight-critical paths and stages an annotated walking order (15–25 waypoints), with audit-finding annotations alongside surrounding-context narration. User walks the agent's tour rather than the code raw.

Both deliverables live in `docs/audits/MASTER_STANDARDS_AUDIT_2026-05-07_MANUAL_REVIEW_GUIDE.md` (or as appendices here, user preference). Filled when Phase 9 begins.

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
