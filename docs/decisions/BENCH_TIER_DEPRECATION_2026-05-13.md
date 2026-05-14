# Bench-Tier Deprecation Evaluation — 2026-05-13

**Status:** R-25-eval council-APPROVED with 6 amendments (NASA/JPL + Prof + ArduPilot + Cubesat, unanimous 2026-05-13). Decision is final; R-25-exec inherits the amended plan.
**Authored:** 2026-05-13 (Session 3 of Deferred-Cleanup Cycle DC-2026-05-13).
**Inputs:** R-25 row in `docs/PROBLEM_REPORTS.md`; bench-tier inventory in `docs/audits/MASTER_STANDARDS_AUDIT_2026-05-07.md` §L2.6f; R-23 (vehicle bench-tier INVPC HardFault) and R-24 (boot-parity) both block on this decision; IRL aerospace dev-code-sequestration research memo 2026-05-13; NASA SWEHB §8.19 + SWE-133 reviewed inline.

---

## Context

The RocketChip firmware builds in 4 tiers:

| Tier | Build dir | `NOT_CERTIFIED_FOR_FLIGHT` | `ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS` | Role |
|---|---|---|---|---|
| Vehicle flight | `build_flight/` | OFF | OFF (default) | Production, what flies |
| Vehicle bench | `build/` | ON | ON | Test-only, includes `src/dev/*` + 40 `#ifdef` gates |
| Station flight | `build_station_flight/` | OFF | OFF | Station production |
| Station bench | `build_station/` | ON | ON | Test-only, includes `station_fault_inject` |

The bench tiers host: `dev_cli.cpp` (`q→` Debug submenu), `diag_stats.cpp` (rate counters), `fault_inject.cpp` (9 `fault_force_*` functions), `replay_inject.cpp` (CSV-streamed sensor replay), `station_fault_inject.cpp` (4 station faults), `station_replay.cpp` (station replay). Plus 40 scattered `#ifdef ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS` gates in production source files. 12 host scripts depend on bench-tier symbols; only 3 are recurring audit gates (`enhanced_fault_injection.py`, `replay_gate_test.py`, `station_bench_sim.py`).

**Why R-25 was opened.** The 2026-05-07 audit cycle's first attempted Phase 4 against the vehicle bench tier surfaced R-23 — INVPC HardFault on cold boot, undetected for an unknown duration. Audit-cycle build-parity gate only checks compilation, not boot. User observation: *"my reason for asking is I have a feeling the bench build isn't well maintained and if there's no persistant purpose and instead it's just an amalgamation of single use testing features then it should be depreciated in favor of temporary code changes using those modules instead."*

**Subsequent project evidence the dual-binary approach has caused real bugs:**

- **R-23** (2026-05-07): vehicle bench tier silently broken; flight-tier audit gate couldn't see it.
- **F-2026-05-13-004** (2026-05-13 audit): 3 production .cpp files drifted out of the `-Wpedantic` gate; visible only when running pedantic over the full source set; missed because the gate runs against the flight tier whose `ROCKETCHIP_SOURCES` excluded them.
- **R-22** (this cycle Session 2, deferred): council-required Core-1 positive-control check via `q→s` / `q→b` Debug submenu doesn't exist on flight-tier firmware. The dual-binary structure made the test infrastructure (R-22) target one binary while the thing actually flying is the other.
- **LL Entry 36** (bench_sim regex rot, 2026-04-11): "tools treated as infrastructure but structured as test artifacts rot silently." The bench tier is the same pattern at the build-config level — a parallel binary treated as if it tracks the flight binary, but with no mechanism enforcing tracking.

**User direction 2026-05-13 (this session):** *"I don't think a parody dev build is the right approach we've already seen the issues it causes."* The user-preferred direction is to retire the parallel bench tier in favor of something that doesn't replicate the dual-binary failure mode. Specifically NOT a "tightened minimal bench tier" (Approach C / B-5c hybrid) — that retains the structural problem at smaller scope.

## IRL aerospace dev-code-sequestration practice

Per 2026-05-13 research memo (`AGENT` agent run, agentId `afd85b8b90b3e2447`; full memo retained in plan-mode discussion record). Headline findings:

**NASA SWEHB §8.19** (Dead / Dormant Code and Safety Critical Software):
> *"Avoid using test or temporary code inside the Code Base. This includes all code that is not directly needed for operation in a live environment. If using test code within the code base is required for testing purposes, it must be removed before the code base is released."*

This is the **strongest position** in any surveyed standard. It does not bless `#ifdef`-out test scaffolding; the rule is read as "don't ship test code in the deliverable executable" and (per the JPL Institutional C Standard's nearly-identical hostility to conditional compilation) it disfavors scattered `#ifdef` gates because of maintenance-hazard and mistaken-activation risks.

**NASA SWEHB SWE-133** (Software Safety Determination):
> *"non safety-critical software residing with safety-critical software is a concern because it may fail in a way that it disables or impairs the functioning of the safety-critical software. ... When methods to separate the code, such as partitioning, can't be used to limit the software defined as safety critical, care must be exercised to assure safety."*

Partitioning is the preferred mitigation; "care must be exercised" is the fallback when partitioning isn't feasible. This points toward **runtime partitioning** (one binary, mode-gated entry points) when source-level separation is infeasible — which is the position the current bench tier is in.

**ArduPilot.** Separate binaries per target (`AP_HAL_ChibiOS` for flight, `AP_HAL_SITL` for simulation, `AP_HAL_ESP32`, etc.). SITL is NOT compiled into the production firmware. Operational mode-switching within the flight binary uses runtime parameters (`SIM_*`, `SCR_ENABLE`, etc.).

**PX4.** SITL is a separate binary. **HITL** (Hardware-In-The-Loop) is the *same flight firmware*, entered via runtime parameter. **System failure injection lives in the flight binary, gated by `SYS_FAILURE_EN` parameter** — *"Failure injection is disabled by default, and can be enabled using the SYS_FAILURE_EN parameter."* Operators inject failures via the `failure` console command or MAVSDK plugin. This is the most direct IRL match for RocketChip's needs.

**F-Prime (JPL flight-software framework).** Two distinct binaries from the same source tree via two CMake build types (`Release` vs `Testing`). Test variant includes auto-generated `TesterBase.cpp`/`Tester.cpp` mirrors of production components. Used by Ingenuity and various NASA cubesats. **This is the "host-only test harness" pattern** — appropriate for host-side unit tests, NOT for runtime test affordances on a flying MCU.

**Curiosity / Perseverance.** Production binary carries >200,000 runtime parameters. Diagnostic modes entered via parametrized commands authenticated by mission-operations authority. Single binary; runtime mode-switching.

## The decision

**Recommendation: Approach A — single-binary runtime test-mode, probe-only gated** (the PX4 `SYS_FAILURE_EN` pattern).

Rationale, mapped to lived RocketChip evidence:

1. **The dual-binary problem has caused real bugs on this project.** R-23 + F-2026-05-13-004 + R-22's design wall are three independent surfacings in the last 7 days. Continuing to maintain a parallel bench tier — even a "tightened minimal" one — repeats the same pattern at smaller scope. The user's framing is correct: this is what to avoid.

2. **NASA SWEHB §8.19 is on-point.** The current 6 dev modules + 40 scattered `#ifdef` gates fall directly under the "avoid test or temporary code inside the Code Base" guidance. The handbook's "must be removed before release" framing is too strong for a hobbyist-tier project (we genuinely need the testing affordances during development), but the framing-of-intent is clear: don't carry test code in the source tree as if it's part of production.

3. **PX4's pattern is the right IRL model.** Failure injection lives in the flight binary, gated by a parameter that defaults to disabled. The probe-only physical-presence requirement (CMSIS-DAP / SWD) maps cleanly onto RocketChip's existing debug-probe workflow (LL Entry 25 SUPERSEDED + DEBUG_PROBE_NOTES.md). An adversary cannot reach the probe over LoRa; an in-flight stray cannot reach the probe without physical bench presence.

4. **Code Classification + Same-binary principle already exists.** `CODING_STANDARDS.md` § Code Classification already documents the "Same binary principle: there is no compile-time flight flag. Flight and ground code coexist in the same binary. The flight state machine controls runtime access — when the system transitions from IDLE to ARMED, ground-only code paths (CLI, diagnostics, USB I/O) are locked out at runtime, not compiled out." Approach A is the natural extension of this principle to fault-injection and diagnostic affordances that are currently compile-out'd into the bench tier.

### Approach A — concrete shape

1. **Move the test affordances into the flight binary**, behind a `g_test_mode_enabled` flag with a **three-condition AND gate** (council amendment #1, per JPL/AP — defense-in-depth, single-bit-flip not arming):
   - Debug probe writes `kTestModeMagic = 0xC0DEFA02U` to a known `.uninitialized_data` SRAM address, AND
   - Current flight phase == kIdle (state-machine condition), AND
   - Boot-time window: `millis() < kTestModeArmWindow` (e.g., 30 s after boot — closes the arming opportunity after init).

   **Probe-only is the sole gate. No CLI fallback** (council amendment #1, per CSE lived experience — CLI fallbacks get normalized into routine ops use, defeating the physical-presence guarantee). Matches the project's existing probe-first discipline (LL Entry 25 SUPERSEDED + DEBUG_PROBE_NOTES.md). If no probe is attached, fault-injection scenarios cannot run.

2. **The `g_test_mode_enabled` flag is itself flight-critical state** (council R-25-exec checklist item C). It must:
   - Be checked at every test-affordance entry point (`fault_force_*` becomes `if (!g_test_mode_enabled) return;`).
   - **Clear on ANY state transition out of IDLE** (council amendment #2, per Prof — Therac-25-class precedent for why a single clearing gate is insufficient).
   - **AND refuse to enter ARM if set** (council amendment #2, second gate — defense-in-depth). The flight state machine already locks out ground-only paths on ARM; this slots in.
   - Be visible in the boot banner and the `p` preflight VERDICT (test mode active → forced NO-GO with reason "test mode active").
   - Be inactive across power-on reset (no persistence — must be re-armed by the operator each session via the three-condition gate above).
   - Be added to `CODING_STANDARDS.md` Code Classification table as **Flight-Critical** classification.
   - A host-side ctest sweeps every `fault_force_*` and `replay_*` symbol and verifies the gate is present at entry (council R-25-exec checklist item C).

3. **`#ifdef ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS` gates ALL go away.** The dev modules become regular `src/` modules. The functions are runtime-callable in the flight binary but gated. 40 `#ifdef` blocks across production source become zero (replaced by `if (!g_test_mode_enabled)` early returns inside the function bodies).

4. **`src/dev/` directory contents migrate** (council amendment #4 — replay goes host-side):
   - `dev_cli.cpp` (q-Debug submenu) → `src/cli/rc_os_debug.cpp` or similar. q-submenu becomes available in the flight binary; **diagnostic reads (`q→s` sensor counts, `q→b` Hardware Status, `q→i` I2C scan, `q→e` ESKF live, `q→d` diag_stats, `q→y` pyro log) work always** (observational, no safety risk); **state-mutating commands (the 0-5 radio-cfg keys) require `g_test_mode_enabled`**.
   - `fault_inject.cpp` → `src/safety/fault_inject.cpp`. All 9 `fault_force_*` functions exist in the flight binary; all gated on `g_test_mode_enabled`.
   - `diag_stats.cpp` → `src/diag/diag_stats.cpp`. Rate counters always emit on `q→s` and `q→d`; no gate needed (observational).
   - **`replay_inject.cpp` DELETES** (council amendment #4, per Cubesat + AP analysis). Today's replay is a CSV-streamer-into-seqlock-buffers — it doesn't simulate sensor timing, it overrides the buffer. Algorithmic coverage is fully provided by **host-side replay**: same ESKF code runs on the workstation, fed by the same CSV. Better visibility, no on-target attack surface, no flash cost. R-25-exec implements `scripts/replay_harness_host.py` (or similar) and retires the on-MCU path. IVP-131's verification model shifts from "stream CSV into MCU buffer" to "run ESKF on host against CSV, compare against ground-truth reference."
   - `station_fault_inject.cpp` → `src/station/station_fault_inject.cpp` (or migrate to `src/safety/`). Same gating as `fault_inject.cpp`.
   - **`station_replay.cpp` DELETES** (council amendment #4, same rationale as `replay_inject.cpp`). Stage-archived already; was never recurring audit infrastructure.

5. **`bench_sim.py` + `station_bench_sim.py` + `enhanced_fault_injection.py` + `replay_gate_test.py`** test scripts update:
   - All `picotool load -f build/rocketchip-dev.uf2` lines become `picotool load -f build_flight/rocketchip.uf2` (single binary).
   - All scripts start with a "enter test mode" preamble: write `kTestModeMagic` to the SRAM location via GDB before running their actual test sequence.
   - The post-test wrap-up writes 0 back to clear the flag (defense-in-depth; the flag is also cleared on reset).

6. **Host-only test artifacts** (codegen test harnesses, `test/test_*.cpp` host unit tests, etc.) stay where they are. Per the IRL research memo's F-Prime pattern, these are correctly host-only and never belong in the MCU binary. Single binary for flight; host-only `ctest` tests for unit coverage. Two distinct surfaces, no overlap.

7. **The 4-tier build matrix collapses to 2 tiers**: vehicle flight + station flight. `verify_build_parity.sh` simplifies. R-24's boot-parity extension becomes 2-tier. R-23 evaporates (the broken bench tier ceases to exist).

### Approach A — risk callouts (per IRL research)

- **Runtime-gate bug ships in flight.** Therac-25 is the canonical example (mode-tracking race). Mitigations in this project: (a) probe-only-write semantics make the gate physically un-armable in flight, (b) flight state machine locks out test-mode entry once ARMED, (c) GO/NO-GO surfaces test-mode-active as forced NO-GO, (d) probe-attach is a physical-presence signal an adversary can't replicate over LoRa.

- **Source bloat in flight binary.** `src/dev/` is currently ~6 modules. Adding them to flight builds increases flash usage. Measurable; not a structural problem. Bench-tier flight builds already pay this cost when flashed; the difference is only that the flight-tier build pays it too. R-25-exec will measure and report.

- **Test-affordance API surface in flight.** More entry points means more attack surface to audit. Mitigated by: (a) all gated on `g_test_mode_enabled` at function-entry, (b) audit-suite walks the gate-check coverage, (c) clang-tidy / Power-of-10 enforcement applies uniformly (currently dev modules are partially excluded from gates — Approach A actually *improves* this).

### Why this isn't Approach B (patch-based)

The user's original framing of Approach B was "temporary code changes using those modules instead." Patch-rot is the failure mode that kills this pattern in real flight-software projects (research memo §6). Without DO-330-class tooling to prove patch-equivalence, patches drift from production code, revert isn't clean, and the test result depends on whether the patch applied correctly. LL Entry 36 documented exactly this failure mode for a different artifact (bench_sim regex rot). Adopting patch-based testing would transfer the rot from build-config to patch files.

### Why this isn't Approach C / B-5c (tightened minimal bench tier)

Retains the structural problem (dual binary, separate gate coverage) at smaller scope. The 3 evidence-of-real-bugs on this project this cycle (R-23, F-004, R-22) all surfaced via the same dual-binary failure mode regardless of bench-tier size. A minimal bench tier still has the property that "what ships isn't what the dev binary exercised."

## R-25-exec follow-on session

R-25-exec inherits the following work (council-amended order per amendment #5 — migrate modules first, then collapse tiers; per amendment #3 — one commit per module, not omnibus):

1. **Gate-mechanism implementation (council amendment #1).** Three-condition AND gate: SRAM-magic AND state==IDLE AND boot-time-window. Probe-only — no CLI fallback. Single commit landing the gate scaffold + ctest sweeping that `g_test_mode_enabled` is checked at every `fault_force_*` / replay entry.

2. **Per-module migration commits (council amendment #3 — incremental, bisectable):**
   - Commit: `dev_cli.cpp` → `src/cli/rc_os_debug.cpp`. q-submenu in flight binary; diagnostic reads always-on; state-mutating commands gated.
   - Commit: `fault_inject.cpp` → `src/safety/fault_inject.cpp`. 9 `fault_force_*` gated.
   - Commit: `diag_stats.cpp` → `src/diag/diag_stats.cpp`. Always-on observational.
   - Commit: `replay_inject.cpp` DELETES + new host-side `scripts/replay_harness_host.py` (council amendment #4). IVP-131 verification-model shift.
   - Commit: `station_fault_inject.cpp` → `src/station/station_fault_inject.cpp` or `src/safety/`. 4 station faults gated.
   - Commit: `station_replay.cpp` DELETES (council amendment #4).

3. **`#ifdef ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS` elimination.** 40 sites across production source; each per-module commit above strips its own `#ifdef` sites. Final commit (post-module-migration) confirms zero remaining `#ifdef`s.

4. **Update host scripts** — separate commit: `bench_sim.py`, `station_bench_sim.py`, `enhanced_fault_injection.py`, `replay_gate_test.py` to (a) target the flight binary only, (b) enter test mode at session start via probe-write before running their actual sequence.

5. **R-23 evaporates** (bench tier ceases to exist).

6. **Tier-collapse commit (per amendment #5 — last, not first):** `verify_build_parity.sh` 4-tier → 2-tier (vehicle flight + station flight). CMakeLists.txt removes `NOT_CERTIFIED_FOR_FLIGHT=ON` build configurations. Delete `build/` and `build_station/` build directories (the dev-tier outputs). Pre-commit hook adjustments to reflect 2-tier reality.

7. **R-24 simplifies** to 2-tier boot-parity (vehicle flight + station flight). Build the boot-parity verifier once after step 6.

8. **R-22 redesign re-executes** against the single flight binary. Council-required Core-1 counter-increment check via `q→s`/`q→b` now lives in the flight binary, gated by test mode. R-22's positive-control path becomes reliable. The 4-iteration redesign + council verdict preserved at `C:\Users\pow-w\.claude\plans\snoopy-wibbling-noodle.md` is inherited.

9. **Stage-archived cleanup (council amendment #6).** `src/dev/station_replay.cpp` already deletes in step 2 (council amendment #4). 9 archived host scripts (`scripts/ack_stress_test.py`, etc.) delete from `scripts/`. Historical record — *why these existed, why they're gone* — preserves as a CHANGELOG entry + a closure note in PROBLEM_REPORTS.md. SESSION_CHECKLIST §Historical-record applies to docs, not source; source dead code goes per SWEHB §8.19 + AK_GUIDELINES §3 Surgical Changes.

10. **`CODING_STANDARDS.md` Code Classification table updates.** Add `g_test_mode_enabled` and the test-mode-gated affordances as new entries. Document the three-condition gate. Add the host-ctest invariant check as an audit-coverage item.

11. **Audit-gate coverage refresh (council R-25-exec checklist item A).** `scripts/hooks/pre-commit` allowlist, `ROCKETCHIP_SOURCES`, applicable clang-tidy gates updated to include the migrated modules. Audit-coverage IMPROVES under Approach A (the dev modules were partially excluded from pedantic gate; now included).

12. **Flash-budget measurement (council R-25-exec checklist item D).** Report the delta of dev-module inclusion on flight binary flash usage. CSE flagged this — not a structural concern but worth quantifying for the record.

13. **CHANGELOG entry is a pointer, not a duplicate** (council R-25-exec checklist item B + project memory `feedback_changelog_timing_and_brevity.md`). R-25-exec's CHANGELOG entry points at the decision doc + commit SHAs; doesn't duplicate the rationale.

## Council verdict (2026-05-13)

**CONSENSUS APPROVE Approach A with 6 amendments + 4 R-25-exec checklist items.** Council: NASA/JPL Avionics Lead, Embedded Systems Professor, ArduPilot Core Contributor, Cubesat Startup Engineer (replacing Senior Aerospace Student for this review — build-architecture / deprecation strategy is NewSpace-shipping framing, not capstone framing).

### Answers to the 7 open questions

**Q1 (Approach A correct for hobbyist scope?):** YES. The dual-binary failure mode produced 3 real bugs in 7 days on a project with one developer. The IRL research's caveat that "SWEHB §8.19 is too strong for hobbyist" is the right read — we keep test affordances, but we host them in the flight binary like PX4 does. No scope mismatch. Unanimous.

**Q2 (probe-only gate mechanism):** **Probe-write to SRAM, three-condition AND gate, NO CLI fallback** (amendment #1). SRAM-magic + state==IDLE + boot-time-window <N seconds. Three conditions ANDed, defense-in-depth. CLI fallback gets normalized into routine ops use (CSE lived experience at cubesat startup); strikes from the design.

**Q3 (clearing semantics):** **Two gates, not one** (amendment #2). Clear on ANY state transition out of IDLE, AND refuse to enter ARM if set. Therac-25-class precedent for why single-gate is insufficient.

**Q4 (`#ifdef` elimination order):** **Incremental, per-module, not omnibus** (amendment #3). 6 per-module commits in R-25-exec, each independently bisectable + verifiable per HW_GATE Rule 3.

**Q5 (`replay_inject.cpp`):** **Migrates host-side; on-MCU path DELETES** (amendment #4). Today's `replay_inject.cpp` is a CSV-streamer-into-seqlock-buffers — doesn't simulate sensor timing, just overrides buffers. Algorithmic coverage fully provided by host-side replay (run ESKF on workstation against same CSV). Better visibility, no on-target attack surface. Same for `station_replay.cpp` (already Stage-archived). IVP-131 verification-model shifts.

**Q6 (build-system simplification sequence):** **Migrate modules first, then collapse tiers** (amendment #5, per Prof — standard refactoring discipline). Bench tier remains buildable during the 6-module migration; collapse to 2-tier matrix is the final commit. Preserves the fallback during migration.

**Q7 (stage-archived cleanup):** **Source deletes, history preserves** (amendment #6). `station_replay.cpp` + 9 archived host scripts delete from `src/` and `scripts/`. Historical decision (why they existed, why they're gone) preserves as CHANGELOG entry + closure note in PROBLEM_REPORTS.md. SESSION_CHECKLIST §Historical-record applies to *docs*, not source; source dead code goes per SWEHB §8.19 + AK_GUIDELINES §3 Surgical Changes. The two project doctrines are not in conflict — they cover different artifact types.

### Independent items raised by council (R-25-exec checklist, not decision-doc amendments)

- **A. Audit-gate coverage refresh.** `scripts/hooks/pre-commit` allowlist, `ROCKETCHIP_SOURCES`, applicable clang-tidy gates updated to include migrated modules. Approach A IMPROVES audit-coverage (dev modules were partially excluded from pedantic gate; now included).
- **B. CHANGELOG entry is a pointer, not a duplicate.** Per project memory `feedback_changelog_timing_and_brevity.md`.
- **C. `g_test_mode_enabled` is flight-critical state.** R-25-exec adds it to `CODING_STANDARDS.md` Code Classification table as Flight-Critical. Host ctest sweeps every `fault_force_*` + replay symbol; verifies the gate is present at entry. SAD invariant section added.
- **D. Flash-budget measurement.** R-25-exec reports the flash delta of dev modules in the flight binary. CSE flagged this — not a structural concern but worth quantifying.

### Persona sign-offs

- **NASA/JPL Avionics Lead:** APPROVE. The Same-binary principle was already a project standard; Approach A extends it consistently. The SRAM-magic + double-gate construction handles the FMEA concern (SEU / stale state / debugger-left-behind).
- **Embedded Systems Professor:** APPROVE. Conditional-compilation hostility (JPL Institutional C Rule 25 + Liebig et al. 2010) supports Approach A theoretically; LL Entry 36 supports it empirically. Migrate-modules-first sequencing is standard refactoring discipline.
- **ArduPilot Core Contributor:** APPROVE. ArduPilot's `SIM_*`-parameter pattern + PX4's `SYS_FAILURE_EN` are the two strongest IRL precedents and they converge on this exact design. R-23 + F-004 + R-22 are the same dual-binary failure mode ArduPilot avoided by never having a parallel "ArduCopter-dev.elf vs ArduCopter.elf."
- **Cubesat Startup Engineer:** APPROVE. Single-binary discipline is non-negotiable at lean NewSpace because re-flashing on orbit is not an option. The dual-binary anti-pattern burned a quarter at my startup. Probe-only (no CLI fallback) is correct — CLI fallbacks get normalized into routine ops use. Flash-budget measurement is worth quantifying for the record but is unlikely structural.

---

## Out of scope (R-25-eval boundaries)

Per the cleanup plan's Session 3 scope: this evaluation produces a decision doc + a follow-on R-25-exec PR. **R-25-eval does NOT execute the migration.** R-25-exec is a separate session that inherits this plan.

Council sign-off here means: "the strategic direction Approach A (single-binary runtime test-mode, probe-only gated) is the right path forward; R-25-exec inherits this plan and executes the migrations + R-23 evaporation + R-22 + R-24 within this framework."

## References

- NASA SWEHB (the handbook as a whole is the reference per `standards/CODING_STANDARDS.md`; specific sections cited where applied):
  - [§8.19 — Dead/Dormant Code and Safety Critical Software](https://swehb.nasa.gov/spaces/SWEHBVC/pages/98369575/8.19+-+Dead+Dormant+Code+and+Safety+Critical+Software)
  - [SWE-133 — Software Safety Determination](https://swehb.nasa.gov/spaces/7150/pages/16449848/SWE-133+-+Software+Safety+Determination)
- IRL research memo 2026-05-13 (Plan-mode agent run `afd85b8b90b3e2447`; full text retained in plan-mode discussion record): aerospace dev-code-sequestration practice + DO-178C / F-Prime / ArduPilot / PX4 prior art.
- Project-internal evidence the dual-binary problem has caused real bugs: R-23, F-2026-05-13-004, R-22 design wall, LL Entry 36.
- `standards/CODING_STANDARDS.md` § Code Classification + Same-binary principle (already-existing precedent for Approach A's framing).
