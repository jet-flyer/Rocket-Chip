# Master Standards Audit 2026-05-07 — Preliminary Remediation Queue

**Status:** Preliminary. Intended to be folded into the dated audit report's `## Remediation` section once the full Master Standards Audit (per `.claude/plans/streamed-launching-lagoon.md`) is run.

**Source:** Phase A.2 (accepted-deviations review) findings from the renaming + re-evaluation of `standards/ACCEPTED_STANDARDS_DEVIATIONS.md`. Of the 13 rows in the file at the start of Phase A.2:

- 1 row remains accepted (CG-1, auto-generated codegen — only canonical case)
- 2 rows moved to Resolved (AP-1, AP-3 — code refactored away)
- 3 rows deleted as non-deviations or misreadings (AP-2, AP-4, and the BM-1/BM-3 framing of P10-2 — see "Holzmann inverted-rule exemption" below)
- 1 row kept active under corrected citation (FP-1 — was wrongly cited as JSF Rule 170, actually a P10 Rule 9 deviation; see R-6/R-6b history)
- 6 items moved here as remediation queue (BM-2, BM-4, BM-5, FP-1 template-dispatch remediation, IO-1, IO-2)

---

## R-1 — BM-2: Core 1 boot-wait loop bounded on vehicle path

**Standard:** P10-2 / JSF AV Rule 25 (loops must have fixed upper bound)
**Location:** `src/core1/sensor_core1.cpp:490` (`core1_entry()` `while (!g_startSensorPhase) { sleep_ms(10); }`)
**Status:** Compliant on station/relay path via Holzmann's inverted-rule exemption (intentionally non-terminating — Core 1 idles forever). On vehicle, the loop terminates when Core 0 signals sensor phase start; bounded in practice but not statically provable.
**Recommended remediation:** Add a max-iteration count (e.g., 1000 iterations × 10 ms = 10 s max boot wait — far above the realistic Core 0 boot sequence ≤1 s). If exceeded, halt with a fault that flows through the same FAULT-health-state mechanism used elsewhere (see R-3/R-4 for the broader fault-handler remediation that this should integrate with). Static analyzer can then prove the bound.
**Estimated effort:** Small — ~10 lines, single function.
**Dependency:** Best done after R-3/R-4 lands (so the bounded-loop fault path can reuse the new capture-state-then-reset machinery instead of re-inventing a halt pattern).
**SPIN coverage note (user-flagged 2026-05-07):** The cross-core handshake (Core 0 setting `g_startSensorPhase` before Core 1's bounded wait expires) is exactly the kind of liveness property SPIN was designed for. Our current SPIN models cover the flight-director state machine but not the boot sequence. Consider extending SPIN coverage in a follow-up: model Core 0 + Core 1 as two processes with `g_startSensorPhase` as a shared atomic, verify the liveness property "Core 1 eventually proceeds OR Core 0 never reaches `start_sensor_phase()`." Out of scope for the R-1 code fix itself, but a clean addition to the Phase 6 (Formal Verification + Simulation Coverage) step of the master audit when it runs.

---

## R-2 — IO-2: GPS PMTK snprintf elimination

**Standard:** JSF AV Rule 22 (`<stdio.h>` prohibited)
**Locations:**
- `src/drivers/gps_pa1010d.cpp` — 6 `snprintf` calls for PMTK command formatting
- `src/drivers/gps_uart.cpp` — 4 `snprintf` calls for PMTK command formatting

**Status:** Real deviation; current code uses `snprintf` with `sizeof()` bounds (MISRA-C 2012 accepted safe subset). The original deviation row cited UBX binary protocol as the migration path, but that requires hardware change (u-blox GPS).
**Recommended remediation:** Replace each `snprintf` callsite with a precomputed `static const char[]` PMTK command array. PMTK commands are fixed strings (e.g., `"$PMTK220,1000*1F\r\n"`) — runtime formatting is unnecessary. ~10 callsites total. Eliminates JSF Rule 22 violation entirely for these files.
**Estimated effort:** Small — single focused commit, ~50 lines net (commands as const arrays + replace `snprintf(buf, sizeof(buf), "$PMTK...")` with `i2c_write(buf, sizeof(buf))` or equivalent).
**Verification:** Existing GPS bench tests cover PMTK command success — those tests prove the bytes-on-wire match after refactor.

---

## R-3 — BM-4: `memmanage_fault_handler()` halt-forever replacement

**Standard:** P10-2 / JSF AV Rule 25 (and broader aerospace safety practice)
**Location:** `src/safety/fault_protection.cpp:28` (`memmanage_fault_handler()`)
**Status:** Current implementation halts forever with interrupts disabled and an LED blink pattern. **This is NOT industry practice for safety-critical aerospace firmware.** ArduPilot, PX4, NASA cFS all use the capture-state-then-reset pattern: a hardfault captures CFSR/HFSR/faulting PC to preserved memory, then triggers `NVIC_SystemReset()`. On the next boot, banner code detects the previous crash and transitions to FAULT health state with full context.
**Recommended remediation:** Two-stage refactor.
1. **In the handler:** Capture minimal state (CFSR at `0xE000ED28`, HFSR at `0xE000ED2C`, stacked PC and LR) to a known-good SRAM region marked `__attribute__((section(".uninitialized_ram")))` (not zeroed at boot). Trigger `NVIC_SystemReset()` (`SCB->AIRCR = (0x05FAUL << 16) | (1UL << 2)`).
2. **In the banner code:** On boot, check the preserved region for a valid "crash record" magic number. If present, log the captured fault info and transition the flight director into FAULT health state via the existing machinery (`src/safety/health_monitor.cpp`). Clear the magic so the next clean boot doesn't re-report.

This uses the existing safe-mode / FAULT-health pivot — no new fault state needed; just wires the capture-then-reset path into the path that already exists.

**Safe-mode integration evaluation (user-flagged 2026-05-07):** Before final design, evaluate whether the existing safe-mode / FAULT-health-state pivot can directly absorb the post-reset detection, or whether new infrastructure is needed. The pivot already replaced the deprecated watchdog-reset path with the project's fault-handler logic — the question is whether a "previous-boot-hardfault" event maps cleanly onto an existing health-state transition or needs a new event/state. Two possible outcomes: (a) clean fit — wire the preserved-SRAM read straight into the existing FAULT-health code path, minimal new infrastructure; (b) new fault category needed — add it. Evaluation belongs in the R-3 implementation planning, not in the remediation queue itself. Either way, the architectural goal is "capture state → reset → on next boot, the existing FAULT machinery owns the recovery."
**Verification:** Use `src/dev/fault_inject.cpp` (already exists for forced MPU faults) to trigger the handler in a bench build, confirm capture record appears in preserved SRAM via GDB, reset proceeds cleanly, post-reset banner reports previous fault.
**References:**
- ArduPilot watchdog/crash dump: https://ardupilot.org/copter/docs/common-watchdog.html
- Memfault firmware watchdog best practices: https://interrupt.memfault.com/blog/firmware-watchdog-best-practices

---

## R-4 — BM-5: `mpu_setup_stack_guard()` halt-on-failure replacement

**Standard:** Same as R-3
**Location:** `src/safety/fault_protection.cpp:79` (`mpu_setup_stack_guard()` failure path)
**Status:** Same halt-forever pattern as BM-4. Failure is at boot (MPU config rejected), which is a different timeline than runtime hardfault but the same architectural decision — halt vs. reset-and-flag.
**Recommended remediation:** Same pattern as R-3. Capture failure info ("MPU rejected configuration at line X") to preserved SRAM, reset, post-reset banner reports it. The vehicle then runs without MPU stack guard but with a logged warning — alternatively, the boot code can decide "MPU is required; reboot loop with increasing delay if it keeps failing." That latter is policy choice.
**Estimated effort:** Small if R-3 lands first — reuses the same preserved-SRAM machinery, just a different crash type code.
**Dependency:** Implement after R-3 so the preserved-SRAM record-format and post-reset detection are already in place.

---

## R-5 — IO-1: stdio.h elimination from flight binary

**Standard:** JSF AV Rule 22 (`<stdio.h>` prohibited)
**Status: MARKED INTERIM.** The current "Ground classification + runtime lockout" rationale is partially superseded by the `NOT_CERTIFIED_FOR_FLIGHT` compile-time exclusion pivot. Many of the existing 62 printfs likely belong in `src/dev/` (compile-excluded from flight binary) rather than `src/cli/` (in flight binary but runtime-locked). Work needed to confirm.

**Pre-remediation grep needed (sub-task R-5a):**
```
grep -rn 'printf\|getchar' src/cli/ src/drivers/ src/main.cpp \
  | awk -F: '{print $1}' | sort | uniq -c
```
Plus a categorization pass: for each printf site, classify as (i) dev-only diagnostic — move to `src/dev/`, (ii) CLI command output that's needed in flight builds (rare, e.g., command echo for ground operator) — convert to custom bounded writer, (iii) driver diagnostic — likely move to `src/dev/`.

**Recommended remediation path** (multi-step, plan-and-stage):
1. **R-5a: Inventory** — grep + classify (single commit, documentation only).
2. **R-5b: Move dev/diagnostic printfs to `src/dev/`** — compile-excluded from flight builds. Likely eliminates 80%+ of sites.
3. **R-5c: Replace remaining flight-binary printfs with custom bounded writer** — implement a small no-variadic helper (`rc_write(const char* s, size_t n)`) that goes direct to USB CDC without going through `<stdio.h>`.
4. **R-5d: Final state** — zero `<stdio.h>` includes in the `NOT_CERTIFIED_FOR_FLIGHT=OFF` flight binary; deviation deleted from this remediation queue and never appears in the accepted-deviations file.

**Estimated effort:** Multi-commit refactor. R-5a is hours; R-5b is small per file but many files; R-5c is moderate. Total ~1 week of focused work.
**Tracking note while interim:** Until R-5 lands, every `<stdio.h>` callsite in the flight binary is technically out of standards compliance. The current code works and the compile-time gate makes most cases safe, but the work is not done. Do not let this entry sit in the "accepted deviations" file — it belongs here in the remediation queue precisely because the intent is to fix it, not accept it.

---

## R-6 / R-6b — FP-1 misreading cleanup + reattribution to correct standard (DONE)

**Status:** COMPLETED 2026-05-07. Documented here for traceability.

**Two-layer issue:**

**R-6 (done, commit `d186fc9`):** The deviations file originally cited "JSF AV Rule 170 prohibits function pointers." That's a misreading. JSF Rule 170 actual wording: *"More than 2 levels of pointer indirection shall not be used"* — about indirection depth, not function pointers. Removed the wrong citation across 6 files.

**R-6b (done, this commit):** R-6's conclusion ("no project-adopted standard prohibits function pointers") was wrong. **Power of 10 Rule 9** (Holzmann/JPL, 2006) explicitly states: *"Limit pointer use to a single dereference, and do not use function pointers."* P10 is project-adopted per `standards/CODING_STANDARDS.md` and is newer than JSF Rule 176 (which only requires typedef discipline). Per the project's standards-precedence rule (newer overrides older absent explicit override), **P10 Rule 9 governs** — function pointers ARE an accepted deviation in our project, just from P10 not JSF.

**Actions applied:**

- FP-1 re-added to `ACCEPTED_STANDARDS_DEVIATIONS.md` under P10 Rule 9 with proper rationale (Ground classification, runs once per calibration pre-flight, eliminates 120 lines of duplication, JSF Rule 176 typedef discipline satisfied).
- Source comments + header + `PROJECT_STATUS.md` + `NOTIFY_CONTRACT.md` supersession note re-cited to P10 Rule 9 (the actually governing standard).
- LL Entry 37 amended to document the two-layer lesson: verify rule wording AND verify the correct standard in the precedence chain.
- `standards/CODING_STANDARDS.md` Foundation section rewritten with the explicit standards-precedence rule and chronological ordering. The R-6 → R-6b near-miss demonstrated that the precedence rule needed to be stated openly, not just understood by veterans.

**Remaining FP-1-related work** (template-dispatch remediation to eliminate the deviation entirely): See **R-6c** below.

---

## R-6c — FP-1 template-dispatch remediation (eliminate the deviation)

**Standard:** P10 Rule 9 (no function pointers in safety-critical code)
**Status:** Accepted deviation while pending remediation. The deviation is bounded (Ground-classified, pre-flight only) but the long-term goal is elimination, per the project's "prefer remediation over acceptance" policy.

**Recommended remediation:** Convert `ResidualFn` / `JacobianFn` from runtime function pointers to **compile-time template parameters**:

```cpp
template <typename Residual, typename Jacobian>
void lm_solve(float* params, ..., Residual residual_fn, Jacobian jacobian_fn) {
    // Same body — calls resolve at compile time
}
```

Templates produce identical machine code to the current function-pointer indirection (compile-time-resolved), eliminate the runtime function pointer entirely (P10 Rule 9 satisfied), and preserve the ~120 lines of dedup. Cost: moderate refactor — the `lm_solve()` body and its helpers need to move into a header (or `.inc` file) so template instantiation can resolve, plus careful testing that the math is bit-identical before and after.

**Estimated effort:** Moderate — single file refactor + comprehensive test against existing calibration outputs to confirm numerical equivalence.

**Verification:** Existing 6-pos and ellipsoid calibration tests cover the math. Diff calibration outputs before/after for identical input samples; require bit-identical results.

**Priority:** Medium — the current accepted-deviation state is safe (Ground-only, pre-flight only), so this is improvement not urgency. Good candidate for a focused remediation session after the master audit completes.

---

## R-9 — Fault-injection harness completion (Phase 4 audit findings)

**Status:** New (surfaced 2026-05-07 during master audit Phase 4 / step 4.3).
**Context:** `scripts/enhanced_fault_injection.py` was a stub before this audit. Rewritten during Phase 4 to drive the device via GDB using `src/dev/fault_inject.cpp` hooks; 1 of 4 scenarios (pyro-misfire) now passes end-to-end with firmware-side positive-control signal observed. Other 3 scenarios surfaced 3 distinct firmware-harness gaps:

### R-9a — `fault_force_launch_abort()` hook

**Standard:** HW_GATE_DISCIPLINE.md Rule 1 (positive-control signal required for the launch-abort scenario)
**Issue:** GDB-scratch-storage QEvt + GDB-call of `QActive_post_(AO_FlightDirector, &$scratch_evt, 0, 0)` does NOT produce an observable `[FD] ABORT*` log. The event is either discarded (FD not in IDLE) or the GDB-scratch QEvt isn't survived by the post mechanism.
**Recommended remediation:** Add a `fault_force_launch_abort()` to `src/dev/fault_inject.cpp` that, in bench-tier:
  1. Asserts FD is in IDLE (returns early if not, with a `[FAULT]` log explaining why)
  2. Builds a static-storage `QEvt` with sig=SIG_ARM, calls `QActive_post_()` on AO_FlightDirector
  3. Waits ~50ms for the dispatch to land
  4. Builds a static-storage `QEvt` with sig=SIG_ABORT, calls `QActive_post_()`
**Estimated effort:** Small — ~30 lines new function + minor refactor of enhanced_fault_injection.py launch-abort scenario to just call the hook.

### R-9b — `fault_force_radio_dropout()` hook

**Standard:** HW_GATE_DISCIPLINE.md Rule 1 (positive-control signal required for radio-dropout)
**Issue:** The radio link-state struct `s` in `ao_rf_manager.cpp` is file-scope-static; GDB's `set variable s.last_rx_ms = 0` silently no-ops because `s` isn't a visible global symbol.
**Recommended remediation:** Add a `fault_force_radio_dropout()` to `src/dev/fault_inject.cpp` that, in bench-tier, pokes the link-state via either: (a) an extern accessor in `ao_rf_manager.cpp` (e.g., `void rf_manager_set_last_rx_ms_for_test(uint32_t ms)`), or (b) directly setting an extern global if `s.last_rx_ms` is hoisted to file-extern scope under `#ifdef ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS`. Option (a) is cleaner.
**Estimated effort:** Small — ~20 lines (extern accessor + fault hook).

### R-9c — log-on-change emit in `health_monitor.cpp`

**Standard:** HW_GATE_DISCIPLINE.md Rule 1 (every state transition has a positive-control signal)
**Issue:** When `kHealthCore1Ok` (or any other health bit) flips from OK to FAULT in the health byte, no log line is emitted. The bit transition is silent. Observed during Phase 4.3 — `g_core1StallTicks` set to 100 successfully flipped the bit, but no `[Health] CORE1 FAULT` line appeared in serial output.
**Recommended remediation:** In `src/safety/health_monitor.cpp`, add log-on-change tracking for each subsystem health bit. When a bit transitions OK→FAULT, emit `[Health] <subsystem> FAULT` to serial. When FAULT→OK (recovery), emit `[Health] <subsystem> RECOVERED`. The current behavior — silent health-byte mutation — fails the FMEA-lite Row K3 ("every state transition has explicit guard + observable positive-control signal") for the health-monitor subsystem-state machine. This benefits real flight operations too: currently the GCS would see the health byte change but the firmware itself doesn't log when it caught a fault.
**Estimated effort:** Moderate — needs change-detection state per subsystem (prev_primary, prev_secondary, prev_critical, prev_mcu fields), printf statements added per transition. ~40-60 lines change.

**R-9 priority:** Medium overall. The audit's Phase 4.1/4.2 already verified the firmware safety properties via FMEA-lite + Koopman; R-9 improves the automated regression harness but doesn't unblock audit conclusions. Good candidate for a focused post-audit remediation session.

**R-9 summary effort:** ~2-3 hours total for R-9a+R-9b+R-9c, mostly in `src/dev/fault_inject.cpp` and `src/safety/health_monitor.cpp`.

---

## R-11 — Promela model: flash_safe_execute / Core 1 lockout / I2C reset protocol

**Status:** New (queued 2026-05-07 from Phase 6 council review). Highest-ROI SPIN extension per Cubesat Startup Engineer persona.

**Standard:** Internal — concurrent protocol with documented bug history (LL Entries 28, 31).

**Issue:** When `flash_safe_execute()` runs, three actors interact concurrently: the calling core, Core 1 (must be `multicore_lockout`'d), and the I2C peripheral (must be reset after the flash op per LL Entry 31). The ordering and invariants of this protocol are documented informally in LL Entries 28+31, but are not formally modeled. Bugs in this protocol have produced sensor init failures (LL 28) and GPS satellite-lock prevention (LL 31). The class is not yet exhaustively verified.

**Recommended remediation:** Author `tools/spin/rocketchip_flash_protocol.pml` modeling:
- 3 processes (flash-caller, Core 1, I2C peripheral state machine)
- Shared `flash_busy` flag
- I2C `peripheral_state` enum (active / corrupt / reset)
- 3-5 LTL properties:
  - `p_no_i2c_during_flash`: `[](flash_busy -> !i2c_transaction)`
  - `p_i2c_reset_after_flash`: `[](flash_busy_falling_edge -> <>i2c_was_reset)`
  - `p_core1_lockout_during_flash`: `[](flash_busy -> core1_locked_out)`
  - Possibly: `p_init_order` (flash before I2C init at boot per LL 31)

**Estimated effort:** ~1 day (model authoring + LTL verification + README update + add to milestone gate).

**Verification:** SPIN exhaustive check with errors: 0 on all properties. Bonus: a multi-actor model would have caught both LL 28 and LL 31 before they produced bench bugs — strong evidence the model is worth its cost.

**Priority:** HIGH within the post-audit SPIN-extension stage. Council unanimously identified this as highest-ROI.

---

## R-12 — Promela model: cross-core boot handshake (closes R-1 / BM-2)

**Status:** New (queued 2026-05-07 from Phase 6 council review). Closes existing R-1/BM-2 from initial remediation queue.

**Standard:** P10-2 / JSF AV Rule 25 (loops with fixed upper bound) for the Core 1 wait loop; NASA ARINC-653 SPIN practice (boot sequence IN SCOPE).

**Issue:** Core 0 sets `g_startSensorPhase` after init; Core 1 waits for it in `core1_entry()` at `src/core1/sensor_core1.cpp:490`. The wait loop is informally bounded by "Core 0 will eventually set the flag" but no formal proof exists. R-1/BM-2 in the existing remediation queue is the code fix (add a max-iteration timeout); R-12 is the corresponding SPIN model.

**Recommended remediation:** Author `tools/spin/rocketchip_boot.pml`:
- 2 processes (Core 0, Core 1)
- Shared atomic `g_startSensorPhase` flag
- 2 LTL properties:
  - `p_core1_eventually_proceeds`: `[](g_startSensorPhase -> <>core1_in_sensor_loop)`
  - `p_no_premature_sensor_read`: `[](core1_reading_sensor -> g_startSensorPhase_was_set)`
- Cover both vehicle (Core 0 sets flag) and station/relay (Core 0 never sets flag, Core 1 idles forever per Holzmann scheduler exemption) cases.

**Estimated effort:** ~1 afternoon (small model, 2 processes, 2 properties).

**Priority:** MEDIUM. Closes R-1/BM-2 with formal verification rather than just static bound. Per NASA ARINC-653 paper, boot sequence is explicitly in SPIN scope.

---

## R-13 — Station-model ride-along rule in SESSION_CHECKLIST.md trigger map

**Status:** New (queued 2026-05-07 from Phase 6 council review — Cubesat Startup Engineer persona).

**Issue:** AGENT_WHITEBOARD lists 4 station SPIN model extensions (multi-pending-in-flight, RadioScheduler TX-window arbitration, MAVLink parser state, station_idle_tick GPS poll interleave) that are correctly deferred until corresponding firmware behavior lands. The trap (per council): when firmware DOES land, the SPIN extension has to ride with it in the same commit window, or it never gets written. Same trigger-driven principle as SESSION_CHECKLIST documents for state-of-system protected docs.

**Recommended remediation:** Edit `.claude/SESSION_CHECKLIST.md` Trigger-Driven Doc Edits → Per-doc trigger map → add a row:

> | `tools/spin/rocketchip_station.pml` and `tools/spin/rocketchip_ao.pml` | When firmware behavior matching a candidate SPIN extension lands (per `AGENT_WHITEBOARD.md` "SPIN model extensions" items — e.g., multi-pending-in-flight, RadioScheduler TX-window arbitration, MAVLink parser state, GPS poll interleave). The SPIN model edit rides in the same commit as the firmware change. |

**Estimated effort:** Trivial — 1 row added to SESSION_CHECKLIST trigger map.

**Priority:** MEDIUM — codifies a discipline that would otherwise rely on memory. DO in Phase 8 of this audit.

---

## Phase 8 in-audit-cycle actions from council review (not deferred):

These bundle into Phase 8 alongside the per-finding remediations from Phases 1-5:

- **P8-SPIN-A: Extend `run_stage_o_ao_spin.sh` to gate on all 4 active models.** Currently runs only `rocketchip_ao.pml`. Per council unanimous, gate FD + RF + station too. Half-day shell work.
- **P8-SPIN-B: Delete `rocketchip_fd_pp.pml` (zero-byte stub).** Empty file implies coverage that doesn't exist. Trivial.
- **R-13: SESSION_CHECKLIST trigger-map row.** See above.
- **Pyro FMEA sub-check:** Review pyro intent/arm/disarm transitions for multi-transition failure modes. If any identified, queue as R-14 (unified pyro protocol SPIN model). If none, log as audited-and-no-finding.

---

## R-10 — Stack-usage tooling (Phase 5 audit findings)

### R-10a — `scripts/analyze_stack_usage.sh` parser bug

**Issue:** Script fails with `line 80: static: syntax error: invalid arithmetic operator` when parsing `.su` lines that contain the literal word "static" (gcc emits this as a usage-classifier column). The script's `$size -gt $THRESHOLD_BYTES` arithmetic eval mis-handles the "static" token, bailing early and reporting `Total estimated stack usage: 0 KiB` (which is wrong — should be the sum across all parsed functions).

**Recommended remediation:** Fix the awk/grep extraction in `analyze_stack_usage.sh` to pull only the numeric column (typically `$(NF-1)`) regardless of trailing classifier. Or use awk-based total computation rather than bash arithmetic. ~10-line fix.

**Estimated effort:** Trivial.

### R-10b — Stack-usage build coverage gap (SDK + QP/C not instrumented)

**Issue:** `-fstack-usage` is applied to project sources via `target_compile_options`, but NOT to vendored Pico SDK or QP/C objects. After a clean rebuild only 22 .su files exist (project-side only); SDK functions in the call stack are invisible to stack-usage analysis. If a SDK function has a deep frame in our call path, we won't catch it.

**Recommended remediation:** Either (a) add a CMake-level flag `-fstack-usage` globally in the stack-usage build configure (risk: SDK build conflicts), or (b) write a runtime stack-watermark check that records max-observed-stack-depth per core via SDK `__StackLimit` / `__StackTop` symbols, exposed via a CLI command. Option (b) is more useful but more work. Option (a) may be straightforward if SDK accepts the flag without complaint.

**Estimated effort:** Small for (a) (~1 hour to test), Moderate for (b) (~3-4 hours).

**Note:** Current project-side max stack is 200 bytes (well below 1024 threshold), so SRAM safety isn't in question — this is about completeness of the analyzer, not a flight-safety gap.

---

## Open question: remediation execution order — RESOLVED 2026-05-12

**Answered by** the new `standards/AUDIT_GUIDANCE.md` Appendix C ("Remediation Execution Ordering"). Research-led answer:

- Treat each finding as a DO-178C-style Problem Report (open → analyzed → in-progress → verified → closed).
- Run impact analysis on the queue before sequencing (capture dependency edges + audit-gate interactions).
- Process in four categories: **(1) Gate Integrity, (2) Shared Foundations, (3) Behavior Changes, (4) Cleanup/Doc.** Within each category, dependencies before dependents; within a dependency level, higher safety risk (mission-phase exposure × intrinsic severity) before lower.
- Atomic per-PR commits by default; clustering exceptions named in C.4.
- Verification cadence: local-per-commit (merge-blocker) + scripted-suite-regression at each category transition + final regression at cycle close.

The Phase 8 sequencing for THIS audit cycle is therefore informed by Appendix C, not invented ad hoc. See the dated audit report's `## Remediation` section preamble for how the current queue maps onto the four categories.

---

## R-7 — CODING_STANDARDS.md: document Holzmann inverted-rule exemption

**Standard:** P10-2 / JSF AV Rule 25 framing
**Issue:** The Holzmann original P10 paper explicitly exempts non-terminating iterations (e.g., RTOS scheduler) from Rule 2, with the inverted requirement: prove the loop *cannot* terminate. Our code uses this exemption for `QF_run()`, `core1_sensor_loop()`, fault-halt loops, station/relay Core 1 idle wait. The exemption is currently not documented in `standards/CODING_STANDARDS.md` — readers have to re-derive it.

**Recommended remediation:** Add a short paragraph to `standards/CODING_STANDARDS.md` under the Power-of-10 section: "Rule 2 scheduler exemption per Holzmann's original paper. Non-terminating loops (RTOS scheduler, Core 1 sensor loop, fault halt) satisfy the inverted form of the rule. Statically provable non-termination = compliance, not deviation. Examples in our tree: `QF_run()` (QP/C cooperative scheduler), `core1_sensor_loop()` (Core 1 device-lifetime polling), `memmanage_fault_handler()` after R-3 remediation (intentional reset-loop on hardfault), station/relay Core 1 idle wait. These do not appear in `ACCEPTED_STANDARDS_DEVIATIONS.md` because they are not deviations."

Plus reference to Holzmann's paper: https://spinroot.com/gerard/pdf/P10.pdf

**Estimated effort:** Trivial — ~10 lines, single edit to CODING_STANDARDS.md.

**Priority:** Low — happens to be a parallel cleanup to R-1 through R-6, doesn't block anything.

---

## Summary table

| ID | Item | Priority | Effort | Dependency |
|----|------|----------|--------|------------|
| R-1 | BM-2: bound Core 1 wait loop on vehicle | Medium | Small | After R-3 |
| R-2 | IO-2: replace GPS PMTK snprintf with const arrays | Medium | Small | None |
| R-3 | BM-4: capture-state-then-reset hardfault handler | High | Moderate | None |
| R-4 | BM-5: reuse R-3 machinery for MPU-config failure | Medium | Small | After R-3 |
| R-5 | IO-1: eliminate stdio.h from flight binary (interim) | High | Multi-commit | R-5a inventory first |
| R-6 | FP-1 false proliferation cleanup (Rule 170 misreading) | **High** | Small | **DONE** — commit `d186fc9` |
| R-6b | FP-1 reattribution to P10 Rule 9 + precedence rule in CODING_STANDARDS.md | **High** | Small | **DONE** — this commit |
| R-6c | FP-1 template-dispatch remediation (eliminate the deviation) | Medium | Moderate | After master audit |
| R-7 | CODING_STANDARDS.md Holzmann exemption note | Low | Trivial | None |

**Total of 8 remediation items** (R-1, R-2, R-3, R-4, R-5, R-6c, R-7 outstanding; R-6 and R-6b done). R-3 and R-5 are the high-priority outstanding ones. R-6/R-6b shipped during the audit setup phase because letting the misreading continue to spread was the highest-impact failure mode; R-6b corrected R-6's near-miss before any further work compounded it.

---

## How this folds into the master audit

This file is a **pre-staged remediation queue** based on Phase A.2 findings. When the master standards audit (per `.claude/plans/streamed-launching-lagoon.md`) runs, its `## Remediation` section in `docs/audits/MASTER_STANDARDS_AUDIT_2026-05-07.md` will:

1. Open with: "Pre-staged remediation queue from Phase A.2 — see `MASTER_STANDARDS_AUDIT_2026-05-07_REMEDIATION_PRELIMINARY.md` for the seven R-1 through R-7 items."
2. Append additional remediation items found during Phase 1–7 of the master audit (Step 1 sweeps, Step 2 standards walk, FMEA findings, etc.).
3. Categorize each per the disposition flow: remediated-now / accepted-as-deviation / deferred.

At the close of the master audit, the dated report's `## Remediation` section will be self-contained — both the Phase A.2 items here AND the master-audit items, in one place. This preliminary file remains for git-history traceability.
