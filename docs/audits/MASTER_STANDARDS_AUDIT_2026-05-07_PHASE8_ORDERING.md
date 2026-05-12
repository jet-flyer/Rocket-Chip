# Master Standards Audit 2026-05-07 — Phase 8 Ordered Remediation Queue

**Status:** Draft — awaiting user review before Phase 8 execution begins.
**Authored:** 2026-05-12.
**Methodology:** `standards/AUDIT_GUIDANCE.md` Appendix C (Remediation Execution Ordering). DO-178C Problem Report lifecycle; 4-category sequencing (Gate Integrity → Shared Foundations → Behavior Changes → Cleanup); verification cadence per C.5.
**Companion:** `MASTER_STANDARDS_AUDIT_2026-05-07_REMEDIATION_PRELIMINARY.md` is the impact-analysis source (each R-N item's "Dependency:" / "Files / mechanism" lines).

---

## Preamble

This audit's remediation queue contains **18 Problem Reports** at Phase 8 entry. Applying Appendix C produces the 4-category ordering below. The order is informed by:

1. **Impact analysis (Appendix C.2):** each PR's "Files / mechanism" footprint and dependency edges drawn from the preliminary remediation file's `Dependency:` lines.
2. **Category assignment (Appendix C.3):** Gate Integrity → Shared Foundations → Behavior Changes → Cleanup.
3. **Within-category ordering:** dependencies before dependents; within a dependency level, higher mission-phase-exposure × intrinsic-severity first.

**Deviations from canonical order:** none. Standard four-category sequencing applies.

**Verification cadence (Appendix C.5):**
- Per commit: Level 1 local verification (per-commit gates, change's own positive-control signal). Cite per `HW_GATE_DISCIPLINE.md` Rule 3 + new Rule 6.
- At each category transition: Level 2 audit-suite regression (Phase 1 + Phase 6 scripted-check suite re-runs end-to-end).
- At cycle close: final regression after Category 4 cleanup, before the dated audit's `## Remediation` section is signed.

---

## Dependency graph (compact form)

```
R-3 ─┬─→ R-4   (MPU-config failure reuses R-3's preserved-SRAM machinery)
     └─→ R-1   (bounded-boot-wait fault path reuses R-3's halt mechanism)

R-5a → R-5b → R-5c → R-5d   (inventory → dev/ migration → custom writer → cleanup)

R-12 ⊃ R-1   (formal-verification model that closes R-1's correctness gap;
              landing R-1 the code change and R-12 the SPIN model is
              independent ordering, but R-12 best lands after R-1 so the
              model matches the code)

All other PRs are independent (no inbound or outbound dependencies in this queue).
```

---

## Category 1 — Gate Integrity

The audit's own tools and gates must be honest before the rest of the work lands. Per Appendix C.3 Category 1: "If a gate is reporting green for the wrong reason, every subsequent 'verified' claim inherits the same defect."

Five PRs in this category. All land before any Category 2 work begins.

| Order | PR | Title | Effort | Notes |
|---|---|---|---|---|
| 1 | **P8-SPIN-B** | Delete `rocketchip_fd_pp.pml` zero-byte stub | Trivial | Empty file implies coverage that doesn't exist. Hard Cat 1 — false signal. |
| 2 | **P8-SPIN-A** | Extend `run_stage_o_ao_spin.sh` to gate all 4 active models | Small (~half day shell) | Currently runs only `rocketchip_ao.pml`. Must include `rocketchip_fd.pml`, `rocketchip_rf_manager.pml`, `rocketchip_station.pml`. |
| 3 | **R-10a** | Fix `scripts/analyze_stack_usage.sh` parser bug | Trivial (~10 lines) | Script silently reports `Total: 0 KiB` due to "static" token mishandling. Gate reports green for wrong reason. |
| 4 | **R-9a** | Add `fault_force_launch_abort()` to `src/dev/fault_inject.cpp` | Small (~30 lines) | Enables `enhanced_fault_injection.py --scenario launch-abort` to observe `[FD] ABORT*` positive-control signal. |
| 5 | **R-9b** | Add `fault_force_radio_dropout()` to `src/dev/fault_inject.cpp` | Small (~20 lines) | Enables radio-dropout scenario to observe link-lost positive-control signal. |

**Category 1 → 2 gate (Level 2 regression):**
- SPIN re-run: now gates 4 models, all PASS.
- `analyze_stack_usage.sh` re-run on existing `.su` files: produces non-zero total.
- `enhanced_fault_injection.py --scenario launch-abort` observes `[FD] ABORT*` end-to-end.
- `enhanced_fault_injection.py --scenario radio-dropout` observes link-lost log end-to-end.
- Host ctest 788/788 PASS (no regression in unrelated tests).

If any of these fail, halt and re-do per Appendix C.7.

---

## Category 2 — Shared Foundations

One PR with high out-degree in the dependency graph. R-1 and R-4 both depend on it.

| Order | PR | Title | Effort | Notes |
|---|---|---|---|---|
| 6 | **R-3** | Capture-state-then-reset hardfault handler (BM-4) | Moderate | Two-stage: (a) `memmanage_fault_handler()` captures CFSR/HFSR/PC to `.uninitialized_ram` and triggers `NVIC_SystemReset()`; (b) banner code on next boot detects the magic and transitions to FAULT health state via existing machinery. Industry-standard pattern (ArduPilot, PX4, NASA cFS). |

**Category 2 → 3 gate (Level 2 regression):**
- Host ctest 788/788 PASS.
- Bench HW verify with `src/dev/fault_inject.cpp`'s forced MPU fault: confirm capture record appears in preserved SRAM via GDB, reset proceeds cleanly, post-reset banner reports previous fault.
- Phase 6 SPIN re-run: 4/4 PASS (no regression from R-3's preserved-SRAM use).
- 3-boot reseat per `HW_GATE_DISCIPLINE.md` Rule 2: post-reset banner reports identical fault info across all 3 boots.

---

## Category 3 — Behavior Changes

11 PRs. Ordered within the category by mission-phase exposure × intrinsic severity. Flight-critical paths first, then ground/pre-flight, then SPIN-model extensions, then audit task.

### Flight-critical paths

| Order | PR | Title | Effort | Notes |
|---|---|---|---|---|
| 7 | **R-4** | MPU-config failure handling (BM-5) | Small after R-3 | Reuses R-3's preserved-SRAM machinery. Different crash-type code. Boot-time, flight-critical. |
| 8 | **R-1** | Bound Core 1 boot-wait loop on vehicle path (BM-2) | Small (~10 lines) | Adds max-iteration cap. Failure flows through R-3's fault path. Vehicle-only — station/relay path satisfies Holzmann scheduler exemption unchanged. |
| 9 | **R-9c** | Log-on-change emit in `src/safety/health_monitor.cpp` | Moderate (~40-60 lines) | Subsystem health bit transitions OK↔FAULT now emit `[Health] <subsystem> FAULT` / `RECOVERED`. Closes Koopman K3 gap for health-monitor state machine. Benefits real flight operations (GCS-observable). |

### Ground / pre-flight paths

| Order | PR | Title | Effort | Notes |
|---|---|---|---|---|
| 10 | **R-5a** | Inventory + classify all `<stdio.h>` callsites in flight binary | Hours, doc-only | Grep + 3-bucket classification: (i) dev-only → `src/dev/`, (ii) flight-needed bounded writer, (iii) driver diagnostic → `src/dev/`. Output: scratch table. |
| 11 | **R-5b** | Move dev/diagnostic printfs to `src/dev/` | Per-file, small each | Eliminates ~80% of stdio.h callsites per the inventory's classification. |
| 12 | **R-5c** | Custom bounded writer `rc_write()` for remaining flight-binary callsites | Moderate | Direct-to-USB-CDC, no `<stdio.h>`. |
| 13 | **R-5d** | Final cleanup: zero `<stdio.h>` includes in `NOT_CERTIFIED_FOR_FLIGHT=OFF` flight binary | Small | Verifies IO-1 deviation eliminated. |
| 14 | **R-2** | GPS PMTK `snprintf` → const arrays | Small (~50 lines net) | 10 callsites across `gps_pa1010d.cpp` + `gps_uart.cpp`. PMTK commands are fixed strings — no runtime formatting needed. Eliminates IO-2 deviation entirely for these files. |
| 15 | **R-6c** | FP-1 template-dispatch (eliminate function-pointer deviation) | Moderate | `lm_solve()` from function pointers → compile-time template params. Identical machine code; eliminates P10 Rule 9 deviation. Requires moving body into header + careful numerical equivalence testing against existing calibration outputs. |

### Formal-verification extensions

| Order | PR | Title | Effort | Notes |
|---|---|---|---|---|
| 16 | **R-11** | New SPIN model: `tools/spin/rocketchip_flash_protocol.pml` | ~1 day | Models flash-caller + Core 1 lockout + I2C peripheral as 3 processes. LTL properties for flash/I2C ordering invariants. Council-flagged HIGHEST-ROI extension (would have caught LL Entry 28 + 31). |
| 17 | **R-12** | New SPIN model: `tools/spin/rocketchip_boot.pml` | ~1 afternoon | Models Core 0 + Core 1 + `g_startSensorPhase`. Closes R-1's correctness gap with formal verification. Best landed after R-1's code change so model and code match. |

### Audit sub-task

| Order | PR | Title | Effort | Notes |
|---|---|---|---|---|
| 18 | **P8-FMEA-Pyro** | Pyro intent/arm/disarm FMEA sub-check | Bounded review | Review pyro state-machine transitions for multi-transition failure modes. Two outcomes: (a) new findings → queue R-14 (unified pyro protocol SPIN model); (b) no findings → log as audited-and-no-finding in the dated report. |
| -- | **R-10b** | Stack-usage build coverage gap (SDK + QP/C not instrumented) | Small (a) or Moderate (b) | **DEFERRED** — current project-side max stack is 200 bytes (well below 1024 threshold), so SRAM safety isn't in question. This is completeness of the analyzer, not a flight-safety gap. **Safety-impact rationale:** existing stack safety is verified empirically; R-10b's gap is in coverage of a check that has already passed, not in the underlying property. Re-evaluate at next stage close if firmware stack usage grows. |

**Category 3 → 4 gate (Level 2 regression):**
- Host ctest 788/788 PASS.
- Phase 1 scripted suite re-run end-to-end: clang-tidy clean across all 4 tiers, cppcheck clean, lizard CCN within bounds, coverage threshold for `src/active_objects/` + `src/eskf/` ≥80%.
- Phase 6 SPIN re-run: 6 models (4 existing + R-11 flash protocol + R-12 boot handshake), all PASS.
- Calibration numerical equivalence: 6-pos + ellipsoid fit outputs bit-identical before vs after R-6c (or documented near-bit-identical with rationale).
- 3-boot reseat: positive-control signals identical across boots per `HW_GATE_DISCIPLINE.md` Rule 2.
- `enhanced_fault_injection.py --scenario all`: 4/4 scenarios observe their firmware-side positive-control signal (post R-9a, R-9b, R-9c).

---

## Category 4 — Cleanup & Documentation

Two PRs. Last band before close.

| Order | PR | Title | Effort | Notes |
|---|---|---|---|---|
| 19 | **R-7** | Document Holzmann inverted-rule exemption in `CODING_STANDARDS.md` | Trivial (~10 lines) | Already covered in `CODING_STANDARDS.md` Foundation section (P10 Rule 2 example). Verify and close as either "already done" or short addition. |
| 20 | **R-13** | SESSION_CHECKLIST trigger-map row for SPIN model ride-along | Trivial (1 row) | Adds a state-of-system protected-doc trigger row: when firmware behavior matching a candidate SPIN extension lands, the corresponding `.pml` edit rides in the same commit. Codifies the discipline that would otherwise rely on memory. |

**Cycle-close gate (Level 2 final regression):**
- Full audit-suite regression one final time post-cleanup commits.
- Phase 1 + Phase 6 PASS end-to-end against the as-shipped state.
- Dated audit report's `## Remediation` section signed with each PR's closure citing observed verification signal.
- Single CHANGELOG entry `### 2026-05-07-NNN` summarizing the audit + remediation cycle.
- PROBLEM_REPORTS.md updated: all 18 PRs migrate to either `closed` or `accepted` or (for R-10b) `deferred` with safety-impact rationale.

---

## DEFER list (with safety-impact rationale per AUDIT_GUIDANCE Step 8)

| PR | Why deferred | Safety-impact rationale |
|---|---|---|
| **R-10b** | Tooling completeness gap, not safety gap | Project-side max stack is 200 bytes vs 1024-byte threshold — empirically verified well within margin. R-10b would extend `-fstack-usage` to SDK/QP/C objects; the property it would prove is already proven by absence of stack-related crashes across 4 stages + thousands of soak-test reads. Re-evaluate at next milestone close if firmware stack usage grows above 50% of threshold. |

No other PRs are deferred. Per the project's "default disposition is REMEDIATE, not ACCEPT" policy (per `ACCEPTED_STANDARDS_DEVIATIONS.md`), every other open PR will be remediated in this cycle.

---

## Independence note (per HW_GATE_DISCIPLINE Rule 6 + Appendix C.6)

This Phase 8 cycle will produce Level 1 verification at each commit and Level 2 verification at each category-transition gate. **Independent verification (Level 3) cannot be performed within this cycle** — the fix author and the gate-runner are the same agent. The next audit cycle (or a focused re-audit per `AUDIT_GUIDANCE.md` decision-table row 6) provides Level 3 credit retroactively.

Closures in this cycle's `## Remediation` section will cite Level 2 credit, not Level 3. PROBLEM_REPORTS.md rows will reflect this.

---

## Effort estimate (rough)

- Category 1: ~3-4 hours (mostly trivial fixes + one moderate dev/ hook)
- Category 2: ~3-5 hours (R-3 moderate refactor + bench verification)
- Category 3: ~8-12 hours total (R-5 multi-step is the biggest single item)
- Category 4: ~30 min
- Inter-band regressions: ~1 hour × 3 transitions = ~3 hours

**Total: ~18-24 hours of focused work**, spread across multiple sessions. Each session ends at a category boundary or a clean per-PR commit boundary, never mid-category.

---

*Ordering authored 2026-05-12. Awaiting user review before Phase 8 work begins. Once approved, this file becomes the execution plan for Phase 8; deviations from the plan get a one-line rationale recorded in the dated audit's `## Remediation` section.*
