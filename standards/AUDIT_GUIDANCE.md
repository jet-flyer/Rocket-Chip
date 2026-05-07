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
   Record new deviations in `STANDARDS_DEVIATIONS.md`. Append fixes to `AUDIT_REMEDIATION.md`. Update this document only for strategy changes (state-of-system trigger).

---

## Appendix A: Manual Audit Guides (Concise)

### A.1 FMEA-lite & Koopman Embedded Review

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

**External (industry standards & guidance):**
- Phil Koopman, “10 Rules for Developing Safety-Critical Code” — https://www.embedded.com/10-rules-for-developing-safety-critical-code/ (adapted for A.1: explicit preconditions/postconditions, no recursion in high-priority paths, every transition has a guard + observable signal, backup paths exercised)
- MIL-STD-1629A (Procedures for Performing a Failure Mode, Effects and Criticality Analysis) and SAE ARP4761 — standard FMEA worksheet structure, severity/probability/detection columns, and emphasis on detection/positive control used in A.1
- Wikipedia: “Failure mode and effects analysis” — https://en.wikipedia.org/wiki/Failure_mode_and_effects_analysis (terminology: failure mode, local/next-higher/end effect, dormancy/latency, risk level)
- JPL Power of 10 Rules (G.J. Holzmann) — https://spinroot.com/gerard/pdf/Power_of_Ten.pdf (cross-referenced via existing `CODING_STANDARDS.md` for bounded loops, no recursion, and explicit guards)

All external references are public, widely cited sources in safety-critical embedded systems. No proprietary or paywalled material was used.

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