# Master Standards Audit 2026-05-07 — Preliminary Remediation Queue

**Status:** Preliminary. Intended to be folded into the dated audit report's `## Remediation` section once the full Master Standards Audit (per `.claude/plans/streamed-launching-lagoon.md`) is run.

**Source:** Phase A.2 (accepted-deviations review) findings from the renaming + re-evaluation of `standards/ACCEPTED_STANDARDS_DEVIATIONS.md`. Of the 13 rows in the file at the start of Phase A.2:

- 1 row remains accepted (CG-1, auto-generated codegen — only canonical case)
- 2 rows moved to Resolved (AP-1, AP-3 — code refactored away)
- 4 rows deleted as non-deviations or misreadings (AP-2, AP-4, FP-1, and the BM-1/BM-3 framing of P10-2 — see "Holzmann inverted-rule exemption" below)
- 6 items moved here as remediation queue (BM-2, BM-4, BM-5, FP-1 false proliferation, IO-1, IO-2)

---

## R-1 — BM-2: Core 1 boot-wait loop bounded on vehicle path

**Standard:** P10-2 / JSF AV Rule 25 (loops must have fixed upper bound)
**Location:** `src/core1/sensor_core1.cpp:490` (`core1_entry()` `while (!g_startSensorPhase) { sleep_ms(10); }`)
**Status:** Compliant on station/relay path via Holzmann's inverted-rule exemption (intentionally non-terminating — Core 1 idles forever). On vehicle, the loop terminates when Core 0 signals sensor phase start; bounded in practice but not statically provable.
**Recommended remediation:** Add a max-iteration count (e.g., 1000 iterations × 10 ms = 10 s max boot wait — far above the realistic Core 0 boot sequence ≤1 s). If exceeded, halt with a fault that flows through the same FAULT-health-state mechanism used elsewhere (see R-3/R-4 for the broader fault-handler remediation that this should integrate with). Static analyzer can then prove the bound.
**Estimated effort:** Small — ~10 lines, single function.
**Dependency:** Best done after R-3/R-4 lands (so the bounded-loop fault path can reuse the new capture-state-then-reset machinery instead of re-inventing a halt pattern).

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

**Estimated effort:** Moderate — ~50–100 lines in `fault_protection.cpp` + small boot-banner code change in `main.cpp`. Worth doing.
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

## R-6 — FP-1 false proliferation cleanup

**Standard:** N/A (this is a misreading-correction task, not a standards-compliance task)
**Issue:** JSF AV Rule 170 was misread as "function pointers prohibited" when its actual wording is "More than 2 levels of pointer indirection shall not be used." Function pointers are not prohibited by JSF; the related rule is Rule 176 ("a typedef will be used to simplify program syntax when declaring function pointers"), which we already comply with via the `ResidualFn` / `JacobianFn` typedefs.

**Affected files** (state-of-system — must update):

| File | Issue |
|---|---|
| `src/calibration/calibration_manager.cpp:950` | Code comment cites Rule 170 as deviation. Remove citation. |
| `src/calibration/calibration_manager.cpp:984` | Comment "See ACCEPTED_STANDARDS_DEVIATIONS.md FP-1" — FP-1 deleted, update comment. |
| `include/rocketchip/notify_backend.h:6` | Header comment cites Rule 170 as rationale for "no function pointer vtable." Reword: the engineering choice stands on compile-time-dispatch merits; the standards-compliance framing was wrong. |
| `docs/PROJECT_STATUS.md:71` | State-of-system entry mentions Rule 170 rationale. Drop the JSF-170 citation. |
| `docs/decisions/NOTIFY_CONTRACT.md` (lines 52, 67) | **Historical-record doc** — frozen on commit per `.claude/SESSION_CHECKLIST.md`. Apply a top-of-file supersession note (not a direct edit to body): "2026-05-07 correction: Rule 170 citations in this document are based on a misreading. Rule 170 governs pointer-indirection depth (≤2 levels), not function-pointer prohibition. The engineering choice (direct function calls, no vtable) stands on compile-time-dispatch merits; the standards-compliance framing was wrong." |
| `docs/IVP.md:3047` (Notify IVP entry) | **Mixed-mode doc**, but the per-IVP entry is historical. Same supersession-note treatment as NOTIFY_CONTRACT.md if needed — or correct in the next IVP entry rather than editing the old one. |
| `CHANGELOG.md:540` and other historical mentions | **Historical record — DO NOT EDIT.** Future readers will understand the chronology via the supersession note in NOTIFY_CONTRACT.md and the LL entry below. |

**Add a LESSONS_LEARNED entry** documenting the misreading + the verification discipline (verify rule wording against primary source before citing) so the misreading doesn't recur.

**Estimated effort:** Small per file (each is a single-line edit), but spans 6+ files. ~1 hour total.

**Priority:** High. The user flagged this as a high-priority item ("be sure no issues were caused" + "fine-toothed comb"). Done as a focused commit immediately after the Phase A.2 commit, before any other audit work, so the false framing doesn't continue to spread.

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
| R-6 | FP-1 false proliferation cleanup | **High** | Small | Done immediately after Phase A.2 |
| R-7 | CODING_STANDARDS.md Holzmann exemption note | Low | Trivial | None |

**Total of 7 remediation items.** R-3, R-5, R-6 are the high-priority ones. R-6 should land as a focused commit immediately after Phase A.2's deviations-file edit, because letting the misreading continue to spread is the highest-impact failure mode.

---

## How this folds into the master audit

This file is a **pre-staged remediation queue** based on Phase A.2 findings. When the master standards audit (per `.claude/plans/streamed-launching-lagoon.md`) runs, its `## Remediation` section in `docs/audits/MASTER_STANDARDS_AUDIT_2026-05-07.md` will:

1. Open with: "Pre-staged remediation queue from Phase A.2 — see `MASTER_STANDARDS_AUDIT_2026-05-07_REMEDIATION_PRELIMINARY.md` for the seven R-1 through R-7 items."
2. Append additional remediation items found during Phase 1–7 of the master audit (Step 1 sweeps, Step 2 standards walk, FMEA findings, etc.).
3. Categorize each per the disposition flow: remediated-now / accepted-as-deviation / deferred.

At the close of the master audit, the dated report's `## Remediation` section will be self-contained — both the Phase A.2 items here AND the master-audit items, in one place. This preliminary file remains for git-history traceability.
