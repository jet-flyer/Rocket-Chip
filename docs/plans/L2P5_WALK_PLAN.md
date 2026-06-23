# L2-P5 Walk — Work Plan (prep · walk · close-out)

**Status:** WORKING — living plan, updated as items close; freezes when L2-P5 closes.
**Established:** 2026-06-22. **Last updated:** 2026-06-23 (session handoff).
**Driver:** `docs/audits/RULE_VERIFIABILITY_TRIAGE.md` (the frozen per-rule classification).
**Closes / unblocks:** L2-P5 in `docs/PROBLEM_REPORTS.md` (then L2-P10); feeds the Cycle-4 remediation doc per `docs/plans/CYCLE_RESIDUALS_AFTER_R5.md`.
**Approved-plan provenance:** this consolidates the council-reviewed Plan 1 (originally `.claude/plans/glistening-wishing-shannon.md`) into the repo so it's permanent.

---

## The three artifacts — do not conflate

| | Document | What it is | Used |
|---|---|---|---|
| **Field manual** | `docs/audits/L2P5_MANUAL_WALK_GUIDE.md` | What to look for *during* the walk (Classes 1–14 + §LV/§SC) | by Nathan, during the walk |
| **Work plan** | `docs/plans/L2P5_WALK_PLAN.md` (this doc) | The prep→walk→close *work*; §CM gating + §RP research live here | before/around the walk |
| **Itinerary** | `docs/audits/L2P5_WALK_ITINERARY.md` | Ordered 185-file coverage map + per-file lenses; the traversal/progress tracker | during the walk |

Making gates/scripts (§CM) and doing research (§RP) is **plan work**, not part of the eyeball walk. The field manual only *references* §CM/§RP as prerequisites.

**Governing principle: COMPLETENESS over brevity.** Every in-scope file walked + coverage recorded (PASS too, not only FAILs); full criteria per class; §RP researched thoroughly. Ballooning is the correct outcome, not a cost to minimize — **applies most pointedly to remediation the gates surface: fix all N, never sample/defer-to-skip/shortcut.** (Measurement turned out small — see below — but the principle stands.)

**Exit criterion (walk-ready):** Phase A + Phase D + corrections, with Phases B/C having cleared+gated the mechanical knowns, AND the Walk Itinerary built. Classes 7/8/10 are scaffolds until §RP fills them.

---

## Phase A — Field manual — ✅ DONE

Classes 1–14 built (agent-tendency watch on each); agent-blind-spot **lens** section as primary framing; §LV/§SC walk-content; §CM/§RP prerequisite stubs; Walk Itinerary companion (185 files, criticality-ordered). Corrections folded: `#pragma once`=**3** (not 1); `rc_log()` is `void` (not return-checkable); `flash_safe_execute` is **`int`** (return-checkable — belongs in CheckedFunctions); `:403→:433` citation.

## Phase B — §CM gating (centralize-into-a-script) — measurement DONE; wiring PENDING

**B.1 measurement — ✅ DONE** (WSL build of `/mnt/c` source; Windows-native blocked by cmd-length/quoting — see memory `wsl-on-demand-builds`). Results:

| Check | Count | Notes |
|---|---|---|
| `-Wshadow` | **1** | `cli/rc_os_commands.cpp:1332` (`kRadToDeg` shadows global). **No balloon.** (eskf.cpp/eskf_codegen.cpp have own compile-opts — unmeasured; codegen exempt) |
| `-Wfloat-equal` | **4** | `eskf_runner.cpp:292-295` wmm sentinels |
| confusable-ids | **0** | — |
| reserved-id | **4** | 2× `__StackBottom` (linker symbol → accepted, TP-2) + 2× `_test_mode_*` (project → renamed, DONE) |
| param-count >6 | **8** | ao_telemetry×3, calibration_manager×3, guard_combinator, flash_flush |

**B.2 gate wiring — PENDING (do after Phase C dispositions):** create `scripts/audit/full_tree_clang_tidy.sh` owning the gated-check list (single source of truth); point `SESSION_CHECKLIST.md` item 17 at it (protected, one-time — matches item-17b pattern); update `scripts/hooks/pre-commit` to the same list; set `.clang-tidy` `CheckedFunctions` (honest list: `i2c_bus_*`, `gps_*`, `flash_safe_execute` — NOT `rc_log` which is void), `ParameterThreshold=6`, enable confusable + reserved-id; NOLINT/consolidate the 3 `__StackBottom` externs; **PROVE the script catches a planted violation** before trusting it (LL 36); finalize CMakeLists flags to `-Werror` after remediation; batch flag changes (one bench_sim).

> **Council amendments (2026-06-22, JPL+Prof+ArduPilot+Cubesat, unanimous):** measure-first; never `-Werror` a dirty tree (remediate in full first); FMEA-rank Phase C; keep CheckedFunctions honest (partial list re-creates the canary); wire gate into pre-commit + item-17 via the script (not hardcoded twice); both-tier rebuild + bench_sim for the flag change.

## Phase C — Clear KNOWN violations PRE-walk (your dispositions) — IN PROGRESS

Mechanical/known violations cleared **before** the walk (walk = semantic discovery only; new findings disposition during it).

**DONE:**
- Reserved-id: `__StackBottom` → **accepted-via-vendoring** (TP-2 in `ACCEPTED_STANDARDS_DEVIATIONS.md`; rule-reason doesn't hold — linker-defined, referenced not authored; precedent Infineon/Xen). `_test_mode_boot_ms`/`_TEST_MODE_SRAM_ATTR` → **renamed** (dropped reserved leading-underscore) in `test_mode.cpp`.
- **Adopted-code policy** established: `CODING_STANDARDS.md` "Adopted (Third-Party/Vendored/Toolchain) Code" subsection (NEVER auto-accept; per-flag evaluation = sequestered + rule-reason-still-holds; two registers; hard floor; scalable to formal MISRA GRP/GCS for Nova/RC-Pro/cubesat/Starcom) + register formalized in `ACCEPTED_STANDARDS_DEVIATIONS.md`.

**RESOLVED (2026-06-23) — dispositions signed off; fix-oriented throughout (no pre-judged accepts). Records written to the standards docs; code changes land in the remediation batch (bench_sim).**

| # | Finding | Sites | Disposition |
|---|---|---|---|
| 1 | shadow | `cli/rc_os_commands.cpp:1332` (`kRadToDeg`) | **FIX** — rename local |
| 2 | float `==` ×4 | `eskf_runner.cpp:292-295` | **FIX** — `CAL_STATUS_WMM_SET` bit in the *existing* `cal_flags` bitfield (no flash-layout change; `static_assert(sizeof==180)` holds), replacing the `!= 0.0f` "unset" sentinel; profile-default path gets a has-default flag. Removes the `0,0`-as-unset ambiguity (GPS firmwares default to `0,0` uninitialized — repo-owner note). NOT accepted. |
| 3 | >6 params ×8 | ao_telemetry×3, calibration_manager×3, guard_combinator, flash_flush | **FIX (refactor)** — project code that *interacts with* MAVLink (not vendored): MAVLink trio → `MavCmdParams` struct; `calibration_apply_*` ×3 → `Vec3` in/out (3 params); `init_combinator` + `save_flight_entry` → config/layout structs |
| 4 | `offsetof` ×2 | `calibration_data.cpp:106,119` | **FIX** — `dataEnd - dataStart` pointer arithmetic (reuses existing `reinterpret_cast`; removes `offsetof`; layout-independent). NOT accepted. |
| 5 | `#pragma once` ×3 | `fault_protection.h`, `mission_profile_data.h`, `shared_state.h` | **FIX** — convert to `#ifndef` include guards |
| 6 | `continue` ×28 | 28 sites / 12 files | **FIX** — guard-skip inversions (cheap; JSF-190 adopted as most-restrictive per Foundation → Worked consolidation decisions) |
| 7 | JSF 175 (nullptr) | — | **RECORDED + FIX** — supersession written (CODING_STANDARDS Foundation → Worked consolidation decisions, Null pointer); QP/C `(void *)0` sender idiom ×23 → `nullptr` (verified codebase otherwise uniformly `nullptr`) |
| 7b | JSF 182 (pointer casts) | `src/` `reinterpret_cast` inventory | **RECORDED + FIX** — per-category vs the rule's real exceptions: 1a literal/HW-addr→ptr **compliant** (Exc 2); CRC struct→`uint8_t*` + buffer→header overlay **remediated** (Exc 1: `const void*` param / `static_cast`); QEvt downcast **centralized** into `evt_cast<E>`; two residuals logged — **CAST-1** (MPU→`uintptr_t`, HW-interface) + **CAST-2** (`evt_cast`, std-layout). Triage over-claim corrected at source. |
| 8 | JSF 113 (single-exit) | — | **RECORDED** — non-enforcement (Foundation → Worked consolidation decisions + cross-standard map) |

*Surfaced & added this cycle (not in the original 8):* QP/C `(void *)0`→`nullptr` ×23 (under #7); JSF-182 `evt_cast` centralization + CAST-2 (under #7b); the QP/C-vs-QP/C++ framework-choice question (WB medium-priority — the C++ edition would retire CAST-2).

## Phase D — §RP research — PENDING

Externally-sourced, **primary-source-verified** (LL 37 anti-fabrication) criteria for the Manual classes (3, 7, 8, 10 + Manual rules in 9/13/14). Sources: C++ Core Guidelines, MISRA-C++/AUTOSAR titles, NASA/JPL, Holzmann, CERT, + guidance on reviewing AI-generated code. Fold per-class checklists into the field manual; stash sources in `docs/audits/L2P5_RP_SOURCES_<date>.md`. Run as a deep-research sitting.

## Phase E — Wrap — PENDING

Triage errata at source (`RULE_VERIFIABILITY_TRIAGE.md`: `#pragma once`=3, `rc_log` void, flash `int`, gate-infra dual-hardcode defect); update this plan + WB; CHANGELOG; commit (batched, bench_sim for the code/flag changes).

---

## Deferred (future sessions)
- **Full re-audit of existing 3rd-party/vendored exceptions** (TP-1 etc.) against the new evaluation discipline — dedicated session (user direction 2026-06-23; only in-scope reserved-id handled now).
- Plan 2: agent-redundancy deliverables (condensed agent-facing triage; smell-first agent walk guide — §RP feeds it).
- Plan 3 / close-out: Cycle-4 remediation doc → close L2-P5 → L2-P10 (CLA-RBM).
- Reconcile triage 225 property-rows vs 274 rule-IDs (low priority).

## Verification
Build clean vehicle + station with finalized flags (`-Werror` ⇒ no shadow/float-equal left); full-tree sweep via the new script clean (or dispositioned); host ctest; bench_sim positive-control for the flag/code changes (HW_GATE Rule 3); field manual 14 classes + §RP criteria + itinerary complete; all §LV/Phase-C items dispositioned.
