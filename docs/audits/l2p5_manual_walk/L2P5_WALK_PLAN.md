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
| **Field manual** | `docs/audits/l2p5_manual_walk/L2P5_MANUAL_WALK_GUIDE.md` | What to look for *during* the walk (Classes 1–14 + §LV/§SC) | by Nathan, during the walk |
| **Work plan** | `docs/audits/l2p5_manual_walk/L2P5_WALK_PLAN.md` (this doc) | The prep→walk→close *work*; §CM gating + §RP research live here | before/around the walk |
| **Itinerary** | `docs/audits/l2p5_manual_walk/L2P5_WALK_ITINERARY.md` | Ordered 185-file coverage map + per-file lenses; the traversal/progress tracker | during the walk |

Making gates/scripts (§CM) and doing research (§RP) is **plan work**, not part of the eyeball walk. The field manual only *references* §CM/§RP as prerequisites.

**Governing principle: COMPLETENESS over brevity.** Every in-scope file walked + coverage recorded (PASS too, not only FAILs); full criteria per class; §RP researched thoroughly. Ballooning is the correct outcome, not a cost to minimize — **applies most pointedly to remediation the gates surface: fix all N, never sample/defer-to-skip/shortcut.** (Measurement turned out small — see below — but the principle stands.)

**Exit criterion (walk-ready):** Phase A + Phase D + corrections, with Phases B/C having cleared+gated the mechanical knowns, AND the Walk Itinerary built. Classes 7/8/10 are scaffolds until §RP fills them.

---

## Phase A — Field manual — ✅ DONE

Classes 1–14 built (agent-tendency watch on each); agent-blind-spot **lens** section as primary framing; §LV/§SC walk-content; §CM/§RP prerequisite stubs; Walk Itinerary companion (185 files, criticality-ordered). Corrections folded: `#pragma once`=**3** (not 1); `rc_log()` is `void` (not return-checkable); `flash_safe_execute` is **`int`** (return-checkable — belongs in CheckedFunctions); `:403→:433` citation.

## Phase B — §CM gating (centralize-into-a-script) — measurement DONE; wiring DONE 2026-06-24

**B.1 measurement — ✅ DONE** (WSL build of `/mnt/c` source; Windows-native blocked by cmd-length/quoting — see memory `wsl-on-demand-builds`). Results:

| Check | Count | Notes |
|---|---|---|
| `-Wshadow` | **1** | `cli/rc_os_commands.cpp:1332` (`kRadToDeg` shadows global). **No balloon.** (eskf.cpp/eskf_codegen.cpp have own compile-opts — unmeasured; codegen exempt) |
| `-Wfloat-equal` | **4** | `eskf_runner.cpp:292-295` wmm sentinels |
| confusable-ids | **0** | — |
| reserved-id | **4** | 2× `__StackBottom` (linker symbol → accepted, TP-2) + 2× `_test_mode_*` (project → renamed, DONE) |
| param-count >6 | **8** | ao_telemetry×3, calibration_manager×3, guard_combinator, flash_flush |

**B.2 gate wiring — ✅ DONE 2026-06-24** (CHANGELOG `2026-06-24-001`; commits `44b360e` + `60ce731`). `full_tree_clang_tidy.sh` + `check_warning_gate_coverage.py` created (single-source gated-check list; hook + `SESSION_CHECKLIST` item 17 both call it — fixes the LL-40 dual-hardcode), `ParameterThreshold=6` set, CMakeLists flags finalized to `-Werror`, planted-proofs/positive-controls green. **One council-approved divergence from the plan below:** project-API return-checking is enforced via **`[[nodiscard]]` contracts under `-Werror`** (compiler-enforced, not lost at a call site) rather than a `CheckedFunctions` list — so `.clang-tidy CheckedFunctions = flash_safe_execute` only (the one vendored fn we can't annotate). `confusable`/`reserved-id` measured (0 / handled) but deliberately **not** per-commit gated (O(n²) slow). Original plan was:

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

## Phase D — §RP research — DONE 2026-06-25

Externally-sourced, **primary-source-verified** (LL 37 anti-fabrication) criteria for the Manual classes (3, 7, 8, 10 + Manual rules in 9/13/14). Sources: C++ Core Guidelines, MISRA-C++/AUTOSAR titles, NASA/JPL, Holzmann, CERT, + guidance on reviewing AI-generated code. Fold per-class checklists into the field manual; stash sources in `docs/audits/L2P5_RP_SOURCES_<date>.md`. Run as a deep-research sitting.

**Done:** produced via **four multi-agent Workflow passes** (~48 agents, fan-out → adversarial verify → synth): (1) spine/gestalt + rule-class criteria; (2) AI-code-review general (SEI/NIST/OWASP + peer-reviewed); (3) agentic-era refresh 2024-2026 (durable error-TYPES vs `[model,year]`-stamped rates); (4) embedded applicability (ADD bare-metal modes / DROP web-centric). Stash: `docs/audits/l2p5_manual_walk/L2P5_RP_SOURCES_2026-06-25.md` (95-source corpus + verified criteria + UNVERIFIED quarantine). Folded into `L2P5_MANUAL_WALK_GUIDE.md` → **The spine** section (gestalt + AI-lens + embedded ADD/DROP + **§D mechanically-checkable→§CM** triage) + the §RP per-class map.

**Residuals (do NOT lose):**
1. ✅ **Spine-2 (function layout / altitude) re-run DONE 2026-06-25** — 20 verified criteria (P.1/P.3, F.2/F.3/F.56, ES.5/6/21/22, NL.16–21, Fowler Slide-Statements/Split-Phase) folded into the stash (Part 1 §Spine-2) + field-manual spine §A.
2. ✅ **UNVERIFIED lists cleared 2026-06-25** (primary-source pass): SF.4/5/7/8, C.20, NL.1-3, F.1/F.3 enforcement, CON54/56 → VERIFIED; Fowler smells + SLAP/Compose-Method → name-only leads. See stash Appendix clearance log.
3. ✅ **Mechanical subset → §CM gate-wiring DONE 2026-06-25.** Audited every spine-§D item against the real build config: **10 of 11 already gated** (`.clang-tidy` + `-Wall -Wextra -Werror`; `-fno-exceptions -fno-rtti` confirmed in all 76 TUs). One real gap — **missing virtual dtor (C.35/OOP52)** — wired via `-Wnon-virtual-dtor` (CMakeLists `:598/:607/:608` authored-C++ under `-Werror` + added to `check_warning_gate_coverage.py` REQUIRED); 0 current violations (no authored virtual functions), positive-controlled on the ARM compiler + 6 real TUs syntax-clean through QP/C+ETL includes. **`-Wconversion`/`-Wsign-conversion` MEASURED 2026-06-25 (not deferred-as-noise): 56 findings** (33 sign / 12 conv / 11 float-conversion) across 21 of 75 TUs, 0 compile errors — tractable; **54 located** in `docs/audits/l2p5_manual_walk/L2P5_WCONVERSION_FINDINGS_2026-06-25.md` → a **§CM mechanical remediation batch** (fix-then-gate; eskf hits bit-exact). **Class 14 demoted to mechanical (§CM/§SC), not a semantic walk** — proven tool-caught. Spine §D updated to verified-status.
4. ✅ **Protected-doc RESOLVED 2026-06-25 (repo-owner authorized).** Both naming items now homed correctly: QP/Samek-vs-JSF divergence in `QP_APPLICATION_GUIDE.md` §6.5 (a decision, not a deviation); the *project-wide* "JSF AV 50-53" over-claim fixed inside `CODING_STANDARDS.md` — "Identifier naming (JSF 45/51/52)" bullet added to Foundation → Worked consolidation decisions (peer to nullptr) + Pre-Commit Checklist `:469` reworded to reference it. Closes the false-completion (CHANGELOG `2026-06-23-001` #2 + triage §459 had claimed it done when only the nullptr bullet landed).

## Phase E — Wrap — DONE 2026-06-25 (except CHANGELOG + commit, deferred to session-end per repo-owner)

- ✅ **Triage errata at source** — dated errata block added to `RULE_VERIFIABILITY_TRIAGE.md` header: `rc_log` is `void` (N/A to return-checking); `#pragma once` was 3 headers (not 1), all remediated to `#ifndef` 2026-06-24; P10-7 canary closed; LL-40 gate dual-hardcode fixed; §459 naming over-claim resolved.
- ✅ **Field-manual Class 2** flipped open-over-claim → RESOLVED; stale `:433` citation → `~:469`.
- ✅ **Plan + WB updated** (this doc + `AGENT_WHITEBOARD.md` handoff): Phases A–E status, §RP done, §CM gate-wiring done, `-Wconversion` measured (56, not noise), `CODING_STANDARDS.md` naming over-claim resolved.
- ⏳ **CHANGELOG + commit** — deferred to session-end (repo-owner direction; the premature `2026-06-25-002` entry is to be reconciled then to cover the gate-wiring + standards fix + false-completion closure, and the working tree committed with bench_sim for the `-Wnon-virtual-dtor` CMake change).

**Walk-ready.** With A + D + corrections complete and the gates wired, the field manual is ready for the actual file-by-file **walk** (185-file itinerary). The semantic walk is the **spine + judgment-heavy classes** (3 comments, 5 scope/lifetime, 7 design, 8 templates, 9/10 concurrency & `volatile`, 13 literal-meaning); the mechanical classes (incl. **Class 14, demoted**) are gate/§CM-covered. Remaining standalone remediation surfaced this cycle: the **56 `-Wconversion`/`-Wsign-conversion` findings** (54 located) — a **§CM mechanical batch** (fix-then-gate), not a walk item.

---

## Deferred (future sessions)
- **Full re-audit of existing 3rd-party/vendored exceptions** (TP-1 etc.) against the new evaluation discipline — dedicated session (user direction 2026-06-23; only in-scope reserved-id handled now).
- Plan 2: agent-redundancy deliverables (condensed agent-facing triage; smell-first agent walk guide — §RP feeds it).
- Plan 3 / close-out: Cycle-4 remediation doc → close L2-P5 → L2-P10 (CLA-RBM).
- Reconcile triage 225 property-rows vs 274 rule-IDs (low priority).

## Verification
Build clean vehicle + station with finalized flags (`-Werror` ⇒ no shadow/float-equal left); full-tree sweep via the new script clean (or dispositioned); host ctest; bench_sim positive-control for the flag/code changes (HW_GATE Rule 3); field manual 14 classes + §RP criteria + itinerary complete; all §LV/Phase-C items dispositioned.
