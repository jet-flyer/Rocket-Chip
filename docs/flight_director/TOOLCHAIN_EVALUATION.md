# Flight Director Toolchain Evaluation

**Date:** 2026-03-15
**IVP:** 67 (Toolchain Validation — QP/C + STARS + QM + SPIN)
**Status:** Complete — decisions made, rationale documented

---

## Summary

IVP-67 required evaluating four tools for the Flight Director state machine implementation. QP/C QEP was adopted as the dispatch engine. STARS, QM, and SPIN were evaluated and deferred with clear criteria for when each becomes valuable.

| Tool | Decision | When to Revisit |
|------|----------|-----------------|
| **QP/C QEP** | Adopted (vendored `lib/qep/`) | N/A — in use |
| **STARS** | Not adopted (commercial) | If budget allows commercial tooling |
| **QM** | Deferred | If statechart grows beyond ~15 states |
| **SPIN** | Deferred | Stage 9 (Active Object concurrency) |

---

## QP/C QEP — Adopted

**What it is:** Hierarchical state machine (UML statecharts / Harel formalism) dispatch engine. Portable C99, ~646 lines. Part of the QP/C Real-Time Embedded Framework by Quantum Leaps.

**Version vendored:** QP/C 8.1.3 (GPL-3.0-or-later, compatible with RocketChip GPL-3.0-or-later)

**Why adopted:**
- Formal UML statechart semantics — entry/exit actions, hierarchical states, history, transition guards
- Battle-tested in automotive, medical, and aerospace (DO-178C projects use QP)
- Portable C99 — compiles for both ARM Cortex-M33 (target) and x86/x64 (host tests)
- Minimal footprint: 646 lines, no heap allocation, no OS dependency
- QEP-only subset avoids pulling in QF/QV/QK kernel code (those come in Stage 9)

**Files vendored into `lib/qep/`:**

| File | Lines | Purpose |
|------|-------|---------|
| `qep_hsm.c` | 646 | HSM dispatch engine — `QHsm_init_`, `QHsm_dispatch_`, transition helpers |
| `qp.h` | 788 | Public API — `QEvt`, `QHsm`, `QState`, signal definitions, transition macros |
| `qp_pkg.h` | 106 | Internal package header — `QAsmAttr`, `QHSM_MAX_NEST_DEPTH_` |
| `qsafe.h` | 152 | FuSa assertions — `Q_ASSERT_ID`, `Q_ERROR_ID`, `Q_onError` declaration |
| `qs_dummy.h` | 28 | QS (software tracing) stubs — no-ops since QS is not used |
| `qp_config.h` | 24 | RocketChip-specific config — QEP-only, FuSa enabled, QS disabled |
| `qp_port.h` | 83 | Platform port — critical sections (PRIMASK on target, no-op on host) |
| `qp_app.c` | 34 | Application callbacks — `Q_onError` (host: stderr+abort), `QP_versionStr` |
| `LICENSE.txt` | 11 | License compatibility documentation |

**Custom port decisions (`qp_port.h`):**
- **Critical sections:** Target uses Pico SDK `save_and_disable_interrupts()` / `restore_interrupts()` (PRIMASK-based). Host uses no-ops (single-threaded tests).
- **`QF_CRIT_EST()` macro:** Target uses `(void)save_and_disable_interrupts()` without saving state, because `QF_CRIT_EST` is only called immediately before `Q_onError()` which is `_Noreturn`. No need to restore interrupts — the function never returns.
- **`Q_onError()` on target:** Defined in `main.cpp` (not `qp_app.c`) to access `DBG_ERROR` macro and device-specific panic behavior (disable interrupts, print module/id, spin for watchdog reset).

**Build integration:**
- Host: `add_library(qep STATIC qep_hsm.c qp_app.c)` with `ROCKETCHIP_HOST_TEST=1`
- Target: Source files added directly to `rocketchip` executable, `lib/qep` in include path
- Both: C99 compilation (extern "C" handled by QP headers)

---

## STARS Autocoder — Not Adopted

**What it is:** SafeState Systems' commercial autocoder. Takes UML statechart models and generates verified QP/C state handler code. Targets DO-178C / IEC 62304 certification workflows.

**Why not adopted:**
- Commercial license required — no open-source version available
- Introduces a proprietary code generation dependency
- The generated code would be QP/C state handler functions — the same ~50-line switch-on-signal functions we write by hand
- Hand-authored handlers are more transparent for code review and git diff

**Fallback (adopted):** Hand-authored QEP state handlers. This is the standard approach used by the majority of QP/C projects. The QEP dispatch engine (`QHsm_dispatch_`) provides the formal state machine semantics — state handlers are straightforward signal dispatch functions.

**When to revisit:** If RocketChip pursues DO-178C certification (unlikely for hobbyist product), or if a commercial license becomes available through a partnership.

---

## QM (QP Modeler) — Deferred

**What it is:** Free graphical modeling tool from QuantumLeaps. Draws UML statechart diagrams and generates QP/C state handler code. Bidirectional — can import existing handlers. Available for Windows/Linux/macOS.

**Why deferred:**
- The Flight Director statechart has 9 states (7 top-level + 2 descent sub-phases). At this scale, hand-authored handlers are more maintainable than maintaining model-code synchronization.
- QM-generated code has a distinctive style (forward declarations, model metadata comments) that differs from the project's JSF AV C++ conventions.
- Introducing QM now would add a tool dependency before the statechart design is stable.

**Value proposition:** QM becomes valuable when statecharts grow complex enough that graphical visualization aids understanding — roughly >15 states with nested hierarchy and orthogonal regions.

**When to revisit:** After Stage 8 is complete and the statechart is stable. If the state count grows significantly (e.g., adding sub-states for pyro sequencing, recovery mode sub-machines), QM's graphical view would aid design reviews. Also valuable for generating documentation diagrams.

---

## SPIN (Promela) — Deferred

**What it is:** Formal model checker for concurrent systems. Exhaustively verifies properties (deadlock freedom, liveness, safety invariants) of systems modeled in Promela (Process Meta Language). Developed by Gerard Holzmann at Bell Labs/JPL. Free and open-source.

**Why deferred:**
- The Flight Director in Stage 8 is single-threaded — QEP dispatch runs in the superloop on Core 0. There is no concurrency to verify.
- SPIN's primary value is proving properties of concurrent systems: no deadlocks, no lost events, no priority inversions, bounded queue depths.
- Writing a Promela model of a single-threaded state machine would only verify the state transition graph — which is more effectively tested by host-side unit tests (IVP-68 plans ~18 tests for state transitions).

**Value proposition:** SPIN becomes highly valuable in Stage 9 (Active Object Architecture, IVP-76–80) when the Flight Director, LED Engine, Logger, and Telemetry become independent Active Objects communicating via event queues. At that point, SPIN can verify:
- No deadlock in the AO event topology
- All events are eventually consumed (no unbounded queue growth)
- Safety properties: `AG(phase == BOOST → pyro_armed)`, `AG(¬(drogue_fired ∧ main_fired))` simultaneously
- Liveness: `AF(phase == LANDED)` from any reachable state

**When to revisit:** Stage 9 (IVP-76–80). Write a Promela model of the AO event topology as part of the migration from superloop to QV cooperative scheduler. The model should cover: AO boundaries, event catalog, pub-sub topology, queue depths, and dual-core constraints.

---

## References

- [QP/C Framework](https://www.state-machine.com/qpc) — Quantum Leaps
- [QP/C on GitHub](https://github.com/QuantumLeaps/qpc) — Source code, examples, ports
- [SPIN Model Checker](https://spinroot.com/) — Gerard Holzmann
- [STARS Autocoder](https://www.safestatesystems.com/) — SafeState Systems
- `docs/flight_director/STATE_MACHINE_FORMALISM_RESEARCH.md` — Earlier research on formalism selection
- `docs/plans/STAGE8_FLIGHT_DIRECTOR.md` — Full Stage 8 implementation plan
