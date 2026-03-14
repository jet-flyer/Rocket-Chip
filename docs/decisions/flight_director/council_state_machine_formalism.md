# Council Review: Flight Director State Machine Formalism & QP Framework Adoption

**Date:** 2026-03-10
**Convened by:** Nathan (Project Lead)
**Panel:** ArduPilot Core Contributor, Retired NASA/JPL Avionics Lead, Embedded Systems Professor, CubeSat Startup Engineer
**Status:** CONSENSUS REACHED — All decisions unanimous

---

## Question on the Table

1. **Primary:** What state machine formalism should the RocketChip Flight Director use? (Resolves SAD Open Question #4: Mealy vs Moore)
2. **Secondary:** Should the project adopt the QP/C framework and/or STARS autocoder toolchain?
3. **Secondary:** Does Stage 8 need restructuring to accommodate these decisions?

---

## Context Provided to Panel

- SAD Open Question #4 (Mealy vs Moore)
- Research findings: NASA JPL Perseverance uses UML statecharts via QEP (~40 HSMs in surface component). JPL uses QEP only, not full QP framework.
- QP/C framework (QEP event processor, QF active object framework, QV cooperative kernel, QS tracing)
- JPL STARS autocoder (open source, generates C++/Promela/Python from QM statechart models)
- Existing FLIGHT_DIRECTOR_DESIGN.md (preliminary placeholder, not authoritative)
- RocketChip's existing bare-metal Pico SDK dual-core AMP architecture

---

## Round 1: Initial Positions

### NASA/JPL Avionics Lead

Confirmed research findings correct from direct experience. The Samek HSM dispatch pattern has been in JPL flight software since well before Perseverance — SCA was developed for the Space Interferometry Mission, used on MRO's Electra radio, validated on MSL's landing radar.

**Key position:** Framing as Mealy vs Moore was always the wrong question. UML statecharts are the correct answer. Every safety-critical state machine in their experience uses entry/exit actions for invariant setup/teardown and transition actions for path-dependent behavior.

**Critical safety argument:** Irreversible actions must be transition-gated, not state-gated. If pyro firing is an entry action, any future developer who adds a new path into that state accidentally fires a pyro. Transition actions force explicit decision for every path.

### ArduPilot Core Contributor

Cited ArduPilot's mode system as an example of what happens without a formal framework — adding a new flight mode requires touching 8-12 files with implicit undocumented contracts. Strongly favored committing to UML statecharts.

**Key observation:** The existing FLIGHT_DIRECTOR_DESIGN.md already describes the QEP pattern (SIG_ENTRY, SIG_EXIT, TRANSITION(), SUPER()) without naming it. The design was already heading this direction.

**Practical concern:** QP assumes it owns the execution model. Recommended QEP-only extraction (same approach as JPL) rather than full QP, to avoid conflict with existing superloop.

**Cycle budget:** QEP dispatch <5µs per event at 150MHz. At 100Hz tick rate with 10ms budget, this is noise compared to ESKF (561µs average).

### Embedded Systems Professor

Grounded the discussion in Harel's 1987 formalism. UML statecharts are a strict superset of both Mealy and Moore — nothing is lost by adopting the hybrid.

**Clarified three action locations:**
- Entry actions = constructors (establish state invariants)
- Exit actions = destructors (guaranteed cleanup)
- Transition actions = source-dependent operations (execute between exit and entry)

**Advocated for QEP framework** over hand-rolling: the LCA (Least Common Ancestor) calculation for hierarchical transitions is subtle and easy to get wrong. QEP handles it correctly by construction.

**Advocated for STARS** as single-source-of-truth: the statechart model IS the documentation, the generated code IS the implementation. They cannot diverge.

### CubeSat Startup Engineer

**Pushed back on scope:** The Flight Director as described is a small state machine (~9 states, ~15-20 transitions). Questioned whether the toolchain overhead of STARS is justified.

**But conceded:** Cited a real CubeSat EPS bug caused by lack of guaranteed exit-before-entry ordering in a hand-rolled FSM. A proper HSM would have prevented it.

**Voted for QEP** (extracted standalone, ~200 lines of C, not a "toolchain" but a library file). **Deferred STARS** until after first working state machine.

**Proposed bench flight simulation IVP** before mission configuration — end-to-end integration test of the state machine.

---

## Round 2: Convergence on Formalism, Debate on Toolchain

### Revised Positions on STARS

**Professor** initially revised to "STARS as verification overlay" (hand-authored code primary, periodic SPIN verification) due to concerns about multi-agent workflow editing QM model files.

**NASA/JPL Lead** agreed: "In flight software, the code that runs on the vehicle is the ultimate source of truth."

### Stage 8 Restructuring

**NASA/JPL Lead:** IVP-67 as written ("implement states and transitions") is at least 2-3 IVPs of actual work.

**ArduPilot Contributor** proposed concrete IVP boundaries:
- IVP-67: QEP Integration + Phase Skeleton (stubs, flat dispatch, CLI injection)
- IVP-67b: Command Handler + Pre-Arm (both tiers)
- IVP-68a: Core guards (5-6 needed for rocket profile, sustain timing, compare_field helper)
- IVP-68b: Multi-method logic and edge detection (AND/OR/TIMEOUT, zero-crossing)
- IVP-69: Action Executor (unchanged)
- New IVP-69b: Bench Flight Simulation (full sequence via CLI-injected stimuli)
- IVP-70: Mission Configuration (rewritten to match FEMA decision: baked-in profile)

**CubeSat Engineer:** Supported breakdown. Added constraint: each IVP should be achievable in 1-3 focused sessions. If any single IVP exceeds a week of work, split it again.

---

## Round 3 (Post-Feedback): Apogee Reclassification

### Nathan's Feedback

"Apogee is really a momentary event, not a persistent state."

### Panel Response — Unanimous Agreement

**NASA/JPL Lead:** "Nathan is absolutely correct. Apogee is a detection event (EVT_APOGEE), not a flight phase. The vehicle doesn't 'be in apogee.' The design doc's treatment of APOGEE as a persistent phase with a 5-second timeout was a workaround for the fact that it shouldn't be a state in the first place."

**Corrected topology:**

```
COAST → (EVT_APOGEE, fire drogue) → DESCENT.DROGUE → (EVT_MAIN_DEPLOY) → DESCENT.MAIN
```

Top-level phases: IDLE, ARMED, BOOST, COAST, DESCENT, LANDED, ABORT (seven states). DESCENT is a superstate containing DROGUE and MAIN sub-phases. APOGEE is purely an event.

**ArduPilot Contributor:** "This maps perfectly to the HSM. `state_flight` superstate contains BOOST, COAST, and DESCENT (itself a superstate). Common abort handling lives on `state_flight`. Tight and clean."

**Professor:** "General principle: if something has no meaningful dwell time and no unique sustained behavior, it's an event, not a state."

---

## Round 4 (Post-Feedback): STARS Viability Revised

### Nathan's Feedback

The multi-agent workflow has evolved toward single-developer with agent cross-checks and a protected file system. The STARS single-source-of-truth model is more viable than the council initially assumed.

### Panel Response — Position Shift

**Professor:** "Given the actual single-author workflow, I'm reversing my earlier hedge. STARS as primary authoring tool is viable and preferable. The statechart model becomes the design document, the generated code is the implementation, and SPIN verification is built into the pipeline from day one."

**ArduPilot Contributor:** Revised to "evaluate STARS integration as part of Stage 8, not deferred."

**CubeSat Engineer:** "Make the first IVP of Stage 8 a toolchain validation IVP. Install STARS, QM, SPIN. Draw a trivial 3-state test statechart. Generate C++, Promela, Python sim. If this takes one session, proceed with STARS as primary. If it turns into a multi-session fight, fall back to hand-authored QEP."

---

## Round 5 (Post-Feedback): QF+QV as Target Architecture

### Nathan's Feedback

The LED behavior fragmentation problem ("why is it solid green") is a symptom of missing subsystem boundaries. Need for error handling engine and LED engine as independent state machines. Asked whether QF+QV is worth full adoption rather than QEP-only. Asked whether QF requires an RTOS.

### Key Finding: QF Does Not Require an RTOS

QP/C's QV cooperative kernel works essentially as a superloop — all active objects execute in the main loop, interrupts always return to the point of preemption. It is functionally identical to the existing architecture but with formalized event passing between subsystems.

### Panel Response — Revised Architecture

**NASA/JPL Lead:** "I need to revise my earlier position. The LED behavior problem is a textbook symptom of missing subsystem boundaries. QF+QV on Core 0 would replace the superloop with something functionally identical but with formalized event passing. The upgrade path from QEP to QF is clean: state handler code is identical, you're wrapping it in an Active Object that gives it an event queue."

**ArduPilot Contributor:** "The LED problem sold me. Under QV, the superloop's `_due` flags and direct function calls become typed events and QP's built-in periodic timers. Same cooperative execution, but cleaner."

**Professor:** "Active Objects enforce encapsulation through event queues. The LED Engine's state is private. The only way to affect it is to send it an event."

**CubeSat Engineer:** "We're in danger of turning Stage 8 from 'build a flight state machine' into 'rearchitect the entire firmware.' Propose: Stage 8 delivers Flight Director using QEP from superloop. LED Engine, error handler, and QV migration happen in a dedicated post-Stage-8 refactoring stage."

**Nathan concurred** with the CubeSat Engineer's scoping: build Stage 8 as intended, break out Active Object migration into a dedicated subsequent stage.

### Critical Constraint (Unanimous)

**QF+QV stays entirely on Core 0.** Core 1 remains the deterministic sensor sampling loop. The seqlock boundary between cores is unchanged. QF does not cross that boundary.

---

## Final Decisions

All decisions unanimous. No dissent.

### Decision 1: UML Statecharts as Formalism

**Resolves SAD Open Question #4.** The answer is neither pure Mealy nor pure Moore — it is UML statecharts (Harel statecharts), which is the formalism used by NASA JPL on Perseverance and supported by all major aerospace state machine tools.

- Entry/exit actions for state-invariant behavior
- Transition actions for source-dependent irreversible operations
- Hierarchical nesting for behavioral inheritance

### Decision 2: Transition-Gated Irreversible Actions (Safety Architecture)

Irreversible actions (pyro firing) must be transition actions, not entry actions. This is a safety architecture decision, not a design preference. Must be documented as a safety rationale in the Flight Director design doc so future developers understand the constraint and don't "simplify" it.

### Decision 3: Apogee Is an Event, Not a State

EVT_APOGEE is a detection event (velocity zero-crossing). Drogue deploy is a transition action on COAST→DESCENT. Top-level phases: IDLE, ARMED, BOOST, COAST, DESCENT, LANDED, ABORT (seven states). DESCENT is a superstate with DROGUE and MAIN sub-phases.

### Decision 4: QEP for Stage 8, QF+QV as Confirmed Target Architecture

- **Stage 8:** Flight Director uses QEP, called from existing superloop. Ships crowdfunding demo.
- **Toolchain validation IVP (within Stage 8):** Compile QP/C (full: QEP+QF+QV) against Pico SDK. Confirm upgrade path viable.
- **Dedicated post-Stage-8 refactoring stage:** Migrate subsystems to Active Objects under QV. LED Engine first. Superloop progressively replaced. Core 1 AMP boundary unchanged.
- State handler code written for Stage 8 is identical to what runs inside Active Objects later. Zero throwaway code.

### Decision 5: STARS as Primary Authoring Tool (Gated)

STARS autocoder evaluated in Stage 8 toolchain validation IVP. If setup is clean (one session), proceed with STARS as primary — QM model is source of truth, generated C++ is build artifact, SPIN verifies safety properties. If setup is problematic, fall back to hand-authored QEP with STARS as deferred verification overlay.

### Decision 6: Stage 8 IVP Restructuring

Split into finer-grained IVPs reflecting actual work scope. Approximate structure:

| IVP | Title | Scope |
|-----|-------|-------|
| 66 | Watchdog Recovery Policy | Unchanged from current IVP |
| NEW | Toolchain Validation | QP/C + STARS + QM + SPIN compilation gate |
| 67 | QEP Integration + Phase Skeleton | Dispatch engine, phase enum, stub handlers, CLI injection |
| 67b | Command Handler + Pre-Arm | CMD_ARM validation, both check tiers, rejection behavior |
| 68a | Core Guard Functions | 5-6 guards for rocket profile, sustain timing, compare_field |
| 68b | Multi-Method Logic | AND/OR/TIMEOUT combinators, edge detection, validity policy |
| 69 | Action Executor | LED, logging, telemetry flag actions on transitions |
| 69b | Bench Flight Simulation (SITL) | Full IDLE→LANDED sequence via simulated stimuli |
| 70 | Mission Configuration | Baked-in const profile (FEMA approach), CLI preview |

Exact IVP numbers to be finalized when IVP.md is updated.

### Decision 7: Reference Materials Added

- Samek, M. *Practical UML Statecharts in C/C++*, 2nd ed. CRC Press, 2008.
- QP/C: `github.com/QuantumLeaps/qpc`
- JPL STARS: `github.com/JPLOpenSource/STARS`
- JPL SCA: `github.com/JPLOpenSource/SCA`
- QM: `state-machine.com`
- SPIN: `spinroot.com`
- Scandore, S. "Mars Perseverance Software," EOC 2021

### Decision 8: QS (QP/Spy) Evaluated Separately

QS binary tracing evaluated as part of the post-Stage-8 QF migration stage. Potential to replace or supplement current flight event logging for state machine behavior.

### Decision 9: FLIGHT_DIRECTOR_DESIGN.md Rewrite

The existing document is a preliminary placeholder. It will be rewritten based on this council's decisions and the research findings. The rewritten version will incorporate: UML statechart formalism, corrected phase topology (apogee as event), safety rationale for transition-gated pyro actions, QEP integration architecture, and the QF+QV upgrade roadmap.

---

## Action Items

1. Update SAD.md: Close Open Question #4 with "UML statecharts (see council review record)"
2. Rewrite FLIGHT_DIRECTOR_DESIGN.md from council findings
3. Restructure IVP.md Stage 8 with finer-grained IVPs
4. Add references to project reference list
5. Toolchain validation IVP: QP/C + STARS + QM + SPIN against Pico SDK
6. Bump DPS310 to 64 SPS as Stage 8 prerequisite task

---

*Council review record for: `docs/decisions/flight_director/council_state_machine_formalism.md`*
