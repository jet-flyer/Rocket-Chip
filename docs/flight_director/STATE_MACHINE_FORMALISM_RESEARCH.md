# State Machine Formalism Research: Resolving SAD Open Question #4

**For:** RocketChip Flight Director (RP2350 bare-metal Pico SDK, dual-core AMP)
**Date:** 2026-03-10
**Resolves:** SAD Open Question #4 (Mealy vs Moore state machine)
**Status:** RESEARCH COMPLETE — Reviewed by council, decisions made

---

## 1. The Question

SAD Section 6.2 specifies Moore-style "Actions on Entry," while Section 6.3 uses Mealy-style event-condition-action syntax. The question was: should Flight Director actions be bound to states (Moore) or to transitions (Mealy)?

**Finding: The question was incorrectly framed as a binary choice.** Real aerospace flight software overwhelmingly uses UML statecharts (Harel statecharts), which combine entry/exit actions (Moore) with transition actions (Mealy) in a single formalism. This is not a compromise — it is a distinct, well-defined formalism with 40 years of theoretical foundation and direct NASA flight heritage.

---

## 2. What Aerospace Actually Uses

### 2.1 NASA JPL / Mars Perseverance

Steve Scandore, Mars 2020 FSW technical lead, confirmed in his Embedded Online Conference 2021 keynote that the Perseverance rover's surface component uses approximately 40 hierarchical state machines based on Miro Samek's QEP event processor. Samek clarified that JPL uses specifically the QEP component (the dispatch engine), not the full QP framework. JPL already had their own event-driven architecture on VxWorks dating back to Pathfinder (1997); QEP slotted in as a better state machine engine within that existing framework.

Each of the ~40 HSMs manages an independent subsystem: mobility (wheel steering, drive sequencing), robotic arm (joint coordination, sample handling), power management, thermal control, communications scheduling, instrument sequencing, etc. They communicate via events, not shared state.

JPL also developed the Statechart Autocoder (SCA), which takes UML statechart models and generates C++ (targeting QP's framework), Python (for interactive simulation), and Promela (for SPIN model-checking). SCA has been used on Mars Reconnaissance Orbiter's Electra radio, Mars Science Laboratory's landing radar, and the Space Interferometry Mission. The newer STARS (State Autocoding for Real-time Systems) tool generates code for both the Quantum Framework and NASA's F Prime framework.

### 2.2 Other Aerospace Systems

| System | Formalism | Actions Location | Hierarchical | Framework |
|---|---|---|---|---|
| NASA Perseverance | UML statecharts | Entry/exit + transitions | Yes (~40 HSMs) | QEP (from QP) |
| JPL Statechart Autocoder | UML statecharts | Entry/exit + transitions | Yes | QP + Promela/SPIN |
| ArduPilot | Informal Moore-like | Entry (`init`) / exit (`exit`) | No | Hand-coded C++ |
| PX4 Autopilot | Table-driven Mealy | On transitions | No | Boolean table + guards |
| ESA TASTE/OpenGEODE | SDL (Mealy-like) | On transitions | Limited | SDL code generation |
| NASA cFS | None prescribed | App-dependent | N/A | Message-bus framework |
| HPR flight computers | Switch-case pseudo-Moore | Hybrid, informal | No | None |

### 2.3 Industry Tooling Convergence

All major aerospace state machine tools implement UML statecharts:

- **SCADE Suite** (Ansys/Esterel): Qualified to DO-178C DAL A via DO-330 TQL-1. Used in Airbus A380, Boeing 787, and Eurocopter flight control software.
- **MATLAB Stateflow**: Implements UML/Harel statecharts. Used by JPL for Deep Space 1 and Deep Impact fault protection code generation.
- **QM (Quantum Modeler)**: Free, cross-platform UML statechart editor generating QP/C code. Used with JPL's STARS autocoder.
- **SPIN model checker**: Verifies Promela models (extended Mealy semantics) generated from UML statechart tools.

No aerospace standard (DO-178C, NASA-STD-8739.8, ECSS-E-ST-40C) mandates a specific formalism, but the tooling ecosystem converges on UML statecharts as the implementation target.

---

## 3. The Three Formalisms Compared

### 3.1 Pure Moore Machine

**Actions on state entry/exit only.** Output depends solely on current state.

**Strengths:**
- Completeness guarantee: you cannot enter a state without executing its entry actions. No developer can accidentally create a transition that bypasses the action.
- Simple to reason about: "What does this state do?" is answered by looking at one place.
- Natural mapping to hardware modes (LED patterns, logging rates, telemetry modes).

**Weaknesses:**
- Source-dependent actions require state splitting. If ABORT needs different actions from BOOST vs COAST, you need ABORT_FROM_BOOST and ABORT_FROM_COAST — state explosion.
- Irreversible actions (pyro firing) as entry actions are dangerous: any path into the state fires the action, including error recovery paths, manual overrides, or test modes added later by developers who don't know about the implicit pyro fire.

### 3.2 Pure Mealy Machine

**Actions on transitions only.** Output depends on current state AND the triggering event/input.

**Strengths:**
- Source-dependent actions are natural: COAST→DESCENT fires drogue, MANUAL_OVERRIDE→DESCENT does not.
- Irreversible actions are explicitly gated to specific paths.
- Verification tools (SPIN, TuLiP) work natively on Mealy semantics.

**Weaknesses:**
- No guaranteed initialization: if a new transition into a state is added, the developer must remember to include all necessary actions. Forgotten actions are silent bugs.
- Common state setup (LED pattern, telemetry mode) must be duplicated on every incoming transition.
- "What does this state do?" requires examining every transition that enters it.

### 3.3 UML Statecharts (Hybrid)

**Entry/exit actions AND transition actions.** Three distinct action locations with well-defined execution ordering.

**Execution order on transition from state A to state B:**
1. Exit actions of A execute (and ancestors up to LCA)
2. Transition action executes
3. Entry actions of B execute (and ancestors down from LCA)

This ordering is guaranteed by the UML specification and enforced by QEP's dispatch algorithm.

**Semantic mapping:**
- **Entry actions** = state-invariant initialization (constructors). Execute regardless of source. Use for: LED patterns, telemetry mode, logging rate, sensor polling configuration, pyro channel arming/safing.
- **Exit actions** = guaranteed cleanup (destructors). Use for: safe outputs, stop timers, flush buffers.
- **Transition actions** = source-dependent, path-specific operations. Execute between exit and entry. Use for: irreversible actions (pyro firing), source-specific abort sequences, conditional initialization.

**Added capability — Hierarchical nesting (behavioral inheritance):**
- Common behavior defined in superstates, inherited by all substates
- A single abort transition on `state_flight` superstate replaces N separate transitions
- Entry/exit actions on superstates guarantee invariants regardless of transition path
- Eliminates state explosion from cross-cutting concerns

---

## 4. Why Hybrid Solves Both RocketChip Design Problems

### 4.1 The Drogue Deploy Problem

Drogue should fire when transitioning from COAST to DESCENT at apogee, but NOT when entering DESCENT from a manual override or error recovery path.

- **Pure Moore:** DESCENT entry action fires drogue regardless of source → dangerous if reached via non-apogee path.
- **Pure Mealy:** Drogue fire on COAST→DESCENT transition → correct, but common DESCENT setup (start descent telemetry, update phase counter) must be duplicated on every incoming transition.
- **UML statechart:** `fire_drogue_pyro()` as transition action on COAST→DESCENT. Common DESCENT setup as DESCENT entry action. Clean, explicit, safe.

### 4.2 The Abort Problem

ABORT needs different actions depending on source phase (safe pyros during BOOST, fire drogue during post-apogee COAST, continue logging during DESCENT).

- **Pure Moore:** Split into ABORT_FROM_BOOST, ABORT_FROM_COAST, etc. → state explosion, even though post-entry behavior is identical.
- **Pure Mealy:** Duplicate common ABORT initialization (safe all channels, switch to emergency telemetry) on every incoming transition.
- **UML statechart:** Common abort setup in ABORT entry action. Source-specific actions on each transition. Or: ABORT as composite state with sub-phases if behavior diverges significantly.

### 4.3 Apogee Is an Event, Not a State

**Critical design clarification from council review:** Apogee is a detection event (EVT_APOGEE, velocity zero-crossing), not a flight phase. The vehicle does not "be in apogee" — it passes through apogee as an instantaneous condition. What happens *at* apogee is a transition action (fire drogue pyro). What happens *after* apogee is a state (DESCENT).

The earlier FLIGHT_DIRECTOR_DESIGN.md treatment of APOGEE as a persistent phase with a 5-second timeout was a workaround for placing it in the wrong category. The correct model:

```
COAST → (EVT_APOGEE detected, transition action: fire drogue) → DESCENT.DROGUE
```

**General principle:** If something has no meaningful dwell time and no unique sustained behavior, it's an event, not a state. A state should represent a flight regime where the system behaves differently for a sustained period (different Q/R matrices, different guard conditions, different telemetry rates).

---

## 5. Safety Architecture: Transition-Gated Irreversible Actions

**This is a safety rationale, not just a design preference.** It must be preserved by future developers.

Irreversible actions (pyrotechnic firing) must be transition actions, not entry actions. Rationale:

1. **Explicit gating:** The action fires only from the intended transition path. An unexpected recovery, error handling, or test path that reaches the target state cannot inadvertently trigger the pyro.

2. **Defense in depth alignment:** The pyro fire path is: transition action → action executor → confidence gate (IVP-73) → GPIO. Three layers of defense. Each is independently auditable.

3. **Verification compatibility:** SPIN can verify "pyro never fires in IDLE" by checking that no transition into any state from IDLE carries a pyro action. This property is structural (statically verifiable from the transition table) rather than behavioral (requiring simulation of all possible execution paths).

4. **NASA alignment:** NASA-STD-8739.8 Section 3.7.3 (SWE-134) requires that safety-critical software rejects out-of-sequence commands and safely transitions between all predefined states. Transition-gated pyro actions make out-of-sequence firing structurally impossible rather than relying on runtime guards alone.

Reversible actions (LED patterns, telemetry mode, logging rate) should be entry/exit actions — the completeness guarantee ensures they're never missed.

---

## 6. Corrected Flight Phase Topology

Based on the apogee reclassification and council review:

### Primary Flight Phases (Top-Level)

```
IDLE → ARMED → BOOST → COAST → DESCENT → LANDED
                                              ↓
Any flight phase → ABORT                   (reset) → IDLE
```

Six flight phases plus ABORT. Seven states total.

### HSM Hierarchy

```
state_top                      (root — handles unhandled events)
├── phase_idle                 (ground, pre-arm)
├── phase_armed                (ground, armed, awaiting launch)
├── state_flight               (superstate — common abort handling)
│   ├── phase_boost            (powered ascent)
│   ├── phase_coast            (unpowered ascent, apogee detection active)
│   └── state_descent          (superstate — recovery phase)
│       ├── phase_drogue       (drogue descent, awaiting main deploy altitude)
│       └── phase_main         (main chute descent)
├── phase_landed               (post-flight, data preservation)
└── phase_abort                (safe state, profile-defined actions)
```

### Key Transitions and Action Types

| Transition | Event | Transition Action (Mealy) | Target Entry Action (Moore) |
|---|---|---|---|
| IDLE → ARMED | EVT_ARMED (pre-arm passed) | — | LED solid amber, start pre-buffer |
| ARMED → BOOST | EVT_LAUNCH | — | Start logging, mark LAUNCH, LED red flash |
| BOOST → COAST | EVT_BURNOUT | — | Mark BURNOUT |
| COAST → DESCENT.DROGUE | EVT_APOGEE | **fire_drogue_pyro()** | Mark APOGEE, DESCENT entry actions |
| DESCENT.DROGUE → DESCENT.MAIN | EVT_MAIN_DEPLOY | **fire_main_pyro()** | Mark MAIN |
| DESCENT.MAIN → LANDED | EVT_LANDING | — | Stop logging, LED green, beacon |
| state_flight → ABORT | EVT_ABORT | Source-phase-specific actions | Safe all outputs, beacon |

Pyro fires are transition actions (bold). All other actions are entry/exit actions.

---

## 7. Framework Selection: QP/C

### 7.1 QEP — The Dispatch Engine

QEP (Quantum Event Processor) is the core state machine dispatch algorithm: ~200 lines of C implementing UML statechart semantics. It handles:
- Hierarchical dispatch (event bubbles up to parent superstates if unhandled)
- LCA (Least Common Ancestor) calculation for transitions
- Guaranteed exit-before-entry action ordering
- Entry/exit actions on superstates during hierarchical transitions

QEP is what JPL uses on Perseverance. It has no dependencies on QF, QV, or any kernel. It's pure algorithmic code that compiles on any C99 target.

### 7.2 QF+QV — The Active Object Framework (Target Architecture)

QF (Quantum Framework) provides Active Objects: independent subsystems with private event queues, communicating exclusively via typed events. QV (cooperative kernel) runs Active Objects in a loop functionally identical to a superloop, checking event queues in priority order.

**QF does not require an RTOS.** QV is a built-in cooperative kernel that replaces the superloop. QP/C can run standalone on bare-metal microcontrollers.

Target Active Objects for RocketChip:
- `AO_FlightDirector` — flight state machine, condition evaluator, command handler
- `AO_LedEngine` — NeoPixel ownership, pattern management, transition animations
- `AO_ErrorHandler` — subsystem health monitoring, error/warning event publishing
- `AO_Logger` — flash write management, flight event recording
- `AO_Telemetry` — downlink scheduling, packet encoding

Each Active Object owns its state exclusively. Inter-module communication is via typed events through QF's publish-subscribe mechanism. This eliminates the class of bugs where one module corrupts another's state (e.g., the LED being set to an unexpected color by scattered code).

### 7.3 QS (QP/Spy) — Binary Tracing

QS logs every state entry, exit, transition, and event dispatch to a ring buffer with ~10x less overhead than printf-style logging. Provides complete state machine replay for post-flight analysis. Evaluated separately from QEP/QF adoption.

### 7.4 Adoption Roadmap

1. **Stage 8:** Flight Director uses QEP, called from existing superloop. State handlers written following QEP conventions. Crowdfunding demo milestone.
2. **Toolchain validation (IVP within Stage 8):** Compile QP/C (QEP+QF+QV) against Pico SDK. Run trivial two-Active-Object demo. Confirm upgrade path is viable.
3. **Dedicated refactoring stage (post-Stage 8):** Migrate subsystems to Active Objects. LED Engine first (most obvious beneficiary). Superloop progressively replaced by QV. Core 1 AMP boundary unchanged — QF+QV stays entirely on Core 0.
4. **State handler code is identical across steps 1-3.** Zero throwaway code.

### 7.5 Resource Footprint

QP/C requires approximately 1 KB RAM and 10 KB flash. Against the RP2350's 520 KB SRAM and 8 MB flash, this is <0.2% of available RAM. Core 0 currently runs at ~15% utilization. QEP dispatch is <5µs per event at 150 MHz.

### 7.6 Licensing

QP/C is dual-licensed: GPLv3 (open source) or commercial. RocketChip is GPL-3.0-or-later — license compatible.

---

## 8. STARS Autocoder Toolchain

### 8.1 Workflow

Draw the Flight Director statechart in QM (Quantum Modeler, free, cross-platform) → STARS generates C++ (QEP dispatch code) + Promela (SPIN verification) + Python (interactive simulator) → SPIN checks safety properties → generated C++ compiles into RocketChip firmware.

The `.qm` file is the authoritative source of truth for the state machine design. The generated code is a build artifact. The statechart model IS the design document.

### 8.2 Verification Pipeline

From a single QM model, STARS generates three outputs:
- **C++ (QEP target):** Flight code for RP2350
- **Promela:** Input for SPIN model checker. Verify properties like "pyro never fires in IDLE," "every path from ARMED eventually reaches LANDED or ABORT," "no two pyro channels fire within 100ms of each other."
- **Python simulator:** Interactive desktop testing — click buttons to send events, observe state transitions, validate guard logic before hardware.

This single-source-of-truth approach prevents the code-model desync problem: the C++ code and the Promela verification model are generated from the same source and cannot diverge.

### 8.3 Availability

- **STARS:** `github.com/JPLOpenSource/STARS` (open source)
- **SCA Rev. 2:** `github.com/JPLOpenSource/SCA` (open source, older)
- **QM:** `state-machine.com` (free, cross-platform, Windows/Linux/Mac)
- **SPIN:** `spinroot.com` (open source)

STARS generates code for both the Quantum Framework (QP/C) and NASA's F Prime framework — enabling the same statechart model to target both the RP2350 flight computer and a Pi-based ground station running F Prime.

### 8.4 Integration with Existing Tools

`tools/state_to_dot.py` (Graphviz visualization) remains useful as a read-back verification tool: parse the generated C++ and produce a topology visualization to confirm it matches the QM model. It serves as an independent check, not a design input.

---

## 9. References

### Primary

- Samek, M. *Practical UML Statecharts in C/C++: Event-Driven Programming for Embedded Systems*, 2nd ed. CRC Press, 2008. — Canonical reference for QEP dispatch algorithm, UML statechart implementation patterns, Active Object model.
- Scandore, S. "Mars Perseverance Software," Embedded Online Conference 2021 (keynote + Q&A). — Confirms QEP usage on Perseverance, ~40 HSMs in surface component.
- Harel, D. "Statecharts: A Visual Formalism for Complex Systems," *Science of Computer Programming*, 8(3):231–274, 1987. — Original statechart formalism.

### Tools and Frameworks

- QP/C framework: `github.com/QuantumLeaps/qpc` (GPLv3 / commercial dual-license)
- QP/C++ framework: `github.com/QuantumLeaps/qpcpp`
- JPL STARS autocoder: `github.com/JPLOpenSource/STARS`
- JPL SCA Rev. 2: `github.com/JPLOpenSource/SCA`
- QM (Quantum Modeler): `state-machine.com/#Downloads` (free, cross-platform)
- SPIN model checker: `spinroot.com`

### Aerospace References

- Benowitz, E. et al. "Auto-coding UML Statecharts for Flight Software," 2nd IEEE SMC-IT, 2006. — JPL SCA development and deployment on SIM, MRO Electra, MSL landing radar.
- Wagstaff, K. et al. "Automatic Code Generation for Instrument Flight Software," — MRO Electra statechart autocoding experience.
- Cummings, D. "Managing Concurrency in Complex Embedded Systems," — JPL event-driven architecture used from Pathfinder through Perseverance.
- NASA-STD-8739.8 Section 3.7.3 (SWE-134) — Safety-critical state transition requirements.

### Existing RocketChip References (Updated)

- `docs/flight_director/RESEARCH.md` — Prior art survey (predecessor to this document)
- `docs/SAD.md` Section 6 — State machine topology (to be updated with this resolution)
- `docs/IVP.md` Stage 8 — Implementation steps (to be restructured per council decision)

---

*This document replaces `docs/flight_director/RESEARCH.md` "Remaining Design Questions" Section 4 (Mealy vs Moore) and resolves SAD Open Question #4.*
