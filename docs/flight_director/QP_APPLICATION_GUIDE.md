# QP/C Application Guide for RocketChip

**For:** RocketChip (RP2350 bare-metal Pico SDK, dual-core AMP)
**Date:** 2026-03-10
**Status:** PRELIMINARY — Subject to toolchain validation IVP results
**Prerequisites:** Council review `council_state_machine_formalism.md`, Research `STATE_MACHINE_FORMALISM_RESEARCH.md`

> This document covers RocketChip-specific integration concerns. For general QP/C documentation, see `state-machine.com/qpc/` and Samek's *Practical UML Statecharts in C/C++* (2nd ed, 2008).

---

## 1. What QP/C Is and What We Use

QP/C is a layered framework. Each layer is independently usable:

```
┌─────────────────────────────────────────────────┐
│  Application Active Objects (AO_FlightDirector,  │
│  AO_LedEngine, AO_Logger, ...)                  │
├─────────────────────────────────────────────────┤
│  QEP — Hierarchical State Machine Dispatcher     │  ← Stage 8 (immediate)
│  UML statechart semantics, LCA transitions       │
├─────────────────────────────────────────────────┤
│  QF — Active Object Framework                    │  ← Post-Stage-8 refactoring
│  Event queues, publish-subscribe, time events    │
├─────────────────────────────────────────────────┤
│  QV — Cooperative Kernel                         │  ← Post-Stage-8 refactoring
│  Priority-based round-robin, replaces superloop  │
├─────────────────────────────────────────────────┤
│  QS — Software Tracing (QP/Spy)                  │  ← Evaluate during refactoring
│  Binary event/state trace, ring buffer, HDLC     │
├─────────────────────────────────────────────────┤
│  BSP — Board Support Package                     │
│  Pico SDK, board.h, mission.h                    │
└─────────────────────────────────────────────────┘
```

**Stage 8 uses QEP only.** The Flight Director is a QEP state machine called from the existing Core 0 superloop. All other layers are adopted incrementally in a dedicated post-Stage-8 refactoring stage.

---

## 2. Dual-Core AMP Constraint

**QP/C operates exclusively on Core 0.**

```
Core 1 (deterministic sensor sampling)          Core 0 (application)
┌──────────────────────────┐    seqlock    ┌──────────────────────────┐
│ IMU read (1kHz)          │──────────────▶│ QEP/QF/QV               │
│ Baro read (64Hz)         │               │   AO_FlightDirector     │
│ GPS read (10Hz)          │               │   AO_LedEngine          │
│ Mag read (100Hz)         │               │   AO_Logger             │
│                          │               │   AO_Telemetry          │
│ No QP code on this core  │               │   AO_ErrorHandler       │
│ No event queues           │               │                          │
│ No Active Objects         │               │ Condition evaluator      │
│ Pure bare-metal loop      │               │   reads FusedState via   │
└──────────────────────────┘               │   seqlock (read-only)    │
                                            └──────────────────────────┘
```

The seqlock boundary between cores is unchanged. QF event queues, QV scheduling, and Active Object state are all Core 0 constructs. Core 1 remains a pure bare-metal deterministic loop with no QP dependencies.

The condition evaluator (part of AO_FlightDirector) reads FusedState from the seqlock — this is a read-only operation from Core 0's perspective, identical to how it works today.

---

## 3. QEP Integration (Stage 8)

### 3.1 How QEP State Handlers Work

Each flight phase is a C function that receives an event and returns a disposition:

```cpp
// State handler signature (QEP convention)
QState phase_coast(FlightDirector* const me, QEvt const* const e) {
    switch (e->sig) {
    case Q_ENTRY_SIG:
        // Moore-style: executes on ANY entry to COAST
        mark_event(EVT_BURNOUT);
        return Q_HANDLED();

    case Q_EXIT_SIG:
        // Cleanup before leaving COAST
        return Q_HANDLED();

    case SIG_APOGEE:
        // Mealy-style: transition action specific to COAST→DESCENT
        fire_drogue_pyro();   // IRREVERSIBLE — transition-gated (safety)
        return Q_TRAN(&phase_drogue);

    case SIG_ABORT:
        // Handled here OR deferred to superstate
        return Q_SUPER(&state_flight);  // defer to parent

    case SIG_TICK:
        // Condition evaluator runs here (100Hz)
        evaluate_guards(me);
        return Q_HANDLED();

    default:
        return Q_SUPER(&state_flight);  // unhandled → parent superstate
    }
}
```

Key macros:
- `Q_HANDLED()` — event consumed by this handler
- `Q_TRAN(&target)` — trigger transition to target state (QEP handles exit/entry ordering)
- `Q_SUPER(&parent)` — defer to parent superstate (behavioral inheritance)

### 3.2 Superstate for Common Behavior

```cpp
// state_flight handles abort for ALL flight phases
QState state_flight(FlightDirector* const me, QEvt const* const e) {
    switch (e->sig) {
    case Q_ENTRY_SIG:
        // Common flight entry: start high-rate logging
        start_flight_logging();
        return Q_HANDLED();

    case Q_EXIT_SIG:
        // Common flight exit: ensure outputs safe
        safe_all_outputs();
        return Q_HANDLED();

    case SIG_ABORT: {
        // Abort handling — source-phase-specific transition actions
        // determined by me->current_phase before transition
        execute_abort_actions(me->flight_state.current_phase);
        return Q_TRAN(&phase_abort);
    }

    default:
        return Q_SUPER(&QHsm_top);  // root
    }
}
```

With this superstate, BOOST, COAST, and DESCENT all inherit abort handling without duplicating it. `SIG_ABORT` in `phase_coast` defers via `Q_SUPER(&state_flight)`, which handles it.

### 3.3 Superloop Integration (Stage 8)

During Stage 8, the Flight Director is a QEP state machine called from the existing superloop:

```cpp
// In main.cpp — unchanged superloop structure
static FlightDirector g_director;

void init_application() {
    // QEP initialization
    FlightDirector_ctor(&g_director);
    QHSM_INIT(&g_director.super, (void*)0);  // initial transition → phase_idle
    g_director.profile = &kRocketProfile;
}

void run_flight_director() {
    // Generate tick event
    static QEvt const tick_evt = { SIG_TICK };
    QHSM_DISPATCH(&g_director.super, &tick_evt);

    // Process any pending commands (CLI, button)
    if (command_pending()) {
        QEvt cmd_evt;
        cmd_evt.sig = get_pending_command_signal();
        QHSM_DISPATCH(&g_director.super, &cmd_evt);
    }
}

// Core 0 superloop — unchanged
while (true) {
    if (fusion_due)    run_fusion();
    if (director_due)  run_flight_director();   // dispatches QEP events
    if (logger_due)    run_logger();
    if (telem_due)     run_telemetry();
    if (ui_due)        run_ui();
}
```

This is functionally identical to the current architecture. The only change is that `run_flight_director()` dispatches events through QEP instead of using a hand-rolled switch-case. All other modules remain unchanged.

### 3.4 Build System Integration

QEP is a small number of C source files from the QP/C repository. Integration approach: **vendor a specific QEP version** into `lib/qep/` (same pattern as ArduPilot math library extraction).

```
lib/
├── ardupilot/          # existing sparse-checkout submodule
└── qep/                # vendored QEP files
    ├── LICENSE.txt      # GPLv3 (compatible with project license)
    ├── qep_hsm.c        # HSM dispatch engine (~200 lines)
    ├── qep_msm.c        # MSM dispatch (optional, simpler variant)
    ├── qp.h             # Main header
    ├── qp_port.h        # RocketChip-specific port configuration
    └── qp_config.h      # Feature selection (QEP only, no QF/QV/QS)
```

CMake integration:

```cmake
# In CMakeLists.txt
add_library(qep STATIC
    lib/qep/qep_hsm.c
)
target_include_directories(qep PUBLIC lib/qep)
target_link_libraries(rocketchip PRIVATE qep)
```

### 3.5 Port Configuration

`qp_port.h` for RP2350 bare-metal Pico SDK:

```cpp
#ifndef QP_PORT_H
#define QP_PORT_H

#include <stdint.h>
#include "pico/critical_section.h"

// QEP-only configuration — no QF, QV, QK, QS
#define Q_NASSERT           // Use project's own assert (JSF compliance)

// Critical section using Pico SDK primitives
// (Only needed if events can be generated from interrupts)
#define QF_CRIT_STAT_       uint32_t status_
#define QF_CRIT_ENTRY_()    status_ = save_and_disable_interrupts()
#define QF_CRIT_EXIT_()     restore_interrupts(status_)

// Include QEP public interface
#include "qep.h"

#endif // QP_PORT_H
```

---

## 4. Migration to QF+QV (Post-Stage-8 Refactoring Stage)

### 4.1 Why Migrate

The superloop has a fundamental limitation: **any module can reach into any other module's state.** This manifests as:

- LED behavior sprinkled across multiple files (the "why is it solid green" problem)
- Error handling logic scattered rather than centralized
- No formal interface between modules — just function calls and shared globals
- No audit trail for inter-module communication

QF Active Objects solve this by enforcing encapsulation through event queues. Each subsystem owns its state privately. The only way to affect it is to send a typed event.

### 4.2 What Changes, What Doesn't

**Unchanged:**
- Core 1 sensor sampling loop (no QP involvement)
- Seqlock boundary between cores
- State handler code inside Active Objects (identical to Stage 8 QEP handlers)
- ESKF, sensor drivers, calibration — these are computational modules, not event-driven actors

**Changed:**
- Superloop `_due` flags and tick dividers → QF time events (periodic timers)
- Direct function calls between modules → typed events through QF publish-subscribe
- Shared globals for inter-module state → encapsulated within Active Objects
- Manual event dispatch in `run_flight_director()` → QF event queue dispatch

### 4.3 Incremental Migration Path

The superloop shrinks as modules migrate. Each step is independently testable:

**Step 1: Flight Director as Active Object**

Wrap the existing QEP state machine in a QF Active Object. The state handlers are unchanged. What changes is how events arrive — from QF's queue instead of manual dispatch in `run_flight_director()`.

```cpp
// Before (Stage 8 superloop):
void run_flight_director() {
    QEvt tick = { SIG_TICK };
    QHSM_DISPATCH(&g_director.super, &tick);
}

// After (QF Active Object):
// QV calls this automatically based on priority and queue status.
// The state handler code inside is IDENTICAL.
```

**Step 2: LED Engine as Active Object**

New Active Object with its own QEP state machine managing NeoPixel patterns. Receives pattern-change events from other Active Objects. Nobody else touches the NeoPixel.

```
AO_FlightDirector                    AO_LedEngine
    │                                     │
    │──── EVT_LED_PATTERN(RED_FLASH) ────▶│
    │                                     │── owns NeoPixel hardware
    │                                     │── manages transitions
    │                                     │── handles blinking/breathing
```

**Step 3: Additional Active Objects**

- `AO_Logger` — owns flash writes, receives log events
- `AO_Telemetry` — owns radio TX scheduling, receives telemetry events
- `AO_ErrorHandler` — monitors subsystem health, publishes alerts

**Step 4: Superloop → QV**

Once all modules are Active Objects, the superloop IS QV:

```cpp
int main() {
    // Initialize QF
    QF_init();

    // Create Active Objects with priorities
    FlightDirector_ctor(&AO_FlightDirector);
    LedEngine_ctor(&AO_LedEngine);
    Logger_ctor(&AO_Logger);
    // ...

    // Start Active Objects (assigns queues and priorities)
    QACTIVE_START(AO_FlightDirector, 5, ...);
    QACTIVE_START(AO_LedEngine, 2, ...);
    QACTIVE_START(AO_Logger, 3, ...);
    // ...

    // QF_run() replaces the superloop — never returns
    // Internally: cooperative round-robin checking queues in priority order
    return QF_run();
}
```

QV's cooperative scheduling is functionally identical to the superloop — it checks each Active Object's queue in priority order, dispatches one event at a time (run-to-completion), and idles (WFI) when all queues are empty.

### 4.4 Perseverance's ~40 HSMs as Reference

Each of Perseverance's ~40 HSMs manages an independent subsystem — mobility, robotic arm, power, thermal, comms, instrument sequencing, etc. They communicate via events, not shared state. RocketChip's Active Objects follow the same pattern at smaller scale:

| Perseverance Subsystem | RocketChip Equivalent |
|---|---|
| Mobility control | AO_FlightDirector |
| Power management | (future: AO_PowerMonitor) |
| Thermal control | (not needed for current hardware) |
| Communications scheduling | AO_Telemetry |
| Instrument sequencing | AO_Logger (flight data recording) |
| Status/fault management | AO_ErrorHandler |
| LED/indicator management | AO_LedEngine |

---

## 5. STARS Autocoder Workflow

### 5.1 Overview

```
┌──────────┐     ┌─────────┐     ┌──────────────────────────────┐
│  QM       │────▶│  STARS   │────▶│  C++ (QEP state handlers)    │──▶ Pico SDK build
│  (model)  │     │  (Java)  │     │  Promela (SPIN verification)  │──▶ SPIN
│  .qm file │     │          │     │  Python (interactive sim)     │──▶ Desktop testing
└──────────┘     └─────────┘     └──────────────────────────────┘
```

### 5.2 Authoring in QM

QM (Quantum Modeler) is a free, cross-platform graphical editor for UML statecharts. It saves models as `.qm` XML files. The Flight Director statechart is drawn in QM — states, transitions, guards, entry/exit actions, transition actions, hierarchy.

The `.qm` file is the **authoritative source of truth** for the state machine design. It is committed to the repo alongside the generated code.

### 5.3 Generated Code

STARS generates state handler functions that follow the exact QEP conventions described in Section 3. The generated code is a build artifact — it should not be hand-edited. If a change is needed, it goes back through the QM model.

### 5.4 Verification with SPIN

SPIN checks safety properties expressed as LTL (Linear Temporal Logic) formulas:

- `[] !(phase == IDLE && pyro_fired)` — "Pyro never fires in IDLE"
- `[] (phase == ARMED -> <> (phase == LANDED || phase == ABORT || phase == IDLE))` — "ARMED eventually resolves"
- `[] (abort_commanded -> <> phase == ABORT)` — "Abort command always reaches ABORT state"
- `[] !(pyro1_fired && pyro2_fired && elapsed < MIN_PYRO_SPACING)` — "Pyro channels respect spacing"

These properties are checked exhaustively against the Promela model generated from the same QM source as the flight code. If the model passes SPIN verification, the generated flight code has the same properties (assuming correct code generation, which is validated by the STARS test suite).

### 5.5 Python Interactive Simulator

STARS generates a Python GUI that mirrors the statechart. Click buttons to send events, observe state transitions, validate guard logic. Useful for:

- Design reviews (demonstrate state machine behavior to non-developers)
- Desktop testing before hardware integration
- Regression testing (scripted event sequences)

### 5.6 Repository Layout

```
tools/
├── qm/
│   └── flight_director.qm    # Authoritative statechart model
├── stars/
│   └── (STARS autocoder JAR and config)
├── spin/
│   ├── flight_director.pml    # Generated Promela model
│   └── safety_properties.ltl  # LTL safety formulas
├── sim/
│   └── flight_director_sim.py # Generated Python simulator
└── state_to_dot.py            # Read-back visualization (existing)

src/flight_director/
├── flight_director.cpp        # Generated by STARS from QM model
├── flight_director.h          # Generated by STARS
├── flight_director_actions.cpp # Hand-written action implementations
└── flight_director_guards.cpp  # Hand-written guard functions
```

Note: STARS generates the state machine skeleton (dispatch, transitions, entry/exit/transition action calls). The action implementations (what `fire_drogue_pyro()` actually does) and guard function bodies (what `guard_accel_body_z()` reads and compares) are hand-written in separate files that the generated code calls into. This separation means the generated code handles control flow while hand-written code handles hardware interaction.

---

## 6. Key Differences from General QP/C Documentation

The QP/C documentation and Samek's book assume a standard QP application. RocketChip has specific constraints:

### 6.1 No Dynamic Memory Allocation

JSF AV C++ Rules (Rev. C) prohibit dynamic allocation in flight-critical paths. QP/C's default event allocation uses a pool allocator. For Stage 8 (QEP-only), events are stack-allocated and dispatched synchronously — no allocator needed. For QF migration, configure QF's event pools as static arrays with compile-time-determined sizes.

### 6.2 No Blocking Anywhere

The existing architecture (and the abandoned FreeRTOS experience) established that blocking is not acceptable. QV's cooperative kernel is non-blocking by design — Active Objects run to completion and return control to the scheduler. This is compatible. QXK (the blocking dual-mode kernel) should NOT be used.

### 6.3 C++20, Not C99

QP/C is written in C99. RocketChip uses C++20. The QEP files compile cleanly as C++ (they use `extern "C"` linkage). State handler functions can be C++ member functions or free functions with C linkage. The generated STARS code targets C++ output.

### 6.4 Existing Assert/Panic Infrastructure

QP/C has its own `Q_ASSERT` mechanism. RocketChip has `ROCKETCHIP_ASSERT` and the panic handler from IVP-30. Define `Q_NASSERT` to disable QP's asserts and use the project's own infrastructure, or map `Q_onAssert` to the existing panic handler.

---

## 7. Evaluation Checklist for Toolchain Validation IVP

The toolchain validation IVP gates the STARS adoption decision. Complete these checks in a single session:

- [ ] QP/C (qpc repo) clones and compiles against Pico SDK ARM Cortex-M33 target
- [ ] Trivial QEP state machine (3 states, 2 transitions) links and runs on RP2350 hardware
- [ ] QM installs and opens on development machine
- [ ] STARS autocoder JAR runs (requires Java — confirm version)
- [ ] STARS generates C++ from a trivial QM model
- [ ] Generated C++ compiles and links against Pico SDK
- [ ] STARS generates Promela from same model
- [ ] SPIN installs and verifies a trivial safety property against generated Promela
- [ ] STARS generates Python simulator from same model
- [ ] Python simulator runs and responds to simulated events

**Gate decision:** If all checks pass in one session → STARS is primary authoring tool. If Java version conflicts, SPIN compilation issues, or generated code incompatibilities arise → fall back to hand-authored QEP, revisit STARS later.

---

## 8. References

- Samek, M. *Practical UML Statecharts in C/C++*, 2nd ed. CRC Press, 2008.
- QP/C documentation: `state-machine.com/qpc/`
- QP/C source: `github.com/QuantumLeaps/qpc`
- QP/C ARM Cortex-M port: `state-machine.com/qpc/arm-cm.html`
- QV cooperative kernel: `state-machine.com/qpc/arm-cm_qv.html`
- QM modeling tool: `state-machine.com/#Downloads`
- STARS autocoder: `github.com/JPLOpenSource/STARS`
- SPIN model checker: `spinroot.com`
- QP/Spy tracing: `state-machine.com/qtools/qs.html`
- QP support forum (superloop integration): `sourceforge.net/p/qpc/discussion/668726/thread/6c9aa11867/`

---

*Application guide for: `docs/flight_director/QP_APPLICATION_GUIDE.md`*
