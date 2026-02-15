# Mission Engine Research: Profiles vs. State Machine

**For:** RocketChip (RP2350 bare-metal Pico SDK, dual-core AMP)
**Date:** 2026-02-13

---

## The Two Systems

RocketChip's Mission Engine is really two interacting but distinct subsystems:

**Mission Profiles** — the configurable "what." A profile declares: which flight phases exist, what transitions connect them, what sensor thresholds trigger transitions, what actions fire on entry/exit, which pyro channels map where, and what gets logged. A rocket profile is different from a balloon profile is different from a vehicle profile. Profiles are data structures — for Phase 5, a single rocket profile baked into firmware as a `const` struct; later, loadable from flash/SD and swappable before arming.

**State Machine** — the runtime "how." A vehicle-agnostic engine that takes a profile as input and executes it. It tracks the current phase, evaluates sensor data against the profile's guard conditions, fires transitions, runs entry/exit actions, and handles aborts. The state machine itself doesn't know what a "rocket" is — it just processes states, events, and transitions as defined by whatever profile is active.

The interaction: **the state machine's topology is defined by the active profile, but the engine is profile-agnostic.** This is what makes RocketChip a platform rather than a single-purpose flight computer.

### Commands, Events, and States Are Three Distinct Things

This distinction is critical and easy to conflate. ArduPilot muddles it — "mode" means both "what the user requested" and "what the system is doing." PX4 handles it better with its `UserModeIntention → FailsafeStateMachine → ModeManagement → nav_state` pipeline, where user requests flow through validation before becoming system state. RocketChip should be explicit:

**Commands** are inputs — things that *request* something happen. `CMD_ARM`, `CMD_DISARM`, `CMD_RESET`, `CMD_ABORT`. They come from the CLI, a button hold, a GCS message, or internal logic. A command can be **rejected** (you send `CMD_ARM` but pre-arm checks fail, so you stay in IDLE).

**Events** are validated facts — things that *have been confirmed* true. `EVT_ARMED` (pre-arm checks passed), `EVT_LAUNCH` (acceleration threshold sustained), `EVT_APOGEE` (velocity zero-crossing confirmed). Events are only generated after validation succeeds. They are the sole input to the state machine.

**States** are what the system *actually is* right now. IDLE, ARMED, BOOST, COAST, etc. States change only when the state machine processes a validated event. The state is always ground truth.

The flow is: **Command → Validation → Event → State Machine → State Change → Actions**

```
User presses ARM button
  → CMD_ARM received by command handler
  → Pre-arm checks run:
      ✓ Sensors healthy?
      ✓ Calibration valid?
      ✓ Battery OK?
      ✓ GPS lock (if required by profile)?
  → All pass → EVT_ARMED generated → state machine transitions IDLE → ARMED
  → Any fail → command rejected, stay in IDLE, report reason to user

Sensor tick during COAST phase:
  → Condition evaluator reads FusedState
  → Kalman velocity < 0 AND baro rate negative → criteria met
  → EVT_APOGEE generated → state machine transitions COAST → APOGEE
```

The state machine never sees raw commands or raw sensor data. It only sees validated events. This keeps the engine dead simple — it only processes things that have already been confirmed valid. All the messy validation logic (pre-arm checks, multi-method sensor fusion, sustained-threshold timers) lives in the command handler and condition evaluator, completely separate from state transition logic.

**Why this matters:** ARM is both a command and a state, but they're not the same thing functionally. The ARMED state only exists after the checks triggered by the ARM command pass. If you conflate them, you end up with validation logic scattered inside the state machine, making it harder to test, harder to reason about, and harder to configure per-profile (different vehicle types need different pre-arm checks).

---

## What RocketChip Had Before

### Deprecated StateTracking.h (Arduino era)

The old code used a flat enum FSM with six system-level states:

```cpp
enum class SystemState {
  INITIALIZING, CALIBRATING, IDLE, PREFLIGHT, ACTIVE, ERROR
};
```

Key characteristics:
- **No flight phase granularity.** ACTIVE covered everything from launch to landing — there was no BOOST, COAST, APOGEE, DESCENT, or LANDED. The system tracked *whether you were logging*, not *what flight phase you were in*.
- **Bitfield transition validation.** A `validTransitions[]` array used bitmasks to whitelist legal transitions. Invalid transitions forced ERROR state. This is a good pattern worth preserving.
- **Launch detection was separate from state tracking.** `checkAutoLaunch()` and `checkAutoLanding()` lived in the StateTracking class but operated on raw sensor thresholds (`LAUNCH_ACCEL_THRESHOLD = 3.0g`, `LANDING_STILLNESS_TIME = 5000ms`). No sensor fusion, no multi-method detection.
- **No mission profiles.** Configuration was entirely compile-time `#define` in config.h. Changing vehicle type meant changing code.
- **Flight stats tracked alongside state.** `FlightStats` (maxAccel, maxAltitude, flightTime) was embedded in the state tracker — mixing concerns.

The inline `.ino` version was even simpler: a `SET_STATE()` macro with transition validation, global booleans for launch/landing detection, and Madgwick filter for AHRS. No separation of concerns at all.

### What the SAD Currently Specifies (Section 6)

The current architecture document already has a more sophisticated design:

```
IDLE → ARMED → BOOST → COAST → DESCENT → LANDED
```

With event-condition-action definitions:
```ini
launch = "state:ARMED AND accel_z > 2.5 AND sustained:50ms"
         -> set_state:BOOST, start_log, mark:LAUNCH, led:red_flash
```

And a MissionEngine decomposition into four components: StateMachine, EventEngine, ActionExecutor, ConditionEvaluator. Plus planned mission definition files (`Mission_Rocket.cpp`, `Mission_HPR.cpp`, `Mission_Freeform.cpp`).

**The SAD's design is directionally correct but needs refinement on the profile/engine boundary.** The mission definitions as `.cpp` files suggest compile-time selection rather than runtime-configurable data structures. The condition evaluator is noted as deferred. The interaction between profiles and the state machine isn't explicitly specified.

---

## How Other Projects Handle This

### The Profile Side (Configuration / Mission Definition)

**ArduPilot** doesn't have runtime-swappable mission profiles in the RocketChip sense. Instead, it uses **compile-time vehicle selection** (ArduCopter vs ArduPlane vs ArduRover) with runtime **parameter tuning** via 1,500+ `AP_Param` values. Flight modes are hardcoded per vehicle. The closest analog to mission profiles is the `AP_Mission` waypoint system, which loads scripted waypoints from storage — but this is navigation, not state machine topology.

**PX4** takes a more modular approach with its **mode requirements system.** Each flight mode declares what it needs (GPS, attitude estimate, manual control) in a central `mode_requirements.cpp` file. This is closer to a profile concept — the mode's behavior is defined declaratively rather than procedurally. PX4's **FlightTask** abstraction separates setpoint generation from mode logic, enabling new flight behaviors without modifying the core state machine.

**SparkyVT's HPR Flight Computer** supports multiple flight configurations (single-stage, two-stage, airstart, booster) selected via SD card settings file before flight. Each configuration changes the state progression and pyro channel assignments. This is the closest analog to RocketChip's mission profiles in the hobby rocketry space — same firmware binary, different behavior via configuration. 250+ successful flights validate this approach.

**BPS.space Signal** uses SD card configuration files with Bluetooth mobile app backup. Configuration includes pyro assignments, TVC parameters, and flight mode selection. Not a full mission profile system, but demonstrates the UX pattern.

**NASA cFS** uses its **Table Service** for runtime-modifiable configuration without code changes. Tables are validated on load, versioned, and can be updated in flight. The **Stored Command (SC)** app provides Relative/Absolute Time Sequences — essentially scripted mission timelines loaded from tables. This is the gold standard for separating "what to do" from "how to do it."

**Key takeaway for profiles:** The converging pattern across projects is **table-driven configuration loaded from storage**, with validation at load time and no code changes needed between missions. SparkyVT and NASA cFS both prove this works at very different scales.

### The State Machine Side (Runtime Execution Engine)

Three distinct architectural families emerge:

#### 1. Flat Enum FSM (Most hobby projects, old RocketChip)

A single `enum` of states, a `switch` statement in the main loop, and threshold checks for transitions.

**Pros:** Dead simple, easy to understand, easy to debug, minimal code size.
**Cons:** State explosion when adding cross-cutting concerns (abort handling, error recovery, telemetry mode changes). Every new state needs explicit transition logic to/from every other reachable state. No code reuse between similar phases.

SparkyVT uses this and it works — but SparkyVT is rocket-only. For a multi-vehicle platform, flat FSMs don't scale.

#### 2. Polymorphic Mode Classes (ArduPilot)

Each flight mode is a C++ class inheriting from a vehicle-specific `Mode` base. The active mode is tracked via pointer; `set_mode()` validates, calls `exit()` on old mode, `init()` on new mode, updates pointer. Each mode's `run()` is called every loop iteration (400Hz for Copter).

Several modes embed internal sub-FSMs. `ModeRTL` progresses through: `RTL_Starting → RTL_InitialClimb → RTL_ReturnHome → RTL_LoiterAtHome → RTL_FinalDescent → RTL_Land`. This is an ad-hoc two-level hierarchy.

Failsafe operates independently per trigger type (radio, GCS, battery, EKF) with configurable actions for each. Failsafe mode persists even after the condition clears.

**Pros:** Clean per-mode encapsulation, easy to add modes, good for deep specialization per vehicle type.
**Cons:** Each vehicle type has its own mode hierarchy (no sharing), failsafe logic is scattered across multiple independent systems, sub-FSMs are ad-hoc without a unifying framework.

#### 3. Hierarchical State Machines / Active Objects (NASA Perseverance, QP Framework)

Mars Perseverance runs ~40 HSMs based on Miro Samek's QP/QEP event processor. 1.2M lines of flight code on VxWorks/RAD750. No semaphores in the entire codebase — fully event-driven Active Objects communicating exclusively via events.

HSMs solve the state explosion problem through **behavioral inheritance.** Common behaviors defined in superstates are automatically inherited by all substates. A single abort transition on the top-level superstate replaces N separate transitions. Entry/exit actions on superstates guarantee safety invariants regardless of transition path.

The QP framework implements this on bare-metal MCUs with as little as 1KB RAM. QEP (the event processor) needs a single function pointer per state machine in RAM. It has a native FreeRTOS port, but more importantly for RocketChip, **QP/C also runs on bare-metal cooperative schedulers** — you don't need an RTOS.

**Pros:** Behavioral inheritance eliminates duplication, entry/exit actions guarantee invariants, mathematically well-defined (UML statecharts), proven at the highest level (Mars), minimal RAM.
**Cons:** Higher conceptual complexity, harder to debug without tooling, overkill if your state machine is genuinely simple.

#### 4. Distributed / Event-Driven (PX4, PSAS)

PX4 splits its state machine across four cooperating modules communicating via uORB pub-sub: Commander (central state/arming), Navigator (mission execution), FlightModeManager (setpoint generation), and vehicle controllers. The failsafe state machine was completely rewritten (PR #20172) into a clean two-phase pattern: hold for `COM_FAIL_ACT_T` seconds, then execute configured action.

PSAS (Portland State Aerospace Society) built an explicitly cFS-inspired flight computer framework (elderberry) using code-generated event loops. Modules are self-contained units whose relationships are defined in YAML, with a Python code generator producing the dispatch code.

**Pros:** Clean separation of concerns, modules are independently testable, scales to complex systems.
**Cons:** Massive overkill for a single-MCU bare-metal system. The coordination overhead doesn't pay off unless you have multiple processors or need true module isolation.

#### 5. What University Rocketry Teams Do

**Waterloo Rocketry** uses a distributed CAN bus approach where each subsystem board has its own state machine, coordinated via `BUS_DOWN_WARNING` messages. Their canard control processor has a dedicated `flight_phase` module with `get_flight_phase()`. This is relevant if RocketChip ever goes multi-board (Gemini).

Most other university teams (and the RP2040-based Pygmy project) use simple flat FSMs — adequate for single-vehicle, single-mission flight computers.

---

## Sensor-Based Phase Detection (Cross-Cutting Concern)

Regardless of state machine architecture, **how you detect flight phase transitions** is where the real complexity lives. This spans both systems — the profile defines *which* methods and *what* thresholds, the state machine *evaluates* them.

### Detection Methods by Transition

| Transition | Primary | Backup | Gotchas |
|---|---|---|---|
| Pad → Launch | Accel > threshold sustained N ms | Baro altitude increase | Must reject vibration; SparkyVT uses 50ms sustained window |
| Launch → Burnout | Accel drops below 1g | Timer from launch | Motor chuffing can cause false triggers |
| Burnout → Apogee | Kalman velocity crosses zero | Baro rate-of-change sign flip | **Transonic shock** corrupts baro — must de-sense during supersonic flight |
| Apogee → Descent | Pyro continuity change or timer | Altitude decreasing sustained | Dual-method: drogue deploy confirmation + altitude check |
| Descent → Landing | Velocity < threshold sustained 5s | GPS ground speed ≈ 0 | Wind can prevent "stillness" — use velocity magnitude not acceleration |

### Mach-Immune Apogee Detection

SparkyVT's key innovation (validated across 250+ flights including Mach 2.3): fuse accelerometer and barometer data, but **de-sense barometer input during supersonic flight.** At transonic speeds, shock waves create false pressure readings that naive algorithms mistake for apogee. The solution:
- Track integrated velocity from accelerometer
- When velocity exceeds ~Mach 0.8, reduce barometer weight in the estimator
- Use accelerometer-integrated position as primary until subsonic
- Resume barometer trust when velocity drops below Mach 0.7 (hysteresis)

This is a profile-level concern — a balloon mission doesn't need Mach correction, a high-power rocket does. The profile should specify which detection methods to use and whether Mach correction applies.

### Multi-Method Voting

For safety-critical transitions (apogee deploy), use at least two independent methods with configurable logic:
- **AND logic** (conservative): both methods must agree → delays deployment but reduces false fires
- **OR logic** (aggressive): either method triggers → fires faster but more false positives
- **PRIMARY + TIMEOUT**: use primary method, but if timeout expires, fire anyway

The profile defines which logic to use per transition. The state machine evaluates it.

---

## Architectural Recommendation for RocketChip

### For the State Machine: Lightweight HSM, Not Full QP

Full QP framework is more than RocketChip needs right now. But flat FSMs won't scale to multi-vehicle support. The sweet spot is a **minimal HSM implementation** — the dispatch pattern without the Active Object overhead.

The core concept: states are functions. Each state function receives an event and returns a disposition (handled, ignored, or "defer to my parent superstate"). The dispatcher walks up the hierarchy until someone handles the event.

```cpp
// Conceptual — state handler signature
typedef State (*StateHandler)(const Event* e);

// Each state handler does:
State state_coast(const Event* e) {
    switch (e->signal) {
    case EVT_ENTRY:   mark_burnout(); return HANDLED;
    case EVT_EXIT:    return HANDLED;
    case EVT_APOGEE:  return TRANSITION(state_descent);
    case EVT_TICK:    evaluate_apogee_conditions(); return HANDLED;
    default:          return SUPER(state_flight);  // defer to superstate
    }
}
```

This gives you behavioral inheritance (common abort handling in `state_flight` superstate), entry/exit action guarantees, and minimal overhead. On bare-metal RP2350, the entire HSM dispatcher is ~200 lines of C, uses ~100 bytes of RAM, and runs in <5µs per event dispatch.

**Why not flat FSM:** The multi-vehicle requirement means you'll have common behavior across vehicle types (abort → safe pyros → beacon) that shouldn't be duplicated per vehicle. An HSM superstate handles this once.

**Why not full QP:** RocketChip's Core 0 superloop is cooperative — you don't need Active Objects, event queues, or run-to-completion semantics enforced by framework. The state machine runs synchronously in `run_mission()` at 100Hz. Events are generated by the sensor evaluation in the same loop, not asynchronously from multiple threads.

### For Mission Profiles: FEMA Approach — Least Things to Break

The SAD currently plans `Mission_Rocket.cpp`, `Mission_HPR.cpp`, etc. as compiled code files. The right direction is profiles as data structures rather than code — but the *amount* of infrastructure matters.

**The FEMA principle: minimize failure modes while preserving future flexibility.** For Phase 5, that means:

- **One hardcoded rocket profile, baked into firmware as a `const` struct.** No profile selection menu, no flash loading, no validation-on-load, no SD card parsing. The `MissionEngine` loads it at boot. It works.
- **The struct layout is designed for future expansion** — no pointers inside it (use indices/IDs so it's position-independent when eventually loaded from flash), stable field order, reserved fields for future use.
- **No profile switching infrastructure yet.** The engine takes a `const MissionProfile*` at init. Today that pointer comes from a compiled-in constant. Tomorrow it can come from flash. The engine doesn't know or care.

This means the only things that can break are the profile data itself (which is `const` and validated at compile time) and the state machine logic. No runtime parsing, no storage I/O, no format versioning — just a struct and an engine.

The profile struct defines what the engine needs to know:

```cpp
struct MissionProfile {
    char name[32];
    uint8_t num_phases;
    PhaseDefinition phases[MAX_PHASES];
    TransitionRule transitions[MAX_TRANSITIONS];
    ActionBinding actions[MAX_ACTIONS];
    SensorConfig sensor_config;
    PyroMapping pyro_map[MAX_PYRO_CHANNELS];
    ServoMapping servo_map[MAX_SERVO_CHANNELS];
    DetectionConfig detection;
};

struct PhaseDefinition {
    uint8_t id;
    char name[16];
    uint8_t parent_phase;     // for HSM hierarchy (0 = top-level)
    uint32_t timeout_ms;      // max time in this phase (0 = no limit)
    uint8_t log_rate_hz;      // logging rate during this phase
};

struct TransitionRule {
    uint8_t from_phase;
    uint8_t to_phase;
    uint8_t guard_id;         // index into guard function table
    uint8_t num_conditions;
    ConditionDef conditions[MAX_CONDITIONS_PER_TRANSITION];
    CombineLogic logic;       // AND, OR, PRIMARY_PLUS_TIMEOUT
};

struct ConditionDef {
    SensorField field;        // which sensor value to check
    CompareOp op;             // GT, LT, EQ, etc.
    float threshold;
    uint32_t sustain_ms;      // must be true for this long
};
```

**Future expansion path (not Phase 5):** Once SD card hardware is integrated, profiles can be serialized to/from flash or SD using the same struct layout as a binary format. A `load_profile_from_flash()` deserializes the same struct the engine already consumes. A CLI profile editor writes to the same format. Validation on load (check transition targets exist, no orphaned phases, valid guard IDs) gets added at that point — not before it's needed.

### The Interaction Layer

The MissionEngine orchestrates all three subsystems, with the command/event/state separation enforced at the architectural level:

```
  CLI / Button / GCS
        │
        ▼ commands
┌────────────────────────────────────────────────────┐
│                  MissionEngine                      │
│                                                    │
│  ┌──────────────┐                                  │
│  │   Command     │  CMD_ARM → pre-arm checks       │
│  │   Handler     │  CMD_ABORT → immediate event     │
│  │              │  CMD_RESET → post-flight only    │
│  └──────┬───────┘                                  │
│         │ validated events only                     │
│         ▼                                          │
│  ┌──────────────┐       ┌──────────────────────┐   │
│  │   Profile     │       │    State Machine     │   │
│  │   Store       │──────▶│    (HSM Engine)      │   │
│  │              │ loads  │                      │   │
│  │ - profiles[] │ topo-  │ - current_state      │   │
│  │ - active_id  │ logy   │ - dispatch(event)    │   │
│  └──────────────┘       │ - transition()       │   │
│                          └──────────┬───────────┘   │
│                                     │                │
│  ┌──────────────┐       ┌──────────▼───────────┐   │
│  │  Condition    │──────▶│   Action Executor    │   │
│  │  Evaluator    │events │                      │   │
│  │              │       │ - fire_pyro()         │   │
│  │ - evaluate() │       │ - set_led()           │   │
│  │ - sustain    │       │ - start_log()         │   │
│  │   timers     │       │ - report_state()      │   │
│  └──────────────┘       └──────────────────────┘   │
│         ▲                                          │
│         │ FusedState (from seqlock, read-only)      │
└─────────┼──────────────────────────────────────────┘
          │
    Core 0 superloop
    (called at 100Hz from run_mission())
```

Execution flow each tick:
1. **Command handler** checks for pending commands (CLI input, button press, GCS message). Validates against current state and profile requirements. If valid, generates event.
2. **Condition evaluator** reads latest FusedState from seqlock. Evaluates active transition guards for the current phase (thresholds, sustain timers, multi-method logic). If any guard fires, generates event.
3. **State machine** processes events via HSM dispatch. May trigger transition.
4. On transition: exit actions of old state, entry actions of new state via **Action executor**.
5. **MissionState** struct updated — available to logger, telemetry, and UI modules in the same superloop.

### Fitting the Bare-Metal Execution Model

This runs entirely within Core 0's cooperative superloop:

```
Core 0 Main Loop (existing pattern):
  if (fusion_due)   run_fusion();
  if (mission_due)  run_mission();     ← MissionEngine::tick()
  if (logger_due)   run_logger();
  if (telem_due)    run_telemetry();
  if (ui_due)       run_ui();
```

No queues, no RTOS primitives, no cross-core synchronization beyond the existing seqlock for sensor data. The state machine reads sensor data synchronously, evaluates conditions synchronously, and dispatches events synchronously. This matches how the rest of the Core 0 modules work.

The 100Hz tick rate gives 10ms per evaluation cycle — more than enough for guard condition evaluation and state transition. The actual sensor fusion (running at 200Hz on Core 0) provides pre-fused altitude, velocity, and attitude estimates to the condition evaluator — the state machine doesn't need to do its own sensor math.

---

## Example: Rocket Profile (Phase 5) vs. Balloon Profile (Future)

The rocket profile is what gets built for Phase 5. The balloon profile illustrates how the same engine supports a completely different vehicle type later — no engine changes, just a different `const MissionProfile`.

**Rocket Profile (Phase 5 — the one we build):**
```
IDLE → ARMED → BOOST → COAST → APOGEE → DROGUE_DESCENT → MAIN_DESCENT → LANDED
```
- Launch detection: accel > 2.5g sustained 50ms
- Burnout detection: accel < 1.0g after launch
- Apogee: Kalman velocity zero-crossing, Mach-corrected baro
- Drogue deploy: on apogee, pyro channel 1
- Main deploy: altitude < 200m AGL, pyro channel 2
- Landing: velocity < 0.5 m/s sustained 5s

**Balloon Profile (future — illustrates platform flexibility):**
```
IDLE → ARMED → ASCENT → FLOAT → DESCENT → LANDED
```
- Launch detection: baro altitude increasing > 2 m/min sustained 30s (no accel spike)
- Float detection: altitude rate < 0.5 m/min sustained 60s
- Descent detection: altitude decreasing > 1 m/min sustained 30s
- Cutdown: altitude > ceiling OR timer expires, pyro channel 1
- Landing: GPS ground speed < 0.5 m/s sustained 30s

**Same state machine engine, different profiles.** The HSM just processes phases and transitions. The profiles define completely different topologies, thresholds, and action bindings. For Phase 5, only the rocket profile exists — but the engine is already profile-agnostic.

---

## Implementation Phasing (Aligned with SAD Phase 5)

**Phase 5a — Flat FSM with HSM-Compatible Signatures:**
- State handlers as function pointers (not a switch on enum) — same signature the HSM dispatcher will use later
- Hardcoded rocket profile as `const MissionProfile` struct
- Command handler with pre-arm check framework (CMD_ARM → validation → EVT_ARMED)
- Flat dispatch: `default: return HANDLED;` (no superstate deferral yet)
- MissionState struct updated each tick (including FlightStats), visible to logger/telemetry
- Single ABORT state reachable from any flight phase
- `tools/state_to_dot.py` generates visualization

**Phase 5b — Condition Evaluator:**
- Multi-method detection with configurable AND/OR/TIMEOUT logic per transition
- Sustain timers (acceleration > threshold for N ms)
- Reads FusedState from seqlock, not raw SensorData
- Guard functions indexed by ID in the profile's TransitionRule

**Phase 5c — HSM Upgrade (Additive, No Refactor):**
- Add `state_flight` superstate with common abort handling
- Change `default: return HANDLED;` → `default: return SUPER(state_flight);` in flight-phase handlers
- Entry/exit action framework on superstates
- ~50 lines of dispatcher change, zero existing handler refactoring
- Validate with HPR dual-deploy profile (drogue + main separation) as second `const` profile

**Phase 5d — Profile Infrastructure (Post-MVP):**
- Profile selection via CLI before arming (choose from compiled-in options)
- Flash/SD storage format (same struct, serialized)
- Validation on load (transition targets, guard IDs, reachability)
- CLI profile editor

---

## Decisions Made

1. **HSM from the start, flat first step.** State handlers use HSM-compatible function-pointer signatures from Phase 5a. Flat dispatch initially (`return HANDLED`), upgraded to hierarchy in 5c by changing one line per handler to `return SUPER(parent)`. Zero throwaway code.

2. **Profile format: FEMA approach.** One hardcoded rocket profile as `const MissionProfile` struct baked into firmware. Struct layout designed for future flash/SD serialization (no pointers, use indices). No profile switching infrastructure until post-MVP.

3. **Command/Event/State separation.** Commands are validated before generating events. The state machine only sees validated events, never raw commands or sensor data. Pre-arm checks live in the command handler, sensor evaluation lives in the condition evaluator, state transition logic lives in the state machine. Three clean layers.

4. **Abort architecture: single ABORT state, profile-driven behavior.** ABORT is a single state reachable from any flight phase. The *actions* executed on ABORT entry are defined per-phase in the mission profile, not hardcoded in the engine. The profile specifies "if ABORT from phase X, execute actions Y." Any phase without an explicit abort action set falls back to the universal default: safe all outputs, go neutral, enable beacon. This mirrors the Space Shuttle's phase-dependent abort modes (Pad Abort, RTLS, TAL, ATO) — different responses pre-planned for different flight regimes, executed by the same system.

   **Critical safety note:** pre-apogee recovery deployment (drogue/main) during thrust or high dynamic pressure is dangerous — it can shred chutes, snap shock cords, or structural-fail the airframe (ref: BPS.space failures). The correct pre-apogee abort for most rockets is: safe all pyros, do nothing, let the vehicle go ballistic. Recovery deploys when the vehicle naturally decelerates through the normal state machine transitions if still running. Profile-defined abort actions must respect flight regime constraints.

   **Abort action safety validation is a GCS-side concern, not onboard.** The RocketChip ground control software validates profiles before upload: cross-referencing abort actions against phase flight regimes (high-Q, supersonic, under-thrust) to flag physically dangerous combinations like pyro deployment during boost or simultaneous pyro fires without separation time. The onboard flight computer executes the profile as given — it trusts that the profile has been validated. This keeps the onboard code lean and deterministic while the ground tools handle "is this profile going to do something stupid" checks.

5. **FlightStats inside MissionState.** Co-located with flight markers (launch_time, apogee_time, etc.) for simplicity. Max altitude, max velocity, max acceleration tracked alongside phase state.

6. **Condition evaluator: compiled guard functions (Option A).** Each guard is a plain C function indexed by ID from the profile. Parameterized by threshold, comparison operator, and sustain values in the `ConditionDef` struct — thresholds and direction are tunable per-profile without recompilation, only new guard *types* require code changes. Interpretive expression parser (Option B) deferred to post-MVP; only needed if user-created vehicle types require guards the firmware author didn't anticipate.

   **Guard implementation pattern: separate functions, shared comparison logic.** Each guard function is named by the sensor field it reads (self-documenting per JSF preference), but comparison direction lives in the `ConditionDef.op` field, not the function name. A shared `compare_field()` helper implements the operator logic exactly once. This gives named, auditable functions without doubling the guard table for above/below variants:

   ```cpp
   static bool compare_field(float value, CompareOp op, float threshold) {
       switch (op) {
       case GT: return value > threshold;
       case LT: return value < threshold;
       case GE: return value >= threshold;
       case LE: return value <= threshold;
       default: return false;  // unknown op = safe default
       }
   }

   bool guard_accel_body_z(const FusedState* s, const ConditionDef* p, bool* valid) {
       *valid = s->accel_valid;
       return compare_field(s->accel_z_body, p->op, p->threshold);
   }
   ```

   **Guard validity output:** every guard returns a `bool* valid` indicating whether its input data was trustworthy. The evaluator applies a per-transition validity policy from the `TransitionRule`: invalid guard → block transition (conservative) or invalid guard → force transition (fail-safe to recovery).

   **Evaluator-level capabilities** (not guard types): sustain timing (guard must be true for N ms continuously — tracked by evaluator, configured in `ConditionDef.sustain_ms`), edge triggering (fire only on false→true transition for zero-crossing detection), and timer-since-event (elapsed time since a logged event, reading timestamps already in `MissionState`).

   Guard type inventory reviewed by council — see council findings below.

7. **Pre-arm check configurability: two-tier model (like abort).** A set of **default safety checks always runs** regardless of profile — these are "is the board OK" fundamentals that no profile can bypass: sensor health, calibration validity, no active errors, storage space. On top of that, each **profile defines additional go/no-go checks** specific to its vehicle type and mission: GPS fix required, tilt limits, pyro continuity, arm switch, battery minimums. Profiles can add or remove checks from the profile-specific tier but cannot touch the default safety tier. Same pattern as abort: universal safe default + profile-specific overlay.

## Council Review Findings

Council review conducted with all four main panelists plus CubeSat Startup Engineer auxiliary. Consensus reached on all three items.

### Guard Function Inventory

The condition evaluator uses compiled guard functions indexed by ID (Option A), with separate functions per sensor field sharing a common `compare_field()` helper. Comparison operator lives in `ConditionDef`, not the function name. Each guard additionally returns a validity flag for its input data.

Evaluator-level capabilities handle sustain timing, edge triggering (zero-crossing detection), timer-since-event, and per-transition validity policy — these are not guard types but evaluator behaviors that compose with any guard.

| Guard Function | Sensor Field | Vehicle Scope |
|---|---|---|
| `guard_accel_body_z` | Body-frame Z accel | Rockets (launch, burnout) |
| `guard_accel_world_mag` | World-frame accel magnitude | All (landing, crash) |
| `guard_altitude_agl` | Altitude above ground | All (deploy, ceiling) |
| `guard_velocity_z` | Vertical velocity | Rockets (apogee), balloons |
| `guard_velocity_mag` | Total velocity magnitude | All (landing) |
| `guard_altitude_rate` | Altitude rate of change | Balloons (ascent/float/descent) |
| `guard_tilt` | Angle from vertical | Rockets (safety), drones |
| `guard_battery` | Battery voltage | All (power management) |
| `guard_gps_speed` | GPS ground speed | All (landing backup) |
| `guard_distance_origin` | Distance from launch point | Drones (geofence) |
| `guard_ekf_confidence` | ESKF innovation / NIS | All (fusion health) |
| `guard_sensor_healthy` | Sensor watchdog status | All (fault detection) |
| `guard_pyro_continuity` | Pyro channel continuity | Rockets w/ pyro (deploy readiness) |
| `guard_arm_switch` | Physical switch state | Titan tier |
| `guard_timer_since_event` | Elapsed since logged event | All (mission elapsed, pyro spacing) |
| `guard_always_true` | — | All (unconditional/sequenced) |

### Pre-Arm Checks (Default Rocket Profile)

Two-tier structure: default safety checks that always run (no profile can bypass) + profile-specific checks that the profile's go/no-go list controls.

**Default tier (always runs, not profile-configurable):**

1. IMU reporting valid data at expected rate
2. Baro reporting valid data
3. Gyro calibration complete (bias offsets applied)
4. Accelerometer calibration valid
5. Gravity vector magnitude within 5% of 9.81 m/s² (accel cal sanity)
6. No active sensor errors in last N seconds
7. Flash storage has space for flight log

**Profile tier (rocket profile defaults, configurable per mission type):**

8. Vehicle approximately level (tilt < configurable threshold, catches "armed while falling off rail")
9. Battery voltage above minimum (if monitored)
10. GPS fix valid with HDOP < threshold (if profile requires GPS)
11. Pyro continuity on configured channels (if Titan tier)
12. Arm switch engaged (if Titan tier with physical switch)

Each check is a named function returning pass/fail with a reason string. The CLI prints which specific check(s) failed on rejected ARM command.

### Abort Behavior

Single ABORT state. Entry actions are **profile-defined per source phase**, with universal default: safe all outputs, go neutral, enable beacon. This mirrors Space Shuttle phase-dependent abort modes (Pad Abort, RTLS, TAL, ATO) — different pre-planned responses depending on when failure occurs and what the vehicle can still do.

**Critical safety constraint:** pre-apogee recovery deployment is dangerous on rockets — high dynamic pressure can shred chutes, snap shock cords, or structural-fail the airframe (ref: BPS.space failures). The correct pre-apogee rocket abort is typically: safe all pyros, do nothing, let the vehicle decelerate ballistically. Recovery deploys through normal state transitions if the state machine is still running.

**Abort action safety validation is a GCS-side concern, not onboard.** The RocketChip ground control software validates profiles before upload: cross-referencing abort actions against phase flight regimes (high-Q, supersonic, under-thrust) to flag physically dangerous combinations like pyro deployment during boost or simultaneous pyro fires without separation time. The onboard flight computer executes the validated profile as given — keeping flight code lean and deterministic.

---

## Remaining Design Questions

1. **Separate design document needed.** This research doc has reached its useful scope as a "what did we learn and decide" reference. A dedicated Mission Engine design document should capture the implementation spec: state handler signatures, dispatch logic, HSM upgrade path, the complete abort behavior matrix, condition evaluator internals (guard table, sustain tracking, edge detection, validity handling), and the `MissionProfile` struct definition with the rocket profile as the first concrete instance. That becomes the "build from" document for Phase 5. **Deferred to Claude Code** which has better repo context for integration with existing SAD, IVP, and codebase.

---

## Sources

- RocketChip repo `docs/SAD.md` Section 3.3, 5, 6 — current architecture
- RocketChip `DEPRECATED/Dev/Claude/Modular/*/StateTracking.h` — old state machine
- ArduPilot mode system — `ardupilot.org/dev/docs/apmcopter-adding-a-new-flight-mode.html`
- PX4 Commander + failsafe rewrite — `github.com/PX4/PX4-Autopilot/pull/20172`
- SparkyVT HPR Flight Computer — `github.com/SparkyVT/HPR-Rocket-Flight-Computer`
- NASA cFS Table Service — `github.com/nasa/cFE`
- QP/Samek HSM framework — `state-machine.com/products/qp`
- Mars Perseverance FSW (Steve Scandore, Embedded Online Conference 2021)
- PSAS elderberry framework — `github.com/psas/elderberry`
- Waterloo Rocketry CAN architecture — `github.com/waterloo-rocketry`
