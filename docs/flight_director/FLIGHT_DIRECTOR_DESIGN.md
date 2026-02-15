# Flight Director Design Specification

**Status:** PRELIMINARY — All details pending implementation validation
**For:** RocketChip (RP2350 bare-metal Pico SDK, dual-core AMP)
**IVP Scope:** IVP-50 (State Machine Core), IVP-51 (Event Engine), IVP-52 (Action Executor), IVP-53 (Confidence-Gated Actions)
**Date:** 2026-02-14

> **Every technical detail in this document is preliminary.** Nothing is finalized until implementation validates the design against the hardware, the sensor pipeline, and the existing codebase. Values, signatures, struct layouts, and state definitions may change during development.

---

## 1. Overview & Scope

### What This Document Covers

The **Flight Director** is the runtime engine that tracks flight state, processes events, executes transitions, and coordinates the condition evaluator and action executor. It is profile-agnostic — the Flight Director doesn't know what a "rocket" is. It follows whatever Mission Profile is loaded.

**Naming convention:** "Flight Director" is the front-facing name in code and documentation. Internal mechanisms — the hierarchical state machine (HSM) dispatcher, the condition evaluator, the action executor — are implementation details. They are described in this design document but are not the public name. In code: `FlightDirector`, `flight_director.h`, `run_flight_director()`.

### What This Document Does NOT Cover

**Mission Profiles** — the configuration data that defines phases, transitions, guard conditions, abort actions, thresholds, and hardware mappings — are documented separately in `docs/mission_profiles/MISSION_PROFILES.md`.

### The Relationship

**The Flight Director follows the Mission Profile.**

A high schooler, a NASA engineer, and a Hackaday reader all understand that sentence.

### Relationship to Existing Documents

- **SAD Section 6** defines the state machine topology and event-condition-action examples. This document refines those into an implementation spec.
- **RESEARCH.md** (in this folder) documents the research, prior art survey, and council-reviewed architectural decisions that inform this design.
- **IVP Stage 6** (IVP-49 through IVP-54) defines the implementation and verification steps.
- **IVP-49** (Watchdog Recovery Policy) is a prerequisite — the Flight Director must know how the system recovers from a mid-flight reboot.

---

## 2. Architecture Summary (PRELIMINARY)

### Command → Event → Flight Director → Action Pipeline

The Flight Director enforces a strict three-layer separation between inputs, state logic, and outputs:

```
  CLI / Button / GCS
        │
        ▼ commands
┌──────────────────────────────────────────────────────┐
│                  Flight Director                      │
│                                                      │
│  ┌──────────────┐                                    │
│  │   Command     │  CMD_ARM → pre-arm checks         │
│  │   Handler     │  CMD_ABORT → immediate event       │
│  │              │  CMD_RESET → post-flight only      │
│  └──────┬───────┘                                    │
│         │ validated events only                       │
│         ▼                                            │
│  ┌──────────────┐       ┌──────────────────────┐     │
│  │   Mission     │       │    State Tracking    │     │
│  │   Profile     │──────▶│    Engine (HSM)      │     │
│  │   (loaded)   │ topo-  │                      │     │
│  │              │ logy   │ - current_phase      │     │
│  └──────────────┘       │ - dispatch(event)    │     │
│                          │ - transition()       │     │
│  ┌──────────────┐       └──────────┬───────────┘     │
│  │  Condition    │                  │                  │
│  │  Evaluator    │──events──▶      │                  │
│  │              │                  │                  │
│  │ - evaluate() │       ┌──────────▼───────────┐     │
│  │ - sustain    │       │   Action Executor    │     │
│  │   timers     │       │                      │     │
│  └──────────────┘       │ - fire_pyro()         │     │
│         ▲                │ - set_led()           │     │
│         │                │ - start_log()         │     │
│         │                │ - report_state()      │     │
│         │ FusedState     └──────────────────────┘     │
│         │ (seqlock,                                   │
│         │  read-only)                                 │
└─────────┼────────────────────────────────────────────┘
          │
    Core 0 superloop
    (called at 100Hz from run_flight_director())
```

**Commands** are inputs — things that *request* something happen. They can be rejected.
**Events** are validated facts — things confirmed true. They are the sole input to the state tracking engine.
**Phases** (states) are what the system *actually is* right now. They change only when the state tracking engine processes a validated event.

### Execution Model

The Flight Director runs entirely within Core 0's cooperative superloop at 100Hz:

```
Core 0 Main Loop:
  if (fusion_due)    run_fusion();           // 200Hz
  if (director_due)  run_flight_director();  // 100Hz ← this module
  if (logger_due)    run_logger();           // 50Hz
  if (telem_due)     run_telemetry();        // 10Hz
  if (ui_due)        run_ui();               // 30Hz
```

No queues, no RTOS primitives, no cross-core synchronization beyond the existing seqlock for sensor data. The Flight Director reads sensor data synchronously, evaluates conditions synchronously, and dispatches events synchronously.

### Tick Execution Flow (PRELIMINARY)

Each `run_flight_director()` tick:

1. **Command handler** checks for pending commands (CLI input, button press). Validates against current phase and profile requirements. If valid, generates event.
2. **Condition evaluator** reads latest FusedState from seqlock. Evaluates active transition guards for the current phase. If any guard fires, generates event.
3. **State tracking engine** processes events via HSM dispatch. May trigger transition.
4. On transition: exit actions of old phase, entry actions of new phase via **action executor**.
5. **FlightState** struct updated — available to logger, telemetry, and UI modules in the same superloop.

---

## 3. State Handler Signatures & Dispatch Logic (PRELIMINARY)

### State Handler Signature

Each flight phase is implemented as a function with a common signature:

```cpp
// PRELIMINARY — exact types TBD during implementation
enum class Disposition { HANDLED, IGNORED };

struct Transition {
    StateHandler target;
};

// State handler function pointer
typedef Disposition (*StateHandler)(const Event* e);

// Macros for dispatch return values
#define HANDLED       Disposition::HANDLED
#define TRANSITION(s) /* sets pending transition target to s, returns HANDLED */
#define SUPER(s)      /* defers to parent superstate s, returns IGNORED */
```

### Phase Handler Pattern

```cpp
// PRELIMINARY — illustrative, not final code
Disposition phase_coast(const Event* e) {
    switch (e->signal) {
    case SIG_ENTRY:   log_event(EVT_BURNOUT); return HANDLED;
    case SIG_EXIT:    return HANDLED;
    case SIG_APOGEE:  return TRANSITION(phase_apogee);
    case SIG_TICK:    /* condition evaluator runs here */ return HANDLED;
    default:          return SUPER(phase_flight);  // defer to superstate
    }
}
```

### Flat Dispatch (Phase 5a — Initial Implementation)

In the initial flat implementation, all handlers use `return HANDLED` for the default case instead of `SUPER()`. The dispatcher simply calls the current phase's handler:

```cpp
// PRELIMINARY — flat dispatch
void dispatch(const Event* e) {
    current_handler(e);
}
```

### HSM Dispatch (Phase 5c — Additive Upgrade)

The upgrade to HSM adds hierarchy walking. When a handler returns `IGNORED` (via `SUPER()`), the dispatcher walks up to the parent superstate:

```cpp
// PRELIMINARY — HSM dispatch (~50 lines added)
void dispatch(const Event* e) {
    StateHandler handler = current_handler;
    while (handler != nullptr) {
        Disposition d = handler(e);
        if (d == HANDLED) return;
        handler = get_parent(handler);  // walk up hierarchy
    }
}
```

The upgrade from flat to HSM requires:
- Changing `default: return HANDLED;` → `default: return SUPER(parent);` in each flight-phase handler (one line each)
- Adding the hierarchy walk loop to the dispatcher (~50 lines)
- Zero changes to existing handler logic

### Entry/Exit Action Order on Transitions (PRELIMINARY)

When transitioning from phase A to phase B through a common ancestor:
1. Call `SIG_EXIT` on A
2. Call `SIG_EXIT` on each ancestor of A up to (but not including) the Least Common Ancestor (LCA)
3. Call `SIG_ENTRY` on each ancestor of B down from (but not including) the LCA
4. Call `SIG_ENTRY` on B

For flat dispatch (Phase 5a), transitions are always peer-to-peer: exit A, enter B.

---

## 4. Flight Phase Definitions (PRELIMINARY)

### Phase Topology (Rocket Mission Profile — Baked In)

```
IDLE → ARMED → BOOST → COAST → APOGEE → DROGUE_DESCENT → MAIN_DESCENT → LANDED
                                                                          ↓
                                                                        (reset)
                                                                          ↓
                                                                        IDLE

Any flight phase → ABORT (via EVT_ABORT)
ABORT → IDLE (via CMD_RESET, manual only)
```

### HSM Hierarchy (PRELIMINARY)

```
state_top                    (root — handles unhandled events)
├── phase_idle               (ground, pre-arm)
├── phase_armed              (ground, armed, awaiting launch)
├── state_flight             (superstate — common abort handling)
│   ├── phase_boost          (powered ascent)
│   ├── phase_coast          (unpowered ascent)
│   ├── phase_apogee         (apogee detection + drogue deploy)
│   ├── phase_drogue_descent (drogue descent, awaiting main deploy)
│   └── phase_main_descent   (main chute descent)
├── phase_landed             (post-flight, data preservation)
└── phase_abort              (safe state, profile-defined actions)
```

`state_flight` superstate handles `EVT_ABORT` once — all flight phases inherit this behavior without duplicating the transition.

### Phase Definitions Table (PRELIMINARY)

| Phase | Entry Condition | Entry Actions | Exit Actions | Timeout |
|-------|----------------|---------------|-------------|---------|
| IDLE | Power-on / reset | LED: breathing blue | — | None |
| ARMED | EVT_ARMED (pre-arm passed) | LED: solid amber, start pre-buffer | — | 180s → IDLE |
| BOOST | EVT_LAUNCH | Start logging, mark LAUNCH, LED: red flash | — | 30s → COAST |
| COAST | EVT_BURNOUT | Mark BURNOUT | — | 60s → APOGEE |
| APOGEE | EVT_APOGEE | Mark APOGEE, fire drogue (pyro 1) | — | 5s → DROGUE_DESCENT |
| DROGUE_DESCENT | Drogue deployed / timeout | — | — | None |
| MAIN_DESCENT | EVT_MAIN_DEPLOY | Fire main (pyro 2), mark MAIN | — | None |
| LANDED | EVT_LANDING | Stop logging, LED: green, beep x5 | — | None |
| ABORT | EVT_ABORT | Profile-defined per source phase | — | None |

> **Timeout values are safety backstops**, not expected transitions. If the primary detection method fails, the timeout forces progression to prevent the vehicle from being stuck in a phase indefinitely.

---

## 5. HSM Upgrade Path (PRELIMINARY)

### Phase 5a → Phase 5c (Zero Throwaway Code)

The flat-to-HSM upgrade is purely additive:

| Change | Scope | Lines |
|--------|-------|-------|
| `default: return HANDLED;` → `default: return SUPER(state_flight);` | Each flight-phase handler | 1 per handler (~7 total) |
| Add hierarchy walk loop to dispatcher | `dispatch()` function | ~50 |
| Add `state_flight` superstate handler | New function | ~15 |
| Add `state_top` root handler | New function | ~10 |

Total: ~80 lines added, 7 lines changed, zero deleted.

---

## 6. Event Definitions (PRELIMINARY)

### Event Signal Enum

```cpp
// PRELIMINARY — signal values TBD
enum class Signal : uint8_t {
    // System events (internal to dispatcher)
    SIG_ENTRY,          // Phase entered
    SIG_EXIT,           // Phase exiting
    SIG_TICK,           // Periodic evaluation (100Hz)

    // Command-generated events (from command handler)
    SIG_ARMED,          // Pre-arm checks passed
    SIG_DISARMED,       // Disarm acknowledged
    SIG_ABORT,          // Abort commanded
    SIG_RESET,          // Reset to IDLE

    // Sensor-generated events (from condition evaluator)
    SIG_LAUNCH,         // Launch detected
    SIG_BURNOUT,        // Motor burnout detected
    SIG_APOGEE,         // Apogee detected
    SIG_DROGUE_DEPLOY,  // Drogue deployment confirmed
    SIG_MAIN_DEPLOY,    // Main altitude reached
    SIG_LANDING,        // Landing detected

    // Phase timeout
    SIG_TIMEOUT,        // Phase timeout expired
};
```

### Event Struct (PRELIMINARY)

```cpp
struct Event {
    Signal signal;
    uint32_t timestamp_us;  // time_us_32() at generation
    union {
        uint8_t  source_phase;   // for ABORT: which phase triggered it
        float    sensor_value;   // for sensor events: triggering value
        uint8_t  command_source; // CLI=0, button=1, GCS=2
    } payload;
};
```

---

## 7. Command Handler (PRELIMINARY)

### Command Processing

Commands are external requests that must be validated before generating events. The command handler is the gatekeeper — it ensures only valid, safe transitions are requested.

| Command | Allowed From | Validation | Generated Event | Rejection Behavior |
|---------|-------------|------------|----------------|--------------------|
| CMD_ARM | IDLE | Pre-arm checks (Section 9) | SIG_ARMED | Stay IDLE, report failed checks |
| CMD_DISARM | ARMED | None (always valid from ARMED) | SIG_DISARMED | Ignored if not ARMED |
| CMD_ABORT | Any flight phase | None (abort is always valid) | SIG_ABORT | Ignored if IDLE/LANDED |
| CMD_RESET | LANDED, ABORT | None | SIG_RESET | Ignored if in flight |

### Command Sources (PRELIMINARY)

- **CLI:** Keypress ('a' for arm, 'd' for disarm, 'x' for abort, 'r' for reset)
- **Button hold:** 2-second hold to arm (hardware debounced)
- **GCS message:** MAVLink command (future)

All sources route through the same command handler — identical validation regardless of source.

---

## 8. Condition Evaluator (PRELIMINARY)

### Guard Function Architecture

The condition evaluator reads FusedState from the seqlock and evaluates transition guards defined in the active Mission Profile. Each guard is a compiled C function indexed by ID.

### Guard Function Signature (PRELIMINARY)

```cpp
// Each guard returns true if its condition is met.
// *valid indicates whether the guard's input data was trustworthy.
typedef bool (*GuardFn)(const FusedState* state,
                        const ConditionDef* params,
                        bool* valid);
```

### Shared Comparison Helper (PRELIMINARY)

```cpp
static bool compare_field(float value, CompareOp op, float threshold) {
    switch (op) {
    case CompareOp::GT: return value > threshold;
    case CompareOp::LT: return value < threshold;
    case CompareOp::GE: return value >= threshold;
    case CompareOp::LE: return value <= threshold;
    default:            return false;  // unknown op = safe default
    }
}
```

### Guard Function Table (PRELIMINARY — From Council Review)

| ID | Guard Function | Sensor Field | Vehicle Scope |
|----|---------------|-------------|---------------|
| 0 | `guard_accel_body_z` | Body-frame Z accel | Rockets (launch, burnout) |
| 1 | `guard_accel_world_mag` | World-frame accel magnitude | All (landing, crash) |
| 2 | `guard_altitude_agl` | Altitude above ground | All (deploy, ceiling) |
| 3 | `guard_velocity_z` | Vertical velocity | Rockets (apogee), balloons |
| 4 | `guard_velocity_mag` | Total velocity magnitude | All (landing) |
| 5 | `guard_altitude_rate` | Altitude rate of change | Balloons (ascent/float) |
| 6 | `guard_tilt` | Angle from vertical | Rockets (safety), drones |
| 7 | `guard_battery` | Battery voltage | All (power management) |
| 8 | `guard_gps_speed` | GPS ground speed | All (landing backup) |
| 9 | `guard_distance_origin` | Distance from launch point | Drones (geofence) |
| 10 | `guard_ekf_confidence` | ESKF innovation / NIS | All (fusion health) |
| 11 | `guard_sensor_healthy` | Sensor watchdog status | All (fault detection) |
| 12 | `guard_pyro_continuity` | Pyro channel continuity | Rockets w/ pyro |
| 13 | `guard_arm_switch` | Physical switch state | Titan tier |
| 14 | `guard_timer_since_event` | Elapsed since logged event | All (mission elapsed) |
| 15 | `guard_always_true` | — | All (unconditional/sequenced) |

### Evaluator-Level Capabilities (PRELIMINARY)

These are NOT guard types — they are evaluator behaviors that compose with any guard:

- **Sustain timing:** Guard must return true continuously for `ConditionDef.sustain_ms` before the evaluator reports it as satisfied. Tracked per-transition with a timer that resets on any false return.
- **Edge triggering:** Fire only on the false→true transition (for zero-crossing detection like apogee). The evaluator tracks previous guard state per-transition.
- **Timer-since-event:** Reads timestamps from FlightState (e.g., "time since LAUNCH > 30s"). Uses `guard_timer_since_event` with the event type in the ConditionDef.
- **Validity policy:** Each TransitionRule specifies what happens when a guard returns `*valid = false`: block the transition (conservative, default) or force the transition (fail-safe to recovery). The policy is per-transition, not per-guard.

### Multi-Method Logic (PRELIMINARY)

Each TransitionRule specifies how multiple conditions combine:

| Logic | Behavior | Use Case |
|-------|----------|----------|
| `AND` | All conditions must be satisfied | Conservative — both methods agree |
| `OR` | Any condition triggers | Aggressive — either method fires |
| `PRIMARY_PLUS_TIMEOUT` | Primary method preferred, timeout fires if primary doesn't | Redundancy — timeout as backup |

---

## 9. Pre-Arm Checks (PRELIMINARY)

### Two-Tier Model

#### Default Tier (Always Runs — Profile Cannot Bypass)

These are "is the board OK" fundamentals:

| # | Check | Failure Reason String |
|---|-------|----------------------|
| 1 | IMU reporting valid data at expected rate | "IMU not reporting" |
| 2 | Baro reporting valid data | "Baro not reporting" |
| 3 | Gyro calibration complete (bias offsets applied) | "Gyro not calibrated" |
| 4 | Accelerometer calibration valid | "Accel not calibrated" |
| 5 | Gravity vector magnitude within 5% of 9.81 m/s² | "Accel cal sanity fail" |
| 6 | No active sensor errors in last N seconds | "Recent sensor errors" |
| 7 | Flash storage has space for flight log | "Storage full" |

#### Profile Tier (Configurable Per Mission Profile)

| # | Check | Failure Reason String | Notes |
|---|-------|-----------------------|-------|
| 8 | Vehicle approximately level (tilt < threshold) | "Tilt exceeds limit" | Catches "armed while tilted on rail" |
| 9 | Battery voltage above minimum | "Battery low" | If monitored |
| 10 | GPS fix valid with HDOP < threshold | "No GPS fix" | If profile requires GPS |
| 11 | Pyro continuity on configured channels | "Pyro N no continuity" | Titan tier |
| 12 | Arm switch engaged | "Arm switch off" | Titan tier |

### Check Function Signature (PRELIMINARY)

```cpp
struct PreArmResult {
    bool passed;
    const char* reason;  // null if passed, static string if failed
};

typedef PreArmResult (*PreArmCheckFn)(const FusedState* state,
                                      const MissionProfile* profile);
```

### CLI Output on Rejected ARM (PRELIMINARY)

```
[ARM] Pre-arm check failed:
  [FAIL] Gyro not calibrated
  [FAIL] Tilt exceeds limit (12.3° > 5.0°)
  [PASS] IMU reporting (7/7 default checks)
  [PASS] Battery OK (3.92V > 3.50V)
```

---

## 10. Abort Behavior Matrix (PRELIMINARY)

### Architecture

Single ABORT phase. Entry actions are **profile-defined per source phase**, with a universal default fallback.

**Universal default (if profile doesn't specify):** Safe all outputs, go neutral, enable beacon.

### Rocket Profile Abort Actions (PRELIMINARY)

| Source Phase | Abort Entry Actions | Safety Rationale |
|-------------|--------------------|--------------------|
| IDLE | Disarm | N/A — not in flight |
| ARMED | Disarm, report reason | Pre-launch, safe to stand down |
| BOOST | **Safe all pyros, do nothing** | High dynamic pressure — deploying chute can shred it or break the airframe |
| COAST | **Safe all pyros, do nothing** | May still be supersonic — same risk as BOOST |
| APOGEE | Fire drogue if not already fired | Recovery is priority — vehicle is decelerating |
| DROGUE_DESCENT | Fire main if not already fired | Recovery is priority — get the main out |
| MAIN_DESCENT | Beacon, continue logging | Already recovering — don't interfere |
| LANDED | Beacon, report | Post-flight — preserve data |

> **Critical safety constraint:** Pre-apogee recovery deployment during thrust or high dynamic pressure is dangerous — it can shred chutes, snap shock cords, or structural-fail the airframe (ref: BPS.space failures). The correct pre-apogee abort for most rockets is: safe all pyros, do nothing, let the vehicle decelerate ballistically.

### Abort Action Safety Validation

**This is a GCS-side concern, not onboard.** The ground control software validates profiles before upload: cross-referencing abort actions against phase flight regimes to flag physically dangerous combinations. The onboard Flight Director executes the validated profile as given — keeping flight code lean and deterministic.

---

## 11. Action Executor (PRELIMINARY)

### Action Types (PRELIMINARY)

| Action | Description | Safety Class |
|--------|-------------|-------------|
| `fire_pyro(channel)` | Fire pyrotechnic channel | **Irreversible** — confidence-gated (IVP-53) |
| `set_led(pattern)` | Set NeoPixel pattern | Reversible |
| `start_log()` | Begin flight data logging | Reversible |
| `stop_log()` | End flight data logging | Reversible |
| `mark_event(type)` | Record timestamped event in FlightState | Reversible |
| `report_state()` | Send state change to telemetry | Reversible |
| `set_beacon(on)` | Enable/disable recovery beacon | Reversible |

### Action Binding Model (PRELIMINARY)

Actions are bound to phases via the Mission Profile:
- **Entry actions:** Executed when a phase is entered (after `SIG_ENTRY` dispatched)
- **Exit actions:** Executed when a phase is exited (after `SIG_EXIT` dispatched)
- **Abort actions:** Executed when ABORT is entered, keyed by source phase

The action executor receives an action list and executes them sequentially. It does not make decisions — it executes what the profile and state tracking engine tell it to.

### Confidence Gating (IVP-53, PRELIMINARY)

Irreversible actions (pyro fires) are gated by the confidence flag from the ESKF confidence gate (IVP-48):
- `confident = true` → action executes normally
- `confident = false` → action is **held**, not discarded. If confidence recovers within a timeout, the action fires. If timeout expires, the action fires anyway (fail-safe to recovery).

---

## 12. FlightState Struct (PRELIMINARY)

### Layout

```cpp
// PRELIMINARY — field types and order TBD
struct FlightState {
    // Current state
    uint8_t  current_phase;       // FlightPhase enum
    uint8_t  previous_phase;      // Phase before last transition
    uint16_t transition_count;    // Total transitions since arm
    uint32_t phase_entry_time_us; // time_us_32() when current phase entered

    // Flight markers (0 = not yet reached)
    uint32_t launch_time_us;
    uint32_t burnout_time_us;
    uint32_t apogee_time_us;
    uint32_t drogue_time_us;
    uint32_t main_time_us;
    uint32_t landing_time_us;

    // Flight statistics (updated continuously during flight)
    float    max_altitude_m;      // Peak altitude AGL
    float    max_velocity_ms;     // Peak velocity magnitude
    float    max_accel_g;         // Peak acceleration in g
    float    apogee_altitude_m;   // Altitude at apogee detection

    // Status flags
    bool     armed;
    bool     in_flight;           // true from BOOST through LANDED
    bool     abort_active;
    uint8_t  abort_source_phase;  // Which phase triggered abort
};
```

### Access Pattern

FlightState is updated by the Flight Director on Core 0 and read by:
- **Logger:** Logs flight events and statistics (Core 0, same superloop)
- **Telemetry:** Reports state changes and stats (Core 0, same superloop)
- **CLI:** Displays current state in `s` status output (Core 0, same superloop)

Since all consumers are on Core 0 in the same superloop, no cross-core synchronization is needed for FlightState.

---

## 13. Integration Points (PRELIMINARY)

### Inputs

| Source | Data | Access Method |
|--------|------|--------------|
| FusedState | Attitude, position, velocity, altitude | Seqlock read (Core 1 → Core 0) |
| CLI / Button | Commands (arm, disarm, abort, reset) | Polled in command handler |
| Mission Profile | Phase definitions, transitions, guards | `const MissionProfile*` loaded at init |
| Confidence Gate (IVP-48) | Confidence flag for pyro gating | Read from confidence output struct |
| Watchdog Recovery (IVP-49) | Recovery boot flag, previous flight phase | Read from scratch registers at boot |

### Outputs

| Consumer | Data | Access Method |
|----------|------|--------------|
| Logger | FlightState, flight events | Direct struct read (Core 0) |
| Telemetry | FlightState, state changes | Direct struct read (Core 0) |
| CLI | Current phase, stats | Direct struct read (Core 0) |
| Pyro channels | Fire commands | GPIO via action executor |
| NeoPixel | LED patterns | Via existing WS2812 driver |

### Profile Interface (PRELIMINARY)

```cpp
// The Flight Director takes a Mission Profile at initialization.
// For Phase 5: baked-in rocket profile as const struct.
// Future: loaded from flash/SD.
class FlightDirector {
public:
    void init(const MissionProfile* profile);
    void tick();  // Called at 100Hz from run_flight_director()

    const FlightState& state() const;

private:
    const MissionProfile* m_profile;
    FlightState m_state;
    StateHandler m_current_handler;
    // ... condition evaluator state, sustain timers, etc.
};
```

### Superloop Integration (PRELIMINARY)

```cpp
// In main.cpp Core 0 superloop
static FlightDirector g_director;

void init_application() {
    g_director.init(&kRocketProfile);  // baked-in for Phase 5
}

void run_flight_director() {
    g_director.tick();
}
```

---

## Appendix: Source Documents

- `docs/flight_director/RESEARCH.md` — Research, prior art, council decisions
- `docs/SAD.md` Section 6 — State machine topology, event-condition-action examples
- `docs/IVP.md` IVP-49 through IVP-54 — Stage 6 implementation steps
- `AGENT_WHITEBOARD.md` — IVP-49 watchdog recovery policy details
- `docs/mission_profiles/MISSION_PROFILES.md` — Mission Profile format (separate doc)
