# Mission Profiles

**Status:** PRELIMINARY STUB — To be fleshed out in a dedicated session
**For:** RocketChip (RP2350 bare-metal Pico SDK, dual-core AMP)
**Date:** 2026-02-14

> **This document is a placeholder.** It establishes the Mission Profile as a distinct concept with its own documentation space. The struct definitions, rocket profile instance, and future expansion details will be developed in a separate session.

---

## What Is a Mission Profile?

A **Mission Profile** is the configuration data that defines what a specific vehicle type does: which flight phases exist, what transitions connect them, what sensor thresholds trigger transitions, what actions fire on entry/exit, which pyro channels map where, and what gets logged.

A rocket profile is different from a balloon profile is different from a vehicle profile. The Mission Profile is the configurable "what." The Flight Director is the runtime "how."

**The Flight Director follows the Mission Profile.**

### Phase 5 Approach

For Phase 5, a single rocket Mission Profile is baked into firmware as a `const` struct. No profile selection, no flash loading, no runtime parsing. The `FlightDirector` loads it at boot. It works.

The struct layout is designed for future expansion — no pointers (use indices so it's position-independent when eventually loaded from flash), stable field order, reserved fields.

---

## Design Decisions

The following decisions were made during the council-reviewed research phase. See `docs/flight_director/RESEARCH.md` for full rationale and prior art survey.

1. **Profiles are data structures, not code.** The SAD originally planned `Mission_Rocket.cpp`, `Mission_HPR.cpp` as compiled code files. The decided direction is profiles as `const` structs — same firmware binary, different behavior via configuration.

2. **FEMA approach: minimize failure modes.** One hardcoded profile for Phase 5. No profile switching infrastructure until post-MVP. The only things that can break are the profile data (compile-time validated) and the Flight Director logic.

3. **No pointers in profile structs.** Use indices and IDs so the layout is position-independent for future flash/SD serialization.

4. **Guard functions are compiled, thresholds are data.** Each guard is a C function indexed by ID. Comparison direction, threshold value, and sustain time are tunable per-profile in the `ConditionDef` without recompilation. Only new guard *types* require code changes.

5. **Two-tier pre-arm checks.** Default safety checks always run (profile cannot bypass). Profile adds vehicle-specific checks on top.

6. **Abort actions are profile-defined per source phase.** Universal default: safe all outputs, go neutral, beacon. Profile overlays phase-specific abort responses.

---

## Struct Definitions (PRELIMINARY — To Be Expanded)

> **All struct layouts below are preliminary sketches from the research phase.** Field names, types, sizes, and organization will be finalized during implementation.

### MissionProfile

```cpp
// PRELIMINARY
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
```

### PhaseDefinition

```cpp
// PRELIMINARY
struct PhaseDefinition {
    uint8_t id;
    char name[16];
    uint8_t parent_phase;     // for HSM hierarchy (0 = top-level)
    uint32_t timeout_ms;      // max time in this phase (0 = no limit)
    uint8_t log_rate_hz;      // logging rate during this phase
};
```

### TransitionRule

```cpp
// PRELIMINARY
struct TransitionRule {
    uint8_t from_phase;
    uint8_t to_phase;
    uint8_t guard_id;         // index into guard function table
    uint8_t num_conditions;
    ConditionDef conditions[MAX_CONDITIONS_PER_TRANSITION];
    CombineLogic logic;       // AND, OR, PRIMARY_PLUS_TIMEOUT
};
```

### ConditionDef

```cpp
// PRELIMINARY
struct ConditionDef {
    SensorField field;        // which sensor value to check
    CompareOp op;             // GT, LT, GE, LE
    float threshold;
    uint32_t sustain_ms;      // must be true for this long
};
```

### ActionBinding, PyroMapping, ServoMapping, DetectionConfig

*To be defined during implementation.*

---

## Baked-In Rocket Profile (PRELIMINARY)

The Phase 5 rocket profile defines:

```
IDLE → ARMED → BOOST → COAST → APOGEE → DROGUE_DESCENT → MAIN_DESCENT → LANDED
```

Key thresholds (all PRELIMINARY — sourced from research doc and SparkyVT validation):
- Launch detection: accel > 2.5g sustained 50ms
- Burnout detection: accel < 1.0g after launch
- Apogee: Kalman velocity zero-crossing, Mach-corrected baro
- Drogue deploy: on apogee, pyro channel 1
- Main deploy: altitude < 200m AGL, pyro channel 2
- Landing: velocity < 0.5 m/s sustained 5s

Full profile instance definition will be developed when this document is fleshed out.

---

## Future Expansion (Post-MVP)

These capabilities are planned but not implemented in Phase 5:

- **Profile selection via CLI** before arming (choose from compiled-in options)
- **Flash/SD storage format** — same struct, serialized as binary blob
- **Validation on load** — check transition targets exist, no orphaned phases, valid guard IDs, reachability analysis
- **CLI profile editor** — modify thresholds and mappings without recompilation
- **GCS profile upload** — MAVLink parameter protocol or custom binary transfer
- **GCS-side abort safety validation** — cross-reference abort actions against flight regimes before upload

### Example: Balloon Profile (Illustrates Platform Flexibility)

Same Flight Director, completely different Mission Profile:

```
IDLE → ARMED → ASCENT → FLOAT → DESCENT → LANDED
```

- Launch: baro altitude increasing > 2 m/min sustained 30s (no accel spike)
- Float: altitude rate < 0.5 m/min sustained 60s
- Cutdown: altitude > ceiling OR timer expires, pyro channel 1
- Landing: GPS ground speed < 0.5 m/s sustained 30s

No Flight Director code changes needed — just a different `const MissionProfile`.

---

## Source Documents

- `docs/flight_director/RESEARCH.md` — Full research, prior art, council decisions
- `docs/flight_director/FLIGHT_DIRECTOR_DESIGN.md` — Flight Director runtime design spec
- `docs/SAD.md` Section 6 — State machine topology
- `docs/IVP.md` Stage 6, IVP-54 — Mission Configuration step
