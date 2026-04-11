# Notification Engine Contract

**Status:** Active (Stage 14, IVP-113)
**Council Review:** NASA/JPL, ArduPilot Core Contributor, Embedded Systems Professor, Advanced Hobbyist Rocketeer — unanimous GO
**Plan:** `.claude/plans/encapsulated-pondering-sparkle.md`

---

## Purpose

Centralized intent-to-display routing. Subsystems report what is happening (intent); the notification engine decides how to display it across output backends.

## Architecture

AO_Notify is the sole mediator between state producers and display consumers. No subsystem calls AO_LedEngine directly.

```
Producers → AO_Notify (intent state + resolver) → Backends
```

**Code classification:** Flight-Support. Runs during flight but output-only — cannot influence flight state transitions, guard conditions, or pyro logic.

---

## Intent Taxonomy

Five categories, each with a per-category `enum class` (Council A3: compile-time category enforcement). One active intent per category. Zero-init = no active intents.

Defined in `include/rocketchip/notify_intents.h`.

---

## Category Priority Order

Resolver iterates categories in this order. First non-kNone category wins.

| Priority | Category | Source | Rationale |
|----------|----------|--------|-----------|
| 1 (highest) | Fault | SIG_HEALTH_STATUS, Core1 vitality | Safety faults override everything |
| 2 | Calibration | AO_Notify_post_cal_intent (CLI) | Cal overlay must be visible during ground ops |
| 3 | Flight | SIG_PHASE_CHANGE, SIG_BEACON_ACTIVE | Phase indication during flight |
| 4 | Radio | SIG_RADIO_STATUS | Link quality when no higher layer active |
| 5 | Sensor | Seqlock read (GPS, ESKF) | Default status when idle |
| 6 (lowest) | Idle | Fallback | Blue blink when nothing else active |

**Calibration above Flight:** Calibration is promoted above Flight Phase in the resolver. This differs from the pre-Stage-14 LED layer stack where Flight (layer 1) was above Calibration (layer 2). Rationale: calibration only runs in IDLE, so the change has no runtime effect, but it structurally enforces that the cal overlay is always visible during ground ops. If in-flight calibration is ever added, this ordering prevents the flight phase pattern from suppressing the cal overlay.

---

## Backend Interface

Direct function calls — no function pointer vtable (JSF AV Rule 170). Platform selection via `#if` compile guards.

```cpp
void notify_backend_led_update(const NotifyState& state);    // Always present
void notify_backend_audio_update(const NotifyState& state);  // Stub (future I2S DAC)
```

---

## AP_Notify Comparison

| Aspect | ArduPilot AP_Notify | RocketChip AO_Notify |
|--------|--------------------|--------------------|
| State model | Flat `_flags` struct (~50 bools) | Per-category typed enums (5 categories) |
| Priority resolution | Each backend decides | Resolver in AO_Notify (single source of truth) |
| Backend interface | Virtual `NotifyDevice` class | Direct function calls (JSF AV Rule 170) |
| Tone format | RTTTL-like strings (`"MFT200L8 O4CEG"`) | Same format (data defined, parser deferred) |

---

## Intent-to-LED Visual Reference

| Intent | LED Mode | Color | Pattern Code |
|--------|----------|-------|-------------|
| **Faults (highest priority)** | | | |
| FaultIntent::kCore1Stall | SOLID | Magenta | kFaultCore1Stall (46) |
| FaultIntent::kSafeMode | SOLID | Red | kFaultSafeMode (45) |
| FaultIntent::kImuFail | BLINK_FAST | Red | kFaultImuFail (44) |
| FaultIntent::kEskfFail | BLINK | Red | kFaultEskfFail (43) |
| FaultIntent::kBaroFail | BLINK_FAST | Orange | kFaultBaroFail (42) |
| FaultIntent::kPioWdt | SOLID | Orange | kFaultPioWdt (41) |
| **Calibration** | | | |
| CalIntent::kGyro | BREATHE | Blue | kCalGyro (1) |
| CalIntent::kLevel | BREATHE | Blue | kCalLevel (2) |
| CalIntent::kBaro | BREATHE | Cyan | kCalBaro (3) |
| CalIntent::kAccelWait | BLINK | Yellow | kCalAccelWait (4) |
| CalIntent::kAccelSample | SOLID | Yellow | kCalAccelSample (5) |
| CalIntent::kMag | RAINBOW | White | kCalMag (6) |
| CalIntent::kSuccess | SOLID | Green | kCalSuccess (7) |
| CalIntent::kFail | BLINK_FAST | Red | kCalFail (8) |
| **Flight Phase** | | | |
| PhaseIntent::kBeacon | BLINK | White | kFdBeacon (27) |
| PhaseIntent::kAbort | BLINK_FAST | Red | kFdAbort (26) |
| PhaseIntent::kLanded | BLINK | Green | kFdLanded (25) |
| PhaseIntent::kMain | BLINK | Red | kFdMain (24) |
| PhaseIntent::kDrogue | BLINK | Red | kFdDrogue (23) |
| PhaseIntent::kCoast | SOLID | Yellow | kFdCoast (22) |
| PhaseIntent::kBoost | SOLID | Red | kFdBoost (21) |
| PhaseIntent::kArmed | SOLID | Orange | kFdArmed (20) |
| **Radio** | | | |
| RadioIntent::kLost | BLINK_FAST | Red | kRxLost (11) |
| RadioIntent::kGap | BLINK | Yellow | kRxGap (10) |
| RadioIntent::kReceiving | SOLID | Green | kRxReceiving (9) |
| **Sensor Status** | | | |
| SensorIntent::kTimeout | SOLID | Magenta | kSensorTimeout (36) |
| SensorIntent::kEskfInit | BLINK_FAST | Red | kSensorEskfInit (30) |
| SensorIntent::kGps3d | SOLID | Green | kSensorGps3d (31) |
| SensorIntent::kGps2d | BLINK | Green | kSensorGps2d (32) |
| SensorIntent::kGpsSearch | BLINK | Yellow | kSensorGpsSearch (33) |
| SensorIntent::kGpsNoNmea | BLINK_FAST | Cyan | kSensorGpsNoNmea (34) |
| SensorIntent::kNoGps | BLINK | Blue | kSensorNoGps (35) |
| **Idle (fallback)** | BLINK | Blue | kSensorNoGps (35) |

---

## Safety Analysis

AO_Notify is output-only. It subscribes to signals published by other AOs and dispatches to display backends. It does not:
- Publish any signal consumed by flight-critical AOs
- Modify flight state or guard conditions
- Influence pyro firing logic
- Read or write calibration data

The SPIN formal model is unchanged by Stage 14. AO_Notify has no representation in the safety model because it cannot affect flight safety.

---

## Council Amendments

| ID | Amendment | Source |
|----|-----------|--------|
| A1 | Core1 vitality check retained in AO_LedEngine as defense-in-depth fallback | NASA/JPL |
| A2 | AO_Telemetry and AO_Logger SIG_HEALTH_STATUS subscriptions preserved | ArduPilot |
| A3 | Per-category typed enums for compile-time safety | Professor |
| A4 | HW soak must include ARM/DISARM cycle and beacon regression check | Rocketeer |
| P1 | Compile-time AO priority uniqueness check in main.cpp | NASA/JPL (plan review) |
| P2 | Intent-to-LED visual reference table in this document | Rocketeer (plan review) |
