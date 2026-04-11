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

---

## IVP-118 Verification Amendment (2026-04-10)

*Added during Stage 14 final audit. Original document dated 2026-04-10 (IVP-113).
This section records the runtime subscriber audit and refines the safety
classification. Content above this line was NOT modified — amendment only.*

### Subscriber Routing Audit (via GDB memory inspection)

Verified on the running vehicle build at Stage 14 completion. All subscriber
bitmaps in `g_subscrList[]` match the plan:

| Signal | Subscribers | Bitmap |
|--------|-------------|--------|
| `SIG_PHASE_CHANGE` | Logger, Notify, HealthMonitor | 0x38 (prios 4,5,6) |
| `SIG_LED_PATTERN` | LedEngine (sole) | 0x02 (prio 2) |
| `SIG_BEACON_ACTIVE` | Notify (sole) | 0x10 (prio 5) |
| `SIG_RADIO_STATUS` | Notify (sole; NOT LedEngine) | 0x10 (prio 5) |
| `SIG_HEALTH_STATUS` | Telemetry, Logger, Notify (NOT LedEngine) | 0x1C (prios 3,4,5) |

### Flight Safety Analysis (Refinement of Earlier Statement)

The "Safety Analysis" section earlier in this document understates the role
of AO_Notify. AO_Notify is **Flight-Support** classification: it runs during
flight and is the primary display path for fault indication. The earlier
statement that it "cannot affect flight safety" is only partially correct —
AO_Notify cannot *cause* a flight safety failure, but it can *fail to
indicate* one, which is a safety concern in its own right.

The precise architectural boundary is:

- **Subscribes to** state-producing signals from flight-critical AOs
  (SIG_PHASE_CHANGE, SIG_HEALTH_STATUS, SIG_BEACON_ACTIVE) and status
  signals (SIG_RADIO_STATUS).
- **Publishes** only SIG_LED_PATTERN (via the LED backend), which has
  exactly one subscriber: AO_LedEngine. LedEngine is a display driver
  with no outputs into flight state.
- **Does not modify** flight state, guard conditions, pyro logic, or
  calibration data.
- **Does not publish** any signal consumed by AO_FlightDirector,
  AO_HealthMonitor, or AO_Logger's pyro/phase tracking path.

The SPIN formal model (11 properties, all passing) has no representation
of AO_Notify because AO_Notify has no code path that can cause a
flight-safety-relevant state transition. The model remains unchanged and
the 11/11 pass count is preserved. This is a property of the architectural
boundary above, not a claim that AO_Notify is inconsequential.

### Defense-in-Depth for Display Failure

Because AO_Notify is the primary display path for fault indication, a
silent AO_Notify crash would deny the pilot visual warnings before the
hardware watchdog eventually resets the system. Mitigation (Council A1):
AO_LedEngine retains a local Core 1 vitality check as a fallback that
fires the stall pattern even if AO_Notify or AO_HealthMonitor have
crashed. The hardware watchdog (5s timeout) is the ultimate safety net.

Core 1 vitality is therefore checked in three places:
1. **Primary:** AO_HealthMonitor at 10Hz — emits via `kHealthCore1Ok` bit
   in SIG_HEALTH_STATUS (IVP-117).
2. **Secondary:** AO_Notify decodes that bit and sets `FaultIntent::kCore1Stall`
   if clear.
3. **Tertiary / fallback:** AO_LedEngine local check — runs in its own
   tick handler, sets the fault layer directly if Core 1 stalls, bypasses
   AO_Notify entirely.

This redundancy is specifically for visual fault indication and is not
reflected in the SPIN model (display-only layer). Whether this pattern
should be extended to other critical checks is tracked on the whiteboard
(IVP-118 notes) for post-Stage-14 evaluation.

### Completion Record

Stage 14 completed: 2026-04-10. All six IVPs (IVP-113 through IVP-118)
passed their gates. Plan: `.claude/plans/encapsulated-pondering-sparkle.md`.
Latent QP use-after-free bug discovered and fixed during IVP-117 HW verify
(see LESSONS_LEARNED.md Entry 35).
