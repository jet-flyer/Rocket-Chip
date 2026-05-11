# Notification Engine Contract

**Status:** Active (Stage 14, IVP-113)
**Council Review:** NASA/JPL, ArduPilot Core Contributor, Embedded Systems Professor, Advanced Hobbyist Rocketeer — unanimous GO
**Plan:** `.claude/plans/encapsulated-pondering-sparkle.md`

---

> **Correction note (2026-05-07):** Citations of "JSF AV Rule 170" in this
> document (as rationale for "direct function calls — no function pointer
> vtable") are based on a misreading. Rule 170's actual wording is "More
> than 2 levels of pointer indirection shall not be used" — it governs
> pointer-indirection depth, not function pointer usage. Function pointers
> are not prohibited by JSF; Rule 176 requires typedef-declared function
> pointers, which would also have been acceptable. The engineering choice
> (direct function calls, no vtable) stands on its own merits:
> compile-time dispatch, no runtime indirection, simpler static analysis.
> The standards-compliance framing was wrong. Document body is left
> unedited as historical record; this correction note is the supersession
> per `.claude/SESSION_CHECKLIST.md` Trigger-Driven Doc Edits convention.

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

---

## Stage L Extensions (IVP-L1 … IVP-L7, 2026-04-18)

Stage L polished the LED+notify system with AP-parity color nudges, a
beacon-overlay composition layer, a pre-arm-fail visual, a boot-init
rainbow, and vehicle/GCS manual beacon triggers. Plan:
`.claude/plans/shimmering-twirling-thimble.md`.

### Beacon Overlay Composition

Two orthogonal flags on `NotifyState` (added Stage L IVP-L2):

- `beacon_auto`   — set by automatic triggers (FD MAIN_DESCENT backstop,
  `kSetBeacon` action). Preserves the state color via the resolver
  overlay so a recovery crew sees good / fault-class / safe-mode from
  distance.
- `beacon_manual` — set by CLI `b` (find-me) or GCS `MAV_CMD_USER_1`.
  Forces pure-white 2Hz regardless of state. Wins over `beacon_auto`.

Both clear on `SIG_PHASE_CHANGE` out of `{kLanded, kAbort}`. The
overlay runs in `apply_beacon_overlay()` after the normal priority loop
in `resolve_led_pattern()`. Composition table:

| Base resolved pattern  | + beacon_auto                  | + beacon_manual |
|------------------------|--------------------------------|-----------------|
| kFdLanded              | kFdLandedBeacon (green+white)  | kFdBeacon (pure white) |
| kFdAbort               | kFdAbortBeacon  (red+white)    | kFdBeacon |
| kFaultImuFail          | kFaultImuFailBeacon (red+white)| kFdBeacon |
| kFaultEskfFail         | kFaultEskfFailBeacon           | kFdBeacon |
| kFaultBaroFail         | kFaultBaroFailBeacon (orange+w)| kFdBeacon |
| kFaultPioWdt           | kFaultPioWdtBeacon             | kFdBeacon |
| kFaultCore1Stall       | kFaultCore1StallBeacon (mag+w) | kFdBeacon |
| kFaultSafeMode         | (unchanged — already blue+white) | kFdBeacon |
| anything else          | kFdBeacon (pure white fallback)| kFdBeacon |

**Rationale for preserving fault color under beacon_auto:** a recovery
crew walking up to the vehicle wants to know at a glance whether the
vehicle is good, faulted, or in safe-mode. State-color retention is
more diagnostically useful than pure white when the beacon was
triggered automatically (the vehicle decided on its own it needed
locating). Under `beacon_manual`, the operator explicitly requested
maximum visibility — they already know the state from telemetry and
want pure white for range.

**Fault diagnostics not in LED when beacon_manual wins:** intentional
loss. Full fault state remains available via serial CLI, MAVLink
telemetry, and the health-status bitfield. LED is a locator, not a
debugger.

### PhaseIntent Extensions (Stage L)

- `kPreArmFail = 10` — ARM rejection visual. Yellow double-flash
  (100/100/100/700 ms, AP parity). Auto-clears after 99 ticks
  (~3 s at 33 Hz). Each repost of `SIG_NOTIFY_PREARM_FAIL` resets the
  counter to full (per JPL council 2026-04-18) — rapid-fire rejections
  refresh the window rather than decrement. Phase transitions
  immediately invalidate the visual. Pure helper
  `prearm_fail_tick_next()` in `include/rocketchip/prearm_fail_ticks.h`
  is host-testable separately.
- `kInit = 11` — boot-init warmup visual. Rainbow (shares
  `WS2812_MODE_RAINBOW` with mag-cal; time-separated by construction —
  mag-cal is operator action, kInit runs once at boot). Auto-clears
  when all three gates satisfied: (a) minimum 99-tick visibility window
  (~3 s at 33Hz) elapsed, (b) `eskf_runner_is_initialized()`, and
  (c) `seqlock.imu_read_count > 0`. The min-ticks gate ensures the
  rainbow is visible even on warm resets where ESKF+IMU are ready
  before AO_Notify's first tick.

### New Signals (Stage L)

- `SIG_BEACON_MANUAL` (28) — added to `RcSignal` enum. Bumps
  `SIG_AO_MAX` from 28 to 29. Published by:
  - `cmd_findme_beacon()` on vehicle (local CLI `b` key)
  - `handle_rx_mavlink_msg()` on vehicle when `MAV_CMD_USER_1`
    (id 31010) arrives over radio
  - Station-side `cmd_findme_beacon()` role-gates to send
    `MAV_CMD_USER_1` over radio via `AO_Telemetry_send_tracked_command`
    (reuses IVP-122 ACK ladder)
- `SIG_BEACON_ACTIVE` (existing) — semantics **changed**: now sets
  `NotifyState.beacon_auto` instead of overwriting `state.phase = kBeacon`.
  Publishers unchanged (FD `kSetBeacon` action + MAIN_DESCENT backstop).
- `SIG_NOTIFY_PREARM_FAIL` (private to AO_Notify) — direct post from
  `AO_Notify_post_prearm_fail()` shim, called by
  `dispatch_flight_command()` on ARM rejection.

### AP-Parity Color Swaps (Stage L IVP-L1)

Deliberate alignment with ArduPilot LED defaults where semantics are
shared:

| Element              | Before IVP-L1          | After IVP-L1         | AP parity |
|----------------------|------------------------|----------------------|-----------|
| kFdArmed             | Orange solid           | Red solid            | ✅ match  |
| kCalGyro             | Blue breathe           | Yellow blink         | ✅ match  |
| kCalLevel            | Blue breathe           | Yellow blink         | ✅ match  |
| kCalMag              | Rainbow                | Rainbow (unchanged)  | ✅ AP also uses rainbow for compass cal |
| kFdPreArmFail (new)  | —                      | Yellow double-flash  | ✅ match  |
| kFdBootInit (new)    | —                      | Rainbow              | ✅ match  |
| kFaultSafeMode       | Red solid              | Blue+white 2Hz alt   | 🎯 rocketry-specific (locator-ready) |

### AP-Parity Comparison Table (Stage L)

Single source of truth for "where are we vs AP": see gap analysis in
plan file §8. Summary: all shared AP concepts have matching colors;
rocketry-specific phases (BOOST/COAST/DROGUE/MAIN/LANDED/ABORT/BEACON)
are deliberate deviations with no AP analog. GPS-glitch intent not yet
implemented (deferred).

### Driver-Layer Modes Added (Stage L IVP-L1)

- `WS2812_MODE_ALTERNATE` — two-color toggle at a configurable
  half-period (default 250 ms each = 2 Hz visual). Used by all
  beacon-overlay patterns. Edge-triggered in `update_alternate()` to
  avoid hammering the PIO FIFO.
- `WS2812_MODE_DOUBLE_FLASH` — hardcoded 100/100/100/700 ms pulse
  pattern for pre-arm-fail. AP-parity shape.

### Station/Vehicle LED Role Divergence

Station runs no `AO_LedEngine` — the Fruit Jam's multi-LED strip is
owned by `AO_Radio` as an RSSI bar (see LL Entry 32 for PIO-contention
rationale). Vehicle uses `AO_LedEngine` for flight-state visualization.
This is intentional role-specific UX: station is a ground device
showing radio link health; vehicle is a flight device showing phase.
Council-reviewed 2026-04-18.

When `cmd_findme_beacon()` runs on station, it's role-gated to skip
the local publish path (no AO_Notify to receive it) and instead send
`MAV_CMD_USER_1` over radio to the vehicle. The vehicle's
`handle_rx_mavlink_msg()` MAVLink command switch maps `MAV_CMD_USER_1`
to `SIG_BEACON_MANUAL` publish — so the visual effect is identical
from the operator's perspective whether the beacon was triggered
locally (vehicle CLI `b`) or over radio (station `b`).

### Completion Record (Stage L)

Stage L completed: 2026-04-18. Seven IVPs (IVP-L1 … IVP-L7) + one
full-tree JSF-AV-rule-1 sweep + one SESSION_CHECKLIST update. Plan:
`.claude/plans/shimmering-twirling-thimble.md`. Council review (4
panels) approved before implementation.

Station→vehicle end-to-end beacon roundtrip not verified during this
stage: the station TX path sends correctly but vehicle radio RX
receives 0 packets due to the pre-existing RadioScheduler TX-window
sync issue (quantified 2026-04-16 at 6.7% first-try ACK rate, IVP-132a.5).
This is tracked as a dedicated future stage — the Stage L code path is
architecturally sound and verified via GDB `set var beacon_manual=true`
showing the resolver + LED driver chain works correctly.
