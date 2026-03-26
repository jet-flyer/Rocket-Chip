# Active Object Migration Plan (Stage 9)

**Status:** PRELIMINARY — high-level outline only. Requires detailed plan
mode + council review before implementation begins in Stage 9.
**Prerequisite:** IVP-75 compile gate (QF+QV linked, BSP written)
**Last Updated:** 2026-03-26 (IVP-75, initial draft)

## Overview

Stage 9 migrates the current superloop architecture to QP/C Active Objects
(AOs). Each subsystem becomes an AO with its own event queue, receiving
events via publish-subscribe. The QV cooperative scheduler replaces the
manual tick-function dispatcher in `main()`.

The Flight Director HSM already uses QEP — it transitions from being
dispatched by the superloop to being a proper QP Active Object.

## Active Object Boundaries

| AO | Priority | Responsibility | Current Location |
|----|----------|---------------|-----------------|
| FlightDirector | Highest | HSM dispatch, guard evaluation, commands | `flight_director_tick()` in main.cpp |
| LedEngine | High | NeoPixel state machine, all LED patterns | `core1_neopixel_update()` in main.cpp |
| Logger | Medium | PCM frame encoding, ring buffer, flash flush | `logging_tick()` in main.cpp |
| Telemetry | Medium | CCSDS encode, LoRa TX, MAVLink | `telemetry_tick()` in main.cpp |
| ErrorHandler | Low | Watchdog feed, health monitoring, recovery | `watchdog_kick_tick()` in main.cpp |

## Event Catalog

| Event | Publisher | Subscribers |
|-------|-----------|------------|
| SIG_SENSOR_DATA | Core 1 (via FIFO) | FlightDirector, Logger |
| SIG_PHASE_CHANGE | FlightDirector | LedEngine, Logger, Telemetry |
| SIG_PYRO_INTENT | FlightDirector | Logger (records), ErrorHandler (validates) |
| SIG_LED_OVERRIDE | CLI (calibration) | LedEngine |
| SIG_LOG_FRAME | Logger | Telemetry (if TX enabled) |
| SIG_HEALTH_CHECK | Timer | ErrorHandler |
| SIG_CLI_COMMAND | CLI task | FlightDirector |

## Dual-Core Constraints

- **Core 0:** QV scheduler runs all AOs (cooperative, single stack)
- **Core 1:** Sensor loop stays as-is (not an AO — bare-metal for deterministic timing)
- **Cross-core:** Sensor data passes via existing seqlock, then Core 0 posts events to AOs
- **QV_onIdle:** WFI for power savings when no events pending

## Migration Steps (IVP-76 through IVP-80)

### IVP-76: QF+QV BSP Activation
- Wire `QF_init()` and `QF_run()` into main.cpp (after existing init)
- `QV_onIdle()` calls WFI
- `QF_onStartup()` configures alarm-based tick source
- Superloop still runs in parallel (dual path during migration)

### IVP-77: LED Engine Active Object
- Extract `core1_neopixel_update()` logic into `LedEngine` AO
- Receives `SIG_PHASE_CHANGE` and `SIG_LED_OVERRIDE` events
- Internal HSM maps flight phase → LED pattern
- Consolidates all NeoPixel overlay code (cal, RX, flight) in one place

### IVP-78: Flight Director Active Object
- `FlightDirector` becomes a proper `QActive` subclass
- Timer events replace manual tick counting
- Guard evaluation triggered by `SIG_SENSOR_DATA` events
- CLI commands posted as events (decoupled from direct dispatch)

### IVP-79: Logger + Telemetry Active Objects
- Logger AO receives sensor data + phase change events
- Telemetry AO receives encoded frames from Logger
- Flash flush handled by Logger AO's deferred event mechanism

### IVP-80: Superloop Removal
- Remove tick-function dispatcher from `main()`
- `main()` becomes: init → `QF_init()` → start AOs → `QF_run()` (never returns)
- Verify all functionality via bench sim + soak test
- Profile CPU usage: QV idle percentage

## Key Decisions

- **QV (cooperative), not QK (preemptive):** Single stack, simpler debugging,
  no priority inversion. Adequate for current event rates (<1kHz).
- **Core 1 stays bare-metal:** Sensor sampling at 1kHz requires deterministic
  timing that AO event dispatch can't guarantee.
- **Static event allocation:** All events are stack-allocated or static
  (`QF_MAX_EPOOL = 0`). No dynamic allocation after init.

## Files

- `lib/qep/` — QP/C 8.1.3 QEP+QF+QV (vendored, IVP-67/75)
- `lib/qep/bsp_qv.c` — BSP callbacks (IVP-75)
- `src/active_objects/` — New directory for AO implementations (Stage 9)
