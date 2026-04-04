# Active Object Architecture

**Status:** Active — update as modules are extracted
**Last Updated:** 2026-04-04
**Council Reviewed:** 2026-04-04 (5 personas: ArduPilot, JPL, Professor, Student, Hobbyist)
**Plan:** `.claude/plans/idempotent-cooking-metcalfe.md`

---

## Overview

RocketChip uses the QP/C QV cooperative scheduler for event-driven subsystem management. Each Active Object (AO) owns its state, communicates via typed events, and runs to completion on each tick. Modules are non-AO code with clean interfaces, called from AOs or the idle bridge.

**QV is cooperative (non-preemptive).** AO handlers must complete within one tick period. No blocking calls inside handlers. Read-only accessors on AO/module state are safe under QV but would require protection under QK (preemptive).

**QF base tick rate:** 100Hz (10ms per tick). AOs derive their rates via timer period.

---

## Active Object Inventory

| AO | File | Rate | Prio | Queue | Owns | Publishes | Subscribes |
|----|------|------|------|-------|------|-----------|------------|
| AO_Radio | `ao_radio.cpp` | 100Hz | 8 | 32 | rfm95w_t, RadioScheduler, RadioAoState | SIG_RADIO_RX, SIG_RADIO_STATUS | SIG_RADIO_TX |
| AO_FlightDirector | `ao_flight_director.cpp` | 100Hz | 6 | 32 | FlightDirector HSM, guard eval, Go/No-Go, PIO timer hooks | SIG_PHASE_CHANGE, SIG_PYRO_INTENT, SIG_LED_PATTERN, SIG_HEALTH_STATUS | SIG_SENSOR_DATA, SIG_CLI_COMMAND |
| AO_Logger | `ao_logger.cpp` | 50Hz | 5 | 32 | RingBuffer, LogDecimator, FlightTable, FusedState builder, SRAM ring | (none) | SIG_SENSOR_DATA, SIG_PHASE_CHANGE, SIG_PYRO_INTENT |
| AO_Telemetry | `ao_telemetry.cpp` | 10Hz | 4 | 8 | CCSDS/MAVLink encode, TelemetryState snapshot, RX decode | SIG_RADIO_TX | SIG_RADIO_RX, SIG_SENSOR_DATA |
| AO_LedEngine | `ao_led_engine.cpp` | 33Hz | 3 | 8 | ws2812 driver, priority layer table, animation state, pattern constants | (none) | SIG_LED_PATTERN, SIG_LED_OVERRIDE, SIG_RADIO_STATUS, SIG_SENSOR_DATA, SIG_PHASE_CHANGE, SIG_HEALTH_STATUS |
| AO_RCOS | `cli/ao_rcos.cpp` | 20Hz | 2 | 16 | CLI output mode, ANSI dashboard, key dispatch | SIG_CLI_COMMAND, SIG_LED_OVERRIDE | (none) |
| AO_Blinker | `ao_blinker.cpp` | 1Hz | 1 | 4 | Board LED heartbeat | (none) | (none) |

*AO_Counter (disabled): jitter measurement diagnostic. Priority 7 reserved for future use.*

---

## Module Inventory

| Module | File | Interface | Called By | Owns |
|--------|------|-----------|----------|------|
| eskf_runner | `src/fusion/eskf_runner.cpp` | `eskf_runner_tick()`, `eskf_runner_get_*()` | Idle bridge (200Hz effective) | ESKF, Mahony, ConfidenceGate, bench stats, epoch, state buffer |
| health_monitor | `src/safety/health_monitor.cpp` | `health_monitor_tick()`, `health_monitor_get_state()` | AO_FlightDirector (10Hz divider) | Health flags, error counters, escalation state, Go/No-Go |
| sensor_core1 | `src/core1/sensor_core1.cpp` | `core1_entry()` | main.cpp (multicore_launch) | Core 1 sensor loop, seqlock write, IMU/baro/GPS reads |
| sensor_seqlock | `include/rocketchip/sensor_seqlock.h` | `seqlock_read()`, `seqlock_write()`, types | sensor_core1 (write), eskf_runner + AOs (read) | shared_sensor_data_t, cross-core atomics |
| led_patterns | `include/rocketchip/led_patterns.h` | Pattern constants, LedLayer enum | AO_LedEngine, rc_os.cpp, flight_actions.h | kCalNeo*, kRxNeo*, kFdNeo* constants |
| cal_hooks | `src/calibration/cal_hooks.cpp` | `cal_pre_hook()`, `cal_post_hook()` | rc_os.cpp callbacks | Cross-core I2C pause/resume, mag read |
| cli_commands | `src/cli/cli_commands.cpp` | `handle_unhandled_key()`, `print_*()`, `cmd_*()` | AO_RCOS, rc_os.cpp | CLI command handlers, display formatters |

---

## Event Flow

```
  Core 1                    Core 0 (QV Scheduler + Idle Bridge)
  --------                  ------------------------------------
  sensor_core1              
    |                       
    | seqlock write         
    v                       
  [seqlock] ----read----> eskf_runner (idle bridge, 200Hz)
                              |
                    publish SIG_SENSOR_DATA
                              |
              +---------------+----------------+
              |               |                |
              v               v                v
        AO_FlightDir    AO_Logger       AO_LedEngine
        (100Hz, P6)     (50Hz, P5)      (33Hz, P3)
              |               |                ^
     SIG_PHASE_CHANGE   writes ring      SIG_LED_PATTERN
     SIG_LED_PATTERN    SIG_PYRO_INTENT  SIG_RADIO_STATUS
              |                          SIG_PHASE_CHANGE
              |                          SIG_HEALTH_STATUS
              |                                ^
              |  health_monitor_tick()          |
              |  (10Hz divider inside FD)       |
              |       |                        |
              |       +-- SIG_HEALTH_STATUS ---+
              |                                 
              v                                
        AO_Telemetry ----SIG_RADIO_TX----> AO_Radio
        (10Hz, P4)   <---SIG_RADIO_RX---- (100Hz, P8)
                                               |
                                        SIG_RADIO_STATUS
                                               |
                                               v
                                         AO_LedEngine

  Idle Bridge (runs when all AO queues empty):
    1. watchdog_kick_tick()     -- permanent
    2. eskf_runner_tick()       -- fusion + publish SIG_SENSOR_DATA
    3. rc_os_update()           -- blocking cal wizards only (conditional)
    4. __wfi()                  -- sleep until next interrupt

  AO_RCOS (20Hz, P2, CLI)
    reads: all module/AO public APIs for display
    posts: SIG_CLI_COMMAND, SIG_LED_OVERRIDE
```

---

## LED Engine Priority Layers

AO_LedEngine evaluates layers top-to-bottom each tick. First active layer wins.

| Layer | Priority | Source | Example |
|-------|----------|--------|---------|
| FAULT | 0 (highest) | SIG_HEALTH_STATUS | Critical sensor failure: red fast blink |
| FLIGHT_PHASE | 1 | SIG_PHASE_CHANGE | Armed: orange solid. Boost: red solid |
| CALIBRATION | 2 | SIG_LED_OVERRIDE | Gyro cal: blue breathe. Mag cal: rainbow |
| RADIO_STATUS | 3 | SIG_RADIO_STATUS | RX receiving: green. Lost: red fast blink |
| SENSOR_STATUS | 4 | SIG_SENSOR_DATA | GPS 3D fix: green solid. Searching: yellow blink |
| IDLE | 5 (lowest) | Default | Blue breathe (ESKF running, no GPS) |

Core 1 vitality check (Council A4): `core1_loop_count` stall for 500ms forces FAULT layer.

---

## Signal Catalog

| Signal | Value | Payload Struct | Publisher(s) | Subscriber(s) |
|--------|-------|----------------|-------------|----------------|
| SIG_TICK | 4 | QEvt (base) | QF tick timer | AO_FlightDirector (internal HSM) |
| SIG_ARM | 5 | QEvt | CLI/Radio | AO_FlightDirector |
| SIG_DISARM | 6 | QEvt | CLI/Radio | AO_FlightDirector |
| SIG_LAUNCH | 7 | QEvt | Guard eval | AO_FlightDirector |
| SIG_BURNOUT | 8 | QEvt | Guard eval | AO_FlightDirector |
| SIG_APOGEE | 9 | QEvt | Guard eval | AO_FlightDirector |
| SIG_MAIN_DEPLOY | 10 | QEvt | Guard eval | AO_FlightDirector |
| SIG_LANDING | 11 | QEvt | Guard eval | AO_FlightDirector |
| SIG_ABORT | 12 | QEvt | CLI/Radio | AO_FlightDirector |
| SIG_RESET | 13 | QEvt | CLI | AO_FlightDirector |
| SIG_SENSOR_DATA | 14 | SensorDataEvt | eskf_runner | AO_FD, AO_Logger, AO_Telem, AO_LedEngine |
| SIG_PHASE_CHANGE | 15 | PhaseChangeEvt | AO_FlightDirector | AO_Logger, AO_Telem, AO_LedEngine |
| SIG_LED_PATTERN | 16 | LedPatternEvt | AO_FlightDirector | AO_LedEngine |
| SIG_LOG_FRAME | 17 | QEvt | AO_Logger (internal) | AO_Logger |
| SIG_TELEM_FRAME | 18 | QEvt | AO_Logger (internal) | AO_Telemetry |
| SIG_PYRO_INTENT | 19 | QEvt | AO_FlightDirector | AO_Logger |
| SIG_LED_OVERRIDE | 20 | LedPatternEvt | AO_RCOS | AO_LedEngine |
| SIG_CLI_COMMAND | 21 | QEvt | AO_RCOS | AO_FlightDirector |
| SIG_HEALTH_CHECK | 22 | QEvt | (reserved) | (reserved) |
| SIG_RADIO_TX | 23 | RadioTxEvt | AO_Telemetry | AO_Radio |
| SIG_RADIO_RX | 24 | RadioRxEvt | AO_Radio | AO_Telemetry |
| SIG_RADIO_STATUS | 25 | RadioStatusEvt | AO_Radio | AO_LedEngine |
| SIG_GCS_CMD | 26 | GcsCmdEvt | AO_Telemetry | AO_FlightDirector |
| SIG_HEALTH_STATUS | 27 | HealthStatusEvt | AO_FlightDirector | AO_LedEngine, AO_Telemetry |
| SIG_AO_MAX | 28 | -- | -- | -- |

Private signals (per-AO, not in catalog): `SIG_AO_MAX + offset` (0=Blinker, 1=Counter, 2=LedEngine, 3=FD, 4=Logger, 5=Telem, 10=Radio).

---

## Idle Bridge Contract

`qv_idle_bridge()` in main.cpp runs when all AO queues are empty. Contains:

1. **watchdog_kick_tick()** — permanent, never moves to AO (Council A2, Stage 9)
2. **eskf_runner_tick()** — permanent, needs zero-latency seqlock polling (Council A1, Stage 13)
3. **rc_os_update()** — blocking calibration wizards only (conditional on output mode)
4. **__wfi()** — ARM wait-for-interrupt, sleeps until next IRQ

Items 1-2 are system invariants. Item 3 moves to non-blocking when calibration state machine is refactored (deferred).

---

## Migration Checklist

| Phase | Description | Status | Commit | Lines Moved | Binary Delta |
|-------|-------------|--------|--------|-------------|-------------|
| 0A | sensor_seqlock.h | **done** | (uncommitted) | 99 | ~0 |
| 0B | led_patterns.h | pending | -- | ~50 | -- |
| 1 | sensor_core1.cpp | pending | -- | ~450 | -- |
| 2 | eskf_runner.cpp | pending | -- | ~400 | -- |
| 3 | Complete AO_FD | pending | -- | ~200 | -- |
| 4 | Complete AO_Logger | pending | -- | ~250 | -- |
| 5 | LED priority compositor | pending | -- | ~150 | -- |
| 6 | health_monitor.cpp | pending | -- | ~180 | -- |
| 7 | cli_commands.cpp | pending | -- | ~840 | -- |
| 8 | cal_hooks.cpp | pending | -- | ~120 | -- |

**Target:** main.cpp < 600 lines after all phases complete.

---

## Design Decisions

### Why ESKF is a Module, Not an AO (Council A1)
The QF base tick is 100Hz. ESKF needs 200Hz effective rate (every 5th IMU sample at 1kHz). A 100Hz AO tick would reduce propagation rate to 100Hz during boost, doubling attitude drift per step. The idle bridge provides zero-latency seqlock polling. ESKF publishes SIG_SENSOR_DATA via `QActive_publish_()` after each predict, getting event-driven downstream benefits without timing penalty.

### Why Health Monitor is a Module, Not an AO (Council A2)
10Hz health checks don't justify a separate AO queue (128 bytes) and priority slot. Called from AO_FlightDirector at a divider. FD publishes SIG_HEALTH_STATUS when health changes.

### Read-Only Accessor Pattern (Council A6)
All module/AO public APIs that return `const*` to internal state are safe under QV cooperative scheduling (single-threaded Core 0). If the project ever migrates to QK preemptive scheduling, these accessors become data races and must be replaced with event-based request/reply or protected with critical sections. Each header documents which functions are read-only accessors.

---

*See also: `tools/spin/rocketchip_ao.pml` for formal SPIN verification model.*
