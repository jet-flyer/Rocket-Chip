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
| AO_FlightDirector | `ao_flight_director.cpp` | 100Hz | 9 | 32 | FlightDirector HSM ([diagram](../audits/cla_rbm/dot/flight_director_hsm.dot)), guard eval, Go/No-Go (incl. RF Link via AO_RfManager state), PIO timer hooks | SIG_PHASE_CHANGE, SIG_PYRO_FIRED, SIG_BEACON_ACTIVE | SIG_SENSOR_DATA |
| AO_Radio | `ao_radio.cpp` | 100Hz | 8 | 32 | rfm95w_t, RadioScheduler, RadioAoState; station-TX gated by `AO_RfManager_next_tx_window_us()` (Stage T IVP-T14) | SIG_RADIO_RX, SIG_RADIO_STATUS | SIG_RADIO_TX |
| AO_RfManager | `ao_rf_manager.cpp` | 10Hz | 7 | 16 | LinkState (kAcq/kTentative/kTrack/kTrackDegraded), sliding-window LQ%, α-filtered anchor estimate, deadman, forced-ACQ; posts vehicle-lost/found notify on transition edges (Stage T IVP-T14) | SIG_NOTIFY_VEHICLE_LOST, SIG_NOTIFY_VEHICLE_FOUND | SIG_RADIO_RX |
| AO_HealthMonitor | `ao_health_monitor.cpp` | 10Hz | 6 | 8 | HealthState, sliding windows, fault latch, staleness counter, Core1 vitality primary check (IVP-117); populates GoNoGoInput RF link fields from AO_RfManager (Stage T IVP-T14) | SIG_HEALTH_STATUS | SIG_PHASE_CHANGE |
| AO_Notify | `ao_notify.cpp` | 33Hz | 5 | 16 | NotifyState, intent resolver, output backend dispatch, sensor status evaluation (Stage 14), beacon overlay + pre-arm fail + boot-init rainbow (Stage L) | SIG_LED_PATTERN (via backend) | SIG_PHASE_CHANGE, SIG_RADIO_STATUS, SIG_HEALTH_STATUS, SIG_BEACON_ACTIVE, SIG_BEACON_MANUAL |
| AO_Logger | `ao_logger.cpp` | 50Hz | 4 | 32 | RingBuffer, LogDecimator, FlightTable, FusedState builder, SRAM ring | (none) | SIG_PHASE_CHANGE, SIG_PYRO_FIRED, SIG_HEALTH_STATUS |
| AO_Telemetry | `ao_telemetry.cpp` | 10Hz | 3 | 8 | CCSDS/MAVLink encode, TelemetryState snapshot, RX decode | SIG_RADIO_TX | SIG_RADIO_RX, SIG_SENSOR_DATA, SIG_HEALTH_STATUS |
| AO_LedEngine | `ao_led_engine.cpp` | 33Hz | 2 | 8 | ws2812 driver, 3-layer compositor (Fault/Notify/Idle), Core1 vitality fallback (A1) | (none) | SIG_LED_PATTERN |
| AO_RCOS | `cli/ao_rcos.cpp` | 20Hz | 1 | 16 | CLI output mode, ANSI dashboard, key dispatch, cal intent posting | SIG_CLI_COMMAND | (none) |

*AO_Blinker (disabled): heartbeat LED demo. AO_Counter (disabled): jitter measurement diagnostic.*

---

## Module Inventory

| Module | File | Interface | Called By | Owns |
|--------|------|-----------|----------|------|
| eskf_runner | `src/fusion/eskf_runner.cpp` | `eskf_runner_tick()`, `eskf_runner_get_*()` | Idle bridge (200Hz effective) | ESKF, Mahony, ConfidenceGate, bench stats, epoch, state buffer |
| health_monitor | `src/safety/health_monitor.cpp` | `health_monitor_tick()`, `health_monitor_get_state()` | AO_HealthMonitor (10Hz) | 2-bit health state, sliding windows, fault latch, Go/No-Go |
| sensor_core1 | `src/core1/sensor_core1.cpp` | `core1_entry()` | main.cpp (multicore_launch) | Core 1 sensor loop, seqlock write, IMU/baro/GPS reads |
| sensor_seqlock | `include/rocketchip/sensor_seqlock.h` | `seqlock_read()`, `seqlock_write()`, types | sensor_core1 (write), eskf_runner + AOs (read) | shared_sensor_data_t, cross-core atomics |
| led_patterns | `include/rocketchip/led_patterns.h` | Pattern constants, LedLayer enum | AO_LedEngine, rc_os.cpp, flight_actions.h | kCalNeo*, kRxNeo*, kFdNeo* constants |
| cal_hooks | `src/calibration/cal_hooks.cpp` | `cal_pre_hook()`, `cal_post_hook()` | rc_os.cpp callbacks | Cross-core I2C pause/resume, mag read |
| cli_commands | `src/cli/cli_commands.cpp` | `handle_unhandled_key()`, `print_*()`, `cmd_*()` | AO_RCOS, rc_os.cpp | CLI command handlers, display formatters |

---

## Event Flow

**Diagram:** `docs/audits/cla_rbm/dot/ao_event_flow.dot` — render with `dot -Tsvg ao_event_flow.dot -o ao_event_flow.svg`

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
              +---------------+-----------+
              |               |           |
              v               v           v
        AO_FlightDir    AO_Logger   AO_Telemetry
        (100Hz, P7)     (50Hz, P4)  (10Hz, P3)
              |               |           ^
     SIG_PHASE_CHANGE   writes ring       | SIG_RADIO_TX
     SIG_PYRO_FIRED     SIG_PYRO_INTENT   v
     SIG_BEACON_ACTIVE                AO_Radio
              |                       (100Hz, P8)
              |                           |
              v                   SIG_RADIO_STATUS
  AO_HealthMonitor (10Hz, P6)             |
      reads seqlock + ESKF + confidence   |
      reads Core 1 vitality (IVP-117)     |
      publishes SIG_HEALTH_STATUS         |
        --> Logger, Telemetry, Notify     |
              |                           |
              |  SIG_HEALTH_STATUS        |
              v                           v
        +---------- AO_Notify (33Hz, P5) -------+
        |   - subscribes PHASE_CHANGE,          |
        |     RADIO_STATUS, HEALTH_STATUS,      |
        |     BEACON_ACTIVE                     |
        |   - reads seqlock for sensor status   |
        |   - maintains NotifyState             |
        |   - runs priority resolver            |
        |   - posts via output backends         |
        +---------------------+-----------------+
                              | SIG_LED_PATTERN
                              v
                        AO_LedEngine (33Hz, P2)
                          - display driver only
                          - 3 layers: Fault / Notify / Idle
                          - Core 1 vitality fallback (A1)
                          - drives ws2812

  Idle Bridge (runs when all AO queues empty):
    1. watchdog_kick_tick()     -- permanent
    2. eskf_runner_tick()       -- fusion + publish SIG_SENSOR_DATA
    3. rc_os_update()           -- blocking cal wizards only (conditional)
    4. __wfi()                  -- sleep until next interrupt

  AO_RCOS (20Hz, P1, CLI)
    reads: all module/AO public APIs for display
    posts: SIG_CLI_COMMAND (to FD), AO_Notify_post_cal_intent() (to Notify)
```

---

## LED Engine Priority Layers (Post Stage 14)

Stage 14 simplified AO_LedEngine from a 6-layer compositor into a pure
display driver with only 3 layers. All intent resolution (flight phase,
calibration, radio, sensor, fault) now lives in AO_Notify's resolver,
which posts a single resolved pattern via SIG_LED_PATTERN.

| Layer | Priority | Source | Example |
|-------|----------|--------|---------|
| FAULT  | 0 (highest) | Core 1 vitality check (local, A1 fallback) | Core 1 stall: magenta solid |
| NOTIFY | 1 | SIG_LED_PATTERN from AO_Notify backend | Whatever AO_Notify's resolver picked |
| IDLE   | 2 (lowest) | Default | Blue blink (kSensorNeoNoGps) |

AO_Notify's resolver (`src/notify/notify_backend_led.cpp`) iterates its
own intent categories in priority order: Fault > Calibration > Flight >
Radio > Sensor > Idle. See `docs/decisions/NOTIFY_CONTRACT.md`.

**Core 1 vitality — 2-layer model (IVP-130 audit, 2026-04-12):** Core 1
vitality is checked in exactly 2 places:

1. **AO_HealthMonitor** (primary, 10Hz): reads seqlock `core1_loop_count`,
   detects stall at 6 consecutive stale ticks (~600ms), sets `kHealthCore1Ok`
   in secondary health byte → `SIG_HEALTH_STATUS` → AO_Notify decodes as
   `FaultIntent::kCore1Stall` → LED fault pattern.

2. **AO_LedEngine** (Council A1 fallback, 33Hz): local seqlock read +
   `kCore1StallThreshold` (17 ticks, ~500ms) → FAULT layer magenta solid.
   Survives AO_HealthMonitor crash.

AO_Notify had unused Core1 vitality fields from IVP-117 (declared but never
wired). Removed in IVP-130. Core1 stall is recoverable via watchdog —
single-fault tolerance (2 independent checks) is sufficient. A third
redundant check was accidental complexity from iterative development.

### Stage L Extensions (IVP-L1 … IVP-L7, 2026-04-18)

Stage L added three visual-only features that don't change the layer
model but extend AO_Notify's `NotifyState` and the resolver:

- **Beacon overlay** — two orthogonal flags `beacon_auto` + `beacon_manual`
  applied by `apply_beacon_overlay()` after the normal priority loop.
  See `docs/decisions/NOTIFY_CONTRACT.md` for the composition table.
  Manual wins over auto; manual forces pure-white 2Hz; auto preserves
  state color by remapping to a +white-alternate sibling pattern.
- **Pre-arm-fail visual** — `PhaseIntent::kPreArmFail` with a pure
  auto-clear helper in `include/rocketchip/prearm_fail_ticks.h`.
  `AO_Notify_post_prearm_fail()` shim is called by
  `dispatch_flight_command()` on ARM rejection. Counter resets to
  full on each repost (rapid-fire rejections refresh the window).
- **Boot-init rainbow** — `PhaseIntent::kInit` default on AO_Notify
  startup. Cleared by the tick handler when ESKF ready AND IMU reads
  published AND a minimum 99-tick (~3s) visibility window has elapsed.
  The min-tick gate is load-bearing on warm resets where the other
  two gates are satisfied by the time Notify starts ticking.

Two new driver modes in `ws2812_status.cpp`:

- `WS2812_MODE_ALTERNATE` — two-color 2Hz toggle with configurable
  half-period. Powers all the `+White` beacon patterns (12, 13, 14-18,
  45).
- `WS2812_MODE_DOUBLE_FLASH` — AP-parity pre-arm-fail shape
  (100/100/100/700 ms). Hardcoded timing.

**Station/vehicle LED role divergence (Stage L council decision):**
Station runs no AO_LedEngine — the Fruit Jam's 5-LED NeoPixel strip is
owned by AO_Radio as an RSSI bar, per LL Entry 32 (PIO contention
avoidance). Station's role is link-quality display; vehicle's role is
flight-state display. Intentional role-specific UX, not a candidate
for future unification.

`cmd_findme_beacon()` on station role-gates to skip the local publish
path and instead send `MAV_CMD_USER_1` over radio — vehicle's
`handle_rx_mavlink_msg()` maps it to `SIG_BEACON_MANUAL` publish, so
the operator sees identical visual behavior whether triggered locally
or over radio.

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
| SIG_SENSOR_DATA | 14 | SensorDataEvt | eskf_runner | AO_FD, AO_Logger, AO_Telem |
| SIG_PHASE_CHANGE | 15 | PhaseChangeEvt | AO_FlightDirector | AO_Logger, AO_HealthMonitor, AO_Notify |
| SIG_LED_PATTERN | 16 | LedPatternEvt | AO_Notify (via backend) | AO_LedEngine |
| SIG_BEACON_ACTIVE | 17 | QEvt | AO_FlightDirector (ABORT timeout + LANDED action) | AO_Notify |
| SIG_PYRO_INTENT | 19 | QEvt | AO_FlightDirector (callback) | AO_Logger (callback) |
| ~~SIG_LED_OVERRIDE~~ | 20 | -- | legacy slot (IVP-116 unused) | -- |
| SIG_CLI_COMMAND | 21 | QEvt | AO_RCOS (sync call) | AO_FlightDirector (sync call) |
| SIG_RADIO_TX | 22 | RadioTxEvt | AO_Telemetry | AO_Radio |
| SIG_RADIO_RX | 23 | RadioRxEvt | AO_Radio | AO_Telemetry |
| SIG_RADIO_STATUS | 24 | RadioStatusEvt | AO_Radio | AO_Notify |
| SIG_GCS_CMD | 25 | GcsCmdEvt | AO_Telemetry | AO_FlightDirector |
| SIG_HEALTH_STATUS | 26 | HealthStatusEvt | AO_HealthMonitor | AO_Notify, AO_Logger, AO_Telemetry |
| SIG_PYRO_FIRED | 27 | PyroFiredEvt | AO_FlightDirector | AO_Logger |
| SIG_BEACON_MANUAL | 28 | QEvt | CLI `b` (vehicle) / MAV_CMD_USER_1 dispatch (vehicle from radio) | AO_Notify (Stage L) |
| SIG_AO_MAX | 29 | -- | -- | -- |

Note: "removed" historical slots (LOG_FRAME, TELEM_FRAME, HEALTH_CHECK) do NOT
create numeric gaps in the enum — the table reflects ACTUAL runtime values.

Private signals (per-AO, not in catalog): `SIG_AO_MAX + offset` (0=Blinker,
1=Counter+HealthMon, 2=LedEngine, 3=FD, 4=Logger, 5=Telem, 6=Notify tick,
7=Notify cal intent, 10=Radio, 20=RCOS).

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
| 0A | sensor_seqlock.h | **done** | `5f2df6d` | 99 | ~0 |
| 0B | led_patterns.h | **done** | `26be55d` | 22 | ~0 |
| 1 | sensor_core1.cpp | **done** | `6171231` | 510 | ~0 |
| 2 | eskf_runner.cpp | **done** | `2dd1644` | 395 | ~0 |
| 3 | Complete AO_FD | **done** | `46662e1` | 168 | ~0 |
| 4 | Complete AO_Logger | **done** | `7fdabfe` | 191 | ~0 |
| 5 | LED priority compositor | **done** | `3c328e9` | ~100 (from core1) | +134 |
| 6 | health_monitor.cpp | **done** | `1c6568b` | ~30 (from FD) | +224 |
| 7 | cli_commands.cpp | **done** | `a72878a` | 1159 | +54 |
| 8 | cal_hooks.cpp | **done** | `4986d5b` | 135 | +33 |

**Target:** main.cpp < 600 lines after all phases complete.

---

## Design Decisions

### Why ESKF is a Module, Not an AO (Council A1)
The QF base tick is 100Hz. ESKF needs 200Hz effective rate (every 5th IMU sample at 1kHz). A 100Hz AO tick would reduce propagation rate to 100Hz during boost, doubling attitude drift per step. The idle bridge provides zero-latency seqlock polling. ESKF publishes SIG_SENSOR_DATA via `QActive_publish_()` after each predict, getting event-driven downstream benefits without timing penalty.

### Health Monitor Promoted to AO (Stage 13, IVP-105)
Originally a module called from FD at 10Hz (Council A2). Promoted to standalone AO_HealthMonitor in Stage 13 after council re-review (5 personas, unanimous). Rationale: health monitor has its own tick rate, state, and consumers — textbook AO pattern. Decoupled from FD so health reporting survives FD handler overruns. Queue depth 8, priority 5 (between FD and Logger). Publishes SIG_HEALTH_STATUS with 2-bit per-subsystem encoding. See `docs/decisions/HEALTH_CONTRACT.md`.

### Read-Only Accessor Pattern (Council A6)
All module/AO public APIs that return `const*` to internal state are safe under QV cooperative scheduling (single-threaded Core 0). If the project ever migrates to QK preemptive scheduling, these accessors become data races and must be replaced with event-based request/reply or protected with critical sections. Each header documents which functions are read-only accessors.

---

*See also: `tools/spin/rocketchip_ao.pml` for formal SPIN verification model.*
