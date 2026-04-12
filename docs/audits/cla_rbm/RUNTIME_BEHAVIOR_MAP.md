# Runtime Behavior Map (RBM)

**Date:** 2026-04-12
**Firmware:** Post-Stage 16A (QP/C QV Active Object architecture, 8 AOs)
**Diagrams:** Graphviz `.dot` sources in `docs/audits/cla_rbm/dot/` — render with `dot -Tsvg <file>.dot -o <file>.svg`

---

## 1. Overview

RocketChip firmware uses the QP/C QV cooperative scheduler on Core 0 for
event-driven subsystem management. Core 1 runs a dedicated sensor polling
loop at 1kHz. All subsystem logic lives in Active Objects (AOs) that
communicate via typed events. The idle bridge runs ESKF fusion and watchdog
when all AO queues are empty.

**Key architectural changes from Stage 7:**
- Superloop replaced by QP/C QV cooperative scheduler (Stage 9)
- 8 Active Objects own all subsystem state and run-to-completion handlers
- ESKF runs in idle bridge (not a timed tick) for zero-latency seqlock polling
- LED priority managed by AO_Notify (intent layer) + AO_LedEngine (display layer)
- Health monitoring is a standalone AO with 2-bit encoding and fault escalation
- CLI owned by AO_RCOS (20Hz, lowest priority)

**Conventions:**
- Graphviz `.dot` diagrams for complex state/flow diagrams (boot, error recovery, cross-core)
- Mermaid for simple state machines (CLI, NeoPixel)
- Indented call trees for function-level breakdown
- CLA timing values annotated where applicable (see `COMPUTATIONAL_LOAD_ANALYSIS.md`)

---

## 2. Core 0 Execution Model — QV Cooperative Scheduler

### QV Dispatch Loop

`QF_run()` (entered from `main()`, never returns) loops:
1. Find highest-priority AO with a non-empty event queue
2. Dispatch one event to that AO's state handler (run-to-completion)
3. If all queues empty → call `qv_idle_bridge()` (see §2b)
4. Repeat

**QF base tick rate:** 100Hz (10ms per tick). AOs derive their rates via
`QTimeEvt_armX()` with a period divider.

### Active Object Priorities and Rates

| Prio | AO | File | Rate | Queue | Roles | Owns |
|------|----|------|------|-------|-------|------|
| 8 | AO_Radio | `ao_radio.cpp` | 100Hz | 32 | All | RFM95W driver, RadioScheduler |
| 7 | AO_FlightDirector | `ao_flight_director.cpp` | 100Hz | 32 | Vehicle | HSM, guard eval, Go/No-Go, pyro |
| 6 | AO_HealthMonitor | `ao_health_monitor.cpp` | 10Hz | 8 | Vehicle | 2-bit health, sliding windows, Core1 vitality |
| 5 | AO_Notify | `ao_notify.cpp` | 33Hz | 16 | Vehicle | Intent resolver, output backend dispatch |
| 4 | AO_Logger | `ao_logger.cpp` | 50Hz | 32 | Vehicle | Ring buffer, log decimator, FusedState builder |
| 3 | AO_Telemetry | `ao_telemetry.cpp` | 10Hz | 8 | Vehicle+Station | CCSDS/MAVLink encode, RX decode, ACK tracking |
| 2 | AO_LedEngine | `ao_led_engine.cpp` | 33Hz | 8 | Vehicle | WS2812 driver, 3-layer compositor |
| 1 | AO_RCOS | `ao_rcos.cpp` | 20Hz | 16 | All | CLI, ANSI dashboard, key dispatch |

### Idle Bridge (`qv_idle_bridge()`)

Runs when all AO queues are empty. Contains system invariants:

```
qv_idle_bridge():
    1. watchdog_kick_tick()           // PIO heartbeat — permanent (Council A2)
    2. eskf_runner_tick()             // Fusion + publish SIG_SENSOR_DATA (Council A1)
       ├── seqlock_read() → snap
       ├── predict(accel, gyro, dt)   [~111µs SRAM, 200Hz effective]
       ├── baro/mag/GPS/ZUPT updates  [when new data available]
       ├── health check → reset if unhealthy (CR-1)
       └── QActive_publish_(SIG_SENSOR_DATA)
    3. AO_Telemetry_cmd_retry_tick()  // Station ACK retry (IVP-122, station mode only)
    4. __wfi()                        // Sleep until next interrupt
```

**Why ESKF is in idle, not an AO:** The QF tick is 100Hz. ESKF needs 200Hz
effective rate (every 5th IMU sample at 1kHz). An AO tick would reduce
propagation rate during boost, doubling attitude drift per step. The idle
bridge provides zero-latency seqlock polling. (Council A1, Stage 13.)

---

## 2b. Concurrency Timeline

```
Phase         Core 0                          Core 1
────────────  ─────────────────────────────   ─────────────────────
BOOT          init_hardware()
                fault handlers + MPU guard
                NeoPixel init
                psram_init() + self_test()
                multicore_launch_core1() ───> core1_entry()
                                              MPU stack guard (Core 1)
                i2c_bus_init()                wait for g_startSensorPhase...
                init_sensors() (probe-first)  (waiting)
                init_peripherals()            (waiting)
                  SPI + radio init
                  telemetry + MAVLink init
                  cal storage init
                  stdio_init_all() (USB)
              init_application()
                QF_init() + QActive_psInit()
                AO start (8 AOs, prio 1-8)
                init_rc_os_hooks()
                g_startSensorPhase=true ────> wait loop unblocks
                                              multicore_lockout_victim_init()
                wait g_core1LockoutReady <─── g_core1LockoutReady=true
                psram flash-safe test
                logging ring + flight table
                watchdog_enable(5s)
              QF_run()  [never returns]

QV DISPATCH   QV loop:                        core1_sensor_loop():
              1. Dispatch highest-prio AO      IMU read (1kHz)
              2. If all empty → idle bridge    Baro read (16Hz)
                 watchdog_kick_tick()           GPS read (10Hz)
                 eskf_runner_tick() ←seqlock── Mag read (100Hz)
                 __wfi()                        seqlock_write()
                                               g_wdtCore1Alive=true
                                               busy_wait(remaining)

CALIBRATION   AO_RCOS posts CalIntentEvt       Core 1 pause handshake
(rare, CLI)   cal_pre_hook()                   (same as before)
              g_core1PauseI2C=true ────────>   stalls, LED orange
              ... calibration reads ...
              g_core1PauseI2C=false ────────>   resumes

FLASH OPS     flash_safe_execute() ─────────> multicore_lockout
(rare)        (erase/write, 1-150ms)          (Core 1 stalled)
              i2c_bus_reset() after ─────────> resume, bus recovered
```

---

## 3. Boot Sequence

See `dot/boot_sequence.dot` for visual diagram.

```
main()
├── init_hardware()  [returns bool: watchdog_reboot]
│   ├── check_watchdog_reboot()              [Sentinel in scratch[0] == 0x52435754]
│   │   └── If true: g_watchdogReboot = true, clear sentinel
│   ├── init_early_hw()
│   │   ├── exception_set_exclusive_handler(HARDFAULT)
│   │   ├── exception_set_exclusive_handler(MEMMANAGE)
│   │   ├── mpu_setup_stack_guard(Core 0)
│   │   ├── gpio_init(LED_PIN) + gpio_set_dir(OUTPUT)
│   │   └── ws2812_status_init(pio0, NEOPIXEL_PIN, count)
│   ├── psram_init() + psram_self_test()     [Before Core 1 — QMI XIP]
│   ├── multicore_launch_core1(core1_entry)  [Core 1 starts early, waits]
│   ├── i2c_bus_init()                       [I2C1, 400kHz — after flash]
│   ├── init_sensors()                       [Probe-first detection]
│   │   ├── sleep_ms(200)                    [Sensor power-up settling]
│   │   ├── i2c_bus_probe(0x69) → icm20948_init()
│   │   │   └── icm20948_enable_bypass()
│   │   ├── i2c_bus_probe(0x77) → baro_dps310_init()
│   │   ├── UART GPS detect → gps_uart_init()
│   │   └── I2C GPS fallback → gps_pa1010d_init()
│   └── init_peripherals()
│       ├── spi_bus_init()
│       ├── rfm95w_init(&g_radio, ...)
│       ├── telemetry_service_init()
│       ├── MAVLink encoder init
│       ├── calibration_storage_init()
│       └── init_usb() → stdio_init_all()
│
├── init_application(watchdogReboot)
│   ├── QF_init()                            [QP/C framework init]
│   ├── QActive_psInit(subscriberList, SIG_AO_MAX)
│   ├── AO start calls (8 AOs, priorities 1-8)
│   ├── init_rc_os_hooks()
│   ├── g_startSensorPhase = true            [Signal Core 1]
│   ├── Wait for g_core1LockoutReady
│   ├── psram_flash_safe_test()
│   ├── init_logging_ring()
│   ├── flight_table_load()
│   ├── calibration_start_baro()
│   └── watchdog sentinel + watchdog_enable(5000)
│
└── QF_run()  [QV scheduler — never returns]
```

### Init Order Constraints

| Order Rule | Reason | Source |
|------------|--------|--------|
| Flash ops before USB | Flash makes all flash inaccessible including USB IRQs | LL Entry 4, 12 |
| I2C init after flash | flash_safe_execute corrupts I2C peripheral | LL Entry 31 |
| PSRAM before Core 1 | QMI XIP interaction requires single-core init | Architecture |
| Sensor probe before init | Prevents bus side-effects from absent devices | LL Entry 28 |
| Core 1 launch early, signal late | Core 1 waits for g_startSensorPhase | Architecture |
| QF_init before AO start | QP framework must be initialized first | QP/C requirement |
| Watchdog after Core 1 lockout ready | Both cores must be running before timeout starts | Architecture |

---

## 4. Event Flow

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
     SIG_PYRO_FIRED                       v
     SIG_BEACON_ACTIVE              AO_Radio
              |                     (100Hz, P8)
              |                         |
              v                 SIG_RADIO_STATUS
  AO_HealthMonitor (10Hz, P6)           |
      reads seqlock + ESKF + confidence |
      reads Core 1 vitality (primary)   |
      publishes SIG_HEALTH_STATUS       |
        --> Logger, Telemetry, Notify   |
              |                         |
              v                         v
        +---------- AO_Notify (33Hz, P5) -------+
        |   subscribes: PHASE_CHANGE,           |
        |     RADIO_STATUS, HEALTH_STATUS,      |
        |     BEACON_ACTIVE                     |
        |   reads seqlock for sensor status     |
        |   runs priority resolver              |
        |   posts via output backends           |
        +---------------------+-----------------+
                              | SIG_LED_PATTERN
                              v
                        AO_LedEngine (33Hz, P2)
                          3 layers: Fault / Notify / Idle
                          Core 1 vitality fallback (A1)
                          drives ws2812

  AO_RCOS (20Hz, P1, CLI)
    reads: all module/AO public APIs for display
    posts: SIG_CLI_COMMAND, CalIntentEvt to AO_Notify
```

### Signal Catalog

See `docs/AO_ARCHITECTURE.md` for the full signal catalog (28 signals),
event payloads, and subscriber map.

---

## 5. Core 1 Sensor Loop

```
core1_entry()
├── mpu_setup_stack_guard(__StackOneBottom)
├── Wait for g_startSensorPhase (spin on atomic)
├── multicore_lockout_victim_init()
├── g_core1LockoutReady = true
└── core1_sensor_loop()

core1_sensor_loop()
├── core1_load_cal_or_defaults(&localCal)
└── while (true)  [1kHz target]
    ├── cycleStartUs = time_us_32()
    ├── core1_check_pause_and_reload(&localCal)  [cal pause handshake]
    │   ├── If g_core1PauseI2C: stall, LED orange, wait for release
    │   └── If g_calReloadPending: reload cal from flash
    ├── core1_read_imu(&localData, &localCal, &imuConsecFail, feedCal)
    │   ├── icm20948_read() → raw data
    │   ├── Validate |A| >= 3.0 m/s² (zero-output fault detection)
    │   ├── core1_apply_imu_cal() — apply calibration
    │   ├── On failure: core1_imu_error_recovery()
    │   │   ├── >= 10 (every 10th): i2c_bus_recover()
    │   │   └── >= 50: icm20948_init() (full device reinit)
    │   └── On success: imuConsecFail = 0
    ├── core1_read_baro(&localData)  [every kCore1BaroDivider cycles]
    ├── core1_read_gps(&localData, &lastGpsReadUs)  [every kCore1GpsDivider]
    │   ├── g_gpsFnUpdate() → parse NMEA
    │   └── 500µs settling delay (PA1010D I2C bus contention fix)
    ├── Mag read via IMU bypass [every kCore1MagDivider cycles]
    ├── seqlock_write(&g_sensorSeqlock, &localData)
    ├── g_wdtCore1Alive = true
    ├── elapsed = time_us_32() - cycleStartUs
    └── busy_wait_us(1000 - elapsed)  [pad to 1ms target]
```

---

## 6. CLI State Machine (AO_RCOS)

CLI is entirely owned by AO_RCOS (20Hz, priority 1). Two modes:

**Vehicle mode:** Interactive calibration menu, sensor status, ESKF live,
flight download, radio status.

**Station mode:** ANSI dashboard (auto-refresh), ARM confirm flow (type
"ARM" + Enter), X-DISARM (capital X), distance-to-rocket, GPS push,
preflight health check, help.

### Vehicle Key Bindings

| Key | Action | Handler |
|-----|--------|---------|
| `h`, `H`, `?` | Print help | AO_RCOS |
| `s`, `S` | Sensor status (one-shot) | AO_RCOS → callback |
| `e` | ESKF live streaming (1Hz) | AO_RCOS → callback |
| `b`, `B` | Reprint boot summary | AO_RCOS → callback |
| `c`, `C` | Enter calibration submenu | AO_RCOS |
| `l`, `L` | Flush log ring buffer | AO_RCOS |
| `f`, `F` | List stored flights | AO_RCOS |
| `d`, `D` | Download flight | AO_RCOS |
| `x`, `X` | Erase all flights ("YES" confirm) | AO_RCOS |
| `t`, `T` | Radio status | AO_RCOS |
| `r` | Cycle TX rate | AO_RCOS |
| `m`, `M` | Toggle MAVLink/CLI output | AO_RCOS |

### Station Key Bindings (IVP-122/123/124)

| Key | Action |
|-----|--------|
| `a` | ARM confirm (type "ARM" + Enter) |
| `X` | DISARM (tracked, ACK-waited) |
| `d` | Distance-to-rocket (haversine + bearing) |
| `p` | Push station GPS to vehicle |
| `P` | Preflight health check |
| `h` | Help |

---

## 7. LED Priority Architecture (Post Stage 14)

AO_Notify is the intent layer — it subscribes to all state-producing signals,
maintains per-category intents, runs a priority resolver, and posts a single
resolved pattern via SIG_LED_PATTERN.

AO_LedEngine is the display layer — it receives patterns and drives the WS2812
through a 3-layer compositor:

| Layer | Priority | Source | Example |
|-------|----------|--------|---------|
| FAULT | 0 (highest) | Core 1 vitality check (local, A1 fallback) | Core 1 stall: magenta solid |
| NOTIFY | 1 | SIG_LED_PATTERN from AO_Notify | Whatever resolver picked |
| IDLE | 2 (lowest) | Default | Blue blink (kSensorNeoNoGps) |

### AO_Notify Intent Categories (priority order)

1. **Fault** — from SIG_HEALTH_STATUS decode (IMU/baro/ESKF/Core1 faults)
2. **Calibration** — from CalIntentEvt direct post (AO_RCOS)
3. **Flight Phase** — from SIG_PHASE_CHANGE (armed/boost/coast/descent/landed/beacon)
4. **Radio** — from SIG_RADIO_STATUS (receiving/gap/lost)
5. **Sensor** — from seqlock read (ESKF init, GPS fix status, timeout)
6. **Idle** — default blue blink

### Core 1 Vitality — 2-Layer Model (IVP-130)

1. **AO_HealthMonitor** (primary, 10Hz): reads seqlock `core1_loop_count`,
   detects stall at 6 consecutive stale ticks (~600ms), publishes via
   SIG_HEALTH_STATUS → AO_Notify decodes as FaultIntent::kCore1Stall.
2. **AO_LedEngine** (Council A1 fallback, 33Hz): local seqlock read +
   `kCore1StallThreshold` (17 ticks, ~500ms) → FAULT layer magenta solid.
   Survives AO_HealthMonitor crash.

---

## 8. Error Recovery Paths

See `dot/error_recovery.dot` for visual diagram.

Each path terminates in one of three outcomes:
- **(a) Recovered** — normal operation resumed
- **(b) Degraded** — running with reduced capability
- **(c) Watchdog Reset** — full reboot

### 8.1 IMU (ICM-20948)

| Trigger | Action | Outcome |
|---------|--------|---------|
| I2C NACK | imuConsecFail++, mark accel/gyro invalid | Retry next cycle |
| 10+ consecutive fails (every 10th) | i2c_bus_recover() (9-clock recovery) | **(a)** Recovered |
| 50+ consecutive fails | icm20948_init() (full device reinit + bypass mode) | **(a)** Recovered |
| Zero-output fault (\|A\| < 3.0 m/s²) | Routes to consecutive fail path | **(a)** Recovered |
| Sustained failure | No ESKF predict, filter coasts | **(b)** Loss: attitude/position/velocity |

### 8.2 Baro (DPS310)

| Trigger | Action | Outcome |
|---------|--------|---------|
| Read failure (MEAS_CFG not ready) | error_count++ | Retry next cycle |
| Sustained read failure | No baro altitude updates | **(b)** GPS+ZUPT fallback |

### 8.3 GPS (PA1010D / UART)

| Trigger | Action | Outcome |
|---------|--------|---------|
| NMEA parse failure | error_count++, skip cycle | **(a)** Recovered |
| No satellite fix | GPS data marked invalid | **(b)** Dead-reckoning only |
| UART RX overflow | Bytes dropped, rxOvf counter | **(a)** Self-correcting |

### 8.4 ESKF Filter

5 health sentinels checked every predict cycle:

| Trigger | Sentinel | Outcome |
|---------|----------|---------|
| NaN/Inf in P matrix | `!P.is_finite()` | **(a)** Filter reset via CR-1 |
| Non-positive P diagonal | `P(i,i) <= 0` | **(a)** Filter reset |
| Quaternion norm drift | `|norm(q) - 1| > 1e-3` | **(a)** Filter reset |
| Implausible biases | Gyro > 0.175 rad/s or accel > 1.0 m/s² | **(a)** Filter reset |
| Velocity divergence | `v.norm() >= 500 m/s` | **(a)** Filter reset |

**CR-1 path:** `eskf_initialized = false` → next cycle calls `eskf_try_init()`
→ waits for IMU stationarity → reinitializes.

### 8.5 Health Monitor Escalation (Stage 13)

| Trigger | Action | Outcome |
|---------|--------|---------|
| Subsystem fault (2-bit encoding) | SIG_HEALTH_STATUS → Notify + Logger + Telem | **(b)** Fault LED |
| Multiple faults in IDLE phase | Auto-DISARM (blocks ARM transition) | **(b)** Safety interlock |
| Core 1 stall (6 stale ticks) | HEALTH_STATUS with kHealthCore1Ok cleared | **(b)** Fault LED |

### 8.6 Flight Director Multi-Channel Landing (Stage P7)

MAIN_DESCENT uses voted landing detection, not a blind timer:

| Path | Condition | Outcome |
|------|-----------|---------|
| Primary | ESKF stationary guard + baro peak guard | SIG_LANDING → LANDED |
| Path 1 | ESKF fault AND baro stationary (conjunction) | SIG_LANDING → LANDED |
| Path 2 | descent_max_duration_ms elapsed (backstop) | SIG_LANDING + SIG_BEACON_ACTIVE |

### 8.7 Watchdog

| Trigger | Action | Outcome |
|---------|--------|---------|
| Core 0 or Core 1 stalled >5s | Watchdog timeout | **(c)** Full reboot |

### 8.8 Radio (LoRa RFM95W)

| Trigger | Action | Outcome |
|---------|--------|---------|
| TX busy (DIO0 not TxDone) | Skip packet, retry next slot | **(b)** Dropped frame |
| SPI failure | error_count++ | **(b)** No telemetry |
| Command ACK timeout (station, 3 retries) | UI shows failure | **(b)** Command not confirmed |

### 8.9 Flash Operations

| Trigger | Action | Outcome |
|---------|--------|---------|
| flash_safe_execute() | Core 1 stalled via multicore_lockout | **(a)** i2c_bus_reset() after |

### 8.10 I2C Bus

| Trigger | Action | Outcome |
|---------|--------|---------|
| SDA stuck LOW | i2c_bus_reset(): deinit → 9-clock → reinit | **(a)** Bus recovered |
| SCL stuck LOW | Early exit (can't pulse) | **(b)** Bus unusable until power cycle |

---

## 9. Cross-Core Communication Map

See `dot/cross_core.dot` for visual diagram.

| Primitive | Type | Writer | Reader | Rate | Purpose |
|-----------|------|--------|--------|------|---------|
| `g_sensorSeqlock` | Seqlock (148B struct) | Core 1 | Core 0 | 1kHz write, 200Hz read | Sensor data transfer |
| `g_startSensorPhase` | atomic\<bool\> | Core 0 | Core 1 | Once at boot | Signal Core 1 to start |
| `g_core1LockoutReady` | atomic\<bool\> | Core 1 | Core 0 | Once at boot | Core 1 ready for flash lockout |
| `g_wdtCore0Alive` | atomic\<bool\> | Core 0 | Core 0 | 1kHz set, 1Hz check | Watchdog Core 0 liveness |
| `g_wdtCore1Alive` | atomic\<bool\> | Core 1 | Core 0 | 1kHz set, 1Hz check | Watchdog Core 1 liveness |
| `g_core1PauseI2C` | atomic\<bool\> | Core 0 | Core 1 | Rare (cal) | Pause Core 1 I2C |
| `g_core1I2CPaused` | atomic\<bool\> | Core 1 | Core 0 | Rare (cal) | Acknowledge pause |
| `g_calReloadPending` | atomic\<bool\> | Core 0 | Core 1 | Rare (cal) | Signal cal data reload |
| `flash_safe_execute` | multicore_lockout | Core 0 | Core 1 stalled | Rare (flash) | Safe flash access |

---

## 10. Sensor Detection Permutations

`init_sensors()` uses probe-first detection. All combinations are valid:

| IMU | Baro | GPS | Behavior |
|-----|------|-----|----------|
| Yes | Yes | Yes (UART) | Full operation — all sensors + ESKF + telemetry |
| Yes | Yes | Yes (I2C) | Full operation — I2C GPS with 500µs settling delay |
| Yes | Yes | No | ESKF without GPS. ZUPT + baro only |
| Yes | No | Yes | ESKF without baro. GPS altitude only |
| Yes | No | No | Attitude-only mode (drifts without aiding) |
| No | Yes | Yes | **ESKF does not run.** Baro/GPS collected but unused |
| No | Yes | No | Baro-only. Temperature and altitude only |
| No | No | Yes | GPS-only. Position/speed from NMEA |
| No | No | No | **Minimal mode.** LED heartbeat, CLI, radio (if present) |

**Radio** is detected separately via SPI probe. Present/absent is orthogonal.

---

## 11. Blocking Code Paths

All AO handlers must complete within one QV tick period (10ms at 100Hz).
Violations cause queue overflow (LL Entry 32).

| Path | Core | Duration | Trigger | Impact |
|------|------|----------|---------|--------|
| `flash_safe_execute` (erase) | Core 0 | ~150ms | Log flush | Core 1 stalled |
| `flash_safe_execute` (write) | Core 0 | ~1-2ms | Log flush | Core 1 briefly stalled |
| `calibration_save()` | Core 0 | ~150ms | CLI cal command | Core 1 stalled |
| `i2c_bus_reset()` | Core 1 | ~1ms | After flash or IMU fail | None |
| `icm20948_reinit()` | Core 1 | ~10ms | 50+ IMU fails | ~10 missed sensor cycles |
| `core1_check_pause` stall | Core 1 | Unbounded (user) | Calibration | Must complete before 5s WDT |
| `__wfi()` | Core 0 | <10ms | Idle bridge | Wakes on next 100Hz tick or USB IRQ |
| `rfm95w_send()` | Core 0 | ~150ms worst | LoRa TX | All AO queues absorb (depth 32) |

**rfm95w_send() is the only remaining blocking call in AO context.** Queue
depth 32 handles 320ms at 100Hz tick rate (2× the 150ms worst case). Proper
fix: non-blocking start/poll split (deferred, Council-reviewed).

---

## 12. Identified Gaps

### Known Gaps

| ID | Description | Impact | Status |
|----|-------------|--------|--------|
| GAP-1 | DPS310 baro has no recovery for frozen continuous mode | Baro stops | Open |
| GAP-2 | RadioScheduler not synced to vehicle RX window | Command delivery unreliable | IVP needed |
| GAP-3 | rfm95w_send() blocks AO handlers for up to 150ms | Queue depth mitigates, not ideal | Non-blocking split deferred |
| GAP-4 | No battery ADC monitoring | No low-voltage warning | Hardware not wired |

---

## Appendix: Staleness Check

Run `python scripts/rbm_check.py` to verify this document is current with the codebase. The script checks:
1. Required function names still exist in source
2. All AOs are documented with correct priorities
3. CLI key bindings are all listed
4. New functions matching key patterns are flagged

Re-run after adding new AOs, CLI commands, state transitions, or error recovery paths.
