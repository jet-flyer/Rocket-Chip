# Runtime Behavior Map

**Date:** 2026-08-24
**Firmware:** post–L2-P5 remediates on `main`; CLA from on-chip `flight-3201eb2` (vehicle v0.16.0)
**Replaces (as current-state):** `docs/audits/cla_rbm/RUNTIME_BEHAVIOR_MAP.md` (2026-04-12, 8 AOs). That file stays as the historical audit snapshot.

**How to read this vs the other maps**

| Map | Question | This document |
|-----|----------|----------------|
| Graphify (AST, 2026-08-24 update: 5109 nodes) | What exists / who calls whom in `src/` | Index only. `.graphifyignore` excludes `docs/audits/` and `scripts/`. |
| SPIN (`SPIN_OK_31` this sitting) | Do these *abstracted* machines violate named LTL claims? | Cited per section. Not wall-clock. |
| CLA (`cla_2026-08-24.md`) | How much of the 5 ms ESKF epoch / 1 ms Core 1 cycle is used? | Numbers on hot paths. Board had DPS310 **failed**. |
| This RBM | What runs, in what order, on which core, when something fails? | The operational story. |

---

## 1. Two cores

**Core 0** runs QP/C **QV** (cooperative). `QF_run()` never returns: dispatch the highest-priority AO with a queued event, else `qv_idle_bridge()`, repeat. QF tick **100 Hz**. AOs are run-to-completion; a blocking handler starves everyone (LL 32).

**Core 1** (vehicle only for sensors) is a 1 kHz `while (true)` loop in `core1_sensor_loop()`: IMU (+ mag via bypass), baro at ÷32, GPS at ÷100, MCU temp at ÷1000, seqlock write, `busy_wait_us` pad to 1000 µs. Station/Relay Core 1 waits on `g_startSensorPhase` forever (Holzmann scheduler exemption).

---

## 2. Boot (until `QF_run`)

```
main()
  init_hardware()          // clocks, stdio, I2C, sensors, radio SPI, PSRAM, Core 1 launch
  init_application()       // cal, FD, anomalous-boot, init_core1_role(), baro auto-zero
  QF_init() / psInit
  start_active_objects()
  QF_run()                 // never returns
```

**Cross-core handshake** (`init_core1_role` + `core1_entry`) — still the protocol `rocketchip_boot.pml` proves (`p_no_premature_sensor_read`, `p_vehicle_core1_eventually_proceeds`):

1. Core 1: MPU stack guard, `multicore_lockout_victim_init()`, `g_core1LockoutReady = true`, then wait for `g_startSensorPhase` (vehicle: 10 s then `crash_record_capture(kCrashReasonCore1BootWait)` + reset).
2. Core 0 vehicle: `g_sensorPhaseActive = true`, `g_startSensorPhase = true`, then wait until `g_core1LockoutReady`.
3. Station/Relay: does **not** set the start flag; still waits for lockout (flash-safe test). `station_idle_tick_init()` if RX role.

Comment line numbers in the `.pml` header still say `main.cpp:340`; the stores now live at `main.cpp:309–322`. The **protocol is the same**.

`init_gps_early()` is gone (L2-P5). GPS is UART-first then I2C probe from `init_gps()` / sensor bring-up.

Anomalous-boot `kProbablyMidFlight` **skips** baro auto-zero (would bias main deploy).

---

## 3. Idle bridge (Core 0, every empty-queue pass)

From `qv_idle_bridge()` in `src/main.cpp`, in order:

1. Test-mode fault inject (no-op unless probe-armed).
2. **`watchdog_kick_tick()`** — PIO heartbeat. Stays here forever (not an AO).
3. **GPS UART reinit** if Core 1 requested it (`gps_uart_take_reinit_request`).
4. **`eskf_runner_tick()`** — seqlock read, every 5th IMU sample (200 Hz), predict + measurements + Mahony + confidence, then `QActive_publish_(SIG_SENSOR_DATA)`.
5. Station only: `AO_Telemetry_cmd_retry_tick()` + `station_idle_tick()` (GPS + MCU temp on Core 0).
6. **`diag_stats_msp_tick()`** — records Core 0 MSP; sampled **in idle**, so it does not see handler depth (CLA MSP 0 B).
7. **`rc_log_drain_to_cdc()`** — empty-ring fast path (LL 39).
8. **`__wfi()`** — wake on 100 Hz tick / USB / IRQ.

**CLA (60 s idle, DPS310 failed, since-boot bench counters):**

| Counter | avg | min | max | duty of 5 ms epoch |
|---------|-----|-----|-----|--------------------|
| `predict` (`g_eskf.predict` only) | 730 µs | 144 | 1090 | 14.6% |
| `full-tick` (`eskf_runner_fusion_cycle`) | 1899 µs | 255 | 70524 | 38% |

`full-tick` does **not** include `QActive_publish_` or AO handlers. Max is lifetime, not the 60 s window. IMU **997.2 Hz** over the window (Core 1 still I2C-bound).

---

## 4. Active objects (9)

Priorities from `start_active_objects()` (`src/main.cpp`). Queue high-water from CLA `'d'` on this board (idle now, high = since boot).

| Prio | AO | Rate | Queue (code) | CLA high | Role | Owns |
|------|----|------|--------------|----------|------|------|
| 9 | AO_FlightDirector | 100 Hz | 32 | 8 | Vehicle | HSM, guards, Go/No-Go, pyro intent |
| 8 | AO_Radio | 100 Hz | 32 | 8 | All | SX1276, RadioScheduler, `send_start`/`send_poll` |
| 7 | AO_RfManager | 10 Hz | 16 | **not in `diag_stats_dump`** | Vehicle+Station | ACQ/Tentative/Track/Degraded, LQ, station TX window |
| 6 | AO_HealthMonitor | 10 Hz | 8 | 1 | not Relay | 2-bit health, Core 1 vitality, Go/No-Go inputs |
| 5 | AO_Notify | 33 Hz | 16 | 4 | Vehicle | Intent resolver, beacon overlays |
| 4 | AO_Logger | 50 Hz | 32 | 5 | Vehicle | PCM ring, flight table |
| 3 | AO_Telemetry | 10 Hz | 8 | 2 | not Relay | CCSDS/MAVLink, RX decode, tracked commands |
| 2 | AO_LedEngine | 33 Hz | 8 | 4 | Vehicle | WS2812 3-layer compositor |
| 1 | AO_RCOS | 20 Hz | 16 | 1 | All | CLI / dashboard |

Start order is **not** priority order (Radio starts before FD). Station: no FD/Notify/Logger/LedEngine; Radio owns RSSI NeoPixels.

Pub/sub list: `docs/AO_ARCHITECTURE.md` (do not fork it here).

**Event spine (vehicle):**

```
Core1 seqlock write
  → idle eskf_runner_tick → SIG_SENSOR_DATA
      → FD, Logger, Telemetry
  FD → SIG_PHASE_CHANGE → Health, Notify, Logger
     → SIG_PYRO_FIRED / SIG_BEACON_ACTIVE
  Health → SIG_HEALTH_STATUS
  Notify → SIG_LED_PATTERN → LedEngine
  Telem → SIG_RADIO_TX → Radio → SIG_RADIO_RX → Telem + RfManager
```

---

## 5. Flight Director HSM

Nine phases (`flight_state.h`): Idle, Armed, Boost, Coast, DrogueDescent, MainDescent, Landed, Abort, **Fault**.

- **Abort:** operator / abort guard. PIO backup timers still deploy.
- **Fault:** not a normal transition. Hardfault / anomalous-boot / `Q_onError` degrade-in-place. ARM does not coexist with Fault. No chip-wide reset in flight (R-21).

SPIN `rocketchip_fd.pml` / `rocketchip_ao.pml` encode Idle…Abort + pyro ordering. They do **not** model `kFault` as a first-class phase. Pyro one-shot / drogue-before-main / no pyro in Idle are the proved subset.

This sitting: `rocketchip_fd.pml` printed `error: max search depth too small` (DFS `-m10000`) on every claim while still reporting `errors: 0`. The gate still counted those claims. Treat FD LTL as **pass with truncated depth**, not a fully explored statespace. `rocketchip_ao.pml` reached depth 706 with a complete search (`errors: 0`).

---

## 6. SPIN — proved subset (this sitting)

Master gate `tools/spin/run_stage_o_ao_spin.sh`: **`SPIN_OK_31`** (36 s, Cygwin). Six models:

| Model | What it is allowed to claim | Not in the model |
|-------|-----------------------------|------------------|
| `rocketchip_ao.pml` (11) | Pyro idle/once/order/ARM; logger/telem/LED get phase pubs; fault blocks ARM / latch / armed→safe | Radio, RfManager, Notify, RCOS, Core 1 body, ESKF math |
| `rocketchip_fd.pml` (8) | Same pyro + flight completes + no Landed without launch | Depth truncated this run |
| `rocketchip_rf_manager.pml` (5) | Track liveness, no TX in ACQ, hysteresis, no silent loss, no Track↔Degraded livelock | Full 10-slot LQ window (bucketed) |
| `rocketchip_station.pml` (2) | One in-flight command terminates; no double-clear | Multi-pending, MAVLink parser, TX-window arb (WB) |
| `rocketchip_boot.pml` (2) | No sensor loop before start flag; vehicle Core 1 proceeds or timeout-reset | MPU/lockout internals |
| `rocketchip_flash_protocol.pml` (3) | Core 1 locked during flash; I2C reset after; no I2C during flash | |

Queue depth in Promela is **[1]**; firmware queues are 8–32. Overflow is an empirical/LL-32 issue, not a SPIN claim.

---

## 7. Core 1 loop (vehicle)

Each 1 ms cycle (`kCore1TargetCycleUs = 1000`):

- Pause/reload if Core 0 holds I2C (`g_core1I2CPaused`).
- Else: IMU (+ mag), baro every 32, GPS every 100 (UART min interval 2 ms), MCU temp every 1000, seqlock write always (even on IMU fail).
- IMU: 10 consecutive fails → bus recover; 50 → device reset.
- Baro: same escalate; **3** `baro_dps310_start_continuous()` re-inits then `g_baroInitialized = false`. April RBM GAP-1 (no baro recovery) is **stale** — reinit exists; this CLA board still ended DPS310-dead after 200 errors.

CLA window: IMU 997.2 Hz, mag 99, GPS 10, baro **0**.

---

## 8. CLI (AO_RCOS)

Vehicle boots **kMenu**. Station boots **kAnsi** dashboard; scripts must send `x` first (`enter_cli_menu`).

Main menu does **not** handle `s`/`e`. Those are **debug** (`q`): `s` sensor/ESKF bench, `e` 1 Hz live, `d` `diag_stats_dump` (SPI RegVersion — after a soak, not during).

---

## 9. Blocking / stall

| Path | Where | Bound | Notes |
|------|-------|-------|-------|
| `flash_safe_execute` | Core 0 | ~1–150 ms | Core 1 lockout; I2C reset after. SPIN flash model. |
| Calibration save | Core 0 | flash-scale | Pause Core 1 I2C |
| `rfm95w_send_start` / `_poll` | AO_Radio | non-blocking | Live TX path. Blocking `rfm95w_send()` **has no callers** in `src/`. April GAP-3 is closed for AOs. |
| Core 1 I2C IMU | Core 1 | ~0.8 ms | Dominates the 1 ms budget |
| `__wfi` | Core 0 idle | until IRQ | Not a stall of AOs |

---

## 10. Error outcomes (recover / degrade / halt)

Every path should end in one of: **(a)** recovered, **(b)** degraded with named loss, **(c)** halt/reset.

| Trigger | Outcome |
|---------|---------|
| IMU I2C fail | (a) bus recover / device reset; (b) ESKF skips if accel/gyro invalid |
| Baro fail | (a) up to 3 continuous re-inits; (b) then baro dead, alt from GPS/ZUPT |
| GPS parse/UART | (b) skip cycle; UART reinit from idle if requested |
| ESKF diverge / P-growth | (a) re-init with backoff brake; (b) no nav, Notify unhealthy |
| Seqlock fail | skip this fusion epoch |
| Health critical (pre-launch) | auto-DISARM; (b) no ARM |
| PIO watchdog | IRQ flag; **`Q_onError` halts** — no in-flight AIRCR (R-21). PIO backup pyro timers keep counting. |
| Core 1 boot wait timeout | (c) crash record + reset (vehicle) |
| `rc_log` ring full | drop bytes (CLA: dropped=0, high_water=1663 B on that soak) |

---

## 11. Station extras

Not in the vehicle CLA soak. Firmware: `station_idle_tick` on Core 0 idle (GPS + MCU temp), HealthMonitor with capability masks (`[N/A]` vs `[FAIL]`), AO_RfManager gates station TX (`AO_RfManager_next_tx_window_us`). SPIN station model is single-command in flight only.

---

## 12. Gaps (current)

| ID | Item | Status |
|----|------|--------|
| RBM-1 | `diag_stats_dump` omits `AO_RfManager` queue | Live gap |
| RBM-2 | MSP tick is idle-only — no handler/Core 1 HWM | CLA cannot claim stack margin |
| RBM-3 | Per-AO WCET / QV coincidence / ESKF period jitter | Not measured (would need new probes) |
| RBM-4 | `rocketchip_fd.pml` DFS depth 9999 | Gate still `errors: 0`; search truncated |
| RBM-5 | RadioScheduler vs vehicle RX window | Characterized Stage T; RfManager anchors TX; full fix still post–CCSDS / WB |
| RBM-6 | Battery ADC | Hardware not wired |
| April GAP-1 baro | 3× reinit then dead | Recovery **exists**; this CLA board still dead |
| April GAP-3 blocking TX | `rfm95w_send()` unused | Closed for AO path |

April Graphviz under `docs/audits/cla_rbm/dot/` is still a useful **picture** of 8-AO flow. It omits RfManager, `rc_log` drain, and `station_idle_tick`. Do not treat those dots as complete.

---

## 13. Refresh

- **CLA:** `python scripts/cla_collect.py --duration 60` (debug `s` / idle / `s` / `d`). Healthy baro required before comparing to 2026-03.
- **SPIN:** Cygwin `bash tools/spin/run_stage_o_ao_spin.sh` → `SPIN_OK_31`.
- **This doc:** when AO start/prio/idle-bridge order changes, or a new SPIN model lands. Edit here, not the 2026-04 audit file.
