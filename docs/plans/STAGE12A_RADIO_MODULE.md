# Stage 12A: Radio Module Extraction + Fruit Jam GCS

## Context

Stage 12 splits into two sub-stages:
- **12A:** Radio module extraction, AO split, Fruit Jam ground station firmware (this plan)
- **12B:** Linux-based GCS (RPi + Yamcs + OpenMCT) — the existing IVP-92–97 placeholders, renumbered

12A is foundational — the radio module extraction and non-blocking TX benefit both the vehicle firmware and the Linux GCS. 12A comes first, 12B builds on top.

### Council Reviews (All Unanimous)

**Council 1 — Radio Universality** (NASA/JPL, Professor, ArduPilot, Student):
Radio is universal. Three Jobs (Vehicle/Station/Relay). RadioConfig separate from MissionProfile. Explicit role selection. Command authority strict.

**Council 2 — RadioScheduler** (NASA/JPL, Professor, CubeSat, ArduPilot):
Thin half-duplex SM. APID routing in Telemetry Service. Non-blocking TX critical path. `submit_packet()` API from day one. FSK = separate driver, Titan tier. Config two-level.

**Council 3 — Final Plan** (NASA/JPL, Professor, CubeSat, ArduPilot):
A1: Dynamic event pools. A2: TX-busy drop+log. A3: TX timeout → kRxWindow. A4: SIG_GCS_CMD signal. A5: Probe-first radio init. A6: Step 2 sub-commits. 100Hz AO_Radio tick. Relay link-layer only.

---

## IVP Allocation

| Range | Sub-Stage | Content |
|-------|-----------|---------|
| IVP-92–98 | **12A: Radio Module + Fruit Jam GCS** | Radio extraction, AO split, station firmware |
| — | **12B: Linux GCS** | Yamcs, OpenMCT, Pi image (numbered when planned) |
| — | **13: Pre-Flight Polish** | Steps preserved, numbered when planned |
| — | **14: Field Tuning** | Steps preserved, numbered when planned |

---

## Stage 12A IVP Definitions

### Summary Table

| Step | Title | Brief Description |
|------|-------|------------------|
| IVP-92 | Non-Blocking Radio TX | Split rfm95w_send() into send_start/send_poll. TX failure escalation. |
| IVP-93 | AO_Radio Extraction | New AO_Radio with RadioScheduler. Vehicle mode: TX-priority with RX windows. |
| IVP-94 | AO_Telemetry Protocol Refactor | Protocol-only AO. SIG_RADIO_TX/RX event flow. SIG_GCS_CMD stub. |
| IVP-95 | Three-Job Device Role | DeviceRole enum (Vehicle/Station/Relay). Station + Relay wiring. |
| IVP-96 | RadioConfig in Mission Profile | [radio] section in .cfg. Generator derives SF/BW/CR. RadioConfig struct. |
| IVP-97 | Fruit Jam Station Enhancements | 5x NeoPixel RSSI bar, station CLI, distance-to-rocket |
| IVP-98 | Relay Job | Link-layer relay in AO_Radio. Seq dedup. LED feedback. |

**Gate:** Fruit Jam receives CCSDS over LoRa, re-encodes to MAVLink, QGC displays live data. Vehicle TX non-blocking (AO queue depth < 4). Relay extends range. All 598+ host tests pass.

---

### IVP-92: Non-Blocking Radio TX

**Prerequisites:** IVP-57 (radio driver verified), Stage 11 complete

**Implement:** Split the blocking `rfm95w_send()` (50-150ms LoRa airtime) into two non-blocking functions:

1. `rfm95w_send_start(dev, data, len)` — Write FIFO, set TX mode, return immediately (~200µs). Returns false if radio not initialized or len > max payload.

2. `rfm95w_send_poll(dev)` → `TxPollResult` — Read RegIrqFlags register (not GPIO DIO0 — latched, can't be missed). Returns `kBusy`, `kDone`, or `kTimeout`. Council Amendment: read IRQ register, not GPIO.

Keep existing `rfm95w_send()` as convenience wrapper (calls start + busy-waits poll) for backward compatibility in existing code and tests.

TX failure escalation (same pattern as IMU consecutive-fail counter):
- 1 consecutive timeout → DBG_ERROR log
- 3 consecutive → `rfm95w_init()` reinit attempt
- 5 consecutive → set error flag visible to health system

Also add missing runtime-change functions:
- `rfm95w_set_spreading_factor(dev, sf)` — SF6-SF12
- `rfm95w_set_coding_rate(dev, cr)` — CR 4/5 through 4/8

**Runtime changeability:** All SX1276 LoRa parameters (SF, BW, CR, frequency, power, sync word, preamble) are register writes that require Standby mode. The RadioScheduler naturally enters Standby between TX/RX transitions, so all settings are changeable at runtime with zero additional overhead. The AO_Radio split makes this clean — `SIG_RADIO_CONFIG` event carries new params, AO_Radio applies them during the next Standby window.

**Boot-time only:** Protocol (CCSDS vs MAVLink) — both TX and RX must agree on packet format. Switching mid-flight requires coordinated change on both sides with no way to signal it if the link is already using the old format. No good reason to switch mid-flight.

**Phase-changeable (both ends must agree on SF):** SF, BW, CR, rate. The RX station tracks the vehicle's flight_state field from received packets and switches its own radio config to match when the vehicle transitions phases. If the station misses the transition packet, it's temporarily deaf until manual resync.

**Files:**
- `src/drivers/rfm95w.h` — Add `TxPollResult` enum, `send_start()`, `send_poll()`, `set_spreading_factor()`, `set_coding_rate()` declarations
- `src/drivers/rfm95w.cpp` — Implement split + new setters. Existing `rfm95w_send()` wraps start+poll.

**[GATE]:**
- `rfm95w_send()` wrapper behavior identical to pre-change (soak 60s, verify TX count + timing)
- `send_start()` returns in <500µs (measure with `time_us_64()`)
- `send_poll()` returns `kDone` after expected airtime (±10ms at SF7/BW250)
- `send_poll()` returns `kTimeout` after 200ms if TX fails
- Escalation: force 5 consecutive failures → verify error flag set
- All host tests pass

**[DIAG]:** If `send_poll()` never returns `kDone`: check RegIrqFlags TxDone bit (bit 3). If bit never sets, check PA config (PA_BOOST vs RFO). If `send_start()` takes >1ms: SPI bus contention — verify no other SPI transactions in flight.

---

### IVP-93: AO_Radio Extraction

**Prerequisites:** IVP-92 (non-blocking TX), IVP-80 (AO_Telemetry exists)

**Implement:** New Active Object `AO_Radio` that owns radio hardware and the RadioScheduler half-duplex state machine. Extracts all radio hardware concerns from main.cpp (~120 lines) and AO_Telemetry.

**RadioScheduler** (~100 lines, `include/rocketchip/radio_scheduler.h`):
Half-duplex state machine, protocol-agnostic (never inspects packet contents).
- `kIdle` — Standby or init failure (probe-first: post SIG_RADIO_STATUS "no hardware") [A5]
- `kTxPending` — Packet received via SIG_RADIO_TX, waiting for TX slot
- `kTxActive` — `send_start()` called, polling `send_poll()` each tick
- `kRxWindow` — Listening between TX slots (Vehicle mode)
- `kRxContinuous` — Always listening (Station/Relay mode)

**AO_Radio** (~150 lines, `src/active_objects/ao_radio.cpp`):
- Tick rate: **100Hz fixed** (every QF tick). No-op cost ~10µs. [Council 3]
- Owns: `rfm95w_t` radio handle, `RadioScheduler`, init flag, escalation counter
- On SIG_RADIO_TX: if kTxActive → **drop and log** [A2]; else `send_start()` → kTxActive
- In kTxActive: `send_poll()` each tick. kDone → kRxWindow. kTimeout → **kRxWindow** (not kIdle — receiving "abort" > sending telemetry) [A3], increment escalation.
- In kRxWindow/kRxContinuous: poll `rfm95w_available()`, on packet → post SIG_RADIO_RX to AO_Telemetry
- Link quality → post SIG_LED_PATTERN to AO_LedEngine (replaces atomic overlay in main.cpp)

**New signals** (`ao_signals.h`):
```
SIG_RADIO_TX     — Encoded packet ready (Telem → Radio)
SIG_RADIO_RX     — Raw packet received (Radio → Telem)
SIG_RADIO_STATUS — Link quality (Radio → LED, CLI)
```

**Event pools** [A1] — `QF_poolInit()` at startup:
- Pool 1: `RadioTxEvt` × 3 (134B each)
- Pool 2: `RadioRxEvt` × 3 (137B each)

**Queue depth:** 8 (handlers complete in <1ms with non-blocking TX)

**Extraction from main.cpp:**
- Lines 238-251: Radio globals → AO_Radio-owned state
- Lines 1915-1935: Radio init → AO_Radio initial transition
- Lines 3226-3257: `telemetry_radio_tick()` → AO_Radio tick handler
- NeoPixel RX overlay → SIG_LED_PATTERN posting

**Files:**
- `src/active_objects/ao_radio.h` — **NEW**
- `src/active_objects/ao_radio.cpp` — **NEW** (~150 lines)
- `include/rocketchip/radio_scheduler.h` — **NEW** (~100 lines)
- `include/rocketchip/ao_signals.h` — Add SIG_RADIO_TX/RX/STATUS + event structs
- `src/main.cpp` — Remove radio globals, init, tick bridges. Add `AO_Radio_start()`.
- `CMakeLists.txt` — Add ao_radio.cpp

**[GATE]:**
- Vehicle build compiles, binary size within ±1KB of pre-extraction
- HW soak: 60s TX, packet count + timing match pre-extraction
- AO queue depth never exceeds 4 during TX (was overflowing at 16 with blocking)
- Radio absent at boot: AO_Radio stays in kIdle, no crash, SIG_RADIO_STATUS posted [A5]
- All host tests pass

**[DIAG]:** Queue overflow (qf_actq id=130) = handler still blocking — verify `send_poll()` path, not `rfm95w_send()`. SIG_RADIO_TX dropped = TX still active from previous packet — expected at high rates, logged. Radio init fail after extraction = SPI init ordering — verify SPI bus init happens before QF_run().

---

### IVP-94: AO_Telemetry Protocol Refactor

**Prerequisites:** IVP-93 (AO_Radio exists)

**Implement:** Refactor AO_Telemetry to be protocol-only — encoding, APID mux, rate dividers. No radio hardware references. No rfm95w includes.

AO_Telemetry owns:
- `TelemetryServiceState` (encoding + rates)
- `TelemetryState` snapshot (from sensor pipeline)
- `MavlinkEncoder` (for direct USB output)

Behavior:
- On tick (2-10Hz): encode CCSDS/MAVLink → allocate `RadioTxEvt` from pool → post SIG_RADIO_TX to AO_Radio
- On SIG_RADIO_RX: decode CCSDS → dispatch by APID → output CSV/MAVLink USB
- On SIG_RADIO_RX (command packet, future): decode → post **SIG_GCS_CMD** to AO_FlightDirector [A4]. Handler is a stub in 12A.
- On SIG_TELEM_FRAME: update telemetry snapshot from sensor pipeline
- Continues encoding for USB MAVLink even if radio absent [A5]

**New signal** (`ao_signals.h`):
```
SIG_GCS_CMD — Uplink command from GCS (Telem → FlightDirector). Stub handler for 12A.
```

Remove `extern "C" telemetry_radio_tick()` and `extern "C" mavlink_direct_tick()` bridges entirely.

**Design-now API** (trivial initial implementation, full queue in Stage 13/14):
```cpp
void telemetry_submit_packet(uint16_t apid, uint8_t priority,
                              const uint8_t* data, uint8_t len);
```
Initial implementation ignores priority and only handles APID 0x001 (nav).

**Files:**
- `src/active_objects/ao_telemetry.cpp` — Refactor to protocol-only
- `src/active_objects/ao_telemetry.h` — Remove radio refs, add state accessor
- `include/rocketchip/ao_signals.h` — Add SIG_GCS_CMD + GcsCmdEvt
- `src/main.cpp` — Remove mavlink_direct_tick(), route CLI through AO accessors

**[GATE]:**
- Vehicle TX: identical packet content + rate to pre-refactor
- Station RX: CSV + MAVLink output identical to pre-refactor
- USB MAVLink output works even with radio absent
- SIG_GCS_CMD signal defined, stub handler logs receipt
- `submit_packet()` API compiles, initial impl passes nav packets through
- All host tests pass

---

### IVP-95: Three-Job Device Role

**Prerequisites:** IVP-94 (AO split complete)

**Implement:** `DeviceRole` enum and three compile-time Job configurations. Each Job determines which AOs are active and how the radio is used.

```cpp
enum class DeviceRole : uint8_t {
    kVehicle = 0,  // FD + Logger + LedEngine + Telemetry + Radio (TX + RX windows)
    kStation = 1,  // LedEngine + Telemetry + Radio (RX continuous)
    kRelay   = 2,  // LedEngine + Radio (RX continuous + relay TX)
};
```

Compile-time selection (CMake defines):
- Default: Vehicle
- `-DROCKETCHIP_JOB_STATION=1`: Station
- `-DROCKETCHIP_JOB_RELAY=1`: Relay

Station: AO_Radio in kRxContinuous from init. No AO_FlightDirector, no AO_Logger. AO_Telemetry decodes + outputs.
Relay: AO_Radio in kRxContinuous. No AO_Telemetry (relay is link-layer only). IVP-98 implements relay logic.

Flash-stored DeviceRole override (future, not this IVP): CLI `role` command saves to flash, overrides compile-time default.

**Files:**
- `include/rocketchip/job.h` — Add DeviceRole enum
- `include/rocketchip/job_relay.h` — **NEW** — Relay constants
- `src/main.cpp` — Conditional AO_start() calls based on DeviceRole

**[GATE]:**
- Vehicle build: all AOs start, TX mode, sensors active. Identical to pre-change.
- Station build (`-DROCKETCHIP_JOB_STATION=1`): compiles, RX continuous, MAVLink USB output, no FD/Logger
- Relay build (`-DROCKETCHIP_JOB_RELAY=1`): compiles, RX continuous, no Telemetry/FD/Logger
- SPIN model: 6/6 safety properties pass (radio extraction doesn't touch safety logic)
- All host tests pass

---

### IVP-96: RadioConfig in Mission Profile

**Prerequisites:** IVP-95 (Jobs defined), IVP-74 (profile generator exists)

**Implement:** Add `[radio]` section to Mission Profile `.cfg` files. Generator reads radio config and outputs `RadioConfig` struct alongside `MissionProfile` in `mission_profile_data.h`.

User-facing config (simple):
```ini
[radio]
protocol = ccsds          # ccsds | mavlink
nav_rate_hz = 5           # 2, 5, or 10
power_dbm = 20            # 2-20
```

Generator derives RF parameters from protocol + rate:
- Protocol + rate → minimum BW/SF to achieve throughput
- Warns/clamps if advanced overrides (V2) conflict with rate
- No advanced overrides exposed in V1

`RadioConfig` struct:
```cpp
struct RadioConfig {
    rc::EncoderType protocol;  // kCcsds or kMavlink
    uint8_t nav_rate_hz;       // 2, 5, or 10
    uint8_t power_dbm;         // 2-20
    uint8_t spreading_factor;  // Derived: 6-12
    uint16_t bandwidth_khz;    // Derived: 125, 250, or 500
    uint8_t coding_rate;       // Derived: 5-8 (CR 4/x)
};
```

AO_Radio reads `RadioConfig` at init instead of hardcoded constants. AO_Telemetry reads protocol + rate.

**Runtime radio config changes:**
Config change blackout is ~300µs total (Standby transition ~100µs + 3 register writes ~30µs + mode relock ~100µs). At 10Hz TX this is 0.3% of a cycle — fits in the RadioScheduler's natural Standby window between packets. Zero packets lost, no buffering needed.

**GCS-initiated config changes (primary path):**
When a bidirectional GCS link exists, radio config changes are commanded by the GCS — not auto-scheduled by flight phase. The GCS has the best view of link quality (RSSI, SNR, packet loss from both ends) and can make informed decisions:
1. GCS detects degrading link → commands vehicle to switch to longer-range config (higher SF)
2. GCS sees strong link → commands higher-rate config (lower SF, wider BW)
3. Operator manual override via CLI or QGC

Change flow: GCS sends config command to vehicle (uplink, IVP-62 path) → vehicle applies + ACKs → GCS applies matching config to station (local) → both sides in sync. No assumptions about phase, no missed transition packets.

**Phase-scheduled config (one-way fallback):**
For core tier deployments without a bidirectional GCS, the `.cfg` can define per-phase radio overrides. AO_Radio subscribes to SIG_PHASE_CHANGE and applies phase-specific config. The station tracks the vehicle's `flight_state` from received packets to stay in sync. This is the simpler, more assumption-heavy path — used only when no GCS uplink is available.

```ini
[radio]
config_source = gcs          # gcs (primary) | phase_schedule (1-way fallback)
```

**Files:**
- `scripts/generate_profile.py` — Parse [radio] section, derive RF params, output RadioConfig
- `profiles/rocket.cfg` — Add [radio] section (ccsds, 5Hz, 20dBm)
- `profiles/hab.cfg` — Add [radio] section (ccsds, 2Hz, 20dBm, SF9 for range)
- `src/flight_director/mission_profile.h` — Add RadioConfig struct (sibling, not inside MissionProfile)
- `src/flight_director/mission_profile_data.h` — Regenerated with RadioConfig

**[GATE]:**
- Generator produces correct RadioConfig from rocket.cfg and hab.cfg
- Vehicle boot uses profile-driven radio config (verify SF/BW/power match .cfg)
- Changing .cfg → rebuild → different radio behavior confirmed
- All host tests pass

---

### IVP-97: Fruit Jam Station Enhancements

**Prerequisites:** IVP-95 (Station Job), Fruit Jam hardware

**Implement:** Station-specific features for the Fruit Jam board:

**97a. 5x NeoPixel RSSI Bar:**
Fruit Jam has 5 NeoPixels (GPIO 32, PIO gpiobase=16). Map RSSI range to 1-5 lit pixels:
- -40 to -70 dBm: 5 pixels green (strong)
- -70 to -90 dBm: 3 pixels yellow (marginal)
- -90 to -110 dBm: 1 pixel red (weak)
- No packets for >5s: all off, slow red pulse

**97b. Station CLI Extensions:**
- `r` key: RX stats (count, RSSI, SNR, gap, CRC errors) — already works
- `t` key: toggle CSV/MAVLink output
- `g` key: GPS status (if GPS connected to Fruit Jam STEMMA QT I2C0)
- `d` key: distance-to-rocket (Haversine: ground GPS vs received lat/lon)

**97c. GPS Ground Position + Altitude Cross-Reference:**
If PA1010D is connected via STEMMA QT (I2C0, GPIO 20/21), use existing `gps_pa1010d` driver (probe-first). Provides:
- Ground station lat/lon/alt
- Distance-to-rocket: Haversine from ground GPS vs received vehicle lat/lon
- **Altitude cross-reference:** `vehicle_AGL ≈ vehicle_baro_alt_mm - station_gps_alt_mm`. More stable than vehicle's own AGL (which uses launch-site baro as reference and drifts with weather). Station GPS vertical accuracy ~3-5m, but useful as sanity check and for display.
- Future: feed station GPS position into MAVLink GLOBAL_POSITION_INT for QGC ground station marker

**[GATE]:**
- RSSI bar visually indicates signal strength at varying distances
- CLI 'd' shows distance and altitude cross-reference when both GPSes locked
- Station soak: 10 min RX, 0 CRC errors, RSSI bar tracks expected levels
- QGC displays attitude + position + flight state from Fruit Jam USB

---

### IVP-98: Relay Job

**Prerequisites:** IVP-95 (Relay Job defined), IVP-93 (AO_Radio)

**Implement:** Link-layer relay in AO_Radio (~50-100 lines). No AO_Telemetry involvement — relay validates packet integrity without decoding payload. [Council 3: link-layer only]

Relay logic (inside AO_Radio kRxContinuous handler):
1. Receive packet via `rfm95w_recv()`
2. Validate: CCSDS sync marker + CRC-16-CCITT (6 bytes header check, no payload decode)
3. Dedup: check 14-bit sequence counter vs last relayed seq. Drop duplicates.
4. Re-TX: post SIG_RADIO_TX to self → `send_start()` → kTxActive
5. LED: blue blink on successful relay

**[GATE]:**
- Relay build compiles (`-DROCKETCHIP_JOB_RELAY=1`)
- 3-node test: Vehicle TX → Relay → Station RX. Station receives packets.
- Duplicate packets dropped (verify seq counter dedup)
- Relay LED blinks blue on each forwarded packet
- Relay binary size: <60KB (minimal AOs)

---

## Stage 12B / 13 / 14

Downstream stages keep their steps in order but IVP numbers are deferred — assigned when each stage is planned. See `docs/IVP.md` for the current step lists.

---

## Critical Files Summary

| File | IVP | Action |
|---|---|---|
| `src/drivers/rfm95w.h` | 92 | MODIFY — send_start/send_poll |
| `src/drivers/rfm95w.cpp` | 92 | MODIFY — split blocking send |
| `src/active_objects/ao_radio.h` | 93 | NEW |
| `src/active_objects/ao_radio.cpp` | 93 | NEW (~150 lines) |
| `include/rocketchip/radio_scheduler.h` | 93 | NEW (~100 lines) |
| `include/rocketchip/ao_signals.h` | 93+94 | MODIFY — new signals + event structs |
| `src/active_objects/ao_telemetry.cpp` | 94 | MODIFY — protocol-only |
| `src/active_objects/ao_telemetry.h` | 94 | MODIFY |
| `src/main.cpp` | 93+94+95 | MODIFY — remove ~120 lines |
| `include/rocketchip/job.h` | 95 | MODIFY — DeviceRole |
| `include/rocketchip/job_relay.h` | 95 | NEW |
| `scripts/generate_profile.py` | 96 | MODIFY — [radio] section |
| `profiles/rocket.cfg` | 96 | MODIFY — add [radio] |
| `profiles/hab.cfg` | 96 | MODIFY — add [radio] |
| `CMakeLists.txt` | 93+95 | MODIFY |

**Unchanged:** `src/telemetry/telemetry_service.h/cpp`, `src/telemetry/telemetry_encoder.h/cpp`, `src/drivers/spi_bus.h/cpp`

---

## Verification Summary

- [ ] IVP-92: Non-blocking TX soak 60s, send_start <500µs, escalation counter works
- [ ] IVP-93: Vehicle binary ±1KB, AO queue depth <4 during TX, radio-absent graceful
- [ ] IVP-94: Identical TX/RX behavior, USB MAVLink without radio, submit_packet() compiles
- [ ] IVP-95: Vehicle/Station/Relay all compile, SPIN 6/6 pass
- [ ] IVP-96: Generator produces correct RadioConfig, profile-driven radio params
- [ ] IVP-97: RSSI bar, CLI distance, 10-min station soak 0 CRC errors
- [ ] IVP-98: 3-node relay test, seq dedup, <60KB relay binary

---

## Council Amendments Registry

### Council 1: Radio Universality
C1-1. Command authority: validate vs flight phase. No pyro via radio.
C1-2. Explicit role selection. Flash-stored override (future).
C1-3. Relay as distinct Job.

### Council 2: RadioScheduler
C2-1. Non-blocking TX (critical path).
C2-2. TX failure escalation: 1→log, 3→reinit, 5→error.
C2-3. `submit_packet()` API from day one.
C2-4. RadioScheduler zero protocol awareness.

### Council 3: Final Plan
C3-A1. QP/C dynamic event pools (RadioTxEvt/RadioRxEvt × 3).
C3-A2. TX-busy: drop and log.
C3-A3. TX timeout → kRxWindow (prioritize receiving commands).
C3-A4. SIG_GCS_CMD for uplink dispatch. Stub handler 12A.
C3-A5. Radio init failure: probe-first, Telem continues for USB.
C3-A6. Sub-commits for Vehicle/Station/Relay wiring.
C3-R1. AO_Radio 100Hz fixed tick.
C3-R2. Relay link-layer only (CRC, no decode).
C3-R3. IRQ register for TX completion, not GPIO DIO0.

### Telemetry Protocol Council (2026-02-27)
TP-1. Two-tier APID: Nav 5-10Hz, Diag 0.5-1Hz. Stage 13/14.
TP-2. FSK bitstream: separate driver, Titan tier.
TP-3. Transfer Frames over serial for wired GCS (12B).
TP-4. Config per profile: protocol+rate+power in .cfg.

---

## Pre-Implementation: IVP Document Update ✓

**DONE.** `docs/IVP.md` updated:
1. ✓ Stage 12 split into 12A (IVP-92–98) and 12B (unnumbered placeholders)
2. ✓ Stages 13-14 steps preserved in order, IVP numbers removed (assigned when planned)
3. ✓ Regression test matrix updated
4. ✓ Plan copied to repo: `docs/plans/STAGE12A_RADIO_MODULE.md`

---

## Companion Document

`docs/RADIO_TELEMETRY_STATUS.md` — Comprehensive tracking for all radio/telemetry work.
