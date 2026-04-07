# Stage 7: Radio & Telemetry — Implementation Plan

**Saved:** 2026-03-07 (after IVP-60 completion)
**Source:** `.claude/plans/polished-jingling-stardust.md`

## Status

| IVP | Description | Status |
|-----|-------------|--------|
| IVP-57 | Radio driver (RFM95W) | **COMPLETE** — 100% loopback, through-wall, 8 gates |
| IVP-58 | Telemetry encoder (CCSDS) | **COMPLETE** — 54B encoder, 19 host tests. MAVLink stub only |
| IVP-59 | Telemetry service (TX) | **COMPLETE** — 2/5/10 Hz CCSDS, replaced test TX, 20 dBm |
| IVP-60 | RX mode + CCSDS decode | **COMPLETE** — Mission Profile, CSV output, NeoPixel, 5-min soak |
| IVP-61 | MAVLink re-encode + QGC + range test | **NEXT** — needs research |
| IVP-62 | Bidirectional commands | Scope risk — can slide to Stage 8 |

## Context

Stage J (Fruit Jam HAL) is complete. The same `main.cpp` now builds and runs on both Feather RP2350 HSTX and Fruit Jam with compile-time board abstraction. All Stage 7 features are board-agnostic — they activate based on radio presence + Mission Profile mode, not board identity.

**Architecture established (IVP-60):**
- Compile-time vehicle/station selection via Mission Profile headers
- `ROCKETCHIP_MISSION_STATION=1` CMake define selects station behavior
- `if constexpr (kRadioModeRx)` compiles out unused TX/RX paths
- CCSDS over LoRa is the primary telemetry format (vehicle→station)
- Station outputs CSV telemetry over USB serial
- Register-based IRQ flags check for `rfm95w_available()` (GPIO DIO0 unreliable on Fruit Jam)

## Telemetry Budget Reference

### Default Config: SF7/BW250, CCSDS 54B

| Metric | Value |
|--------|-------|
| Max LoRa payload | 128 bytes |
| Airtime per CCSDS packet (54B) | ~25 ms |
| Duty cycle @ 10 Hz CCSDS | ~25% |
| Airtime per MAVLink 3-msg (105B) | ~45 ms |
| Duty cycle @ 10 Hz MAVLink | ~45% |
| TX power (field) | 20 dBm |
| SX1276 sensitivity @ SF7/BW250 | -120 dBm |
| Link margin @ 10 km (with antennas) | +29.3 dB |

**BW125 available** as Mission Profile option for HAB/long-range at reduced rate (5 Hz max).

---

## IVP-61: MAVLink Re-Encode + QGC Validation + Range Test

**Goal:** Add MAVLink output to RX mode for QGC/Mission Planner compatibility. End-to-end: TX board → LoRa → RX board → USB → QGC displays attitude + map. Plus mandatory range test.

**Implementation:** Add fastmavlink library. Complete MAVLink encoder stub. In RX mode, re-encode decoded CCSDS packets as MAVLink v2 and output on USB CDC. CLI suppressed in RX mode (USB is pure MAVLink binary).

**MAVLink IDs:** system_id=1, component_id=1. QGC auto-detects from HEARTBEAT.

**New dependency:** `lib/fastmavlink/` git submodule. Cherry-pick 3 messages: HEARTBEAT, ATTITUDE_QUATERNION, GLOBAL_POSITION_INT.

**Files:**
- `lib/fastmavlink/` — NEW submodule
- `include/rocketchip/telemetry_encoder.h` — MODIFY (complete MAVLink encoder)
- `src/telemetry/telemetry_encoder.cpp` — MODIFY (MAVLink encode_nav implementation)
- `src/telemetry/telemetry_service.cpp` — MODIFY (RX mode outputs MAVLink binary instead of CSV)
- `test/test_telemetry_encoder.cpp` — MODIFY (MAVLink wire format tests)
- `CMakeLists.txt` — MODIFY (fastmavlink include path)

### HW Verification Gate

1. Flash TX at 20 dBm, 10 Hz CCSDS
2. Connect RX USB to PC, launch QGC
3. QGC serial connection on RX COM port, 115200 baud
4. **Attitude test:** Tilt TX board — QGC attitude indicator tracks
5. **GPS test:** QGC map shows position marker (if GPS fix)
6. **Stability soak:** 10 min continuous — no disconnects, no stale data
7. **Range test (MANDATORY):**
   - Walk TX board to max distance (target 500m+)
   - Record RSSI, SNR, packet loss at 10+ distance points (GPS-verified)
   - Test at BW250 primary, optionally BW125 for comparison
   - Plot RSSI vs distance, compare to FSPL model
   - Data → `docs/benchmarks/RANGE_TEST_RESULTS.md`

---

## IVP-62: Bidirectional Commands (Scope Risk — Can Slide)

**Goal:** Uplink command channel from ground station to flight computer. Reception + ACK/NACK. Full command execution deferred to Stage 8.

**RX window strategy:** RX windows enabled **only at <=2 Hz** (ground/IDLE). At 10 Hz flight rate, pure TX.

**Command packet format (uplink):**
```
[1B cmd_id] [1B seq] [0-16B payload] [2B CRC-16] = 4-20 bytes
```

**Initial commands:** HEARTBEAT_REQ, ARM (ACK-only), DISARM (ACK-only), PARAM_GET, PARAM_SET, CAL_START (ACK-only).

### HW Verification Gate

1. Ground station CLI sends HEARTBEAT_REQ → ACK within 500ms
2. 100 commands at 1/sec → >90% ACK rate
3. PARAM_SET + PARAM_GET round-trip
4. Unknown command → NACK
5. Downlink uninterrupted during uplink
6. 5-min soak: 10 Hz downlink + periodic uplink

---

## Council Reviews

### Original (pre-Stage J)
CubeSat Startup Engineer, Advanced Hobbyist Rocketeer, Retired NASA/JPL Avionics Lead, Senior Aerospace Student. **Approved unanimously.** Amendments A1-A5 incorporated.

### Focused Review (2026-03-07, post-Stage J)
Same 4 personas. Reviewed the RX-mode-in-main.cpp architecture change. **Consensus on all 5 questions:**

1. **Auto-detect sufficient:** `has_radio && !has_imu → RX` cleanly partitions all hardware configs. Mission Profile override is future extension, not v1.
2. **No failure modes:** Watchdog survives (Core 1 alive unconditional). ESKF inert. Logging empty. Flash ops safe.
3. **Core 1 stays running:** Disabling breaks watchdog handshake. Empty loop costs <1% CPU.
4. **Defer MAVLink binary output:** Text/CSV first (IVP-60). MAVLink re-encode at IVP-61 when QGC needed. Avoids USB CDC multiplexing.
5. **NeoPixel adequate with blink overlay:** Solid green, 2Hz yellow, fast red. Document thresholds in banner.

**Key recommendation:** Add `[MODE] TX/RX` to boot banner. Separate receive/decode from re-encode in code structure.

**Risk:** IVP-62 (bidirectional) is scope risk. Can slide to Stage 8 without breaking QGC demo.
