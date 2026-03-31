# Radio & Telemetry Status

**Purpose:** Single tracking document for all radio, telemetry, and ground station work — done, planned, and deferred. Prevents decisions from being buried in council minutes.

**Last Updated:** 2026-03-30

---

## Architecture Overview

### Three-Layer Telemetry Model (Council-approved 2026-02-27)

1. **Onboard Log:** PCM fixed frames → PSRAM ring buffer → flash post-landing (200Hz, full ESKF state + raw sensors)
2. **Live Telemetry:** CCSDS Space Packets → LoRa (5-10Hz, attitude + position + health)
3. **Post-Flight:** Decommutate PCM log → repackage as CCSDS → replay to OpenMCT at full rate

### AO Stack (3 council reviews, 2026-03-30)

```
Application (FD, Logger) → submit_packet(apid, pri, data, len)
         ↓ SIG_TELEM_FRAME
AO_Telemetry (protocol) → CCSDS/MAVLink encoding, APID mux, rate dividers
         ↓ SIG_RADIO_TX / ↑ SIG_RADIO_RX    ↓ SIG_GCS_CMD (uplink)
AO_Radio (hardware)     → RadioScheduler     AO_FlightDirector
         ↓ SPI                               (stub handler for 12A)
Radio Driver (rfm95w)
```

- AO_Telemetry is radio-agnostic. AO_Radio is protocol-agnostic.
- Swapping radio hardware (RFM69, SX1280, WiFi) only changes AO_Radio.
- AO_Radio ticks at 100Hz (fixed). AO_Telemetry ticks at encode rate (2-10Hz).
- RadioTxEvt/RadioRxEvt use QP/C dynamic event pools (depth 3 each).
- TX-busy policy: drop and log. TX timeout → kRxWindow (prioritize RX).
- Relay: link-layer only in AO_Radio (CRC check, no payload decode).
- Radio init failure: probe-first, "no hardware" flag, Telem still encodes for USB MAVLink.

### Device Roles (Three Jobs)

| Role | Radio | Sensors | ESKF | Flight Director |
|------|-------|---------|------|-----------------|
| Vehicle | TX (+ optional RX windows) | Active | Active | Active |
| Station | RX continuous | Optional | Inactive | Inactive |
| Relay | RX → re-TX | None | Inactive | Inactive |

---

## Completed Work

### Stage 7: Radio & Telemetry (2026-02-25 to 2026-03-08)

| IVP | Title | Date | Notes |
|-----|-------|------|-------|
| IVP-57 | RFM95W Radio Driver | 2026-02-25 | 100% loopback, through-wall, 8 gates. RSSI/SNR verified. |
| IVP-58 | Telemetry Encoder | 2026-03-03 | CCSDS 54B + MAVLink 4-msg set. Strategy pattern. CRC-16-CCITT. |
| IVP-59 | Telemetry Service (TX) | 2026-03-05 | 2/5/10Hz configurable. Duty cycle tracking. APID prioritization stub. |
| IVP-60 | RX Mode + CCSDS Decode | 2026-03-07 | CSV output, NeoPixel RX status, 5-min soak. |
| IVP-61 | MAVLink Re-encode + QGC | 2026-03-08 | QGC High Latency mode. Heartbeat + SYS_STATUS + ATTITUDE + GLOBAL_POS. |

**Stage 7 Soak Results:** 607 packets, 0 CRC errors, 98.7% delivery rate.

### Data Structures (IVP-49, Stage 6)

- **TelemetryState** (45B packed): quaternion Q15, lat/lon 1e7, alt mm, velocity cm/s, baro, GPS fix/sats, flight state, health, battery, MET
- **FlightMetadata**: UTC epoch anchor for MET-to-wall-clock reconstruction
- **CCSDS packet**: 6B primary header + 4B secondary (MET) + 42B nav payload + 2B CRC = 54B
- **APID definitions**: kApidNav=0x001 (active), kApidDiag=0x002 (defined, unused)

### Radio Hardware Verified

- RFM95W (SX1276) on SPI, GPIO-controlled CS
- 915MHz ISM band, 20dBm TX power
- LoRa SF7/BW250 primary mode
- Fruit Jam: SPI1 (GPIO 28/30/31), CS=10, RST=6, IRQ=5

---

## Planned (Stage 12A — Fruit Jam GCS)

### Non-Blocking TX Split (Critical Path)

Split `rfm95w_send()` into `send_start()` (write FIFO + set TX mode, ~200µs) and `send_poll()` (check RegIrqFlags, ~2µs). Eliminates the 50-150ms blocking call inside AO handler that caused queue overflow crash (LL Entry 32).

TX failure escalation: 1 consecutive timeout → log, 3 → chip reinit, 5 → error flag to health system.

### AO_Radio / AO_Telemetry Split

Extract radio hardware concerns from AO_Telemetry into new AO_Radio. AO_Telemetry becomes protocol-only (encoding, APID mux, rates). AO_Radio owns RadioScheduler + driver. Communication via SIG_RADIO_TX / SIG_RADIO_RX events.

### RadioScheduler (~100 lines)

Half-duplex state machine. TX-priority with RX windows between slots. States: kIdle, kTxActive (polling send_poll), kRxWindow, kRxContinuous. Never inspects packet contents.

### Three-Job System

DeviceRole enum: kVehicle, kStation, kRelay. Compile-time default with flash-stored CLI override (override deferred to later step). Radio capability universal — job determines how the radio is used.

### RadioConfig in `.cfg`

User-facing: protocol (ccsds/mavlink), nav_rate_hz (2/5/10), power_dbm (2-20). Generator derives SF/BW/CR. Advanced overrides deferred to V2.

### Station Enhancements

- 5x NeoPixel RSSI bar (Fruit Jam has 5 NeoPixels)
- Station CLI: RX stats, output toggle, GPS status, distance-to-rocket
- Relay Job: RX → validate CCSDS → re-TX, seq dedup, blue LED

---

## Deferred Work (Tracked)

### Stage 13/14: Multi-APID Telemetry

| Item | Source | Notes |
|------|--------|-------|
| APID 0x002 diagnostics packet | Council 2026-02-27 | Nav at 5-10Hz, Diag at 0.5-1Hz. APID field already in header. |
| Diagnostics packet definition | Council 2026-02-27 | Content TBD: ESKF P-diagonals, innovation stats, confidence |
| TX packet queue (rate dividers) | Council 2026-03-30 | Nav-priority tiebreak, depth 2+1. Not a priority queue DS. |
| RX APID routing dispatch | Council 2026-03-30 | Switch on APID in telemetry service. Trivial when second APID exists. |
| `submit_packet(apid, priority)` API | Council 2026-03-30 | Design now (Stage 12A), implement trivially. Full queue in 13/14. |

### IVP-62: Bidirectional MAVLink Commands (Branch: ivp62-wip)

**Fully implemented, deferred due to QGC USB CDC buffer timing issue.**

What exists on branch:
- mavlink_rx handler
- Command dispatch (param/command/mission)
- flight_state.h updates
- 14 host tests

Blocker: When vehicle streams MAVLink sticky, USB CDC buffers accumulate. On QGC connect, buffered data dumps, overwhelming parser. QGC 3.5s heartbeat timeout fires before next live heartbeat.

Fix approaches:
1. Circular output buffer with timestamp-based discard
2. Flush USB CDC buffer on connect detect
3. Heartbeat-only mode until GCS heartbeat received

Fruit Jam LoRa bridge works (radio naturally drops old packets).

### IVP-63: FSK Continuous Bitstream (Titan Tier)

**Separate driver from LoRa.** Same SX1276 chip but fundamentally different software stack.

- SX1276 FSK continuous mode: raw bitstream 1.2-300 kbps, no packet engine
- Requires software bit-sync, CCSDS sync markers, Transfer Frame construction
- Ground-side needs DSP: sync detection, bit-slip correction, frame alignment
- IRIG/PCM heritage — natural fit for IRIG ground station tools
- **Council unanimous: do not mix with LoRa RadioScheduler**

### IVP-64: Radio Mode Profiles (Stage 13/14)

Mission Profile-driven radio register config:
- SF7/BW125: default (5-15km range)
- SF9-12/BW125: HAB (15-50+km, low data rate)
- SF6/BW500: short-range/drone (1-5km, high rate)
- FSK packet: 50-100 kbps high-rate

### IVP-65: Native MAVLink TX

Transmit MAVLink natively over LoRa (not CCSDS + re-encode) for drone profiles or direct QGC without ground-side translation.

### Transfer Frames

- Skip over LoRa (duplicates LoRa packet features: sync, framing, CRC, FEC)
- Use over serial/UART for wired GCS (Stage 12B Yamcs ingestion)
- Use over Gemini inter-board bus (future)
- Use over FSK continuous bitstream (Titan tier)

### Ground Station (Stage 12B — Linux)

| IVP | Title | Status |
|-----|-------|--------|
| IVP-92 | Yamcs Instance + XTCE | Placeholder |
| IVP-93 | OpenMCT Integration | Placeholder |
| IVP-94 | Ground Station Pi Image | Placeholder |
| IVP-95 | Post-Flight Analysis Tools | Placeholder |
| IVP-96 | Web Dashboard | Placeholder |
| IVP-97 | Mobile / Remote Access | Placeholder |

### Command Authority (Safety — Council Review Required)

When bidirectional comms are wired up:
- Received commands validated against flight phase
- BOOST/COAST/DESCENT: reject all state-changing commands, log only
- IDLE/ARMED: allow arm/disarm with confirmation sequence
- **Pyro commands via radio: NEVER.** Pyro is autonomous or manual arm switch.
- Council review required before any vehicle accepts radio commands in flight.

### Other Deferred Items

| Item | Source | Notes |
|------|--------|-------|
| 5-NeoPixel RSSI bar | Whiteboard | Old code in `ground_station/radio_rx.cpp`, needs port to new arch |
| Range test results | IVP-61 gate | `docs/benchmarks/RANGE_TEST_RESULTS.md` not yet created |
| XTCE dictionary template | Council 2026-02-27 | For Yamcs (Stage 12B) |
| PCM frame expansion | Whiteboard | Current 55B full. Extended frame for Stage 12 GCS decode. |
| Mission Profile boot-load | Whiteboard | `.cfg` → binary blob → flash. Deferred to Stage 12 GCS infra. |
| Flash-stored DeviceRole | Council 2026-03-30 | CLI `role` command, persists across reboots |

---

## Link Budget Reference

| Metric | Value | Source |
|--------|-------|--------|
| Max LoRa payload | 128 bytes | SX1276 datasheet |
| CCSDS packet (54B) airtime @ SF7/BW250 | ~25 ms | Calculated |
| MAVLink 3-msg (105B) airtime @ SF7/BW250 | ~45 ms | Calculated |
| Duty cycle @ 10Hz CCSDS | ~25% | Calculated |
| TX power (field) | 20 dBm | RFM95W PA_BOOST |
| SX1276 sensitivity @ SF7/BW250 | -120 dBm | Datasheet |
| Link margin @ 10 km | +29.3 dB | Link budget calc |
| Practical range (omni-omni, ground) | ~2 km LOS | Estimated |
| Practical range (to apogee) | ~5-8 km | Better Fresnel zone |

---

## Decision Documents (Reference)

| Document | Date | Content |
|----------|------|---------|
| `docs/decisions/Telem+logging/council_telemetry_protocol.md` | 2026-02-27 | Protocol selection, APID strategy, FSK, config |
| `docs/decisions/Telem+logging/council_data_logging.md` | 2026-03-03 | Logging tiers, struct model, stage resequencing |
| `docs/decisions/Telem+logging/telemetry_comparison.md` | 2026-02-27 | Protocol × radio comparison, link budgets, airtime matrix |
| `docs/decisions/Telem+logging/revised_ivp_stages.md` | 2026-03-03 | Stage numbering after logging pull-forward |
| `docs/plans/STAGE7_RADIO_TELEMETRY.md` | 2026-03-07 | Stage 7 implementation plan (complete) |
| `docs/plans/STAGE12A_RADIO_MODULE.md` | 2026-03-30 | Stage 12A plan: AO split, RadioScheduler, 3 council reviews |

---

*Update this document when radio/telemetry work is completed, new decisions are made, or deferred items are promoted to active development.*
