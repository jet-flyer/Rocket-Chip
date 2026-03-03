# Revised IVP Stage Layout

**Date:** 2026-03-03
**Basis:** Telemetry Protocol Council Review + Data Logging Council Review
**Status:** Pending repo update via Claude Code

---

## Stage Overview

| Stage | Name | IVP Range | Status | Milestone |
|---|---|---|---|---|
| 1 | Foundation | IVP-01 -- IVP-08 | Complete | |
| 2 | Single-Core Sensors | IVP-09 -- IVP-18 | Complete | **Minimum Viable Demo** |
| 3 | Dual-Core Integration | IVP-19 -- IVP-30 | Complete | |
| 4 | GPS Navigation | IVP-31 -- IVP-33 | Complete | |
| M | Magnetometer Calibration | IVP-34 -- IVP-38 | Complete | |
| 5 | Sensor Fusion | IVP-39 -- IVP-48 | Complete | |
| **6** | **Data Logging** | **IVP-49 -- IVP-56** | **Pulled forward from Stage 9** | |
| **7** | **Radio & Telemetry** | **IVP-57 -- IVP-65** | **TX + RX + translation** | |
| 8 | Flight Director | IVP-66 -- IVP-72 | Placeholder | **Crowdfunding Demo Ready** |
| 9 | Adaptive Estimation & Safety | IVP-73 -- IVP-77 | Placeholder | |
| **10** | **Ground Station** | **IVP-78 -- IVP-83** | **Dedicated platform (Pi/PC)** | |
| 11 | System Integration | IVP-84 -- IVP-88 | Placeholder | **Flight Ready** |

---

## Stage 6: Data Logging

**Purpose:** Define canonical onboard data model, implement PCM-based flight logging to PSRAM and flash. Establishes data structures that all downstream consumers (telemetry encoder, flight director, GCS) read from. Pulled forward from original Stage 9 because telemetry encoder depends on these definitions.

**Dependencies:** Stage 5 (FusionTask outputs FusedState)

**Pre-stage check:** Verify ICM-20948 DLPF register configuration matches intended bandwidth and output rate. Read back ACCEL_CONFIG, GYRO_CONFIG, SMPLRT_DIV registers. Confirm oversampling/DLPF bandwidth appropriate for logging fidelity. Correct before proceeding if mismatched.

| Step | Title | Description |
|---|---|---|
| IVP-49 | Data Model & ICD | Define FusedState (float32, ESKF-internal), TelemetryState (fixed-point wire-ready), SensorSnapshot (raw pre-calibration). Shared headers in include/rocketchip/. Fixed-point scaling documented as code comments -- the struct IS the ICD. Conversion functions FusedState to TelemetryState. |
| IVP-50 | Timestamp Architecture | 4-byte MET from time_us_64() in every frame header (49.7-day range). UTC epoch anchor from GPS (primary) or RTC (secondary, e.g. PCF8523 on Adalogger FeatherWing). Fallback to boot-relative MET. Both MET and UTC anchor recorded for post-flight reconstruction. |
| IVP-51 | PCM Frame Format | 16-bit sync word + frame header (4B MET, 1B frame type, 1B length) + payload. Frame type byte identifies tier (Economy=0, Standard=1, Research=2). Decommutation table as const struct from Mission Profile. Encode/decode roundtrip gate. |
| IVP-52 | PSRAM Ring Buffer | Write TelemetryState PCM frames at configurable rate (1-200 Hz slider, 50 Hz default). Averaging decimation from fusion output rate. Pre-launch continuous capture from boot. Wraparound handling. 10+ min continuous write verification. |
| IVP-53 | Flash Storage & Flight Table | Post-landing PSRAM to raw flash sector flush. Flight log table in Tier 1 storage (start_sector, end_sector, MET_start, duration, frame_size, rate_hz). Pre-erase sector pool during pre-launch idle for future HAB flush. |
| IVP-54 | USB Log Download | CLI lists flights from log table, downloads raw data over USB serial. Python host script decodes PCM frames to CSV (time, altitude, accel_xyz, gyro_xyz, lat, lon). |
| IVP-55 | Raw Sensor Logging *(placeholder -- Stage 9)* | Toggle to include SensorSnapshot in log frames. Independent of log rate. Per-sensor bitmask in advanced config. Research Mode gated. |
| IVP-56 | Economy Tier & HAB Flush *(placeholder -- Stage 9)* | 1-2 Hz decimation with reduced field set. Periodic flash flush every 60s for long-duration missions. Required for HAB Mission Profile. |

**Gate:** Flight log captured to PSRAM at 50 Hz during simulated flight, flushed to flash after landing, downloaded via USB, decoded to CSV with correct timestamps and plausible values. Pre-launch data (5+ sec before trigger) preserved.

---

## Stage 7: Radio & Telemetry

**Purpose:** Encode logged data for radio transmission, manage radio link, implement receive-side decoding and protocol translation. Includes TX (vehicle) and RX (ground-side RocketChip in Receiver mode) firmware.

**Dependencies:** Stage 6 (TelemetryState struct, PCM frame format), RFM95W hardware

**Note:** Before implementing IVP-60, evaluate whether RX firmware should be a Receiver Mission Profile rather than bespoke code. A Receiver profile that configures the same firmware for RX-only mode with translation output fits the Mission Profile ecosystem cleanly -- any RocketChip board becomes a ground receiver by selecting a profile.

| Step | Title | Description |
|---|---|---|
| IVP-57 | RFM95W Radio Driver | SPI init, LoRa TX/RX, polling. **Already verified (2026-02-25).** Renumbered from original IVP-49. |
| IVP-58 | Telemetry Encoder | TelemetryEncoder strategy interface. CcsdsEncoder (6B primary big-endian + 4B secondary + 42B nav + optional 2B CRC = 52-54B). MavlinkEncoder (HEARTBEAT + ATTITUDE_QUATERNION + GLOBAL_POSITION_INT = 105B). Mission Profile selects encoder. Default: MAVLink for Rocket_Edu, CCSDS for Rocket_Pro. |
| IVP-59 | Telemetry Service | Configurable TX rate (2 Hz default, burst during powered flight). APID prioritization for CCSDS. Duty cycle management (<50% at SF7/BW125 for uplink margin). |
| IVP-60 | Translation Layer | RX-mode firmware for ground-side RocketChip. Receives any RocketChip telemetry (CCSDS or MAVLink), decodes, outputs translated data over USB serial. Generic: any supported input to any supported output. Also accepts MAVLink from external sources. Future: WiFi/Bluetooth output for production. Evaluate as Receiver Mission Profile. |
| IVP-61 | Mission Planner / QGC Validation | End-to-end: TX sends, RX receives and translates to MAVLink, QGC/Mission Planner displays attitude + map + flight state. User acceptance gate. |
| IVP-62 | Bidirectional Commands | Command reception/parsing on vehicle. Arm, calibrate, parameter set. Partial -- reception and ACK/NACK here, full execution depends on Stage 8 state machine. |
| IVP-63 | FSK Continuous Bitstream *(placeholder -- Stage 9)* | SX1276 FSK continuous mode for IRIG-heritage transport. Ground-side sync detection/frame alignment. Research Mode gated. |
| IVP-64 | Radio Mode Profiles *(placeholder -- Stage 9)* | Mission Profile-driven radio config: SF7/BW125 default, SF9-12 HAB, SF6/BW500 short-range, FSK packet high-rate. Same hardware, register config changes. |
| IVP-65 | Native MAVLink TX *(placeholder)* | Transmit MAVLink natively over LoRa for drone profiles or direct QGC without translation. Mission Profile option. |

**Gate:** TX RocketChip transmits, RX RocketChip receives and translates, QGC/Mission Planner displays live attitude + map + flight state. Stable 10+ min at SF7/BW125.

---

## Stage 8: Flight Director (IVP-66 -- IVP-72)

Placeholder. Flight state machine, watchdog, pyro channel control, recovery deployment, arming/disarming. Recovery beeper / audio beacon scoped here (product-critical for hobbyist adoption per council review). **Crowdfunding Demo Ready milestone.**

---

## Stage 9: Adaptive Estimation & Safety (IVP-73 -- IVP-77)

Placeholder. Phase-aware ESKF Q/R tuning, confidence-gated actions, sensor health monitoring. Deferred logging and radio features land here alongside estimation work: IVP-55 (raw sensor logging, Research Mode), IVP-56 (economy tier / HAB flush), IVP-63 (FSK continuous), IVP-64 (radio mode profiles).

---

## Stage 10: Ground Station (IVP-78 -- IVP-83)

**Purpose:** Dedicated ground station platform on Raspberry Pi or PC. Everything beyond the RX RocketChip board. Independent of vehicle firmware once Stage 7 packet format is stable. Can be developed in parallel with Stages 8-9.

| Step | Title | Description |
|---|---|---|
| IVP-78 | Yamcs Instance & XTCE *(placeholder)* | XTCE dictionary for CCSDS packets. Yamcs on dedicated RPi (4/5, 2+ GB RAM). Direct CCSDS ingestion. For Rocket_Pro users. |
| IVP-79 | OpenMCT Integration *(placeholder)* | OpenMCT via openmct-yamcs plugin. Custom layouts. NASA Punk aesthetic. |
| IVP-80 | Ground Station Pi Image *(placeholder)* | Pre-built RPi OS image. Yamcs, OpenMCT, RX serial ingestion, auto-start. Turnkey: flash SD, power on, connect antenna, operational. |
| IVP-81 | Post-Flight Analysis Tools *(placeholder)* | PCM to CCSDS replay to Yamcs. Flight timeline with MET-to-UTC mapping. Multi-flight overlay. |
| IVP-82 | Web Dashboard *(placeholder)* | Browser-based telemetry. Lighter than full OpenMCT for field use. Runs on GCS Pi or standalone. |
| IVP-83 | Mobile / Remote Access *(placeholder)* | WiFi AP on ground station Pi for phone/tablet access. Field use: launch, walk to recovery, monitor GPS on phone. |

---

## Stage 11: System Integration (IVP-84 -- IVP-88)

Placeholder. End-to-end flight test, Mission Profile validation, full system stress testing. **Flight Ready milestone.**

---

## Key Architectural Decisions Referenced

- **Three-struct data model:** FusedState / TelemetryState / SensorSnapshot (Data Logging Council)
- **TelemetryEncoder strategy interface:** CCSDS primary, MAVLink secondary, Mission Profile selects (Protocol Council)
- **PCM fixed frames for logging:** Sync word + MET + frame type + payload (Data Logging Council)
- **Raw flash over LittleFS:** Sequential PCM stream needs no filesystem (Data Logging Council)
- **2 Hz default TX, burst during powered flight:** Duty cycle management (Protocol Council)
- **CCSDS on wire, RX translates to MAVLink:** User never sees CCSDS (Protocol Council)
- **Logging tiers as decimation slider:** 1-200 Hz with Economy/Standard/Research as presets (Data Logging Council)
- **RX board as Mission Profile:** Evaluate Receiver profile vs bespoke firmware before IVP-60

## Renumbering Notes

All IVP numbers from IVP-49 onward are renumbered. The radio driver previously verified as IVP-49 becomes IVP-57. Old Stage 9 (IVP-63-67) is retired and replaced by new Stage 6 (IVP-49-56). Mechanical renumbering of all repo references to be handled by Claude Code.
