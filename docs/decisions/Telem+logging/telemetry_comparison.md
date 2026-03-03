# RocketChip Telemetry Protocol × Radio Comparison

**Status:** Draft for Council Review  
**Date:** 2026-02-27  
**Scope:** IVP-50 (Telemetry Encoder), IVP-52 (GCS Compatibility)  
**Context:** Consolidation of Claude + Grok protocol research for RBTP definition

---

## 1. Reference Telemetry Payload

Data needed on the ground per telemetry epoch. Full 24-state ESKF output belongs in the onboard log, not the radio link.

| Field | Encoding | Bytes | Notes |
|---|---|---|---|
| Attitude quaternion (w,x,y,z) | int16×4 Q15 | 8 | ±1.0 range, sub-degree resolution |
| Position (lat, lon, alt) | int32×3 (1e-7 deg, mm) | 12 | Standard GPS scaling |
| Velocity NED | int16×3 cm/s | 6 | ±327 m/s (Mach 0.96) |
| Barometric altitude | int32 mm | 4 | ±2,147 km |
| GPS groundspeed | uint16 cm/s | 2 | 0–655 m/s |
| GPS fix + satellites | uint8 packed | 1 | fix[2:0] + sats[7:3] |
| Flight state | uint8 enum | 1 | IDLE/ARMED/BOOST/COAST/APOGEE/DROGUE/MAIN/LANDED |
| Health + sensor status | uint8 bitfield | 1 | ESKF healthy, GPS valid, baro valid, etc. |
| Battery voltage | uint16 mV | 2 | 0–65535 mV |
| RSSI | int8 dBm | 1 | Radio signal strength |
| Mission Elapsed Time | uint32 ms | 4 | 49.7 day range (HPR flights can exceed 10+ min) |
| **Total payload** | | **42** | Slimmed from 64B by removing ESKF biases (diagnostic) |

> **Design note:** ESKF accelerometer and gyroscope biases (12B), covariance health metrics, and raw sensor data are diagnostic — they belong in the PCM flash log at 200 Hz, not the 10 Hz radio telemetry stream. This saves 22 bytes per radio packet vs the earlier 64B payload design.

---

## 2. Radio Physical Layer Options

All options use the 915 MHz ISM band. Range estimates assume ~3 dBi TX antenna (VAS XFire Pro / TBS Immortal T) + directional/tracking RX patch antenna, adding 8–15 dB margin over rubber duck antennas. All at 20 dBm TX power.

**PHY** = physical layer — the radio modulation and RF hardware, independent of the protocol framing above it.

### 2.1 RFM95W (SX1276) — Current Hardware (IVP-49 verified)

The SX1276 chip supports **three operating modes on the same hardware**:

| Mode | Modulation | Throughput | 52B pkt airtime | Max pkt/s | Range | Key property |
|---|---|---|---|---|---|---|
| LoRa SF7/BW125 | Chirp SS | 5.5 kbps | 102.7 ms | 9.7/s | 5–15 km | Default config (IVP-49) |
| LoRa SF9/BW125 | Chirp SS | 1.8 kbps | 328.7 ms | 3.0/s | 15–30 km | HAB / high-altitude |
| LoRa SF12/BW125 | Chirp SS | 0.3 kbps | 2302 ms | 0.4/s | 30–50+ km | Maximum range |
| LoRa SF7/BW250 | Chirp SS | 10.9 kbps | 51.4 ms | 19.5/s | 3–10 km | Moderate high-rate |
| LoRa SF6/BW500 | Chirp SS | 37.5 kbps | 14.1 ms | 70.9/s | 1–5 km | Short-range burst mode |
| **FSK packet mode** | **FSK** | **1.2–300 kbps** | **9.8–0.3 ms** | **102–3000+/s** | **0.5–5 km** | **Same chip, register config change** |
| **FSK continuous mode** | **FSK** | **1.2–300 kbps** | **N/A (stream)** | **Continuous** | **0.5–5 km** | **Raw bitstream — no packets. IRIG/PCM native transport.** |

> **Critical finding:** The SX1276 in FSK continuous mode operates as a raw bitstream radio with no packet engine, no hardware CRC, no preamble detection. Data is clocked through the FIFO or DIO pins. This is exactly the transport layer IRIG-style PCM telemetry was designed for. The RFM95W can serve as both a LoRa packet radio AND an IRIG-heritage continuous bitstream radio, selectable per Mission Profile. No hardware change needed.

### 2.2 RFM69 (SX1231) — Alternative Radio (not in current BOM)

| Mode | Modulation | Throughput | 52B pkt airtime | Max pkt/s | Range | Key property |
|---|---|---|---|---|---|---|
| FSK 50 kbps | FSK | 50 kbps | 9.8 ms | 102/s | 1–5 km | RadioHead Mesh proven |
| FSK 100 kbps | FSK | 100 kbps | 4.9 ms | 204/s | 0.5–3 km | Near-max practical rate |
| FSK 300 kbps | FSK | 300 kbps | 1.6 ms | 614/s | 0.3–1 km | Theoretical max |

| Dimension | RFM95W (SX1276) | RFM69 (SX1231) |
|---|---|---|
| IVP-49 driver | ✅ Verified | ❌ New driver needed |
| LoRa spread-spectrum | ✅ 6–12 dB processing gain | ❌ FSK only |
| FSK mode | ✅ Packet + continuous | ✅ Packet + continuous |
| Hardware FEC | ✅ LoRa coding rate 4/5–4/8 | ❌ None |
| Encryption | ❌ Software only | ✅ AES-128 in hardware |
| Mesh support | LoRaMesher (distance-vector) | RadioHead Mesh (store-and-forward) |
| Adafruit ecosystem | FeatherWing #3231 | FeatherWing #3229 |
| Continuous bitstream | ✅ FSK continuous mode | ✅ Continuous mode |

**Recommendation:** Stay on RFM95W. Same chip covers LoRa (long range) + FSK packet (high rate) + FSK continuous (bitstream). RFM69 viable as future Booster Pack for dedicated short-range/mesh applications.

---

## 3. Protocol Framing Options

Overhead per telemetry epoch for the 42-byte reference payload defined in §1.

| Protocol | OH (B) | Wire (B) | Components | Loss detection | Multiplexing |
|---|---|---|---|---|---|
| Raw binary struct | 0 | 42 | None | None | None |
| PCM fixed frame (sync only) | 2 | 44 | 2B sync word | None (resync on next frame) | None (position-based) |
| CCSDS Space Packet (pruned) | 10 | 52 | 6B primary + 4B secondary | 14-bit seq/APID | 11-bit APID (2048) |
| CCSDS SP + CRC-16 | 12 | 54 | 6B primary + 4B secondary + 2B CRC | 14-bit seq + integrity | APID |
| CCSDS Transfer Frame + SP | 18 | 60 | 6B TF hdr + 6B SP primary + 4B SP sec + 2B FECF | Frame + packet seq | APID + 3-bit VC (8 channels) |
| MAVLink v2 custom msg | 12 | 54 | 10B header + 2B CRC (payload includes 4B timestamp) | 8-bit sequence | 24-bit message ID |
| MAVLink v2 minimal std (3 msgs) | 36 | 105 | 3× (10B hdr + 2B CRC) = 36B; HB(9)+ATT_Q(32)+GPOS(28) | 8-bit seq per msg | Message ID |
| MAVLink v2 full std (6 msgs) | 72 | 244 | 6× (10B hdr + 2B CRC); MISSING biases, ESKF health | 8-bit seq per msg | Message ID |
| CSP | 4 | 46 | 4B header (priority+addr+port+flags) | Optional RDP | 6-bit port (64) |
| CSP + CRC-32 | 8 | 50 | 4B header + 4B CRC-32 | RDP + integrity | Port |

> **Notes on MAVLink standard messages:**  
> - The 3-message minimal set (HEARTBEAT + ATTITUDE_QUATERNION + GLOBAL_POSITION_INT) gives QGC/MP a working attitude ball + map position. It's missing biases, baro alt, ESKF health, and RSSI — those would need a custom message anyway.  
> - The 6-message full set (adding GPS_RAW_INT + SYS_STATUS + VFR_HUD) totals 244B, which exceeds the 255B LoRa max payload and requires multi-packet scheduling.  
> - Standard MAVLink message field types are fixed by the XML spec and CRC_EXTRA seed — you cannot compress them. Custom messages can use any MAVLink-supported type (int16, uint8, etc.).  
> - MAVLink fields that appear "wasted" for rockets (rollspeed, airspeed, throttle, sensor bitmasks) serve legitimate purposes in the drone/multirotor context MAVLink was designed for.

---

## 4. Airtime Matrix — Protocol × Radio Mode

Milliseconds per complete telemetry epoch (42B payload + protocol overhead).

| Protocol (wire bytes) | LoRa SF7/125k | LoRa SF9/125k | LoRa SF6/500k | FSK 50k (95W or 69) | FSK 100k |
|---|---|---|---|---|---|
| Raw struct (42B) | 87.3 ms | 287.7 ms | 11.6 ms | 8.2 ms | 4.1 ms |
| PCM frame (44B) | 92.4 ms | 287.7 ms | 12.2 ms | 8.5 ms | 4.2 ms |
| CSP (46B) | 92.4 ms | 308.2 ms | 12.8 ms | 8.8 ms | 4.4 ms |
| CCSDS SP (52B) | 102.7 ms | 328.7 ms | 14.1 ms | 9.8 ms | 4.9 ms |
| CCSDS SP+CRC (54B) | 102.7 ms | 349.2 ms | 14.1 ms | 10.1 ms | 5.0 ms |
| CCSDS TF+SP (60B) | 112.9 ms | 369.7 ms | 15.4 ms | 11.0 ms | 5.5 ms |
| MAVLink custom (54B) | 102.7 ms | 349.2 ms | 14.1 ms | 10.1 ms | 5.0 ms |
| MAVLink 3-msg (105B) | 179.5 ms | 574.5 ms | 25.0 ms | 18.2 ms | 9.1 ms |
| MAVLink 6-msg (244B) | 384.3 ms | 1209.3 ms | N/A (>255B) | 40.5 ms | 20.2 ms |
| **Max CCSDS SP pkt/s** | **9.7/s** | **3.0/s** | **70.9/s** | **102.5/s** | **204.9/s** |

> **Key insight from Grok's analysis (confirmed):** Radio mode selection is a 10–20× throughput lever. Protocol overhead selection is a 1.2–1.5× lever. Pick your radio mode per mission first, then optimize the protocol.

---

## 5. SX1276 FSK Continuous Mode — IRIG/PCM Over Radio

The SX1276's FSK continuous mode enables true IRIG-style bitstream telemetry over the same RFM95W hardware. In this mode:

- No packet engine — raw bits clocked through FIFO
- No hardware preamble, sync, CRC, or length field
- Software handles frame sync, decommutation, error detection
- Bitrates 1.2 kbps to 300 kbps
- Same 915 MHz ISM band, same antenna, same power

**PCM frame over FSK continuous mode (50 kbps):**

| Parameter | Value |
|---|---|
| Frame size | 44 bytes (2B sync + 42B payload) |
| Frame rate at 50 kbps | 142 frames/sec |
| Frame rate at 100 kbps | 284 frames/sec |
| Sync word | 16-bit or 32-bit Barker/PN code |
| Error detection | Software CRC per frame or per N frames |
| Range | 0.5–5 km (no spread-spectrum gain) |

**Where this wins over LoRa packet mode:**

- Higher effective data rate at short range (no LoRa preamble/header overhead per packet)
- Continuous stream — no inter-packet dead time
- Natural fit for IRIG heritage ground station tools
- Frame rate can be locked to sensor sample rate (deterministic)
- Graceful degradation: partial frame corruption doesn't lose the whole epoch

**Where LoRa packet mode wins:**

- Spread-spectrum processing gain → much better range
- Hardware FEC (coding rate) → better BER at same SNR
- Packet CRC catches errors before they reach software
- Simpler software — no sync word detection, no manual framing

**Mission Profile mapping:**

| Profile | Recommended radio mode | Rationale |
|---|---|---|
| Model rocket (<2 km) | LoRa SF7/BW125 | Range + simplicity |
| HPR (2–15 km, 10+ min flight) | LoRa SF7/BW125 or SF9 | Range critical, altitude + distance |
| HAB / high altitude | LoRa SF9–12/BW125 | Maximum range, low data rate acceptable |
| Drone (<2 km, needs QGC) | LoRa SF6/BW500 or FSK packet | High rate, short range, MAVLink compat |
| RC vehicle (<1 km) | FSK packet 50–100 kbps | High rate, very short range |
| Bench test / development | FSK continuous or packet | Maximum data rate, cable or close range |
| IRIG-heritage bitstream | FSK continuous 50–100 kbps | True PCM transport, range limited |

---

## 6. CCSDS Transfer Frame Wrapper Assessment

The original concept: wrap payload (MAVLink or custom) inside a CCSDS Transfer Frame to gain frame-level FEC, virtual channel multiplexing, and full standards compliance. This is exactly how NASA/ESA missions layer their protocols.

**What the Transfer Frame adds (8B total: 6B header + 2B FECF):**

- Spacecraft ID (10 bits) — vehicle identification in multi-vehicle operations
- Virtual Channel ID (3 bits) — 8 virtual channels for stream separation
- Master/Virtual Channel Frame Counters — per-channel loss detection
- Frame Error Control Field (FECF) — CRC-16 or optional Reed-Solomon

**Assessment by transport:**

| Transport | TF justified? | Rationale |
|---|---|---|
| LoRa packet mode | **No** | LoRa provides sync, framing, length, CRC, and FEC (coding rate). TF duplicates all of these. APID multiplexing (2048 streams) exceeds VC multiplexing (8 channels). 8B overhead buys nothing new. |
| SX1276 FSK continuous | **Maybe** | No hardware framing in continuous mode. TF provides sync and error detection that the radio doesn't. But a simple sync word + CRC is lighter than full TF. |
| Raw serial / UART | **Yes** | No native framing on serial. TF provides sync, length delimiting, and error detection. Relevant for wired ground station, inter-MCU buses (Gemini), non-LoRa radio links. |
| Dedicated RF (S-band, UHF FM) | **Yes** | Continuous bitstream radios need ASM + TF for frame sync. This is the use case TF was designed for. Future Nova / high-power links. |

---

## 7. Qualitative Comparison

|  | CCSDS SP (pruned) | CCSDS TF+SP | MAVLink v2 custom | MAVLink v2 std | CSP | Raw / PCM frame |
|---|---|---|---|---|---|---|
| **GCS compatibility** | OpenMCT + Yamcs | OpenMCT + Yamcs | QGC (limited*) | QGC/MP native | Custom only | Custom only |
| **Multiplexing** | APID (2048) | APID + VC (8) | MsgID (16.7M) | MsgID (16.7M) | Port (64) | None |
| **Loss detection** | 14-bit seq/APID | Frame + pkt seq | 8-bit seq | 8-bit seq | Optional RDP | None |
| **Schema evolution** | XTCE dictionary | XTCE dictionary | XML dialect | Fixed by spec | None std | None |
| **Encode complexity** | memcpy + 6B hdr | memcpy + 18B hdr | mavgen codegen | mavgen codegen | libcsp API | memcpy |
| **Error correction** | LoRa FEC only | TF FECF + LoRa | LoRa FEC only | LoRa FEC only | LoRa FEC only | LoRa FEC only |
| **Bidirectional cmd** | TM/TC type bit | TM/TC type bit | Native cmd/ack | Native cmd/ack | RDP sessions | No |
| **Code size** | ~0.5 KB | ~1 KB | ~3–5 KB | ~3–5 KB | ~8–15 KB** | ~0 |
| **RTOS required?** | No | No | No | No | Yes*** | No |
| **Standards heritage** | NASA/ESA/CubeSat | NASA/ESA (full) | ArduPilot/PX4 | ArduPilot/PX4 | GomSpace sats | IRIG (onboard) |
| **Onboard logging** | Possible | Overkill | ArduPilot .bin | ArduPilot .bin | No | Ideal |
| **Mission Profile fit** | APID = profile | VC = profile | Dialect XML | N/A | Port mapping | Frame layout |
| **Bitstream transport** | Over FSK cont. | Native fit | No | No | No | Native fit |
| **`#ifdef` option** | Primary | Future serial | ✅ Compile-time | ✅ Compile-time | Future CAN | Onboard log |

\* QGC shows raw field names for custom messages, no custom widgets  
\** libcsp with OS abstraction; bare-metal shim feasible at ~2–4 KB  
\*** libcsp requires FreeRTOS/Zephyr/POSIX; thin OS shim needed for bare-metal

---

## 8. Payload Encoding Options (inside any protocol frame)

| Format | Encoded size (42B equiv.) | Code size (ARM) | Schema evolution | Best library |
|---|---|---|---|---|
| Raw packed struct | 42 B | ~0 | None (breaking changes) | N/A |
| Nanopb (protobuf) | 36–50 B | 2.5–5 KB | Excellent | nanopb |
| CBOR (integer keys) | 44–55 B | 0.6–2 KB | Good (with zcbor) | NanoCBOR / zcbor |
| MessagePack | 45–53 B | 1–3 KB | Moderate | CMP |
| FlatBuffers | 70–100 B | 5–15 KB | Good | flatcc (AVOID: vtable bloat) |

**Recommendation:** Raw packed struct for IVP-50 (simplest, wire = struct = memcpy). Migrate to nanopb or CBOR when schema evolution becomes a real need (multiple firmware versions in the field). Encoding choice is internal to the CCSDS data field and transparent to the protocol layer.

---

## 9. Onboard Logging (Separate from Radio Telemetry)

| Format | Frame OH | Full-rate write | 8 MB flash duration | Notes |
|---|---|---|---|---|
| PCM fixed frame | 2B sync | 200 Hz × 128B = 25.6 KB/s | ~5.2 min | Ideal: fixed size, trivial ring buffer, struct unpack decom |
| IRIG-106 Ch10 | 24B header | Variable | ~4.5 min | Pro format: typed channels, video support, established tools. Future Titan tier. |
| MAVLink .bin | 12B/msg | Variable | ~3–4 min | ArduPilot compatible, larger per-record overhead |
| Raw struct dump | 0 | 200 Hz × 64B = 12.8 KB/s | ~10.4 min | Maximum density, no sync recovery after corruption |

**Recommendation:** PCM-style fixed frames for initial implementation (IVP-63+). IRIG-106 Ch10 container format worth evaluating for Titan tier with camera/video capability. Decommutation table maps to Mission Profile parameter dictionary.

---

## 10. Protocols Worth Further Investigation

### 10.1 CSP for Internal Bus (Future Multi-MCU)

CSP over CAN becomes compelling when RocketChip evolves to multi-MCU architectures: Gemini dual-Core carrier board, Booster Packs with dedicated processors. The RP2350 lacks native CAN — would need MCP2515 or similar SPI-to-CAN bridge. Flag for hardware discussion alongside Gemini board design.

### 10.2 CCSDS Proximity-1 Data Service Concepts

Session management layer (hailing, MAC sublayer) is inappropriate for rocket telemetry. However, the data service model (expedited vs sequence-controlled delivery, priority levels) maps well to critical vs housekeeping telemetry packet prioritization. Worth mining for IVP-51 (Telemetry Service) design patterns.

### 10.3 SpaceWire / SpaceFibre for Nova

SpaceWire (ECSS-E-ST-50-12C): 2–400 Mbps over LVDS. Standard onboard bus for ESA/NASA spacecraft. FPGAs commonly have LVDS-capable I/O banks (Lattice ECP5, Xilinx Artix-7, Intel MAX 10). An FPGA Booster Pack with SpaceWire interfaces is feasible for CubeSat instrument integration testing.

SpaceFibre (ECSS-E-ST-50-11C): 2.5+ Gbps successor with QoS and fault isolation. Requires fiber optic or advanced electrical transceivers. Relevant for Nova concept documentation, not near-term hardware.

### 10.4 NASA cFE/cFS Extractable Code

The `CCSDS_PrimaryHeader_t` struct and accessor macros from `cfe/modules/msg/` are ~200 LOC of portable C. Worth extracting as reference implementation rather than dependency. Full cFS requires RTOS + ~2 MB SRAM — not feasible on RP2350.

---

## 11. Summary Recommendation

| Layer | Choice | Rationale |
|---|---|---|
| **Radio hardware** | RFM95W (SX1276), IVP-49 done | LoRa + FSK packet + FSK continuous on same chip |
| **Default radio mode** | LoRa SF7/BW125 | Best range/throughput balance for rocketry |
| **Short-range mode** | LoRa SF6/BW500 or FSK packet | Configurable per Mission Profile |
| **Bitstream mode** | FSK continuous 50–100 kbps | IRIG/PCM heritage, same hardware |
| **Wire protocol** | CCSDS Space Packet (pruned) | 10B overhead, APID mux, 14-bit seq, wire=struct |
| **Application CRC** | CRC-16 (2B, optional) | Catches encoder bugs beyond LoRa link CRC |
| **MAVLink support** | `#ifdef ENABLE_MAVLINK` compile option | Drone Mission Profile QGC compatibility |
| **Transfer Frames** | Skip over LoRa; use over serial/Gemini | LoRa duplicates TF features |
| **Payload encoding** | Raw packed struct (IVP-50) | Simplest; migrate to nanopb when needed |
| **Onboard logging** | PCM fixed frames to flash/PSRAM | Max density, deterministic, IRIG heritage |
| **Ground station** | Yamcs + XTCE + OpenMCT (primary) | Aerospace-grade, CCSDS native |
| **GCS alt** | MAVLink bridge for drone profiles | QGC/MP compatibility where needed |

### Three-Layer Telemetry Architecture

1. **ONBOARD LOG:** PCM-style fixed frames → flash/PSRAM (200 Hz, full ESKF state + raw sensors)
2. **LIVE TELEMETRY:** CCSDS Space Packets → LoRa or FSK (5–10 Hz, attitude + position + health)
3. **POST-FLIGHT:** Decommutate PCM log → repackage as CCSDS → replay to OpenMCT at full rate

### Action Items Pending Council Review

- [ ] Define IVP-50 scope: CCSDS SP encoder (6B primary + 4B secondary + 42B payload + optional 2B CRC)
- [ ] Define IVP-52 scope: OpenMCT GCS via Yamcs + XTCE, with MAVLink bridge as compile option
- [ ] Investigate SX1276 FSK continuous mode for RBTP bitstream transport option
- [ ] Extract cFE CCSDS header implementation as reference
- [ ] Create XTCE dictionary template for Mission Profile parameter definitions
- [ ] Design two-tier packet strategy: critical (high-rate APID) vs housekeeping (low-rate APID)
- [ ] Flag CSP + CAN for Gemini carrier board hardware discussion
- [ ] Flag FPGA + LVDS (SpaceWire) for Nova Booster Pack concept
