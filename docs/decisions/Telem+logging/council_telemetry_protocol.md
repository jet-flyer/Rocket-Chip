# Council Review: Telemetry Protocol Selection

**Date:** 2026-02-27
**Topic:** Wire protocol, radio mode strategy, and telemetry architecture for RBTP / IVP-50+
**Reference:** Telemetry Protocol × Radio Comparison document (2026-02-27)
**Panel:** NASA/JPL Lead, Senior Aerospace Student, CubeSat Engineer (aux), Advanced Hobbyist Rocketeer (aux), Embedded Professor
**Benched:** ArduPilot Core Contributor (limited domain relevance for protocol selection)

---

## Background

Independent protocol × radio comparison analysis covering CCSDS Space Packet, MAVLink v2, CSP, raw/PCM framing, and CCSDS Transfer Frame wrapper options across RFM95W LoRa modes, RFM95W FSK modes, and RFM69 FSK. Reference telemetry payload slimmed from 64B to 42B by removing ESKF biases (diagnostic data belongs in onboard log) and extending MET to 4 bytes (HPR flights exceed 10+ minutes).

Critical finding: SX1276 chip inside RFM95W supports FSK continuous mode — raw bitstream radio with no packet engine. Enables IRIG-heritage PCM telemetry over same hardware as IVP-49, selectable per Mission Profile.

---

## Round 1

### CubeSat Engineer

CCSDS Space Packet with pruned stack is the right call. APID multiplexing is the killer feature — within six months you'll want housekeeping, event reports, command ACKs, and diagnostic streams. APID lets you add packet types without touching the nav encoder.

Concerns:
1. **Don't trim biases entirely from radio.** Two-tier APID strategy: APID 1 for slim 42B nav at 10 Hz, APID 2 for full-state diagnostics at 1 Hz. APID architecture makes this free.
2. **FSK continuous mode requires fundamentally different ground RX software.** Packet mode delivers clean payload via SPI interrupt. Continuous mode needs software sync detection, bit-slip correction, frame alignment. Real DSP work on ground side.
3. **Range estimates are optimistic.** Factor 3 dB tumble loss on dipole TX during recovery descent plus 2 dB polarization mismatch. Real margin at 10 km is thinner than documented. Argument for LoRa as default — spread-spectrum processing gain buys margin FSK doesn't.

### NASA/JPL Lead

Three-layer architecture (PCM onboard, CCSDS over radio, replay to GCS) is sound — essentially what every sounding rocket program does.

Transfer Frame assessment correct: skip over LoRa, use over serial. Relevant for Gemini dual-board and wired umbilical pre-launch checkout.

cFE header extraction: ~50 LOC portable C, worth as reference not dependency. Note big-endian wire format per CCSDS 133.0-B-2 §4.1.1 vs RP2350 little-endian — need explicit byte-swap.

**Critical operational constraint:** At 10 Hz TX with SF7/BW125, duty cycle ~100%. Zero margin for uplink. Either drop to 5 Hz or accept unidirectional link during flight.

### Advanced Hobbyist Rocketeer

**MAVLink should be the default for rocket profiles.** The 105B 3-message set fits one LoRa packet, gives QGC a working map + attitude display. CCSDS is the power-user path.

The hobbyist rocketry community will not adopt a flight computer requiring a Java application server for basic telemetry.

SF7/BW125 right for default. 2 Hz position during descent is plenty. Recovery beeper is the most important telemetry feature after "records altitude."

### Embedded Professor

Strategy pattern solves the adoption concern. Mission Profile selects encoder. `Rocket_Edu` defaults to MAVLink + QGC. `Rocket_Pro` uses CCSDS. Both always compiled in (~4.5 KB total — trivial on 8 MB flash). No `#ifdef`, no recompilation.

PCM flash log and CCSDS radio telemetry must share same data structures. `FusedState` is canonical. Logger memcpy's it. Encoder extracts subset. Three views of same data, not three paths.

### Senior Aerospace Student

**Scope creep warning.** Proposed tight IVP-50: define interface, implement MAVLink encoder, verify QGC. "Live telemetry to QGroundControl" gets funded. Three-layer CCSDS architecture gets bookmarked.

Sequence correctly: MAVLink now, CCSDS next, advanced features after crowdfunding.

---

## Round 2

### CubeSat Engineer

Pushback: implementing MAVLink first shapes the interface around message-oriented assumptions. CCSDS is simpler (buffer + type tag). Design interface around simpler model, let MAVLink adapt. CCSDS encoder first (~80 LOC), MAVLink adapter second (~200 LOC).

### Advanced Hobbyist Rocketeer

Fine with CCSDS first *internally* as long as out-of-box default is MAVLink + QGC. Recovery beeper flagged for Flight Director stage.

### NASA/JPL Lead

CCSDS encoder is mechanically simpler — no codegen, no CRC_EXTRA. `FusedState` should use fixed-point types matching wire encoding. 2 Hz default with powered-flight burst is correct operational concept.

### Embedded Professor

Interface with `packet_type` parameter. ESKF needs float32 internally; conversion to fixed-point once at fusion output into `TelemetryState`. Both logger and encoder read `TelemetryState`.

### Senior Aerospace Student

Revised scope: define interface + `TelemetryState` struct, implement both CCSDS and MAVLink encoders, Mission Profile selects.

---

## Round 3 — Convergence

- **CubeSat Engineer:** Consensus. CCSDS secondary header must include 4-byte MET.
- **NASA/JPL Lead:** Consensus. Big-endian wire format, explicit byte-swap.
- **Hobbyist Rocketeer:** Consensus. MAVLink as default out-of-box.
- **Embedded Professor:** Consensus. `TelemetryState` struct IS the ICD.
- **Senior Aerospace Student:** Consensus called.

---

## Consensus

| Decision | Choice |
|---|---|
| Wire protocol | CCSDS Space Packet (pruned) primary, MAVLink v2 secondary, strategy interface |
| Default encoder | MAVLink for `Rocket_Edu`, CCSDS for `Rocket_Pro` |
| Radio mode | LoRa SF7/BW125 default, configurable per Mission Profile |
| TX rate | 2 Hz default, burst 5-10 Hz powered flight |
| Transfer Frames | Skip over LoRa; future serial/Gemini use |
| Payload encoding | Raw packed struct, fixed-point in `TelemetryState` |
| Onboard logging | PCM fixed frames (separate council review) |
| Ground station | QGC for demo; Yamcs/OpenMCT as dedicated GCS stage |

---

## Post-Council Clarifications

1. **APID multiplexing** enables different packet types and rates on same link (nav at 5 Hz + diagnostics at 0.5 Hz), not simultaneous transport modes. SX1276 is in one radio mode at a time.
2. **MAVLink translation on RX board**, not PC-side. CCSDS on wire, RX translates to MAVLink over USB for QGC. User never sees CCSDS.
3. **Pro/Edu/Dev profiles** map encoder, rate, radio mode, and GCS expectations.
4. **FSK continuous mode** worth having as debug profile early, not fully deferred.
5. **RX board is just another RocketChip.** Evaluate whether `Receiver` Mission Profile is cleaner than bespoke RX firmware.
6. **Bespoke RPi image** planned for OpenMCT GCS route.
