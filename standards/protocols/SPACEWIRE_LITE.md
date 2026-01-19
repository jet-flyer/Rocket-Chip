# SpaceWire-Lite Protocol Specification

**Status:** Aspirational Standard (Design-for-Certifiability)
**Version:** 0.1 Draft
**Last Updated:** 2026-01-19
**Reference Standard:** ECSS-E-ST-50-12C (SpaceWire Links, Nodes, Routers and Networks)

---

## 1. Overview

SpaceWire-Lite is a simplified implementation of the ECSS-E-ST-50-12C SpaceWire standard, tailored for the RocketChip Gemini carrier board's inter-MCU communication. This specification implements only the **physical and data-link layers** for point-to-point connections, explicitly excluding the network layer (routing) which is reserved for future Titan-tier development.

### 1.1 Design Philosophy

- **Design for certifiability** without requiring formal certification
- **NASA-style documentation** templates (ICD, FMEA, test procedures)
- **Incremental implementation**: GPIO prototype first, LVDS hardware later
- **Proven heritage**: SpaceWire is flight-proven on numerous space missions

### 1.2 Scope

| Layer | ECSS-E-ST-50-12C | SpaceWire-Lite | Notes |
|-------|------------------|----------------|-------|
| Physical | Full | Partial | LVDS via transceiver; GPIO prototype mode |
| Data-Link | Full | Full | Data-strobe encoding, flow control |
| Network | Full | **Excluded** | No routing - point-to-point only |
| Transport | Optional | **Excluded** | Future Titan scope |

### 1.3 Related Documents

- `docs/icd/GEMINI_PROTOCOL_ICD.md` - Gemini-specific message formats and failover protocol
- `docs/icd/EXPANSION_CONNECTOR_ICD.md` - Physical connector specification
- `docs/GEMINI_CARRIER_BOARD.md` - Carrier board design overview

---

## 2. Physical Layer

### 2.1 Electrical Options

SpaceWire-Lite supports two electrical implementations:

#### 2.1.1 Production: LVDS (Low-Voltage Differential Signaling)

Per ECSS-E-ST-50-12C, SpaceWire uses LVDS for noise immunity and EMI reduction.

| Parameter | Specification | Notes |
|-----------|---------------|-------|
| Standard | TIA/EIA-644 LVDS | Differential signaling |
| Voltage Swing | 250-450 mV | Measured at receiver |
| Common Mode | 1.2V typical | |
| Data Rate | 2-400 Mbps | SpaceWire spec; Gemini targets 10 Mbps |
| Transceiver | SN65LVDS049 | Texas Instruments quad LVDS |

**Important:** LVDS **cannot** be implemented with RP2350 GPIO/PIO alone. External transceiver hardware is mandatory for production Gemini boards.

#### 2.1.2 Prototype: Single-Ended GPIO

For initial protocol validation using two Core boards connected via jumper wires:

| Parameter | Specification | Notes |
|-----------|---------------|-------|
| Voltage | 3.3V LVCMOS | RP2350 native GPIO |
| Data Rate | 1-10 Mbps | PIO-limited |
| Distance | <10 cm | Noise susceptibility limits range |
| Purpose | Protocol development only | Not for flight hardware |

### 2.2 Signal Definitions

SpaceWire uses four signals per direction (Data and Strobe, each differential):

| Signal | Direction | Description |
|--------|-----------|-------------|
| D+ / D- | TX | Data signal (differential pair) |
| S+ / S- | TX | Strobe signal (differential pair) |
| D+ / D- | RX | Data signal (differential pair) |
| S+ / S- | RX | Strobe signal (differential pair) |

**Total:** 8 wires for bidirectional link (4 TX + 4 RX)

For GPIO prototype mode, signals are single-ended:
- `DATA_TX`, `STROBE_TX` (outputs)
- `DATA_RX`, `STROBE_RX` (inputs)

### 2.3 Connector

See `docs/icd/EXPANSION_CONNECTOR_ICD.md` for physical connector specification. SpaceWire signals are allocated on the Gemini-specific pins of the expansion connector.

---

## 3. Data-Link Layer

### 3.1 Data-Strobe Encoding (DS Encoding)

SpaceWire uses Data-Strobe (DS) encoding where:
- **Data line** carries the actual data bits
- **Strobe line** transitions whenever Data does NOT transition

This ensures exactly one transition (Data XOR Strobe) per bit period, enabling clock recovery without a PLL.

```
Bit:     0   1   1   0   1   0   0   1
Data:   _/‾‾‾‾‾‾\_____/‾‾‾\_____/‾‾‾
Strobe: _____/‾‾‾‾‾\___/‾‾‾\___/‾‾‾\_
XOR:    _/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\  (clock recovered)
```

**Clock Recovery:** `CLK = DATA XOR STROBE`

### 3.2 PIO Implementation

The RP2350's PIO (Programmable I/O) state machines are ideal for DS encoding/decoding:

```
┌─────────────────────────────────────────────────────────┐
│                    RP2350 PIO                           │
│  ┌───────────────┐         ┌───────────────┐           │
│  │  TX State     │         │  RX State     │           │
│  │  Machine      │         │  Machine      │           │
│  │               │         │               │           │
│  │  Data ──────────────────────────────▶ GPIO (D_TX)   │
│  │  Strobe ────────────────────────────▶ GPIO (S_TX)   │
│  │               │         │               │           │
│  │               │         │ GPIO (D_RX) ─────▶ Data   │
│  │               │         │ GPIO (S_RX) ─────▶ Strobe │
│  │               │         │ XOR ──────────▶ Clock     │
│  └───────────────┘         └───────────────┘           │
└─────────────────────────────────────────────────────────┘
```

**Key advantage:** No external PLL required for clock recovery. The XOR operation can be performed in PIO or by simple logic gate.

### 3.3 Character Format

SpaceWire characters are 10 bits:

| Bits | Content | Description |
|------|---------|-------------|
| 0 | Parity | Odd parity over bits 1-9 |
| 1 | Data/Control | 0 = Data character, 1 = Control character |
| 2-9 | Payload | 8-bit data or control code |

#### 3.3.1 Control Characters

| Character | Code | Description |
|-----------|------|-------------|
| FCT | 00000000 | Flow Control Token |
| EOP | 00000001 | End of Packet (normal) |
| EEP | 00000010 | End of Packet (error) |
| ESC | 00000011 | Escape (prefix for time-codes) |
| NULL | ESC + FCT | Link initialization/keep-alive |

### 3.4 Flow Control

SpaceWire uses credit-based flow control:

1. Receiver advertises buffer space by sending FCT tokens
2. Each FCT grants 8 bytes of credit
3. Transmitter decrements credit per byte sent
4. Transmitter pauses when credit exhausted

**Gemini implementation:** 64-byte receive buffer = 8 FCT credits initially

### 3.5 Link Initialization

SpaceWire link startup sequence:

```
State: ErrorReset → ErrorWait → Ready → Started → Connecting → Run

1. ErrorReset: Hold for 6.4 µs minimum
2. ErrorWait: Wait for silence (12.8 µs)
3. Ready: Begin transmitting NULLs
4. Started: Detect first NULL from partner
5. Connecting: Exchange NULLs, verify link
6. Run: Normal operation, send FCTs
```

**Timeout:** If no NULL received within 12.8 µs after sending first NULL, return to ErrorReset.

---

## 4. Packet Format

### 4.1 Basic Packet Structure

```
┌─────────┬─────────────────────────┬─────┐
│ Header  │      Payload            │ EOP │
│ (opt)   │    (0-N bytes)          │     │
└─────────┴─────────────────────────┴─────┘
```

For Gemini point-to-point links, the header is minimal (no routing required).

### 4.2 Gemini Message Encapsulation

See `docs/icd/GEMINI_PROTOCOL_ICD.md` for Gemini-specific message types encapsulated within SpaceWire packets.

---

## 5. Error Handling

### 5.1 Error Detection

| Error Type | Detection Method | Recovery |
|------------|------------------|----------|
| Parity | Odd parity check | Send EEP, discard packet |
| Disconnect | No transitions for 850 ns | Link reset |
| Credit | Transmit with no credit | Protocol violation - link reset |
| Escape | Invalid escape sequence | Send EEP |

### 5.2 Error Reporting

Errors are reported via:
1. EEP character in data stream
2. Link state machine status register
3. Error counters (accessible via debug interface)

---

## 6. Timing Parameters

### 6.1 SpaceWire Standard Timing

| Parameter | Min | Typical | Max | Notes |
|-----------|-----|---------|-----|-------|
| Bit Rate | 2 Mbps | 10 Mbps | 400 Mbps | Gemini targets 10 Mbps |
| NULL Period | - | 100 ns | - | At 10 Mbps |
| Disconnect Timeout | 850 ns | - | - | No transitions |
| ErrorReset Duration | 6.4 µs | - | - | Minimum hold time |
| ErrorWait Duration | 12.8 µs | - | - | Silence detection |

### 6.2 Gemini-Specific Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Heartbeat Interval | 100 ms | Health monitoring |
| Failover Detection | 500 ms | 5 missed heartbeats |
| Command Timeout | 50 ms | Response required |

---

## 7. Implementation Phases

### 7.1 Phase 1: Protocol Validation (Current)

- Two Core boards connected via jumper wires
- Single-ended GPIO mode
- PIO-based DS encoding
- Focus: Validate protocol logic and timing

### 7.2 Phase 2: Gemini PCB Design

- Integrate SN65LVDS049 LVDS transceivers
- Hardware voting logic (74LVC1G08/1G32)
- Isolation circuits (digital or optical)
- Focus: Hardware design for production

### 7.3 Phase 3: Electrical Validation

- Noise immunity testing
- Failover timing verification
- EMI/EMC characterization
- Focus: Flight-worthiness assessment

---

## 8. Compliance Notes

### 8.1 ECSS-E-ST-50-12C Deviations

| Requirement | Standard | SpaceWire-Lite | Rationale |
|-------------|----------|----------------|-----------|
| Network Layer | Required | Excluded | Point-to-point only; routing is Titan scope |
| LVDS | Required | Optional (proto) | GPIO mode for development |
| Time-Codes | Required | Deferred | Not needed for Gemini failover |
| Galvanic Isolation | Required | Implemented | Via digital isolators |

### 8.2 Future Compliance Path

Full ECSS-E-ST-50-12C compliance is achievable by:
1. Adding routing capability (network layer)
2. Implementing time-code distribution
3. Formal verification of link state machine
4. Qualification testing per ECSS standards

This is reserved for Titan-tier or space-rated variants.

---

## 9. References

1. ECSS-E-ST-50-12C: SpaceWire - Links, nodes, routers and networks (ESA, 2008)
2. TIA/EIA-644: Electrical Characteristics of Low Voltage Differential Signaling (LVDS)
3. SN65LVDS049 Datasheet (Texas Instruments)
4. RP2350 Datasheet - PIO Chapter (Raspberry Pi)

---

## Appendix A: PIO Program Pseudocode

### A.1 DS Encoder (Transmit)

```
; DS Encoding: Strobe toggles when Data does NOT toggle
; Ensures one transition per bit (Data XOR Strobe)

.program ds_encode
    set pins, 0b00          ; Initial state: D=0, S=0
.wrap_target
    out x, 1                ; Get next bit
    jmp !x, data_zero
data_one:
    set pins, 0b10 [1]      ; D=1, S unchanged (toggle later if needed)
    jmp check_strobe
data_zero:
    set pins, 0b00 [1]      ; D=0, S unchanged
check_strobe:
    ; Toggle strobe if data didn't change from previous
    ; (Implementation details depend on state tracking)
.wrap
```

### A.2 DS Decoder (Receive)

```
; DS Decoding: Clock = Data XOR Strobe
; Sample Data on recovered clock edge

.program ds_decode
.wrap_target
    wait 1 pin 0            ; Wait for transition on D or S
    in pins, 1              ; Sample Data bit
    wait 0 pin 0            ; Wait for opposite edge
.wrap
```

*Note: Actual PIO programs require careful timing calibration. These are conceptual illustrations.*

---

## Appendix B: Glossary

| Term | Definition |
|------|------------|
| DS Encoding | Data-Strobe encoding used by SpaceWire |
| ECSS | European Cooperation for Space Standardization |
| EOP | End of Packet marker |
| EEP | Error End of Packet marker |
| FCT | Flow Control Token |
| LVDS | Low-Voltage Differential Signaling |
| NULL | Idle character (ESC + FCT) for link keep-alive |
| PIO | Programmable I/O (RP2350 feature) |

---

*Document maintained in: `standards/protocols/SPACEWIRE_LITE.md`*
