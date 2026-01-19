# Gemini Inter-MCU Protocol Interface Control Document

**Document Number:** RC-ICD-002
**Version:** 1.0 Draft
**Last Updated:** 2026-01-19
**Status:** Preliminary Design

---

## 1. Purpose and Scope

This Interface Control Document defines the communication protocol between the two RP2350 MCUs on a Gemini carrier board. The protocol provides:

- **Health monitoring** via heartbeat messages
- **State synchronization** between primary and secondary modules
- **Failover coordination** for fault-tolerant operation
- **Command arbitration** for pyro and deployment actions

### 1.1 Related Documents

- `standards/protocols/SPACEWIRE_LITE.md` - Physical and data-link layer specification
- `docs/icd/EXPANSION_CONNECTOR_ICD.md` - Physical connector interface
- `docs/GEMINI_CARRIER_BOARD.md` - Carrier board design overview
- `docs/SAD.md` - Software Architecture Document

---

## 2. Protocol Overview

### 2.1 Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         GEMINI PROTOCOL STACK                       │
├─────────────────────────────────────────────────────────────────────┤
│  Application Layer                                                  │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐   │
│  │  Heartbeat  │ │    State    │ │   Failover  │ │   Command   │   │
│  │  Service    │ │    Sync     │ │   Manager   │ │   Arbiter   │   │
│  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘   │
├─────────────────────────────────────────────────────────────────────┤
│  Message Layer                                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  Message Framing | CRC-16 | Sequence Numbers | Ack/Nack     │   │
│  └─────────────────────────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────────────────┤
│  Transport Layer (SpaceWire-Lite)                                   │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  Packet Framing | Flow Control | Error Detection            │   │
│  └─────────────────────────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────────────────┤
│  Physical Layer                                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  DS Encoding | GPIO (proto) or LVDS (production)            │   │
│  └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.2 Design Principles

1. **Deterministic** - Bounded latency for all message types
2. **Fail-safe** - Hardware voting logic operates even if both MCUs hang
3. **Symmetric** - Either module can assume primary role
4. **Minimal** - Only essential messages for redundancy management

---

## 3. Message Format

### 3.1 General Message Structure

All messages use a common framing format:

```
┌──────┬──────┬──────┬──────────┬─────────────┬───────┬──────┐
│ SYNC │ LEN  │ SEQ  │ MSG_TYPE │   PAYLOAD   │ CRC16 │ EOP  │
│ (2B) │ (1B) │ (1B) │   (1B)   │  (0-64B)    │  (2B) │ (1B) │
└──────┴──────┴──────┴──────────┴─────────────┴───────┴──────┘
```

| Field | Size | Description |
|-------|------|-------------|
| SYNC | 2 bytes | Synchronization pattern: `0xAA55` |
| LEN | 1 byte | Total message length (including header, excluding SYNC) |
| SEQ | 1 byte | Sequence number (0-255, wrapping) |
| MSG_TYPE | 1 byte | Message type identifier |
| PAYLOAD | 0-64 bytes | Message-specific data |
| CRC16 | 2 bytes | CRC-16-CCITT over LEN through PAYLOAD |
| EOP | 1 byte | SpaceWire End of Packet marker |

**Minimum message size:** 8 bytes (no payload)
**Maximum message size:** 72 bytes (64-byte payload)

### 3.2 CRC-16 Calculation

| Parameter | Value |
|-----------|-------|
| Polynomial | 0x1021 (CCITT) |
| Initial Value | 0xFFFF |
| Input Reflection | No |
| Output Reflection | No |
| Final XOR | 0x0000 |

CRC is calculated over bytes from LEN through end of PAYLOAD.

---

## 4. Message Types

### 4.1 Message Type Summary

| Code | Name | Direction | Rate | Description |
|------|------|-----------|------|-------------|
| 0x01 | HEARTBEAT | Both | 10 Hz | Health status |
| 0x02 | HEARTBEAT_ACK | Both | Response | Heartbeat acknowledgment |
| 0x10 | STATE_SYNC | Primary → Secondary | On change | Mission state update |
| 0x11 | STATE_ACK | Secondary → Primary | Response | State acknowledged |
| 0x20 | SENSOR_SUMMARY | Primary → Secondary | 10 Hz | Key sensor values |
| 0x30 | ARM_REQUEST | Either | On demand | Request to arm pyros |
| 0x31 | ARM_CONFIRM | Either | Response | Confirm arm ready |
| 0x32 | FIRE_NOTIFY | Either | On demand | Notify intent to fire |
| 0x40 | FAILOVER_INIT | Secondary | On fault | Initiating failover |
| 0x41 | FAILOVER_ACK | Primary | Response | Failover acknowledged |
| 0xF0 | PING | Either | On demand | Link test |
| 0xF1 | PONG | Either | Response | Link test response |

### 4.2 HEARTBEAT (0x01)

Sent by both modules at 10 Hz to indicate health.

**Payload (12 bytes):**

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | role | 0=Primary, 1=Secondary |
| 1 | 1 | state | Current mission state |
| 2 | 2 | mission_time_ms | Mission time (lower 16 bits) |
| 4 | 1 | health_flags | Bit field (see below) |
| 5 | 1 | error_count | Communication errors since boot |
| 6 | 2 | battery_mv | Battery voltage in mV |
| 8 | 4 | uptime_s | Seconds since boot |

**Health Flags (bit field):**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | IMU_OK | IMU responding |
| 1 | BARO_OK | Barometer responding |
| 2 | GPS_OK | GPS has fix |
| 3 | RADIO_OK | Radio link active |
| 4 | STORAGE_OK | Flash storage available |
| 5 | ARMED | Pyros armed |
| 6 | LOGGING | Data logging active |
| 7 | RESERVED | - |

### 4.3 STATE_SYNC (0x10)

Sent by primary module when mission state changes.

**Payload (16 bytes):**

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | new_state | New mission state |
| 1 | 1 | prev_state | Previous state |
| 2 | 4 | state_entry_time_ms | Time of state entry |
| 6 | 4 | mission_time_ms | Current mission time |
| 10 | 2 | altitude_m | Current altitude (meters) |
| 12 | 2 | velocity_mps | Current velocity (m/s × 10) |
| 14 | 2 | flags | State-specific flags |

### 4.4 SENSOR_SUMMARY (0x20)

Sent by primary to secondary at 10 Hz for state verification.

**Payload (24 bytes):**

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 2 | accel_x | X acceleration (mg) |
| 2 | 2 | accel_y | Y acceleration (mg) |
| 4 | 2 | accel_z | Z acceleration (mg) |
| 6 | 2 | gyro_x | X rotation (0.1 °/s) |
| 8 | 2 | gyro_y | Y rotation (0.1 °/s) |
| 10 | 2 | gyro_z | Z rotation (0.1 °/s) |
| 12 | 4 | pressure_pa | Barometric pressure (Pa) |
| 16 | 2 | altitude_m | Computed altitude (m) |
| 18 | 2 | velocity_z | Vertical velocity (cm/s) |
| 20 | 4 | timestamp_us | Sample timestamp |

### 4.5 ARM_REQUEST (0x30)

Request to arm pyro channels. Both MCUs must send this for hardware voting logic to arm.

**Payload (4 bytes):**

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | channel_mask | Channels to arm (bit mask) |
| 1 | 1 | arm_code | Safety code (0x5A) |
| 2 | 2 | reserved | Must be 0x0000 |

### 4.6 FIRE_NOTIFY (0x32)

Notification that a pyro channel is being fired. Either MCU can fire once armed.

**Payload (4 bytes):**

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | channel | Channel being fired (0-3) |
| 1 | 1 | fire_code | Safety code (0xA5) |
| 2 | 2 | mission_time_ms | Time of fire command |

### 4.7 FAILOVER_INIT (0x40)

Sent by secondary when taking over as primary.

**Payload (8 bytes):**

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | reason | Failover reason code |
| 1 | 1 | last_known_state | Last known primary state |
| 2 | 4 | last_heartbeat_ms | Time of last primary heartbeat |
| 6 | 2 | reserved | Must be 0x0000 |

**Failover Reason Codes:**

| Code | Reason |
|------|--------|
| 0x01 | Heartbeat timeout |
| 0x02 | Invalid state detected |
| 0x03 | Communication error threshold |
| 0x04 | Explicit handover request |

---

## 5. Heartbeat and Health Monitoring

### 5.1 Timing Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Heartbeat Interval | 100 ms | 10 Hz nominal rate |
| Heartbeat Timeout | 500 ms | 5 missed heartbeats |
| Failover Hysteresis | 1000 ms | Delay before role assumption |
| Recovery Period | 5000 ms | Time before allowing re-failover |

### 5.2 Health State Machine

```
                        ┌─────────────┐
                        │   HEALTHY   │
                        │ (heartbeats │
            ┌───────────│  received)  │───────────┐
            │           └─────────────┘           │
            │                  │                  │
            │ miss 1-2         │ miss 3-4         │ miss 5+
            │ heartbeats       │ heartbeats       │ heartbeats
            ▼                  ▼                  ▼
     ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
     │   WARNING   │───▶│  DEGRADED   │───▶│   FAILED    │
     └─────────────┘    └─────────────┘    └─────────────┘
            │                  │                  │
            │ heartbeat        │ heartbeat        │ no recovery
            │ received         │ received         │ without reset
            ▼                  ▼                  ▼
     ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
     │   HEALTHY   │◀───│   HEALTHY   │    │  FAILOVER   │
     └─────────────┘    └─────────────┘    │  TRIGGERED  │
                                           └─────────────┘
```

---

## 6. Failover Protocol

### 6.1 Normal Operation

```
┌─────────────────┐                    ┌─────────────────┐
│     PRIMARY     │                    │    SECONDARY    │
│    (Core A)     │                    │    (Core B)     │
└────────┬────────┘                    └────────┬────────┘
         │                                      │
         │───── HEARTBEAT (role=PRIMARY) ──────▶│
         │                                      │
         │◀──── HEARTBEAT (role=SECONDARY) ─────│
         │                                      │
         │───── STATE_SYNC (on change) ────────▶│
         │                                      │
         │◀──── STATE_ACK ─────────────────────│
         │                                      │
         │───── SENSOR_SUMMARY (10 Hz) ────────▶│
         │                                      │
    (repeats)                              (monitors)
```

### 6.2 Failover Sequence

When secondary detects primary failure:

```
┌─────────────────┐                    ┌─────────────────┐
│     PRIMARY     │                    │    SECONDARY    │
│   (FAILED)      │                    │   (TAKING OVER) │
└────────┬────────┘                    └────────┬────────┘
         │                                      │
         │    (no heartbeat for 500ms)          │
         │                                      │
         │◀──── FAILOVER_INIT ─────────────────│
         │                                      │
         │     (wait 1000ms hysteresis)         │
         │                                      │
         │                                      │
         │              ┌─────────────────────────────────┐
         │              │ SECONDARY BECOMES PRIMARY       │
         │              │ - Assumes mission control       │
         │              │ - Continues state machine       │
         │              │ - Maintains pyro authority      │
         │              └─────────────────────────────────┘
         │                                      │
         │     (original primary recovers)      │
         │                                      │
         │───── HEARTBEAT (role=PRIMARY) ──────▶│
         │                                      │
         │◀──── HEARTBEAT (role=PRIMARY) ──────│ (conflict!)
         │                                      │
         │     (higher uptime wins, or          │
         │      hardware role select)           │
```

### 6.3 Role Arbitration

When both modules claim primary role:

1. **Hardware role select** takes precedence (ROLE_SELECT pin)
2. If hardware select is floating, **higher uptime** wins
3. Loser transitions to secondary role
4. Recovery period (5s) prevents rapid role flapping

---

## 7. Pyro Safety Protocol

### 7.1 Hardware Voting Logic

The Gemini carrier board implements hardware-based pyro safety:

```
ARM Logic (AND gate - 74LVC1G08):
    ARM_OUT = ARM_A AND ARM_B

    Both MCUs must agree to ARM. If either MCU is:
    - Not asserting ARM, or
    - Failed/hung with ARM low
    Pyros remain safe (disarmed).

FIRE Logic (OR gate - 74LVC1G32, gated by ARM_OUT):
    FIRE_OUT = ARM_OUT AND (FIRE_A OR FIRE_B)

    Once armed, EITHER MCU can fire. This ensures:
    - Deployment occurs even if one MCU fails during flight
    - Single-point-of-failure is eliminated
```

### 7.2 Arming Sequence

```
┌─────────────────┐                    ┌─────────────────┐
│     PRIMARY     │                    │    SECONDARY    │
└────────┬────────┘                    └────────┬────────┘
         │                                      │
         │  (Both MCUs detect ARM conditions    │
         │   - Altitude > threshold, or         │
         │   - Manual arm command)              │
         │                                      │
         │───── ARM_REQUEST (ch=0x01) ─────────▶│
         │                                      │
         │◀──── ARM_CONFIRM ───────────────────│
         │                                      │
    [Assert ARM_A]                        [Assert ARM_B]
         │                                      │
         ▼                                      ▼
┌─────────────────────────────────────────────────────────┐
│              HARDWARE: ARM_OUT = HIGH                   │
│              (Pyros are now armed)                      │
└─────────────────────────────────────────────────────────┘
```

### 7.3 Fire Sequence

```
┌─────────────────┐                    ┌─────────────────┐
│     PRIMARY     │                    │    SECONDARY    │
│   (detects      │                    │   (monitors)    │
│    apogee)      │                    │                 │
└────────┬────────┘                    └────────┬────────┘
         │                                      │
         │───── FIRE_NOTIFY (ch=0, drogue) ────▶│
         │                                      │
    [Assert FIRE_A pulse]                  [Log event]
         │                                      │
         ▼                                      │
┌─────────────────────────────────────────────────────────┐
│              HARDWARE: FIRE_OUT = HIGH (pulsed)         │
│              (Pyro fires)                               │
└─────────────────────────────────────────────────────────┘
```

---

## 8. Error Handling

### 8.1 Message-Level Errors

| Error | Detection | Recovery |
|-------|-----------|----------|
| CRC mismatch | CRC-16 validation | Discard, request retransmit |
| Sequence gap | SEQ number check | Log gap, continue |
| Invalid type | Unknown MSG_TYPE | Discard, log error |
| Timeout | No ACK within 50ms | Retry up to 3 times |

### 8.2 Link-Level Errors

| Error | Detection | Recovery |
|-------|-----------|----------|
| SpaceWire EEP | Error marker received | Discard packet |
| Disconnect | No transitions 850ns | Link reinitialize |
| Stuck | Same state >10ms | Link reset |

### 8.3 System-Level Errors

| Error | Detection | Recovery |
|-------|-----------|----------|
| Partner failure | 5 missed heartbeats | Failover |
| State divergence | STATE_SYNC mismatch | Log, resync |
| Both claim primary | Role conflict | Arbitration |

---

## 9. Implementation Notes

### 9.1 FreeRTOS Task Architecture

```cpp
// Gemini communication task
// Priority: 4 (same as MissionTask)
// Rate: 100 Hz processing, 10 Hz heartbeat TX

void GeminiTask(void* params) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    uint32_t heartbeatCounter = 0;

    while (true) {
        // Process incoming messages
        processIncomingMessages();

        // Send heartbeat at 10 Hz
        if (++heartbeatCounter >= 10) {
            heartbeatCounter = 0;
            sendHeartbeat();
        }

        // Check partner health
        updatePartnerHealth();

        // Handle state sync (primary only)
        if (isPrimary() && stateChanged()) {
            sendStateSync();
        }

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));
    }
}
```

### 9.2 Buffer Sizing

| Buffer | Size | Notes |
|--------|------|-------|
| TX buffer | 128 bytes | 2 max-size messages |
| RX buffer | 256 bytes | 4 max-size messages |
| Message queue | 16 entries | Incoming message queue |

### 9.3 Timing Budget

| Operation | Max Time | Notes |
|-----------|----------|-------|
| Message encode | 50 µs | Including CRC |
| Message decode | 50 µs | Including CRC verify |
| Heartbeat processing | 100 µs | Parse and update state |
| SpaceWire TX (72 bytes) | 576 µs | At 1 Mbps GPIO mode |
| SpaceWire TX (72 bytes) | 58 µs | At 10 Mbps LVDS mode |

---

## 10. Testing and Verification

### 10.1 Unit Tests

- [ ] Message encode/decode round-trip
- [ ] CRC-16 calculation
- [ ] Sequence number handling
- [ ] Health state machine transitions

### 10.2 Integration Tests

- [ ] Heartbeat exchange (2 Core boards)
- [ ] State synchronization
- [ ] Simulated failover
- [ ] Pyro arm/fire sequence (with mock hardware)

### 10.3 System Tests

- [ ] Full mission with failover injection
- [ ] EMI/noise immunity (LVDS mode)
- [ ] Long-duration stability (24+ hours)

---

## 11. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-01-19 | Claude | Initial draft |

---

*Document maintained in: `docs/icd/GEMINI_PROTOCOL_ICD.md`*
