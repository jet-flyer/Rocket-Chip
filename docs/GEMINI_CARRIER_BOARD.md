# Gemini Carrier Board Design Document

**Version:** 1.0 Draft
**Last Updated:** 2026-01-19
**Status:** Preliminary Design
**Roadmap Position:** Post-crowdfunding stretch goal

---

## 1. Overview

Gemini is a carrier board that mounts two identical RocketChip Core modules to create a fault-tolerant redundant flight computer. The name "Gemini" references both the Latin word for "twins" and NASA's Gemini program heritage.

> **Note:** Gemini is planned as a **crowdfunding stretch goal**, not part of the initial MVP. Development will begin after successful funding and delivery of Core/Main tiers. Closer stretch goals include features like Betaflight port and additional Booster Packs.

**Key Features:**
- Dual RP2350 MCUs for redundancy
- Hardware voting logic for pyro safety
- SpaceWire-derived inter-MCU communication
- Independent power regulation per module
- Automatic failover capability

### 1.1 Design Philosophy

- **Design for certifiability** without requiring formal certification
- **Document to NASA-style templates** (ICD, FMEA, test procedures)
- **Hardware safety** - Critical pyro decisions in discrete logic, not firmware
- **Graceful degradation** - Either module can operate if the other fails

### 1.2 Related Documents

| Document | Path | Description |
|----------|------|-------------|
| SpaceWire-Lite Spec | `standards/protocols/SPACEWIRE_LITE.md` | Inter-MCU communication protocol |
| Expansion Connector ICD | `docs/icd/EXPANSION_CONNECTOR_ICD.md` | Physical connector interface |
| Gemini Protocol ICD | `docs/icd/GEMINI_PROTOCOL_ICD.md` | Message formats and failover |
| Hardware Reference | `docs/HARDWARE.md` | Component specifications |
| Software Architecture | `docs/SAD.md` | Software design |

---

## 2. System Architecture

### 2.1 Block Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           GEMINI CARRIER BOARD                               │
│                                                                              │
│  ┌─────────────────────────────┐    ┌─────────────────────────────┐        │
│  │        CORE MODULE A        │    │        CORE MODULE B        │        │
│  │         (Primary)           │    │        (Secondary)          │        │
│  │  ┌─────────────────────┐   │    │   ┌─────────────────────┐   │        │
│  │  │      RP2350         │   │    │   │      RP2350         │   │        │
│  │  │   Dual Cortex-M33   │   │    │   │   Dual Cortex-M33   │   │        │
│  │  │   520KB SRAM        │   │    │   │   520KB SRAM        │   │        │
│  │  │   8MB PSRAM         │   │    │   │   8MB PSRAM         │   │        │
│  │  └─────────┬───────────┘   │    │   └─────────┬───────────┘   │        │
│  │            │               │    │             │               │        │
│  │  ┌─────────┴───────────┐   │    │   ┌─────────┴───────────┐   │        │
│  │  │ IMU | Baro | GPS    │   │    │   │ IMU | Baro | GPS    │   │        │
│  │  │ Radio | Storage     │   │    │   │ Radio | Storage     │   │        │
│  │  └─────────────────────┘   │    │   └─────────────────────┘   │        │
│  │                            │    │                             │        │
│  │  LDO_A ──┐                 │    │                 ┌── LDO_B   │        │
│  └──────────┼─────────────────┘    └─────────────────┼───────────┘        │
│             │                                        │                     │
│      ┌──────┴──────┐          SpaceWire          ┌───┴──────┐             │
│      │   POWER A   │◄────────────────────────────▶   POWER B │             │
│      │  Regulator  │           (LVDS)            │ Regulator │             │
│      └──────┬──────┘                             └───┬───────┘             │
│             │                                        │                     │
│             │    ┌─────────────────────────────┐     │                     │
│             │    │     ISOLATION BARRIER       │     │                     │
│             │    │  (Digital or Optical)       │     │                     │
│             │    └─────────────────────────────┘     │                     │
│             │                                        │                     │
│        ARM_A│                                        │ARM_B                │
│       FIRE_A│                                        │FIRE_B               │
│             │    ┌─────────────────────────────┐     │                     │
│             └───▶│   HARDWARE VOTING LOGIC     │◀────┘                     │
│                  │                             │                           │
│                  │  ARM_OUT = ARM_A AND ARM_B  │                           │
│                  │  FIRE_OUT = (FIRE_A OR FIRE_B) AND ARM_OUT              │
│                  │                             │                           │
│                  │  74LVC1G08 (AND)            │                           │
│                  │  74LVC1G32 (OR)             │                           │
│                  └──────────────┬──────────────┘                           │
│                                 │                                          │
│                    ┌────────────┴────────────┐                             │
│                    │     PYRO CHANNELS       │                             │
│                    │  CH0: Drogue Chute      │                             │
│                    │  CH1: Main Chute        │                             │
│                    │  CH2: Staging (opt)     │                             │
│                    │  CH3: Aux (opt)         │                             │
│                    └─────────────────────────┘                             │
│                                                                             │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │                      COMMON INTERFACES                                │  │
│  │   VBAT_IN | USB | Debug | Status LEDs | Arm Switch | Continuity      │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Key Design Decisions

#### 2.2.1 Dual RP2350 (not RP2040)

| Factor | Decision | Rationale |
|--------|----------|-----------|
| MCU Choice | RP2350 | Single toolchain (already built for RP2350) |
| Backup Capability | Full | Backup MCU can assume full flight computer role |
| Price Difference | ~$0.10/unit | Irrelevant at scale |
| Production Commitment | 20+ years | Both RP2040 and RP2350 have long-term availability |

#### 2.2.2 SpaceWire-Derived Communication

| Aspect | Choice | Rationale |
|--------|--------|-----------|
| Standard | ECSS-E-ST-50-12C subset | Flight-proven, well-documented |
| Layers | Physical + Link only | Network/routing is Titan scope |
| Encoding | Data-Strobe via PIO | Clock recovery via XOR, no PLL needed |
| Electrical | LVDS (SN65LVDS049) | Cannot be PIO-only; external transceiver required |
| Prototype | Single-ended GPIO | For protocol development before PCB |

#### 2.2.3 Hardware Voting Logic

**Critical Safety Decision:** Pyro control via discrete logic gates, not firmware.

| Gate | Function | Part | Description |
|------|----------|------|-------------|
| AND | ARM | 74LVC1G08 | Both MCUs must agree to ARM |
| OR | FIRE | 74LVC1G32 | Either MCU can FIRE once armed |

**Why hardware voting?**
- Works even if both processors are hung
- No firmware bugs can bypass safety logic
- Deterministic timing (nanoseconds, not milliseconds)
- Inspectable/auditable design

#### 2.2.4 Power and Isolation

| Feature | Implementation | Benefit |
|---------|----------------|---------|
| Separate regulators | LDO per module | Independent failure domains |
| Digital isolation | Si8620 or similar | Fault containment |
| Optical option | IL300 or similar | Maximum isolation (future) |
| OR-ing diodes | Ideal diode controller | Shared battery, no backfeed |

---

## 3. Hardware Components

### 3.1 Bill of Materials (Key Components)

| Component | Part Number | Qty | Function |
|-----------|-------------|-----|----------|
| Core Module | RocketChip Core | 2 | Flight computers |
| LVDS Transceiver | SN65LVDS049 | 2 | SpaceWire electrical interface |
| AND Gate | 74LVC1G08 | 1 | ARM voting logic |
| OR Gate | 74LVC1G32 | 1 | FIRE voting logic |
| Digital Isolator | Si8620 | 2 | I2C isolation |
| LDO Regulator | AP2112K-3.3 | 2 | 3.3V power per module |
| Ideal Diode | LTC4357 | 2 | Battery OR-ing |
| MOSFET Driver | Si2302 | 4 | Pyro channel drivers |
| Connector | JST-SH 4-pin | 2 | Qwiic/STEMMA QT |

### 3.2 Mechanical

| Parameter | Value |
|-----------|-------|
| Board Size | TBD (target: 60mm × 80mm) |
| Mounting | 4× M3 mounting holes |
| Module Interface | 2× Feather footprint |
| Weight Target | <50g (without modules) |

---

## 4. Communication Architecture

### 4.1 Inter-MCU Link

The SpaceWire-Lite link between Core modules provides:

| Feature | Specification |
|---------|---------------|
| Data Rate | 10 Mbps (LVDS), 1 Mbps (GPIO prototype) |
| Latency | <1 ms end-to-end |
| Error Rate | <10⁻⁹ (with LVDS) |
| Encoding | Data-Strobe (DS) |
| Flow Control | Credit-based (8-byte tokens) |

See `standards/protocols/SPACEWIRE_LITE.md` for protocol details.

### 4.2 Message Types

Primary message categories (see `docs/icd/GEMINI_PROTOCOL_ICD.md` for details):

| Category | Rate | Purpose |
|----------|------|---------|
| Heartbeat | 10 Hz | Health monitoring |
| State Sync | On change | Mission state coordination |
| Sensor Summary | 10 Hz | Cross-validation |
| Pyro Commands | On demand | ARM/FIRE coordination |
| Failover | On fault | Role transfer |

---

## 5. Redundancy and Failover

### 5.1 Failure Modes

| Failure | Detection | Response |
|---------|-----------|----------|
| Primary MCU hang | Heartbeat timeout (500ms) | Secondary takes over |
| Primary MCU crash | Watchdog + heartbeat | Secondary takes over |
| Communication loss | No messages (500ms) | Both continue independently |
| Sensor failure | Health flags | Partner sensor data |
| Power loss (one module) | Voltage monitor | Other module continues |

### 5.2 Failover Timing

```
Event Timeline:
────────────────────────────────────────────────────────────────────────
t=0ms      Primary fails (stops sending heartbeats)
t=100ms    Secondary: 1 missed heartbeat (WARNING)
t=200ms    Secondary: 2 missed heartbeats
t=300ms    Secondary: 3 missed heartbeats (DEGRADED)
t=400ms    Secondary: 4 missed heartbeats
t=500ms    Secondary: 5 missed heartbeats (FAILED)
           Secondary sends FAILOVER_INIT
t=1500ms   Secondary assumes PRIMARY role (after 1000ms hysteresis)
────────────────────────────────────────────────────────────────────────
Total failover time: 1.5 seconds (configurable)
```

### 5.3 Pyro Safety During Failover

The hardware voting logic ensures pyro safety regardless of software state:

| Scenario | ARM_A | ARM_B | FIRE_A | FIRE_B | PYRO |
|----------|-------|-------|--------|--------|------|
| Both idle | 0 | 0 | X | X | SAFE |
| One armed | 1 | 0 | X | X | SAFE |
| Both armed, no fire | 1 | 1 | 0 | 0 | ARMED |
| Both armed, A fires | 1 | 1 | 1 | 0 | FIRE |
| Both armed, B fires | 1 | 1 | 0 | 1 | FIRE |
| A fails while armed | 0 | 1 | X | X | SAFE (disarms) |
| A hangs with ARM=1 | 1 | 1 | 1 | X | FIRE (B can fire) |

**Key insight:** If one MCU fails with ARM asserted but cannot FIRE, the other MCU can still fire the pyro. This ensures deployment even with single-MCU failure during flight.

---

## 6. Development Path

### 6.1 Phase 1: Protocol Validation (Current)

**Goal:** Validate SpaceWire-Lite protocol and Gemini messaging

| Task | Status | Notes |
|------|--------|-------|
| Two Core boards + jumper wires | Pending | Single-ended GPIO mode |
| PIO DS encoder/decoder | Pending | RP2350 PIO programs |
| Heartbeat exchange | Pending | Basic health monitoring |
| State synchronization | Pending | Mission state sharing |
| Simulated failover | Pending | Software-triggered |

**Hardware needed:**
- 2× Adafruit Feather RP2350 HSTX (#6130)
- 4× jumper wires (D_TX, S_TX, D_RX, S_RX)
- USB cables for power/debug

### 6.2 Phase 2: Gemini PCB Design

**Goal:** Design production carrier board

| Task | Status | Notes |
|------|--------|-------|
| Schematic design | Pending | KiCad |
| LVDS transceiver integration | Pending | SN65LVDS049 |
| Voting logic circuit | Pending | 74LVC1G08/32 |
| Isolation circuits | Pending | Si8620 or optical |
| Power regulation | Pending | Dual LDO, OR-ing |
| PCB layout | Pending | 4-layer minimum |
| Design review | Pending | Council process |

### 6.3 Phase 3: Electrical Validation

**Goal:** Verify production hardware

| Task | Status | Notes |
|------|--------|-------|
| Board bring-up | Pending | Basic functionality |
| LVDS signal integrity | Pending | Eye diagram, jitter |
| Noise immunity testing | Pending | Conducted/radiated |
| Failover timing | Pending | Verify <2s |
| Pyro voting verification | Pending | All failure modes |
| Environmental testing | Pending | Temp, vibration |

---

## 7. HSTX FPC Option (Low Priority)

The RP2350 HSTX (High-Speed Transmit) interface offers a zero-hardware fallback for inter-MCU communication via FPC cable. However, this is **low priority** because:

1. HSTX is primarily designed for video output, not bidirectional communication
2. Protocol work may not transfer cleanly from SpaceWire
3. SpaceWire provides better heritage and documentation

**Status:** Research item, not planned for initial Gemini development.

---

## 8. Future Considerations

### 8.1 Titan Integration

Gemini is independent of the Titan tier, but compatible:

| Feature | Gemini Standalone | Gemini + Titan |
|---------|-------------------|----------------|
| MCU count | 2 (redundant pair) | 2+ (scalable) |
| Communication | SpaceWire P2P | SpaceWire network |
| Pyro channels | 2-4 | Expandable |
| TVC support | Via Core modules | Dedicated controller |

### 8.2 Certification Path

For applications requiring formal certification:

1. FMEA (Failure Mode and Effects Analysis) document
2. Formal verification of voting logic
3. Qualification testing per relevant standards
4. Independent design review

---

## 9. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| LVDS signal integrity | Medium | High | Prototype validation |
| PIO timing accuracy | Low | Medium | Characterization testing |
| Failover race conditions | Medium | High | Formal state machine analysis |
| Power isolation failure | Low | High | Redundant isolation |
| Supply chain (components) | Low | Medium | Multi-source parts |

---

## 10. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-01-19 | Claude | Initial draft from design session |

---

*Document maintained in: `docs/GEMINI_CARRIER_BOARD.md`*
