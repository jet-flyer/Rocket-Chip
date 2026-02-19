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
| Hardware Reference | `docs/hardware/HARDWARE.md` | Component specifications |
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

### 8.2 Dual-IMU Fusion and EKF Lane Switching

With two independent Core modules, Gemini naturally provides two independent IMU/baro/GPS sensor suites. This opens the door to an ArduPilot EKF3-style **lane switching** architecture, where each module runs its own EKF instance and the system selects the healthiest estimate in real-time.

#### Concept

ArduPilot's EKF3 runs up to 5 parallel "lanes" — each an independent filter instance that can use different sensor sources. When a lane's innovation checks fail (indicating sensor divergence), the system switches to a healthy lane. Gemini could implement a simplified version:

| Lane | IMU Source | Baro Source | Runs On |
|------|-----------|-------------|---------|
| Lane 0 | Module A IMU | Module A Baro | Core A |
| Lane 1 | Module B IMU | Module B Baro | Core B |

Each module runs its own ESKF and publishes a health/confidence metric. A selector (on whichever module is Primary) compares innovation magnitudes and covariance traces to pick the best estimate for state decisions.

#### Communication Bandwidth Requirements

The current SENSOR_SUMMARY message (24 bytes at 10 Hz) is designed for cross-validation, not fusion. Dual-IMU lane switching requires **full-rate sensor exchange** so each module can optionally run a cross-check against the partner's raw data:

| Data | Size | Rate | Bandwidth |
|------|------|------|-----------|
| Raw IMU (accel+gyro, 6-axis int16) | 12 bytes | 200 Hz | 19.2 kbps |
| Raw magnetometer (3-axis int16) | 6 bytes | 50 Hz | 2.4 kbps |
| Barometer (pressure + temp) | 6 bytes | 50 Hz | 2.4 kbps |
| EKF state summary (quaternion + bias flags) | 24 bytes | 50 Hz | 9.6 kbps |
| **Total** | | | **~34 kbps** |

This is well within SpaceWire-Lite capacity (1 Mbps GPIO prototype, 10 Mbps LVDS production) but requires a **high-rate bidirectional sensor channel** — the existing protocol only defines Primary→Secondary sensor flow at 10 Hz.

#### Protocol Implications

The current Gemini protocol (Section 4) would need:

1. **New message type: SENSOR_RAW (bidirectional, 200 Hz)** — Full-rate IMU exchange between modules. Must be lightweight (minimal framing overhead at this rate).
2. **New message type: EKF_HEALTH (bidirectional, 50 Hz)** — Each module publishes its EKF innovation magnitudes, covariance trace, and a lane-health enum (GOOD / DEGRADED / FAILED).
3. **Bidirectional sensor flow** — Current design is Primary→Secondary only for SENSOR_SUMMARY. Lane switching requires both modules to share raw data simultaneously.
4. **Latency budget** — At 200 Hz, messages must complete within 5 ms. At 10 Mbps LVDS, a 20-byte message takes ~16 µs (wire time), so latency is dominated by encode/decode, not link speed. At 1 Mbps GPIO prototype, wire time is ~160 µs — still feasible but tighter.

#### Considerations Before Implementation

- **Do both modules need the partner's raw IMU, or just the EKF health summary?** If lane switching only compares filter outputs (not raw sensor cross-fusion), the bandwidth requirement drops to ~12 kbps and the existing 10 Hz SENSOR_SUMMARY could be extended rather than adding a 200 Hz channel.
- **SpaceWire-Lite flow control at high rates** — The credit-based flow control (8-byte tokens) needs to be validated at 200 Hz bidirectional. Potential for head-of-line blocking if one direction backs up.
- **PIO state machine utilization** — DS encoding/decoding at 200 Hz message rate is within PIO capability, but concurrent bidirectional traffic means the PIO programs must handle full-duplex without stalling.
- **This is a stretch-goal feature** — Basic Gemini failover (heartbeat + state sync) should be validated first. Lane switching adds complexity that's only justified if sensor disagreement is a real failure mode in the field.

#### Reference

- ArduPilot EKF3 lane switching: `libraries/AP_NavEKF3/AP_NavEKF3_core.cpp` — `UpdateFilter()`, innovation check logic, and lane selection in the `AP_NavEKF3` wrapper class.

### 8.3 Cross-Board Watchdog Handoff

On single-board RocketChip (Core/Main), a hardware watchdog reset triggers a full chip reset and the system must recover from scratch (see IVP-51 Watchdog Recovery Policy). On Gemini, the partner board can take over before the WDT fires — this is the primary advantage of dual-redundancy.

#### Watchdog-to-Failover Integration

| Event | Single Board | Gemini |
|-------|-------------|--------|
| Subsystem failure (ESKF diverge) | Disable subsystem, continue degraded | Disable on failing board, partner takes nav authority |
| Core 1 sensor hang | WDT fires, full reboot | Partner detects via missed heartbeat, takes sensor authority before WDT fires |
| Core 0 main loop hang | WDT fires, full reboot | Partner detects via missed heartbeat, assumes Primary role |
| WDT reset on one board | Recovery boot path (IVP-51) | Partner already Primary, rebooted board rejoins as Secondary after self-test |
| WDT reset on both boards | Safe mode (reboot counter) | Both enter safe mode — hardware voting logic keeps pyro safe regardless |

#### Key Design Principle

The 5s watchdog timeout (IVP-30) is intentionally longer than the Gemini heartbeat failover window (1.5s per Section 5.2). This means **Gemini failover completes before the watchdog fires** in all single-board failure cases. The WDT only fires if the failing board's own internal recovery (subsystem restart, Core 1 relaunch) also fails — it's the third line of defense after subsystem restart and partner takeover.

#### Rejoin Protocol

When a board reboots after a WDT reset while its partner is operating:
1. Rebooted board reads scratch registers, detects WDT reboot
2. Enters Secondary role (does NOT attempt to reclaim Primary)
3. Runs full self-test (sensor init, ESKF health, I2C bus check)
4. Reports status to partner via SpaceWire HEALTH message
5. Partner validates (compares rebooted board's sensor data against its own for `⚠️ VALIDATE 30s`)
6. Only after validation: rebooted board eligible for Primary role again
7. If self-test fails or validation fails: rebooted board stays in degraded/monitoring mode

#### Ground-Side Implications

The single-board LAUNCH_ABORT policy (IVP-51) applies to each board independently on Gemini. If either board experiences a WDT reset during pre-flight, the entire system requires operator acknowledgement before arming. Both boards must report healthy for the ARM consensus (hardware AND gate) to engage.

### 8.4 Per-Board Status Indicators

Each Core module on the Gemini carrier should have its own dedicated status indicator visible to the operator. The carrier board provides:

| Indicator | Location | Purpose |
|-----------|----------|---------|
| LED_A (RGB or NeoPixel) | Near Module A slot | Module A health and role status |
| LED_B (RGB or NeoPixel) | Near Module B slot | Module B health and role status |
| LED_SYS (single color) | Center of carrier | System-level status (armed, safe mode, link health) |

#### LED Patterns (Preliminary)

| State | LED_A / LED_B | LED_SYS |
|-------|---------------|---------|
| Healthy Primary | Solid green | — |
| Healthy Secondary | Solid blue | — |
| Degraded (self-test partial) | Yellow blink | — |
| WDT recovery (rejoining) | Magenta blink | — |
| Failed (not responding) | Off or solid red | — |
| System armed | — | Solid amber |
| Launch abort (WDT event) | — | Red blink |
| Safe mode (reboot loop) | Solid red | Solid red |
| SpaceWire link healthy | — | Green (dim) |
| SpaceWire link down | — | Red (dim) |

These indicators allow ground crew to visually distinguish which board has a problem without needing serial terminal access. During pre-flight inspection, both board LEDs must show green before arming is permitted.

### 8.5 Certification Path

For applications requiring formal certification:

1. FMEA (Failure Mode and Effects Analysis) document
2. Formal verification of voting logic
3. Qualification testing per relevant standards
4. Independent design review

### 8.6 Dedicated ELRS Communications Core

On Gemini, one Core module could be dedicated to communications rather than redundancy — running as a dedicated ELRS receiver driving RF hardware directly, while the other Core handles navigation, logging, and control. This creates a natural "pilot and co-pilot" separation mirroring how spacecraft partition avionics between mission and bus functions.

Two implementation approaches exist:

**Port ELRS firmware to RP2350:** The ELRS source is open (C++ with HAL abstraction). The RP2350 is vastly more capable than the ESP8285 (single-core 80 MHz) that many ELRS receivers run. ELRS timing-critical work (FHSS hop synchronization, precise packet timing) would benefit from PIO and dual Cortex-M33 cores. Requires writing an RP2350 platform layer for ELRS and maintaining sync with upstream releases. Loses WiFi configuration path unless an ESP32 is added.

**Implement ELRS OTA protocol natively:** Clean-room receiver implementation in RocketChip firmware — just the over-the-air packet format, FHSS sequences, binding phrase handling, and LoRa modem configuration. The RP2350 drives LR1121 RF hardware (on Telstar Booster Pack) over SPI. Decoded RC channels and MAVLink frames go directly to shared memory via the SpaceWire inter-MCU link — no CRSF serial layer needed, lower latency than UART. Risk: ELRS OTA protocol changes between major versions could break compatibility.

**Additional opportunity — Integrated Remote ID:** With a dedicated Core running communications, FAA Remote ID (ASTM F3411-22a) broadcast could be implemented directly in firmware using a Bluetooth module on the Telstar board, eliminating the need for a separate RID add-on. The communications Core would have GPS data from the mission Core via SpaceWire and could format and broadcast Open Drone ID messages (reference implementation: https://github.com/opendroneid/opendroneid-core-c). See `docs/hardware/TELSTAR_BOOSTER_PACK.md` Section 8.3 for RID details.

Both approaches require Telstar Booster Pack hardware with direct-drive LR1121 RF (Option B or C in `docs/hardware/TELSTAR_BOOSTER_PACK.md`). This is a post-Gemini-validation milestone — basic dual-redundancy failover should be proven first.

See `docs/hardware/TELSTAR_BOOSTER_PACK.md` for full Telstar Booster Pack design documentation.

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
| 1.1 | 2026-02-03 | Claude Code CLI | Added Section 8.2: Dual-IMU fusion and EKF lane switching |
| 1.2 | 2026-02-05 | Claude | Added Section 8.6: Dedicated ELRS communications core |
| 1.3 | 2026-02-12 | Claude Code CLI | Added Sections 8.3-8.4: Cross-board watchdog handoff, per-board status LEDs |

---

*Document maintained in: `docs/hardware/GEMINI_CARRIER_BOARD.md`*
