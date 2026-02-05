# Telstar Booster Pack Design Document

**Version:** 1.0 Draft
**Last Updated:** 2026-02-05
**Status:** Preliminary Design
**Roadmap Position:** Post-MVP Booster Pack

---

## 1. Overview

Telstar is a Booster Pack providing a complete bidirectional RC control + telemetry link, plus FPV video transmission, for drone and vehicle use cases. Named after Telstar 1 (1962), the first active communications satellite — which pioneered live transatlantic television transmission.

Telstar combines an ExpressLRS (ELRS) RC receiver with a 5.8 GHz analog FPV video transmitter on a single Booster Pack. It serves a fundamentally different mission profile than the base board's built-in RFM95W LoRa radio:

| Radio | Role | Direction | Use Case |
|-------|------|-----------|----------|
| RFM95W (base board) | Simple telemetry downlink | One-way | Altitude, GPS, status for rocketry |
| Telstar (ELRS + VTx) | Full RC control + MAVLink telemetry + FPV video | Bidirectional | Drones, vehicles, TVC rockets |

Inspired by the TBS Crossfire Sixty9 AIO concept (RC receiver + VTx on one board) but built around the open-source ELRS ecosystem.

### 1.1 Related Documents

| Document | Path | Description |
|----------|------|-------------|
| Hardware Reference | `docs/hardware/HARDWARE.md` | Component specs, GPIO, pin assignments |
| Gemini Carrier Board | `docs/hardware/GEMINI_CARRIER_BOARD.md` | Dual-Core redundant flight computer |
| Expansion Connector ICD | `docs/icd/EXPANSION_CONNECTOR_ICD.md` | Physical connector interface |
| Gemini Protocol ICD | `docs/icd/GEMINI_PROTOCOL_ICD.md` | Inter-MCU protocol |
| Project Overview | `docs/PROJECT_OVERVIEW.md` | Product tiers and Booster Pack summary |
| PIO Documentation | `docs/PIO/` | PIO program designs |

---

## 2. ExpressLRS (ELRS) Background

ExpressLRS is an open-source RC link protocol: https://github.com/ExpressLRS/ExpressLRS

### 2.1 RF Hardware Generations

ELRS uses Semtech LoRa RF chips across three hardware generations:

| Generation | Band | Chip | Notes |
|------------|------|------|-------|
| Gen 1 | 900 MHz | SX1276 | Same chip family as the RFM95W on RocketChip's base board |
| Gen 2 | 2.4 GHz | SX1280 | Higher packet rate, smaller antenna |
| Gen 3 | Dual-band | LR1121 | Single chip handles both 900 MHz and 2.4 GHz |

### 2.2 MCU Options

Existing ELRS receivers use: ESP8285, ESP32, ESP32-C3, STM32.

### 2.3 Wired Interface

CRSF protocol over UART at 416666 baud (flight controller connection).

### 2.4 Operating Modes

ELRS supports two modes relevant to RocketChip:

| Mode | Description | Telemetry |
|------|-------------|-----------|
| **CRSF mode** | RC channels + limited predefined telemetry frames | GPS, battery, attitude, link stats |
| **MAVLink mode** | Full bidirectional MAVLink over single radio link | RC + GCS over one connection, replaces separate telemetry radio |

### 2.5 ELRS Gemini Mode

True diversity with dual independent RF chips (dual LR1121 for Xrossband). Simultaneous 900 MHz + 2.4 GHz operation for superior link quality in challenging RF environments.

The naming convergence with our Gemini carrier board is not coincidental — both reference duality from the same space program heritage. Gemini mode is particularly valuable for Telstar's telemetry robustness, providing two independent RF paths so antenna orientation changes during flight don't cause signal loss.

---

## 3. CRSF Protocol Summary

CRSF is purely a **wired serial framing protocol**, not an RF protocol. It defines how bytes are packaged over UART between physically connected devices.

### 3.1 Specifications

| Parameter | Value |
|-----------|-------|
| Official spec | https://github.com/tbs-fpv/tbs-crsf-spec |
| Community extensions | https://github.com/crsf-wg/crsf |
| Max frame size | 64 bytes (sync + payload + CRC8) |
| CRC polynomial | 0xD5 |
| Byte order | Big endian |

### 3.2 Physical Configurations

| Configuration | Wiring | Baud Rate | Use |
|---------------|--------|-----------|-----|
| Single-wire half-duplex | 1 wire | 400 kbaud default, negotiable to 1-2 Mbaud | RC handset <-> TX module |
| Dual-wire full-duplex | 2 wires | 416666 baud default, non-inverted, 3.0-3.3V | RX module <-> flight controller |

DMA-capable UART recommended for best performance.

### 3.3 Implementation Notes

- The "famous robustness" people attribute to CRSF actually comes from the RF link layer (LoRa FHSS), which is completely separate from the wired framing protocol
- Widely implemented: Betaflight, INAV, ArduPilot, ExpressLRS, EdgeTX/OpenTX, CRSFforArduino library

---

## 4. Physical Design Options

Three approaches with tradeoffs:

### 4.1 Option A: OTS Module Mounting (Recommended for V1)

Castellated pad footprints matching common ELRS receiver form factors (20x13mm nano standard). Users solder on whichever ELRS receiver they prefer (e.g., RadioMaster XR4, BetaFPV SuperX for Gemini Xrossband diversity).

| Aspect | Assessment |
|--------|------------|
| Risk | Low — leverages existing ELRS hardware ecosystem |
| Upgrade path | Users upgrade receivers independently as hardware generations evolve (SX1276->SX1280->LR1121 happened fast) |
| Board size | Slightly larger |
| Assembly | User soldering required |

### 4.2 Option B: Direct-Drive RF

Onboard LR1121 (or dual LR1121 for Gemini diversity) with PA/LNA, antenna matching, RF shielding, TCXO. Real RF PCB engineering: impedance-matched traces, proper ground planes, regulatory certification.

| Aspect | Assessment |
|--------|------------|
| Integration | Tighter, potentially better RF performance, single-board product |
| Complexity | Significant PCB complexity increase |
| Certification | RF certification burden (FCC, CE) |

### 4.3 Option C: Hybrid

Direct-drive LR1121 as primary path, plus SPI/GPIO broken out to castellated pads for external module bypass.

| Aspect | Assessment |
|--------|------------|
| Flexibility | Best of both worlds |
| PCB real estate | More required |

### 4.4 V1 Recommendation

**Option A.** Break out SPI and GPIO lines to the Booster Pack connector alongside UART so the interface supports both "talk CRSF to an OTS receiver" and "drive RF hardware directly" from the same physical pack — future-proofing for direct-drive variants.

---

## 5. FPV Video Transmitter

Integrated 5.8 GHz analog video transmitter (VTx) on the Telstar board.

| Parameter | Specification |
|-----------|---------------|
| Frequency | 5.8 GHz band |
| Type | Analog video (not digital) |
| Power | Adjustable, up to 1W with appropriate cooling and RF shielding |
| Antenna connector | SMA or U.FL for 5.8 GHz antenna |
| Video input | Camera connects directly to VTx |
| Control | VTx power/channel control via CRSF telemetry commands (Smart Audio or similar protocol) |

**Design notes:**
- Inspired by TBS Crossfire Sixty9 AIO: single board combining RC receiver + VTx
- RocketChip does not process video — just relays VTx configuration commands through the CRSF channel
- This is analog FPV video, not digital (DJI O3/HDZero/Walksnail). Digital FPV systems are closed ecosystems with proprietary protocols. Analog remains the open, hackable, and most widely compatible option for custom builds.

---

## 6. Interface to RocketChip

### 6.1 PIO-UART at 416666 Baud

Does not consume a hardware UART — the RP2350 has only 2, needed for GPS and debug.

| Parameter | Value |
|-----------|-------|
| Baud rate | 416666 |
| PIO clock divider | 150 MHz / 416666 ~ 360.0 (clean division) |
| PIO state machines | 2 from one PIO block: TX and RX, clocked at 8x or 16x baud rate |
| RX buffering | DMA filling circular buffer |
| Frame parsing | Polled from main loop, triggered by DMA completion |
| CRC validation | CRC8 table lookup in software |

### 6.2 PIO Resource Usage

Two PIO state machines from one PIO block. Keep second PIO block free for other uses (WS2812 LEDs, SpaceWire on Gemini, etc.).

PIO programs are ~8-10 instructions each, well within 32-instruction shared memory per block. Pico SDK `uart_rx.pio` and `uart_tx.pio` examples are directly adaptable.

---

## 7. MAVLink Integration

### 7.1 Path 1: CRSF-Native Telemetry

CRSF has predefined telemetry frame types (GPS, battery, attitude, link stats). RocketChip populates these frames, sent back through ELRS to handset. Displayed via LUA scripts on EdgeTX/OpenTX transmitter.

Lightweight but limited to predefined sensor types.

### 7.2 Path 2: Native MAVLink Mode (Preferred for GCS)

ELRS provides full bidirectional MAVLink — native telemetry downlink and RC control uplink over single radio. ELRS receiver outputs MAVLink directly on UART instead of CRSF. RocketChip treats as standard MAVLink serial port.

| Feature | Details |
|---------|---------|
| Retry system | ELRS "stubborn sender" for reliable delivery |
| Eliminates | Need for separate telemetry radio (e.g., separate RFM95W link to GCS) |

**Critical bandwidth note:** On legacy 900 MHz SX1276 hardware, MAVLink throughput is ~400 bps (severely limited). 2.4 GHz or dual-band LR1121 strongly recommended (~4000 bps throughput).

---

## 8. Standalone Product Potential

**Important design consideration:** Telstar has significant potential as a standalone product for the broader FPV and drone community, independent of the RocketChip ecosystem. When designing the PCB, consider breakouts and features that enable standalone use.

### 8.1 Standalone Use Cases

- **FPV racing/freestyle pilots** — All-in-one ELRS RX + VTx board, competing with TBS Sixty9
- **Custom drone builders** — Drop-in comms board for DIY frames running Betaflight/INAV/ArduPilot
- **RC fixed-wing/helicopter** — ELRS receiver with integrated VTx for FPV aircraft
- **Educational/STEM** — Affordable, open-source comms platform for drone building courses

### 8.2 Breakouts for Standalone Operation

Design the Telstar PCB with these breakouts available even when not connected to a RocketChip Core board:

| Breakout | Purpose | Notes |
|----------|---------|-------|
| CRSF UART TX/RX pads | Direct connection to any FC | Standard 416666 baud, 3.3V |
| PWM output pads (4-6 channels) | Direct servo/ESC control without FC | For simple RC builds, ELRS receivers support PWM output mode |
| 5V and GND pads | Power input from FC or battery regulator | Standard drone power |
| VTx video input pad | Camera connection | Analog video in |
| VTx antenna SMA/U.FL | 5.8 GHz antenna | Standard FPV antenna connector |
| ELRS antenna U.FL (x2 for diversity) | RC link antennas | Dual for Gemini diversity mode |
| UART2 / secondary serial | RID module, GPS input, or telemetry | See Section 8.3 |
| Boot/bind button | ELRS binding and firmware flash | Standard ELRS workflow |
| LED indicator | Link status | Standard ELRS LED patterns |

### 8.3 FAA Remote ID (RID) Module Support

FAA requires Remote ID for drones operating in US airspace (ASTM F3411-22a). Broadcast RID uses Bluetooth and/or WiFi beacons to transmit drone identity and position.

**Design considerations:**

- **UART passthrough for RID add-on modules:** Telstar should provide a secondary UART for RID modules like the Dronetag BS (17x14mm, 1g). These modules communicate via MAVLink over UART — receive position data from FC, broadcast via Bluetooth.
- **MAVLink forwarding:** When Telstar operates in MAVLink mode, it can forward position data to a connected RID module without requiring a separate FC serial port.
- **Physical mounting:** Castellated pads or JST-GH connector for RID module attachment directly on the Telstar board, keeping wiring clean.
- **Power:** RID modules need 3.3-5V at low current. Telstar's onboard regulation can supply this.
- **Bluetooth preferred over WiFi for RID:** Less interference with 2.4 GHz RC control, lower power consumption (10 mW vs 100 mW), uses separate beacon channels from drone frequencies.
- **Regulatory context:** Standard Remote ID = built into drone by manufacturer. Broadcast Remote ID modules = add-ons, limited to visual line-of-sight (VLOS). Supporting RID makes Telstar more attractive for builders needing compliance.
- **Open Drone ID:** Reference C library at https://github.com/opendroneid/opendroneid-core-c — could potentially run directly on RP2350 in Gemini dedicated-core configuration, eliminating separate RID module entirely.

### 8.4 Mounting and Form Factor

| Parameter | Specification |
|-----------|---------------|
| Mounting holes | 20x20mm and 30x30mm (standard drone stack patterns) |
| Castellated edges | Allow soldering directly onto carrier board or custom PCB |
| Booster Pack connector | Standard RocketChip expansion interface |
| Weight target | <5g without antennas (competitive with existing AIO boards) |

Include both mounting hole patterns or provide adapters (like TBS Sixty9).

---

## 9. Relationship to Base Board Radio

The RFM95W (SX1276) on the base board stays dedicated to RocketChip's native simple telemetry. Although the SX1276 is electrically the same chip family used by 900 MHz ELRS receivers, running ELRS requires a complete firmware stack (FHSS, timing sync, binding, telemetry scheduling) on a dedicated MCU.

Porting ELRS to the RP2350 to drive the existing RFM95W directly is technically possible but architecturally wrong — ties up CPU time when a $12 ELRS receiver handles it autonomously.

Telstar and RFM95W serve genuinely different mission profiles with no overlap.

---

## 10. Updated Booster Pack Lineup

| Pack | Name Origin | Function | Primary Interface | Status |
|------|-------------|----------|-------------------|--------|
| Mercury | Mercury program | Telemetry expansion (native LoRa) | SPI | Planned |
| Vulcan | Vulcan rocket | Pyro channels, servo PWM for TVC | GPIO/PIO | Planned |
| Juno | Juno rocket | GPS module, backup barometer | I2C/UART | Planned |
| Apollo | Apollo program | Solar charging, extended battery | Power rails | Planned |
| **Telstar** | **Telstar satellite** | **ELRS RC link + FPV video + RID** | **PIO-UART + SPI** | **Planned** |

---

## 11. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-02-05 | Claude | Initial draft from design session |

---

*Document maintained in: `docs/hardware/TELSTAR_BOOSTER_PACK.md`*
