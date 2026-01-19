# Expansion Connector Interface Control Document

**Document Number:** RC-ICD-001
**Version:** 1.0 Draft
**Last Updated:** 2026-01-19
**Status:** Preliminary Design

---

## 1. Purpose and Scope

This Interface Control Document (ICD) defines the physical and electrical interface for RocketChip expansion modules, including:

- **Booster Packs** - Telemetry, GPS/Navigation, Pyro/Servo, Power expansion modules
- **Gemini Carrier Board** - Dual-Core redundant flight computer configuration

The expansion connector is based on the **Adafruit Feather standard** to leverage existing ecosystem compatibility while adding RocketChip-specific extensions.

### 1.1 Related Documents

- `docs/GEMINI_CARRIER_BOARD.md` - Gemini carrier board design
- `docs/icd/GEMINI_PROTOCOL_ICD.md` - Inter-MCU protocol for Gemini
- `standards/protocols/SPACEWIRE_LITE.md` - SpaceWire-Lite communication protocol
- `docs/HARDWARE.md` - Hardware specifications

---

## 2. Physical Interface

### 2.1 Connector Type

| Parameter | Specification |
|-----------|---------------|
| Base Standard | Adafruit Feather |
| Connector Type | 0.1" (2.54mm) pitch headers |
| Row Spacing | 0.9" (22.86mm) |
| Pin Count | 16 pins per side (32 total) + optional extensions |
| Mating Height | Standard: 8mm; Low-profile: 5mm |

### 2.2 Mechanical Dimensions

```
                    ← 2.0" (50.8mm) →
    ┌──────────────────────────────────────┐
    │  ○  ○  ○  ○  ○  ○  ○  ○  ○  ○  ○  ○  │  ↑
    │                                      │  │
    │         FEATHER MODULE AREA          │  0.9"
    │                                      │  (22.86mm)
    │                                      │  │
    │  ○  ○  ○  ○  ○  ○  ○  ○  ○  ○  ○  ○  │  ↓
    └──────────────────────────────────────┘
       1  2  3  4  5  6  7  8  9 10 11 12
```

**Pin 1 Location:** Near USB connector end (standard Feather orientation)

### 2.3 Castellated Module Option

The Core board supports castellated edge connections for direct soldering onto carrier boards:

| Parameter | Specification |
|-----------|---------------|
| Castellation Pitch | 2.54mm (matching header pitch) |
| Castellation Diameter | 1.0mm typical |
| Plating | ENIG or hard gold for solderability |
| Solder Mask Clearance | 0.15mm from pad edge |

---

## 3. Pin Assignments

### 3.1 Standard Feather Pinout (32 pins)

Based on Adafruit Feather RP2350 HSTX (#6130):

#### Left Header (Pins 1-16, component side view)

| Pin | Name | Type | Description |
|-----|------|------|-------------|
| 1 | RST | I | Reset (active low) |
| 2 | 3V3 | PWR | 3.3V regulated output |
| 3 | AREF | I | Analog reference (not used on RP2350) |
| 4 | GND | PWR | Ground |
| 5 | A0 | I/O | GPIO26 / ADC0 |
| 6 | A1 | I/O | GPIO27 / ADC1 |
| 7 | A2 | I/O | GPIO28 / ADC2 |
| 8 | A3 | I/O | GPIO29 / ADC3 |
| 9 | D24 | I/O | GPIO24 |
| 10 | D25 | I/O | GPIO25 |
| 11 | SCK | I/O | GPIO18 / SPI0 SCK |
| 12 | MOSI | I/O | GPIO19 / SPI0 TX |
| 13 | MISO | I/O | GPIO16 / SPI0 RX |
| 14 | RX | I/O | GPIO1 / UART0 RX |
| 15 | TX | I/O | GPIO0 / UART0 TX |
| 16 | NC | - | Not connected |

#### Right Header (Pins 17-32, component side view)

| Pin | Name | Type | Description |
|-----|------|------|-------------|
| 17 | VBAT | PWR | Battery input (3.7-4.2V) |
| 18 | EN | I | Enable (pull low to disable regulator) |
| 19 | VBUS | PWR | USB 5V (when connected) |
| 20 | D13 | I/O | GPIO13 / LED |
| 21 | D12 | I/O | GPIO12 |
| 22 | D11 | I/O | GPIO11 |
| 23 | D10 | I/O | GPIO10 |
| 24 | D9 | I/O | GPIO9 |
| 25 | D6 | I/O | GPIO6 |
| 26 | D5 | I/O | GPIO5 |
| 27 | SCL | I/O | GPIO3 / I2C1 SCL |
| 28 | SDA | I/O | GPIO2 / I2C1 SDA |
| 29 | NC | - | Not connected |
| 30 | NC | - | Not connected |
| 31 | NC | - | Not connected |
| 32 | NC | - | Not connected |

### 3.2 RocketChip Signal Allocation

#### 3.2.1 Common Signals (All Configurations)

| Function | Pin(s) | Notes |
|----------|--------|-------|
| I2C (Qwiic) | SDA, SCL | Expansion sensors, GPS |
| SPI | SCK, MOSI, MISO | Radio, flash, high-rate sensors |
| UART | TX, RX | Debug, GPS alternate |
| ADC | A0-A3 | Battery, pyro continuity |
| Power | 3V3, GND, VBAT | Module power |

#### 3.2.2 Booster Pack Specific

| Pack Type | Signals Used | Chip Select / IRQ |
|-----------|--------------|-------------------|
| Telemetry (Radio) | SPI + D10 (CS) + D6 (IRQ) | Per radio module spec |
| GPS/Navigation | I2C (0x10) or UART | - |
| Pyro/Servo | D9, D11, D12 (pyro) + PIO PWM | Safety interlocks required |
| Power | VBAT, EN, ADC | Battery management |

#### 3.2.3 Gemini-Specific Allocation

For Gemini carrier board inter-MCU communication:

| Signal | Pin | Direction | Description |
|--------|-----|-----------|-------------|
| SW_DATA_TX | D24 | O | SpaceWire Data (to partner) |
| SW_STROBE_TX | D25 | O | SpaceWire Strobe (to partner) |
| SW_DATA_RX | D5 | I | SpaceWire Data (from partner) |
| SW_STROBE_RX | D6 | I | SpaceWire Strobe (from partner) |
| HEARTBEAT | D9 | I/O | Heartbeat signal (cross-connected) |
| ROLE_SELECT | D10 | I | Primary/Secondary role (active low = primary) |
| ARM_OUT | D11 | O | ARM signal to voting logic |
| FIRE_OUT | D12 | O | FIRE signal to voting logic |

**Note:** In GPIO prototype mode, these are direct connections. In production Gemini boards, LVDS transceivers are interposed on SW_* signals.

> **Configuration Note:** Gemini pin allocations (D5, D6, D9-D12) overlap with some Booster Pack allocations. This is intentional - Gemini is a **carrier board configuration**, not a Booster Pack. When using Gemini:
> - Each Core module retains its own radio/sensor interfaces
> - Inter-MCU pins are dedicated to the carrier board's SpaceWire link
> - Pyro control routes through Gemini's hardware voting logic
> - Booster Packs requiring these pins (Telemetry, Pyro/Servo) are not used directly; their functions are handled by the carrier board or individual Core modules

---

## 4. Electrical Specifications

### 4.1 Power Rails

| Rail | Voltage | Current (max) | Notes |
|------|---------|---------------|-------|
| 3V3 | 3.3V ±5% | 500mA total | Regulated from VBAT or VBUS |
| VBAT | 3.0-4.2V | 2A peak | Direct battery connection |
| VBUS | 5V ±10% | 500mA | USB power when connected |

### 4.2 Digital I/O

| Parameter | Specification |
|-----------|---------------|
| Logic Levels | 3.3V LVCMOS |
| VOH | >2.6V @ 4mA |
| VOL | <0.4V @ 4mA |
| VIH | >2.0V |
| VIL | <0.8V |
| Input Leakage | <1µA |
| ESD Protection | 2kV HBM (internal to RP2350) |

### 4.3 Analog Inputs (A0-A3)

| Parameter | Specification |
|-----------|---------------|
| Resolution | 12-bit |
| Input Range | 0V to 3.3V |
| Input Impedance | >1MΩ |
| Conversion Time | ~2µs |

### 4.4 I2C (Qwiic Compatible)

| Parameter | Specification |
|-----------|---------------|
| Standard | Qwiic / STEMMA QT |
| Speed Modes | Standard (100kHz), Fast (400kHz), Fast+ (1MHz) |
| Pull-ups | 4.7kΩ on Core board |
| Connector | JST SH 4-pin (on Core board) |

### 4.5 SPI

| Parameter | Specification |
|-----------|---------------|
| Clock Rate | Up to 50MHz (device dependent) |
| Mode | Mode 0 (CPOL=0, CPHA=0) default |
| Chip Select | Active low, directly driven GPIO |

---

## 5. Signal Integrity

### 5.1 Trace Requirements

| Signal Type | Impedance | Max Length | Notes |
|-------------|-----------|------------|-------|
| SPI (SCK, MOSI, MISO) | 50Ω ±10% | 5cm | Series termination recommended |
| I2C (SDA, SCL) | - | 30cm | Open drain, external pull-ups |
| SpaceWire (GPIO mode) | 50Ω | 10cm | Prototype only |
| SpaceWire (LVDS) | 100Ω diff | 1m | With proper termination |

### 5.2 Decoupling

Each expansion module should include:
- 10µF bulk capacitor on 3V3 rail
- 100nF ceramic capacitor near each IC
- Additional decoupling per IC requirements

---

## 6. Gemini Carrier Board Interface

### 6.1 Block Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                     GEMINI CARRIER BOARD                            │
│                                                                     │
│  ┌───────────────┐              ┌───────────────┐                  │
│  │   CORE A      │              │   CORE B      │                  │
│  │  (Primary)    │   SpaceWire  │  (Secondary)  │                  │
│  │               │◄────────────►│               │                  │
│  │  RP2350       │   (4 wires)  │  RP2350       │                  │
│  │               │              │               │                  │
│  └───────┬───────┘              └───────┬───────┘                  │
│          │                              │                           │
│          │ ARM_A                        │ ARM_B                     │
│          │ FIRE_A                       │ FIRE_B                    │
│          ▼                              ▼                           │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                   HARDWARE VOTING LOGIC                      │   │
│  │                                                              │   │
│  │   ARM_OUT = ARM_A AND ARM_B        (74LVC1G08)              │   │
│  │   FIRE_OUT = FIRE_A OR FIRE_B      (74LVC1G32)              │   │
│  │            (when armed)                                      │   │
│  └──────────────────────────┬──────────────────────────────────┘   │
│                             │                                       │
│                             ▼                                       │
│                      ┌─────────────┐                               │
│                      │  PYRO OUT   │                               │
│                      │  (to e-match)                               │
│                      └─────────────┘                               │
└─────────────────────────────────────────────────────────────────────┘
```

### 6.2 Power Isolation

Each Core module on the Gemini carrier has independent power regulation:

| Feature | Specification |
|---------|---------------|
| Isolation Type | Digital isolators (e.g., Si8620) |
| Isolated Rails | I2C, GPIO control signals |
| Shared Rails | VBAT (with ideal diode OR-ing) |
| Failure Mode | Either module can operate if other fails |

### 6.3 Role Selection

| ROLE_SELECT State | Core Role | Notes |
|-------------------|-----------|-------|
| LOW (grounded) | Primary | Actively controls mission |
| HIGH (floating/pulled) | Secondary | Hot standby, monitors primary |

Role selection is determined by carrier board hardware (jumper or switch), not firmware.

---

## 7. Expansion Module Requirements

### 7.1 Mechanical

- Must fit within Feather form factor (2.0" x 0.9")
- Height above board: 10mm max (excluding headers)
- Weight: 10g max recommended

### 7.2 Electrical

- Total current draw: <200mA from 3V3 rail (per module)
- Inrush current: <500mA for 10ms max
- All I/O must be 3.3V compatible (no 5V tolerant assumptions)

### 7.3 Identification

Each expansion module should implement one of:
1. **I2C EEPROM** at address 0x50 with module ID
2. **Resistor ID** on A3 pin (voltage divider)
3. **I2C device presence** at known address

---

## 8. Compliance and Testing

### 8.1 Verification Tests

| Test | Method | Pass Criteria |
|------|--------|---------------|
| Mechanical Fit | Physical insertion | Seats fully, no interference |
| Power | Load test | 3V3 within spec at 200mA |
| I2C | Scanner | All devices respond |
| SPI | Loopback | Data integrity at 10MHz |
| GPIO | Toggle test | Correct logic levels |

### 8.2 Environmental

| Parameter | Operating | Storage |
|-----------|-----------|---------|
| Temperature | -20°C to +60°C | -40°C to +85°C |
| Humidity | 5% to 95% RH (non-condensing) | 5% to 95% RH |
| Vibration | Per NAR/TRA safety codes | - |

---

## 9. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-01-19 | Claude | Initial draft |

---

*Document maintained in: `docs/icd/EXPANSION_CONNECTOR_ICD.md`*
