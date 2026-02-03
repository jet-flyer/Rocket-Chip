# RocketChip Hardware Documentation

## Product Naming

### Tiers
| Tier | Name | Description |
|------|------|-------------|
| Base | **Core** | Bare-bones, lightweight, local logging only |
| Main | *TBD* | Full sensors, optional telemetry via Booster Packs |
| Advanced | **Titan** | High-performance sensors, pyro, RTOS, TVC |

*Note: Nova reserved for future product (potentially space-rated hardware).*

### Booster Packs
Expansion modules following rocketry-themed naming (specific names TBD when boards are designed):
- **Telemetry** - RFM69HCW radio, antenna
- **Pyro/Servo** - Pyro channels, servo PWM for TVC
- **Navigation** - GPS, backup baro
- **Power** - Solar charging, extended battery

---

## Current Prototype Hardware

### Core Components
| Function | Part | Adafruit P/N | Specs | Notes |
|----------|------|--------------|-------|-------|
| MCU | Feather RP2350 HSTX | #6130 | Dual M33 @ 150MHz, 520KB SRAM, 8MB PSRAM | Primary dev board |
| IMU | ICM-20948 9-DoF | #4554 | 9-axis (accel/gyro/mag) | STEMMA QT/I2C, 0x69 (default) |
| Barometer | DPS310 | #4494 | ±1Pa precision | STEMMA QT/I2C |
| Battery | Li-Ion 400mAh | #3898 | 3.7V, fits between Feather headers | |
| Debug | SWD Debug Probe | #5699 | RP2040/RP2350 compatible | For crash debugging, timing analysis |

### Onboard Features Used
- PSRAM (8MB) - High-speed data buffering
- Flash - Persistent storage
- NeoPixel - Status indication

---

## Auxiliary Hardware (On Hand)

Available for testing but not in active prototype:

### MCUs / Dev Boards
| Part | Adafruit P/N | Notes |
|------|--------------|-------|
| Pico 2W | #6087 | RP2350 + WiFi/BT, wireless dev option |
| KB2040 | #5302 | RP2040 Pro Micro form factor |
| Tiny 2350 | #6248 / Pimoroni PIM721 | RP2350, tiny footprint with castellations - evaluating for Core board candidate |
| ESP32-S3 Reverse TFT Feather | #5691 | Built-in display + WiFi/BT, 4MB flash, 2MB PSRAM |
| Feather M0 RFM69HCW | #3176 | Older M0 with integrated radio - potential for dedicated GCS/relay tasks |
| Feather M0 Adalogger | #2796 | Older M0 with SD card - potential for dedicated logging tasks |

### Sensors
| Part | Adafruit P/N | Notes |
|------|--------------|-------|
| ISM330DHCX + LIS3MDL FeatherWing | #4569 | 9-DoF combo, higher precision option |
| LSM6DSOX + LIS3MDL FeatherWing | #4517 | 9-DoF combo, alternative IMU option |
| BNO055 | #2472 | Onboard sensor fusion, 100Hz output |
| MPU-6050 | #3886 | 6-DoF accel/gyro, STEMMA QT |
| BMP280 | #2651 | Barometer, lower precision than DPS310 |
| VL53L4CD ToF Sensor | #5396 | Time of flight distance sensor, STEMMA QT |

### Accessories
| Part | Adafruit P/N | Notes |
|------|--------------|-------|
| FeatherWing OLED 128x64 | #4650 | Status display option |
| Proto PiCowBell | #5905 | Prototyping plate for Pico |
| Li-Ion 150mAh | #1317 | Smaller battery option |

---

## Planned / Future Hardware

### Navigation (Booster Pack)
| Part | Adafruit P/N | Notes |
|------|--------------|-------|
| PA1010D Mini GPS | #4415 | 10Hz, -165dBm, STEMMA QT - on hand |
| Ultimate GPS FeatherWing | #3133 | FeatherWing form factor - on hand |

#### GPS Interface Note

**Current approach:** PA1010D via I2C (Qwiic/STEMMA QT). The module outputs NMEA sentences over I2C, which we parse directly. This approach:
- Works with current hardware (no wiring changes)
- Verified working in `tests/smoke_tests/gps_i2c_test.cpp`

**Alternative approach:** If I2C GPS causes issues (e.g., bus contention, timing problems at high sensor rates), switch to UART:
- PA1010D also supports UART (requires rewiring TX/RX instead of Qwiic)
- Ultimate GPS FeatherWing uses UART natively
- UART provides more reliable communication for high-rate sensor systems

**Recommendation:** Start with I2C for simplicity. If bus contention becomes problematic, migrate to UART.

### Telemetry (Booster Pack)
| Part | Adafruit P/N | Notes |
|------|--------------|-------|
| LoRa Radio FeatherWing (RFM95W 915MHz) | #3231 | Current testing board, US ISM band |
| RFM69HCW Radio FeatherWing (915MHz) | #3229 | Alternative, shorter range but cheaper |

### Pro Tier Upgrades
| Part | Adafruit P/N | Notes |
|------|--------------|-------|
| ADXL375 High-G Accelerometer | #5374 | ±200g for high-power rockets |
| BMP580 | #6407 | 2cm noise @ 85Hz, upgrade from DPS310 |
| Pyro channels | TBD | MOSFET drivers or dedicated board |

### Storage Options
| Part | Notes |
|------|-------|
| Onboard PSRAM | 8MB, high-speed buffering (current) |
| Onboard Flash | Persistent storage (current) |
| SD Card FeatherWing | External logging option |

---

## Alternatives Considered

| Current | Alternative | Trade-off |
|---------|-------------|-----------|
| RP2350 HSTX Feather | ESP32-S3 Reverse TFT (#5691) | Built-in display + WiFi/BT, but less PSRAM (2MB vs 8MB) |
| DPS310 | BMP280 | Cheaper, lower precision |
| ICM-20948 | ISM330DHCX + LIS3MDL (#4569) | Different precision/range characteristics |

---

## Reference Designs

Studied for hardware/software architecture:

| Board | Relevance |
|-------|-----------|
| madflight FC3v2 | RP2350B flight controller with ICM-45686, BMP580, FreeRTOS - close to RocketChip Pro target |

## Ground Station Hardware

| Part | Adafruit P/N | Notes |
|------|--------------|-------|
| RFM95W LoRa Breakout (915MHz) | #3072 | Current GCS receiver, pairs with #3231 FeatherWing |
| Feather M0 Basic Proto | #2772 | RX bridge MCU - hosts RFM95W breakout |
| Adafruit Fruit Jam | - | RP2350 mini computer - candidate for GCS platform |

### Ground Station

**Status**: Planning to use Adafruit Fruit Jam with RC GCS code

**Previous approach (deprecated)**: Feather M0 + RFM95W breakout as RX bridge - code in `ground_station/` for reference only.

### Radio Configuration

Radio parameters are configured in `src/hal/Radio_RFM95W.cpp`:

| Parameter | Value | Notes |
|-----------|-------|-------|
| Frequency | 915.0 MHz | US ISM band |
| Spreading Factor | SF7 | Fastest, ~5.5 kbps |
| Bandwidth | 125 kHz | Standard LoRa |
| Coding Rate | 4/5 | Minimum error correction |
| Sync Word | 0x12 | Private network |
| TX Power | +20 dBm | Maximum legal for ISM |

**Note**: Ground station must use matching settings. See `ground_station/radio_rx.cpp` for reference implementation (deprecated).

## Market Benchmarks

Comparable rocket flight computers for feature/positioning reference:

| Product | Vendor | Notes |
|---------|--------|-------|
| **Fluctus** | Silicdyne | All-in-one: logging, telemetry, GPS, pyro, staging, remote control. Bundled ground station. Benchmark for Pro tier. |
| **STARLIGHT MINI** | Circuit Wizardry | Affordable ($20-50), basic logging + single pyro. Comparable to Core tier. |
| **TeleMetrum / TeleMega / EasyMega** | Altus Metrum | Premium standard: telemetry, GPS, multiple pyro. High-end benchmark. |
| **Eggtimer Quasar / Proton** | Eggtimer Rocketry | Popular hobbyist dual-deploy, optional telemetry. Affordable. |
| **AVA (All Vehicle Avionics)** | BPS.space | Active stability, TVC, landing capability. Inspiration for advanced control features. |

---

## Peripheral Interfaces

### Interface Summary by Tier

| Interface | Core | Mid | Pro/Titan | Primary Use Cases |
|-----------|:----:|:---:|:---------:|-------------------|
| **SPI** | ✓ | ✓ | ✓ | Sensors (high-rate), radio, flash, SD card |
| **I2C** | ✓ | ✓ | ✓ | GPS, Qwiic expansion, low-rate sensors |
| **UART** | ✓ | ✓ | ✓ | Debug console, GPS (alt), some radios |
| **PIO** | ✓ | ✓ | ✓ | PWM output, WS2812 LEDs, custom protocols |
| **GPIO** | ✓ | ✓ | ✓ | Arm switch, LEDs, pyro fire, continuity sense |
| **ADC** | ✓ | ✓ | ✓ | Battery voltage, pyro continuity |
| **CAN** | - | - | ✓ | Multi-board communication, Titan tier |
| **HSTX** | - | - | ✓ | High-speed video/data output, Titan tier |

### Interface Details

#### SPI
Primary high-speed bus for performance-critical peripherals.
- **Clock**: Up to 50MHz for sensors, 10MHz typical
- **Use**: IMU (ISM330DHCX), high-G accel (ADXL375), baro, radio (RFM9x/SX126x), flash, SD card
- **Note**: Preferred over I2C for sensors at >200Hz sample rates

#### I2C
Secondary bus for expansion and lower-rate peripherals.
- **Clock**: 400kHz (Fast Mode), 1MHz (Fast Mode Plus) where supported
- **Use**: GPS modules, Qwiic/STEMMA QT expansion, displays, bench testing
- **Note**: Qwiic connector retained for expandability even as primary sensors move to SPI

#### UART
Serial communication for debug and select peripherals.
- **Baud**: 115200 typical (debug), up to 921600 for GPS
- **Use**: Debug console, GPS (alternative to I2C), some radio modules
- **Note**: USB CDC available for development, hardware UART for flight

#### PIO (Programmable I/O)
RP2350's programmable state machines for timing-critical I/O.
- **Use Cases**:
  - **PWM Output**: TVC servos, deployment servos, ESCs - offloads timing from CPU
  - **WS2812/NeoPixel**: Status LEDs without CPU overhead
  - **Custom Protocols**: Bit-banged interfaces, signal capture
- **Note**: Preferred over hardware PWM for servo control due to flexibility and core offloading

#### GPIO
General-purpose digital I/O for discrete signals.
- **Outputs**: Pyro fire channels (via MOSFET), status LEDs, enable signals
- **Inputs**: Arm switch (debounced), continuity sense (with pull-ups), limit switches
- **Note**: Pyro channels require hardware interlocks - see Safety section

#### ADC
Analog input for voltage monitoring.
- **Resolution**: 12-bit
- **Use**: Battery voltage (via divider), pyro continuity check, analog sensors
- **Note**: RP2350 ADC is adequate for monitoring; not intended for high-precision analog sensing

#### CAN (Titan Tier)
Controller Area Network for robust multi-board communication.
- **Use**: Distributed avionics, sensor pods, redundant systems
- **Status**: Future - requires CAN transceiver hardware
- **Note**: Overkill for single-board designs; reserved for complex Titan configurations

#### HSTX (Titan Tier)
High-Speed Transmit interface on RP2350.
- **Use**: DVI/HDMI video output, high-bandwidth data streaming
- **Status**: Future - exploring use cases (ground station display, debug visualization)
- **Note**: Available on HSTX Feather; potential for real-time telemetry visualization

### Bus Performance Comparison

| Bus | Speed | Overhead @ 1kHz | Best For |
|-----|-------|-----------------|----------|
| SPI @ 10MHz | 10 Mbps | ~1.2% | High-rate sensors, radio |
| I2C @ 400kHz | 400 Kbps | ~30% | Expansion, low-rate sensors |
| I2C @ 1MHz | 1 Mbps | ~12% | GPS, displays |
| UART @ 921600 | ~1 Mbps | N/A | GPS, debug |

> **Design Principle**: SPI for flight-critical, high-rate data. I2C for expansion and bench convenience. PIO for timing-critical outputs.

---

## GPIO Assignments

### Feather Ecosystem Compatibility (Design Constraint)

**For Core and Main tier boards, Feather ecosystem compatibility is a design goal:**

- Standard Feather pinout must be maintained for 3rd-party FeatherWing compatibility
- Pin functions (SPI, I2C, UART, ADC) must match Adafruit Feather RP2350 assignments
- Booster Packs should use pins that don't conflict with common FeatherWings
- Only break compatibility if a critical technical requirement demands it

This ensures users can stack standard FeatherWings (displays, sensors, storage) with RocketChip boards.

**Exception:** Gemini is a separate carrier board with its own PCB - it repurposes some GPIO for inter-MCU communication and is not expected to be FeatherWing-compatible.

### Adafruit Pin Naming Convention

**Important:** Adafruit guides and tutorials use Arduino-style pin naming, not raw GPIO numbers.

| Arduino Name | Feather RP2350 GPIO | Notes |
|--------------|---------------------|-------|
| D5 | GPIO5 | Digital pins map directly |
| D6 | GPIO6 | |
| D9 | GPIO9 | |
| D10 | GPIO10 | Common SPI CS for FeatherWings |
| D11 | GPIO11 | Common RST for FeatherWings |
| D12 | GPIO12 | |
| D13 | GPIO13 | |
| A0 | GPIO26 | Analog pins start at GPIO26 |
| A1 | GPIO27 | |
| A2 | GPIO28 | |
| A3 | GPIO29 | |

When Adafruit documentation says "CS on pin 10", they mean GPIO10 (silkscreen label "D10" on Feather).

**Example - LoRa Radio FeatherWing (#3231):**
- "Feather M0" jumper position: CS=D10 (GPIO10), RST=D11 (GPIO11), IRQ=D6 (GPIO6)
- These are the Arduino pin names that map to GPIO numbers on RP2350

### Feather RP2350 HSTX Built-in Hardware

| GPIO | Function | Notes |
|------|----------|-------|
| 0 | UART0 TX | Serial1 default TX |
| 1 | UART0 RX | Serial1 default RX |
| 2 | I2C1 SDA | STEMMA QT connector, Wire default |
| 3 | I2C1 SCL | STEMMA QT connector, Wire default |
| 7 | Red LED | Built-in general purpose LED |
| 8 | PSRAM CS | **DO NOT USE** if PSRAM onboard |
| 12-19 | HSTX | HSTX peripheral pins (also on back connector) |
| 20 | SPI0 MISO | Default SPI MISO |
| 21 | NeoPixel | Built-in RGB status LED (PIO-driven) |
| 22 | SPI0 SCK | Default SPI SCK |
| 23 | SPI0 MOSI | Default SPI MOSI |

### Standard Feather RP2350 Pinout (Reference)

| Function | GPIO | Feather Pin | Notes |
|----------|------|-------------|-------|
| UART TX | GPIO0 | TX | Standard UART |
| UART RX | GPIO1 | RX | Standard UART |
| I2C SDA | GPIO2 | SDA | Qwiic/STEMMA QT |
| I2C SCL | GPIO3 | SCL | Qwiic/STEMMA QT |
| SPI RX | GPIO16 | MISO | Standard SPI |
| SPI SCK | GPIO18 | SCK | Standard SPI |
| SPI TX | GPIO19 | MOSI | Standard SPI |
| ADC0 | GPIO26 | A0 | Analog input |
| ADC1 | GPIO27 | A1 | Analog input |
| ADC2 | GPIO28 | A2 | Analog input |
| ADC3 | GPIO29 | A3 | Analog input |

### RocketChip Pin Assignments

| GPIO | Function | Notes |
|------|----------|-------|
| 2 | I2C1 SDA | IMU/Mag FeatherWing, DPS310 baro, GPS |
| 3 | I2C1 SCL | IMU/Mag FeatherWing, DPS310 baro, GPS |
| 7 | Red LED | Heartbeat/debug indicator |
| 21 | NeoPixel | Status LED (states defined per application) |
| 20 | SPI0 MISO | (Reserved for SPI sensors) |
| 22 | SPI0 SCK | (Reserved for SPI sensors) |
| 23 | SPI0 MOSI | (Reserved for SPI sensors) |
| - | SPI CS (multiple) | Per-device chip selects TBD |
| - | PIO PWM | TVC/deployment servos TBD |
| - | Arm Switch | Input with debounce TBD |
| - | Pyro Fire | Output via MOSFET TBD |
| - | Battery ADC | Voltage monitoring TBD |

### HSTX Connector (Back of Board)

The 22-pin HSTX connector on the back provides:
- GPIO 12-19 (8 consecutive pins for HSTX peripheral or general GPIO)
- Additional GPIO
- 3.3V power
- GND

**Note**: HSTX peripheral enables DVI output without overclocking or PIO. Future use for debug visualization or ground station display.

### Important Notes

1. **PSRAM (GPIO 8)**: If board has 8MB PSRAM installed, GPIO 8 is reserved for PSRAM CS. Do not use as general GPIO.
2. **E9 Erratum (A2 silicon)**: Affects high-impedance inputs and internal pulldowns. Use 8.2K or smaller external pull-down resistors where needed.
3. **LED Pin**: Always use `PICO_DEFAULT_LED_PIN` instead of hardcoding GPIO 7. The Pico SDK defines the correct pin for each board variant. Hardcoding GPIO 7 may work on some boards but fail on others.

---

## I2C Address Map

| Address | Device | Notes |
|---------|--------|-------|
| 0x68 or 0x69 | ICM-20948 | Primary 9-axis IMU (accel/gyro/AK09916 mag) |
| 0x77 or 0x76 | DPS310 | Barometer |
| 0x10 | PA1010D | GPS module |
| 0x6A or 0x6B | ISM330DHCX | Auxiliary IMU (if used) |
| 0x1C or 0x1E | LIS3MDL | Auxiliary magnetometer (if used) |
| 0x28 or 0x29 | BNO055 | Auxiliary IMU (if used) |

### Known Conflicts
- DPS310 and BMP280/BMP580 share 0x77/0x76 - cannot use simultaneously without multiplexer

### Sensor Bus Performance

> **Performance Note:** At 1kHz control loops, I2C @ 400kHz creates ~30% overhead per sensor read. SPI @ 10MHz reduces this to ~1.2%. For Pro tier or any application requiring 1kHz+ IMU sampling, prefer SPI-based sensors.

---

## Sourcing Guidelines

| Vendor | Role | Notes |
|--------|------|-------|
| **Adafruit** | Default | Feather ecosystem, excellent docs and libraries |
| **SparkFun** | Secondary | Unique breakouts, Qwiic/STEMMA QT compatible |
| **Pololu** | Niche | Compact designs, voltage regulators, motor controllers |
| **Tindie** | Niche | Indie marketplace for specialty boards (e.g., FPGA breakouts) |
| **DigiKey/Mouser** | Components | Discrete parts and production quantities |

---

## Regulatory Notes

### Telemetry (US)
- **915MHz ISM Band** - License-free but power-limited
- FCC Part 15 limits apply
- Duty cycle and power restrictions for license-free operation
- Check local regulations for non-US deployments

### Pyro Channels
- Safety interlocks required in firmware
- Physical arm switch recommended for Pro tier
- Follow NAR/TRA safety codes for high-power rocketry

---

## Gemini Carrier Board Hardware

The Gemini carrier board mounts two Core modules for redundant flight computer operation. See `docs/GEMINI_CARRIER_BOARD.md` for full design documentation.

### Gemini-Specific Components

| Component | Part Number | Qty | Function |
|-----------|-------------|-----|----------|
| LVDS Transceiver | SN65LVDS049 | 2 | SpaceWire-Lite electrical interface |
| AND Gate | 74LVC1G08 | 1 | ARM voting logic (both MCUs must agree) |
| OR Gate | 74LVC1G32 | 1 | FIRE voting logic (either MCU can fire) |
| Digital Isolator | Si8620 | 2 | Inter-module isolation |
| LDO Regulator | AP2112K-3.3 | 2 | Independent 3.3V per module |
| Ideal Diode Controller | LTC4357 | 2 | Battery OR-ing |
| MOSFET (Pyro Driver) | Si2302 | 4 | Pyro channel switches |

### Hardware Voting Logic

Pyro safety is implemented in discrete logic, not firmware:

```
ARM_OUT  = ARM_A AND ARM_B        (74LVC1G08)
FIRE_OUT = ARM_OUT AND (FIRE_A OR FIRE_B)  (74LVC1G32 + gate)
```

| Safety Feature | Implementation |
|----------------|----------------|
| Both agree to ARM | AND gate requires both MCU ARM signals |
| Either can FIRE | OR gate allows single MCU to fire once armed |
| Fail-safe on hang | If MCU hangs with ARM low, pyros stay safe |
| Works without firmware | Discrete logic operates even if both CPUs crash |

### Inter-MCU Communication

| Parameter | GPIO Prototype | LVDS Production |
|-----------|----------------|-----------------|
| Data Rate | 1 Mbps | 10 Mbps |
| Max Distance | 10 cm | 1 m |
| Noise Immunity | Low | High |
| Transceiver | None (direct GPIO) | SN65LVDS049 |

Protocol: SpaceWire-Lite (see `standards/protocols/SPACEWIRE_LITE.md`)

### Power Architecture

Each Core module has independent power regulation:

| Feature | Specification |
|---------|---------------|
| Input | Shared VBAT with ideal diode OR-ing |
| Regulation | Dedicated LDO per module |
| Isolation | Digital isolators on control signals |
| Failure Mode | Either module operates if other fails |

### Related Documents

- `docs/GEMINI_CARRIER_BOARD.md` - Design overview
- `docs/icd/EXPANSION_CONNECTOR_ICD.md` - Connector pinout
- `docs/icd/GEMINI_PROTOCOL_ICD.md` - Inter-MCU protocol
- `standards/protocols/SPACEWIRE_LITE.md` - Communication protocol
