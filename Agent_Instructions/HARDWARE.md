# RocketChip Hardware Documentation

## Product Naming

### Tiers
| Tier | Name | Description |
|------|------|-------------|
| Base | **Core** | Bare-bones, lightweight, local logging only |
| Main | *TBD* | Full sensors, optional telemetry via Booster Packs |
| Advanced | *Nova or Titan (TBD)* | High-performance sensors, pyro, RTOS, TVC |

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
| IMU | ICM-20948 | #4554 | 9-DoF, ±16g accel, ±2000dps gyro | STEMMA QT/I2C |
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
| Adafruit Fruit Jam | - | RP2350 mini computer - candidate for GCS platform |

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

## GPIO Assignments

*To be updated as pin assignments are finalized for new codebase.*

| GPIO | Function | Notes |
|------|----------|-------|
| - | I2C SDA | STEMMA QT sensors |
| - | I2C SCL | STEMMA QT sensors |
| - | SPI (if used) | SD card, high-speed sensors |
| - | NeoPixel | Status LED |

---

## I2C Address Map

| Address | Device | Notes |
|---------|--------|-------|
| 0x68 or 0x69 | ICM-20948 | Current IMU |
| 0x77 or 0x76 | DPS310 | Barometer |
| 0x6A or 0x6B | ISM330DHCX | Auxiliary IMU (if used) |
| 0x1C or 0x1E | LIS3MDL | Auxiliary mag (if used) |
| 0x28 or 0x29 | BNO055 | Auxiliary IMU (if used) |

### Known Conflicts
- DPS310 and BMP280/BMP580 share 0x77/0x76 - cannot use simultaneously without multiplexer

---

## Sourcing Policy

**Preferred:** Adafruit components for consistent availability, documentation, and library support.

**Secondary:** Pimoroni for unique prototyping hardware, especially compact form factors and specialized RP2040/RP2350 boards. Note: Pimoroni products are sourced directly from Pimoroni (some items previously carried by Adafruit are being delisted).

**Alternatives considered when:**
- Notably better performance for the application
- Significantly lower cost (>30% savings at scale)
- Adafruit/Pimoroni don't carry equivalent part

All substitutions require changelog entry with rationale.

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
