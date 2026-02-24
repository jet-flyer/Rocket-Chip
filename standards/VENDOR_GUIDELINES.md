# Vendor & OEM Guidelines

**Status:** PRELIMINARY — needs human review before treating as authoritative
**Last Updated:** 2026-02-20
**Purpose:** Centralized reference for vendor-specific constraints, datasheet-sourced values, and OEM recommendations. Prevents re-discovery of known hardware gotchas.

> **Review notice:** This document was assembled from scattered codebase references, LL entries, and web research. Values sourced from datasheets should be verified against the actual PDFs once acquired. Timing measurements are from Stage 2/3 hardware testing and are validated. I2C protocol details for PA1010D are from vendor app notes (not yet stored locally). Flag any discrepancies.

**Rule:** Before implementing or modifying any hardware driver, check this document first. When a new vendor constraint is discovered, add it here — not just in code comments or LL entries.

---

## Datasheet Inventory

Local copies stored in `docs/hardware/datasheets/` for centralized offline access.

| Component | Datasheet | Local Copy | Notes |
|-----------|-----------|------------|-------|
| ICM-20948 | TDK InvenSense DS-000189 v1.3 | `docs/hardware/datasheets/ICM-20948-datasheet-v1.3.pdf` | Includes AK09916 mag register map |
| DPS310 | Infineon DPS310 v01_02 | `docs/hardware/datasheets/DPS310-datasheet.pdf` | 41 pages, register map + noise specs |
| PA1010D / MT3333 | CDTop CD-PA1010D v0.03 | `docs/hardware/datasheets/PA1010D-datasheet-v03.pdf` | Module datasheet (27 pages) |
| PA1010D I2C protocol | GlobalTop "NMEA over I2C" app note | `docs/hardware/datasheets/PA1010D-NMEA-over-I2C-appnote.pdf` | I2C protocol details |
| PA1010D I2C protocol | Quectel L76-L I2C App Note V1.0 | `docs/hardware/datasheets/Quectel-L76-L-I2C-appnote.pdf` | Same MT3333 chipset, same protocol |
| RP2350 | Raspberry Pi | `docs/hardware/datasheets/rp2350-datasheet.pdf` | https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf |
| SX1276 / RFM95W | Semtech SX1276/77/78/79 | `docs/hardware/datasheets/SX1276-datasheet.pdf` | LoRa transceiver IC (Stage 9) |
| RFM69HCW | HopeRF RFM69HCW v1.1 | `docs/hardware/datasheets/RFM69HCW-datasheet.pdf` | ISM transceiver module (79 pages) |
| ISM330DHCX | ST ISM330DHCX | `docs/hardware/datasheets/ISM330DHCX-datasheet.pdf` | 6-DoF IMU (Adafruit 9-DoF FeatherWing) |
| LIS3MDL | ST LIS3MDL | `docs/hardware/datasheets/LIS3MDL-datasheet.pdf` | 3-axis magnetometer (Adafruit 9-DoF FeatherWing) |

---

## ICM-20948 (TDK InvenSense) — 9-Axis IMU

**Datasheet:** DS-000189 v1.3 (local copy available)
**I2C Address:** `0x69` (Adafruit default, AD0=HIGH). Alternate `0x68` requires solder jumper.
**LL Entries:** 13 (address), 21 (I2C master race condition)

### Bank-Switching Architecture

The ICM-20948 uses 4 register banks sharing address space 0x00–0x7F. Bank select register `0x7F` is common to all banks.

| Bank | Contents |
|------|----------|
| 0 | Sensor data (accel, gyro, temp, mag via EXT_SLV), WHO_AM_I, USER_CTRL |
| 1 | Self-test configuration |
| 2 | Accel/gyro config (ODR, full-scale range) |
| 3 | I2C master config (autonomous AK09916 mag reads) |

### Magnetometer Access: I2C Bypass Mode (Current)

**Implemented 2026-02-10.** The ICM-20948's internal I2C master has been disabled. Instead, bypass mode (`INT_PIN_CFG` bit 1, `I2C_BYPASS_EN=1`) connects the AK09916 magnetometer directly to the external I2C bus at address `0x0C`. This follows the ArduPilot `AP_InertialSensor_Invensensev2.cpp` approach.

- **Eliminates** the bank-switching race condition (LL Entry 21) — Bank 3 registers are no longer used
- **Eliminates** the I2C master stall and disable/enable corruption issues during calibration
- **AK09916 reads** at `0x0C` directly via `i2c_bus_read_reg()`, rate-divided to ~100Hz
- **Bank 3 namespace** removed from `icm20948.cpp` (bypass mode does not use the I2C master)

### I2C Master Race Condition (Historical — Resolved)

The internal I2C master previously read the AK09916 autonomously at ~100Hz using Bank 3 registers. It shared the bank-select register (0x7F) with external I2C reads. At rapid read rates (calibration), bank collisions caused accel registers to return zeros after ~150 reads. This was the motivation for migrating to bypass mode.

### Key Registers

| Register | Bank | Address | Purpose |
|----------|------|---------|---------|
| WHO_AM_I | 0 | 0x00 | Device ID = 0xEA |
| USER_CTRL | 0 | 0x03 | Bit 5: I2C_MST_EN |
| PWR_MGMT_1 | 0 | 0x06 | Sleep, clock select |
| INT_PIN_CFG | 0 | 0x0F | Bit 1: I2C_BYPASS_EN |
| ACCEL_XOUT_H | 0 | 0x2D | Accel data start (6 bytes) |
| GYRO_XOUT_H | 0 | 0x33 | Gyro data start (6 bytes) |
| TEMP_OUT_H | 0 | 0x39 | Temperature (2 bytes) |
| REG_BANK_SEL | all | 0x7F | Bank select (shared!) |

---

## DPS310 (Infineon) — Barometer

**Datasheet:** MISSING — acquire before IVP-41 (ESKF noise tuning)
**I2C Address:** `0x77` (SDO floating/low). Alternate `0x76`.
**Driver:** `lib/ruuvi.dps310.c` (proven third-party wrapper)

### Configuration

| Parameter | Value | Source |
|-----------|-------|--------|
| Oversampling | 8x | Driver default (precision vs speed balance) |
| Measurement rate | 8 Hz | Hardware data-ready gated (IVP-26 measured 7.9 Hz) |
| Precision | ±1 Pa | Adafruit product page |
| I2C read time | ~251 µs | Stage 2 measurement (IVP-25/26) |

### Gaps

- Noise density (Pa/√Hz) — needed for ESKF measurement noise matrix R
- Temperature compensation characteristics
- Altitude conversion accuracy vs raw pressure accuracy

---

## PA1010D (CDTop) — GPS Module

**Datasheet:** MISSING
**Chipset:** MediaTek MT3333
**Drivers:** `gps_uart.cpp` (UART, preferred), `gps_pa1010d.cpp` (I2C, fallback)
**LL Entries:** 20 (I2C bus interference), 24 (I2C settling delay)

### Transport Selection

GPS is available over both UART and I2C. UART is preferred — see `docs/SENSOR_ARCHITECTURE.md`.

| Transport | Driver | Interface | Rate | Notes |
|-----------|--------|-----------|------|-------|
| **UART** (preferred) | `gps_uart.cpp` | FeatherWing UART pins | 57600 baud, 10Hz | Interrupt-driven ring buffer. No bus contention. |
| **I2C** (fallback) | `gps_pa1010d.cpp` | Qwiic (0x10) | 400kHz, 10Hz | Requires 500µs settling delay after reads (LL Entry 24). Causes bus interference with IMU/baro (LL Entry 20). |

Detection order at boot: UART probed first (2-second presence timeout), I2C fallback if UART GPS absent.

### UART Protocol (gps_uart.cpp)

| Parameter | Value | Source |
|-----------|-------|--------|
| Default baud | 9600 | Factory default (MT3333) |
| Negotiated baud | 57600 | Set via PMTK251 in `gps_uart_init()` |
| Fix rate | 10Hz | Set via PMTK220,100 after baud negotiation |
| Ring buffer | 512 bytes | ISR-driven, handles 10Hz burst without overflow |
| Sentence filter | RMC+GGA+GSA | PMTK314 with GSA enabled for HDOP/fix type |

**Baud negotiation:** `gps_uart_init()` always starts at 9600 (cold-start safe), sends PMTK251 to switch to 57600, then reinits UART at the new baud. This handles both fresh power-on and warm reboot.

### I2C Protocol (gps_pa1010d.cpp, from GlobalTop/Quectel app notes)

| Parameter | Value | Source |
|-----------|-------|--------|
| I2C address | 0x10 | Fixed |
| TX buffer size | 255 bytes | GlobalTop app note |
| Max read per transaction | 255 bytes | GlobalTop app note |
| Inter-read delay | 2 ms minimum | GlobalTop app note (buffer refill time) |
| Command input interval | 10 ms minimum | GlobalTop app note |
| Padding byte | 0x0A (LF) | Repeats last valid byte when buffer empty |
| Post-read settling | 500µs `busy_wait_us()` | Required on shared bus (LL Entry 24) |

**I2C Vendor Recommendations:**
1. **Read the full 255-byte buffer** per transaction. Vendor explicitly warns against partial reads.
2. **Filter 0x0A padding** — discard `0x0A` bytes except when preceded by `0x0D` (legitimate CRLF in NMEA).
3. **Do NOT include 0x10 in I2C bus scans** — probing triggers continuous NMEA streaming that interferes with other bus devices.

### I2C Bus Interference Behavior

The PA1010D is fundamentally a UART device with an I2C wrapper. When any I2C read occurs (including a 1-byte probe), it begins streaming NMEA data on the bus. On a shared bus with IMU and baro (without settling delay):
- IMU: ~8.4% read failure rate (LL Entry 24, gps-10)
- Baro: 100% read failure rate (LL Entry 20)

**Resolution:** 500µs `busy_wait_us()` after each GPS I2C read eliminates contention (0% error rate at 10Hz, LL Entry 24 gps-12c). UART transport avoids this entirely.

### PMTK Commands Used

| Command | Purpose | Used By |
|---------|---------|---------|
| `PMTK251,57600` | Set baud rate to 57600 | UART init (negotiate from 9600) |
| `PMTK314,0,1,0,1,1,0,...` | Enable RMC+GGA+GSA | UART init (GSA for HDOP/fix type) |
| `PMTK314,0,1,0,1,0,0,...` | Enable RMC+GGA only | I2C init (minimal output) |
| `PMTK220,<ms>` | Set fix interval | Both (1000=1Hz, 200=5Hz, 100=10Hz) |

### NMEA Output Rates (at 1Hz fix, all sentences)

| Sentence | ~Bytes | Content |
|----------|--------|---------|
| GGA | 72 | Position, altitude, fix quality, HDOP |
| RMC | 68 | Speed, course, date/time, validity |
| GSA | 65 | DOP values, active satellites |
| GSV (×3) | 210 | Satellites in view |
| VTG | 34 | Track/speed made good |
| **Total** | **~449** | All enabled |
| **RMC+GGA only** | **~139** | After PMTK314 filter |
| **RMC+GGA+GSA** | **~205** | UART default (filter with GSA) |

---

## RP2350 (Raspberry Pi) — MCU

**Datasheet:** `docs/hardware/datasheets/rp2350-datasheet.pdf` (source: https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
**SDK Version:** 2.2.0

### Silicon Errata

| ID | Description | Impact | SDK Mitigation |
|----|-------------|--------|----------------|
| E2 | SIO register aliasing above +0x180 | Spurious hardware spinlock releases | `PICO_USE_SW_SPIN_LOCKS=1` (default). Uses LDAEXB/STREXB instead. Transparent to API. |
| (SDK #252) | I2C SDA hold time | Bus timeouts with multiple devices | Fixed in SDK 1.2.0 (PR #273). Our 2.2.0 has the fix. |

### Memory Constraints

| Constraint | Limit | Consequence | Source |
|------------|-------|-------------|--------|
| Stack size | Limited (default) | >1KB locals cause silent overflow | LL Entry 1 |
| PSRAM shared data | Unsafe cross-core | XIP cache coherency + exclusive monitor fails | SEQLOCK_DESIGN.md §5 |
| 64-bit atomics | Spinlock #13 fallback | Disables IRQs — use uint32_t instead | SEQLOCK_DESIGN.md §9 |

### USB CDC / Flash Interaction

**Init order is critical** (LL Entries 3, 4, 12):

```
1. Bus recovery + I2C init
2. Flash operations (calibration load, storage init)
3. stdio_init_all()  ← USB starts here
4. Wait for connection, then print
```

Flash operations make ALL flash inaccessible — including USB IRQ handlers. Reversing steps 2 and 3 breaks USB permanently until power cycle.

### Bus Timing (Stage 2/4 measurements)

**I2C (400kHz):**

| Device | Read Time | Rate |
|--------|-----------|------|
| ICM-20948 (23 bytes) | ~774 µs | ~1 kHz |
| DPS310 (6 bytes) | ~251 µs | ~8 Hz (data-ready gated) |
| PA1010D I2C (255 bytes) | ~5.8 ms (estimated) + 500µs settling | 10 Hz (if I2C GPS used) |

**UART (GPS, preferred):**

| Parameter | Value |
|-----------|-------|
| Baud rate | 57600 (negotiated from 9600 at init) |
| 10Hz burst size | ~205 bytes/fix (RMC+GGA+GSA) |
| Ring buffer | 512 bytes (ISR-driven, handles 10Hz without overflow) |

---

## Adafruit Feather RP2350 HSTX — Carrier Board

### Pin Assignments

| Function | GPIO | Notes |
|----------|------|-------|
| Built-in LED | 7 | Use `PICO_DEFAULT_LED_PIN`, not hardcoded |
| NeoPixel | 21 | PIO-driven WS2812 |
| I2C1 SDA | 2 | STEMMA QT / Qwiic connector |
| I2C1 SCL | 3 | STEMMA QT / Qwiic connector |
| PSRAM CS | 8 | **DO NOT USE** if 8MB PSRAM installed |
| SPI0 MISO | 20 | |
| SPI0 SCK | 22 | |
| SPI0 MOSI | 23 | |

### Board-Specific Notes

- Set `PICO_BOARD "adafruit_feather_rp2350"` before SDK import in CMakeLists.txt
- External I2C pull-ups present on STEMMA QT connector
- No built-in battery voltage divider (unlike some Feather variants)

---

## Adding New Components

When integrating a new sensor, radio, or peripheral:

1. **Acquire the datasheet** and store locally in `lib/<component>/ref/` or `docs/datasheets/`
2. **Read the datasheet** before writing any driver code — not just the Adafruit tutorial
3. **Check for errata** and app notes (often separate documents from the main datasheet)
4. **Add an entry to this document** with: I2C/SPI address, key constraints, timing, known gotchas
5. **Cross-reference** with Adafruit/SparkFun/ArduPilot implementations per CODING_STANDARDS.md Prior Art Research section
6. **Update the Datasheet Inventory table** at the top of this document

---

*See also: `CODING_STANDARDS.md` (Prior Art Research section), `.claude/LESSONS_LEARNED.md` (debugging journal), `docs/hardware/HARDWARE.md` (physical inventory)*
