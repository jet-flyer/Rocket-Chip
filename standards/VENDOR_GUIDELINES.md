# Vendor & OEM Guidelines

**Status:** PRELIMINARY — needs human review before treating as authoritative
**Last Updated:** 2026-02-07
**Purpose:** Centralized reference for vendor-specific constraints, datasheet-sourced values, and OEM recommendations. Prevents re-discovery of known hardware gotchas.

> **Review notice:** This document was assembled from scattered codebase references, LL entries, and web research. Values sourced from datasheets should be verified against the actual PDFs once acquired. Timing measurements are from Stage 2/3 hardware testing and are validated. I2C protocol details for PA1010D are from vendor app notes (not yet stored locally). Flag any discrepancies.

**Rule:** Before implementing or modifying any hardware driver, check this document first. When a new vendor constraint is discovered, add it here — not just in code comments or LL entries.

---

## Datasheet Inventory

Local copies prevent broken links and ensure offline access. Store PDFs in `lib/<component>/ref/` alongside the driver or library.

| Component | Datasheet | Local Copy | Notes |
|-----------|-----------|------------|-------|
| ICM-20948 | TDK InvenSense DS-000189 v1.3 | `lib/icm20948/ref/datasheet_ICM-20948-v1.3.pdf` | Includes AK09916 mag register map |
| DPS310 | Infineon | **MISSING** | Needed for ESKF noise tuning (IVP-41) |
| PA1010D / MT3333 | GlobalTop FGPMMOPA6H / CDTop | **MISSING** | Module datasheet |
| PA1010D I2C protocol | GlobalTop "NMEA over I2C" app note | **MISSING** | Available at sparkfun.com (see URLs below) |
| PA1010D I2C protocol | Quectel L76-L I2C App Note V1.0 | **MISSING** | Same MT3333 chipset, same protocol |
| RP2350 | Raspberry Pi | Online only | https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf |
| RFM95W | HopeRF / Semtech SX1276 | **MISSING** | Needed before Stage 8 (telemetry) |

### Source URLs (for acquisition)

- DPS310: https://www.infineon.com/dgdl/Infineon-DPS310-DataSheet-v01_02-EN.pdf
- PA1010D I2C app note (GlobalTop): https://cdn.sparkfun.com/assets/parts/1/2/2/8/0/GTOP_NMEA_over_I2C_Application_Note.pdf
- PA1010D I2C app note (Quectel): https://www.dragino.com/downloads/downloads/datasheet/other_vendors/L76-L/Quectel_L76-L_I2C_Application_Note_V1.0.pdf
- RFM95W / SX1276: https://cdn-shop.adafruit.com/product-files/3179/sx1276_77_78_79.pdf

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

### I2C Master Race Condition (CRITICAL)

The internal I2C master autonomously reads the AK09916 magnetometer at ~100Hz using Bank 3 registers. It shares the bank-select register (0x7F) with external I2C reads.

- **At normal 10Hz polling:** No issue — enough time between reads for bank state to settle.
- **At rapid read rates (calibration):** Bank 3 switches from internal master collide with external Bank 0 reads. After ~150 rapid reads, accel registers return zeros.
- **Mitigation:** Disable I2C master before rapid reads: `icm20948_set_i2c_master_enable(dev, false)`.
- **Future:** Migrate to I2C bypass mode (`INT_PIN_CFG` bit 1) per ArduPilot `AP_InertialSensor_Invensensev2.cpp`. This connects AK09916 directly to the external bus, eliminating the race entirely.

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

**Datasheet:** MISSING — acquire before IVP-31
**Chipset:** MediaTek MT3333
**I2C Address:** `0x10`
**LL Entry:** 20 (bus interference)

### I2C Protocol (from GlobalTop/Quectel app notes)

| Parameter | Value | Source |
|-----------|-------|--------|
| TX buffer size | 255 bytes | GlobalTop app note |
| Max read per transaction | 255 bytes | GlobalTop app note |
| Inter-read delay | 2 ms minimum | GlobalTop app note (buffer refill time) |
| Command input interval | 10 ms minimum | GlobalTop app note |
| Padding byte | 0x0A (LF) | Repeats last valid byte when buffer empty |
| Buffer overflow | Data lost | No backpressure — new NMEA data dropped when full |

### Vendor Recommendations

1. **Read the full 255-byte buffer** per transaction. Vendor explicitly warns against partial reads: "It is not recommended to adopt this scheme, there may be certain risks." (Quectel forum)
2. **Pico SDK has no read size limit** — `i2c_read_blocking()` accepts any length. The 32-byte limit in Adafruit Arduino library is a Wire.h software constraint (ATmega legacy), not a hardware limit.
3. **Filter 0x0A padding** — discard `0x0A` bytes except when preceded by `0x0D` (legitimate CRLF in NMEA).
4. **Do NOT include 0x10 in I2C bus scans** — probing triggers continuous NMEA streaming that interferes with other bus devices.

### Bus Interference Behavior

The PA1010D is fundamentally a UART device with an I2C wrapper. When any I2C read occurs (including a 1-byte probe), it begins streaming NMEA data on the bus. On a shared bus with IMU and baro:
- IMU: ~17% read failure rate
- Baro: 100% read failure rate

**Resolution:** GPS reads must be on the same core that owns the I2C bus (Core 1). GPS was physically removed from Qwiic chain during Stage 2; reconnect at IVP-31.

### PMTK Commands Used

| Command | Purpose | When |
|---------|---------|------|
| `PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0` | Enable RMC+GGA only | Init (reduces output from ~449 to ~139 bytes/sec) |
| `PMTK220,<ms>` | Set fix interval | Rate change (1000=1Hz, 200=5Hz, 100=10Hz) |

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

---

## RP2350 (Raspberry Pi) — MCU

**Datasheet:** Online (https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
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

### I2C Bus Timing (Stage 2 measurements)

| Device | Read Time | Rate |
|--------|-----------|------|
| ICM-20948 (23 bytes) | ~774 µs | ~1 kHz |
| DPS310 (6 bytes) | ~251 µs | ~8 Hz (data-ready gated) |
| PA1010D (255 bytes) | ~5.8 ms (estimated) | 10 Hz |

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
