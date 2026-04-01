# RP2350 Board Comparison

**Purpose:** Authoritative hardware reference for compile-time board abstraction (Stage G).
Covers all RP2350-based boards in the RocketChip ecosystem. Currently: Feather RP2350 HSTX
(flight) and Fruit Jam (GCS). Future: Pico 2W, Tiny 2350, etc.

## Feather RP2350 HSTX vs Fruit Jam

Both boards share the same RP2350 silicon family and 8MB PSRAM. Differences are
pin assignments, peripherals, and form factor.

**Sources:** Adafruit product pages (#6130, #6200), learn.adafruit.com pinout guides.

---

## Summary

| Feature | Feather RP2350 HSTX (#6130) | Fruit Jam (#6200) |
|---------|----------------------------|-------------------|
| **MCU** | RP2350A (dual M33 @ 150MHz) | RP2350B (dual M33 @ 150MHz) |
| **Flash** | 8 MB | 16 MB |
| **PSRAM** | 8 MB (APS6404L, GPIO 8 CS) | 8 MB |
| **SRAM** | 520 KB | 520 KB |
| **Form Factor** | Feather (50.8 x 22.8 mm) | Credit card (85.6 x 54 mm) |
| **USB Client** | USB-C | USB-C |
| **USB Host** | No | Yes (2-port hub, GPIO 1/2) |
| **WiFi/BT** | No | ESP32-C6 coprocessor (AirLift) |
| **Display Output** | HSTX FPC connector | DVI/HDMI via HSTX |
| **Audio** | No | I2S DAC (TLV320DAC3100) + speaker |
| **SD Card** | No (FeatherWing add-on) | MicroSD slot (SPI0 or SDIO) |
| **Battery** | LiPo charger (200mA+) | No (USB powered) |
| **NeoPixels** | 1 (GPIO 21) | 5 (GPIO 32) |
| **Onboard LED** | Red, GPIO 7 (active high) | Red, GPIO 29 (active low, inverted) |
| **Buttons** | Reset + Boot | Boot (GPIO 0) + Button2 (GPIO 4) + Button3 (GPIO 5) |
| **STEMMA QT** | I2C1 (GPIO 2/3) | I2C0 (GPIO 20/21) |
| **SWD Debug** | 3-pin JST SH | 3-pin JST SH (PicoProbe) |

---

## Pin Assignments (Board Abstraction Layer)

These are the pins that differ between boards and must be selected at compile time
via `board.h`. The board abstraction layer (Stage G.1) provides these as `constexpr`
constants in per-board headers.

### I2C Bus

| Pin | Feather RP2350 HSTX | Fruit Jam |
|-----|---------------------|-----------|
| I2C Instance | `i2c1` | `i2c0` |
| SDA | GPIO 2 | GPIO 20 |
| SCL | GPIO 3 | GPIO 21 |

### SPI Bus (Radio)

| Pin | Feather RP2350 HSTX | Fruit Jam |
|-----|---------------------|-----------|
| SPI Instance | `spi0` | `spi1` |
| MISO | GPIO 20 | GPIO 28 |
| MOSI | GPIO 23 | GPIO 31 |
| SCK | GPIO 22 | GPIO 30 |

### Radio (RFM95W)

| Pin | Feather RP2350 HSTX | Fruit Jam |
|-----|---------------------|-----------|
| CS | GPIO 10 | GPIO 10 |
| RST | GPIO 11 | GPIO 6 |
| IRQ (DIO0) | GPIO 6 | GPIO 5 |

**Note:** On Fruit Jam, GPIO 5 is shared with Button3. Radio IRQ and button
cannot be used simultaneously — the button polling code must account for this.

### NeoPixel / WS2812

| Parameter | Feather RP2350 HSTX | Fruit Jam |
|-----------|---------------------|-----------|
| GPIO | 21 | 32 |
| Count | 1 | 5 |
| PIO gpiobase | 0 (default) | 16 (required for GPIO 32+) |

**Risk R2:** GPIO 32+ on RP2350B requires PIO `gpiobase=16` for pins in the 16-47
range. This is a less-tested SDK path. Verify early in Stage G.3.

### Onboard LED

| Parameter | Feather RP2350 HSTX | Fruit Jam |
|-----------|---------------------|-----------|
| GPIO | 7 (`PICO_DEFAULT_LED_PIN`) | 29 |
| Logic | Active HIGH | Active LOW (inverted) |

### Buttons

| Button | Feather RP2350 HSTX | Fruit Jam |
|--------|---------------------|-----------|
| Boot/UF2 | Reset + Boot (no GPIO) | GPIO 0 |
| User 1 | N/A | GPIO 4 |
| User 2 | N/A | GPIO 5 (shared with radio IRQ) |

---

## Peripherals Available Only on Fruit Jam

These peripherals exist only on the Fruit Jam and are not part of the initial
board abstraction. They are noted here for future Stage G+ or Mission Profile work.

### ESP32-C6 WiFi Coprocessor (AirLift)

| Pin | GPIO | Function |
|-----|------|----------|
| MOSI | 31 | SPI data to ESP (shared with radio SPI) |
| MISO | 28 | SPI data from ESP (shared with radio SPI) |
| SCK | 30 | SPI clock (shared with radio SPI) |
| CS | 46 | ESP chip select |
| IRQ | 3 | ESP interrupt |
| BUSY | 11 | ESP ready/busy |
| RESET | 22 | Peripheral reset (shared with audio DAC) |
| TX | 8 | UART1 TX to ESP |
| RX | 9 | UART1 RX from ESP |

**SPI bus sharing:** The ESP32-C6 shares SPI1 with the RFM95W radio. Both use
different CS pins (ESP=GPIO 46, Radio=GPIO 10). SPI bus must be arbitrated —
only one device active at a time. This is standard SPI multi-device operation.

**Future use:** WiFi bridge to PC/QGC (eliminates USB tether), OTA firmware
updates, cloud telemetry relay, mesh networking with other RocketChip nodes.

### MicroSD Card

| Pin | GPIO | Function |
|-----|------|----------|
| SCK | 34 | SPI0 clock |
| MOSI | 35 | SPI0 MOSI |
| MISO | 36 | SPI0 MISO |
| CS | 39 | Chip select |
| Detect | 33 | Card detect |

**Note:** SD card uses SPI0, which is separate from the radio SPI1. No bus
contention with the radio.

### I2S Audio (TLV320DAC3100)

| Pin | GPIO | Function |
|-----|------|----------|
| DIN | 24 | Data input |
| MCLK | 25 | Main clock |
| BCLK | 26 | Bit clock |
| WS | 27 | Word select |
| IRQ | 23 | Interrupt |
| RESET | 22 | Peripheral reset (shared with ESP32-C6) |

### DVI/HDMI Output (HSTX)

GPIO 12-19 dedicated to HSTX differential pairs (CK, D0, D1, D2).

### USB Host

| Pin | GPIO | Function |
|-----|------|----------|
| D+ | 1 | USB data+ |
| D- | 2 | USB data- |
| 5V Power | 11 | Host port power enable |

---

## Peripherals Available Only on Feather RP2350 HSTX

| Feature | Details |
|---------|---------|
| LiPo charger | 200mA+, JST connector, charge status LED |
| HSTX FPC | 22-pin FPC for external DVI/display |
| Feather form factor | Compatible with all FeatherWings |

---

## PSRAM Notes

Both boards have 8MB PSRAM. The flight firmware's PSRAM ring buffer
(`src/logging/psram_init.h`, `ring_buffer.h`) and flash flush pipeline
(`flash_flush.h`) can be reused on the Fruit Jam without modification,
provided the PSRAM CS pin is correct for each board.

**Feather:** PSRAM CS = GPIO 8 (confirmed, in `board_feather_rp2350.h`)
**Fruit Jam:** PSRAM CS = GPIO 47 (from Adafruit schematic, standard RP2350B
PSRAM pin — same as Pimoroni RP2350B boards). Defined in `board_fruit_jam.h`.
To be verified during J.2 parity gate: `psram_init(47)` must detect 8MB.

---

## Implications for Stage J (Fruit Jam HAL) and Stage 7 (Radio & Telemetry)

1. **PSRAM reuse:** GCS can use the same PSRAM ring buffer as flight firmware
   for high-rate RX packet logging. No need for a separate flash-only CSV logger.
2. **Flash capacity:** 16MB flash (vs 8MB) gives more room for flight tables
   and range test logs.
3. **ESP32-C6:** Future WiFi bridge to QGC eliminates USB tether requirement.
   SPI bus shared with radio — needs CS arbitration but no hardware conflict.
4. **SD card:** Available for bulk logging if flash is insufficient. Separate
   SPI bus (SPI0) from radio (SPI1).
5. **3 buttons:** Boot + 2 user buttons for field mode toggle, BW switching, etc.
6. **5 NeoPixels:** RSSI bar graph without external LEDs.
7. **Audio DAC:** Future audible alerts (packet received beep, link lost alarm).

---

## RP2350A vs RP2350B Silicon Differences (Discovered)

Behavioral differences between the RP2350A (Feather) and RP2350B (Fruit Jam) packages
discovered during RocketChip development. These are NOT documented in the Raspberry Pi
datasheet differences — they were found empirically.

### GPIO Pad Isolation at Reset (2026-03-31)

**Issue:** On RP2350B, GPIO pads 20/21 (I2C0 STEMMA QT) start with `ISO=1` (pad isolated),
`PDE=1` (pull-down enabled), `IE=0` (input disabled). On RP2350A, GPIO 2/3 (I2C1) do NOT
start isolated — they are immediately usable after reset.

**Impact:** `i2c_bus_recover()` was called before `gpio_set_function()`, which clears the
ISO bit. On RP2350A this worked fine (pads not isolated). On RP2350B, the recovery bit-banged
on electrically isolated pads — SDA/SCL appeared high from pull-ups but no I2C transactions
actually reached the bus. All device probes returned NACK.

**Fix:** Configure GPIO pads (SIO function + pull-ups) before calling `i2c_bus_recover()`,
then set I2C function after recovery. See commit `f7c5cce` (2026-03-31).

**Applicability:** Any RP2350B board using GPIO pins that start isolated. The Pico SDK's
`gpio_set_function()` clears the ISO bit, but code that touches GPIO before calling
`gpio_set_function()` (bit-bang recovery, probe, etc.) must de-isolate first.

---

*Last updated: 2026-03-31. Source: Adafruit product pages + learn.adafruit.com pinout guides + empirical testing.*
