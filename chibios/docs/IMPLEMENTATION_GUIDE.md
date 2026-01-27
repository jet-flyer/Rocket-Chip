# ChibiOS RP2350 ArduPilot HAL Extension

## Project Context

RocketChip is pivoting from FreeRTOS to ChibiOS for ArduPilot integration. ChibiOS RP2350 support was merged to ArduPilot trunk January 8, 2026. The FreeRTOS implementation is preserved in the `freertos` branch.

**Goal:** Extend ArduPilot's AP_HAL_ChibiOS to support RP2350, enabling ArduPilot firmware to run on RocketChip hardware (Adafruit RP2350 HSTX Feather).

## Hardware Target

**Board:** Adafruit Feather RP2350 with HSTX Port (+ 8MB PSRAM variant)
- MCU: RP2350A (dual Cortex-M33 @ 150MHz)
- Flash: 8MB external QSPI
- SRAM: 520KB
- PSRAM: 8MB (optional, uses GPIO8)
- Crystal: 12MHz

**RocketChip Sensors:**
| Sensor | Type | Interface | Address | ArduPilot Driver |
|--------|------|-----------|---------|------------------|
| ICM20948 | 9-axis IMU | I2C | 0x68 | AP_InertialSensor_Invensensev2 |
| LIS3MDL | Magnetometer | I2C | 0x1E | AP_Compass_LIS3MDL |
| DPS310 | Barometer | I2C | 0x77 | AP_Baro_DPS280 |
| PA1010D | GPS | UART | - | Serial GPS |

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                      ArduPilot                              │
│  (Copter/Plane/Rover - unchanged)                           │
├─────────────────────────────────────────────────────────────┤
│                    AP_HAL_ChibiOS                           │
│  ┌─────────────────────────────────────────────────────────┐│
│  │ Existing STM32 code paths                               ││
│  │ #if defined(STM32H7) || defined(STM32F4) ...            ││
│  ├─────────────────────────────────────────────────────────┤│
│  │ NEW: RP2350 code paths                                  ││
│  │ #elif defined(RP2350)                                   ││
│  └─────────────────────────────────────────────────────────┘│
├─────────────────────────────────────────────────────────────┤
│                       ChibiOS                               │
│  os/hal/ports/RP/RP2350/ ✓ COMPLETE (Jan 2026)              │
├─────────────────────────────────────────────────────────────┤
│                      RP2350 Hardware                        │
└─────────────────────────────────────────────────────────────┘
```

## What Exists vs What's Needed

### ✅ EXISTS (ChibiOS upstream)
```
ChibiOS/os/hal/ports/RP/
├── RP2040/          # Complete
├── RP2350/          # Complete (Jan 2026)
└── LLD/             # Low-level drivers (GPIO, I2C, SPI, UART, DMA, etc.)
```

### 🔴 NEEDED (ArduPilot hwdef system)

| File | Location | Est. Lines |
|------|----------|------------|
| RP2350xx.py | hwdef/scripts/ | ~600 |
| chibios_hwdef.py patches | hwdef/scripts/ | ~500 |
| rp_mcuconf.h | hwdef/common/ | ~200 |
| rp_util.c/h | hwdef/common/ | ~300 |
| Driver mods | AP_HAL_ChibiOS/*.cpp | ~400 |
| hwdef.dat | hwdef/RocketChip/ | ~80 |

## RP2350 vs STM32 Key Differences

| Aspect | STM32 | RP2350 |
|--------|-------|--------|
| GPIO naming | PA0-PF15 (port+pin) | GPIO0-GPIO47 (flat) |
| Pin muxing | AF0-15 | FUNCSEL 0-31 |
| PAL macros | PAL_STM32_MODE_xxx | PAL_RP_MODE_xxx |
| DMA | Request-based, complex | Channel-based, any-to-any |
| Flash | Internal | External QSPI |

---

## Pin Database Generation (RP2350xx.py)

**CRITICAL: Do not use pattern-based extrapolation.**

### Source 1: RP2350 Datasheet
Fetch https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf

Parse Section 1.2.3 "GPIO functions (Bank 0)" - Table 2 contains exact mappings.

### Source 2: pico-sdk FUNCSEL values
File: `src/rp2350/hardware_structs/include/hardware/structs/io_bank0.h`

```c
GPIO_FUNC_HSTX = 0,      // RP2350 only
GPIO_FUNC_SPI = 1,
GPIO_FUNC_UART = 2,
GPIO_FUNC_I2C = 3,
GPIO_FUNC_PWM = 4,
GPIO_FUNC_SIO = 5,
GPIO_FUNC_PIO0 = 6,
GPIO_FUNC_PIO1 = 7,
GPIO_FUNC_PIO2 = 8,      // RP2350 only
GPIO_FUNC_GPCK = 9,      // Also XIP_CS1, CORESIGHT_TRACE
GPIO_FUNC_USB = 10,
GPIO_FUNC_UART_AUX = 11, // RP2350 only
GPIO_FUNC_NULL = 31,
```

### Source 3: Adafruit HSTX Feather specifics
- HSTX pins: GPIO12-19
- PSRAM: GPIO8 (reserved on PSRAM variant)
- ADC: GPIO26-29 only (RP2350A)
- Default I2C: GPIO2/3 (I2C1)

### Output format
```python
AltFunction_map = {
    "GPIO0:UART0_TX": 2,
    "GPIO0:SPI0_RX": 1,
    "GPIO0:I2C0_SDA": 3,
    # ... all 48 GPIOs × applicable functions
}

ADC_Map = {
    "GPIO26": 0,  # ADC0
    "GPIO27": 1,
    "GPIO28": 2,
    "GPIO29": 3,
}
```

---

## RocketChip hwdef.dat

```
MCU RP2xxx RP2350xx
APJ_BOARD_ID 9999
OSCILLATOR_HZ 12000000
FLASH_SIZE_KB 16384

define HAL_RAM_SIZE_KB 520

# UART0 - Debug/Console
GPIO0 UART0_TX UART0
GPIO1 UART0_RX UART0

# UART1 - GPS
GPIO4 UART1_TX UART1
GPIO5 UART1_RX UART1

# I2C1 - Sensors (Feather default)
GPIO2 I2C1_SDA I2C1
GPIO3 I2C1_SCL I2C1

# LED
GPIO25 LED OUTPUT LOW GPIO(0)

SERIAL_ORDER OTG1 UART0 UART1
I2C_ORDER I2C1

define HAL_INS_PROBE_LIST PROBE_IMU_I2C(Invensensev2, 1, 0x68, ROTATION_NONE)
define HAL_MAG_PROBE_LIST PROBE_MAG_I2C(LIS3MDL, 1, 0x1E, ROTATION_NONE)
define HAL_BARO_PROBE_LIST PROBE_BARO_I2C(DPS280, 1, 0x77)
```

---

## Validation Phases

### Phase 0: Pure ChibiOS (no ArduPilot)
- [ ] LED blink (GPIO25)
- [ ] UART console (115200)
- [ ] I2C scan finds 0x68, 0x1E, 0x77
- [ ] SPI loopback

### Phase 1: hwdef Infrastructure  
- [ ] RP2350xx.py parses without error
- [ ] chibios_hwdef.py recognizes RP2xxx
- [ ] `python3 chibios_hwdef.py RocketChip/hwdef.dat` works

### Phase 2: ArduPilot Compilation
```bash
./waf configure --board=RocketChip
./waf copter  # Target: 0 errors
```

### Phase 3: Boot Validation
```
APM: ArduPilot RocketChip V4.x.x
INS: 1 gyros detected
INS: 1 accels detected
Barometer 1 detected DPS310
Compass 1 detected LIS3MDL
```

---

## File Structure

```
Rocket-Chip/
├── chibios/
│   ├── README.md
│   ├── PROGRESS.md
│   ├── phase0_validation/
│   │   ├── test_led_blink/
│   │   ├── test_uart/
│   │   ├── test_i2c_scan/
│   │   └── test_spi/
│   └── ardupilot_extension/
│       ├── hwdef/
│       │   ├── scripts/RP2350xx.py
│       │   ├── common/rp_mcuconf.h
│       │   └── RocketChip/hwdef.dat
│       └── patches/
└── docs/CHIBIOS_MIGRATION.md
```

---

## Immediate First Task

**Start with Phase 0:** Create `chibios/phase0_validation/test_led_blink/`

1. Initialize ChibiOS on RP2350
2. Blink GPIO25 (onboard LED)
3. Prove toolchain + ChibiOS HAL work

Then: UART → I2C → SPI tests before touching ArduPilot.

---

## Resources

- RP2350 Datasheet: https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf
- Adafruit HSTX Feather: https://learn.adafruit.com/adafruit-feather-rp2350/pinouts
- ChibiOS trunk: https://github.com/ChibiOS/ChibiOS (`os/hal/ports/RP/`)
- ArduPilot hwdef: `libraries/AP_HAL_ChibiOS/hwdef/`
