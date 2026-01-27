# ChibiOS ArduPilot HAL Extension for RP2350

## Current Phase: 0 - Pure ChibiOS Validation

Proving ChibiOS runs on RP2350 hardware before ArduPilot integration.

## Setup

### Prerequisites

```bash
# ARM toolchain (should already be available from Pico SDK setup)
arm-none-eabi-gcc --version

# picotool for flashing
picotool version
```

### Clone ChibiOS

ChibiOS is not included as a submodule. Clone it manually:

```bash
cd chibios/
git clone --depth 1 https://github.com/ChibiOS/ChibiOS.git
```

## Directory Structure

```
chibios/
├── ChibiOS/               # ChibiOS source (clone separately)
├── phase0_validation/     # Pure ChibiOS tests (no ArduPilot)
│   └── test_led_blink/    # Phase 0 test project
├── ardupilot_extension/   # hwdef files and patches for ArduPilot
└── docs/                  # Documentation
    ├── IMPLEMENTATION_GUIDE.md
    └── KNOWN_ISSUES.md
```

## Building Phase 0 Tests

```bash
cd phase0_validation/test_led_blink
make

# Flash to device
make upload
# or
picotool load -f build/rocketchip_blink.uf2
```

## Phase Status

- [ ] Phase 0: Pure ChibiOS validation
  - [ ] LED blink test
  - [ ] UART console test
  - [ ] I2C scan test
  - [ ] SPI loopback test
- [ ] Phase 1: hwdef infrastructure
- [ ] Phase 2: ArduPilot compilation
- [ ] Phase 3: Boot validation
- [ ] Phase 4: Sensor validation

See [PROGRESS.md](PROGRESS.md) for detailed status.

## Configuration

The test projects are configured for:
- **Dual-core SMP**: Both cores enabled
- **Flash execution**: XIP from QSPI flash
- **Target board**: Adafruit Feather RP2350 HSTX (GPIO25 = LED)

## Known Issues

See [docs/KNOWN_ISSUES.md](docs/KNOWN_ISSUES.md) for ChibiOS-specific issues and workarounds.
