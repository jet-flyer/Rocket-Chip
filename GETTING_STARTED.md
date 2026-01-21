# Getting Started with RocketChip

This guide will help you set up your development environment and build the RocketChip firmware from source.

## Overview

RocketChip is a modular flight computer platform for RP2350-based boards (Raspberry Pi Pico 2, Adafruit Feather RP2350) running FreeRTOS SMP. For project vision and architecture details, see [PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md).

## Prerequisites

### Hardware

- **Development Board**: Raspberry Pi Pico 2 or Adafruit Feather RP2350 HSTX
- **USB Cable**: USB-C or Micro-USB (depending on board)
- **Optional Sensors**: ICM-20948 IMU, DPS310 Barometer (see [docs/HARDWARE.md](docs/HARDWARE.md))

### Software

You'll need the following tools installed:

- **CMake** (3.13 or later)
- **ARM GCC Toolchain** (arm-none-eabi-gcc)
- **Build essentials** (make, etc.)
- **picotool** (optional, for advanced flashing/debugging)
- **Git**

## Toolchain Installation

### Ubuntu/Debian

```bash
sudo apt update
sudo apt install cmake build-essential git
sudo apt install gcc-arm-none-eabi libnewlib-arm-none-eabi
```

For picotool (optional):
```bash
sudo apt install libusb-1.0-0-dev
git clone https://github.com/raspberrypi/picotool.git
cd picotool
mkdir build && cd build
cmake ..
make
sudo make install
```

### macOS

Using Homebrew:
```bash
brew install cmake git
brew install --cask gcc-arm-embedded
```

For picotool:
```bash
brew install picotool
```

### Windows

1. **CMake**: Download from [cmake.org](https://cmake.org/download/)
2. **ARM GCC**: Download from [ARM Developer](https://developer.arm.com/downloads/-/gnu-rm)
3. **Git**: Download from [git-scm.com](https://git-scm.com/download/win)
4. **Build tools**: Install [MinGW](https://www.mingw-w64.org/) or use WSL2 with Ubuntu instructions above

For picotool on Windows, building from source with Visual Studio or using WSL2 is recommended.

### Verify Installation

```bash
arm-none-eabi-gcc --version    # Should show ARM GCC version
cmake --version                 # Should show 3.13+
git --version                   # Should show git version
```

## Cloning the Repository

Clone the repository with submodules (pico-sdk and FreeRTOS-Kernel):

```bash
git clone --recursive https://github.com/jet-flyer/Rocket-Chip.git
cd Rocket-Chip
```

If you already cloned without `--recursive`, initialize submodules:

```bash
git submodule update --init --recursive
```

### Submodule Structure

The project uses several git submodules:

| Submodule | Purpose |
|-----------|---------|
| `pico-sdk/` | Raspberry Pi Pico SDK (hardware abstraction, USB, GPIO) |
| `FreeRTOS-Kernel/` | FreeRTOS SMP branch for dual-core support |
| `lib/st_drivers/` | ST platform-independent sensor drivers (ISM330DHCX, LIS3MDL, DPS310) |
| `lib/ardupilot/` | ArduPilot libraries (AP_Math, calibration, filters) |

**For CI/Test Environments**: If submodules weren't initialized at clone time, run:
```bash
git submodule update --init --recursive
```

**Note**: The pico-sdk submodule contains its own nested submodules (tinyusb, btstack, etc.) which are initialized automatically with `--recursive`.

## Building the Firmware

### Quick Build (Recommended)

Use the provided build script:

```bash
./build.sh
```

This script will:
1. Verify ARM GCC is installed
2. Create the `build/` directory
3. Run CMake with proper SDK paths
4. Build using all available cores

### Manual Build

If you prefer manual control:

```bash
mkdir build
cd build
cmake -DPICO_SDK_PATH=../pico-sdk ..
make -j$(nproc)
```

### Build Outputs

After a successful build, you'll find these files in `build/`:

| File | Purpose |
|------|---------|
| `freertos_validation.uf2` | Flash via USB bootloader |
| `freertos_validation.elf` | Debugging with GDB/OpenOCD |
| `freertos_validation.bin` | Raw binary |
| `freertos_validation.hex` | Intel HEX format |

## Flashing to Hardware

### UF2 Bootloader Method (Recommended)

1. **Enter Bootloader Mode**:
   - Hold the **BOOTSEL** button on your Pico 2 / Feather RP2350
   - While holding, connect the USB cable to your computer
   - Release the button

2. **Copy Firmware**:
   - A drive named `RPI-RP2` will appear
   - Copy `build/freertos_validation.uf2` to this drive
   - The device will automatically reboot and run the firmware

### Using picotool (Alternative)

```bash
# Load firmware (device must be in BOOTSEL mode)
picotool load build/freertos_validation.uf2

# Load and reboot
picotool load -x build/freertos_validation.uf2

# Check device info
picotool info
```

## Verifying the Build

### Expected Behavior

After flashing, you should see:

1. **LED Blink**: The onboard LED blinks at 2Hz (twice per second)
2. **USB Serial**: Connect at 115200 baud to see output

### Serial Monitor

Connect to the USB serial port:

```bash
# Linux
screen /dev/ttyACM0 115200

# macOS
screen /dev/tty.usbmodem* 115200

# Or use any serial terminal (PuTTY, minicom, etc.)
```

### Expected Output

```
[BOOT] RocketChip FreeRTOS SMP Validation
[BOOT] Running on RP2350 dual-core
[BOOT] Tasks starting...
[STATUS] Sensor samples: 1000, Logger processed: 1000, Queue depth: 0, Free heap: 61440
```

Status reports appear every 5 seconds showing:
- **Sensor samples**: Should increment ~1000/sec
- **Logger processed**: Should track sensor count
- **Queue depth**: Should stay near 0
- **Free heap**: Should be ~60KB

## Troubleshooting

### Build Issues

| Problem | Solution |
|---------|----------|
| `arm-none-eabi-gcc: not found` | Install ARM GCC toolchain (see installation above) |
| CMake errors about pico-sdk | Run `git submodule update --init --recursive` |
| Permission denied on `build.sh` | Run `chmod +x build.sh` |

### Flashing Issues

| Problem | Solution |
|---------|----------|
| `RPI-RP2` drive doesn't appear | Ensure BOOTSEL is held while connecting USB |
| UF2 file rejected | Verify you built for RP2350, not RP2040 |
| picotool can't find device | Check USB connection, try different port |

### Runtime Issues

| Problem | Solution |
|---------|----------|
| No LED blink | Check `PICO_DEFAULT_LED_PIN` (GPIO 25 for Pico 2, GPIO 13 for Feather) |
| No serial output | Wait 2-3 seconds after power-on for USB enumeration |
| Serial shows garbage | Verify baud rate is 115200 |

For detailed troubleshooting, see [docs/TOOLCHAIN_VALIDATION.md](docs/TOOLCHAIN_VALIDATION.md).

## Project Structure

```
Rocket-Chip/
├── src/
│   ├── main.c              # FreeRTOS validation firmware
│   └── hooks.c             # FreeRTOS hooks
├── include/                # Header files (planned)
├── pico-sdk/               # Raspberry Pi Pico SDK (submodule)
├── FreeRTOS-Kernel/        # FreeRTOS SMP (submodule)
├── docs/                   # Architecture documentation
│   ├── SAD.md              # Software Architecture Document
│   ├── SCAFFOLDING.md      # Directory structure
│   └── TOOLCHAIN_VALIDATION.md
├── standards/              # Coding and workflow standards
├── build.sh                # Build script
├── CMakeLists.txt          # CMake configuration
└── FreeRTOSConfig.h        # FreeRTOS settings
```

## Next Steps

Once you have a working build:

1. **Read the Architecture**: [docs/SAD.md](docs/SAD.md) explains the system design
2. **Understand the Hardware**: [docs/HARDWARE.md](docs/HARDWARE.md) lists sensors and pin assignments
3. **Review Coding Standards**: [standards/CODING_STANDARDS.md](standards/CODING_STANDARDS.md)
4. **Check Project Status**: [PROJECT_STATUS.md](PROJECT_STATUS.md) for current work

## Contributing

1. Create a feature branch from `main`
2. Follow the [coding standards](standards/CODING_STANDARDS.md)
3. Test on hardware before submitting PR
4. See [standards/GIT_WORKFLOW.md](standards/GIT_WORKFLOW.md) for branch naming conventions

## Further Documentation

| Document | Description |
|----------|-------------|
| [PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md) | Project vision and product tiers |
| [docs/HARDWARE.md](docs/HARDWARE.md) | Hardware specs, I2C addresses, GPIO pins |
| [docs/TOOLCHAIN_VALIDATION.md](docs/TOOLCHAIN_VALIDATION.md) | Detailed build/flash guide |
| [docs/SAD.md](docs/SAD.md) | Software Architecture Document |
| [CHANGELOG.md](CHANGELOG.md) | Project history and changes |

## License

RocketChip is licensed under the GNU General Public License v3.0. See [LICENSE](LICENSE) for details.
