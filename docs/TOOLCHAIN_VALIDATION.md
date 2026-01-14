# RocketChip Toolchain Validation

## Overview

This document describes the toolchain validation for RocketChip on RP2350 hardware using FreeRTOS SMP.

**Status**: ✅ Fully validated on hardware
**Date**: 2026-01-14 (Updated)
**Toolchain**: Pure CMake + Pico SDK 2.1.0 + FreeRTOS SMP

## Background

### Why Pure CMake?

After evaluating PlatformIO for RP2350 support, we discovered that:
- RP2350 support in PlatformIO requires the Arduino-Pico framework
- Arduino framework adds unnecessary abstraction layers for real-time systems
- Pure CMake with Pico SDK provides direct hardware access and better control

The council decision (see CROSS_AGENT_REVIEW.md) confirmed this approach aligns with RocketChip's real-time requirements.

## Toolchain Components

| Component | Version | Purpose |
|-----------|---------|---------|
| Pico SDK | 2.1.0 | RP2350 hardware abstraction, USB, peripherals |
| FreeRTOS Kernel | SMP branch | Dual-core real-time scheduler |
| CMake | 3.13+ | Build system |
| ARM GCC | 10.3+ | Compiler toolchain (from Pico SDK) |

## Build Configuration

### Key CMake Settings

```cmake
PICO_BOARD = "pico2"
PICO_PLATFORM = "rp2350-arm-s"
FREERTOS_CONFIG_FILE_DIRECTORY = "${CMAKE_SOURCE_DIR}"
```

### Critical FreeRTOSConfig.h Settings

These settings are **required** for RP2350 Armv8-M architecture:

```c
#define configENABLE_MPU                   0  // MPU not used
#define configENABLE_TRUSTZONE             0  // No TrustZone
#define configRUN_FREERTOS_SECURE_ONLY     1  // Non-secure mode only
#define configENABLE_FPU                   1  // Enable FPU
#define configNUMBER_OF_CORES              2  // Dual-core SMP
#define configUSE_CORE_AFFINITY            1  // Pin tasks to cores
#define configUSE_16_BIT_TICKS             0  // Must be 0 for SMP
#define configTOTAL_HEAP_SIZE              (64 * 1024)
#define configTICK_RATE_HZ                 1000
```

**Warning**: Incorrect Armv8-M settings will cause hard faults or scheduler failures!

## Validation Test Architecture

### Task Design

| Task | Core | Priority | Rate | Purpose |
|------|------|----------|------|---------|
| Sensor | 1 | 5 (high) | 1kHz | Simulate sensor sampling |
| Logger | 1 | 3 (med) | Event-driven | Process sensor data from queue |
| UI | 0 | 1 (low) | 2Hz | LED blink + status reporting |

### Inter-Task Communication

- **Queue**: 32-element queue from Sensor → Logger
- **Shared state**: `system_status_t` structure for monitoring

### Core Affinity Strategy

- **Core 0**: UI task (low priority, handles USB serial + LED)
- **Core 1**: Sensor + Logger (high-priority real-time processing)

This design prevents USB serial latency from interfering with sensor sampling.

## Building

### Prerequisites

Install ARM GCC toolchain:

**Ubuntu/Debian:**
```bash
sudo apt install gcc-arm-none-eabi cmake build-essential
```

**macOS:**
```bash
brew install --cask gcc-arm-embedded
```

**Windows:**
Download from [ARM Developer](https://developer.arm.com/downloads/-/gnu-rm)

### Build Commands

Use the provided build script:
```bash
./build.sh
```

Or manually:
```bash
# Configure (specify local pico-sdk path)
mkdir build && cd build
cmake -DPICO_SDK_PATH=../pico-sdk -DPICO_NO_PICOTOOL=1 ..

# Build
make -j4

# Flash (copy UF2 to BOOTSEL drive)
cp freertos_validation.uf2 /media/RPI-RP2/
```

### Build Warnings (Expected)

```
Warning: TinyUSB submodule not initialized
```

This is **harmless** - the pico-sdk includes TinyUSB by default, but the submodule initialization check warns anyway. USB will work correctly.

## Expected Behavior on Hardware

### Normal Operation

When functioning correctly, you should observe:

1. **LED Activity** (starts immediately)
   - Blinks at 2Hz (500ms on, 500ms off)
   - Steady, consistent rhythm

2. **USB Serial Output** (after ~2s enumeration delay)
   - Connect at 115200 baud
   - Status reports every 5 seconds:

   ```
   === System Status ===
   Sensor samples:   5234
   Logger processed: 5234
   Queue depth:      0 / 32
   Free heap:        60416 bytes
   ====================
   ```

3. **Key Metrics**
   - Sensor samples increment ~1000/sec
   - Logger processed matches sensor count (±few samples)
   - Queue depth near 0 (logger keeping up)
   - Free heap stable around 60KB

### Startup Sequence

```
Time | Event
-----|---------------------------------------
0s   | Power on, LED off
0s   | USB enumeration begins
2s   | Startup banner printed to serial
2s   | Tasks created and scheduler starts
2s   | LED begins blinking at 2Hz
7s   | First status report
12s  | Second status report (sensors ~5000)
```

## Failure Modes & Debugging

### Hardware Issues

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| No LED activity | Wrong LED pin for board | Check `PICO_DEFAULT_LED_PIN` for your board |
| | USB power insufficient | Use powered hub or external 5V supply |
| | Board not flashed | Verify UF2 copied to BOOTSEL drive |

### Scheduler Failures

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| LED stuck on, no serial | Crashed before scheduler | Check FreeRTOSConfig.h Armv8-M settings |
| "Scheduler returned" | Heap exhausted at startup | Increase `configTOTAL_HEAP_SIZE` |
| | Static memory allocation failed | Check hooks.c memory allocations |

### Runtime Errors

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| Rapid LED flash (fast, ~5Hz) | `vApplicationMallocFailedHook` | Heap exhausted - reduce allocations |
| Slow LED flash (1Hz) | `vApplicationStackOverflowHook` | Increase task stack sizes |
| Sensor count not incrementing | Core 1 not running | Check `vTaskCoreAffinitySet()` calls |
| | Sensor task crashed | Add debug prints to sensor task |
| Queue depth growing | Logger starved by higher priority | Check task priorities |
| | Priority inversion | Review queue access patterns |
| | Core 1 scheduler issue | Verify SMP configuration |

### USB Serial Issues

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| No serial output | Wrong baud rate | Use 115200 baud |
| | Connected before enumeration | Wait 3-5 seconds after power-on |
| | USB-C cable data lines not connected | Try different cable |
| Serial garbled | Clock configuration mismatch | Check `configCPU_CLOCK_HZ` |
| Serial drops characters | USB task priority too low | Already on Core 0 - acceptable |

## Hardware-Specific Notes

### Raspberry Pi Pico 2

- **Board**: `PICO_BOARD = "pico2"`
- **LED Pin**: GPIO 25 (PICO_DEFAULT_LED_PIN)
- **USB**: Micro-USB connector

### Adafruit Feather RP2350

- **Board**: `PICO_BOARD = "adafruit_feather_rp2350"`
- **LED Pin**: GPIO 7 (PICO_DEFAULT_LED_PIN)
- **USB**: USB-C connector
- **I2C**: STEMMA QT on i2c1 (GPIO 2/3)

## Validation Complete

The following have been validated on Adafruit Feather RP2350:

- ✅ FreeRTOS SMP dual-core scheduler running
- ✅ I2C sensor communication (ISM330DHCX, LIS3MDL, DPS310)
- ✅ HAL layer (Bus, GPIO, ADC, PWM, Timing, PIO, UART)
- ✅ USB CDC serial output
- ✅ Hardware debugging (Debug Probe + OpenOCD + GDB)

## Future Work

- SPI flash logging
- DMA transfer integration
- Performance profiling (jitter measurement)

## Known Limitations

1. **Passive Idle Hook Not Used**
   - `configUSE_PASSIVE_IDLE_HOOK = 0`
   - Core 1 uses default idle behavior
   - Could enable for power profiling

2. **No Runtime Stats**
   - `configGENERATE_RUN_TIME_STATS = 0`
   - Can enable for CPU usage tracking (needs timer)

3. **Picotool Disabled**
   - Build uses `-DPICO_NO_PICOTOOL=1`
   - Avoids CMake fetching picotool source
   - UF2 creation still works

## References

- [Pico SDK Documentation](https://www.raspberrypi.com/documentation/pico-sdk/)
- [FreeRTOS SMP Documentation](https://www.freertos.org/symmetric-multiprocessing-introduction.html)
- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
- [CROSS_AGENT_REVIEW.md](../CROSS_AGENT_REVIEW.md) - Council decision on pure CMake approach
