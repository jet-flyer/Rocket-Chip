# RocketChip Project Status

**Last Updated:** 2026-01-14 by Claude Code CLI

## Current Phase
Phase 1 (Foundation) - HAL implemented, validation tests passing.

## Completed
- [x] Software Architecture Document (SAD) created - `docs/SAD.md`
- [x] Directory structure scaffolding defined - `docs/SCAFFOLDING.md`
- [x] Header stubs for core modules (HAL, services, Mission Engine)
- [x] PlatformIO configuration (Core/Main/Titan environments)
- [x] ArduPilot compatibility shim structure
- [x] Product naming finalized: Core, Main+Packs, Titan (Nova reserved)
- [x] Logging format decision: MAVLink binary default, CSV/MATLAB export
- [x] Toolchain validation: Pure CMake + Pico SDK + FreeRTOS SMP (PlatformIO requires Arduino framework for RP2350)
- [x] GETTING_STARTED.md created
- [x] Board configuration corrected for Adafruit Feather RP2350 (was configured for standard Pico2)
- [x] Simple LED blink test validated on hardware
- [x] Raspberry Pi Debug Probe configured with correct firmware (debugprobe.uf2) and OpenOCD
- [x] FreeRTOS SMP validation complete: dual-core scheduler running, tasks executing on both cores with proper priority and affinity, inter-task queues operational, USB serial output working
- [x] Hardware debugging workflow established: OpenOCD + GDB + VS Code integration functional
- [x] TinyUSB initialized and functional: stdio_init_all() properly initializes TinyUSB stack, USB-CDC serial working correctly
- [x] HAL implemented: Bus (I2C/SPI), GPIO, ADC, PWM, Timing, PIO (NeoPixel), UART - see `src/hal/README.md`
- [x] HAL validation smoke test (`smoke_hal_validation`) - all tests passing (HAL init, timing, GPIO, ADC, I2C sensors, NeoPixel)
- [x] I2C sensors verified: ISM330DHCX (0x6A), LIS3MDL (0x1C), DPS310 (0x77) all responding correctly
- [x] USB CDC serial output pattern documented in `standards/DEBUG_OUTPUT.md`

## Active Work
- [ ] Fix LED blinking in hal_validation smoke test (see Known Issues)
- [ ] Initialize TinyUSB git submodule in pico-sdk (eliminates harmless build warning)
- [ ] Sensor driver implementation (IMU, Baro) using HAL

## Blocked On
Nothing currently.

## Next Milestone
Phase 1 FreeRTOS validation: ✅ COMPLETE
- Project compiles, uploads, runs FreeRTOS SMP scheduler with multiple tasks
- Dual-core operation validated (Core 0: UI task, Core 1: Sensor + Logger tasks)
- Hardware debugging functional (Debug Probe + OpenOCD + GDB)

Next: Sensor integration (IMU, Barometer) and HAL driver implementation

## Architecture Documents
- `docs/SAD.md` - Software Architecture Document (modules, interfaces, task model)
- `docs/SCAFFOLDING.md` - Directory structure and implementation status

## Back Burner
- Middle tier product name decision
- Ground station software evaluation
- Telemetry protocol final selection (CRSF/CCSDS vs raw MAVLink)
- Evaluate compliant stand-in for MissionEngine condition evaluator (custom non-recursive parser per MISRA/JSF; skip if core requirements inherently non-compliant)

## Side Projects
- Pegasus flight controller (full FC for drones, glide-back boosters - informed by madflight FC3)
- Fruit Jam as potential ground station platform
- OpenMCT integration exploration

## Known Issues
- Legacy Arduino codebase archived in DEPRECATED/ (not carried forward)
- **hal_validation LED not blinking**: In `smoke_hal_validation`, the red LED (GPIO 13) does not blink in the main loop or while waiting for serial connection, despite `gpio_init()`, `gpio_set_dir()`, and `gpio_put()` being called correctly. NeoPixel works fine. The imu_qwiic_test uses identical LED code and works. Suspect interaction between HAL GPIO class usage during tests and subsequent raw gpio_put() calls, or a pin conflict. Serial output works correctly. Debug probe recommended to investigate.

## Future Documentation Planned
- `docs/ICD.md` - Interface Control Document (Booster Pack connector)
- `docs/TEST_PLAN.md` - Test procedures and criteria
