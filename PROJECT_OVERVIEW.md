# RocketChip Project Overview

## Vision

RocketChip is the "Flipper Zero of motion tracking"—a portable, hackable, high-performance motion data platform. Originally designed for model rocketry telemetry, the same hardware and firmware serves RC aircraft, high-altitude balloons, vehicle dynamics, thrust vector control, and applications not yet imagined.

Open-source. Modular. Built to be extended.

## Product Tiers

| Tier | Name | Description |
|------|------|-------------|
| Base | **Core** | Bare-bones, lightweight board. Local data logging only. Minimal sensors, maximum hackability. |
| Main | *TBD* | Full sensor suite with local logging. Optional telemetry and peripherals via Booster Packs. |
| Advanced | **Titan** | High-performance sensors, pyro channel triggering, RTOS, TVC support. |

*Note: Nova reserved for future product (potentially space-rated hardware).*

The Core board uses a castellated design allowing it to be soldered directly onto carrier boards or used standalone.

## Booster Packs

Booster Packs are expansion modules for the main tier board (RocketChip's equivalent of FeatherWings).

Naming convention: rocket program names or terms strongly associated with spaceflight.

| Function | Description |
|----------|-------------|
| Telemetry | Radio module, antenna connector for live data transmission |
| Pyro/Servo | Pyro channels for deployment/staging, servo PWM for TVC |
| Navigation | GPS module, backup barometer for recovery tracking |
| Power | Solar charging, extended battery for long-duration missions |

## Target Applications

- **Model Rocketry** (primary) - Launch detection, flight logging, recovery tracking
- **RC Aircraft/Drones** - Flight data recording, blackbox logging
- **High-Altitude Balloons** - Long-duration tracking, telemetry, environmental sensing
- **Vehicle Dynamics** - Track day logging, G-force capture, lap timing
- **Education/STEM** - Hands-on aerospace concepts, data analysis projects

## Technical Foundation

- **Platform**: RP2350 (Adafruit Feather ecosystem)
- **RTOS**: FreeRTOS for real-time operations
- **Libraries**: ArduPilot module integration (AP_Math, filters, calibration) via compatibility shims
- **Architecture**: Mission Engine for event-condition-action workflows
- **Development**: PlatformIO + Pico SDK

## Terminology

| Term | Meaning |
|------|---------|
| **Mission** | A configuration defining sensors, states, events, actions, and logging for a specific use case. Not "Profile." |
| **Booster Pack** | Expansion module that adds functionality to the main board. |
| **Core** | The base-tier board, or the central processor board in a stacked configuration. |

## Related Documentation

- `docs/SAD.md` - Software Architecture Document (system design, modules, interfaces)
- `docs/SCAFFOLDING.md` - Directory structure and module overview
- `HARDWARE.md` - Current prototype hardware, pin assignments, sourcing
- `standards/CODING_STANDARDS.md` - Coding standards, protocols, safety requirements
- `standards/DEBUG_OUTPUT.md` - Debug output conventions

## Future Documentation

- `docs/ICD.md` - Interface Control Document (Booster Pack connector spec) - planned
- `docs/TEST_PLAN.md` - Test procedures and pass/fail criteria - planned

## Repository

https://github.com/jet-flyer/Rocket-Chip
