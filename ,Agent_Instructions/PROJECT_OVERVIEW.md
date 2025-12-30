# RocketChip Project Overview

## Vision

RocketChip is the "Flipper Zero of motion tracking"—a portable, hackable, high-performance motion data platform. Originally designed for model rocketry telemetry, the same hardware and firmware serves RC aircraft, high-altitude balloons, vehicle dynamics, thrust vector control, and applications not yet imagined.

Open-source. Modular. Built to be extended.

## Product Tiers

| Tier | Name | Description |
|------|------|-------------|
| Base | **Core** | Bare-bones, lightweight board. Local data logging only. Minimal sensors, maximum hackability. |
| Main | *TBD* | Full sensor suite with local logging. Optional telemetry and peripherals via Booster Packs. |
| Advanced | *Nova or Titan (TBD)* | High-performance sensors, pyro channel triggering, RTOS, TVC support. |

The Core board uses a castellated design allowing it to be soldered directly onto carrier boards or used standalone.

## Booster Packs

Booster Packs are expansion modules for the main tier board (RocketChip's equivalent of FeatherWings).

Naming is pivoting from Roman gods to rocket program names or terms strongly associated with spaceflight. Categories:

| Function | Description |
|----------|-------------|
| Telemetry | LoRa radio, antenna connector for live data transmission |
| Pyro/Servo | Pyro channels for deployment/staging, servo PWM for TVC |
| Navigation | GPS module, backup barometer for recovery tracking |
| Power | Solar charging, extended battery for long-duration missions |

Specific names TBD.

## Target Applications

- **Model Rocketry** (primary) - Launch detection, flight logging, recovery tracking
- **RC Aircraft/Drones** - Flight data recording, blackbox logging
- **High-Altitude Balloons** - Long-duration tracking, telemetry, environmental sensing
- **Vehicle Dynamics** - Track day logging, G-force capture, lap timing
- **Education/STEM** - Hands-on aerospace concepts, data analysis projects

## Technical Direction

- **Platform**: RP2350 (Adafruit Feather ecosystem)
- **Development**: Post-Arduino clean sheet refactor underway
- **Libraries**: ArduPilot module integration from the start (AP_Math, filters, calibration)
- **Architecture**: Mission Engine as separate module/library (see Mission Engine documentation)
- **RTOS**: FreeRTOS for Pro tier, evaluating for lower tiers

## Terminology

| Term | Meaning |
|------|---------|
| **Mission** | A configuration defining sensors, states, events, actions, and logging for a specific use case. Not "Profile." |
| **Booster Pack** | Expansion module that adds functionality to the main board. |
| **Core** | The base-tier board, or the central processor board in a stacked configuration. |

## Related Documentation

- `BOM.md` - Current prototype hardware and pin assignments
- `Mission Engine Architecture` - Detailed Mission Engine design (separate from agent instructions)

## Repository

https://github.com/jet-flyer/Rocket-Chip
