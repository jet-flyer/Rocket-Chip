# RocketChip Project Overview

## Vision

RocketChip is the "Flipper Zero of motion tracking"—a portable, hackable, high-performance motion data platform. Originally designed for model rocketry telemetry, the same hardware and firmware serves RC aircraft, high-altitude balloons, vehicle dynamics, thrust vector control, and applications not yet imagined.

Open-source. Modular. Built to be extended.

## Product Tiers

| Tier | Name | Description |
|------|------|-------------|
| Base | **Core** | Bare-bones, lightweight board. Local data logging only. Minimal sensors, maximum hackability. |
| Main | *TBD* | Full sensor suite with local logging. Optional telemetry and peripherals via Booster Packs. |
| Advanced | **Titan** | High-performance sensors, pyro channel triggering, deterministic control loops, TVC support. |

*Note: Nova reserved for future product (potentially space-rated hardware).*

The Core board uses a castellated design allowing it to be soldered directly onto carrier boards or used standalone.

### Gemini Carrier Board (Stretch Goal)

**Gemini** is a **separate carrier board PCB** that mounts two identical Core modules to create a fault-tolerant redundant flight computer. Named for "twins" (Latin) and NASA heritage, Gemini is independent of the tier system—it's specifically for pairing Core modules.

Unlike Booster Packs (which stack on a single Core/Main board), Gemini is its own board with Core modules soldered or socketed onto it. Standard FeatherWing compatibility does not apply to Gemini—it repurposes GPIO for inter-MCU communication.

> **Roadmap:** Gemini is a **post-crowdfunding stretch goal**. Nearer-term stretch goals include Betaflight port and additional Booster Packs. Gemini development begins after successful delivery of Core/Main tiers.

| Feature | Description |
|---------|-------------|
| Redundancy | Dual RP2350 MCUs with automatic failover |
| Communication | SpaceWire-Lite inter-MCU link |
| Pyro Safety | Hardware voting logic (AND to ARM, OR to FIRE) |
| Power | Independent regulation per module with isolation |

See `docs/hardware/GEMINI_CARRIER_BOARD.md` for detailed design documentation.

## Booster Packs

Booster Packs are expansion modules for the main tier board (RocketChip's equivalent of FeatherWings).

Naming convention: rocket program names or terms strongly associated with spaceflight.

| Function | Description |
|----------|-------------|
| Telemetry | Radio module, antenna connector for live data transmission |
| Pyro/Servo | Pyro channels for deployment/staging, servo PWM for TVC |
| Navigation | GPS module, backup barometer for recovery tracking |
| Power | Solar charging, extended battery for long-duration missions |
| RC Link + Video (Telstar) | ELRS receiver for RC control + MAVLink, 5.8 GHz FPV video transmitter, RID module support. Also usable as standalone FPV comms board. |

## Target Applications

- **Model Rocketry** (primary) - Launch detection, flight logging, recovery tracking
- **RC Aircraft/Drones** - Flight data recording, blackbox logging
- **High-Altitude Balloons** - Long-duration tracking, telemetry, environmental sensing
- **Vehicle Dynamics** - Track day logging, G-force capture, lap timing
- **Education/STEM** - Hands-on aerospace concepts, data analysis projects

## Technical Foundation

- **Platform**: RP2350 (Adafruit Feather ecosystem)
- **Feather Compatibility**: Core/Main boards maintain standard Feather pinout for 3rd-party FeatherWing compatibility
- **Architecture**: Bare-metal Pico SDK with polling main loop
- **Sensor Fusion**: Custom ESKF + MMAE architecture (see docs/ESKF/)
- **Architecture**: Flight Director for event-condition-action workflows
- **Development**: CMake + Pico SDK
- **Communication**: SpaceWire-Lite for Gemini inter-MCU (aspirational standard, design-for-certifiability)
- **Configuration**: RC_OS serial terminal interface for setup/calibration without PC software

## Terminology

| Term | Meaning |
|------|---------|
| **Mission** | A configuration defining sensors, states, events, actions, and logging for a specific use case. Not "Profile." |
| **Booster Pack** | Expansion module that adds functionality to the main board. |
| **Core** | The base-tier board, or the central processor board in a stacked configuration. |

## Related Documentation

- `docs/SAD.md` - Software Architecture Document (system design, modules, interfaces)
- `docs/SCAFFOLDING.md` - Directory structure and module overview
- `docs/hardware/HARDWARE.md` - Current prototype hardware, pin assignments, sourcing
- `docs/hardware/GEMINI_CARRIER_BOARD.md` - Gemini redundant flight computer design
- `docs/hardware/TELSTAR_BOOSTER_PACK.md` - Telstar ELRS RC link + FPV video Booster Pack
- `docs/icd/EXPANSION_CONNECTOR_ICD.md` - Feather-based expansion connector specification
- `docs/icd/GEMINI_PROTOCOL_ICD.md` - Gemini inter-MCU protocol specification
- `standards/CODING_STANDARDS.md` - Coding standards, protocols, safety requirements
- `standards/DEBUG_OUTPUT.md` - Debug output conventions
- `standards/GIT_WORKFLOW.md` - Git workflow and branch management practices
- `standards/protocols/SPACEWIRE_LITE.md` - SpaceWire-Lite communication protocol (aspirational)

## Future Documentation

- `docs/TEST_PLAN.md` - Test procedures and pass/fail criteria - planned

## Repository

https://github.com/jet-flyer/Rocket-Chip
