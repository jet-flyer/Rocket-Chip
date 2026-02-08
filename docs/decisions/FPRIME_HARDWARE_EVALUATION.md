# F' Hardware Evaluation: Boards On Hand

**Date:** 2026-02-08
**Status:** Research — no implementation started
**Related:** [TITAN_BOARD_ANALYSIS.md](TITAN_BOARD_ANALYSIS.md) (F' evaluation addendum)

---

## Purpose

Evaluate the practical path to running F' (F Prime) on hardware we already own, and identify what work is needed for each board.

---

## Boards Available

| Board | Chip | F' Status | Form Factor | Weight |
|-------|------|-----------|-------------|--------|
| **Adafruit Feather RP2350 HSTX** | RP2350 (2x M33 @ 150MHz) | fprime-arduino LedBlinker tested (v3.5.0) | 50.8 x 22.8mm | ~5g |
| **Matek H743-SLIM V3** | STM32H743VIH6 (M7 @ 480MHz) | Not tested — needs Zephyr board def | 36 x 36mm | 7g |
| **BeagleBone Black** | TI AM3358 (A8 @ 1GHz) | Officially supported (VxWorks 7), community-proven (Linux) | 86.4 x 54.6mm | ~40g |

---

## 1. Matek H743-SLIM V3

### What It Has

- STM32H743VIH6: Cortex-M7 @ 480MHz, 1MB SRAM, 2MB flash
- Dual IMU: ICM42688P (SPI1) + ICM42605 (SPI4)
- Barometer: DPS310 on I2C2
- 7x UARTs, 4x SPI, 2x I2C, 1x FDCAN, SD card (SDIO), USB-C
- SWD pads (PA13/PA14) for debug probe
- Full ArduPilot support (hwdef exists, validated across board revisions)
- **Note:** EOL on Matek's website. Board is in hand, no need to buy.

### What It Needs for F'

**Step 1: Zephyr Board Definition** (main work item)

No Zephyr board def exists for the Matek H743-SLIM. The NUCLEO-H743ZI def exists for the same SoC and can serve as the starting point.

Work required:
- Copy `nucleo_h743zi` board directory as skeleton
- Remap all pin assignments using ArduPilot's `hwdef.dat` as the authoritative reference (hardware-validated across multiple board revisions)
- Remove Ethernet, Arduino header DTSI, and user button nodes
- Add SPI4 (IMU2), I2C2 (DPS310 baro), SDMMC1 (SD card)
- Change HSE config (standalone crystal, not ST-Link bypass)
- Remap LEDs to PE3 (blue) and PE4 (green)
- Set `board.cmake` to use CMSIS-DAP or J-Link runner (no on-board ST-Link)
- Test with `samples/basic/blinky` targeting PE3

**Step 2: F' Deployment**

The `sobkulir/fprime-zephyr-app` project demonstrated F' on NUCLEO-H723ZG (same H7 family). Retargeting to H743:
- Change build command to `-DBOARD=matek_h743_slim`
- Create DTS overlay mapping F' telemetry to UART7 (PE7/PE8, has CTS/RTS)
- Route debug console to USB CDC (no ST-Link VCP on Matek)
- F' requires two serial connections: debug console + telemetry (binary packets to fprime-gds)

**Step 3: Sensor Integration**

Once F' runs:
- Write F' Passive Components wrapping the onboard ICM42688P and DPS310
- These are the same sensor families as RocketChip's existing drivers (different IMU model, same DPS310 baro)
- fprime-zephyr provides the Zephyr I2C/SPI abstractions

### Effort Estimate

| Task | Effort | Dependency |
|------|--------|------------|
| Zephyr board def | 2-3 days | None |
| F' LedBlinker on Matek | 1-2 days | Board def |
| F' with telemetry + GDS | 1-2 days | LedBlinker working |
| Sensor components | 3-5 days | Telemetry working |
| **Total** | **~1-2 weeks** | |

### Limitations

- **Single-core only.** Zephyr SMP for Cortex-M is blocked (issue #59826). The H743 is single-core anyway (no M4 co-processor like the H755).
- **fprime-zephyr is experimental.** ~10 commits in the reference repo. Known build system workarounds (Stub.cpp linking hack).
- **No F' Active Components recommended on Zephyr MCU targets.** Limited to Passive Components with rate group scheduling.

### Why It's Interesting Anyway

- Full ArduPilot flight controller with validated sensor hardware — dual IMU + baro onboard means no external wiring
- 480MHz M7 with 1MB RAM and double-precision FPU — significantly more capable than RP2350 for single-core compute
- SD card for flight logging
- FDCAN for future inter-board communication
- If F' on Zephyr matures, this board becomes a compelling standalone Titan candidate
- The ArduPilot hwdef provides a verified pin map, eliminating hardware bring-up guesswork

---

## 2. BeagleBone Black

### What It Has

- TI Sitara AM3358: Cortex-A8 @ 1GHz, 512MB DDR3L, 4GB eMMC
- Full Linux (Debian), also supports VxWorks 7
- Ethernet, USB host/client, HDMI, 2x 46-pin expansion headers (GPIO, I2C, SPI, UART, ADC, PWM)
- No onboard flight sensors — all external via expansion headers or capes

### F' Status

**Officially supported** on the F' supported platforms page (via VxWorks 7). Community-confirmed working on Linux (Debian) with full feature set — commands, telemetry, events, parameters, file uplink/downlink, TCP GDS connectivity.

Cross-compilation uses `fprime-arm-linux` toolchain. Multiple university groups have deployed F' on BBB successfully.

### What It Needs for F'

**Almost nothing.** F' on Linux on BBB is a known-working configuration:
1. Cross-compile F' with ARM Linux toolchain
2. Deploy to BBB over SSH/SCP
3. Connect GDS via Ethernet (TCP) or UART
4. Full Active Components, threading, all F' features available

Sensor hardware would need to be added externally (I2C/SPI breakouts on the expansion headers).

### Limitations

- **Large and heavy.** 86.4 x 54.6mm, ~40g — roughly 6x the weight of the Matek and 3x the footprint. Not practical as a flight computer for model rockets.
- **No onboard sensors.** Requires external breakouts for IMU, baro, GPS.
- **No WiFi.** Ethernet only (unless USB WiFi dongle added).
- **Single-core.** The Cortex-A8 is single-core, so no multicore advantage over the Pi Zero 2 W (which has 4x A53 cores + WiFi for less weight).
- **Power.** ~1.75W typical — less than Pi Zero 2 W but still significant for battery-powered flight.
- **Older platform.** The AM3358 is a 2012-era SoC. Still capable, but the Pi Zero 2 W offers more performance, WiFi, and less weight.

### Best Use Case

**Desktop F' development and testing station.** The BBB is excellent for:
- Learning F' framework development without cross-compilation hassles (native ARM Linux)
- Testing F' deployments before porting to flight hardware
- Running fprime-gds locally with Ethernet connectivity
- Validating component logic before deploying to constrained MCU targets

It is **not a flight hardware candidate** due to size and weight, but it's a useful development tool that's already in hand.

---

## 3. Comparison Summary

| Criterion | RP2350 Feather | Matek H743-SLIM | BeagleBone Black |
|-----------|---------------|-----------------|------------------|
| **F' readiness** | LedBlinker tested | Needs Zephyr board def | Ready (Linux) |
| **Work to first F' blink** | ~1 hour | ~1-2 weeks | ~1 day |
| **Full F' features** | No (baremetal, passive only) | No (Zephyr, passive only) | **Yes** (Linux, full threading) |
| **Multicore F'** | No | No | No (single-core A8) |
| **Onboard sensors** | None (external Qwiic) | Dual IMU + baro | None |
| **Flight practical** | **Yes** (5g, 51x23mm) | **Yes** (7g, 36x36mm) | No (40g, 86x55mm) |
| **Debug** | SWD (probe) | SWD (pads) | SSH/GDB (Linux) |
| **ArduPilot support** | No (RP2350 not supported) | **Full** | No |
| **Best role** | Core/Main tier (bare-metal) | Titan candidate (if F'/Zephyr matures) | F' dev/test station |

---

## Recommended Approach

1. **Use BBB now** as an F' learning and prototyping platform — get familiar with FPP, component development, GDS, and the F' build system without fighting MCU constraints
2. **Create Matek Zephyr board def** when ready to explore F' on MCU — the ArduPilot hwdef eliminates guesswork, and the onboard sensors mean immediate hardware integration
3. **Keep RP2350 Feather on bare-metal Pico SDK** for Core/Main tier — F' on RP2350 is too limited (no Active Components, single-core) to justify the framework overhead
4. **Evaluate Pi Zero 2 W** (~$15, order separately) for the hybrid architecture if full F' on a flight-weight Linux SBC is the goal

---

*This document supplements the F' evaluation in [TITAN_BOARD_ANALYSIS.md](TITAN_BOARD_ANALYSIS.md). See `COUNCIL_PROCESS.md` for review protocol.*
