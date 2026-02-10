# ESP32-S3 Pivot Analysis

**Date:** 2026-02-10
**Status:** Evaluation — no decision made
**Candidate Board:** Adafruit ESP32-S3 Reverse TFT Feather (#5691)
**Current Platform:** Adafruit Feather RP2350 HSTX (#6130)

**Codebase snapshot at time of analysis:**
- `main` branch, commit `0d69625`
- IVP progress: Stages 1-4 + Phase M complete, Stage 5 not started
- Source: ~5,034 lines across 10 `.cpp` files
- Binary: 155,648 bytes

---

## 1. Motivation

The RP2350 is ~1 year old silicon with platform-level quirks that have consumed significant debugging time:

| Issue | Time Lost | Status |
|-------|-----------|--------|
| I2C bus corruption from picotool rapid flash cycles | ~1.5 hours | Workaround (use debug probe) |
| BSS layout / codegen sensitivity causing I2C degradation | ~4+ hours | Unsolved, deferred |
| Non-blocking USB init causing 40-90s delayed I2C failures | ~3+ hours | Reverted (4 variants tried) |
| I2C bus recovery `i2c_deinit()` ordering | ~2 hours | Fixed (LL Entry 28) |
| picotool vs debug probe confusion across sessions | Recurring | Documented |

These are all **platform-layer** issues. The application code (sensors, calibration, CLI, math) is stable and validated (384K+ IMU reads, 0 errors).

The ESP32-S3 is a mature platform (shipping since 2021) with a large user base and production-proven ESP-IDF framework. FreeRTOS is native, not bolted on.

---

## 2. Hardware Comparison

| Spec | ESP32-S3 (#5691) | RP2350 HSTX (#6130) | Impact |
|------|-------------------|---------------------|--------|
| CPU | Dual Xtensa LX7 @ 240MHz | Dual Cortex-M33 @ 150MHz | +60% clock speed |
| FPU | Single-precision | Single-precision | Same constraint for ESKF |
| SRAM | 512KB | 520KB | Equivalent |
| PSRAM | **2MB** | **8MB** | **-75%** — tighter pre-launch buffer |
| Flash | **4MB** | **8MB** | **-50%** — tighter for flight logs |
| WiFi | 802.11 b/g/n | None | Free telemetry for ground testing |
| BLE | 5.0 | None | Config/download without cable |
| Display | Built-in 240x135 TFT | None | Status without serial terminal |
| STEMMA QT | Yes | Yes | Same sensor chain |
| Form Factor | Feather | Feather | Same expansion ecosystem |
| USB | USB-C (native USB) | USB-C (native USB) | Equivalent |
| PIO | None | 3 blocks x 4 SM | Lost (WS2812 → RMT) |
| HSTX | None | Yes | Lost (unused currently) |
| Debug | JTAG (built-in USB) | SWD (external probe) | Simpler debug setup |

### Critical Constraints

- **2MB PSRAM:** Pre-launch ring buffer shrinks from 512KB budget to ~256KB (~2.5s at full rate vs ~5s). Adequate for most rocket flights but tighter.
- **4MB Flash:** Firmware (~155KB currently) + NVS + flight logs. LittleFS partition ~3.5MB. Sufficient for Core/Main tiers.
- **No PIO:** WS2812 driver must use ESP32 RMT peripheral. RMT is purpose-built for this — proven in Adafruit NeoPixel ESP32 library.

---

## 3. Framework Comparison

| Aspect | RP2350 (Pico SDK) | ESP32-S3 (ESP-IDF) |
|--------|-------------------|---------------------|
| RTOS | None (bare-metal) | FreeRTOS (native, mandatory) |
| Dual-core | Manual (`multicore_launch_core1`) | `xTaskCreatePinnedToCore()` |
| Cross-core sync | Custom seqlock + atomics + DMB | FreeRTOS queues, stream buffers, semaphores |
| I2C driver | Thin wrapper over DW_apb_i2c | Full driver with bus recovery built in |
| Flash storage | Custom dual-sector + `flash_safe_execute()` | NVS (wear-leveled, key-value, built-in) |
| Watchdog | Hardware WDT, manual kick | `esp_task_wdt` (per-task, automatic) |
| USB serial | CDC via TinyUSB | CDC via TinyUSB (or hardware UART) |
| WiFi/BLE | N/A | Native stack on Core 0 |
| OTA updates | N/A | Built-in ESP-IDF OTA API |
| Stack protection | Custom MPU guard regions | FreeRTOS stack overflow detection |
| LED driver | Custom PIO program | RMT peripheral or library |
| Community | Growing (RP2350 is new) | Massive (ESP32 since 2016) |

### ESP-IDF Advantages for This Project

1. **`i2c_master_clear_bus()`** — built-in bus recovery, no custom 9-clock hack
2. **NVS** — replaces ~260 lines of custom flash storage code
3. **FreeRTOS task pinning** — dual-core is a first-class feature, not manual coordination
4. **WiFi telemetry** — MAVLink over WiFi for ground testing without LoRa hardware
5. **Built-in TFT** — status display without serial terminal connection
6. **OTA** — firmware updates over WiFi (stretch goal becomes trivial)

---

## 4. Framework Support Assessment

| Framework | ESP32-S3 Support | RP2350 Support | Notes |
|-----------|-----------------|----------------|-------|
| **ESP-IDF + FreeRTOS** | Native, production-ready | N/A | The only viable path on ESP32 |
| **ArduPilot** | Experimental (S3 in progress) | No HAL | No OEM has shipped ESP32 AP board |
| **F Prime (NASA)** | No | Yes (Pico 2 via Zephyr) | ESP32 memory constraints unresolved |
| **cFS (NASA)** | No | No | Requires Linux/RTEMS/VxWorks |

**Decision: ESP-IDF is the only production-ready option for ESP32-S3.**

---

## 5. Codebase Portability Assessment

### File-by-File Analysis

| File | Lines | SDK Coupling | Port Effort | What Changes |
|------|-------|-------------|-------------|-------------|
| `main.cpp` | 1,073 | 95% | **Rewrite** | Dual-core arch, MPU, watchdog, seqlock, fault handlers |
| `ws2812_status.cpp` | 356 | 80% | **Rewrite** | PIO → RMT peripheral |
| `calibration_storage.cpp` | 262 | 60% | **Moderate** | Custom flash → NVS |
| `i2c_bus.cpp` | 262 | 50% | **Moderate** | Pico I2C API → ESP-IDF I2C API |
| `rc_os.cpp` | 774 | 30% | **Low** | USB CDC → UART, timing functions |
| `icm20948.cpp` | 756 | 15% | **Low** | Timing functions only |
| `baro_dps310.cpp` | 156 | 15% | **Low** | Timing functions only |
| `gps_pa1010d.cpp` | 312 | 10% | **Low** | Timing functions only |
| `calibration_manager.cpp` | 673 | 5% | **Trivial** | Replace `pico_rand()` → `esp_random()` |
| `calibration_data.cpp` | 235 | 0% | **None** | Copy as-is |

**Summary:** ~38% rewrite, ~32% minor changes, ~30% copy verbatim.

### What Copies Directly (Reference Code)

- Calibration math (Levenberg-Marquardt, 6-position accel, ellipsoid mag fit)
- ICM-20948 register sequences and conversion formulas
- DPS310 coefficient math (ruuvi library — portable C)
- GPS NMEA parsing (lwGPS library — portable C)
- CLI command structure and menu logic
- All config constants (I2C addresses, sensor thresholds, timing values)
- All data structures (`SensorData`, `FusedState`, `MissionState`, calibration structs)

### What Gets Written Fresh (ESP-IDF Idiomatic)

- `app_main()` + FreeRTOS task architecture
- I2C driver wrapper (~100 lines using ESP-IDF API)
- NeoPixel via RMT
- NVS storage (simpler than current flash code)
- Cross-task communication (queues/stream buffers)
- Watchdog setup (`esp_task_wdt`)
- WiFi/BLE telemetry (bonus — new capability)
- TFT display driver (bonus — new capability)

---

## 6. Architecture Document Applicability

| Document | Reuse | Adapt | Rewrite | Notes |
|----------|-------|-------|---------|-------|
| **SAD.md** | ~50% | ~25% | ~25% | App architecture transfers. Platform layer, inter-core comms, memory layout rewrite |
| **IVP.md** | ~55% | ~25% | ~20% | Gate criteria are portable. Stage 3 (dual-core) is a full rewrite. Stages 5-6 (fusion, mission) ~90% reusable |
| **SCAFFOLDING.md** | ~85% | ~10% | ~5% | Directory structure mostly keeps |
| **CODING_STANDARDS.md** | ~80% | ~15% | ~5% | JSF AV rules are platform-independent. Platform constraints section rewrites |
| **LESSONS_LEARNED.md** | ~40% | ~30% | ~30% | Sensor/calibration lessons transfer. RP2350-specific entries become historical |
| **HARDWARE.md** | ~60% | ~30% | ~10% | Sensors, I2C map, Booster Packs all keep. Pin assignments, MCU specs, PIO section rewrite |

---

## 7. Recommended Approach: Ground-Up with Reference

A fresh ESP-IDF project using the current codebase as reference is cleaner than a line-by-line port. The sensor math and algorithms copy directly; the architecture gets written in ESP-IDF idiom from the start.

### Build Order Estimate

| Phase | Duration | Work |
|-------|----------|------|
| **1. Platform bringup** | Days 1-2 | ESP-IDF project, I2C scan, serial output, NeoPixel via RMT |
| **2. Sensor drivers** | Days 3-5 | IMU + baro (copy register sequences, new I2C wrapper) |
| **3. Dual-core + CLI** | Week 2 | FreeRTOS sensor task on Core 1, CLI on UART, NVS storage |
| **4. Calibration** | Week 2-3 | Copy math wholesale, adapt storage to NVS |
| **5. GPS + fusion** | Week 3 | GPS driver, begin ESKF (math library copies directly) |
| **6. WiFi telemetry** | Week 3-4 | Bonus capability — MAVLink over WiFi |

**Total: 2-3 weeks** for equivalent functionality to current RP2350 baseline (Stages 1-4 + Phase M).

### Dual-Core Mapping

```
ESP32-S3 Core 0 — "Protocol CPU"
  WiFi/BLE stack (ESP-IDF managed)
  CLI / UART task
  Telemetry task
  Fusion task (ESKF, 200Hz)
  Mission engine task

ESP32-S3 Core 1 — "Application CPU"
  Sensor sampling task (pinned, highest priority)
  IMU @ 1kHz, Baro @ 50Hz, GPS @ 10Hz
```

This mirrors the current RP2350 split (Core 0 = USB/CLI, Core 1 = sensors) with WiFi added to Core 0.

---

## 8. Risks and Mitigations

| Risk | Severity | Mitigation |
|------|----------|------------|
| 2MB PSRAM limits pre-launch buffer | Medium | Reduce buffer to ~256KB (~2.5s). Adequate for model rockets. SD card for extended logging |
| 4MB flash limits flight log storage | Medium | NVS is more efficient than custom flash. SD card FeatherWing for bulk logs |
| 1kHz IMU jitter from FreeRTOS scheduler | Low | Pin task to Core 1, max priority, `vTaskDelayUntil()`. Measure and compare |
| WiFi stack disrupts Core 0 timing | Low | Fusion runs on Core 0 at 200Hz — WiFi IRQs are brief. Monitor and adjust if needed |
| New platform, new bugs | Medium | ESP-IDF is battle-tested. But every platform has quirks. Budget time for unknowns |
| Loss of momentum / sunk cost | Medium | Current RP2350 code is reference, not wasted. Algorithms and architecture transfer |

---

## 9. Decision Criteria

Factors favoring the pivot:
- Platform maturity (ESP32-S3 vs RP2350 silicon age)
- Built-in WiFi/BLE (eliminates LoRa for ground testing)
- Built-in display
- Native FreeRTOS (vs bare-metal multicore wrangling)
- Built-in I2C bus recovery
- OTA update capability
- Larger community and library ecosystem

Factors favoring staying on RP2350:
- Current code is stable and validated (384K reads, 0 errors)
- 8MB PSRAM / 8MB flash (4x / 2x more)
- PIO for custom protocols
- HSTX for future video output
- No porting effort — continue to Stage 5 immediately
- F Prime (NASA) supports Pico 2 via Zephyr

**No decision has been made.** This document captures the analysis for future reference.

---

*Document maintained in: `docs/hardware/ESP32_S3_PIVOT_ANALYSIS.md`*
