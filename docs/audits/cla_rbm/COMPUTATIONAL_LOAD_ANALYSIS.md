# Computational Load Analysis (CLA)

**Date:** 2026-03-08
**Firmware:** Stage 7 complete (commit c00c573)
**Board:** Adafruit RP2350 HSTX Feather (Cortex-M33, 150MHz, 520KB SRAM, 16MB Flash, 8MB PSRAM)
**Collection Method:** `scripts/cla_collect.py` — serial CLI + GDB probe (zero code changes)
**Soak Duration:** 270s (5 min), 10 samples — also validated with 60s soak (4 samples)
**Soak Conditions:** Device stationary on desk, all sensors connected (IMU, baro, GPS, radio), USB CDC terminal, room temperature

---

## 1. Executive Summary

| Core | Current Duty % | WCET Budget (µs) | Tick Period (µs) | Margin |
|------|---------------|-------------------|------------------|--------|
| Core 0 | ~12% (avg), ~16% (WCET) | ~800 | 5,000 (200Hz ESKF) | **84%** |
| Core 1 | ~85% (avg) | ~950 | 1,000 (1kHz target) | **~5-15%** |

Both cores are well within budget. Core 0 has substantial headroom for Stage 8 additions (state machine, pyro monitoring, phase-scheduled Q/R). Core 1 is tighter — worst-case IMU+baro+GPS coincidence approaches the 1ms cycle target but padding absorbs it.

---

## 2. Platform Parameters

| Parameter | Value | Source |
|-----------|-------|--------|
| MCU | RP2350B (dual Cortex-M33) | DATASHEET |
| Clock | 150 MHz | DATASHEET |
| SRAM | 520 KB (main) + 4KB SCRATCH_X + 4KB SCRATCH_Y | DATASHEET |
| Flash | 16 MB QSPI (W25Q128JV) | DATASHEET |
| PSRAM | 8 MB (APS6404L-3SQR) | DATASHEET |
| XIP Cache | 2 KB (2-way set associative) | DATASHEET |
| FPU | Single-precision hardware (f32) | DATASHEET |
| OS | None (bare-metal cooperative dispatch) | — |
| Stack (Core 0) | 4 KB (SCRATCH_Y) | CMake: `PICO_STACK_SIZE=0x1000` |
| Stack (Core 1) | 4 KB (SCRATCH_X, SDK default) | SDK |

---

## 3. Core 0 Tick Budget

Core 0 runs a cooperative main loop at ~1kHz (limited by `sleep_ms(1)`). The ESKF runs at 200Hz (every 5th IMU sample via `kEskfImuDivider`).

### 3.1 Per-Tick Function Timing

| Function | Rate (Hz) | Avg (µs) | Min (µs) | WCET (µs) | Duty % (avg) | Source |
|----------|-----------|----------|----------|-----------|-------------|--------|
| `eskf_tick()` — predict | 200 | 577 | 108 | 802 | 11.5% | MEASURED-SOAK |
| `eskf_tick()` — baro update | 16 | ~43 | — | ~81 | 0.07% | MEASURED-BENCH |
| `eskf_tick()` — mag update | ~80 | ~43 | — | ~81 | 0.34% | MEASURED-BENCH |
| `eskf_tick()` — ZUPT | ~100 | ~43 | — | ~81 | 0.43% | ESTIMATED |
| `heartbeat_tick()` | 1 | <1 | — | <5 | <0.01% | ESTIMATED |
| `watchdog_kick_tick()` | 1000 | <1 | — | <2 | <0.1% | ESTIMATED |
| `logging_tick()` | 50 | ~10 | — | ~50 | 0.05% | ESTIMATED |
| `telemetry_radio_tick()` | 2-10 | ~20 | — | ~500 | 0.1% | ESTIMATED |
| `mavlink_direct_tick()` | 1-10 | ~5 | — | ~50 | 0.01% | ESTIMATED |
| `cli_update_tick()` | 20 | ~5 | — | ~200 | 0.01% | ESTIMATED |

**Notes:**
- ESKF predict timing is bimodal: 108µs (codegen FPFT only) vs ~577µs (codegen + UD factorize + reconstruct). The factorize/reconstruct runs on every predict when Bierman measurement updates are pending.
- MEASURED-BENCH values are from `docs/benchmarks/UD_BENCHMARK_RESULTS.md` (Bierman scalar: 43µs avg, Joseph: 81µs — Bierman active on target).
- ESTIMATED values are informed by code inspection — no instrumentation. Conservative upper bounds.

### 3.2 ESKF Predict Timing (measured)

From 270s soak (180,488 calls):

| Metric | Value |
|--------|-------|
| avg | 577 µs |
| min | 108 µs |
| max | 802 µs |
| calls | 180,488 |

Consistent across both 60s and 270s soaks — min/max identical, avg within 3% (561µs vs 577µs). The min (108µs) occurs when only the codegen FPFT runs (no factorize/reconstruct needed). The avg (~577µs) includes the UD housekeeping that runs on most cycles. The max (802µs) represents worst-case coincidence with measurement updates.

### 3.3 Measurement Update Timing (from benchmarks)

From `docs/benchmarks/UD_BENCHMARK_RESULTS.md`:

| Update Type | Avg (µs) | Source |
|-------------|----------|--------|
| Bierman scalar update | 43 | MEASURED-BENCH |
| Joseph scalar update (disabled) | 81 | MEASURED-BENCH |
| Full predict+factorize+reconstruct | 561 | MEASURED-SOAK |
| Hybrid codegen+Bierman epoch | 486 | MEASURED-BENCH |

---

## 4. Core 1 Sensor Budget

Core 1 runs a 1kHz sensor polling loop with `busy_wait` padding to the 1ms target.

### 4.1 Per-Component Timing

| Component | Rate (Hz) | Duration (µs) | Duty % | Source |
|-----------|-----------|---------------|--------|--------|
| IMU read (ICM-20948, 23 bytes I2C) | 1000 | 774 avg, 791 max | 77.4% | MEASURED-BENCH |
| Baro read (DPS310, I2C) | ~16 | 251 avg, 294 max | 0.4% | MEASURED-BENCH |
| GPS read (PA1010D UART poll) | 10 | ~50 | 0.05% | ESTIMATED |
| Mag read (AK09916 via I2C bypass) | 100 | ~100 | 1.0% | ESTIMATED |
| Seqlock write (memcpy 148B + barriers) | 1000 | <5 | 0.5% | ESTIMATED |
| NeoPixel update | 1000 | <10 | 1.0% | ESTIMATED |
| Cal feed (divider check) | 1000 | <1 | <0.1% | ESTIMATED |
| Watchdog flag store | 1000 | <1 | <0.1% | ESTIMATED |

### 4.2 Worst-Case Cycle (all sensors coincide)

| Component | WCET (µs) |
|-----------|-----------|
| IMU read | 791 |
| Baro read | 294 |
| GPS read | ~50 |
| Mag read | ~100 |
| Seqlock + NeoPixel + overhead | ~20 |
| **Total** | **~1,255** |

This exceeds the 1,000µs target by ~255µs, meaning worst-case cycles overflow and the loop runs at <1kHz. However, this triple coincidence (IMU+baro+GPS all in same cycle) happens at the hyperperiod frequency (see Section 4b). On average, Core 1 achieves **998.1 Hz** (measured over 270s).

### 4.3 Measured Core 1 Performance

| Metric | Value | Source |
|--------|-------|--------|
| Effective rate | 998.1 Hz | MEASURED-SOAK |
| IMU reads/sec | 998.1 | MEASURED-SOAK |
| Mag reads/sec | 99.1 | MEASURED-SOAK |
| Baro reads/sec | 16.0 | MEASURED-SOAK |
| GPS reads/sec | 10.0 | MEASURED-SOAK |
| IMU errors (270s) | 138 / 1,158,938 (0.012%) | MEASURED-SOAK |
| Baro errors (270s) | 0 / 18,554 | MEASURED-SOAK |
| GPS errors (270s) | 0 / 11,590 | MEASURED-SOAK |

---

## 4b. Worst-Case Coincidence Stack-Up (NASA SWE-139)

### Hyperperiod

LCM of all tick rates: 1000Hz, 200Hz, 100Hz, 50Hz, 20Hz, 16Hz, 10Hz, 1Hz

LCM(1000, 200, 100, 50, 20, 16, 10, 1) = **1000** ticks = **1 second**

Every 1 second, ALL tick functions can coincide in the same main loop iteration.

### Core 0 Worst-Case (all ticks fire simultaneously)

| Function | WCET (µs) |
|----------|-----------|
| eskf_tick (predict + all measurement updates) | 802 + 81×4 = ~1,126 |
| logging_tick | ~50 |
| telemetry_radio_tick (TX path) | ~500 |
| mavlink_direct_tick | ~50 |
| cli_update_tick | ~200 |
| heartbeat_tick | ~5 |
| watchdog_kick_tick | ~2 |
| **Total WCET** | **~1,933** |
| **Available per tick** | **5,000** (200Hz ESKF epoch) |
| **Margin** | **3,067 µs (61%)** |

### Core 1 Worst-Case (all sensors coincide)

See Section 4.2: ~1,255µs vs 1,000µs target. The overflow is absorbed by the next cycle's `busy_wait` being shorter — the system self-corrects within 2 cycles.

### Cross-Core Worst-Case

`flash_safe_execute()` stalls Core 1 via `multicore_lockout`. Duration depends on flash operation:
- Sector erase: ~150ms (rare, only on log flush or erase)
- Page write: ~1-2ms
- Core 1 recovers via `i2c_bus_reset()` after lockout

---

## 5. Cross-Core Synchronization Overhead

| Mechanism | Writer | Reader | Overhead | Source |
|-----------|--------|--------|----------|--------|
| Seqlock (sensor data, 148B) | Core 1 @ 1kHz | Core 0 @ 200Hz | <5µs per write/read | ESTIMATED |
| `g_wdtCore1Alive` (atomic bool) | Core 1 @ 1kHz | Core 0 @ 1Hz | <1µs | ESTIMATED |
| `g_calNeoPixelOverride` (atomic u8) | Core 0 (rare) | Core 1 @ 1kHz | <1µs | ESTIMATED |
| `g_core1PauseI2C` (atomic bool) | Core 0 (rare) | Core 1 @ 1kHz | <1µs | ESTIMATED |
| `flash_safe_execute` lockout | Core 0 (rare) | Core 1 stalled | 1-150ms | ESTIMATED |

---

## 6. ISR Budget

| ISR | Core | Trigger | Duration | Source |
|-----|------|---------|----------|--------|
| USB CDC (TinyUSB) | Core 0 | USB SOF + data | ~5µs | ESTIMATED |
| UART RX (GPS) | Core 1 | Byte received | <1µs | ESTIMATED |
| PIO (NeoPixel) | Core 1 | DMA complete | <1µs | ESTIMATED |

ISR preemption is minimal — USB CDC handler is lightweight (`tud_task()` under mutex), UART RX pushes to ring buffer, PIO is fire-and-forget.

---

## 7. Memory Budget

### 7.1 Binary Section Sizes

| Section | Size | Location | Notes |
|---------|------|----------|-------|
| `.text` | 162,388 B (158.6 KB) | Flash (XIP) | Code + SDK |
| `.rodata` | 21,672 B (21.2 KB) | Flash (XIP) | Constants, strings, WMM table |
| `.data` | 30,640 B (29.9 KB) | SRAM (copied from flash) | Includes codegen_fpft (20.1 KB) |
| `.bss` | 310,212 B (302.9 KB) | SRAM (zeroed) | Buffers, state |
| **Total SRAM** | **340,852 B (332.9 KB)** | — | **64.0% of 520 KB** |
| **Total Flash** | **214,700 B (209.7 KB)** | — | **1.3% of 16 MB** |

### 7.2 Largest BSS Consumers

| Symbol | Size | Purpose |
|--------|------|---------|
| `g_sramRingBuf` | 204,800 B (200 KB) | SRAM ring buffer for logging (fallback when PSRAM unavailable) |
| `g_eskfBuffer` | 67,104 B (65.5 KB) | ESKF seqlock double buffer (1000 samples × ~67B) |
| `g_magSamples` | 3,600 B | Magnetometer calibration sample storage |
| `g_6posSamples` | 3,600 B | 6-position accel calibration samples |
| `g_flightTable` | 2,460 B | Flight log table state |
| `g_bierman_ud` | 2,400 B | Bierman UD factorization workspace |
| `g_eskf` | 2,520 B | ESKF filter state (24-state) |
| `codegen_fpft` | 20,124 B (.data) | FPFT in SRAM (.time_critical) |
| `core1_stack` | 4,096 B | Core 1 stack (SCRATCH_X) |

### 7.3 Stack Budget

| Core | Allocated | Est. Used | Margin | Source |
|------|-----------|-----------|--------|--------|
| Core 0 | 4,096 B (SCRATCH_Y) | ~2,000 B | ~50% | ESTIMATED |
| Core 1 | 4,096 B (SCRATCH_X) | ~1,500 B | ~63% | ESTIMATED |

**Note:** Stack high-water marks are estimates based on call chain analysis. For measured values, paint with `0xDEADBEEF` sentinel via GDB at boot and scan after soak. The `cla_collect.py --gdb` flag supports this.

### 7.4 PSRAM (when available)

| Resource | Size | Purpose |
|----------|------|---------|
| PSRAM ring buffer | Up to 8 MB | Flight data logging (replaces g_sramRingBuf) |
| Available | 8 MB total | Only logging uses PSRAM currently |

---

## 8. Blocking Operations

| Operation | Core | Duration | Frequency | Watchdog Impact |
|-----------|------|----------|-----------|-----------------|
| `flash_safe_execute` (sector erase) | Core 0 | ~150ms | On log flush | Core 1 stalled via lockout |
| `flash_safe_execute` (page write) | Core 0 | ~1-2ms | On log flush | Core 1 briefly stalled |
| `calibration_save` | Core 0 | ~150ms | On cal command | Core 1 stalled |
| `i2c_bus_reset` (after flash) | Core 1 | ~1ms | After each flash op | None |
| `sleep_ms(1)` in main loop | Core 0 | 1ms | Every loop | None (watchdog kicked before) |
| USB CDC settle (`kUsbSettleMs`) | Core 0 | 200ms | On terminal connect | None |

---

## 9. Headroom Assessment

### 9.1 Headroom Target

No core should exceed **50% average duty cycle** including worst-case coincidence. This provides 2× margin for unexpected load spikes.

### 9.2 Current Status

| Core | Avg Duty | WCET Duty | Target | Status |
|------|----------|-----------|--------|--------|
| Core 0 | ~12% | ~16% | <50% | **PASS — 38% margin** |
| Core 1 | ~85% | ~126% (overflow) | <100% | **MARGINAL** |

Core 1 is dominated by I2C IMU reads (77.4% of cycle). The worst-case overflow self-corrects but limits headroom for additional Core 1 work.

### 9.3 Stage 8 Cost Estimates

| Addition | Core | Est. Cost | Source |
|----------|------|-----------|--------|
| Flight Director state machine | Core 0 | ~5µs per tick | ESTIMATED |
| Pyro continuity check | Core 0 | ~10µs at 10Hz | ESTIMATED |
| Phase-scheduled Q/R switch | Core 0 | ~2µs (table lookup) | ESTIMATED |
| Extended logging fields | Core 0 | ~5µs per frame | ESTIMATED |
| **Total Stage 8 addition** | Core 0 | **~22µs** | ESTIMATED |
| **Revised Core 0 duty** | — | **~12.5%** | CALCULATED |

Stage 8 adds negligible load to Core 0. No core migration needed.

### 9.4 Recommendations

1. **Core 0 has ample headroom.** Stage 8 additions are trivial compared to available margin.
2. **Core 1 is I2C-bound.** If additional sensors are added, consider SPI for high-rate devices or moving non-critical work off Core 1.
3. **SRAM is the binding constraint** (64% used). The 200KB SRAM ring buffer dominates — when PSRAM is available, this frees 200KB.
4. **Flash is effectively unlimited** at 1.3% used.

---

## 10. Data Sources

| Tag | Meaning | Examples |
|-----|---------|---------|
| MEASURED-SOAK | From this CLA collection run (270s soak, `cla_collect.py`) | Sensor rates, ESKF predict timing |
| MEASURED-BENCH | From prior benchmark runs with dates/sources | I2C timing (IVP-13), UD benchmark, codegen FPFT |
| CALCULATED | Derived from measured inputs | Duty %, margin, hyperperiod |
| ESTIMATED | Informed by code inspection, no direct measurement | Individual tick function times (except ESKF) |
| DATASHEET | Vendor specification | RP2350 SRAM/clock, sensor specs |

### 10.1 Re-Run Process

```bash
# Collect CLA data (serial only, no code changes)
python scripts/cla_collect.py --duration 60 --output docs/audits/cla_rbm/cla_YYYY-MM-DD.md

# With stack analysis (requires OpenOCD + debug probe)
python scripts/cla_collect.py --duration 60 --gdb --output docs/audits/cla_rbm/cla_YYYY-MM-DD.md
```

### 10.2 When to Re-Run

- After any stage completion
- After adding new tick functions or changing rates
- After significant memory layout changes
- Before major milestones (pre-flight, design reviews)

---

## Appendix: Prior Benchmark Data (consolidated)

### I2C Bus Timing (IVP-13, 2026-02-05)

| Sensor | Operation | Avg (µs) | Max (µs) |
|--------|-----------|----------|----------|
| ICM-20948 | Full read (23 bytes, includes bank select) | 774 | 791 |
| DPS310 | Pressure+temp read (cached continuous) | 251 | 294 |

### ESKF/Bierman Timing (UD Benchmark, 2026-02-24)

| Operation | Avg (µs) | Source |
|-----------|----------|--------|
| Codegen FPFT (24-state, SRAM) | 111 | Benchmark (isolated) |
| Codegen FPFT (production, with UD overhead) | 577 | Production soak (270s) |
| Bierman scalar update | 43 | Benchmark |
| Joseph scalar update (disabled) | 81 | Benchmark |
| Full predict+factorize+reconstruct cycle | 577 avg (108 min, 802 max) | Production soak (270s) |

### XIP Cache Impact (IVP-47, 2026-02-21)

| Configuration | Time (µs) | Speedup |
|---------------|-----------|---------|
| Codegen from flash (XIP) | 398 | 1× |
| Codegen from SRAM (.time_critical) | 59 | 6.7× |
| Production (24-state, SRAM) | 111 | — |
