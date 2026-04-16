# Hardware Budgets

**Purpose:** Track measured timing, memory, and resource budgets for firmware planning.

> **See also:** `docs/audits/cla_rbm/COMPUTATIONAL_LOAD_ANALYSIS.md` for current per-core
> CPU utilization, ESKF timing, sensor rates, and memory budget (measured 2026-03-08,
> Stage 7 firmware). The data below is from IVP-13 (2026-02-05) and serves as a
> historical baseline — the CLA supersedes it for current planning.

---

## I2C Bus Timing (400kHz Fast Mode)

Measured at IVP-13 (2026-02-05), single-core polling on Core 0.

| Sensor | Operation | Avg (us) | Max (us) | Notes |
|--------|-----------|----------|----------|-------|
| ICM-20948 | Full read (accel+gyro+temp+mag, 23 bytes) | 774 | 791 | Includes bank select |
| DPS310 | Pressure+temp read via ruuvi `get_last_result` | 251 | 294 | Reads cached continuous data |

**Combined per-cycle:** ~1025 us (IMU + baro at same tick)

**Bus utilization at target rates:**
- IMU 100Hz: 77.4 ms/s (7.7% of 1s)
- Baro 50Hz: 12.6 ms/s (1.3% of 1s)
- **Total: ~90 ms/s (9.0%)** — plenty of headroom for GPS or additional sensors

**Budget ceiling:** At 400kHz I2C, theoretical max throughput is ~50,000 bytes/s. Current usage: ~3,550 bytes/s (23B * 100Hz + ~10B * 50Hz). Well under limit.

---

## Polling Rate Validation

| Sensor | Target Hz | Achieved Hz | Tolerance | Duration | Errors |
|--------|-----------|-------------|-----------|----------|--------|
| ICM-20948 | 100 | 100.0 | +/-5% | 60s | 0 |
| DPS310 | 50 | 50.0 | +/-10% | 60s | 0 |

Note: `sleep_ms(1)` in main loop caps effective loop rate at ~1kHz. Sufficient for current targets. Remove or reduce for Stage 3 if higher rates needed.

---

## PIO State Machines

RP2350 has 3 PIO blocks x 4 state machines = 12 total.

| PIO | SM | Use | Status |
|-----|----|-----|--------|
| PIO0 | 0 | WS2812 NeoPixel | Active |
| PIO0 | 1-3 | Available | |
| PIO1 | 0-3 | Available | |
| PIO2 | 0-3 | Available | |

---

## Memory

Measured from `arm-none-eabi-size -A build_flight/rocketchip.elf` (flight binary,
2026-04-16, post-IVP-131). Reproduce:
```bash
cd build_flight && cmake --build . && arm-none-eabi-size -A rocketchip.elf
```

| Resource | Used | Available | Notes |
|----------|------|-----------|-------|
| Flash (.text) | 128 KB (131,080 B) | 16 MB | Executable code (XIP) |
| Flash (.rodata) | 32 KB (32,476 B) | (shared) | Constants, strings, lookup tables |
| Flash (.binary_info) | 32 B | (shared) | Pico SDK metadata |
| Flash (total firmware) | ~160 KB | 16 MB | text + rodata + binary_info |
| SRAM (.data) | 31 KB (31,560 B) | 520 KB | Initialized globals (includes .time_critical codegen) |
| SRAM (.bss) | 306 KB (313,480 B) | 520 KB | Zero-initialized globals (QP AO queues, ESKF state, sensor ring buffers) |
| SRAM (.ram_vector_table) | 272 B | 520 KB | Relocated vector table |
| SRAM (heap) | 2 KB | (allocated) | Pico SDK heap (unused post-init) |
| SRAM (total) | ~340 KB | 520 KB | **65% utilization** |
| Stack (Core 0, .stack_dummy) | 4 KB | 4 KB | MPU-protected with 64-byte guard (LL Entry 1) |
| Stack (Core 1, .stack1_dummy) | 4 KB | 4 KB | Core 1 sensor loop |
| PSRAM | 8 MB (ring buffer) | 8 MB | Adafruit Feather HSTX APS6404L-3SQR via QMI CS1. Active — logger uses full 8 MB as ring buffer (~48 min @ 50Hz). SRAM fallback (`g_sramRingBuf` 200 KB) used if PSRAM self-test fails |

**Note:** `.bss` grew from 100 KB (Stage 16A) to 306 KB. Top contributors
measured via `arm-none-eabi-nm --size-sort -r build_flight/rocketchip.elf`:

| Symbol | Size | File | Purpose |
|--------|------|------|---------|
| `g_sramRingBuf` | 200 KB (0x32000) | `ao_logger.cpp:75` | SRAM fallback ring buffer (allocated always; used only if PSRAM unavailable) |
| `g_eskfBuffer` | 68 KB (0x109A0) | `eskf_runner.cpp` | ESKF state circular buffer (1000 samples × 68B) |
| `core1_stack` | 4 KB (0x1000) | main.cpp | Core 1 sensor loop stack |
| `g_sectorBuf` | 4 KB | `flush_sectors()` | Flash sector flush buffer |
| `g_6posSamples` | 3.6 KB | calibration | 6-position accel cal samples |
| `g_magSamples` | 3.6 KB | calibration | Mag calibration samples |
| `g_flightTable` | 2.5 KB | flight_table.cpp | Flight log metadata |
| Misc AO queues + buffers | ~20 KB | active_objects | 8 AOs × 32 queue depth |

The 200KB `g_sramRingBuf` dominates `.bss`. It's the **static SRAM fallback**
ring buffer — always allocated regardless of whether PSRAM is active. On the
Feather RP2350 HSTX with working PSRAM, the fallback is unused (logger writes
to PSRAM at `psram_base_ptr()`) but the 200KB remains reserved in SRAM.

Two independent memory regions for the ring buffer:
1. **SRAM fallback** (`g_sramRingBuf`, 200 KB): always in `.bss`, used when PSRAM unavailable. Decimation rate `kDecimationSram` (~25 Hz → 145s capacity).
2. **PSRAM** (8 MB external): used when `psram_self_test` passes at boot. Decimation `kDecimationPsram` (~50 Hz → 48 min capacity).

The logger picks one at init based on PSRAM detect result. **Tiny 2350 port:**
no external PSRAM → fallback path only → 145s log capacity.

---

## Power

Not yet measured — requires ammeter on LiPo supply line. Values below are
datasheet estimates for planning. Actual measurement is a Stage 16C field test item.

| State | Estimated (mA) | Source | Notes |
|-------|---------------|--------|-------|
| Idle (LED off, sensors sleeping) | ~25 | RP2350 datasheet (active, no peripherals) | Excludes NeoPixel quiescent |
| Active polling (IMU 1kHz + baro 50Hz) | ~45 | ICM-20948 DS (3.5mA) + DPS310 DS (0.5mA) + RP2350 | Core 1 sensor loop |
| GPS active | ~55 | PA1010D DS (25mA) + above | I2C or UART mode |
| All sensors + LoRa TX | ~175 | RFM95W DS (+120mA @ +20dBm TX) | Peak during TX burst |
| NeoPixel (1 LED, white, full) | +60 | WS2812B DS (60mA max) | Actual usage much lower (dim patterns) |
