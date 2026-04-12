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

Measured from `arm-none-eabi-size -A build/rocketchip.elf` (post-Stage 16A, 2026-04-12).

| Resource | Used | Available | Notes |
|----------|------|-----------|-------|
| Flash (.text) | 118 KB | 16 MB | Executable code (XIP) |
| Flash (.rodata) | 33 KB | (shared) | Constants, strings, lookup tables |
| Flash (total firmware) | ~152 KB | 16 MB | .text + .rodata + .binary_info |
| SRAM (.data) | 31 KB | 520 KB | Initialized globals (includes .time_critical codegen) |
| SRAM (.bss) | 100 KB | 520 KB | Zero-initialized globals (QP AO queues, ESKF state, ring buffers) |
| SRAM (heap) | 2 KB | (allocated) | Pico SDK heap (unused post-init) |
| SRAM (total) | ~134 KB | 520 KB | 26% utilization |
| Stack (Core 0) | 4 KB | 4 KB | Linker-allocated (.stack_dummy). Monitor for large locals (LL Entry 1) |
| Stack (Core 1) | 4 KB | 4 KB | Linker-allocated (.stack1_dummy) |
| PSRAM | 0 | 8 MB | Adafruit Feather HSTX. Reserved for flight log ring buffer (future) |

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
