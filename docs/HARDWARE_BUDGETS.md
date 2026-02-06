# Hardware Budgets

**Purpose:** Track measured timing, memory, and resource budgets for firmware planning.

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

## Memory (TBD)

Track stack usage, heap, and PSRAM allocation as firmware grows.

| Resource | Used | Available | Notes |
|----------|------|-----------|-------|
| Flash (text) | 60.6 KB | 16 MB | As of IVP-13 |
| SRAM (data+bss) | 3.9 KB | 520 KB | RP2350 internal |
| PSRAM | 0 | 8 MB | Adafruit Feather HSTX |
| Stack (Core 0) | TBD | Default | Monitor for large locals (LL Entry 1) |

---

## Power (TBD)

Measure current draw at various states once hardware is stable.

| State | Current (mA) | Notes |
|-------|-------------|-------|
| Idle (LED off, sensors sleeping) | TBD | |
| Active polling (IMU 100Hz + baro 50Hz) | TBD | |
| GPS active | TBD | |
| All sensors + telemetry | TBD | |
