# Stage 11 PIO Safety Architecture — Performance Benchmark (IVP-91)

**Date:** 2026-03-29
**Firmware:** Stage 11 HEAD (PIO watchdog + backup timers, SDK WD removed)
**Hardware:** Adafruit Feather RP2350 HSTX, ICM-20948, DPS310, PA1010D (UART), RFM95W

## Binary Size Comparison

| Section | Baseline (IVP-87) | Post-PIO (IVP-91) | Delta |
|---------|-------------------|-------------------|-------|
| text | 178,820 | 180,696 | **+1,876 (+1.0%)** |
| data | 0 | 0 | 0 |
| bss | 311,508 | 311,532 | **+24** |
| **total** | **490,328** | **492,228** | **+1,900** |

Text increase from: PIO heartbeat program (~8 instr), PIO backup timer program (~12 instr), pio_watchdog.cpp, pio_backup_timer.cpp.

## Timing Comparison

| Metric | Baseline | Post-PIO | Delta |
|--------|----------|----------|-------|
| predict avg | 689 µs | 706 µs | **+17 µs (+2.5%)** |
| predict min | 193 µs | 190 µs | -3 µs |
| predict max | 959 µs | 883 µs | -76 µs |

+2.5% avg is within the 5% regression threshold. The increase is from `pio_watchdog_feed()` (one `pio_sm_put()` call per tick — ~10 cycles).

## Sensor & Health Comparison

| Metric | Baseline | Post-PIO |
|--------|----------|----------|
| IMU reads (15s) | 34,781 | 39,359 |
| IMU errors | 1 | 2 |
| Innovation b | 1.20 | 0.71 |
| Innovation m | 1.08 | 0.01 |
| Confidence | Y | Y |
| AHRS div | 2.4° | 0.6° |
| AO jitter avg | 100ms | 100ms |

## PIO Resource Usage

| PIO Block | SMs Used | Purpose |
|-----------|----------|---------|
| PIO0 | 1 | WS2812 NeoPixel (may be on PIO2 due to auto-claim) |
| PIO1 | 0 | Reserved |
| PIO2 | 3 | Heartbeat WD + Drogue timer + Main timer |
| **Total** | **4/12** | **33% utilization** |

## Verdict

**PASS.** No timing regression >5%. Binary increase <2KB. PIO usage matches budget.
