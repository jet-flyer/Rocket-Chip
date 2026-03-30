# Stage 10 Baseline Performance Benchmark (IVP-87)

**Date:** 2026-03-29
**Firmware:** Stage 10 HEAD (commit 390e4eb + IVP renumber)
**Hardware:** Adafruit Feather RP2350 HSTX, ICM-20948, DPS310, PA1010D (UART), RFM95W

## Binary Size

| Section | Bytes |
|---------|-------|
| text | 178,820 |
| data | 0 |
| bss | 311,508 |
| **total** | **490,328** |

## Timing (15s soak, ESKF healthy)

| Metric | Value |
|--------|-------|
| predict avg | 689 µs |
| predict min | 193 µs |
| predict max | 959 µs |
| predict calls | 2,909 (in 15s ≈ 194/s) |

## Sensor Reads (15s)

| Sensor | Reads | Errors | Rate |
|--------|-------|--------|------|
| IMU | 34,781 | 1 | ~2,319/s |
| Mag | 3,458 | 0 | ~231/s |
| Baro | 1,086 | 0 | ~72/s |
| GPS | 347 | 0 | ~23/s |

## Stage 10 Features

| Feature | Status |
|---------|--------|
| Phase Q/R | att=1.0 vel=1.0 (IDLE baseline) |
| Innovation ratios | b=1.20 m=1.08 gp=0.00 gv=0.00 |
| Confidence gate | conf=Y div=2.4° unc=0ms |
| ESKF health | HEALTHY, qnorm=1.000000 |

## Notes

- ESKF enters P-growth failure after ~30-45s on bench (baro ground reference
  drift causes innovation divergence). This triggers CR-1 reset → eventual
  ESKF disable via IVP-66 watchdog recovery policy. Pre-existing condition,
  not caused by Stage 10.
- The 15s reading captures steady-state ESKF performance before P-growth trigger.
- AO jitter: 100ms avg (100Hz tick, nominal).
- PIO usage: 1/12 SMs (WS2812 only).
