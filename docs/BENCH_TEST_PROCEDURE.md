# Bench Test Procedure

**Purpose:** Formalize the Stage 16B soak-test procedure so future runs are reproducible and the pass criteria are explicit.

## Two-Tier Soak

### Tier 1: Diagnostic Soak (5 min, USB powered, bench binary)

**When:** After every code change touching Core 0/Core 1 paths during Stage 16B.

**Binary:** `build/rocketchip.elf` (bench tier, full dev hooks available).

**Procedure:**
1. Flash via probe:
   ```
   arm-none-eabi-gdb build/rocketchip.elf -batch \
     -ex "target extended-remote localhost:3333" \
     -ex "monitor reset halt" -ex "load" -ex "monitor resume"
   ```
2. Run `scripts/soak_gdb.gdb` (GDB-only, no serial) for 5 min.
3. Verify: zero crashes, IMU ≥990 Hz, baro ≥30 Hz, zero accumulated errors.

**Pass criteria:**
- IMU read rate ≥ 990 Hz
- Baro read rate ≥ 30 Hz
- IMU error count = 0
- Baro error count = 0
- MSP stays ≥ 0x20081800 (within 4 KB of top of Core 0 stack)
- Core1 loop count matches IMU read count (±1)

### Tier 2: Integration Soak (30 min, battery powered, flight binary)

**When:** Gate before Stage 17 / any field test.

**Binary:** `build_flight/rocketchip.elf` (flight tier, no dev hooks linked).

**Procedure:**
1. Build flight binary:
   ```
   cd build_flight && cmake --build . --target rocketchip
   ```
2. Flash via probe (same GDB `load` + `monitor resume` as above).
3. Unplug USB. Confirm target running on LiPo (probe should still reach target; probe USB is independent of target USB CDC).
4. Run `scripts/soak_30min.gdb` — captures snapshots at T=0, 5, 10, 15, 20, 25, 30 min to `logs/soak_30min.log`.
5. Verify all snapshots pass the Tier 1 criteria, plus:
   - No watchdog resets (core1_loop_count should advance monotonically — no reset-zero spikes).
   - MSP unchanged from boot (0x20081fd0 idle value).
   - IMU read rate stable across all intervals (no monotone decay).

**Pass criteria:** All metrics within Tier 1 bounds for all 7 snapshots.

## Reference Results

**IVP-132 Phase 4, 2026-04-16, 400 mAh LiPo + probe attached, no USB:**

| T | IMU reads | Baro reads | IMU err | Baro err | MSP |
|---|-----------|------------|---------|----------|-----|
| 0min | 459,121 | 14,347 | 0 | 0 | 0x20081fd0 |
| 5min | 758,359 | 23,698 | 0 | 0 | 0x20081fd0 |
| 10min | 1,057,593 | 33,049 | 0 | 0 | 0x20081fd0 |
| 15min | 1,356,824 | 42,400 | 0 | 0 | 0x20081fd0 |
| 20min | 1,656,069 | 51,752 | 0 | 0 | 0x20081fd0 |
| 25min | 1,955,295 | 61,102 | 0 | 0 | 0x20081fd0 |
| 30min | 2,254,536 | 70,454 | 0 | 0 | 0x20081fd0 |

Sustained rates: IMU 997 Hz, Baro 31.2 Hz, GPS 12.5 Hz, Core1 loops match IMU reads exactly. Zero errors across 2.25M IMU reads over 30 min.

## Preconditions: Verify Before Claiming a Pass

**A soak is meaningless if the thing being soaked isn't actually engaged.**
Learned the hard way 2026-04-16: a station soak claimed PASS while the
radio had never initialized (wrong `PICO_BOARD` in the build config).
Queue watermarks stayed "stable" at idle because nothing was posting
events — which *looks* exactly like a healthy soak.

**Rule: every soak MUST verify at T=0 that each subsystem under test is
actually live.** Absence of signal is not proof of health.

### Required T=0 Preconditions (vehicle soak)

```
# Board + build identity
print PICO_BOARD                          # expect adafruit_feather_rp2350
print kBuildTag / version string          # matches what you flashed
# Init flags — must all be true
print g_imuInitialized                    # true
print g_baroInitialized                   # true
print g_gpsInitialized                    # true (outdoors) or document absence
print g_neopixelInitialized               # true
print g_spiOk                             # true
print g_eskfInitialized                   # true
# Peripheral-specific init
print AO_Radio->state.initialized         # true
print AO_Radio->state.radio.initialized   # true
print AO_Radio->state.radio.cs_pin        # matches kRadioCsPin for board
print AO_Radio->state.radio.rst_pin       # matches kRadioRstPin for board
print AO_Radio->state.radio.irq_pin       # matches kRadioIrqPin for board
# Liveness baselines
print g_sensorSeqlock.data.imu_read_count # will climb
print g_sensorSeqlock.data.core1_loop_count # will climb
```

### Required T=0 Preconditions (station soak)

```
# Board + build identity
print PICO_BOARD                          # expect adafruit_fruit_jam
# Role flags
print ROCKETCHIP_JOB_STATION              # 1
print kRadioModeRx                        # true
# Peripheral init
print g_neopixelInitialized               # true
print g_spiOk                             # true
print AO_Radio->state.initialized         # true
print AO_Radio->state.radio.initialized   # true
print AO_Radio->state.scheduler.phase     # expect kRxContinuous (2), NOT kIdle
print AO_Radio->state.radio.cs_pin        # matches Fruit Jam (10)
print AO_Radio->state.radio.rst_pin       # matches Fruit Jam (6)
print AO_Radio->state.radio.irq_pin       # matches Fruit Jam (5)
```

**If any check fails, the soak is invalid until the underlying issue is
fixed.** Do NOT run the soak and post-hoc explain away the zero.

### Liveness Growth (not just stability)

At T=5min (first mid-check), verify that metrics expected to *grow* have
actually grown vs. T=0:

- Vehicle: `imu_read_count`, `baro_read_count`, `core1_loop_count`,
  `gps_read_count` (if fix). Growth rate should match expected sample
  rates (IMU ~1kHz, baro ~31Hz, GPS ~10Hz).
- Station (vehicle transmitting): `AO_Radio->state.rx_count` must be > 0
  after 60 seconds of a vehicle at expected link range.
- Station (vehicle off): document this case explicitly — soak validates
  only Core 0 + QP stability, NOT radio RX path.

### The "Frankenstein Build" Anti-Pattern

A binary that:
- compiles and links cleanly,
- flashes and boots without asserts,
- Core 0 runs its QV loop normally,

...can still have **the wrong hardware abstraction** if the build was
configured with the wrong `PICO_BOARD`, wrong role flags, wrong mission
profile, or similar. The firmware doesn't know it's on the wrong board —
it just tries to drive GPIO 11 as RADIO_RST because that's what
`kRadioRstPin` was set to at compile time. The peripheral silently fails,
the AO that owns it silently gives up, and downstream code runs happily
because it doesn't depend on that AO succeeding.

**Mitigation is procedural until Stage 16C automates it:**

1. Before creating a new `build_*` directory, copy the `cmake` invocation
   from the closest working build's `CMakeCache.txt` — do not assume
   defaults are right.
2. After `cmake -B build_xxx`, `grep -E "PICO_BOARD|ROCKETCHIP_JOB|BUILD_FOR_FLIGHT"
   build_xxx/CMakeCache.txt` and confirm the values match the target.
3. After flashing, read the boot banner (or GDB-print the build tag and
   board name symbols) before starting any soak.
4. Soak scripts MUST include the T=0 precondition block above.

## Troubleshooting

- **Probe disconnect mid-soak:** OpenOCD session can be restarted; target keeps running on battery. But halting the target mid-codegen_fpft-push can leave MSP momentarily below MSPLIM, triggering a spurious STKOF fault. Observed once during IVP-132 setup; did not reproduce on reset. If a soak fault fires and the last-known MSP was fine, try one clean reset before investigating the binary.
- **COM7 stuck after serial test:** See `C:\Users\pow-w\.claude\projects\c--Users-pow-w-Documents-Rocket-Chip\memory\feedback_com7_stuck.md`. Soak procedure deliberately avoids Python serial to sidestep this.
- **GDB locale warning (CP1252 → UTF-32):** Cosmetic, Windows-only. Ignore.
