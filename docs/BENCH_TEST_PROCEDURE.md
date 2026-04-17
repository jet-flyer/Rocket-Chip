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

## Troubleshooting

- **Probe disconnect mid-soak:** OpenOCD session can be restarted; target keeps running on battery. But halting the target mid-codegen_fpft-push can leave MSP momentarily below MSPLIM, triggering a spurious STKOF fault. Observed once during IVP-132 setup; did not reproduce on reset. If a soak fault fires and the last-known MSP was fine, try one clean reset before investigating the binary.
- **COM7 stuck after serial test:** See `C:\Users\pow-w\.claude\projects\c--Users-pow-w-Documents-Rocket-Chip\memory\feedback_com7_stuck.md`. Soak procedure deliberately avoids Python serial to sidestep this.
- **GDB locale warning (CP1252 → UTF-32):** Cosmetic, Windows-only. Ignore.
