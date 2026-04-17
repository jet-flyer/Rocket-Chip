# RocketChip Pre-Flight Checklist

**Purpose:** Walkthrough procedure for preparing the vehicle and station for
field test. Formalizes the `p` command (Go/No-Go check) into a written
sequence usable at the launch site, plus bench-side verification steps,
field kit checklist, and fail-response procedures.

**Status:** Stage 17 entry document. Update from experience each flight.

**Scope:** Low-power passive-recovery flights (F/G motor, stock chute, no
live pyro). Not yet certified for live-charge or mission-class flights
(see `docs/IVP.md` Stage 17 gate milestone).

---

## Before Leaving for the Launch Site

### 1. Vehicle Firmware Verification

Use the debug probe on the bench to confirm the flight binary is ready:

```bash
# Start OpenOCD
taskkill //F //IM openocd.exe 2>/dev/null; sleep 2
/c/Users/pow-w/.pico-sdk/openocd/0.12.0+dev/openocd \
  -s /c/Users/pow-w/.pico-sdk/openocd/0.12.0+dev/scripts \
  -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "adapter speed 5000" &

# Flash the flight binary (NOT bench — flight tier has no fault hooks)
cmake --build build_flight --target rocketchip
/c/Users/pow-w/.pico-sdk/toolchain/14_2_Rel1/bin/arm-none-eabi-gdb.exe \
  build_flight/rocketchip.elf -batch \
  -ex "target extended-remote localhost:3333" \
  -ex "monitor reset halt" -ex "load" -ex "monitor resume"
```

- [ ] Flight binary flashes without errors
- [ ] Build tag printed on serial banner matches the release being flown
- [ ] Git hash in version string matches the expected commit

### 2. Bench 5-Minute Diagnostic Soak

Run the Tier 1 soak per `docs/BENCH_TEST_PROCEDURE.md`:

```bash
python scripts/bench_sim.py
```

Pass criteria:
- [ ] IMU read rate ≥ 990 Hz
- [ ] Baro read rate ≥ 30 Hz
- [ ] Zero IMU errors, zero baro errors after 5 min
- [ ] Zero watchdog resets
- [ ] MSP stays above 0x20081800

If any criterion fails, **do not go to the field.** Diagnose first.

### 3. Calibration Check

Confirm calibration is loaded and sane:

Serial CLI sequence:
- Press `s` → view sensor status. Accel magnitude should read ~9.8 m/s² at
  rest. Gyro bias should be under 0.01 rad/s on each axis at rest.
- Press `q` → `s` → view ESKF state. Velocity should be < 0.1 m/s at rest.
  If drifting, **redo calibration** before leaving.

### 4. Mission Profile Verification

- [ ] `rocket.cfg` (or configured mission profile) matches the flight:
      motor class, expected apogee, pyro channel assignments
- [ ] Drogue / main timeouts in `mission_profile_data.h` are appropriate
      for expected flight duration (default: 15s drogue backup, 45s main
      backup after BOOST detection)
- [ ] `abort_fires_drogue_from_boost/coast` flags match intent

### 5. Station Firmware + Link Check

- [ ] Station flight binary flashed (`build_station_flight/rocketchip.elf`)
- [ ] Station boots, USB CDC banner shows version match
- [ ] Bench radio link test: power up both boards indoors at close range,
      confirm station shows vehicle telemetry frames arriving. Acceptance:
      any valid `[CMD]` or `nav_seq` output within 30 s.

### 6. Field Kit Packing

- [ ] Both boards + USB cables
- [ ] Laptop with build environment + OpenOCD + probe
- [ ] Debug probe with SWD cable (for emergency reflash)
- [ ] LiPo batteries — **fully charged** (multimeter to confirm ≥ 3.9 V)
- [ ] Spare LiPo (400 mAh minimum, same or larger capacity)
- [ ] Spare USB-C cables
- [ ] Motors per mission profile + backup igniter
- [ ] Launch rail / pad (1010 recommended for modified Big Daddy, see IVP-135)
- [ ] Recovery wadding, chute (stock for first flights)
- [ ] Multimeter / continuity tester
- [ ] Printed copy of this checklist

---

## At the Launch Site

### 7. Power-Up Sequence

1. **Vehicle:** connect LiPo. Status LED should flash boot pattern, then
   settle into the IDLE pattern.
2. **Station:** connect USB to laptop, open serial terminal on COM7.
3. **Verify station receives vehicle telemetry** — some valid nav frames
   within 60 seconds. If no frames:
   - Check vehicle is actually powered (LED visible)
   - Check station radio init printed in banner (`AO_Radio RX continuous`)
   - Move boards closer (≤ 1 m) and retry
4. **Station GPS** — press `q` → `s` → view station GPS satellites. Wait
   up to 60 s for cold start fix (PA1010D datasheet typical). Outdoor
   sky view required.

### 8. Pre-Arm Preflight (`p` Command)

On the vehicle serial CLI, press `p`. Expected output:

```
=== PREFLIGHT ===
IMU:      GO
Baro:     GO
ESKF:     GO
GPS:      GO
Radio:    GO
Flash:    GO
Watchdog: GO
PIO WDT:  GO
----------------
VERDICT:  GO
```

**Every line must say GO.** If any is not GO:

| Symptom | Likely cause | Action |
|---------|--------------|--------|
| `IMU: FAULT` | ICM-20948 not responding on I2C | Power-cycle vehicle, recheck. If persistent, abort flight, bench-debug. |
| `IMU: DEGRADED` | High error rate, sensor stuck returning zeros (LL Entry 29) | Let it stabilize 30 s. If no recovery, abort. |
| `Baro: FAULT` | DPS310 not responding | Same as IMU. Check wadding isn't blocking baro port. |
| `ESKF: NOT_READY` | Filter still converging (normal for first 10 s) | Wait, recheck `p`. |
| `ESKF: FAULT` | Velocity divergence detected (LL Entry 29) | Abort. Debug on bench. |
| `GPS: NOT_READY` | No fix yet (< 6 sats or HDOP > 5) | Wait up to 3 min for outdoor lock. |
| `GPS: FAULT` | GPS driver not responding | Power-cycle. If persistent, can fly WITHOUT GPS for short flights — document as caveat. |
| `Radio: ABSENT` | SX1276 init failed | Abort. Check antenna + power. |
| `Flash: FAULT` | PSRAM/flash log init failed | Abort. Flash storage is needed for post-flight log download. |
| `Watchdog: FAULT` | Hardware watchdog not armed | Abort. System can't self-recover on hang. |
| `PIO WDT: FAULT` | PIO backup timer not running | Abort. Pyro backup unavailable. |

**Hard rule: No "GO" verdict = no ARM.**

### 9. Arm Sequence

Once preflight is GO:

1. Load vehicle into airframe (avionics bay)
2. Verify battery secured, no loose wires
3. Close avionics bay, confirm latches engaged
4. Install on launch rail
5. Range officer clears the area
6. **On station CLI:** press `a` to send ARM command. Wait for ACK.
   - Expected: `[CMD] ACK'd (seq=N)` within 2 s
   - If DENIED: check vehicle `p` output — something changed since step 8
   - If no ACK after 3 retries: RadioScheduler sync issue (see
     AGENT_WHITEBOARD "RadioScheduler Sync" — known intermittent). Retry
     ARM. If still fails, DISARM on station and re-preflight from step 7.
7. Verify vehicle LED changes to ARMED pattern
8. Station dashboard should show `state: ARMED`

### 10. Launch

Standard NAR/TRA safety code applies. Not covered here — see NAR safety code.

### 11. Post-Launch Monitoring

On station:
- Watch `state:` field transition IDLE → ARMED → BOOST → COAST → APOGEE →
  DROGUE → MAIN → LANDED
- Watch telemetry frame rate — expected ~5 Hz (default after IVP-132a)
- RSSI / SNR should stay reasonable throughout flight (baseline varies
  by distance; < -110 dBm RSSI or < 0 dB SNR indicates marginal link)

### 12. Recovery

- [ ] Wait for LANDED state (or timeout if GPS/ESKF failed)
- [ ] Physically recover rocket
- [ ] **Do not approach until chute is grounded** — NAR safety code
- [ ] Power down vehicle (disconnect LiPo) before handling

### 13. Post-Flight

On bench back at base:
- [ ] Download flight log (TBD — log pull command not yet in PRE_FLIGHT scope)
- [ ] Archive telemetry captured on station side
- [ ] Power-cycle vehicle, recheck `p` — sensors should be GO again
- [ ] Note any anomalies in flight report for this launch

---

## Abort Procedure (Range-Safety Driven)

If range officer calls abort before launch or during flight:

- **On pad (before launch):** press `X` on station to send DISARM.
  Verify ACK. Vehicle returns to IDLE. Safe the motor.
- **In flight:** no in-flight abort is supported on current hardware
  (no FTS). Flight must be allowed to complete per range safety procedure.

---

## Fallback: No-GPS Flight

If GPS won't lock but all other subsystems are GO, short hops (< 500 m
apogee, clear line-of-sight) can proceed with GPS FAULT acknowledged:
- Landing detection falls back to baro + accel conjunction (IVP-119/120/121
  multi-channel landing detection)
- Station distance display will show "UNKNOWN"
- Post-flight log will have local coordinates only (no geofix)

Document the GPS-absent decision in the flight report.

---

## Related Documents

- `docs/BENCH_TEST_PROCEDURE.md` — diagnostic and integration soak procedures
- `docs/hardware/HARDWARE.md` — board pin assignments, pyro channels
- `docs/FAULT_INJECTION.md` — GDB-based fault injection (bench only)
- `docs/mission_profiles/` — mission profile reference
- `docs/airframe/big_daddy_integration.md` — airframe build notes (IVP-135)

---

## Checklist Card (Print-and-Carry)

A compressed one-page version of the critical-path items, suitable to
print and carry to the range. TBD — generate from this doc once we have
real flight experience to trim what's actually needed at the pad vs.
what's bench-only.
