# RC_OS Verification Checklist

**Purpose:** Track verification of all RC_OS menu items and functionality
**Last Updated:** 2026-01-31
**Version Being Tested:** v0.3.1

---

## Test Environment

- [x] Hardware connected (RP2350 Feather)
- [x] Terminal connected to COM port
- [x] Debug probe available (optional but recommended)

---

## Main Menu Tests

### Help Command ('h')

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| Press 'h' | Shows system status banner | ✓ | Working |
| Shows uptime | Uptime in milliseconds displayed | ✓ | Working |
| Shows heap | Free heap bytes displayed | ✓ | Working |
| Shows commands | h, s, c commands listed | ✓ | Working |

### Sensor Status ('s')

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| Press 's' | Shows sensor status | ✓ | Working |
| IMU data shown | Accel/gyro values displayed | ✓ | Working |
| Baro data shown | Pressure and temp displayed | ✓ | Working |
| Sample counts | IMU/Mag/Baro sample counts shown | ✓ | Working |
| Error counts | Error counters shown (should be 0) | ✓ | Working |

### Calibration Menu ('c')

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| Press 'c' | Enters calibration menu | ✓ | Working |
| Menu displays | Shows w,l,a,g,m,b,x options | ✓ | Working |

---

## Calibration Menu Tests

### Exit ('x')

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| Press 'x' | Returns to main menu | ✓ | Working |
| "Returning to main" | Message displayed | ✓ | Working |

### Level Calibration ('l')

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| Press 'l' | Starts level cal | ✓ | Prompts for ENTER first |
| Keep device flat | Message prompts user | ✓ | Working |
| Completes quickly | ~2 seconds typical | ✓ | Working |
| Shows "OK!" | Success message | ✓ | Working |
| MAVLink path | `[GCS] Accel calibration requested (type=1)` in output | ✓ | Working |

### Gyro Calibration ('g')

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| Press 'g' | Attempts gyro cal | ✓ | Working |
| Not implemented | Shows "not yet implemented" | ✓ | Working |
| MAVLink path | `[GCS] Gyro calibration requested` in output | ✓ | Working |

### Baro Calibration ('b')

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| Press 'b' | Attempts baro cal | ✓ | Working |
| Auto-calibrates | Shows "calibrates automatically at boot" | ✓ | Working |
| MAVLink path | `[GCS] Barometer calibration requested` in output | ✓ | Working |

### 6-Position Accel Cal ('a')

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| Press 'a' | Starts 6-pos accel cal | ✓ | Working |
| Position prompts | Prompts for each of 6 positions | ✓ | Working |
| ENTER confirms | Each position confirmed with ENTER | ✓ | Working |
| 'x' cancels | Can cancel mid-calibration | ✓ | Working |
| MAVLink path | `[GCS] Accel calibration requested (type=2)` | ✓ | Working |

### Compass Calibration ('m')

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| Press 'm' | Starts compass cal | ✓ | Working |
| Rotation prompt | Prompts to rotate device | ✓ | Working |
| 'x' completes | Press x when done rotating | ✓ | Working |
| MAVLink path | `[GCS] Compass calibration requested` | ✓ | Working |

### Calibration Wizard ('w')

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| Press 'w' | Starts wizard | ✓ | Working |
| Step 1: Level | Level cal step shown | ✓ | Working |
| ENTER proceeds | Runs level cal on ENTER | ✓ | Working |
| 'x' skips | Can skip each step | ✓ | Working |
| Step 2: 6-pos | 6-pos accel step shown | ✓ | Working |
| Step 3: Compass | Compass cal step shown | ✓ | Working |
| Wizard complete | "Wizard Complete" shown | ✓ | Working |
| Returns to main | Back to main menu after | ✓ | Working |

---

## Robustness Tests

### Terminal Connect/Disconnect

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| Disconnect terminal | LED blinks slowly (1Hz) | ✓ | Working |
| Reconnect terminal | Banner appears, CLI works | ✓ | VSCode auto-reconnects, CLI responsive |
| Rapid connect/disconnect | No crash, no hang | ✓ | Working |

### Input Handling

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| Unknown key in main | Ignored, no output | ✓ | Working |
| Unknown key in cal menu | Ignored, no output | ✓ | Working |
| CR/LF handling | Works with any line ending | ✓ | VSCode uses CRLF, works |
| ESC key | Works as 'x' in cal menu | - | Not implemented (low priority) |

### MAVLink Coexistence

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| MAVLink start byte (0xFE) | Routes to GCS parser | | |
| MAVLink v2 start (0xFD) | Routes to GCS parser | | |
| ASCII after MAVLink | CLI still works | | |

---

## Calibration Persistence

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| Run level cal | Completes successfully | | |
| Power cycle | Reboot device | | |
| Check cal status | Still shows calibrated | | |

---

## Known Issues

*Document any issues found during testing here*

| Issue | Severity | Notes |
|-------|----------|-------|
| No arrow key navigation | Low | Single-key commands only (future enhancement) |
| Menu flow was unintuitive | Fixed | Calibrations now stay in cal menu after completion |

---

## Test Session Log

### Session: 2026-01-31

**Tester:** Claude + User
**Firmware:** v0.3.1

**Summary:**
- [x] Main menu tests complete
- [x] Calibration menu tests complete (all options verified: w, l, a, g, m, b, x)
- [x] Robustness tests complete (ESC key not implemented, low priority)
- [x] All implemented tests passing (MAVLink coexistence and persistence deferred)

**Notes:**
- Fixed menu flow: calibrations now stay in calibration menu after completion
- Added ENTER prompt before running actual calibrations
- Level cal via wizard and standalone both working
- Gyro cal shows "not implemented" as expected
- Baro cal shows "auto at boot" as expected
- 6-pos accel cal (a) and compass cal (m) verified working
- Persistence test blocked until calibration save is fully implemented

**Next Steps (Calibration Persistence):**

**Session 2026-01-31 (continued) - Calibration persistence debugging:**

1. **Fixed: Storage timer tick not registered**
   - Added `hal.scheduler->register_timer_process(FUNCTOR_BIND(&g_storage, &Storage::_timer_tick, void))` to `hal_init()`
   - Without this, `set_and_save()` queues writes but they never flush to flash
   - File: `lib/ap_compat/AP_HAL_RP2350/HAL_RP2350_Class.cpp:140-143`

2. **Fixed: AP_Param::load_all() not called**
   - Added `AP_Param::load_all()` after `AP_Param::setup()` in `initSensors()`
   - `setup()` validates/initializes header, but `load_all()` actually loads saved values from flash
   - File: `src/services/SensorTask.cpp:115`

3. **CURRENT ISSUE: Calibration offsets remain zero after successful calibration**
   - GDB inspection shows `_accel_offset[0] = {x=0, y=0, z=0}` even after "calibration successful"
   - Raw accel values show ~30° tilt (X: +4.6, Z: +8.4) even when device is level
   - Calibration reports success but offset is NOT being stored in `_accel_offset`
   - Possible causes to investigate:
     a. Calibration convergence failing internally (not reaching save code path)
     b. `INS_PARAM_WRAPPER` not working correctly for `set_and_save`
     c. Timing issue: samples being read outside the actual calibration window
   - User observed: device position during calibration doesn't seem to affect final values

4. **Debugging notes:**
   - `simple_accel_cal()` takes 50 samples × 5ms = 250ms per iteration, up to 40 iterations
   - Device must be held level THE ENTIRE TIME "Calibrating..." is shown (2-3 seconds)
   - `_accel_offset(k).set_and_save(new_accel_offset[k])` at line 2662 should save offset
   - GDB command to check: `print *((AP_InertialSensor*)0x200188b8)` then look for `_accel_offset_old_param`

**Files modified this session:**
- `lib/ap_compat/AP_HAL_RP2350/HAL_RP2350_Class.cpp` - storage timer tick registration
- `src/services/SensorTask.cpp` - AP_Param::load_all() call

---
