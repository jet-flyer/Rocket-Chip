# RC_OS Verification Checklist

**Purpose:** Track verification of all RC_OS menu items and functionality
**Last Updated:** 2026-01-30
**Version Being Tested:** v0.3.1

---

## Test Environment

- [ ] Hardware connected (RP2350 Feather)
- [ ] Terminal connected to COM port
- [ ] Debug probe available (optional but recommended)

---

## Main Menu Tests

### Help Command ('h')

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| Press 'h' | Shows system status banner | | |
| Shows uptime | Uptime in milliseconds displayed | | |
| Shows heap | Free heap bytes displayed | | |
| Shows commands | h, s, c commands listed | | |

### Sensor Status ('s')

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| Press 's' | Shows sensor status | | |
| IMU data shown | Accel/gyro values displayed | | |
| Baro data shown | Pressure and temp displayed | | |
| Sample counts | IMU/Mag/Baro sample counts shown | | |
| Error counts | Error counters shown (should be 0) | | |

### Calibration Menu ('c')

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| Press 'c' | Enters calibration menu | | |
| Menu displays | Shows w,l,a,g,m,b,x options | | |

---

## Calibration Menu Tests

### Exit ('x')

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| Press 'x' | Returns to main menu | | |
| "Returning to main" | Message displayed | | |

### Level Calibration ('l')

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| Press 'l' | Starts level cal | | |
| Keep device flat | Message prompts user | | |
| Completes quickly | ~2 seconds typical | | |
| Shows "OK!" | Success message | | |
| MAVLink path | `[GCS] Accel calibration requested (type=1)` in output | | |

### Gyro Calibration ('g')

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| Press 'g' | Attempts gyro cal | | |
| Not implemented | Shows "not yet implemented" | | |
| MAVLink path | `[GCS] Gyro calibration requested` in output | | |

### Baro Calibration ('b')

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| Press 'b' | Attempts baro cal | | |
| Auto-calibrates | Shows "calibrates automatically at boot" | | |
| MAVLink path | `[GCS] Barometer calibration requested` in output | | |

### 6-Position Accel Cal ('a')

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| Press 'a' | Starts 6-pos accel cal | | |
| Position prompts | Prompts for each of 6 positions | | |
| ENTER confirms | Each position confirmed with ENTER | | |
| 'x' cancels | Can cancel mid-calibration | | |
| MAVLink path | `[GCS] Accel calibration requested (type=2)` | | |

### Compass Calibration ('m')

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| Press 'm' | Starts compass cal | | |
| Rotation prompt | Prompts to rotate device | | |
| 'x' completes | Press x when done rotating | | |
| MAVLink path | `[GCS] Compass calibration requested` | | |

### Calibration Wizard ('w')

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| Press 'w' | Starts wizard | | |
| Step 1: Level | Level cal step shown | | |
| ENTER proceeds | Runs level cal on ENTER | | |
| 'x' skips | Can skip each step | | |
| Step 2: 6-pos | 6-pos accel step shown | | |
| Step 3: Compass | Compass cal step shown | | |
| Wizard complete | "Wizard Complete" shown | | |
| Returns to main | Back to main menu after | | |

---

## Robustness Tests

### Terminal Connect/Disconnect

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| Disconnect terminal | LED blinks slowly (1Hz) | | |
| Reconnect terminal | Banner appears, CLI works | | |
| Rapid connect/disconnect | No crash, no hang | | |

### Input Handling

| Test | Expected | Status | Notes |
|------|----------|--------|-------|
| Unknown key in main | Ignored, no output | | |
| Unknown key in cal menu | Ignored, no output | | |
| CR/LF handling | Works with any line ending | | |
| ESC key | Works as 'x' in cal menu | | |

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
| | | |

---

## Test Session Log

### Session: 2026-01-30

**Tester:** [Your name]
**Firmware:** v0.3.1 (commit 0112b6e)

**Summary:**
- [ ] Main menu tests complete
- [ ] Calibration menu tests complete
- [ ] Robustness tests complete
- [ ] All tests passing

**Notes:**

---
