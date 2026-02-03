# Status Indicators and Messages

**Purpose:** Central reference for all user-facing status feedback: LEDs, serial messages, telemetry codes, and audio (future).

---

## 1. LED Indicators

### Hardware

| LED | GPIO | Type | Purpose |
|-----|------|------|---------|
| Red LED | 7 | Simple on/off | Heartbeat, error patterns |
| NeoPixel | 21 | WS2812 RGB | Primary status indicator |

### Boot Sequence

| Phase | Red LED | NeoPixel | Duration |
|-------|---------|----------|----------|
| Power on | Off | Off | <100ms |
| FreeRTOS starting | Off | Off | <100ms |
| Normal operation | 1Hz heartbeat | Rainbow cycle | Continuous |

### Normal Operation

| State | Red LED | NeoPixel Color | NeoPixel Mode |
|-------|---------|----------------|---------------|
| Idle/Demo | 1Hz heartbeat | Rainbow | Rainbow cycle |
| Initializing | 1Hz heartbeat | Yellow | Solid |
| Ready | 1Hz heartbeat | Green | Solid |
| Armed | 1Hz heartbeat | Green | Breathe |
| Calibrating | 1Hz heartbeat | Cyan | Breathe |
| Recording | 1Hz heartbeat | Blue | Solid |

### Flight States (Future)

| State | NeoPixel Color | NeoPixel Mode |
|-------|----------------|---------------|
| Boost | White | Solid |
| Coast | Purple | Solid |
| Apogee | Magenta | Blink |
| Descent | Orange | Breathe |
| Landed | Green | Slow blink |

### Error States

| Error | Red LED | NeoPixel | Cause |
|-------|---------|----------|-------|
| Malloc failed | Fast blink | Red, fast blink | Heap exhausted |
| Stack overflow | Long-short-short | Orange, fast blink | Task stack exceeded |
| Sensor failure | Fast blink | Red, solid | IMU/Baro not responding |
| Storage error | 2Hz blink | Yellow, fast blink | Flash write failed |

### LED Pattern Reference

| Pattern | Timing | Use |
|---------|--------|-----|
| Heartbeat | 100ms on, 900ms off | Normal operation |
| Fast blink | 100ms on/off (5Hz) | Critical error |
| Breathe | Sinusoidal 10-100%, 2s cycle | Activity/waiting |
| Rainbow | HSV cycle, 6s period | Demo/idle |

### Color Reference

| Color | Hex | Meaning |
|-------|-----|---------|
| Red | `#400000` | Error, critical |
| Orange | `#402000` | Stack overflow, warning |
| Yellow | `#404000` | Initializing, caution |
| Green | `#004000` | Ready, nominal |
| Cyan | `#004040` | Calibrating, USB |
| Blue | `#000040` | Recording |
| Purple | `#200040` | Coast phase |
| Magenta | `#400040` | Apogee |
| White | `#404040` | Boost |

---

## 2. Serial Messages

### USB CDC Banner

```
=== RocketChip v0.1.1-neopixel ===
Build: Feb 02 2026 22:15:30
FreeRTOS: V11.1.0 (SMP)
Cores: 2

Blink task pinned to Core 0
UI task running on Core 1

Press Enter for status...
```

### Status Report (on keypress)

```
--- Status #1 ---
Uptime: 12345 ms
Blink stack HWM: 222 words free
UI stack HWM: 121 words free
Heap free: 58496 bytes
```

### Future CLI Messages

| Command | Response |
|---------|----------|
| `h` | Help menu |
| `s` | Sensor status |
| `c` | Calibration menu |
| `l` | Level calibration |
| `v` | Version info |

### Error Messages (Future)

| Code | Message | Severity |
|------|---------|----------|
| `E001` | IMU not detected | Critical |
| `E002` | Barometer not detected | Critical |
| `E003` | Storage init failed | Critical |
| `E004` | Calibration invalid | Warning |
| `E005` | Low battery | Warning |

---

## 3. MAVLink Status (Future)

### Heartbeat Status

| MAV_STATE | Meaning | LED |
|-----------|---------|-----|
| `MAV_STATE_UNINIT` | Booting | Yellow |
| `MAV_STATE_BOOT` | Initializing | Yellow |
| `MAV_STATE_CALIBRATING` | Calibration | Cyan |
| `MAV_STATE_STANDBY` | Ready | Green |
| `MAV_STATE_ACTIVE` | Armed/Flight | Green breathe |
| `MAV_STATE_CRITICAL` | Error | Red |
| `MAV_STATE_EMERGENCY` | Critical error | Red fast blink |

### Status Text Messages

| Severity | Prefix | Example |
|----------|--------|---------|
| Emergency | `[!]` | `[!] Stack overflow in SensorTask` |
| Critical | `[C]` | `[C] IMU communication lost` |
| Error | `[E]` | `[E] Calibration failed` |
| Warning | `[W]` | `[W] Compass interference detected` |
| Info | `[I]` | `[I] Calibration complete` |
| Debug | `[D]` | `[D] Sensor rate: 1000Hz` |

---

## 4. Audio Indicators (Future)

Reserved for piezo buzzer feedback.

| Event | Pattern | Notes |
|-------|---------|-------|
| Boot complete | Single beep | 100ms, 2kHz |
| Armed | Rising tone | 100ms 1kHz → 2kHz |
| Disarmed | Falling tone | 100ms 2kHz → 1kHz |
| Error | Triple beep | 3x 100ms, 1kHz |
| Low battery | Slow beeps | 500ms interval |

---

## 5. Implementation Status

| Component | Status | Location |
|-----------|--------|----------|
| WS2812 driver | ✅ Complete | `src/drivers/ws2812_status.c` |
| Red LED heartbeat | ✅ Complete | `src/main.cpp` |
| Error hook LEDs | ✅ Complete | `src/main.cpp` |
| USB CDC banner | ✅ Complete | `src/main.cpp` |
| Status report | ✅ Complete | `src/main.cpp` |
| CLI commands | ⬜ Pending | - |
| MAVLink status | ⬜ Pending | - |
| State machine integration | ⬜ Pending | - |
| Audio buzzer | ⬜ Pending | - |

---

*Last updated: 2026-02-02*
