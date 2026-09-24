# Status Indicators and Messages

**Written:** 2026-02-02, FreeRTOS bring-up. Sections 2–5 below are that sketch.

**LEDs:** the tables that used to live in section 1 (green breathe = armed, white = boost, purple = coast) are not the firmware. Operator colors are `docs/USER_GUIDE.md`. Intent → pattern code is `docs/decisions/NOTIFY_CONTRACT.md`. Codes are `include/rocketchip/led_patterns.h`, drawn by `ao_led_engine.cpp`.

---

## 1. LED Indicators

See the operator card and the notify contract above. One NeoPixel chain per board (`kNeoPixelCount`), drawn by `AO_Notify` / `AO_LedEngine` on every role. Armed is red solid.

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

The MAV_STATE → LED map from 2026-02-02 is retired with section 1. Live colors are `docs/USER_GUIDE.md`.

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
