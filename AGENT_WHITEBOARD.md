# Agent Whiteboard

**Purpose:** Communication across context windows and between agents.

## Use Cases
1. **Cross-agent review** - Flag concerns about other agents' work (see `CROSS_AGENT_REVIEW.md`)
2. **Cross-context handoff** - Notes for future Claude sessions when context is lost
3. **Work-in-progress tracking** - Track incomplete tasks spanning multiple sessions
4. **Hardware decisions pending** - Flag items needing user input before code changes

**Last reviewed by Nathan**: *[DATE]*

---

## Open Flags

*No open flags.*

---

## Resolved

### 2026-02-05: I2C Bus Not Working — RESOLVED

**Root cause:** Two issues compounding:
1. **No bus recovery on boot.** Picotool `--force` reboots leave sensors mid-I2C-transaction with SDA held low. Without recovery, no devices respond on next boot.
2. **Stage 2 init code corrupting bus.** IMU init (bank switching, mag I2C master setup), baro init, and flash ops during calibration storage init were interfering with bus state, preventing GPS (0x10) from being detected.

**Fix:** Added `i2c_bus_recover()` call before `i2c_init()` in `i2c_bus_init()`. Stripped Stage 2 back to clean Stage 1 baseline. All 3 devices (0x69, 0x77, 0x10) now detected reliably at 400kHz.

**Lesson:** 100kHz was a red herring — 400kHz works fine. The problem was always bus state corruption, not speed.
