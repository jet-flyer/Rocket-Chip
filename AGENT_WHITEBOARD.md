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

### 2026-02-05: PA1010D GPS Removed from I2C Bus — Needs Proper Driver at IVP-31

**Severity:** Deferred (not blocking Stage 2)
**Reporter:** Claude Code CLI

**Issue:** GPS module on shared Qwiic I2C bus causes bus interference when probed during bus scan. IMU gets ~17% read failures, baro gets 100% failures. Physically removed for Stage 2 work.

**Root cause:** PA1010D is a UART-over-I2C device that streams NMEA continuously after being probed. Full bus scan (0x08-0x77) triggers it.

**Fix needed at IVP-31:**
- Write proper GPS driver with 32-byte chunked reads (per Adafruit_GPS library pattern)
- Filter 0x0A padding bytes
- Time-slice GPS reads with IMU/baro in main loop
- Do NOT include GPS address in bus scan
- SDK 2.2.0 already has the SDA hold time fix (PR #273)

**See:** LL Entry 20, Pico SDK #252/#273

**Files affected:** `i2c_bus.c` (skip 0x10 in scan), `main.cpp` (GPS removed from expected list)

---

## Resolved

*Resolved items are erased. See LESSONS_LEARNED.md for historical debugging context.*
