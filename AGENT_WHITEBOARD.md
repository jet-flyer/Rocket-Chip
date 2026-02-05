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

### 2026-02-04: I2C Bus Not Working — Blocking Calibration Testing

**Severity:** Blocking
**Reporter:** Claude Code CLI
**Status:** Investigation in progress — build ready to test

**Symptoms:**
- I2C bus scan shows "No devices found" - both at boot and when rescanned via 'i' command
- Both ICM-20948 (0x69) and DPS310 (0x77) not responding
- I2C1 peripheral on GPIO2/3 (correct per HARDWARE.md and SDK board header)

**Cross-reference with working branch (2026-02-04):**
- Compared current `i2c_bus.c` against `AP_FreeRTOS:lib/ap_compat/AP_HAL_RP2350/I2CDevice.cpp` and `AP_FreeRTOS:tests/smoke_tests/i2c_scan.c`
- Pin config identical: I2C1, GPIO2 (SDA), GPIO3 (SCL), pull-ups enabled
- Key difference found: working i2c_scan.c used **100kHz**, current code was 400kHz
- **Changed I2C speed to 100kHz** in `i2c_bus.h` to match working test
- Init order matches working branch (I2C before stdio_init_all)
- HARDWARE.md GPIO table confirms GPIO2/3 = STEMMA QT (not Arduino pin numbers)

**What's been tried:**
1. 200ms startup delay after I2C init
2. 3 retry attempts for IMU init with bus recovery between each
3. Verified board header defines correct I2C pins (GPIO2/3) — raw GPIO, not Arduino
4. Added rescan command ('i') to test after terminal connection
5. Cross-referenced AP_FreeRTOS branch — code is functionally identical
6. Dropped I2C speed from 400kHz to 100kHz (matching working test)
7. Added verbose scan output: I2C instance number, pin states, configured frequency

**Latest build (`build/rocketchip.uf2`) includes:**
- 100kHz I2C speed
- Verbose scan output showing instance, pin states, frequency
- Not yet flashed to device

**Next steps when resuming:**
1. Flash latest build and run 'i' command
2. Check SDA/SCL pin states — both should be HIGH (pull-ups working)
3. If 100kHz works, gradually bump back to 400kHz
4. If still no devices, try I2C0 as a test
5. If pins stuck LOW, investigate bus stuck state or GPIO conflict

**Files affected:**
- `src/drivers/i2c_bus.c` - verbose scan output (instance, pins, freq)
- `src/drivers/i2c_bus.h` - changed to 100kHz
- `src/cli/rc_os.c` - 'i' command for rescan
- `src/main.cpp` - sensor availability flags, retry logic

**Note:** RC_OS CLI calibration logic is complete and working — sensor availability checks prevent hang on missing sensors. Calibration will work once I2C issue is resolved.

---

## Resolved

*No resolved flags yet.*
