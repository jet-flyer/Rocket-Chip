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

### 2026-02-08: F' Evaluation Complete — Architectural Decision Pending

**Status:** Research complete, awaiting user decision
**Reporter:** Claude Code CLI

Comprehensive F' evaluation added to `docs/decisions/TITAN_BOARD_ANALYSIS.md` (Sections 1-15 of the addendum). Section 13 added 2026-02-08: H7 board candidates — no maker H7 dev boards exist, Matek H743 flight controllers recommended for Path A. Three viable Titan paths identified:
- **Path A:** STM32H7 + F'/Zephyr (high risk — F' not running on H7 yet)
- **Path B:** Pi Zero 2 W + F'/Linux (medium risk — proven framework, new integration)
- **Path C:** Hybrid Pi Zero 2 W + RP2350 (recommended — mirrors Pixhawk architecture)

Key findings: F' multicore only works on Linux (pthreads). Zephyr has no Cortex-M SMP (blocked, no date). F' uses its own protocol, not MAVLink. Cherry-pick alternative exists (adopt F' patterns without framework).

**No action needed until Titan development begins.** Core/Main IVP progression unchanged.

---

### 2026-02-07: Stage 4 GPS — Ready for IVP-31 Implementation

**Status:** Ready — needs board plugged in with GPS on Qwiic chain
**Reporter:** Claude Code CLI

Stage 4 IVPs redesigned for dual-core architecture (GPS on Core 1). Driver updated (255-byte full-buffer reads, PMTK314 sentence filter). `main.cpp` is clean Stage 3 — no GPS code yet.

**Next steps:**
- Physically reconnect PA1010D to Qwiic chain
- Implement IVP-31 in `main.cpp` (GPS init before Core 1 launch, GPS reads on Core 1 sensor loop, seqlock publish)
- Standards review after build, before flash

**See:** `docs/IVP.md` IVP-31, `standards/VENDOR_GUIDELINES.md` PA1010D section

---

### 2026-02-07: Protected File Updates Pending Approval

**Status:** Partially resolved
**Reporter:** Claude Code CLI

- ~~`CODING_STANDARDS.md` file classification table — .c → .cpp (done 2026-02-08)~~
- `CODING_STANDARDS.md` — needs cross-reference to new `standards/VENDOR_GUIDELINES.md` in Prior Art Research section
- `SEQLOCK_DESIGN.md` line 69 — comment says "reserved for IVP-33", now IVP-31 (cosmetic)

---

### 2026-02-07: u-blox GPS Hardware Available — Future Flight GPS

**Status:** Noted for IVP-31+ planning
**Reporter:** Claude Code CLI

User has a **Matek M8Q-5883** on hand (SAM-M8Q u-blox + QMC5883L compass, UART GPS + I2C compass). Module is EOL. Current production alternative: **Matek M9N-5883** (NEO-M9N). The M10Q-5883 is also EOL.

**Key advantages over PA1010D:** UBX binary protocol (no snprintf/NMEA), dedicated UART bus (no I2C contention), external QMC5883L compass at 0x0D (no address conflict with ICM-20948/DPS310).

**Note:** IVP-31 continues with PA1010D on I2C intentionally — stress-testing I2C bus sharing is the goal. u-blox migration is for production/flight builds.

**Config path difference:** M8 uses legacy `UBX-CFG-MSG`; M10 uses `UBX-CFG-VALSET`. Same output parser (NAV-PVT). ArduPilot's `AP_GPS_UBLOX` handles both.

---

### 2026-02-08: D3 main.cpp Function Length Refactoring — Planned, Blocked on GPS

**Status:** Plan approved (council-reviewed), blocked on IVP-31 GPS completion
**Reporter:** Claude Code CLI

`main()` is ~992 lines, `core1_entry()` ~367 lines. Full decomposition plan in `~/.claude/plans/shimmying-snuggling-wilkes.md`. Council approved (Professor + ArduPilot): tick-function dispatcher pattern, `g_lastTickFunction` watchdog tracking, seqlock scope preservation. `.clang-tidy` config created with `readability-function-size` (60-line threshold) + `readability-identifier-naming`. LLVM install pending (needs admin elevation for `choco install llvm`).

**Phase A** (tooling): `.clang-tidy` created, LLVM install pending.
**Phase B** (refactoring): Starts after GPS work is stable.

---

### 2026-02-08: ArduPilot LED Patterns — Implement with State Machine (IVP-46)

**Status:** Future — implement when flight state machine is written
**Reporter:** Claude Code CLI

When implementing the flight state machine (IVP-46: Action Executor), map NeoPixel patterns to ArduPilot's standard LED codes. AP uses: solid green = GPS lock, yellow = no GPS, blue = armed, red blink = error/failsafe, etc. Current IVP-31 GPS NeoPixel (yellow blink = searching, green = fix) is an interim implementation — replace with full AP-style state-driven patterns at IVP-46.

**Reference:** ArduPilot `AP_Notify` / `NeoPixel.cpp` for pattern definitions.

---

### 2026-02-08: clang-tidy Standards Audit Integration — Back Burner

**Status:** Moderate priority, deferred until next standards audit cycle
**Reporter:** Claude Code CLI

clang-tidy can replace much of the manual `STANDARDS_AUDIT` process. Beyond the function-size and naming checks already configured, additional checks map to our standards: `readability-magic-numbers` (JSF 151), `google-runtime-int` (JSF 209 fixed-width types), `cppcoreguidelines-avoid-goto` (P10-1), `bugprone-*` and `cert-*` (MISRA-adjacent). The `.clang-tidy` config would become a machine-readable encoding of our standards, with `// NOLINT` for accepted deviations.

**Wait until:** Next scheduled audit. We just completed one (2026-02-07). Incrementally enable checks and validate against ARM cross-compile environment before enforcing.

---

### 2026-02-08: ICM-20948 I2C Bypass Mode Migration — Recommended

**Status:** Recommended by council before IVP-36 (ESKF)
**Reporter:** Claude Code CLI (IVP-31 council review)

Council unanimously recommended migrating ICM-20948 from I2C master mode to I2C bypass mode (ArduPilot's approach). Eliminates bank-switching race (LL Entry 21) permanently. Currently mitigated by disabling I2C master during calibration, but adding GPS traffic increases collision window. Not blocking for IVP-31, but should happen before ESKF integration.

See: ArduPilot `AP_InertialSensor_Invensensev2.cpp`, ICM-20948 datasheet `INT_PIN_CFG` register `I2C_BYPASS_EN` bit.

---

### 2026-02-07: Missing Vendor Datasheets

**Status:** Open — acquire before implementation needs them
**Reporter:** Claude Code CLI

| Document | Priority | Needed By |
|----------|----------|-----------|
| DPS310 datasheet (Infineon) | HIGH | IVP-36 (ESKF noise tuning) |
| PA1010D datasheet + app notes (GlobalTop/Quectel) | HIGH | IVP-31 (GPS integration) |
| RFM95W / SX1276 datasheet (Semtech) | MEDIUM | Stage 8 (telemetry) |

Source URLs in `standards/VENDOR_GUIDELINES.md` Datasheet Inventory section.

---

## Resolved

*Resolved items are erased. See LESSONS_LEARNED.md for historical debugging context.*
