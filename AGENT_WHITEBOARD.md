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

Comprehensive F' evaluation added to `docs/decisions/TITAN_BOARD_ANALYSIS.md` (Sections 1-14 of the addendum). Three viable Titan paths identified:
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

**Status:** Waiting on user
**Reporter:** Claude Code CLI

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
