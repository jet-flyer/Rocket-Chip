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
