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

### 2026-02-06: Stage 3 Session Plan (Dual-Core Integration)

**Status:** Planned — starting next
**Reporter:** Claude Code CLI (reviewed with Nathan)

Stage 3 (IVP-19 through IVP-30) grouped into work sessions. Multicore before GPS confirmed as correct order.

| Session | IVP Steps | Focus | Notes |
|---------|-----------|-------|-------|
| **A** | IVP-19, IVP-20 | Core 1 alive + atomic counter | Get dual-core running, verify cross-core visibility |
| **B** | IVP-21, IVP-22, IVP-23 | Spinlock, FIFO, doorbell | Exercise all RP2350 inter-core primitives |
| **C** | IVP-24 | Seqlock (single-buffer) | **Design doc approved:** `docs/decisions/SEQLOCK_DESIGN.md`. Council unanimous with 7 modifications (all incorporated). |
| **D** | IVP-25, IVP-26 | IMU + baro on Core 1 | Real sensor migration. Uses I2C timing from Stage 2 (IMU 774us, baro 251us) |
| **E** | IVP-27, IVP-28 | USB stress + flash under dual-core | Stability. Note: `multicore_lockout` uses FIFO — can't overlap with app FIFO messages |
| **F** | IVP-29, IVP-30 | MPU stack guard + watchdog | Safety nets |

**Key constraints (updated with research findings 2026-02-06):**
- GPS module physically removed — reconnect at IVP-31 (Stage 4)
- **Seqlock design decided:** Single-buffer seqlock, 124-byte struct (council added `mag_read_count` + `core1_loop_count`), Core 1 applies calibration. See `docs/decisions/SEQLOCK_DESIGN.md`.
- **RP2350-E2 (not E17):** SIO register aliasing breaks HW spinlocks. SDK uses SW spinlocks by default (`PICO_USE_SW_SPIN_LOCKS=1`). Transparent to API users.
- **FIFO is reserved:** `multicore_lockout`/`flash_safe_execute` claims FIFO IRQ exclusively. IVP-22 is exercise-only — FIFO cannot be used for app messaging in final architecture.
- **Polling > doorbells:** Core 0 at 200Hz always finds fresh data. Seqlock sequence check costs 5-7 cycles. Doorbells deferred to sleep-based power architecture (future).
- **SRAM only for shared data:** PSRAM has XIP cache coherency issues + exclusive monitor doesn't cover it. C11 atomics on PSRAM silently fail cross-core.
- I2C master disable/enable hooks (LL Entry 21) carry into Core 1 calibration flow
- **SAD Section 4.3 seqlock code has bugs** — missing `__dmb()` barriers, unnecessary double-buffer, wrong errata ID. Do not copy verbatim. Use `SEQLOCK_DESIGN.md` as reference.

---

### 2026-02-06: Full JSF AV Standards Audit Needed Post-Stage 3

**Severity:** Medium (technical debt)
**Reporter:** Claude Code CLI

**Issue:** Current "standards audit" checks ~10 critical rules (naming, types, memory, USB guards, magic numbers, atomics, cross-core invariants). JSF AV has 222 rules, JPL C standard adds more. Many rules are unchecked:
- Cyclomatic complexity limits
- Function length limits
- Switch/case fall-through analysis
- Const correctness
- Pointer aliasing rules
- Null checks on all pointer dereferences
- Header include guard naming conventions
- Comment-to-code ratios
- And ~200 more

**Recommended:** Dedicate a full session between Stage 3 and Stage 4 to:
1. Create a JSF AV compliance checklist document (`standards/JSF_AV_CHECKLIST.md`)
2. Run a systematic audit against ALL applicable rules
3. Log findings and prioritize fixes
4. Update `STANDARDS_DEVIATIONS.md` with any accepted deviations

**Scope:** All `src/` files (~2500 lines across 12 source files as of Session D).

---

## Resolved

*Resolved items are erased. See LESSONS_LEARNED.md for historical debugging context.*
