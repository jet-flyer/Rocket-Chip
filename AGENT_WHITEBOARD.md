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

### 2026-02-08: Stage 4 GPS (IVP-31/32/33) — COMPLETE ✅

**Status:** Hardware-verified, committed
**Reporter:** Claude Code CLI

All Stage 4 IVPs complete. IVP-31 across 12 build iterations (gps-1 through gps-12c). IVP-32 GPS fix confirmed outdoors (PPS lock, lat/lon verified). IVP-33 CLI `s` command shows all GPS states. See LL Entry 24 for I2C contention data.

**NeoPixel green bug** deferred to IVP-46 state engine.

---

### 2026-02-08: BSS Layout Regression — UNSOLVED, DEFERRED

**Status:** Root cause unknown, does not block current work
**Reporter:** Claude Code CLI

Adding new static globals (especially with `std::atomic`) to `main.cpp` causes ~30-80% IMU I2C error rate. Investigated this session:

- **`alignas(64)` alone:** Not sufficient (82% errors with dummy atomic struct)
- **`alignas(64)` + flag grouping into `core_flags_t`:** Also caused errors (~14K reads clean, then degradation)
- **Baseline `ivp31-gps-12`:** Rock solid (52K+ reads, 0 errors, survives `b` key press)
- **Minimal change (build tag + printf format only):** Rock solid (68K+ reads, 0 errors)

**Ruled out:** SRAM bank contention via alignment (fix didn't help). Exclusive monitor granule collision (seqlock uses STL not LDREX/STREX — council review).

**Not yet tried:** (1) Move new globals to separate `.cpp` file, (2) `__attribute__((section))` BSS isolation, (3) `.map` file diff between working and broken, (4) BUSCTRL performance counters, (5) `objdump -d` comparison of generated code around seqlock read/write.

**Impact:** Does not block any current IVP. Will matter when Stage 5 (ESKF) needs new cross-core globals. Investigate then.

---

### 2026-02-07: Protected File Updates Pending Approval

**Status:** Partially resolved
**Reporter:** Claude Code CLI

- ~~`CODING_STANDARDS.md` file classification table — .c → .cpp (done 2026-02-08)~~
- `CODING_STANDARDS.md` — needs cross-reference to new `standards/VENDOR_GUIDELINES.md` in Prior Art Research section
- `SEQLOCK_DESIGN.md` line 69 — comment says "reserved for IVP-33", now IVP-31 (cosmetic)

---

### 2026-02-08: FeatherWing UART GPS Migration — Hardware on Hand

**Status:** Pending hardware soldering
**Reporter:** Claude Code CLI

User has **Adafruit Ultimate GPS FeatherWing (product 3133)** — PA1616D, UART-only, 99 channels, uFL external antenna connector. Eliminates I2C bus contention entirely (no settling delay needed). Key advantages over PA1010D:
- UART: dedicated bus, no I2C sharing with IMU/baro
- 99 channels vs 33: faster TTFF, full multi-constellation parallel search
- uFL connector: external antenna for better signal in airframe

**Implementation:** New `gps_uart.cpp` driver (UART read, same lwGPS parser). `g_gpsOnI2C` flag already in place — UART GPS init won't set it, delay auto-skipped.

**Blocked on:** User soldering the FeatherWing.

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

**Status:** Plan approved (council-reviewed), unblocked (IVP-31 GPS complete)
**Reporter:** Claude Code CLI

`main()` is ~992 lines, `core1_entry()` ~367 lines. Full decomposition plan in `~/.claude/plans/shimmying-snuggling-wilkes.md`. Council approved (Professor + ArduPilot): tick-function dispatcher pattern, `g_lastTickFunction` watchdog tracking, seqlock scope preservation. `.clang-tidy` config created with `readability-function-size` (60-line threshold) + `readability-identifier-naming`. LLVM install pending (needs admin elevation for `choco install llvm`).

**Phase A** (tooling): `.clang-tidy` created, LLVM install pending.
**Phase B** (refactoring): Starts after GPS work is stable.

---

### 2026-02-08: ArduPilot LED Patterns — Implement with State Machine (IVP-46)

**Status:** Future — implement when flight state machine is written
**Reporter:** Claude Code CLI

When implementing the flight state machine (IVP-46: Action Executor), map NeoPixel patterns to ArduPilot's standard LED codes. AP uses: solid green = GPS lock, yellow = no GPS, blue = armed, red blink = error/failsafe, etc. Current IVP-31 GPS NeoPixel (yellow blink = searching, green = fix) is an interim implementation — replace with full AP-style state-driven patterns at IVP-46.

**Known bug (gps-10):** NeoPixel does not transition to green when GPS PPS LED confirms fix. The `gps_valid` flag logic (GGA-primary + RMC + GSA) is correct, but the Core 1 NeoPixel update path doesn't trigger the green transition in practice. Root cause not fully diagnosed — likely a timing issue between `gps_valid` becoming true and the NeoPixel state check on the next sensor loop iteration. Deferred to IVP-46 state engine which will handle all LED state transitions centrally.

**Also cosmetic:** Brief cyan flash on boot before yellow blink. This is the Core 1 test dispatcher's initial state (line 573) before `kSkipVerifiedGates` advances to sensor phase (~1-2s). Not a bug.

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
