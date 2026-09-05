# LL Stale-Assumptions Audit — 2026-09-05

**Status:** Durable audit. Part A SUPERSEDE banners for LL 20 / 21 / 24
landed in `958f610` (LL-25 pattern). This file is the full disposition
table + Part B pick-list. Part B remains **PROPOSE ONLY** — no
banners until Nathan picks.

**Baseline tip before Part A:** `a277fa6`. **Part A commit:** `958f610`.
**Scope:** Append-only STATUS banners above historical bodies; no body
rewrites; no CHANGELOG; no push.

**LL 47 facts used for Part B:** Live 1 kHz `I=` freeze was FPV-twist of
STEMMA QT 4-core. Untwisted shortest Adafruit QT: GPS last and first
both `I=` climbing. Never twist SDA with SCL. Falsified as live-path
freeze root cause once untwisted: 100 kHz Standard Mode, skip-GPS-poll
(= unplug), OpenOCD `program` / extra reset halt, skip `reset_block`
attach, GPS-last.

---

## 1. Method

Read every `## Entry N` body in `docs/agents/LESSONS_LEARNED.md`
(archived FreeRTOS Entries 7—10, 14, 17—19 not re-scored). Checked
prescriptions against active `src/` (`i2c_master.*`, `gps_pa1010d.cpp`,
`icm20948.cpp`, `cal_hooks.*`), `docs/SENSOR_ARCHITECTURE.md`,
`docs/hardware/HARDWARE.md`, and standing WB rows on I2C / STEMMA /
GPS-on-bus / recover / kHz / cable / poll order / stretch.

Disposition: **KEEP** = still cite as standing truth; **SUPERSEDED** =
prescriptions wrong / replaced (banner applied or proposed);
**STALE-MECHANISM** = named mechanism gone from active path;
**REVISIT** = still useful but framing / API / open status needs Nathan.

---

## 2. Disposition table

| Entry | Title / topic | Disposition | Notes / pointer |
|------:|---------------|-------------|-----------------|
| 1 | Stack overflow from large locals | KEEP | General MCU lesson |
| 2 | Version strings / build tags | KEEP | Process |
| 3 | BASEPRI blocking USB IRQs | KEEP | Hardware/IRQ |
| 4 | Flash ops break USB | KEEP | Flash + USB class |
| 5 | Prefer debug probe before BOOTSEL | KEEP | Process |
| 6 | WS2812 requires `begin()` | KEEP | Driver contract |
| 11 | Prioritize debug probe over LED debug | KEEP | Process |
| 12 | USB CDC init order | KEEP | Enum order |
| 13 | ICM-20948 addr 0x69 not 0x68 | KEEP | Hardware fact |
| 15 | USB terminal affects program state | KEEP | Host CDC |
| 16 | PuTTY truncates initial CDC | KEEP | Tooling |
| 20 | PA1010D probed — interference; 32-byte chunks | **SUPERSEDED (Part A landed)** | Freeze — LL 47; Rx — `kGpsMaxRead=255`; residue: UART flight / I2C stress / no casual `0x10` scan / no I2C GPS until ICM bypass |
| 21 | ICM internal-master bank race; disable-master-during-cal | **SUPERSEDED (Part A landed)** | Permanent bypass; do not re-land cal disable-master |
| 22 | USB reconnect degrades Core 1 IMU | REVISIT (non-twist) | Open observation; API names legacy `i2c_bus_reset` |
| 23 | CLI I2C scan vs Core 1 ownership | KEEP | Flag still live |
| 24 | Never hot-path recover; 500 µs settle | **SUPERSEDED (Part A landed)** | Recover ban disproved; `kPostReadUs=2000`; live API `i2c_master_*` |
| 25 | Picotool corrupts I2C | SUPERSEDED (prior) | Pattern entry; 2026-04-22 |
| 26 | JSF AV 213 / clang-tidy parens | KEEP | Tooling |
| 27 | "Codegen sensitivity" was picotool corruption | REVISIT (non-twist) | Methodology KEEP; narrative vs SUPERSEDED LL 25 |
| 28 | `i2c_bus_recover` corrupts DW_apb | STALE-MECHANISM (non-twist) | Legacy-only symbol; live = `i2c_master_*` |
| 29 | ICM silent zero — ESKF divergence | KEEP | Health lesson |
| 30 | RP2350 XIP cache / `.time_critical` | KEEP | Chip fact |
| 31 | `flash_safe_execute` corrupts I2C + GPS lock | REVISIT (non-twist) | Mechanism live; API prescription legacy |
| 32 | Blocking drivers vs QV contract | KEEP | Scheduler |
| 33 | PIO GPIO init vs adjacent I2C pins | KEEP | Pinmux |
| 34 | PC fan — baro / ESKF P-growth | KEEP | Bench env |
| 35 | Stack-local QP events = UAF | KEEP | QP lifetime |
| 36 | Test artifact vs infrastructure | KEEP | Gate methodology |
| 37 | Rule-citation discipline | KEEP | Process |
| 38 | Code = current-state; datasheets = possibility | KEEP | Research |
| 39 | `rc_log` idle drain vs Core 1 I2C | KEEP | Fix in tree |
| 40 | Pre-commit gate: categories not enumerations | KEEP | Gate design |
| 41 | RP2350B pads start isolated | KEEP | Pad fact |
| 42 | PIO program lifecycle asymmetry | KEEP | PIO lifecycle |
| 43 | "Clean" static gate = negative evidence | KEEP | Process |
| 44 | Mass rename: clang-refactor AST | KEEP | Tooling |
| 45 | Worktree CHANGELOG not on main until merge | KEEP | Process |
| 46 | Post-flash extra restart / GDB attach = process not E2 | KEEP | Process/E2 class (see Part B vs I= freeze lore) |
| 47 | FPV-twist of STEMMA 4-core freezes I2C | KEEP | Current cable-dress authority |

**Entries reviewed:** 39 live bodies. Archived FreeRTOS set not scored.

---

## 3. Needs Nathan — Part B (LL 47 cable untwist)

**Do not banner these until Nathan picks.** Only Part A (LL 20/21/24)
got STATUS headers this commit.

| Entry / WB | What it claimed | How twist (LL 47) interacts | Recommend |
|------------|-----------------|-----------------------------|-----------|
| Agent lore / residual "GPS-last freezes live `I=`" (outside LL 47) | Chain order (GPS last) is the live-path freeze cause | Untwisted: GPS last **and** first both `I=` climbing. HARDWARE.md already prefers GPS-first for power/probe, not as freeze root | **SUPERSEDE** lore if still cited; docs already largely corrected — no LL banner unless a specific entry still asserts freeze-by-order |
| "Drop to 100 kHz" as I2C stability fix | Standard Mode should calm the bus | Separate sitting: 100 kHz was **worse**. Not a standing LL prescription, but agents may still propose it | **SUPERSEDE** as advice if proposed; no LL entry to banner unless one asserts it |
| "Skip GPS poll ≈ unplug" | Stopping polls clears GPS-on-bus pain | Falsified: powered `0x10` still enough to die; USB unplug is recovery not a pass | **SUPERSEDE** equivalence; KEEP physical-unplug evidence in LL 20 history |
| OpenOCD `program` / extra reset halt / skip `reset_block` attach as `I=` freeze remedy | Flash/attach process fixes freeze | Falsified as **freeze root cause** once twist known. LL 46 remains valid for E2 / counted-boot process (different class) | **KEEP** LL 46 for process/E2; **do not** cite as I2C freeze fix — optional STATUS note only if Nathan wants cross-link |
| Throttle / leave 1 kHz IMU rate | Rate reduction as bus-stability fix | LL 47: IMU ~0.8 mA vs GPS ~30 mA; throttling 1 kHz does not fix twist class | **KEEP** software rate choices for other reasons; **SUPERSEDE** as twist/freeze remedy |
| Soft error / settle physics (LL 24 body) vs hard freeze | ~8.4% IMU errors after GPS read; 500 µs settle | Twist is a **different class** (hard `I=` stick). Soft shared-bus contention can still be real on clean cables | **KEEP** soft-contention physics as residue (already in Part A banner); do not conflate with LL 47 |
| Stretch-aware wait / hot-path abort (`i2c_master`) | Software recovers timeouts / stretch | Twist can look like timeout storms; harness check first (LL 47 Prevention) | **KEEP** software; gate "rewrite the driver" on confirmed untwist |
| WB: I²C bus backend rework-eval (PIO / Flipper) | Fruit Jam GPS cold-boot / bus pain — consider PIO master | Trigger may mix cable dress with DW_apb pain. LL 47: do not start an I2C driver rewrite because `I=` froze until 4-cores confirmed **not** twisted | **REVISIT** trigger text — require harness/untwist check before blaming backend |
| WB: RP2350B/Fruit Jam persistent bus-corruption hypothesis | One boot not fully explained by "cable theory alone" | Twist strengthens cable-dress class for live freeze; chip hypothesis may still cover other intermittency | **REVISIT** priority / framing — keep passive unless a non-twist repro remains |
| FLASHING.md `I2C_IF_DIS` latch (SYSRESETREQ, STEMMA 3V3 up) | MCU-only reset latch mid-byte | Explicitly **different** class; untwist does not close it (LL 47 Related) | **KEEP** |

### Also open from full LL audit (non-twist — not Part B banners)

Carried from the propose-only pass; Nathan still picks before any
STATUS header:

1. **LL 22 — REVISIT** — open observation; rename `i2c_bus_reset` → abort/reattach in any STATUS, or leave historical.
2. **LL 27 — REVISIT** — keep methodology; clarify vs SUPERSEDED LL 25.
3. **LL 28 — STALE-MECHANISM** — recommended SUPERSEDED-style STATUS for legacy `i2c_bus_recover` symbol.
4. **LL 31 — REVISIT** — keep flash/`flash_safe_execute` lesson; STATUS rename API to `i2c_master_abort_and_idle` + `reattach`.

---

## 4. Part A apply record (`958f610`)

| Entry | Banner | Residue kept |
|------:|--------|--------------|
| 20 | SUPERSEDED 2026-09-05 | UART flight GPS; I2C PA1010D stress slave; no I2C GPS on IMU/baro until ICM bypass; no casual `0x10` scan |
| 21 | SUPERSEDED 2026-09-05 | Multi-bank / internal-master race class still real if master re-enabled |
| 24 | SUPERSEDED 2026-09-05 | UART-over-I2C settle need; UART preferred; soft shared-bus contention ≠ LL 47 freeze |
| 25 | Already SUPERSEDED 2026-04-22 | Pattern for banners |

WB row **"Audit all LESSONS_LEARNED entries for stale assumptions"**
erased (done).

---

## 5. Verification scraps

| Claim | Evidence |
|-------|----------|
| GPS read size 255, post-read 2 ms | `gps_pa1010d.cpp`: `kGpsMaxRead = 255`, `kPostReadUs = 2000` |
| Bypass active | `icm20948.cpp` `enable_bypass`; `cal_hooks` = mag read + reload |
| Live recover API | `i2c_master.cpp` `on_timeout()` / abort / reattach |
| Scan skips GPS addr | CLI / `kI2cAddrPa1010d` skip |
| Twist measured | LL 47; `docs/hardware/HARDWARE.md` Qwiic harness rule; `docs/FLASHING.md` troubleshooting |
| Hot-path recover ban disproved | WB seed 2026-09-01 (54% → 98.4% when removed) |

---

---

## 6. Residuals (out of LL banners)

- standards/VENDOR_GUIDELINES.md may still list 500 µs beside GlobalTop 2 ms
- One CLI PASS string may still say 500us settling delay

These are polish, not Part B banner candidates.

*End of durable audit. Part B candidates await Nathan.*
