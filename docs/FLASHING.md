# Flashing

**Purpose:** Single reference for how to flash RocketChip firmware onto a
physical board and confirm it landed correctly. Covers method choice
(picotool vs debug probe), multi-board targeting by chip serial, and
post-flash verification.

---

## Methods — picotool vs debug probe

Both methods produce the same outcome when they succeed. Choose by
situation.

### Debug probe (SWD) — preferred for iterative development

```bash
# Start OpenOCD once per session (idempotent):
taskkill //F //IM openocd.exe 2>/dev/null; sleep 2; \
  /c/Users/pow-w/.pico-sdk/openocd/0.12.0+dev/openocd \
  -s /c/Users/pow-w/.pico-sdk/openocd/0.12.0+dev/scripts \
  -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "adapter speed 5000" &

# Flash and run (each iteration):
/c/Users/pow-w/.pico-sdk/toolchain/14_2_Rel1/bin/arm-none-eabi-gdb.exe \
  build/rocketchip.elf -batch \
  -ex "target extended-remote localhost:3333" \
  -ex "monitor reset halt" \
  -ex "load" \
  -ex "monitor resume"
```

**Why preferred for iteration:** SWD halts both cores cleanly before
`load` — no in-progress I2C/SPI/flash operations get interrupted
mid-transaction. Post-flash state is deterministic; no need for power
cycles between attempts.

**Caveats:**
- Use `monitor resume` (not `monitor reset run`) for dual-core targets
  in batch mode. `reset run` can leave Core 1 stuck at bootrom `0x000000da`
  while Core 0 waits for a cross-core flag. See `docs/agents/DEBUG_PROBE_NOTES.md`.
- Probe physical wiring is limited to whichever board has SWD pins
  connected. See the project's hardware notes (e.g., repo-documented
  probe-to-board mapping) to know which board the probe currently reaches.
- There are two OpenOCD installations on this system; **always use the
  Pico SDK path** (`/c/Users/pow-w/.pico-sdk/openocd/0.12.0+dev/`). The
  Chocolatey one lacks RP2350 support.

### picotool — preferred for single-shot and multi-board targeting

```bash
# Flash a specific board by chip serial (reboots → flashes → runs):
picotool load -f --ser <chip-serial> <path-to-rocketchip.uf2>
```

The `-f` flag does the USB-vendor reboot into BOOTSEL automatically; no
manual BOOTSEL dance, no separate 1200-baud poke. `--ser <chip-serial>`
selects which connected device to target when multiple RP2350s are
plugged in.

**Why preferred for multi-board:** `--ser` gives explicit targeting by
ROM-burned serial, independent of USB enumeration order or bus/address
changes after reboot.

**Caveats — iterative flashing:**
- Historically, rapid back-to-back `picotool -f` cycles could cumulatively
  corrupt the I2C bus via interrupted transactions. That failure mode is
  fixed in-tree (LL Entries 25, 27, 28, 31 — `i2c_bus_recover()` now
  deinits/reinits the peripheral; `flash_safe_execute()` callers reset
  I2C afterward). In current code, picotool for iteration is fine; the
  probe is still nicer because it halts cleanly rather than mid-reboot.
- If `picotool` can't find the device, close any open serial monitor on
  the target's COM port and retry.

### Don't use the 1200-baud BOOTSEL poke

`picotool load -f` already handles the reboot. A separate
`serial.Serial(port, baudrate=1200)` open-and-close before the flash
adds nothing and creates failure modes (OSError when the port
disappears mid-open, stuck-BOOTSEL state if picotool isn't called
immediately after).

---

## Multi-board targeting by chip serial

When multiple RocketChip RP2350s are connected, every `picotool` command
should use `--ser <chip-serial>` to target the specific physical board.

### Where chip-serial ↔ board mappings live

**The repo is the source of truth.** Grep the repo for the serial to
find its documented role:

```bash
grep -rn <chip-serial> docs/
```

A hit like `docs/plans/STAGE_T_FIX_PLAN.md:339:... --ser BEC71B8EDC6AEBD1`
is authoritative — the doc names it as station, vehicle, or whatever
role.

### Do not trust

- Agent memory for chip-serial ↔ board mappings. Every historical
  Frankenstein-flash incident on this project traces back to a
  memory-held serial mapping that was never verified against repo docs.
- USB bus/address ordering. These shift across reboots and
  re-enumeration events.
- COM-port numbers as identity proxies. Even if the user's current
  bench has stable COM assignments, that's a local convention, not an
  authoritative identity source.

### If the mapping isn't in repo docs

Ask the user. Do not guess from bus/address, memory, or other
ephemeral state.

### Inspect a specific board's binary

```bash
picotool info -f --ser <chip-serial> -a
```

Shows `pico_board:`, build date, binary range, and metadata blocks of
the currently-flashed UF2. Useful for cross-checking before a flash.

---

## Board / Firmware Verification

**Purpose:** Confirm the firmware running on a physical board was built
for that board. Independent of flashing method.

### The failure mode this prevents (Frankenstein builds)

A binary that compiles, links, flashes, and boots without asserts —
with Core 0 running its QV loop normally — can still have **the wrong
hardware abstraction** if the build was configured with the wrong
`PICO_BOARD`, wrong role flags, wrong mission profile, etc. The
firmware doesn't know it's on the wrong board: it drives GPIO 11 as
`RADIO_RST` because that's what `kRadioRstPin` was set to at compile
time. The peripheral silently fails, the AO that owns it silently
gives up, and downstream code runs happily because it doesn't depend
on that AO succeeding.

### Why banner/picotool checks don't catch it alone

Both `picotool info --ser <serial>` and the running firmware banner's
`Board:` line only report what's *baked into the UF2*. They do not
prove the UF2 is running on the board it was compiled for. A
Frankenstein board prints its intended board name perfectly while every
peripheral silently fails.

The self-reinforcing trap: verifying "serial ↔ binary" consistency
always passes for a correctly-built UF2, regardless of which physical
chip received it. The thing you actually care about — "serial ↔
physical board" — requires observing the hardware, not the firmware's
self-report.

### Verification Checklist

Run in order. **Stop and investigate** at the first failing check —
downstream checks assume earlier ones passed.

**Pre-flight: chip-serial ↔ board mapping**

- Source of truth is the repo. See "Multi-board targeting by chip
  serial" above. Ask the user if docs don't cover it. Never trust
  agent memory.

**Tier 1 — physical observables (authoritative):**

1. **Status LED / NeoPixel lights up at all.** Each board wires the
   status LED to a different GPIO; wrong-board firmware drives a pin
   that isn't connected or is used for something else. Completely dark
   (not red, not dim) status LED from boot = strong Frankenstein
   signal.
2. **Role-specific peripherals function:**
   - **Station:** RSSI bar climbs out of "dead" state once the vehicle
     is transmitting. `Pkts:` counter advances. `Last:` stays low
     (seconds, not minutes). A dead radio is the canonical symptom of
     station firmware on vehicle hardware (wrong SPI/CS/IRQ pins for
     the RFM95W).
   - **Vehicle:** I2C sensors return plausible data. `s` CLI output
     shows IMU/baro reads completing, not zeros or "not found". Core 1
     loop count advances.
3. **Board-unique features only light up on their board.** Fruit Jam
   has HSTX/DVI output; Feather doesn't. If a Feather binary ends up
   on Fruit Jam hardware, HSTX init won't produce output on a
   connected screen.

Tier 1 requires the firmware's compile-time pin/peripheral config to
actually match the hardware under it. A wrong-board flash fails these.

**Tier 2 — binary ↔ serial consistency (necessary but not sufficient):**

4. `picotool info -f --ser <serial>` — check the `pico_board:` line
   matches the board the repo documents that serial to be.
5. Read the running firmware banner — `Board:` line should match (4).

Tier 2 alone is the tautology to avoid. A Frankenstein board passes
Tier 2 cleanly while failing Tier 1. **Tier 2 is corroborating
evidence; Tier 1 is the verdict.**

---

## Build-time prevention

The verification checklist catches mistakes after they reach hardware.
These practices prevent the mistake at build time:

1. Before creating a new `build_*` directory, copy the `cmake`
   invocation from the closest working build's `CMakeCache.txt` — do
   not assume defaults are right.
2. After `cmake -B build_xxx`, run
   `grep -E "PICO_BOARD|ROCKETCHIP_JOB" build_xxx/CMakeCache.txt`
   and confirm values match the target role. (Note: `NOT_CERTIFIED_FOR_FLIGHT`
   was retired in R-25-exec 2026-05-13; the project produces a single
   flight binary per role with test affordances runtime-gated by
   `rc::test_mode_active()`. See
   `docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md`.)
3. After flashing, read the boot banner (or GDB-print the build tag
   and board name symbols) before starting any soak.
4. The root `CMakeLists.txt` auto-picks the correct `PICO_BOARD`
   default when `ROCKETCHIP_JOB_STATION=1` is set (as of 2026-04-22) —
   but this only protects fresh configures, not cached builds with
   previously-wrong values.
5. Use a **monotonic build iteration tag** during extended debug
   sessions (e.g., `kBuildTag = "IVP30-fix-3"`, incremented on every
   rebuild). `__DATE__ __TIME__` alone blurs together during rapid
   rebuilds and the same binary flashed twice looks identical. Always
   verify the tag in serial output before starting a test cycle. See
   `docs/agents/LESSONS_LEARNED.md` Entry 2.

---

## Troubleshooting

- **Serial monitor blocks picotool:** If picotool can't find the device,
  close any serial monitor / COM port holder on the target port and
  retry. COM port locks prevent re-enumeration.
- **GDB "Remote communication error":** Use the Pico SDK's GDB
  (`/c/Users/pow-w/.pico-sdk/toolchain/14_2_Rel1/bin/arm-none-eabi-gdb.exe`),
  not Chocolatey's. Version-mismatch issue. Details:
  `docs/agents/DEBUG_PROBE_NOTES.md`.
- **"Unable to find CMSIS-DAP device":** Stale OpenOCD processes
  holding the probe's USB device. `taskkill //F //IM openocd.exe`,
  wait 2s, restart with full path. Details:
  `docs/agents/DEBUG_PROBE_NOTES.md`.
- **USB enumerates to a new COM port after flash:** Normal on Windows
  after re-enumeration. Don't use COM numbers as identity — identify
  boards by chip serial or firmware banner content per the checklist
  above.
- **Board becomes unresponsive after `picotool -f` or SWD halt-mid-spinlock (E2 signature):**
  Both `picotool load -f` / `picotool info -f` (via USB vendor
  reboot) and OpenOCD `monitor halt` during a spinlock wait can
  produce the same RP2350-E2 spinlock deadlock. The
  `PICO_SW_SPIN_LOCKS_NO_EXTEXCLALL=1` workaround doesn't cover these
  paths (see `standards/RP2350_ERRATA.md` E2 row, R-1 / R-2).
  **Underlying cause** (best current understanding): SIO block state
  and/or exclusive-monitor state persists across the warm reboot
  issued by either tool, and SIO isn't reset by the warm boot path.
  The probe itself isn't the bug — it's that the probe (or picotool)
  is what *issued* the warm reboot. Any warm-reboot path would hit
  the same silicon gap.

  **Recovery:** unplug the board VBUS. Whether the probe also needs
  to be replugged depends on whether OpenOCD's DAP session got stuck
  — often it does because the target vanished mid-session, so replug
  both as the reliable option. A target-only reset or a probe-only
  replug will NOT clear it — the SIO state survives warm reboot and
  only a full board power cycle brings it back.

  **Prevention:** avoid warm-reboot-inducing actions when you don't
  need them. For running devices, `picotool info` (no `-f`) reads
  what it can without rebooting. For flashing, prefer the debug probe
  (SWD `load` is cleaner than picotool `-f`). For debugging, avoid
  `monitor halt` while the target is in a `spin_lock_blocking` wait
  — use breakpoints past the spinlock acquisition instead, or
  `monitor reset halt` to re-enter from a known state.

  **Rescue-DP (not our recovery path — noted for completeness).**
  RP2350 provides a dedicated recovery Access Port (`RP_AP:CTRL`
  bit 31 `RESCUE_RESTART`) intended for DAP-stuck scenarios.
  Raspberry Pi's OpenOCD fork exposes it via `target/rp2040-rescue.cfg`
  (same file used for RP2350). We do not use it as a primary recovery
  step because physical replug reliably clears our observed R-1/R-2
  symptoms and Rescue-DP's applicability to them is unverified
  (it targets DAP state, not the application-level peripheral state
  we suspect is involved). If physical replug ever *stops* reliably
  clearing an incident, Rescue-DP is worth trying as a next step —
  log the attempt and outcome in `standards/RP2350_ERRATA.md` E2
  incident log.

---

## See also

- `docs/agents/DEBUG_PROBE_NOTES.md` — OpenOCD startup, GDB commands, debug
  probe troubleshooting beyond flashing
- `docs/BENCH_TEST_PROCEDURE.md` — soak-test procedure; uses this
  flashing doc + verification checklist as preconditions
- `docs/agents/LESSONS_LEARNED.md` Entries 25, 27, 28, 31, 36 — historical
  flash-related failure modes and their fixes
- `standards/DEBUG_OUTPUT.md` — build iteration tags, serial-terminal
  compatibility notes
