# Board / Firmware Verification

**Purpose:** Confirm that the firmware currently running on a physical
board was built for that board. Independent of flashing method (picotool,
debug probe, UF2 copy, DFU, etc.) — this is about identity, not transport.

**When to use:**
- After any flash, before trusting bench results
- When a peripheral "silently fails" with no error (dead radio, dead
  sensor, dark LED) — first triage step before blaming hardware
- When a previously-working board starts misbehaving after someone
  else touched it

## The failure mode this prevents

A binary that compiles cleanly, flashes cleanly, boots without asserts,
and runs Core 0 normally can still have **the wrong hardware
abstraction** if the build was configured with the wrong `PICO_BOARD`,
wrong role flags, wrong mission profile, etc. The firmware doesn't know
it's on the wrong board — it tries to drive GPIO 11 as `RADIO_RST`
because that's what `kRadioRstPin` was set to at compile time. The
peripheral silently fails, the AO that owns it silently gives up, and
downstream code runs happily because it doesn't depend on that AO
succeeding.

This is the **Frankenstein Build** anti-pattern.

### Why the obvious checks don't catch it

The temptation is to verify by reading the firmware banner's `Board:`
line, or by running `picotool info --ser <serial>` and checking the
`pico_board:` field. **Both only report what is baked into the UF2.**
They do not prove the UF2 is running on the board it was compiled for.
A Frankenstein board prints its intended board name perfectly while
every peripheral silently fails.

The self-reinforcing trap: verifying "serial ↔ binary" consistency
always passes for a correctly-built UF2, regardless of which physical
chip received it. The thing you actually care about — "serial ↔
physical board" — requires observing the hardware, not the firmware's
self-report.

## Verification Checklist

Run in order. **Stop and investigate** at the first failing check —
downstream checks assume earlier ones passed.

### Pre-flight: chip-serial ↔ board mapping

- Source of truth for "which chip serial is which physical board" is
  the **repo docs**. Grep for the serial in `docs/` to find its
  documented role (e.g., `grep -rn BEC71B8E docs/`).
- Do not pull the mapping from agent memory or from assumptions about
  USB bus/address ordering. Every historical Frankenstein-flash incident
  on this project traces back to memory-held identity claims that were
  never verified against repo docs.
- If the repo doesn't document the mapping, **ask the user** before
  flashing. Do not guess.

### Tier 1 — physical observables (authoritative)

These require the firmware's compile-time pin/peripheral config to
actually match the hardware under it. A wrong-board flash fails these.

1. **Status LED / NeoPixel lights up at all.** Each board wires the
   status LED to a different GPIO; wrong-board firmware drives a pin
   that isn't connected or is used for something else. Completely dark
   (not red, not dim) status LED from boot = strong Frankenstein
   signal.

2. **Role-specific peripherals function:**

   - **Station:** RSSI bar climbs out of its "dead" state once the
     vehicle is transmitting. `Pkts:` counter advances. `Last:` stays
     low (seconds, not minutes). A dead radio is the canonical symptom
     of station firmware on vehicle hardware (wrong SPI/CS/IRQ pins for
     the RFM95W).
   - **Vehicle:** I2C sensors return plausible data. `s` CLI output
     shows IMU/baro reads completing, not zeros or "not found". Core 1
     loop count advances.

3. **Board-unique features only light up on their board.** Fruit Jam
   has HSTX/DVI output; Feather doesn't. If a Feather binary ends up on
   Fruit Jam hardware, HSTX init won't produce output on a connected
   screen.

### Tier 2 — binary ↔ serial consistency (necessary but not sufficient)

4. `picotool info -f --ser <serial>` — check the `pico_board:` line
   matches the board the repo documents that serial to be.
5. Read the running firmware banner — the `Board:` line should match
   (4).

Tier 2 alone is the tautology to avoid. A Frankenstein board passes
Tier 2 cleanly while failing Tier 1. **Tier 2 is corroborating
evidence; Tier 1 is the verdict.**

## Build-time prevention (separate from post-flash verification)

The checklist above catches mistakes after they reach hardware. These
practices prevent the mistake at build time:

1. Before creating a new `build_*` directory, copy the `cmake`
   invocation from the closest working build's `CMakeCache.txt` — do
   not assume defaults are right.
2. After `cmake -B build_xxx`, run
   `grep -E "PICO_BOARD|ROCKETCHIP_JOB" build_xxx/CMakeCache.txt`
   and confirm the values match the target role. (`NOT_CERTIFIED_FOR_FLIGHT`
   was retired by R-25-exec 2026-05-13; single flight binary per role.)
3. After flashing, read the boot banner (or GDB-print the build tag
   and board name symbols) before starting any soak.
4. The root `CMakeLists.txt` auto-picks the correct `PICO_BOARD`
   default when `ROCKETCHIP_JOB_STATION=1` is set (as of 2026-04-22) —
   but this only protects fresh configures, not cached builds with
   previously-wrong values.

## See also

- `docs/BENCH_TEST_PROCEDURE.md` — soak-test procedure (uses this
  checklist as a precondition before soak results are trusted)
- `.claude/LESSONS_LEARNED.md` Entry 36 — honor-system gates and the
  "infrastructure vs artifact" framing
- `.claude/DEBUG_PROBE_NOTES.md` — debug probe flash transport (one of
  several valid transports; orthogonal to this checklist)
