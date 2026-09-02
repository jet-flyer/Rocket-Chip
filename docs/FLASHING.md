# Flashing

**Purpose:** How to put RocketChip firmware on a board and prove it is
running. This file is the source of truth. Scripts implement it;
`docs/agents/DEBUG_PROBE_NOTES.md` is inspect/debug, not a second flash
recipe.

---

## Hard rules

These are not optional. They come from the 2026-09 STEMMA sitting
(PA1010D / ICM-20948 / DPS310 on Feather QT, Analog AN-686).

1. **Never OpenOCD `program`.** `program` is `reset init` then write.
   `rp2350.cfg` has no SRST, so that reset is MCU-only SYSRESETREQ.
   STEMMA 3V3 stays up. The *old* image reboots, Core 1 starts I2C,
   then halt lands mid-byte and slaves latch (`I2C_IF_DIS` until a
   real 3V3 drop).
2. **Never `reset halt` / `reset run` / extra post-write reset.** An
   extra `reset halt` after write latched Hardware 10/13 (ICM and baro
   NACK, GPS still ACK). `reset run` can leave Core 1 at bootrom
   `0x000000da`.
3. **USB unplug is recovery when stuck, never a pass/fail step and
   never a counted boot.** It is a module power cycle (STEMMA 3V3
   drops). Do not unplug to "make the flash count."
4. **Do not flash unless CDC is live.** Park (`u`) needs a running
   image. No CDC → refuse. Flashing a live bus without park is the
   `program` failure mode.
5. **Hold DTR low** on any serial open used for park. pyserial pulses
   DTR on open/close by default; that USB-resets Core 0 into bootrom
   (`pc=0x1ea`) so `u` never holds.
6. **Score live `s` twice.** `I=` must rise. Banner Hardware 13/13 is
   init only. A freeze after a green banner is a fail.
7. **Operator never holds BOOTSEL** and never uses a 1200-baud serial
   poke. Last-resort button BOOTSEL is only if CDC is dead **and** the
   probe cannot attach (LL Entry 5).

---

## Why the probe path is this shape

| Piece | Why |
|---|---|
| CLI `u` park | ABORT-then-STOP, Core 1 off the bus, `tud_disconnect`, then WFI. TinyUSB ISRs live in flash; erase with USB IRQs armed kills CDC. |
| Halt both cores, no reset | Cores stopped. STEMMA slaves still powered, but the master is idle. |
| `flash write_image erase` + `verify_image` | Writes the ELF. Does not `reset init` first. |
| MSP/PC from `0x10000000`, `xpsr=0x01000000`, resume Core 0 | Vector-table start of the **new** image. `runtime_init` / `multicore_reset_core1` bring Core 1 up via PSM, not SYSRESETREQ. |
| Repo `openocd_cmsis_dap.cfg` | Session `reset_config none` (no SRST) and gdb-attach halt-only. Do not `cortex_m reset_config none` — this OpenOCD only accepts `sysresetreq`/`vectreset` there. Starting on stock `rp2350.cfg` reintroduces SYSRESETREQ on connect. |

Do not reconstruct this with GDB `load` + `monitor reset halt`. That is
the old recipe and it is wrong on this hardware.

---

## Iterative flash (debug probe) — default

Probe must be on the board you are flashing (typical: vehicle Feather).

### 1. Start OpenOCD once per session

From repo root:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/start_openocd_pico_sdk.ps1
```

Confirm `127.0.0.1:3333` (GDB) and `:4444` (telnet). The script kills
any existing `openocd.exe`, waits 2 s, and starts Pico SDK OpenOCD
(`%USERPROFILE%\.pico-sdk\openocd\0.12.0+dev\`) with
`openocd_cmsis_dap.cfg`. Do **not** start with
`interface/cmsis-dap.cfg` + `target/rp2350.cfg` alone.

Two OpenOCD installs exist. Chocolatey `openocd` has no RP2350 support.
Always the Pico SDK path. Details: `docs/agents/DEBUG_PROBE_NOTES.md`.

Kill OpenOCD before a user power cycle so the next attach is not a
stale DAP: `taskkill /F /IM openocd.exe`.

### 2. Build

```powershell
cmake --build build_flight
```

Station (probe usually not wired there):
`cmake --build build_station_flight`.

### 3. Flash

CDC must already be up (LED on, COM port present):

```powershell
python scripts/flash_elf_halt_write.py --elf build_flight/rocketchip.elf
```

`--dump` waits ~12 s then prints banner, Hardware `b`, and two `s`
samples. `--no-park` skips CLI `u` — only if the core is already in
WFI park. `--dump-only` is inspect, no write.

The script will **refuse** if no vehicle CDC is found. That is
intentional.

### 4. What the script does (do not re-type this by hand unless debugging the script)

1. Open the vehicle CDC with DTR held low. Send `q` then `u`. Close
   still holding DTR low.
2. OpenOCD telnet `:4444`:
   - `rp2350.cm0 cortex_m smp off`
   - halt Core 1, halt Core 0
   - `flash write_image erase <elf>`
   - `verify_image <elf>`
   - `read_memory 0x10000000 32 2` → `reg msp` / `reg pc`
   - `reg xpsr 0x01000000`
   - `resume` Core 0
3. Success text contains `verified`. Vector-resume **is** Boot 1 once
   CDC and LED are back. Do not follow it with another reset.

Attach stops the cores (LED off for the duration). That is halt, not
RP2350-E2. After resume, LED and CDC should return. If they do not:
kill OpenOCD first (it may still be holding halt), then VBUS + probe
unplug if still dead. Do not halt-to-inspect a missing banner and do
not write again until the LED is on.

### 5. Prove it ran

- LED on, CDC enumerates.
- Boot banner `Board:` matches the physical board.
- Hardware `b`: sensors the bench has should ACK. GPS PMTK is only a
  gate if the PA1010D is on the chain; UART GPS is `gps_uart.cpp` and
  is not this bus.
- `s` twice, a second or two apart: `I=` must increase. Stuck `I=`
  after a 13/13 banner is a fail.

`python scripts/bench_sim.py` is the land gate when firmware paths
changed. It talks to whatever image is already on the chip; flash
first.

---

## Picotool — station / no-SWD board

Use when the probe is on the other board (typical: Feather has SWD,
Fruit Jam does not). `--ser` is mandatory if more than one RP2350 is
plugged in.

```bash
picotool load -f --ser <chip-serial> <path-to-rocketchip.uf2>
```

`-f` talks to a **running** CDC device, issues a USB vendor reboot,
writes, returns to the app. That is a **warm MCU reset**. STEMMA 3V3
stays up. Firmware must bring the bus up without a cable pull.

Do **not** use `picotool reboot -f` or `reboot -a -f` for extra counted
boots (2026-08-20: CDC gone until a physical USB replug). Extra MCU
restarts on an idle bus: CLI `k` (ABORT-then-STOP then watchdog). Slave
POR: VBUS cycle. Close any serial monitor if picotool cannot find the
device.

`picotool info` without `-f` reads without rebooting.

---

## Counted boots (HW gate Rule 2)

See `standards/HW_GATE_DISCIPLINE.md` Rule 2.

| Action | What it is |
|---|---|
| Vector-resume after `flash_elf_halt_write.py` | Boot 1 once LED+CDC are up |
| CLI `k` | MCU restart, bus idle (ABORT-then-STOP then watchdog) |
| USB VBUS unplug/replug | Slave POR (3V3 drops). Recovery, not a gate step |
| `reset halt` / `program` / picotool `reboot -f` | Not a valid counted boot on STEMMA |

---

## Multi-board targeting by chip serial

When more than one RP2350 is connected, every `picotool` command uses
`--ser <chip-serial>`.

**The repo is the source of truth.** Grep `docs/` for the serial. A hit
that names station vs vehicle is authoritative.

Do not trust: agent memory, USB bus/address order, COM-port numbers.
If the mapping is not in the repo, ask the user.

Inspect a flashed UF2:

```bash
picotool info -f --ser <chip-serial> -a
```

That reports what is **in the UF2**, not whether it is on the correct
physical board. See verification below.

---

## Board / firmware verification

A binary that links, flashes, and boots can still have the wrong
`PICO_BOARD` / role. The firmware will drive the pins it was compiled
for. That is a Frankenstein board: banner looks fine, peripherals are
dead.

Run in order. Stop at the first fail.

**Pre-flight:** chip-serial ↔ board from the repo (above).

**Tier 1 — physical (authoritative):**

1. Status LED / NeoPixel lights at all. Completely dark from boot is a
   strong wrong-board signal (each board wires the LED to a different
   GPIO).
2. Role-specific peripherals:
   - **Station:** RSSI bar climbs, `Pkts:` advances, `Last:` stays low
     once the vehicle is transmitting.
   - **Vehicle:** `s` shows IMU/baro completing, not zeros. Core 1
     loop count advances. `I=` rises on a second sample.
3. Board-unique features only on their board (Fruit Jam HSTX/DVI;
   Feather does not).

**Tier 2 — binary ↔ serial (necessary, not sufficient):**

4. `picotool info -f --ser <serial>` `pico_board:` matches the repo.
5. Running banner `Board:` matches (4).

Tier 2 always passes for a correctly-built UF2 on the wrong chip.
**Tier 1 is the verdict.**

---

## Build-time prevention

1. Before a new `build_*` directory, copy the `cmake` invocation from a
   working build's `CMakeCache.txt`.
2. After configure:
   `grep -E "PICO_BOARD|ROCKETCHIP_JOB" build_xxx/CMakeCache.txt`.
   Single flight binary per role; test hooks are runtime-gated by
   `rc::test_mode_active()` (`docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md`).
3. After flash, read the banner before any soak.
4. Root `CMakeLists.txt` picks `PICO_BOARD` when
   `ROCKETCHIP_JOB_STATION=1` on a **fresh** configure, not a cached
   wrong one.
5. Monotonic build tag during long debug (`kBuildTag`); `__DATE__
   __TIME__` blurs. LL Entry 2.

---

## Troubleshooting

- **Park refused / no CDC:** Do not flash. Kill OpenOCD if cores may
  still be halted. If LED is off, resume is missing — not an automatic
  E2. If still dead: VBUS + probe unplug (recovery).
- **CDC dies during write:** TinyUSB ISRs are in flash. Park `u` must
  `tud_disconnect` first. DTR pulse on serial close will also kill the
  park. Use the script, not a hand `Serial()` open.
- **Hardware 10/13, ICM/baro NACK, GPS ACK:** MCU-only reset with
  STEMMA 3V3 up (`I2C_IF_DIS`). USB POR is the unwedge. Do not retry
  `reset halt` or `program`.
- **Core 0 in bootrom after "flash":** DTR pulsed, or `reset` was
  issued. Replug is recovery. Next flash: park with DTR held low.
- **Serial monitor blocks picotool:** Close the COM holder and retry.
- **GDB "Remote communication error" / errno 10061:** Pico SDK GDB
  (`…/toolchain/14_2_Rel1/bin/arm-none-eabi-gdb.exe`), not Chocolatey.
- **"Unable to find CMSIS-DAP device":** Stale OpenOCD.
  `taskkill /F /IM openocd.exe`, wait 2 s, run the start script.
- **New COM port after flash:** Normal on Windows. Do not use COM
  numbers as identity.
- **Unresponsive after picotool `-f` or halt-mid-spinlock (E2):** See
  `standards/RP2350_ERRATA.md` E2. Recovery is VBUS unplug (and often
  probe replug). A target-only warm reset will not clear SIO state.
  Prevention: prefer the probe path above; do not `picotool info -f`
  when you only needed `info`. Do not `monitor halt` inside
  `spin_lock_blocking`. Missing LED right after attach is usually
  still-halted cores — kill OpenOCD / resume, do not log E2 until
  that is ruled out.
- **Rescue-DP:** `RP_AP:CTRL` `RESCUE_RESTART` /
  `target/rp2040-rescue.cfg` is **not** our primary recovery. Replug
  first. Try Rescue-DP only if physical replug stops working, and log
  it on the E2 incident table.

---

## See also

- `scripts/flash_elf_halt_write.py` — implements the iterative path
- `scripts/start_openocd_pico_sdk.ps1` / `openocd_cmsis_dap.cfg`
- `docs/agents/DEBUG_PROBE_NOTES.md` — inspect, GDB, OpenOCD installs
- `standards/HW_GATE_DISCIPLINE.md` Rule 2 — counted boots
- `docs/BENCH_TEST_PROCEDURE.md` — soak; uses this file as the flash
  precondition
- `docs/agents/LESSONS_LEARNED.md` — historical flash failures (do not
  treat old `reset halt` recipes there as current procedure)
- `docs/deprecated/` — pre-rewrite snapshots of this file and
  `DEBUG_PROBE_NOTES.md` (`origin/main` `9f8aef4`). Not procedure.
- `standards/DEBUG_OUTPUT.md` — build tags, serial-terminal notes
