# Debug Probe Usage Notes

Inspect and debug with the Adafruit SWD probe. **Flashing is
`docs/FLASHING.md`.** Do not copy a GDB `load` / `reset halt` recipe
from this file. Pre-rewrite snapshot: `docs/deprecated/DEBUG_PROBE_NOTES.md`.

## Probe first

Use the probe before LED-only debugging when USB is dead, the chip
hardfaulted, or output never started. The probe can halt and dump
registers even with CDC down. Do not ask for a BOOTSEL button unless
CDC is dead **and** the probe cannot attach (LL Entry 5).

LED watching is fine for runtime-without-breakpoints and for production
images that will not have a probe. Prefer serial diagnostics over LED
patterns when CDC is up.

## Two OpenOCD installs

| Version | Path | RP2350 |
|---|---|---|
| Chocolatey | `C:/ProgramData/chocolatey/lib/openocd/` | No (0.12.0, 2023) |
| Pico SDK | `%USERPROFILE%/.pico-sdk/openocd/0.12.0+dev/` | Yes (0.12.0+dev) |

`openocd` on PATH is the Chocolatey build. Always the Pico SDK binary.
Same split for GDB: Pico SDK `14_2_Rel1`, not Chocolatey
`arm-none-eabi-gdb` (errno 10061 against this OpenOCD).

## Start OpenOCD

From repo root, every time:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/start_openocd_pico_sdk.ps1
```

Idempotent: kills every `openocd.exe`, waits 2 s, starts Pico SDK
OpenOCD with `openocd_cmsis_dap.cfg` (halt-only gdb-attach, no
SYSRESETREQ). Confirm `127.0.0.1:3333` and `:4444`.

Do **not** start with `interface/cmsis-dap.cfg` + `target/rp2350.cfg`
alone — that default SYSRESETREQs on connect. See `docs/FLASHING.md`.

When finished: `taskkill /F /IM openocd.exe`. Kill it before a user
power cycle so the next attach is not a stale DAP.

`Start-Process -ArgumentList` in PowerShell splits `-c "adapter speed
5000"`. Use the `.ps1`; it passes one `.Arguments` string.

### Pre-commit `bench_sim`

Firmware-path commits need OpenOCD on `:3333` **and** vehicle CDC so
`python scripts/bench_sim.py` can find the board. The hook does not
flash; it talks to whatever image is already on the chip. Flash first
(`docs/FLASHING.md`), then commit. `--no-verify` needs explicit
repo-owner approval in this session.

## Flash

```powershell
python scripts/flash_elf_halt_write.py --elf build_flight/rocketchip.elf
```

Park + halt + `write_image` + vector resume. Never `program`. Never
`reset halt` after write. Full rules: `docs/FLASHING.md`.

Attach stops the cores (LED off until resume). That is halt, not E2.

## Inspect (halt, no reset)

```bash
/c/Users/pow-w/.pico-sdk/toolchain/14_2_Rel1/bin/arm-none-eabi-gdb.exe \
  build_flight/rocketchip.elf -batch \
  -ex "target extended-remote localhost:3333" \
  -ex "monitor halt" -ex "bt"
```

Same pattern for `print`, `info threads`, `info locals`. Do not
`monitor reset halt` with STEMMA 3V3 up. After GDB disconnects, halt
again if you need control — still no reset.

Prefer waiting for CDC to inspect a boot that has not enumerated yet.
Halting a missing banner is part of a stuck flash process, not a
firmware dump.

`resume`, never `reset run`. `reset run` can leave Core 1 at bootrom
`0x000000da` while Core 0 waits on a cross-core flag.

## USB IRQs vs flash erase

TinyUSB handlers live in flash. Erase/write with USB IRQs armed breaks
CDC. The park path calls `tud_disconnect` before WFI so that cannot
happen. Do not "fix" this by doing flash work before `stdio_init_all`
on a running board — that is a different problem (boot-time storage
init). Iterative SWD flash uses park.

## Known issues

1. **Multiline bash + GDB.** `\` continuations are unreliable. One line
   or a `.gdb` script.
2. **GDB batch timeouts.** Short timeouts; batch mode can hang on a
   breakpoint that never hits.
3. **Wrong OpenOCD / GDB.** Chocolatey OpenOCD has no `rp2350.cfg`.
   Chocolatey GDB vs Pico SDK OpenOCD → errno 10061 even when `:3333`
   is listening.
4. **CMSIS-DAP not found.** Stale `openocd.exe` holding the probe.
   `taskkill /F /IM openocd.exe`, wait 2 s, run the start script.
   `pkill` is not in Windows Git Bash.
5. **USB I/O error, PC=0x00000000.** Kill OpenOCD, wait, restart via
   the script, `monitor halt` (not `reset halt`). If still dead: VBUS
   + probe unplug. USB unplug is recovery.
6. **Dual USB.** Target CDC and the probe are separate host
   connections. A crashed target USB stack can still make the probe
   report I/O errors. Power-cycle the target.

## Probe before printf

Printf changes timing, can block on the CDC mutex, and can hide the
bug. Halt and backtrace first.

A monotonic build tag in the banner beats `__DATE__ __TIME__` during
rapid rebuilds (`docs/FLASHING.md` build-time prevention, LL Entry 2).

## Ports

- GDB: 3333
- Telnet: 4444
- TCL: 6666
