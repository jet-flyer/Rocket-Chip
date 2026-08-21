# L2-P5 disposition baseline — 2026-08-20

HEAD: `2cf1a54` (`grok/l2p5-disposition` worktree).
Vehicle chip serial (repo): `02FBDDB8E1CA1281` (Feather). Station not on USB this sitting.

## Identity (do not use CLI banner as proof)

| Check | Result |
|-------|--------|
| CMakeCache `PICO_BOARD` | `adafruit_feather_rp2350` |
| CMakeCache `ROCKETCHIP_JOB_STATION` | unset (vehicle default) |
| UF2 SHA256 | `C74611D53CD039FB044CFF94046C408D3A3BD53CBCCD22A6A151E43A8901686E` |
| `picotool info` on that UF2 | `pico_board: adafruit_feather_rp2350`, binary `0x10000000–0x10037070`, sdk 2.2.0, build date Aug 20 2026 |
| `picotool verify -f --ser 02FBDDB8E1CA1281` vs that UF2 | **OK** (flash contents = file) |
| NeoPixel (owner, Tier 1) | **solid green**, not dark |
| ELF `size` | text 231532, data 0, bss 324748 |

CLI `v0.16.0` / `Board:` strings are **not** identity evidence.

GDB/SWD was **not** used. Probe `extended-remote` failed with Windows error 138 (twice from PowerShell, once from Git Bash + `.gdb` script). One-shot flash used `picotool load -f --ser` per `FLASHING.md` single-shot path. Probe unplugged after; not required for this baseline.

## Authored LOC (before disposition edits)

Counted `.c/.cc/.cpp/.h/.hpp` under `src/` and `include/rocketchip/`, excluding `EXTERNAL/`, `pico-sdk`, `lib/`.

- `src/`: 153 files, 34808 lines
- `include/rocketchip/`: 33 files, 3125 lines
- **total: 186 files, 37933 lines**

## Host / HW function

- Host ctest (`build_host`): **858/858 PASS** (19.4 s).
- Vehicle `bench_sim.py --port COM5`: **2/2 PASS** on three boots. Pass requires `[FD] PYRO FIRED: DROGUE/MAIN (primary)` on the happy path.

| Boot | Restart method | bench_sim |
|------|----------------|-----------|
| 1 | After `picotool load -f` + `verify` reboot | 2/2 PASS, sensors healthy GO |
| 2 | Owner USB power-cycle of Feather | 2/2 PASS, sensors healthy GO |
| 3 | OpenOCD telnet `:4444` `reset halt` + `resume` (not GDB) | 2/2 PASS, sensors healthy GO |

Do **not** use `picotool reboot -a -f` for extra boots: it dropped CDC (no COM, not BOOTSEL) until a physical USB replug.

## GDB / probe notes (this sitting)

Windows `arm-none-eabi-gdb.exe` `target extended-remote` failed with **error 138** (“join a drive to a directory on a joined drive”). **Not in repo docs** (docs cover GDB error **10061** = Chocolatey GDB vs Pico SDK OpenOCD). OpenOCD itself was healthy: CMSIS-DAP `E663AC91D3487137`, SWD DPIDR `0x4c013477`, both M33 cores examined. Probe DAP LEDs on = OpenOCD `DAP_CONNECT`. Workaround: OpenOCD telnet `:4444`. Do not treat WSL as a tested GDB fix — usbipd/USB-in-VM was confirmed; the full WSL flash/GDB/bench loop was preliminary and not the lived daily path.
