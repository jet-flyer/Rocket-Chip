# Phase 0 Baseline — WSL Soft Pivot (2026-05-27)

**Captured while:** Vehicle (SN 02FBDDB8E1CA1281) + Debug Probe (SN E663AC91D3487137) connected.  
**Environment:** Native Windows (current daily driver).  
**Purpose:** Establish known-good state before any WSL changes. This is the reference for later comparison once the pivot begins.

---

## Git / Repo State

- Branch: `main`
- Status: Clean working tree, up to date with origin/main.
- Verified: `git status`

---

## Host Testing

- Suite: `ctest --test-dir build_host`
- Result: **857/857 PASS**
- Last run during this baseline session: successful, zero failures.

---

## Target Builds (Flight Binaries)

- Vehicle (`build_flight`): Clean (`ninja: no work to do`)
- Station (`build_station_flight`): Clean (`ninja: no work to do`)
- Both roles compile cleanly on the current Windows toolchain.

---

## Hardware-in-the-Loop Gates (Critical for This Project)

### bench_sim.py (Vehicle)

- Live run performed 2026-05-27 with vehicle on COM7:
  - Auto-detected via VID:PID + banner classification.
  - Result: **2/2 PASS** in 6.5 seconds.
  - Positive-control signals observed:
    - "sensors healthy → GO"
    - Happy-path flight (IDLE → ARMED → BOOST → COAST → DROGUE_DESCENT → MAIN_DESCENT → LANDED)
    - Abort-from-BOOST case also passed.
- Historical supporting logs (March 2026 IVP-74 runs) also show repeated successful positive-control executions.

### station_bench_sim.py

- Not executed live during this capture (station not plugged in at the time).
- User confirmed station hardware is available on request.

---

## picotool Baseline (Explicitly Called Out by User as High-Interest Item)

- Version on this machine: **picotool v2.2.0-a4** (Windows, MSVC build)
- Command used: `picotool info -f --ser 02FBDDB8E1CA1281`
- Behavior observed:
  - Tool correctly used `--ser` to target the specific vehicle (multi-device safe).
  - Forced reboot into BOOTSEL, read the binary, returned to application mode.
  - Successfully reported:
    - Binary name: `rocketchip`
    - Features: USB stdin/stdout
    - Target: RP2350 (ARM Secure image)
    - Flash range: 0x10000000 – 0x1003674c
- This is the exact pattern that will change when moving to WSL + usbipd.

---

## OpenOCD / Debug Probe

- Currently used adapter speed: **5000** (`adapter speed 5000`)
- OpenOCD binary: Pico SDK build at `/c/Users/pow-w/.pico-sdk/openocd/0.12.0+dev/`
- Command pattern (from `docs/agents/DEBUG_PROBE_NOTES.md` and `scripts/start_openocd_pico_sdk.ps1`):
  ```
  taskkill //F //IM openocd.exe 2>/dev/null; sleep 2; \
  /c/Users/pow-w/.pico-sdk/openocd/0.12.0+dev/openocd \
    -s /c/Users/pow-w/.pico-sdk/openocd/0.12.0+dev/scripts \
    -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
    -c "adapter speed 5000" &
  ```
- Full GDB + flash + banner read cycles have been routine on this setup.

---

## WSL State

- No WSL distributions installed (`wsl -l -v` returns "no installed distributions").
- WSL version on host: 2.7.3.0 (kernel 6.6.114.1-1)
- This is a clean starting point for the pivot.

---

## Windows Terminal

- Already installed and actively used by the operator (as stated at start of session).
- This will become the primary host for the WSL profile once Phase 1 is executed.

---

## Summary of Known-Good Windows Baseline

| Area                  | Status                  | Notes |
|-----------------------|-------------------------|-------|
| Host ctest            | 857/857 PASS            | Current reference |
| Vehicle target build  | Clean                   | - |
| Station target build  | Clean                   | - |
| bench_sim (live)      | 2/2 PASS, positive controls observed | 2026-05-27 run on COM7 |
| picotool --ser        | Functional (v2.2.0-a4)  | Confirmed with vehicle serial |
| OpenOCD speed         | 5000                    | Documented & used |
| WSL                   | None installed          | Clean slate |
| Windows Terminal      | In active use           | Ready for WSL profile |

This baseline was captured on 2026-05-27 while vehicle + probe hardware was connected. It serves as the reference point for all future WSL-side comparisons during the soft pivot.

**Next:** Once user confirms, proceed to Phase 1 (WSL distro + foundational config) or perform additional live station runs if desired.