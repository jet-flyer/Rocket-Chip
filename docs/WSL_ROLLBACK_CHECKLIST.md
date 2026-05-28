# WSL Rollback Checklist

**Purpose:** Quick, actionable steps to fully switch back to a native Windows workflow if the WSL path becomes problematic ("try until it breaks" policy).

Use this when you decide the Windows path needs to become primary again (temporarily or permanently).

---

## 1. Stop using the WSL worktree

- Close any terminals / VS Code windows pointing at `~/Rocket-Chip`.
- From now on, open the repo from its normal Windows location:
  ```
  C:\Users\pow-w\Documents\Rocket-Chip
  ```
- If you were using a Windows Terminal profile that started in `~`, switch back to your normal Windows profile or change `startingDirectory` back to the repo path.

---

## 2. Re-point PICO_SDK_PATH (Windows)

On Windows, the Pico SDK is normally at:

```
C:\Users\pow-w\.pico-sdk
```

Make sure your environment (VS Code, CMake, terminal, etc.) is pointing at the Windows copy, **not** the WSL one.

If you have any local `.env`, `settings.json`, or shell profiles on Windows that were temporarily pointing at the WSL paths during the pivot, revert them.

---

## 3. Switch back to Windows OpenOCD + Debug Probe

- Use the Pico SDK OpenOCD on Windows (the one under `.pico-sdk/openocd/...`).
- Use the standard Windows OpenOCD startup command from `docs/agents/DEBUG_PROBE_NOTES.md` (or `scripts/start_openocd_pico_sdk.ps1`).
- The probe should be connected directly via USB on Windows (not passed through usbipd).

**Do not** use the OpenOCD binary that lives inside the WSL distro for Windows-native debugging.

---

## 4. Hardware Access on Windows

- The boards will now appear as normal COM ports (e.g. COM7 for vehicle, COM9 for station).
- Update any temporary environment variables or script shortcuts you were using for the WSL by-id paths.
- If you had `USBDEV` / `STATION_USBDEV` set in your Windows environment for testing, clear or repoint them to the COM ports.

Example for running benches on Windows:

```powershell
$env:USBDEV = "COM7"
python scripts/bench_sim.py

$env:USBDEV = "COM9"
python scripts/station_bench_sim.py
```

---

## 5. Run Gates Natively on Windows

To confirm the Windows path is healthy:

- Run a full host `ctest --test-dir build_host`
- Run `bench_sim.py` and `station_bench_sim.py` on the actual COM ports
- (If doing a hardware gate that requires the probe) Start OpenOCD on Windows and perform at least one flash + banner read

This is the same check required by milestone checklist item 17c.

---

## 6. Optional: Cleanly Release WSL Side

If you want to fully stop using the WSL side for a while:

```powershell
wsl --terminate Ubuntu-24.04
# or
wsl --shutdown
```

Then detach the boards if desired:

```powershell
usbipd detach --busid 1-9
usbipd detach --busid 1-10
usbipd detach --busid 6-3
```

---

## 7. Verify You're Fully Back on Windows

Quick sanity checks:

- `cmake --version` and `python --version` should resolve to your normal Windows toolchain (not WSL).
- When you open the repo in VS Code, it should **not** offer to reopen in WSL (or you should decline).
- Running `bench_sim.py` should use a COM port, not a `/dev/serial/by-id/...` path.
- OpenOCD should be the Windows Pico SDK version.

---

## When to Come Back to WSL

You can return to the WSL path at any time by:

1. Re-attaching the boards via usbipd.
2. Re-opening the repo in VS Code Remote-WSL or using the WSL Terminal profile with `startingDirectory: "~"`.
3. Using the Linux FS worktree again (`~/Rocket-Chip`).

The worktree remains valid even if you spend time on Windows.

---

**This checklist is intentionally minimal.** It exists because the steady-state policy is "try until it breaks." When you decide the Windows path needs to be primary again, this is the lightweight path back.