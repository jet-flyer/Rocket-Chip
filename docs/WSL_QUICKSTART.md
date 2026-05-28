# WSL Quickstart — Daily Usage (No Manual Tab Required)

This guide is for day-to-day work. It focuses on the fastest path to run tests from Windows/PowerShell **without manually opening a WSL tab every time**.

For the full setup story and all quirks, see [WSL_SETUP.md](WSL_SETUP.md).

---

## Core Principle

You do **not** need to keep a WSL terminal open.

Use on-demand invocation:

```powershell
wsl -d Ubuntu-24.04 -- bash -l -c 'cd ~/Rocket-Chip && <your command here>'
```

The `-l` ensures login shell behavior (so `/etc/profile.d/rocketchip.sh` is sourced).

---

## Recommended Daily Pattern (Tests from PowerShell)

### 1. Make sure devices are attached (when needed)

After a `wsl --shutdown`, Windows reboot, or long idle, devices may drop back to "Shared".

Run this in PowerShell (once per session if needed):

```powershell
usbipd attach --wsl --busid 1-9    # Debug Probe
usbipd attach --wsl --busid 1-10   # Vehicle
usbipd attach --wsl --busid 6-3    # Station
```

You only need to do this when `usbipd list` shows them as `Shared` instead of `Attached`.

### 2. Run tests without opening WSL

**Vehicle bench:**

```powershell
wsl -d Ubuntu-24.04 -- bash -l -c '
  cd ~/Rocket-Chip &&
  USBDEV=/dev/serial/by-id/usb-Raspberry_Pi_Pico_02FBDDB8E1CA1281-if00 \
  python3 scripts/bench_sim.py
'
```

**Station bench:**

```powershell
wsl -d Ubuntu-24.04 -- bash -l -c '
  cd ~/Rocket-Chip &&
  STATION_USBDEV=/dev/serial/by-id/usb-Raspberry_Pi_Pico_BEC71B8EDC6AEBD1-if00 \
  python3 scripts/station_bench_sim.py
'
```

**Both in one go (recommended for full verification):**

```powershell
wsl -d Ubuntu-24.04 -- bash -l -c '
  cd ~/Rocket-Chip &&
  USBDEV=/dev/serial/by-id/usb-Raspberry_Pi_Pico_02FBDDB8E1CA1281-if00 \
  STATION_USBDEV=/dev/serial/by-id/usb-Raspberry_Pi_Pico_BEC71B8EDC6AEBD1-if00 \
  python3 scripts/bench_sim.py && \
  python3 scripts/station_bench_sim.py
'
```

---

## Why This Works Well

- The `USBDEV` / `STATION_USBDEV` variables are honored by the scripts (added during the 2026 WSL pivot).
- Using the full `/dev/serial/by-id/...` paths gives stable targeting even if `ttyACM` numbers shift.
- `bash -l` ensures the WSL environment (PICO_SDK_PATH, PATH additions) is set up the same way as an interactive login.
- You stay in your normal PowerShell / Windows Terminal workflow.

---

## Common One-Liners

**Just check the environment from Windows:**

```powershell
wsl -d Ubuntu-24.04 -- bash -l -c 'echo "PICO_SDK_PATH=$PICO_SDK_PATH"; which picotool; picotool version'
```

**Run a specific test with limited runtime (good for quick checks):**

```powershell
wsl -d Ubuntu-24.04 -- bash -l -c '
  cd ~/Rocket-Chip &&
  USBDEV=/dev/serial/by-id/usb-Raspberry_Pi_Pico_02FBDDB8E1CA1281-if00 \
  python3 scripts/bench_sim.py --max-runtime 30
'
```

---

## When You Still Need to Open a WSL Tab

- Debugging something interactively inside WSL (gdb, manual picocom, etc.).
- Running long interactive sessions or when you want to watch live output more easily.
- Initial setup or when troubleshooting usbipd / udev issues.

For normal test runs and CI-like verification, the on-demand `wsl -d ...` pattern above is preferred.

---

## Quick Troubleshooting

- **"No such file or directory" for the by-id path** → The board is not currently attached. Re-run the three `usbipd attach` commands.
- **Tests can't find the board** → Make sure you exported `USBDEV` / `STATION_USBDEV` inside the `-c '...'` string.
- **PICO_SDK_PATH not set** → Make sure you're using `bash -l` (login shell).
- **Permissions issues on /dev/ttyACM*** → Re-attach the boards after the udev rule was installed (or run the detach+attach cycle again).

For deeper diagnosis, see [WSL_SETUP.md](WSL_SETUP.md).

---

## End of Session / Proper Shutdown

When you are finished with a session that made heavy use of WSL, it is good practice to shut the distro down cleanly. This frees system resources and releases any USB devices that were attached.

### Recommended commands (run from PowerShell)

**Gentle / preferred for this distro only:**
```powershell
wsl --terminate Ubuntu-24.04
```

**Stronger (shuts down the entire WSL2 subsystem):**
```powershell
wsl --shutdown
```

After either command, the attached boards will drop back to "Shared" in `usbipd list` and will need to be re-attached the next time you work.

### For Agents

At the end of any session that involved significant WSL usage (running builds, bench_sim, OpenOCD, etc. inside the distro), the agent should remind the user to run a proper shutdown of the distro (or execute `wsl --terminate Ubuntu-24.04` on their behalf if appropriate). This is not a formal pre-commit or session-end checklist item, but it is expected agent behavior for good WSL hygiene.

---

**Bottom line:** Treat WSL as a service you invoke on demand rather than a terminal you have to keep open. The combination of stable by-id paths + the `USBDEV` variables + on-demand `wsl -d` invocation removes the need to manually open the tab for most test runs. At the end of a WSL-heavy session, run a clean shutdown.