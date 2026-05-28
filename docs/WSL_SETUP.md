# WSL Setup for Rocket-Chip Development

**Purpose:** Practical guide for using WSL2 (Ubuntu 24.04) as a first-class development environment for Rocket-Chip on Windows hardware. Covers usbipd passthrough for the Debug Probe + Vehicle + Station boards, stable device naming, toolchain setup, and known quirks.

This setup allows native-Linux-FS build performance and reliable hardware access while keeping Windows as a supported fallback.

---

## Hardware & Device Identification

The two Rocket-Chip boards (Vehicle and Station) share the same USB VID:PID (`2e8a:0009`). The Debug Probe uses `2e8a:000c`.

**Never rely on `/dev/ttyACM*` numbers.** Use the stable symlinks under `/dev/serial/by-id/` instead. These embed the unique board serial numbers:

- **Vehicle**: `usb-Raspberry_Pi_Pico_02FBDDB8E1CA1281-if00`
- **Station**: `usb-Raspberry_Pi_Pico_BEC71B8EDC6AEBD1-if00`
- **Debug Probe**: `usb-Raspberry_Pi_Debug_Probe__CMSIS-DAP__E663AC91D3487137-if01`

These paths are consistent across re-plugs, WSL restarts, and Windows reboots (once udev rules are in place).

**Environment variables for scripts** (supported in `bench_sim.py` and `station_bench_sim.py`):

```bash
export USBDEV=/dev/serial/by-id/usb-Raspberry_Pi_Pico_02FBDDB8E1CA1281-if00
export STATION_USBDEV=/dev/serial/by-id/usb-Raspberry_Pi_Pico_BEC71B8EDC6AEBD1-if00
```

---

## WSL2 Distro & Configuration

### Recommended distro
Ubuntu 24.04 LTS.

### `.wslconfig` (Windows side)

Place at `C:\Users\<you>\.wslconfig`:

```ini
[wsl2]
memory=20GB
processors=10
swap=12GB
localhostForwarding=true
pageReporting=true
```

These values were tuned for a 32 GB Ryzen 7 9800X3D system. Adjust downward if you have less RAM.

After editing, run `wsl --shutdown` from PowerShell.

### `/etc/wsl.conf` (inside WSL)

```ini
[automount]
enabled = true
root = /mnt/
options = "metadata,uid=1000,gid=1000,case=off"
mountFsTab = true

[network]
hostname = rocketchip-wsl
generateHosts = false
generateResolvConf = true

[interop]
appendWindowsPath = false

[boot]
systemd = true
```

**Important effects:**
- `generateHosts = false` → causes the cosmetic `sudo: unable to resolve host rocketchip-wsl` warning. Harmless.
- `systemd = true` → required for modern udev behavior.

After changes: `wsl --shutdown` then reopen your WSL terminal.

---

## Windows Terminal

Create or edit a profile for Ubuntu-24.04 and set:

```json
"startingDirectory": "~"
```

This makes new tabs start in your Linux home directory instead of a Windows path (`/mnt/c/...`). Strongly recommended for performance and muscle memory.

---

## usbipd-win + Hardware Passthrough

### Installation
```powershell
winget install usbipd
```

### Binding & Policies (one-time)

```powershell
# Probe
usbipd bind --busid <probe-busid>
usbipd policy add --effect Allow --operation AutoBind --hardware-id 2e8a:000c

# Both boards (same VID:PID — one policy covers both)
usbipd bind --busid <vehicle-busid>
usbipd bind --busid <station-busid>
usbipd policy add --effect Allow --operation AutoBind --hardware-id 2e8a:0009
```

### Daily Attach (when devices drop)

After `wsl --shutdown`, Windows reboot, or long idle:

```powershell
usbipd attach --wsl --busid <probe-busid>
usbipd attach --wsl --busid <vehicle-busid>
usbipd attach --wsl --busid <station-busid>
```

**Note:** `--auto-attach` creates a blocking watcher process. For daily use, either run the plain `attach` commands when needed or keep a small helper script.

---

## udev Rules (Non-Root Serial Access)

Create `/etc/udev/rules.d/99-rocketchip.rules`:

```bash
sudo tee /etc/udev/rules.d/99-rocketchip.rules > /dev/null << 'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="2e8a", ATTR{idProduct}=="000c", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="2e8a", ATTR{idProduct}=="0009", MODE="0666", GROUP="plugdev"

SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000c", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0009", MODE="0666", GROUP="plugdev"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty --subsystem-match=usb
```

Add your user to the group once:

```bash
sudo usermod -aG plugdev $USER
# Then log out and back in (or reboot WSL)
```

After a fresh attach of the boards, the tty nodes and their by-id symlinks should show `plugdev` + 0666.

---

## Toolchain (Inside WSL)

### Pico SDK
```bash
git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git ~/pico-sdk
```

### picotool
Build from source (v2.2.0-a4 worked well):

```bash
git clone https://github.com/raspberrypi/picotool.git ~/picotool
cd ~/picotool
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
```

### OpenOCD (Raspberry Pi fork)
Build the RP2350-capable fork. The version built during this pivot was `0.12.0+dev-00004-gacff23ffd`.

Install build deps first:

```bash
sudo apt install libtool automake autoconf libusb-1.0-0-dev
```

Then follow the standard RP fork build with `--enable-internal-jimtcl`.

### Persistent Environment

Create `/etc/profile.d/rocketchip.sh`:

```bash
sudo tee /etc/profile.d/rocketchip.sh > /dev/null << 'EOF'
export PICO_SDK_PATH=$HOME/pico-sdk
export PATH=$PATH:/usr/local/bin:$HOME/.local/bin
EOF
```

This ensures `PICO_SDK_PATH` is available in login shells (including many `wsl -d ...` invocations when using `bash -l`).

---

## Running Code & Tests from WSL

```bash
cd ~/Rocket-Chip

# Firmware (vehicle)
PICO_SDK_PATH=$HOME/pico-sdk cmake -B build_wsl -G Ninja
cmake --build build_wsl --target rocketchip -j$(nproc)

# Station
PICO_SDK_PATH=$HOME/pico-sdk cmake -B build_station_wsl -G Ninja -DROCKETCHIP_JOB_STATION=1
cmake --build build_station_wsl --target rocketchip -j$(nproc)

# Hardware gates (recommended)
USBDEV=/dev/serial/by-id/usb-Raspberry_Pi_Pico_02FBDDB8E1CA1281-if00 \
    python3 scripts/bench_sim.py

STATION_USBDEV=/dev/serial/by-id/usb-Raspberry_Pi_Pico_BEC71B8EDC6AEBD1-if00 \
    python3 scripts/station_bench_sim.py
```

---

## Known Quirks & Workarounds

- **Re-attach after shutdown**: `wsl --shutdown` or Windows reboot drops attachments back to "Shared". Re-run the three `usbipd attach --wsl` commands.
- **tty permissions**: The udev rule only takes effect on *new* enumerations. After updating the rule, detach + re-attach the boards.
- **sudo hostname warning**: Harmless side-effect of `generateHosts = false`. Ignore it.
- **picotool `reboot -f -s`**: In the v2.2.0-a4 build used here, `info --ser` and force-reboot via other means worked, but the `reboot` subcommand's device selection flags were limited. Manual BOOTSEL button remains a reliable fallback when needed.
- **OpenOCD speed**: Under usbipd, 2000–3000 kHz is often more stable than the 5000 kHz used natively on Windows.

---

## Dual-Path Notes

- **WSL (recommended for daily work)**: Faster incremental builds, consistent Linux tool behavior, stable serial naming via by-id.
- **Windows fallback**: Use when usbipd is misbehaving, for certain Windows-only tools, or when you need the absolute fastest OpenOCD adapter speed.

The scripts in `scripts/` now support `USBDEV` / `STATION_USBDEV` on both platforms, so the same commands work with minimal changes.

---

## Quick Reference — Fresh Session

**PowerShell (admin if binding):**
```powershell
usbipd attach --wsl --busid 1-9   # probe
usbipd attach --wsl --busid 1-10  # vehicle
usbipd attach --wsl --busid 6-3   # station
```

**WSL tab:**
```bash
cd ~/Rocket-Chip
# (PICO_SDK_PATH should be set via profile.d in login shells)
```

---

**This document captures the working configuration from the 2026 WSL soft pivot.** Temporary inconsistencies with other docs are acceptable while the pivot settles. Update other files only when facts change in a trigger-driven way (see `docs/agents/SESSION_CHECKLIST.md`).