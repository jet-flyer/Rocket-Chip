#!/usr/bin/env python3
"""
RocketChip Warm-Reboot Audit (R-22)

R-25-exec step 11 (2026-05-13): redesigned against the single flight
binary per council-APPROVED Approach A
(docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md). The 4-iteration
draft + council verdict captured at
`C:\\Users\\pow-w\\.claude\\plans\\snoopy-wibbling-noodle.md` is the
authoritative design; this implementation inherits the 7 council
amendments (G-W1 counter-increment positive control, G-W4 same,
chip-serial emission, operator runbook, downgrade-G-W3-to-soft framing,
follow-on PR log, explicit Rule-4 labeling).

Would have caught the SIO_FIFO_IRQ wedge that R-19 fixed if it had
existed in earlier cycles.

Gates (per HW_GATE_DISCIPLINE.md Rule 1 positive-control requirement):

  G-W1 (AIRCR via probe) — HARD with Core-1 counter-increment check
    Probe writes AIRCR -> wait for re-enum -> open menu ->
    Debug submenu 's' twice (>=1.2s apart) -> IMU/baro counters
    must advance + Hardware Status 14/14 OK. AIRCR is per-processor
    on RP2350 -- banner alone proves Core 0 only; the counter
    check is the actual Core-1 alive signal (R-19 wedge scenario).

  G-W2 (3x probe reset run) — HARD
    Probe `monitor reset run` x3 -> peek_banner between each ->
    Banner identity stable across 3 boots. Probe path resets both
    cores cleanly.

  G-W3 (Nx picotool burst) — HARD (downgradeable to soft per
    Risk #3 if picotool flakes)
    picotool reboot -f --ser <serial> x N ->
    _wait_for_device_serial between each -> final peek_banner.

  G-W4 (compound AIRCR + picotool) — HARD with G-W1 counter check
    G-W1 sequence + 2x G-W3 -> final peek_banner.

Uses peek_banner from _rc_test_common.py for every banner read
(proven Windows USB CDC survival pattern: thread-with-join-timeout,
OSError-22 tolerance). watchdog_s=300 self-kills on wedge.

Usage:
    python scripts/warm_reboot_audit.py [--chip-serial SERIAL]
        [--burst N] [--skip-aircr] [--skip-picotool] [--verbose]
        [--watchdog-s N]

Exit codes:
    0 = all enabled gates passed
    1 = one or more gate failures
    2 = setup error / watchdog fired / probe not available
"""

import argparse
import os
import subprocess
import sys
import time

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from _rc_test_common import (  # noqa: E402
    Banner,
    Role,
    arm_test_mode_via_probe,
    peek_banner,
    open_classified_port,
    enter_cli_menu,
    find_target_port,
    rc_test,
    TARGET_VEHICLE_ANY,
    TARGET_STATION_ANY,
    TARGET_EITHER_ANY,
    ROCKETCHIP_USB_VID,
    ROCKETCHIP_USB_PID,
)

GDB = r'C:\Users\pow-w\.pico-sdk\toolchain\14_2_Rel1\bin\arm-none-eabi-gdb.exe'
OPENOCD_PORT = 3333
PICOTOOL = r'C:\Users\pow-w\.pico-sdk\picotool\2.2.0\picotool\picotool.exe'

# Default knobs
DEFAULT_BURST = 5
PROBE_RESET_SETTLE_S = 3.0   # post-AIRCR re-enum window
PICOTOOL_RESET_SETTLE_S = 4.0  # post-picotool re-enum window
COUNTER_DELTA_WINDOW_S = 1.2  # >=1.2s between counter reads
IMU_DELTA_MIN = 100           # Core 1 IMU runs at ~1kHz -> ~1000 in 1.2s
BARO_DELTA_MIN = 5            # Baro runs at ~50Hz -> ~60 in 1.2s


# ============================================================================
# Probe + serial helpers
# ============================================================================

def _gdb_run(*ex_cmds, timeout=20.0):
    args = [GDB, '-batch',
            '-ex', f'target extended-remote localhost:{OPENOCD_PORT}']
    for c in ex_cmds:
        args.extend(['-ex', c])
    try:
        r = subprocess.run(args, capture_output=True, text=True,
                           timeout=timeout)
        return r.returncode == 0, r.stdout + r.stderr
    except subprocess.TimeoutExpired:
        return False, '<GDB TIMEOUT>'


def _probe_listening() -> bool:
    """OpenOCD on :3333 — quick connect probe."""
    ok, _ = _gdb_run('monitor halt', timeout=5.0)
    return ok


def _probe_reset_run() -> bool:
    """`monitor reset run` — resets both cores via bootrom."""
    ok, _ = _gdb_run('monitor reset run')
    return ok


def _probe_aircr_reset() -> bool:
    """Write 0x05FA0004 to AIRCR (SYSRESETREQ) via probe."""
    # AIRCR = 0xE000ED0C; SYSRESETREQ bit = 2; VECTKEY = 0x05FA0000.
    ok, _ = _gdb_run(
        'monitor halt',
        'monitor mww 0xE000ED0C 0x05FA0004',
        'monitor resume',
    )
    return ok


def _picotool_reboot(chip_serial: str) -> bool:
    if not chip_serial:
        return False
    try:
        r = subprocess.run(
            [PICOTOOL, 'reboot', '-f', '--ser', chip_serial],
            capture_output=True, text=True, timeout=15.0,
        )
        return r.returncode == 0
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False


def _detect_vehicle_chip_serial() -> str:
    """Read picotool info -a output, extract serial of attached RP2350."""
    try:
        r = subprocess.run(
            [PICOTOOL, 'info', '-a'],
            capture_output=True, text=True, timeout=10.0,
        )
        for ln in r.stdout.splitlines():
            ln = ln.strip()
            if ln.startswith('flash id:'):
                return ln.split(':', 1)[1].strip()
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass
    return ''


def _wait_for_device_serial(timeout_s=15.0, post_enum_settle=3.0) -> str:
    """Poll serial.tools.list_ports until a RocketChip VID:PID device
    appears, then settle for `post_enum_settle` seconds. Returns the
    discovered port name, or '' on timeout."""
    import serial.tools.list_ports
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        for p in serial.tools.list_ports.comports():
            if (getattr(p, 'vid', None) == ROCKETCHIP_USB_VID and
                    getattr(p, 'pid', None) == ROCKETCHIP_USB_PID):
                time.sleep(post_enum_settle)
                return p.device
        time.sleep(0.5)
    return ''


# ============================================================================
# Core-1 alive positive-control: q->s + q->b read
# ============================================================================

def _read_sensor_counts(port: str, timeout_s: float = 6.0) -> dict:
    """Open the port, enter menu, send q->s, parse IMU/baro counts.
    Returns {'imu': int, 'baro': int, 'gps': int, 'mag': int} or {}."""
    import re
    import serial
    try:
        with open_classified_port(port, target=TARGET_EITHER_ANY) as ser:
            # Make sure we're at the main menu first
            enter_cli_menu(ser)
            time.sleep(0.1)
            # Enter Debug submenu, then 's' for sensor counts
            ser.write(b'q')
            time.sleep(0.2)
            ser.read(4096)  # drain menu banner
            ser.write(b's')
            time.sleep(0.4)
            buf = ser.read(8192).decode('utf-8', errors='replace')
            # Exit back to main menu
            ser.write(b'z')
            time.sleep(0.1)
    except (serial.SerialException, OSError):
        return {}
    m = re.search(
        r'Reads:\s+I=(\d+)\s+M=(\d+)\s+B=(\d+)\s+G=(\d+)', buf)
    if not m:
        return {}
    return {
        'imu': int(m.group(1)),
        'mag': int(m.group(2)),
        'baro': int(m.group(3)),
        'gps': int(m.group(4)),
    }


def _read_hw_status_ok(port: str) -> tuple:
    """Read the Hardware Status block (q->b), return (pass, total)
    parsed from 'Hardware: N/N OK' line, or (0, 0) on failure."""
    import re
    import serial
    try:
        with open_classified_port(port, target=TARGET_EITHER_ANY) as ser:
            enter_cli_menu(ser)
            time.sleep(0.1)
            ser.write(b'q')
            time.sleep(0.2)
            ser.read(8192)
            ser.write(b'b')
            time.sleep(0.5)
            buf = ser.read(16384).decode('utf-8', errors='replace')
            ser.write(b'z')
            time.sleep(0.1)
    except (serial.SerialException, OSError):
        return (0, 0)
    m = re.search(r'Hardware:\s+(\d+)/(\d+)\s+OK', buf)
    if not m:
        return (0, 0)
    return (int(m.group(1)), int(m.group(2)))


# ============================================================================
# Gates
# ============================================================================

def _baseline_banner(port: str) -> Banner:
    return peek_banner(port)


def _gate_w1_aircr(port: str, baseline: Banner, verbose: bool) -> bool:
    """G-W1: AIRCR reset, Core 1 still alive."""
    print('[G-W1] AIRCR reset via probe...')
    if not _probe_aircr_reset():
        print('[G-W1] FAIL: probe AIRCR write failed')
        return False
    new_port = _wait_for_device_serial(
        timeout_s=15.0, post_enum_settle=PROBE_RESET_SETTLE_S)
    if not new_port:
        print('[G-W1] FAIL: device did not re-enumerate post-AIRCR')
        return False
    # Sample 1: counters at t0
    c1 = _read_sensor_counts(new_port)
    if not c1:
        print('[G-W1] FAIL: could not read sensor counts (sample 1)')
        return False
    if verbose:
        print(f'[G-W1]   sample 1: imu={c1["imu"]} baro={c1["baro"]}')
    time.sleep(COUNTER_DELTA_WINDOW_S)
    # Sample 2: counters at t1
    c2 = _read_sensor_counts(new_port)
    if not c2:
        print('[G-W1] FAIL: could not read sensor counts (sample 2)')
        return False
    if verbose:
        print(f'[G-W1]   sample 2: imu={c2["imu"]} baro={c2["baro"]}')
    imu_delta = c2['imu'] - c1['imu']
    baro_delta = c2['baro'] - c1['baro']
    if imu_delta < IMU_DELTA_MIN:
        print(f'[G-W1] FAIL: Core 1 IMU stuck (delta {imu_delta} < {IMU_DELTA_MIN})')
        return False
    if baro_delta < BARO_DELTA_MIN:
        print(f'[G-W1] FAIL: Core 1 baro stuck (delta {baro_delta} < {BARO_DELTA_MIN})')
        return False
    # Hardware Status check
    hw_pass, hw_total = _read_hw_status_ok(new_port)
    if hw_total == 0:
        print('[G-W1] FAIL: could not read Hardware Status block')
        return False
    if hw_pass != hw_total:
        print(f'[G-W1] FAIL: Hardware Status degraded ({hw_pass}/{hw_total})')
        return False
    print(f'[G-W1] PASS: Core 1 alive (imu+{imu_delta}, baro+{baro_delta}, '
          f'HW {hw_pass}/{hw_total} OK)')
    return True


def _gate_w2_probe_reset_x3(port: str, baseline: Banner,
                            verbose: bool) -> bool:
    """G-W2: 3x probe reset run, banner stable."""
    print('[G-W2] 3x probe reset run...')
    for i in range(1, 4):
        if not _probe_reset_run():
            print(f'[G-W2] FAIL: probe reset run #{i}')
            return False
        new_port = _wait_for_device_serial(
            timeout_s=10.0, post_enum_settle=2.0)
        if not new_port:
            print(f'[G-W2] FAIL: no re-enum after reset #{i}')
            return False
        b = peek_banner(new_port)
        if not b or not b.is_known():
            print(f'[G-W2] FAIL: banner unreadable after reset #{i}')
            return False
        if verbose:
            print(f'[G-W2]   boot #{i}: {b.short_summary()}')
        if (b.role != baseline.role or b.build != baseline.build or
                b.version != baseline.version):
            print(f'[G-W2] FAIL: banner drift at reset #{i} '
                  f'(was {baseline.short_summary()}, now {b.short_summary()})')
            return False
    print('[G-W2] PASS: banner stable across 3 probe resets')
    return True


def _gate_w3_picotool_burst(port: str, baseline: Banner, burst: int,
                            chip_serial: str, verbose: bool) -> bool:
    """G-W3: N x picotool reboot -f --ser, banner stable."""
    print(f'[G-W3] {burst}x picotool reboot (--ser {chip_serial})...')
    for i in range(1, burst + 1):
        if not _picotool_reboot(chip_serial):
            print(f'[G-W3] FAIL: picotool reboot #{i} '
                  f'(empirical flake -- consider Risk #3 soft downgrade)')
            return False
        new_port = _wait_for_device_serial(
            timeout_s=15.0, post_enum_settle=PICOTOOL_RESET_SETTLE_S)
        if not new_port:
            print(f'[G-W3] FAIL: no re-enum after picotool #{i}')
            return False
        if verbose:
            b = peek_banner(new_port)
            if b and b.is_known():
                print(f'[G-W3]   boot #{i}: {b.short_summary()}')
    # Final banner compare
    final = peek_banner(new_port)
    if not final or not final.is_known():
        print('[G-W3] FAIL: final banner unreadable')
        return False
    if (final.role != baseline.role or final.build != baseline.build or
            final.version != baseline.version):
        print(f'[G-W3] FAIL: banner drift post-burst '
              f'(was {baseline.short_summary()}, now {final.short_summary()})')
        return False
    print(f'[G-W3] PASS: banner stable after {burst} picotool reboots')
    return True


def _gate_w4_compound(port: str, baseline: Banner, chip_serial: str,
                      verbose: bool) -> bool:
    """G-W4: G-W1 (AIRCR + counter check) + 2x picotool."""
    print('[G-W4] Compound: AIRCR + counter-check + 2x picotool...')
    if not _gate_w1_aircr(port, baseline, verbose):
        print('[G-W4] FAIL: G-W1 subsequence')
        return False
    # 2 picotool reboots
    for i in range(1, 3):
        if not _picotool_reboot(chip_serial):
            print(f'[G-W4] FAIL: picotool subsequence #{i}')
            return False
        new_port = _wait_for_device_serial(
            timeout_s=15.0, post_enum_settle=PICOTOOL_RESET_SETTLE_S)
        if not new_port:
            print(f'[G-W4] FAIL: no re-enum after picotool #{i}')
            return False
    final = peek_banner(new_port)
    if not final or not final.is_known():
        print('[G-W4] FAIL: final banner unreadable')
        return False
    if (final.role != baseline.role or final.build != baseline.build or
            final.version != baseline.version):
        print(f'[G-W4] FAIL: banner drift post-compound '
              f'(was {baseline.short_summary()}, now {final.short_summary()})')
        return False
    print('[G-W4] PASS: compound sequence clean')
    return True


# ============================================================================
# Main
# ============================================================================

@rc_test(target=TARGET_EITHER_ANY, watchdog_s=300)
def main():
    parser = argparse.ArgumentParser(
        description='R-22 warm-reboot audit (R-25-exec redesign 2026-05-13)')
    parser.add_argument('--role', choices=['vehicle', 'station'],
                        default='vehicle',
                        help='Target role (default: vehicle). Station '
                             'auto-skips G-W1/G-W4 -- Core-1 counter '
                             'check has no station-side equivalent.')
    parser.add_argument('--elf', default=None,
                        help='ELF path for probe arming (default: '
                             'build_flight/rocketchip.elf for vehicle, '
                             'build_station_flight/rocketchip.elf for station)')
    parser.add_argument('--chip-serial', default=None,
                        help='Chip serial for picotool --ser '
                             '(auto-detected if omitted)')
    parser.add_argument('--burst', type=int, default=DEFAULT_BURST,
                        help=f'picotool burst count (default {DEFAULT_BURST})')
    parser.add_argument('--skip-aircr', action='store_true',
                        help='Skip G-W1 + G-W4 (AIRCR-related gates)')
    parser.add_argument('--skip-picotool', action='store_true',
                        help='Skip G-W3 + G-W4 (picotool-related gates)')
    parser.add_argument('--skip-arm', action='store_true',
                        help='Skip the test-mode probe-arm step '
                             '(use if firmware is already in test mode)')
    parser.add_argument('--verbose', action='store_true')
    parser.add_argument('--watchdog-s', type=int, default=300,
                        help='Wall-clock watchdog (default 300s)')
    args = parser.parse_args()

    # Tool availability check
    if not _probe_listening():
        print('SETUP ERROR: OpenOCD probe not listening on '
              f'127.0.0.1:{OPENOCD_PORT}. Start probe and retry.')
        sys.exit(2)

    # Role-specific target + ELF default
    if args.role == 'station':
        target = TARGET_STATION_ANY
        default_elf = 'build_station_flight/rocketchip.elf'
    else:
        target = TARGET_VEHICLE_ANY
        default_elf = 'build_flight/rocketchip.elf'
    elf_path = args.elf or default_elf

    # Chip-serial detection (per ArduPilot amendment — anti-Frankenstein)
    chip_serial = args.chip_serial or _detect_vehicle_chip_serial()
    if not chip_serial and not args.skip_picotool:
        print('SETUP ERROR: chip serial not detected. '
              'Use --chip-serial or --skip-picotool.')
        sys.exit(2)

    # Arm test mode via probe (writes kTestModeMagic, resets).
    # Both vehicle and station will boot into kMenu when armed
    # (forced in AO_RCOS_start when magic was observed).
    if not args.skip_arm:
        print(f'[ARM] Arming test mode via probe ({elf_path})...')
        if not arm_test_mode_via_probe(elf_path):
            print('SETUP ERROR: failed to arm test mode via probe.')
            sys.exit(2)
        time.sleep(3.0)  # let boot-time-window capture the magic + AOs start

    # Initial port + baseline banner
    port, info = find_target_port(target)
    if not port:
        print(f'SETUP ERROR: {args.role} port not found ({info})')
        sys.exit(2)
    baseline = peek_banner(port)
    if not baseline or not baseline.is_known():
        print('SETUP ERROR: baseline banner could not be parsed')
        sys.exit(2)
    print(f'baseline: {baseline.short_summary()}  port={port}  role={args.role}')
    print(f'chip_serial={chip_serial}')

    # Run gates. G-W1 + G-W4's Core-1 counter check is vehicle-only;
    # station auto-skips them (no fast-incrementing counter on station-
    # side q->s output; AO_Radio_get_state()->rx_count only advances
    # when vehicle is TXing, which is not guaranteed during the audit).
    is_station = (args.role == 'station')
    results = {}
    if args.skip_aircr:
        print('[G-W1] SKIP (--skip-aircr)')
        print('[G-W4] SKIP (--skip-aircr)')
    elif is_station:
        print('[G-W1] SKIP (station: no Core-1 counter check applicable)')
        print('[G-W4] SKIP (station: no Core-1 counter check applicable)')
    else:
        results['G-W1'] = _gate_w1_aircr(port, baseline, args.verbose)
    results['G-W2'] = _gate_w2_probe_reset_x3(port, baseline, args.verbose)
    if args.skip_picotool:
        print('[G-W3] SKIP (--skip-picotool)')
        print('[G-W4] SKIP (--skip-picotool)')
    else:
        results['G-W3'] = _gate_w3_picotool_burst(
            port, baseline, args.burst, chip_serial, args.verbose)
    if (not args.skip_aircr and not args.skip_picotool and not is_station):
        results['G-W4'] = _gate_w4_compound(
            port, baseline, chip_serial, args.verbose)

    # Summary
    print('')
    print('=== Warm-Reboot Audit Summary ===')
    failed = 0
    for name, ok in results.items():
        print(f'  {name}: {"PASS" if ok else "FAIL"}')
        if not ok:
            failed += 1
    print(f'chip_serial={chip_serial}')
    print(f'===  {len(results) - failed}/{len(results)} PASS  ===')
    return 0 if failed == 0 else 1


if __name__ == '__main__':
    main()
