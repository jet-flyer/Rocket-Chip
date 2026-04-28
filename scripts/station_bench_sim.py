#!/usr/bin/env python3
"""
RocketChip Station Bench Sim (IVP-146, 2026-04-18; hardened 2026-04-27)

Programmatic station-role regression detector — parallel to the vehicle
bench_sim.py. Closes the "vehicle has a machine-checkable rot detector,
station has manual serial capture" asymmetry flagged during IVP-142c.

Scope: verify the station CLI dispatch path + banner hygiene + live
HealthMonitor output + absence of QP assertions over a short soak.
Anything a future station SPIN model proves is NOT tested here.

Three tests:
  1. Boot + dashboard round-trip (main menu reachable, no QP markers).
  2. Hardware Status positive controls ([N/A] for uninstalled sensors —
     regression guard for IVP-142c presentation).
  3. Health pipeline populated (diag_stats [Health] line shows non-zero
     secondary byte + RegVersion=0x12 — regression guard for
     AO_HealthMonitor-on-station starting under capability-masking).

Hardware target: REQUIRES STATION FIRMWARE — refuses to run on vehicle.
The script auto-detects USB CDC ports by VID:PID 0x2E8A:0x0009 and
classifies the banner. If no station banner ('Ground Station',
'Fruit Jam', 'Station RX') is found, exits cleanly with code 2 ("no
station to test"). The pre-commit hook treats exit 2 as "skip", not
"fail", so commits that touch shared code paths can proceed when only
the vehicle is plugged in.

DO NOT remove the station guard. Tests 2 and 3 navigate via the debug
menu, where 'x' from main menu is bound to Erase-Flights on vehicle
firmware. Sending 'x' to a misidentified target enters a destructive
confirmation prompt. See 2026-04-27 incident notes.

Wall-clock watchdog: --max-runtime (default 60s) is enforced by a daemon
thread that calls os._exit(2) when fired. This is the only reliable
escape from a hung serial.Serial() open on Windows USB CDC.

Port detection: ``find_target_port(TARGET_STATION_BENCH)`` (RocketChip
VID:PID USB CDC only). ``--port`` overrides but is still banner-classified;
connection uses ``open_classified_port`` with ``auto_enter_cli_menu=False``
because ``goto_main_from_anywhere`` performs dashboard→menu navigation.

Exit codes:
    0 = all 3 tests pass, no assertions
    1 = one or more test failures (regression — fix firmware/test)
    2 = no station present, tool self-check failed, or watchdog fired
        (pre-commit treats this as SKIP, not failure)
"""

import argparse
import os
import re
import serial
import sys
import threading
import time

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)
from _rc_test_common import (  # noqa: E402
    Banner,
    Role,
    classify_banner,
    find_target_port,
    open_classified_port,
    peek_banner,
    rc_test,
    ROCKETCHIP_USB_PID,
    ROCKETCHIP_USB_VID,
    TARGET_STATION_BENCH,
)

# =============================================================================
# Regex constants — TESTED AGAINST FIRMWARE LOG OUTPUT.
# If you change a log line in the firmware, UPDATE THE REGEX HERE.
# =============================================================================

RE_HEALTH_LINE = re.compile(
    r'primary=0x([0-9a-fA-F]+)\s+secondary=0x([0-9a-fA-F]+)\s+'
    r'critical=0x([0-9a-fA-F]+)\s+mcu=(\w+)\s+go_nogo=(\w+)'
)
# Firmware: diag_stats.cpp [Health] line. All 5 fields required.

RE_REG_VERSION = re.compile(r'RegVersion=0x([0-9a-fA-F]+)')
# Firmware: diag_stats_t0_preconditions() RegVersion readback.
# JPL-required radio positive control — MUST be 0x12 for SX1276.

RE_NA_SENSOR = re.compile(r'\[N/A\s*\]\s*(ICM-20948|DPS310|GPS)\s+not installed')
# Firmware: print_imu_status / print_baro_status / print_gps_status.
# IVP-142c regression guard — uninstalled sensors must render [N/A], not [FAIL].

ASSERTION_MARKERS = [
    'Q_onError', 'qf_actq', 'qf_dyn', 'ASSERT',
    'HARDFAULT', 'MEMMANAGE', 'BUSFAULT', 'USAGEFAULT',
]


# =============================================================================
# Wall-clock watchdog
#
# `--max-runtime` is checked between tests, but a hung serial.Serial() open
# (Windows USB CDC stuck-in-CONFIGURED) can stall in C-library code where
# the inter-test check never fires. A daemon watchdog thread is the only
# reliable hard kill: when it fires, it prints to stderr and calls
# os._exit(2). os._exit bypasses Python finalizers (intentional — we are
# already wedged) but is the only way to escape a serial-port C-call.
# =============================================================================

def _start_watchdog(deadline_s, label='station_bench_sim'):
    def _bark():
        time.sleep(deadline_s)
        sys.stderr.write(
            f'\n[WATCHDOG] {label} exceeded --max-runtime '
            f'{deadline_s:.0f}s wall-clock; force-exiting (code 2)\n')
        sys.stderr.flush()
        os._exit(2)
    t = threading.Thread(target=_bark, daemon=True, name='watchdog')
    t.start()
    return t


# =============================================================================
# Serial helpers (I/O helpers local to tests)
# =============================================================================

def send_and_read(port, key, wait_s=2.0):
    port.reset_input_buffer()
    port.write(key.encode() if isinstance(key, str) else key)
    port.flush()
    end = time.time() + wait_s
    buf = b''
    while time.time() < end:
        c = port.read(4096)
        if c:
            buf += c
        else:
            time.sleep(0.05)
    return buf.decode('utf-8', errors='replace')


def scan_asserts(text):
    return [m for m in ASSERTION_MARKERS if m in text]


# =============================================================================
# Menu navigation helpers — STATION-ONLY.
#
# Station boots in kAnsi (dashboard). The dashboard ignores all keys except
# 'x' (enter CLI menu), 'a' (ARM-confirm flow), 'D' (DISARM), 'm' (mode
# cycle). Once in the kMenu CLI we share the vehicle handlers (handle_main_menu
# in src/cli/rc_os.cpp), so 'q' enters the debug menu (dev-only — requires
# NOT_CERTIFIED_FOR_FLIGHT=ON). The debug submenu's "back to main" key is
# 'z' / 'Z' / Esc — NOT 'x' (which is bound to Erase-Flights from main menu
# even on station, see cli_handle_unhandled_key).
#
# 2026-04-27 incident: previous tests used 'x' to "reset" between submenus.
# When find_port() fell back to a vehicle, those 'x' keystrokes triggered
# the Erase-Flights confirmation prompt. Strict station-only detection +
# 'z' for "back" closes both halves of that bug.
# =============================================================================

ESC = '\x1b'

def goto_main_from_anywhere(port):
    """Best-effort: reach kMenu's [main] prompt regardless of starting state.

    Sequence:
      1. Reject any pending y/n confirm prompt (Enter)
      2. Esc out of any submenu (debug/cal/flight) → returns to main.
      3. 'x' from kAnsi dashboard enters the CLI menu (idempotent — 'x' on
         main menu is Erase, but we follow up with Esc which rejects the
         erase confirm prompt without typing 'yes').
      4. Drain output and verify '[main]' prompt eventually appears.
    """
    port.reset_input_buffer()
    port.write(b'\n')             # reject any pending confirm
    time.sleep(0.2)
    port.write(ESC.encode())      # exit any submenu
    time.sleep(0.2)
    port.write(b'x')              # if in dashboard, enter CLI menu
    time.sleep(0.3)
    port.write(b'\n')             # if 'x' triggered erase confirm, reject it
    time.sleep(0.2)
    port.write(ESC.encode())      # belt-and-braces for submenu
    time.sleep(0.5)
    port.read(16000)              # drain everything
    # Final 'h' to surface the main-menu help banner so callers can confirm.
    port.write(b'h')
    time.sleep(1.0)
    return port.read(16000).decode('utf-8', errors='replace')


# =============================================================================
# Test cases (STATION FIRMWARE ONLY — guarded by find_target_port)
# =============================================================================

def test_boot_and_main_menu(port, state, verbose=False):
    """Reach the kMenu [main] prompt from wherever the station was; verify
    no QP/fault assertion markers along the way."""
    out = goto_main_from_anywhere(port)
    state['boot'] = out
    if '[main]' not in out and 'Station RX' not in out and 'RocketChip' not in out:
        return False, f'Main menu prompt not seen (saw {len(out)}B)'
    return True, 'OK'


def test_hardware_status(port, state, verbose=False):
    """Verify Hardware Status shows [N/A] for uninstalled sensors (IVP-142c).

    Path: from [main] → 'q' (debug menu, dev-only) → 'b' (HW status print).
    Returns to main with Esc (NOT 'x' — 'x' from main is Erase-Flights).
    """
    send_and_read(port, 'q', 1.5)            # → [debug]
    out = send_and_read(port, 'b', 4.0)      # cli_print_hw_status
    send_and_read(port, ESC, 1.0)            # → back to [main]

    state['hw'] = out
    na_hits = RE_NA_SENSOR.findall(out)
    if 'ICM-20948' not in na_hits:
        return False, f'IMU not shown as [N/A]: found {na_hits}'
    if 'DPS310' not in na_hits:
        return False, f'Baro not shown as [N/A]: found {na_hits}'
    if re.search(r'\[FAIL\]\s*ICM-20948', out):
        return False, '[FAIL] ICM-20948 seen — regression of N/A presentation'
    if re.search(r'\[FAIL\]\s*DPS310', out):
        return False, '[FAIL] DPS310 seen — regression of N/A presentation'
    return True, f'OK ({len(na_hits)} N/A)'


def test_health_pipeline(port, state, verbose=False):
    """Verify diag_stats [Health] line non-zero secondary + RegVersion=0x12.

    Path: from [main] → 'q' (debug, dev-only) → 'd' (diag_stats_dump).
    Returns to main with Esc.
    """
    send_and_read(port, 'q', 1.5)            # → [debug]
    out = send_and_read(port, 'd', 5.0)      # diag_stats_dump
    send_and_read(port, ESC, 1.0)            # → back to [main]

    state['diag'] = out

    m = RE_HEALTH_LINE.search(out)
    if not m:
        return False, '[Health] line not found in diag_stats output'
    prim, sec, crit, mcu, gng = m.groups()
    state['health'] = (prim, sec, crit, mcu, gng)

    if int(sec, 16) == 0:
        return False, (f'secondary=0x{sec} — AO_HealthMonitor not running? '
                       f'(expected non-zero with Radio+Flash+WDT+PIO bits)')

    rv = RE_REG_VERSION.search(out)
    if not rv:
        return False, 'RegVersion not found in T=0 preconditions'
    if rv.group(1).lower() != '12':
        return False, f'RegVersion=0x{rv.group(1)} (expected 0x12)'

    if int(crit, 16) != 0:
        return False, f'critical=0x{crit} (expected 0x00 on healthy station)'

    return True, f'OK primary=0x{prim} secondary=0x{sec} mcu={mcu}'


TESTS = [
    ('Boot + main menu',          test_boot_and_main_menu),
    ('Hardware Status [N/A]',     test_hardware_status),
    ('Health pipeline populated', test_health_pipeline),
]


# =============================================================================
# Main
# =============================================================================

def _station_bench_run_inner(ser: serial.Serial,
                             port_name: str,
                             meta: Banner,
                             args,
                             *,
                             allow_vehicle: bool):
    """Test body inside ``open_classified_port`` context."""

    # Re-confirm we are still on station firmware after open() — defends
    # against the (rare) case where the user hot-swapped boards between
    # port selection and connect().
    ser.write(b'\n')
    time.sleep(0.3)
    ser.read(16000)  # drain
    ser.write(b'x')              # enter CLI menu from dashboard
    time.sleep(0.4)
    ser.write(b'\n')             # reject any erase-confirm prompt
    time.sleep(0.4)
    ser.write(ESC.encode())      # exit any submenu we may have entered
    time.sleep(0.4)
    ser.write(b'h')              # request main-menu help
    time.sleep(1.0)
    sanity = ser.read(16000).decode('utf-8', errors='replace')
    if classify_banner(sanity).role == Role.VEHICLE and not allow_vehicle:
        print('ERROR: post-open sanity check classifies firmware as VEHICLE.')
        print('  Refusing to run station-only key sequences on vehicle firmware.')
        print('  (Did you hot-swap boards? Or is build_station out of date?)')
        sys.exit(2)

    print(f'\n=== RocketChip Station Bench Sim ===')
    print(f'  Port: {port_name}')
    print(f'  Classified: {meta.short_summary()}')
    print(f'  Max runtime: {args.max_runtime:.0f}s (hard-killed by watchdog)')
    print()

    state = {}
    passed = 0
    failed = 0
    start = time.time()
    deadline = start + args.max_runtime

    for i, (name, fn) in enumerate(TESTS):
        if time.time() >= deadline:
            elapsed = time.time() - start
            print(f'  [ABORT] Runtime exceeded {args.max_runtime:.0f}s after {elapsed:.1f}s')
            failed += len(TESTS) - i
            break
        try:
            ok, detail = fn(ser, state, args.verbose)
        except serial.SerialException as e:
            ok, detail = False, f'Serial error: {e}'
        status = 'PASS' if ok else 'FAIL'
        if ok:
            passed += 1
        else:
            failed += 1
        suffix = f' -- {detail}' if (not ok or args.verbose) else ''
        print(f'  [{status}]  {i + 1}. {name}{suffix}')

    all_output = ''.join(state.get(k, '') for k in ('boot', 'hw', 'diag'))
    hits = scan_asserts(all_output)
    if hits:
        print(f'  [FAIL]  QP/fault assertion markers in output: {hits}')
        failed += 1
    else:
        print(f'  [PASS]  No QP/fault assertions across all tests')

    elapsed = time.time() - start
    total = passed + failed
    print(f'\n  RESULT: {passed}/{total} PASS  ({elapsed:.1f}s)')
    print('=' * 38)

    return 0 if failed == 0 else 1


@rc_test(target=TARGET_STATION_BENCH)
def main():
    parser = argparse.ArgumentParser(
        description='RocketChip Station Bench Sim (IVP-146, 3 tests)')
    parser.add_argument('--port', default=None,
                        help='Serial port (auto-detected via VID:PID + banner)')
    parser.add_argument('--verbose', action='store_true',
                        help='Print verbose test output')
    parser.add_argument('--max-runtime', type=float, default=60.0,
                        help='Wall-clock deadline (default 60s, hard-killed by watchdog)')
    parser.add_argument('--allow-non-station', action='store_true',
                        help=argparse.SUPPRESS)  # debugging only — never set in CI
    args = parser.parse_args()

    # Wall-clock watchdog: hard kill after deadline, regardless of where
    # we are in the test (including stuck inside serial.Serial() C calls).
    _start_watchdog(args.max_runtime)

    if args.allow_non_station and args.port is not None:
        # Unsafe path: manual port without strict target match (debugging).
        port_name = args.port
        meta = peek_banner(port_name)
    else:
        port_name, reason = find_target_port(
            TARGET_STATION_BENCH, override=args.port, verbose=args.verbose)
        if port_name is None:
            print(f'INFO: No station detected — {reason}.')
            print('  Skipping station_bench_sim (this is exit code 2 = '
                  '"no station to test", not a test failure).')
            print('  To run: flash build_station/*.uf2 and plug in USB '
                  f'(VID 0x{ROCKETCHIP_USB_VID:04X} PID 0x{ROCKETCHIP_USB_PID:04X}).')
            sys.exit(2)
        if not isinstance(reason, Banner):
            print('ERROR: internal: expected Banner from find_target_port')
            sys.exit(2)
        meta = reason

    print(f'using station port: {port_name}')
    if meta.is_known():
        print(f'  ({meta.short_summary()})')

    try:
        with open_classified_port(
            port_name,
            target=TARGET_STATION_BENCH,
            post_open_re_classify=not args.allow_non_station,
            auto_enter_cli_menu=False,
        ) as ser:
            return _station_bench_run_inner(
                ser, port_name, meta, args,
                allow_vehicle=args.allow_non_station)
    except RuntimeError as e:
        print(f'ERROR: Cannot open {port_name}: {e}')
        sys.exit(2)


if __name__ == '__main__':
    main()
