#!/usr/bin/env python3
"""
RocketChip Station Bench Sim (IVP-146, 2026-04-18; hardened 2026-04-27)

Programmatic station-role regression detector — parallel to the vehicle
bench_sim.py. Closes the "vehicle has a machine-checkable rot detector,
station has manual serial capture" asymmetry flagged during IVP-142c.

Scope: verify the station CLI dispatch path + banner hygiene + live
HealthMonitor output + absence of QP assertions over a short soak.
Anything a future station SPIN model proves is NOT tested here.

Four tests:
  1. Boot + dashboard round-trip (main menu reachable, no QP markers).
  2. Hardware Status against the expected hardware (--expect from the gate's
     board profile; without it the legacy rule: IMU and baro shown [N/A],
     the IVP-142c presentation guard). Unexpected parts are warnings only.
  3. Health pipeline populated (diag_stats [Health] line: secondary byte
     non-zero, critical 0, regression guard for AO_HealthMonitor-on-station
     under capability-masking).
  4. Radio RegVersion=0x12 (SX1276 positive control), only when a radio is
     expected or detected; otherwise N/A by name.
  N/A checks print as "hw_na: <check> (<part> not fitted)" and unexpected
  hardware as "hw_warn: hw-unexpected:<role>:<part> ..." for the gate.

Hardware target: REQUIRES STATION FIRMWARE — refuses to run on vehicle.
The script auto-detects USB CDC ports by VID:PID 0x2E8A:0x0009 and
classifies the banner. If no station banner ('Ground Station',
'Fruit Jam', 'Station RX') is found, exits cleanly with code 2 ("no
station to test"). The pre-push gate treats exit 2 as FAIL when the
push needs the station bench (every firmware or gate push, since
2026-10-08): a skipped station is a gate that did not run.

DO NOT remove the station guard. Tests 2 and 3 navigate via the debug
menu, where 'x' from main menu is bound to Erase-Flights on vehicle
firmware. Sending 'x' to a misidentified target enters a destructive
confirmation prompt. See 2026-04-27 incident notes.

Wall-clock watchdog: --max-runtime (default 60s) is enforced by a daemon
thread that calls os._exit(2) when fired. This is the only reliable
escape from a hung serial.Serial() open on Windows USB CDC.

Port detection: ``find_target_port(TARGET_STATION_ANY)`` (RocketChip
VID:PID USB CDC only). ``--port`` overrides but is still banner-classified;
connection uses ``open_classified_port`` with ``auto_enter_cli_menu=False``
because ``goto_main_from_anywhere`` performs dashboard→menu navigation.

Exit codes:
    0 = every test that applies passes (N/A tests are named, not counted)
    1 = one or more test failures (regression — fix firmware/test)
    2 = no station present, tool self-check failed, or watchdog fired
        (the pre-push gate treats this as FAIL when the station is required)
"""

import argparse
import functools
import os
import re
import serial
import sys
import threading
import time

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)
from pathlib import Path

from _rc_test_common import (  # noqa: E402
    Banner,
    Role,
    banner_matches_elf_error,
    classify_banner,
    ensure_station_in_dashboard_state,
    EXIT_UPTIME_NOT_RISING,
    STATION_UPTIME_KEYS,
    check_uptime,
    uptime_gap_extra_s,
    STATION_UPTIME_GAP_S,
    EXIT_UPTIME_RESET,
    PORT_LOST_MSG,
    NO_DASH_UPTIME_MSG,
    HOLD_EXPIRED_MSG,
    board_config,
    role_usb_serial,
    TARGET_STATION_ANY,
    make_reconnect,
    report_usb_fault,
    wait_for_target_port,
    open_classified_port,
    print_board_usb_serial,
    peek_banner,
    rc_test,
    refuse_stale_tree_and_elf,
    ROCKETCHIP_USB_PID,
    ROCKETCHIP_USB_VID,
    TARGET_STATION_ANY,  # R-25-exec step 7: bench/flight collapsed, accept either
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
# No chip on the SPI bus reads back 0x00 or 0xFF (src/drivers/rfm95w.h:
# "returns 0x00/0xFF when not present"). The firmware prints the line anyway.
NO_RADIO_REG_VERSIONS = frozenset({'00', 'ff'})
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
# in src/cli/rc_os.cpp), so 'q' enters the debug menu (post-R-25-exec: always
# available for diagnostic reads; state-mutating commands gated by
# rc::test_mode_active()). The debug submenu's "back to main" key is
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


# Hardware Status names per part (print_*_status in rc_os_commands.cpp).
# Parts are generic (Nathan 2026-11-23: any IMU / any baro counts). Today the
# firmware's status print only names the ICM-20948 and DPS310 drivers; when it
# reports another IMU or baro, add the name the firmware prints here.
HW_STATUS_NAMES = {'imu': 'ICM-20948', 'baro': 'DPS310', 'gps': 'GPS'}
# Without --expect (a manual run): today's station rule, IMU and baro absent
# and shown [N/A], radio fitted. GPS is not judged.
LEGACY_EXPECT = None


def judge_hw_status(out, expect):
    """Hardware Status text -> (ok, detail, na, warns). Pure, host-tested.

    expect None: legacy rule (IMU and baro must show [N/A]).
    expect dict: per part, fitted = shown and not [N/A] / [FAIL];
    optional = checked when detected, N/A by name when not (never fails for
    being missing); absent = should show [N/A], if not that is a warning only
    (Nathan 2026-11-23: unexpected hardware warns); not-fitted = N/A by name.
    """
    na_hits = RE_NA_SENSOR.findall(out)
    if expect is None:
        if 'ICM-20948' not in na_hits:
            return False, f'IMU not shown as [N/A]: found {na_hits}', [], []
        if 'DPS310' not in na_hits:
            return False, f'Baro not shown as [N/A]: found {na_hits}', [], []
        if re.search(r'\[FAIL\]\s*ICM-20948', out):
            return False, '[FAIL] ICM-20948 seen — regression of N/A presentation', [], []
        if re.search(r'\[FAIL\]\s*DPS310', out):
            return False, '[FAIL] DPS310 seen — regression of N/A presentation', [], []
        return True, f'OK ({len(na_hits)} N/A)', [], []
    na, warns, bad = [], [], []
    for part, name in HW_STATUS_NAMES.items():
        state = expect[part]
        failed = re.search(r'\[FAIL\]\s*' + re.escape(name), out) is not None
        if state == board_config.NOT_FITTED:
            na.append(part)
        elif state == board_config.FITTED:
            if name in na_hits:
                bad.append(f'{name} expected but shown [N/A]')
            elif failed:
                bad.append(f'{name} expected but shown [FAIL]')
        elif state == board_config.OPTIONAL:
            if failed:
                bad.append(f'{name} (optional) fitted but shown [FAIL]')
            elif name in na_hits:
                na.append(part)
        else:  # absent
            if failed:
                bad.append(f'[FAIL] {name} seen — regression of N/A presentation')
            elif name not in na_hits:
                warns.append((part, f'{name} detected but not expected on this board'))
    if bad:
        return False, '; '.join(bad), na, warns
    return True, f'OK ({len(na_hits)} N/A shown)', na, warns


def test_hardware_status(port, state, verbose=False):
    """Hardware Status matches the expected hardware (IVP-142c presentation:
    an uninstalled sensor renders [N/A], never [FAIL]).

    Path: from [main] → 'q' (debug menu) → 'b' (HW status print).
    Returns to main with Esc (NOT 'x' — 'x' from main is Erase-Flights).
    """
    goto_main_from_anywhere(port)
    send_and_read(port, 'q', 1.5)            # → [debug]
    out = send_and_read(port, 'b', 4.0)      # cli_print_hw_status
    send_and_read(port, ESC, 1.0)            # → back to [main]

    state['hw'] = out
    ok, detail, na, warns = judge_hw_status(out, state.get('expect'))
    state.setdefault('hw_na', []).extend(('Hardware Status', p) for p in na)
    state.setdefault('hw_warn', []).extend(warns)
    return ok, detail


def judge_health(out):
    """diag_stats text -> (ok, detail, groups). Health pipeline only, every
    station (secondary non-zero, critical 0). Pure, host-tested."""
    m = RE_HEALTH_LINE.search(out)
    if not m:
        return False, '[Health] line not found in diag_stats output', None
    prim, sec, crit, mcu, gng = m.groups()
    if int(sec, 16) == 0:
        return False, (f'secondary=0x{sec} — AO_HealthMonitor not running? '
                       f'(expected non-zero with Flash+WDT+PIO bits)'), m.groups()
    if int(crit, 16) != 0:
        return False, f'critical=0x{crit} (expected 0x00 on healthy station)', m.groups()
    return True, f'OK primary=0x{prim} secondary=0x{sec} mcu={mcu}', m.groups()


def judge_radio_version(out, radio_state):
    """(ok or None for N/A, detail, warns). RegVersion=0x12 is the SX1276
    positive control (council, IVP-132a.4); it runs only when a radio is
    expected (Nathan 2026-11-23). radio_state None: legacy, radio required."""
    rv = RE_REG_VERSION.search(out)
    if radio_state == board_config.NOT_FITTED:
        return None, 'radio not fitted', []
    if radio_state == board_config.OPTIONAL and (
            not rv or rv.group(1).lower() in NO_RADIO_REG_VERSIONS):
        return None, 'optional radio not detected', []
    if radio_state == board_config.ABSENT:
        if rv and rv.group(1).lower() == '12':
            return True, 'no radio expected', [('radio', 'SX1276 RegVersion=0x12 '
                                                'read but no radio expected')]
        return True, 'no radio expected', []
    if not rv:
        return False, 'RegVersion not found in T=0 preconditions', []
    if rv.group(1).lower() != '12':
        return False, f'RegVersion=0x{rv.group(1)} (expected 0x12)', []
    return True, 'OK RegVersion=0x12', []


def test_health_pipeline(port, state, verbose=False):
    """diag_stats [Health] line: secondary non-zero, critical 0.

    Path: from [main] → 'q' (debug, dev-only) → 'd' (diag_stats_dump).
    Returns to main with Esc. The radio check reads the same dump.
    """
    send_and_read(port, 'q', 1.5)            # → [debug]
    out = send_and_read(port, 'd', 5.0)      # diag_stats_dump
    send_and_read(port, ESC, 1.0)            # → back to [main]

    state['diag'] = out
    ok, detail, groups = judge_health(out)
    if groups:
        state['health'] = groups
    return ok, detail


def test_radio_version(port, state, verbose=False):
    """Radio positive control from the diag dump read by the health test."""
    exp = state.get('expect')
    ok, detail, warns = judge_radio_version(state.get('diag', ''),
                                            exp['radio'] if exp else None)
    state.setdefault('hw_warn', []).extend(warns)
    if ok is None:
        state.setdefault('hw_na', []).append(('Radio RegVersion', 'radio'))
    return ok, detail


TESTS = [
    ('Boot + main menu',          test_boot_and_main_menu),
    ('Hardware Status',           test_hardware_status),
    ('Health pipeline populated', test_health_pipeline),
    ('Radio RegVersion',          test_radio_version),
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

    # Ensure the station board is in a known-good initial state (dashboard)
    # before any classification or test logic runs. After flash/reboot/'x'
    # the board can legitimately be in kmenu. We drive it to dashboard here
    # so the existing safety guards + navigation helpers see the expected
    # banner. This is the "ensure good state" fix for the default dashboard
    # change (2026-05-27).
    if meta.role == Role.STATION:
        ensure_station_in_dashboard_state(ser)
        # Uptime check. A vanished port is waited for (RC_PORT_WAIT_S, USB
        # serial first) and reopened: back lower = reset (4); never back =
        # port lost (3). USB only, no probe.
        verdict, ups, new_ser = check_uptime(
            ser, STATION_UPTIME_KEYS,
            reconnect=make_reconnect(TARGET_STATION_ANY, ser,
                                     override=getattr(args, 'port', None),
                                     prefer_serial=getattr(args, '_role_ser', None),
                                     wait=functools.partial(
                                         wait_for_target_port,
                                         strict=getattr(args, '_strict', False))))
        print(f'  uptime check (dashboard Up s over 3 polls): {ups}')
        if verdict != 'ok':
            last = next((u for u in reversed(ups) if u is not None), None)
            if verdict == 'reset':
                report_usb_fault('uptime dropped during the check (board reset)',
                                 'station', appeared=True, seconds=None,
                                 port=port_name, last_uptime=last)
                sys.exit(EXIT_UPTIME_RESET)
            report_usb_fault(verdict if verdict in (PORT_LOST_MSG, NO_DASH_UPTIME_MSG, HOLD_EXPIRED_MSG)
                             else 'uptime not rising',
                             'station', appeared=True, seconds=None,
                             port=port_name, last_uptime=last)
            sys.exit(EXIT_UPTIME_NOT_RISING)
        if new_ser is not ser:      # port came back mid-check; keep using it
            ser = new_ser
            ensure_station_in_dashboard_state(ser)

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
    current = classify_banner(sanity)

    # Proper fix for the post-default-dashboard change (2026-05-27):
    # The discovery-time classification (meta) already confirmed this is a station
    # board (via VID/PID + station tokens). We must not let a transient kmenu
    # banner flip the *role* decision and refuse the run. kmenu vs dashboard
    # is only a navigation concern now.
    #
    # We still protect against true role mismatches (e.g. hot-swap to a vehicle).
    if current.role == Role.VEHICLE and meta.role != Role.STATION and not allow_vehicle:
        print('ERROR: post-open sanity check classifies firmware as VEHICLE.')
        print('  Refusing to run station-only key sequences on vehicle firmware.')
        print('  (Did you hot-swap boards? Or is build_station_flight out of date?)')
        sys.exit(2)

    # If we are a known station but currently in kmenu, the ensure helper above
    # should have driven us to dashboard. If not, we still proceed (the tests
    # themselves contain navigation logic), but we log for visibility.
    if meta.role == Role.STATION and current.role != Role.STATION:
        print('INFO: station board presented kmenu on sanity read; proceeding with discovery role (station).')

    print(f'\n=== RocketChip Station Bench Sim ===')
    print(f'  Port: {port_name}')
    print(f'  Classified: {meta.short_summary()}')
    print(f'  Max runtime: {args.max_runtime:.0f}s (hard-killed by watchdog)')
    print()

    state = {'expect': getattr(args, '_expect', None)}
    passed = 0
    failed = 0
    skipped = 0
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
        status = 'N/A ' if ok is None else ('PASS' if ok else 'FAIL')
        if ok is None:
            skipped += 1
        elif ok:
            passed += 1
        else:
            failed += 1
        suffix = f' -- {detail}' if (ok is None or not ok or args.verbose) else ''
        print(f'  [{status}]  {i + 1}. {name}{suffix}')

    all_output = ''.join(state.get(k, '') for k in ('boot', 'hw', 'diag'))
    hits = scan_asserts(all_output)
    if hits:
        print(f'  [FAIL]  QP/fault assertion markers in output: {hits}')
        failed += 1
    else:
        print(f'  [PASS]  No QP/fault assertions across all tests')

    for check, part in state.get('hw_na', []):
        print(board_config.hw_na_line(check, part))
    for part, what in state.get('hw_warn', []):
        print(board_config.hw_warn_line('station', part, what))
    elapsed = time.time() - start
    total = passed + failed
    na = f', {skipped} N/A' if skipped else ''
    print(f'\n  RESULT: {passed}/{total} PASS{na}  ({elapsed:.1f}s)')
    print('=' * 38)

    return 0 if failed == 0 else 1


@rc_test(target=TARGET_STATION_ANY)  # R-25-exec step 7: bench/flight collapsed
def main():
    parser = argparse.ArgumentParser(
        description='RocketChip Station Bench Sim (IVP-146, 4 tests)')
    parser.add_argument('--port', default=os.environ.get("USBDEV") or os.environ.get("STATION_USBDEV"),
                        help='Serial port (auto-detected via VID:PID + banner; honors $USBDEV or $STATION_USBDEV for stable /dev/serial/by-id/ in WSL)')
    parser.add_argument('--verbose', action='store_true',
                        help='Print verbose test output')
    parser.add_argument('--max-runtime', type=float, default=60.0,
                        help='Wall-clock deadline (default 60s, hard-killed by watchdog)')
    parser.add_argument('--allow-non-station', action='store_true',
                        help=argparse.SUPPRESS)  # debugging only — never set in CI
    parser.add_argument('--commit', default=None,
                        help='Sha being pushed (pre-push gate). The ELF must be a '
                             'clean build of that firmware tree. Default: working tree.')
    parser.add_argument('--expect', default=None,
                        help='Expected hardware from the pre-push gate, e.g. '
                             'imu=absent,baro=absent,radio=fitted,gps=absent. '
                             'Default: the legacy station rule.')
    args = parser.parse_args()
    try:
        args._expect = board_config.parse_expect(args.expect)
    except board_config.BoardConfigError as e:
        print(f'ERROR: {e}')
        sys.exit(2)

    # Wall-clock watchdog: hard kill after deadline, regardless of where
    # we are in the test (including stuck inside serial.Serial() C calls).
    # RC_UPTIME_GAP_S (test-only) widens the uptime check; the watchdog gets
    # exactly that extra time (2 gaps + margin). Unset: --max-runtime as is.
    _start_watchdog(args.max_runtime + uptime_gap_extra_s(STATION_UPTIME_GAP_S))

    if args.allow_non_station and args.port is not None:
        # Unsafe path: manual port without strict target match (debugging).
        port_name = args.port
        meta = peek_banner(port_name)
    else:
        try:
            role_ser, strict = role_usb_serial('station')
        except board_config.BoardConfigError as e:
            print(f'ERROR: {e}')
            sys.exit(2)
        pw = wait_for_target_port(
            TARGET_STATION_ANY, override=args.port, verbose=args.verbose,
            prefer_serial=role_ser, strict=strict)
        args._role_ser, args._strict = role_ser, strict
        port_name, reason = pw.port, pw.meta
        if port_name is None:
            report_usb_fault('port never appeared' if not pw.appeared else 'not this role',
                             'station', appeared=pw.appeared, seconds=None,
                             last_uptime=None)
            print(f'INFO: No station detected — {reason}.')
            print('  Skipping station_bench_sim (this is exit code 2 = '
                  '"no station to test", not a test failure).')
            print('  To run: flash build_station_flight/*.uf2 and plug in USB '
                  f'(VID 0x{ROCKETCHIP_USB_VID:04X} PID 0x{ROCKETCHIP_USB_PID:04X}).')
            sys.exit(2)
        if not isinstance(reason, Banner):
            print('ERROR: internal: expected Banner from find_target_port')
            sys.exit(2)
        meta = reason

        repo = Path(_SCRIPTS_DIR).resolve().parent
        expected, ident_err = refuse_stale_tree_and_elf(repo, 'station', commit=args.commit)
        if ident_err:
            print('ERROR: leftover / unflashed image would make this gate a lie.')
            print(f'  {ident_err}')
            sys.exit(1)
        if meta.git_hash:
            banner_err = banner_matches_elf_error(meta, expected)
            if banner_err:
                print('ERROR: leftover / unflashed image would make this gate a lie.')
                print(f'  {banner_err}')
                sys.exit(1)
            print(f'  image: flight-{expected.git_hash} '
                  f'sha256={expected.sha256[:12]}…')
        else:
            print('  dashboard peek has no flight-<sha>; ELF flash record is '
                  'the attribution until kmenu (VERSIONING.md SWE-084)')

    print(f'using station port: {port_name}')
    print_board_usb_serial(port_name)
    if meta.is_known():
        print(f'  ({meta.short_summary()})')

    try:
        with open_classified_port(
            port_name,
            target=TARGET_STATION_ANY,  # R-25-exec step 7: bench/flight collapsed, accept either
            post_open_re_classify=False,  # We rely on discovery-time role + ensure_station_in_dashboard_state + relaxed script guard
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
