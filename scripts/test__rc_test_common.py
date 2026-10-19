#!/usr/bin/env python3
"""Host-side tests for scripts/_rc_test_common.py.

No hardware required. Run as:
    python scripts/test__rc_test_common.py

Council-required tests (2026-04-27 review):
  1. classify_banner correctly identifies vehicle vs station vs unknown.
  2. classify_banner extracts build_type, version, board.
  3. Target.matches respects role and build flags.
  4. Banner is frozen (cannot be mutated).
  5. Watchdog fires at the deadline and exits with code 2.
  6. @rc_test enforces the 0/1/2 exit-code contract:
       - main returns int -> sys.exit(int)
       - main returns None -> sys.exit(0)
       - main raises -> sys.exit(1) with stderr summary
       - main raises KeyboardInterrupt -> sys.exit(2)
       - main calls sys.exit(N) -> propagates as-is
  7. Decorator stashes target on both wrapper and original function.
  8. find_vehicle_and_station_ports rejects conflicting dual overrides (mocked).
  9. find_vehicle_and_station_ports assigns station then vehicle from mock scan.
  10. pre_commit_matrix matches staged path prefixes like the pre-commit hook.
  11. leftover banner flight-<sha> and rebuilt-unflashed ELF sha fail closed.
  12. board_usb_serial reads the USB serial from enumeration (pre-push note).

Exit code: 0 if all tests pass, 1 if any fail.
"""

from __future__ import annotations

import dataclasses
import os
import subprocess
import sys
import textwrap
import time

# Allow running from project root or from scripts/.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from _rc_test_common import (  # noqa: E402
    Banner, Build, Mode, Role, Target,
    TARGET_VEHICLE_ANY, TARGET_VEHICLE_FLIGHT,
    TARGET_STATION_ANY, TARGET_STATION_FLIGHT,
    TARGET_EITHER_ANY,
    classify_banner, passive_dump_needs_help, rc_test,
    ExpectedImage, banner_matches_elf_error, elf_sha256,
    flash_stamp_error, record_flashed_elf,
)


# Banner grammar SSOT: standards/VERSIONING.md (2026-08-26).
# 2026-04-27 captures kept as compatibility samples for the old RCOS field.

VEHICLE_FLIGHT_BANNER = """\
==============================================
  RocketChip 0.16.3-dev flight-deadbe
  Board: Adafruit Feather RP2350 HSTX
  Profile: Rocket  Uptime: 5s
==============================================
Hardware: 14/14 OK

h-Help  p-Preflight  c-Calibration  f-Flight
[main]
"""

# Live leftover after banner drain + 'h' (Buzz 2026-08-28, COM5 0.16.3-dev).
VEHICLE_HELP_AFTER_DRAIN = """\
h  help
x  erase flights
[main]
"""

STATION_KMENU_BANNER = """\
========================================
  RocketChip 0.16.3-dev flight-deadbe - Station RX
  Board: Adafruit Fruit Jam
========================================

Status:  h-Help  s-Sensor  b-Boot  p-Preflight
Radio:   t-Status  r-Rate  m-Mode(ANSI/CSV/MAVLink)
Station: g-GPS  d-Distance  p-GPS-Push
Command: a-ARM(confirm)  X-DISARM
Flight:  l-FlushLog  x-Erase
========================================
[main]
"""

STATION_DASHBOARD_BANNER = """\
=== RocketChip Ground Station ===
State: IDLE     MET: 0:00.0
Alt:  0.0 m
'm' mode cycle  'x' menu
"""

STATION_DASHBOARD_WAITING = """\
=== RocketChip Ground Station ===
Waiting for vehicle packets...
RX: 0 pkts  CRC err: 0
Uptime: 12s
'm' mode cycle  'x' menu
"""

# Garbled / unknown firmware: nothing recognisable.
UNKNOWN_BANNER = """\
random uart noise nothing useful here
just bytes
"""

# HEALTH-only CDC dump: long, no role tokens. peek_banner must still send 'h'.
HEALTH_RING_DUMP = (
    "[123456] HEALTH: Core1 -> FAULT\n"
    "[123556] HEALTH: Core1 RECOVERED\n"
) * 20


# ---------------------------------------------------------------------------
# Test harness
# ---------------------------------------------------------------------------

_failures: list[str] = []


def check(label: str, predicate: bool, detail: str = '') -> None:
    if predicate:
        print(f'  [PASS] {label}')
    else:
        msg = f'{label}' + (f' -- {detail}' if detail else '')
        print(f'  [FAIL] {msg}')
        _failures.append(msg)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_classify_vehicle_flight() -> None:
    print('test_classify_vehicle_flight')
    b = classify_banner(VEHICLE_FLIGHT_BANNER)
    check('role is VEHICLE',  b.role is Role.VEHICLE,      f'got {b.role}')
    check('build is FLIGHT',  b.build is Build.FLIGHT,     f'got {b.build}')
    check('version captured', b.version == '0.16.3-dev',       f'got {b.version!r}')
    check('board captured',   b.board is not None
          and 'feather rp2350 hstx' in (b.board or ''),    f'got {b.board!r}')
    check('is_vehicle()',     b.is_vehicle())
    check('is_station() False', not b.is_station())
    check('is_known()',       b.is_known())
    check('git_hash captured', b.git_hash == 'deadbe', f'got {b.git_hash!r}')


def test_classify_station_kmenu() -> None:
    print('test_classify_station_kmenu')
    b = classify_banner(STATION_KMENU_BANNER)
    check('role is STATION',  b.role is Role.STATION,      f'got {b.role}')
    check('board is Fruit Jam', b.board is not None
          and 'fruit jam' in (b.board or ''),              f'got {b.board!r}')
    check('is_station()',     b.is_station())
    check('mode is KMENU',    b.mode is Mode.KMENU,        f'got {b.mode}')
    check('in_kmenu()',       b.in_kmenu())
    check('not in_dashboard()', not b.in_dashboard())


def test_classify_station_dashboard() -> None:
    print('test_classify_station_dashboard')
    b = classify_banner(STATION_DASHBOARD_BANNER)
    check('role is STATION',  b.role is Role.STATION,      f'got {b.role}')
    check('build UNKNOWN (no tag)', b.build is Build.UNKNOWN, f'got {b.build}')
    check('mode is DASHBOARD', b.mode is Mode.DASHBOARD,   f'got {b.mode}')
    check('in_dashboard()',   b.in_dashboard())
    check('not in_kmenu()',   not b.in_kmenu())


def test_classify_station_dashboard_waiting() -> None:
    print('test_classify_station_dashboard_waiting')
    b = classify_banner(STATION_DASHBOARD_WAITING)
    check('role is STATION',  b.role is Role.STATION,      f'got {b.role}')
    check('mode is DASHBOARD', b.mode is Mode.DASHBOARD,   f'got {b.mode}')


def test_classify_vehicle_help_after_banner_drain() -> None:
    print('test_classify_vehicle_help_after_banner_drain')
    b = classify_banner(VEHICLE_HELP_AFTER_DRAIN)
    check('role is VEHICLE after h leftover', b.role is Role.VEHICLE, str(b))
    check('mode is KMENU', b.mode is Mode.KMENU, str(b))


def test_classify_vehicle_modes() -> None:
    print('test_classify_vehicle_modes')
    b = classify_banner(VEHICLE_FLIGHT_BANNER)
    check('vehicle flight is KMENU', b.mode is Mode.KMENU, f'got {b.mode}')


def test_classify_unknown() -> None:
    print('test_classify_unknown')
    b = classify_banner(UNKNOWN_BANNER)
    check('role UNKNOWN',     b.role is Role.UNKNOWN,      f'got {b.role}')
    check('build UNKNOWN',    b.build is Build.UNKNOWN,    f'got {b.build}')
    check('not is_known()',   not b.is_known())


def test_health_ring_dump_needs_help() -> None:
    print('test_health_ring_dump_needs_help')
    b = classify_banner(HEALTH_RING_DUMP)
    check('HEALTH-only dump is UNKNOWN', b.role is Role.UNKNOWN, str(b))
    check('dump is long (>= 64)', len(HEALTH_RING_DUMP) >= 64,
          f'len={len(HEALTH_RING_DUMP)}')
    check('long UNKNOWN dump still needs h',
          passive_dump_needs_help(b))
    vf = classify_banner(VEHICLE_FLIGHT_BANNER)
    check('classified vehicle does not need h',
          not passive_dump_needs_help(vf))
    mixed = classify_banner(HEALTH_RING_DUMP + VEHICLE_HELP_AFTER_DRAIN)
    check('HEALTH + help leftover is vehicle',
          mixed.role is Role.VEHICLE, str(mixed))
    check('classified mixed dump does not need h',
          not passive_dump_needs_help(mixed))


def test_classify_empty() -> None:
    print('test_classify_empty')
    for empty in ('', '   ', None):  # type: ignore[arg-type]
        b = classify_banner(empty)  # type: ignore[arg-type]
        check(f'empty {empty!r} -> UNKNOWN',
              b.role is Role.UNKNOWN and b.build is Build.UNKNOWN)


def test_target_matches() -> None:
    print('test_target_matches')
    vf = classify_banner(VEHICLE_FLIGHT_BANNER)
    sk = classify_banner(STATION_KMENU_BANNER)
    sd = classify_banner(STATION_DASHBOARD_BANNER)  # build=UNKNOWN
    uk = classify_banner(UNKNOWN_BANNER)

    # vehicle-any matches vehicle banners.
    check('VEHICLE_ANY matches vehicle-flight', TARGET_VEHICLE_ANY.matches(vf))
    check('VEHICLE_ANY rejects station',        not TARGET_VEHICLE_ANY.matches(sk))
    check('VEHICLE_ANY rejects unknown',        not TARGET_VEHICLE_ANY.matches(uk))

    # vehicle-flight matches vehicle-flight banner.
    check('VEHICLE_FLIGHT matches vehicle-flight',
          TARGET_VEHICLE_FLIGHT.matches(vf))
    check('VEHICLE_FLIGHT rejects station-kmenu',
          not TARGET_VEHICLE_FLIGHT.matches(sk))

    # station-any matches station banners.
    check('STATION_ANY matches station-kmenu',  TARGET_STATION_ANY.matches(sk))
    check('STATION_ANY matches station-dashboard',
          TARGET_STATION_ANY.matches(sd))  # build UNKNOWN -> permissive
    check('STATION_ANY rejects vehicle',        not TARGET_STATION_ANY.matches(vf))

    # station-flight permissive on UNKNOWN build (dashboard banner has no tag).
    check('STATION_FLIGHT permissive on UNKNOWN build',
          TARGET_STATION_FLIGHT.matches(sd))

    # EITHER matches both vehicle and station.
    check('EITHER matches vehicle',  TARGET_EITHER_ANY.matches(vf))
    check('EITHER matches station',  TARGET_EITHER_ANY.matches(sk))
    check('EITHER rejects unknown',  not TARGET_EITHER_ANY.matches(uk))


def test_banner_frozen() -> None:
    print('test_banner_frozen')
    b = classify_banner(VEHICLE_FLIGHT_BANNER)
    try:
        # dataclasses.replace is allowed; direct mutation is not.
        b.role = Role.STATION  # type: ignore[misc]
    except dataclasses.FrozenInstanceError:
        check('Banner frozen (FrozenInstanceError on mutate)', True)
        return
    check('Banner frozen (FrozenInstanceError on mutate)', False,
          'mutation succeeded (frozen=True missing)')


def test_target_frozen() -> None:
    print('test_target_frozen')
    t = TARGET_VEHICLE_FLIGHT
    try:
        t.role = Role.STATION  # type: ignore[misc]
    except dataclasses.FrozenInstanceError:
        check('Target frozen', True)
        return
    check('Target frozen', False, 'mutation succeeded')


def test_target_str() -> None:
    print('test_target_str')
    check('VEHICLE_ANY str',     str(TARGET_VEHICLE_ANY)     == 'vehicle-any')
    check('VEHICLE_FLIGHT str',  str(TARGET_VEHICLE_FLIGHT)  == 'vehicle-flight')
    check('STATION_FLIGHT str',  str(TARGET_STATION_FLIGHT)  == 'station-flight')


# ---------------------------------------------------------------------------
# Decorator + watchdog tests via subprocess
# ---------------------------------------------------------------------------

def _run_subprocess_script(body: str, timeout_s: float = 10.0) -> tuple[int, str, str]:
    """Run a small Python program in a subprocess. Returns (rc, stdout, stderr)."""
    proc = subprocess.run(
        [sys.executable, '-u', '-c', body],
        capture_output=True, text=True, timeout=timeout_s,
        env={**os.environ, 'PYTHONDONTWRITEBYTECODE': '1'},
    )
    return proc.returncode, proc.stdout, proc.stderr


def test_decorator_returns_int() -> None:
    print('test_decorator_returns_int')
    body = textwrap.dedent(f"""
        import sys
        sys.path.insert(0, {os.path.dirname(os.path.abspath(__file__))!r})
        from _rc_test_common import rc_test, TARGET_VEHICLE_ANY

        @rc_test(target=TARGET_VEHICLE_ANY)
        def main():
            return 7

        main()
    """)
    rc, _out, _err = _run_subprocess_script(body)
    check('main returns 7 -> exit 7', rc == 7, f'got {rc}')


def test_decorator_returns_none() -> None:
    print('test_decorator_returns_none')
    body = textwrap.dedent(f"""
        import sys
        sys.path.insert(0, {os.path.dirname(os.path.abspath(__file__))!r})
        from _rc_test_common import rc_test, TARGET_VEHICLE_ANY

        @rc_test(target=TARGET_VEHICLE_ANY)
        def main():
            print('main ran')

        main()
    """)
    rc, _out, _err = _run_subprocess_script(body)
    check('main returns None -> exit 0', rc == 0, f'got {rc}')


def test_decorator_uncaught_exception() -> None:
    print('test_decorator_uncaught_exception')
    body = textwrap.dedent(f"""
        import sys
        sys.path.insert(0, {os.path.dirname(os.path.abspath(__file__))!r})
        from _rc_test_common import rc_test, TARGET_VEHICLE_ANY

        @rc_test(target=TARGET_VEHICLE_ANY)
        def main():
            raise RuntimeError('boom')

        main()
    """)
    rc, _out, err = _run_subprocess_script(body)
    check('uncaught exception -> exit 1', rc == 1, f'got {rc}')
    check('stderr mentions exception',
          'RuntimeError' in err and 'boom' in err,
          f'stderr={err!r}')


def test_decorator_keyboard_interrupt() -> None:
    print('test_decorator_keyboard_interrupt')
    body = textwrap.dedent(f"""
        import sys
        sys.path.insert(0, {os.path.dirname(os.path.abspath(__file__))!r})
        from _rc_test_common import rc_test, TARGET_VEHICLE_ANY

        @rc_test(target=TARGET_VEHICLE_ANY)
        def main():
            raise KeyboardInterrupt

        main()
    """)
    rc, _out, err = _run_subprocess_script(body)
    check('KeyboardInterrupt -> exit 2', rc == 2, f'got {rc}')
    check('stderr mentions interrupt', 'interrupted' in err.lower(), f'stderr={err!r}')


def test_decorator_sys_exit_passthrough() -> None:
    print('test_decorator_sys_exit_passthrough')
    body = textwrap.dedent(f"""
        import sys
        sys.path.insert(0, {os.path.dirname(os.path.abspath(__file__))!r})
        from _rc_test_common import rc_test, TARGET_VEHICLE_ANY

        @rc_test(target=TARGET_VEHICLE_ANY)
        def main():
            sys.exit(2)  # explicit skip

        main()
    """)
    rc, _out, _err = _run_subprocess_script(body)
    check('sys.exit(2) propagates', rc == 2, f'got {rc}')


def test_decorator_target_introspection() -> None:
    print('test_decorator_target_introspection')

    @rc_test(target=TARGET_STATION_FLIGHT, watchdog_s=42.0)
    def fake_main() -> int:
        return 0

    check('wrapper.__rc_target__ is set',
          getattr(fake_main, '__rc_target__', None) == TARGET_STATION_FLIGHT)
    check('wrapper.__rc_watchdog_s__ is set',
          getattr(fake_main, '__rc_watchdog_s__', None) == 42.0)


def test_watchdog_fires() -> None:
    print('test_watchdog_fires')
    body = textwrap.dedent(f"""
        import sys, time
        sys.path.insert(0, {os.path.dirname(os.path.abspath(__file__))!r})
        from _rc_test_common import rc_test, TARGET_VEHICLE_ANY

        @rc_test(target=TARGET_VEHICLE_ANY, watchdog_s=1.0)
        def main():
            time.sleep(30)  # watchdog should kill us first
            return 0

        main()
    """)
    t0 = time.time()
    rc, _out, err = _run_subprocess_script(body, timeout_s=10.0)
    elapsed = time.time() - t0
    check('watchdog -> exit 2', rc == 2, f'got {rc}')
    check('watchdog fires within 3s', elapsed < 3.0, f'took {elapsed:.2f}s')
    check('watchdog stderr message', 'WATCHDOG' in err, f'stderr={err!r}')


def test_find_vehicle_and_station_rejects_same_port() -> None:
    """find_vehicle_and_station_ports refuses identical overrides."""
    print('test_find_vehicle_and_station_rejects_same_port')
    from unittest.mock import patch

    from _rc_test_common import (
        TARGET_STATION_ANY,
        TARGET_VEHICLE_ANY,
        classify_banner,
        find_vehicle_and_station_ports,
    )

    b_st = classify_banner(STATION_KMENU_BANNER)
    b_v = classify_banner(VEHICLE_FLIGHT_BANNER)

    def fake_ftp(target, override=None, verbose=False):  # noqa: ARG001
        if target is TARGET_STATION_ANY:
            return 'COM99', b_st
        if target is TARGET_VEHICLE_ANY:
            return 'COM99', b_v
        return None, 'bad target'

    with patch('_rc_test_common.find_target_port', side_effect=fake_ftp):
        vp, sp, vb, sb, reason = find_vehicle_and_station_ports(
            station_override='COM99', vehicle_override='COM99')

    check('ports None on clash', vp is None and sp is None, f'{vp=} {sp=}')
    check('same port in reason',
          'same port' in reason.lower(),
          reason)


def test_find_vehicle_and_station_discovers_pair() -> None:
    """No overrides: assigns first classified station then vehicle."""
    print('test_find_vehicle_and_station_discovers_pair')
    from unittest.mock import patch

    from _rc_test_common import find_vehicle_and_station_ports

    b_st = classify_banner(STATION_KMENU_BANNER)
    b_v = classify_banner(VEHICLE_FLIGHT_BANNER)

    with patch('_rc_test_common._candidate_ports',
               return_value=['COM9', 'COM10']), \
         patch('_rc_test_common.peek_banner', side_effect=[b_st, b_v]):
        vp, sp, vb, sb, reason = find_vehicle_and_station_ports()

    check('no error string', reason == '', repr(reason))
    check('stations COM9',
          sp == 'COM9' and sb is not None and sb.role.name == 'STATION',
          f'{sp=} {sb=}')
    check('vehicle COM10',
          vp == 'COM10' and vb is not None and vb.role.name == 'VEHICLE',
          f'{vp=} {vb=}')

def test_board_usb_serial_from_enumeration() -> None:
    """The bench prints the benched board's USB serial (pre-push note
    ``boards:`` line). Enumeration only, mocked here."""
    print('test_board_usb_serial_from_enumeration')
    import io
    from contextlib import redirect_stdout
    from types import SimpleNamespace
    from unittest.mock import patch

    from _rc_test_common import board_usb_serial, print_board_usb_serial

    from _rc_test_common import ROCKETCHIP_USB_PID, ROCKETCHIP_USB_VID
    ports = [SimpleNamespace(device='COM5', serial_number='E6614C311B4A2E2F',
                             vid=ROCKETCHIP_USB_VID, pid=ROCKETCHIP_USB_PID),
             SimpleNamespace(device='COM7', serial_number=None,
                             vid=ROCKETCHIP_USB_VID, pid=ROCKETCHIP_USB_PID),
             SimpleNamespace(device='COM4', serial_number='E663AC91D3487137',
                             vid=0x2E8A, pid=0x000C)]
    with patch('serial.tools.list_ports.comports', return_value=ports):
        check('serial of COM5', board_usb_serial('COM5') == 'E6614C311B4A2E2F')
        check('no serial -> None', board_usb_serial('COM7') is None)
        check('absent port -> None', board_usb_serial('COM9') is None)
        check('debug probe port (PID 0x000C) -> None, never a board',
              board_usb_serial('COM4') is None)
        buf = io.StringIO()
        with redirect_stdout(buf):
            print_board_usb_serial('COM7')
        check('unknown is printed, never an empty value',
              buf.getvalue() == 'board_usb_serial: unknown\n', buf.getvalue())


def test_usb_fault_handling() -> None:
    """Port wait budget, USB-serial preference, uptime check exit 2 / 3 / 4.
    Serial and clock mocked; no board."""
    print('test_usb_fault_handling')
    import io as _io
    from contextlib import redirect_stdout
    import _rc_test_common as rc

    class Clock:
        def __init__(self): self.t = 0.0
        def now(self): return self.t
        def sleep(self, s): self.t += s
    clk = Clock()
    calls = []
    def find_never(target, override=None, verbose=False):
        calls.append(override); return None, 'no RocketChip USB CDC ports plugged in'
    buf = _io.StringIO()
    with redirect_stdout(buf):
        pw = rc.wait_for_target_port(rc.TARGET_VEHICLE_ANY, budget_s=20.0,
                                     find=find_never, ports=lambda: {},
                                     sleep=clk.sleep, clock=clk.now)
    check('never appears -> no port, appeared False', pw.port is None and not pw.appeared)
    check('wait is bounded by the budget (<= 20 s)', clk.t <= 20.0 and len(calls) >= 10,
          f'{clk.t} {len(calls)}')

    clk = Clock(); n = [0]
    def find_late(target, override=None, verbose=False):
        n[0] += 1
        return ('COM9', 'BANNER') if n[0] >= 4 else (None, 'none')
    with redirect_stdout(_io.StringIO()):
        pw = rc.wait_for_target_port(rc.TARGET_VEHICLE_ANY, budget_s=20.0,
                                     find=find_late, ports=lambda: {'S1': 'COM9'},
                                     sleep=clk.sleep, clock=clk.now)
    check('late port within budget -> found, time logged',
          pw.port == 'COM9' and pw.seconds == 3.0, str(pw))

    got = []
    def find_rec(target, override=None, verbose=False):
        got.append(override); return override, 'BANNER'
    with redirect_stdout(_io.StringIO()):
        pw = rc.wait_for_target_port(rc.TARGET_STATION_ANY, prefer_serial='abc1',
                                     find=find_rec,
                                     ports=lambda: {'ABC1': 'COM12', 'XYZ': 'COM3'},
                                     sleep=clk.sleep, clock=clk.now)
    check('USB serial preferred over COM number, still classified',
          got == ['COM12'] and pw.method == 'USB serial abc1', str(got))

    seq = iter([10, 12, 14])
    v, vals, _ = rc.check_uptime(None, (b'z',), read=lambda s, k: next(seq),
                                 sleep=lambda s: None)
    check('uptime rising over 3 polls -> pass', v == 'ok' and vals == [10, 12, 14])
    seq = iter([10, 10, 10])
    v, _, _ = rc.check_uptime(None, (b'z',), read=lambda s, k: next(seq),
                              sleep=lambda s: None)
    check('uptime frozen -> not rising (exit 3)', v == 'not rising')
    seq = iter([10, None, 14, 18])
    v, vals, _ = rc.check_uptime(None, (b'z',), read=lambda s, k: next(seq),
                                 sleep=lambda s: None)
    check('station: a None poll is retried once -> 3 real rising readings pass',
          v == 'ok' and vals == [10, 14, 18], f'{v} {vals}')
    seq = iter([10, None, None, 18])
    v, vals, _ = rc.check_uptime(None, (b'z',), read=lambda s, k: next(seq),
                                 sleep=lambda s: None)
    check('station: still None after the retry -> no dashboard uptime after retry (exit 3)',
          v == rc.NO_DASH_UPTIME_MSG and 'after retry' in v, f'{v} {vals}')
    seq = iter([10, None, 3, 6])
    v, _, _ = rc.check_uptime(None, (b'z',), read=lambda s, k: next(seq),
                              sleep=lambda s: None)
    check('station: retry then a drop -> reset (exit 4)', v == 'reset')
    slept = []
    seq = iter([10, 12, 14])
    rc.check_uptime(None, (b'z',), read=lambda s, k: next(seq), sleep=slept.append)
    check('RC_UPTIME_GAP_S unset -> default 1.5 s gap', slept == [1.5, 1.5], str(slept))
    os.environ['RC_UPTIME_GAP_S'] = '20'
    try:
        slept = []
        seq = iter([10, None, 14, 18])
        v, vals, _ = rc.check_uptime(None, (b'z',), read=lambda s, k: next(seq),
                                     sleep=slept.append)
        vslept = []
        vv, _, _ = rc.vehicle_reopen_uptimes(
            rc.TARGET_VEHICLE_ANY, wait=lambda t, override=None, prefer_serial=None:
            rc.PortWait('COM5', 'B', True, 0.0, 'x'), read=lambda p, it=iter([1, 4, 7]): next(it), sleep=vslept.append)
    finally:
        del os.environ['RC_UPTIME_GAP_S']
    check('RC_UPTIME_GAP_S=20 -> 20 s between station readings, retry gap unchanged',
          v == 'ok' and slept == [20.0, 1.5, 20.0], str(slept))
    check('RC_UPTIME_GAP_S=20 -> 20 s between vehicle reopens, settle unchanged',
          vv == 'ok' and vslept == [2.0, 20.0, 20.0], str(vslept))
    check('RC_UPTIME_GAP_S unset -> no watchdog extension (60 s / 120 s as is)',
          rc.uptime_gap_extra_s(1.5) == 0.0 and rc.uptime_gap_extra_s(3.0) == 0.0)
    for bad in ('', 'x', '0', '-5', '1'):
        os.environ['RC_UPTIME_GAP_S'] = bad
        try:
            ok_bad = rc.uptime_gap_extra_s(1.5) == 0.0
        finally:
            del os.environ['RC_UPTIME_GAP_S']
        check(f'RC_UPTIME_GAP_S={bad!r} -> no watchdog extension', ok_bad)
    os.environ['RC_UPTIME_GAP_S'] = '20'
    try:
        st, ve = rc.uptime_gap_extra_s(1.5), rc.uptime_gap_extra_s(3.0)
    finally:
        del os.environ['RC_UPTIME_GAP_S']
    check('RC_UPTIME_GAP_S=20 -> station watchdog +47 s (2*18.5+10), vehicle +44 s',
          st == 47.0 and ve == 44.0, f'{st} {ve}')
    # RC_UPTIME_HOLD_S: keep reading after the 3 until reset / loss / expiry.
    import contextlib, io as _io
    class HC:
        def __init__(self): self.t = 0.0
        def now(self): return self.t
        def sleep(self, d): self.t += d
    def hold_run(fn, **kw):
        out = _io.StringIO()
        os.environ['RC_UPTIME_HOLD_S'] = '30'
        try:
            with contextlib.redirect_stdout(out):
                r = fn(**kw)
        finally:
            del os.environ['RC_UPTIME_HOLD_S']
        return r, out.getvalue()
    hc = HC(); seq = iter([10, 12, 14, 16, 18, 3])
    (v, vals, _), out = hold_run(rc.check_uptime, ser=None, keys=(b'z',),
                                 read=lambda s, k: next(seq), sleep=hc.sleep, clock=hc.now)
    check('hold, station: drop mid-hold -> reset (exit 4), HOLD line printed',
          v == 'reset' and vals[-1] == 3 and 'HOLD: press reset now (up to 30 s)' in out, f'{v} {vals} {out!r}')
    hc = HC()
    (v, vals, _), out = hold_run(rc.check_uptime, ser=None, keys=(b'z',),
                                 read=lambda s, k: 10 + int(hc.t), sleep=hc.sleep, clock=hc.now)
    check('hold, station: expires with no drop -> hold expired: no reset seen (exit 3)',
          v == rc.HOLD_EXPIRED_MSG == 'hold expired: no reset seen' and hc.t >= 30, f'{v} {hc.t}')
    def lost_in_hold(after):
        def read(ser, k):
            read.n += 1
            if ser == 'OLD' and read.n > 5:
                raise rc.serial.SerialException('device gone')
            return 100 + read.n if ser == 'OLD' else after
        read.n = 0
        return read
    hc = HC()
    (v, vals, ser), _ = hold_run(rc.check_uptime, ser='OLD', keys=(b'z',), read=lost_in_hold(2),
                                 sleep=hc.sleep, clock=hc.now, reconnect=lambda: 'NEW')
    check('hold, station: port lost, back lower -> reset (exit 4)',
          v == 'reset' and ser == 'NEW' and vals[-1] == 2, f'{v} {vals}')
    hc = HC()
    (v, vals, ser), _ = hold_run(rc.check_uptime, ser='OLD', keys=(b'z',), read=lost_in_hold(2),
                                 sleep=hc.sleep, clock=hc.now, reconnect=lambda: None)
    check('hold, station: port lost, never back -> port lost mid-check (exit 3)',
          v == rc.PORT_LOST_MSG and ser is None, f'{v} {vals}')
    PWh = rc.PortWait
    def vwait(fail_after=None):
        def w(target, override=None, prefer_serial=None):
            w.n += 1
            if fail_after is not None and w.n > fail_after:
                return PWh(None, 'gone', False, None, 'x')
            return PWh('COM5', 'B', True, 0.0, 'x')
        w.n = 0
        return w
    hc = HC(); vseq = iter([1, 4, 7, 9, 2])
    (v, vals, _), out = hold_run(rc.vehicle_reopen_uptimes, target=rc.TARGET_VEHICLE_ANY,
                                 wait=vwait(), read=lambda p: next(vseq),
                                 sleep=hc.sleep, clock=hc.now)
    check('hold, vehicle: reopen shows lower uptime (reset, port back) -> reset (exit 4)',
          v == 'reset' and vals == [1, 4, 7, 9, 2] and 'HOLD:' in out, f'{v} {vals}')
    hc = HC()
    (v, vals, _), _ = hold_run(rc.vehicle_reopen_uptimes, target=rc.TARGET_VEHICLE_ANY,
                               wait=vwait(fail_after=4), read=lambda p: 10 + int(hc.t),
                               sleep=hc.sleep, clock=hc.now)
    check('hold, vehicle: port never back -> port lost mid-check (exit 3)',
          v == rc.PORT_LOST_MSG, f'{v} {vals}')
    hc = HC()
    (v, vals, _), _ = hold_run(rc.vehicle_reopen_uptimes, target=rc.TARGET_VEHICLE_ANY,
                               wait=vwait(), read=lambda p: 10 + int(hc.t),
                               sleep=hc.sleep, clock=hc.now)
    check('hold, vehicle: expires with no drop -> hold expired: no reset seen (exit 3)',
          v == rc.HOLD_EXPIRED_MSG and hc.t >= 30, f'{v} {hc.t}')
    # Stale held-open handle: reads go empty (no raise) after the reset; the
    # hold must reopen and see the lower uptime, logging every poll.
    def stale_read(ser, k):
        stale_read.n += 1
        if ser == 'OLD':
            return 100 + stale_read.n if stale_read.n <= 4 else None
        return 2
    stale_read.n = 0
    hc = HC(); reo = []
    (v, vals, ser), out = hold_run(rc.check_uptime, ser='OLD', keys=(b'z',), read=stale_read,
                                   sleep=hc.sleep, clock=hc.now,
                                   reconnect=lambda: reo.append(1) or 'NEW', present=lambda: True)
    check('hold, station: stale handle reads empty -> reopen -> lower uptime -> reset (exit 4)',
          v == 'reset' and ser == 'NEW' and vals[-1] == 2 and len(reo) == 1, f'{v} {vals} {reo}')
    check('hold, station: one flushed HOLD poll line per poll (uptime, port, reopens)',
          'HOLD poll 1 ' in out and 'uptime 2s, port present, reopens 1 (reopened)' in out, out)
    hc = HC(); pres = iter([True, False, True])
    vseq = iter([1, 4, 7, 9, None, 3])
    (v, vals, _), out = hold_run(rc.vehicle_reopen_uptimes, target=rc.TARGET_VEHICLE_ANY,
                                 wait=vwait(), read=lambda p: next(vseq), sleep=hc.sleep,
                                 clock=hc.now, present=lambda: next(pres))
    check('hold, vehicle: port ABSENT poll, no-uptime poll, then lower uptime -> reset; each poll logged',
          v == 'reset' and vals[-1] == 3 and 'HOLD poll 2 ' in out and 'uptime no uptime, port ABSENT' in out
          and 'HOLD poll 3 ' in out, out)
    import threading as _th
    blk = _th.Event()
    t_open = rc._open_with_timeout(lambda p: blk.wait(5) and 'S', 'COM9', timeout_s=0.2)
    blk.set()
    check('banner open that blocks -> treated as no reading (None), never hangs', t_open is None)
    check('banner open that raises -> None',
          rc._open_with_timeout(lambda p: (_ for _ in ()).throw(OSError('x')), 'COM9', 0.5) is None)
    for bad in (None, '', 'x', '0', '-1'):
        if bad is not None:
            os.environ['RC_UPTIME_HOLD_S'] = bad
        try:
            n = []
            seq = iter([10, 12, 14, 16])
            out = _io.StringIO()
            with contextlib.redirect_stdout(out):
                v, vals, _ = rc.check_uptime(None, (b'z',), read=lambda s, k: (n.append(1), next(seq))[1],
                                             sleep=lambda s: None)
            ext = rc.uptime_gap_extra_s(1.5)
        finally:
            os.environ.pop('RC_UPTIME_HOLD_S', None)
        check(f'RC_UPTIME_HOLD_S={bad!r} -> unchanged: 3 reads, ok, no HOLD line, no watchdog extension',
              v == 'ok' and len(n) == 3 and 'HOLD' not in out.getvalue() and ext == 0.0)
    os.environ['RC_UPTIME_HOLD_S'] = '120'
    try:
        hx = rc.uptime_gap_extra_s(1.5)
    finally:
        del os.environ['RC_UPTIME_HOLD_S']
    check('RC_UPTIME_HOLD_S=120 -> station watchdog +156.5 s (120+1.5+20 port+5 banner+10)',
          hx == 156.5, str(hx))
    here = os.path.dirname(os.path.abspath(__file__))
    for script, const in (('station_bench_sim.py', 'STATION_UPTIME_GAP_S'),
                          ('bench_sim.py', 'VEHICLE_UPTIME_GAP_S')):
        with open(os.path.join(here, script), encoding='utf-8') as fh:
            src = fh.read()
        check(f'{script}: watchdog = --max-runtime + uptime_gap_extra_s({const})',
              f'_start_watchdog(args.max_runtime + uptime_gap_extra_s({const}))' in src
              and src.count('_start_watchdog(args.') == 1)

    # Station: port vanishes mid-check (read raises), reconnect after the wait.
    def vanish_then(vals_after):
        it = iter(vals_after)
        def read(ser, k):
            if ser == 'OLD' and read.n >= 1:
                raise rc.serial.SerialException('device gone')
            read.n += 1
            return 100 if ser == 'OLD' else next(it)
        read.n = 0
        return read
    v, vals, ser = rc.check_uptime('OLD', (b'z',), read=vanish_then([105, 108]),
                                   sleep=lambda s: None, reconnect=lambda: 'NEW')
    check('station: vanished, back with higher uptime -> pass on the new port',
          v == 'ok' and vals == [100, 105, 108] and ser == 'NEW', f'{v} {vals} {ser}')
    v, vals, _ = rc.check_uptime('OLD', (b'z',), read=vanish_then([2, 5]),
                                 sleep=lambda s: None, reconnect=lambda: 'NEW')
    check('station: vanished, back with lower uptime -> reset (exit 4)',
          v == 'reset', f'{v} {vals}')
    v, vals, ser = rc.check_uptime('OLD', (b'z',), read=vanish_then([]),
                                   sleep=lambda s: None, reconnect=lambda: None)
    check('station: vanished, never back -> port lost mid-check (exit 3)',
          v == rc.PORT_LOST_MSG == 'port lost mid-check' and ser is None, f'{v} {vals}')
    PW0 = rc.PortWait
    closed = []
    class Dead:
        def close(self): closed.append(True)
    waits0 = []
    def w_back(target, override=None, prefer_serial=None):
        waits0.append(prefer_serial); return PW0('COM12', 'B', True, 4.0, 'USB serial S')
    rcn = rc.make_reconnect(rc.TARGET_STATION_ANY, Dead(), prefer_serial='S',
                            wait=w_back, open_fn=lambda p: 'OPEN:' + p)
    check('make_reconnect: closes dead port, serial-first wait, reopens',
          rcn() == 'OPEN:COM12' and closed and waits0 == ['S'])
    rcn = rc.make_reconnect(rc.TARGET_STATION_ANY, Dead(),
                            wait=lambda t, override=None, prefer_serial=None:
                            PW0(None, 'gone', False, None, 'x'),
                            open_fn=lambda p: 'x')
    check('make_reconnect: never back -> None', rcn() is None)

    class FakeSer:
        def __init__(self, data): self.data = [data]; self.sent = []
        def write(self, b): self.sent.append(b)
        def flush(self): pass
        def read(self, n): return self.data.pop(0) if self.data else b''
    clk = Clock()
    fs = FakeSer(b'Stn GPS: 3D      9 sat  Up: 812s\r\n')
    v = rc.read_uptime(fs, rc.STATION_UPTIME_KEYS, sleep=clk.sleep, clock=clk.now)
    import inspect
    check('station dashboard "Up: Ns" parsed, only z sent', v == 812 and fs.sent == [b'z'])
    fs = FakeSer(b'  Profile: Rocket  Uptime: 95s\r\n')
    v = rc.read_uptime(fs, (), sleep=clk.sleep, clock=clk.now)
    check('vehicle banner "Uptime: Ns" parsed', v == 95)
    check("vehicle poll never sends 'b' (find-me beacon)",
          b'b' not in (rc.VEHICLE_UPTIME_KEYS or ()))
    import bench_sim
    bsrc = inspect.getsource(bench_sim)
    check("bench_sim: no 'b' key, no NOT CHECKED path, uses the reopen uptime check",
          "b'b'" not in bsrc and 'NOT CHECKED' not in bsrc
          and 'vehicle_reopen_uptimes' in bsrc)

    check('classify: rising -> ok', rc.classify_uptimes([4095, 4098, 4101]) == 'ok')
    check('classify: flat -> not rising', rc.classify_uptimes([10, 10, 13]) == 'not rising')
    check('classify: drop -> reset', rc.classify_uptimes([4095, 3, 6]) == 'reset')
    check('classify: missing sample -> not rising',
          rc.classify_uptimes([10, None, 16]) == 'not rising')
    PW = rc.PortWait
    waits = []
    def wait_ok(target, override=None, prefer_serial=None):
        waits.append(prefer_serial); return PW('COM5', 'B', True, 0.0, 'USB serial X')
    seq = iter([4095, 4098, 4101]); slept = []
    v, vals, port = rc.vehicle_reopen_uptimes(rc.TARGET_VEHICLE_ANY, prefer_serial='X',
                                              wait=wait_ok, read=lambda p: next(seq),
                                              sleep=slept.append)
    check('3 reopens, each re-resolved with the serial wait, gaps between',
          v == 'ok' and vals == [4095, 4098, 4101] and waits == ['X'] * 3
          and slept == [2.0, 3.0, 3.0] and port == 'COM5', f'{v} {vals} {waits} {slept}')
    seq = iter([50, 50, 50])
    v, _, _ = rc.vehicle_reopen_uptimes(rc.TARGET_VEHICLE_ANY, wait=wait_ok,
                                        read=lambda p: next(seq), sleep=lambda s: None)
    check('frozen uptime -> not rising (exit 3)', v == 'not rising')
    seq = iter([4095, 2, 5])
    v, _, _ = rc.vehicle_reopen_uptimes(rc.TARGET_VEHICLE_ANY, wait=wait_ok,
                                        read=lambda p: next(seq), sleep=lambda s: None)
    check('uptime drop -> reset (exit 4)', v == 'reset')
    n = [0]
    def wait_gone(target, override=None, prefer_serial=None):
        n[0] += 1
        return PW('COM5', 'B', True, 0.0, 'x') if n[0] == 1 else PW(None, 'gone', False, None, 'x')
    v, vals, _ = rc.vehicle_reopen_uptimes(rc.TARGET_VEHICLE_ANY, wait=wait_gone,
                                           read=lambda p: 100, sleep=lambda s: None)
    check('vehicle: vanished, never back -> port lost mid-check (exit 3)',
          v == rc.PORT_LOST_MSG and vals == [100], str(vals))
    # Vanished between reopens: the banner read fails once, the re-find
    # (normal wait) finds it again.
    for after, want, label in (([None, 4105, 4108], 'ok', 'higher uptime -> pass'),
                               ([None, 3, 6], 'reset', 'lower uptime -> reset (exit 4)')):
        seq = iter([4100] + after); waits.clear()
        v, vals, _ = rc.vehicle_reopen_uptimes(rc.TARGET_VEHICLE_ANY, prefer_serial='X',
                                               wait=wait_ok, read=lambda p: next(seq),
                                               sleep=lambda s: None)
        check(f'vehicle: vanished, back with {label}',
              v == want and None not in vals and waits == ['X'] * 4, f'{v} {vals} {waits}')
    # Buzz, build 3e9c38e2: first reopen gave no banner -> false exit 3.
    seq = iter([None, 52, 61, 70]); waits.clear(); slept2 = []
    v, vals, _ = rc.vehicle_reopen_uptimes(rc.TARGET_VEHICLE_ANY, prefer_serial='X',
                                           wait=wait_ok, read=lambda p: next(seq),
                                           sleep=slept2.append)
    check('vehicle: [None -> retry 52], 61, 70 -> pass (retry uses the serial-first wait)',
          v == 'ok' and vals == [52, 61, 70] and waits == ['X'] * 4
          and len(slept2) == 4, f'{v} {vals} {waits} {slept2}')
    seq = iter([None, None, 52, 61, 70]); waits.clear(); sl = []
    v, vals, _ = rc.vehicle_reopen_uptimes(rc.TARGET_VEHICLE_ANY, prefer_serial='X',
                                           wait=wait_ok, read=lambda p: next(seq),
                                           sleep=sl.append)
    check('vehicle: empty twice, then a reading -> pass (2 retries, 3 s apart, serial-first)',
          v == 'ok' and vals == [52, 61, 70] and waits == ['X'] * 5
          and sl[:3] == [2.0, 3.0, 3.0], f'{v} {vals} {waits} {sl}')
    seq = iter([None, None, None, 61, 70])
    v, vals, _ = rc.vehicle_reopen_uptimes(rc.TARGET_VEHICLE_ANY, wait=wait_ok,
                                           read=lambda p: next(seq), sleep=lambda s: None)
    check('vehicle: empty on all 3 tries -> no banner after retries (exit 3)',
          v == rc.NO_BANNER_MSG == 'no banner after retries' and vals == [None], f'{v} {vals}')
    seq = iter([None, 52, 61, 30])
    v, _, _ = rc.vehicle_reopen_uptimes(rc.TARGET_VEHICLE_ANY, wait=wait_ok,
                                        read=lambda p: next(seq), sleep=lambda s: None)
    check('vehicle: drop after a retry -> still reset (exit 4)', v == 'reset')

    class LateSer:   # banner arrives 4 s after open
        def __init__(self, clk): self.clk = clk; self.sent = False; self.closed = False
        def write(self, b): raise AssertionError('vehicle sends nothing')
        def flush(self): pass
        def read(self, n):
            if self.clk.t >= 4.0 and not self.sent:
                self.sent = True; return b'  Profile: Rocket  Uptime: 52s\r\n'
            return b''
        def close(self): self.closed = True
    clk = Clock(); ls = LateSer(clk)
    v = rc.banner_uptime('COM5', open_fn=lambda p: ls, sleep=clk.sleep, clock=clk.now)
    check('vehicle: banner 4 s late -> read (RC_BANNER_WAIT_S default 5), returns on match',
          v == 52 and clk.t < 4.5 and ls.closed, f'{v} {clk.t}')
    clk = Clock(); ls = LateSer(clk)
    os.environ['RC_BANNER_WAIT_S'] = '3'
    try:
        v = rc.banner_uptime('COM5', open_fn=lambda p: ls, sleep=clk.sleep, clock=clk.now)
    finally:
        del os.environ['RC_BANNER_WAIT_S']
    check('RC_BANNER_WAIT_S=3 -> 4 s late banner missed (configurable)', v is None)
    clk = Clock(); ls = LateSer(clk)
    v = rc.read_uptime(ls, (), listen_s=2.5, max_s=5.0, sleep=clk.sleep, clock=clk.now)
    check('station read: nothing in 2.5 s -> keeps listening up to RC_BANNER_WAIT_S', v == 52)
    src_open = inspect.getsource(rc.open_cdc_hold_dtr)
    check('vehicle open drops DTR, pauses, raises it (fresh connect edge)',
          'dtr_low_s' in src_open and rc.DTR_LOW_S > 0
          and 'dtr_low_s=DTR_LOW_S' in inspect.getsource(rc.banner_uptime))
    seq = iter([None, 52, 61, 3])
    v, _, _ = rc.vehicle_reopen_uptimes(rc.TARGET_VEHICLE_ANY, wait=wait_ok,
                                        read=lambda p: next(seq), sleep=lambda s: None)
    check('vehicle: retry, then a drop -> reset (exit 4)', v == 'reset')
    import bench_sim as _bs2
    check('bench_sim reports the no-banner verdict, not "uptime not rising"',
          'NO_BANNER_MSG' in inspect.getsource(_bs2))
    n2 = [0]
    def wait_gone2(target, override=None, prefer_serial=None):
        n2[0] += 1
        return PW('COM5', 'B', True, 0.0, 'x') if n2[0] == 1 else PW(None, 'gone', False, None, 'x')
    seq = iter([4100, None])
    v, _, _ = rc.vehicle_reopen_uptimes(rc.TARGET_VEHICLE_ANY, wait=wait_gone2,
                                        read=lambda p: next(seq), sleep=lambda s: None)
    check('vehicle: read fails and re-find fails -> port lost (never exit 2)',
          v == rc.PORT_LOST_MSG)
    import bench_sim as _bs
    _bsrc = inspect.getsource(_bs)
    check('bench_sim: exit 2 only before the uptime check (no "else 2" after it)',
          'else 2)' not in _bsrc and 'PORT_LOST_MSG' in _bsrc)
    sent = []
    class Ser:
        def __init__(self): self.d = [b'  Profile: Rocket  Uptime: 4095s\r\n']; self.closed = False
        def write(self, b): sent.append(b)
        def flush(self): pass
        def read(self, n): return self.d.pop(0) if self.d else b''
        def close(self): self.closed = True
    s1 = Ser(); clk = Clock()
    v = rc.banner_uptime('COM5', open_fn=lambda p: s1, sleep=clk.sleep, clock=clk.now)
    check('banner_uptime: parses the banner, sends nothing, closes the port',
          v == 4095 and sent == [] and s1.closed, f'{v} {sent}')
    check('exit codes 2 / 3 / 4 distinct',
          len({rc.EXIT_NO_BOARD, rc.EXIT_UPTIME_NOT_RISING, rc.EXIT_UPTIME_RESET}) == 3)
    check('exit codes: 2 no board, 3 frozen/port lost, 4 reset', rc.EXIT_NO_BOARD == 2 and rc.EXIT_UPTIME_NOT_RISING == 3 and rc.EXIT_UPTIME_RESET == 4)

    buf = _io.StringIO()
    with redirect_stdout(buf):
        rc.report_usb_fault('uptime not rising', 'station', appeared=True,
                            seconds=2.0, last_uptime=812, port='COM12')
    out = buf.getvalue()
    check('failure log: appeared, time, last uptime, transport, manual hint only',
          'port appeared: yes' in out and '2.0s' in out and 'last uptime: 812' in out
          and 'transport: USB CDC' in out and 'picotool info -f' in out
          and 'never run it' in out, out)
    import inspect
    src = inspect.getsource(rc.wait_for_target_port) + inspect.getsource(rc.check_uptime)
    check('no picotool / reflash call in the connect path',
          'subprocess' not in src and 'picotool' not in src)


def test_pre_commit_matrix_triggers() -> None:
    print('test_pre_commit_matrix_triggers')
    ci_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ci')
    if ci_dir not in sys.path:
        sys.path.insert(0, ci_dir)
    from pre_commit_matrix import match  # noqa: E402

    # Policy 2026-05-16 (commit a40e7f5, council unanimous, LL Entry 40):
    # ANY firmware-affecting path triggers the bench.
    # Role rule (Nathan, 2026-10-08): a firmware or gate change benches
    # BOTH roles; only an all-[station-only] change benches the station alone.

    # Pure-doc / non-firmware: neither gate.
    ft, st = match(['README.md'])
    check('no trigger on unrelated', not ft and not st)

    # Firmware path -> both roles (one source list builds both images).
    ft_f, st_f = match(['src/flight_director/flight_director.cpp'])
    check('flight_critical -> both roles', ft_f and st_f)

    # Station-role-only: station bench only.
    ft_s, st_s = match(['src/station/main.cpp'])
    check('station_only_station_bench', (not ft_s) and st_s)

    # Dashboard is station-role-only (pad ANSI), not vehicle ELF behavior.
    ft_d, st_d = match(['src/cli/rc_os_dashboard.cpp'])
    check('dashboard station_only', (not ft_d) and st_d)

    # rc_os_commands.cpp: flight (anything in src/ is firmware).
    ft_os, _ = match(['src/cli/rc_os_commands.cpp'])
    check('rc_os_cpp flight_critical', ft_os)

    # Root CMakeLists.txt: NOW triggers flight (build system can
    # change the ELF — link order, compile flags, source list).
    # Empirical motivator: Cubesat lived case where a
    # target_link_libraries reorder changed init order.
    ft_c, st_c = match(['CMakeLists.txt'])
    check('root cmake triggers both roles', ft_c and st_c)

    # 8adab2d regression class — main.cpp + rc_log.h. Prior narrow
    # regex would have returned False here; new regex catches it.
    ft_8a, _ = match(['src/main.cpp', 'include/rocketchip/rc_log.h'])
    check('main_cpp_rc_log_h_triggers (LL 39 regression class)', ft_8a)

    # Gate-self-rot vector: scripts/hooks, scripts/ci, bench_sim.
    # If these break, the gate's own behavior changes silently.
    ft_h, _ = match(['scripts/hooks/pre-commit'])
    check('hook_self_rot', ft_h)
    ft_m, _ = match(['scripts/ci/pre_commit_matrix.py'])
    check('matrix_self_rot', ft_m)
    ft_b, _ = match(['scripts/bench_sim.py'])
    check('bench_sim_self_rot', ft_b)
    ft_rc, _ = match(['scripts/_rc_test_common.py'])
    check('rc_test_common_self_rot', ft_rc)
    ft_fl, _ = match(['scripts/flash_elf_halt_write.py'])
    check('flash_elf_self_rot', ft_fl)
    # A gate change benches BOTH roles (2026-10-08).
    for gp in ('scripts/hooks/pre-commit', 'scripts/station_bench_sim.py',
               'scripts/bench_sim.py'):
        check(f'gate {gp} -> both roles', match([gp]) == (True, True))

    # Tests are exempt (host-ctest gate covers them).
    ft_t, st_t = match(['test/test_command_handler.cpp'])
    check('test_dir_exempt', not ft_t and not st_t)

    # Pure-doc commit (typical R-5 doc-only ride-along).
    ft_pd, st_pd = match(['docs/PROJECT_STATUS.md', 'CHANGELOG.md',
                          'AGENT_WHITEBOARD.md'])
    check('pure_doc_exempt', not ft_pd and not st_pd)


def test_banner_refuses_leftover_git_hash() -> None:
    print('test_banner_refuses_leftover_git_hash')
    from pathlib import Path

    b = classify_banner(VEHICLE_FLIGHT_BANNER)
    expected = ExpectedImage(
        role='vehicle',
        elf=Path('rocketchip.elf'),
        version_header=Path('version.h'),
        git_hash='c93019d',
        build_identity='x',
        sha256='ab',
    )
    err = banner_matches_elf_error(b, expected)
    check('leftover hash is an error', err is not None and 'leftover' in err,
          repr(err))
    match = ExpectedImage(
        role='vehicle',
        elf=Path('rocketchip.elf'),
        version_header=Path('version.h'),
        git_hash='deadbe',
        build_identity='x',
        sha256='ab',
    )
    check('matching hash is ok', banner_matches_elf_error(b, match) is None)
    dash = classify_banner(STATION_DASHBOARD_BANNER)
    check('dashboard has no git_hash', dash.git_hash is None)


def test_flash_stamp_refuses_rebuilt_elf() -> None:
    print('test_flash_stamp_refuses_rebuilt_elf')
    import tempfile
    from pathlib import Path

    with tempfile.TemporaryDirectory() as td:
        elf = Path(td) / 'rocketchip.elf'
        elf.write_bytes(b'image-v1')
        record_flashed_elf(elf)
        v1 = ExpectedImage(
            role='vehicle',
            elf=elf,
            version_header=Path(td) / 'version.h',
            git_hash='deadbe',
            build_identity='x',
            sha256=elf_sha256(elf),
        )
        check('stamp matches flashed bytes', flash_stamp_error(v1) is None)
        elf.write_bytes(b'image-v2-rebuilt')
        v2 = ExpectedImage(
            role='vehicle',
            elf=elf,
            version_header=Path(td) / 'version.h',
            git_hash='deadbe',
            build_identity='x',
            sha256=elf_sha256(elf),
        )
        err = flash_stamp_error(v2)
        check('rebuilt ELF without flash is an error',
              err is not None and 'not flashed' in err, repr(err))


def test_firmware_tree_identity_gate() -> None:
    """Build ID = firmware-tree hash; docs commits keep it (2026-10-08)."""
    print('test_firmware_tree_identity_gate')
    import tempfile
    from pathlib import Path
    from _rc_test_common import tree_matches_elf_error
    import firmware_tree

    repo = Path(os.path.dirname(os.path.abspath(__file__))).parent
    try:
        want = firmware_tree.elf_tree_id(repo, 'HEAD')
    except firmware_tree.FirmwareTreeError as exc:
        check('firmware tree hash computable', False, repr(exc))
        return
    check('hash is a 40-hex blob id', len(want) == 40, want)
    with tempfile.TemporaryDirectory() as td:
        elf = Path(td) / 'rocketchip.elf'
        elf.write_bytes(b'image')

        def img(tree_id: str) -> ExpectedImage:
            return ExpectedImage(
                role='vehicle', elf=elf,
                version_header=Path(td) / 'version.h',
                git_hash='deadbe', build_identity='x',
                sha256=elf_sha256(elf), firmware_tree_id=tree_id)

        check('push mode: matching clean build passes',
              tree_matches_elf_error(repo, img(want), commit='HEAD') is None)
        err = tree_matches_elf_error(repo, img('0' * 40), commit='HEAD')
        check('push mode: other tree refused',
              err is not None and 'firmware tree' in err, repr(err))
        err = tree_matches_elf_error(repo, img(want + '-dirty'), commit='HEAD')
        check('push mode: dirty build refused',
              err is not None and 'dirty' in err, repr(err))
        err = tree_matches_elf_error(repo, img(''), commit='HEAD')
        check('missing kFirmwareTreeId refused',
              err is not None and 'kFirmwareTreeId' in err, repr(err))
    check('docs path is not firmware',
          not firmware_tree.in_section(
              'docs/x.md', firmware_tree.load(repo), 'elf'))


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main() -> int:
    tests = [
        test_classify_vehicle_flight,
        test_usb_fault_handling,
        test_classify_station_kmenu,
        test_classify_station_dashboard,
        test_classify_station_dashboard_waiting,
        test_classify_vehicle_modes,
        test_classify_vehicle_help_after_banner_drain,
        test_classify_unknown,
        test_health_ring_dump_needs_help,
        test_classify_empty,
        test_target_matches,
        test_banner_frozen,
        test_target_frozen,
        test_target_str,
        test_decorator_returns_int,
        test_decorator_returns_none,
        test_decorator_uncaught_exception,
        test_decorator_keyboard_interrupt,
        test_decorator_sys_exit_passthrough,
        test_decorator_target_introspection,
        test_watchdog_fires,
        test_find_vehicle_and_station_rejects_same_port,
        test_find_vehicle_and_station_discovers_pair,
        test_board_usb_serial_from_enumeration,
        test_pre_commit_matrix_triggers,
        test_banner_refuses_leftover_git_hash,
        test_firmware_tree_identity_gate,
        test_flash_stamp_refuses_rebuilt_elf,
    ]

    print(f'=== _rc_test_common.py host tests ===')
    print(f'  {len(tests)} test functions, no hardware required')
    print()

    for t in tests:
        t()
    print()

    if _failures:
        print(f'FAILED: {len(_failures)} check(s)')
        for f in _failures:
            print(f'  - {f}')
        return 1
    print('ALL CHECKS PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
