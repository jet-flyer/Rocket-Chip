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
    TARGET_VEHICLE_ANY, TARGET_VEHICLE_BENCH, TARGET_VEHICLE_FLIGHT,
    TARGET_STATION_ANY, TARGET_STATION_BENCH, TARGET_STATION_FLIGHT,
    TARGET_EITHER_ANY,
    classify_banner, rc_test,
)


# Canned banner samples captured 2026-04-27 from real firmware on the
# bench. These are the source of truth for the regex / token tests.

VEHICLE_BENCH_BANNER = """\
==============================================
  RocketChip v0.16.0  RCOS v0.5.0  dev-b411e3a
  Board: Adafruit Feather RP2350 HSTX
  Profile: Rocket  Uptime: 33s
==============================================
Hardware: 14/14 OK
Flash: 4 flights, 27% used

h-Help  p-Preflight  c-Calibration  f-Flight
g-Flights  d-Download  l-Flush  x-Erase
t-Radio  r-Rate  m-MAVLink  b-Beacon  q-Debug
[main]
"""

VEHICLE_FLIGHT_BANNER = """\
==============================================
  RocketChip v0.16.0  RCOS v0.5.0  flight-deadbe
  Board: Adafruit Feather RP2350 HSTX
  Profile: Rocket  Uptime: 5s
==============================================
Hardware: 14/14 OK

h-Help  p-Preflight  c-Calibration  f-Flight
[main]
"""

STATION_KMENU_BANNER = """\
========================================
  RocketChip v0.16.0  RCOS v0.5.0 - Station RX
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

def test_classify_vehicle_bench() -> None:
    print('test_classify_vehicle_bench')
    b = classify_banner(VEHICLE_BENCH_BANNER)
    check('role is VEHICLE',  b.role is Role.VEHICLE,      f'got {b.role}')
    check('build is DEV',     b.build is Build.DEV,        f'got {b.build}')
    check('version captured', b.version == '0.16.0',       f'got {b.version!r}')
    check('board captured',   b.board is not None
          and 'feather rp2350 hstx' in (b.board or ''),    f'got {b.board!r}')
    check('is_vehicle()',     b.is_vehicle())
    check('is_station() False', not b.is_station())
    check('is_known()',       b.is_known())


def test_classify_vehicle_flight() -> None:
    print('test_classify_vehicle_flight')
    b = classify_banner(VEHICLE_FLIGHT_BANNER)
    check('role is VEHICLE',  b.role is Role.VEHICLE,      f'got {b.role}')
    check('build is FLIGHT',  b.build is Build.FLIGHT,     f'got {b.build}')


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


def test_classify_vehicle_modes() -> None:
    print('test_classify_vehicle_modes')
    b = classify_banner(VEHICLE_BENCH_BANNER)
    check('vehicle bench is KMENU', b.mode is Mode.KMENU,  f'got {b.mode}')
    b = classify_banner(VEHICLE_FLIGHT_BANNER)
    check('vehicle flight is KMENU', b.mode is Mode.KMENU, f'got {b.mode}')


def test_classify_unknown() -> None:
    print('test_classify_unknown')
    b = classify_banner(UNKNOWN_BANNER)
    check('role UNKNOWN',     b.role is Role.UNKNOWN,      f'got {b.role}')
    check('build UNKNOWN',    b.build is Build.UNKNOWN,    f'got {b.build}')
    check('not is_known()',   not b.is_known())


def test_classify_empty() -> None:
    print('test_classify_empty')
    for empty in ('', '   ', None):  # type: ignore[arg-type]
        b = classify_banner(empty)  # type: ignore[arg-type]
        check(f'empty {empty!r} -> UNKNOWN',
              b.role is Role.UNKNOWN and b.build is Build.UNKNOWN)


def test_target_matches() -> None:
    print('test_target_matches')
    vb = classify_banner(VEHICLE_BENCH_BANNER)
    vf = classify_banner(VEHICLE_FLIGHT_BANNER)
    sk = classify_banner(STATION_KMENU_BANNER)
    sd = classify_banner(STATION_DASHBOARD_BANNER)  # build=UNKNOWN
    uk = classify_banner(UNKNOWN_BANNER)

    # vehicle-any should match both vehicle banners.
    check('VEHICLE_ANY matches vehicle-bench',  TARGET_VEHICLE_ANY.matches(vb))
    check('VEHICLE_ANY matches vehicle-flight', TARGET_VEHICLE_ANY.matches(vf))
    check('VEHICLE_ANY rejects station',        not TARGET_VEHICLE_ANY.matches(sk))
    check('VEHICLE_ANY rejects unknown',        not TARGET_VEHICLE_ANY.matches(uk))

    # vehicle-bench should match only vehicle-bench.
    check('VEHICLE_BENCH matches vehicle-bench',     TARGET_VEHICLE_BENCH.matches(vb))
    check('VEHICLE_BENCH rejects vehicle-flight',
          not TARGET_VEHICLE_BENCH.matches(vf))
    check('VEHICLE_BENCH rejects station-kmenu',
          not TARGET_VEHICLE_BENCH.matches(sk))

    # vehicle-flight should reject vehicle-bench.
    check('VEHICLE_FLIGHT rejects vehicle-bench',
          not TARGET_VEHICLE_FLIGHT.matches(vb))
    check('VEHICLE_FLIGHT matches vehicle-flight',
          TARGET_VEHICLE_FLIGHT.matches(vf))

    # station-any matches station banners.
    check('STATION_ANY matches station-kmenu',  TARGET_STATION_ANY.matches(sk))
    check('STATION_ANY matches station-dashboard',
          TARGET_STATION_ANY.matches(sd))  # build UNKNOWN -> permissive
    check('STATION_ANY rejects vehicle',        not TARGET_STATION_ANY.matches(vb))

    # station-bench: dashboard has UNKNOWN build, should still match
    # (permissive on UNKNOWN per design comment in matches()).
    check('STATION_BENCH permissive on UNKNOWN build',
          TARGET_STATION_BENCH.matches(sd))

    # EITHER matches both vehicle and station.
    check('EITHER matches vehicle',  TARGET_EITHER_ANY.matches(vb))
    check('EITHER matches station',  TARGET_EITHER_ANY.matches(sk))
    check('EITHER rejects unknown',  not TARGET_EITHER_ANY.matches(uk))


def test_banner_frozen() -> None:
    print('test_banner_frozen')
    b = classify_banner(VEHICLE_BENCH_BANNER)
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
    t = TARGET_VEHICLE_BENCH
    try:
        t.role = Role.STATION  # type: ignore[misc]
    except dataclasses.FrozenInstanceError:
        check('Target frozen', True)
        return
    check('Target frozen', False, 'mutation succeeded')


def test_target_str() -> None:
    print('test_target_str')
    check('VEHICLE_ANY str',     str(TARGET_VEHICLE_ANY)     == 'vehicle-any')
    check('VEHICLE_DEV str',     str(TARGET_VEHICLE_BENCH)   == 'vehicle-dev')
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

    @rc_test(target=TARGET_STATION_BENCH, watchdog_s=42.0)
    def fake_main() -> int:
        return 0

    check('wrapper.__rc_target__ is set',
          getattr(fake_main, '__rc_target__', None) == TARGET_STATION_BENCH)
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
    b_v = classify_banner(VEHICLE_BENCH_BANNER)

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
    b_v = classify_banner(VEHICLE_BENCH_BANNER)

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

def test_pre_commit_matrix_triggers() -> None:
    print('test_pre_commit_matrix_triggers')
    ci_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ci')
    if ci_dir not in sys.path:
        sys.path.insert(0, ci_dir)
    from pre_commit_matrix import match  # noqa: E402

    # Policy 2026-05-16 (commit a40e7f5, council unanimous, LL Entry 40):
    # ANY firmware-affecting path triggers FLIGHT_BENCH. STATION_SCOPE
    # stays narrow (no SWD on Fruit Jam per DEBUG_PROBE_NOTES.md).

    # Pure-doc / non-firmware: neither gate.
    ft, st = match(['README.md'])
    check('no trigger on unrelated', not ft and not st)

    # Firmware path → flight bench (regardless of label).
    ft_f, st_f = match(['src/flight_director/flight_director.cpp'])
    check('flight_critical', ft_f and not st_f)

    # Station-only path: NOW triggers flight (it's still firmware,
    # all firmware is flight code per the policy) AND triggers station.
    ft_s, st_s = match(['src/station/main.cpp'])
    check('station_only_triggers_both', ft_s and st_s)

    # Dashboard: both scopes.
    ft_d, st_d = match(['src/cli/rc_os_dashboard.cpp'])
    check('dashboard both scopes', ft_d and st_d)

    # rc_os_commands.cpp: flight (anything in src/ is firmware).
    ft_os, _ = match(['src/cli/rc_os_commands.cpp'])
    check('rc_os_cpp flight_critical', ft_os)

    # Root CMakeLists.txt: NOW triggers flight (build system can
    # change the ELF — link order, compile flags, source list).
    # Empirical motivator: Cubesat lived case where a
    # target_link_libraries reorder changed init order.
    ft_c, st_c = match(['CMakeLists.txt'])
    check('root cmake triggers flight', ft_c and not st_c)

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

    # Tests are exempt (host-ctest gate covers them).
    ft_t, st_t = match(['test/test_command_handler.cpp'])
    check('test_dir_exempt', not ft_t and not st_t)

    # Pure-doc commit (typical R-5 doc-only ride-along).
    ft_pd, st_pd = match(['docs/PROJECT_STATUS.md', 'CHANGELOG.md',
                          'AGENT_WHITEBOARD.md'])
    check('pure_doc_exempt', not ft_pd and not st_pd)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main() -> int:
    tests = [
        test_classify_vehicle_bench,
        test_classify_vehicle_flight,
        test_classify_station_kmenu,
        test_classify_station_dashboard,
        test_classify_station_dashboard_waiting,
        test_classify_vehicle_modes,
        test_classify_unknown,
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
        test_pre_commit_matrix_triggers,
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
