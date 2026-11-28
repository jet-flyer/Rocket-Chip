#!/usr/bin/env python3
"""
RocketChip Bench Flight Simulation (minimal replacement, 2026-04-11)

Replaces the retired bench_flight_sim.py (IVP-73) after it was found to
have silently bit-rotten for 5 days across 4 commits.

Scope: verify the CLI dispatch path + log format + real-time FD tick +
action callback invocation end-to-end. Anything SPIN proves (state machine
safety, liveness, pyro ordering) is NOT tested here. Anything host tests
prove (guard logic, command_handler reject strings) is NOT tested here.

Two tests:
  1. Happy path: ARM -> launch -> boost -> apogee -> drogue -> main -> land
  2. Abort-with-pyro: ARM -> launch -> BOOST -> ABORT -> drogue fires

With --expect (pre-push gate, board profile), the preflight is also judged
against the expected hardware: a NO-GO with the IMU or baro fitted fails,
and the T2 Radio HW line is judged against radio= (the VERDICT is Tier 1
only, so it never covers the radio). N/A checks print as "hw_na: ..." and
unexpected hardware as "hw_warn: hw-unexpected:vehicle:<part> ...".

Port selection uses ``find_target_port`` (RocketChip VID:PID USB CDC only ---
never picks a bare COM by string alone). Overrides with ``--port`` are still
banner-classified; station firmware is rejected. Connection uses
``open_classified_port`` for a post-open re-classify guard.

Refuses leftover firmware: banner ``flight-<sha>`` must match the ELF
``kGitHash``, the ELF's ``kFirmwareTreeId`` must match this firmware tree
(or ``--commit <sha>`` from the pre-push gate), and
``rocketchip.elf.flashed.json`` must record this ELF's sha256. A PASS
on last week's image is a failed gate (HW_GATE_DISCIPLINE Rule 5).

Exit codes:
    0 = both tests pass
    1 = one or more test failures (including leftover / unflashed image)
    2 = could not connect to target or tool self-check failed
"""

import functools
import argparse
import os
import re
import serial
import sys
import threading
import time
from pathlib import Path

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)
from _rc_test_common import (  # noqa: E402
    Banner,
    banner_matches_elf_error,
    EXIT_UPTIME_NOT_RISING,
    PORT_LOST_MSG,
    NO_BANNER_MSG,
    HOLD_EXPIRED_MSG,
    board_config,
    role_usb_serial,
    EXIT_UPTIME_RESET,
    vehicle_reopen_uptimes,
    uptime_gap_extra_s,
    VEHICLE_UPTIME_GAP_S,
    open_classified_port,
    print_board_usb_serial,
    rc_test,
    refuse_stale_tree_and_elf,
    report_usb_fault,
    TARGET_VEHICLE_ANY,
    wait_for_target_port,
)


# =============================================================================
# Wall-clock watchdog
#
# `--max-runtime` is checked between tests, but a hung serial.Serial() open
# (Windows USB CDC stuck-in-CONFIGURED) can stall in C-library code where
# the inter-test check never fires. A daemon watchdog thread is the only
# reliable hard kill: when it fires, it prints to stderr and calls
# os._exit(2) — bypassing finalizers (intentional — we are already wedged).
# Mirrors the same primitive in scripts/station_bench_sim.py.
# =============================================================================

def _start_watchdog(deadline_s, label='bench_sim'):
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
# Regex constants — TESTED AGAINST FIRMWARE LOG OUTPUT.
# If you change a log line in the firmware, UPDATE THE REGEX HERE.
# See LESSONS_LEARNED.md Entry 36 for the "soft gate" root-cause context.
# =============================================================================

RE_TRANSITION = re.compile(r'\[FD\] (\w+) -> (\w+)')
# Firmware: "[FD] %s -> %s\n" at src/flight_director/flight_director.cpp:66
# Verified 2026-04-11.

RE_PYRO = re.compile(r'\[FD\] PYRO FIRED: (DROGUE|MAIN) \(primary\)')
# Firmware: "[FD] PYRO FIRED: %s (primary)\n" at
# src/active_objects/ao_flight_director.cpp:178
# Renamed from "PYRO INTENT" in commit 2254b16 (2026-04-06).
# Verified 2026-04-11.

RE_PHASE = re.compile(r'Phase:\s+(\w+)')
# Firmware: "  Phase:       %s\n" at src/active_objects/ao_flight_director.cpp:303
# Verified 2026-04-11.

RE_PROMPT = re.compile(r'\[(main|flight)\]\s*')

# =============================================================================
# Test case definitions
# =============================================================================

class TestCase:
    def __init__(self, name, commands, expected_transitions, expected_pyro,
                 verify_final_phase):
        self.name = name
        self.commands = commands
        self.expected_transitions = expected_transitions
        self.expected_pyro = expected_pyro
        self.verify_final_phase = verify_final_phase

TESTS = [
    TestCase(
        name='Happy path (nominal flight)',
        commands=['a', 'l', 'b', 'p', 'm', 'n'],
        expected_transitions=[
            ('IDLE', 'ARMED'), ('ARMED', 'BOOST'), ('BOOST', 'COAST'),
            ('COAST', 'DROGUE_DESCENT'), ('DROGUE_DESCENT', 'MAIN_DESCENT'),
            ('MAIN_DESCENT', 'LANDED'),
        ],
        expected_pyro=['DROGUE', 'MAIN'],
        verify_final_phase='LANDED',
    ),
    TestCase(
        name='Abort from BOOST (no pyro under default profile)',
        commands=['a', 'l', 'x'],
        expected_transitions=[
            ('IDLE', 'ARMED'), ('ARMED', 'BOOST'), ('BOOST', 'ABORT'),
        ],
        expected_pyro=[],  # Default rocket profile: abort_fires_drogue_from_boost=false
        verify_final_phase='ABORT',
    ),
]

# =============================================================================
# Serial helpers (post-connect)
# =============================================================================

def send_key(port, key, timeout_s=2.0):
    """Send a key and collect output until prompt or timeout."""
    port.write(key.encode())
    buf = ''
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        chunk = port.read(4096)
        if chunk:
            buf += chunk.decode('utf-8', errors='replace')
            if RE_PROMPT.search(buf):
                break
        else:
            time.sleep(0.05)
    return buf


def enter_flight_menu(port, verbose=False):
    """Navigate to flight director submenu."""
    send_key(port, 'z', 2.0)  # back to main if in submenu
    time.sleep(0.3)
    port.read(4096)  # drain
    out = send_key(port, 'f', 2.0)
    if verbose:
        print(f'  flight menu: {out[:80]}...')


def reset_target(port, verbose=False):
    """Reset FD to IDLE. Ensures flight menu context before querying phase."""
    # Ensure we're in the flight menu (s/status only works there)
    send_key(port, 'z', 1.0)  # back to main if nested
    time.sleep(0.2)
    port.read(4096)  # drain
    send_key(port, 'f', 1.0)  # enter flight menu
    time.sleep(0.2)
    port.read(4096)  # drain

    for _ in range(3):
        out = send_key(port, 'r', 2.0)  # RESET
        time.sleep(0.3)
        out += send_key(port, 's', 2.0)  # status
        m = RE_PHASE.search(out)
        if m and m.group(1) == 'IDLE':
            return True
        # If in LANDED or ABORT, RESET should work. If in flight phase,
        # inject landing first.
        if m and m.group(1) in ('DROGUE_DESCENT', 'MAIN_DESCENT'):
            send_key(port, 'n', 1.0)  # SIG_LANDING
            time.sleep(0.3)
            send_key(port, 'r', 1.0)  # RESET
        elif m and m.group(1) == 'ARMED':
            send_key(port, 'd', 1.0)  # DISARM
        elif m and m.group(1) in ('BOOST', 'COAST'):
            send_key(port, 'x', 1.0)  # ABORT
            time.sleep(0.3)
            send_key(port, 'r', 1.0)  # RESET
        time.sleep(0.3)
    return False


# =============================================================================
# Test runner
# =============================================================================

def run_test(port, tc, verbose=False):
    """Run a single test case. Returns (pass, detail)."""
    if not reset_target(port, verbose):
        return False, 'Could not reset to IDLE'

    port.read(4096)  # drain
    time.sleep(0.2)

    # Send all commands and collect combined output.
    # After the last command, wait briefly for async FD transition logs —
    # the CLI returns [flight] prompt immediately but the [FD] X -> Y
    # transition logs arrive asynchronously from the QP scheduler's event
    # dispatch loop (typically within 100-200ms of the command).
    combined = ''
    for cmd in tc.commands:
        combined += send_key(port, cmd, 2.0)

    # Wait for async FD logs — two reads with a gap to catch split CDC packets
    time.sleep(0.5)
    trailing = port.read(4096)
    if trailing:
        combined += trailing.decode('utf-8', errors='replace')
    time.sleep(0.3)
    trailing2 = port.read(4096)
    if trailing2:
        combined += trailing2.decode('utf-8', errors='replace')

    if verbose:
        print(f'  output ({len(combined)} chars)')

    # Check transitions
    found_trans = RE_TRANSITION.findall(combined)
    for expected in tc.expected_transitions:
        if expected not in found_trans:
            return False, (f'Missing transition: {expected[0]} -> {expected[1]}\n'
                           f'  Got: {found_trans}')

    # Check pyro fires
    found_pyro = RE_PYRO.findall(combined)
    for expected in tc.expected_pyro:
        if expected not in found_pyro:
            return False, (f'Missing pyro fire: {expected}\n'
                           f'  Got: {found_pyro}')

    # Verify final phase
    if tc.verify_final_phase:
        out = send_key(port, 's', 2.0)
        m = RE_PHASE.search(out)
        actual = m.group(1) if m else None
        if actual != tc.verify_final_phase:
            return False, (f'Phase mismatch: expected {tc.verify_final_phase}, '
                           f'got {actual}')

    return True, 'OK'


@rc_test(target=TARGET_VEHICLE_ANY)
def main():
    parser = argparse.ArgumentParser(
        description='RocketChip Bench Flight Simulation (minimal, 2 tests)')
    parser.add_argument('--port', default=os.environ.get("USBDEV"),
                        help='Serial port (auto-detected if not specified; honors $USBDEV for stable /dev/serial/by-id/ paths in WSL)')
    parser.add_argument('--verbose', action='store_true',
                        help='Print all serial traffic')
    parser.add_argument('--max-runtime', type=float, default=120.0,
                        help='Wall-clock deadline (default 120s, hard-killed by watchdog)')
    parser.add_argument('--commit', default=None,
                        help='Sha being pushed (pre-push gate). The ELF must be a '
                             'clean build of that firmware tree. Default: working tree.')
    parser.add_argument('--expect', default=None,
                        help='Expected hardware from the pre-push gate, e.g. '
                             'imu=fitted,baro=fitted,radio=optional,gps=optional. '
                             'With IMU or baro fitted, a preflight NO-GO fails '
                             'the bench; the T2 Radio HW line is judged against '
                             'radio=. Default: NO-GO only warns and the radio '
                             'is not judged (legacy).')
    args = parser.parse_args()
    try:
        args._expect = board_config.parse_expect(args.expect)
    except board_config.BoardConfigError as e:
        print(f'ERROR: {e}')
        sys.exit(2)

    # Wall-clock watchdog: hard kill after deadline, even if stuck inside
    # serial.Serial() C calls. See scripts/station_bench_sim.py for context.
    # RC_UPTIME_GAP_S (test-only) widens the uptime check; the watchdog gets
    # exactly that extra time (2 gaps + margin). Unset: --max-runtime as is.
    _start_watchdog(args.max_runtime + uptime_gap_extra_s(VEHICLE_UPTIME_GAP_S))

    repo = Path(_SCRIPTS_DIR).resolve().parent
    expected, ident_err = refuse_stale_tree_and_elf(repo, 'vehicle', commit=args.commit)
    if ident_err or expected is None:
        print('ERROR: leftover / unflashed image would make this gate a lie.')
        print(f'  {ident_err}')
        sys.exit(1)

    # --- Port detection + connect ---
    try:
        role_ser, strict = role_usb_serial('vehicle')
    except board_config.BoardConfigError as e:
        print(f'ERROR: {e}')
        sys.exit(2)
    pw = wait_for_target_port(
        TARGET_VEHICLE_ANY, override=args.port, verbose=args.verbose,
        prefer_serial=role_ser, strict=strict)
    port_name, meta = pw.port, pw.meta
    if port_name is None:
        print('ERROR: No vehicle RocketChip USB CDC port found.')
        print(f'  {meta}')
        report_usb_fault('port never appeared' if not pw.appeared else 'not this role',
                         'vehicle', appeared=pw.appeared, seconds=None,
                         last_uptime=None)
        print('  Plug the vehicle Feather, unplug station-only rigs, or set '
              '--port.')
        sys.exit(2)

    if not isinstance(meta, Banner):
        print('ERROR: internal: expected Banner from find_target_port')
        sys.exit(2)

    banner_err = banner_matches_elf_error(meta, expected)
    if banner_err:
        print('ERROR: leftover / unflashed image would make this gate a lie.')
        print(f'  {banner_err}')
        sys.exit(1)

    print(f'Using {port_name} ({meta.short_summary()})')
    print_board_usb_serial(port_name)
    print(f'  image: flight-{expected.git_hash} sha256={expected.sha256[:12]}…')

    # Uptime check: the connect banner prints 'Uptime: Ns' on every fresh open.
    # Close/reopen 3 times; uptime must rise strictly. Nothing is sent.
    # A vanished port is waited for (RC_PORT_WAIT_S, USB serial first):
    # back lower = reset (4); never back = port lost (3). USB only, no probe.
    verdict, ups, hb_port = vehicle_reopen_uptimes(
        TARGET_VEHICLE_ANY, override=args.port,
        prefer_serial=role_ser,
        wait=functools.partial(wait_for_target_port, strict=strict))
    print(f'  uptime check (banner Uptime s over 3 reopens): {ups}')
    if verdict != 'ok':
        last = next((u for u in reversed(ups) if u is not None), None)
        if verdict == 'reset':
            report_usb_fault('uptime dropped between reopens (board reset)',
                             'vehicle', appeared=True, seconds=pw.seconds,
                             port=hb_port, last_uptime=last)
            sys.exit(EXIT_UPTIME_RESET)
        report_usb_fault(verdict if verdict in (PORT_LOST_MSG, NO_BANNER_MSG, HOLD_EXPIRED_MSG)
                         else 'uptime not rising', 'vehicle', appeared=True,
                         seconds=pw.seconds, port=hb_port, last_uptime=last)
        sys.exit(EXIT_UPTIME_NOT_RISING)
    port_name = hb_port or port_name

    try:
        # find_target_port already classified. A second CDC open with the
        # default DTR pulse USB-resets into bootrom (FLASHING rule 5) so
        # post-open re-classify reads unknown. Desk 2026-09-03.
        with open_classified_port(port_name, target=TARGET_VEHICLE_ANY,
                                  post_open_re_classify=False) as ser:
            return _bench_sim_run_inner(ser, port_name, meta, args)
    except RuntimeError as e:
        print(f'ERROR: Cannot open {port_name}: {e}')
        sys.exit(2)


# Sensor parts whose health the preflight GO verdict covers.
PREFLIGHT_PARTS = ('imu', 'baro')
NOGO_FAIL_HINT = 'power-cycle the vehicle, then rerun'


def preflight_nogo_message(expect):
    """Preflight never reached GO. With IMU or baro expected fitted that is a
    test fail (Nathan 2026-11-23); otherwise (manual run, or no sensors
    expected) it stays the legacy warning. Pure, host-tested."""
    fitted = [p for p in PREFLIGHT_PARTS
              if expect and expect.get(p) == board_config.FITTED]
    if fitted:
        return (f'FAIL: preflight NO-GO after 15 s with {" and ".join(fitted)} '
                f'expected fitted. A warm reboot can leave the sensors NO-GO: '
                f'{NOGO_FAIL_HINT}.')
    return 'WARNING: sensor health timeout — tests may fail due to NO-GO'



# Preflight station lines: cli_print_preflight() prints every station as
# "T%u %-10s %s" (tier, name, reason), e.g. "T2 Radio HW   GO" or
# "T2 Radio HW   NO-GO NOT INIT" (src/cli/rc_os_commands.cpp, reasons in
# src/flight_director/go_nogo_checks.cpp). Verified 2026-11-23.
RE_PREFLIGHT_RADIO = re.compile(r'^T2 Radio HW\s+(GO|NO-GO)\b(.*?)\s*$', re.MULTILINE)
PREFLIGHT_RADIO_CHECK = 'Radio HW (preflight)'
PREFLIGHT_GPS_CHECK = 'GPS (preflight)'


def judge_preflight_radio(out, radio_state):
    """Preflight text -> (ok or None for N/A, detail, warns). Pure, host-tested.

    The VERDICT is Tier 1 only, so a missing radio still reads GO; the bench
    judges the T2 Radio HW line itself (Nathan 2026-11-23: radio code needs
    the radio). Radio HW is SX1276 init / SPI reachable: GO means a chip
    that passed its RegVersion check, NO-GO NOT INIT means no chip came up
    (an unstacked FeatherWing reads 0x00/0xFF and never inits). The line
    cannot tell "not there" from "there but init failed", so NO-GO is the
    absent reading; a missing line is no reading at all.
      fitted:      GO passes; NO-GO or no line fails.
      optional:    GO passes; NO-GO or no line is N/A by name.
      not-fitted:  NO-GO or no line is N/A; GO warns (unexpected) and passes.
      absent:      NO-GO or no line passes; GO warns and passes.
    None (no --expect): not judged, the legacy vehicle bench."""
    if radio_state is None:
        return True, 'not judged (no --expect)', []
    m = RE_PREFLIGHT_RADIO.search(out or '')
    go = m is not None and m.group(1) == 'GO'
    seen = f'T2 Radio HW {m.group(1)}{m.group(2)}' if m else 'no T2 Radio HW line'
    if radio_state == board_config.FITTED:
        if go:
            return True, 'OK T2 Radio HW GO', []
        why = (f'{seen} (radio expected fitted; preflight VERDICT does not '
               f'cover the radio)' if m else
               f'{seen} in the preflight output (radio expected fitted)')
        return False, why, []
    if go:
        if radio_state == board_config.OPTIONAL:
            return True, 'OK T2 Radio HW GO', []
        return True, 'radio not expected', [('radio', 'Radio HW GO (SX1276 came up) '
                                             'but no radio expected')]
    if radio_state == board_config.ABSENT:
        return True, 'no radio expected', []
    if radio_state == board_config.NOT_FITTED:
        return None, f'radio not fitted ({seen})', []
    return None, f'optional radio not detected ({seen})', []


def judge_preflight_gps(gps_state):
    """(ok or None, detail). The preflight GPS line is lock (fix >= 2D and
    >= 4 sats, go_nogo_checks.h), not presence: a fitted GPS reads NO-GO NO
    LOCK indoors. So the vehicle bench cannot confirm a GPS is fitted, and
    an expected-fitted GPS fails rather than passing unchecked. Today both
    profiles mark GPS optional, which the vehicle bench does not judge."""
    if gps_state == board_config.FITTED:
        return False, ('GPS expected fitted, but the vehicle bench cannot check '
                       'GPS presence (the preflight GPS line is lock, not '
                       'presence). Mark GPS optional in the profile or add a '
                       'vehicle Hardware Status check.')
    return None, 'not judged on the vehicle bench'


def judge_preflight_hw(out, expect):
    """(results, na, warns) for the Tier 2 parts in --expect. results:
    [(check name, ok or None, detail)]; na: [(check, part)] for hw_na:;
    warns: [(part, message)] for hw_warn:. No --expect: nothing judged."""
    if not expect:
        return [], [], []
    results, na = [], []
    ok, detail, warns = judge_preflight_radio(out, expect['radio'])
    results.append((PREFLIGHT_RADIO_CHECK, ok, detail))
    if ok is None:
        na.append((PREFLIGHT_RADIO_CHECK, 'radio'))
    ok, detail = judge_preflight_gps(expect['gps'])
    if ok is not None:
        results.append((PREFLIGHT_GPS_CHECK, ok, detail))
    return results, na, warns

def _bench_sim_run_inner(ser, port_name: str, meta: Banner, args):
    """Test body inside open_classified_port context."""

    # --- Wait for sensor health before testing ---
    # After boot, sensors need a few seconds to report healthy. The Go/No-Go
    # check blocks ARM if Tier 1 sensors aren't ready. Poll preflight status
    # until VERDICT: GO or timeout.
    print('waiting for sensor health...')
    health_deadline = time.time() + 15.0
    preflight_out = ''  # last full preflight print, for the Tier 2 judges
    while time.time() < health_deadline:
        send_key(ser, 'z', 1.0)  # ensure main menu
        time.sleep(0.3)
        ser.read(4096)  # drain
        ser.write(b'p')
        time.sleep(1.0)
        health_out = ser.read(4096).decode('utf-8', errors='replace')
        if 'VERDICT' in health_out:
            preflight_out = health_out
        if 'VERDICT:  GO' in health_out:
            print('  sensors healthy — GO')
            break
        if args.verbose:
            verdict_line = [l for l in health_out.split('\n') if 'VERDICT' in l]
            if verdict_line:
                print(f'  {verdict_line[0].strip()}')
        time.sleep(1.0)
    else:
        msg = preflight_nogo_message(getattr(args, '_expect', None))
        if msg.startswith('FAIL'):
            print(msg)
            return 1
        print(msg)

    # The VERDICT covers Tier 1 only; the radio and GPS are Tier 2 lines that
    # never change it (go_nogo_checks.cpp). Judge them here against --expect.
    hw_results, hw_na, hw_warn = judge_preflight_hw(
        preflight_out, getattr(args, '_expect', None))

    # Inject keys are compile-time ROCKETCHIP_DEV_MODE + runtime toggle.
    # Toggle needs USB + IDLE (ABORT leftover from a prior run refuses).
    # DTR on open clears it. v is a toggle: send until ON (at most twice).
    if not reset_target(ser, args.verbose):
        print('WARNING: could not reset FD to IDLE before DEV_MODE')
    send_key(ser, 'z', 1.0)
    time.sleep(0.2)
    ser.read(4096)
    out = ''
    for _ in range(2):
        out = send_key(ser, 'v', 2.0)
        if 'DEV_MODE on' in out:
            break
    if 'DEV_MODE on' not in out:
        print('WARNING: DEV_MODE toggle did not print ON - inject tests may fail')
        print(f'  toggle: {out[:160]!r}')

    # --- Enter flight menu ---
    enter_flight_menu(ser, args.verbose)

    # --- Run tests ---
    print(f'\n=== RocketChip Bench Sim ===')
    print(f'  Port: {port_name}')
    print(f'  Classified: {meta.short_summary()}')
    print(f'  Max runtime: {args.max_runtime:.0f}s')
    print()

    passed = 0
    failed = 0
    skipped = 0
    for name, ok, detail in hw_results:
        status = 'N/A ' if ok is None else ('PASS' if ok else 'FAIL')
        if ok is None:
            skipped += 1
        elif ok:
            passed += 1
        else:
            failed += 1
        print(f'  [{status}]  {name} -- {detail}')
    start_time = time.time()
    deadline = start_time + args.max_runtime

    for i, tc in enumerate(TESTS):
        if time.time() >= deadline:
            elapsed = time.time() - start_time
            print(f'  [ABORT] Runtime exceeded {args.max_runtime:.0f}s '
                  f'after {elapsed:.1f}s')
            failed += len(TESTS) - i
            break

        try:
            ok, detail = run_test(ser, tc, args.verbose)
        except serial.SerialException as e:
            ok, detail = False, f'Serial error: {e}'

        status = 'PASS' if ok else 'FAIL'
        if ok:
            passed += 1
        else:
            failed += 1

        suffix = '' if ok else f' -- {detail}'
        print(f'  [{status}]  {i + 1}. {tc.name}{suffix}')

    for check, part in hw_na:
        print(board_config.hw_na_line(check, part))
    for part, what in hw_warn:
        print(board_config.hw_warn_line('vehicle', part, what))
    elapsed = time.time() - start_time
    total = passed + failed
    na = f', {skipped} N/A' if skipped else ''
    print(f'\n  RESULT: {passed}/{total} PASS{na}  ({elapsed:.1f}s)')
    print(f'={"=" * 30}')

    return 0 if failed == 0 else 1


if __name__ == '__main__':
    main()
