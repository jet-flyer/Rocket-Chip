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

Serial port is auto-detected by default (probes for RocketChip boot banner).
Use --port to override for multi-device setups.

Exit codes:
    0 = both tests pass
    1 = one or more test failures
    2 = could not connect to target or tool self-check failed
"""

import argparse
import os
import re
import serial
import serial.tools.list_ports
import sys
import threading
import time

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)
from _rc_test_common import rc_test, TARGET_VEHICLE_ANY  # noqa: E402


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

RE_BOOT_BANNER = re.compile(r'RocketChip v\S+\s+Build:\s+(\S+)')
# Firmware: "RocketChip v%s  Build: %s\n" at src/cli/rc_os_commands.cpp:744
# Printed automatically at boot, before any CLI interaction.
# Verified 2026-04-11.

RE_PHASE = re.compile(r'Phase:\s+(\w+)')
# Firmware: "  Phase:       %s\n" at src/active_objects/ao_flight_director.cpp:303
# Verified 2026-04-11.

RE_PROMPT = re.compile(r'\[(main|flight)\]\s*')

LAST_PORT_FILE = os.path.join(os.path.dirname(__file__), '.bench_sim_last_port')

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
# Serial helpers
# =============================================================================

def auto_detect_port(verbose=False):
    """Auto-detect the RocketChip serial port by probing for known responses."""
    # Try cached port first
    if os.path.exists(LAST_PORT_FILE):
        try:
            cached = open(LAST_PORT_FILE).read().strip()
            if cached and _probe_port(cached, verbose):
                if verbose:
                    print(f'  auto-detect: cached port {cached} OK')
                return cached
        except (OSError, serial.SerialException):
            pass

    # Scan all ports
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if verbose:
        print(f'  auto-detect: scanning {len(ports)} ports: {ports}')

    for port in ports:
        if _probe_port(port, verbose):
            # Cache for next time
            try:
                with open(LAST_PORT_FILE, 'w') as f:
                    f.write(port)
            except OSError:
                pass
            return port

    return None


def _probe_port(port, verbose=False):
    """Try to open a port and check for RocketChip response.

    Uses a thread with a join timeout to prevent hanging on ports held by
    other processes (e.g., the debug probe's CDC interface held by OpenOCD).
    On Windows, serial.Serial() can block indefinitely if the port is held
    by another process; Python's timeout parameter only applies to reads,
    not to the open itself. The thread approach is the only reliable way to
    bound the open time.
    """
    import threading

    result = [False]

    def _try_open():
        try:
            p = serial.Serial(port, 115200, timeout=0.5, write_timeout=0.5)
            time.sleep(0.3)
            p.read(4096)  # drain
            p.write(b'h')  # help command
            time.sleep(0.5)
            data = p.read(4096).decode('utf-8', errors='replace')
            p.close()
            # Accept any RocketChip banner, but REJECT the station —
            # bench_sim targets the VEHICLE. Station has its own sim.
            # Fixes wrong-port-pick when both boards are plugged in.
            if 'Ground Station' in data or 'Fruit Jam' in data:
                pass  # station port — skip
            elif 'RocketChip' in data or '[main]' in data or '[flight]' in data:
                result[0] = True
        except (serial.SerialException, OSError, PermissionError):
            pass

    t = threading.Thread(target=_try_open, daemon=True)
    t.start()
    t.join(timeout=3.0)  # 3s max per port — generous but bounded

    if t.is_alive():
        if verbose:
            print(f'  auto-detect: {port} timed out (likely held by another process)')
        return False  # Thread still running = hung on open. Daemon thread dies with process.

    if result[0] and verbose:
        print(f'  auto-detect: {port} responded with RocketChip signature')
    elif verbose:
        print(f'  auto-detect: {port} no response or error')

    return result[0]


def connect(port, retries=5, retry_delay=1.5):
    """Open serial port with retry for transient USB CDC failures."""
    last_err = None
    for attempt in range(retries):
        try:
            p = serial.Serial(port, 115200, timeout=0.1)
            time.sleep(1.0)
            p.read(10000)  # drain
            time.sleep(0.5)
            p.read(10000)  # drain again
            if attempt > 0:
                print(f'  (connected on attempt {attempt + 1})')
            return p
        except (serial.SerialException, OSError) as e:
            last_err = e
            if attempt < retries - 1:
                print(f'  connect attempt {attempt + 1}/{retries}: {e}; '
                      f'retrying in {retry_delay}s')
                time.sleep(retry_delay)
    raise RuntimeError(f'Could not open {port} after {retries} attempts: {last_err}')


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
    parser.add_argument('--port', default=None,
                        help='Serial port (auto-detected if not specified)')
    parser.add_argument('--verbose', action='store_true',
                        help='Print all serial traffic')
    parser.add_argument('--max-runtime', type=float, default=120.0,
                        help='Wall-clock deadline (default 120s, hard-killed by watchdog)')
    args = parser.parse_args()

    # Wall-clock watchdog: hard kill after deadline, even if stuck inside
    # serial.Serial() C calls. See scripts/station_bench_sim.py for context.
    _start_watchdog(args.max_runtime)

    # --- Port detection ---
    port_name = args.port
    if port_name is None:
        print('auto-detecting serial port...')
        port_name = auto_detect_port(verbose=args.verbose)
        if port_name is None:
            print('ERROR: Could not auto-detect RocketChip serial port.')
            print('  No port responded with a RocketChip signature.')
            print('  Ensure the device is powered and connected.')
            print('  Or specify --port manually.')
            sys.exit(2)
    print(f'auto-detected serial port: {port_name}')

    # --- Connect ---
    try:
        ser = connect(port_name)
    except (RuntimeError, serial.SerialException, OSError) as e:
        print(f'ERROR: Cannot open {port_name}: {e}')
        sys.exit(2)

    # --- Capture build tag from boot banner (if visible) ---
    time.sleep(0.5)
    banner_data = ser.read(4096).decode('utf-8', errors='replace')
    build_tag = '?'
    m = RE_BOOT_BANNER.search(banner_data)
    if m:
        build_tag = m.group(1)

    # --- Wait for sensor health before testing ---
    # After boot, sensors need a few seconds to report healthy. The Go/No-Go
    # check blocks ARM if Tier 1 sensors aren't ready. Poll preflight status
    # until VERDICT: GO or timeout.
    print('waiting for sensor health...')
    health_deadline = time.time() + 15.0
    while time.time() < health_deadline:
        send_key(ser, 'z', 1.0)  # ensure main menu
        time.sleep(0.3)
        ser.read(4096)  # drain
        ser.write(b'p')
        time.sleep(1.0)
        health_out = ser.read(4096).decode('utf-8', errors='replace')
        if 'VERDICT:  GO' in health_out:
            print('  sensors healthy — GO')
            break
        if args.verbose:
            verdict_line = [l for l in health_out.split('\n') if 'VERDICT' in l]
            if verdict_line:
                print(f'  {verdict_line[0].strip()}')
        time.sleep(1.0)
    else:
        print('WARNING: sensor health timeout — tests may fail due to NO-GO')

    # --- Enter flight menu ---
    enter_flight_menu(ser, args.verbose)

    # --- Run tests ---
    print(f'\n=== RocketChip Bench Sim ===')
    print(f'  Port: {port_name}')
    print(f'  Build: {build_tag}')
    print(f'  Max runtime: {args.max_runtime:.0f}s')
    print()

    passed = 0
    failed = 0
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

    elapsed = time.time() - start_time
    total = passed + failed
    print(f'\n  RESULT: {passed}/{total} PASS  ({elapsed:.1f}s)')
    print(f'={"=" * 30}')

    ser.close()
    sys.exit(0 if failed == 0 else 1)


if __name__ == '__main__':
    main()
