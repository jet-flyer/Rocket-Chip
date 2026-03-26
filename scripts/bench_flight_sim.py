#!/usr/bin/env python3
"""
RocketChip Bench Flight Simulation (IVP-73)

Automated bench test for the Flight Director state machine. Sends CLI key
commands over serial, parses [FD] transition logs, validates sequence and
actions, produces pass/fail summary.

Tests the command -> validation -> HSM dispatch -> action executor pipeline.
Guards were validated in IVP-70/71 host tests; this tests the plumbing.

Council review: NASA/JPL, ArduPilot, Professor, Rocketeer (unanimous APPROVE).
Amendments A1-A5 incorporated.

Usage:
    python scripts/bench_flight_sim.py [--port COM7] [--verbose] [--test N]
"""

import argparse
import re
import serial
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime


# =============================================================================
# Regex patterns (matched against firmware output)
# =============================================================================
RE_TRANSITION = re.compile(r'\[FD\] (\w+) -> (\w+)')
RE_PYRO = re.compile(r'\[FD\] PYRO INTENT: (DROGUE|MAIN)')
RE_REJECTED = re.compile(r'\[FD\] Command rejected: (.+)')
RE_ABORT_IGN = re.compile(r'\[FD\] ABORT in DESCENT')
RE_GO_NOGO = re.compile(r'\[GO/NO-GO\] Platform: (\d+)/(\d+) (GO|NO-GO)')
RE_PHASE = re.compile(r'Phase:\s+(\w+)')
RE_BUILD_TAG = re.compile(r'Build:\s+(\S+)')
RE_PROMPT = re.compile(r'\[flight\]\s*')

# NeoPixel colors per phase (A5: visual aide, not verified programmatically)
PHASE_LED = {
    'IDLE': 'normal status',
    'ARMED': 'amber solid',
    'BOOST': 'red solid',
    'COAST': 'yellow solid',
    'DROGUE_DESCENT': 'red blink',
    'MAIN_DESCENT': 'red blink',
    'LANDED': 'green blink / white beacon',
    'ABORT': 'red fast blink',
}


# =============================================================================
# Test case data structure (A3: data-driven)
# =============================================================================
@dataclass
class TestCase:
    name: str
    # Commands to send (single characters). Use tuples for (key, description).
    commands: list
    # Expected [FD] X -> Y transitions in order
    expected_transitions: list = field(default_factory=list)
    # Expected [FD] PYRO INTENT: X lines
    expected_pyro: list = field(default_factory=list)
    # Expected [FD] Command rejected: X substring (None = no rejection expected)
    expected_rejection: str = None
    # Expected "ABORT in DESCENT" message
    expect_abort_ignored: bool = False
    # Phase to verify via status query after all commands (None = skip)
    verify_phase: str = None
    # Expect NO pyro intent (assert none found)
    expect_no_pyro: bool = False


# =============================================================================
# Test definitions (A2: rejection tests first)
# =============================================================================
TESTS = [
    # --- Rejection tests (no ARM needed, run first) ---
    TestCase(
        name='ABORT from IDLE (rejected)',
        commands=['x'],
        expected_rejection='Cannot abort from IDLE/LANDED',
    ),
    TestCase(
        name='RESET from IDLE (rejected)',
        commands=['r'],
        expected_rejection='Only from LANDED or ABORT',
    ),
    TestCase(
        name='DISARM from IDLE (rejected)',
        commands=['d'],
        expected_rejection='Not ARMED',
    ),

    # --- Abort path tests (need ARM) ---
    TestCase(
        name='ABORT from ARMED (no pyro)',
        commands=['a', 'x'],
        expected_transitions=[('IDLE', 'ARMED'), ('ARMED', 'ABORT')],
        expect_no_pyro=True,
        verify_phase='ABORT',
    ),
    TestCase(
        name='ABORT from BOOST (drogue pyro)',
        commands=['a', 'l', 'x'],
        expected_transitions=[
            ('IDLE', 'ARMED'), ('ARMED', 'BOOST'), ('BOOST', 'ABORT')],
        expected_pyro=['DROGUE'],
        verify_phase='ABORT',
    ),
    TestCase(
        name='ABORT from COAST (drogue pyro)',
        commands=['a', 'l', 'b', 'x'],
        expected_transitions=[
            ('IDLE', 'ARMED'), ('ARMED', 'BOOST'),
            ('BOOST', 'COAST'), ('COAST', 'ABORT')],
        expected_pyro=['DROGUE'],
        verify_phase='ABORT',
    ),
    TestCase(
        name='ABORT from DESCENT (ignored)',
        commands=['a', 'l', 'b', 'p', 'x'],
        expected_transitions=[
            ('IDLE', 'ARMED'), ('ARMED', 'BOOST'),
            ('BOOST', 'COAST'), ('COAST', 'DROGUE_DESCENT')],
        expected_pyro=['DROGUE'],  # From apogee transition, not from abort
        expect_abort_ignored=True,
        verify_phase='DROGUE_DESCENT',
    ),

    # --- Happy path ---
    TestCase(
        name='Happy path (full flight)',
        commands=['a', 'l', 'b', 'p', 'm', 'n'],
        expected_transitions=[
            ('IDLE', 'ARMED'), ('ARMED', 'BOOST'),
            ('BOOST', 'COAST'), ('COAST', 'DROGUE_DESCENT'),
            ('DROGUE_DESCENT', 'MAIN_DESCENT'), ('MAIN_DESCENT', 'LANDED')],
        expected_pyro=['DROGUE', 'MAIN'],
        verify_phase='LANDED',
    ),

    # --- Double ARM ---
    TestCase(
        name='Double ARM (rejected)',
        commands=['a', 'a'],
        expected_transitions=[('IDLE', 'ARMED')],
        expected_rejection='Not in IDLE',
    ),
]


# =============================================================================
# Serial helper
# =============================================================================
class SerialHelper:
    def __init__(self, port, baud, verbose=False):
        self.port_name = port
        self.baud = baud
        self.verbose = verbose
        self.port = None
        self.buffer = ''

    def connect(self):
        """Open serial port with drain + settle."""
        self.port = serial.Serial(self.port_name, self.baud, timeout=0.1)
        time.sleep(1.0)
        self.port.read(10000)  # drain
        time.sleep(0.5)
        self.port.read(10000)  # drain again
        self.buffer = ''

    def close(self):
        if self.port and self.port.is_open:
            self.port.close()

    def send(self, key, wait_prompt=True, timeout_s=2.0):
        """Send a key, collect output until [flight] prompt or timeout.

        A1: Sync on prompt instead of fixed sleep.
        Returns the new output added to the buffer.
        """
        start_pos = len(self.buffer)
        self.port.write(key.encode())

        deadline = time.time() + timeout_s
        while time.time() < deadline:
            chunk = self.port.read(4096)
            if chunk:
                text = chunk.decode('utf-8', errors='replace')
                self.buffer += text
                if self.verbose:
                    sys.stdout.write(text)
                    sys.stdout.flush()
                # Check for prompt sync marker
                if wait_prompt and RE_PROMPT.search(self.buffer[start_pos:]):
                    break
            else:
                time.sleep(0.05)

        return self.buffer[start_pos:]

    def send_main(self, key, timeout_s=2.0):
        """Send a key from main menu (sync on [main] prompt)."""
        start_pos = len(self.buffer)
        self.port.write(key.encode())

        deadline = time.time() + timeout_s
        while time.time() < deadline:
            chunk = self.port.read(4096)
            if chunk:
                text = chunk.decode('utf-8', errors='replace')
                self.buffer += text
                if self.verbose:
                    sys.stdout.write(text)
                    sys.stdout.flush()
                if '[main]' in self.buffer[start_pos:]:
                    break
            else:
                time.sleep(0.05)

        return self.buffer[start_pos:]

    def drain(self):
        """Discard pending input and reset buffer."""
        time.sleep(0.2)
        self.port.read(10000)
        self.buffer = ''


# =============================================================================
# Flight sim runner
# =============================================================================
class FlightSimRunner:
    def __init__(self, ser):
        self.ser = ser
        self.results = []
        self.build_tag = '?'
        self.log_lines = []

    def log(self, msg):
        self.log_lines.append(f'[{time.strftime("%H:%M:%S")}] {msg}')
        if self.ser.verbose:
            print(f'  >> {msg}')

    def check_build_tag(self):
        """Read boot info to get build tag."""
        self.ser.drain()
        # Go to main menu first
        self.ser.send_main('z')
        self.ser.drain()
        out = self.ser.send_main('b', timeout_s=2.0)
        m = RE_BUILD_TAG.search(out)
        if m:
            self.build_tag = m.group(1)
        self.log(f'Build tag: {self.build_tag}')

    def enter_flight_menu(self):
        """Enter the flight director CLI sub-menu."""
        self.ser.drain()
        # Ensure we're in main menu
        self.ser.send_main('z')
        self.ser.drain()
        out = self.ser.send('f')
        if 'Flight Director Menu' not in out and 'flight' not in out:
            self.log('WARNING: Flight menu entry not confirmed')

    def get_phase(self):
        """Query current phase via status command."""
        out = self.ser.send('s')
        m = RE_PHASE.search(out)
        return m.group(1) if m else None

    def reset_to_idle(self):
        """Context-aware reset to IDLE. Returns True if successful."""
        # Try up to 3 times
        for attempt in range(3):
            phase = self.get_phase()
            if phase == 'IDLE':
                return True

            if phase in ('LANDED', 'ABORT'):
                self.ser.send('r')  # RESET → IDLE
            elif phase == 'ARMED':
                self.ser.send('d')  # DISARM → IDLE
            elif phase in ('BOOST', 'COAST'):
                self.ser.send('x')  # ABORT
                time.sleep(0.3)
                self.ser.send('r')  # RESET → IDLE
            elif phase in ('DROGUE_DESCENT', 'MAIN_DESCENT'):
                # ABORT is ignored in DESCENT superstate (Amendment #1).
                # Must LAND first, then RESET.
                self.ser.send('n')  # LANDING → LANDED
                time.sleep(0.3)
                self.ser.send('r')  # RESET → IDLE
            else:
                self.ser.send('r')

            time.sleep(0.3)

        phase = self.get_phase()
        if phase == 'IDLE':
            return True
        self.log(f'ERROR: Could not reset to IDLE (stuck in {phase})')
        return False

    def run_test(self, tc):
        """Run a single test case. Returns (pass, detail_str)."""
        # Reset to IDLE before test
        if not self.reset_to_idle():
            return False, 'Could not reset to IDLE'

        self.ser.drain()
        self.ser.buffer = ''

        # Send all commands
        for cmd in tc.commands:
            self.ser.send(cmd)

        output = self.ser.buffer

        # Check expected transitions
        found_transitions = RE_TRANSITION.findall(output)
        for expected in tc.expected_transitions:
            if expected not in found_transitions:
                return False, (f'Missing transition: {expected[0]} -> '
                               f'{expected[1]}\n  Got: {found_transitions}')

        # Check expected pyro intents
        found_pyro = RE_PYRO.findall(output)
        for expected in tc.expected_pyro:
            if expected not in found_pyro:
                return False, (f'Missing pyro intent: {expected}\n'
                               f'  Got: {found_pyro}')

        # Check no-pyro assertion
        if tc.expect_no_pyro and found_pyro:
            return False, f'Unexpected pyro intent: {found_pyro}'

        # Check expected rejection
        if tc.expected_rejection:
            found_rej = RE_REJECTED.findall(output)
            if not any(tc.expected_rejection in r for r in found_rej):
                return False, (f'Missing rejection: "{tc.expected_rejection}"'
                               f'\n  Got: {found_rej}')

        # Check abort-ignored
        if tc.expect_abort_ignored:
            if not RE_ABORT_IGN.search(output):
                return False, 'Missing "ABORT in DESCENT" message'

        # Verify final phase via status query
        if tc.verify_phase:
            actual = self.get_phase()
            if actual != tc.verify_phase:
                return False, (f'Phase mismatch: expected {tc.verify_phase}, '
                               f'got {actual}')

        return True, 'OK'

    def run_all(self, test_num=None):
        """Run all tests (or a single test). Returns True if all pass."""
        tests = TESTS
        if test_num is not None:
            idx = test_num - 1
            if idx < 0 or idx >= len(TESTS):
                print(f'ERROR: Test {test_num} out of range (1-{len(TESTS)})')
                return False
            tests = [TESTS[idx]]

        print(f'=== RocketChip Bench Flight Simulation ===')
        print(f'  Build: {self.build_tag}')
        print()

        passed = 0
        failed = 0
        start_time = time.time()

        for i, tc in enumerate(tests):
            num = (test_num if test_num else i + 1)
            self.log(f'Running test {num}: {tc.name}')

            try:
                ok, detail = self.run_test(tc)
            except serial.SerialException as e:
                ok, detail = False, f'Serial error: {e}'

            status = 'PASS' if ok else 'FAIL'
            if ok:
                passed += 1
            else:
                failed += 1

            # A5: Show expected LED in verbose mode
            led_note = ''
            if self.ser.verbose and tc.verify_phase and tc.verify_phase in PHASE_LED:
                led_note = f'  (LED: {PHASE_LED[tc.verify_phase]})'

            print(f'  [{status}]  {num}. {tc.name}'
                  f'{led_note}'
                  f'{" -- " + detail if not ok else ""}')
            self.log(f'  Result: {status} {detail}')

        elapsed = time.time() - start_time
        total = passed + failed
        print()
        print(f'  RESULT: {passed}/{total} PASS  ({elapsed:.1f}s)')
        print(f'={"=" * 42}')

        return failed == 0

    def write_log(self):
        """A4: Write timestamped log file."""
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        tag = self.build_tag.replace('/', '_')
        filename = f'bench_sim_{tag}_{ts}.log'
        try:
            with open(filename, 'w') as f:
                for line in self.log_lines:
                    f.write(line + '\n')
                f.write(f'\nSerial buffer:\n{self.ser.buffer}\n')
            print(f'  Log: {filename}')
        except OSError as e:
            print(f'  Log write failed: {e}')


# =============================================================================
# Main
# =============================================================================
def main():
    parser = argparse.ArgumentParser(
        description='RocketChip Bench Flight Simulation (IVP-73)')
    parser.add_argument('--port', default='COM7',
                        help='Serial port (default: COM7)')
    parser.add_argument('--baud', type=int, default=115200,
                        help='Baud rate (default: 115200)')
    parser.add_argument('--verbose', action='store_true',
                        help='Print all serial traffic')
    parser.add_argument('--test', type=int, default=None,
                        help='Run only test N (1-9)')
    args = parser.parse_args()

    ser = SerialHelper(args.port, args.baud, args.verbose)

    try:
        ser.connect()
    except serial.SerialException as e:
        print(f'ERROR: Cannot open {args.port}: {e}')
        sys.exit(1)

    runner = FlightSimRunner(ser)

    try:
        # Check build tag
        runner.check_build_tag()

        # Enter flight menu
        runner.enter_flight_menu()

        # Run tests
        all_pass = runner.run_all(test_num=args.test)

        # A4: Write log
        runner.write_log()

    except KeyboardInterrupt:
        print('\nInterrupted.')
        all_pass = False
    finally:
        ser.close()

    # A4: Exit code
    sys.exit(0 if all_pass else 1)


if __name__ == '__main__':
    main()
