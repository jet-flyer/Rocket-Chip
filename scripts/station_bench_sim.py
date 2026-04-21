#!/usr/bin/env python3
"""
RocketChip Station Bench Sim (IVP-146, 2026-04-18)

Programmatic station-role regression detector — parallel to the vehicle
bench_sim.py. Closes the "vehicle has a machine-checkable rot detector,
station has manual serial capture" asymmetry flagged during IVP-142c.

Scope: verify the station CLI dispatch path + banner hygiene + live
HealthMonitor output + absence of QP assertions over a short soak.
Anything a future station SPIN model proves is NOT tested here.

Three tests:
  1. Boot + dashboard round-trip (main menu reachable, no QP markers).
  2. Hardware Status positive controls (Radio RegVersion=0x12, [N/A] for
     uninstalled sensors — regression guard for IVP-142c presentation).
  3. Health pipeline populated (diag_stats [Health] line shows non-zero
     secondary byte — regression guard for AO_HealthMonitor-on-station
     starting under capability-masking).

Port detection: filters USB CDC ports by RocketChip VID:PID
(0x2E8A:0x0009). Use --port to override.

Exit codes:
    0 = all tests pass, no assertions
    1 = one or more test failures
    2 = could not connect to target or tool self-check failed
"""

import argparse
import re
import serial
import serial.tools.list_ports
import sys
import time

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

ROCKETCHIP_USB_VID = 0x2E8A
ROCKETCHIP_USB_PID = 0x0009


# =============================================================================
# Serial helpers
# =============================================================================

def _peek_banner(port_name, timeout=2.0):
    """Briefly connect + read banner text to identify vehicle vs station.
    Returns the banner string (lowercase) or '' on failure."""
    try:
        s = serial.Serial(port_name, 115200, timeout=0.5)
        s.write(b'h')
        time.sleep(timeout)
        data = s.read(8000)
        s.close()
        return data.decode('utf-8', errors='replace').lower()
    except (serial.SerialException, OSError):
        return ''


def find_port():
    """Return the first USB CDC port identifying as the station, else the
    first RocketChip port at all (backward-compat for single-board setups).
    Station is identified by 'ground station' or 'fruit jam' in its banner.
    When both vehicle and station are plugged in (Stage T dev pattern), this
    ensures we connect to the station, not the vehicle."""
    candidates = [
        info.device for info in serial.tools.list_ports.comports()
        if info.vid == ROCKETCHIP_USB_VID and info.pid == ROCKETCHIP_USB_PID
    ]
    if not candidates:
        return None
    # Probe each candidate for station banner text.
    for port in candidates:
        banner = _peek_banner(port)
        if 'ground station' in banner or 'fruit jam' in banner:
            return port
    # Fallback: no station match — return first candidate (old behavior).
    return candidates[0]


def connect(port, retries=5, retry_delay=1.5):
    last_err = None
    for attempt in range(retries):
        try:
            p = serial.Serial(port, 115200, timeout=0.1)
            time.sleep(1.0)
            p.read(10000)
            time.sleep(0.3)
            p.read(10000)
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
# Test cases
# =============================================================================

def test_boot_and_main_menu(port, state, verbose=False):
    """Verify `x` reaches the main menu (or remains there) without assertions."""
    out = send_and_read(port, 'x', 2.0)
    state['boot'] = out
    if '[main]' not in out and 'RocketChip' not in out:
        return False, f'Main menu prompt not seen (saw {len(out)}B)'
    return True, 'OK'


def test_hardware_status(port, state, verbose=False):
    """Verify Hardware Status shows [N/A] for uninstalled sensors (IVP-142c)."""
    send_and_read(port, 'q', 1.5)
    out = send_and_read(port, 'b', 4.0)
    send_and_read(port, 'x', 1.0)

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
    """Verify diag_stats [Health] line non-zero secondary + RegVersion=0x12."""
    send_and_read(port, 'q', 1.5)
    out = send_and_read(port, 'd', 5.0)
    send_and_read(port, 'x', 1.0)

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

def main():
    parser = argparse.ArgumentParser(
        description='RocketChip Station Bench Sim (IVP-146, 3 tests)')
    parser.add_argument('--port', default=None,
                        help='Serial port (auto-detected via VID:PID)')
    parser.add_argument('--verbose', action='store_true',
                        help='Print verbose test output')
    parser.add_argument('--max-runtime', type=float, default=60.0,
                        help='Top-level wall-clock deadline (default 60s)')
    args = parser.parse_args()

    port_name = args.port or find_port()
    if port_name is None:
        print('ERROR: No RocketChip USB CDC port found '
              f'(VID=0x{ROCKETCHIP_USB_VID:04X} PID=0x{ROCKETCHIP_USB_PID:04X}).')
        print('  Flash the station with build_station/*.uf2 and connect USB,')
        print('  or specify --port manually.')
        sys.exit(2)
    print(f'using station port: {port_name}')

    try:
        ser = connect(port_name)
    except (RuntimeError, serial.SerialException, OSError) as e:
        print(f'ERROR: Cannot open {port_name}: {e}')
        sys.exit(2)

    print(f'\n=== RocketChip Station Bench Sim ===')
    print(f'  Port: {port_name}')
    print(f'  Max runtime: {args.max_runtime:.0f}s')
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

    ser.close()
    sys.exit(0 if failed == 0 else 1)


if __name__ == '__main__':
    main()
