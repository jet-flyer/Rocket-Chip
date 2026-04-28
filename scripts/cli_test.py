#!/usr/bin/env python3
"""
RocketChip CLI Test Script

Uses Python serial to test CLI functionality reliably.
Run from project root: python scripts/cli_test.py [--port COMn] <test_name>

Available tests:
  status    - Show sensor status
  help      - Show help menu
  level_cal - Run level calibration (device must be flat)
  menu      - Show calibration menu
  all       - Run all non-destructive tests
"""

import argparse
import serial
import time
import sys
import os

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)
from _rc_test_common import (  # noqa: E402
    Banner,
    find_target_port,
    open_classified_port,
    rc_test,
    TARGET_VEHICLE_ANY,
)

def send_cmd(port, cmd, wait=0.3):
    """Send command and read response"""
    port.write(cmd.encode() if isinstance(cmd, str) else cmd)
    time.sleep(wait)
    return port.read(4000).decode('utf-8', errors='replace')

def test_status(port):
    """Test sensor status command"""
    print("=== TEST: Sensor Status ===")
    response = send_cmd(port, 's', 0.5)
    print(response)

    # Verify expected content
    checks = [
        ('AP_INS:', 'IMU status'),
        ('AP_Compass:', 'Compass status'),
        ('Baro:', 'Barometer status'),
        ('Accel:', 'Accel data'),
        ('Gyro:', 'Gyro data'),
    ]

    passed = True
    for keyword, desc in checks:
        if keyword in response:
            print(f"  [OK] {desc} present")
        else:
            print(f"  [FAIL] {desc} MISSING")
            passed = False

    return passed

def test_help(port):
    """Test help command"""
    print("=== TEST: Help Menu ===")
    response = send_cmd(port, 'h', 0.3)
    print(response)

    checks = [
        ('RocketChip System Status', 'Header'),
        ('h - This help', 'Help option'),
        ('s - Sensor status', 'Status option'),
        ('c - Calibration menu', 'Calibration option'),
    ]

    passed = True
    for keyword, desc in checks:
        if keyword in response:
            print(f"  [OK] {desc} present")
        else:
            print(f"  [FAIL] {desc} MISSING")
            passed = False

    return passed

def test_calibration_menu(port):
    """Test calibration menu"""
    print("=== TEST: Calibration Menu ===")
    response = send_cmd(port, 'c', 0.3)
    print(response)

    checks = [
        ('Calibration Menu', 'Header'),
        ('l - Level calibration', 'Level cal option'),
        ('a - 6-position accel', '6-pos option'),
        ('m - Compass calibration', 'Compass option'),
        ('x - Return to main', 'Exit option'),
    ]

    passed = True
    for keyword, desc in checks:
        if keyword in response:
            print(f"  [OK] {desc} present")
        else:
            print(f"  [FAIL] {desc} MISSING")
            passed = False

    # Return to main menu
    send_cmd(port, 'x', 0.2)
    return passed

def test_level_cal(port):
    """Test level calibration (device must be flat and still!)"""
    print("=== TEST: Level Calibration ===")
    print("NOTE: Device must be FLAT and STILL!")

    # Enter calibration menu
    send_cmd(port, 'c', 0.2)

    # Start level cal
    response = send_cmd(port, 'l', 0.3)
    print(response)

    if 'FLAT and STILL' not in response:
        print("  [FAIL] Level cal prompt not shown")
        send_cmd(port, 'x', 0.2)  # Exit
        return False

    # Send Enter to start calibration
    print("Starting calibration...")
    response = send_cmd(port, '\r', 3.0)
    print(response)

    passed = 'OK' in response or 'completed successfully' in response
    if passed:
        print("  [OK] Level calibration completed")
    else:
        print("  [FAIL] Level calibration FAILED")

    # Return to main menu
    send_cmd(port, 'x', 0.2)
    return passed

@rc_test(target=TARGET_VEHICLE_ANY)
def main():
    tests = {
        'status': test_status,
        'help': test_help,
        'menu': test_calibration_menu,
        'level_cal': test_level_cal,
    }

    parser = argparse.ArgumentParser(
        description='RocketChip CLI serial tests (vehicle firmware).')
    parser.add_argument('--port', default=None,
                        help='Serial port (auto-detect RocketChip USB CDC if omitted)')
    parser.add_argument('test_name', nargs='?',
                        help=f"One of: {', '.join(sorted(tests.keys()))}, all")
    args = parser.parse_args()

    if args.test_name is None:
        print(__doc__)
        print(f"\nUsage: python {sys.argv[0]} [--port COMn] <test_name>")
        print(f"Available tests: {', '.join(tests.keys())}, all")
        return 1

    test_name = args.test_name.lower()

    port_name, meta = find_target_port(
        TARGET_VEHICLE_ANY, override=args.port, verbose=False)
    if port_name is None:
        print(f'INFO: no vehicle port — {meta}')
        print('  Plug the vehicle Feather or pass --port.')
        sys.exit(2)
    if not isinstance(meta, Banner):
        print('ERROR: internal: expected Banner from find_target_port')
        sys.exit(2)

    print(f'Using {port_name} ({meta.short_summary()})\n')

    try:
        with open_classified_port(port_name, target=TARGET_VEHICLE_ANY) as port:
            time.sleep(0.2)
            try:
                port.read(1000)
            except serial.SerialException:
                pass

            if test_name == 'all':
                results = {}
                for name, func in tests.items():
                    if name != 'level_cal':  # Skip destructive tests in 'all'
                        results[name] = func(port)
                        print()

                print("=== SUMMARY ===")
                for name, passed in results.items():
                    status = "PASS" if passed else "FAIL"
                    print(f"  {name}: {status}")
                if not results or not all(results.values()):
                    return 1
            elif test_name in tests:
                passed = tests[test_name](port)
                print(f"\nResult: {'PASS' if passed else 'FAIL'}")
                if not passed:
                    return 1
            else:
                print(f"Unknown test: {test_name}")
                print(f"Available: {', '.join(tests.keys())}, all")
                return 1

    except RuntimeError as e:
        print(f'ERROR: cannot open {port_name}: {e}')
        sys.exit(2)

    return 0

if __name__ == '__main__':
    main()
