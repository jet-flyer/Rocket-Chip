#!/usr/bin/env python3
"""
RocketChip CLI Test Script

Uses Python serial to test CLI functionality reliably.
Run from project root: python scripts/cli_test.py [test_name]

Available tests:
  status    - Show sensor status
  help      - Show help menu
  level_cal - Run level calibration (device must be flat)
  menu      - Show calibration menu
  all       - Run all non-destructive tests
"""

import serial
import time
import sys

PORT = 'COM6'
BAUD = 115200

def connect():
    """Connect to serial port and wait for device"""
    port = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(0.5)  # Let device print banner
    # Drain any pending data
    port.read(1000)
    return port

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

def main():
    tests = {
        'status': test_status,
        'help': test_help,
        'menu': test_calibration_menu,
        'level_cal': test_level_cal,
    }

    if len(sys.argv) < 2:
        print(__doc__)
        print(f"\nUsage: python {sys.argv[0]} <test_name>")
        print(f"Available tests: {', '.join(tests.keys())}, all")
        return 1

    test_name = sys.argv[1].lower()

    try:
        port = connect()
        print(f"Connected to {PORT}\n")
    except Exception as e:
        print(f"Failed to connect to {PORT}: {e}")
        return 1

    try:
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
        elif test_name in tests:
            passed = tests[test_name](port)
            print(f"\nResult: {'PASS' if passed else 'FAIL'}")
        else:
            print(f"Unknown test: {test_name}")
            print(f"Available: {', '.join(tests.keys())}, all")
            return 1
    finally:
        port.close()

    return 0

if __name__ == '__main__':
    sys.exit(main())
