#!/usr/bin/env python3
"""
RocketChip 6-Position Accelerometer Calibration Helper

Human-interactive serial helper for IVP-17 6-position accel calibration.
YOU move the board; this script handles serial I/O and parses results.

Usage:
    python scripts/accel_cal_6pos.py [COM_PORT]

    Default port: COM6

IVP-17 Gate Requirements (pass/fail criteria):
    1. All 6 positions collected (progress 100% each)
    2. Ellipsoid fit converges (result = CAL_RESULT_OK)
    3. Offset < 5 m/s^2 per axis
    4. Scale (diag) 0.8-1.2 per axis
    5. After applying: gravity magnitude 9.81 +/-0.02 m/s^2
    6. Persists across power cycle
"""

import serial
import time
import sys
import re

DEFAULT_PORT = 'COM6'
BAUD = 115200

# Position descriptions for the human operator
POSITIONS = [
    "LEVEL (+Z up) - Flat on table, component side UP",
    "LEFT SIDE (+Y up) - Standing on left edge",
    "RIGHT SIDE (-Y up) - Standing on right edge",
    "NOSE DOWN (+X up) - Standing on USB connector end",
    "NOSE UP (-X up) - Standing on opposite end from USB",
    "INVERTED (-Z up) - Flat on table, component side DOWN",
]


def connect(port_name):
    """Connect to RocketChip serial port."""
    port = serial.Serial(port_name, BAUD, timeout=2)
    time.sleep(1.0)  # Let device settle + print banner
    port.read(4096)  # Drain banner
    return port


def send_and_read(port, data, wait=0.5, max_bytes=4096):
    """Send data and read response."""
    port.write(data if isinstance(data, bytes) else data.encode())
    time.sleep(wait)
    return port.read(max_bytes).decode('utf-8', errors='replace')


def navigate_to_cal_menu(port):
    """Navigate from main menu to calibration menu."""
    response = send_and_read(port, 'c', 0.3)
    if 'Calibration Menu' not in response:
        print("[WARN] Didn't see calibration menu, trying again...")
        response = send_and_read(port, 'c', 0.3)
    return 'Calibration Menu' in response


def parse_results(output):
    """Parse calibration results from firmware output."""
    results = {
        'offset': [None, None, None],
        'scale': [None, None, None],
        'offdiag': [None, None, None],
        'gravity_mag': None,
        'gravity_error': None,
        'fit_ok': False,
        'save_ok': False,
    }

    # Parse offset line: "Offset: X=0.1234 Y=-0.0567 Z=0.0890"
    m = re.search(r'Offset:\s+X=([\d.e+-]+)\s+Y=([\d.e+-]+)\s+Z=([\d.e+-]+)', output)
    if m:
        results['offset'] = [float(m.group(1)), float(m.group(2)), float(m.group(3))]

    # Parse scale line: "Scale:  X=1.0012 Y=0.9987 Z=1.0023"
    m = re.search(r'Scale:\s+X=([\d.e+-]+)\s+Y=([\d.e+-]+)\s+Z=([\d.e+-]+)', output)
    if m:
        results['scale'] = [float(m.group(1)), float(m.group(2)), float(m.group(3))]

    # Parse offdiag line: "Offdiag: XY=0.0012 XZ=-0.0034 YZ=0.0056"
    m = re.search(r'Offdiag:\s+XY=([\d.e+-]+)\s+XZ=([\d.e+-]+)\s+YZ=([\d.e+-]+)', output)
    if m:
        results['offdiag'] = [float(m.group(1)), float(m.group(2)), float(m.group(3))]

    # Parse gravity magnitude: "Avg gravity magnitude: 9.8065 m/s^2"
    m = re.search(r'Avg gravity magnitude:\s+([\d.]+)', output)
    if m:
        results['gravity_mag'] = float(m.group(1))

    # Parse error: "PASS (error 0.0002 < 0.02)" or "WARNING: error 0.0300 > 0.02"
    m = re.search(r'error\s+([\d.]+)', output)
    if m:
        results['gravity_error'] = float(m.group(1))

    results['fit_ok'] = 'Computing ellipsoid fit... OK' in output
    results['save_ok'] = 'Saving to flash... OK' in output

    return results


def check_ivp17_gates(results):
    """Check IVP-17 gate requirements."""
    print("\n" + "=" * 50)
    print("  IVP-17 Gate Check")
    print("=" * 50)

    all_pass = True

    # Gate 2: Fit converged
    status = "PASS" if results['fit_ok'] else "FAIL"
    print(f"  [{status}] Gate 2: Ellipsoid fit converged")
    if not results['fit_ok']:
        all_pass = False

    # Gate 3: Offset < 5 m/s^2 per axis
    if results['offset'][0] is not None:
        offset_ok = all(abs(v) < 5.0 for v in results['offset'])
        status = "PASS" if offset_ok else "FAIL"
        print(f"  [{status}] Gate 3: Offset < 5.0 m/s^2 "
              f"(X={results['offset'][0]:.4f} Y={results['offset'][1]:.4f} "
              f"Z={results['offset'][2]:.4f})")
        if not offset_ok:
            all_pass = False
    else:
        print("  [SKIP] Gate 3: Could not parse offset")

    # Gate 4: Scale 0.8-1.2 per axis
    if results['scale'][0] is not None:
        scale_ok = all(0.8 <= v <= 1.2 for v in results['scale'])
        status = "PASS" if scale_ok else "FAIL"
        print(f"  [{status}] Gate 4: Scale 0.8-1.2 "
              f"(X={results['scale'][0]:.4f} Y={results['scale'][1]:.4f} "
              f"Z={results['scale'][2]:.4f})")
        if not scale_ok:
            all_pass = False
    else:
        print("  [SKIP] Gate 4: Could not parse scale")

    # Gate 5: Gravity magnitude 9.81 +/-0.02
    if results['gravity_mag'] is not None:
        grav_error = abs(results['gravity_mag'] - 9.80665)
        grav_ok = grav_error < 0.02
        status = "PASS" if grav_ok else "FAIL"
        print(f"  [{status}] Gate 5: Gravity magnitude "
              f"{results['gravity_mag']:.4f} m/s^2 (error {grav_error:.4f})")
        if not grav_ok:
            all_pass = False
    else:
        print("  [SKIP] Gate 5: Could not parse gravity magnitude")

    # Gate 6: Saved to flash
    status = "PASS" if results['save_ok'] else "FAIL"
    print(f"  [{status}] Gate 6: Saved to flash")
    if not results['save_ok']:
        all_pass = False

    print("=" * 50)
    verdict = "ALL GATES PASS" if all_pass else "GATE FAILURES"
    print(f"  IVP-17: {verdict}")
    print("=" * 50)
    print()

    if all_pass:
        print("Gate 6 (power cycle persistence): Unplug USB, replug,")
        print("press 'h' in main menu and verify Accel shows '6POS'.")

    return all_pass


def main():
    port_name = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT

    print("=" * 50)
    print("  RocketChip 6-Position Accel Calibration")
    print("=" * 50)
    print()
    print("This script guides you through physically positioning")
    print("the board in 6 orientations. The firmware collects")
    print("samples and runs an ellipsoid fit.")
    print()
    print("You will need to hold the board still in each position")
    print("for about 1 second when prompted.")
    print()

    try:
        print(f"Connecting to {port_name}...")
        port = connect(port_name)
        print(f"Connected to {port_name}")
    except Exception as e:
        print(f"Failed to connect to {port_name}: {e}")
        print("Is the device plugged in and terminal closed?")
        return 1

    try:
        # Navigate to calibration menu
        print("\nNavigating to calibration menu...")
        if not navigate_to_cal_menu(port):
            print("ERROR: Could not open calibration menu")
            return 1
        print("Calibration menu open.\n")

        # Start 6-pos calibration
        print("Starting 6-position calibration ('a')...")
        response = send_and_read(port, 'a', 1.0)
        print(response)

        if '6-Position Accelerometer Calibration' not in response:
            print("ERROR: 6-pos calibration did not start")
            print("Check that IMU is available and callback is wired.")
            return 1

        # Collect all firmware output during the calibration
        all_output = response

        # Guide user through each position
        for pos in range(6):
            print(f"\n{'=' * 40}")
            print(f"  Position {pos + 1}/6: {POSITIONS[pos]}")
            print(f"{'=' * 40}")

            # Read firmware prompt
            time.sleep(0.5)
            prompt = port.read(2048).decode('utf-8', errors='replace')
            all_output += prompt
            if prompt.strip():
                print(prompt)

            input(">>> Press ENTER here when board is positioned... ")

            # Send ENTER to firmware
            port.write(b'\r')

            # Wait for sampling to complete (50 samples at ~100Hz = ~500ms + processing)
            time.sleep(2.0)
            result = port.read(4096).decode('utf-8', errors='replace')
            all_output += result
            print(result)

            if 'FAILED' in result and 'motion' in result.lower():
                print("Motion was detected. Try holding more still.")
                # Firmware handles retries internally, read more output
                time.sleep(1.0)
                more = port.read(4096).decode('utf-8', errors='replace')
                all_output += more
                if more.strip():
                    print(more)

            if 'aborted' in result.lower() or 'Calibration aborted' in all_output:
                print("\nCalibration was aborted by firmware.")
                return 1

        # Wait for fit computation and results
        print("\nWaiting for ellipsoid fit computation...")
        time.sleep(3.0)
        final_output = port.read(8192).decode('utf-8', errors='replace')
        all_output += final_output
        print(final_output)

        # Parse and check IVP-17 gates
        results = parse_results(all_output)
        passed = check_ivp17_gates(results)

        # Return to main menu
        send_and_read(port, 'x', 0.3)

        return 0 if passed else 1

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        # Send ESC to cancel any active calibration
        port.write(b'\x1b')
        time.sleep(0.5)
        return 1

    finally:
        port.close()
        print("Serial port closed.")


if __name__ == '__main__':
    sys.exit(main())
