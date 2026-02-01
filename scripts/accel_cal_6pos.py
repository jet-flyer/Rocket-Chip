#!/usr/bin/env python3
"""
RocketChip 6-Position Accelerometer Calibration

Interactive script that guides you through the 6 positions.
The script handles serial communication, you handle moving the device.

Usage: python scripts/accel_cal_6pos.py
"""

import serial
import time
import sys

PORT = 'COM6'
BAUD = 115200

POSITIONS = [
    "LEVEL (flat on table, connector toward you)",
    "LEFT SIDE (left edge down)",
    "RIGHT SIDE (right edge down)",
    "NOSE DOWN (USB connector up)",
    "NOSE UP (USB connector down)",
    "BACK (upside down, screen facing down)",
]

def main():
    print("=== RocketChip 6-Position Accel Calibration ===\n")

    try:
        port = serial.Serial(PORT, BAUD, timeout=2)
        time.sleep(0.5)
        port.read(1000)  # Drain buffer
        print(f"Connected to {PORT}\n")
    except Exception as e:
        print(f"Failed to connect to {PORT}: {e}")
        return 1

    try:
        # Enter calibration menu
        port.write(b'c')
        time.sleep(0.3)
        port.read(2000)

        # Start 6-position cal
        port.write(b'a')
        time.sleep(0.5)
        response = port.read(2000).decode('utf-8', errors='replace')
        print(response)

        if 'Starting 6-position' not in response and '[1/6]' not in response:
            print("ERROR: Failed to start calibration")
            return 1

        # Process each position
        for i, position in enumerate(POSITIONS, 1):
            print(f"\n{'='*50}")
            print(f"POSITION {i}/6: {position}")
            print(f"{'='*50}")

            input("Press ENTER when device is in position...")

            # Send Enter to device
            port.write(b'\r')
            print("Collecting samples...")

            # Wait for collection (up to 10 seconds)
            time.sleep(0.5)
            for _ in range(20):
                response = port.read(500).decode('utf-8', errors='replace')
                if response:
                    print(response, end='')
                if 'done!' in response.lower() or 'complete' in response.lower():
                    break
                if 'failed' in response.lower() or 'error' in response.lower():
                    print(f"\nERROR during position {i}")
                    break
                time.sleep(0.5)

        # Final response
        time.sleep(1)
        response = port.read(2000).decode('utf-8', errors='replace')
        print(response)

        if 'complete' in response.lower() or 'success' in response.lower():
            print("\n✓ Calibration completed successfully!")
        else:
            print("\n⚠ Check if calibration completed properly")

    except KeyboardInterrupt:
        print("\n\nCancelling calibration...")
        port.write(b'x')
        time.sleep(0.2)
        port.read(1000)
    finally:
        port.close()

    return 0

if __name__ == '__main__':
    sys.exit(main())
