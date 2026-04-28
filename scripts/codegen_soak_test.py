#!/usr/bin/env python3
"""
Codegen Sensitivity Disproof Test

Connects to RocketChip serial, verifies build tag, then polls sensor
status every 30s for ~6 minutes. Reports IMU read count and error count
at each interval. PASS = 0 IMU errors after 6 minutes.

Usage: python scripts/codegen_soak_test.py [--port COMn] [EXPECTED_TAG]
"""

import argparse
import serial
import sys
import time
import re
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

BAUD = 115200
POLL_INTERVAL_S = 30
SOAK_DURATION_S = 360  # 6 minutes
TIMEOUT = 1

def reopen_port(port_name):
    """Close and reopen serial port to recover from USB CDC glitch."""
    for attempt in range(5):
        try:
            p = serial.Serial(port_name, BAUD, timeout=TIMEOUT)
            time.sleep(1.0)
            return p
        except serial.SerialException:
            time.sleep(1.0)
    return None

def drain(port):
    """Drain any buffered data."""
    time.sleep(0.3)
    try:
        while port.in_waiting > 0:
            port.read(port.in_waiting)
            time.sleep(0.1)
    except serial.SerialException:
        pass

def send_and_read(port, cmd, wait_s=1.5):
    """Send a single-char command and read response."""
    drain(port)
    try:
        port.write(cmd.encode())
    except serial.SerialException:
        return ""
    time.sleep(wait_s)
    data = b""
    try:
        while port.in_waiting > 0:
            data += port.read(port.in_waiting)
            time.sleep(0.1)
    except serial.SerialException:
        pass
    return data.decode("utf-8", errors="replace")

def parse_reads_errors(text):
    """Parse 'Reads: I=123 B=456 G=789  Errors: I=0 B=0 G=0' line."""
    m = re.search(
        r"Reads:\s+I=(\d+)\s+B=(\d+)\s+G=(\d+)\s+Errors:\s+I=(\d+)\s+B=(\d+)\s+G=(\d+)",
        text
    )
    if m:
        return {
            "imu_reads": int(m.group(1)),
            "baro_reads": int(m.group(2)),
            "gps_reads": int(m.group(3)),
            "imu_errors": int(m.group(4)),
            "baro_errors": int(m.group(5)),
            "gps_errors": int(m.group(6)),
        }
    return None

@rc_test(target=TARGET_VEHICLE_ANY)
def main():
    parser = argparse.ArgumentParser(
        description='Codegen sensitivity disproof soak (vehicle firmware).')
    parser.add_argument('--port', default=None,
                        help='Serial port (auto-detect RocketChip USB CDC if omitted)')
    parser.add_argument('expected_tag', nargs='?', default=None,
                        help='Optional build SHA substring to verify in banner')
    args = parser.parse_args()

    port_name, meta = find_target_port(
        TARGET_VEHICLE_ANY, override=args.port, verbose=False)
    if port_name is None:
        print(f'INFO: no vehicle port — {meta}')
        print('  Plug the vehicle Feather or pass --port.')
        sys.exit(2)
    if not isinstance(meta, Banner):
        print('ERROR: internal: expected Banner from find_target_port')
        sys.exit(2)

    expected_tag = args.expected_tag

    print(f"=== Codegen Soak Test ===")
    print(f"Port: {port_name} ({meta.short_summary()}), "
          f"Duration: {SOAK_DURATION_S}s, Poll: {POLL_INTERVAL_S}s")
    if expected_tag:
        print(f"Expected build tag: {expected_tag}")
    print()

    try:
        with open_classified_port(port_name, target=TARGET_VEHICLE_ANY) as port:
            time.sleep(2.0)
            drain(port)

            # Give firmware time to detect USB connection and print banner
            time.sleep(1.0)
            drain(port)

            # Step 1: Verify build tag via 'b' command (hardware status / boot banner)
            print("[1] Checking build tag...")
            response = send_and_read(port, "b", wait_s=2.0)

            if "Build:" in response:
                build_line = [l for l in response.splitlines() if "Build:" in l]
                if build_line:
                    print(f"    {build_line[0].strip()}")
                    if expected_tag and expected_tag not in response:
                        print(f"FAIL: Expected tag '{expected_tag}' not found in banner!")
                        return 1
            else:
                # Maybe banner already printed, try again
                print("    No 'Build:' in response, retrying...")
                response = send_and_read(port, "b", wait_s=3.0)
                if "Build:" in response:
                    build_line = [l for l in response.splitlines() if "Build:" in l]
                    if build_line:
                        print(f"    {build_line[0].strip()}")
                else:
                    print(f"    WARNING: Could not verify build tag. Response:\n{response[:200]}")
    except RuntimeError as e:
        print(f'ERROR: cannot open {port_name}: {e}')
        sys.exit(2)

    # Reopen port to clear any USB CDC glitch from initial connection
    time.sleep(1.0)
    port = reopen_port(port_name)
    if port is None:
        print("FAIL: Could not reopen port after banner check")
        return 1

    try:
        # Step 2: Initial status check
        print("[2] Initial sensor status...")
        response = send_and_read(port, "s", wait_s=2.0)
        stats = parse_reads_errors(response)
        if stats:
            print(f"    IMU: {stats['imu_reads']} reads, {stats['imu_errors']} errors")
            print(f"    Baro: {stats['baro_reads']} reads, {stats['baro_errors']} errors")
        else:
            print(f"    WARNING: Could not parse status. Response:\n{response[:300]}")

        # Step 3: Soak loop
        print(f"\n[3] Starting {SOAK_DURATION_S}s soak test (polling every {POLL_INTERVAL_S}s)...")
        print(f"    {'Time':>6}  {'IMU Reads':>12}  {'IMU Err':>8}  {'Baro Reads':>12}  {'Baro Err':>8}  {'Rate/s':>8}")
        print(f"    {'-'*6}  {'-'*12}  {'-'*8}  {'-'*12}  {'-'*8}  {'-'*8}")

        start_time = time.time()
        prev_imu_reads = stats["imu_reads"] if stats else 0
        prev_time = start_time
        any_errors = False

        while True:
            elapsed = time.time() - start_time
            if elapsed >= SOAK_DURATION_S:
                break
            time.sleep(POLL_INTERVAL_S)

            elapsed = time.time() - start_time
            response = send_and_read(port, "s", wait_s=2.0)
            stats = parse_reads_errors(response)

            if stats:
                dt = time.time() - prev_time
                rate = (stats["imu_reads"] - prev_imu_reads) / dt if dt > 0 else 0
                prev_imu_reads = stats["imu_reads"]
                prev_time = time.time()

                err_flag = " <<<" if stats["imu_errors"] > 0 or stats["baro_errors"] > 0 else ""
                if stats["imu_errors"] > 0 or stats["baro_errors"] > 0:
                    any_errors = True

                print(f"    {elapsed:5.0f}s  {stats['imu_reads']:>12,}  {stats['imu_errors']:>8,}  "
                      f"{stats['baro_reads']:>12,}  {stats['baro_errors']:>8,}  {rate:>8.0f}{err_flag}")
            else:
                print(f"    {elapsed:5.0f}s  [parse error]")

        # Step 4: Final assessment
        print()
        if stats:
            print(f"=== FINAL: {stats['imu_reads']:,} IMU reads, {stats['imu_errors']:,} errors "
                  f"| {stats['baro_reads']:,} baro reads, {stats['baro_errors']:,} errors ===")

        if any_errors:
            print("RESULT: FAIL — errors detected during soak")
        elif stats and stats["imu_reads"] > 0:
            print("RESULT: PASS — zero errors")
        else:
            print("RESULT: INCONCLUSIVE — could not read sensor data")

        return 0 if (not any_errors and stats and stats["imu_reads"] > 0) else 1
    finally:
        try:
            port.close()
        except serial.SerialException:
            pass

if __name__ == "__main__":
    main()
