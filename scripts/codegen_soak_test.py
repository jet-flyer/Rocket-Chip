#!/usr/bin/env python3
"""
Codegen Sensitivity Disproof Test

Connects to RocketChip serial, verifies build tag, then polls sensor
status every 30s for ~6 minutes. Reports IMU read count and error count
at each interval. PASS = 0 IMU errors after 6 minutes.

Usage: python scripts/codegen_soak_test.py [COM_PORT] [EXPECTED_TAG]
"""

import serial
import sys
import time
import re

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM6"
EXPECTED_TAG = sys.argv[2] if len(sys.argv) > 2 else None
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

def main():
    print(f"=== Codegen Soak Test ===")
    print(f"Port: {PORT}, Duration: {SOAK_DURATION_S}s, Poll: {POLL_INTERVAL_S}s")
    if EXPECTED_TAG:
        print(f"Expected build tag: {EXPECTED_TAG}")
    print()

    # Wait for USB CDC to enumerate after fresh boot
    print(f"Waiting for {PORT} to enumerate...")
    port = None
    for attempt in range(10):
        try:
            port = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
            print(f"Connected on {PORT}")
            break
        except serial.SerialException:
            time.sleep(1.0)
    if port is None:
        print(f"FAIL: Cannot open {PORT} after 10 attempts")
        sys.exit(1)

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
            if EXPECTED_TAG and EXPECTED_TAG not in response:
                print(f"FAIL: Expected tag '{EXPECTED_TAG}' not found in banner!")
                port.close()
                sys.exit(1)
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

    # Reopen port to clear any USB CDC glitch from initial connection
    port.close()
    time.sleep(1.0)
    port = reopen_port(PORT)
    if port is None:
        print("FAIL: Could not reopen port after banner check")
        sys.exit(1)

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

    port.close()
    return 0 if (not any_errors and stats and stats["imu_reads"] > 0) else 1

if __name__ == "__main__":
    sys.exit(main())
