#!/usr/bin/env python3
"""
I2C Bus Soak Test

Connects to RocketChip serial, verifies build tag, then polls sensor
status at regular intervals. Monitors IMU, baro, and GPS error counts.

PASS  = zero errors after full soak duration
FAIL  = any sensor errors detected (early exit by default)

Usage:
  python scripts/i2c_soak_test.py [OPTIONS] [COM_PORT] [EXPECTED_TAG]

Options:
  --no-early-exit   Run full duration even if errors appear (for data collection)
  --duration=N      Soak duration in seconds (default: 360)
  --poll=N          Poll interval in seconds (default: 15)
  --reset           Reset MCU via SWD debug probe before starting soak

Examples:
  python scripts/i2c_soak_test.py                          # Defaults
  python scripts/i2c_soak_test.py --reset COM6 baseline    # Clean reset + soak
  python scripts/i2c_soak_test.py --no-early-exit COM6     # Collect full data
  python scripts/i2c_soak_test.py --duration=120 COM6      # Quick 2-min soak
"""

import serial
import subprocess
import sys
import time
import re

# Defaults
DEFAULT_PORT = "COM6"
DEFAULT_BAUD = 115200
DEFAULT_POLL_S = 15
DEFAULT_DURATION_S = 360  # 6 minutes
DEFAULT_TIMEOUT = 1

# Early exit: how many consecutive polls with zero new IMU reads = stall
STALL_THRESHOLD = 2

# Debug probe paths (Pico SDK — NOT system OpenOCD/GDB)
OPENOCD_EXE = r"C:\Users\pow-w\.pico-sdk\openocd\0.12.0+dev\openocd.exe"
OPENOCD_SCRIPTS = r"C:\Users\pow-w\.pico-sdk\openocd\0.12.0+dev\scripts"
GDB_EXE = r"C:\Users\pow-w\.pico-sdk\toolchain\14_2_Rel1\bin\arm-none-eabi-gdb.exe"
ELF_PATH = r"build\rocketchip.elf"


def parse_args():
    """Parse positional and --flag arguments."""
    early_exit = True
    duration_s = DEFAULT_DURATION_S
    poll_s = DEFAULT_POLL_S
    port = DEFAULT_PORT
    expected_tag = None
    do_reset = False

    positional = []
    for arg in sys.argv[1:]:
        if arg == "--no-early-exit":
            early_exit = False
        elif arg == "--reset":
            do_reset = True
        elif arg.startswith("--duration="):
            duration_s = int(arg.split("=", 1)[1])
        elif arg.startswith("--poll="):
            poll_s = int(arg.split("=", 1)[1])
        else:
            positional.append(arg)

    if len(positional) >= 1:
        port = positional[0]
    if len(positional) >= 2:
        expected_tag = positional[1]

    return port, expected_tag, early_exit, duration_s, poll_s, do_reset


def reset_mcu_via_probe():
    """Reset the MCU via SWD debug probe (OpenOCD + GDB).

    Halts both cores, resets cleanly. No mid-transaction I2C interruption.
    USB CDC will re-enumerate after reset.
    """
    print("[0] Resetting MCU via debug probe...")

    # Start OpenOCD (kill any stale instance first)
    subprocess.run(
        ["taskkill", "/F", "/IM", "openocd.exe"],
        capture_output=True,
    )
    time.sleep(2.0)

    ocd_proc = subprocess.Popen(
        [
            OPENOCD_EXE,
            "-s", OPENOCD_SCRIPTS,
            "-f", "interface/cmsis-dap.cfg",
            "-f", "target/rp2350.cfg",
            "-c", "adapter speed 5000",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    time.sleep(3.0)  # Let OpenOCD connect to probe

    if ocd_proc.poll() is not None:
        stderr = ocd_proc.stderr.read().decode(errors="replace")
        print(f"    FAIL: OpenOCD exited immediately. Is debug probe connected?")
        print(f"    stderr: {stderr[:300]}")
        return False

    # Issue reset via GDB
    result = subprocess.run(
        [
            GDB_EXE,
            ELF_PATH,
            "-batch",
            "-ex", "target extended-remote localhost:3333",
            "-ex", "monitor reset run",
        ],
        capture_output=True,
        timeout=15,
    )

    # Kill OpenOCD — we only needed it for the reset
    subprocess.run(["taskkill", "/F", "/IM", "openocd.exe"], capture_output=True)

    if result.returncode != 0:
        stderr = result.stderr.decode(errors="replace")
        print(f"    FAIL: GDB reset failed (rc={result.returncode})")
        print(f"    stderr: {stderr[:300]}")
        return False

    print("    MCU reset. Waiting for USB CDC re-enumeration...")
    time.sleep(5.0)  # USB CDC takes a few seconds to re-enumerate after reset
    return True


def reopen_port(port_name):
    """Close and reopen serial port to recover from USB CDC glitch."""
    for attempt in range(5):
        try:
            p = serial.Serial(port_name, DEFAULT_BAUD, timeout=DEFAULT_TIMEOUT)
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
        text,
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


def has_errors(stats):
    """Check if any sensor has errors."""
    if stats is None:
        return False
    return (
        stats["imu_errors"] > 0
        or stats["baro_errors"] > 0
        or stats["gps_errors"] > 0
    )


def print_table_header():
    """Print the soak table column headers."""
    print(
        f"    {'Time':>6}  {'IMU Reads':>12}  {'IMU Err':>8}  "
        f"{'Baro Reads':>11}  {'Baro Err':>8}  "
        f"{'GPS Reads':>10}  {'GPS Err':>8}  {'IMU/s':>6}"
    )
    print(
        f"    {'-'*6}  {'-'*12}  {'-'*8}  "
        f"{'-'*11}  {'-'*8}  "
        f"{'-'*10}  {'-'*8}  {'-'*6}"
    )


def print_table_row(elapsed, stats, rate, flag=""):
    """Print one row of the soak table."""
    print(
        f"    {elapsed:5.0f}s  {stats['imu_reads']:>12,}  {stats['imu_errors']:>8,}  "
        f"{stats['baro_reads']:>11,}  {stats['baro_errors']:>8,}  "
        f"{stats['gps_reads']:>10,}  {stats['gps_errors']:>8,}  "
        f"{rate:>6.0f}{flag}"
    )


def main():
    port_name, expected_tag, early_exit, duration_s, poll_s, do_reset = parse_args()

    mode = "early-exit" if early_exit else "full-duration"
    print(f"=== I2C Bus Soak Test ===")
    print(f"Port: {port_name}, Duration: {duration_s}s, Poll: {poll_s}s, Mode: {mode}")
    if expected_tag:
        print(f"Expected build tag: {expected_tag}")
    print()

    # Optional: reset MCU via debug probe for clean start
    if do_reset:
        if not reset_mcu_via_probe():
            print("FAIL: Could not reset MCU via probe. Continuing without reset...")
            # Don't abort — user may want to test anyway

    # Connect
    print(f"Waiting for {port_name} to enumerate...")
    port = None
    for attempt in range(10):
        try:
            port = serial.Serial(port_name, DEFAULT_BAUD, timeout=DEFAULT_TIMEOUT)
            print(f"Connected on {port_name}")
            break
        except serial.SerialException:
            time.sleep(1.0)
    if port is None:
        print(f"FAIL: Cannot open {port_name} after 10 attempts")
        return 1

    time.sleep(2.0)
    drain(port)
    time.sleep(1.0)
    drain(port)

    # Step 1: Verify build tag
    print("[1] Checking build tag...")
    response = send_and_read(port, "b", wait_s=2.0)
    if "Build:" not in response:
        response = send_and_read(port, "b", wait_s=3.0)

    if "Build:" in response:
        build_line = [l for l in response.splitlines() if "Build:" in l]
        if build_line:
            print(f"    {build_line[0].strip()}")
            if expected_tag and expected_tag not in response:
                print(f"FAIL: Expected tag '{expected_tag}' not found!")
                port.close()
                return 1
    else:
        print(f"    WARNING: Could not verify build tag. Response:\n{response[:200]}")

    # Reopen port
    port.close()
    time.sleep(1.0)
    port = reopen_port(port_name)
    if port is None:
        print("FAIL: Could not reopen port after banner check")
        return 1

    # Step 2: Initial status
    print("[2] Initial sensor status...")
    response = send_and_read(port, "s", wait_s=2.0)
    stats = parse_reads_errors(response)
    if stats:
        print(f"    IMU:  {stats['imu_reads']} reads, {stats['imu_errors']} errors")
        print(f"    Baro: {stats['baro_reads']} reads, {stats['baro_errors']} errors")
        print(f"    GPS:  {stats['gps_reads']} reads, {stats['gps_errors']} errors")
        if has_errors(stats):
            print("    WARNING: errors already present at start (prior session residual)")
    else:
        print(f"    WARNING: Could not parse status. Response:\n{response[:300]}")

    # Step 3: Soak loop
    print(f"\n[3] Starting {duration_s}s soak (polling every {poll_s}s)...")
    print_table_header()

    start_time = time.time()
    prev_imu_reads = stats["imu_reads"] if stats else 0
    prev_time = start_time
    any_errors = False
    stall_count = 0
    fail_reason = None

    while True:
        elapsed = time.time() - start_time
        if elapsed >= duration_s:
            break
        time.sleep(poll_s)

        elapsed = time.time() - start_time
        response = send_and_read(port, "s", wait_s=2.0)
        stats = parse_reads_errors(response)

        if stats is None:
            print(f"    {elapsed:5.0f}s  [parse error]")
            continue

        dt = time.time() - prev_time
        delta_reads = stats["imu_reads"] - prev_imu_reads
        rate = delta_reads / dt if dt > 0 else 0
        prev_imu_reads = stats["imu_reads"]
        prev_time = time.time()

        # Check for errors
        flag = ""
        if has_errors(stats):
            any_errors = True
            flag = "  <<< ERRORS"

        # Check for stall (zero new IMU reads)
        if delta_reads == 0:
            stall_count += 1
            if stall_count >= STALL_THRESHOLD:
                flag = "  <<< STALL"
                fail_reason = f"IMU read stall ({stall_count} consecutive polls with 0 new reads)"
        else:
            stall_count = 0

        print_table_row(elapsed, stats, rate, flag)

        # Early exit on errors
        if early_exit and has_errors(stats):
            fail_reason = (
                f"I2C errors detected at {elapsed:.0f}s — "
                f"IMU:{stats['imu_errors']} Baro:{stats['baro_errors']} GPS:{stats['gps_errors']}"
            )
            print(f"\n    EARLY EXIT: {fail_reason}")
            break

        # Early exit on stall
        if early_exit and stall_count >= STALL_THRESHOLD:
            print(f"\n    EARLY EXIT: {fail_reason}")
            break

    # Step 4: Final assessment
    print()
    elapsed = time.time() - start_time
    if stats:
        print(
            f"=== FINAL ({elapsed:.0f}s): "
            f"IMU {stats['imu_reads']:,}r/{stats['imu_errors']:,}e | "
            f"Baro {stats['baro_reads']:,}r/{stats['baro_errors']:,}e | "
            f"GPS {stats['gps_reads']:,}r/{stats['gps_errors']:,}e ==="
        )

    if fail_reason:
        print(f"RESULT: FAIL — {fail_reason}")
    elif any_errors:
        print("RESULT: FAIL — errors detected during soak")
    elif stats and stats["imu_reads"] > 0:
        completed = "full" if elapsed >= duration_s - 1 else f"partial ({elapsed:.0f}s)"
        print(f"RESULT: PASS — zero errors ({completed} soak)")
    else:
        print("RESULT: INCONCLUSIVE — could not read sensor data")

    port.close()
    return 0 if (not any_errors and not fail_reason and stats and stats["imu_reads"] > 0) else 1


if __name__ == "__main__":
    sys.exit(main())
