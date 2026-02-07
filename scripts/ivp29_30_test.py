#!/usr/bin/env python3
"""
IVP-29/30 Soak & Watchdog Test Monitor

Monitors serial output during IVP-29 gate check and IVP-30 watchdog soak
(5 minutes). After soak completes, guides user through manual tests:
  - 'o' Core 0 stack overflow (MPU fault test)
  - 'w' Core 0 stall (watchdog test)
  - 'W' Core 1 stall (watchdog test)

Usage:
    python scripts/ivp29_30_test.py [--port COM6]

The device should already be running with IVP-29/30 active (kSkipVerifiedGates=true
skips IVP-10 through IVP-28).
"""

import serial
import time
import sys
import argparse
from datetime import datetime, timedelta

PORT = 'COM6'
BAUD = 115200
LOG_FILE = 'ivp29_30_log.txt'
SOAK_DURATION_S = 5 * 60  # 5 minutes


def parse_args():
    parser = argparse.ArgumentParser(description='IVP-29/30 soak + watchdog monitor')
    parser.add_argument('--port', default=PORT, help='Serial port (default COM6)')
    return parser.parse_args()


def open_log(path):
    f = open(path, 'w', encoding='utf-8')
    f.write(f"# IVP-29/30 Test Log — {datetime.now().isoformat()}\n\n")
    return f


def log_and_print(logf, line):
    print(line, end='')
    logf.write(line)
    logf.flush()


def drain(port):
    port.timeout = 0.1
    while port.read(4096):
        pass
    port.timeout = 1


def wait_for_ivp29(port, logf, timeout_s=120):
    """Wait for IVP-29 gate check output."""
    log_and_print(logf, "[script] Waiting for IVP-29 gate check...\n")
    port.timeout = 2
    buf = ''
    start = time.time()
    while time.time() - start < timeout_s:
        chunk = port.read(4096)
        if chunk:
            text = chunk.decode('utf-8', errors='replace')
            buf += text
            log_and_print(logf, text)
            if 'IVP-29: AUTOMATED GATES' in buf:
                log_and_print(logf, "\n[script] [PASS] IVP-29 gate check detected\n")
                return True
        time.sleep(0.1)
    log_and_print(logf, "[script] [WARN] IVP-29 output not seen within timeout\n")
    return False


def monitor_ivp30_soak(port, logf):
    """Monitor the 5-minute IVP-30 watchdog soak."""
    log_and_print(logf, "\n[script] Monitoring IVP-30 watchdog soak (5 min)...\n")
    log_and_print(logf, f"[script] Estimated completion: "
                  f"{(datetime.now() + timedelta(seconds=SOAK_DURATION_S)).strftime('%H:%M:%S')}\n\n")

    port.timeout = 5
    buf = ''
    start = time.time()

    while True:
        try:
            chunk = port.read(4096)
        except (serial.SerialException, OSError):
            log_and_print(logf, "[script] USB disconnected during soak!\n")
            return False

        if chunk:
            text = chunk.decode('utf-8', errors='replace')
            buf += text
            log_and_print(logf, text)

            if 'IVP-30: AUTOMATED GATES' in buf:
                log_and_print(logf, "\n[script] [PASS] IVP-30 soak completed successfully\n")
                return True

        # Safety timeout (soak + 2 min margin)
        if time.time() - start > SOAK_DURATION_S + 120:
            log_and_print(logf, "[script] [WARN] Soak exceeded timeout\n")
            return False

    return False


def reconnect_port(port, logf, timeout_s=30, preserve_output=False):
    """Wait for port to reappear after watchdog reset."""
    if port.is_open:
        try:
            port.close()
        except Exception:
            pass

    log_and_print(logf, f"[script] Waiting for device to reboot (up to {timeout_s}s)...\n")
    for _ in range(timeout_s):
        try:
            port.open()
            time.sleep(1)
            if not preserve_output:
                drain(port)
            return True
        except serial.SerialException:
            time.sleep(1)
    return False


def check_watchdog_banner(port, logf, timeout_s=15):
    """Read post-reboot output and check for WATCHDOG RESET banner."""
    port.timeout = 2
    buf = ''
    start = time.time()
    while time.time() - start < timeout_s:
        chunk = port.read(4096)
        if chunk:
            text = chunk.decode('utf-8', errors='replace')
            buf += text
            log_and_print(logf, text)
            if 'WATCHDOG RESET' in buf:
                log_and_print(logf, "\n[script] [PASS] Watchdog reset banner detected\n")
                return True
    log_and_print(logf, "\n[script] [FAIL] No WATCHDOG RESET banner within timeout\n")
    return False


def run_manual_test(port, logf, key, description, expect_reset=True):
    """Send a test key and verify behavior."""
    log_and_print(logf, f"\n[script] === {description} ===\n")

    if expect_reset:
        log_and_print(logf, "[script] Device should reset within 5 seconds.\n")
    else:
        log_and_print(logf, "[script] Device should fault (red LED 3-fast+1-slow).\n")
        log_and_print(logf, "[script] Power cycle to recover.\n")

    input(f"[script] Press ENTER to send '{key}'...")
    log_and_print(logf, f"[script] Sending '{key}'...\n")

    try:
        port.write(key.encode())
    except (serial.SerialException, OSError):
        log_and_print(logf, "[script] Port already disconnected\n")

    # Read any output before disconnect/reset
    time.sleep(0.5)
    try:
        text = port.read(4096).decode('utf-8', errors='replace')
        if text:
            log_and_print(logf, text)
    except (serial.SerialException, OSError):
        pass

    if expect_reset:
        # Wait for watchdog reset, then check banner
        time.sleep(6)  # Wait for 5s watchdog timeout + margin
        if reconnect_port(port, logf, preserve_output=True):
            return check_watchdog_banner(port, logf)
        else:
            log_and_print(logf, "[script] [FAIL] Could not reconnect after reset\n")
            return False
    else:
        # Stack overflow: device faults, needs power cycle
        log_and_print(logf, "\n[script] >>> POWER CYCLE the device, then press ENTER <<<\n")
        input("[script] Press ENTER after power cycle...")
        if reconnect_port(port, logf):
            # After power cycle (not watchdog), no watchdog banner expected
            time.sleep(2)
            try:
                text = port.read(4096).decode('utf-8', errors='replace')
                if text:
                    log_and_print(logf, text)
            except (serial.SerialException, OSError):
                pass
            log_and_print(logf, "[script] [PASS] Device recovered after power cycle\n")
            return True
        else:
            log_and_print(logf, "[script] [FAIL] Could not reconnect after power cycle\n")
            return False


def summarize(logf):
    summary = """
================================================
  IVP-29/30 Test Complete
================================================
  Log saved to: {log}

  Review the log for PASS/FAIL status on:
  - IVP-29: MPU stack guard (auto + manual)
  - IVP-30: Watchdog soak + stall tests
================================================
""".format(log=LOG_FILE)
    log_and_print(logf, summary)


def main():
    args = parse_args()
    logf = open_log(LOG_FILE)

    log_and_print(logf, f"[script] Connecting to {args.port}...\n")

    try:
        port = serial.Serial(args.port, BAUD, timeout=2)
    except serial.SerialException as e:
        log_and_print(logf, f"[script] ERROR: Cannot open {args.port}: {e}\n")
        log_and_print(logf, "[script] Make sure no other serial monitor is connected.\n")
        logf.close()
        return 1

    time.sleep(0.5)
    drain(port)

    # Phase 1: Wait for IVP-29 gate check
    if not wait_for_ivp29(port, logf):
        log_and_print(logf, "[script] Continuing anyway (may already be past IVP-29)...\n")

    # Phase 2: Monitor IVP-30 soak
    soak_ok = monitor_ivp30_soak(port, logf)

    if not soak_ok:
        log_and_print(logf, "[script] Soak did not complete. Check device.\n")
        port.close()
        summarize(logf)
        logf.close()
        return 1

    # Read remaining test command prompts
    time.sleep(1)
    try:
        text = port.read(4096).decode('utf-8', errors='replace')
        if text:
            log_and_print(logf, text)
    except (serial.SerialException, OSError):
        pass

    # Phase 3: Manual tests
    log_and_print(logf, "\n[script] === MANUAL TEST PHASE ===\n")
    log_and_print(logf, "[script] Disconnect debug probe before stall tests!\n\n")

    # Test 1: Core 0 stall (watchdog)
    run_manual_test(port, logf, 'w', 'Core 0 Stall (Watchdog Test)', expect_reset=True)

    # Test 2: Core 1 stall (watchdog)
    run_manual_test(port, logf, 'W', 'Core 1 Stall (Watchdog Test)', expect_reset=True)

    # Test 3: Stack overflow (MPU fault)
    run_manual_test(port, logf, 'o', 'Core 0 Stack Overflow (MPU Fault Test)', expect_reset=False)

    port.close()
    summarize(logf)
    logf.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
