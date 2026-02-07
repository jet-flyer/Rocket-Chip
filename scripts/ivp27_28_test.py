#!/usr/bin/env python3
"""
IVP-27/28 Soak & Flash Test Monitor

Monitors USB serial during the 10-minute IVP-27 soak, prompts the user
for manual gate actions at the right times, then captures IVP-28 flash
test output.  Saves a complete log to ivp27_28_log.txt.

Usage:
    python scripts/ivp27_28_test.py [--port COM6]

The device should already be running with IVP-27 soak active.
If the soak hasn't started yet (fresh boot), the script waits for it.
"""

import serial
import time
import sys
import argparse
import os
import re
from datetime import datetime, timedelta

PORT = 'COM6'
BAUD = 115200
LOG_FILE = 'ivp27_28_log.txt'
SOAK_DURATION_S = 10 * 60  # 10 minutes


def parse_args():
    parser = argparse.ArgumentParser(description='IVP-27/28 soak monitor')
    parser.add_argument('--port', default=PORT, help='Serial port (default COM6)')
    return parser.parse_args()


def open_log(path):
    f = open(path, 'w', encoding='utf-8')
    f.write(f"# IVP-27/28 Test Log — {datetime.now().isoformat()}\n\n")
    return f


def log_and_print(logf, line):
    """Print to console and write to log file."""
    print(line, end='')
    logf.write(line)
    logf.flush()


def drain(port):
    """Read and discard any pending data."""
    port.timeout = 0.1
    while port.read(4096):
        pass
    port.timeout = 1


def read_until_quiet(port, timeout_s=2):
    """Read until no data for timeout_s seconds."""
    buf = b''
    port.timeout = timeout_s
    while True:
        chunk = port.read(4096)
        if not chunk:
            break
        buf += chunk
    return buf.decode('utf-8', errors='replace')


def wait_for_ivp27_start(port, logf):
    """Wait until we see the IVP-27 banner or status line."""
    log_and_print(logf, "[script] Waiting for IVP-27 soak to start...\n")
    port.timeout = 2
    buf = ''
    while True:
        chunk = port.read(4096)
        if chunk:
            text = chunk.decode('utf-8', errors='replace')
            buf += text
            log_and_print(logf, text)
            if 'IVP-27: USB Stability Soak' in buf or '[IVP-27]' in buf:
                return True
        # Don't spin too tight
        time.sleep(0.1)


def cli_gate_test(port, logf):
    """Send 'h' and check response — IVP-27 Gate 2."""
    log_and_print(logf, "\n[script] === GATE 2: CLI Responsiveness ===\n")
    log_and_print(logf, "[script] Sending 'h' command...\n")
    drain(port)
    t0 = time.time()
    port.write(b'h')
    # Read response with 2s timeout
    port.timeout = 2
    resp = b''
    while time.time() - t0 < 2:
        chunk = port.read(4096)
        if chunk:
            resp += chunk
            if b'RocketChip' in resp or b'help' in resp.lower():
                break
    elapsed = time.time() - t0
    text = resp.decode('utf-8', errors='replace')
    log_and_print(logf, text)

    if 'RocketChip' in text or 'help' in text.lower():
        log_and_print(logf, f"[script] [PASS] Gate 2: CLI responded in {elapsed:.2f}s\n")
        return True
    else:
        log_and_print(logf, f"[script] [FAIL] Gate 2: No CLI response after {elapsed:.2f}s\n")
        return False


def disconnect_gate(port, logf):
    """Guide user through IVP-27 Gate 3 — disconnect/reconnect."""
    log_and_print(logf, "\n[script] === GATE 3: USB Disconnect/Reconnect ===\n")
    log_and_print(logf, "[script] >>> DISCONNECT the USB cable NOW. <<<\n")
    log_and_print(logf, "[script] Waiting 60 seconds...\n")

    # Close the port so Windows releases it
    if port.is_open:
        try:
            port.close()
        except Exception:
            pass

    for remaining in range(60, 0, -10):
        log_and_print(logf, f"[script]   {remaining}s remaining...\n")
        time.sleep(10)

    log_and_print(logf, "[script] >>> RECONNECT the USB cable NOW. <<<\n")

    if reconnect_port(port, logf):
        # Read whatever output resumed
        port.timeout = 3
        try:
            text = port.read(8192).decode('utf-8', errors='replace')
        except (serial.SerialException, OSError):
            text = ''
        log_and_print(logf, text)
        if len(text) > 10:
            log_and_print(logf, "[script] [PASS] Gate 3: Output resumed after reconnect\n")
            return True
        else:
            log_and_print(logf, "[script] [FAIL] Gate 3: No output after reconnect\n")
            return False
    else:
        log_and_print(logf, "[script] [FAIL] Gate 3: Could not reopen port after 30s\n")
        return False


def keymash_gate(port, logf):
    """IVP-27 Gate 4 — rapid key mash for 10 seconds."""
    log_and_print(logf, "\n[script] === GATE 4: Key Mash Stress ===\n")
    log_and_print(logf, "[script] Sending rapid keystrokes for 10 seconds...\n")
    drain(port)

    t0 = time.time()
    keys_sent = 0
    while time.time() - t0 < 10:
        # Send a mix of valid and invalid keys
        port.write(b'hhhssscccxxx\r\n')
        keys_sent += 14
        time.sleep(0.05)  # ~200 keys/sec

    log_and_print(logf, f"[script] Sent {keys_sent} keystrokes in 10s\n")
    time.sleep(1)

    # Check device still responds
    drain(port)
    port.write(b'h')
    port.timeout = 2
    resp = port.read(4096).decode('utf-8', errors='replace')
    log_and_print(logf, resp[:500] if len(resp) > 500 else resp)

    if 'RocketChip' in resp or 'help' in resp.lower():
        log_and_print(logf, "[script] [PASS] Gate 4: Device survived key mash, CLI responds\n")
        return True
    else:
        log_and_print(logf, "[script] [FAIL] Gate 4: Device unresponsive after key mash\n")
        return False


def reconnect_port(port, logf, timeout_s=30):
    """Close port if open, wait for reconnect, reopen."""
    if port.is_open:
        port.close()

    log_and_print(logf, f"[script] Waiting for port to reappear (up to {timeout_s}s)...\n")
    for _ in range(timeout_s):
        try:
            port.open()
            time.sleep(1)
            drain(port)
            return True
        except serial.SerialException:
            time.sleep(1)
    return False


def monitor_soak(port, logf):
    """Monitor the IVP-27 soak, running manual gates at appropriate times."""
    log_and_print(logf, "\n[script] Monitoring 10-minute soak...\n")
    log_and_print(logf, f"[script] Estimated completion: {(datetime.now() + timedelta(seconds=SOAK_DURATION_S)).strftime('%H:%M:%S')}\n\n")

    gate2_done = False
    gate3_done = False
    gate4_done = False
    soak_complete = False
    start_time = time.time()

    port.timeout = 5

    while not soak_complete:
        # Read any available output — handle USB disconnect gracefully
        try:
            chunk = port.read(4096)
        except (serial.SerialException, OSError):
            # USB cable was physically disconnected
            elapsed = time.time() - start_time
            log_and_print(logf, "\n[script] USB disconnected detected!\n")

            if not gate3_done:
                # User disconnected for Gate 3 — run the gate flow
                gate3_done = True
                log_and_print(logf, "\n[script] === GATE 3: USB Disconnect/Reconnect ===\n")
                log_and_print(logf, "[script] Disconnect detected. Waiting 60 seconds...\n")
                if port.is_open:
                    try:
                        port.close()
                    except Exception:
                        pass

                for remaining in range(60, 0, -10):
                    log_and_print(logf, f"[script]   {remaining}s remaining...\n")
                    time.sleep(10)

                log_and_print(logf, "[script] >>> RECONNECT the USB cable NOW. <<<\n")
                if reconnect_port(port, logf):
                    port.timeout = 3
                    try:
                        text = port.read(8192).decode('utf-8', errors='replace')
                        log_and_print(logf, text)
                        if len(text) > 10:
                            log_and_print(logf, "[script] [PASS] Gate 3: Output resumed after reconnect\n")
                        else:
                            log_and_print(logf, "[script] [FAIL] Gate 3: No output after reconnect\n")
                    except (serial.SerialException, OSError):
                        log_and_print(logf, "[script] [FAIL] Gate 3: Read failed after reconnect\n")
                    port.timeout = 5
                else:
                    log_and_print(logf, "[script] [FAIL] Gate 3: Could not reopen port\n")
                    return False
            else:
                # Unexpected disconnect — try to reconnect
                log_and_print(logf, "[script] Unexpected disconnect. Trying to reconnect...\n")
                if port.is_open:
                    try:
                        port.close()
                    except Exception:
                        pass
                log_and_print(logf, "[script] >>> RECONNECT the USB cable. <<<\n")
                if not reconnect_port(port, logf):
                    log_and_print(logf, "[script] [FAIL] Could not reconnect\n")
                    return False
                port.timeout = 5
            continue

        if chunk:
            text = chunk.decode('utf-8', errors='replace')
            log_and_print(logf, text)

            # Check for soak completion
            if 'IVP-27:' in text and 'AUTOMATED GATES' in text:
                soak_complete = True
                break

        elapsed = time.time() - start_time

        # At ~1 minute: test CLI (Gate 2)
        if not gate2_done and elapsed >= 60:
            gate2_done = True
            cli_gate_test(port, logf)

        # At ~3.5 minutes: disconnect gate (Gate 3) if user hasn't already
        if not gate3_done and elapsed >= 3.5 * 60:
            gate3_done = True
            disconnect_gate(port, logf)

        # At ~8 minutes: key mash (Gate 4)
        if not gate4_done and elapsed >= 8 * 60:
            gate4_done = True
            keymash_gate(port, logf)

        # Safety timeout — soak should be done in ~11 minutes max
        if elapsed > 12 * 60:
            log_and_print(logf, "[script] [WARN] Soak exceeded 12 minutes — stopping monitor\n")
            break

    return soak_complete


def capture_ivp28(port, logf):
    """Capture IVP-28 flash test output (runs automatically after IVP-27)."""
    log_and_print(logf, "\n[script] Waiting for IVP-28 flash test output...\n")

    port.timeout = 10
    buf = ''
    t0 = time.time()

    while time.time() - t0 < 30:
        chunk = port.read(4096)
        if chunk:
            text = chunk.decode('utf-8', errors='replace')
            buf += text
            log_and_print(logf, text)

            # IVP-28 ends with this line
            if 'IVP-28:' in buf and 'AUTOMATED GATES' in buf:
                return True

    log_and_print(logf, "[script] [WARN] IVP-28 output not captured within 30s\n")
    return False


def summarize(logf):
    """Print final summary."""
    summary = """
================================================
  IVP-27/28 Test Complete
================================================
  Log saved to: {log}

  MANUAL CHECK REMAINING:
  - Gate 6: Power cycle the device, reconnect,
    and verify calibration data persists.
    (Run: python scripts/cli_test.py status)
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

    # Wait for IVP-27 to start (or pick up mid-soak)
    wait_for_ivp27_start(port, logf)

    # Monitor the soak and run manual gates
    soak_ok = monitor_soak(port, logf)

    if soak_ok:
        # IVP-28 runs automatically after IVP-27
        capture_ivp28(port, logf)

    port.close()
    summarize(logf)
    logf.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
