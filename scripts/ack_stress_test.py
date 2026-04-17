#!/usr/bin/env python3
"""
IVP-132a.5 Cross-board ACK protocol stress test.

Station-side harness: sends DISARM commands ('X') at 0.1 Hz for 5 minutes
(30 commands total), captures ACK / retry / failure counts, computes
success rate.

DISARM is used instead of ARM because:
  - Single keystroke (ARM requires multi-char confirmation flow)
  - Same tracked-command path (same ACK logic)
  - Safe to repeat — even if vehicle is in an unexpected state, DISARM
    just sets/keeps the armed flag = false

Setup:
  - Vehicle: powered on (USB or battery), firmware running, radio TX slot open
  - Station: powered on, USB serial exposed on COM7 (or whichever port),
    station flight or bench binary OK
  - Python host: runs this script, talks to station's USB CDC

Known issue (AGENT_WHITEBOARD "RadioScheduler Sync"): station TX timing is
not synchronized to vehicle RX windows, so ACKs are probabilistic.
This test's purpose is to *measure* the failure rate, not to pass a
specific threshold.

Optional fault injection (requires debug probe on station):
  --inject-drops N    call fault_force_station_rx_drop(N) before each command
                      to simulate RF fade during ACK receive.

Usage:
  # Basic 5-min, 0.1Hz, no fault injection
  python scripts/ack_stress_test.py --port COM7

  # 2-min accelerated
  python scripts/ack_stress_test.py --port COM7 --duration 120 --interval 2

  # With RX-drop fault injection (probe must be on station)
  python scripts/ack_stress_test.py --port COM7 --inject-drops 1

WARNING: serial port handling — see
  C:/Users/pow-w/.claude/projects/.../memory/feedback_serial_port.md
  Ctrl+C cleanly; if COM7 gets stuck, reset station via GDB.
"""

import argparse
import re
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional

try:
    import serial
except ImportError:
    print("pyserial required: pip install pyserial", file=sys.stderr)
    sys.exit(1)


# Firmware log lines this script watches for.
# Format lines from src/active_objects/ao_telemetry.cpp + rc_os_commands.cpp:
#   "[CMD] DISARM sent, waiting for ACK..."  (on send)
#   "[CMD] ACK'd (seq=N)"                    (on match)
#   "[CMD] DENIED (seq=N)"                   (on reject)
#   "[CMD] retry <attempt> of 3 (seq=<seq>)" (on timeout retry — IVP-122)
#   "[CMD] FAILED after retries (seq=<seq>)" (on final fail — IVP-122)
# Firmware strings (exact, from src/active_objects/ao_telemetry.cpp):
#   "[CMD] DISARM sent, waiting for ACK..."
#   "[CMD] ACK'd (seq=N)"
#   "[CMD] DENIED (seq=N)"
#   "[CMD] Retry N (seq=N)"
#   "[CMD] No ACK after 3 retries"
RE_SENT = re.compile(r"\[CMD\]\s+DISARM sent")
RE_ACK = re.compile(r"\[CMD\]\s+ACK'd\s*\(seq=(\d+)\)")
RE_DENIED = re.compile(r"\[CMD\]\s+DENIED\s*\(seq=(\d+)\)")
RE_RETRY = re.compile(r"\[CMD\]\s+Retry\s+(\d+)\s*\(seq=(\d+)\)")
RE_FAILED = re.compile(r"\[CMD\]\s+No ACK after")


@dataclass
class TestResult:
    sent: int = 0
    acked: int = 0
    denied: int = 0
    retries: int = 0
    failed: int = 0
    raw_lines: List[str] = field(default_factory=list)


def drain_and_classify(ser, result: TestResult, timeout_s: float = 0.2):
    """Read available bytes from station, classify any [CMD] lines."""
    end = time.time() + timeout_s
    buf = b""
    while time.time() < end:
        chunk = ser.read(256)
        if not chunk:
            time.sleep(0.01)
            continue
        buf += chunk
        if b"\n" in buf:
            break

    for line_bytes in buf.splitlines():
        try:
            line = line_bytes.decode("utf-8", errors="replace").strip()
        except Exception:
            continue
        if not line:
            continue
        result.raw_lines.append(line)
        if RE_ACK.search(line):
            result.acked += 1
            print(f"  [ACK] {line}")
        elif RE_DENIED.search(line):
            result.denied += 1
            print(f"  [DENIED] {line}")
        elif RE_RETRY.search(line):
            result.retries += 1
            print(f"  [RETRY] {line}")
        elif RE_FAILED.search(line):
            result.failed += 1
            print(f"  [FAILED] {line}")


def run(port: str, duration_s: float, interval_s: float, inject_drops: int):
    if inject_drops > 0:
        print("NOTE: --inject-drops requires the debug probe on the station + "
              "OpenOCD running. This script does NOT drive GDB automatically; "
              "it assumes you've wired fault_force_station_rx_drop via another "
              "terminal. This iteration: prints a reminder before each send.")

    ser = serial.Serial(port, 115200, timeout=0.1)
    time.sleep(0.2)
    ser.reset_input_buffer()

    print(f"IVP-132a.5 ACK stress test — port={port} duration={duration_s}s "
          f"interval={interval_s}s inject_drops={inject_drops}")
    print("-" * 60)

    result = TestResult()
    start = time.time()
    next_send = start

    try:
        while time.time() - start < duration_s:
            now = time.time()
            if now >= next_send:
                if inject_drops > 0:
                    print(f"[t={now-start:5.1f}s] REMINDER: inject "
                          f"fault_force_station_rx_drop({inject_drops}) via GDB "
                          "BEFORE next send -----")
                print(f"[t={now-start:5.1f}s] SEND DISARM")
                ser.write(b"X")
                result.sent += 1
                next_send += interval_s
                # Give firmware a chance to process before draining output
                time.sleep(0.1)
            drain_and_classify(ser, result, timeout_s=0.3)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        # Final drain to catch trailing ACKs
        time.sleep(1.0)
        drain_and_classify(ser, result, timeout_s=1.0)
        ser.close()

    elapsed = time.time() - start
    print("\n" + "=" * 60)
    print("RESULTS")
    print("=" * 60)
    print(f"Duration:    {elapsed:.1f} s")
    print(f"Sent:        {result.sent}")
    print(f"ACK'd:       {result.acked}")
    print(f"DENIED:      {result.denied}")
    print(f"Retries:     {result.retries}")
    print(f"Failed:      {result.failed}")
    if result.sent > 0:
        pct_ack = 100.0 * result.acked / result.sent
        pct_fail = 100.0 * result.failed / result.sent
        print(f"ACK rate:    {pct_ack:.1f}%")
        print(f"Fail rate:   {pct_fail:.1f}%")
    print("=" * 60)

    # Save raw transcript for debug
    ts = time.strftime("%Y%m%d_%H%M%S")
    log_path = f"logs/ack_stress_{ts}.log"
    try:
        with open(log_path, "w") as f:
            for line in result.raw_lines:
                f.write(line + "\n")
        print(f"Raw transcript saved to: {log_path}")
    except Exception as e:
        print(f"(could not save transcript: {e})")


def main():
    ap = argparse.ArgumentParser(description="IVP-132a.5 ACK stress test")
    ap.add_argument("--port", default="COM7", help="Station serial port (default COM7)")
    ap.add_argument("--duration", type=float, default=300.0,
                    help="Test duration in seconds (default 300 = 5 min)")
    ap.add_argument("--interval", type=float, default=10.0,
                    help="Send interval in seconds (default 10 = 0.1 Hz)")
    ap.add_argument("--inject-drops", type=int, default=0,
                    help="N for fault_force_station_rx_drop(N) reminder "
                         "per send (requires probe + manual GDB)")
    args = ap.parse_args()
    run(args.port, args.duration, args.interval, args.inject_drops)


if __name__ == "__main__":
    main()
