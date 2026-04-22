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
import csv
import os
import re
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

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
RE_RETRY = re.compile(r"\[CMD\]\s+Retry\s+(\d+)(?:/\d+)?\s*\(seq=(\d+)\)")
RE_FAILED = re.compile(r"\[CMD\]\s+No ACK after")


@dataclass
class CommandRecord:
    """Per-command state for IVP-T1 CSV output.
    Seq assigned from firmware log lines. sent_host_t filled in FIFO order
    as host sends match up with firmware-reported seqs (DISARM is serial)."""
    seq: int = -1
    sent_host_t: float = 0.0  # host timestamp (s since epoch)
    ack_host_t: Optional[float] = None
    ack_retry_slot: int = -1  # 0 = first-try, 1/2/3 = retry slot that ACK'd; -1 = none
    denied: bool = False
    failed: bool = False  # "No ACK after 3 retries"
    retry_count: int = 0  # max retry attempt number seen


@dataclass
class TestResult:
    sent: int = 0
    acked: int = 0
    denied: int = 0
    retries: int = 0
    failed: int = 0
    # Per-retry-slot success (IVP-T1 piggyback data; subsumes former T5)
    first_try_success: int = 0  # ACK with no prior retry
    retry_1_success: int = 0
    retry_2_success: int = 0
    retry_3_success: int = 0
    raw_lines: List[str] = field(default_factory=list)
    # Per-command records keyed by firmware seq
    by_seq: Dict[int, CommandRecord] = field(default_factory=dict)
    # FIFO queue of host send timestamps waiting to be matched to a seq
    unmatched_send_times: List[float] = field(default_factory=list)
    # Seq ordering as observed from firmware (for FIFO host-send matching)
    seq_order: List[int] = field(default_factory=list)


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

    def ensure_rec(seq: int) -> CommandRecord:
        """Get or create CommandRecord for seq. First time we see a seq,
        match it to the oldest unmatched host-send timestamp (FIFO)."""
        rec = result.by_seq.get(seq)
        if rec is None:
            rec = CommandRecord(seq=seq)
            if result.unmatched_send_times:
                rec.sent_host_t = result.unmatched_send_times.pop(0)
            result.by_seq[seq] = rec
            result.seq_order.append(seq)
        return rec

    for line_bytes in buf.splitlines():
        try:
            line = line_bytes.decode("utf-8", errors="replace").strip()
        except Exception:
            continue
        if not line:
            continue
        result.raw_lines.append(line)
        now = time.time()

        m = RE_ACK.search(line)
        if m:
            result.acked += 1
            seq = int(m.group(1))
            rec = ensure_rec(seq)
            rec.ack_host_t = now
            rec.ack_retry_slot = rec.retry_count
            if rec.retry_count == 0:
                result.first_try_success += 1
            elif rec.retry_count == 1:
                result.retry_1_success += 1
            elif rec.retry_count == 2:
                result.retry_2_success += 1
            elif rec.retry_count == 3:
                result.retry_3_success += 1
            print(f"  [ACK] {line}")
            continue
        m = RE_DENIED.search(line)
        if m:
            result.denied += 1
            seq = int(m.group(1))
            rec = ensure_rec(seq)
            rec.denied = True
            print(f"  [DENIED] {line}")
            continue
        m = RE_RETRY.search(line)
        if m:
            result.retries += 1
            attempt = int(m.group(1))
            seq = int(m.group(2))
            rec = ensure_rec(seq)
            rec.retry_count = max(rec.retry_count, attempt)
            print(f"  [RETRY] {line}")
            continue
        if RE_FAILED.search(line):
            result.failed += 1
            # Attribute to oldest pending (sent but unresolved) record.
            # Scan seq_order from oldest for one not ACK'd/denied/failed.
            for seq in result.seq_order:
                rec = result.by_seq[seq]
                if rec.ack_host_t is None and not rec.denied and not rec.failed:
                    rec.failed = True
                    break
            print(f"  [FAILED] {line}")


def run(port: str, duration_s: float, interval_s: float, inject_drops: int,
        max_commands: int, csv_path: Optional[str]):
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
        n_sent = 0
        while time.time() - start < duration_s and (
                max_commands == 0 or n_sent < max_commands):
            now = time.time()
            if now >= next_send:
                if inject_drops > 0:
                    print(f"[t={now-start:5.1f}s] REMINDER: inject "
                          f"fault_force_station_rx_drop({inject_drops}) via GDB "
                          "BEFORE next send -----")
                print(f"[t={now-start:5.1f}s] SEND DISARM")
                ser.write(b"X")
                result.sent += 1
                result.unmatched_send_times.append(now)
                n_sent += 1
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
    print(f"  first-try:   {result.first_try_success}")
    print(f"  retry 1:     {result.retry_1_success}")
    print(f"  retry 2:     {result.retry_2_success}")
    print(f"  retry 3:     {result.retry_3_success}")
    print(f"DENIED:      {result.denied}")
    print(f"Retries:     {result.retries} (total retry log lines)")
    print(f"Failed:      {result.failed}")
    if result.sent > 0:
        pct_ack = 100.0 * result.acked / result.sent
        pct_first = 100.0 * result.first_try_success / result.sent
        pct_fail = 100.0 * result.failed / result.sent
        print(f"ACK rate:       {pct_ack:.1f}%")
        print(f"First-try rate: {pct_first:.1f}%  (IVP-T1 headline metric)")
        print(f"Fail rate:      {pct_fail:.1f}%")
    print("=" * 60)

    # Save raw transcript for debug
    ts = time.strftime("%Y%m%d_%H%M%S")
    log_path = f"logs/ack_stress_{ts}.log"
    try:
        os.makedirs("logs", exist_ok=True)
        with open(log_path, "w") as f:
            for line in result.raw_lines:
                f.write(line + "\n")
        print(f"Raw transcript saved to: {log_path}")
    except Exception as e:
        print(f"(could not save transcript: {e})")

    # IVP-T1 CSV output — per-command record
    if csv_path:
        try:
            os.makedirs(os.path.dirname(csv_path) or ".", exist_ok=True)
            with open(csv_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow([
                    "seq", "sent_host_t", "ack_host_t",
                    "ack_retry_slot", "retry_count",
                    "denied", "failed",
                ])
                for seq in result.seq_order:
                    rec = result.by_seq[seq]
                    w.writerow([
                        rec.seq,
                        f"{rec.sent_host_t:.3f}" if rec.sent_host_t else "",
                        f"{rec.ack_host_t:.3f}" if rec.ack_host_t else "",
                        rec.ack_retry_slot,
                        rec.retry_count,
                        int(rec.denied),
                        int(rec.failed),
                    ])
            print(f"CSV saved to: {csv_path}")
        except Exception as e:
            print(f"(could not save CSV: {e})")


def main():
    ap = argparse.ArgumentParser(description="ACK stress test (IVP-132a.5 + Stage T)")
    ap.add_argument("--port", default="COM7", help="Station serial port (default COM7)")
    ap.add_argument("--duration", type=float, default=300.0,
                    help="Test duration in seconds (default 300 = 5 min). "
                         "Set very high when using --count to cap by count.")
    ap.add_argument("--interval", type=float, default=10.0,
                    help="Send interval in seconds (default 10 = 0.1 Hz; "
                         "Stage T IVP-T1 uses 1.0 = 1 Hz)")
    ap.add_argument("--count", type=int, default=0,
                    help="Max commands to send (0 = run to --duration). "
                         "Stage T IVP-T1 uses 100.")
    ap.add_argument("--csv", default=None,
                    help="Write per-command CSV to this path (IVP-T1 output).")
    ap.add_argument("--inject-drops", type=int, default=0,
                    help="N for fault_force_station_rx_drop(N) reminder "
                         "per send (requires probe + manual GDB)")
    args = ap.parse_args()
    run(args.port, args.duration, args.interval, args.inject_drops,
        args.count, args.csv)


if __name__ == "__main__":
    main()
