#!/usr/bin/env python3
"""
Stage T Batch B IVP-T14d wrap: Wilson 95% CI on first-try DISARM ACK rate.

Uses the T14b firmware-side retry counters (`[CmdRetryStats]` block from
`q -> d` diag dump) to get deterministic counts, avoiding the serial-
scrape race that `ack_stress_test.py` has under rapid retry output.

Protocol:
  1. Cycle station to kMenu, run `q -> d` diag dump -> capture baseline
     DISARM stats (sent / 1st / retry_rescued / fail).
  2. Return to kAnsi dashboard.
  3. Send N DISARM keypresses spaced by --interval seconds.
  4. Wait --settle seconds for final retry windows to drain.
  5. Re-enter kMenu, run `q -> d` again -> capture post stats.
  6. Diff the two snapshots to get counts for THIS test window.
  7. Compute Wilson 95% CI on first-try / total.

Usage:
  python scripts/stage_t_wilson_ci.py --port COM9 --n 100 --interval 1 \
    --csv logs/stage_t/t14_wilson.csv
"""
import argparse
import math
import re
import sys
import time
from dataclasses import dataclass

import serial


# Line format in [CmdRetryStats] block:
#   class    sent   1st  retry   fail retries
#   DISARM   123    45    78      0    234
RE_DISARM_LINE = re.compile(
    r"\s*DISARM\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)"
)


@dataclass
class Stats:
    sent: int = 0
    first_try: int = 0
    retry_rescued: int = 0
    failed: int = 0
    total_retries: int = 0


def drain(ser, seconds=0.5):
    t0 = time.time()
    while time.time() - t0 < seconds:
        n = ser.in_waiting
        if n:
            ser.read(n)
        time.sleep(0.05)


def send_and_wait(ser, data: bytes, wait_s: float):
    ser.write(data)
    ser.flush()
    time.sleep(wait_s)


def capture_disarm_stats(ser) -> Stats:
    """Navigate to kMenu -> q -> d, read diag dump, extract DISARM row.
    Assumes station is already in kMenu (or kAnsi and a prior 'x' got us
    to kMenu). Caller's responsibility to ensure mode.
    """
    drain(ser)
    # Enter debug submenu (harmless if already in debug)
    send_and_wait(ser, b"q", 0.5)
    drain(ser)
    # Trigger diag dump
    ser.write(b"d")
    ser.flush()
    # Diag dump is multi-KB; read for a few seconds
    buf = b""
    t0 = time.time()
    while time.time() - t0 < 4.0:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
        time.sleep(0.05)
    # Exit debug submenu back to main via ESC (any unmatched key in
    # debug menu exits; ESC is neutral in main menu too). Avoids the
    # lowercase-x = erase_flights and uppercase-X = DISARM footguns.
    send_and_wait(ser, b"\x1b", 0.3)
    drain(ser)

    text = buf.decode("utf-8", errors="replace")
    stats = Stats()
    for line in text.splitlines():
        m = RE_DISARM_LINE.search(line)
        if m:
            stats.sent = int(m.group(1))
            stats.first_try = int(m.group(2))
            stats.retry_rescued = int(m.group(3))
            stats.failed = int(m.group(4))
            stats.total_retries = int(m.group(5))
            return stats
    print("WARNING: could not parse DISARM stats from diag dump.", file=sys.stderr)
    print("Raw output tail:", file=sys.stderr)
    print(text[-600:], file=sys.stderr)
    return stats


def wilson_ci(k: int, n: int, z: float = 1.96) -> tuple[float, float, float]:
    """Wilson score interval; returns (point, lo, hi) as fractions."""
    if n == 0:
        return (0.0, 0.0, 0.0)
    p = k / n
    denom = 1 + z * z / n
    center = (p + z * z / (2 * n)) / denom
    margin = (z / denom) * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return (p, max(0.0, center - margin), min(1.0, center + margin))


def back_to_dashboard(ser):
    """Cycle back to kAnsi from kMenu."""
    drain(ser)
    send_and_wait(ser, b"m", 0.5)
    drain(ser)


def ensure_kmenu(ser):
    """Put station in kMenu if it's in kAnsi; neutral if already in kMenu.
    Detects mode by probing: send ESC (harmless in both modes), then send
    'h' and read the response. If response contains '--- Help ---' (main
    menu help), we're in kMenu. If the response is streaming dashboard
    frames, we need to send 'x' to exit.
    """
    drain(ser, 0.3)
    ser.write(b"h")
    ser.flush()
    time.sleep(0.8)
    buf = b""
    while ser.in_waiting:
        buf += ser.read(ser.in_waiting)
        time.sleep(0.05)
    text = buf.decode("utf-8", errors="replace")
    if "--- Help ---" in text or "[main]" in text:
        return  # Already in kMenu
    # Must be in kAnsi; 'x' exits to kMenu.
    ser.write(b"x")
    ser.flush()
    time.sleep(1.0)
    drain(ser, 0.5)


def run(port: str, n: int, interval: float, settle: float, csv_path: str | None):
    print(f"Stage T Wilson CI — port={port} n={n} interval={interval}s "
          f"settle={settle}s")
    with serial.Serial(port, 115200, timeout=0.5) as ser:
        time.sleep(2.0)
        drain(ser, 1.0)

        print("\n[1/4] Ensuring kMenu mode...")
        ensure_kmenu(ser)

        print("\n[1/3] Capturing baseline DISARM stats...")
        baseline = capture_disarm_stats(ser)
        print(f"  baseline: sent={baseline.sent} 1st={baseline.first_try} "
              f"retry={baseline.retry_rescued} fail={baseline.failed}")

        # Stay in kMenu for the command burst — in kMenu, 'X' is the
        # tracked-DISARM keystroke (kAnsi 'D' would work too but requires
        # a mode cycle that can race with dashboard redraw).
        print(f"\n[2/3] Sending {n} DISARM commands (kMenu 'X') at {interval}s interval...")
        t_start = time.time()
        for i in range(n):
            ser.write(b"X")
            ser.flush()
            now = time.time()
            elapsed = now - t_start
            next_send = t_start + (i + 1) * interval
            # Continuously drain serial input during the wait so CDC buffer
            # on the host doesn't back-pressure our writes. Station emits
            # [CMD] lines + (if in kAnsi) dashboard frames; in kMenu this
            # is mostly [CMD] output from retry ticks.
            while time.time() < next_send:
                pending = ser.in_waiting
                if pending:
                    ser.read(pending)
                time.sleep(0.05)
            if (i + 1) % 10 == 0 or i == 0:
                print(f"  [{elapsed:6.1f}s] sent {i + 1}/{n}")
        print(f"  all {n} sent in {time.time() - t_start:.1f}s")

        print(f"\n  settle: waiting {settle}s for final retry windows...")
        time.sleep(settle)

        print("\n[3/3] Capturing post-test DISARM stats...")
        post = capture_disarm_stats(ser)
        print(f"  post:     sent={post.sent} 1st={post.first_try} "
              f"retry={post.retry_rescued} fail={post.failed}")

    # Diff
    d_sent          = post.sent - baseline.sent
    d_first_try     = post.first_try - baseline.first_try
    d_retry_rescued = post.retry_rescued - baseline.retry_rescued
    d_failed        = post.failed - baseline.failed
    d_retries       = post.total_retries - baseline.total_retries

    print("\n" + "=" * 62)
    print("RESULTS (this test window only)")
    print("=" * 62)
    print(f"  Host sends:         {n}")
    print(f"  Firmware sent:      {d_sent}")
    print(f"  First-try ACK:      {d_first_try}")
    print(f"  Retry-rescued ACK:  {d_retry_rescued}")
    print(f"  Failed (all retries exhausted): {d_failed}")
    print(f"  Total retries used: {d_retries}")

    if d_sent != n:
        print(f"\n  WARNING: firmware-observed sends ({d_sent}) != host sends ({n}). "
              f"Dedupe or dropped keystrokes possible.")

    n_resolved = d_first_try + d_retry_rescued + d_failed
    if n_resolved == 0:
        print("\n  No commands resolved — cannot compute CI.")
        return

    p_first, lo, hi = wilson_ci(d_first_try, n_resolved)
    p_any = (d_first_try + d_retry_rescued) / n_resolved

    print(f"\n  First-try ACK rate: {p_first*100:.1f}%  "
          f"Wilson 95% CI [{lo*100:.2f}%, {hi*100:.2f}%]  (gate: >=95%)")
    print(f"  Eventually-ACK'd:   {p_any*100:.1f}%  "
          f"({d_first_try + d_retry_rescued}/{n_resolved})")
    print(f"  Failed (no ACK):    {d_failed/n_resolved*100:.1f}%  "
          f"({d_failed}/{n_resolved})")
    print("=" * 62)

    if csv_path:
        with open(csv_path, "w") as f:
            f.write("metric,value\n")
            f.write(f"n,{n}\n")
            f.write(f"fw_sent,{d_sent}\n")
            f.write(f"first_try,{d_first_try}\n")
            f.write(f"retry_rescued,{d_retry_rescued}\n")
            f.write(f"failed,{d_failed}\n")
            f.write(f"total_retries_used,{d_retries}\n")
            f.write(f"first_try_rate,{p_first:.4f}\n")
            f.write(f"wilson_lo,{lo:.4f}\n")
            f.write(f"wilson_hi,{hi:.4f}\n")
            f.write(f"eventually_ack_rate,{p_any:.4f}\n")
        print(f"\n  CSV saved: {csv_path}")


def main():
    ap = argparse.ArgumentParser(description="Stage T Wilson CI for DISARM first-try rate")
    ap.add_argument("--port", default="COM9", help="Station COM port (default COM9)")
    ap.add_argument("--n", type=int, default=100, help="Number of DISARM commands (default 100)")
    ap.add_argument("--interval", type=float, default=1.0,
                    help="Seconds between sends (default 1.0)")
    ap.add_argument("--settle", type=float, default=5.0,
                    help="Seconds to wait after last send before final stats capture (default 5)")
    ap.add_argument("--csv", default=None, help="Write summary CSV to this path")
    args = ap.parse_args()
    run(args.port, args.n, args.interval, args.settle, args.csv)


if __name__ == "__main__":
    main()
