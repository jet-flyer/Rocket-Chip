#!/usr/bin/env python3
"""Stage T IVP-T2: cheat-mode sync ceiling (HOST-SIDE implementation).

Instead of modifying firmware (which proved unstable in practice), the host
harness watches vehicle serial for `[STAGE_T] state TX->RXW` transitions
and sends `X` on the station serial within a tight window after seeing one.

This puts the station TX reliably inside the vehicle's kRxWindow. The
success rate measured by this harness is the UPPER BOUND on what any
sync-based fix could achieve.

Station = COM9 (known-good firmware, unchanged)
Vehicle = COM7 (Stage T logging build, emits STAGE_T state transitions)

Requires vehicle built with -DROCKETCHIP_STAGE_T_LOGGING=ON.

Usage:
    python scripts/stage_t2_cheat.py --count 100 [--delay-ms 10]

Arguments:
    --delay-ms N   fire X this many ms after seeing TX->RXW (default 10)
                   Vehicle kRxWindow is ~100ms; firing 10-50ms in gives
                   plenty of margin while not arriving before RX is ready.
"""
import argparse
import csv
import os
import re
import sys
import threading
import time
from collections import deque

try:
    import serial
except ImportError:
    print("pyserial required", file=sys.stderr)
    sys.exit(1)


RE_TX_RXW = re.compile(r"\[STAGE_T\] state TX->RXW t=(\d+)")
RE_ACK = re.compile(r"\[CMD\]\s+ACK'd\s*\(seq=(\d+)\)")
RE_RETRY = re.compile(r"\[CMD\]\s+Retry\s+(\d+)\s*\(seq=(\d+)\)")
RE_FAILED = re.compile(r"\[CMD\]\s+No ACK after")
RE_DENIED = re.compile(r"\[CMD\]\s+DENIED\s*\(seq=(\d+)\)")


class Shared:
    def __init__(self):
        self.last_rxw_entry_host_t = 0.0  # host wall-clock (s)
        self.last_rxw_entry_us = 0  # firmware us timestamp
        self.stop = threading.Event()


def tail_vehicle(port, shared, log_f):
    """Thread: tail vehicle serial, update last_rxw_entry_host_t on TX->RXW."""
    ser = serial.Serial(port, 115200, timeout=0.1)
    time.sleep(0.2)
    ser.reset_input_buffer()
    buf = b""
    while not shared.stop.is_set():
        try:
            chunk = ser.read(512)
        except Exception:
            break
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line_b, buf = buf.split(b"\n", 1)
            try:
                line = line_b.decode("utf-8", errors="replace").rstrip("\r")
            except Exception:
                continue
            now = time.time()
            log_f.write(f"{now:.6f} {line}\n")
            m = RE_TX_RXW.search(line)
            if m:
                shared.last_rxw_entry_host_t = now
                shared.last_rxw_entry_us = int(m.group(1))
    try:
        ser.close()
    except Exception:
        pass


def run(count, delay_ms, cmd_timeout_s, station_port, vehicle_port, csv_path):
    os.makedirs("logs/stage_t", exist_ok=True)
    vehicle_log_path = os.path.splitext(csv_path)[0] + "_vehicle.log"
    station_log_path = os.path.splitext(csv_path)[0] + ".log"

    shared = Shared()
    vehicle_log_f = open(vehicle_log_path, "w", encoding="utf-8")
    veh_thread = threading.Thread(
        target=tail_vehicle, args=(vehicle_port, shared, vehicle_log_f),
        daemon=True)
    veh_thread.start()
    time.sleep(1.5)  # let vehicle tail get a few TX->RXW samples

    # Open station
    st = serial.Serial(station_port, 115200, timeout=0.1)
    time.sleep(0.3)
    st.reset_input_buffer()

    station_log_f = open(station_log_path, "w", encoding="utf-8")
    delay_s = delay_ms / 1000.0

    # Per-command records
    records = {}  # seq -> dict
    seq_order = []  # order seqs observed
    unmatched_send_times = deque()  # FIFO of host send ts

    stats = {
        'sent': 0, 'acked': 0, 'first_try': 0, 'r1': 0, 'r2': 0, 'r3': 0,
        'denied': 0, 'failed': 0,
    }

    def ensure_rec(seq):
        if seq not in records:
            records[seq] = {
                'seq': seq, 'sent_host_t': 0.0, 'ack_host_t': None,
                'ack_retry_slot': -1, 'retry_count': 0,
                'denied': 0, 'failed': 0,
                'send_after_rxw_ms': -1,
            }
            seq_order.append(seq)
            if unmatched_send_times:
                rec = records[seq]
                rec['sent_host_t'], rec['send_after_rxw_ms'] = \
                    unmatched_send_times.popleft()
        return records[seq]

    def drain_station(timeout_s=0.1):
        end = time.time() + timeout_s
        sbuf = b""
        while time.time() < end:
            try:
                chunk = st.read(256)
            except Exception:
                break
            if not chunk:
                continue
            sbuf += chunk
            if b"\n" in sbuf:
                break
        now = time.time()
        for line_b in sbuf.splitlines():
            line = line_b.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            station_log_f.write(line + "\n")
            m = RE_ACK.search(line)
            if m:
                seq = int(m.group(1))
                rec = ensure_rec(seq)
                rec['ack_host_t'] = now
                rec['ack_retry_slot'] = rec['retry_count']
                stats['acked'] += 1
                if rec['retry_count'] == 0:
                    stats['first_try'] += 1
                elif rec['retry_count'] == 1:
                    stats['r1'] += 1
                elif rec['retry_count'] == 2:
                    stats['r2'] += 1
                elif rec['retry_count'] == 3:
                    stats['r3'] += 1
                print(f"  [ACK]    {line}")
                continue
            m = RE_DENIED.search(line)
            if m:
                seq = int(m.group(1))
                rec = ensure_rec(seq)
                rec['denied'] = 1
                stats['denied'] += 1
                print(f"  [DENIED] {line}")
                continue
            m = RE_RETRY.search(line)
            if m:
                attempt = int(m.group(1))
                seq = int(m.group(2))
                rec = ensure_rec(seq)
                rec['retry_count'] = max(rec['retry_count'], attempt)
                print(f"  [RETRY]  {line}")
                continue
            if RE_FAILED.search(line):
                stats['failed'] += 1
                # Attribute to oldest unresolved
                for s in seq_order:
                    r = records[s]
                    if r['ack_host_t'] is None and not r['denied'] and not r['failed']:
                        r['failed'] = 1
                        break
                print(f"  [FAIL]   {line}")

    start = time.time()
    print(f"=== Stage T IVP-T2 cheat-mode (host) ===")
    print(f"  count={count}  delay_after_RXW={delay_ms}ms")
    print(f"  station={station_port}  vehicle={vehicle_port}")
    print(f"  CSV: {csv_path}")
    print(f"  vehicle log: {vehicle_log_path}")
    print(f"  station log: {station_log_path}")
    print()

    try:
        for i in range(count):
            # Wait for the next TX->RXW transition on vehicle (fresh event)
            pre = shared.last_rxw_entry_host_t
            wait_start = time.time()
            while shared.last_rxw_entry_host_t == pre:
                if time.time() - wait_start > 2.0:
                    print(f"  [WARN] no TX->RXW in 2s — vehicle STAGE_T log?")
                    break
                # Meanwhile drain station for any late ACKs from prior sends
                drain_station(timeout_s=0.02)
                time.sleep(0.005)

            # Fire delay_ms after observed TX->RXW transition
            target = shared.last_rxw_entry_host_t + delay_s
            while time.time() < target:
                time.sleep(0.001)

            send_t = time.time()
            actual_delay_ms = (send_t - shared.last_rxw_entry_host_t) * 1000.0
            st.write(b"X")
            st.flush()
            stats['sent'] += 1
            unmatched_send_times.append((send_t, actual_delay_ms))
            print(f"[t={send_t-start:6.2f}s] SEND DISARM  "
                  f"(delay after TX->RXW: {actual_delay_ms:.1f}ms)")

            # Wait between commands — 10s per the T1 pattern (max retry window)
            # to let each command complete its retry lifecycle before next.
            end_at = send_t + cmd_timeout_s
            while time.time() < end_at:
                drain_station(timeout_s=0.1)

        # Final drain
        time.sleep(1.0)
        drain_station(timeout_s=2.0)

    except KeyboardInterrupt:
        print("\n[interrupted]")
    finally:
        shared.stop.set()
        veh_thread.join(timeout=2.0)
        station_log_f.close()
        vehicle_log_f.close()
        try:
            st.close()
        except Exception:
            pass

    # Write CSV
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "sent_host_t", "ack_host_t",
                    "ack_retry_slot", "retry_count",
                    "denied", "failed", "send_after_rxw_ms"])
        for seq in seq_order:
            r = records[seq]
            w.writerow([
                r['seq'],
                f"{r['sent_host_t']:.3f}" if r['sent_host_t'] else "",
                f"{r['ack_host_t']:.3f}" if r['ack_host_t'] else "",
                r['ack_retry_slot'], r['retry_count'],
                r['denied'], r['failed'],
                f"{r['send_after_rxw_ms']:.1f}" if r['send_after_rxw_ms'] >= 0 else "",
            ])

    # Summary
    n = stats['sent']
    print()
    print("=" * 60)
    print("RESULTS — T2 CHEAT MODE (host-side sync)")
    print("=" * 60)
    print(f"  Sent:          {n}")
    print(f"  ACK'd:         {stats['acked']}  "
          f"({100.0*stats['acked']/max(n,1):.1f}%)")
    print(f"    first-try:   {stats['first_try']}  "
          f"({100.0*stats['first_try']/max(n,1):.1f}%)  <-- T2 headline")
    print(f"    retry 1:     {stats['r1']}")
    print(f"    retry 2:     {stats['r2']}")
    print(f"    retry 3:     {stats['r3']}")
    print(f"  DENIED:        {stats['denied']}")
    print(f"  Failed:        {stats['failed']}  "
          f"({100.0*stats['failed']/max(n,1):.1f}%)")
    print("=" * 60)


def main():
    ap = argparse.ArgumentParser(description="Stage T IVP-T2 cheat-mode (host)")
    ap.add_argument("--count", type=int, default=100,
                    help="Commands to send (default 100)")
    ap.add_argument("--delay-ms", type=int, default=10,
                    help="Ms after vehicle TX->RXW before firing X "
                         "(default 10, window is ~100ms)")
    ap.add_argument("--cmd-timeout", type=float, default=10.0,
                    help="Seconds per command (default 10 — retry lifecycle)")
    ap.add_argument("--station-port", default="COM9")
    ap.add_argument("--vehicle-port", default="COM7")
    ap.add_argument("--csv", default="logs/stage_t/t2_cheat.csv")
    args = ap.parse_args()
    run(args.count, args.delay_ms, args.cmd_timeout,
        args.station_port, args.vehicle_port, args.csv)


if __name__ == "__main__":
    main()
