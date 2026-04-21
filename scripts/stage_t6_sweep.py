#!/usr/bin/env python3
"""Stage T IVP-T6 — LoRa bandwidth sweep.

**What T6 measures:** usability of each radio config — the ACK rate of
routine commands (DISARM) at BW125/250/500. Question: "when vehicle is
on BW500 and station sends DISARM, how reliably does it land?"

**What T6 does NOT measure:** reliability of SET_RADIO_CONFIG itself.
SET is a one-time infrastructure op; its reliability is characterized
elsewhere (T1 baseline, T5.5 symmetric revert, T7 retry compression).

**Split:**
  - Config-switching (infrastructure): BOTH boards via USB CDC debug
    menu `q<digit>z` (digits 0..5 = whitelist indexes). No RF involved.
    Deterministic.
  - Measurement (system under test): 5-min DISARM ACK stress over RF.
    FIFO-matched ACK counts. Goes in the CSV.

Prerequisites:
  - Bench firmware on both boards (dev CLI not in flight builds).
  - `ROCKETCHIP_RADIO_PERSIST` NOT defined — persistence must be off so
    swept configs don't leak into flash between runs.

Configs swept (rc::kRadioConfigTable indexes):
  C0  idx 0  BW 125 / 5  Hz  — default / control (reproduces T1 baseline)
  C0P idx 1  BW 125 / 10 Hz  — same BW, higher rate
  C1  idx 2  BW 250 / 10 Hz  — halfway
  C2  idx 3  BW 500 / 10 Hz  — primary candidate

Pass criteria (Stage T plan):
  - First-try ACK >= 60% on C2 (BW500).
  - C0 reproduces T1 baseline within noise.

Usage:
  python scripts/stage_t6_sweep.py
  python scripts/stage_t6_sweep.py --count 50 --interval 10
  python scripts/stage_t6_sweep.py --configs C0,C1,C2
"""
from __future__ import annotations

import argparse
import csv
import os
import re
import sys
import time
from dataclasses import dataclass, field
from typing import Optional

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial not installed. pip install pyserial",
          file=sys.stderr)
    sys.exit(2)


# (label, whitelist_idx, bw_khz, nav_hz)
SWEEP = [
    ("C0",  0, 125,  5),
    ("C0P", 1, 125, 10),
    ("C1",  2, 250, 10),
    ("C2",  3, 500, 10),
]

ROCKETCHIP_USB_VID = 0x2E8A
ROCKETCHIP_USB_PID = 0x0009

RE_DISARM_SENT = re.compile(r"\[CMD\]\s+DISARM sent")
RE_ACK_OK      = re.compile(r"\[CMD\]\s+ACK'd\s+\(seq=(\d+)\)")
RE_ACK_DENIED  = re.compile(r"\[CMD\]\s+DENIED\s+\(seq=(\d+)\)")
RE_PKTS        = re.compile(r"Pkts:\s*(\d+)")
RE_RX_COUNT    = re.compile(r"RX:\s*(\d+)\s*pkts")
RE_RADIO_ROW   = re.compile(
    r"Radio:\s*BW(\d+)\s+(\d+)Hz\s+SF(\d+)\s+CR(\d+)\s*\|\s*"
    r"Vehicle:\s*BW(\d+)\s+(\d+)Hz")
RE_CFG_LINE    = re.compile(r"\[cfg\] local radio -> BW(\d+) (\d+)Hz")


# ----------------------------------------------------------------------------
# Port discovery — identify vehicle vs station by banner.
# ----------------------------------------------------------------------------

def _peek_banner(port_name: str) -> str:
    try:
        s = serial.Serial(port_name, 115200, timeout=0.5)
        s.write(b"h")
        time.sleep(1.5)
        data = s.read(8000)
        s.close()
        return data.decode("utf-8", errors="replace").lower()
    except (serial.SerialException, OSError):
        return ""


def find_ports() -> tuple[Optional[str], Optional[str]]:
    cands = [
        p.device for p in serial.tools.list_ports.comports()
        if p.vid == ROCKETCHIP_USB_VID and p.pid == ROCKETCHIP_USB_PID
    ]
    veh, stn = None, None
    for p in cands:
        b = _peek_banner(p)
        if "ground station" in b or "fruit jam" in b:
            stn = p
        elif "feather" in b or "[main]" in b:
            if "ground station" not in b:
                veh = p
    return veh, stn


# ----------------------------------------------------------------------------
# Safe write (Windows CDC is flaky mid-reboot)
# ----------------------------------------------------------------------------

def _safe_write(ser: serial.Serial, data: bytes) -> bool:
    try:
        ser.write(data)
        return True
    except (serial.SerialException, OSError):
        try:
            ser.close()
            time.sleep(0.3)
            ser.open()
            time.sleep(0.3)
            ser.write(data)
            return True
        except (serial.SerialException, OSError):
            return False


# ----------------------------------------------------------------------------
# Config switching over USB CDC
# ----------------------------------------------------------------------------

def set_local_config(ser: serial.Serial, label: str,
                      whitelist_idx: int) -> bool:
    """Send `q<digit>z` to enter debug menu, set local config, exit.
    Returns True if the digit was sent (trust the firmware — [cfg]
    confirmation may be hidden by station dashboard rendering)."""
    ser.reset_input_buffer()
    if not _safe_write(ser, b"q"):
        return False
    time.sleep(0.5)
    ser.reset_input_buffer()
    if not _safe_write(ser, str(whitelist_idx).encode()):
        return False
    time.sleep(1.0)
    _safe_write(ser, b"z")
    time.sleep(0.3)
    ser.reset_input_buffer()
    return True


def switch_both(veh: serial.Serial, stn: serial.Serial,
                 idx: int, bw: int, nav: int,
                 verify_timeout_s: float = 20.0) -> bool:
    """Drive both boards to a specific whitelist index over USB.
    Returns True only after confirming BOTH:
      (a) station dashboard Radio row shows station==vehicle==target
      (b) station Pkts counter has climbed (>=3 packets received on the
          new config) — proves the link is actually working, not just
          that configs are reported as matching.

    Returns False if either check fails within verify_timeout_s."""
    print(f"    -> vehicle: local cfg idx {idx} (BW{bw}/{nav}Hz)",
          flush=True)
    set_local_config(veh, "vehicle", idx)
    print(f"    -> station: local cfg idx {idx} (BW{bw}/{nav}Hz)",
          flush=True)
    set_local_config(stn, "station", idx)

    # Settle — radios need ~200ms for the backstop apply plus time for
    # the first nav packet on the new config to land.
    print("    -> settle 5s", flush=True)
    time.sleep(5.0)

    # Capture baseline Pkts before verification window.
    stn.reset_input_buffer()
    time.sleep(2.0)
    baseline_data = stn.read(30000).decode("utf-8", errors="replace")
    baseline_pkts = None
    for m in RE_PKTS.finditer(baseline_data):
        baseline_pkts = int(m.group(1))
    if baseline_pkts is None:
        # Station not rendering dashboard — try RX: fallback.
        for m in RE_RX_COUNT.finditer(baseline_data):
            baseline_pkts = int(m.group(1))
    if baseline_pkts is None:
        print("    [FAIL] could not read station Pkts baseline", flush=True)
        return False
    print(f"    -> baseline Pkts={baseline_pkts}", flush=True)

    # Verify (a) Radio row match AND (b) Pkts climbed >= 3.
    row_ok = False
    pkts_ok = False
    deadline = time.time() + verify_timeout_s
    while time.time() < deadline:
        time.sleep(2.0)
        data = stn.read(30000).decode("utf-8", errors="replace")
        # (a) Radio row match
        for m in RE_RADIO_ROW.finditer(data):
            sbw, snav, ssf, scr, vbw, vnav = map(int, m.groups())
            if (sbw == bw and snav == nav
                    and vbw == bw and vnav == nav):
                row_ok = True
        # (b) Pkts climb
        last_pkts = None
        for m in RE_PKTS.finditer(data):
            last_pkts = int(m.group(1))
        if last_pkts is None:
            for m in RE_RX_COUNT.finditer(data):
                last_pkts = int(m.group(1))
        if last_pkts is not None and last_pkts >= baseline_pkts + 3:
            pkts_ok = True
        if row_ok and pkts_ok:
            print(f"    -> link confirmed at BW{bw}/{nav}Hz: "
                  f"Radio row matches, Pkts {baseline_pkts}->{last_pkts}",
                  flush=True)
            return True

    # Failure diagnostics for the operator.
    print(f"    [FAIL] BW{bw}/{nav}Hz verification failed after "
          f"{verify_timeout_s:.0f}s:", flush=True)
    print(f"          Radio row match: {row_ok}", flush=True)
    print(f"          Pkts climb >=3:  {pkts_ok}", flush=True)
    return False


# reset_both_to_default removed — with ROCKETCHIP_RADIO_PERSIST undef
# (default during Stage T testing), every boot/power-cycle already gives
# kDefaultRocketRadioConfig. A mid-session "reset to default" is just a
# switch to whitelist idx 0 (handled by set_local_config).


# ----------------------------------------------------------------------------
# ACK stress harness
# ----------------------------------------------------------------------------

@dataclass
class AckResult:
    sent: int = 0
    first_try_ack: int = 0
    eventual_ack: int = 0
    denied: int = 0
    no_ack: int = 0
    latencies_ms: list[float] = field(default_factory=list)

    @property
    def first_try_pct(self) -> float:
        return 100.0 * self.first_try_ack / self.sent if self.sent else 0.0

    @property
    def eventual_pct(self) -> float:
        return 100.0 * self.eventual_ack / self.sent if self.sent else 0.0

    @property
    def mean_latency_ms(self) -> float:
        return (sum(self.latencies_ms) / len(self.latencies_ms)
                if self.latencies_ms else 0.0)


def run_ack_stress(stn: serial.Serial, count: int, interval_s: float,
                    label: str) -> AckResult:
    """Send `count` DISARMs over `interval_s`. FIFO-match ACKs."""
    result = AckResult()
    first_try_window = interval_s - 0.5
    outstanding: list[tuple[int, float]] = []  # (send_idx, send_time)

    def drain_once() -> None:
        while True:
            line = stn.readline().decode("utf-8", errors="replace")
            if not line:
                return
            if RE_ACK_OK.search(line):
                if outstanding:
                    _, t_send = outstanding.pop(0)
                    now = time.time()
                    lat_ms = 1000.0 * (now - t_send)
                    result.eventual_ack += 1
                    result.latencies_ms.append(lat_ms)
                    if lat_ms <= 1000.0 * first_try_window:
                        result.first_try_ack += 1
            elif RE_ACK_DENIED.search(line):
                if outstanding:
                    outstanding.pop(0)
                    result.denied += 1

    stn.reset_input_buffer()
    start = time.time()
    for i in range(count):
        target = start + i * interval_s
        while time.time() < target:
            drain_once()
            time.sleep(0.05)

        if not _safe_write(stn, b"X"):
            continue
        result.sent += 1
        outstanding.append((i, time.time()))
        time.sleep(0.15)
        drain_once()

        elapsed = time.time() - start
        print(f"  [{label}] {i+1}/{count} t={elapsed:5.1f}s "
              f"eventual={result.eventual_ack} "
              f"first_try={result.first_try_ack} "
              f"pending={len(outstanding)}", flush=True)

    # Trailing drain — give slow retries 15s to land.
    deadline = time.time() + 15.0
    while time.time() < deadline and outstanding:
        drain_once()
        time.sleep(0.2)

    result.no_ack = len(outstanding)
    return result


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------

def canary(stn: serial.Serial, timeout_s: float = 20.0) -> bool:
    """Station has seen at least 1 nav packet. Station defaults to kAnsi
    (dashboard) at boot — dashboard frames print every ~1s. Just wait
    and scrape Pkts/RX from the stream."""
    deadline = time.time() + timeout_s
    stn.reset_input_buffer()
    while time.time() < deadline:
        time.sleep(2.0)
        data = stn.read(40000).decode("utf-8", errors="replace")
        for m in RE_PKTS.finditer(data):
            if int(m.group(1)) >= 1:
                return True
        for m in RE_RX_COUNT.finditer(data):
            if int(m.group(1)) >= 1:
                return True
    return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=30)
    ap.add_argument("--interval", type=float, default=10.0)
    ap.add_argument("--configs", default="C0,C0P,C1,C2")
    ap.add_argument("--csv", default="logs/stage_t/t6_summary.csv")
    args = ap.parse_args()

    veh_port, stn_port = find_ports()
    if not (veh_port and stn_port):
        print(f"ERROR: could not find both ports (veh={veh_port}, "
              f"stn={stn_port})")
        return 2
    print(f"Vehicle port: {veh_port}")
    print(f"Station port: {stn_port}")

    selected = set(args.configs.split(","))
    sweep = [(lbl, idx, bw, nav) for (lbl, idx, bw, nav) in SWEEP
             if lbl in selected]
    if not sweep:
        print(f"ERROR: no configs selected from {args.configs}")
        return 2

    os.makedirs(os.path.dirname(args.csv), exist_ok=True)

    veh = serial.Serial(veh_port, 115200, timeout=0.05)
    stn = serial.Serial(stn_port, 115200, timeout=0.05)
    time.sleep(1.0)

    results: list[tuple[str, int, int, AckResult]] = []
    try:
        # Start clean — drive both boards to idx 0 (default).
        # Persistence-off means they already boot there, but if the previous
        # sweep run ended with boards on a different config, this normalises.
        print("\n--- Initialize both to idx 0 (default) ---")
        set_local_config(veh, "vehicle", 0)
        set_local_config(stn, "station", 0)
        time.sleep(5.0)

        print("\n--- Canary: station receiving packets? ---")
        if not canary(stn):
            print("CANARY FAIL: station not seeing nav packets.")
            return 1
        print("CANARY OK: link healthy")

        for (label, idx, bw, nav) in sweep:
            print(f"\n=== [{label}] BW{bw} {nav}Hz "
                  f"— {args.count} sends @ {args.interval}s ===")

            if not switch_both(veh, stn, idx, bw, nav):
                print(f"  [ABORT] link not confirmed at BW{bw}/{nav}Hz. "
                      "Measuring now would produce meaningless data. "
                      "Stopping sweep.", flush=True)
                # Best-effort: drop back to idx 0 so boards match again.
                set_local_config(veh, "vehicle", 0)
                set_local_config(stn, "station", 0)
                return 1
            r = run_ack_stress(stn, args.count, args.interval, label)
            print(f"  [{label}] first_try={r.first_try_pct:5.1f}%  "
                  f"eventual={r.eventual_pct:5.1f}%  "
                  f"mean_lat={r.mean_latency_ms:.0f}ms  "
                  f"denied={r.denied} no_ack={r.no_ack}")
            results.append((label, bw, nav, r))
    finally:
        print("\n--- Restore both boards to idx 0 (default) ---")
        set_local_config(veh, "vehicle", 0)
        set_local_config(stn, "station", 0)
        veh.close()
        stn.close()

    # Write CSV.
    with open(args.csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["config", "bw_khz", "nav_hz", "sent",
                    "first_try_ack", "eventual_ack", "denied", "no_ack",
                    "first_try_pct", "eventual_pct", "mean_lat_ms"])
        for (label, bw, nav, r) in results:
            w.writerow([label, bw, nav, r.sent,
                        r.first_try_ack, r.eventual_ack, r.denied, r.no_ack,
                        f"{r.first_try_pct:.2f}", f"{r.eventual_pct:.2f}",
                        f"{r.mean_latency_ms:.1f}"])

    print(f"\nResults written to {args.csv}")
    print("\n=== SUMMARY ===")
    for (label, bw, nav, r) in results:
        tag = ""
        if label == "C2" and r.first_try_pct >= 60.0:
            tag = "  <-- MEETS 60% PASS THRESHOLD"
        elif label == "C2":
            tag = "  <-- BELOW 60% PASS THRESHOLD"
        print(f"  {label:4s} BW{bw:3d}/{nav:2d}Hz: "
              f"first_try={r.first_try_pct:5.1f}%  "
              f"eventual={r.eventual_pct:5.1f}%{tag}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
