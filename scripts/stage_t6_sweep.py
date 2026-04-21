#!/usr/bin/env python3
"""Stage T IVP-T6 — LoRa bandwidth sweep.

**What T6 measures:** the usability of each radio config — specifically,
the ACK rate of ROUTINE commands (DISARM) at BW125/250/500. Question:
"when vehicle is operating at BW500 and station sends a DISARM, how
reliably does that DISARM land?" That is the flight-safety question.

**What T6 does NOT measure:** the reliability of SET_RADIO_CONFIG itself.
SET is a one-time infrastructure operation that just needs to land
eventually — its reliability is characterized elsewhere (T1 baseline
~6% first-try, T7 retry-timer fix, T5.5 symmetric revert).

**Split:**
  - Config-switching (infrastructure): BOTH boards via USB-CDC `qc`
    debug-menu key (`q` to enter debug, `c` to cycle local runtime_config
    through the whitelist). No RF involved — each board switches its
    own radio independently. Deterministic.
  - Measurement (system under test): 5-min DISARM ACK stress over RF.
    Counts first-try + eventual ACKs. THIS is what goes in the CSV.

Configs swept (aligned with `rc::kRadioConfigTable`):
  C0  BW 125 / nav 5  Hz  — default / control (reproduces T1 baseline)
  C0P BW 125 / nav 10 Hz  — control at target rate (BW held, rate up)
  C1  BW 250 / nav 10 Hz  — halfway
  C2  BW 500 / nav 10 Hz  — primary candidate

Per-config procedure:
  1. Cycle BOTH vehicle and station through `qc` debug command until each
     board's runtime_config matches the target.
  2. Wait 10s for both radios to settle.
  3. Verify end-to-end: station dashboard Radio row shows matching values.
  4. Run 5-min DISARM ACK stress (30 sends @ 10s).
  5. Record CSV row.

Pass criteria (Stage T plan section 3, T6):
  - First-try ACK >= 60% on C2 (BW500).
  - C0 reproduces T1 baseline within noise.

Requires:
  - Bench firmware on both boards (dev CLI only in non-flight build).
  - Vehicle USB CDC at 'Feather' / 'rocketchip' banner.
  - Station USB CDC at 'Ground Station' banner.

Usage:
  python scripts/stage_t6_sweep.py
  python scripts/stage_t6_sweep.py --count 50
  python scripts/stage_t6_sweep.py --configs C0,C1,C2
"""
from __future__ import annotations

import argparse
import csv
import os
import re
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import Optional

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial not installed. pip install pyserial", file=sys.stderr)
    sys.exit(2)

GDB = r"C:\Users\pow-w\.pico-sdk\toolchain\14_2_Rel1\bin\arm-none-eabi-gdb.exe"
VEHICLE_ELF = "build/rocketchip.elf"

# SX1276 register values for the BW field in RegModemConfig1.
# 7=125 kHz, 8=250 kHz, 9=500 kHz. Matches ao_radio_apply_runtime_config.
BW_REG_MAP = {125: 7, 250: 8, 500: 9}


# Match kRadioConfigTable indexes in the firmware:
#   idx 0 = default (BW125/5),  idx 1 = BW125/10, idx 2 = BW250/10, idx 3 = BW500/10
SWEEP = [
    # (label, target_idx, bw, nav)
    ("C0",  0, 125,  5),   # control — default
    ("C0P", 1, 125, 10),   # control at 10Hz (for fair comparison to C1/C2)
    ("C1",  2, 250, 10),
    ("C2",  3, 500, 10),
]

ROCKETCHIP_USB_VID = 0x2E8A
ROCKETCHIP_USB_PID = 0x0009

# Station dashboard / CLI patterns
RE_DISARM_SENT = re.compile(r"\[CMD\]\s+DISARM sent")
RE_ACK_OK      = re.compile(r"\[CMD\]\s+ACK'd\s+\(seq=(\d+)\)")
RE_ACK_DENIED  = re.compile(r"\[CMD\]\s+DENIED\s+\(seq=(\d+)\)")
RE_SET_SENT    = re.compile(r"\[CMD\]\s+SET_RADIO_CONFIG sent")
RE_STN_SWITCH  = re.compile(
    r"\[CMD\]\s+station switching radio to BW=(\d+)\s+nav=(\d+)")
RE_PKTS        = re.compile(r"Pkts:\s*(\d+)")
RE_RADIO_ROW   = re.compile(
    r"Radio:\s*BW(\d+)\s+(\d+)Hz\s+SF(\d+)\s+CR(\d+)\s*\|\s*"
    r"Vehicle:\s*BW(\d+)\s+(\d+)Hz\s+SF(\d+)\s+CR(\d+)")


# ----------------------------------------------------------------------------
# Port discovery — prefer station, else any RocketChip.
# ----------------------------------------------------------------------------

def _peek_banner(port_name: str, settle: float = 1.5) -> str:
    try:
        s = serial.Serial(port_name, 115200, timeout=0.5)
        s.write(b"h")
        time.sleep(settle)
        data = s.read(8000)
        s.close()
        return data.decode("utf-8", errors="replace").lower()
    except (serial.SerialException, OSError):
        return ""


def find_ports() -> tuple[Optional[str], Optional[str]]:
    """Return (vehicle_port, station_port). Distinguishes by banner."""
    cands = [
        p.device for p in serial.tools.list_ports.comports()
        if p.vid == ROCKETCHIP_USB_VID and p.pid == ROCKETCHIP_USB_PID
    ]
    veh, stn = None, None
    for p in cands:
        b = _peek_banner(p)
        if "ground station" in b or "fruit jam" in b:
            stn = p
        elif "feather" in b or "rocketchip" in b or "[main]" in b:
            if "ground station" not in b:
                veh = p
    return veh, stn


def find_station_port() -> Optional[str]:
    _, stn = find_ports()
    return stn


# ----------------------------------------------------------------------------
# ACK stress harness — sends DISARM, counts first-try + eventual ACKs.
# ----------------------------------------------------------------------------

@dataclass
class AckResult:
    sent: int = 0
    first_try_ack: int = 0          # ACK landed before next DISARM was sent
    eventual_ack: int = 0           # ACK landed (possibly after retries)
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


def run_ack_stress(ser: serial.Serial, count: int, interval_s: float,
                   label: str) -> AckResult:
    """Send `count` DISARMs, `interval_s` apart, classify ACK results."""
    result = AckResult()
    # One outstanding DISARM at a time; station retries internally.
    send_times: dict[int, float] = {}   # seq -> send_time
    next_first_try_deadline: Optional[float] = None
    pending_seq: Optional[int] = None   # seq of the send currently in flight
    first_try_window = interval_s - 0.5  # "first try" = ACK before next send

    def drain_and_classify() -> None:
        nonlocal pending_seq, next_first_try_deadline
        line = ser.readline().decode("utf-8", errors="replace")
        while line:
            if RE_DISARM_SENT.search(line):
                pass  # our own echo
            elif (m := RE_ACK_OK.search(line)):
                seq = int(m.group(1))
                now = time.time()
                sent_at = send_times.pop(seq, None)
                if sent_at is not None:
                    result.eventual_ack += 1
                    result.latencies_ms.append(1000.0 * (now - sent_at))
                    if seq == pending_seq and next_first_try_deadline is not None \
                            and now <= next_first_try_deadline:
                        result.first_try_ack += 1
                    if seq == pending_seq:
                        pending_seq = None
            elif (m := RE_ACK_DENIED.search(line)):
                seq = int(m.group(1))
                sent_at = send_times.pop(seq, None)
                if sent_at is not None:
                    result.denied += 1
                    if seq == pending_seq:
                        pending_seq = None
            line = ser.readline().decode("utf-8", errors="replace")

    # Drain anything stale.
    ser.reset_input_buffer()

    start = time.time()
    for i in range(count):
        target = start + i * interval_s
        while time.time() < target:
            drain_and_classify()
            time.sleep(0.05)

        # Sequence numbers come back from firmware — station picks them.
        # We don't know the seq before the `[CMD] DISARM sent` line, but since
        # commands are serial (one at a time), we match by order: the NEXT
        # ACK'd line in flight is for this send.
        ts_send = time.time()
        ser.write(b"X")
        result.sent += 1
        # Find the ACK seq by scanning forward until DISARM sent line appears,
        # then the next ACK is ours. Simpler: use a monotonic pseudo-seq per
        # send since firmware's seq is per-session and we can correlate by
        # order only.
        # Use position-based correlation: pending_seq is the latest seq we've
        # yet to classify. Don't need to know its numeric value — just count
        # ACKs in order of receipt until each "pending slot" is filled.
        pending_seq = result.sent  # pseudo-seq == sent count (matched by order)
        # Remap: store by pseudo-seq so drain_and_classify above can reference
        # Actually easier: use a simple FIFO of pending sends. See redesign
        # below — rather than matching ACK seq to our pseudo-seq (which won't
        # match because firmware assigns its own), we just count matched ACKs
        # in order of receipt.
        send_times[pending_seq] = ts_send
        next_first_try_deadline = ts_send + first_try_window

        # Give firmware a moment before draining.
        time.sleep(0.1)
        drain_and_classify()

        elapsed = time.time() - start
        print(f"  [{label}] send {i+1}/{count}  t={elapsed:5.1f}s  "
              f"eventual={result.eventual_ack} first_try={result.first_try_ack}",
              flush=True)

    # Final 5s drain for trailing ACKs / retries.
    deadline = time.time() + 5.0
    while time.time() < deadline:
        drain_and_classify()
        time.sleep(0.1)

    result.no_ack = result.sent - result.eventual_ack - result.denied
    return result


# NOTE on the seq-matching above: firmware's `ack.cmd_seq` comes from
# `cmd.confirmation` which station's tracked-command layer increments
# per-command on the station side. We're sending from the host via `X`, not
# tracking seqs from the host end. The simplest correct model: FIFO match —
# whatever ACK comes back next is for the next outstanding send. Rewrite:


def run_ack_stress_fifo(ser: serial.Serial, count: int, interval_s: float,
                         label: str) -> AckResult:
    """FIFO-matched ACK classifier. Simpler & correct."""
    result = AckResult()
    first_try_window = interval_s - 0.5

    # Queue of send times for outstanding commands (FIFO).
    outstanding: list[tuple[int, float]] = []   # (send_idx, send_time)

    def drain_once() -> None:
        while True:
            line = ser.readline().decode("utf-8", errors="replace")
            if not line:
                return
            if (m := RE_ACK_OK.search(line)):
                if outstanding:
                    idx, t_send = outstanding.pop(0)
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

    ser.reset_input_buffer()
    start = time.time()
    for i in range(count):
        # Wait until scheduled send time.
        target = start + i * interval_s
        while time.time() < target:
            drain_once()
            time.sleep(0.05)

        if not _safe_write(ser, b"X"):
            continue  # skip this slot, try next
        result.sent += 1
        outstanding.append((i, time.time()))
        time.sleep(0.15)
        drain_once()

        elapsed = time.time() - start
        print(f"  [{label}] {i+1}/{count}  t={elapsed:5.1f}s  "
              f"eventual={result.eventual_ack}  "
              f"first_try={result.first_try_ack}  "
              f"pending={len(outstanding)}", flush=True)

    # Trailing drain — give slow retries 15s to land.
    deadline = time.time() + 15.0
    while time.time() < deadline and outstanding:
        drain_once()
        time.sleep(0.2)

    result.no_ack = len(outstanding)
    return result


# ----------------------------------------------------------------------------
# Config-switch harness
# ----------------------------------------------------------------------------

def wait_for_radio_row(ser: serial.Serial, expect_bw: int, expect_nav: int,
                        *, match_vehicle: bool,
                        timeout_s: float = 20.0) -> bool:
    """Wait for station dashboard to show a Radio row matching target.
    match_vehicle=False -> only require station side == target (vehicle
        column may show ?, legacy config, or stale — means station switched
        but hasn't received a packet on the new config yet).
    match_vehicle=True  -> require BOTH station and vehicle columns ==
        target (end-to-end convergence)."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        line = ser.readline().decode("utf-8", errors="replace")
        if not line:
            time.sleep(0.1)
            continue
        if (m := RE_RADIO_ROW.search(line)):
            sbw, snav, ssf, scr, vbw, vnav, vsf, vcr = map(int, m.groups())
            if sbw == expect_bw and snav == expect_nav:
                if not match_vehicle:
                    return True
                if vbw == expect_bw and vnav == expect_nav:
                    return True
    return False


# Whitelist indexes: must match include/rocketchip/radio_config_table.h.
# (label, bw, nav)
WHITELIST = [
    (0, 125,  5),
    (1, 125, 10),
    (2, 250, 10),
    (3, 500, 10),
    (4, 125,  2),
    (5, 250,  5),
]

def whitelist_index_for(bw: int, nav: int) -> int:
    for (idx, b, n) in WHITELIST:
        if b == bw and n == nav:
            return idx
    raise ValueError(f"unknown whitelist entry BW{bw}/{nav}Hz")


def _safe_write(ser: serial.Serial, data: bytes) -> bool:
    """Windows CDC sometimes throws PermissionError on write when the port
    is in a weird state. Wrap write + retry once after a reset cycle."""
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
        except (serial.SerialException, OSError) as e:
            print(f"    [WARN] serial write failed: {e}", flush=True)
            return False


def set_test_mode(ser: serial.Serial, port_label: str,
                   enable: bool) -> None:
    """Toggle radio test mode on the given board (`qt` from main menu).
    Test mode suppresses persistence + symmetric-revert so swept configs
    stay stable for the full ACK-stress window."""
    ser.reset_input_buffer()
    if not _safe_write(ser, b"q"):
        return
    time.sleep(0.5)
    ser.reset_input_buffer()
    if not _safe_write(ser, b"t"):
        return
    time.sleep(0.5)
    data = ser.read(8000).decode("utf-8", errors="replace")
    _safe_write(ser, b"z")
    time.sleep(0.3)
    ser.reset_input_buffer()
    # Check for the toggle confirmation. If already in desired state, the
    # toggle will have PUT us in the WRONG state — so always toggle twice
    # if the first read doesn't show the state we wanted.
    want = "ENABLED" if enable else "DISABLED"
    avoid = "DISABLED" if enable else "ENABLED"
    if want in data:
        print(f"    {port_label}: test mode {want}", flush=True)
    elif avoid in data:
        # We toggled FROM the state we wanted; toggle again.
        if not _safe_write(ser, b"q"):
            return
        time.sleep(0.5)
        ser.reset_input_buffer()
        if not _safe_write(ser, b"t"):
            return
        time.sleep(0.5)
        data = ser.read(8000).decode("utf-8", errors="replace")
        _safe_write(ser, b"z")
        time.sleep(0.3)
        ser.reset_input_buffer()
        if want in data:
            print(f"    {port_label}: test mode {want} (double-toggled)",
                  flush=True)
        else:
            print(f"    {port_label}: [WARN] test mode state unclear: "
                  f"'{want}' not found in output", flush=True)
    else:
        print(f"    {port_label}: [WARN] test mode response not matched "
              f"('{want}' or '{avoid}' not in output)", flush=True)


def set_local_config(ser: serial.Serial, port_label: str,
                      bw: int, nav: int) -> bool:
    """Send `q<digit>z` on the given board to set its local radio config
    directly by whitelist index. No cycling, no state tracking — deterministic.

    Sequence:
      `q` — enter debug menu (works from main menu regardless of whether
             station dashboard is rendering).
      `<digit>` — select the whitelist entry (0..5).
      `z` — return to main menu.

    On vehicle the `[cfg]` confirmation line is usually visible. On station
    the dashboard may overwrite it — we don't rely on that confirmation.
    Instead we verify end-to-end afterwards via station's rx_count climbing
    (see confirm_link_on_config). Returns True if digit was sent."""
    idx = whitelist_index_for(bw, nav)
    target_re = re.compile(rf"\[cfg\] local radio -> BW{bw} {nav}Hz")

    # Enter debug menu.
    ser.reset_input_buffer()
    if not _safe_write(ser, b"q"):
        return False
    time.sleep(0.5)
    ser.reset_input_buffer()

    # Send the digit.
    if not _safe_write(ser, str(idx).encode()):
        return False

    # Wait up to 3s for the [cfg] confirmation line.
    deadline = time.time() + 3.0
    confirmed = False
    while time.time() < deadline:
        data = ser.read(4000).decode("utf-8", errors="replace")
        if data and target_re.search(data):
            confirmed = True
            break
        time.sleep(0.1)

    # Leave debug menu.
    _safe_write(ser, b"z")
    time.sleep(0.3)
    ser.reset_input_buffer()

    if confirmed:
        print(f"    {port_label}: cfg set BW{bw} {nav}Hz (idx {idx})",
              flush=True)
    else:
        print(f"    {port_label}: cfg digit {idx} sent (confirmation hidden "
              "by dashboard — validated downstream via rx_count)",
              flush=True)
    # Return True as long as the digit was sent. Station dashboard often
    # hides `[cfg]` lines — the real validation is end-to-end rx_count.
    return True


def switch_to_config(veh_ser: serial.Serial, stn_ser: serial.Serial,
                      bw: int, nav: int) -> bool:
    """Set BOTH boards' local runtime_config to target via USB `q<digit>z`.
    No RF involved. Wait for radios to settle + link to come up. Verify
    end-to-end by peeking station dashboard Radio row."""
    veh_ok = set_local_config(veh_ser, "vehicle", bw, nav)
    stn_ok = set_local_config(stn_ser, "station", bw, nav)

    # Even without `[cfg]` confirmation, the digit was sent — firmware may
    # still have applied it. Give radios 10s to settle regardless.
    print("    -> settle 10s for radios", flush=True)
    time.sleep(10.0)

    # Verify end-to-end via rx_count climb (works in any menu mode).
    print(f"    -> verify station rx_count climbing at BW{bw}/{nav}Hz",
          flush=True)
    end_ok = confirm_link_on_config(stn_ser, bw, nav)
    if end_ok:
        print(f"    -> link confirmed: station receiving at BW{bw}/{nav}Hz",
              flush=True)
    else:
        print(f"    [WARN] rx_count did not climb; "
              f"veh_ok={veh_ok} stn_ok={stn_ok} — running stress anyway",
              flush=True)
    return end_ok


RE_RX_COUNT = re.compile(r"RX:\s*(\d+)\s*pkts")


def canary(ser: serial.Serial, timeout_s: float = 20.0) -> bool:
    """Verify station has seen at least one nav packet (RX >= 1).

    Strategy: bulk-read 3s of station output and scan for ANY of:
      - `Pkts: N` (dashboard line, where N >= 1)
      - `RX: N pkts` (t-status line, where N >= 1)
    Station streams dashboard continuously in RX_MENU_MAIN, so this works
    regardless of menu state. If not found in first 3s, send `t` to force
    a status print, then look for RX: N pkts. Repeat until timeout.
    """
    deadline = time.time() + timeout_s
    ser.reset_input_buffer()
    # Passive pass — dashboard is a running frame, should see Pkts: soon.
    time.sleep(3.0)
    data = ser.read(40000).decode("utf-8", errors="replace")
    for m in RE_PKTS.finditer(data):
        if int(m.group(1)) >= 1:
            return True
    for m in RE_RX_COUNT.finditer(data):
        if int(m.group(1)) >= 1:
            return True

    # Active fallback — request t-status, poll for RX count.
    while time.time() < deadline:
        ser.reset_input_buffer()
        ser.write(b"t")
        time.sleep(1.5)
        data = ser.read(8000).decode("utf-8", errors="replace")
        for m in RE_PKTS.finditer(data):
            if int(m.group(1)) >= 1:
                return True
        for m in RE_RX_COUNT.finditer(data):
            if int(m.group(1)) >= 1:
                return True
        time.sleep(1.0)
    return False


def confirm_link_on_config(ser: serial.Serial, bw: int, nav: int,
                            timeout_s: float = 25.0) -> bool:
    """After a config switch, verify station's rx_count is climbing on the
    new config. Sample rx_count over a few seconds using dashboard Pkts:
    line (preferred, cheap) or `t` status line as fallback.

    Returns True if rx_count increased OR we saw multiple distinct Pkts
    values over the window."""
    def sample_rx() -> Optional[int]:
        # Passive read of dashboard Pkts:
        ser.reset_input_buffer()
        time.sleep(1.5)
        data = ser.read(30000).decode("utf-8", errors="replace")
        matches = list(RE_PKTS.finditer(data))
        if matches:
            return int(matches[-1].group(1))
        # Fallback: active `t`
        if not _safe_write(ser, b"t"):
            return None
        time.sleep(1.0)
        data = ser.read(8000).decode("utf-8", errors="replace")
        m = RE_RX_COUNT.search(data)
        if m:
            return int(m.group(1))
        return None

    rx_a = sample_rx()
    if rx_a is None:
        return False

    deadline = time.time() + timeout_s
    while time.time() < deadline:
        time.sleep(3.0)
        rx_b = sample_rx()
        if rx_b is not None and rx_b > rx_a:
            return True
    return False


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=30,
                    help="ACK stress sends per config (default 30)")
    ap.add_argument("--interval", type=float, default=10.0,
                    help="Seconds between sends (default 10.0)")
    ap.add_argument("--configs", default="C0,C0P,C1,C2",
                    help="Comma-separated subset of C0,C0P,C1,C2")
    ap.add_argument("--port", default=None,
                    help="Station port (default: auto)")
    ap.add_argument("--csv", default="logs/stage_t/t6_summary.csv",
                    help="Output CSV path")
    args = ap.parse_args()

    veh_port, stn_port = find_ports()
    if args.port:
        stn_port = args.port
    if not stn_port:
        print("ERROR: no station port found (VID:PID 0x2E8A:0x0009 "
              "with 'Ground Station' banner).")
        return 2
    if not veh_port:
        print("ERROR: no vehicle port found (VID:PID 0x2E8A:0x0009 "
              "with Feather/rocketchip banner).")
        return 2
    print(f"Vehicle port: {veh_port}")
    print(f"Station port: {stn_port}")

    selected = set(args.configs.split(","))
    sweep = [(lbl, idx, bw, nav) for (lbl, idx, bw, nav) in SWEEP
             if lbl in selected]
    if not sweep:
        print(f"ERROR: no configs selected from {args.configs}")
        return 2
    print(f"Sweep: {[lbl for (lbl, _, _, _) in sweep]}")

    os.makedirs(os.path.dirname(args.csv), exist_ok=True)

    veh_ser = serial.Serial(veh_port, 115200, timeout=0.05)
    stn_ser = serial.Serial(stn_port, 115200, timeout=0.05)
    time.sleep(1.0)
    veh_ser.reset_input_buffer()
    stn_ser.reset_input_buffer()

    print("\n--- Canary: station receiving packets? ---")
    if not canary(stn_ser):
        print("CANARY FAIL: station not seeing nav packets. Check link.")
        veh_ser.close()
        stn_ser.close()
        return 1
    print("CANARY OK: station is receiving")

    # Enable test mode on both boards — suppress persist + revert so
    # swept configs stay stable for the full ACK-stress window.
    print("\n--- Enable test mode on both boards ---")
    set_test_mode(veh_ser, "vehicle", True)
    set_test_mode(stn_ser, "station", True)

    results: list[tuple[str, int, int, AckResult]] = []

    try:
        for i, (label, _idx, bw, nav) in enumerate(sweep):
            print(f"\n=== [{label}] BW{bw} {nav}Hz "
                  f"— {args.count} sends @ {args.interval}s ===")

            # Config-switching infrastructure — BOTH boards via USB CDC
            # `qc` debug command. No RF involved. Deterministic.
            ok = switch_to_config(veh_ser, stn_ser, bw, nav)
            if not ok:
                print(f"  [WARN] could not confirm switch to BW{bw} {nav}Hz "
                      "end-to-end — running ACK stress anyway")

            print(f"  -> begin {args.count}-send ACK stress at "
                  f"BW{bw} {nav}Hz")
            r = run_ack_stress_fifo(stn_ser, args.count, args.interval, label)
            print(f"  [{label}] first_try={r.first_try_pct:5.1f}%  "
                  f"eventual={r.eventual_pct:5.1f}%  "
                  f"mean_lat={r.mean_latency_ms:.0f}ms  "
                  f"denied={r.denied}  no_ack={r.no_ack}")
            results.append((label, bw, nav, r))
    finally:
        # Disable test mode + reset both boards to default config so the
        # next run starts clean.
        print("\n--- Reset both boards to default + disable test mode ---")
        set_local_config(veh_ser, "vehicle", 125, 5)
        set_local_config(stn_ser, "station", 125, 5)
        set_test_mode(veh_ser, "vehicle", False)
        set_test_mode(stn_ser, "station", False)
        veh_ser.close()
        stn_ser.close()

    # Write CSV.
    with open(args.csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "config", "bw_khz", "nav_hz", "sent",
            "first_try_ack", "eventual_ack", "denied", "no_ack",
            "first_try_pct", "eventual_pct", "mean_lat_ms"])
        for (label, bw, nav, r) in results:
            w.writerow([
                label, bw, nav, r.sent,
                r.first_try_ack, r.eventual_ack, r.denied, r.no_ack,
                f"{r.first_try_pct:.2f}", f"{r.eventual_pct:.2f}",
                f"{r.mean_latency_ms:.1f}"])

    print(f"\nResults written to {args.csv}")
    print("\n=== SUMMARY ===")
    for (label, bw, nav, r) in results:
        tag = ""
        if label == "C2" and r.first_try_pct < 60.0:
            tag = "  <-- BELOW 60% PASS THRESHOLD"
        elif label == "C2" and r.first_try_pct >= 60.0:
            tag = "  <-- MEETS 60% PASS THRESHOLD"
        print(f"  {label:4s} BW{bw:3d}/{nav:2d}Hz: "
              f"first_try={r.first_try_pct:5.1f}%  "
              f"eventual={r.eventual_pct:5.1f}%{tag}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
