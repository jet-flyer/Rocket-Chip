#!/usr/bin/env python3
"""Stage T IVP-T4: ambient RSSI baseline (simplified, no firmware change).

Host-side: poll station's 't' radio status command at 1 Hz for N seconds.
Station emits last_rx_rssi which reflects the most recent received packet.
When vehicle is unplugged, the station receives no packets, so last_rx_rssi
stays stale — but the station's 'Pkts' counter tells us if any RF activity
was picked up (ambient interference would produce CRC-fail or phantom RXs).

This is a REDUCED T4 from the original plan. Full CAD-mode sweep deferred
because it requires firmware changes (same class that broke T2). The
ambient-interference question is answered by "did the station register any
packets when vehicle was off?"

Protocol:
  Operator: unplug vehicle BEFORE running this script.
  Script: polls station for 60 s, records pkts delta every second.
  If pkts delta > 0 anywhere → interference on the band.
  If pkts stays fixed → ambient floor below SX1276 sensitivity threshold
    (~-130 dBm at SF7/BW125), no interference concern.

Usage:
  python scripts/stage_t4_ambient.py [--duration 60] [--port COM9]
  ``--port`` may be omitted for RocketChip station auto-detect (VID:PID + banner).
"""
import argparse
import csv
import os
import re
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial required", file=sys.stderr)
    sys.exit(1)

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)
from _rc_test_common import (  # noqa: E402
    Banner,
    find_target_port,
    open_classified_port,
    rc_test,
    TARGET_STATION_ANY,
)


RE_PKTS = re.compile(r"Pkts:\s*(\d+)")
RE_RSSI = re.compile(r"RSSI:\s*(?:\x1b\[\d+m)?(-?\d+)\s*dBm")
RE_CRC = re.compile(r"CRC err:\s*(\d+)")
RE_RXPKT = re.compile(r"RX:\s*(\d+)\s*pkts")


def poll_station(ser) -> dict:
    """One-shot poll: send 't', read dashboard, return parsed counters."""
    time.sleep(0.2)
    try:
        ser.reset_input_buffer()
    except Exception:
        pass
    ser.write(b"t")
    time.sleep(1.0)
    data = ser.read(8000).decode("utf-8", errors="replace")

    result = {"pkts": None, "rssi": None, "crc_err": None, "rx_pkts": None}
    for line in data.split("\n"):
        m = RE_PKTS.search(line)
        if m and result["pkts"] is None:
            result["pkts"] = int(m.group(1))
        m = RE_RSSI.search(line)
        if m and result["rssi"] is None:
            result["rssi"] = int(m.group(1))
        m = RE_CRC.search(line)
        if m and result["crc_err"] is None:
            result["crc_err"] = int(m.group(1))
        m = RE_RXPKT.search(line)
        if m and result["rx_pkts"] is None:
            result["rx_pkts"] = int(m.group(1))
    return result


@rc_test(target=TARGET_STATION_ANY)
def main():
    ap = argparse.ArgumentParser(description="Stage T IVP-T4 ambient RSSI")
    ap.add_argument("--port", default=None)
    ap.add_argument("--duration", type=int, default=60,
                    help="Seconds to poll (default 60)")
    ap.add_argument("--csv", default="logs/stage_t/t4_ambient.csv")
    args = ap.parse_args()

    os.makedirs(os.path.dirname(args.csv) or ".", exist_ok=True)

    port_name, meta = find_target_port(
        TARGET_STATION_ANY, override=args.port, verbose=False)
    if port_name is None:
        print(f'INFO: no station port — {meta}', file=sys.stderr)
        print('  Plug station FW or pass --port.', file=sys.stderr)
        sys.exit(2)
    if not isinstance(meta, Banner):
        print('ERROR: internal: expected Banner', file=sys.stderr)
        sys.exit(2)

    print("=== Stage T IVP-T4 — ambient RSSI / interference check ===")
    print("")
    print("PROTOCOL: vehicle must be UNPLUGGED or powered OFF for this test.")
    print(f"          Station {port_name} ({meta.short_summary()}) polled at 1 Hz "
          f"for {args.duration}s.")
    print("")
    print("Baseline sample before test:")

    try:
        with open_classified_port(
            port_name,
            target=TARGET_STATION_ANY,
            timeout=0.3,
            auto_enter_cli_menu=False,
        ) as ser:
            baseline = poll_station(ser)
            print(f"  pkts={baseline['pkts']}  rx_pkts={baseline['rx_pkts']}  "
                  f"crc_err={baseline['crc_err']}  rssi={baseline['rssi']}")
            print("")
            print(f"Now UNPLUG the vehicle and wait 2 seconds, then sampling starts...")
            time.sleep(5)
            print(f"Starting sample loop ({args.duration}s)...")

            samples = []
            start = time.time()
            last_pkts = None
            while time.time() - start < args.duration:
                t_rel = time.time() - start
                r = poll_station(ser)
                if r["pkts"] is None:
                    print(f"  [t={t_rel:5.1f}s] parse fail, retrying")
                    continue
                delta = 0 if last_pkts is None else r["pkts"] - last_pkts
                samples.append({
                    "t_rel": t_rel,
                    "pkts": r["pkts"],
                    "rx_pkts": r["rx_pkts"],
                    "crc_err": r["crc_err"],
                    "rssi": r["rssi"],
                    "delta_pkts": delta,
                })
                marker = " [NEW PACKET]" if delta > 0 else ""
                print(f"  [t={t_rel:5.1f}s] pkts={r['pkts']}  crc_err={r['crc_err']}  "
                      f"rssi={r['rssi']}dBm  delta={delta}{marker}")
                last_pkts = r["pkts"]
                time.sleep(1.0)
    except RuntimeError as e:
        print(f'ERROR: cannot open {port_name}: {e}', file=sys.stderr)
        sys.exit(2)

    # Final analysis — handle None crc_err (dashboard mode doesn't emit it as separate line)
    total_new_pkts = samples[-1]["pkts"] - (samples[0]["pkts"] - samples[0]["delta_pkts"])
    if samples[-1]["crc_err"] is not None and samples[0]["crc_err"] is not None:
        total_new_crc = samples[-1]["crc_err"] - samples[0]["crc_err"]
    else:
        total_new_crc = 0  # not reported by station in dashboard mode
        print("  (CRC-err not exposed in dashboard output; "
              "treating as 0 for test purposes)")
    print()
    print("=" * 60)
    print("RESULTS — T4 ambient RSSI/interference")
    print("=" * 60)
    print(f"Duration:        {args.duration} s")
    print(f"New Pkts seen:   {total_new_pkts}  "
          f"(expected 0 if vehicle off and no interference)")
    print(f"New CRC errors:  {total_new_crc}  "
          f"(expected 0 if no interfering Tx'ers on band)")
    if total_new_pkts > 0 or total_new_crc > 0:
        print(f"  [WARN] Non-zero RX activity with vehicle unplugged.")
        print(f"         Possible interference source on 915 MHz.")
    else:
        print(f"  [OK] No ambient interference detected at the station's")
        print(f"       sensitivity threshold (~-130 dBm, SF7/BW125).")
    print("=" * 60)

    # Save CSV
    with open(args.csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_rel", "pkts", "rx_pkts", "crc_err", "rssi", "delta_pkts"])
        for s in samples:
            w.writerow([f"{s['t_rel']:.2f}", s["pkts"], s["rx_pkts"],
                        s["crc_err"], s["rssi"], s["delta_pkts"]])
    print(f"CSV: {args.csv}")


if __name__ == "__main__":
    main()
