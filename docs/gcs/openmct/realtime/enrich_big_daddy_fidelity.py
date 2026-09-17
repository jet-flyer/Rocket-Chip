#!/usr/bin/env python3
"""Enrich Big Daddy glance CSV with full-fidelity glass columns (1:1 names for oMCT)."""
from __future__ import annotations

import csv
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]  # docs/gcs/openmct
SRC = ROOT / "fixtures" / "omct_big_daddy_glance.csv"
OUT = ROOT / "fixtures" / "omct_big_daddy_fidelity.csv"

EXTRA = [
    "rssi",
    "snr",
    "baro",
    "accel_g",
    "q_w",
    "q_x",
    "q_y",
    "q_z",
    "imu_temp_c",
    "baro_temp_c",
    "die_temp_c",
    "gps_fix",
    "phase_event",
]


def load_rows(path: Path) -> tuple[list[str], list[dict]]:
    lines = [L for L in path.read_text(encoding="utf-8").splitlines() if L.strip() and not L.strip().startswith("#")]
    r = csv.DictReader(lines)
    rows = list(r)
    return list(r.fieldnames or []), rows


def quat_for_phase(name: str, met_s: float) -> tuple[float, float, float, float]:
    # Simple unit quat: pad identity; boost nose-up pitch; coast hold; descent mild.
    name = (name or "").upper()
    if name in ("ARMED", "LANDED"):
        return 1.0, 0.0, 0.0, 0.0
    if name == "BOOST":
        # pitch ~ +20 deg about Y
        a = math.radians(20.0)
        return math.cos(a / 2), 0.0, math.sin(a / 2), 0.0
    if name == "COAST":
        a = math.radians(5.0)
        return math.cos(a / 2), 0.0, math.sin(a / 2), 0.0
    # DESCENT
    a = math.radians(-10.0)
    return math.cos(a / 2), 0.0, math.sin(a / 2), 0.0


def main() -> None:
    src, out = SRC, OUT
    if not src.exists():
        raise SystemExit(f"missing {src}")
    header, rows = load_rows(src)
    prev_v = None
    prev_met = None
    prev_state = None
    for row in rows:
        met_ms = float(row.get("met_ms") or 0)
        met_s = met_ms / 1000.0
        vvel = float(row.get("vvel_mps") or 0)
        if prev_v is None or prev_met is None:
            accel_g = 0.0
        else:
            dt = max(1e-3, (met_ms - prev_met) / 1000.0)
            accel_g = ((vvel - prev_v) / dt) / 9.80665
        prev_v, prev_met = vvel, met_ms

        state = row.get("flight_state_name") or ""
        phase_event = 1 if state != prev_state else 0
        prev_state = state

        temp = float(row.get("temp_c") or 22)
        sats = int(float(row.get("gps_sats") or 0))
        # fix quality heuristic: 0 none, 1 dead-reckoning-ish, 2 2D, 3 3D
        if sats >= 6:
            gps_fix = 3
        elif sats >= 4:
            gps_fix = 2
        elif sats > 0:
            gps_fix = 1
        else:
            gps_fix = 0

        qw, qx, qy, qz = quat_for_phase(state, met_s)
        row["rssi"] = row.get("rssi_dbm") or ""
        row["snr"] = row.get("snr_db") or ""
        row["baro"] = row.get("baro_alt_m") or ""
        row["accel_g"] = f"{accel_g:.4f}"
        row["q_w"], row["q_x"], row["q_y"], row["q_z"] = (f"{qw:.5f}", f"{qx:.5f}", f"{qy:.5f}", f"{qz:.5f}")
        row["imu_temp_c"] = f"{temp + 1.5:.2f}"
        row["baro_temp_c"] = f"{temp:.2f}"
        row["die_temp_c"] = f"{temp + 8.0:.2f}"
        row["gps_fix"] = str(gps_fix)
        row["phase_event"] = str(phase_event)

    fieldnames = header + [c for c in EXTRA if c not in header]
    out.parent.mkdir(parents=True, exist_ok=True)
    preamble = [
        "# Rocket Chip oMCT full-fidelity facsimile (enriched from Big Daddy glance)",
        "# Source: omct_big_daddy_glance.csv + derived accel_g/quat/temps/gps_fix/phase_event",
        "# Vehicle-only fields (quat/IMU/temps/Vbatt) included for glass layout even when USB/m cannot scrape them yet",
    ]
    with out.open("w", encoding="utf-8", newline="\n") as f:
        for line in preamble:
            f.write(line + "\n")
        w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        w.writeheader()
        for row in rows:
            w.writerow({k: row.get(k, "") for k in fieldnames})
    print(f"wrote {out} rows={len(rows)} cols={len(fieldnames)}")


if __name__ == "__main__":
    main()
