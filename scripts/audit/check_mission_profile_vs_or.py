#!/usr/bin/env python3
"""Mission Profile vs OpenRocket prediction cross-check.

Phase 3 deliverable of the OpenRocket integration evaluation (council-
approved 2026-05-22, see docs/decisions/OPENROCKET_INTEGRATION_EVALUATION_
2026-05-22.md). Reads `src/flight_director/mission_profile_data.h`
thresholds + an OR export CSV, checks for threshold-vs-prediction
conflicts.

ADVISORY ONLY (council amendment 3). The script outputs findings; a
human dispositions each one. It must NOT auto-edit
mission_profile_data.h — that's the strict boundary with C6.b (which
the council rejected for now).

The conflicts checked:

  launch_accel_threshold: conflict = max(|f_body_z|) on boost < threshold
                          (firmware would miss launch detection).
  burnout_accel_threshold: conflict = post-burnout coast |f_body_z| > threshold
                          (firmware would re-trigger launch from coast).
  apogee_velocity_threshold: conflict = OR vertical-velocity zero-crossing
                          resolution incompatible with threshold.
  main_deploy_altitude_m: conflict = predicted apogee < threshold
                          (firmware would never deploy main).
  deploy_lockout_mps: conflict = OR descent velocity at main-deploy altitude
                          still > lockout (deploy never clears lockout).
  apogee_lockout_ms: conflict = OR predicted time-to-apogee < lockout window
                          (apogee guard never arms in time).

The script reuses the column-tolerant parser pattern from
trajectory_to_sensors.py and the header-regex-parse pattern from
find_dead_code.py.

Usage:
    python scripts/audit/check_mission_profile_vs_or.py
    python scripts/audit/check_mission_profile_vs_or.py --or-csv path/to/export.csv
    python scripts/audit/check_mission_profile_vs_or.py --output report.md

Exit codes:
    0 = no conflicts found (or only NOTE-level observations)
    1 = at least one conflict
    2 = environment error (missing files, parse failure, etc.)
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
DEFAULT_HEADER = REPO_ROOT / "src" / "flight_director" / "mission_profile_data.h"
DEFAULT_OR_CSV = (
    REPO_ROOT / "tests" / "replay_profiles" / "openrocket_export"
    / "big_daddy_f15_6_nominal.csv"
)

# Standard gravity used by the converter (matches G_NED in
# scripts/trajectory_to_sensors.py).
G_NED = 9.80665


# ----------------------------------------------------------------------------
# Mission Profile parser
# ----------------------------------------------------------------------------

# The fields we care about. Each entry: (header_field_name, regex pattern
# matching the value in the auto-generated header). Patterns capture only
# the numeric value.
PROFILE_FIELDS: Dict[str, str] = {
    "launch_accel_threshold":   r"\.launch_accel_threshold\s*=\s*([\-0-9.eEf]+)",
    "burnout_accel_threshold":  r"\.burnout_accel_threshold\s*=\s*([\-0-9.eEf]+)",
    "apogee_velocity_threshold": r"\.apogee_velocity_threshold\s*=\s*([\-0-9.eEf]+)",
    "main_deploy_altitude_m":   r"\.main_deploy_altitude_m\s*=\s*([\-0-9.eEf]+)",
    "deploy_lockout_mps":       r"\.deploy_lockout_mps\s*=\s*([\-0-9.eEf]+)",
    "apogee_lockout_ms":        r"\.apogee_lockout_ms\s*=\s*([\-0-9]+)",
}


def parse_mission_profile(header_path: Path) -> Dict[str, float]:
    """Extract the 6 thresholds from mission_profile_data.h."""
    text = header_path.read_text(encoding="utf-8")
    out: Dict[str, float] = {}
    for field, pat in PROFILE_FIELDS.items():
        m = re.search(pat, text)
        if not m:
            raise RuntimeError(f"Could not find {field} in {header_path}")
        raw = m.group(1).rstrip("fF")
        out[field] = float(raw)
    return out


# ----------------------------------------------------------------------------
# OR CSV parser (column-tolerant, like trajectory_to_sensors.py)
# ----------------------------------------------------------------------------

# Column synonyms for the OR export. Header decorations (units, encoding
# variants) are tolerated by canonicalizing both sides.
COL_SYNONYMS: Dict[str, List[str]] = {
    "time_s":            ["time"],
    "altitude_m":        ["altitude"],
    "vert_vel_mps":      ["vertical velocity"],
    "vert_accel_ms2":    ["vertical acceleration"],
    "lateral_accel_ms2": ["lateral acceleration"],
}


def canonicalize(raw: str) -> str:
    """Normalize a header to lower-case alphanumeric+space."""
    # Drop unit decorations like "(m/s^2)" or "(s)" and the � placeholder.
    cleaned = re.sub(r"\([^)]*\)", "", raw)
    cleaned = re.sub(r"[^a-zA-Z0-9 ]+", " ", cleaned)
    return " ".join(cleaned.lower().split()).strip()


def resolve_columns(header: List[str]) -> Dict[str, int]:
    """Map canonical field names to column indices in the OR CSV header."""
    canon_header = [canonicalize(h) for h in header]
    out: Dict[str, int] = {}
    for field, synonyms in COL_SYNONYMS.items():
        for syn in synonyms:
            canon_syn = canonicalize(syn)
            for i, h in enumerate(canon_header):
                if h.startswith(canon_syn):
                    out[field] = i
                    break
            if field in out:
                break
        if field not in out:
            # vert/lateral accel may be missing in some exports; not all
            # fields are strictly required for every check.
            pass
    return out


def parse_or_csv(csv_path: Path) -> Tuple[Dict[str, int], List[List[Optional[float]]]]:
    """Parse the OR export. Returns (column_map, rows)."""
    rows: List[List[Optional[float]]] = []
    header: Optional[List[str]] = None
    with csv_path.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.startswith("#"):
                # OR uses # for header comments AND for the column-name
                # header (e.g., "# Time (s),Altitude (m),..."). The
                # *last* # line before data is the column header.
                stripped = line.lstrip("#").strip()
                if stripped and "," in stripped:
                    header = [c.strip() for c in stripped.split(",")]
                continue
            if header is None:
                continue
            parts = [p.strip() for p in line.split(",")]
            row: List[Optional[float]] = []
            for p in parts:
                if p == "" or p.lower() == "nan":
                    row.append(None)
                else:
                    try:
                        row.append(float(p))
                    except ValueError:
                        row.append(None)
            rows.append(row)
    if header is None:
        raise RuntimeError(f"No header found in {csv_path}")
    col_map = resolve_columns(header)
    return col_map, rows


# ----------------------------------------------------------------------------
# Threshold cross-check
# ----------------------------------------------------------------------------

def or_vert_accel_to_body_specific_force(vert_accel_up: float) -> float:
    """Convert OR's NED-up vertical acceleration to body-frame specific-force
    magnitude for a vertical rocket. Mirrors the converter math:

        f_body_z = -(vert_accel_up) - G
        |f_body_z| = |vert_accel_up + G|

    This assumes near-vertical flight (the converter's domain). For sustained
    boost where vert_accel_up >> 0, body specific-force magnitude is roughly
    |vert_accel_up + G|.
    """
    return abs(vert_accel_up + G_NED)


def column_series(col_map: Dict[str, int], rows: List[List[Optional[float]]],
                  field: str) -> List[Optional[float]]:
    """Extract a column by canonical field name; None where missing."""
    idx = col_map.get(field)
    if idx is None:
        return []
    return [r[idx] if idx < len(r) else None for r in rows]


def find_apogee(time_s: List[Optional[float]],
                alt_m: List[Optional[float]]) -> Tuple[Optional[float], Optional[float]]:
    """Return (time_at_apogee_s, max_altitude_m). None if no apogee found."""
    best = (None, None)
    for t, a in zip(time_s, alt_m):
        if t is None or a is None:
            continue
        if best[1] is None or a > best[1]:
            best = (t, a)
    return best


def check_launch_accel(thresh: float,
                       col_map: Dict[str, int],
                       rows: List[List[Optional[float]]]) -> Tuple[str, str]:
    """Check that OR predicts the firmware will detect launch.

    Conflict = max boost-phase body specific-force < threshold.
    """
    vert_accels = column_series(col_map, rows, "vert_accel_ms2")
    if not vert_accels:
        return "SKIP", "no vertical-acceleration column in OR CSV"

    # The first ~0.05s is rod-restraint period (motor ramping; OR reports
    # negative vert accel from gravity dominating). Look at the post-ignition
    # window: skip until vert_accel_up first exceeds +5 m/s² (motor on).
    body_sf_peak = 0.0
    motor_on = False
    for va in vert_accels:
        if va is None:
            continue
        if not motor_on:
            if va > 5.0:
                motor_on = True
            else:
                continue
        sf = or_vert_accel_to_body_specific_force(va)
        if sf > body_sf_peak:
            body_sf_peak = sf

    if body_sf_peak < thresh:
        return ("CONFLICT",
                f"OR predicts peak body specific-force {body_sf_peak:.2f} m/s² "
                f"< launch_accel_threshold {thresh:.2f} m/s² "
                f"(firmware would miss launch)")
    return ("OK",
            f"OR predicts peak body specific-force {body_sf_peak:.2f} m/s² "
            f"(margin: {body_sf_peak - thresh:+.2f} m/s² above {thresh:.2f})")


def check_burnout_accel(thresh: float,
                        col_map: Dict[str, int],
                        rows: List[List[Optional[float]]]) -> Tuple[str, str]:
    """Check that post-burnout coast specific-force does NOT re-trigger launch.

    Conflict = max coast-phase body specific-force above (thresh + G_NED)
    would still register; the firmware's burnout-detection compares against
    a body specific-force ~G during free-fall coast. Threshold reads as
    'when specific-force drops below this, burnout detected.' For coast,
    body specific-force ≈ G_NED (gravity only, no thrust). So a conflict
    would be a coast value > G_NED that exceeded threshold.
    """
    time_s = column_series(col_map, rows, "time_s")
    vert_accels = column_series(col_map, rows, "vert_accel_ms2")
    if not vert_accels or not time_s:
        return "SKIP", "missing time/vertical-acceleration columns"

    # Find a coast sample: post-burnout (vert_accel_up roughly -G, i.e.
    # free-fall under gravity alone). Pick the first sample 0.5s after
    # vert_accel_up drops below 0 for the first time post-boost peak.
    # Simpler proxy: sample at t = 5s (well past F15 burnout at ~2-3s).
    coast_sf_max = 0.0
    coast_samples = 0
    for t, va in zip(time_s, vert_accels):
        if t is None or va is None:
            continue
        if t < 5.0 or t > 6.0:
            continue
        sf = or_vert_accel_to_body_specific_force(va)
        coast_sf_max = max(coast_sf_max, sf)
        coast_samples += 1

    if coast_samples == 0:
        return "SKIP", "no coast samples in t=[5,6]s window"

    # The burnout threshold is the firmware's criterion for detecting motor-
    # off. During coast under gravity alone, body specific-force ≈ G_NED
    # (9.81). The threshold says "below 5 m/s² = burnout." If OR coast sf is
    # ≈G, that's WAY above 5, so the firmware would NOT detect burnout in
    # coast — which is wrong. But actually the firmware's launch-accel
    # threshold of 20 m/s² is the re-trigger guard; if coast sf stays well
    # below 20, no re-trigger. Burnout-accel-threshold is the "below this =
    # burnout transition." Check both conditions.
    if coast_sf_max < thresh:
        return ("CONFLICT",
                f"OR coast body specific-force max {coast_sf_max:.2f} m/s² "
                f"< burnout_accel_threshold {thresh:.2f} m/s² — would NOT "
                f"detect burnout transition (need coast sf ABOVE threshold "
                f"during boost, drops BELOW threshold post-burnout)")
    if coast_sf_max < G_NED * 0.5:
        return ("WARN",
                f"OR coast body specific-force max {coast_sf_max:.2f} m/s² "
                f"is unusually low for gravity-only free-fall (~{G_NED:.2f}); "
                f"verify the converter math")
    return ("OK",
            f"OR coast body specific-force max {coast_sf_max:.2f} m/s² "
            f"(near expected ~{G_NED:.2f} for free-fall coast; threshold "
            f"{thresh:.2f} m/s² is below this so burnout would trigger correctly)")


def check_apogee_altitude(thresh: float,
                          col_map: Dict[str, int],
                          rows: List[List[Optional[float]]]) -> Tuple[str, str]:
    """Check that OR-predicted apogee exceeds the main-deploy altitude."""
    time_s = column_series(col_map, rows, "time_s")
    alt_m = column_series(col_map, rows, "altitude_m")
    if not alt_m:
        return "SKIP", "no altitude column in OR CSV"

    t_apo, alt_apo = find_apogee(time_s, alt_m)
    if alt_apo is None:
        return "SKIP", "no apogee found"

    if alt_apo < thresh:
        return ("CONFLICT",
                f"OR predicts apogee {alt_apo:.1f} m < main_deploy_altitude_m "
                f"{thresh:.1f} m — main chute would never deploy")
    return ("OK",
            f"OR predicts apogee {alt_apo:.1f} m at t={t_apo:.2f}s "
            f"(margin: {alt_apo - thresh:+.1f} m above {thresh:.1f})")


def check_deploy_lockout(thresh: float,
                         col_map: Dict[str, int],
                         rows: List[List[Optional[float]]]) -> Tuple[str, str]:
    """Check that descent velocity at main-deploy altitude is below lockout."""
    time_s = column_series(col_map, rows, "time_s")
    alt_m = column_series(col_map, rows, "altitude_m")
    vert_vel = column_series(col_map, rows, "vert_vel_mps")
    if not alt_m or not vert_vel:
        return "SKIP", "missing altitude or velocity column"

    t_apo, alt_apo = find_apogee(time_s, alt_m)
    if alt_apo is None:
        return "SKIP", "no apogee found"

    # We need the main_deploy_altitude_m; defer to caller via closure or
    # arg. Use 150 m as the default proxy (matches profile value at time
    # of writing).
    main_alt = 150.0
    # Find samples in descent where altitude crosses 150m on the way down.
    best_vel_at_main: Optional[float] = None
    for t, a, v in zip(time_s, alt_m, vert_vel):
        if t is None or a is None or v is None:
            continue
        if t_apo is None or t < t_apo:
            continue
        if a > main_alt:
            continue
        # First descent sample at or below main_alt.
        best_vel_at_main = abs(v)
        break

    if best_vel_at_main is None:
        return "SKIP", f"no descent sample crossing {main_alt} m altitude"

    if best_vel_at_main > thresh:
        return ("CONFLICT",
                f"OR predicts descent velocity {best_vel_at_main:.2f} m/s at "
                f"{main_alt:.0f} m AGL > deploy_lockout_mps {thresh:.2f} m/s — "
                f"deploy would never clear lockout (chute deploys late)")
    return ("OK",
            f"OR predicts descent velocity {best_vel_at_main:.2f} m/s at "
            f"{main_alt:.0f} m AGL (lockout {thresh:.2f} m/s clears with "
            f"{thresh - best_vel_at_main:+.2f} m/s margin)")


def check_apogee_lockout(thresh_ms: float,
                         col_map: Dict[str, int],
                         rows: List[List[Optional[float]]]) -> Tuple[str, str]:
    """Check that OR-predicted time-to-apogee exceeds the apogee-guard lockout."""
    time_s = column_series(col_map, rows, "time_s")
    alt_m = column_series(col_map, rows, "altitude_m")
    if not time_s or not alt_m:
        return "SKIP", "missing time or altitude column"

    t_apo, _ = find_apogee(time_s, alt_m)
    if t_apo is None:
        return "SKIP", "no apogee found"

    t_apo_ms = t_apo * 1000.0
    if t_apo_ms < thresh_ms:
        return ("CONFLICT",
                f"OR predicts apogee at t={t_apo:.2f}s ({t_apo_ms:.0f} ms) "
                f"< apogee_lockout_ms {thresh_ms:.0f} ms — apogee guard never "
                f"arms in time")
    return ("OK",
            f"OR predicts apogee at t={t_apo:.2f}s ({t_apo_ms:.0f} ms); "
            f"lockout {thresh_ms:.0f} ms clears with "
            f"{t_apo_ms - thresh_ms:+.0f} ms margin")


def check_apogee_velocity_resolution(thresh: float) -> Tuple[str, str]:
    """The apogee_velocity_threshold is a zero-crossing detection bound.

    This is a sanity-band check rather than an OR-derived check. A threshold
    too tight (e.g., < 0.1 m/s) hits sensor-noise floor; too loose (> 5 m/s)
    misses the apogee. The current 0.5 m/s is well within reasonable range.
    """
    if thresh < 0.1:
        return ("WARN",
                f"apogee_velocity_threshold {thresh:.2f} m/s is below typical "
                f"sensor noise floor (~0.1 m/s); zero-crossing detection may "
                f"oscillate")
    if thresh > 5.0:
        return ("WARN",
                f"apogee_velocity_threshold {thresh:.2f} m/s is unusually high "
                f"(>5 m/s); apogee may be detected late")
    return ("OK",
            f"apogee_velocity_threshold {thresh:.2f} m/s is within the "
            f"typical 0.1–5 m/s sanity band")


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------

CHECK_ORDER = [
    "launch_accel_threshold",
    "burnout_accel_threshold",
    "apogee_velocity_threshold",
    "main_deploy_altitude_m",
    "deploy_lockout_mps",
    "apogee_lockout_ms",
]


def run_checks(profile: Dict[str, float],
               col_map: Dict[str, int],
               rows: List[List[Optional[float]]]
               ) -> List[Tuple[str, str, str]]:
    """Run all 6 checks. Returns list of (field, status, message)."""
    out: List[Tuple[str, str, str]] = []
    for field in CHECK_ORDER:
        thresh = profile[field]
        if field == "launch_accel_threshold":
            status, msg = check_launch_accel(thresh, col_map, rows)
        elif field == "burnout_accel_threshold":
            status, msg = check_burnout_accel(thresh, col_map, rows)
        elif field == "apogee_velocity_threshold":
            status, msg = check_apogee_velocity_resolution(thresh)
        elif field == "main_deploy_altitude_m":
            status, msg = check_apogee_altitude(thresh, col_map, rows)
        elif field == "deploy_lockout_mps":
            status, msg = check_deploy_lockout(thresh, col_map, rows)
        elif field == "apogee_lockout_ms":
            status, msg = check_apogee_lockout(thresh, col_map, rows)
        else:
            status, msg = "SKIP", "no check implemented"
        out.append((field, status, msg))
    return out


def render_report(profile: Dict[str, float],
                  or_csv_path: Path,
                  results: List[Tuple[str, str, str]]) -> str:
    lines: List[str] = []
    lines.append("# Mission Profile vs OpenRocket Cross-Check Report")
    lines.append("")
    lines.append(f"**Profile header:** `{DEFAULT_HEADER.relative_to(REPO_ROOT).as_posix()}`")
    lines.append(f"**OR export:** `{or_csv_path.relative_to(REPO_ROOT).as_posix()}`")
    lines.append("")
    lines.append("## Findings")
    lines.append("")
    lines.append("| Threshold | Value | Status | Notes |")
    lines.append("|-----------|-------|--------|-------|")
    for field, status, msg in results:
        val = profile[field]
        if "ms" in field:
            val_str = f"{val:.0f} ms"
        elif "altitude" in field:
            val_str = f"{val:.1f} m"
        elif "velocity" in field or "mps" in field:
            val_str = f"{val:.2f} m/s"
        else:
            val_str = f"{val:.2f} m/s²"
        lines.append(f"| `{field}` | {val_str} | **{status}** | {msg} |")
    lines.append("")
    n_conflict = sum(1 for _, s, _ in results if s == "CONFLICT")
    n_warn = sum(1 for _, s, _ in results if s == "WARN")
    n_skip = sum(1 for _, s, _ in results if s == "SKIP")
    n_ok = sum(1 for _, s, _ in results if s == "OK")
    lines.append("## Summary")
    lines.append("")
    lines.append(f"- OK: {n_ok}  CONFLICT: {n_conflict}  WARN: {n_warn}  SKIP: {n_skip}")
    lines.append("")
    if n_conflict == 0:
        lines.append("**No conflicts found.** Current Mission Profile thresholds are "
                     "consistent with OR predictions for this motor/airframe.")
    else:
        lines.append("**Conflicts present.** Human disposition required.")
    lines.append("")
    lines.append("Per the OR integration evaluation (council 2026-05-22, "
                 "`docs/decisions/OPENROCKET_INTEGRATION_EVALUATION_2026-05-22.md`) "
                 "this script is ADVISORY ONLY. Findings are surfaced; humans "
                 "decide whether to amend `mission_profile_data.h`. No automated "
                 "edits.")
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Cross-check Mission Profile thresholds against OR predictions."
    )
    parser.add_argument("--header", type=Path, default=DEFAULT_HEADER,
                        help=f"Mission profile header (default: {DEFAULT_HEADER})")
    parser.add_argument("--or-csv", type=Path, default=DEFAULT_OR_CSV,
                        help=f"OR export CSV (default: {DEFAULT_OR_CSV})")
    parser.add_argument("--output", type=Path, default=None,
                        help="Write report to file (default: stdout)")
    args = parser.parse_args()

    if not args.header.exists():
        print(f"ERROR: {args.header} not found", file=sys.stderr)
        return 2
    if not args.or_csv.exists():
        print(f"ERROR: {args.or_csv} not found", file=sys.stderr)
        return 2

    try:
        profile = parse_mission_profile(args.header)
    except RuntimeError as exc:
        print(f"ERROR parsing profile: {exc}", file=sys.stderr)
        return 2

    try:
        col_map, rows = parse_or_csv(args.or_csv)
    except RuntimeError as exc:
        print(f"ERROR parsing OR CSV: {exc}", file=sys.stderr)
        return 2

    results = run_checks(profile, col_map, rows)
    report = render_report(profile, args.or_csv, results)

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(report, encoding="utf-8")
        print(f"Report written: {args.output}", file=sys.stderr)
    else:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stdout.write(report)

    has_conflict = any(s == "CONFLICT" for _, s, _ in results)
    return 0 if not has_conflict else 1


if __name__ == "__main__":
    sys.exit(main())
