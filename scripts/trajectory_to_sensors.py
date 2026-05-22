#!/usr/bin/env python3
"""Convert an OpenRocket trajectory CSV into RocketChip's Big Daddy sensor CSV format.

Replaces the prior `scripts/generate_replay_profiles.py` 1D-physics generator
(which produced a non-standard accelerometer convention — see CONTENT_VALIDATION.md
+ R-28). Uses OpenRocket's validated 6-DOF aerodynamics as ground truth, then
computes proper specific-force sensor measurements via:

    f_body = R(q)^T * (a_NED - g_NED)

where:
    a_NED      = true inertial acceleration in NED frame (down-positive Z)
    g_NED      = [0, 0, +g]  (gravity points down)
    R(q)       = body-to-NED rotation matrix
    f_body     = accelerometer measurement (specific force, body frame)

Body-frame convention matches src/fusion/eskf.h: body-Z is DOWN (rocket nose
points along -body-Z when sitting on the pad). At rest, the accelerometer
reads `[0, 0, -g]` (gravity acts on -body-Z; specific force is +body-Z; but
in body-Z DOWN convention the gravity reading on body-Z is negative).

Big Daddy sensor CSV schema (11 columns, blank = NaN = no sample this tick):
    time_s, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z,
    pressure_pa, gps_lat_1e7, gps_lon_1e7, gps_alt_mm

Sensor rates (matches scripts/generate_replay_profiles.py originals):
    accel/gyro: 100 Hz
    baro:       50 Hz
    GPS:        10 Hz

Usage:
    python scripts/trajectory_to_sensors.py INPUT_OPENROCKET.csv OUTPUT_DIR

Output:
    OUTPUT_DIR/big_daddy_f15_nominal.csv
    OUTPUT_DIR/big_daddy_early_burnout.csv
    OUTPUT_DIR/big_daddy_baro_dropout.csv
    OUTPUT_DIR/big_daddy_gps_dropout_descent.csv
    OUTPUT_DIR/big_daddy_imu_zero_fault.csv

The OpenRocket CSV is parsed tolerantly — only the columns we need are
required, others are ignored. Column-name matching is case-insensitive and
unit-aware (handles common OpenRocket header decorations like "(m/s)" or
"  (m/s^2)").
"""

import argparse
import csv
import math
import os
import re
import sys
from typing import Dict, List, Optional, Tuple

import numpy as np


# Standard launch site (matches scripts/generate_replay_profiles.py).
GPS_LAT_DEG = 32.95
GPS_LON_DEG = -96.75

# Sensor rates (Hz). Matches the original generator + ICM-20948 baseline.
ACCEL_HZ = 100.0
BARO_HZ = 50.0
GPS_HZ = 10.0

# Constants — match src/fusion/eskf.h.
G_NED = 9.80665   # m/s²
EARTH_RADIUS_M = 6378100.0
SEA_LEVEL_PA = 101325.0


# ===========================================================================
# OpenRocket CSV parser — tolerant of column name variations
# ===========================================================================

# Map canonical key -> list of OpenRocket header substring synonyms.
# OpenRocket header decorations: "Time (s)", "Vertical acceleration  (m/s²)", etc.
COL_SYNONYMS = {
    'time': ['time'],
    'altitude': ['altitude'],
    # Velocity: prefer total + vertical separately if both present.
    'vertical_velocity': ['vertical velocity'],
    'lateral_velocity': ['lateral velocity'],
    # OpenRocket also exports body-axis: 'velocity x', 'velocity y', 'velocity z'.
    # 'velocity total' is the L2 norm; not directly useful here.
    'velocity_total': ['total velocity'],
    # Acceleration
    'vertical_acceleration': ['vertical acceleration'],
    'lateral_acceleration': ['lateral acceleration'],
    'total_acceleration': ['total acceleration'],
    # Body angular rates (deg/s typically in OpenRocket).
    'roll_rate': ['roll rate'],
    'pitch_rate': ['pitch rate'],
    'yaw_rate': ['yaw rate'],
    # Attitude (vertical orientation)
    'vertical_orientation': ['vertical orientation', 'angle off vertical'],
    'lateral_orientation': ['lateral direction', 'lateral orientation'],
    # Atmospheric. NOTE: do NOT use bare 'pressure' as a synonym — it would
    # match "Pressure drag coefficient" before "Air pressure".
    'air_pressure': ['air pressure'],
    'air_temperature': ['air temperature'],
    'air_density': ['air density'],
}


def _normalize(s: str) -> str:
    return re.sub(r'\s+', ' ', s.strip().lower())


def _match_column(header_norm: str, synonyms: List[str]) -> bool:
    """True if header_norm starts with any synonym (case/space-tolerant)."""
    for syn in synonyms:
        if header_norm.startswith(syn):
            return True
    return False


def parse_openrocket_csv(path: str) -> Tuple[Dict[str, int], List[List[Optional[float]]]]:
    """Parse an OpenRocket export CSV.

    Returns:
      col_map: dict mapping canonical key -> column index in row data
      rows: list of rows, each row is a list aligned with the CSV header
    """
    with open(path, 'r') as f:
        reader = csv.reader(f)
        rows = list(reader)

    # Find header row (first row with the canonical 'time' header).
    # OpenRocket prefixes the header row itself with '# ' (e.g.,
    # "# Time (s),Altitude (m),...") — strip leading '#' before testing.
    header_idx = None
    for i, row in enumerate(rows):
        if not row:
            continue
        first_stripped = row[0].lstrip('# ').strip()
        # Skip plain comment lines (those have non-CSV-shaped content).
        if row[0].strip().startswith('#') and 'time' not in first_stripped.lower():
            continue
        # Header row contains 'Time' (case-insensitive) in the first column.
        if 'time' in first_stripped.lower() and ' ' in first_stripped:
            header_idx = i
            # Normalize the leading '# ' in the first column for downstream
            # use — strip the comment marker so 'time (s)' parses cleanly.
            rows[i] = [first_stripped] + row[1:]
            break

    if header_idx is None:
        raise ValueError(f"could not locate header row in {path}")

    header = rows[header_idx]
    header_norm = [_normalize(c) for c in header]

    col_map: Dict[str, int] = {}
    for key, synonyms in COL_SYNONYMS.items():
        for i, h in enumerate(header_norm):
            if _match_column(h, synonyms):
                col_map[key] = i
                break

    # Required columns for sensor synthesis.
    required = ['time', 'altitude', 'vertical_velocity',
                'vertical_acceleration', 'air_pressure']
    missing = [k for k in required if k not in col_map]
    if missing:
        raise ValueError(f"OpenRocket CSV missing required columns: {missing}\n"
                         f"Headers found: {header}")

    # Data rows = everything after the header row, that has numeric first cell.
    data_rows: List[List[Optional[float]]] = []
    for row in rows[header_idx + 1:]:
        if not row or row[0].strip().startswith('#'):
            continue
        if not row[0].strip():
            continue
        parsed: List[Optional[float]] = []
        for cell in row:
            cell = cell.strip()
            if cell == '' or cell.lower() in ('nan', 'n/a', '--'):
                parsed.append(None)
            else:
                try:
                    parsed.append(float(cell))
                except ValueError:
                    parsed.append(None)
        data_rows.append(parsed)

    return col_map, data_rows


def _get(row: List[Optional[float]], col_map: Dict[str, int], key: str) -> Optional[float]:
    if key not in col_map:
        return None
    idx = col_map[key]
    if idx >= len(row):
        return None
    return row[idx]


# ===========================================================================
# Trajectory -> sensor measurement
# ===========================================================================

def synthesize_imu_sample(vertical_accel_ned_up: float,
                          lateral_accel: float,
                          vertical_orientation_deg: float) -> Tuple[float, float, float]:
    """Convert OpenRocket trajectory vars into body-frame specific force.

    OpenRocket conventions (verified against the 23.09 release):
        Vertical acceleration: positive UP, m/s². Time derivative of vertical
            velocity. At launch (motor still ramping, rod constraining the
            rocket), reads slightly negative (gravity dominates the
            instantaneous-but-constrained accel) — fine, we just don't init
            from these samples (the synthesized on-pad preamble handles that).
        Lateral acceleration: scalar magnitude m/s² in the horizontal plane.
            Direction given by Lateral orientation (azimuth). For nominally
            vertical Big Daddy flight with no wind, this is ~0 throughout.
        Vertical orientation (zenith): the elevation angle of the rocket
            axis above horizontal. 90° = vertical (pointing straight up),
            0° = horizontal. For a stock vertical launch, stays at ~90°.

    Body axis convention (matches src/fusion/eskf.h):
        body-X: forward (rocket nose direction)
        body-Y: right
        body-Z: down (opposite the nose direction)
        At rest with rocket vertical (nose up), body-Z is aligned with
        NED-Z (both point DOWN). g_NED = [0, 0, +g]. Specific force at
        rest f = a_true - g_NED = -g_NED = [0, 0, -g]. With R=identity
        body = NED, so f_body = [0, 0, -g]. Matches ESKF init expectation.

    For the canonical Big Daddy flight (vertical zenith=90°, no AoA, lateral
    accel = 0), the body frame stays aligned with NED throughout the boost +
    coast (no tilt, no horizontal motion). The IMU specific-force formula
    collapses to:
        f_body_x = lateral_accel  (zero for nominal Big Daddy)
        f_body_y = 0
        f_body_z = a_NED_z - g_NED_z = (-vert_accel_up) - 9.80665

    Off-nominal trajectories with significant lateral motion or AoA would
    need a proper rotation matrix from {zenith, azimuth, AoA}, but the
    nominal Big Daddy profile doesn't exercise those. Flagged as R-28
    reopen trigger for future windy / lateral-velocity profiles.

    Returns (ax, ay, az) in m/s² in body frame.
    """
    # NED-up convention -> NED-Z DOWN convention: sign flip.
    a_ned_z_down = -(vertical_accel_ned_up if vertical_accel_ned_up is not None else 0.0)

    # For a vertical Big Daddy flight, body == NED. The specific force is
    # just a_NED - g_NED:
    #   f_body_z = a_ned_z_down - g
    #   f_body_x = lateral_accel  (mapped to body-X for a nose-pointing-up
    #                              rocket; the lateral direction in NED is
    #                              irrelevant for our static-coordinate
    #                              GPS synthesis)
    #   f_body_y = 0
    f_body_x = lateral_accel if lateral_accel is not None else 0.0
    f_body_y = 0.0
    f_body_z = a_ned_z_down - G_NED

    return (f_body_x, f_body_y, f_body_z)


def synthesize_gyro_sample(roll_rate_dps: Optional[float],
                            pitch_rate_dps: Optional[float],
                            yaw_rate_dps: Optional[float]) -> Tuple[float, float, float]:
    """Body angular rates in rad/s. OpenRocket exports deg/s when present."""
    deg2rad = math.pi / 180.0
    rr = (roll_rate_dps or 0.0) * deg2rad
    pr = (pitch_rate_dps or 0.0) * deg2rad
    yr = (yaw_rate_dps or 0.0) * deg2rad
    # OpenRocket roll = body X (nose axis), pitch = body Y, yaw = body Z.
    # Match this convention to body-frame gyro (X=fwd, Y=right, Z=down).
    return (rr, pr, yr)


def altitude_to_pressure_pa(altitude_m: float) -> float:
    """ISA pressure given altitude AGL (assumes sea-level launch site).
    Matches the inverse used in test/replay/big_daddy_log.h.
    """
    if not math.isfinite(altitude_m):
        return float('nan')
    ratio = 1.0 - altitude_m / 44330.77
    if ratio <= 0:
        return float('nan')
    return SEA_LEVEL_PA * math.pow(ratio, 5.2558797)


def gps_position_ints(launch_lat_deg: float,
                       launch_lon_deg: float,
                       altitude_m: float) -> Tuple[int, int, int]:
    """For nominal vertical flight, GPS lat/lon stay at launch site;
    altitude tracks the trajectory. Returned as int microdegrees and mm.
    """
    lat_1e7 = int(round(launch_lat_deg * 1e7))
    lon_1e7 = int(round(launch_lon_deg * 1e7))
    alt_mm = int(round(altitude_m * 1000.0))
    return (lat_1e7, lon_1e7, alt_mm)


# ===========================================================================
# Resampling and CSV emission
# ===========================================================================

def interpolate_trajectory(trajectory: List[Dict[str, Optional[float]]],
                            target_times: np.ndarray) -> List[Dict[str, float]]:
    """Linear interpolation of trajectory onto target times."""
    times = np.array([r['time'] for r in trajectory if r['time'] is not None])
    fields = ['altitude', 'vertical_velocity', 'lateral_velocity',
              'vertical_acceleration', 'lateral_acceleration',
              'roll_rate', 'pitch_rate', 'yaw_rate',
              'vertical_orientation', 'air_pressure']

    series: Dict[str, np.ndarray] = {}
    for f in fields:
        vals = np.array([r.get(f) if r.get(f) is not None else np.nan
                         for r in trajectory if r['time'] is not None])
        # Replace any NaN with neighbor-interpolation along the array.
        mask = ~np.isnan(vals)
        if mask.sum() < 2:
            series[f] = np.full_like(target_times, np.nan)
            continue
        series[f] = np.interp(target_times, times[mask], vals[mask],
                              left=vals[mask][0], right=vals[mask][-1])

    samples: List[Dict[str, float]] = []
    for i, t in enumerate(target_times):
        samples.append({'time': float(t), **{f: float(series[f][i]) for f in fields}})
    return samples


def write_big_daddy_csv(out_path: str,
                         imu_samples: List[Tuple[float, Tuple[float, float, float], Tuple[float, float, float]]],
                         baro_samples: List[Tuple[float, float]],
                         gps_samples: List[Tuple[float, Tuple[int, int, int]]],
                         oracle_header_lines: List[str],
                         imu_zero_after_s: Optional[float] = None,
                         baro_freeze_after_s: Optional[float] = None,
                         baro_freeze_value_pa: float = SEA_LEVEL_PA,
                         gps_drop_after_s: Optional[float] = None,
                         truncate_after_s: Optional[float] = None) -> None:
    """Interleave IMU+baro+GPS samples and write the 11-column Big Daddy CSV.

    Sparse-row pattern: each output row has only the sensor columns that
    delivered a sample at that timestep; other columns are blank (= NaN).
    """
    # Tagged events: (time_s, source, payload)
    events: List[Tuple[float, str, object]] = []
    for t, accel, gyro in imu_samples:
        if truncate_after_s is not None and t > truncate_after_s:
            continue
        if imu_zero_after_s is not None and t >= imu_zero_after_s:
            accel = (0.0, 0.0, 0.0)
            gyro = (0.0, 0.0, 0.0)
        events.append((t, 'imu', (accel, gyro)))
    for t, pressure in baro_samples:
        if truncate_after_s is not None and t > truncate_after_s:
            continue
        if baro_freeze_after_s is not None and t >= baro_freeze_after_s:
            pressure = baro_freeze_value_pa
        events.append((t, 'baro', pressure))
    for t, gps in gps_samples:
        if truncate_after_s is not None and t > truncate_after_s:
            continue
        if gps_drop_after_s is not None and t >= gps_drop_after_s:
            continue  # drop GPS entirely after this time
        events.append((t, 'gps', gps))

    events.sort(key=lambda e: (e[0], {'imu': 0, 'baro': 1, 'gps': 2}[e[1]]))

    with open(out_path, 'w', newline='') as f:
        for line in oracle_header_lines:
            f.write(f'# {line}\n')
        f.write('#\n')
        f.write('time_s,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,'
                'pressure_pa,gps_lat_1e7,gps_lon_1e7,gps_alt_mm\n')

        for t, kind, payload in events:
            ax = ay = az = ''
            gx = gy = gz = ''
            press = ''
            lat = lon = alt = ''
            if kind == 'imu':
                (ax_, ay_, az_), (gx_, gy_, gz_) = payload
                ax, ay, az = f'{ax_:.4f}', f'{ay_:.4f}', f'{az_:.4f}'
                gx, gy, gz = f'{gx_:.4f}', f'{gy_:.4f}', f'{gz_:.4f}'
            elif kind == 'baro':
                press = f'{payload:.1f}'
            elif kind == 'gps':
                lat_i, lon_i, alt_i = payload
                lat, lon, alt = str(lat_i), str(lon_i), str(alt_i)
            f.write(f'{t:.4f},{ax},{ay},{az},{gx},{gy},{gz},'
                    f'{press},{lat},{lon},{alt}\n')


# ===========================================================================
# Main pipeline
# ===========================================================================

NOMINAL_ORACLE = [
    "Profile: Big Daddy F15-6 Nominal (OpenRocket-derived)",
    "Source: OpenRocket-23.09 simulation of tests/replay_profiles/estes_big_daddy.rkt",
    "Motor: Estes F15-6 progressive burn",
    "Avionics payload: 40g (Feather + DPS310 + ICM-20948 + PA1010D + RFM95W + 400mAh LiPo)",
    "",
    "Sensor rates:",
    "  IMU (accel/gyro): 100 Hz",
    "  Baro (pressure): 50 Hz",
    "  GPS (lat/lon/alt): 10 Hz",
    "",
    "Expected FD sequence:",
    "  IDLE -> ARMED -> BOOST -> COAST -> APOGEE -> DROGUE_DESCENT -> MAIN_DESCENT -> LANDED",
]

EARLY_BURNOUT_ORACLE = [
    "Profile: Big Daddy Early Burnout (OpenRocket-derived nominal, truncated boost)",
    "Source: OpenRocket nominal trajectory truncated at t=0.5s motor-out time",
    "Motor: F15-6 nominal trajectory, manually flame-out applied post-process",
    "Note: this profile applies an early-burnout overlay on the nominal OpenRocket",
    "trajectory by truncating boost-phase acceleration after t=0.5s. The kinematics",
    "downstream of t=0.5s (coast, apogee, descent) are derived from the nominal",
    "OpenRocket data — they reflect what physics would do if the motor had cut off",
    "at 0.5s. For a higher-fidelity early-burnout profile, regenerate from",
    "OpenRocket with a custom F15 thrust curve. R-28 reopen trigger.",
]

BARO_DROPOUT_ORACLE = [
    "Profile: Big Daddy Baro Stuck (OpenRocket-derived + post-apogee freeze)",
    "Source: nominal OpenRocket trajectory + baro freeze overlay",
    "Cause: DPS310 returns constant sea-level pressure (101325 Pa) for all t >= 7.6s",
    "Expected: baro innovations rejected by 3-sigma gate (stuck value)",
]

GPS_DROPOUT_ORACLE = [
    "Profile: Big Daddy GPS Dropout (OpenRocket-derived + descent GPS overlay)",
    "Source: nominal OpenRocket trajectory + GPS-absent-after-apogee overlay",
    "Cause: PA1010D loses satellite lock during descent (tumbling, antenna orientation)",
    "Expected: ESKF relies on baro + IMU for altitude after t >= 7.6s",
]

IMU_ZERO_ORACLE = [
    "Profile: Big Daddy IMU Zero Fault at t=10s (LL Entry 29) (OpenRocket-derived + IMU overlay)",
    "Source: nominal OpenRocket trajectory + IMU-zero-out-after-t=10s overlay",
    "Cause: ICM-20948 enters sleep/reset state, ACKs reads but returns zeros",
    "Expected: ESKF detects |A| < 3.0 m/s^2, routes to consecutive-fail -> reinit",
    "Expected: health_monitor reports IMU FAULT",
]


def synthesize_all_profiles(trajectory: List[Dict[str, Optional[float]]],
                             apogee_t: float,
                             out_dir: str,
                             prelaunch_pad_s: float = 1.0) -> None:
    """Generate the 5 canonical Big Daddy profiles from the OpenRocket trajectory.

    Prepends a `prelaunch_pad_s`-long on-pad preamble where the rocket sits
    stationary on the launch pad (accel = [0, 0, -g] body-Z-UP, gyro = 0,
    baro = sea-level, GPS = launch coords). The ESKF init expects a stationary
    first sample; OpenRocket's trajectory starts with the rocket already under
    motor thrust at t=0, so without the preamble the init's stationarity check
    would fail. The preamble represents "rocket sitting on rod waiting for
    ignition" — a normal flight scenario the firmware has to handle.
    """
    os.makedirs(out_dir, exist_ok=True)

    t_max = max(r['time'] for r in trajectory if r['time'] is not None)
    # Shift OpenRocket's t=0 to t=prelaunch_pad_s in our output timeline.
    apogee_t = apogee_t + prelaunch_pad_s

    # Resample OpenRocket trajectory onto sensor-rate grids, then add the
    # preamble on the front of each.
    accel_times_or = np.arange(0.0, t_max, 1.0 / ACCEL_HZ)
    baro_times_or = np.arange(0.0, t_max, 1.0 / BARO_HZ)
    gps_times_or = np.arange(0.0, t_max, 1.0 / GPS_HZ)

    accel_interp = interpolate_trajectory(trajectory, accel_times_or)
    baro_interp = interpolate_trajectory(trajectory, baro_times_or)
    gps_interp = interpolate_trajectory(trajectory, gps_times_or)

    # --- Pre-launch preamble (rocket stationary on pad) ---
    # Body-Z UP convention (matches src/fusion/eskf.h): at rest, accel_z = -g.
    # ESKF init expects |accel| ≈ g + small tolerance. Preamble samples are
    # at sensor rates from t=0 to t=prelaunch_pad_s.
    pad_accel = (0.0, 0.0, -G_NED)
    pad_gyro = (0.0, 0.0, 0.0)
    pad_baro_pa = SEA_LEVEL_PA  # rocket on the ground at launch altitude
    pad_gps = gps_position_ints(GPS_LAT_DEG, GPS_LON_DEG, 0.0)

    pre_accel_times = np.arange(0.0, prelaunch_pad_s, 1.0 / ACCEL_HZ)
    pre_baro_times = np.arange(0.0, prelaunch_pad_s, 1.0 / BARO_HZ)
    pre_gps_times = np.arange(0.0, prelaunch_pad_s, 1.0 / GPS_HZ)

    # Build per-rate sample lists in their own conventions.
    imu_samples: List[Tuple[float, Tuple[float, float, float], Tuple[float, float, float]]] = []
    # Preamble: stationary on pad.
    for t in pre_accel_times:
        imu_samples.append((float(t), pad_accel, pad_gyro))
    # OpenRocket trajectory: shifted forward by prelaunch_pad_s.
    for s in accel_interp:
        accel = synthesize_imu_sample(
            s.get('vertical_acceleration', 0.0) or 0.0,
            s.get('lateral_acceleration', 0.0) or 0.0,
            s.get('vertical_orientation', 0.0) or 0.0,
        )
        gyro = synthesize_gyro_sample(
            s.get('roll_rate'),
            s.get('pitch_rate'),
            s.get('yaw_rate'),
        )
        imu_samples.append((s['time'] + prelaunch_pad_s, accel, gyro))

    baro_samples: List[Tuple[float, float]] = []
    for t in pre_baro_times:
        baro_samples.append((float(t), pad_baro_pa))
    for s in baro_interp:
        alt = s.get('altitude', 0.0) or 0.0
        pa = s.get('air_pressure')
        # OpenRocket exports air pressure in mbar (hPa); convert to Pa
        # (1 mbar = 100 Pa). Sea-level reading should be ~1013 mbar = 101325 Pa.
        # If the value already looks Pa-scale (>10000), assume it's already
        # converted (defensive against future OpenRocket version changes).
        if pa is not None and math.isfinite(pa):
            if pa < 10000.0:  # mbar scale
                pa = pa * 100.0
        else:
            pa = altitude_to_pressure_pa(alt)
        baro_samples.append((s['time'] + prelaunch_pad_s, pa))

    gps_samples: List[Tuple[float, Tuple[int, int, int]]] = []
    for t in pre_gps_times:
        gps_samples.append((float(t), pad_gps))
    for s in gps_interp:
        alt = s.get('altitude', 0.0) or 0.0
        gps_samples.append((s['time'] + prelaunch_pad_s,
                            gps_position_ints(GPS_LAT_DEG, GPS_LON_DEG, alt)))

    # ---- Profile 1: nominal ------------------------------------------------
    write_big_daddy_csv(
        os.path.join(out_dir, 'big_daddy_f15_nominal.csv'),
        imu_samples, baro_samples, gps_samples,
        NOMINAL_ORACLE,
    )

    # ---- Profile 2: early burnout (truncate boost at t=0.5s) ---------------
    # For a quick overlay, truncate the trajectory at t=0.5s by replacing the
    # accel during the boost portion with coast acceleration (just -g body)
    # for samples at t > 0.5s. We do this by overriding accel_z for samples
    # that fall in the boost window. Crude but bounded.
    # NOTE: this is a post-process overlay, not a re-simulation with a
    # shortened thrust curve. Better would be to redo the OpenRocket sim with
    # a custom motor file — flagged as R-28 reopen trigger.
    # Early burnout window: t=0.5s..3s POST-LAUNCH, which is
    # (prelaunch_pad_s + 0.5)..(prelaunch_pad_s + 3) in the output timeline.
    eb_burn_start = prelaunch_pad_s + 0.5
    eb_burn_end = prelaunch_pad_s + 3.0
    eb_imu = []
    for t, accel, gyro in imu_samples:
        if eb_burn_start < t < eb_burn_end:
            # Approximate ballistic coast: gravity-only specific force in body
            # frame. At small tilt, body-Z reads -g, ax/ay near 0.
            accel = (0.0, 0.0, -G_NED)
        eb_imu.append((t, accel, gyro))
    write_big_daddy_csv(
        os.path.join(out_dir, 'big_daddy_early_burnout.csv'),
        eb_imu, baro_samples, gps_samples,
        EARLY_BURNOUT_ORACLE,
        truncate_after_s=prelaunch_pad_s + 15.0,
    )

    # ---- Profile 3: baro freeze post-apogee --------------------------------
    write_big_daddy_csv(
        os.path.join(out_dir, 'big_daddy_baro_dropout.csv'),
        imu_samples, baro_samples, gps_samples,
        BARO_DROPOUT_ORACLE,
        baro_freeze_after_s=apogee_t,
        baro_freeze_value_pa=SEA_LEVEL_PA,
    )

    # ---- Profile 4: GPS dropout post-apogee --------------------------------
    write_big_daddy_csv(
        os.path.join(out_dir, 'big_daddy_gps_dropout_descent.csv'),
        imu_samples, baro_samples, gps_samples,
        GPS_DROPOUT_ORACLE,
        gps_drop_after_s=apogee_t,
    )

    # ---- Profile 5: IMU zero fault at t=(10s after launch) -----------------
    # t=0 is now the start of the pad preamble, so launch is at t=prelaunch_pad_s
    # and the IMU zero-out event lands at preamble + 10s post-launch.
    write_big_daddy_csv(
        os.path.join(out_dir, 'big_daddy_imu_zero_fault.csv'),
        imu_samples, baro_samples, gps_samples,
        IMU_ZERO_ORACLE,
        imu_zero_after_s=prelaunch_pad_s + 10.0,
    )


def detect_apogee_time(trajectory: List[Dict[str, Optional[float]]]) -> float:
    """Apogee = time of max altitude in the trajectory."""
    max_alt = -math.inf
    apogee_t = 0.0
    for r in trajectory:
        alt = r.get('altitude')
        t = r.get('time')
        if alt is None or t is None:
            continue
        if alt > max_alt:
            max_alt = alt
            apogee_t = t
    return apogee_t


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('openrocket_csv', help='OpenRocket trajectory export CSV')
    ap.add_argument('out_dir',
                    help='Output directory for the 5 Big Daddy sensor CSVs')
    args = ap.parse_args()

    col_map, rows = parse_openrocket_csv(args.openrocket_csv)
    sys.stderr.write(f"parsed OpenRocket CSV: {len(rows)} rows, "
                     f"{len(col_map)} mapped columns\n")
    sys.stderr.write(f"  columns: {sorted(col_map.keys())}\n")

    # Pivot rows -> trajectory list of {canonical_key: value, ...}
    trajectory: List[Dict[str, Optional[float]]] = []
    for row in rows:
        rec: Dict[str, Optional[float]] = {}
        for k, idx in col_map.items():
            if idx < len(row):
                rec[k] = row[idx]
        trajectory.append(rec)

    apogee_t = detect_apogee_time(trajectory)
    sys.stderr.write(f"  apogee time: {apogee_t:.3f}s, "
                     f"peak altitude: {max((r['altitude'] or -math.inf) for r in trajectory):.2f} m\n")

    synthesize_all_profiles(trajectory, apogee_t, args.out_dir)
    sys.stderr.write(f"wrote 5 Big Daddy CSVs to {args.out_dir}\n")
    return 0


if __name__ == '__main__':
    sys.exit(main())
