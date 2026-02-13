#!/usr/bin/env python3
"""Generate 5 canonical synthetic trajectories for ESKF testing.

IVP-42b: Produces ideal (noise-free) sensor data and ground truth for:
  1. Static (60s)
  2. Constant velocity (30s)
  3. Constant acceleration (30s)
  4. Constant turn rate (10s)
  5. Banked turn (10s)

All trajectories at 200 Hz (dt = 0.005s). No sensor noise.
CSV format per ESKF_TESTING_GUIDE.md Section 5.3.

Physics conventions (must match eskf.cpp):
  - Body frame: accelerometer reads specific force f = a_true - g
  - NED frame: gravity = [0, 0, +g], North = +X, East = +Y, Down = +Z
  - Quaternion: body-to-NED, Hamilton, scalar-first [w, x, y, z]
"""

import numpy as np
import os

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
G = 9.80665
DT = 0.005  # 200 Hz
OUTDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data')

# Reference NED magnetic field (µT) — mid-latitude, zero declination.
# Matches test/test_eskf_mag_update.cpp kMagNed.
MAG_NED = np.array([20.0, 0.0, 45.0])

HEADER = (
    "timestamp_us,ax,ay,az,gx,gy,gz,"
    "baro_alt_m,gps_lat_1e7,gps_lon_1e7,gps_alt_m,gps_vn,gps_ve,gps_vd,gps_fix,"
    "mx,my,mz,"
    "truth_qw,truth_qx,truth_qy,truth_qz,"
    "truth_pn,truth_pe,truth_pd,"
    "truth_vn,truth_ve,truth_vd"
)

NAN_GPS_BARO = ",".join(["nan"] * 8)  # baro(1) + gps(7)


def quat_from_euler(roll, pitch, yaw):
    """ZYX Euler angles to Hamilton quaternion [w, x, y, z].

    Matches rc::Quat::from_euler() convention.
    """
    cr = np.cos(roll / 2)
    sr = np.sin(roll / 2)
    cp = np.cos(pitch / 2)
    sp = np.sin(pitch / 2)
    cy = np.cos(yaw / 2)
    sy = np.sin(yaw / 2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([w, x, y, z])


def quat_to_dcm(q):
    """Hamilton quaternion [w, x, y, z] to 3x3 body-to-NED DCM.

    Matches rc::Quat::to_rotation_matrix() convention.
    """
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


def mag_body_from_quat(q, mag_ned):
    """Compute body-frame magnetometer reading from truth quaternion.

    mag_body = R(q)^T * mag_ned  (NED→body = inverse of body→NED rotation).
    """
    R = quat_to_dcm(q)
    return R.T @ mag_ned


def write_csv(filename, rows):
    """Write trajectory rows to CSV with header."""
    filepath = os.path.join(OUTDIR, filename)
    os.makedirs(OUTDIR, exist_ok=True)
    with open(filepath, 'w', newline='') as f:
        f.write(HEADER + '\n')
        for row in rows:
            f.write(row + '\n')
    print(f"  {filename}: {len(rows)} rows")


def format_row(t_us, ax, ay, az, gx, gy, gz, mx, my, mz,
               qw, qx, qy, qz, pn, pe, pd, vn, ve, vd):
    """Format one CSV row with IMU + mag + truth columns, NaN for baro/GPS."""
    return (
        f"{t_us},{ax:.7g},{ay:.7g},{az:.7g},{gx:.7g},{gy:.7g},{gz:.7g},"
        f"{NAN_GPS_BARO},"
        f"{mx:.7g},{my:.7g},{mz:.7g},"
        f"{qw:.9g},{qx:.9g},{qy:.9g},{qz:.9g},"
        f"{pn:.9g},{pe:.9g},{pd:.9g},"
        f"{vn:.9g},{ve:.9g},{vd:.9g}"
    )


# ---------------------------------------------------------------------------
# Trajectory 1: Static (60s)
# ---------------------------------------------------------------------------
def generate_static():
    duration = 60.0
    n_steps = int(duration / DT)
    q = quat_from_euler(0.0, 0.0, 0.0)
    mag_b = mag_body_from_quat(q, MAG_NED)
    rows = []
    for i in range(n_steps):
        t = i * DT
        t_us = int(t * 1e6)
        rows.append(format_row(
            t_us,
            0.0, 0.0, -G,       # accel body: specific force at rest
            0.0, 0.0, 0.0,       # gyro body: no rotation
            mag_b[0], mag_b[1], mag_b[2],  # mag body: constant (no rotation)
            q[0], q[1], q[2], q[3],  # quat: identity
            0.0, 0.0, 0.0,       # position: origin
            0.0, 0.0, 0.0,       # velocity: zero
        ))
    write_csv('static_1min.csv', rows)


# ---------------------------------------------------------------------------
# Trajectory 2: Constant Velocity (30s) — 10 m/s North
# ---------------------------------------------------------------------------
def generate_const_velocity():
    duration = 30.0
    n_steps = int(duration / DT)
    v_north = 10.0
    q = quat_from_euler(0.0, 0.0, 0.0)
    mag_b = mag_body_from_quat(q, MAG_NED)
    rows = []
    for i in range(n_steps):
        t = i * DT
        t_us = int(t * 1e6)
        rows.append(format_row(
            t_us,
            0.0, 0.0, -G,        # no body accel, just gravity
            0.0, 0.0, 0.0,        # no rotation
            mag_b[0], mag_b[1], mag_b[2],
            q[0], q[1], q[2], q[3],   # identity quaternion
            v_north * t, 0.0, 0.0, # position: linear
            v_north, 0.0, 0.0,     # velocity: constant
        ))
    write_csv('const_velocity_30s.csv', rows)


# ---------------------------------------------------------------------------
# Trajectory 3: Constant Acceleration (30s) — 1 m/s² North
# ---------------------------------------------------------------------------
def generate_const_accel():
    duration = 30.0
    n_steps = int(duration / DT)
    a_north = 1.0  # m/s²
    q = quat_from_euler(0.0, 0.0, 0.0)
    mag_b = mag_body_from_quat(q, MAG_NED)
    rows = []
    for i in range(n_steps):
        t = i * DT
        t_us = int(t * 1e6)
        # Specific force: a_body - g_body = [1, 0, 0] - [0, 0, -g] = [1, 0, -g]
        rows.append(format_row(
            t_us,
            a_north, 0.0, -G,     # accel body
            0.0, 0.0, 0.0,        # no rotation
            mag_b[0], mag_b[1], mag_b[2],
            q[0], q[1], q[2], q[3],   # identity quaternion
            0.5 * a_north * t * t, 0.0, 0.0,  # position: s = 0.5*a*t²
            a_north * t, 0.0, 0.0,             # velocity: v = a*t
        ))
    write_csv('const_accel_30s.csv', rows)


# ---------------------------------------------------------------------------
# Trajectory 4: Constant Turn Rate (10s)
# 30°/s yaw, 5 m/s forward, level flight
# ---------------------------------------------------------------------------
def generate_const_turn():
    duration = 10.0
    n_steps = int(duration / DT)
    omega = np.radians(30.0)  # 0.5236 rad/s
    v = 5.0                   # m/s forward speed
    r = v / omega             # turn radius ~9.549 m

    # Centripetal acceleration: v * omega directed body +Y
    # (positive yaw = left turn, centrifugal pushes body +Y)
    a_centripetal = v * omega  # ~2.618 m/s²

    rows = []
    for i in range(n_steps):
        t = i * DT
        t_us = int(t * 1e6)
        psi = omega * t  # yaw angle

        # Truth quaternion: from_euler(0, 0, psi)
        q = quat_from_euler(0.0, 0.0, psi)

        # Body-frame mag rotates with device
        mag_b = mag_body_from_quat(q, MAG_NED)

        # NED position: circle
        pn = r * np.sin(psi)
        pe = r * (1.0 - np.cos(psi))

        # NED velocity
        vn = v * np.cos(psi)
        ve = v * np.sin(psi)

        rows.append(format_row(
            t_us,
            0.0, a_centripetal, -G,  # body accel: no forward, centripetal +Y, gravity -Z
            0.0, 0.0, omega,         # gyro: yaw rate about body Z
            mag_b[0], mag_b[1], mag_b[2],
            q[0], q[1], q[2], q[3],
            pn, pe, 0.0,
            vn, ve, 0.0,
        ))
    write_csv('const_turn_10s.csv', rows)


# ---------------------------------------------------------------------------
# Trajectory 5: Banked Turn (10s)
# 30° bank, coordinated turn, 5 m/s forward
# ---------------------------------------------------------------------------
def generate_banked_turn():
    duration = 10.0
    n_steps = int(duration / DT)
    bank = np.radians(30.0)   # 30° roll
    v = 5.0                   # m/s forward speed

    # Coordinated turn: omega = g * tan(bank) / v
    omega = G * np.tan(bank) / v  # ~1.133 rad/s (~64.9°/s)
    r = v / omega                  # ~4.413 m

    # Body-frame accelerometer: [0, 0, -g/cos(bank)]
    az_body = -G / np.cos(bank)    # ~-11.326

    # Body-frame gyro: NED yaw rate [0,0,omega] projected through roll
    # R^T(phi) * [0, 0, omega] = [0, omega*sin(phi), omega*cos(phi)]
    gx = 0.0
    gy = omega * np.sin(bank)   # ~0.567 rad/s
    gz = omega * np.cos(bank)   # ~0.981 rad/s

    rows = []
    for i in range(n_steps):
        t = i * DT
        t_us = int(t * 1e6)
        psi = omega * t  # yaw angle

        # Truth quaternion: from_euler(roll=bank, pitch=0, yaw=psi)
        q = quat_from_euler(bank, 0.0, psi)

        # Body-frame mag rotates with device (both roll and yaw)
        mag_b = mag_body_from_quat(q, MAG_NED)

        # NED position: circle (horizontal plane)
        pn = r * np.sin(psi)
        pe = r * (1.0 - np.cos(psi))

        # NED velocity
        vn = v * np.cos(psi)
        ve = v * np.sin(psi)

        rows.append(format_row(
            t_us,
            0.0, 0.0, az_body,      # body accel (coordinated: no lateral)
            gx, gy, gz,              # body gyro
            mag_b[0], mag_b[1], mag_b[2],
            q[0], q[1], q[2], q[3],
            pn, pe, 0.0,
            vn, ve, 0.0,
        ))
    write_csv('banked_turn_10s.csv', rows)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    print("Generating 5 canonical trajectories...")
    generate_static()
    generate_const_velocity()
    generate_const_accel()
    generate_const_turn()
    generate_banked_turn()
    print("Done.")
