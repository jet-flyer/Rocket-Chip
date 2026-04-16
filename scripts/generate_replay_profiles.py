#!/usr/bin/env python
"""Generate sensor-rate flight profile CSVs for replay harness (IVP-131).

Physics model: 1D vertical flight (drag + gravity + thrust + chute).
Source data: Estes Big Daddy .rkt geometry + Estes F15-6 thrust curve.
Council review: NASA/JPL, Rocketeer, Space Camp Counselor (2026-04-15).

Outputs CSV at sensor rates:
  - accel_x/y/z at 100Hz (NED frame, z-down = positive when stationary)
  - baro pressure at 50Hz (ISA atmosphere model)
  - GPS lat/lon/alt at 10Hz (fixed lat/lon, altitude from sim)

Each profile includes:
  - Ground-truth oracle (expected FD state transitions, pyro fire times)
  - Physical phase labels (motor burning, coasting, chute deployed, etc.)
"""
import csv
import math
import os

# ==========================================================================
# Physical constants
# ==========================================================================
G = 9.80665
P_SEA = 101325.0
T_SEA = 288.15
LAPSE = 0.0065
R_AIR = 287.053

# ==========================================================================
# Big Daddy parameters (from .rkt file + Estes specs)
# ==========================================================================
DRY_MASS_KG = 0.070        # Estes actual (not .rkt computed 160g)
PAYLOAD_KG = 0.050         # avionics + battery
BODY_OD_M = 0.0762         # 76.2mm BT-80
REF_AREA = math.pi * (BODY_OD_M / 2) ** 2
CD_BODY = 0.50             # typical hobby rocket (0.3-0.6 range)
CHUTE_DIAM_M = 0.457       # 18" parachute
CHUTE_AREA = math.pi * (CHUTE_DIAM_M / 2) ** 2
CD_CHUTE = 1.5

# Estes F15-6 thrust curve (from ThrustCurve.org RASP data, NAR certified)
# Source: https://www.thrustcurve.org/simfiles/5f4294d20002e900000007df/
THRUST_CURVE = [
    (0.000,  0.00),
    (0.063,  2.13),
    (0.118,  4.41),
    (0.158,  8.36),
    (0.228, 13.68),
    (0.340, 20.82),
    (0.386, 26.75),
    (0.425, 25.38),
    (0.481, 22.19),
    (0.583, 17.93),
    (0.883, 16.11),
    (1.191, 14.59),
    (1.364, 15.35),
    (1.569, 15.65),
    (1.727, 14.74),
    (2.000, 14.28),
    (2.390, 13.68),
    (2.680, 13.08),
    (2.960, 13.07),
    (3.250, 13.05),
    (3.350, 13.00),
    (3.390,  7.30),
    (3.400,  0.00),
]
MOTOR_BURN_TIME_S = THRUST_CURVE[-1][0]
MOTOR_TOTAL_MASS_KG = 0.103   # F15 total mass
MOTOR_PROP_MASS_KG = 0.060    # F15 propellant mass
MOTOR_DELAY_S = 6.0            # -6 delay

# ==========================================================================
# Atmosphere model (ISA troposphere)
# ==========================================================================
def isa_pressure(alt_m):
    return P_SEA * (1 - LAPSE * alt_m / T_SEA) ** (G / (LAPSE * R_AIR))

def isa_density(alt_m):
    T = T_SEA - LAPSE * alt_m
    return isa_pressure(alt_m) / (R_AIR * T)

# ==========================================================================
# Thrust curve (piecewise linear interpolation)
# ==========================================================================
def thrust(t):
    if t <= 0 or t >= THRUST_CURVE[-1][0]:
        return 0.0
    for i in range(1, len(THRUST_CURVE)):
        t0, f0 = THRUST_CURVE[i - 1]
        t1, f1 = THRUST_CURVE[i]
        if t0 <= t <= t1:
            frac = (t - t0) / (t1 - t0)
            return f0 + frac * (f1 - f0)
    return 0.0

def mass_at(t):
    base = DRY_MASS_KG + PAYLOAD_KG + (MOTOR_TOTAL_MASS_KG - MOTOR_PROP_MASS_KG)
    if t >= MOTOR_BURN_TIME_S:
        return base
    return base + MOTOR_PROP_MASS_KG * (1 - t / MOTOR_BURN_TIME_S)

# ==========================================================================
# 1D flight simulation with chute deployment
# ==========================================================================
def simulate_flight(dt=0.001, max_time=180.0, early_burnout_t=None):
    results = []
    alt = 0.0
    vel = 0.0
    launched = False
    apogee_t = None
    chute_deployed = False

    t = 0.0
    while t < max_time:
        m = mass_at(t)
        T = thrust(t)
        if early_burnout_t is not None and t > early_burnout_t:
            T = 0.0
        rho = isa_density(max(alt, 0))

        # Drag: body drag during boost/coast, chute drag after deployment
        if chute_deployed:
            drag_force = 0.5 * rho * vel * abs(vel) * CD_CHUTE * CHUTE_AREA
        else:
            drag_force = 0.5 * rho * vel * abs(vel) * CD_BODY * REF_AREA

        F_net = T - m * G - drag_force
        a = F_net / m

        if not launched and T > 0 and a > 0:
            launched = True

        if not launched:
            a = 0.0
            vel = 0.0

        # Detect apogee (velocity crosses zero going down)
        if launched and apogee_t is None and vel < 0:
            apogee_t = t

        # Chute deployment at apogee + motor delay
        if apogee_t is not None and not chute_deployed and t >= apogee_t + MOTOR_DELAY_S:
            chute_deployed = True

        # Body-frame Z accel (NED: Z-down positive)
        if not launched:
            accel_body_z = G
        else:
            accel_body_z = -a

        phase = 'pad'
        if launched and T > 0:
            phase = 'motor_burning'
        elif launched and apogee_t is None:
            phase = 'coasting_up'
        elif launched and not chute_deployed and vel < 0:
            phase = 'ballistic_descent'
        elif chute_deployed and alt > 0:
            phase = 'chute_descent'

        results.append((t, max(alt, 0), vel, accel_body_z, phase))

        vel += a * dt
        alt += vel * dt

        if launched and alt <= 0 and vel < 0:
            alt = 0
            vel = 0
            for _ in range(int(3.0 / dt)):
                t += dt
                results.append((t, 0, 0, G, 'landed'))
            break

        t += dt

    return results

# ==========================================================================
# CSV generation at sensor rates
# ==========================================================================
GPS_LAT = 32.9500
GPS_LON = -96.7500

def write_profile(filename, sim_data, oracle_lines, fault_fn=None):
    accel_period = 0.01
    baro_period = 0.02
    gps_period = 0.1

    with open(filename, 'w', newline='') as f:
        for line in oracle_lines:
            f.write(f'# {line}\n')
        f.write('#\n')

        writer = csv.writer(f)
        writer.writerow([
            'time_s', 'accel_x', 'accel_y', 'accel_z',
            'gyro_x', 'gyro_y', 'gyro_z',
            'pressure_pa', 'gps_lat_1e7', 'gps_lon_1e7', 'gps_alt_mm'
        ])

        next_a = 0.0
        next_b = 0.0
        next_g = 0.0

        for t, alt, vel, az, phase in sim_data:
            row_fault = fault_fn(t, alt, vel) if fault_fn else None
            a_due = t >= next_a
            b_due = t >= next_b
            g_due = t >= next_g
            if not (a_due or b_due or g_due):
                continue

            ax, ay = 0.0, 0.0
            a_z = az if a_due else float('nan')
            pres = isa_pressure(alt) if b_due else float('nan')
            glat = int(GPS_LAT * 1e7) if g_due else ''
            glon = int(GPS_LON * 1e7) if g_due else ''
            galt = int(alt * 1000) if g_due else ''

            if row_fault:
                ax, ay, a_z, pres, glat, glon, galt = row_fault

            pres_str = f'{pres:.1f}' if isinstance(pres, float) and not math.isnan(pres) else ''
            writer.writerow([
                f'{t:.4f}',
                f'{ax:.4f}', f'{ay:.4f}', f'{a_z:.4f}' if not (isinstance(a_z, float) and math.isnan(a_z)) else '',
                '0.0000', '0.0000', '0.0000',
                pres_str, glat, glon, galt
            ])

            if a_due: next_a = t + accel_period
            if b_due: next_b = t + baro_period
            if g_due: next_g = t + gps_period

# ==========================================================================
# Profile definitions (council-reviewed)
# ==========================================================================
def main():
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           '..', 'tests', 'replay_profiles')
    os.makedirs(out_dir, exist_ok=True)

    sim = simulate_flight()

    # Find key events
    apogee_alt, apogee_t, chute_t, landed_t = 0, 0, 0, sim[-1][0]
    for t, alt, vel, _, phase in sim:
        if alt > apogee_alt:
            apogee_alt = alt
            apogee_t = t
        if phase == 'chute_descent' and chute_t == 0:
            chute_t = t

    print(f'Sim: apogee={apogee_alt:.1f}m at t={apogee_t:.1f}s, '
          f'chute at t={chute_t:.1f}s, landed t={landed_t:.1f}s')

    # --- 1: Nominal ---
    write_profile(
        os.path.join(out_dir, 'big_daddy_f15_nominal.csv'), sim,
        [
            'Profile: Big Daddy F15-6 Nominal',
            f'Motor: Estes F15-6 progressive burn, {MOTOR_BURN_TIME_S}s burn',
            f'Mass: {DRY_MASS_KG*1000:.0f}g airframe + {PAYLOAD_KG*1000:.0f}g payload + {MOTOR_TOTAL_MASS_KG*1000:.0f}g motor',
            '',
            'Physical phases:',
            f'  t=0.00-{MOTOR_BURN_TIME_S:.2f}s : motor burning (progressive thrust 5-22N)',
            f'  t={MOTOR_BURN_TIME_S:.2f}-{apogee_t:.1f}s : coasting under momentum',
            f'  t={apogee_t:.1f}s : apogee ({apogee_alt:.1f}m AGL)',
            f'  t={apogee_t:.1f}-{chute_t:.1f}s : ballistic descent (no chute yet, -6 delay)',
            f'  t={chute_t:.1f}s : ejection charge fires, chute deploys',
            f'  t={chute_t:.1f}-{landed_t:.1f}s : descending on chute (~5 m/s)',
            f'  t={landed_t:.1f}s : ground impact',
            '',
            'Expected FD sequence:',
            '  IDLE -> ARMED -> BOOST -> COAST -> APOGEE -> DROGUE_DESCENT -> MAIN_DESCENT -> LANDED',
            f'Expected drogue fire: ~t={apogee_t:.1f}s (at apogee detection)',
        ])

    # --- 2: Early burnout (council: renamed from motor_cato) ---
    sim_eb = simulate_flight(early_burnout_t=0.5)
    eb_apogee_alt, eb_apogee_t = 0, 0
    for t, alt, vel, _, _ in sim_eb:
        if alt > eb_apogee_alt:
            eb_apogee_alt = alt
            eb_apogee_t = t
    write_profile(
        os.path.join(out_dir, 'big_daddy_early_burnout.csv'), sim_eb,
        [
            'Profile: Big Daddy Early Burnout (flame-out at 0.5s)',
            'Cause: motor grain crack or underperformance (NOT a CATO)',
            '',
            'Physical phases:',
            '  t=0.00-0.50s : partial motor burn',
            f'  t=0.50-{eb_apogee_t:.1f}s : coasting (low energy)',
            f'  t={eb_apogee_t:.1f}s : low apogee ({eb_apogee_alt:.1f}m AGL)',
            '',
            'Expected FD: BOOST (short) -> COAST -> APOGEE -> DROGUE_DESCENT -> LANDED',
        ])

    # --- 3: IMU zero fault at t=10s ---
    def imu_zero_fault(t, alt, vel):
        if t >= 10.0:
            return (0.0, 0.0, 0.0, isa_pressure(alt),
                    int(GPS_LAT * 1e7), int(GPS_LON * 1e7), int(alt * 1000))
        return None

    write_profile(
        os.path.join(out_dir, 'big_daddy_imu_zero_fault.csv'), sim,
        [
            'Profile: Big Daddy IMU Zero Fault at t=10s (LL Entry 29)',
            'Cause: ICM-20948 enters sleep/reset state, ACKs reads but returns zeros',
            '',
            'Physical phases: same as nominal until t=10s, then:',
            '  t>=10s : all accel/gyro read as (0,0,0)',
            '',
            'Expected: ESKF detects |A| < 3.0 m/s^2, routes to consecutive-fail -> reinit',
            'Expected: health_monitor reports IMU FAULT',
        ],
        fault_fn=imu_zero_fault)

    # --- 4: Baro stuck at sea level during descent ---
    def baro_dropout(t, alt, vel):
        if t >= apogee_t:
            return (0.0, 0.0, -vel + G, P_SEA,
                    int(GPS_LAT * 1e7), int(GPS_LON * 1e7), int(alt * 1000))
        return None

    write_profile(
        os.path.join(out_dir, 'big_daddy_baro_dropout.csv'), sim,
        [
            'Profile: Big Daddy Baro Stuck at Sea Level During Descent',
            'Cause: DPS310 returns constant sea-level pressure after apogee',
            '',
            'Physical phases: same as nominal, but after apogee:',
            f'  t>={apogee_t:.1f}s : baro reads {P_SEA:.0f} Pa (sea level) regardless of altitude',
            '',
            'Expected: baro innovations rejected by 3-sigma gate (stuck value)',
            'Expected: ESKF relies on GPS + IMU for altitude',
            'Expected: landing detection via conjunction guard',
        ],
        fault_fn=baro_dropout)

    # --- 5: GPS dropout after apogee ---
    def gps_dropout(t, alt, vel):
        if t >= apogee_t:
            return (0.0, 0.0, -vel + G, isa_pressure(alt), '', '', '')
        return None

    write_profile(
        os.path.join(out_dir, 'big_daddy_gps_dropout_descent.csv'), sim,
        [
            'Profile: Big Daddy GPS Dropout After Apogee',
            'Cause: PA1010D loses satellite lock during descent (tumbling, antenna orientation)',
            '',
            'Physical phases: same as nominal, but after apogee:',
            f'  t>={apogee_t:.1f}s : no GPS fixes, ESKF position drifts',
            '',
            'Expected: landing detected via baro + conjunction (IVP-119/120/121)',
            'Expected: multi-channel landing detection handles GPS-denied case',
        ],
        fault_fn=gps_dropout)

    print(f'Generated 5 profiles in {out_dir}')

if __name__ == '__main__':
    main()
