#!/usr/bin/env python3
"""
generate_fpft.py — SymPy codegen for ESKF 15-state FPFT covariance propagation.

Generates eskf_codegen.cpp containing codegen_fpft(), a flat scalar C++ function
that computes P_new = F * P * F^T + Q_d for the 15-state error-state Kalman filter.

Uses Common Subexpression Elimination (CSE) to minimize redundant computation.
The generated code replaces the dense O(N^3) triple product with O(N^2) scalar ops.

Reference: Sola (2017) "Quaternion kinematics for the error-state Kalman filter", Eq. 269
PX4 ECL uses the same SymPy/SymForce codegen approach (BSD-3 licensed).

Usage:
    python scripts/generate_fpft.py

Output:
    src/fusion/eskf_codegen.cpp

The generated file is committed to the repo. Re-run this script only when the F matrix
structure or Q_d noise constants change.
"""

import sys
import datetime
import numpy as np

try:
    import sympy as sp
except ImportError:
    print("ERROR: sympy not installed. Run: pip install sympy", file=sys.stderr)
    sys.exit(1)

# ============================================================================
# Constants from eskf.h — must match exactly (enforced by static_assert in C++)
# ============================================================================
SIGMA_GYRO = 2.618e-4           # rad/s/sqrt(Hz)
SIGMA_ACCEL = 2.256e-3          # m/s^2/sqrt(Hz)
SIGMA_GYRO_BIAS_WALK = 1.0e-5   # rad/s^2/sqrt(Hz)
SIGMA_ACCEL_BIAS_WALK = 1.0e-4  # m/s^3/sqrt(Hz)

N = 15  # state size

# State indices (match eskf_state.h)
IDX_ATT = 0    # [0..2]  attitude error
IDX_POS = 3    # [3..5]  position
IDX_VEL = 6    # [6..8]  velocity
IDX_AB  = 9    # [9..11] accelerometer bias
IDX_GB  = 12   # [12..14] gyroscope bias


def build_symbolic_F():
    """Build the 15x15 discrete-time state transition matrix F symbolically.

    F = I + dt * F_delta (Sola 2017, Eq. 269)

    Non-zero off-diagonal blocks:
      F[att,att] = I - dt*skew(w)       (dense 3x3)
      F[att,gb]  = -dt*I                (scaled identity)
      F[pos,vel] = +dt*I                (scaled identity)
      F[vel,att] = -dt*R*skew(a)        (dense 3x3)
      F[vel,ab]  = -dt*R                (dense 3x3)
    """
    dt = sp.Symbol('dt')

    # Rotation matrix R (body-to-NED DCM) — 9 individual symbols
    R = sp.Matrix(3, 3, lambda i, j: sp.Symbol(f'R{i}{j}'))

    # Bias-corrected accel and gyro
    ax, ay, az = sp.symbols('ax ay az')
    wx, wy, wz = sp.symbols('wx wy wz')

    # Skew-symmetric matrices
    skew_w = sp.Matrix([
        [0,   -wz,  wy],
        [wz,   0,  -wx],
        [-wy,  wx,   0],
    ])
    skew_a = sp.Matrix([
        [0,   -az,  ay],
        [az,   0,  -ax],
        [-ay,  ax,   0],
    ])

    # Start with identity
    F = sp.eye(N)

    # F[att,att] = I - dt*skew(w)
    F[IDX_ATT:IDX_ATT+3, IDX_ATT:IDX_ATT+3] = sp.eye(3) - dt * skew_w

    # F[att,gb] = -dt*I
    F[IDX_ATT:IDX_ATT+3, IDX_GB:IDX_GB+3] = -dt * sp.eye(3)

    # F[pos,vel] = dt*I
    F[IDX_POS:IDX_POS+3, IDX_VEL:IDX_VEL+3] = dt * sp.eye(3)

    # F[vel,att] = -dt*R*skew(a)
    F[IDX_VEL:IDX_VEL+3, IDX_ATT:IDX_ATT+3] = -dt * R * skew_a

    # F[vel,ab] = -dt*R
    F[IDX_VEL:IDX_VEL+3, IDX_AB:IDX_AB+3] = -dt * R

    return F, dt, R, (ax, ay, az), (wx, wy, wz)


def build_symbolic_P():
    """Build a symmetric 15x15 P matrix with 120 unique symbols (upper triangle)."""
    P = sp.zeros(N, N)
    for i in range(N):
        for j in range(i, N):
            sym = sp.Symbol(f'P{i}_{j}')
            P[i, j] = sym
            P[j, i] = sym  # symmetric
    return P


def build_Qd_diagonal(dt_sym):
    """Build diagonal Q_d = dt * diag(sigma^2 values).

    Q_d structure (15 diagonal elements):
      [0..2]  sigma_gyro^2 * dt       (attitude)
      [3..5]  0                        (position — no direct noise)
      [6..8]  sigma_accel^2 * dt       (velocity)
      [9..11] sigma_accel_bias^2 * dt  (accel bias walk)
      [12..14] sigma_gyro_bias^2 * dt  (gyro bias walk)
    """
    diag = [0.0] * N

    sg2 = SIGMA_GYRO ** 2
    sa2 = SIGMA_ACCEL ** 2
    sab2 = SIGMA_ACCEL_BIAS_WALK ** 2
    sgb2 = SIGMA_GYRO_BIAS_WALK ** 2

    for i in range(3):
        diag[IDX_ATT + i] = sg2
        # IDX_POS: 0 (no direct noise)
        diag[IDX_VEL + i] = sa2
        diag[IDX_AB + i]  = sab2
        diag[IDX_GB + i]  = sgb2

    # Return as symbolic expression: diag[i] * dt
    Qd = sp.zeros(N, N)
    for i in range(N):
        if diag[i] != 0.0:
            # Use exact rational representation to avoid float artifacts in SymPy
            Qd[i, i] = sp.Rational(diag[i]).limit_denominator(10**15) * dt_sym
    return Qd


def run_self_test(F_sym, P_sym, dt_sym, R_sym, accel_syms, gyro_syms):
    """Council R1: Verify symbolic FPFT against NumPy dense reference with non-trivial inputs."""
    print("Running self-test (R1: non-trivial inputs)...")

    # 30-degree roll rotation matrix
    roll = np.deg2rad(30.0)
    cr, sr = np.cos(roll), np.sin(roll)
    R_np = np.array([
        [1.0,  0.0,  0.0],
        [0.0,  cr,  -sr],
        [0.0,  sr,   cr],
    ])

    # Non-trivial P: run 10 dense propagation steps from identity
    dt_val = 0.005
    ax_val, ay_val, az_val = 0.1, -0.05, -9.80665
    wx_val, wy_val, wz_val = 0.01, 0.02, 0.03

    P_np = np.eye(N) * 0.01  # small initial covariance

    # Build F numerically
    skew_w_np = np.array([
        [0, -wz_val, wy_val],
        [wz_val, 0, -wx_val],
        [-wy_val, wx_val, 0],
    ])
    skew_a_np = np.array([
        [0, -az_val, ay_val],
        [az_val, 0, -ax_val],
        [-ay_val, ax_val, 0],
    ])

    F_np = np.eye(N)
    F_np[IDX_ATT:IDX_ATT+3, IDX_ATT:IDX_ATT+3] = np.eye(3) - dt_val * skew_w_np
    F_np[IDX_ATT:IDX_ATT+3, IDX_GB:IDX_GB+3] = -dt_val * np.eye(3)
    F_np[IDX_POS:IDX_POS+3, IDX_VEL:IDX_VEL+3] = dt_val * np.eye(3)
    F_np[IDX_VEL:IDX_VEL+3, IDX_ATT:IDX_ATT+3] = -dt_val * R_np @ skew_a_np
    F_np[IDX_VEL:IDX_VEL+3, IDX_AB:IDX_AB+3] = -dt_val * R_np

    # Build Q_d numerically
    Qd_np = np.zeros((N, N))
    for i in range(3):
        Qd_np[IDX_ATT+i, IDX_ATT+i] = SIGMA_GYRO**2 * dt_val
        Qd_np[IDX_VEL+i, IDX_VEL+i] = SIGMA_ACCEL**2 * dt_val
        Qd_np[IDX_AB+i,  IDX_AB+i]  = SIGMA_ACCEL_BIAS_WALK**2 * dt_val
        Qd_np[IDX_GB+i,  IDX_GB+i]  = SIGMA_GYRO_BIAS_WALK**2 * dt_val

    # Propagate P 10 steps to get non-trivial P
    for _ in range(10):
        P_np = F_np @ P_np @ F_np.T + Qd_np

    # Now compute FPFT + Qd with this non-trivial P (reference)
    ref = F_np @ P_np @ F_np.T + Qd_np

    # Build substitution dict for symbolic evaluation
    ax_s, ay_s, az_s = accel_syms
    wx_s, wy_s, wz_s = gyro_syms
    subs = {dt_sym: dt_val, ax_s: ax_val, ay_s: ay_val, az_s: az_val,
            wx_s: wx_val, wy_s: wy_val, wz_s: wz_val}
    for i in range(3):
        for j in range(3):
            subs[R_sym[i, j]] = float(R_np[i, j])
    for i in range(N):
        for j in range(i, N):
            subs[sp.Symbol(f'P{i}_{j}')] = float(P_np[i, j])

    # Evaluate symbolic FPFT + Qd
    Qd_sym = build_Qd_diagonal(dt_sym)
    FPFT_plus_Qd = F_sym * P_sym * F_sym.T + Qd_sym
    sympy_result = np.array(FPFT_plus_Qd.subs(subs).evalf().tolist(), dtype=float)

    # Compare
    max_err = np.max(np.abs(sympy_result - ref))
    print(f"  Max absolute error (symbolic vs NumPy): {max_err:.2e}")
    if max_err > 1e-10:
        print(f"  FAIL: max error {max_err:.2e} exceeds 1e-10", file=sys.stderr)
        # Show worst elements
        worst = np.unravel_index(np.argmax(np.abs(sympy_result - ref)), ref.shape)
        print(f"  Worst element: [{worst[0]},{worst[1]}] "
              f"sympy={sympy_result[worst]:.15e} ref={ref[worst]:.15e}")
        sys.exit(1)
    print("  PASS")
    return P_np, ref  # return for single-step test reference generation


def _collect_P_refs(intermediates, reduced_exprs):
    """Find all P[i][j] symbols referenced in intermediates and output expressions."""
    import re
    all_refs = set()
    for _, expr in intermediates:
        c_str = sp.ccode(expr)
        for m in re.finditer(r'P(\d+)_(\d+)', c_str):
            all_refs.add((int(m.group(1)), int(m.group(2))))
    for expr in reduced_exprs:
        c_str = sp.ccode(expr)
        for m in re.finditer(r'P(\d+)_(\d+)', c_str):
            all_refs.add((int(m.group(1)), int(m.group(2))))
    return sorted(all_refs)


def emit_cpp(intermediates, reduced_exprs, output_path):
    """Emit the generated C++ file.

    CRITICAL: P is updated in-place, so we must snapshot all input P values
    into local variables before computing outputs. CSE intermediates and output
    expressions reference p_i_j locals (not P[i][j] directly) to avoid
    read-after-write hazards.
    """
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    sympy_ver = sp.__version__

    # Find all P elements referenced in the expressions
    p_refs = _collect_P_refs(intermediates, reduced_exprs)

    lines = []
    lines.append(f"// AUTO-GENERATED by scripts/generate_fpft.py -- DO NOT EDIT")
    lines.append(f"// SymPy {sympy_ver}, generated {now}")
    lines.append(f"// State size: {N}, CSE intermediates: {len(intermediates)}")
    lines.append(f"// Upper-triangle outputs: {len(reduced_exprs)}")
    lines.append(f"// P input snapshots: {len(p_refs)}")
    lines.append(f"//")
    lines.append(f"// Baked-in Q_d constants (must match eskf.h):")
    lines.append(f"//   kSigmaGyro         = {SIGMA_GYRO}")
    lines.append(f"//   kSigmaAccel        = {SIGMA_ACCEL}")
    lines.append(f"//   kSigmaGyroBiasWalk = {SIGMA_GYRO_BIAS_WALK}")
    lines.append(f"//   kSigmaAccelBiasWalk= {SIGMA_ACCEL_BIAS_WALK}")
    lines.append(f"//")
    lines.append(f"// Reference: Sola (2017) Eq. 269, PX4 ECL codegen pattern")
    lines.append(f"")
    lines.append(f"// R3: Sync constants for static_assert in eskf.cpp")
    lines.append(f"namespace codegen {{")
    lines.append(f"constexpr float kSigmaGyro = {SIGMA_GYRO}f;")
    lines.append(f"constexpr float kSigmaAccel = {SIGMA_ACCEL}f;")
    lines.append(f"constexpr float kSigmaGyroBiasWalk = {SIGMA_GYRO_BIAS_WALK}f;")
    lines.append(f"constexpr float kSigmaAccelBiasWalk = {SIGMA_ACCEL_BIAS_WALK}f;")
    lines.append(f"}}  // namespace codegen")
    lines.append(f"")
    lines.append(f"// Computes P = F * P * F^T + Q_d for 15-state ESKF")
    lines.append(f"// P: symmetric 15x15 covariance (updated in place)")
    lines.append(f"// R: 3x3 body-to-NED DCM")
    lines.append(f"// ax,ay,az: bias-corrected accelerometer (m/s^2)")
    lines.append(f"// wx,wy,wz: bias-corrected gyroscope (rad/s)")
    lines.append(f"// dt_val: time step (s)")
    lines.append(f"// Execute from SRAM to avoid XIP cache thrashing (10KB+ function)")
    lines.append(f"__attribute__((section(\".time_critical.codegen_fpft\")))")
    lines.append(f"void codegen_fpft(float P[15][15],")
    lines.append(f"                  const float R[3][3],")
    lines.append(f"                  float ax, float ay, float az,")
    lines.append(f"                  float wx, float wy, float wz,")
    lines.append(f"                  float dt_val) {{")

    # Snapshot P inputs into locals to avoid read-after-write hazards
    lines.append(f"    // Snapshot P inputs (in-place update safety)")
    for i, j in p_refs:
        lines.append(f"    const float p{i}_{j} = P[{i}][{j}];")
    lines.append(f"")

    # Emit CSE intermediates — use p_i_j locals instead of P[i][j]
    for sym, expr in intermediates:
        c_expr = sp.ccode(expr)
        # Replace P symbols: P0_1 -> p0_1 (local snapshot)
        c_expr = _replace_P_to_local(c_expr)
        # Replace R symbols: R01 -> R[0][1]
        c_expr = _replace_R_symbols(c_expr)
        # Replace dt -> dt_val
        c_expr = c_expr.replace('dt', 'dt_val')
        lines.append(f"    const float {sym} = {c_expr};")

    lines.append(f"")
    lines.append(f"    // Upper triangle assignments (120 elements)")

    # Emit upper triangle P assignments
    idx = 0
    for i in range(N):
        for j in range(i, N):
            c_expr = sp.ccode(reduced_exprs[idx])
            c_expr = _replace_P_to_local(c_expr)
            c_expr = _replace_R_symbols(c_expr)
            c_expr = c_expr.replace('dt', 'dt_val')
            lines.append(f"    P[{i}][{j}] = {c_expr};")
            idx += 1

    lines.append(f"")
    lines.append(f"    // Mirror lower triangle")
    for i in range(N):
        for j in range(i+1, N):
            lines.append(f"    P[{j}][{i}] = P[{i}][{j}];")

    lines.append(f"}}")
    lines.append(f"")

    with open(output_path, 'w', newline='\n') as f:
        f.write('\n'.join(lines))

    # Also emit header for sync constants (R3)
    header_path = output_path.replace('.cpp', '.h')
    hlines = []
    hlines.append(f"// AUTO-GENERATED by scripts/generate_fpft.py -- DO NOT EDIT")
    hlines.append(f"#ifndef ESKF_CODEGEN_H")
    hlines.append(f"#define ESKF_CODEGEN_H")
    hlines.append(f"")
    hlines.append(f"// Codegen FPFT function declaration")
    hlines.append(f"void codegen_fpft(float P[15][15], const float R[3][3],")
    hlines.append(f"                  float ax, float ay, float az,")
    hlines.append(f"                  float wx, float wy, float wz,")
    hlines.append(f"                  float dt_val);")
    hlines.append(f"")
    hlines.append(f"// R3: Sync constants — baked into codegen, must match eskf.h")
    hlines.append(f"namespace codegen {{")
    hlines.append(f"inline constexpr float kSigmaGyro = {SIGMA_GYRO}f;")
    hlines.append(f"inline constexpr float kSigmaAccel = {SIGMA_ACCEL}f;")
    hlines.append(f"inline constexpr float kSigmaGyroBiasWalk = {SIGMA_GYRO_BIAS_WALK}f;")
    hlines.append(f"inline constexpr float kSigmaAccelBiasWalk = {SIGMA_ACCEL_BIAS_WALK}f;")
    hlines.append(f"}}  // namespace codegen")
    hlines.append(f"")
    hlines.append(f"#endif  // ESKF_CODEGEN_H")
    hlines.append(f"")
    with open(header_path, 'w', newline='\n') as f:
        f.write('\n'.join(hlines))

    print(f"Generated {output_path}")
    print(f"Generated {header_path}")
    print(f"  CSE intermediates: {len(intermediates)}")
    print(f"  Upper triangle outputs: {len(reduced_exprs)}")
    print(f"  Total lines: {len(lines)} (.cpp) + {len(hlines)} (.h)")


def _replace_P_to_local(expr_str):
    """Replace P0_1 -> p0_1 (local snapshot variable) in C code string."""
    import re
    def repl(m):
        i, j = m.group(1), m.group(2)
        return f'p{i}_{j}'
    return re.sub(r'P(\d+)_(\d+)', repl, expr_str)


def _replace_R_symbols(expr_str):
    """Replace R01 -> R[0][1] etc. in C code string."""
    import re
    def repl(m):
        i, j = m.group(1), m.group(2)
        return f'R[{i}][{j}]'
    return re.sub(r'R(\d)(\d)', repl, expr_str)


def main():
    print(f"generate_fpft.py — SymPy {sp.__version__}")
    print(f"Building symbolic F ({N}x{N})...")
    F, dt_sym, R_sym, accel_syms, gyro_syms = build_symbolic_F()

    print(f"Building symbolic P ({N}x{N}, 120 unique symbols)...")
    P_sym = build_symbolic_P()

    print(f"Building symbolic Q_d ({N}x{N} diagonal)...")
    Qd_sym = build_Qd_diagonal(dt_sym)

    print(f"Computing F * P * F^T + Q_d symbolically...")
    FPFT = F * P_sym * F.T + Qd_sym

    # Self-test before CSE (R1)
    P_test_np, ref_np = run_self_test(F, P_sym, dt_sym, R_sym, accel_syms, gyro_syms)

    # Extract upper triangle expressions for CSE
    print(f"Extracting upper triangle (120 expressions)...")
    upper_tri_exprs = []
    for i in range(N):
        for j in range(i, N):
            upper_tri_exprs.append(FPFT[i, j])

    print(f"Running CSE (this may take a minute)...")
    intermediates, reduced = sp.cse(upper_tri_exprs, optimizations='basic')
    print(f"  CSE found {len(intermediates)} common subexpressions")

    # Emit C++
    output_path = "src/fusion/eskf_codegen.cpp"
    emit_cpp(intermediates, reduced, output_path)

    print("\nDone. Remember to:")
    print("  1. Add src/fusion/eskf_codegen.cpp to CMakeLists.txt")
    print("  2. Wire codegen_fpft() into ESKF::predict()")
    print("  3. Build and run tests (Test 8 validates codegen vs dense)")


if __name__ == '__main__':
    main()
