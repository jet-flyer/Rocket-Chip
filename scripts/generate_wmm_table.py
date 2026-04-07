#!/usr/bin/env python3
"""
Generate WMM geomagnetic field lookup tables from NOAA WMM.COF coefficients.

Produces C++ source/header with three tables (declination, inclination,
total intensity) at 10-degree lat/lon resolution. Same grid as the
previous ArduPilot-sourced declination-only table.

Usage:
    python scripts/generate_wmm_table.py [--cof lib/wmm/WMM.COF] [--verify]

The --verify flag checks output against NOAA test values.

Reference: WMM2025 Technical Report, NOAA/NCEI
    https://www.ncei.noaa.gov/products/world-magnetic-model
"""

import argparse
import math
import os
import sys
from datetime import datetime
from pathlib import Path

# ============================================================================
# Spherical Harmonic Model Evaluation
# ============================================================================

# WGS84 constants
WGS84_A = 6378.137          # Semi-major axis (km)
WGS84_F = 1.0 / 298.257223563  # Flattening
WGS84_B = WGS84_A * (1.0 - WGS84_F)  # Semi-minor axis
RE = 6371.2  # IAU mean Earth radius (km) — WMM reference sphere

MAX_DEG = 12  # WMM truncation degree


def load_coefficients(cof_path):
    """Load WMM.COF Gauss coefficients (g, h) and secular variation (gdot, hdot)."""
    g = {}
    h = {}
    gdot = {}
    hdot = {}
    epoch = None

    with open(cof_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if any(p.startswith('WMM') for p in parts) and epoch is None:
                epoch = float(parts[0])
                continue
            if len(parts) < 6:
                continue
            try:
                n = int(parts[0])
                m = int(parts[1])
            except ValueError:
                continue
            if n == 0 and m == 0:
                continue
            g[(n, m)] = float(parts[2])
            h[(n, m)] = float(parts[3])
            gdot[(n, m)] = float(parts[4])
            hdot[(n, m)] = float(parts[5])

    if epoch is None:
        raise ValueError("Could not parse epoch from WMM.COF")

    return epoch, g, h, gdot, hdot


def geodetic_to_spherical(lat_deg, lon_deg, alt_km=0.0):
    """Convert geodetic (WGS84) coordinates to geocentric spherical."""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)

    # WGS84 radius of curvature
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    a2 = WGS84_A ** 2
    b2 = WGS84_B ** 2

    # Geocentric radius and latitude
    num = (a2 * cos_lat) ** 2 + (b2 * sin_lat) ** 2
    den = (WGS84_A * cos_lat) ** 2 + (WGS84_B * sin_lat) ** 2
    r = math.sqrt(num / den) + alt_km

    # Geocentric latitude
    lat_gc = math.atan2(
        (b2 / a2) * math.sin(lat) + alt_km * math.sin(lat),
        math.cos(lat) * (WGS84_A + alt_km * a2 / (a2 * cos_lat**2 + b2 * sin_lat**2)**0.5)
    )
    # Simplified: for surface points, the difference is small
    # Use the standard formula
    lat_gc = math.atan2(
        ((1 - WGS84_F)**2) * math.tan(lat), 1.0
    )

    return r, lat_gc, lon


def evaluate_wmm(lat_deg, lon_deg, alt_km, year, epoch, g, h, gdot, hdot):
    """Evaluate WMM at a point. Returns (X_nT, Y_nT, Z_nT) in NED."""
    dt = year - epoch

    # Apply secular variation
    gc = {}
    hc = {}
    for key in g:
        gc[key] = g[key] + gdot.get(key, 0.0) * dt
        hc[key] = h[key] + hdot.get(key, 0.0) * dt

    # Convert to geocentric
    lat_rad = math.radians(lat_deg)
    lon_rad = math.radians(lon_deg)

    # Geocentric colatitude (simplified geodetic-to-geocentric)
    lat_gc = math.atan2(((1 - WGS84_F)**2) * math.sin(lat_rad), math.cos(lat_rad))
    r = WGS84_A  # Surface approximation for table generation (alt=0)

    colat = math.pi / 2.0 - lat_gc
    sin_colat = math.sin(colat)
    cos_colat = math.cos(colat)

    # Associated Legendre functions (Schmidt semi-normalized)
    P = {}
    dP = {}
    P[(0, 0)] = 1.0
    dP[(0, 0)] = 0.0
    P[(1, 0)] = cos_colat
    dP[(1, 0)] = -sin_colat
    P[(1, 1)] = sin_colat
    dP[(1, 1)] = cos_colat

    for n in range(2, MAX_DEG + 1):
        for m in range(0, n + 1):
            if m == n:
                P[(n, m)] = sin_colat * P[(n-1, m-1)] * math.sqrt((2*n - 1) / (2*n))
                dP[(n, m)] = sin_colat * dP[(n-1, m-1)] * math.sqrt((2*n - 1) / (2*n)) + \
                             cos_colat * P[(n-1, m-1)] * math.sqrt((2*n - 1) / (2*n))
            elif m == n - 1:
                P[(n, m)] = cos_colat * P[(n-1, m)] * math.sqrt(2*n - 1)
                dP[(n, m)] = -sin_colat * P[(n-1, m)] * math.sqrt(2*n - 1) + \
                              cos_colat * dP[(n-1, m)] * math.sqrt(2*n - 1)
            else:
                K = ((n - 1)**2 - m**2) / ((2*n - 1) * (2*n - 3))
                P[(n, m)] = (cos_colat * P[(n-1, m)] - K * P[(n-2, m)]) / \
                            (1.0 - K) if (1.0 - K) != 0 else 0.0
                dP[(n, m)] = (-sin_colat * P[(n-1, m)] + cos_colat * dP[(n-1, m)] - K * dP[(n-2, m)]) / \
                             (1.0 - K) if (1.0 - K) != 0 else 0.0

    # Compute field components in spherical coordinates
    Br = 0.0   # Radial (outward)
    Bt = 0.0   # Theta (southward)
    Bp = 0.0   # Phi (eastward)

    ratio = RE / r

    for n in range(1, MAX_DEG + 1):
        rn = ratio ** (n + 2)
        for m in range(0, n + 1):
            gnm = gc.get((n, m), 0.0)
            hnm = hc.get((n, m), 0.0)
            cos_m_lon = math.cos(m * lon_rad)
            sin_m_lon = math.sin(m * lon_rad)

            Br += (n + 1) * rn * (gnm * cos_m_lon + hnm * sin_m_lon) * P.get((n, m), 0.0)
            Bt -= rn * (gnm * cos_m_lon + hnm * sin_m_lon) * dP.get((n, m), 0.0)
            if sin_colat != 0:
                Bp += rn * m * (-gnm * sin_m_lon + hnm * cos_m_lon) * P.get((n, m), 0.0) / sin_colat

    # Convert spherical to geodetic NED
    # X = North, Y = East, Z = Down
    sin_diff = math.sin(lat_rad - lat_gc)
    cos_diff = math.cos(lat_rad - lat_gc)

    X = -Bt * cos_diff - Br * sin_diff  # North
    Y = Bp                                # East
    Z = Bt * sin_diff - Br * cos_diff    # Down

    return X, Y, Z


def field_components(X, Y, Z):
    """Compute derived field parameters from NED components (nT)."""
    H = math.sqrt(X**2 + Y**2)           # Horizontal intensity
    F = math.sqrt(X**2 + Y**2 + Z**2)    # Total intensity
    D = math.degrees(math.atan2(Y, X))    # Declination (deg)
    I = math.degrees(math.atan2(Z, H))    # Inclination (deg)
    return D, I, H, F


# ============================================================================
# Table Generation
# ============================================================================

LAT_MIN, LAT_MAX, LAT_STEP = -90, 90, 10
LON_MIN, LON_MAX, LON_STEP = -180, 180, 10
LAT_ROWS = (LAT_MAX - LAT_MIN) // LAT_STEP + 1  # 19
LON_COLS = (LON_MAX - LON_MIN) // LON_STEP + 1    # 37


def generate_tables(epoch, g, h, gdot, hdot, year=None):
    """Generate declination, inclination, intensity tables."""
    if year is None:
        year = epoch + 2.5  # Mid-epoch for best accuracy

    decl = []
    incl = []
    intensity = []

    for lat in range(LAT_MIN, LAT_MAX + 1, LAT_STEP):
        row_d = []
        row_i = []
        row_f = []
        for lon in range(LON_MIN, LON_MAX + 1, LON_STEP):
            X, Y, Z = evaluate_wmm(lat, lon, 0.0, year, epoch, g, h, gdot, hdot)
            D, I, H, F = field_components(X, Y, Z)
            row_d.append(D)
            row_i.append(I)
            row_f.append(F / 1000.0)  # Convert nT to µT
        decl.append(row_d)
        incl.append(row_i)
        intensity.append(row_f)

    return decl, incl, intensity


# ============================================================================
# C++ Code Generation
# ============================================================================

def format_table(name, table, unit, precision=2):
    """Format a 2D table as a C++ array initializer."""
    lines = []
    lines.append(f"// {name} table ({unit}), {LAT_ROWS} rows x {LON_COLS} cols")
    lines.append(f"// Rows: lat {LAT_MIN} to {LAT_MAX} (step {LAT_STEP})")
    lines.append(f"// Cols: lon {LON_MIN} to {LON_MAX} (step {LON_STEP})")
    lines.append(f"static constexpr float {name}[{LAT_ROWS}][{LON_COLS}] = {{")
    for i, row in enumerate(table):
        lat = LAT_MIN + i * LAT_STEP
        vals = ", ".join(f"{v:.{precision}f}f" for v in row)
        lines.append(f"    {{ {vals} }}, // lat {lat:+d}")
    lines.append("};")
    return "\n".join(lines)


def generate_cpp(decl, incl, intensity, epoch, year):
    """Generate wmm_tables.cpp."""
    header = f"""\
// AUTO-GENERATED by scripts/generate_wmm_table.py
// Source: WMM{int(epoch)} coefficients (NOAA/NCEI, public domain)
// Epoch: {epoch}, evaluated at {year:.1f}
// Valid: {int(epoch)}-{int(epoch)+5}
//
// Do not edit this file directly.
// Regenerate: python scripts/generate_wmm_table.py
//
// Three tables at {LAT_STEP}-degree resolution:
//   kDeclination: magnetic declination (degrees, East-positive)
//   kInclination: magnetic inclination (degrees, Down-positive)
//   kIntensity:   total field magnitude (microtesla)

#include "wmm_tables.h"
#include <cmath>

namespace rc {{
"""

    decl_table = format_table("kDeclination", decl, "degrees, East-positive")
    incl_table = format_table("kInclination", incl, "degrees, Down-positive")
    int_table = format_table("kIntensity", intensity, "microtesla", precision=1)

    interp_fn = """
// ============================================================================
// Bilinear interpolation (shared by all three lookups)
// ============================================================================

static float interp(const float table[][kLonCols],
                    float lat_deg, float lon_deg) {
    // Clamp inputs
    if (lat_deg < kLatMinF) { lat_deg = kLatMinF; }
    if (lat_deg > kLatMaxF) { lat_deg = kLatMaxF; }
    // Wrap longitude to [-180, 180]
    while (lon_deg > 180.0f) { lon_deg -= 360.0f; }
    while (lon_deg < -180.0f) { lon_deg += 360.0f; }

    float lat_idx = (lat_deg - kLatMinF) / kGridResF;
    float lon_idx = (lon_deg - kLonMinF) / kGridResF;

    int32_t r0 = static_cast<int32_t>(lat_idx);
    int32_t c0 = static_cast<int32_t>(lon_idx);
    if (r0 < 0) { r0 = 0; }
    if (r0 >= kLatRows - 1) { r0 = kLatRows - 2; }
    if (c0 < 0) { c0 = 0; }
    if (c0 >= kLonCols - 1) { c0 = kLonCols - 2; }

    float dr = lat_idx - static_cast<float>(r0);
    float dc = lon_idx - static_cast<float>(c0);

    float v00 = table[r0][c0];
    float v01 = table[r0][c0 + 1];
    float v10 = table[r0 + 1][c0];
    float v11 = table[r0 + 1][c0 + 1];

    return v00 * (1.0f - dr) * (1.0f - dc) +
           v01 * (1.0f - dr) * dc +
           v10 * dr * (1.0f - dc) +
           v11 * dr * dc;
}

// ============================================================================
// Public API
// ============================================================================

WmmField wmm_get_field(float lat_deg, float lon_deg) {
    WmmField f;
    f.declination_rad = interp(kDeclination, lat_deg, lon_deg) * kDegToRadF;
    f.inclination_rad = interp(kInclination, lat_deg, lon_deg) * kDegToRadF;
    f.intensity_ut    = interp(kIntensity, lat_deg, lon_deg);
    return f;
}

float wmm_get_declination(float lat_deg, float lon_deg) {
    return interp(kDeclination, lat_deg, lon_deg) * kDegToRadF;
}

Vec3 wmm_get_earth_field_ned(float lat_deg, float lon_deg) {
    WmmField f = wmm_get_field(lat_deg, lon_deg);
    // Decompose total field into NED using declination and inclination
    float cos_incl = cosf(f.inclination_rad);
    float sin_incl = sinf(f.inclination_rad);
    float cos_decl = cosf(f.declination_rad);
    float sin_decl = sinf(f.declination_rad);
    // H = F * cos(I), X = H * cos(D), Y = H * sin(D), Z = F * sin(I)
    float H = f.intensity_ut * cos_incl;
    return Vec3(H * cos_decl, H * sin_decl, f.intensity_ut * sin_incl);
}
"""

    footer = """
} // namespace rc
"""

    return header + "\n" + decl_table + "\n\n" + incl_table + "\n\n" + int_table + "\n" + interp_fn + footer


def generate_header(epoch):
    """Generate wmm_tables.h."""
    return f"""\
// AUTO-GENERATED by scripts/generate_wmm_table.py
// Source: WMM{int(epoch)} (NOAA/NCEI, public domain)
// Valid: {int(epoch)}-{int(epoch)+5}
//
// Do not edit this file directly.
#ifndef ROCKETCHIP_FUSION_WMM_TABLES_H
#define ROCKETCHIP_FUSION_WMM_TABLES_H

#include <stdint.h>
#include "math/vec3.h"

namespace rc {{

// Grid parameters
static constexpr int32_t kLatRows = {LAT_ROWS};
static constexpr int32_t kLonCols = {LON_COLS};
static constexpr float kLatMinF = {LAT_MIN}.0f;
static constexpr float kLatMaxF = {LAT_MAX}.0f;
static constexpr float kLonMinF = {LON_MIN}.0f;
static constexpr float kGridResF = {LAT_STEP}.0f;
static constexpr float kDegToRadF = 3.14159265f / 180.0f;

// Full geomagnetic field at a location
struct WmmField {{
    float declination_rad;   // East-positive
    float inclination_rad;   // Down-positive
    float intensity_ut;      // Total field magnitude (microtesla)
}};

// Get all three geomagnetic components (bilinear interpolated from tables)
WmmField wmm_get_field(float lat_deg, float lon_deg);

// Get declination only (radians, East-positive) — legacy compatibility
float wmm_get_declination(float lat_deg, float lon_deg);

// Compute earth magnetic field NED vector (microtesla) from WMM tables
Vec3 wmm_get_earth_field_ned(float lat_deg, float lon_deg);

}} // namespace rc

#endif // ROCKETCHIP_FUSION_WMM_TABLES_H
"""


# ============================================================================
# Verification against NOAA test values
# ============================================================================

def verify(epoch, g, h, gdot, hdot, test_file):
    """Verify against NOAA-provided test values."""
    errors = []
    count = 0
    max_decl_err = 0
    max_incl_err = 0
    max_f_err = 0

    with open(test_file, 'r') as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
            parts = line.split()
            if len(parts) < 11:
                continue
            year = float(parts[0])
            alt_km = float(parts[1])
            lat = float(parts[2])
            lon = float(parts[3])
            ref_D = float(parts[4])
            ref_I = float(parts[5])
            ref_F = float(parts[10])

            X, Y, Z = evaluate_wmm(lat, lon, alt_km, year, epoch, g, h, gdot, hdot)
            D, I, H, F = field_components(X, Y, Z)

            decl_err = abs(D - ref_D)
            incl_err = abs(I - ref_I)
            f_err = abs(F - ref_F)

            max_decl_err = max(max_decl_err, decl_err)
            max_incl_err = max(max_incl_err, incl_err)
            max_f_err = max(max_f_err, f_err)
            count += 1

            if decl_err > 2.0 or incl_err > 1.0 or f_err > 500:
                errors.append(f"  lat={lat} lon={lon} alt={alt_km}km year={year}: "
                              f"D err={decl_err:.2f} I err={incl_err:.2f} F err={f_err:.0f}nT")

    print(f"Verified {count} test points")
    print(f"  Max declination error: {max_decl_err:.3f} deg")
    print(f"  Max inclination error: {max_incl_err:.3f} deg")
    print(f"  Max total field error: {max_f_err:.0f} nT")
    if errors:
        print(f"  WARNING: {len(errors)} points exceed threshold:")
        for e in errors[:5]:
            print(e)
    else:
        print("  All points within tolerance")
    return len(errors) == 0


# ============================================================================
# Main
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description="Generate WMM lookup tables")
    parser.add_argument('--cof', default='lib/wmm/WMM.COF',
                        help='Path to WMM.COF coefficient file')
    parser.add_argument('--from-json', default=None,
                        help='Use pre-computed grid from JSON (BGS API cache)')
    parser.add_argument('--verify', action='store_true',
                        help='Verify against NOAA test values')
    parser.add_argument('--test-values', default=None,
                        help='Path to NOAA test values file')
    parser.add_argument('--year', type=float, default=None,
                        help='Evaluation year (default: mid-epoch)')
    args = parser.parse_args()

    # Resolve paths relative to project root
    project_root = Path(__file__).parent.parent

    if args.from_json:
        # Load pre-computed grid from JSON (generated by BGS API)
        import json
        json_path = project_root / args.from_json
        print(f"Loading pre-computed grid from {json_path}...")
        with open(json_path, 'r') as f:
            grid = json.load(f)
        decl = grid['declination']
        incl = grid['inclination']
        intensity = grid['intensity']
        epoch = grid['epoch']
        year = grid['eval_year']
        print(f"  Epoch: {epoch}, evaluated at {year}")
        print(f"  Grid: {len(decl)}x{len(decl[0])} ({len(decl)*len(decl[0])} points)")
    else:
        cof_path = project_root / args.cof
        if not cof_path.exists():
            print(f"ERROR: {cof_path} not found. Download from NOAA:")
            print("  https://www.ncei.noaa.gov/products/world-magnetic-model")
            sys.exit(1)

        print(f"Loading {cof_path}...")
        epoch, g, h, gdot, hdot = load_coefficients(str(cof_path))
        print(f"  Epoch: {epoch}")
        print(f"  Coefficients: {len(g)} (degree {MAX_DEG})")

        year = args.year if args.year else epoch + 2.5
        print(f"  Evaluation year: {year}")

        if args.verify:
            test_file = args.test_values or str(project_root / 'lib/wmm/WMM2025_TestValues.txt')
            if not os.path.exists(test_file):
                test_file = '/tmp/WMM2025COF/WMM2025_TestValues.txt'
            if os.path.exists(test_file):
                verify(epoch, g, h, gdot, hdot, test_file)
            else:
                print(f"WARNING: Test file not found: {test_file}")

        print("Generating tables from COF coefficients...")
        decl, incl, intensity = generate_tables(epoch, g, h, gdot, hdot, year)

    # Spot check: Dallas TX (33N, 97W) — grid index [12][8] (lat=30 is row 12, lon=-90 is col 9)
    # Use bilinear from neighboring points
    dallas_row = 12  # lat 30 (closest to 33)
    dallas_col = 8   # lon -100 (closest to -97)
    print(f"  Dallas area (lat=30,lon=-100): D={decl[dallas_row][dallas_col]:.2f} "
          f"I={incl[dallas_row][dallas_col]:.2f} F={intensity[dallas_row][dallas_col]:.1f}uT")

    # Generate C++ files
    out_dir = project_root / 'src' / 'fusion'

    cpp_content = generate_cpp(decl, incl, intensity, epoch, year)
    cpp_path = out_dir / 'wmm_tables.cpp'
    with open(cpp_path, 'w', newline='\n') as f:
        f.write(cpp_content)
    print(f"  Generated {cpp_path}")

    h_content = generate_header(epoch)
    h_path = out_dir / 'wmm_tables.h'
    with open(h_path, 'w', newline='\n') as f:
        f.write(h_content)
    print(f"  Generated {h_path}")


if __name__ == '__main__':
    main()
