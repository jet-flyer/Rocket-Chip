#!/usr/bin/env python3
"""
RocketChip Mission Profile Generator (IVP-74)

Parses a human-readable .cfg file and generates a C++ header with a
constexpr MissionProfile struct. The generated header is compiled into
firmware — no runtime config loading.

Usage:
    python scripts/generate_profile.py profiles/rocket.cfg
    python scripts/generate_profile.py profiles/hab.cfg --symbol kHabProfile

The .cfg format is ArduPilot .param inspired: NAME VALUE # comment
"""

import argparse
import hashlib
import os
import sys
from pathlib import Path

# ============================================================================
# Field definitions: (cfg_name, cpp_field, type, validation)
#
# Types: 'float', 'uint32', 'bool'
# Validation: (min, max) or None for bools
# ============================================================================
FIELDS = [
    # IMPORTANT: Order must match MissionProfile struct declaration in
    # mission_profile.h. Designated initializers require declaration order.
    #
    # Timing
    ('ARMED_TIMEOUT_MS',    'armed_timeout_ms',          'uint32', (1000, 3600000)),
    ('ABORT_TIMEOUT_MS',    'abort_timeout_ms',           'uint32', (1000, 3600000)),
    ('COAST_TIMEOUT_MS',    'coast_timeout_ms',           'uint32', (1000, 600000)),
    # Guard thresholds
    ('LAUNCH_ACCEL',        'launch_accel_threshold',     'float',  (0.5, 500.0)),
    ('LAUNCH_HOLD_MS',      'launch_sustain_ms',          'uint32', (10, 5000)),
    ('BURNOUT_ACCEL',       'burnout_accel_threshold',    'float',  (0.1, 100.0)),
    ('BURNOUT_HOLD_MS',     'burnout_sustain_ms',         'uint32', (10, 5000)),
    ('APOGEE_VEL',          'apogee_velocity_threshold',  'float',  (0.01, 50.0)),
    ('APOGEE_HOLD_MS',      'apogee_sustain_ms',          'uint32', (10, 5000)),
    ('BARO_HOLD_MS',        'baro_peak_sustain_ms',       'uint32', (10, 5000)),
    ('MAIN_ALT_M',          'main_deploy_altitude_m',     'float',  (10.0, 10000.0)),
    ('MAIN_HOLD_MS',        'main_deploy_sustain_ms',     'uint32', (10, 5000)),
    ('LAND_VEL',            'landing_velocity_threshold', 'float',  (0.01, 10.0)),
    ('LAND_HOLD_MS',        'landing_sustain_ms',         'uint32', (100, 30000)),
    # Safety lockouts
    ('DEPLOY_LOCKOUT_MPS',  'deploy_lockout_mps',         'float',  (1.0, 1000.0)),
    ('APOGEE_LOCKOUT_MS',   'apogee_lockout_ms',          'uint32', (0, 60000)),
    # Timer backups
    ('BURNOUT_BACKUP_MS',   'burnout_backup_ms',          'uint32', (1000, 300000)),
    ('MAIN_BACKUP_MS',      'main_backup_ms',             'uint32', (5000, 3600000)),
    # Combinator config
    ('APOGEE_BOTH',         'apogee_require_both',        'bool',   None),
    # Emergency override
    ('EMERG_DEPLOY',        'emergency_deploy_anytime',   'bool',   None),
    # Abort behavior
    ('ABORT_DROGUE_BOOST',  'abort_fires_drogue_from_boost', 'bool', None),
    ('ABORT_DROGUE_COAST',  'abort_fires_drogue_from_coast', 'bool', None),
    # Pre-arm checks
    ('REQUIRE_GPS',         'require_gps_lock',           'bool',   None),
    ('REQUIRE_MAG',         'require_mag_cal',            'bool',   None),
    ('REQUIRE_RADIO',       'require_radio',              'bool',   None),
    # Pyro (last in struct)
    ('HAS_PYRO',            'has_pyro',                   'bool',   None),
]

# Profile ID mapping
PROFILE_IDS = {
    'Rocket':   'ProfileId::kRocket',
    'HAB':      'ProfileId::kHab',
    'Freeform': 'ProfileId::kFreeform',
}


def parse_cfg(filepath):
    """Parse a .cfg file into a dict of {NAME: raw_string_value}."""
    params = {}
    name = None

    with open(filepath, 'r') as f:
        for lineno, line in enumerate(f, 1):
            stripped = line.strip()
            if not stripped or stripped.startswith('#'):
                continue

            # Strip inline comment
            if '#' in stripped:
                stripped = stripped[:stripped.index('#')].strip()

            parts = stripped.split()
            if len(parts) < 2:
                print(f'WARNING: line {lineno}: expected NAME VALUE, got: {stripped}')
                continue

            key = parts[0]
            val = parts[1]

            if key == 'PROFILE_NAME':
                name = val
            else:
                if key in params:
                    print(f'WARNING: line {lineno}: duplicate key {key}')
                params[key] = val

    return name, params


def validate_and_convert(params):
    """Validate all fields and convert to typed values. Returns dict or exits."""
    result = {}
    errors = []

    for cfg_name, cpp_field, ftype, bounds in FIELDS:
        if cfg_name not in params:
            errors.append(f'Missing required field: {cfg_name}')
            continue

        raw = params[cfg_name]

        try:
            if ftype == 'float':
                val = float(raw)
                if bounds and (val < bounds[0] or val > bounds[1]):
                    errors.append(
                        f'{cfg_name} = {val}: out of range '
                        f'[{bounds[0]}, {bounds[1]}]')
                result[cpp_field] = ('float', val)

            elif ftype == 'uint32':
                val = int(raw)
                if val < 0:
                    errors.append(f'{cfg_name} = {val}: must be non-negative')
                elif bounds and (val < bounds[0] or val > bounds[1]):
                    errors.append(
                        f'{cfg_name} = {val}: out of range '
                        f'[{bounds[0]}, {bounds[1]}]')
                result[cpp_field] = ('uint32', val)

            elif ftype == 'bool':
                if raw not in ('0', '1'):
                    errors.append(f'{cfg_name} = {raw}: must be 0 or 1')
                else:
                    result[cpp_field] = ('bool', raw == '1')

        except ValueError:
            errors.append(f'{cfg_name} = {raw}: invalid {ftype} value')

    # Check for unknown fields
    known = {f[0] for f in FIELDS}
    for key in params:
        if key not in known:
            print(f'WARNING: unknown field: {key} (ignored)')

    if errors:
        print('ERRORS in profile:')
        for e in errors:
            print(f'  - {e}')
        sys.exit(1)

    return result


def generate_header(name, values, cfg_path, symbol, output_path):
    """Generate the C++ header file."""
    # Compute source file hash for traceability (A2)
    with open(cfg_path, 'rb') as f:
        src_hash = hashlib.sha256(f.read()).hexdigest()[:16]

    profile_id = PROFILE_IDS.get(name, 'ProfileId::kRocket')

    lines = []
    lines.append('// AUTO-GENERATED by scripts/generate_profile.py')
    lines.append(f'// GENERATED_FROM: {cfg_path} (sha256: {src_hash})')
    lines.append('//')
    lines.append('// Do not edit this file directly.')
    lines.append(f'// Edit {cfg_path} and re-run the generator.')
    lines.append('')
    lines.append('#pragma once')
    lines.append('#include "mission_profile.h"')
    lines.append('')
    lines.append('namespace rc {')
    lines.append('')
    lines.append(f'inline constexpr MissionProfile {symbol} = {{')
    lines.append(f'    .id = {profile_id},')

    # Name field — truncate to 15 chars (char[16] with null)
    safe_name = name[:15]
    lines.append(f'    .name = "{safe_name}",')
    lines.append('')

    # Emit fields in struct order
    for _, cpp_field, _, _ in FIELDS:
        ftype, val = values[cpp_field]
        if ftype == 'float':
            lines.append(f'    .{cpp_field} = {val}f,')
        elif ftype == 'uint32':
            lines.append(f'    .{cpp_field} = {val},')
        elif ftype == 'bool':
            lines.append(f'    .{cpp_field} = {"true" if val else "false"},')

    lines.append('};')
    lines.append('')

    # Static assertions (A3)
    lines.append(f'// Compile-time validation')
    lines.append(f'static_assert({symbol}.launch_accel_threshold > 0.0f,')
    lines.append(f'              "LAUNCH_ACCEL must be positive");')
    lines.append(f'static_assert({symbol}.main_deploy_altitude_m > 0.0f,')
    lines.append(f'              "MAIN_ALT_M must be positive");')
    lines.append(f'static_assert({symbol}.armed_timeout_ms > 0,')
    lines.append(f'              "ARMED_TIMEOUT_MS must be positive");')
    lines.append(f'static_assert({symbol}.landing_sustain_ms >= 100,')
    lines.append(f'              "LAND_HOLD_MS must be at least 100ms");')
    lines.append('')
    lines.append('} // namespace rc')
    lines.append('')

    content = '\n'.join(lines)

    with open(output_path, 'w', newline='\n') as f:
        f.write(content)


def main():
    parser = argparse.ArgumentParser(
        description='Generate C++ MissionProfile header from .cfg file')
    parser.add_argument('config', help='Path to .cfg file')
    parser.add_argument('--symbol', default='kDefaultRocketProfile',
                        help='C++ symbol name (default: kDefaultRocketProfile)')
    parser.add_argument('--output',
                        default='src/flight_director/mission_profile_data.h',
                        help='Output header path')
    args = parser.parse_args()

    if not os.path.exists(args.config):
        print(f'ERROR: Config file not found: {args.config}')
        sys.exit(1)

    name, params = parse_cfg(args.config)
    if name is None:
        print('ERROR: PROFILE_NAME not found in config')
        sys.exit(1)

    values = validate_and_convert(params)

    generate_header(name, values, args.config, args.symbol, args.output)

    print(f'Generated {args.output}')
    print(f'  Profile: {name}')
    print(f'  Symbol:  {args.symbol}')
    print(f'  Fields:  {len(values)}')
    print(f'  Source:  {args.config}')


if __name__ == '__main__':
    main()
