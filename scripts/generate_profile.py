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
    # Pyro
    ('HAS_PYRO',            'has_pyro',                   'bool',   None),
]

# Phase Q/R fields — 8-value rows (one value per flight phase).
# Order: IDLE ARMED BOOST COAST DROGUE_DESC MAIN_DESC LANDED ABORT
# These map to MissionProfile.phase_qr (PhaseQRTable struct).
QR_FIELDS = [
    # (cfg_name, struct_path, bounds)
    # Q scales — multipliers on baseline sigma^2 (must be >= 1.0)
    ('Q_ATT_SCALE',   'q_scale.attitude',   (1.0, 1000.0)),
    ('Q_VEL_SCALE',   'q_scale.velocity',   (1.0, 1000.0)),
    ('Q_ABIAS_SCALE', 'q_scale.accel_bias', (1.0, 1000.0)),
    ('Q_GBIAS_SCALE', 'q_scale.gyro_bias',  (1.0, 1000.0)),
    # R values — absolute measurement noise (must be > 0)
    ('R_BARO',        'r.r_baro',           (1e-6, 1e6)),
    ('R_MAG',         'r.r_mag',            (1e-6, 1e6)),
    ('R_GPS_POS',     'r.r_gps_pos',        (1e-6, 1e6)),
    ('R_GPS_VEL',     'r.r_gps_vel',        (1e-6, 1e6)),
]

# Radio config fields (IVP-96) — optional, defaults used if absent
RADIO_FIELDS = [
    # (cfg_name, default_value, type, validation)
    ('RADIO_PROTOCOL',  0,  'uint8',  (0, 1)),     # 0=CCSDS, 1=MAVLink
    ('RADIO_RATE_HZ',   2,  'uint8',  (1, 10)),    # 1-10 Hz
    ('RADIO_POWER_DBM', 20, 'uint8',  (2, 20)),    # 2-20 dBm
]

NUM_PHASES = 8  # matches FlightPhase::kCount

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

            if key == 'PROFILE_NAME':
                name = parts[1]
            else:
                if key in params:
                    print(f'WARNING: line {lineno}: duplicate key {key}')
                # Store all values (multi-value for Q/R rows, single for others)
                params[key] = parts[1:]

    return name, params


def validate_and_convert(params):
    """Validate all fields and convert to typed values. Returns (scalars, qr) or exits."""
    result = {}
    errors = []

    for cfg_name, cpp_field, ftype, bounds in FIELDS:
        if cfg_name not in params:
            errors.append(f'Missing required field: {cfg_name}')
            continue

        raw = params[cfg_name][0]  # scalar fields: first (only) value

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

    # Validate Q/R phase fields (8 values per row)
    qr_data = {}  # cfg_name -> [float; 8]
    for cfg_name, struct_path, bounds in QR_FIELDS:
        if cfg_name not in params:
            errors.append(f'Missing required Q/R field: {cfg_name}')
            continue

        raw_vals = params[cfg_name]
        if len(raw_vals) != NUM_PHASES:
            errors.append(
                f'{cfg_name}: expected {NUM_PHASES} values, got {len(raw_vals)}')
            continue

        try:
            vals = [float(v) for v in raw_vals]
        except ValueError:
            errors.append(f'{cfg_name}: non-numeric value in row')
            continue

        for i, v in enumerate(vals):
            if v < bounds[0] or v > bounds[1]:
                errors.append(
                    f'{cfg_name}[{i}] = {v}: out of range '
                    f'[{bounds[0]}, {bounds[1]}]')
        qr_data[cfg_name] = vals

    # Validate QR_RAMP_STEPS (scalar)
    ramp_steps = 20  # default
    if 'QR_RAMP_STEPS' in params:
        try:
            ramp_steps = int(params['QR_RAMP_STEPS'][0])
            if ramp_steps < 1 or ramp_steps > 100:
                errors.append(
                    f'QR_RAMP_STEPS = {ramp_steps}: out of range [1, 100]')
        except ValueError:
            errors.append(f'QR_RAMP_STEPS: invalid integer')

    # Parse radio config fields (optional, defaults used if absent)
    radio_cfg = {}
    for cfg_name, default_val, ftype, bounds in RADIO_FIELDS:
        if cfg_name in params:
            try:
                val = int(params[cfg_name][0])
                if bounds and (val < bounds[0] or val > bounds[1]):
                    errors.append(
                        f'{cfg_name} = {val}: out of range '
                        f'[{bounds[0]}, {bounds[1]}]')
                radio_cfg[cfg_name] = val
            except ValueError:
                errors.append(f'{cfg_name}: invalid integer')
                radio_cfg[cfg_name] = default_val
        else:
            radio_cfg[cfg_name] = default_val

    # Check for unknown fields
    known = {f[0] for f in FIELDS}
    known.update(f[0] for f in QR_FIELDS)
    known.update(f[0] for f in RADIO_FIELDS)
    known.add('QR_RAMP_STEPS')
    for key in params:
        if key not in known:
            print(f'WARNING: unknown field: {key} (ignored)')

    if errors:
        print('ERRORS in profile:')
        for e in errors:
            print(f'  - {e}')
        sys.exit(1)

    return result, qr_data, ramp_steps, radio_cfg


def _emit_phase_qr(qr_data, ramp_steps):
    """Emit PhaseQRTable designated initializer block."""
    # Phase names for comments
    phase_names = ['IDLE', 'ARMED', 'BOOST', 'COAST',
                   'DROGUE_DESC', 'MAIN_DESC', 'LANDED', 'ABORT']

    # Build per-phase entries from the row data
    # QR_FIELDS order: Q_ATT, Q_VEL, Q_ABIAS, Q_GBIAS, R_BARO, R_MAG, R_GPS_POS, R_GPS_VEL
    q_att   = qr_data.get('Q_ATT_SCALE',   [1.0] * NUM_PHASES)
    q_vel   = qr_data.get('Q_VEL_SCALE',   [1.0] * NUM_PHASES)
    q_abias = qr_data.get('Q_ABIAS_SCALE', [1.0] * NUM_PHASES)
    q_gbias = qr_data.get('Q_GBIAS_SCALE', [1.0] * NUM_PHASES)
    r_baro  = qr_data.get('R_BARO',        [0.001] * NUM_PHASES)
    r_mag   = qr_data.get('R_MAG',         [0.008] * NUM_PHASES)
    r_gpos  = qr_data.get('R_GPS_POS',     [12.25] * NUM_PHASES)
    r_gvel  = qr_data.get('R_GPS_VEL',     [0.25] * NUM_PHASES)

    lines = []
    lines.append('    .phase_qr = {.phases = {')
    for i in range(NUM_PHASES):
        lines.append(f'        // {phase_names[i]} ({i})')
        lines.append(f'        {{.q_scale = {{{q_att[i]}f, {q_vel[i]}f, '
                     f'{q_abias[i]}f, {q_gbias[i]}f}},')
        lines.append(f'         .r = {{{r_baro[i]}f, {r_mag[i]}f, '
                     f'{r_gpos[i]}f, {r_gvel[i]}f}}}},')
    lines.append(f'    }}, .ramp_steps = {ramp_steps}}},')
    return lines


def _derive_rf_params(radio_cfg):
    """Derive SF/BW/CR from protocol + rate."""
    protocol = radio_cfg['RADIO_PROTOCOL']
    rate = radio_cfg['RADIO_RATE_HZ']

    # Packet size: CCSDS=54B, MAVLink=105B
    pkt_size = 54 if protocol == 0 else 105

    # At SF7, airtime per packet:
    #   BW125: ~100ms (54B), ~200ms (105B)
    #   BW250: ~50ms (54B),  ~100ms (105B)
    #   BW500: ~25ms (54B),  ~50ms (105B)
    # Max duty cycle target: 50% → max rate = 1000 / (2 * airtime_ms)

    # Start with BW125 (longest range), increase if rate requires it
    if protocol == 0:  # CCSDS 54B
        if rate <= 5:
            bw = 125  # airtime ~100ms, 5Hz = 50% duty
        else:
            bw = 250  # airtime ~50ms, 10Hz = 50% duty
    else:  # MAVLink 105B
        if rate <= 2:
            bw = 125  # airtime ~200ms, 2Hz = 40% duty
        elif rate <= 5:
            bw = 250  # airtime ~100ms, 5Hz = 50% duty
        else:
            bw = 500  # airtime ~50ms, 10Hz = 50% duty

    return 7, bw, 5  # SF7, derived BW, CR 4/5


def generate_header(name, values, qr_data, ramp_steps, cfg_path, symbol,
                    radio_cfg, output_path):
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
    lines.append('#include "rocketchip/radio_config.h"')
    lines.append('')
    lines.append('namespace rc {')
    lines.append('')
    lines.append(f'inline constexpr MissionProfile {symbol} = {{')
    lines.append(f'    .id = {profile_id},')

    # Name field — truncate to 15 chars (char[16] with null)
    safe_name = name[:15]
    lines.append(f'    .name = "{safe_name}",')
    lines.append('')

    # Emit scalar fields in struct order
    for _, cpp_field, _, _ in FIELDS:
        ftype, val = values[cpp_field]
        if ftype == 'float':
            lines.append(f'    .{cpp_field} = {val}f,')
        elif ftype == 'uint32':
            lines.append(f'    .{cpp_field} = {val},')
        elif ftype == 'bool':
            lines.append(f'    .{cpp_field} = {"true" if val else "false"},')

    # Emit phase Q/R table
    lines.append('')
    lines.extend(_emit_phase_qr(qr_data, ramp_steps))

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
    lines.append(f'static_assert({symbol}.phase_qr.ramp_steps >= 1,')
    lines.append(f'              "QR_RAMP_STEPS must be >= 1");')
    # Emit RadioConfig (IVP-96)
    sf, bw, cr = _derive_rf_params(radio_cfg)
    protocol_enum = 'EncoderType::kCcsds' if radio_cfg['RADIO_PROTOCOL'] == 0 else 'EncoderType::kMavlink'
    radio_symbol = symbol.replace('Profile', 'RadioConfig').replace('profile', 'radio_config')
    if radio_symbol == symbol:
        radio_symbol = symbol + 'Radio'

    lines.append('')
    lines.append(f'inline constexpr RadioConfig {radio_symbol} = {{')
    lines.append(f'    .protocol         = {protocol_enum},')
    lines.append(f'    .nav_rate_hz      = {radio_cfg["RADIO_RATE_HZ"]},')
    lines.append(f'    .power_dbm        = {radio_cfg["RADIO_POWER_DBM"]},')
    lines.append(f'    .spreading_factor = {sf},')
    lines.append(f'    .bandwidth_khz    = {bw},')
    lines.append(f'    .coding_rate      = {cr},')
    lines.append('};')

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

    values, qr_data, ramp_steps, radio_cfg = validate_and_convert(params)

    generate_header(name, values, qr_data, ramp_steps, args.config,
                    args.symbol, radio_cfg, args.output)

    print(f'Generated {args.output}')
    print(f'  Profile: {name}')
    print(f'  Symbol:  {args.symbol}')
    print(f'  Fields:  {len(values)}')
    print(f'  Source:  {args.config}')


if __name__ == '__main__':
    main()
