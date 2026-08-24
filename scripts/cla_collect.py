#!/usr/bin/env python3
"""
CLA snapshot: measured clock/load from a running vehicle.

Replaces the Stage-7 superloop soak (periodic main-menu 's'/'e' + optional
GDB). After the AO/QV migration those keys live under the debug menu, 'e'
is a 1 Hz stream we do not need ('s' already prints predict/full-tick and
NIS), and Windows GDB extended-remote is not the stack path (diag 'd' MSP).

Method (idle window, no CLI spam during the wait):
  1. kMenu -> debug ('q')
  2. 's' at T=0  (sensor counts + ESKF bench already compiled in)
  3. wait --duration seconds with no keys
  4. 's' at T=end (rate = delta counts / dt)
  5. 'd' once    (T=0 identity, AO queues, MSP depth, rc_log high-water)
  6. 'z' back to main

Duration default 60 s: same window the 2026-03-08 CLA used to validate that
predict avg was within 3% of the 270 s run (COMPUTATIONAL_LOAD_ANALYSIS.md
§3.2). Pass --duration 270 to match that longer soak.

ESKF epoch: idle-bridge runs every kEskfImuDivider=5 IMU samples → 200 Hz
at 1 kHz IMU (src/fusion/eskf_runner.cpp). Duty% = avg_us / 5000 * 100.

Usage:
  python scripts/cla_collect.py
  python scripts/cla_collect.py --duration 60 --output docs/audits/cla_rbm/cla_2026-08-24.md
  python scripts/cla_collect.py --port COM5 --duration 270
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)
from _rc_test_common import (  # noqa: E402
    Banner,
    enter_cli_menu,
    find_target_port,
    open_classified_port,
    rc_test,
    TARGET_VEHICLE_FLIGHT,
)

# 200 Hz ESKF epoch at 1 kHz IMU / divider 5 (eskf_runner.cpp).
ESKF_EPOCH_US = 5000
# Core 1 loop target (CLA 2026-03-08 §4; sensor_core1 1 ms cycle).
CORE1_PERIOD_US = 1000

# Host-side USB wait: 's' dump is ~2 KB via rc_log idle drain; 'd' is ~1.3 KB
# (diag_stats.cpp). Same order as other scripts' 1.5–2.5 s waits, padded.
READ_TIMEOUT_S = 4.0
IDLE_GAP_S = 0.35


def _read_quiet(ser, timeout_s: float) -> str:
    """Accumulate until `IDLE_GAP_S` of silence or timeout."""
    deadline = time.time() + timeout_s
    chunks: list[bytes] = []
    last_data = time.time()
    while time.time() < deadline:
        n = ser.in_waiting
        if n:
            chunks.append(ser.read(n))
            last_data = time.time()
        elif chunks and (time.time() - last_data) >= IDLE_GAP_S:
            break
        else:
            time.sleep(0.05)
    return b''.join(chunks).decode('utf-8', errors='replace')


def _send_key(ser, key: bytes, timeout_s: float = READ_TIMEOUT_S) -> str:
    try:
        ser.reset_input_buffer()
    except (OSError, Exception):
        pass
    ser.write(key)
    ser.flush()
    return _read_quiet(ser, timeout_s)


def _enter_debug(ser) -> str:
    text = _send_key(ser, b'q', timeout_s=2.0)
    if '--- Debug ---' in text or 's-Sensors' in text:
        return text
    raise RuntimeError(
        'did not reach debug menu after q '
        f'(got {len(text)} chars, snippet={text[-200:]!r})')


def parse_sensor_status(text: str) -> dict:
    """Parse debug-menu 's' / cli_print_sensor_status()."""
    data: dict = {}

    m = re.search(
        r'Reads:\s+I=(\d+)\s+M=(\d+)\s+B=(\d+)\s+G=(\d+)\s+'
        r'Errors:\s+I=(\d+)\s+B=(\d+)\s+G=(\d+)',
        text)
    if m:
        data['imu_reads'] = int(m.group(1))
        data['mag_reads'] = int(m.group(2))
        data['baro_reads'] = int(m.group(3))
        data['gps_reads'] = int(m.group(4))
        data['imu_errors'] = int(m.group(5))
        data['baro_errors'] = int(m.group(6))
        data['gps_errors'] = int(m.group(7))

    m = re.search(
        r'predict:\s+(\d+)us avg,\s+(\d+)us min,\s+(\d+)us max \((\d+) calls\)',
        text)
    if m:
        data['predict_avg_us'] = int(m.group(1))
        data['predict_min_us'] = int(m.group(2))
        data['predict_max_us'] = int(m.group(3))
        data['predict_calls'] = int(m.group(4))

    m = re.search(
        r'full-tick:\s+(\d+)us avg,\s+(\d+)us min,\s+(\d+)us max \((\d+) calls\)',
        text)
    if m:
        data['full_avg_us'] = int(m.group(1))
        data['full_min_us'] = int(m.group(2))
        data['full_max_us'] = int(m.group(3))
        data['full_calls'] = int(m.group(4))

    m = re.search(
        r'gate:\s+bA=(\d+)/(\d+)\s+mA=(\d+)/(\d+)\s+mR=(\d+)\s+'
        r'gA=(\d+)/(\d+)\s+zA=(\d+)/(\d+)',
        text)
    if m:
        data['baro_accepts'] = int(m.group(1))
        data['baro_total'] = int(m.group(2))
        data['mag_accepts'] = int(m.group(3))
        data['mag_total'] = int(m.group(4))
        data['mag_resets'] = int(m.group(5))
        data['gps_accepts'] = int(m.group(6))
        data['gps_total'] = int(m.group(7))
        data['zupt_accepts'] = int(m.group(8))
        data['zupt_total'] = int(m.group(9))

    m = re.search(r'bNIS=([0-9.]+)\s+mNIS=([0-9.]+)', text)
    if m:
        data['bNIS'] = float(m.group(1))
        data['mNIS'] = float(m.group(2))
    m = re.search(r'qnorm=([0-9.]+)', text)
    if m:
        data['qnorm'] = float(m.group(1))
    m = re.search(r'Mdiv=([0-9.]+)\s+deg', text)
    if m:
        data['mahony_div_deg'] = float(m.group(1))
    m = re.search(r'conf=([YN])', text)
    if m:
        data['confident'] = m.group(1) == 'Y'

    return data


def parse_diag(text: str) -> dict:
    """Parse debug-menu 'd' / diag_stats_dump()."""
    data: dict = {}
    m = re.search(r'fw_version=(\S+)', text)
    if m:
        data['fw_version'] = m.group(1)
    m = re.search(r'build_config=(\S+)', text)
    if m:
        data['build_config'] = m.group(1)
    m = re.search(r'job_role=(\S+)', text)
    if m:
        data['job_role'] = m.group(1)
    m = re.search(r'board=(.+)$', text, re.M)
    if m:
        data['board'] = m.group(1).strip()
    m = re.search(r'git=(\S+)\s+build_tag=(\S+)', text)
    if m:
        data['git'] = m.group(1)
        data['build_tag'] = m.group(2)
    m = re.search(r'RegVersion=(0x[0-9a-fA-F]+)', text)
    if m:
        data['reg_version'] = m.group(1)
    m = re.search(r'\[SPI\] error_count=(\d+)', text)
    if m:
        data['spi_errors'] = int(m.group(1))
    m = re.search(
        r'\[MSP\] initial=(0x[0-9a-fA-F]+) min=(0x[0-9a-fA-F]+) depth=(\d+) bytes',
        text)
    if m:
        data['msp_initial'] = m.group(1)
        data['msp_min'] = m.group(2)
        data['msp_depth_bytes'] = int(m.group(3))

    queues = re.findall(
        r'^\s+(\S+)\s+depth=(\d+)\s+use=(\d+)\s+high=(\d+)',
        text, re.M)
    if queues:
        data['ao_queues'] = [
            {'name': n, 'depth': int(d), 'use': int(u), 'high': int(h)}
            for n, d, u, h in queues
        ]

    m = re.search(r'go_nogo=(\S+)', text)
    if m:
        data['go_nogo'] = m.group(1)
    m = re.search(r'core1 loops=(\d+)', text)
    if m:
        data['core1_loops'] = int(m.group(1))
    m = re.search(r'\[RcLog\] dropped=(\d+) bytes\s+high_water=(\d+) bytes', text)
    if m:
        data['rclog_dropped'] = int(m.group(1))
        data['rclog_high_water'] = int(m.group(2))
    m = re.search(r'\[Uptime\] (\d+) ms', text)
    if m:
        data['uptime_ms'] = int(m.group(1))
    m = re.search(
        r'tx=(\d+) rx=(\d+) rx_crc_err=(\d+) tx_consec_fail=(\d+)',
        text)
    if m:
        data['radio_tx'] = int(m.group(1))
        data['radio_rx'] = int(m.group(2))
        data['radio_crc'] = int(m.group(3))
        data['radio_tx_fail'] = int(m.group(4))
    return data


def _duty_pct(avg_us: int, period_us: int) -> float:
    return round(100.0 * avg_us / period_us, 2)


def _rate(v0: int, v1: int, dt: float) -> float | None:
    if dt <= 0:
        return None
    return round((v1 - v0) / dt, 1)


def format_markdown(port: str, duration_s: int, banner: Banner,
                    t0: dict, t1: dict, diag: dict,
                    raw_s0: str, raw_s1: str, raw_d: str,
                    dt: float) -> str:
    now = datetime.now().strftime('%Y-%m-%d %H:%M')
    lines = [
        f'# CLA snapshot — {now}',
        '',
        'Two debug-menu `s` samples with an idle wait, then one `d`. '
        'No `e` live stream. No GDB. Method: `scripts/cla_collect.py` '
        '(2026-08 rewrite).',
        '',
        f'**Duration:** {duration_s}s idle ({dt:.1f}s between `s` samples)',
        f'**Device:** {port}',
        f'**Banner:** `{banner.short_summary()}`',
        '',
        '## Identity (`d` T=0 block)',
        '',
    ]
    if diag:
        lines += [
            '| Field | Value |',
            '|-------|-------|',
        ]
        for k in ('fw_version', 'build_config', 'job_role', 'board',
                  'git', 'build_tag', 'reg_version', 'go_nogo', 'uptime_ms'):
            if k in diag:
                lines.append(f'| {k} | {diag[k]} |')
        lines.append('')
        if diag.get('reg_version') and diag['reg_version'].lower() != '0x12':
            lines.append(
                f'**WARN:** RegVersion {diag["reg_version"]} (expect 0x12).')
            lines.append('')

    s1 = t1
    if 'predict_avg_us' in s1 or 'full_avg_us' in s1:
        lines += [
            '## ESKF bench (idle-bridge, compiled-in counters)',
            '',
            'Epoch 200 Hz (`kEskfImuDivider=5` at 1 kHz IMU). '
            f'Duty% = avg_us / {ESKF_EPOCH_US} × 100.',
            '',
            '| Counter | avg µs | min | max | calls | duty % |',
            '|---------|--------|-----|-----|-------|--------|',
        ]
        if 'predict_avg_us' in s1:
            lines.append(
                f"| predict | {s1['predict_avg_us']} | {s1['predict_min_us']} | "
                f"{s1['predict_max_us']} | {s1['predict_calls']} | "
                f"{_duty_pct(s1['predict_avg_us'], ESKF_EPOCH_US)} |")
        if 'full_avg_us' in s1:
            lines.append(
                f"| full-tick | {s1['full_avg_us']} | {s1['full_min_us']} | "
                f"{s1['full_max_us']} | {s1['full_calls']} | "
                f"{_duty_pct(s1['full_avg_us'], ESKF_EPOCH_US)} |")
        lines.append('')
        lines.append(
            'predict = codegen FPFT (+ UD housekeep when pending). '
            'full-tick = one idle-bridge `eskf_runner_tick()` wall time '
            '(replaces the Stage-7 `eskf_tick()` row). '
            'min/avg/max/calls are **since-boot** (counters never reset); '
            'only the sensor-rate table is the idle-window delta.')
        lines.append('')

    lines += [
        '## Sensor rates (delta over idle window)',
        '',
        '| Sensor | T=0 | T=end | Hz | errors T=end |',
        '|--------|-----|-------|----|--------------|',
    ]
    for name, rk, ek in (
            ('IMU', 'imu_reads', 'imu_errors'),
            ('mag', 'mag_reads', None),
            ('baro', 'baro_reads', 'baro_errors'),
            ('GPS', 'gps_reads', 'gps_errors'),
    ):
        v0 = t0.get(rk)
        v1 = t1.get(rk)
        hz = _rate(v0, v1, dt) if v0 is not None and v1 is not None else None
        err = t1.get(ek, '—') if ek else '—'
        lines.append(
            f'| {name} | {v0 if v0 is not None else "?"} | '
            f'{v1 if v1 is not None else "?"} | '
            f'{hz if hz is not None else "?"} | {err} |')
    lines.append('')
    imu_hz = _rate(t0.get('imu_reads', 0), t1.get('imu_reads', 0), dt) if (
        'imu_reads' in t0 and 'imu_reads' in t1) else None
    if imu_hz:
        lines.append(
            f'Core 1 effective IMU rate **{imu_hz} Hz** vs 1000 Hz target '
            f'({CORE1_PERIOD_US} µs cycle).')
        lines.append('')

    if any(k in s1 for k in ('bNIS', 'mNIS', 'qnorm', 'mahony_div_deg', 'confident')):
        lines += [
            '## Filter health (final `s`)',
            '',
            '| Metric | Value |',
            '|--------|-------|',
        ]
        for k in ('bNIS', 'mNIS', 'qnorm', 'mahony_div_deg', 'confident'):
            if k in s1:
                lines.append(f'| {k} | {s1[k]} |')
        lines.append('')

    if 'baro_accepts' in s1:
        lines += [
            '## Gate counts (final `s`)',
            '',
            '| Update | accept / total |',
            '|--------|----------------|',
            f"| baro | {s1['baro_accepts']}/{s1['baro_total']} |",
            f"| mag | {s1['mag_accepts']}/{s1['mag_total']} "
            f"(resets {s1['mag_resets']}) |",
            f"| GPS | {s1['gps_accepts']}/{s1['gps_total']} |",
            f"| ZUPT | {s1['zupt_accepts']}/{s1['zupt_total']} |",
            '',
        ]

    if 'msp_depth_bytes' in diag:
        lines += [
            '## Core 0 stack (MSP watermark from QV idle, not GDB)',
            '',
            f"initial {diag['msp_initial']}, min {diag['msp_min']}, "
            f"depth **{diag['msp_depth_bytes']} B** "
            '(vs 4096 B `PICO_STACK_SIZE`). `diag_stats_msp_tick()` samples '
            'MSP from QV idle, so this is idle-pointer not handler WCET. '
            'Core 1 HWM is not in this dump.',
            '',
        ]
    if 'ao_queues' in diag:
        lines += [
            '## AO queue high-water (`d`)',
            '',
            '| AO | depth | use now | high |',
            '|----|-------|---------|------|',
        ]
        for q in diag['ao_queues']:
            lines.append(
                f"| {q['name']} | {q['depth']} | {q['use']} | {q['high']} |")
        lines.append('')
        lines.append(
            '`AO_RfManager` is not printed by `diag_stats_dump()` '
            '(gap vs 9-AO inventory).')
        lines.append('')
    if 'rclog_high_water' in diag:
        lines += [
            '## rc_log / radio (`d`)',
            '',
            f"rc_log dropped={diag.get('rclog_dropped')} B, "
            f"high_water={diag.get('rclog_high_water')} B.",
            '',
        ]
        if 'radio_tx' in diag:
            lines.append(
                f"radio tx={diag['radio_tx']} rx={diag['radio_rx']} "
                f"crc_err={diag['radio_crc']} tx_consec_fail="
                f"{diag['radio_tx_fail']}.")
            lines.append('')
        if 'spi_errors' in diag:
            lines.append(f"SPI error_count={diag['spi_errors']}.")
            lines.append('')
        if 'core1_loops' in diag and 'uptime_ms' in diag and diag['uptime_ms']:
            hz = round(diag['core1_loops'] / (diag['uptime_ms'] / 1000.0), 1)
            lines.append(
                f"core1_loops={diag['core1_loops']} over uptime "
                f"{diag['uptime_ms']} ms → **{hz} Hz** (boot-to-now, "
                'not soak-window).')
            lines.append('')

    lines += [
        '## Not measured this sitting',
        '',
        '- Per-AO handler WCET / QV coincidence (would need new probes).',
        '- ESKF period jitter (σ of epoch spacing).',
        '- Core 1 stack HWM.',
        '- Station `station_idle_tick` (vehicle snapshot only).',
        '',
        '## Raw dumps',
        '',
        '### `s` at T=0',
        '',
        '```',
        raw_s0.strip() or '(empty)',
        '```',
        '',
        '### `s` at T=end',
        '',
        '```',
        raw_s1.strip() or '(empty)',
        '```',
        '',
        '### `d` at T=end',
        '',
        '```',
        raw_d.strip() or '(empty)',
        '```',
        '',
    ]
    return '\n'.join(lines)


@rc_test(target=TARGET_VEHICLE_FLIGHT)
def main() -> int:
    parser = argparse.ArgumentParser(
        description='Vehicle CLA snapshot (debug s, idle, s, d)')
    parser.add_argument('--port', default=None,
                        help='Serial port (auto-detect vehicle CDC if omitted)')
    parser.add_argument(
        '--duration', type=int, default=60,
        help='Idle seconds between s samples (default 60; 270 matches 2026-03 primary)')
    parser.add_argument(
        '--output', type=str, default=None,
        help='Markdown path (default: docs/audits/cla_rbm/cla_YYYY-MM-DD.md)')
    args = parser.parse_args()
    if args.duration < 5:
        print('ERROR: --duration must be >= 5 s (need a real idle window)')
        return 2

    port_name, meta = find_target_port(
        TARGET_VEHICLE_FLIGHT, override=args.port, verbose=False)
    if port_name is None:
        print(f'INFO: no vehicle flight port — {meta}')
        return 2
    if not isinstance(meta, Banner):
        print('ERROR: expected Banner from find_target_port')
        return 2

    out = args.output
    if not out:
        day = datetime.now().strftime('%Y-%m-%d')
        out = f'docs/audits/cla_rbm/cla_{day}.md'

    print(f'CLA snapshot on {port_name} ({meta.short_summary()})')
    print(f'  idle {args.duration}s → {out}')

    try:
        with open_classified_port(port_name, target=TARGET_VEHICLE_FLIGHT,
                                  baud=115200, timeout=0.1) as ser:
            if not enter_cli_menu(ser, settle_s=1.0, verify=True):
                print('ERROR: could not confirm kMenu')
                return 2
            _enter_debug(ser)

            print('  T=0  debug s ...')
            t_s0 = time.time()
            raw_s0 = _send_key(ser, b's')
            s0 = parse_sensor_status(raw_s0)
            if 'imu_reads' not in s0:
                print('ERROR: first s did not parse Reads: line')
                print(raw_s0[-500:])
                _send_key(ser, b'z', timeout_s=1.0)
                return 1
            print(f'    IMU reads={s0["imu_reads"]} predict='
                  f'{s0.get("predict_avg_us", "?")}us')

            print(f'  idle {args.duration}s (no keys) ...')
            time.sleep(args.duration)

            print('  T=end debug s ...')
            t_s1 = time.time()
            raw_s1 = _send_key(ser, b's')
            s1 = parse_sensor_status(raw_s1)
            if 'imu_reads' not in s1:
                print('ERROR: second s did not parse Reads: line')
                print(raw_s1[-500:])
                _send_key(ser, b'z', timeout_s=1.0)
                return 1

            print('  T=end debug d ...')
            raw_d = _send_key(ser, b'd')
            diag = parse_diag(raw_d)

            _send_key(ser, b'z', timeout_s=1.0)
    except RuntimeError as exc:
        print(f'ERROR: {exc}')
        return 2

    dt = t_s1 - t_s0
    md = format_markdown(port_name, args.duration, meta, s0, s1, diag,
                         raw_s0, raw_s1, raw_d, dt)
    path = Path(out)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(md, encoding='utf-8')
    print(f'wrote {path} ({dt:.1f}s between samples)')
    if 'full_avg_us' in s1:
        print(f'  full-tick {s1["full_avg_us"]}us avg, '
              f'duty {_duty_pct(s1["full_avg_us"], ESKF_EPOCH_US)}%')
    imu_hz = _rate(s0['imu_reads'], s1['imu_reads'], dt)
    if imu_hz is not None:
        print(f'  IMU {imu_hz} Hz over window')
    return 0


if __name__ == '__main__':
    sys.exit(main())
