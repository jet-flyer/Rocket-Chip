#!/usr/bin/env python
"""IVP-132 soak test.

Keeps device running for N seconds and captures diag_stats at intervals.
Tracks: watchdog resets, queue high-water growth, error counts, MSP depth,
health latch transitions.

This is a passive soak — the device runs whatever AOs are scheduled
(ESKF, health, radio TX, LED engine, etc.) without any injected commands.
That's the realistic flight scenario: idle on the pad waiting for launch.

Usage:
    python scripts/soak_test.py --duration 300    # 5-min diagnostic
    python scripts/soak_test.py --duration 1800   # 30-min integration
"""
import argparse
import os
import sys
import time

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)
from _rc_test_common import (  # noqa: E402
    Banner,
    find_target_port,
    open_classified_port,
    rc_test,
    TARGET_VEHICLE_BENCH,
)

def read_diag_stats(port, timeout=5.0):
    """Send q-d keystrokes, capture diag_stats output."""
    port.reset_input_buffer()
    port.write(b'q')
    time.sleep(0.5)
    port.write(b'd')
    time.sleep(1.0)
    end = time.time() + timeout
    data = b''
    while time.time() < end:
        chunk = port.read(4096)
        if not chunk:
            if data and data.count(b'====') >= 2:
                break
            continue
        data += chunk
    # Back to main menu
    port.write(b'z')
    time.sleep(0.3)
    port.reset_input_buffer()
    return data.decode('utf-8', errors='replace')

def parse_stats(text):
    """Extract key metrics from diag_stats output."""
    stats = {}
    for line in text.split('\n'):
        line = line.strip()
        if 'MSP]' in line and 'depth=' in line:
            stats['msp_depth_bytes'] = int(line.split('depth=')[1].split()[0])
        elif 'high=' in line and line.startswith('AO_'):
            parts = line.split()
            name = parts[0]
            high = int(parts[3].split('=')[1])
            stats[f'queue_{name}_high'] = high
        elif 'tx=' in line and 'rx=' in line:
            toks = line.split()
            for t in toks:
                if '=' in t:
                    k, v = t.split('=')
                    stats[f'radio_{k}'] = int(v.rstrip(','))
        elif 'primary=' in line:
            toks = line.split()
            for t in toks:
                if '=' in t:
                    k, v = t.split('=')
                    stats[f'health_{k}'] = v
        elif 'IMU reads=' in line:
            for part in line.split():
                if '=' in part:
                    k, v = part.split('=')
                    if v.isdigit():
                        stats[f'sensor_{k}'] = int(v)
        elif 'Uptime' in line and 'ms' in line:
            stats['uptime_ms'] = int(line.split()[1])
    return stats

@rc_test(target=TARGET_VEHICLE_BENCH, watchdog_s=86400.0)
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default=None,
                        help='Vehicle USB CDC (default: VID 0x2E8A auto-detect)')
    parser.add_argument('--duration', type=int, default=300)
    parser.add_argument('--interval', type=int, default=60,
                        help='Seconds between stats snapshots')
    args = parser.parse_args()

    print(f'=== IVP-132 Soak Test ({args.duration}s, snapshot every {args.interval}s) ===', flush=True)

    port_name, meta = find_target_port(
        TARGET_VEHICLE_BENCH, override=args.port, verbose=True)
    if port_name is None:
        print(f'INFO: skip — {meta}', flush=True)
        return 2

    assert isinstance(meta, Banner)
    print(f'target banner: {meta.short_summary()}', flush=True)

    try:
        with open_classified_port(port_name, target=TARGET_VEHICLE_BENCH) as port:
            port.reset_input_buffer()

            snapshots = []
            start = time.time()
            next_snap = 0

            while time.time() - start < args.duration:
                elapsed = int(time.time() - start)
                if elapsed >= next_snap:
                    print(f'\n[T={elapsed}s] Capturing stats...', flush=True)
                    raw = read_diag_stats(port)
                    s = parse_stats(raw)
                    s['_raw'] = raw
                    s['_wall_time'] = elapsed
                    snapshots.append(s)
                    print(f'  uptime={s.get("uptime_ms",0)/1000:.0f}s  '
                          f'IMU_reads={s.get("sensor_reads","?")}  '
                          f'radio_tx={s.get("radio_tx","?")}  '
                          f'health={s.get("health_primary","?")}  '
                          f'FD_high={s.get("queue_AO_FlightDirector_high","?")}',
                          flush=True)
                    next_snap = elapsed + args.interval
                else:
                    time.sleep(1)

            print(f'\n[T={int(time.time()-start)}s] Final capture...', flush=True)
            raw = read_diag_stats(port)
            final = parse_stats(raw)
            final['_raw'] = raw
            snapshots.append(final)
    except RuntimeError as e:
        print(f'ERROR: {e}', flush=True)
        return 2

    # Analysis (same as prior: single port closed by context mgr)
    print('\n=== Soak Analysis ===', flush=True)
    if len(snapshots) < 2:
        print('INSUFFICIENT SAMPLES', flush=True)
        return 1

    first = snapshots[0]
    last = snapshots[-1]
    dt_s = (last.get('uptime_ms', 0) - first.get('uptime_ms', 0)) / 1000.0
    if dt_s <= 0:
        print(f'WARN: uptime did not advance — possible watchdog reset', flush=True)
    else:
        print(f'Time window: {dt_s:.1f}s', flush=True)

    # Check for regressions
    ok = True
    # Queue high-water should be stable (not growing)
    for key in [k for k in first if k.startswith('queue_')]:
        d_first = first.get(key, 0)
        d_last = last.get(key, 0)
        if d_last > d_first + 2:
            print(f'WARN: {key} grew: {d_first} -> {d_last}', flush=True)
            ok = False
    # IMU reads should be ~999/s
    first_reads = first.get('sensor_reads', 0)
    last_reads = last.get('sensor_reads', 0)
    if dt_s > 0:
        rate = (last_reads - first_reads) / dt_s
        print(f'IMU rate: {rate:.0f} Hz (expect ~999)', flush=True)
        if rate < 900:
            print(f'FAIL: IMU rate degraded', flush=True)
            ok = False

    # Radio tx
    first_tx = first.get('radio_tx', 0)
    last_tx = last.get('radio_tx', 0)
    if dt_s > 0:
        tx_rate = (last_tx - first_tx) / dt_s
        print(f'Radio TX rate: {tx_rate:.1f} Hz', flush=True)

    # Health
    print(f'Health transitions: first primary=0x{first.get("health_primary","?")} last=0x{last.get("health_primary","?")}', flush=True)

    print(f'\nRESULT: {"PASS" if ok else "FAIL"}', flush=True)
    return 0 if ok else 1

if __name__ == '__main__':
    main()
