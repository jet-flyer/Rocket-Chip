#!/usr/bin/env python3
"""rc_log drain limitation measurement — R-29 bench reproducer.

Question: does the rc_log empty-ring fast-path (LL Entry 39) sufficiently
protect Core 1's I2C transactions under SUSTAINED rc_log output, or does
the contention return when the ring is non-empty for sustained periods?

Protocol (council-approved 2026-05-22, NASA/JPL + Professor + Cubesat
unanimous):

  Phase A — quiet baseline:
    1. Enter `q` (debug submenu)
    2. Send `d` once -> capture initial diag_stats
    3. Sleep 60s (no host->target traffic)
    4. Send `d` once -> capture final diag_stats
    5. Compute imu_reads/sec, imu_errs over the 60s window.

  Phase B — sustained-output stress:
    1. Enter `q` (debug submenu)
    2. Loop for ~60s: send `d`, wait ~50ms, read drain
       (each `d` -> ~39 rc_log calls = ~1-2KB ring traffic, keeps the
        ring non-empty for sustained periods)
    3. Parse FIRST dump's counters as initial, LAST dump's as final.
    4. Compute imu_reads/sec, imu_errs over the actual measured window.

Expected if no degradation:
  Phase A: imu_reads/sec ~900-1000, imu_errs = 0
  Phase B: same as A (sustained output doesn't break Core 1's I2C)

Expected if degradation reproducible:
  Phase A: imu_reads/sec ~900-1000, imu_errs = 0
  Phase B: imu_reads/sec drops AND/OR imu_errs starts climbing

Output: prints a verdict table.
"""

from __future__ import annotations

import re
import sys
import time

# pylint: disable=wrong-import-position
sys.path.insert(0, str(__file__.rsplit('\\', 2)[0] + '\\scripts'))
from _rc_test_common import (  # noqa: E402
    rc_test, find_target_port, open_classified_port,
    TARGET_VEHICLE_ANY,
)

_RE_IMU_LINE = re.compile(
    r'IMU reads=(\d+) errs=(\d+)\s+baro reads=(\d+) errs=(\d+)\s+gps reads=(\d+) errs=(\d+)'
)
_RE_CORE1_LOOPS = re.compile(r'core1 loops=(\d+)')


def _parse_diag(text: str) -> dict | None:
    """Extract counters from a diag_stats_dump() output blob."""
    m = _RE_IMU_LINE.search(text)
    if m is None:
        return None
    c = _RE_CORE1_LOOPS.search(text)
    return {
        'imu_reads': int(m.group(1)),
        'imu_errs': int(m.group(2)),
        'baro_reads': int(m.group(3)),
        'baro_errs': int(m.group(4)),
        'gps_reads': int(m.group(5)),
        'gps_errs': int(m.group(6)),
        'core1_loops': int(c.group(1)) if c else 0,
    }


def _drain(ser, deadline: float) -> str:
    """Read all bytes available until `deadline` (wall-clock)."""
    out = bytearray()
    while time.time() < deadline:
        chunk = ser.read(4096)
        if chunk:
            out.extend(chunk)
        else:
            time.sleep(0.01)
    return out.decode('utf-8', errors='replace')


def _capture_dump(ser, settle_s: float = 1.5) -> dict | None:
    """Send `d` and wait for the diag_stats_dump output. Returns parsed counters."""
    ser.reset_input_buffer()
    ser.write(b'd')
    ser.flush()
    deadline = time.time() + settle_s
    text = _drain(ser, deadline)
    return _parse_diag(text)


def _enter_debug_submenu(ser, settle_s: float = 1.0) -> bool:
    """Send `q` to enter the debug submenu; return True if we got the [debug] banner."""
    ser.reset_input_buffer()
    ser.write(b'q')
    ser.flush()
    time.sleep(settle_s)
    text = ser.read(8000).decode('utf-8', errors='replace')
    return '[debug]' in text.lower() or 'debug' in text.lower()


def _phase_a_quiet(ser, duration_s: float = 60.0) -> tuple[dict, dict, float]:
    """Phase A: capture initial counters, sleep quiet, capture final."""
    print(f'  [phase A] quiet baseline: capturing initial counters...')
    initial = _capture_dump(ser)
    if initial is None:
        raise RuntimeError('phase A: could not parse initial diag dump')
    print(f'    initial: imu_reads={initial["imu_reads"]}, '
          f'imu_errs={initial["imu_errs"]}, '
          f'core1_loops={initial["core1_loops"]}')

    t_start = time.time()
    print(f'    sleeping {duration_s}s with no host->target traffic...')
    time.sleep(duration_s)
    actual_window = time.time() - t_start

    print(f'  [phase A] capturing final counters...')
    final = _capture_dump(ser)
    if final is None:
        raise RuntimeError('phase A: could not parse final diag dump')
    print(f'    final:   imu_reads={final["imu_reads"]}, '
          f'imu_errs={final["imu_errs"]}, '
          f'core1_loops={final["core1_loops"]}')

    return initial, final, actual_window


def _phase_b_stressed(ser, duration_s: float = 60.0, write_rate_hz: float = 100.0) -> tuple[dict, dict, float, int]:
    """Phase B: write `d` at write_rate_hz target without reading between writes.

    Reading drains the firmware's TX FIFO; not reading lets the firmware's
    drain path stay maximally loaded (USB CDC TX backpressure means the
    rc_log ring stays non-empty for sustained periods, which is exactly
    the failure mode LL Entry 39 known-limitation describes).
    """
    print(f'  [phase B] sustained `d` writes at target {write_rate_hz:.0f}Hz, target {duration_s}s...')
    # First dump = initial counters reference
    initial = _capture_dump(ser)
    if initial is None:
        raise RuntimeError('phase B: could not parse initial diag dump')
    print(f'    initial: imu_reads={initial["imu_reads"]}, '
          f'imu_errs={initial["imu_errs"]}, '
          f'core1_loops={initial["core1_loops"]}')

    gap_s = 1.0 / write_rate_hz
    t_start = time.time()
    deadline = t_start + duration_s
    dump_count = 0

    # Drain any backlog from the initial capture so the firmware's TX FIFO starts empty.
    ser.reset_input_buffer()

    while time.time() < deadline:
        try:
            ser.write(b'd')
            ser.flush()
            dump_count += 1
        except Exception:
            pass
        # Yield briefly. Don't read — that would drain the TX FIFO and reduce
        # the very pressure we're trying to apply.
        time.sleep(gap_s)
        # Drain just the host-side OS buffer occasionally to prevent it from
        # filling up and blocking writes. Every ~100 writes is plenty.
        if (dump_count % 100) == 0:
            ser.read(20000)  # discard

    actual_window = time.time() - t_start
    print(f'    sent {dump_count} `d` writes over {actual_window:.1f}s '
          f'(~{dump_count/actual_window:.1f}/sec)')

    # Drain accumulated output then ask cleanly for final counters.
    time.sleep(1.0)
    ser.read(200000)  # discard backlog
    time.sleep(0.5)
    ser.reset_input_buffer()
    final = _capture_dump(ser, settle_s=2.0)
    if final is None:
        raise RuntimeError('phase B: could not parse final diag dump')
    print(f'    final:   imu_reads={final["imu_reads"]}, '
          f'imu_errs={final["imu_errs"]}, '
          f'core1_loops={final["core1_loops"]}')

    return initial, final, actual_window, dump_count


def _report(label: str, initial: dict, final: dict, window: float) -> dict:
    dr_imu = final['imu_reads'] - initial['imu_reads']
    de_imu = final['imu_errs'] - initial['imu_errs']
    dr_baro = final['baro_reads'] - initial['baro_reads']
    de_baro = final['baro_errs'] - initial['baro_errs']
    dr_gps = final['gps_reads'] - initial['gps_reads']
    de_gps = final['gps_errs'] - initial['gps_errs']
    dc1 = final['core1_loops'] - initial['core1_loops']
    return {
        'label': label,
        'window_s': window,
        'imu_reads_per_sec': dr_imu / window if window > 0 else 0,
        'imu_errs_total': de_imu,
        'baro_reads_per_sec': dr_baro / window if window > 0 else 0,
        'baro_errs_total': de_baro,
        'gps_reads_per_sec': dr_gps / window if window > 0 else 0,
        'gps_errs_total': de_gps,
        'core1_loops_per_sec': dc1 / window if window > 0 else 0,
    }


def _print_table(rows: list[dict]) -> None:
    print()
    print('=' * 80)
    print('RESULTS')
    print('=' * 80)
    print(f'{"Phase":<25} {"window":>8} {"IMU r/s":>10} {"IMU errs":>10} '
          f'{"baro r/s":>10} {"baro errs":>10} {"core1/s":>10}')
    print('-' * 80)
    for r in rows:
        print(f'{r["label"]:<25} {r["window_s"]:>7.1f}s '
              f'{r["imu_reads_per_sec"]:>10.1f} '
              f'{r["imu_errs_total"]:>10d} '
              f'{r["baro_reads_per_sec"]:>10.1f} '
              f'{r["baro_errs_total"]:>10d} '
              f'{r["core1_loops_per_sec"]:>10.1f}')
    print('=' * 80)


@rc_test(target=TARGET_VEHICLE_ANY, watchdog_s=240)
def main():
    print('R-29 bench measurement: rc_log drain limitation under sustained output')
    print('=' * 80)
    port, banner = find_target_port(TARGET_VEHICLE_ANY, verbose=True)
    if port is None:
        print(f'SKIP: {banner}')
        return 2
    print(f'  vehicle on {port}: {banner.short_summary()}')

    with open_classified_port(port, target=TARGET_VEHICLE_ANY) as ser:
        # Drain any boot/connect noise.
        time.sleep(1.0)
        ser.read(20000)

        # Enter debug submenu so `d` triggers diag_stats_dump.
        print('Entering debug submenu...')
        if not _enter_debug_submenu(ser):
            print('WARNING: did not see [debug] banner after `q`; continuing anyway')

        # Phase A
        print()
        print('PHASE A — quiet baseline (60s, no host->target traffic except 2 dumps)')
        a_init, a_final, a_window = _phase_a_quiet(ser, duration_s=60.0)
        row_a = _report('A: quiet baseline', a_init, a_final, a_window)

        # Brief settle between phases.
        time.sleep(2.0)
        ser.reset_input_buffer()

        # Phase B
        print()
        print('PHASE B — sustained `d` writes at 100Hz (60s, no reads between writes)')
        b_init, b_final, b_window, b_dumps = _phase_b_stressed(ser, duration_s=60.0, write_rate_hz=100.0)
        row_b = _report('B: stressed (100 d/s)', b_init, b_final, b_window)
        row_b['dump_count'] = b_dumps

        _print_table([row_a, row_b])

        # Verdict
        baseline_imu_rps = row_a['imu_reads_per_sec']
        stressed_imu_rps = row_b['imu_reads_per_sec']
        degradation_pct = (1.0 - stressed_imu_rps / baseline_imu_rps) * 100 if baseline_imu_rps > 0 else 0
        print()
        print(f'IMU rate degradation under sustained output: {degradation_pct:+.1f}%')
        print(f'IMU errors during stress phase: {row_b["imu_errs_total"]}')
        print()
        if row_b['imu_errs_total'] > 0 or degradation_pct > 5.0:
            print('VERDICT: degradation REPRODUCED — option B (dedicated repeating-timer) justified')
            return 1  # signal "we have a real finding"
        else:
            print('VERDICT: no degradation under tested stress profile')
            print('         (~20 dumps/sec, 60s sustained, ~1-2KB/dump = ~20-40KB/s ring traffic)')
            return 0

    return 0


if __name__ == '__main__':
    main()
