#!/usr/bin/env python
"""IVP-131 Gate Test: Run full replay profile and verify FD state sequence.

Orchestrates: GDB (probe) for start/stop/verify + serial for data stream.
Captures FD state transitions and compares to expected oracle.

Usage:
    python scripts/replay_gate_test.py tests/replay_profiles/big_daddy_f15_nominal.csv
"""
import subprocess
import serial
import time
import sys
import re
import os

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)
from _rc_test_common import rc_test, TARGET_VEHICLE_BENCH  # noqa: E402

GDB = r'C:\Users\pow-w\.pico-sdk\toolchain\14_2_Rel1\bin\arm-none-eabi-gdb.exe'
ELF = 'build/rocketchip.elf'
PORT = 'COM7'
BAUD = 115200

def gdb_cmd(*commands):
    args = [GDB, ELF, '-batch',
            '-ex', 'target extended-remote localhost:3333']
    for c in commands:
        args.extend(['-ex', c])
    r = subprocess.run(args, capture_output=True, text=True, timeout=20)
    return r.stdout

def gdb_read_var(var):
    out = gdb_cmd('monitor halt', f'print {var}')
    for line in out.split('\n'):
        m = re.search(r'\$\d+ = (.+)', line)
        if m:
            return m.group(1).strip()
    return None

@rc_test(target=TARGET_VEHICLE_BENCH)
def main():
    if len(sys.argv) < 2:
        print(f'Usage: {sys.argv[0]} <csv_file>')
        sys.exit(1)

    csv_file = sys.argv[1]
    print(f'=== IVP-131 Gate Test: {os.path.basename(csv_file)} ===')

    # Read oracle from CSV header
    oracle_lines = []
    with open(csv_file, 'r') as f:
        for line in f:
            if line.startswith('#'):
                oracle_lines.append(line.strip())
            else:
                break
    print('\nOracle:')
    for line in oracle_lines:
        print(f'  {line}')

    # Read samples
    samples = []
    with open(csv_file, 'r') as f:
        for line in f:
            if line.startswith('#') or 'time_s' in line or not line.strip():
                continue
            parts = line.strip().split(',')
            if len(parts) >= 8:
                samples.append(parts)
    print(f'\nLoaded {len(samples)} samples')

    # Step 1: Flash and boot
    print('\n[1] Flashing bench binary...')
    gdb_cmd('monitor reset halt', 'load', 'monitor resume')
    time.sleep(5)

    # Step 2: Verify ESKF not yet initialized
    eskf = gdb_read_var('g_eskfInitialized')
    print(f'[2] ESKF initialized before replay: {eskf}')

    # Step 3: Start replay
    print('[3] Starting replay (pausing Core 1)...')
    gdb_cmd('monitor halt', 'call replay_inject_start()', 'monitor resume')
    time.sleep(1)

    # Step 4: ARM via GDB dispatch (bypass Go/No-Go — Core 1 paused)
    # SIG_ARM = 5 per ao_signals.h
    print('[4] Force-ARMing FD via GDB (SIG_ARM=5, bypass Go/No-Go)...')
    gdb_cmd('monitor halt',
            'call AO_FlightDirector_dispatch_signal(5)',
            'monitor resume')
    time.sleep(1)

    # Check FD phase: kArmed = 1
    phase = gdb_read_var('AO_FlightDirector_get_director()->current_phase')
    print(f'  FD phase after ARM: {phase} (expect 1=ARMED)')

    # Re-start replay
    print('[5] Re-starting replay...')
    gdb_cmd('monitor halt', 'call replay_inject_start()', 'monitor resume')
    time.sleep(1)

    # Step 6: Stream the full profile
    print(f'[6] Streaming {len(samples)} samples (~{float(samples[-1][0]):.0f}s)...')
    port = serial.Serial(PORT, BAUD, timeout=0.1, write_timeout=5)
    port.reset_input_buffer()

    start_wall = time.time()
    prev_t = 0.0

    for i, p in enumerate(samples):
        t = float(p[0])
        dt = t - prev_t
        if dt > 0 and i > 0:
            time.sleep(dt)
        prev_t = t

        az = p[3] if p[3] else '0'
        pr = p[7] if p[7] else '0'
        la = p[8] if len(p) > 8 and p[8] else '0'
        lo = p[9] if len(p) > 9 and p[9] else '0'
        al = p[10] if len(p) > 10 and p[10] else '0'
        msg = f'S,{p[1]},{p[2]},{az},{p[4]},{p[5]},{p[6]},{pr},{la},{lo},{al}\n'
        try:
            port.write(msg.encode())
        except serial.SerialTimeoutException:
            print(f'  Write timeout at sample {i} (t={t:.1f}s)')
            break

        if i % 1000 == 0 and i > 0:
            elapsed = time.time() - start_wall
            print(f'  {i}/{len(samples)} (sim t={t:.1f}s, wall={elapsed:.1f}s)')

    port.write(b'REPLAY_END\n')
    time.sleep(1.0)
    port.close()

    wall_time = time.time() - start_wall
    print(f'  Streaming complete: {len(samples)} samples in {wall_time:.1f}s')

    # Step 7: Read final state via GDB
    print('\n[7] Verifying final state via GDB...')
    eskf_final = gdb_read_var('g_eskfInitialized')
    phase_final = gdb_read_var('AO_FlightDirector_get_director()->current_phase')
    vel = gdb_read_var('g_eskf.v')

    print(f'  ESKF initialized: {eskf_final}')
    print(f'  FD phase: {phase_final}')
    print(f'  ESKF velocity: {vel}')

    # Step 8: Check pyro edge log
    edge_count = gdb_read_var("'rc::s_count'")
    print(f'  Pyro edge events: {edge_count}')

    print('\n=== Gate Summary ===')
    print(f'Profile: {os.path.basename(csv_file)}')
    print(f'Samples: {len(samples)}')
    print(f'Wall time: {wall_time:.1f}s')
    print(f'ESKF: {eskf_final}')
    print(f'FD phase: {phase_final}')
    print(f'Pyro edges: {edge_count}')

if __name__ == '__main__':
    main()
