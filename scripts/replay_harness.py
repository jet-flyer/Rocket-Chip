#!/usr/bin/env python
"""Replay harness: streams CSV sensor profiles into the firmware via serial.

Usage:
    python scripts/replay_harness.py tests/replay_profiles/big_daddy_f15_nominal.csv

Protocol:
    1. Opens serial port (COM7, 115200)
    2. Sends 'q' then 'r' to enter debug menu → replay mode
       (Or use GDB: call replay_inject_start())
    3. Streams CSV lines as "S,ax,ay,az,gx,gy,gz,press,lat,lon,alt\\n"
       at the rate specified by the CSV time column
    4. Sends "REPLAY_END\\n" when done
    5. Reads FD state transitions from serial output

The firmware injects each sample into the seqlock, and the ESKF/FD
pipeline processes it as if it came from real sensors.
"""
import csv
import serial
import sys
import time
import re
import argparse

def main():
    parser = argparse.ArgumentParser(description='RocketChip Replay Harness')
    parser.add_argument('csv_file', help='Path to sensor profile CSV')
    parser.add_argument('--port', default='COM7', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--use-cli', action='store_true',
                        help='Start replay via CLI keys (default: assume GDB started it)')
    parser.add_argument('--arm-first', action='store_true',
                        help='Send ARM command before replay (needed for FD state transitions)')
    args = parser.parse_args()

    # Read CSV, skip comment lines
    samples = []
    with open(args.csv_file, 'r') as f:
        for line in f:
            if line.startswith('#') or line.strip() == '':
                continue
            break  # header row
        reader = csv.DictReader(f, fieldnames=[
            'time_s', 'accel_x', 'accel_y', 'accel_z',
            'gyro_x', 'gyro_y', 'gyro_z',
            'pressure_pa', 'gps_lat_1e7', 'gps_lon_1e7', 'gps_alt_mm'
        ])
        # Re-read including the header row we just consumed
        f.seek(0)
        for line in f:
            if line.startswith('#') or line.strip() == '':
                continue
            if 'time_s' in line:
                continue  # skip header
            parts = line.strip().split(',')
            if len(parts) < 8:
                continue
            samples.append(parts)

    print(f'Loaded {len(samples)} samples from {args.csv_file}')

    port = serial.Serial(args.port, args.baud, timeout=0.1)
    time.sleep(0.5)
    port.reset_input_buffer()  # non-blocking drain

    if args.use_cli:
        print('Entering debug menu and starting replay via CLI...')
        port.write(b'q')
        time.sleep(0.3)
        port.write(b'r')
        time.sleep(0.5)

    if args.arm_first:
        print('Sending ARM command...')
        port.write(b'f')  # flight menu
        time.sleep(0.2)
        port.write(b'a')  # ARM
        time.sleep(1.0)

    # Drain any output
    port.read(10000)

    print(f'Streaming {len(samples)} samples...')
    start_time = time.time()
    prev_t = 0.0

    for i, parts in enumerate(samples):
        t = float(parts[0])
        ax = parts[1]
        ay = parts[2]
        az = parts[3] if parts[3] else '0'
        gx = parts[4]
        gy = parts[5]
        gz = parts[6]
        press = parts[7] if parts[7] else '0'
        lat = parts[8] if len(parts) > 8 and parts[8] else '0'
        lon = parts[9] if len(parts) > 9 and parts[9] else '0'
        alt = parts[10] if len(parts) > 10 and parts[10] else '0'

        # Wait for correct timing
        dt = t - prev_t
        if dt > 0 and i > 0:
            time.sleep(dt)
        prev_t = t

        line = f'S,{ax},{ay},{az},{gx},{gy},{gz},{press},{lat},{lon},{alt}\n'
        port.write(line.encode())

        if i % 500 == 0 and i > 0:
            elapsed = time.time() - start_time
            print(f'  {i}/{len(samples)} samples sent (sim t={t:.1f}s, wall={elapsed:.1f}s)')

    # End replay
    port.write(b'REPLAY_END\n')
    time.sleep(1.0)

    # Read available output (non-blocking)
    output = b''
    while True:
        chunk = port.read(4096)
        if not chunk:
            break
        output += chunk
    output = output.decode('utf-8', errors='replace')
    port.close()

    print(f'\nReplay complete. {len(samples)} samples in {time.time()-start_time:.1f}s')
    print('\n--- Firmware output ---')
    print(output)

    # Check for FD state transitions
    transitions = re.findall(r'\[FD\].*', output)
    if transitions:
        print('\n--- FD State Transitions ---')
        for t in transitions:
            print(f'  {t}')
    else:
        print('\nNo FD state transitions captured in output.')
        print('(FD transitions may require ARM state — use --arm-first)')

if __name__ == '__main__':
    main()
