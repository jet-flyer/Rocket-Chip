#!/usr/bin/env python3
"""
RocketChip Computational Load Analysis (CLA) Data Collection Script

Collects timing and performance data from a running RocketChip device via:
  1. Serial CLI commands ('s' for sensor status, 'e' for ESKF status)
  2. GDB/OpenOCD memory reads for stack high-water marks and internal counters

Requirements:
  - pyserial: pip install pyserial
  - Device connected via USB (COM7)
  - For stack analysis: OpenOCD running with debug probe connected

Usage:
  python scripts/cla_collect.py                       # CLI-only collection
  python scripts/cla_collect.py --duration 60          # 60s soak with periodic sampling
  python scripts/cla_collect.py --gdb                  # Include GDB stack/memory analysis
  python scripts/cla_collect.py --output cla_data.md   # Save to file
  python scripts/cla_collect.py --compare prev.md      # Compare against baseline

The script is non-destructive and requires no firmware changes.
"""

import argparse
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    import serial
except ImportError:
    print("ERROR: pyserial required. Install with: pip install pyserial")
    sys.exit(1)

# --- Configuration ---
SERIAL_PORT = 'COM7'
BAUD = 115200
GDB_PATH = '/c/Users/pow-w/.pico-sdk/toolchain/14_2_Rel1/bin/arm-none-eabi-gdb.exe'
ELF_PATH = 'build/rocketchip.elf'
OPENOCD_PORT = 3333

# --- Serial helpers ---

def serial_connect(port=SERIAL_PORT, baud=BAUD):
    """Connect to device serial port."""
    try:
        s = serial.Serial(port, baud, timeout=2)
        time.sleep(0.5)
        s.read(4096)  # Drain buffer
        return s
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {port}: {e}")
        sys.exit(1)


def serial_cmd(port, cmd, wait=1.0, bufsize=8192):
    """Send a CLI key and collect response."""
    port.write(cmd.encode() if isinstance(cmd, str) else cmd)
    time.sleep(wait)
    return port.read(bufsize).decode('utf-8', errors='replace')


def serial_stop_streaming(port):
    """Stop any active streaming mode (ESKF live, etc.) by sending a neutral key."""
    port.write(b'\n')  # Any key stops ESKF live
    time.sleep(0.3)
    port.read(4096)  # Drain stop message


def serial_collect_sensor_status(port):
    """Collect sensor status ('s') output cleanly."""
    serial_stop_streaming(port)
    port.read(4096)  # Drain any remaining output
    return serial_cmd(port, 's', wait=1.5, bufsize=8192)


def serial_collect_eskf_status(port):
    """Collect ESKF status. 'e' starts 1Hz streaming; collect ~2s then stop."""
    serial_stop_streaming(port)
    port.read(4096)  # Drain
    port.write(b'e')
    time.sleep(2.5)  # Collect 2 samples at 1Hz
    data = port.read(8192).decode('utf-8', errors='replace')
    # Stop streaming
    port.write(b'\n')
    time.sleep(0.3)
    port.read(4096)  # Drain stop message
    return data


# --- Parsers ---

def parse_sensor_status(text):
    """Parse 's' command output for timing/count data.

    Expected format:
      Reads: I=12345 M=678 B=901 G=234  Errors: I=0 B=0 G=0
      Log: 500 frames stored, 1000 capacity
    """
    data = {}

    # Sensor read counts: "Reads: I=NNN M=NNN B=NNN G=NNN"
    m = re.search(r'Reads:\s+I=(\d+)\s+M=(\d+)\s+B=(\d+)\s+G=(\d+)', text)
    if m:
        data['imu_reads'] = int(m.group(1))
        data['mag_reads'] = int(m.group(2))
        data['baro_reads'] = int(m.group(3))
        data['gps_reads'] = int(m.group(4))

    # Sensor error counts: "Errors: I=NNN B=NNN G=NNN"
    m = re.search(r'Errors:\s+I=(\d+)\s+B=(\d+)\s+G=(\d+)', text)
    if m:
        data['imu_errors'] = int(m.group(1))
        data['baro_errors'] = int(m.group(2))
        data['gps_errors'] = int(m.group(3))

    # Log frames: "Log: NNN frames stored, NNN capacity"
    m = re.search(r'Log:\s+(\d+)\s+frames\s+stored,\s+(\d+)\s+capacity', text)
    if m:
        data['log_frames'] = int(m.group(1))
        data['log_capacity'] = int(m.group(2))

    # ESKF predict timing (also appears in 's' output)
    m = re.search(r'predict:\s+(\d+)us\s+avg,\s+(\d+)us\s+min,\s+(\d+)us\s+max\s+\((\d+)\s+calls\)',
                  text)
    if m:
        data['predict_avg_us'] = int(m.group(1))
        data['predict_min_us'] = int(m.group(2))
        data['predict_max_us'] = int(m.group(3))
        data['predict_calls'] = int(m.group(4))

    # ESKF gate counters: "gate: bA=8106/10615 mA=58083/58083 mR=0 gA=0/0 zA=107472/115202"
    m = re.search(r'gate:\s+bA=(\d+)/(\d+)\s+mA=(\d+)/(\d+)\s+mR=(\d+)\s+gA=(\d+)/(\d+)\s+zA=(\d+)/(\d+)',
                  text)
    if m:
        data['baro_accepts'] = int(m.group(1))
        data['baro_total'] = int(m.group(2))
        data['mag_accepts'] = int(m.group(3))
        data['mag_total'] = int(m.group(4))
        data['mag_rejects'] = int(m.group(5))
        data['gps_accepts'] = int(m.group(6))
        data['gps_total'] = int(m.group(7))
        data['zupt_accepts'] = int(m.group(8))
        data['zupt_total'] = int(m.group(9))

    return data


def parse_eskf_status(text):
    """Parse 'e' command output for ESKF timing and health."""
    data = {}

    # FPFT predict timing
    m = re.search(r'predict:\s+(\d+)us\s+avg,\s+(\d+)us\s+min,\s+(\d+)us\s+max\s+\((\d+)\s+calls\)',
                  text)
    if m:
        data['predict_avg_us'] = int(m.group(1))
        data['predict_min_us'] = int(m.group(2))
        data['predict_max_us'] = int(m.group(3))
        data['predict_calls'] = int(m.group(4))

    # Health indicators
    m = re.search(r'bNIS=([0-9.]+)', text)
    if m:
        data['bNIS'] = float(m.group(1))

    m = re.search(r'mNIS=([0-9.]+)', text)
    if m:
        data['mNIS'] = float(m.group(1))

    m = re.search(r'Mdiv=([0-9.]+)', text)
    if m:
        data['mahony_div_deg'] = float(m.group(1))

    # qnorm
    m = re.search(r'qnorm=([0-9.]+)', text)
    if m:
        data['qnorm'] = float(m.group(1))

    return data


# --- GDB helpers ---

def gdb_read_variable(var_name):
    """Read a static variable via GDB batch mode. Returns string value or None."""
    cmd = [
        GDB_PATH, ELF_PATH, '-batch',
        '-ex', f'target extended-remote localhost:{OPENOCD_PORT}',
        '-ex', 'monitor halt',
        '-ex', f'print {var_name}',
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        # Parse GDB output: "$1 = 12345"
        m = re.search(r'\$\d+\s*=\s*(.+)', result.stdout)
        if m:
            return m.group(1).strip()
    except (subprocess.TimeoutExpired, FileNotFoundError):
        pass
    return None


def gdb_read_memory(addr, count_words):
    """Read memory words via GDB. Returns list of uint32 values."""
    cmd = [
        GDB_PATH, ELF_PATH, '-batch',
        '-ex', f'target extended-remote localhost:{OPENOCD_PORT}',
        '-ex', 'monitor halt',
        '-ex', f'x/{count_words}xw {addr}',
        '-ex', 'monitor resume',
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        values = []
        for m in re.finditer(r'0x([0-9a-fA-F]+)', result.stdout):
            values.append(int(m.group(1), 16))
        return values
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return []


def gdb_stack_hwm(stack_symbol, stack_size_bytes, sentinel=0xDEADBEEF):
    """
    Scan stack for high-water mark using GDB.

    The Pico SDK paints stacks with 0 when PICO_USE_STACK_GUARDS=1.
    We scan from the bottom for zero words (unpainted = used).

    Returns (used_bytes, total_bytes) or None on failure.
    """
    # Get stack bottom address
    cmd = [
        GDB_PATH, ELF_PATH, '-batch',
        '-ex', f'target extended-remote localhost:{OPENOCD_PORT}',
        '-ex', 'monitor halt',
        '-ex', f'print &{stack_symbol}',
        '-ex', 'monitor resume',
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        m = re.search(r'0x([0-9a-fA-F]+)', result.stdout)
        if not m:
            return None
        bottom_addr = int(m.group(1), 16)
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None

    # Read first 256 words from stack bottom (scan for zeros = unpainted)
    scan_words = min(stack_size_bytes // 4, 256)
    words = gdb_read_memory(hex(bottom_addr), scan_words)
    if not words:
        return None

    # Count zero words from bottom (SDK initializes stack to 0)
    unpainted = 0
    for w in words:
        if w != 0:
            break
        unpainted += 1

    used = stack_size_bytes - (unpainted * 4)
    return (used, stack_size_bytes)


def collect_gdb_data():
    """Collect data via GDB memory reads."""
    data = {}

    print("  Reading ESKF benchmark variables via GDB...")
    for var in ['g_eskfBenchMin', 'g_eskfBenchMax', 'g_eskfBenchSum', 'g_eskfBenchCount']:
        val = gdb_read_variable(var)
        if val:
            data[var] = val

    print("  Reading stack high-water marks...")
    c0 = gdb_stack_hwm('__StackBottom', 0x1000)
    if c0:
        data['core0_stack_used'], data['core0_stack_total'] = c0
        data['core0_stack_margin_pct'] = round(
            100 * (c0[1] - c0[0]) / c0[1], 1)

    c1 = gdb_stack_hwm('__StackOneBottom', 0x1000)
    if c1:
        data['core1_stack_used'], data['core1_stack_total'] = c1
        data['core1_stack_margin_pct'] = round(
            100 * (c1[1] - c1[0]) / c1[1], 1)

    return data


# --- Soak test ---

def run_soak(port, duration_s, sample_interval_s=10):
    """Run a timed soak, sampling CLI stats at intervals."""
    samples = []
    start = time.time()
    sample_num = 0

    print(f"Starting {duration_s}s soak (sampling every {sample_interval_s}s)...")

    while time.time() - start < duration_s:
        elapsed = time.time() - start
        sample_num += 1

        print(f"  Sample {sample_num} at t={elapsed:.0f}s...")

        s_out = serial_collect_sensor_status(port)
        e_out = serial_collect_eskf_status(port)

        sample = {
            'time_s': round(elapsed, 1),
            'sensor': parse_sensor_status(s_out),
            'eskf': parse_eskf_status(e_out),
            'raw_s': s_out,
            'raw_e': e_out,
        }
        samples.append(sample)

        # Wait for next interval
        next_sample = start + sample_num * sample_interval_s
        wait = next_sample - time.time()
        if wait > 0:
            time.sleep(wait)

    return samples


# --- Output ---

def compute_rates(samples):
    """Compute per-second rates from first/last samples."""
    if len(samples) < 2:
        return {}

    first = samples[0]
    last = samples[-1]
    dt = last['time_s'] - first['time_s']
    if dt <= 0:
        return {}

    rates = {}
    for key in ['imu_reads', 'mag_reads', 'baro_reads', 'gps_reads', 'core1_loops']:
        v0 = first['sensor'].get(key, 0)
        v1 = last['sensor'].get(key, 0)
        if v0 and v1:
            rates[f'{key}_per_s'] = round((v1 - v0) / dt, 1)

    return rates


def format_markdown(samples, gdb_data=None, rates=None):
    """Format collected data as markdown."""
    lines = []
    lines.append(f"# CLA Data Collection — {datetime.now().strftime('%Y-%m-%d %H:%M')}")
    lines.append("")
    lines.append(f"**Duration:** {samples[-1]['time_s']:.0f}s soak, "
                 f"{len(samples)} samples")
    lines.append(f"**Device:** COM7 USB CDC")
    lines.append("")

    # ESKF Predict Timing — prefer 's' output (has call count), fall back to 'e'
    last_sensor = samples[-1].get('sensor', {})
    eskf = samples[-1].get('eskf', {})
    predict_src = last_sensor if 'predict_avg_us' in last_sensor else eskf
    if 'predict_avg_us' in predict_src:
        lines.append("## ESKF Predict Timing (codegen FPFT)")
        lines.append("")
        lines.append("| Metric | Value | Source |")
        lines.append("|--------|-------|--------|")
        lines.append(f"| avg | {predict_src['predict_avg_us']} µs | MEASURED-SOAK |")
        lines.append(f"| min | {predict_src['predict_min_us']} µs | MEASURED-SOAK |")
        lines.append(f"| max | {predict_src['predict_max_us']} µs | MEASURED-SOAK |")
        lines.append(f"| calls | {predict_src['predict_calls']} | MEASURED-SOAK |")
        lines.append("")

    # Sensor rates
    if rates:
        lines.append("## Sensor Rates (measured)")
        lines.append("")
        lines.append("| Sensor | Rate (Hz) | Source |")
        lines.append("|--------|-----------|--------|")
        for key, val in sorted(rates.items()):
            name = key.replace('_per_s', '').replace('_', ' ')
            lines.append(f"| {name} | {val} | MEASURED-SOAK |")
        lines.append("")

    # Error counts
    last_sensor = samples[-1].get('sensor', {})
    if last_sensor:
        lines.append("## Sensor Error Counts")
        lines.append("")
        lines.append("| Sensor | Reads | Errors | Error Rate |")
        lines.append("|--------|-------|--------|------------|")
        for sensor in ['imu', 'baro', 'gps']:
            reads = last_sensor.get(f'{sensor}_reads', 0)
            errors = last_sensor.get(f'{sensor}_errors', 0)
            rate = f"{100*errors/reads:.3f}%" if reads > 0 else "N/A"
            lines.append(f"| {sensor.upper()} | {reads} | {errors} | {rate} |")
        lines.append("")

    # Health
    if eskf:
        lines.append("## Filter Health")
        lines.append("")
        lines.append("| Metric | Value |")
        lines.append("|--------|-------|")
        for key in ['bNIS', 'mNIS', 'mahony_div_deg', 'qnorm']:
            if key in eskf:
                lines.append(f"| {key} | {eskf[key]} |")
        lines.append("")

    # GDB data
    if gdb_data:
        lines.append("## Stack High-Water Marks")
        lines.append("")
        lines.append("| Core | Used | Total | Margin |")
        lines.append("|------|------|-------|--------|")
        if 'core0_stack_used' in gdb_data:
            lines.append(f"| Core 0 | {gdb_data['core0_stack_used']} B "
                         f"| {gdb_data['core0_stack_total']} B "
                         f"| {gdb_data['core0_stack_margin_pct']}% |")
        if 'core1_stack_used' in gdb_data:
            lines.append(f"| Core 1 | {gdb_data['core1_stack_used']} B "
                         f"| {gdb_data['core1_stack_total']} B "
                         f"| {gdb_data['core1_stack_margin_pct']}% |")
        lines.append("")

    # Raw final sample for reference
    lines.append("## Raw CLI Output (final sample)")
    lines.append("")
    lines.append("### Sensor Status ('s')")
    lines.append("```")
    lines.append(samples[-1].get('raw_s', '(no data)').strip())
    lines.append("```")
    lines.append("")
    lines.append("### ESKF Status ('e')")
    lines.append("```")
    lines.append(samples[-1].get('raw_e', '(no data)').strip())
    lines.append("```")
    lines.append("")

    return '\n'.join(lines)


# --- Main ---

def main():
    parser = argparse.ArgumentParser(description='RocketChip CLA Data Collection')
    parser.add_argument('--port', default=SERIAL_PORT, help='Serial port (default: COM7)')
    parser.add_argument('--duration', type=int, default=60,
                        help='Soak duration in seconds (default: 60)')
    parser.add_argument('--interval', type=int, default=10,
                        help='Sample interval in seconds (default: 10)')
    parser.add_argument('--gdb', action='store_true',
                        help='Include GDB stack/memory analysis (requires OpenOCD)')
    parser.add_argument('--output', type=str, default=None,
                        help='Output file (default: stdout)')
    parser.add_argument('--compare', type=str, default=None,
                        help='Compare against baseline file (not yet implemented)')
    args = parser.parse_args()

    SERIAL_PORT_USED = args.port

    print(f"Connecting to {SERIAL_PORT_USED}...")
    port = serial_connect(SERIAL_PORT_USED)

    # Run soak
    samples = run_soak(port, args.duration, args.interval)

    # Compute rates
    rates = compute_rates(samples)

    # Optional GDB analysis
    gdb_data = None
    if args.gdb:
        print("Running GDB memory analysis...")
        port.close()  # Release serial so GDB can work
        gdb_data = collect_gdb_data()

    # Format output
    md = format_markdown(samples, gdb_data, rates)

    if args.output:
        outpath = Path(args.output)
        outpath.write_text(md, encoding='utf-8')
        print(f"\nData written to {outpath}")
    else:
        print("\n" + md)

    if not args.gdb:
        port.close()


if __name__ == '__main__':
    main()
