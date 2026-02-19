#!/usr/bin/env python3
"""
ESKF GPS Soak Test (IVP-46)

Connects to RocketChip serial, enters ESKF live mode ('e'), and parses
the compact 1Hz output to validate GPS fusion behavior.

Modes:
  Indoor (default):  Verifies no false GPS origin, ZUPT active, velocity bounded.
  Outdoor:           Waits for GPS fix, validates position convergence and NIS.
  Walk:              Interactive walk test with scripted prompts.

Usage:
  python scripts/eskf_gps_soak.py [OPTIONS] [COM_PORT] [BUILD_TAG]

Options:
  --outdoor           Outdoor stationary soak (wait for GPS fix)
  --walk              Interactive walk test
  --duration=N        Soak duration in seconds (default: 60 indoor, 300 outdoor)
  --reset             Reset MCU via SWD debug probe before starting

Examples:
  python scripts/eskf_gps_soak.py COM6 ivp46-1
  python scripts/eskf_gps_soak.py --outdoor --duration=300 COM6 ivp46-1
  python scripts/eskf_gps_soak.py --walk COM6 ivp46-1
"""

import serial
import subprocess
import sys
import time
import re
import math

# Defaults
DEFAULT_PORT = "COM6"
DEFAULT_BAUD = 115200
DEFAULT_TIMEOUT = 1
DEFAULT_INDOOR_DURATION = 60
DEFAULT_OUTDOOR_DURATION = 300

# Debug probe paths
OPENOCD_EXE = r"C:\Users\pow-w\.pico-sdk\openocd\0.12.0+dev\openocd.exe"
OPENOCD_SCRIPTS = r"C:\Users\pow-w\.pico-sdk\openocd\0.12.0+dev\scripts"
GDB_EXE = r"C:\Users\pow-w\.pico-sdk\toolchain\14_2_Rel1\bin\arm-none-eabi-gdb.exe"
ELF_PATH = r"build\rocketchip.elf"

# ESKF live output regex:
# alt=0.00 vz=0.00 vh=0.00 pN=0.0 pE=0.0 Y=47.3 Patt=0.0012 Pp=0.300 bNIS=0.42 mNIS=0.31 gNIS=0.00 vNIS=0.00 Z=Y G=N
ESKF_LIVE_RE = re.compile(
    r"alt=([\d.e+-]+)\s+vz=([\d.e+-]+)\s+vh=([\d.e+-]+)\s+"
    r"pN=([\d.e+-]+)\s+pE=([\d.e+-]+)\s+Y=([\d.e+-]+)\s+"
    r"Patt=([\d.e+-]+)\s+Pp=([\d.e+-]+)\s+"
    r"bNIS=([\d.e+-]+)\s+mNIS=([\d.e+-]+)\s+"
    r"gNIS=([\d.e+-]+)\s+vNIS=([\d.e+-]+)\s+"
    r"Z=([YN])\s+G=([YN])"
)


def parse_args():
    mode = "indoor"
    duration_s = None
    port = DEFAULT_PORT
    expected_tag = None
    do_reset = False

    positional = []
    for arg in sys.argv[1:]:
        if arg == "--outdoor":
            mode = "outdoor"
        elif arg == "--walk":
            mode = "walk"
        elif arg == "--reset":
            do_reset = True
        elif arg.startswith("--duration="):
            duration_s = int(arg.split("=", 1)[1])
        else:
            positional.append(arg)

    if len(positional) >= 1:
        port = positional[0]
    if len(positional) >= 2:
        expected_tag = positional[1]

    if duration_s is None:
        duration_s = DEFAULT_OUTDOOR_DURATION if mode == "outdoor" else DEFAULT_INDOOR_DURATION

    return port, expected_tag, mode, duration_s, do_reset


def reset_mcu_via_probe():
    print("[0] Resetting MCU via debug probe...")
    subprocess.run(["taskkill", "/F", "/IM", "openocd.exe"], capture_output=True)
    time.sleep(2.0)

    ocd_proc = subprocess.Popen(
        [OPENOCD_EXE, "-s", OPENOCD_SCRIPTS,
         "-f", "interface/cmsis-dap.cfg", "-f", "target/rp2350.cfg",
         "-c", "adapter speed 5000"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    time.sleep(3.0)

    gdb_result = subprocess.run(
        [GDB_EXE, ELF_PATH, "-batch",
         "-ex", "target extended-remote localhost:3333",
         "-ex", "monitor reset halt",
         "-ex", "load",
         "-ex", "monitor reset run"],
        capture_output=True, text=True, timeout=30,
    )
    time.sleep(1.0)
    subprocess.run(["taskkill", "/F", "/IM", "openocd.exe"], capture_output=True)
    time.sleep(2.0)

    if gdb_result.returncode == 0:
        print("[0] Reset + flash OK")
    else:
        print(f"[0] WARNING: GDB returned {gdb_result.returncode}")
        print(gdb_result.stderr[:500])


def connect_serial(port):
    print(f"[1] Connecting to {port}...")
    ser = serial.Serial(port, DEFAULT_BAUD, timeout=DEFAULT_TIMEOUT)
    time.sleep(0.5)
    ser.read(4096)  # Drain buffer
    return ser


def verify_build_tag(ser, expected_tag):
    if expected_tag is None:
        print("[2] No build tag specified — skipping verification")
        return True

    print(f"[2] Verifying build tag: {expected_tag}")
    ser.write(b's')
    time.sleep(1.5)
    response = ser.read(8192).decode('utf-8', errors='replace')

    if expected_tag in response:
        print(f"[2] Build tag '{expected_tag}' confirmed")
        return True
    else:
        print(f"[2] WARNING: Build tag '{expected_tag}' not found in status output")
        lines = response.strip().split('\n')
        for line in lines[:3]:
            print(f"    {line.strip()}")
        return False


def enter_eskf_live(ser):
    """Enter ESKF live mode by pressing 'e'."""
    print("[3] Entering ESKF live mode...")
    ser.write(b'e')
    time.sleep(1.0)
    ser.read(4096)  # Drain initial output


def parse_eskf_line(line):
    """Parse one ESKF live output line, return dict or None."""
    m = ESKF_LIVE_RE.search(line)
    if not m:
        return None
    return {
        'alt': float(m.group(1)),
        'vz': float(m.group(2)),
        'vh': float(m.group(3)),
        'pN': float(m.group(4)),
        'pE': float(m.group(5)),
        'yaw': float(m.group(6)),
        'Patt': float(m.group(7)),
        'Pp': float(m.group(8)),
        'bNIS': float(m.group(9)),
        'mNIS': float(m.group(10)),
        'gNIS': float(m.group(11)),
        'vNIS': float(m.group(12)),
        'zupt': m.group(13) == 'Y',
        'origin': m.group(14) == 'Y',
    }


def run_indoor_soak(ser, duration_s):
    """Indoor soak: no GPS fix expected. Verify ZUPT, velocity bounded, no origin."""
    print(f"\n{'='*60}")
    print(f"Indoor ESKF GPS Soak — {duration_s}s")
    print(f"{'='*60}")

    enter_eskf_live(ser)

    start_time = time.time()
    samples = []
    unhealthy_count = 0
    origin_appeared = False

    while time.time() - start_time < duration_s:
        try:
            raw = ser.readline().decode('utf-8', errors='replace').strip()
        except serial.SerialException:
            print("[!] Serial error — connection lost")
            break

        if not raw:
            continue

        if "UNHEALTHY" in raw:
            unhealthy_count += 1
            print(f"  [!] UNHEALTHY at t={time.time()-start_time:.0f}s")
            continue

        d = parse_eskf_line(raw)
        if d is None:
            continue

        samples.append(d)
        elapsed = time.time() - start_time
        if d['origin']:
            origin_appeared = True

        # Progress every 10s
        if len(samples) % 10 == 0:
            print(f"  t={elapsed:.0f}s: alt={d['alt']:.2f} vh={d['vh']:.3f} "
                  f"Z={'Y' if d['zupt'] else 'N'} G={'Y' if d['origin'] else 'N'}")

    # Exit live mode
    ser.write(b'e')
    time.sleep(0.5)
    ser.read(4096)

    # Analysis
    print(f"\n--- Results ({len(samples)} samples, {duration_s}s) ---")

    failures = []

    # Check 1: No GPS origin indoors
    if origin_appeared:
        failures.append("GPS origin appeared indoors (G=Y) — unexpected")
    print(f"  G=N throughout (no origin): {'PASS' if not origin_appeared else 'FAIL'}")

    # Check 2: ZUPT active most of the time
    zupt_count = sum(1 for s in samples if s['zupt'])
    zupt_pct = (zupt_count / len(samples) * 100) if samples else 0
    zupt_ok = zupt_pct > 50.0
    if not zupt_ok:
        failures.append(f"ZUPT active only {zupt_pct:.0f}% (expected >50%)")
    print(f"  ZUPT active: {zupt_pct:.0f}% {'PASS' if zupt_ok else 'FAIL'}")

    # Check 3: Horizontal velocity bounded
    max_vh = max((s['vh'] for s in samples), default=0)
    vh_ok = max_vh < 0.5
    if not vh_ok:
        failures.append(f"Max vh={max_vh:.3f} m/s (expected <0.5)")
    print(f"  Max vh: {max_vh:.3f} m/s {'PASS' if vh_ok else 'FAIL'}")

    # Check 4: No UNHEALTHY events
    health_ok = unhealthy_count == 0
    if not health_ok:
        failures.append(f"{unhealthy_count} UNHEALTHY events")
    print(f"  Unhealthy events: {unhealthy_count} {'PASS' if health_ok else 'FAIL'}")

    # Check 5: Altitude stable within ±2m
    if samples:
        alts = [s['alt'] for s in samples]
        alt_range = max(alts) - min(alts)
        alt_ok = alt_range < 4.0  # ±2m
        if not alt_ok:
            failures.append(f"Alt range={alt_range:.2f}m (expected <4m)")
        print(f"  Alt range: {alt_range:.2f}m {'PASS' if alt_ok else 'FAIL'}")

    # Final verdict
    print()
    if not failures:
        print("=== INDOOR SOAK: PASS ===")
        return True
    else:
        print("=== INDOOR SOAK: FAIL ===")
        for f in failures:
            print(f"  - {f}")
        return False


def run_outdoor_soak(ser, duration_s):
    """Outdoor soak: wait for GPS fix, validate position convergence."""
    print(f"\n{'='*60}")
    print(f"Outdoor ESKF GPS Soak — {duration_s}s")
    print(f"{'='*60}")

    enter_eskf_live(ser)

    start_time = time.time()
    samples = []
    unhealthy_count = 0
    origin_time = None
    origin_hdop = None

    while time.time() - start_time < duration_s:
        try:
            raw = ser.readline().decode('utf-8', errors='replace').strip()
        except serial.SerialException:
            print("[!] Serial error — connection lost")
            break

        if not raw:
            continue

        if "UNHEALTHY" in raw:
            unhealthy_count += 1
            continue

        d = parse_eskf_line(raw)
        if d is None:
            continue

        samples.append(d)
        elapsed = time.time() - start_time

        if d['origin'] and origin_time is None:
            origin_time = elapsed
            print(f"  [*] Origin established at t={elapsed:.0f}s")

        # Progress every 30s
        if len(samples) % 30 == 0:
            pos_dist = math.sqrt(d['pN']**2 + d['pE']**2) if d['origin'] else 0
            print(f"  t={elapsed:.0f}s: pN={d['pN']:.1f} pE={d['pE']:.1f} "
                  f"d={pos_dist:.1f}m gNIS={d['gNIS']:.2f} vNIS={d['vNIS']:.2f} "
                  f"Z={'Y' if d['zupt'] else 'N'} G={'Y' if d['origin'] else 'N'}")

    # Exit live mode
    ser.write(b'e')
    time.sleep(0.5)
    ser.read(4096)

    # Analysis
    print(f"\n--- Results ({len(samples)} samples, {duration_s}s) ---")
    failures = []

    # Check 1: Origin established within 120s
    origin_ok = origin_time is not None and origin_time < 120.0
    if origin_time is None:
        failures.append("GPS origin never established")
    elif origin_time >= 120.0:
        failures.append(f"Origin took {origin_time:.0f}s (expected <120s)")
    print(f"  Origin established: {'%.0fs' % origin_time if origin_time else 'never'} "
          f"{'PASS' if origin_ok else 'FAIL'}")

    # Remaining checks only if we have origin
    gps_samples = [s for s in samples if s['origin']]

    if gps_samples:
        # Check 2: Max position deviation < 5m
        max_dist = max(math.sqrt(s['pN']**2 + s['pE']**2) for s in gps_samples)
        dist_ok = max_dist < 5.0
        if not dist_ok:
            failures.append(f"Max position deviation={max_dist:.1f}m (expected <5m)")
        print(f"  Max position deviation: {max_dist:.1f}m {'PASS' if dist_ok else 'FAIL'}")

        # Check 3: Mean GPS position NIS < 3.0
        gnis_vals = [s['gNIS'] for s in gps_samples if s['gNIS'] > 0]
        mean_gnis = sum(gnis_vals) / len(gnis_vals) if gnis_vals else 0
        gnis_ok = mean_gnis < 3.0
        if not gnis_ok:
            failures.append(f"Mean gNIS={mean_gnis:.2f} (expected <3.0)")
        print(f"  Mean gNIS: {mean_gnis:.2f} {'PASS' if gnis_ok else 'FAIL'}")

        # Check 4: Mean GPS velocity NIS < 3.0
        vnis_vals = [s['vNIS'] for s in gps_samples if s['vNIS'] > 0]
        mean_vnis = sum(vnis_vals) / len(vnis_vals) if vnis_vals else 0
        vnis_ok = mean_vnis < 3.0 or len(vnis_vals) == 0  # OK if no velocity updates (stationary)
        if not vnis_ok:
            failures.append(f"Mean vNIS={mean_vnis:.2f} (expected <3.0)")
        print(f"  Mean vNIS: {mean_vnis:.2f} {'PASS' if vnis_ok else 'FAIL'}")

    # Check 5: No UNHEALTHY events
    health_ok = unhealthy_count == 0
    if not health_ok:
        failures.append(f"{unhealthy_count} UNHEALTHY events")
    print(f"  Unhealthy events: {unhealthy_count} {'PASS' if health_ok else 'FAIL'}")

    # Final verdict
    print()
    if not failures:
        print("=== OUTDOOR SOAK: PASS ===")
        return True
    else:
        print("=== OUTDOOR SOAK: FAIL ===")
        for f in failures:
            print(f"  - {f}")
        return False


def run_walk_test(ser, duration_s):
    """Interactive walk test with scripted prompts."""
    print(f"\n{'='*60}")
    print(f"Interactive Walk Test")
    print(f"{'='*60}")

    enter_eskf_live(ser)
    failures = []

    # Sub-test 1: Stationary baseline (30s auto)
    print("\n--- Sub-test 1: Stationary baseline (30s) ---")
    print("  Keep device stationary...")
    samples = []
    start = time.time()
    while time.time() - start < 30:
        raw = ser.readline().decode('utf-8', errors='replace').strip()
        d = parse_eskf_line(raw)
        if d:
            samples.append(d)

    if samples:
        baseline_pN = sum(s['pN'] for s in samples[-10:]) / min(10, len(samples))
        baseline_pE = sum(s['pE'] for s in samples[-10:]) / min(10, len(samples))
        max_drift = max(math.sqrt((s['pN']-baseline_pN)**2 + (s['pE']-baseline_pE)**2)
                       for s in samples)
        drift_ok = max_drift < 5.0
        if not drift_ok:
            failures.append(f"Baseline drift={max_drift:.1f}m (expected <5m)")
        print(f"  Baseline drift: {max_drift:.1f}m {'PASS' if drift_ok else 'FAIL'}")
    else:
        failures.append("No ESKF data received during baseline")
        print("  No data received — FAIL")
        baseline_pN = 0
        baseline_pE = 0

    # Pause output for user prompt (per DEBUG_OUTPUT.md guidelines)
    ser.write(b'e')  # Exit live mode
    time.sleep(0.5)
    ser.read(4096)

    # Sub-test 2: Walk north
    print("\n--- Sub-test 2: Walk north ~10m ---")
    input("[script] Walk ~10m north, then press ENTER when stopped...")
    ser.write(b'e')  # Re-enter live mode
    time.sleep(2.0)

    walk_samples = []
    start = time.time()
    while time.time() - start < 10:
        raw = ser.readline().decode('utf-8', errors='replace').strip()
        d = parse_eskf_line(raw)
        if d:
            walk_samples.append(d)

    if walk_samples:
        pN_now = walk_samples[-1]['pN']
        delta = pN_now - baseline_pN
        walk_ok = 5.0 < delta < 20.0
        if not walk_ok:
            failures.append(f"Walk north delta={delta:.1f}m (expected 5-20m)")
        print(f"  pN delta from baseline: {delta:.1f}m {'PASS' if walk_ok else 'FAIL'}")

    # Sub-test 3: Return to start
    ser.write(b'e')
    time.sleep(0.5)
    ser.read(4096)

    print("\n--- Sub-test 3: Return to start ---")
    input("[script] Walk back to start position, then press ENTER...")
    ser.write(b'e')
    time.sleep(2.0)

    return_samples = []
    start = time.time()
    while time.time() - start < 10:
        raw = ser.readline().decode('utf-8', errors='replace').strip()
        d = parse_eskf_line(raw)
        if d:
            return_samples.append(d)

    if return_samples:
        pN_ret = return_samples[-1]['pN']
        return_delta = abs(pN_ret - baseline_pN)
        ret_ok = return_delta < 5.0
        if not ret_ok:
            failures.append(f"Return delta={return_delta:.1f}m (expected <5m)")
        print(f"  Return delta from baseline: {return_delta:.1f}m {'PASS' if ret_ok else 'FAIL'}")

    # Sub-test 4: GPS dropout
    ser.write(b'e')
    time.sleep(0.5)
    ser.read(4096)

    print("\n--- Sub-test 4: GPS dropout ---")
    input("[script] Cover GPS antenna or enter building, then press ENTER...")
    ser.write(b'e')
    time.sleep(2.0)

    dropout_samples = []
    start = time.time()
    while time.time() - start < 20:
        raw = ser.readline().decode('utf-8', errors='replace').strip()
        if "UNHEALTHY" in raw:
            failures.append("ESKF went UNHEALTHY during GPS dropout")
            print("  [!] UNHEALTHY during dropout — FAIL")
            break
        d = parse_eskf_line(raw)
        if d:
            dropout_samples.append(d)

    if dropout_samples and "UNHEALTHY" not in str(failures):
        print(f"  ESKF stayed healthy during dropout — PASS")

    # Sub-test 5: Reconvergence
    ser.write(b'e')
    time.sleep(0.5)
    ser.read(4096)

    print("\n--- Sub-test 5: Reconvergence ---")
    input("[script] Return to open sky, then press ENTER...")
    ser.write(b'e')
    time.sleep(2.0)

    reconv_samples = []
    start = time.time()
    origin_restored = False
    while time.time() - start < 30:
        raw = ser.readline().decode('utf-8', errors='replace').strip()
        d = parse_eskf_line(raw)
        if d:
            reconv_samples.append(d)
            if d['origin']:
                origin_restored = True
                break

    if origin_restored:
        print(f"  GPS reconverged at t={time.time()-start:.0f}s — PASS")
    else:
        failures.append("GPS did not reconverge within 30s")
        print("  GPS did not reconverge — FAIL")

    # Exit live mode
    ser.write(b'e')
    time.sleep(0.5)
    ser.read(4096)

    # Final verdict
    print()
    if not failures:
        print("=== WALK TEST: PASS ===")
        return True
    else:
        print("=== WALK TEST: FAIL ===")
        for f in failures:
            print(f"  - {f}")
        return False


def main():
    port, expected_tag, mode, duration_s, do_reset = parse_args()

    print(f"ESKF GPS Soak Test — mode={mode}, duration={duration_s}s, port={port}")
    print()

    if do_reset:
        reset_mcu_via_probe()

    ser = connect_serial(port)

    try:
        if expected_tag:
            verify_build_tag(ser, expected_tag)

        if mode == "indoor":
            passed = run_indoor_soak(ser, duration_s)
        elif mode == "outdoor":
            passed = run_outdoor_soak(ser, duration_s)
        elif mode == "walk":
            passed = run_walk_test(ser, duration_s)
        else:
            print(f"Unknown mode: {mode}")
            passed = False

        sys.exit(0 if passed else 1)

    finally:
        ser.close()


if __name__ == "__main__":
    main()
