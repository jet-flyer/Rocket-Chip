#!/usr/bin/env python3
"""
RocketChip Enhanced Fault Injection Harness

Drives fault scenarios via GDB on the flight firmware (build_flight/rocketchip.elf)
and captures the firmware-side positive-control signal per HW_GATE_DISCIPLINE.md
Rule 1 + master-audit amendment 7.

R-25-exec step 7 (2026-05-13): retargeted to the single flight binary
per council-APPROVED Approach A (BENCH_TIER_DEPRECATION_2026-05-13.md).
Now arms test mode via probe at session start (writes
rc::kTestModeMagic to rc::g_test_mode_arm_magic, resets) before
calling any fault_force_*() hook — every hook checks
rc::test_mode_active() at entry and no-ops on production boots.

Each scenario:
  1. Drives the device into the fault condition via GDB (call a fault_force_*()
     hook from src/safety/fault_inject.cpp OR set a state variable directly).
  2. Captures serial output for a bounded window.
  3. Greps for the expected firmware-side positive-control log line.
  4. Reports SCENARIO_<name>_COMPLETE only if BOTH the script step succeeded AND
     the firmware-side signal was observed.

Scenarios:
  - launch-abort:   GDB-post SIG_ABORT to FD after force-ARM
                    Expected firmware signal: '[FD] ABORT*' log line
  - pyro-misfire:   GDB call fault_force_pio_sm_halt()
                    Expected firmware signal: '[FAULT] PIO2 all SMs disabled'
  - radio-dropout:  GDB set radio last_rx_ms to age out (force link-lost)
                    Expected firmware signal: link-lost log line
  - core1-stall:    GDB set g_core1StallTicks above threshold
                    Expected firmware signal: AO_HealthMonitor core1 FAULT log

Usage:
    python scripts/enhanced_fault_injection.py --scenario launch-abort
    python scripts/enhanced_fault_injection.py --scenario all
    python scripts/enhanced_fault_injection.py --verify        # self-test (no HW)

Prerequisites:
    - OpenOCD on 127.0.0.1:3333 with CMSIS-DAP probe attached
    - Flight firmware flashed (build_flight/rocketchip.elf)
    - Vehicle USB CDC enumerated on a known COM port
"""

import argparse
import subprocess
import sys
import time
import re
import os

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from _rc_test_common import arm_test_mode_via_probe  # noqa: E402

GDB = r'C:\Users\pow-w\.pico-sdk\toolchain\14_2_Rel1\bin\arm-none-eabi-gdb.exe'
ELF = 'build_flight/rocketchip.elf'
BAUD = 115200


def gdb_cmd(*commands, timeout=20):
    """Run GDB batch commands. Returns stdout+stderr text."""
    args = [GDB, ELF, '-batch',
            '-ex', 'target extended-remote localhost:3333']
    for c in commands:
        args.extend(['-ex', c])
    try:
        r = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
        return r.stdout + r.stderr
    except subprocess.TimeoutExpired:
        return '<GDB TIMEOUT>'


def gdb_call_with_capture(port, *gdb_commands, post_capture_s=2.5):
    """Open serial first, run GDB commands while serial is being recorded
    on a background thread, then keep capturing for post_capture_s after GDB
    returns. Returns the captured serial text.

    This pattern is required because the firmware log lines fire DURING the
    GDB call (when fault_force_*() runs), not after — serial must be open
    BEFORE the GDB call to catch them.
    """
    import serial
    import threading
    chunks = []
    stop = threading.Event()
    p = serial.Serial(port, BAUD, timeout=0.1)
    def reader():
        while not stop.is_set():
            data = p.read(4096)
            if data:
                chunks.append(data)
    t = threading.Thread(target=reader, daemon=True)
    t.start()
    time.sleep(0.3)  # let reader settle
    args = [GDB, ELF, '-batch',
            '-ex', 'target extended-remote localhost:3333']
    for c in gdb_commands:
        args.extend(['-ex', c])
    try:
        subprocess.run(args, capture_output=True, text=True, timeout=20)
    except subprocess.TimeoutExpired:
        pass
    time.sleep(post_capture_s)
    stop.set()
    t.join(timeout=1)
    p.close()
    return b''.join(chunks).decode('utf-8', errors='replace')


def find_vehicle_port():
    """Find the COM port the bench vehicle is on. RP2350 vid 0x2e8a pid 0x0009."""
    import serial.tools.list_ports
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x2e8a and p.pid == 0x0009:
            return p.device
    return None


# ============================================================================
# Scenarios — each returns (ok: bool, evidence: str)
# ============================================================================

def scenario_launch_abort(port: str):
    """Call fault_force_launch_abort() (R-9a); expect '[FD] ABORT*' log line.

    Uses the dedicated firmware hook which dispatches SIG_ARM then SIG_ABORT
    via AO_FlightDirector_dispatch_signal() — the same path CLI and
    telemetry use. Replaces the prior GDB-scratch QEvt approach that
    couldn't reliably produce posted events from inside a GDB call.
    """
    print('[FAULT] launch-abort: calling fault_force_launch_abort() via GDB...')
    log = gdb_call_with_capture(
        port,
        'monitor halt',
        'call fault_force_launch_abort()',
        'monitor resume',
        post_capture_s=3.5,
    )
    m = re.search(r'\[FD\] ABORT\b[^\n]*', log)
    if not m:
        m = re.search(r'\[FD\] CRITICAL FAULT.*ABORT[^\n]*', log)
    return (m is not None, m.group(0) if m else '<no [FD] ABORT log line observed>')


def scenario_pyro_misfire(port: str):
    """Halt PIO2 backup timers; expect '[FAULT] PIO2 all SMs disabled' log."""
    print('[FAULT] pyro-misfire: calling fault_force_pio_sm_halt() via GDB...')
    log = gdb_call_with_capture(
        port,
        'monitor halt',
        'call fault_force_pio_sm_halt()',
        'monitor resume',
        post_capture_s=2.0,
    )
    m = re.search(r'\[FAULT\] PIO2 all SMs disabled[^\n]*', log)
    return (m is not None, m.group(0) if m else '<no PIO-halt log line observed>')


def scenario_radio_dropout(port: str):
    """Call fault_force_radio_dropout() (R-9b); expect link-lost log line.

    Uses the dedicated firmware hook which calls
    AO_RfManager_force_last_rx_ms_for_test(0) and emits a [FAULT] line.
    Replaces the prior `set variable s.last_rx_ms = 0` approach which
    silently no-op'd because `s` is a file-scope-static struct member,
    not a visible global symbol.
    """
    print('[FAULT] radio-dropout: calling fault_force_radio_dropout() via GDB...')
    log = gdb_call_with_capture(
        port,
        'monitor halt',
        'call fault_force_radio_dropout()',
        'monitor resume',
        post_capture_s=3.0,
    )
    m = re.search(r'(link.{0,8}lost|\[AO_RF\][^\n]*lost|\[CMD\][^\n]*reject|kHealthRadio[^\n]*FAULT)',
                  log, re.IGNORECASE)
    if m:
        return (True, m.group(0))
    return (False, '<no link-lost log observed>')


def scenario_core1_stall(port: str):
    """Set g_core1StallTicks above threshold; expect AO_HealthMonitor FAULT propagation."""
    print('[FAULT] core1-stall: setting rc::g_core1StallTicks above threshold via GDB...')
    log = gdb_call_with_capture(
        port,
        'monitor halt',
        'set variable rc::g_core1StallTicks = 100',
        'monitor resume',
        post_capture_s=4.0,
    )
    m = re.search(r'(CORE1[^\n]*FAULT|core1.{0,10}vitality[^\n]*FAULT|kHealthCore1Ok[^\n]*FAULT|\[Health\][^\n]*core1)',
                  log, re.IGNORECASE)
    if m:
        return (True, m.group(0))
    return (False, '<no core1-FAULT log observed; health byte may change silently (no log-on-change emit)>')


SCENARIOS = {
    'launch-abort': scenario_launch_abort,
    'pyro-misfire': scenario_pyro_misfire,
    'radio-dropout': scenario_radio_dropout,
    'core1-stall': scenario_core1_stall,
}


def run_scenario(name: str, port: str, dump_log_path: str = None) -> bool:
    print(f'[FAULT] === Running scenario: {name} ===')
    fn = SCENARIOS[name]
    ok, evidence = fn(port)
    status = 'OBSERVED' if ok else 'MISSING'
    print(f'[FAULT] Firmware-side signal: {status} -- {evidence}')
    if ok:
        print(f'[FAULT] Script positive-control: SCENARIO_{name.upper()}_COMPLETE')
    else:
        print(f'[FAULT] Script signal SUPPRESSED -- firmware-side signal missing per HW_GATE Rule 1 anti-evidence')
    return ok


def verify() -> bool:
    """Self-test mode -- no HW required. Validates script structure only."""
    print('=== Fault Injection Self-Test (script structure only -- no HW driven) ===')
    for name in SCENARIOS:
        print(f'[FAULT] Scenario registered: {name}')
    print('VERDICT: PASS -- Self-test: 4 scenarios registered, script structure validated')
    print('NOTE: --verify is a script self-test only. Real fault injection requires --scenario <name> with HW attached.')
    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', choices=list(SCENARIOS.keys()) + ['all'])
    parser.add_argument('--verify', action='store_true')
    parser.add_argument('--port', help='Vehicle COM port (auto-detected if omitted)')
    args = parser.parse_args()

    if args.verify:
        success = verify()
        sys.exit(0 if success else 1)

    if not args.scenario:
        print('No scenario specified. Use --scenario <name|all> or --verify.')
        parser.print_help()
        sys.exit(2)

    port = args.port or find_vehicle_port()
    if not port:
        print('ERROR: vehicle USB CDC not found (vid 0x2e8a pid 0x0009). Use --port to specify.')
        sys.exit(2)
    print(f'[FAULT] Using port: {port}')
    print(f'[FAULT] Using ELF: {ELF}')

    # R-25-exec step 7 (2026-05-13): arm test mode at session start.
    # Probe writes rc::kTestModeMagic + resets; fault_force_*() entries
    # check test_mode_active() at entry and no-op without it.
    print('[FAULT] Arming test mode via probe (rc::g_test_mode_arm_magic = 0x7E57BABE)...')
    if not arm_test_mode_via_probe(ELF):
        print('ERROR: failed to arm test mode via probe (OpenOCD attached?)')
        sys.exit(2)
    # Give the firmware boot-time-window the chance to capture the magic
    # before scenarios start firing fault_force_*() entries.
    time.sleep(2.0)

    if args.scenario == 'all':
        results = {}
        for name in SCENARIOS:
            results[name] = run_scenario(name, port)
            time.sleep(1)
        all_pass = all(results.values())
        print('')
        print('=== SUMMARY ===')
        for name, ok in results.items():
            print(f'  {name:16s} {"PASS" if ok else "PARTIAL"}')
        sys.exit(0 if all_pass else 1)
    else:
        ok = run_scenario(args.scenario, port)
        sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
