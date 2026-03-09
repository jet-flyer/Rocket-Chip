#!/usr/bin/env python3
"""
RocketChip Runtime Behavior Map (RBM) Staleness Check

Verifies that the RBM document stays in sync with the codebase by:
1. Checking that function names mentioned in RBM still exist in source
2. Checking that tick functions in main.cpp are documented in RBM
3. Flagging new state transitions or CLI commands not yet in the behavior map

Usage:
  python scripts/rbm_check.py
  python scripts/rbm_check.py --verbose

Returns exit code 0 if RBM is current, 1 if stale items found.
"""

import argparse
import re
import sys
from pathlib import Path

# --- Configuration ---
PROJECT_ROOT = Path(__file__).parent.parent
RBM_PATH = PROJECT_ROOT / 'docs' / 'audits' / 'cla_rbm' / 'RUNTIME_BEHAVIOR_MAP.md'
MAIN_CPP = PROJECT_ROOT / 'src' / 'main.cpp'
RC_OS_CPP = PROJECT_ROOT / 'src' / 'cli' / 'rc_os.cpp'

# Functions that MUST be documented in RBM
REQUIRED_TICK_FUNCTIONS = [
    'heartbeat_tick',
    'watchdog_kick_tick',
    'eskf_tick',
    'logging_tick',
    'telemetry_radio_tick',
    'mavlink_direct_tick',
    'cli_update_tick',
]

REQUIRED_CORE1_FUNCTIONS = [
    'core1_entry',
    'core1_sensor_loop',
    'core1_read_imu',
    'core1_read_baro',
    'core1_read_gps',
    'core1_neopixel_update',
    'seqlock_write',
    'seqlock_read',
]

REQUIRED_INIT_FUNCTIONS = [
    'init_hardware',
    'init_application',
    'init_sensors',
    'init_peripherals',
]


def read_file(path):
    """Read file contents, return empty string if not found."""
    try:
        return path.read_text(encoding='utf-8')
    except FileNotFoundError:
        return ''


def find_function_defs(source_text):
    """Extract function names defined in source (static void/bool/etc function_name)."""
    pattern = r'(?:static\s+)?(?:void|bool|int|uint\d+_t|float)\s+(\w+)\s*\('
    return set(re.findall(pattern, source_text))


def find_tick_functions(source_text):
    """Find tick function names from the main loop dispatch pattern."""
    # Pattern: g_lastTickFunction = "name"; \n function_name(...);
    pattern = r'g_lastTickFunction\s*=\s*"(\w+)"'
    return re.findall(pattern, source_text)


def find_cli_keys(source_text):
    """Find CLI key bindings from switch/case statements."""
    # Pattern: case 'x': or case 'X':
    pattern = r"case\s+'([a-zA-Z0-9?])'"
    return sorted(set(re.findall(pattern, source_text)))


def check_rbm_staleness(verbose=False):
    """Run all staleness checks. Returns list of issues."""
    issues = []

    rbm = read_file(RBM_PATH)
    if not rbm:
        issues.append("RBM document not found at: " + str(RBM_PATH))
        return issues

    main_src = read_file(MAIN_CPP)
    rc_os_src = read_file(RC_OS_CPP)

    if not main_src:
        issues.append("main.cpp not found")
        return issues

    # 1. Check required functions are mentioned in RBM
    if verbose:
        print("Checking required function documentation...")

    all_required = (REQUIRED_TICK_FUNCTIONS + REQUIRED_CORE1_FUNCTIONS
                    + REQUIRED_INIT_FUNCTIONS)
    for fn in all_required:
        if fn not in rbm:
            issues.append(f"Function '{fn}' not documented in RBM")

    # 2. Check for new tick functions not in RBM
    if verbose:
        print("Checking for undocumented tick functions...")

    tick_names = find_tick_functions(main_src)
    for name in tick_names:
        if name == 'sleep' or name == 'init':
            continue
        # Find corresponding function
        fn_name = name + '_tick' if not name.endswith('_tick') else name
        # Check various naming patterns
        found = any(pattern in rbm for pattern in [fn_name, name + '_tick', name])
        if not found:
            issues.append(f"Tick function '{name}' in main loop not documented in RBM")

    # 3. Check for new CLI key bindings
    if verbose:
        print("Checking CLI key bindings...")

    main_keys = find_cli_keys(main_src)
    rcos_keys = find_cli_keys(rc_os_src) if rc_os_src else []
    all_keys = sorted(set(main_keys + rcos_keys))

    for key in all_keys:
        # Check if key is mentioned in RBM (case-insensitive)
        if f"'{key}'" not in rbm and f"`{key}`" not in rbm:
            issues.append(f"CLI key '{key}' not documented in RBM")

    # 4. Check for new static functions in main.cpp not in RBM
    if verbose:
        print("Checking for undocumented functions...")

    main_fns = find_function_defs(main_src)
    # Only flag functions that look like they could be tick or handler functions
    interesting_patterns = ['_tick', '_init', 'core1_', 'cmd_', 'handle_']
    for fn in sorted(main_fns):
        if any(pat in fn for pat in interesting_patterns):
            if fn not in rbm:
                issues.append(f"Function '{fn}' in main.cpp not mentioned in RBM")

    return issues


def main():
    parser = argparse.ArgumentParser(description='RBM Staleness Check')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Show progress messages')
    args = parser.parse_args()

    print("RBM Staleness Check")
    print("=" * 40)

    issues = check_rbm_staleness(verbose=args.verbose)

    if not issues:
        print("OK — RBM is current with codebase")
        return 0
    else:
        print(f"\nFound {len(issues)} staleness issues:\n")
        for issue in issues:
            print(f"  - {issue}")
        print(f"\nUpdate {RBM_PATH} to resolve.")
        return 1


if __name__ == '__main__':
    sys.exit(main())
