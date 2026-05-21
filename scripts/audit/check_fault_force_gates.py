#!/usr/bin/env python3
"""R-25-exec audit invariant — mechanical check.

The runtime test-mode gate (`rc::test_mode_active()`) is the single point
of refusal for every state-mutating test affordance per
docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md (council-APPROVED
Approach A). The audit invariant is:

  every fault_force_* entry in src/safety/{fault_inject, station_fault_inject}.cpp
  must call fi_test_mode_gate() / fis_test_mode_gate() as the first
  executable line of its body, OR be in this script's documented
  recovery-action exemption allowlist.

  every state-mutating Debug-menu branch in src/cli/rc_os_debug.cpp must
  check rc::test_mode_active() before executing.

Previously this was a grep + manual walk-through. This script makes the
check mechanical so the audit gate can never silently rot.

Source: standards/CODING_STANDARDS.md "R-25-exec test-mode audit invariant
(2026-05-13)" + docs/PROBLEM_REPORTS.md R-25-exec touch list ("audit
invariant: every fault_force_* checks rc::test_mode_active() at entry;
verified by grep + manual walk-through"). This script replaces the manual
walk-through.

Exit codes:
    0 = audit invariant holds (or only NOTE-level findings)
    1 = audit invariant violated (at least one entry missing its gate)
    2 = environment skip (source files not readable)
"""

from __future__ import annotations

import os
import re
import sys
from typing import List, Optional, Tuple


# ----------------------------------------------------------------------------
# Allowlist of fault_force_* entries that are recovery actions and
# intentionally NOT gated. Each entry must include a one-line rationale
# that matches what the source code's adjacent comment says.
# ----------------------------------------------------------------------------

RECOVERY_EXEMPT_VEHICLE = {
    'fault_force_core0_stall_clear': (
        'Recovery action — clearing the stall must remain reachable '
        'even after test_mode_active() clears on IDLE-exit.'
    ),
}

RECOVERY_EXEMPT_STATION = {
    'fault_force_station_gps_restore': (
        'Recovery action — clearing the injected fault must remain '
        'reachable even after test_mode_active() clears. Symmetric '
        'with vehicle fault_force_core0_stall_clear.'
    ),
}


# ----------------------------------------------------------------------------
# Source-text parser
# ----------------------------------------------------------------------------

def _repo_root() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.dirname(os.path.dirname(here))


# Match a fault_force_<name>(...) function definition. Captures the
# function name. Accepts `void` or `bool` return + arbitrary parameters.
# The `extern "C"` and `__attribute__((used))` attributes appear on
# preceding lines (we don't require them on the same line).
_RE_FUNC_DEF = re.compile(
    r'^\s*(?:void|bool)\s+(fault_force_[a-z0-9_]+)\s*\([^)]*\)\s*\{?\s*$'
)

# Match the gate call. Accepts either fi_test_mode_gate (vehicle) or
# fis_test_mode_gate (station). Must be inside an `if (!...)` early-return
# pattern. The gate's argument is a string literal (verified by uniform
# `"function_name"` shape — the script's `name`-arg matching is loose
# because the rejection log isn't load-bearing for the audit invariant).
_RE_GATE_CALL = re.compile(
    r'if\s*\(\s*!\s*(fi_test_mode_gate|fis_test_mode_gate)\s*\(\s*"[^"]*"\s*\)\s*\)\s*\{?\s*return\s*;\s*\}?'
)


def find_function_bodies(
    src_path: str,
    expected_gate_fn: str,
    exempt: dict,
) -> List[Tuple[str, int, Optional[str]]]:
    """Walk a fault_inject*.cpp source file and yield each
    fault_force_* function with its first non-comment, non-empty body
    line (or `None` if no body could be parsed).

    Returns list of (function_name, line_number, first_body_line_or_None).
    """
    with open(src_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    results = []
    i = 0
    n = len(lines)
    while i < n:
        m = _RE_FUNC_DEF.match(lines[i])
        if not m:
            i += 1
            continue
        func_name = m.group(1)
        func_def_line = i + 1  # 1-based
        # Walk to first body line. Handle `void name(){` on same line,
        # or `void name()` followed by `{` on next.
        j = i + 1
        # Skip the opening brace if it's on its own line.
        if j < n and lines[j].strip() == '{':
            j += 1
        # Now find first non-comment, non-blank line.
        first_body = None
        while j < n:
            stripped = lines[j].strip()
            if stripped == '':
                j += 1
                continue
            if stripped.startswith('//'):
                j += 1
                continue
            # Multi-line /* ... */ block — skip until close.
            if stripped.startswith('/*'):
                while j < n and '*/' not in lines[j]:
                    j += 1
                j += 1  # skip the closing-line too
                continue
            # Closing brace = empty body.
            if stripped == '}':
                first_body = '(empty body)'
                break
            first_body = stripped
            break
        results.append((func_name, func_def_line, first_body))
        i = j + 1
    return results


def check_vehicle_or_station(
    src_path: str,
    expected_gate_fn: str,
    exempt: dict,
    label: str,
) -> Tuple[int, int, List[str]]:
    """Return (n_gated_ok, n_violations, list_of_violation_messages)."""
    if not os.path.exists(src_path):
        return (0, 0, [f'  ERR: source file not found: {src_path}'])

    bodies = find_function_bodies(src_path, expected_gate_fn, exempt)
    n_ok = 0
    n_violation = 0
    n_exempt = 0
    findings: List[str] = []
    msgs: List[str] = []

    findings.append(f'\n{label} — {len(bodies)} fault_force_* entries found')
    findings.append(f'  source: {os.path.relpath(src_path, _repo_root())}')
    findings.append(f'  expected gate: {expected_gate_fn}()')

    for name, line, first_body in bodies:
        if name in exempt:
            n_exempt += 1
            findings.append(
                f'  [EXEMPT]   {name} (line {line}) — recovery action'
            )
            continue
        if first_body is None:
            n_violation += 1
            msg = (f'{label}: {name} at line {line} — could not parse '
                   f'function body')
            msgs.append(msg)
            findings.append(f'  [PARSE-FAIL] {name} (line {line})')
            continue
        # Look for the gate-call pattern on the first body line.
        m = _RE_GATE_CALL.search(first_body)
        if m is None:
            n_violation += 1
            msg = (f'{label}: {name} at line {line} — first body line is '
                   f'{first_body!r} but expected '
                   f'`if (!{expected_gate_fn}("...")) {{ return; }}`')
            msgs.append(msg)
            findings.append(f'  [VIOLATE]  {name} (line {line})')
            findings.append(f'             first body: {first_body[:100]}')
            continue
        gate_fn_used = m.group(1)
        if gate_fn_used != expected_gate_fn:
            n_violation += 1
            msg = (f'{label}: {name} at line {line} — called '
                   f'{gate_fn_used}() but expected {expected_gate_fn}()')
            msgs.append(msg)
            findings.append(f'  [VIOLATE]  {name} (line {line}) wrong gate '
                            f'fn: {gate_fn_used}')
            continue
        n_ok += 1
        findings.append(f'  [GATED]    {name} (line {line})')

    findings.append(f'  Summary: {n_ok} gated + {n_exempt} exempt + '
                    f'{n_violation} violations')
    for f in findings:
        print(f)
    return (n_ok, n_violation, msgs)


# ----------------------------------------------------------------------------
# rc_os_debug.cpp state-mutating-branch verification
# ----------------------------------------------------------------------------

# State-mutating Debug-menu cases that must check rc::test_mode_active()
# before executing. Maps case-label -> short description (just for the
# audit report; the check itself looks for the test_mode_active() call
# within the case body, not for the description string).
DEBUG_MENU_STATE_MUTATING_CASES = {
    "'l'": 'LED test (dev_led_test_menu / dev_led_test_force) — writes LED engine override',
    "'0'..'5'": 'Local radio config set — drives AO_Radio_set_pending_config',
}


def check_debug_menu_gates(src_path: str) -> Tuple[int, int, List[str]]:
    """Verify state-mutating Debug-menu branches in rc_os_debug.cpp call
    rc::test_mode_active() before executing.

    Returns (n_gated_ok, n_violations, violation_messages).
    """
    if not os.path.exists(src_path):
        return (0, 0, [f'  ERR: source file not found: {src_path}'])

    with open(src_path, 'r', encoding='utf-8') as f:
        text = f.read()

    msgs: List[str] = []
    findings = ['\nrc_os_debug.cpp — state-mutating Debug-menu branches']
    findings.append(f'  source: {os.path.relpath(src_path, _repo_root())}')

    # LED-test branch — `case 'l': case 'L':` followed by an
    # `if (!rc::test_mode_active())` check before `dev_led_test_menu()`.
    led_branch_re = re.compile(
        r"case\s+'l'\s*:\s*case\s+'L'\s*:\s*\n"   # case 'l': case 'L':
        r"(?:[^\n]*\n){0,4}?"                      # up to 4 lines of comment
        r"\s*if\s*\(\s*!\s*rc::test_mode_active\s*\(\s*\)\s*\)",
        re.MULTILINE,
    )
    if led_branch_re.search(text):
        findings.append("  [GATED]    case 'l'/'L' — LED test")
        n_ok_led = 1
        n_violation_led = 0
    else:
        msgs.append("rc_os_debug.cpp: case 'l'/'L' (LED test) does not call "
                    "rc::test_mode_active() before executing")
        findings.append("  [VIOLATE]  case 'l'/'L' — LED test missing gate")
        n_ok_led = 0
        n_violation_led = 1

    # Radio-config-set branch — `case '0': case '1': ... case '5':`
    # followed by an `if (!rc::test_mode_active())` check.
    cfg_branch_re = re.compile(
        r"case\s+'0'\s*:\s*case\s+'1'\s*:\s*case\s+'2'\s*:\s*case\s+'3'\s*:\s*case\s+'4'\s*:\s*case\s+'5'\s*:\s*\{?\s*\n"
        r"(?:[^\n]*\n){0,4}?"
        r"\s*if\s*\(\s*!\s*rc::test_mode_active\s*\(\s*\)\s*\)",
        re.MULTILINE,
    )
    if cfg_branch_re.search(text):
        findings.append("  [GATED]    case '0'..'5' — local radio config set")
        n_ok_cfg = 1
        n_violation_cfg = 0
    else:
        msgs.append("rc_os_debug.cpp: case '0'..'5' (radio config set) does not "
                    "call rc::test_mode_active() before executing")
        findings.append("  [VIOLATE]  case '0'..'5' — radio config set missing gate")
        n_ok_cfg = 0
        n_violation_cfg = 1

    n_ok = n_ok_led + n_ok_cfg
    n_violation = n_violation_led + n_violation_cfg
    findings.append(f'  Summary: {n_ok} gated + {n_violation} violations')

    for line in findings:
        print(line)

    return (n_ok, n_violation, msgs)


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------

def main() -> int:
    repo = _repo_root()
    print('=' * 72)
    print('  R-25-exec audit invariant check')
    print('  source: standards/CODING_STANDARDS.md "R-25-exec test-mode '
          'audit invariant"')
    print('=' * 72)

    vehicle_src = os.path.join(repo, 'src', 'safety', 'fault_inject.cpp')
    station_src = os.path.join(repo, 'src', 'safety', 'station_fault_inject.cpp')
    debug_src = os.path.join(repo, 'src', 'cli', 'rc_os_debug.cpp')

    for src in (vehicle_src, station_src, debug_src):
        if not os.path.exists(src):
            print(f'\nSKIP: source file not present at {src}')
            return 2

    n_ok_v, n_viol_v, msgs_v = check_vehicle_or_station(
        vehicle_src, 'fi_test_mode_gate', RECOVERY_EXEMPT_VEHICLE,
        label='Vehicle (src/safety/fault_inject.cpp)',
    )
    n_ok_s, n_viol_s, msgs_s = check_vehicle_or_station(
        station_src, 'fis_test_mode_gate', RECOVERY_EXEMPT_STATION,
        label='Station (src/safety/station_fault_inject.cpp)',
    )
    n_ok_d, n_viol_d, msgs_d = check_debug_menu_gates(debug_src)

    total_ok = n_ok_v + n_ok_s + n_ok_d
    total_viol = n_viol_v + n_viol_s + n_viol_d

    print('\n' + '=' * 72)
    print(f'  TOTAL: {total_ok} gated entries + {total_viol} violations')
    print('=' * 72)

    if total_viol > 0:
        print('\nVIOLATIONS:')
        for m in msgs_v + msgs_s + msgs_d:
            print(f'  - {m}')
        return 1
    print('\nAUDIT INVARIANT HOLDS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
