#!/usr/bin/env python3
"""Evaluate staged paths against HW-gate regexes from CONFIG_TEST_MATRIX doctrine.

Emitted for `eval` in POSIX pre-commit hooks (single source vs duplicating grep in bash):

  TRIGGER_FLIGHT_BENCH=0|1   # bench_sim.py when flight-critical FW paths staged
  TRIGGER_STATION_BENCH=0|1    # station_bench_sim.py when station-scope paths staged

Patterns must stay aligned with docs/CONFIG_TEST_MATRIX.md (Tier 6b). Host `ctest`
is handled separately in the hook when build_host/ exists."""

from __future__ import annotations

import re
import subprocess
import sys

FLIGHT_CRITICAL = re.compile(
    r'^('
    r'src/flight_director/'
    r'|src/active_objects/ao_flight_director'
    r'|src/active_objects/ao_logger'
    r'|src/cli/rc_os'
    r')'
)

STATION_SCOPE = re.compile(
    r'^('
    r'src/station/'
    r'|src/cli/rc_os_dashboard'
    r'|src/active_objects/ao_rcos'
    r'|src/active_objects/ao_telemetry'
    r'|src/active_objects/ao_radio'
    r'|src/safety/health_monitor'
    r'|src/drivers/mcu_temp'
    r'|include/rocketchip/job_capabilities'
    r'|include/rocketchip/board_fruit_jam'
    r')'
)


def _repo_root() -> str:
    return subprocess.check_output(
        ['git', 'rev-parse', '--show-toplevel'], text=True).strip()


def _git_staged_paths() -> list[str]:
    root = _repo_root()
    r = subprocess.run(
        ['git', 'diff', '--cached', '--name-only', '--diff-filter=ACMR'],
        capture_output=True, text=True, cwd=root, check=False,
    )
    if r.returncode != 0:
        return []
    return [ln for ln in r.stdout.splitlines() if ln.strip()]


def match(paths: list[str]) -> tuple[bool, bool]:
    tri_f = False
    tri_s = False
    for p in paths:
        if FLIGHT_CRITICAL.search(p):
            tri_f = True
        if STATION_SCOPE.search(p):
            tri_s = True
        if tri_f and tri_s:
            break
    return tri_f, tri_s


def main() -> int:
    paths = _git_staged_paths()
    f, s = match(paths)
    print(f'TRIGGER_FLIGHT_BENCH={1 if f else 0}')
    print(f'TRIGGER_STATION_BENCH={1 if s else 0}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
