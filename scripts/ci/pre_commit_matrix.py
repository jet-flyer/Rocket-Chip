#!/usr/bin/env python3
"""Evaluate staged paths against HW-gate regexes from CONFIG_TEST_MATRIX doctrine.

Emitted for `eval` in POSIX pre-commit hooks (single source vs duplicating grep in bash):

  TRIGGER_FLIGHT_BENCH=0|1   # bench_sim.py when firmware-affecting paths staged
  TRIGGER_STATION_BENCH=0|1  # station_bench_sim.py when station-scope paths staged

Patterns must stay aligned with docs/CONFIG_TEST_MATRIX.md (Tier 6b). Host `ctest`
is handled separately in the hook when build_host/ exists.

POLICY: "categories not enumerations" (council 2026-05-16, unanimous).

The FLIGHT_CRITICAL regex below is intentionally broad — ANY change that can
produce a different `rocketchip.elf` triggers bench_sim. The narrow
enumeration that lived here through commit 8cd6368 was a known structural
soft gate: file paths drift behind code, and the gate's scope did not catch
up when behavior moved between files. Two lived-experience cases:

  - LL Entry 36 (bench_sim regex rot, 2026-04-11): the gate's regex enumerated
    log-line tokens that the firmware later renamed; the gate kept passing on
    the wrong shape for 5 days.
  - LL Entry 39 (rc_log idle-drain IMU regression, 2026-05-16): commit 8adab2d
    touched `src/main.cpp` and `include/rocketchip/rc_log.h`. Neither was in
    the prior FLIGHT_CRITICAL enumeration, so the hook skipped bench_sim. The
    commit landed and silently broke Core 1's IMU reads via a downstream
    mutex/IRQ-contention path. bench_sim would have caught it instantly via
    its "wait for sensor health: VERDICT GO" gate — IF the trigger had fired.

Precedent: R-25-exec (2026-05-13, docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md)
unanimous council approval to retire `NOT_CERTIFIED_FOR_FLIGHT` / dev-tier
compile-time framing in favor of "single binary, runtime test-mode gating;
all code on the chip is flight code." This regex applies the same principle
to gate scope.

Adding files HERE: any path that produces firmware bytes (src, include,
CMakeLists, cmake helpers, vendored libs we link, gate scripts that are
themselves load-bearing per LL 36 self-rot prevention).

Removing files HERE: not without a council. The exemption discipline is
per-incident (--no-verify with repo-owner approval per DEBUG_PROBE_NOTES.md),
not category-broadening.
"""

from __future__ import annotations

import re
import subprocess
import sys

# FLIGHT_CRITICAL: anything that can change rocketchip.elf, plus the gate
# scripts themselves (LL 36 self-rot vector).
FLIGHT_CRITICAL = re.compile(
    r'^('
    # Firmware sources + headers — everything that compiles or transitively
    # influences rocketchip.elf.
    r'src/'
    r'|include/'
    # Build system — link order, compile flags, source list.
    r'|CMakeLists\.txt'
    r'|cmake/'
    # Vendored libraries we link into the firmware.
    r'|EXTERNAL/etl-'
    r'|lib/'
    # Gate self-rot vector: if these break, the rest of the gate can pass
    # while a real regression slips through. Force bench_sim on changes
    # to the gate machinery itself.
    r'|scripts/hooks/'
    r'|scripts/ci/'
    r'|scripts/bench_sim\.py'
    r'|scripts/station_bench_sim\.py'
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
