#!/usr/bin/env python3
"""Pre-commit hook gate-matrix fixture test (F-2026-05-13-001).

Audit-time validation of `scripts/ci/pre_commit_matrix.py` — the path
classifier that decides whether a staged diff triggers the vehicle
bench_sim HW gate (`TRIGGER_FLIGHT_BENCH`) and/or the station bench_sim
HW gate (`TRIGGER_STATION_BENCH`).

Exercises `pre_commit_matrix.match()` against a synthetic-staged-diff
fixture table of (paths, expected_flight, expected_station) tuples
covering known-good (gate should fire) and known-bad / no-fire
(gate should NOT fire) scenarios.

This is NOT a replacement for runtime hook enforcement — the hook
itself runs on every real commit and is exercised by the developer's
day-to-day workflow. This script answers a different question at
audit time: does the matrix classifier still produce the verdicts the
hook depends on?

Source: AUDIT_GUIDANCE.md Tier 1.3. Closes F-2026-05-13-001 (DEFER -> REMEDIATE).

Exit codes:
    0 = all fixture rows pass
    1 = at least one fixture row failed
    2 = could not import pre_commit_matrix
"""

from __future__ import annotations

import os
import sys


def _repo_root() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.dirname(os.path.dirname(here))  # scripts/audit/ -> repo


def _import_matrix():
    root = _repo_root()
    ci_dir = os.path.join(root, 'scripts', 'ci')
    if ci_dir not in sys.path:
        sys.path.insert(0, ci_dir)
    try:
        import pre_commit_matrix  # type: ignore
        return pre_commit_matrix
    except ImportError as e:
        print(f'ERROR: cannot import scripts/ci/pre_commit_matrix.py: {e}')
        sys.exit(2)


# =============================================================================
# Fixture table
#
# Each row: (label, paths, expected_flight, expected_station)
#
# - paths: list of paths as if produced by `git diff --cached --name-only`
# - expected_flight: True iff TRIGGER_FLIGHT_BENCH should fire
# - expected_station: True iff TRIGGER_STATION_BENCH should fire
#
# The matrix regexes are anchored at start-of-string (`^`) and pattern-match
# only path prefixes — anything not matching the regex should NOT fire either
# gate. Fixture exercises both edges.
# =============================================================================

FIXTURES = [
    # ----- Known-good: SHOULD fire flight gate -----
    ('flight_director module',
     ['src/flight_director/flight_director.cpp'], True, False),
    ('ao_flight_director cpp',
     ['src/active_objects/ao_flight_director.cpp'], True, False),
    ('ao_flight_director header',
     ['src/active_objects/ao_flight_director.h'], True, False),
    ('ao_logger cpp',
     ['src/active_objects/ao_logger.cpp'], True, False),
    ('cli/rc_os core',
     ['src/cli/rc_os.cpp'], True, False),

    # ----- Known-good: SHOULD fire station gate -----
    ('station subdirectory',
     ['src/station/something.cpp'], False, True),
    # rc_os_dashboard matches BOTH regexes: the flight prefix `src/cli/rc_os`
    # catches anything starting with rc_os (including _dashboard), and the
    # station regex names rc_os_dashboard explicitly. Both gates fire — by
    # design, since station-side changes that touch the dashboard module
    # also touch the CLI core path.
    ('cli/rc_os_dashboard (matches both regexes)',
     ['src/cli/rc_os_dashboard.cpp'], True, True),
    ('ao_rcos',
     ['src/active_objects/ao_rcos.cpp'], False, True),
    ('ao_telemetry',
     ['src/active_objects/ao_telemetry.cpp'], False, True),
    ('ao_radio',
     ['src/active_objects/ao_radio.cpp'], False, True),
    ('health_monitor',
     ['src/safety/health_monitor.cpp'], False, True),
    ('mcu_temp driver',
     ['src/drivers/mcu_temp.cpp'], False, True),
    ('job_capabilities header',
     ['include/rocketchip/job_capabilities.h'], False, True),
    ('board_fruit_jam header',
     ['include/rocketchip/board_fruit_jam.h'], False, True),

    # ----- Known-good: SHOULD fire BOTH gates (path mix) -----
    ('flight + station paths combined',
     ['src/flight_director/flight_director.cpp',
      'src/active_objects/ao_radio.cpp'], True, True),
    ('rc_os core + dashboard (both rc_os* -> flight; dashboard -> station)',
     ['src/cli/rc_os.cpp', 'src/cli/rc_os_dashboard.cpp'], True, True),

    # ----- Known-bad / no-fire: SHOULD NOT fire either gate -----
    ('doc-only change',
     ['docs/PROJECT_STATUS.md'], False, False),
    ('changelog only',
     ['CHANGELOG.md'], False, False),
    ('whiteboard only',
     ['AGENT_WHITEBOARD.md'], False, False),
    ('audit doc only',
     ['docs/audits/MASTER_STANDARDS_AUDIT_2026-05-13.md'], False, False),
    ('test file only',
     ['test/test_command_handler.cpp'], False, False),
    ('build script only',
     ['CMakeLists.txt'], False, False),
    ('host-only sensor seqlock header',
     ['include/rocketchip/sensor_seqlock.h'], False, False),
    ('unrelated driver',
     ['src/drivers/icm20948.cpp'], False, False),
    ('audit-tooling script',
     ['scripts/audit/pre_commit_fixture_test.py'], False, False),
    ('empty staged paths',
     [], False, False),

    # ----- Edge cases: path-prefix anchoring -----
    # Matrix regexes are anchored at `^`; a path containing "src/cli/rc_os"
    # as a substring but not a prefix should NOT match.
    ('substring-not-prefix flight',
     ['docs/audits/src/cli/rc_os_history.md'], False, False),
    ('substring-not-prefix station',
     ['docs/notes/src/active_objects/ao_radio.md'], False, False),

    # ao_logger* covers ao_logger.cpp AND ao_logger.h, AND ao_logger_writer.*
    # (any file starting with "src/active_objects/ao_logger"). Document that.
    ('ao_logger header (same prefix)',
     ['src/active_objects/ao_logger.h'], True, False),

    # rc_os* covers rc_os_commands.cpp, rc_os_dashboard.cpp, etc. The first
    # two go to flight; rc_os_dashboard goes to station explicitly (station
    # regex is also matched). Document overlap.
    ('rc_os_commands -> flight only',
     ['src/cli/rc_os_commands.cpp'], True, False),
]


def main() -> int:
    matrix = _import_matrix()

    print('=== Pre-commit gate-matrix fixture test ===')
    print(f'Source: {matrix.__file__}')
    print(f'Fixture rows: {len(FIXTURES)}')
    print()

    passed = 0
    failed = 0
    for label, paths, exp_f, exp_s in FIXTURES:
        got_f, got_s = matrix.match(paths)
        ok = (got_f == exp_f) and (got_s == exp_s)
        if ok:
            passed += 1
            status = 'PASS'
        else:
            failed += 1
            status = 'FAIL'
        if not ok:
            print(f'  [{status}] {label}')
            print(f'         paths   = {paths}')
            print(f'         expect  = flight={exp_f}, station={exp_s}')
            print(f'         got     = flight={got_f}, station={got_s}')

    print()
    print(f'  RESULT: {passed}/{len(FIXTURES)} PASS')
    print('=' * 44)
    return 0 if failed == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
