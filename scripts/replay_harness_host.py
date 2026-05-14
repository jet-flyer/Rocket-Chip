#!/usr/bin/env python3
"""
Host-side replay harness — placeholder skeleton.

R-25-exec step 7 (2026-05-13, per council amendment #4 in
docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md): the on-MCU
CSV-streamer replay paths (src/dev/replay_inject.cpp +
src/dev/station_replay.cpp) were deleted in steps 5 + 6. Per IVP-131's
verification-model shift, replay coverage moves host-side: run the
ESKF on the host workstation against the same CSV profiles
(tests/replay_profiles/*.csv) and compare against a ground-truth
reference.

This file is a SKELETON / PLACEHOLDER. The full host-side ESKF
replay implementation is tracked separately — it requires:

  1. A host-buildable ESKF library (the rc_flight_director +
     rc_math + rc_logging libs already build on host; we need to
     verify ESKF state-update is reachable without the QP/C AO
     scaffolding).
  2. A CSV→ESKF driver that emits per-tick state snapshots.
  3. A comparison harness against tests/replay_profiles/*.csv
     ground-truth columns (phase transitions, max altitude, abort
     conditions).
  4. Coverage parity with the IVP-131 oracle that
     scripts/replay_gate_test.py used to provide.

Until the implementation lands, this script just informs the
operator of the path forward. Per AUDIT_GUIDANCE.md Appendix C.5,
the replay-gate coverage is currently DEFERRED-WITH-RATIONALE:
the on-MCU path was removed because it could not be enabled in
the flight binary without violating the runtime-partitioning
discipline (SWE-133); the host-side rewrite is the IRL-aerospace
pattern (PX4 SITL, ArduPilot SITL).
"""

import sys


_USAGE = """\
Usage: python scripts/replay_harness_host.py <profile.csv>

NOTE: Implementation pending. This script is a placeholder for the
post-R-25-exec host-side replay harness (see module docstring).

Status:
  - On-MCU replay_inject.cpp + station_replay.cpp DELETED in R-25-exec
    steps 5/6 (council amendment #4, 2026-05-13).
  - Host-side ESKF replay is the IVP-131 verification-model shift.
  - Until implemented, IVP-131 coverage is deferred-with-rationale per
    AUDIT_GUIDANCE.md Appendix C.5.
"""


def main():
    print(_USAGE)
    print('exit code 2 = "not implemented yet"', file=sys.stderr)
    sys.exit(2)


if __name__ == '__main__':
    main()
