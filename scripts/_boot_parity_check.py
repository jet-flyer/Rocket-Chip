#!/usr/bin/env python3
"""Boot-parity banner-verification helper for verify_boot_parity.sh.

R-25-exec step 12 (2026-05-13). Reads expected role from argv,
finds the RocketChip USB CDC port, calls peek_banner, and verifies
classification matches the expected role.

Exit codes:
    0 = banner matches expected role and parses cleanly
    1 = banner mismatch or unparseable
    2 = port not found
"""

import os
import sys

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from _rc_test_common import (
    Role,
    find_target_port,
    peek_banner,
    TARGET_VEHICLE_ANY,
    TARGET_STATION_ANY,
)


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ('vehicle', 'station'):
        print('usage: _boot_parity_check.py {vehicle|station}',
              file=sys.stderr)
        sys.exit(2)
    expected = sys.argv[1]
    target = (TARGET_VEHICLE_ANY if expected == 'vehicle'
              else TARGET_STATION_ANY)

    port, info = find_target_port(target)
    if port is None:
        print(f'  no {expected} USB CDC port found ({info})',
              file=sys.stderr)
        sys.exit(2)
    banner = peek_banner(port)
    if banner is None or not banner.is_known():
        print(f'  banner unparseable on {port}', file=sys.stderr)
        sys.exit(1)
    expected_role = (Role.VEHICLE if expected == 'vehicle'
                     else Role.STATION)
    if banner.role is not expected_role:
        print(f'  role mismatch: expected={expected} got={banner.role}',
              file=sys.stderr)
        sys.exit(1)
    print(f'  banner: {banner.short_summary()}')
    sys.exit(0)


if __name__ == '__main__':
    main()
