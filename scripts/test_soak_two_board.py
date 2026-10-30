#!/usr/bin/env python3
"""Host test: soak_two_board parsers. No hardware. No ports."""
from __future__ import annotations

import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import soak_two_board as soak  # noqa: E402

# Print formats and repo records. logs/soak/ is gitignored.
V1 = '  RocketChip v0.16.0  RCOS v0.5.0  flight-e0a307e'
V2 = 'firmware tree 644b4a564399'
V3 = 'banner flight-c7d70cc6'
S1 = 'COP-P: lock  N(R)=3 V(S)=4 V(R)=2  MAC A/s1'
S2 = '  COP-P: waiting peer PLCW  N(R)=0 V(S)=0 V(R)=0  MAC A/s1'
L1 = (
    'LINK delivered=1 missing=0 crc_fail=1 copp_rtx=7 '
    'air_ms=3 air_bytes=16 copp_sync_loss=4'
)
L2 = (
    'LINK delivered=2991 missing=0 crc_fail=15 copp_rtx=0 '
    'air_ms=1 air_bytes=1 copp_sync_loss=0'
)
OLD_COP_P_RE = r'COP-P\s+(lock|waiting peer PLCW)'


def main() -> int:
    assert soak.last_str(V1 + '\n' + V2, soak.VEH_GIT_RE) == 'e0a307e'
    assert soak.last_str(V3, soak.VEH_GIT_RE) == 'c7d70cc6'
    assert soak.last_str(V2, soak.VEH_GIT_RE) is None

    assert soak.COP_P_RE == r'COP-P:?\s+(lock|waiting(?: peer PLCW)?)'
    assert soak.last_str(S1, soak.COP_P_RE) == 'lock'
    assert soak.last_str(S2, soak.COP_P_RE) == 'waiting peer PLCW'
    assert re.search(OLD_COP_P_RE, S1) is None
    assert soak.last_str(
        'Air: starcom  COP-P waiting  MAC A/s50', soak.COP_P_RE) == 'waiting'
    assert soak.last_str(
        'Air: starcom  COP-P lock  N(R)=2 V(S)=1 V(R)=2  MAC A/s50  nav',
        soak.COP_P_RE) == 'lock'
    assert soak.last_str(
        'RF Link: COP-P  LQ 100%  RX 2.3 Hz  [OK]', soak.COP_P_RE) is None

    first = soak.last_link(L1)
    assert first is not None
    assert first['copp_rtx'] == 7
    assert first['copp_sync_loss'] == 4
    last = soak.last_link(L1 + '\n' + L2)
    assert last is not None
    assert last['copp_rtx'] == 0
    assert last['delivered'] == 2991
    assert soak.last_link('') is None

    text = soak.score(
        'A0',
        V1 + '\n' + V2 + '\n' + L1,
        S1 + '\n' + L2,
        1.0,
        {},
    )
    assert 'veh_git: e0a307e' in text
    assert 'stn_copp: lock' in text
    assert 'veh_copp_rtx: 7' in text
    assert 'stn_copp_rtx: 0' in text
    assert 'uplink_disarm_sent: 0' in text

    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
