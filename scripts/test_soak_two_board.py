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

# A1 station CLI `t` block, logs/soak/2026-10-30_A1_baseline_2dbm_579f0976_stn.txt.
STN_RATE = (
    'RATE window_ms=312156 (boot-relative)\n'
    '  nav_submit=0  pltu_post=480\n'
    '  tx_start=480  tx_done=480\n'
    '  tx_busy_drop=0  tx_hold_replace=0\n'
    '  rx_crc_ok=3090  rx_crc_fail=0  station_tx=480\n'
    'RATE_HZ window_s=312 (Hz = n / window_s)\n'
    '  nav_submit=0.0 pltu_post=1.5\n'
    '  tx_start=1.5 tx_done=1.5\n'
    '  rx_crc_ok=9.9 station_tx=1.5\n'
    'LINK delivered=3090 missing=6 crc_fail=1 copp_rtx=0 air_ms=11223 air_bytes=6840 copp_sync_loss=0'
)

# A1 vehicle end CLI `t`, same log, vehicle file.
VEH_END = (
    'TX: 3147 sent  0 fail  tx=idle\n'
    'RX: 372 pkts  0 CRC err\n'
    'COP-P: lock  N(R)=0 V(S)=0 V(R)=0  MAC A/s50\n'
    'CFG: BW=250 SF=7 CR=5 nav=10Hz pwr=2dBm\n'
    'RATE window_ms=310434 (boot-relative)\n'
    '  nav_submit=2670  pltu_post=3147\n'
    '  tx_start=3147  tx_done=3147\n'
    '  tx_busy_drop=0  tx_hold_replace=0\n'
    '  rx_crc_ok=0  rx_crc_fail=0  station_tx=0\n'
    'RATE_HZ window_s=310 (Hz = n / window_s)\n'
    '  nav_submit=8.6 pltu_post=10.1\n'
    '  tx_start=10.1 tx_done=10.1\n'
    '  rx_crc_ok=0.0 station_tx=0.0\n'
    'LINK delivered=372 missing=0 crc_fail=0 copp_rtx=0 air_ms=184279 air_bytes=193603 copp_sync_loss=0'
)

# A1 vehicle start CLI `t` (01:50:27.746). RATE tx_start=2656.
VEH_START = (
    'TX: 2656 sent  0 fail  tx=idle\n'
    'RX: 313 pkts  0 CRC err\n'
    'COP-P: lock  N(R)=0 V(S)=0 V(R)=0  MAC A/s62\n'
    'CFG: BW=250 SF=7 CR=5 nav=10Hz pwr=2dBm\n'
    'RATE window_ms=262334 (boot-relative)\n'
    '  nav_submit=2253  pltu_post=2656\n'
    '  tx_start=2656  tx_done=2656\n'
    '  tx_busy_drop=0  tx_hold_replace=0\n'
    '  rx_crc_ok=0  rx_crc_fail=0  station_tx=0\n'
    'RATE_HZ window_s=262 (Hz = n / window_s)\n'
    '  nav_submit=8.5 pltu_post=10.1\n'
    '  tx_start=10.1 tx_done=10.1\n'
    '  rx_crc_ok=0.0 station_tx=0.0\n'
    'LINK delivered=313 missing=0 crc_fail=0 copp_rtx=0 air_ms=155578 air_bytes=163464 copp_sync_loss=0'
)

STN_DASH_START = 'Pkts: 2557   CRC: 1     Last: 0.0s'


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
    assert 'stn_copp_rtx_since_boot: 0' in text
    assert 'stn_copp_rtx: ' not in text
    assert 'uplink_disarm_sent: 0' in text

    stn_rate = soak.parse_rate_block(STN_RATE)
    assert stn_rate is not None
    assert stn_rate['tx_start'] == 480
    assert stn_rate['rx_crc_ok'] == 3090
    assert stn_rate['pltu_post'] == 480
    assert stn_rate['station_tx'] == 480
    assert stn_rate['rx_crc_fail'] == 0
    assert stn_rate['window_ms'] == 312156
    assert soak.last_rate(STN_RATE, 'tx_start') == 480
    colon = STN_RATE.replace('RATE window_ms=', 'RATE: window_ms=', 1)
    assert soak.last_rate(colon, 'tx_start') == 480

    veh_end_rate = soak.parse_rate_block(VEH_END)
    assert veh_end_rate is not None
    assert veh_end_rate['nav_submit'] == 2670
    assert veh_end_rate['tx_done'] == 3147

    scored_stn = soak.score('A1', '', STN_RATE, 1.0, {})
    assert 'stn_link_missing_since_boot: 6' in scored_stn
    assert 'stn_link_missing: ' not in scored_stn
    assert 'stn_copp_sync_loss: ' not in scored_stn

    scored_delta = soak.score(
        'A1', VEH_END, '', 1.0, {}, veh_start=VEH_START)
    assert 'veh_tx_sent_delta: 491' in scored_delta
    assert 'veh_copp_rtx_delta: 0' in scored_delta
    scored_no_start = soak.score('A1', VEH_END, '', 1.0, {})
    assert 'veh_tx_sent_delta: None' in scored_no_start
    assert 'veh_tx_sent_end: 3147' in scored_no_start

    # Dashboard Pkts/CRC and CLI RX pkts/CRC err are the same counters.
    stn_end = (
        'RX: 3090 pkts  seq=0  -62dBm  10dB SNR  1 CRC err\n' + STN_RATE
    )
    scored_dash = soak.score(
        'A1', '', stn_end, 1.0, {}, stn_start=STN_DASH_START + '\n')
    assert 'stn_rx_pkts_start: 2557' in scored_dash
    assert 'stn_rx_pkts_end: 3090' in scored_dash
    assert 'stn_rx_pkts_delta: 533' in scored_dash
    assert 'stn_crc_err_delta: 0' in scored_dash
    assert 'stn_link_missing_since_boot: 6' in scored_dash
    assert 'stn_copp_rtx_since_boot: 0' in scored_dash
    assert 'stn_copp_sync_loss_since_boot: 0' in scored_dash
    assert 'stn_link_missing_delta' not in scored_dash
    assert 'stn_copp_rtx_delta' not in scored_dash
    assert 'stn_copp_sync_loss_delta' not in scored_dash
    assert (
        'stn_link_counts_note: since boot; run started right after a '
        'power cycle only if the operator says so'
    ) in scored_dash

    layout_score = soak.score('A1', '', '', 1.0, {})
    assert 'layout: Nathan\'s standard desk layout' in layout_score
    assert 'operator_bar' not in layout_score
    other = soak.score('A1', '', '', 1.0, {}, layout='bench-floor')
    assert 'layout: NON-STANDARD: bench-floor' in other

    args_paper = {
        'nav_ms': '118.0',
        'plcw_ms': '46.3',
        'duty_pct': '82',
        'nav_hz': 5,
    }
    from_cfg = soak.score('A1', VEH_END, '', 1.0, args_paper)
    assert 'paper_nav_toa_ms: 59.0' in from_cfg
    assert 'paper_plcw_toa_ms: 23.2' in from_cfg
    assert 'paper_nav_hz: 10' in from_cfg
    assert 'paper_source: vehicle CFG' in from_cfg
    from_args = soak.score('A1', 'no cfg here', '', 1.0, args_paper)
    assert 'paper_source: cli args (no CFG line)' in from_args
    assert 'paper_nav_toa_ms: 118.0' in from_args
    assert 'paper_duty_pct: 82' in from_args

    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
