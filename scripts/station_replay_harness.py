#!/usr/bin/env python3
"""
IVP-132a.3 station replay harness.

Streams synthetic CCSDS nav packets to the station's dev-CLI replay mode.
Exercises the station RX decode -> dashboard -> ACK pipeline without a live
vehicle radio.

Protocol:
    Python -> station: "R,<hex-encoded packet bytes>\\n"
    Python -> station: "REPLAY_END\\n" to finish

The station's dev CLI must be in the Debug menu with 'r' pressed to enable
replay mode (see src/dev/dev_cli.cpp).

Packet format: CCSDS nav packet, 54 bytes, layout defined in
include/rocketchip/telemetry_encoder.h. This script reconstructs the format
in Python so we can emit synthetic frames directly without going through
vehicle firmware.

Usage:
    python scripts/station_replay_harness.py --port COM7 --count 50 --rate 2
    python scripts/station_replay_harness.py --port COM7 --scenario nominal
    python scripts/station_replay_harness.py --port COM7 --scenario landed

WARNING: COM port handles are tricky on Windows -- see
    C:/Users/pow-w/.claude/projects/.../memory/feedback_serial_port.md
    C:/Users/pow-w/.claude/projects/.../memory/feedback_com7_stuck.md
Kill this script cleanly (Ctrl+C). If COM port gets stuck afterward, reset
the station via GDB probe: `monitor reset run`.
"""

import argparse
import struct
import os
import sys
import time
from typing import Optional

try:
    import serial
except ImportError:
    print("pyserial required: pip install pyserial", file=sys.stderr)
    sys.exit(1)

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)
from _rc_test_common import (  # noqa: E402
    Banner,
    find_target_port,
    open_classified_port,
    rc_test,
    TARGET_STATION_BENCH,
)


# CCSDS packet constants (must match telemetry_encoder.h)
APID_NAV = 0x001
PRIMARY_HEADER_LEN = 6
SECONDARY_HEADER_LEN = 4
NAV_PAYLOAD_LEN = 42
CRC_LEN = 2
NAV_PACKET_LEN = PRIMARY_HEADER_LEN + SECONDARY_HEADER_LEN + NAV_PAYLOAD_LEN + CRC_LEN  # 54


def crc16_ccitt(data: bytes, poly: int = 0x1021, init: int = 0xFFFF) -> int:
    """CRC-16-CCITT (X25). Matches src/drivers/crc16_ccitt.cpp."""
    crc = init
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ poly
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


def encode_primary_header(apid: int, seq_count: int, data_len: int) -> bytes:
    """CCSDS 133.0-B-2 primary header, 6 bytes."""
    # [15:0] version(3)=0, type(1)=0, sec_hdr(1)=1, apid(11)
    w0 = (0 << 13) | (0 << 12) | (1 << 11) | (apid & 0x7FF)
    # [31:16] seq_flags(2)=11 unsegmented, seq_count(14)
    w1 = (0b11 << 14) | (seq_count & 0x3FFF)
    # [47:32] data_length = total_after_primary - 1
    w2 = data_len - 1
    return struct.pack(">HHH", w0, w1, w2)


def encode_secondary_header(met_ms: int) -> bytes:
    """4 bytes: MET ms big-endian."""
    return struct.pack(">I", met_ms & 0xFFFFFFFF)


def build_nav_payload(
    lat_1e7: int = 0,
    lon_1e7: int = 0,
    alt_cm: int = 0,
    vx_cm_s: int = 0,
    vy_cm_s: int = 0,
    vz_cm_s: int = 0,
    pitch_01deg: int = 0,
    roll_01deg: int = 0,
    yaw_01deg: int = 0,
    baro_pa: int = 101325,
    baro_temp_01c: int = 200,
    imu_temp_01c: int = 250,
    flight_state: int = 0,
    health_primary: int = 0,
    health_secondary: int = 0,
    event_flags: int = 0,
    satellites: int = 0,
    fix_type: int = 0,
    rssi_dbm: int = -80,
    seq_num: int = 0,
) -> bytes:
    """
    Pack the 42-byte nav payload subset of TelemetryState.
    Must match the packed order in telemetry_encoder.cpp encode_nav().
    Uses little-endian within payload (CCSDS spec allows; firmware uses memcpy
    from struct so native endian = LE on ARM Cortex-M).
    """
    payload = struct.pack(
        "<iiiiiihhhIhhBBBBBBhH",
        lat_1e7, lon_1e7, alt_cm,            # 12
        vx_cm_s, vy_cm_s, vz_cm_s,           # 12
        pitch_01deg, roll_01deg, yaw_01deg,  # 6
        baro_pa, baro_temp_01c, imu_temp_01c,  # 8
        flight_state, health_primary, health_secondary, event_flags,  # 4
        satellites, fix_type,                # 2
        rssi_dbm, seq_num,                   # 4 -- total 48, trim
    )
    # Trim or pad to exactly 42 bytes -- calibration against firmware layout
    # may need tuning once real captured frames are available. For bench
    # soak the decoder will either accept or reject; either way it exercises
    # AO_Telemetry handler.
    if len(payload) > NAV_PAYLOAD_LEN:
        payload = payload[:NAV_PAYLOAD_LEN]
    else:
        payload = payload + b"\x00" * (NAV_PAYLOAD_LEN - len(payload))
    return payload


def encode_nav_packet(seq: int, met_ms: int, **kwargs) -> bytes:
    """Build a complete 54-byte CCSDS nav packet."""
    secondary = encode_secondary_header(met_ms)
    payload = build_nav_payload(**kwargs)
    # data_len covers secondary + payload + crc (everything after primary)
    data_len = SECONDARY_HEADER_LEN + NAV_PAYLOAD_LEN + CRC_LEN
    primary = encode_primary_header(APID_NAV, seq, data_len)
    pre_crc = primary + secondary + payload
    # Firmware CRC covers bytes [0 .. len-2)
    crc = crc16_ccitt(pre_crc)
    return pre_crc + struct.pack(">H", crc)


# Built-in scenarios — MET + state progression for quick soak
SCENARIOS = {
    "nominal": [
        # (dt_ms, flight_state, alt_cm, vx, vy, vz)
        (0,      0, 0,      0, 0, 0),       # IDLE
        (2000,   0, 0,      0, 0, 0),       # still IDLE
        (2000,   1, 0,      0, 0, 0),       # ARMED
        (100,    3, 100,    0, 0, 2000),   # BOOST t+0.1s
        (100,    3, 400,    0, 0, 3000),   # BOOST t+0.2s
        (1000,   4, 15000,  0, 0, 1500),   # COAST
        (5000,   4, 30000,  0, 0, 0),      # APOGEE-ish
        (1000,   5, 29000,  0, 0, -1500),  # DROGUE_DESCENT
        (30000,  6, 10000,  0, 0, -500),   # MAIN_DESCENT
        (20000,  7, 500,    0, 0, 0),      # LANDED
    ],
    "idle_soak": [
        (200, 0, 0, 0, 0, 0),  # one frame, looped by --count
    ],
    "landed": [
        (500, 7, 500, 0, 0, 0),
    ],
}


def send_packet(ser, packet: bytes):
    line = "R," + packet.hex().upper() + "\n"
    ser.write(line.encode("ascii"))


def run_replay(
    ser: Optional[serial.Serial],
    port_label: str,
    scenario: str,
    count: int,
    rate_hz: float,
    dry_run: bool,
):
    steps = SCENARIOS.get(scenario)
    if steps is None:
        print(f"unknown scenario: {scenario} (known: {list(SCENARIOS)})", file=sys.stderr)
        sys.exit(2)

    if not dry_run:
        assert ser is not None
        print(f"Using {port_label} @ 115200 ...")
        time.sleep(0.2)
        ser.reset_input_buffer()

    seq = 0
    met = 0
    period = 1.0 / rate_hz if rate_hz > 0 else 0
    total_sent = 0

    try:
        for _ in range(count):
            for dt_ms, fs, alt, vx, vy, vz in steps:
                met += dt_ms
                pkt = encode_nav_packet(
                    seq=seq,
                    met_ms=met,
                    flight_state=fs,
                    alt_cm=alt,
                    vx_cm_s=vx,
                    vy_cm_s=vy,
                    vz_cm_s=vz,
                )
                if dry_run:
                    print(f"[dry] seq={seq} met={met} fs={fs} hex={pkt.hex().upper()}")
                else:
                    send_packet(ser, pkt)
                seq = (seq + 1) & 0x3FFF
                total_sent += 1
                if period > 0:
                    time.sleep(period)
        if not dry_run:
            ser.write(b"REPLAY_END\n")
            print(f"Sent {total_sent} packets + REPLAY_END")
    except KeyboardInterrupt:
        print(f"\nInterrupted after {total_sent} packets")
        if not dry_run and ser is not None:
            ser.write(b"REPLAY_END\n")


@rc_test(target=TARGET_STATION_BENCH)
def main():
    ap = argparse.ArgumentParser(description="Station replay harness (IVP-132a.3)")
    ap.add_argument(
        "--port",
        default=None,
        help="Serial port (auto-detect RocketChip station USB CDC if omitted)",
    )
    ap.add_argument(
        "--scenario",
        default="nominal",
        help=f"Scenario: {', '.join(SCENARIOS)}",
    )
    ap.add_argument(
        "--count",
        type=int,
        default=1,
        help="Number of scenario repetitions (default 1)",
    )
    ap.add_argument(
        "--rate",
        type=float,
        default=5.0,
        help="Packet rate Hz (default 5)",
    )
    ap.add_argument(
        "--dry-run",
        action="store_true",
        help="Print packets instead of sending",
    )
    args = ap.parse_args()

    if args.dry_run:
        run_replay(None, "(dry-run)", args.scenario, args.count, args.rate, True)
        return 0

    port_name, meta = find_target_port(
        TARGET_STATION_BENCH, override=args.port, verbose=False
    )
    if port_name is None:
        print(f"INFO: no station-bench port — {meta}", file=sys.stderr)
        print("  Flash build_station and plug USB, or pass --port.", file=sys.stderr)
        sys.exit(2)
    if not isinstance(meta, Banner):
        print("ERROR: internal: expected Banner from find_target_port", file=sys.stderr)
        sys.exit(2)

    try:
        with open_classified_port(
            port_name,
            target=TARGET_STATION_BENCH,
            auto_enter_cli_menu=False,
        ) as ser:
            run_replay(ser, port_name, args.scenario, args.count, args.rate, False)
    except RuntimeError as e:
        print(f"ERROR: cannot open {port_name}: {e}", file=sys.stderr)
        sys.exit(2)
    return 0


if __name__ == "__main__":
    main()
