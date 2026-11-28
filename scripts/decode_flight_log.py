#!/usr/bin/env python3
"""
RocketChip Flight Log Decoder (IVP-54b)

Decodes PCM binary flight logs from RocketChip firmware.
Supports serial download from device and offline file decoding.

Usage:
    # List flights on device
    python decode_flight_log.py --port COM7 --list

    # Download and decode flight 1 to CSV
    python decode_flight_log.py --port COM7 --flight 1 --output flight1.csv

    # Decode a previously captured binary file
    python decode_flight_log.py --decode raw.bin --output decoded.csv

Frame format: 61-byte slots (include/rocketchip/pcm_frame.h)
    Standard: [sync 2B][MET 4B][type=1][len=51][TelemetryState 51B][CRC-16 2B]
    Event:    [sync 2B][MET 4B][type=3][len=5][event_id 1B][data 4B][CRC-16 2B]
              (15 bytes, zero-padded to a 61-byte slot by pcm_pack_event_slot)

Download protocol: "RCBIN:<n>:<frames>:<rate>:<frame_size>" line, the frames,
then "RCEND:<crc32 hex>" on success or "RCERR:<reason>:<frames_sent>" if the
device gave up (host stopped reading, or the port closed).
"""

import argparse
import struct
import sys
import time
import os

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)
from _rc_test_common import (  # noqa: E402
    Banner,
    find_target_port,
    open_classified_port,
    TARGET_VEHICLE_ANY,
)

# ---------------------------------------------------------------------------
# CRC-16-CCITT (matching firmware src/logging/crc16_ccitt.h)
# ---------------------------------------------------------------------------

CRC16_TABLE = []

def _build_crc16_table():
    """Build CRC-16-CCITT table (poly 0x1021, init 0xFFFF)."""
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
        CRC16_TABLE.append(crc)

_build_crc16_table()


def crc16_ccitt(data: bytes) -> int:
    """Compute CRC-16-CCITT over data bytes."""
    crc = 0xFFFF
    for b in data:
        idx = ((crc >> 8) ^ b) & 0xFF
        crc = ((crc << 8) ^ CRC16_TABLE[idx]) & 0xFFFF
    return crc


# ---------------------------------------------------------------------------
# CRC-32 (IEEE 802.3, matching firmware src/logging/crc32.h)
# ---------------------------------------------------------------------------

def crc32(data: bytes) -> int:
    """Compute CRC-32 (IEEE 802.3) matching firmware implementation."""
    import binascii
    return binascii.crc32(data) & 0xFFFFFFFF


# ---------------------------------------------------------------------------
# PCM frame constants (matching include/rocketchip/pcm_frame.h)
# test_decode_flight_log.py checks these against the C++ headers.
# ---------------------------------------------------------------------------

SYNC_HIGH = 0xEB
SYNC_LOW = 0x90
FRAME_SIZE = 61            # kPcmFrameStandardSize; every log slot is this size
HEADER_SIZE = 8            # PcmFrameHeader
CRC_SIZE = 2
PAYLOAD_SIZE = 51          # kPcmStandardPayloadLen, sizeof(TelemetryState)
FRAME_TYPE_STANDARD = 1    # kPcmFrameTypeStandard
FRAME_TYPE_EVENT = 3       # kPcmFrameTypeEvent
EVENT_PAYLOAD_SIZE = 5     # kPcmEventPayloadLen
EVENT_FRAME_SIZE = 15      # kPcmFrameEventSize

# TelemetryState struct layout (51 bytes, packed, little-endian).
# Field order and types match include/rocketchip/telemetry_state.h.
TELEM_FORMAT = '<4h 3i 3h ih H BBBb H I B 6B'
TELEM_FIELDS = [
    'q_w', 'q_x', 'q_y', 'q_z',           # int16 x4: quaternion * 32767
    'lat_1e7', 'lon_1e7', 'alt_mm',         # int32 x3
    'vel_n_cms', 'vel_e_cms', 'vel_d_cms',  # int16 x3: cm/s
    'baro_alt_mm', 'baro_vvel_cms',         # int32 + int16: mm, cm/s
    'gps_speed_cms',                        # uint16: cm/s
    'gps_fix_sats', 'flight_state',         # uint8, uint8
    'health', 'temperature_c',              # uint8, int8
    'battery_mv',                           # uint16
    'met_ms',                               # uint32
    'flags',                                # uint8
    'utc_hour', 'utc_minute', 'utc_second',  # uint8 x6
    'utc_day', 'utc_month', 'utc_year',
]

EVENT_FORMAT = '<B4B'
EVENT_NAMES = {   # LogEventId
    1: 'pyro_fired_drogue',
    2: 'pyro_fired_main',
    3: 'abort_triggered',
    4: 'safe_mode_entry',
    5: 'safe_mode_exit',
    6: 'phase_change',
    7: 'confidence_lost',
    8: 'confidence_recovered',
}

# flags byte (telemetry_state.h)
FLAG_ZUPT_ACTIVE = 1 << 0
FLAG_MET_MISSION = 1 << 1
FLAG_UTC_VALID = 1 << 2

assert struct.calcsize(TELEM_FORMAT) == PAYLOAD_SIZE, \
    f"TelemetryState size mismatch: {struct.calcsize(TELEM_FORMAT)} != {PAYLOAD_SIZE}"
assert len(TELEM_FIELDS) == len(struct.unpack(TELEM_FORMAT, bytes(PAYLOAD_SIZE)))
assert HEADER_SIZE + PAYLOAD_SIZE + CRC_SIZE == FRAME_SIZE
assert HEADER_SIZE + EVENT_PAYLOAD_SIZE + CRC_SIZE == EVENT_FRAME_SIZE
assert struct.calcsize(EVENT_FORMAT) == EVENT_PAYLOAD_SIZE


def _frame_valid(frame_bytes: bytes, frame_type: int, payload_len: int) -> bool:
    """Triple gate from pcm_frame.h: sync, type + length, CRC-16."""
    total = HEADER_SIZE + payload_len + CRC_SIZE
    if len(frame_bytes) < total:
        return False
    if frame_bytes[0] != SYNC_HIGH or frame_bytes[1] != SYNC_LOW:
        return False
    if frame_bytes[6] != frame_type or frame_bytes[7] != payload_len:
        return False
    crc_len = HEADER_SIZE + payload_len
    crc_stored = struct.unpack_from('<H', frame_bytes, crc_len)[0]
    return crc16_ccitt(frame_bytes[:crc_len]) == crc_stored


def decode_frame(frame_bytes: bytes) -> dict | None:
    """Decode one 61-byte standard frame. Returns dict or None if invalid."""
    if len(frame_bytes) != FRAME_SIZE:
        return None
    if not _frame_valid(frame_bytes, FRAME_TYPE_STANDARD, PAYLOAD_SIZE):
        return None

    met_ms = struct.unpack_from('<I', frame_bytes, 2)[0]
    payload = frame_bytes[HEADER_SIZE:HEADER_SIZE + PAYLOAD_SIZE]
    record = dict(zip(TELEM_FIELDS, struct.unpack(TELEM_FORMAT, payload)))

    # Frame header MET is the stamp the logger used; the payload carries its own.
    record['frame_met_ms'] = met_ms
    record['met_s'] = met_ms / 1000.0
    record['q_w_f'] = record['q_w'] / 32767.0
    record['q_x_f'] = record['q_x'] / 32767.0
    record['q_y_f'] = record['q_y'] / 32767.0
    record['q_z_f'] = record['q_z'] / 32767.0
    record['lat'] = record['lat_1e7'] * 1e-7
    record['lon'] = record['lon_1e7'] * 1e-7
    record['alt_msl_m'] = record['alt_mm'] / 1000.0
    record['vel_n_mps'] = record['vel_n_cms'] / 100.0
    record['vel_e_mps'] = record['vel_e_cms'] / 100.0
    record['vel_d_mps'] = record['vel_d_cms'] / 100.0
    record['baro_alt_m'] = record['baro_alt_mm'] / 1000.0
    record['baro_vvel_mps'] = record['baro_vvel_cms'] / 100.0
    record['gps_speed_mps'] = record['gps_speed_cms'] / 100.0
    record['gps_fix'] = (record['gps_fix_sats'] >> 4) & 0x0F
    record['gps_sats'] = record['gps_fix_sats'] & 0x0F
    # health: 4 subsystems x 2 bits, IMU[1:0] Baro[3:2] ESKF[5:4] GPS[7:6]
    record['health_imu'] = record['health'] & 0x03
    record['health_baro'] = (record['health'] >> 2) & 0x03
    record['health_eskf'] = (record['health'] >> 4) & 0x03
    record['health_gps'] = (record['health'] >> 6) & 0x03
    record['zupt_active'] = bool(record['flags'] & FLAG_ZUPT_ACTIVE)
    record['met_mission'] = bool(record['flags'] & FLAG_MET_MISSION)
    record['utc_valid'] = bool(record['flags'] & FLAG_UTC_VALID)
    record['temp_c'] = record['temperature_c']
    record['battery_v'] = record['battery_mv'] / 1000.0

    return record


def decode_event(frame_bytes: bytes) -> dict | None:
    """Decode an event frame (15 bytes, or zero-padded in a 61-byte slot)."""
    if len(frame_bytes) < EVENT_FRAME_SIZE:
        return None
    if not _frame_valid(frame_bytes, FRAME_TYPE_EVENT, EVENT_PAYLOAD_SIZE):
        return None
    met_ms = struct.unpack_from('<I', frame_bytes, 2)[0]
    event_id, d0, d1, d2, d3 = struct.unpack_from(EVENT_FORMAT, frame_bytes,
                                                 HEADER_SIZE)
    return {
        'met_ms': met_ms,
        'met_s': met_ms / 1000.0,
        'event_id': event_id,
        'event': EVENT_NAMES.get(event_id, f'unknown_{event_id}'),
        'data': (d0, d1, d2, d3),
    }


def decode_frames(data: bytes) -> tuple[list[dict], list[dict], int]:
    """Split a download into 61-byte slots. Returns (records, events, corrupt)."""
    records = []
    events = []
    corrupt = 0
    for i in range(len(data) // FRAME_SIZE):
        slot = data[i * FRAME_SIZE:(i + 1) * FRAME_SIZE]
        record = decode_frame(slot)
        if record is not None:
            records.append(record)
            continue
        event = decode_event(slot)
        if event is not None:
            events.append(event)
            continue
        corrupt += 1
    return records, events, corrupt


# CSV column order (human-readable, SI units)
CSV_COLUMNS = [
    'met_s', 'q_w_f', 'q_x_f', 'q_y_f', 'q_z_f',
    'lat', 'lon', 'alt_msl_m',
    'vel_n_mps', 'vel_e_mps', 'vel_d_mps',
    'baro_alt_m', 'baro_vvel_mps',
    'gps_speed_mps', 'gps_fix', 'gps_sats',
    'flight_state', 'health_imu', 'health_baro', 'health_eskf', 'health_gps',
    'zupt_active', 'met_mission', 'temp_c', 'battery_mv',
    'utc_valid', 'utc_year', 'utc_month', 'utc_day',
    'utc_hour', 'utc_minute', 'utc_second',
]

EVENT_CSV_COLUMNS = ['met_s', 'event_id', 'event', 'd0', 'd1', 'd2', 'd3']


def _csv_cell(v) -> str:
    if isinstance(v, bool):
        return '1' if v else '0'
    if isinstance(v, float):
        return f'{v:.6f}'
    return str(v)


def events_path_for(output_path: str) -> str:
    stem, ext = os.path.splitext(output_path)
    return f'{stem}_events{ext or ".csv"}'


def decode_binary(data: bytes, output_path: str | None = None) -> tuple[int, int]:
    """Decode binary data into CSV. Returns (good_frames, corrupt_frames).

    good_frames counts standard and event frames. Events go to
    <output>_events.csv next to the main CSV.
    """
    total = len(data) // FRAME_SIZE
    if total == 0:
        print("No complete frames in data.")
        return 0, 0
    if len(data) % FRAME_SIZE:
        print(f"Warning: {len(data) % FRAME_SIZE} trailing bytes "
              f"(not a whole {FRAME_SIZE}-byte frame) ignored.")

    rows, events, corrupt = decode_frames(data)

    if output_path:
        with open(output_path, 'w') as f:
            f.write(','.join(CSV_COLUMNS) + '\n')
            for r in rows:
                f.write(','.join(_csv_cell(r.get(c, '')) for c in CSV_COLUMNS) + '\n')
        print(f"Wrote {len(rows)} frames to {output_path}")
        if events:
            ev_path = events_path_for(output_path)
            with open(ev_path, 'w') as f:
                f.write(','.join(EVENT_CSV_COLUMNS) + '\n')
                for e in events:
                    vals = [e['met_s'], e['event_id'], e['event'], *e['data']]
                    f.write(','.join(_csv_cell(v) for v in vals) + '\n')
            print(f"Wrote {len(events)} events to {ev_path}")

    print(f"Frames: {len(rows)} standard, {len(events)} event, "
          f"{corrupt} corrupt, {total} total")
    if rows:
        dt = rows[-1]['met_s'] - rows[0]['met_s']
        print(f"Duration: {dt:.1f}s ({rows[0]['met_s']:.1f}s - {rows[-1]['met_s']:.1f}s)")

    return len(rows) + len(events), corrupt


def split_download(binary_data: bytes, total_bytes: int,
                   frame_size: int) -> tuple[bytes, str, str]:
    """Split what followed the RCBIN line into (frames, status, footer).

    status is 'end' (RCEND footer right after total_bytes of frames),
    'error' (device sent RCERR and stopped early), or 'missing'.
    On 'error' only the frames_sent whole frames named in
    RCERR:<reason>:<frames_sent> are kept; bytes of a run the device was
    still writing, and the line break before RCERR, are dropped.
    """
    if binary_data[total_bytes:total_bytes + 6] == b'RCEND:':
        footer = binary_data[total_bytes:].split(b'\n', 1)[0]
        return bytes(binary_data[:total_bytes]), 'end', footer.decode('ascii', 'replace').strip()
    err_idx = binary_data.find(b'\nRCERR:')
    if err_idx >= 0:
        footer = binary_data[err_idx + 1:].split(b'\n', 1)[0]
        footer_text = footer.decode('ascii', 'replace').strip()
        parts = footer_text.split(':')
        try:
            kept = int(parts[2]) * frame_size
        except (IndexError, ValueError):
            kept = (err_idx // frame_size) * frame_size
        kept = min(kept, err_idx)
        return bytes(binary_data[:kept]), 'error', footer_text
    return bytes(binary_data[:total_bytes]), 'missing', ''


# ---------------------------------------------------------------------------
# Serial communication
# ---------------------------------------------------------------------------

def serial_list_flights(port):
    """Send 'f' to device and print flight list. `port` is open pyserial handle."""
    time.sleep(0.5)
    port.read(8192)  # drain
    time.sleep(0.1)
    port.write(b'f')
    time.sleep(1)
    data = port.read(8192)
    text = data.decode('utf-8', errors='replace')
    try:
        print(text)
    except UnicodeEncodeError:
        print(text.encode('ascii', errors='replace').decode('ascii'))


def serial_download_flight(port, flight_num: int,
                            output_path: str | None = None,
                            raw_path: str | None = None):
    """Download flight N via 'd' command, decode to CSV. `port` is open handle."""
    time.sleep(0.5)
    port.read(8192)  # drain
    time.sleep(0.1)

    port.write(b'd')
    time.sleep(0.3)
    port.write(f'{flight_num}\r'.encode())

    # Read text header: RCBIN:id:frames:rate:frame_size\n
    header_line = b''
    deadline = time.time() + 10
    while time.time() < deadline:
        chunk = port.read(256)
        header_line += chunk
        if b'RCBIN:' in header_line:
            break

    header_text = header_line.decode('utf-8', errors='replace')
    # Find RCBIN line
    rcbin_idx = header_text.find('RCBIN:')
    if rcbin_idx < 0:
        print(f"No RCBIN header received. Got: {header_text[:200]}")
        return

    # Parse header
    newline_idx = header_text.find('\n', rcbin_idx)
    if newline_idx < 0:
        # Need more data
        more = port.read(256).decode('utf-8', errors='replace')
        header_text += more
        newline_idx = header_text.find('\n', rcbin_idx)

    header_str = header_text[rcbin_idx:newline_idx].strip()
    parts = header_str.split(':')
    if len(parts) < 5:
        print(f"Malformed RCBIN header: {header_str}")
        return

    flight_id = int(parts[1])
    frame_count = int(parts[2])
    rate = int(parts[3])
    frame_size = int(parts[4])
    total_bytes = frame_count * frame_size

    print(f"Downloading flight {flight_id}: {frame_count} frames, "
          f"{rate}Hz, {frame_size}B/frame, {total_bytes} bytes total")

    # Read binary data — everything between RCBIN header and RCEND footer
    # The binary data starts right after the RCBIN header line
    remaining_after_header = header_line[header_line.find(b'\n', header_line.find(b'RCBIN:')) + 1:]
    binary_data = bytearray(remaining_after_header)

    # We need total_bytes of binary + RCEND footer
    start_time = time.time()
    while len(binary_data) < total_bytes + 20:  # +20 for RCEND line
        remaining = total_bytes + 100 - len(binary_data)
        chunk_size = min(remaining, 4096)
        chunk = port.read(chunk_size)
        if not chunk:
            # Timeout — check if we already have RCEND
            if b'RCEND:' in binary_data:
                break
            elapsed = time.time() - start_time
            if elapsed > 60:
                print("Timeout waiting for data.")
                break
            continue
        binary_data.extend(chunk)
        err_idx = binary_data.find(b'\nRCERR:')
        if err_idx >= 0 and binary_data.find(b'\n', err_idx + 1) >= 0:
            break  # device gave up; nothing more is coming

        # Progress
        pct = min(100, len(binary_data) * 100 // (total_bytes + 20))
        print(f"\r  {len(binary_data)}/{total_bytes} bytes ({pct}%)", end='', flush=True)

    print()  # newline after progress

    raw_binary, status, footer = split_download(binary_data, total_bytes,
                                                 frame_size)
    if status == 'error':
        print(f"Device aborted the download: {footer}. "
              f"Got {len(raw_binary) // frame_size} whole frames; data is incomplete.")
    elif status == 'missing':
        print("Warning: RCEND footer not found. Data may be incomplete.")
    else:
        expected_crc_str = footer.split(':')[1].strip() if ':' in footer else ''
        try:
            expected_crc = int(expected_crc_str, 16)
            actual_crc = crc32(raw_binary)
            if actual_crc == expected_crc:
                print(f"Transport CRC-32: OK ({expected_crc:#010x})")
            else:
                print(f"Transport CRC-32: MISMATCH! "
                      f"expected={expected_crc:#010x} got={actual_crc:#010x}")
        except ValueError:
            print(f"Could not parse CRC from footer: {footer}")

    elapsed = time.time() - start_time
    speed = len(raw_binary) / elapsed if elapsed > 0 else 0
    print(f"Transfer: {len(raw_binary)} bytes in {elapsed:.1f}s ({speed/1024:.1f} KB/s)")

    # Save raw binary if requested
    if raw_path:
        with open(raw_path, 'wb') as f:
            f.write(raw_binary)
        print(f"Raw binary saved to {raw_path}")

    # Decode to CSV
    csv_path = output_path
    if csv_path is None:
        csv_path = f"flight{flight_num}.csv"
    decode_binary(raw_binary, csv_path)


def serial_interactive(port, output_path: str | None = None,
                       raw_path: str | None = None):
    """List flights on device and prompt user to pick one."""
    time.sleep(0.5)
    port.read(8192)  # drain
    time.sleep(0.1)
    port.write(b'f')
    time.sleep(1)
    data = port.read(8192)

    text = data.decode('utf-8', errors='replace')
    try:
        print(text)
    except UnicodeEncodeError:
        print(text.encode('ascii', errors='replace').decode('ascii'))

    # Parse flight count from "N flights" in output
    if 'No flights stored' in text or '0 flights' in text:
        print("No flights to download.")
        return

    choice = input("Flight # to download (or Enter to quit): ").strip()
    if not choice:
        return
    try:
        flight_num = int(choice)
    except ValueError:
        print(f"Invalid number: {choice}")
        return

    csv_path = output_path if output_path else f"flight{flight_num}.csv"
    bin_path = raw_path if raw_path else f"flight{flight_num}.bin"

    serial_download_flight(port, flight_num, csv_path, bin_path)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='RocketChip Flight Log Decoder',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('--port', help='Serial port (e.g. COM7, /dev/ttyACM0)')
    parser.add_argument('--list', action='store_true',
                        help='List flights on device')
    parser.add_argument('--flight', type=int,
                        help='Download and decode flight N from device')
    parser.add_argument('--decode', metavar='FILE',
                        help='Decode a binary file offline')
    parser.add_argument('--output', '-o', metavar='FILE',
                        help='Output CSV path')
    parser.add_argument('--raw', metavar='FILE',
                        help='Save raw binary to file (with --flight)')

    args = parser.parse_args()

    if args.decode:
        if not os.path.exists(args.decode):
            print(f"File not found: {args.decode}")
            sys.exit(1)
        with open(args.decode, 'rb') as f:
            data = f.read()
        print(f"Decoding {len(data)} bytes from {args.decode}")
        good, corrupt = decode_binary(data, args.output)
        sys.exit(0 if corrupt == 0 else 1)

    # Serial operations — vehicle firmware with stored flights
    try:
        import serial  # noqa: F401
    except ImportError:
        print("pyserial required: pip install pyserial")
        sys.exit(1)

    port_name, meta = find_target_port(
        TARGET_VEHICLE_ANY, override=args.port, verbose=True)
    if port_name is None:
        print(f'No vehicle port: {meta}')
        print('  Plug the vehicle or pass --port.')
        sys.exit(2)
    if not isinstance(meta, Banner):
        print('ERROR: internal: expected Banner from find_target_port')
        sys.exit(2)

    print(f'Using {port_name} ({meta.short_summary()})')

    try:
        with open_classified_port(
            port_name,
            target=TARGET_VEHICLE_ANY,
            timeout=10.0,
        ) as port:
            if args.list:
                serial_list_flights(port)
            elif args.flight is not None:
                serial_download_flight(port, args.flight, args.output, args.raw)
            else:
                serial_interactive(port, args.output, args.raw)
    except RuntimeError as e:
        print(f'ERROR: cannot open {port_name}: {e}')
        sys.exit(2)


if __name__ == '__main__':
    main()
