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

Frame format: 55 bytes (PCM Standard)
    [sync 2B][MET 4B][type 1B][len 1B][TelemetryState 45B][CRC-16 2B]
"""

import argparse
import struct
import sys
import time
import os

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
# ---------------------------------------------------------------------------

SYNC_HIGH = 0xEB
SYNC_LOW = 0x90
FRAME_SIZE = 55
HEADER_SIZE = 8
PAYLOAD_SIZE = 45
FRAME_TYPE_STANDARD = 1

# TelemetryState struct layout (45 bytes, packed, little-endian)
# Matches include/rocketchip/telemetry_state.h exactly
TELEM_FORMAT = '<4h 3i 3h ih HBB Bb H I B'
TELEM_FIELDS = [
    'q_w', 'q_x', 'q_y', 'q_z',           # int16 x4: quaternion * 32767
    'lat_1e7', 'lon_1e7', 'alt_mm',         # int32 x3
    'vel_n_cms', 'vel_e_cms', 'vel_d_cms',  # int16 x3: cm/s
    'baro_alt_mm', 'baro_vvel_cms',         # int32 + int16: mm, cm/s
    'gps_speed_cms',                        # uint16: cm/s
    'gps_fix_sats', 'flight_state',         # uint8, uint8
    'health', 'temperature_c',              # int8 (health=uint8 treated as signed ok)
    'battery_mv',                           # uint16
    'met_ms',                               # uint32
    '_reserved',                            # uint8
]

assert struct.calcsize(TELEM_FORMAT) == PAYLOAD_SIZE, \
    f"TelemetryState size mismatch: {struct.calcsize(TELEM_FORMAT)} != {PAYLOAD_SIZE}"


def decode_frame(frame_bytes: bytes) -> dict | None:
    """Decode a single 55-byte PCM frame. Returns dict or None if invalid."""
    if len(frame_bytes) != FRAME_SIZE:
        return None

    # Gate 1: sync word
    if frame_bytes[0] != SYNC_HIGH or frame_bytes[1] != SYNC_LOW:
        return None

    # Gate 2: frame type and payload length
    frame_type = frame_bytes[6]
    payload_len = frame_bytes[7]
    if frame_type != FRAME_TYPE_STANDARD or payload_len != PAYLOAD_SIZE:
        return None

    # Gate 3: CRC-16 over bytes 0-52
    crc_computed = crc16_ccitt(frame_bytes[:53])
    crc_stored = struct.unpack_from('<H', frame_bytes, 53)[0]
    if crc_computed != crc_stored:
        return None

    # Decode header
    met_ms = struct.unpack_from('<I', frame_bytes, 2)[0]

    # Decode payload
    payload = frame_bytes[HEADER_SIZE:HEADER_SIZE + PAYLOAD_SIZE]
    values = struct.unpack(TELEM_FORMAT, payload)
    record = dict(zip(TELEM_FIELDS, values))

    # Apply scaling to produce SI units
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
    record['eskf_healthy'] = bool(record['health'] & 0x01)
    record['zupt_active'] = bool(record['health'] & 0x02)
    record['temp_c'] = record['temperature_c']
    record['battery_v'] = record['battery_mv'] / 1000.0

    return record


# CSV column order (human-readable, SI units)
CSV_COLUMNS = [
    'met_s', 'q_w_f', 'q_x_f', 'q_y_f', 'q_z_f',
    'lat', 'lon', 'alt_msl_m',
    'vel_n_mps', 'vel_e_mps', 'vel_d_mps',
    'baro_alt_m', 'baro_vvel_mps',
    'gps_speed_mps', 'gps_fix', 'gps_sats',
    'flight_state', 'eskf_healthy', 'zupt_active',
    'temp_c', 'battery_mv',
]


def decode_binary(data: bytes, output_path: str | None = None) -> tuple[int, int]:
    """Decode binary data into CSV. Returns (total_frames, corrupt_frames)."""
    total = len(data) // FRAME_SIZE
    if total == 0:
        print("No complete frames in data.")
        return 0, 0

    good = 0
    corrupt = 0
    rows = []

    for i in range(total):
        frame_bytes = data[i * FRAME_SIZE:(i + 1) * FRAME_SIZE]
        record = decode_frame(frame_bytes)
        if record is None:
            corrupt += 1
            continue
        good += 1
        rows.append(record)

    if output_path:
        with open(output_path, 'w') as f:
            f.write(','.join(CSV_COLUMNS) + '\n')
            for r in rows:
                vals = []
                for col in CSV_COLUMNS:
                    v = r.get(col, '')
                    if isinstance(v, float):
                        vals.append(f'{v:.6f}')
                    elif isinstance(v, bool):
                        vals.append('1' if v else '0')
                    else:
                        vals.append(str(v))
                f.write(','.join(vals) + '\n')
        print(f"Wrote {good} frames to {output_path}")

    # Summary
    if total > 0 and good > 0:
        dt = rows[-1]['met_s'] - rows[0]['met_s']
        print(f"Frames: {good} good, {corrupt} corrupt, {total} total")
        print(f"Duration: {dt:.1f}s ({rows[0]['met_s']:.1f}s - {rows[-1]['met_s']:.1f}s)")
    else:
        print(f"Frames: {good} good, {corrupt} corrupt, {total} total")

    return good, corrupt


# ---------------------------------------------------------------------------
# Serial communication
# ---------------------------------------------------------------------------

def auto_detect_port():
    """Auto-detect RocketChip serial port."""
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        for p in ports:
            # Pico CDC: VID 2E8A, PID 0009
            if p.vid == 0x2E8A and p.pid == 0x0009:
                return p.device
    except ImportError:
        pass
    return None


def serial_list_flights(port_name: str):
    """Send 'f' to device and print flight list."""
    import serial
    port = serial.Serial(port_name, 115200, timeout=3)
    time.sleep(0.5)
    port.read(8192)  # drain
    time.sleep(0.1)
    port.write(b'f')
    time.sleep(1)
    data = port.read(8192)
    text = data.decode('utf-8', errors='replace')
    # Handle Windows console encoding limitations
    try:
        print(text)
    except UnicodeEncodeError:
        print(text.encode('ascii', errors='replace').decode('ascii'))
    port.close()


def serial_download_flight(port_name: str, flight_num: int,
                            output_path: str | None = None,
                            raw_path: str | None = None):
    """Download flight N via 'd' command, decode to CSV."""
    import serial

    port = serial.Serial(port_name, 115200, timeout=10)
    time.sleep(0.5)
    port.read(8192)  # drain
    time.sleep(0.1)

    # Send 'd' then flight number + enter
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
        port.close()
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
        port.close()
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

        # Progress
        pct = min(100, len(binary_data) * 100 // (total_bytes + 20))
        print(f"\r  {len(binary_data)}/{total_bytes} bytes ({pct}%)", end='', flush=True)

    print()  # newline after progress
    port.close()

    # Find RCEND footer in the data
    rcend_idx = binary_data.find(b'RCEND:')
    if rcend_idx < 0:
        print("Warning: RCEND footer not found. Data may be incomplete.")
        raw_binary = bytes(binary_data[:total_bytes])
    else:
        raw_binary = bytes(binary_data[:rcend_idx])
        # Parse CRC from footer
        footer = binary_data[rcend_idx:].decode('utf-8', errors='replace').strip()
        if ':' in footer:
            expected_crc_str = footer.split(':')[1].strip()
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


def serial_interactive(port_name: str, output_path: str | None = None,
                       raw_path: str | None = None):
    """List flights on device and prompt user to pick one."""
    import serial

    port = serial.Serial(port_name, 115200, timeout=3)
    time.sleep(0.5)
    port.read(8192)  # drain
    time.sleep(0.1)
    port.write(b'f')
    time.sleep(1)
    data = port.read(8192)
    port.close()

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

    serial_download_flight(port_name, flight_num, csv_path, bin_path)


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

    # Serial operations
    port_name = args.port
    if port_name is None:
        port_name = auto_detect_port()
        if port_name is None:
            print("No port specified and auto-detect failed. Use --port.")
            sys.exit(1)
        print(f"Auto-detected: {port_name}")

    try:
        import serial  # noqa: F401
    except ImportError:
        print("pyserial required: pip install pyserial")
        sys.exit(1)

    if args.list:
        serial_list_flights(port_name)
    elif args.flight is not None:
        serial_download_flight(port_name, args.flight, args.output, args.raw)
    else:
        # Interactive mode: list flights and prompt for selection
        serial_interactive(port_name, args.output, args.raw)


if __name__ == '__main__':
    main()
