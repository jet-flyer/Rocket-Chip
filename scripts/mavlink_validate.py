#!/usr/bin/env python3
"""
MAVLink v2 Wire Format Validator for IVP-61

Reads raw bytes from the ground station COM port and validates:
  1. MAVLink v2 framing (0xFD magic, correct payload lengths)
  2. Expected message IDs (HEARTBEAT=0, SYS_STATUS=1, ATTITUDE=30, GLOBAL_POSITION_INT=33)
  3. Monotonic sequence numbers (no gaps or resets)
  4. CRC-16/MCRF4XX integrity
  5. 1 Hz heartbeat timing

Usage:
  python scripts/mavlink_validate.py [COM_PORT] [DURATION_S]
  python scripts/mavlink_validate.py COM9        # 30s default
  python scripts/mavlink_validate.py COM9 60     # 60s run

Requires: pip install pyserial
"""

import serial
import struct
import sys
import time

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM9"
DURATION = int(sys.argv[2]) if len(sys.argv) > 2 else 30
BAUD = 115200

# MAVLink v2 CRC-16/MCRF4XX (X.25)
# Same as used by c_library_v2: init=0xFFFF, poly=0x1021
def crc_accumulate(byte, crc):
    tmp = byte ^ (crc & 0xFF)
    tmp ^= (tmp << 4) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF

def mavlink_crc(data, crc_extra):
    crc = 0xFFFF
    for b in data:
        crc = crc_accumulate(b, crc)
    crc = crc_accumulate(crc_extra, crc)
    return crc

# CRC_EXTRA seeds per message ID (from c_library_v2 headers)
CRC_EXTRA = {
    0:  50,   # HEARTBEAT
    1:  124,  # SYS_STATUS
    30: 39,   # ATTITUDE
    33: 104,  # GLOBAL_POSITION_INT
}

MSG_NAMES = {
    0:  "HEARTBEAT",
    1:  "SYS_STATUS",
    30: "ATTITUDE",
    33: "GLOBAL_POSITION_INT",
}

# MAVLink v2 truncates trailing zero bytes, so actual payload can be shorter
# than the canonical length. We check <= max, not ==.
MAX_PAYLOAD_LEN = {
    0:  9,
    1:  31,
    30: 28,
    33: 28,  # I(4)+i(4)+i(4)+i(4)+i(4)+h(2)+h(2)+h(2)+H(2) = 28
}


def parse_frame(port):
    """Read one MAVLink v2 frame from serial. Returns (msgid, seq, payload) or None."""
    # Sync to 0xFD
    while True:
        b = port.read(1)
        if len(b) == 0:
            return None  # timeout
        if b[0] == 0xFD:
            break

    # Read header bytes 1-9 (after magic)
    hdr = port.read(9)
    if len(hdr) < 9:
        return None

    payload_len = hdr[0]
    incompat_flags = hdr[1]
    compat_flags = hdr[2]
    seq = hdr[3]
    sysid = hdr[4]
    compid = hdr[5]
    msgid = hdr[6] | (hdr[7] << 8) | (hdr[8] << 16)

    # Read payload + CRC
    payload = port.read(payload_len)
    crc_bytes = port.read(2)
    if len(payload) < payload_len or len(crc_bytes) < 2:
        return None

    frame_crc = struct.unpack('<H', crc_bytes)[0]

    return {
        'msgid': msgid,
        'seq': seq,
        'sysid': sysid,
        'compid': compid,
        'payload_len': payload_len,
        'payload': payload,
        'crc': frame_crc,
        'header': hdr,
        'incompat': incompat_flags,
        'compat': compat_flags,
    }


def validate_crc(frame):
    """Validate CRC-16/MCRF4XX with CRC_EXTRA."""
    msgid = frame['msgid']
    if msgid not in CRC_EXTRA:
        return False, f"Unknown msgid {msgid}, can't verify CRC"

    # CRC covers bytes 1-9 (header after magic) + payload
    data = bytes(frame['header']) + bytes(frame['payload'])
    expected = mavlink_crc(data, CRC_EXTRA[msgid])
    if frame['crc'] != expected:
        return False, f"CRC mismatch: got 0x{frame['crc']:04X}, expected 0x{expected:04X}"
    return True, "OK"


def decode_heartbeat(payload):
    """Decode HEARTBEAT payload fields."""
    padded = payload.ljust(9, b'\x00')
    custom_mode, mav_type, autopilot, base_mode, system_status = struct.unpack(
        '<IBBBBx', padded[:9])
    return {
        'custom_mode': custom_mode,
        'type': mav_type,
        'autopilot': autopilot,
        'base_mode': base_mode,
        'system_status': system_status,
    }


def decode_attitude(payload):
    """Decode ATTITUDE payload fields."""
    # Pad to canonical length — MAVLink v2 truncates trailing zeros
    padded = payload.ljust(28, b'\x00')
    time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed = struct.unpack(
        '<Iffffff', padded[:28])
    return {
        'time_boot_ms': time_boot_ms,
        'roll_deg': roll * 57.2958,
        'pitch_deg': pitch * 57.2958,
        'yaw_deg': yaw * 57.2958,
    }


def decode_global_pos(payload):
    """Decode GLOBAL_POSITION_INT payload fields."""
    # GLOBAL_POSITION_INT: I(4)+i(4)+i(4)+i(4)+i(4)+h(2)+h(2)+h(2)+H(2) = 28 bytes, 9 fields
    padded = payload.ljust(28, b'\x00')
    time_boot_ms, lat, lon, alt, relative_alt, vx, vy, vz, hdg = struct.unpack(
        '<IiiiihhhH', padded[:28])
    return {
        'time_boot_ms': time_boot_ms,
        'lat': lat * 1e-7,
        'lon': lon * 1e-7,
        'alt_m': alt * 0.001,
        'rel_alt_m': relative_alt * 0.001,
        'vx_cms': vx,
        'vy_cms': vy,
        'vz_cms': vz,
        'hdg': hdg,
    }


def main():
    print(f"=== MAVLink v2 Wire Format Validator (IVP-61) ===")
    print(f"Port: {PORT}, Baud: {BAUD}, Duration: {DURATION}s")
    print()

    try:
        port = serial.Serial(PORT, BAUD, timeout=2)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {PORT}: {e}")
        sys.exit(1)

    time.sleep(0.5)
    port.reset_input_buffer()

    # Counters
    msg_counts = {}
    crc_errors = 0
    seq_errors = 0
    last_seq = -1
    total_frames = 0
    heartbeat_times = []
    start = time.time()
    unknown_msgs = set()

    print(f"{'Time':>6s}  {'Seq':>4s}  {'MsgID':>5s}  {'Name':<22s}  {'CRC':>5s}  Details")
    print("-" * 90)

    while time.time() - start < DURATION:
        frame = parse_frame(port)
        if frame is None:
            continue

        total_frames += 1
        msgid = frame['msgid']
        seq = frame['seq']
        elapsed = time.time() - start

        # Count messages
        msg_counts[msgid] = msg_counts.get(msgid, 0) + 1

        # CRC check
        crc_ok, crc_msg = validate_crc(frame)
        if not crc_ok:
            crc_errors += 1
        crc_str = "OK" if crc_ok else "FAIL"

        # Sequence check
        if last_seq >= 0:
            expected_seq = (last_seq + 1) & 0xFF
            if seq != expected_seq:
                seq_errors += 1
                crc_str += f" seq_gap({last_seq}->{seq})"
        last_seq = seq

        # Decode details
        name = MSG_NAMES.get(msgid, f"UNKNOWN({msgid})")
        details = ""

        if msgid == 0:  # HEARTBEAT
            heartbeat_times.append(elapsed)
            d = decode_heartbeat(frame['payload'])
            details = f"type={d.get('type','')} state={d.get('system_status','')} mode={d.get('custom_mode','')}"
        elif msgid == 30:  # ATTITUDE
            d = decode_attitude(frame['payload'])
            details = f"R={d.get('roll_deg',0):.1f}° P={d.get('pitch_deg',0):.1f}° Y={d.get('yaw_deg',0):.1f}°"
        elif msgid == 33:  # GLOBAL_POSITION_INT
            d = decode_global_pos(frame['payload'])
            lat = d.get('lat', 0)
            lon = d.get('lon', 0)
            alt = d.get('alt_m', 0)
            hdg = d.get('hdg', 0)
            if lat == 0 and lon == 0:
                details = "NO_GPS"
            else:
                details = f"({lat:.7f}, {lon:.7f}) alt={alt:.1f}m hdg={hdg}"
        elif msgid == 1:  # SYS_STATUS
            details = f"plen={frame['payload_len']}"
        else:
            unknown_msgs.add(msgid)
            details = f"payload_len={frame['payload_len']}"

        # Validate payload lengths (MAVLink v2 truncates trailing zeros)
        if msgid in MAX_PAYLOAD_LEN:
            if frame['payload_len'] > MAX_PAYLOAD_LEN[msgid]:
                details += f" BAD_LEN(max {MAX_PAYLOAD_LEN[msgid]}, got {frame['payload_len']})"

        # Validate sysid/compid
        if frame['sysid'] != 1 or frame['compid'] != 1:
            details += f" BAD_ID(sys={frame['sysid']},comp={frame['compid']})"

        print(f"{elapsed:6.1f}s  {seq:4d}  {msgid:5d}  {name:<22s}  {crc_str:>5s}  {details}")

    port.close()

    # Summary
    print()
    print("=" * 90)
    print("SUMMARY")
    print("=" * 90)
    print(f"Total frames:    {total_frames}")
    print(f"CRC errors:      {crc_errors}")
    print(f"Sequence gaps:   {seq_errors}")
    print(f"Unknown msgids:  {unknown_msgs if unknown_msgs else 'none'}")
    print()

    print("Message counts:")
    for mid in sorted(msg_counts.keys()):
        name = MSG_NAMES.get(mid, f"UNKNOWN({mid})")
        print(f"  {name:<22s} (ID {mid:3d}): {msg_counts[mid]}")

    # Heartbeat interval check
    if len(heartbeat_times) >= 2:
        intervals = [heartbeat_times[i+1] - heartbeat_times[i]
                      for i in range(len(heartbeat_times) - 1)]
        avg_interval = sum(intervals) / len(intervals)
        min_interval = min(intervals)
        max_interval = max(intervals)
        print(f"\nHeartbeat interval: avg={avg_interval:.2f}s min={min_interval:.2f}s max={max_interval:.2f}s")
        if 0.8 <= avg_interval <= 1.2:
            print("  -> 1 Hz heartbeat: PASS")
        else:
            print(f"  -> 1 Hz heartbeat: FAIL (expected ~1.0s, got {avg_interval:.2f}s)")

    # Pass/fail verdict
    print()
    errors = []
    if crc_errors > 0:
        errors.append(f"{crc_errors} CRC errors")
    if seq_errors > 0:
        errors.append(f"{seq_errors} sequence gaps")
    if unknown_msgs:
        errors.append(f"unknown message IDs: {unknown_msgs}")
    if total_frames == 0:
        errors.append("no frames received")
    for mid in [0, 30, 33]:
        if msg_counts.get(mid, 0) == 0:
            errors.append(f"missing {MSG_NAMES[mid]}")

    if errors:
        print(f"VERDICT: FAIL — {'; '.join(errors)}")
    else:
        print("VERDICT: PASS — All MAVLink v2 frames valid")


if __name__ == '__main__':
    main()
