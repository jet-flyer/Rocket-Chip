#!/usr/bin/env python3
"""Host test: scripts/decode_flight_log.py against the current frame format.

No hardware. Checks the decoder's frame constants and TelemetryState layout
against the C++ headers (so a struct change breaks this test, not a flight
download), CRC-16 against the standard vector, a standard-frame round trip,
an event frame zero-padded into a 61-byte slot (pcm_pack_event_slot), a mixed
stream, and the RCEND / RCERR footer split.
"""
from __future__ import annotations

import re
import struct
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import decode_flight_log as dfl  # noqa: E402

FAILS = 0


def check(name: str, ok: bool, detail: object = '') -> None:
    global FAILS
    print(f'  [{"PASS" if ok else "FAIL"}] {name}')
    if not ok:
        FAILS += 1
        if detail:
            print('    ' + str(detail).replace('\n', '\n    '))


def header_text(rel: str) -> str:
    return (ROOT / rel).read_text(encoding='utf-8')


def cpp_constant(text: str, name: str) -> int:
    m = re.search(rf'\b{name}\s*=\s*(0x[0-9A-Fa-f]+|\d+)U?\s*;', text)
    if m is None:
        raise AssertionError(f'{name} not found')
    return int(m.group(1), 0)


# C++ fixed-width type -> struct format char
CPP_TO_STRUCT = {
    'int8_t': 'b', 'uint8_t': 'B',
    'int16_t': 'h', 'uint16_t': 'H',
    'int32_t': 'i', 'uint32_t': 'I',
}


def expand_format(fmt: str) -> str:
    """'<4h 3i' -> '<hhhhiii' so two struct formats compare field by field."""
    return re.sub(r'(\d+)([A-Za-z])', lambda m: m.group(2) * int(m.group(1)),
                  fmt.replace(' ', ''))


def telemetry_fields() -> list[tuple[str, str]]:
    text = header_text('include/rocketchip/telemetry_state.h')
    body = re.search(r'struct\s+__attribute__\(\(packed\)\)\s+TelemetryState\s*\{(.*?)\};',
                     text, re.S)
    if body is None:
        raise AssertionError('TelemetryState struct not found')
    fields = []
    for line in body.group(1).splitlines():
        code = line.split('//', 1)[0]
        m = re.match(r'\s*(u?int\d+_t)\s+(\w+)\s*;', code)
        if m:
            fields.append((m.group(1), m.group(2)))
    return fields


def test_constants_match_headers() -> None:
    pcm = header_text('include/rocketchip/pcm_frame.h')
    tel = header_text('include/rocketchip/telemetry_state.h')
    check('FRAME_SIZE == kPcmFrameStandardSize',
          dfl.FRAME_SIZE == cpp_constant(pcm, 'kPcmFrameStandardSize'))
    check('PAYLOAD_SIZE == kPcmStandardPayloadLen',
          dfl.PAYLOAD_SIZE == cpp_constant(pcm, 'kPcmStandardPayloadLen'))
    check('EVENT_PAYLOAD_SIZE == kPcmEventPayloadLen',
          dfl.EVENT_PAYLOAD_SIZE == cpp_constant(pcm, 'kPcmEventPayloadLen'))
    check('EVENT_FRAME_SIZE == kPcmFrameEventSize',
          dfl.EVENT_FRAME_SIZE == cpp_constant(pcm, 'kPcmFrameEventSize'))
    check('FRAME_TYPE_STANDARD == kPcmFrameTypeStandard',
          dfl.FRAME_TYPE_STANDARD == cpp_constant(pcm, 'kPcmFrameTypeStandard'))
    check('FRAME_TYPE_EVENT == kPcmFrameTypeEvent',
          dfl.FRAME_TYPE_EVENT == cpp_constant(pcm, 'kPcmFrameTypeEvent'))
    check('sync bytes match',
          (dfl.SYNC_HIGH, dfl.SYNC_LOW) ==
          (cpp_constant(pcm, 'kPcmSyncHigh'), cpp_constant(pcm, 'kPcmSyncLow')))
    m = re.search(r'static_assert\(sizeof\(TelemetryState\)\s*==\s*(\d+)', tel)
    check('PAYLOAD_SIZE == static_assert sizeof(TelemetryState)',
          m is not None and int(m.group(1)) == dfl.PAYLOAD_SIZE)
    m = re.search(r'static_assert\(sizeof\(PcmFrameStandard\)\s*==\s*(\w+)', pcm)
    check('PcmFrameStandard static_assert uses kPcmFrameStandardSize',
          m is not None and m.group(1) == 'kPcmFrameStandardSize')
    for name, bit in (('kFlagsZuptActive', dfl.FLAG_ZUPT_ACTIVE),
                      ('kFlagsMetMission', dfl.FLAG_MET_MISSION),
                      ('kFlagsUtcValid', dfl.FLAG_UTC_VALID)):
        m = re.search(rf'\b{name}\s*=\s*\(1U\s*<<\s*(\d+)\)', tel)
        check(f'{name} bit matches', m is not None and (1 << int(m.group(1))) == bit)


def test_telemetry_layout_matches_header() -> None:
    fields = telemetry_fields()
    names = [n for _, n in fields]
    fmt = '<' + ''.join(CPP_TO_STRUCT[t] for t, _ in fields)
    check('TelemetryState field names and order', names == dfl.TELEM_FIELDS,
          f'header={names}\ndecoder={dfl.TELEM_FIELDS}')
    check('TelemetryState field types', fmt == expand_format(dfl.TELEM_FORMAT),
          f'header={fmt} decoder={dfl.TELEM_FORMAT}')
    check('TelemetryState size', struct.calcsize(fmt) == dfl.PAYLOAD_SIZE)


def test_event_names_match_enum() -> None:
    pcm = header_text('include/rocketchip/pcm_frame.h')
    body = re.search(r'enum class LogEventId[^{]*\{(.*?)\};', pcm, re.S)
    ids = sorted(int(v) for v in re.findall(r'=\s*(\d+)\s*,', body.group(1))) if body else []
    check('EVENT_NAMES covers every LogEventId', ids == sorted(dfl.EVENT_NAMES),
          f'header={ids} decoder={sorted(dfl.EVENT_NAMES)}')


def test_crc16_vector() -> None:
    check('CRC-16-CCITT("123456789") == 0x29B1',
          dfl.crc16_ccitt(b'123456789') == 0x29B1)


def make_standard(frame_met_ms: int, **overrides) -> bytes:
    values = {name: 0 for name in dfl.TELEM_FIELDS}
    values.update(overrides)
    payload = struct.pack(dfl.TELEM_FORMAT, *(values[n] for n in dfl.TELEM_FIELDS))
    head = struct.pack('<BBIBB', dfl.SYNC_HIGH, dfl.SYNC_LOW, frame_met_ms,
                       dfl.FRAME_TYPE_STANDARD, dfl.PAYLOAD_SIZE)
    body = head + payload
    return body + struct.pack('<H', dfl.crc16_ccitt(body))


def make_event_slot(met_ms: int, event_id: int, data: tuple[int, int, int, int]) -> bytes:
    head = struct.pack('<BBIBB', dfl.SYNC_HIGH, dfl.SYNC_LOW, met_ms,
                       dfl.FRAME_TYPE_EVENT, dfl.EVENT_PAYLOAD_SIZE)
    body = head + struct.pack(dfl.EVENT_FORMAT, event_id, *data)
    frame = body + struct.pack('<H', dfl.crc16_ccitt(body))
    return frame + bytes(dfl.FRAME_SIZE - len(frame))   # pcm_pack_event_slot zero pad


def test_standard_round_trip() -> None:
    frame = make_standard(123456, q_w=32767, lat_1e7=-123456789, baro_alt_mm=1500,
                          health=0b10_01_11_00, temperature_c=-5, met_ms=123400,
                          flags=dfl.FLAG_ZUPT_ACTIVE | dfl.FLAG_UTC_VALID,
                          utc_year=26, utc_month=11, utc_day=21)
    check('standard frame is 61 bytes', len(frame) == 61)
    r = dfl.decode_frame(frame)
    check('standard frame decodes', r is not None)
    if r is None:
        return
    check('header MET', r['frame_met_ms'] == 123456 and abs(r['met_s'] - 123.456) < 1e-9)
    check('payload fields', r['q_w'] == 32767 and r['lat_1e7'] == -123456789 and
          r['baro_alt_mm'] == 1500 and r['temperature_c'] == -5 and r['met_ms'] == 123400)
    check('health split', (r['health_imu'], r['health_baro'], r['health_eskf'],
                           r['health_gps']) == (0, 3, 1, 2))
    check('flags split', r['zupt_active'] and r['utc_valid'] and not r['met_mission'])
    check('utc date', (r['utc_year'], r['utc_month'], r['utc_day']) == (26, 11, 21))
    bad = bytearray(frame)
    bad[20] ^= 0x01
    check('CRC mismatch rejected', dfl.decode_frame(bytes(bad)) is None)
    check('old 55-byte frame rejected', dfl.decode_frame(frame[:55]) is None)


def test_event_slot() -> None:
    slot = make_event_slot(7000, 6, (1, 2, 0, 0))
    check('event slot is 61 bytes', len(slot) == 61)
    check('event slot is not a standard frame', dfl.decode_frame(slot) is None)
    e = dfl.decode_event(slot)
    check('event decodes', e is not None and e['event'] == 'phase_change' and
          e['data'] == (1, 2, 0, 0) and e['met_ms'] == 7000, e)


def test_mixed_stream() -> None:
    frames = [make_standard(100 * i, met_ms=100 * i) for i in range(5)]
    stream = (frames[0] + frames[1] + make_event_slot(150, 1, (0, 0, 0, 0)) +
              frames[2] + b'\xff' * 61 + frames[3] + frames[4])
    rows, events, corrupt = dfl.decode_frames(stream)
    check('mixed stream counts', (len(rows), len(events), corrupt) == (5, 1, 1),
          (len(rows), len(events), corrupt))
    check('mixed stream order', [r['frame_met_ms'] for r in rows] == [0, 100, 200, 300, 400])


def test_split_download() -> None:
    frames = make_standard(1) + make_standard(2)
    total = len(frames)
    crc = dfl.crc32(frames)
    raw, status, footer = dfl.split_download(frames + f'RCEND:{crc:08X}\r\n'.encode(), total, 61)
    check('RCEND split', status == 'end' and raw == frames and footer == f'RCEND:{crc:08X}')
    # Device handed 1 whole frame plus 10 bytes of the next run, then gave up.
    # stdio may send the line break as CRLF.
    sent = frames[:61 + 10]
    raw, status, footer = dfl.split_download(sent + b'\r\nRCERR:STALL:1\r\n', total, 61)
    check('RCERR split keeps frames_sent whole frames',
          status == 'error' and raw == frames[:61] and footer == 'RCERR:STALL:1',
          (status, len(raw), footer))
    raw, status, _ = dfl.split_download(frames, total, 61)
    check('missing footer', status == 'missing' and raw == frames)


def main() -> int:
    print('decode_flight_log host test')
    test_constants_match_headers()
    test_telemetry_layout_matches_header()
    test_event_names_match_enum()
    test_crc16_vector()
    test_standard_round_trip()
    test_event_slot()
    test_mixed_stream()
    test_split_download()
    print('FAIL' if FAILS else 'PASS', f'({FAILS} failures)')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
