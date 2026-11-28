#!/usr/bin/env python3
"""LoRa two-board baseline. Writes only under this directory.

Does not flash. Does not reboot the station. Vehicle CLI 'k' runs once,
and only from the debug menu (q then k).
"""
from __future__ import annotations

import datetime as dt
import math
import os
import re
import statistics
import sys
import threading
import time

import serial

HERE = os.path.dirname(os.path.abspath(__file__))
VEH_PORT = "COM5"
STN_PORT = "COM7"
SOAK_S = 300.0
RECOVERY_TIMEOUT_S = 90.0
NAV_USER_BYTES = 51  # kNavSduUserBytes; firmware static_assert
NAV_PLTU_BYTES = 69  # kRadioConfigNavPltuBytes
REPORT_PLTU_BYTES = 14
PREAMBLE_SYMBOLS = 8
FREQ_HZ = 915_000_000
SEQ_MOD = 16384  # 14-bit space-packet sequence

ANSI_RE = re.compile(
    r"\x1b\[[?0-9;]*[A-Za-z]|\x1b\].*?(?:\x07|\x1b\\)|\x1b[()][0-9A-Za-z]"
)


def now_local() -> dt.datetime:
    return dt.datetime.now().astimezone()


def ts_text(t: dt.datetime | None = None) -> str:
    t = t or now_local()
    return t.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]


def open_cdc(port: str) -> serial.Serial:
    """DTR low across open so pyserial does not pulse the board into bootrom."""
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 115200
    ser.timeout = 0.2
    ser.write_timeout = 1.0
    ser.dsrdtr = False
    ser.rtscts = False
    ser.dtr = False
    ser.rts = False
    ser.open()
    ser.dtr = True
    time.sleep(0.3)
    return ser


class LineLog:
    def __init__(self, name: str, ser: serial.Serial, path: str) -> None:
        self.name = name
        self.ser = ser
        self.path = path
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._fh = open(path, "w", encoding="utf-8", newline="\n")
        self._buf = bytearray()
        self._text_parts: list[str] = []
        self._thr = threading.Thread(target=self._run, name=f"log-{name}", daemon=True)
        self.dead = False

    def start(self) -> None:
        self._thr.start()

    def _emit(self, line: str) -> None:
        stamped = f"[{ts_text()}] {line}\n"
        self._text_parts.append(stamped)
        self._fh.write(stamped)
        self._fh.flush()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(4096)
            except serial.SerialException as exc:
                with self._lock:
                    self._emit(f"SERIAL_ERR {exc}")
                    self.dead = True
                break
            if not chunk:
                continue
            with self._lock:
                self._buf.extend(chunk)
                while True:
                    nl = self._buf.find(b"\n")
                    if nl < 0:
                        break
                    raw = bytes(self._buf[:nl])
                    del self._buf[: nl + 1]
                    if raw.endswith(b"\r"):
                        raw = raw[:-1]
                    text = raw.decode("utf-8", errors="replace")
                    self._emit(text)

    def note(self, text: str) -> None:
        with self._lock:
            self._emit(text)

    def send(self, data: bytes) -> None:
        self.note(f">>> {data!r}")
        self.ser.write(data)
        self.ser.flush()

    def text(self) -> str:
        with self._lock:
            return "".join(self._text_parts)

    def stop(self) -> None:
        self._stop.set()
        self._thr.join(timeout=2.0)
        with self._lock:
            if self._buf:
                text = bytes(self._buf).decode("utf-8", errors="replace")
                self._buf.clear()
                self._emit(text)
            try:
                self._fh.close()
            except OSError:
                pass


def strip_ansi(text: str) -> str:
    return ANSI_RE.sub("", text)


def between(text: str, start: str, end: str) -> str:
    i = text.find(start)
    if i < 0:
        return ""
    j = text.find(end, i + len(start))
    if j < 0:
        return text[i:]
    return text[i:j]


def all_ints(text: str, pattern: str) -> list[int]:
    return [int(m.group(1)) for m in re.finditer(pattern, text)]


def last_match(text: str, pattern: str) -> re.Match[str] | None:
    found = None
    for m in re.finditer(pattern, text):
        found = m
    return found


def first_match(text: str, pattern: str) -> re.Match[str] | None:
    return re.search(pattern, text)


def airtime_us(sf: int, bw_khz: int, payload: int, cr: int) -> int:
    """SX1276 §4.1.1.6 integer form used by rfm95w_airtime_us.

    coding_rate on the CLI is 5..8 and is the (CR+4) multiplier.
    Preamble is 8 symbols. That preamble length is not read back.
    """
    if bw_khz <= 0:
        bw_khz = 125
    t_sym = (1 << sf) * 1000 // bw_khz
    t_pre = (t_sym * 49) // 4
    num = 8 * payload - 4 * sf + 44
    den = 4 * sf
    chunks = 0 if num <= 0 else (num + den - 1) // den
    n_pay = 8 + chunks * cr
    return t_pre + n_pay * t_sym


def parse_rate_blocks(text: str) -> list[dict[str, int]]:
    blocks = []
    parts = text.split("RATE window_ms=")
    for part in parts[1:]:
        window = re.match(r"(\d+)", part)
        if window is None:
            continue
        block: dict[str, int] = {"window_ms": int(window.group(1))}
        for key in (
            "nav_submit",
            "pltu_post",
            "tx_start",
            "tx_done",
            "tx_busy_drop",
            "tx_hold_replace",
            "rx_crc_ok",
            "rx_crc_fail",
            "station_tx",
        ):
            m = re.search(rf"\b{key}=(\d+)", part)
            if m:
                block[key] = int(m.group(1))
        # Stop at the next prompt-ish so a partial dump is still kept.
        blocks.append(block)
    return blocks


LINK_RE = re.compile(
    r"\bLINK delivered=(\d+) missing=(\d+) crc_fail=(\d+) "
    r"copp_rtx=(\d+) air_ms=(\d+) air_bytes=(\d+) copp_sync_loss=(\d+)"
)


def parse_link_lines(text: str) -> list[dict[str, int]]:
    """One dict per CLI `t` LINK line. Keys match the firmware line."""
    rows = []
    for m in LINK_RE.finditer(text):
        rows.append(
            {
                "delivered": int(m.group(1)),
                "missing": int(m.group(2)),
                "crc_fail": int(m.group(3)),
                "copp_rtx": int(m.group(4)),
                "air_ms": int(m.group(5)),
                "air_bytes": int(m.group(6)),
                "copp_sync_loss": int(m.group(7)),
            }
        )
    return rows


def link_delta(rows: list[dict[str, int]], key: str) -> int | None:
    if len(rows) < 2 or key not in rows[0] or key not in rows[-1]:
        return None
    return rows[-1][key] - rows[0][key]


def parse_cfg(text: str) -> dict[str, int] | None:
    m = last_match(text, r"CFG: BW=(\d+) SF=(\d+) CR=(\d+) nav=(\d+)Hz pwr=(\d+)dBm")
    if m is None:
        return None
    return {
        "bw": int(m.group(1)),
        "sf": int(m.group(2)),
        "cr": int(m.group(3)),
        "nav": int(m.group(4)),
        "pwr": int(m.group(5)),
    }


def parse_pa(text: str) -> str:
    m = last_match(text, r"PA: ([^\n]+)")
    return m.group(1).strip() if m else "not seen"


def parse_identity(text: str) -> dict[str, str]:
    out: dict[str, str] = {}
    m = last_match(text, r"RocketChip\s+(\S+)\s+(\S+)-([0-9a-fA-F]+)")
    if m:
        out["version"] = m.group(1)
        out["build"] = m.group(2)
        out["git"] = m.group(3)
    m = last_match(text, r"ID: flight-([0-9a-fA-F]+) tree=(\S+)")
    if m:
        out["flight"] = m.group(1)
        out["tree"] = m.group(2)
    elif "git" in out:
        out["flight"] = out["git"]
    m = last_match(text, r"Board: ([^\n]+)")
    if m:
        out["board"] = m.group(1).strip()
    m = last_match(text, r"Air: (\S+)")
    if m:
        out["air"] = m.group(1)
    m = last_match(text, r"TX_POWER\s+locked\s+(\d+)dBm")
    if m:
        out["tx_power_locked"] = m.group(1)
    return out


def dashboard_samples(text: str) -> list[dict[str, int]]:
    """One sample per dashboard Seq line, with the RSSI/Pkts above it."""
    samples = []
    rssi = None
    snr = None
    pkts = None
    crc = None
    stamp = None
    for line in text.splitlines():
        tm = re.match(r"\[([^\]]+)\]\s*(.*)$", line)
        if not tm:
            continue
        body = strip_ansi(tm.group(2))
        stamp = tm.group(1)
        rm = re.search(r"RSSI:\s*(-?\d+)\s*dBm", body)
        if rm:
            rssi = int(rm.group(1))
        sm = re.search(r"SNR:\s*(-?\d+)\s*dB", body)
        if sm:
            snr = int(sm.group(1))
        pm = re.search(r"Pkts:\s*(\d+)\s+CRC:\s*(\d+)", body)
        if pm:
            pkts = int(pm.group(1))
            crc = int(pm.group(2))
        qm = re.search(r"Seq:\s*(\d+)", body)
        if qm and rssi is not None:
            samples.append(
                {
                    "seq": int(qm.group(1)),
                    "rssi": rssi,
                    "snr": snr if snr is not None else 0,
                    "snr_ok": 1 if snr is not None else 0,
                    "pkts": pkts if pkts is not None else -1,
                    "crc": crc if crc is not None else -1,
                    "t": stamp,
                }
            )
    return samples


def parse_stamp(text: str) -> dt.datetime:
    return dt.datetime.strptime(text, "%Y-%m-%dT%H:%M:%S.%f").astimezone()


def seq_gap(prev: int, cur: int) -> int:
    d = (cur - prev) % SEQ_MOD
    if d == 0:
        return 0
    return d - 1


def longest_miss(seqs: list[int]) -> tuple[int, int, int]:
    """Return longest missed run, compared steps, and total missed numbers."""
    longest = 0
    steps = 0
    missed = 0
    prev = None
    for s in seqs:
        if prev is None:
            prev = s
            continue
        if s == prev:
            continue
        steps += 1
        gap = seq_gap(prev, s)
        missed += gap
        if gap > longest:
            longest = gap
        prev = s
    return longest, steps, missed


def stats(vals: list[float]) -> dict[str, float] | None:
    if not vals:
        return None
    out = {
        "n": float(len(vals)),
        "mean": statistics.fmean(vals),
        "min": min(vals),
        "max": max(vals),
    }
    if len(vals) >= 2:
        out["stdev"] = statistics.stdev(vals)
    else:
        out["stdev"] = 0.0
    return out


def fmt(v: float, nd: int = 3) -> str:
    if math.isnan(v) or math.isinf(v):
        return "n/a"
    return f"{v:.{nd}f}"


def delta(end: dict[str, int] | None, start: dict[str, int] | None, key: str) -> int | None:
    if end is None or start is None:
        return None
    if key not in end or key not in start:
        return None
    return end[key] - start[key]


def send_gap(log: LineLog, chars: str, gap: float = 0.45) -> None:
    for ch in chars:
        log.send(ch.encode("ascii"))
        time.sleep(gap)


def wait_text(log: LineLog, needle: str, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if needle in log.text():
            return True
        time.sleep(0.1)
    return False


def phase(msg: str) -> None:
    print(msg, flush=True)


def main() -> int:
    os.makedirs(HERE, exist_ok=True)
    veh_path = os.path.join(HERE, "veh_raw.log")
    stn_path = os.path.join(HERE, "stn_raw.log")
    phase("PHASE probe open")
    try:
        veh_ser = open_cdc(VEH_PORT)
    except (serial.SerialException, OSError) as exc:
        phase(f"FAILED vehicle open {exc}")
        return 2
    try:
        stn_ser = open_cdc(STN_PORT)
    except (serial.SerialException, OSError) as exc:
        phase(f"FAILED station open {exc}")
        veh_ser.close()
        return 2

    veh = LineLog("veh", veh_ser, veh_path)
    stn = LineLog("stn", stn_ser, stn_path)
    veh.start()
    stn.start()
    veh.note(">>> PORT_OPEN COM5 dtr-low-then-high")
    stn.note(">>> PORT_OPEN COM7 dtr-low-then-high")

    # Banner follows DTR. Do not send keys until the prompt exists.
    phase("PHASE wait banner")
    time.sleep(2.5)
    wait_text(veh, "[main]", 8.0)
    wait_text(stn, "[main]", 8.0)
    time.sleep(0.4)

    phase("PHASE identity and radio status")
    # Help, radio status, settings catalog, debug HW + diag, back to main.
    # No 'x' (vehicle erase). No 'r' (vehicle rate cycle). No 'n' (preset hop).
    # No 'a' / 'D' / 'b' on main (beacon or arm).
    send_gap(veh, "ht")
    time.sleep(0.8)
    send_gap(veh, "sl")
    time.sleep(0.6)
    send_gap(veh, "z")
    time.sleep(0.3)
    send_gap(veh, "qbd")
    time.sleep(1.2)
    send_gap(veh, "z")
    time.sleep(0.4)

    send_gap(stn, "ht")
    time.sleep(0.8)
    send_gap(stn, "sl")
    time.sleep(0.6)
    send_gap(stn, "z")
    time.sleep(0.3)
    send_gap(stn, "qbd")
    time.sleep(1.2)
    send_gap(stn, "z")
    time.sleep(0.5)

    veh_id_txt = strip_ansi(veh.text())
    stn_id_txt = strip_ansi(stn.text())
    veh_cfg = parse_cfg(veh_id_txt)
    stn_cfg = parse_cfg(stn_id_txt)
    veh_pa = parse_pa(veh_id_txt)
    stn_pa = parse_pa(stn_id_txt)
    phase(f"PHASE cfg veh={veh_cfg} pa={veh_pa} stn={stn_cfg} pa={stn_pa}")
    phase("PHASE power left as flashed (TX_POWER catalog class is locked)")

    # Soak. Station stays on the pad so each RX paints Seq/RSSI.
    # 't' is main-menu radio status. rc_os_update reads it even while the pad paints.
    veh.note(">>> SOAK_START")
    stn.note(">>> SOAK_START")
    phase("PHASE soak 300s")
    t0 = time.monotonic()
    next_poll = 0.0
    while True:
        elapsed = time.monotonic() - t0
        if elapsed >= SOAK_S:
            break
        if elapsed >= next_poll:
            veh.send(b"t")
            time.sleep(0.35)
            stn.send(b"t")
            next_poll += 60.0
            phase(f"PHASE soak t={elapsed:.0f}s")
        time.sleep(0.2)

    veh.send(b"t")
    time.sleep(0.4)
    stn.send(b"t")
    time.sleep(1.6)
    veh.note(">>> SOAK_END")
    stn.note(">>> SOAK_END")
    phase("PHASE soak end diag")

    # End diag while the vehicle is still up. Then one CLI reboot.
    send_gap(veh, "qd", gap=0.6)
    time.sleep(1.0)
    send_gap(stn, "qdz", gap=0.6)
    time.sleep(1.2)

    phase("PHASE vehicle CLI k")
    veh.note(">>> REBOOT_K")
    stn.note(">>> REBOOT_K")
    k_wall = now_local()
    veh.send(b"k")
    k_mono = time.monotonic()

    # COM5 drops across the watchdog reboot. Reopen it to catch the new banner.
    # Do not send another k. Do not touch COM7. Do not truncate veh_raw.log.
    deadline = k_mono + RECOVERY_TIMEOUT_S
    post_stop = threading.Event()
    post_thr: threading.Thread | None = None
    while time.monotonic() < deadline:
        if post_thr is None and veh.dead and (time.monotonic() - k_mono) > 1.0:
            try:
                veh.stop()
            except Exception:
                pass
            try:
                veh_ser.close()
            except Exception:
                pass
            try:
                veh_ser = open_cdc(VEH_PORT)
            except (serial.SerialException, OSError) as exc:
                phase(f"PHASE veh reopen wait {exc}")
                veh_ser = None
                time.sleep(1.0)
            if veh_ser is not None:
                held = veh_ser

                def _post() -> None:
                    buf = bytearray()
                    with open(veh_path, "a", encoding="utf-8", newline="\n") as fh:
                        fh.write(f"[{ts_text()}] >>> VEH_REOPEN after k\n")
                        fh.flush()
                        while not post_stop.is_set():
                            try:
                                chunk = held.read(4096)
                            except serial.SerialException as exc2:
                                fh.write(f"[{ts_text()}] SERIAL_ERR {exc2}\n")
                                fh.flush()
                                break
                            if not chunk:
                                continue
                            buf.extend(chunk)
                            while True:
                                nl = buf.find(b"\n")
                                if nl < 0:
                                    break
                                raw = bytes(buf[:nl])
                                del buf[: nl + 1]
                                if raw.endswith(b"\r"):
                                    raw = raw[:-1]
                                text = raw.decode("utf-8", errors="replace")
                                fh.write(f"[{ts_text()}] {text}\n")
                                fh.flush()

                post_thr = threading.Thread(target=_post, name="veh-post", daemon=True)
                post_thr.start()
                phase("PHASE veh reopened")
        if (time.monotonic() - k_mono) > 15.0 and _station_resumed(stn.text(), k_wall):
            time.sleep(3.0)
            break
        time.sleep(0.25)
    else:
        phase("PHASE recovery timeout")

    post_stop.set()
    if post_thr is not None:
        post_thr.join(timeout=3.0)
    time.sleep(0.3)
    stn.note(">>> CAPTURE_END")
    try:
        if not veh.dead:
            veh.note(">>> CAPTURE_END")
        else:
            with open(veh_path, "a", encoding="utf-8", newline="\n") as fh:
                fh.write(f"[{ts_text()}] >>> CAPTURE_END\n")
    except Exception:
        pass

    phase("PHASE analyze")
    try:
        veh.stop()
    except Exception:
        pass
    stn.stop()
    try:
        stn_ser.close()
    except Exception:
        pass
    try:
        veh_ser.close()
    except Exception:
        pass

    write_summary(veh_path, stn_path)
    phase("DONE baseline")
    return 0


def _station_resumed(text: str, k_wall: dt.datetime) -> bool:
    soak_end_pkts = None
    # Use Pkts after REBOOT_K that exceed the last Pkts before REBOOT_K by > 2
    # and whose stamp is at least 1 s after k.
    pre, _, post = text.partition(">>> REBOOT_K")
    pre_s = dashboard_samples(pre)
    post_s = dashboard_samples(post)
    if not pre_s or not post_s:
        return False
    base = pre_s[-1]["pkts"]
    for s in post_s:
        try:
            t = parse_stamp(s["t"])
        except ValueError:
            continue
        if (t - k_wall).total_seconds() >= 1.0 and s["pkts"] > base + 2:
            return True
    return False


def write_summary(veh_path: str, stn_path: str) -> None:
    veh_raw = open(veh_path, encoding="utf-8", errors="replace").read()
    stn_raw = open(stn_path, encoding="utf-8", errors="replace").read()
    veh = strip_ansi(veh_raw)
    stn = strip_ansi(stn_raw)

    pre_v = between(veh, "PORT_OPEN", "SOAK_START")
    pre_s = between(stn, "PORT_OPEN", "SOAK_START")
    soak_v = between(veh, "SOAK_START", "SOAK_END")
    soak_s = between(stn, "SOAK_START", "SOAK_END")
    # Identity lines can also appear inside soak 't' dumps. Prefer pre-soak,
    # then the whole log.
    vid = parse_identity(pre_v) or parse_identity(veh)
    sid = parse_identity(pre_s) or parse_identity(stn)
    vcfg = parse_cfg(pre_v) or parse_cfg(soak_v) or parse_cfg(veh)
    scfg = parse_cfg(pre_s) or parse_cfg(soak_s) or parse_cfg(stn)
    vpa = parse_pa(pre_v if "PA:" in pre_v else veh)
    spa = parse_pa(pre_s if "PA:" in pre_s else stn)

    v_rates = parse_rate_blocks(soak_v)
    s_rates = parse_rate_blocks(soak_s)
    v_link = parse_link_lines(soak_v)
    s_link = parse_link_lines(soak_s)
    v0 = v_rates[0] if v_rates else None
    v1 = v_rates[-1] if v_rates else None
    s0 = s_rates[0] if s_rates else None
    s1 = s_rates[-1] if s_rates else None

    def tx_line(text: str) -> tuple[int, int] | None:
        m = last_match(text, r"TX: (\d+) sent  (\d+) fail")
        if m is None:
            return None
        return int(m.group(1)), int(m.group(2))

    def rx_line(text: str) -> tuple[int, int, int, int] | None:
        m = last_match(
            text,
            r"RX: (\d+) pkts\s+seq=(\d+)\s+(-?\d+)dBm\s+(-?\d+)dB SNR\s+(\d+) CRC err",
        )
        if m is None:
            return None
        return (
            int(m.group(1)),
            int(m.group(2)),
            int(m.group(5)),
            int(m.group(3)),
        )

    # First and last inside the soak slice.
    def first_tx(text: str) -> tuple[int, int] | None:
        m = first_match(text, r"TX: (\d+) sent  (\d+) fail")
        if m is None:
            return None
        return int(m.group(1)), int(m.group(2))

    def first_rx(text: str) -> tuple[int, int] | None:
        m = first_match(
            text,
            r"RX: (\d+) pkts\s+seq=(\d+)\s+(-?\d+)dBm\s+(-?\d+)dB SNR\s+(\d+) CRC err",
        )
        if m is None:
            return None
        return int(m.group(1)), int(m.group(5))

    tx0 = first_tx(soak_v)
    tx1 = tx_line(soak_v)
    rx0 = first_rx(soak_s)
    rx1 = rx_line(soak_s)

    samples = dashboard_samples(soak_s)
    # Drop idle redraws that repeat the same packet count.
    fresh = []
    last_pkts = None
    for s in samples:
        if s["pkts"] < 0:
            continue
        if last_pkts is None or s["pkts"] != last_pkts:
            fresh.append(s)
            last_pkts = s["pkts"]

    seqs = [s["seq"] for s in fresh]
    longest, seq_steps, missed_seq = longest_miss(seqs)
    rssi_vals = [float(s["rssi"]) for s in fresh]
    snr_vals = [float(s["snr"]) for s in fresh if s["snr_ok"]]
    rssi_st = stats(rssi_vals)
    snr_st = stats(snr_vals)

    # Window from first to last soak timestamp.
    def span_s(text: str) -> float | None:
        stamps = re.findall(r"\[(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d{3})\]", text)
        if len(stamps) < 2:
            return None
        a = parse_stamp(stamps[0])
        b = parse_stamp(stamps[-1])
        return (b - a).total_seconds()

    def boot_span(a: dict[str, int] | None, b: dict[str, int] | None) -> float | None:
        if a is None or b is None:
            return None
        if "window_ms" not in a or "window_ms" not in b:
            return None
        return (b["window_ms"] - a["window_ms"]) / 1000.0

    veh_span = boot_span(v0, v1)
    stn_span = boot_span(s0, s1)
    log_span = span_s(soak_s) or span_s(soak_v) or SOAK_S
    window_s = stn_span or veh_span or log_span

    d_tx = None
    if tx0 and tx1:
        d_tx = tx1[0] - tx0[0]
    d_tx_done = delta(v1, v0, "tx_done")
    d_tx_start = delta(v1, v0, "tx_start")
    d_busy = delta(v1, v0, "tx_busy_drop")
    d_hold = delta(v1, v0, "tx_hold_replace")
    d_nav = delta(v1, v0, "nav_submit")
    d_pltu = delta(v1, v0, "pltu_post")
    d_rx = None
    d_crc = None
    if rx0 and rx1:
        d_rx = rx1[0] - rx0[0]
        d_crc = rx1[2] - rx0[1]
    d_crc_ok = delta(s1, s0, "rx_crc_ok")
    d_crc_fail = delta(s1, s0, "rx_crc_fail")
    d_stn_tx = delta(s1, s0, "station_tx")
    if d_stn_tx is None:
        d_stn_tx = delta(v1, v0, "station_tx")

    # Prefer the explicit TX sent counter. Fall back to tx_done.
    sent = d_tx if d_tx is not None else d_tx_done
    got = d_rx
    loss_rate = None
    missing = None
    if sent is not None and got is not None and sent > 0:
        missing = sent - got
        loss_rate = missing / sent

    sf = (vcfg or scfg or {}).get("sf", 7)
    bw = (vcfg or scfg or {}).get("bw", 125)
    cr = (vcfg or scfg or {}).get("cr", 5)
    nav_hz = (vcfg or scfg or {}).get("nav", 0)
    pwr = (vcfg or {}).get("pwr")
    stn_pwr = (scfg or {}).get("pwr")

    toa_nav_us = airtime_us(sf, bw, NAV_PLTU_BYTES, cr)
    toa_rep_us = airtime_us(sf, bw, REPORT_PLTU_BYTES, cr)
    toa_nav_s = toa_nav_us / 1_000_000.0
    toa_rep_s = toa_rep_us / 1_000_000.0

    delivered_nav = 0
    prev = None
    for s in seqs:
        if prev is None or s != prev:
            if prev is not None:
                delivered_nav += 1
            prev = s
    # delivered_nav counts steps, which is frames after the first.
    if seqs:
        delivered_nav = seq_steps  # each step is one accepted new seq; misses are inside the step
        # Frames observed = steps + 1 if we saw at least one, but steps already
        # skip duplicates. Count unique advances + 1.
        delivered_nav = 1 + seq_steps

    # Bytes/s uses observed nav frames that arrived (not the missing ones).
    observed_nav = 0
    prev = None
    for s in seqs:
        if prev is None or s != prev:
            observed_nav += 1
            prev = s

    payload_Bps = None
    if log_span and observed_nav:
        payload_Bps = observed_nav * NAV_USER_BYTES / log_span

    offered_hz = None
    if d_nav is not None and veh_span:
        offered_hz = d_nav / veh_span
    delivered_hz = None
    if observed_nav and log_span:
        delivered_hz = observed_nav / log_span
    radio_hz = None
    if got is not None and stn_span:
        radio_hz = got / stn_span
    stn_tx_hz = None
    if d_stn_tx is not None and stn_span:
        stn_tx_hz = d_stn_tx / stn_span
    tx_hz = None
    if sent is not None and veh_span:
        tx_hz = sent / veh_span

    duty_veh = None
    if sent is not None and veh_span:
        duty_veh = sent * toa_nav_s / veh_span
    duty_stn = None
    if d_stn_tx is not None and stn_span:
        duty_stn = d_stn_tx * toa_rep_s / stn_span

    air_per_byte_us = toa_nav_us / NAV_USER_BYTES

    # Recovery.
    pre_k, _, post_k = stn.partition(">>> REBOOT_K")
    pre_samples = dashboard_samples(pre_k)
    post_samples = dashboard_samples(post_k)
    k_stamp = None
    km = re.search(r"\[([^\]]+)\] >>> REBOOT_K", stn_raw)
    if km:
        try:
            k_stamp = parse_stamp(strip_ansi(km.group(1)) if False else km.group(1))
        except ValueError:
            k_stamp = None
    # The stamp is on our note line and has no ANSI.
    if k_stamp is None:
        km = re.search(r"\[(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d{3})\] >>> REBOOT_K", stn)
        if km:
            k_stamp = parse_stamp(km.group(1))

    base_pkts = pre_samples[-1]["pkts"] if pre_samples else None
    t_pre = None
    if pre_samples:
        try:
            t_pre = parse_stamp(pre_samples[-1]["t"])
        except ValueError:
            t_pre = None
    new_after = []
    for s in post_samples:
        try:
            t = parse_stamp(s["t"])
        except ValueError:
            continue
        if k_stamp is not None and t < k_stamp:
            continue
        if base_pkts is not None and s["pkts"] <= base_pkts:
            continue
        new_after.append((t, s))
    first_after = new_after[0] if new_after else None
    # In-flight frames can arrive just after k. The link-back sample is the
    # first new frame that follows a silence of at least 1 s.
    resume_after_gap = None
    prev_t = t_pre
    prev_pkts = base_pkts
    for t, s in new_after:
        # The pad repaints the last packet at 1 Hz while the link is down.
        if prev_pkts is not None and s["pkts"] <= prev_pkts:
            continue
        if prev_t is not None and (t - prev_t).total_seconds() >= 1.0:
            resume_after_gap = (t, s)
            break
        prev_t = t
        prev_pkts = s["pkts"]
    if resume_after_gap is None:
        resume_after_gap = first_after

    def rel(t: dt.datetime | None) -> str:
        if t is None or k_stamp is None:
            return "not measured"
        return f"{(t - k_stamp).total_seconds():.3f} s (board-measured)"

    recovery = rel(resume_after_gap[0] if resume_after_gap else (first_after[0] if first_after else None))
    first_rx_after_k = rel(first_after[0] if first_after else None)

    # Command path evidence in the whole station log.
    acks = re.findall(r"ACK\s+(\d+)ms", stn)
    tries = re.findall(r"Try\s+(\d+)/(\d+)", stn)
    cmd_rows = re.findall(r"\[CMD\][^\n]+", stn)
    retry_block = re.findall(r"\[CmdRetryStats\][\s\S]{0,800}?={5,}|\[CmdRetryStats\][\s\S]{0,500}", stn)

    # Board cost: queue high water and msp. Not IRQ time.
    msp = last_match(stn, r"\[MSP\] initial=(\S+) min=(\S+) depth=(\d+) bytes")
    radio_q = last_match(stn, r"AO_Radio\s+depth=(\d+) use=(\d+) high=(\d+)")
    veh_msp = last_match(veh, r"\[MSP\] initial=(\S+) min=(\S+) depth=(\d+) bytes")
    veh_q = last_match(veh, r"AO_Radio\s+depth=(\d+) use=(\d+) high=(\d+)")
    dropped = last_match(stn, r"\[RcLog\] dropped=(\d+) bytes  high_water=(\d+) bytes")
    veh_dropped = last_match(veh, r"\[RcLog\] dropped=(\d+) bytes  high_water=(\d+) bytes")

    # COP-P
    copp_v = last_match(soak_v, r"COP-P: ([^\n]+)")
    copp_s = last_match(soak_s, r"COP-P:?\s+((?:lock|waiting)[^\n]*)")

    # Counters at end for the fail field (consecutive, not a delta).
    fail_now = tx1[1] if tx1 else None

    flight = vid.get("flight", "not seen")
    stn_flight = sid.get("flight", "not seen")
    date = dt.date.today().isoformat()

    def nlabel(v: int | None, kind: str) -> str:
        if v is None:
            return "not exposed by firmware" if kind == "missing" else "not measured"
        return f"{v} ({kind})"

    lines = []
    a = lines.append
    a("# LoRa link baseline")
    a("")
    if flight == "3e9c38e2" and stn_flight == "3e9c38e2":
        a("Measured build flight-3e9c38e2 (identical to main 47005521).")
    else:
        a(f"Measured build flight-{flight} on the vehicle and flight-{stn_flight} on the station.")
        a("The expected build was flight-3e9c38e2 (identical to main 47005521).")
    a(f"Date: {date}.")
    a(f"Vehicle flight id: {flight} ({'board-measured' if flight != 'not seen' else 'not measured'}).")
    a(f"Vehicle tree: {vid.get('tree', 'not seen')} (board-measured).")
    a(f"Vehicle banner: version {vid.get('version', '?')}, build {vid.get('build', '?')}, board {vid.get('board', '?')}, air {vid.get('air', '?')} (board-measured).")
    a(f"Station flight id: {stn_flight} (board-measured).")
    a(f"Station tree: {sid.get('tree', 'not seen')} (board-measured).")
    a(f"Station banner: version {sid.get('version', '?')}, build {sid.get('build', '?')}, board {sid.get('board', '?')}, air {sid.get('air', '?')} (board-measured).")
    a("")
    a("LoRa parameters:")
    a(f"- SF {sf} (board-measured, CFG line).")
    a(f"- BW {bw} kHz (board-measured, CFG line).")
    a(f"- CR {cr} (board-measured, CFG line; 5 means 4/5).")
    a(f"- Nav rate {nav_hz} Hz (board-measured, CFG line).")
    a(f"- Station CFG nav {scfg.get('nav') if scfg else 'not seen'} Hz, SF {scfg.get('sf') if scfg else 'not seen'}, BW {scfg.get('bw') if scfg else 'not seen'} kHz, CR {scfg.get('cr') if scfg else 'not seen'} (board-measured).")
    a(f"- Frequency {FREQ_HZ} Hz (not exposed by a live register read; firmware init constant kDefaultFreqHz).")
    a(f"- Preamble {PREAMBLE_SYMBOLS} symbols (not exposed by a live register read; firmware init writes RegPreambleLsb 0x08).")
    a(f"- Nav on-air size {NAV_PLTU_BYTES} bytes (not exposed by firmware counters; source kRadioConfigNavPltuBytes).")
    a(f"- Useful nav payload {NAV_USER_BYTES} bytes (not exposed by firmware counters; source kNavSduUserBytes).")
    a(f"- Station report PLTU {REPORT_PLTU_BYTES} bytes (not exposed by firmware counters; source byte_pump kReportPltuBytes).")
    a("")
    a("Power:")
    a(f"- Vehicle CFG pwr {pwr} dBm (board-measured).")
    a(f"- Vehicle PA register: {vpa} (board-measured).")
    a(f"- Station CFG pwr {stn_pwr} dBm (board-measured).")
    a(f"- Station PA register: {spa} (board-measured).")
    a("- TX_POWER catalog class is locked. No runtime CLI sets power alone. Power was not changed.")
    a("- The debug HW line prints a fixed '20dBm' string. That string is not the PA register.")
    a("")
    a("Distance: unknown. No bench note in this run states the spacing.")
    a("")
    a(f"Vehicle counter span: {fmt(veh_span, 3) if veh_span else 'not measured'} s (board-measured, RATE window_ms end minus start).")
    a(f"Station counter span: {fmt(stn_span, 3) if stn_span else 'not measured'} s (board-measured, RATE window_ms end minus start).")
    a(f"Pad log span: {fmt(log_span, 1)} s (board-measured, SOAK_START to SOAK_END).")
    a(f"RATE dumps in that window: vehicle {len(v_rates)}, station {len(s_rates)} (board-measured).")
    a("")
    a("## Frame delivery")
    a("")
    a(f"Vehicle TX sent delta: {nlabel(sent, 'board-measured')}.")
    a("The TX sent field is tx_count. It increments on TX done.")
    a(f"Vehicle tx_start delta: {nlabel(d_tx_start, 'board-measured')}.")
    a(f"Vehicle tx_done delta: {nlabel(d_tx_done, 'board-measured')}.")
    a(f"Station RX pkts delta: {nlabel(got, 'board-measured')}.")
    a("Station rx_count increments on a non-empty radio packet. A LoRa CRC fail with an empty FIFO does not increment it.")
    a(f"Station CRC err delta from the RX status line: {nlabel(d_crc, 'board-measured')}.")
    a(f"Station RATE rx_crc_ok delta: {nlabel(d_crc_ok, 'board-measured')}.")
    a(f"Station RATE rx_crc_fail delta: {nlabel(d_crc_fail, 'board-measured')}.")
    a(f"Vehicle LINK delivered delta: {nlabel(link_delta(v_link, 'delivered'), 'board-measured')}.")
    a(f"Vehicle LINK missing delta: {nlabel(link_delta(v_link, 'missing'), 'board-measured')}.")
    a(f"Vehicle LINK crc_fail delta: {nlabel(link_delta(v_link, 'crc_fail'), 'board-measured')}.")
    a(f"Station LINK delivered delta: {nlabel(link_delta(s_link, 'delivered'), 'board-measured')}.")
    a(f"Station LINK missing delta: {nlabel(link_delta(s_link, 'missing'), 'board-measured')}.")
    a(f"Station LINK crc_fail delta: {nlabel(link_delta(s_link, 'crc_fail'), 'board-measured')}.")
    a("LINK is one line from CLI t. missing is the 14-bit sequence gap on each APID.")
    a("A repeat adds no missing count. The first count on an APID opens that lane.")
    a("On this build the station RATE rx_crc_ok counter moved with RX pkts. The CLI CRC err field is a different counter. Both are reported.")
    a("The status-line seq field stayed 0. Nav sequence used for gaps is the pad Seq field.")
    if loss_rate is None:
        a("Frame loss rate: not measured.")
    else:
        a(f"Missing radio frames (TX sent minus station RX): {missing} (computed).")
        a(f"Frame loss rate: {fmt(loss_rate * 100, 3)} percent (computed).")
    a(f"Dashboard frames with a new Pkts value: {len(fresh)} (board-measured).")
    a(f"Seq steps compared: {seq_steps} (board-measured).")
    a(f"Longest run of missed seq numbers: {longest} (computed from dashboard Seq, modulo {SEQ_MOD}).")
    a(f"Total missed seq numbers in that series: {missed_seq} (computed).")
    a("A missed run is the gap between successive distinct Seq values, minus one.")
    a("Same-seq paints with a new Pkts value are extra radio frames. They are not nav-seq gaps.")
    a("Idle pad redraws that repeat Pkts are not extra frames.")
    if copp_v:
        a(f"Vehicle COP-P at last soak line: {copp_v.group(1).strip()} (board-measured).")
    else:
        a("Vehicle COP-P line: not seen in the soak slice.")
    if copp_s:
        a(f"Station COP-P at last soak line: {copp_s.group(1).strip()} (board-measured).")
    else:
        a("Station COP-P line: not seen in the soak slice.")
    a("")
    a("## TX faults")
    a("")
    a(f"tx_busy_drop delta: {nlabel(d_busy, 'board-measured')}.")
    a(f"tx_hold_replace delta: {nlabel(d_hold, 'board-measured')}.")
    a(f"TX fail field at the last soak status: {nlabel(fail_now, 'board-measured')}.")
    a("That fail field is tx_consec_fail. It is a consecutive count. It is not a cumulative fault total.")
    a("A cumulative TX-timeout counter is not exposed by firmware.")
    if d_tx_start is not None and d_tx_done is not None:
        a(f"tx_start minus tx_done over the window: {d_tx_start - d_tx_done} (computed).")
        a("A positive value is in-flight work or a start that did not finish. It is not a named fault counter.")
    a("")
    a("## Signal")
    a("")
    if rssi_st is None:
        a("RSSI: not measured. The pad did not print RSSI during the soak.")
    else:
        a(f"RSSI n={int(rssi_st['n'])} mean={fmt(rssi_st['mean'], 2)} dBm min={fmt(rssi_st['min'], 0)} dBm max={fmt(rssi_st['max'], 0)} dBm sample stdev={fmt(rssi_st['stdev'], 2)} dB (board-measured samples, computed stats).")
        a("Each sample is one pad paint with a new Pkts value. The firmware prints RSSI as an integer dBm.")
    if snr_st is None:
        a("SNR: not measured.")
    else:
        a(f"SNR n={int(snr_st['n'])} mean={fmt(snr_st['mean'], 2)} dB min={fmt(snr_st['min'], 0)} dB max={fmt(snr_st['max'], 0)} dB sample stdev={fmt(snr_st['stdev'], 2)} dB (board-measured samples, computed stats).")
    a("")
    a("## Air time per delivered payload byte")
    a("")
    a(f"Nav on-air time: {toa_nav_us} us (computed, Semtech SX1276 §4.1.1.6, same integer form as rfm95w_airtime_us).")
    a(f"Vehicle LINK air_ms delta: {nlabel(link_delta(v_link, 'air_ms'), 'board-measured')}.")
    a(f"Vehicle LINK air_bytes delta: {nlabel(link_delta(v_link, 'air_bytes'), 'board-measured')}.")
    a(f"Station LINK air_ms delta: {nlabel(link_delta(s_link, 'air_ms'), 'board-measured')}.")
    a(f"Station LINK air_bytes delta: {nlabel(link_delta(s_link, 'air_bytes'), 'board-measured')}.")
    a("LINK air_ms sums rfm95w_airtime_us for each TX that finished.")
    a("LINK air_bytes is the modem payload length of those frames.")
    a("A TX timeout does not add. The airtime function assumes coding rate 4/5.")
    a("TX-start and TX-done are still counts. They are not timestamps.")
    a(f"Useful bytes per nav frame: {NAV_USER_BYTES} (firmware constant, not a live counter).")
    a(f"Air time per delivered payload byte: {fmt(air_per_byte_us, 2)} us/byte (computed).")
    a("The quotient uses whole nav on-air time, including preamble, header, and CRC, divided by the 51-byte nav user payload.")
    a(f"Station report on-air time: {toa_rep_us} us for {REPORT_PLTU_BYTES} bytes (computed, same formula).")
    a("")
    a("## Throughput and rates")
    a("")
    a(f"Time on air per nav frame: {fmt(toa_nav_s * 1000, 3)} ms (computed).")
    a(f"Time on air per station report frame: {fmt(toa_rep_s * 1000, 3)} ms (computed).")
    if payload_Bps is None:
        a("Payload bytes/s: not measured.")
    else:
        a(f"Payload bytes/s: {fmt(payload_Bps, 2)} (computed).")
        a(f"That rate is observed distinct Seq paints times {NAV_USER_BYTES} bytes, divided by the pad log span.")
    if offered_hz is None:
        a(f"Offered telemetry rate from nav_submit: not measured. Configured nav is {nav_hz} Hz (board-measured).")
    else:
        a(f"Offered telemetry rate from nav_submit delta: {fmt(offered_hz, 3)} Hz (computed over the vehicle counter span).")
        a(f"Configured nav: {nav_hz} Hz (board-measured).")
    a(f"nav_submit delta: {nlabel(d_nav, 'board-measured')}.")
    a(f"pltu_post delta: {nlabel(d_pltu, 'board-measured')}.")
    if delivered_hz is None:
        a("Delivered telemetry rate from Seq paints: not measured.")
    else:
        a(f"Delivered telemetry rate from distinct Seq paints: {fmt(delivered_hz, 3)} Hz (computed).")
    a(f"Vehicle TX rate: {fmt(tx_hz, 3) if tx_hz is not None else 'not measured'} Hz (computed from TX sent over the vehicle counter span).")
    if radio_hz is None:
        a("Delivered radio-packet rate from station RX delta: not measured.")
    else:
        a(f"Delivered radio-packet rate from station RX delta: {fmt(radio_hz, 3)} Hz (computed).")
    if stn_tx_hz is None:
        a("Station TX rate: not measured.")
    else:
        a(f"Station TX rate: {fmt(stn_tx_hz, 3)} Hz (computed from station_tx delta).")
    a(f"station_tx delta: {nlabel(d_stn_tx, 'board-measured')}.")
    veh_rx_hits = re.findall(r"RX: (\d+) pkts  (\d+) CRC err", soak_v)
    if len(veh_rx_hits) >= 2:
        vrx0 = int(veh_rx_hits[0][0])
        vrx1 = int(veh_rx_hits[-1][0])
        vcr0 = int(veh_rx_hits[0][1])
        vcr1 = int(veh_rx_hits[-1][1])
        a(f"Vehicle RX pkts delta: {vrx1 - vrx0} (board-measured).")
        a(f"Vehicle RX CRC err delta: {vcr1 - vcr0} (board-measured).")
        if d_stn_tx is not None and d_stn_tx > 0:
            up_miss = d_stn_tx - (vrx1 - vrx0)
            a(f"Uplink frames not in the vehicle RX counter: {up_miss} (computed, station_tx minus vehicle RX).")
            a(f"Uplink loss rate: {fmt(100.0 * up_miss / d_stn_tx, 2)} percent (computed).")
            a("The two snapshots are about 0.4 s apart. That skew is a few frames, not this whole gap.")
    if duty_veh is None:
        a("Vehicle duty cycle: not measured.")
    else:
        a(f"Vehicle duty cycle: {fmt(duty_veh * 100, 2)} percent (computed).")
        a("Duty uses TX sent delta times computed nav on-air time, divided by the vehicle counter span.")
    if duty_stn is None:
        a("Station duty cycle: not measured.")
    else:
        a(f"Station duty cycle: {fmt(duty_stn * 100, 2)} percent (computed).")
        a("Station duty uses station_tx delta times computed 14-byte report on-air time.")
        a("A station frame that is not a 14-byte report makes this duty low or high.")
    if duty_veh is not None and duty_stn is not None:
        a(f"Sum of the two duty figures: {fmt((duty_veh + duty_stn) * 100, 2)} percent (computed).")
        a("The sum is channel occupancy only if the two transmissions do not overlap.")
    a("")
    a("## Turnaround and first frame")
    a("")
    a("Key-up to first good frame on a cold start was not a separate test. The link was already up.")
    a(f"First station packet after the vehicle CLI reboot stamp: {first_rx_after_k}.")
    a("That first packet is still the pre-reboot sequence. It was already on the air.")
    a("The first frame after the vehicle came back is in Recovery.")
    a("RX window miss rate: not exposed by firmware.")
    a("The pad RX Hz figure is a 2 s sliding count of rx_crc_ok. It is not a miss counter.")
    a("Half-duplex switch time is not exposed by firmware.")
    a("")
    a("## Command path")
    a("")
    a("No command was sent on this run. ARM was not sent.")
    if acks:
        a(f"ACK millisecond values seen in the station log: {', '.join(acks)} (board-measured).")
    else:
        a("Command ACK time: not exercised. The pad can print ACK <ms> when a command completes.")
    if tries:
        a(f"CMD Try lines seen: {tries} (board-measured).")
    else:
        a("Command resend count: not exercised.")
    if cmd_rows:
        a("CMD log lines (board-measured):")
        for row in cmd_rows[:12]:
            a(f"- {row.strip()}")
    else:
        a("No [CMD] line was in the station log.")
    a(f"Vehicle LINK copp_rtx delta: {nlabel(link_delta(v_link, 'copp_rtx'), 'board-measured')}.")
    a(f"Station LINK copp_rtx delta: {nlabel(link_delta(s_link, 'copp_rtx'), 'board-measured')}.")
    a("copp_rtx counts COP-P sequence resends. The counter lives in COP-P. LoRa and FSK share it.")
    a(f"Vehicle LINK copp_sync_loss delta: {nlabel(link_delta(v_link, 'copp_sync_loss'), 'board-measured')}.")
    a(f"Station LINK copp_sync_loss delta: {nlabel(link_delta(s_link, 'copp_sync_loss'), 'board-measured')}.")
    a("copp_sync_loss counts COP-P LOSS OF SYNCHRONIZATION. 211.0-B-6 Annex D item 8. That event is SE4. It is not a resend.")
    if retry_block:
        a("Station diag printed CmdRetryStats. See stn_raw.log. Rows with a zero sent count are omitted by the firmware.")
    else:
        a("CmdRetryStats block: not seen.")
    a("")
    a("## Recovery")
    a("")
    a("The vehicle got one CLI soft reboot. The key is debug-menu k, after q.")
    a("The station was not rebooted.")
    a(f"Time from the REBOOT_K stamp until the station showed packets again: {recovery}.")
    a("Idle 1 Hz repaints of the last packet are not a resume.")
    a("The resume sample is the next Pkts increase after a silence of at least 1 s.")
    a(f"First RX after the stamp, including a frame already on the air: {first_rx_after_k}.")
    if resume_after_gap:
        a(f"Resume Pkts={resume_after_gap[1]['pkts']} Seq={resume_after_gap[1]['seq']} (board-measured).")
    elif first_after:
        a(f"First post-k Pkts={first_after[1]['pkts']} Seq={first_after[1]['seq']} (board-measured).")
    else:
        a("No post-reboot station packet was parsed before the wait ended.")
    a("")
    a("## Board cost")
    a("")
    a("Radio interrupt time: not exposed by firmware.")
    a("CPU time in the radio path: not exposed by firmware.")
    a("Flash-write stall: not exposed by firmware. This run did not flush the log.")
    if veh_q:
        a(f"Vehicle AO_Radio queue depth={veh_q.group(1)} use={veh_q.group(2)} high={veh_q.group(3)} (board-measured, diag).")
    else:
        a("Vehicle AO_Radio queue: not seen.")
    if radio_q:
        a(f"Station AO_Radio queue depth={radio_q.group(1)} use={radio_q.group(2)} high={radio_q.group(3)} (board-measured, diag).")
    else:
        a("Station AO_Radio queue: not seen.")
    if veh_msp:
        a(f"Vehicle MSP depth={veh_msp.group(3)} bytes (board-measured, diag). This is stack, not radio CPU time.")
    if msp:
        a(f"Station MSP depth={msp.group(3)} bytes (board-measured, diag). This is stack, not radio CPU time.")
    if veh_dropped:
        a(f"Vehicle rc_log dropped={veh_dropped.group(1)} bytes high_water={veh_dropped.group(2)} bytes (board-measured).")
    if dropped:
        a(f"Station rc_log dropped={dropped.group(1)} bytes high_water={dropped.group(2)} bytes (board-measured).")
    a("")
    a("## FSK only: sync")
    a("")
    a("N/A for LoRa. LoRa has no ASM. This baseline does not count ASM misses.")
    a(f"Vehicle LINK copp_sync_loss delta: {nlabel(link_delta(v_link, 'copp_sync_loss'), 'board-measured')}.")
    a(f"Station LINK copp_sync_loss delta: {nlabel(link_delta(s_link, 'copp_sync_loss'), 'board-measured')}.")
    a("That count is Annex D item 8. It is separate from copp_rtx.")
    a("")
    a("## Metrics that need new firmware counters")
    a("")
    a("These are the holes an FSK comparison will hit if the firmware stays as it is.")
    a("- LINK air_ms uses the SX1276 airtime formula at TX done. It is not a pair of TX timestamps.")
    a("- Cumulative TX failures. The CLI fail field is consecutive only.")
    a("- LINK crc_fail is the baseline CRC count. The older CLI CRC err field and RATE rx_crc_fail are still separate.")
    a("- Longest drop run inside the firmware, so the host does not infer it from pad paints.")
    a("- RX window misses. The 2 s RX Hz display is not a miss counter.")
    a("- Half-duplex key-up to first good frame, as a timer.")
    a("- Command ACK time as a retained counter. The pad holds ACK text for about 3 s, then clears it. LINK copp_rtx is the COP-P resend count.")
    a("- Radio interrupt time and CPU time in the radio path.")
    a("- Flash-write stall time.")
    a("- A live readback of frequency and preamble length. Both are init constants today.")
    a("- ASM miss count. LoRa does not use ASM. LINK copp_sync_loss is the COP-P loss-of-sync count.")
    a("")

    path = os.path.join(HERE, "summary.md")
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(lines))
    phase(f"WROTE {path}")


if __name__ == "__main__":
    try:
        if len(sys.argv) > 1 and sys.argv[1] == "--analyze-only":
            write_summary(
                os.path.join(HERE, "veh_raw.log"),
                os.path.join(HERE, "stn_raw.log"),
            )
            raise SystemExit(0)
        raise SystemExit(main())
    except SystemExit:
        raise
    except Exception as exc:
        phase(f"FAILED {type(exc).__name__}: {exc}")
        raise
