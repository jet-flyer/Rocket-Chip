#!/usr/bin/env python3
"""Two-board RF soak capture (TWO_BOARD_SOAK.md).

Opens vehicle COM5 + station COM7 without peek_banner / find_target_port.
DTR is held low at open (no bootrom pulse), then asserted so TinyUSB CDC
prints. Does not attach OpenOCD. Does not picotool.
"""
from __future__ import annotations

import argparse
import datetime as dt
import os
import re
import sys
import threading
import time

import serial

VEH_PORT = "COM5"
STN_PORT = "COM7"
VEH_SER = "02FBDDB8E1CA1281"
STN_SER = "BEC71B8EDC6AEBD1"

# Strip ANSI for scoring (dashboard is color + home/clear).
# Allow CSI '?' private modes and OSC; soak station logs are 90% CSI.
_ANSI_RE = re.compile(
    rb"\x1b\[[?0-9;]*[A-Za-z]|\x1b\].*?(?:\x07|\x1b\\)|\x1b[()][0-9A-Za-z]"
)


def open_cdc(port: str) -> serial.Serial:
    """Open TinyUSB CDC without peek_banner / find_target_port.

    Open with DTR low (FLASHING rule 5: pyserial default DTR-high pulse
    USB-resets Core 0 into bootrom). Then SETDTR so tud_cdc_connected()
    is true and rc_log drains. Do not use DTR_CONTROL_HANDSHAKE
    (dsrdtr=True) — that never presents CDC DTR. Do not abandon a
    CreateFile thread; killing it wedges usbser.sys and forces a replug.
    """
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
    time.sleep(0.5)
    return ser


def ts() -> str:
    return dt.datetime.now().strftime("%H:%M:%S.%f")[:-3]


class PortLogger:
    def __init__(self, name: str, ser: serial.Serial, path: str) -> None:
        self.name = name
        self.ser = ser
        self.path = path
        self.raw = bytearray()
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._fh = open(path, "wb")
        self._thr = threading.Thread(target=self._run, name=f"log-{name}", daemon=True)

    def start(self) -> None:
        self._thr.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(4096)
            except serial.SerialException as exc:
                line = f"[{ts()}] SERIAL_ERR {exc}\n".encode()
                with self._lock:
                    self.raw.extend(line)
                    self._fh.write(line)
                    self._fh.flush()
                break
            if not chunk:
                continue
            stamped = f"[{ts()}] ".encode() + chunk.replace(b"\r\n", b"\n")
            if not stamped.endswith(b"\n") and b"\n" in chunk:
                pass
            with self._lock:
                self.raw.extend(chunk)
                self._fh.write(stamped)
                self._fh.flush()

    def send(self, data: bytes) -> None:
        note = f"[{ts()}] >>> {data!r}\n".encode()
        with self._lock:
            self._fh.write(note)
            self._fh.flush()
        self.ser.write(data)
        self.ser.flush()

    def snapshot_text(self) -> str:
        with self._lock:
            blob = bytes(self.raw)
        clean = _ANSI_RE.sub(b"", blob)
        return clean.decode("utf-8", errors="replace")

    def stop(self) -> None:
        self._stop.set()
        self._thr.join(timeout=2.0)
        try:
            self._fh.close()
        except OSError:
            pass


def last_int(text: str, pattern: str) -> int | None:
    hits = list(re.finditer(pattern, text, re.I))
    if not hits:
        return None
    return int(hits[-1].group(1))


def last_str(text: str, pattern: str) -> str | None:
    hits = list(re.finditer(pattern, text, re.I))
    if not hits:
        return None
    return hits[-1].group(1)


def strip_ansi_text(text: str) -> str:
    blob = text.encode("utf-8", errors="replace")
    return _ANSI_RE.sub(b"", blob).decode("utf-8", errors="replace")


def last_rate(text: str, key: str) -> int | None:
    # Dashboard RATE sits on a CSI-wrapped line. Strip first, then take
    # the last RATE: line's token so home/clear refreshes don't glue keys.
    clean = strip_ansi_text(text)
    return last_int(clean, rf"RATE:[^\n]*\b{re.escape(key)}\s*=\s*(\d+)")


def rate_hz(count: int | None, window_ms: int | None) -> str | None:
    if count is None or window_ms is None or window_ms < 1000:
        return None
    return f"{count / (window_ms / 1000.0):.2f}"


LAYOUT_BLURB = {
    "A": "A (station on metal PC, ~1.5 m, antennas up)",
    "B": "B (vehicle on floor behind chassis, no LOS, antennas orthogonal)",
    "C": "C (station lifted off metal, 2 m LOS, antennas vertical)",
    "D": "D (outdoors range)",
}


def score(cell: str, veh: str, stn: str, duration_s: float, paper: dict,
          layout: str = "A") -> str:
    lines = []
    def add(k: str, v: object) -> None:
        lines.append(f"{k}: {v}")

    add("cell", cell)
    add("duration_s", f"{duration_s:.1f}")
    add("layout", LAYOUT_BLURB.get(layout, layout))
    add("operator_bar", "yellow (RSSI -80..-100; consistent with 2 dBm vs hist -40 @ +20)")
    add("veh_git", last_str(veh, r"([0-9a-f]{7,})") or last_str(veh, r"flight-([0-9a-f]+)"))
    add("veh_air", last_str(veh, r"Air:\s+(\S+)"))
    add("stn_air", last_str(stn, r"Air:\s+(\S+)"))
    add("regversion_veh", last_str(veh, r"RegVersion=(0x[0-9a-fA-F]+)"))
    add("hardware_veh", last_str(veh, r"Hardware:\s+(\d+/\d+ OK)"))
    add("cfg_veh", last_str(veh, r"(CFG: BW=\d+ SF=\d+ CR=\d+ nav=\d+Hz pwr=\d+dBm)"))
    add("cfg_stn_radio_row", last_str(stn, r"(Radio: BW\d+ \d+Hz SF\d+ CR\d+[^\n]*)"))
    add("veh_tx", last_int(veh, r"TX:\s+(\d+)\s+sent"))
    add("stn_rx", last_int(stn, r"Pkts:\s+(\d+)") or last_int(stn, r"RX:\s+(\d+)\s+pkts"))
    add("stn_crc", last_int(stn, r"(\d+)\s+CRC err") or last_int(stn, r"CRC errors:\s+(\d+)"))
    add("stn_rssi_dbm", last_int(stn, r"RSSI:\s+(-?\d+)\s*dBm"))
    add("stn_snr_db", last_int(stn, r"SNR:\s+(-?\d+)\s*dB"))
    add("veh_copp", last_str(veh, r"COP-P:\s+(lock|waiting peer PLCW)"))
    add("stn_copp", last_str(stn, r"COP-P\s+(lock|waiting peer PLCW)"))
    add("stn_rf_link", last_str(stn, r"(RF Link: [^\n]+)"))
    add("arm_sent", "ARM sent" in stn or "[CMD] ARM sent" in stn)
    add("arm_refused", "[SC] LoRa command refused" in stn or "[SC] LoRa command refused" in veh)
    add("fd_arm", bool(re.search(r"\[fd\].*ARM|ARMED|arm_ok", veh, re.I)))
    add("waiting_packets", "Waiting for vehicle packets" in stn)
    add("paper_nav_toa_ms", paper.get("nav_ms"))
    add("paper_plcw_toa_ms", paper.get("plcw_ms"))
    add("paper_duty_5hz_pct", paper.get("duty_pct"))

    veh_win = last_rate(veh, "window_ms")
    stn_win = last_rate(stn, "window_ms")
    add("veh_rate_window_ms", veh_win)
    add("stn_rate_window_ms", stn_win)
    for key in (
        "nav_submit", "pltu_post", "tx_start", "tx_done",
        "tx_busy_drop", "tx_hold_replace", "rx_crc_ok", "rx_crc_fail",
        "station_tx",
    ):
        vn = last_rate(veh, key)
        sn = last_rate(stn, key)
        add(f"veh_{key}_n", vn)
        add(f"stn_{key}_n", sn)
        add(f"veh_{key}_hz", rate_hz(vn, veh_win))
        add(f"stn_{key}_hz", rate_hz(sn, stn_win))

    tx = last_int(veh, r"TX:\s+(\d+)\s+sent")
    rx = last_int(stn, r"Pkts:\s+(\d+)") or last_int(stn, r"RX:\s+(\d+)\s+pkts")
    if tx is None and rx is None:
        add("positive_control", "FAIL no TX/RX parsed — CDC silent or station held")
    elif tx == 0 and (rx is None or rx == 0):
        add("positive_control", "FAIL both tx=0 after window — do not score")
    else:
        add("positive_control", "PASS radio alive")

    # Nav rate from station pkt delta is computed by caller if needed.
    return "\n".join(lines) + "\n"


def paper_toa(sf: int, bw: int, pl: int) -> float:
    t_sym = (1 << sf) * 1000 / bw
    t_pre = (t_sym * 49) / 4
    num = 8 * pl - 4 * sf + 44
    den = 4 * sf
    chunks = 0 if num <= 0 else (num + den - 1) // den
    n_pay = 8 + chunks * 5
    return (t_pre + n_pay * t_sym) / 1000.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cell", default="A1")
    ap.add_argument("--tag", default="bw125_5hz_clsA")
    ap.add_argument("--duration", type=float, default=120.0)
    ap.add_argument("--arm-at", type=float, default=90.0,
                    help="seconds; <0 skips ARM")
    ap.add_argument("--disarm-at", type=float, default=110.0)
    ap.add_argument("--stn-init", default="",
                    help="chars to send on station at t=2s (e.g. xr = menu then SET cycle)")
    ap.add_argument("--stn-init-gap", type=float, default=1.6,
                    help="seconds between stn-init chars so SET can ACK")
    ap.add_argument("--veh-init", default="",
                    help="chars to send on vehicle at t=2.5s (e.g. r = cycle nav rate)")
    ap.add_argument("--bw", type=int, default=125,
                    help="LoRa BW kHz for paper ToA (must match flashed CFG)")
    ap.add_argument("--nav-hz", type=int, default=5,
                    help="nav Hz for paper duty (must match flashed CFG)")
    ap.add_argument("--layout", default="A", choices=sorted(LAYOUT_BLURB),
                    help="physical layout class for the score file")
    args = ap.parse_args()

    date = dt.date.today().isoformat()
    log_dir = os.path.join(os.path.dirname(__file__), "..", "logs", "soak")
    log_dir = os.path.normpath(log_dir)
    os.makedirs(log_dir, exist_ok=True)
    veh_path = os.path.join(log_dir, f"{date}_{args.cell}_{args.tag}.txt")
    stn_path = os.path.join(log_dir, f"{date}_{args.cell}_{args.tag}_stn.txt")
    score_path = os.path.join(log_dir, f"{date}_{args.cell}_{args.tag}_score.txt")

    nav_ms = paper_toa(7, args.bw, 63)
    plcw_ms = paper_toa(7, args.bw, 14)
    period_ms = 1000.0 / float(args.nav_hz)
    paper = {
        "nav_ms": f"{nav_ms:.1f}",
        "plcw_ms": f"{plcw_ms:.1f}",
        "duty_pct": f"{(nav_ms + plcw_ms) / period_ms * 100:.0f}",
    }

    def log(msg: str) -> None:
        print(msg, flush=True)

    log(f"capture {args.cell} {args.tag} {args.duration:.0f}s")
    log(f"  vehicle {VEH_PORT} ({VEH_SER}) -> {veh_path}")
    log(f"  station {STN_PORT} ({STN_SER}) -> {stn_path}")
    log(f"  paper ToA nav={nav_ms:.1f}ms plcw={plcw_ms:.1f}ms duty@{args.nav_hz}Hz={paper['duty_pct']}%")

    veh_ser = open_cdc(VEH_PORT)
    log(f"  {VEH_PORT} open ok")
    stn_ser = None
    stn_open_err = None
    try:
        stn_ser = open_cdc(STN_PORT)
        log(f"  {STN_PORT} open ok")
    except (serial.SerialException, OSError, PermissionError) as exc:
        stn_open_err = str(exc)
        log(f"  {STN_PORT} UNAVAILABLE: {exc}")
        with open(stn_path, "w", encoding="utf-8") as fh:
            fh.write(f"[{ts()}] STATION_CDC_UNAVAILABLE {exc}\n")

    veh = PortLogger("veh", veh_ser, veh_path)
    stn = PortLogger("stn", stn_ser, stn_path) if stn_ser is not None else None
    veh.start()
    if stn is not None:
        stn.start()

    t0 = time.monotonic()
    armed = False
    disarmed = False
    cfg_sent = False
    diag_sent = False
    t10_checked = False
    cfg2_sent = False
    last_live = 0.0
    stn_init_sent = False
    veh_init_sent = False

    try:
        while True:
            elapsed = time.monotonic() - t0
            if elapsed >= args.duration:
                break
            if args.stn_init and stn is not None and not stn_init_sent and elapsed >= 2.0:
                for ch in args.stn_init:
                    stn.send(ch.encode("ascii"))
                    time.sleep(max(0.3, args.stn_init_gap))
                stn_init_sent = True
                log(f"  t={elapsed:.0f}s station init {args.stn_init!r}")
            if args.veh_init and not veh_init_sent and elapsed >= 2.5:
                for ch in args.veh_init:
                    veh.send(ch.encode("ascii"))
                    time.sleep(0.3)
                veh_init_sent = True
                log(f"  t={elapsed:.0f}s vehicle init {args.veh_init!r}")
            if stn is not None and elapsed - last_live >= 10.0:
                stxt = stn.snapshot_text()
                rssi = last_int(stxt, r"RSSI:\s+(-?\d+)\s*dBm")
                snr = last_int(stxt, r"SNR:\s+(-?\d+)\s*dB")
                rx = last_int(stxt, r"Pkts:\s+(\d+)") or last_int(stxt, r"RX:\s+(\d+)\s+pkts")
                crc = last_int(stxt, r"(\d+)\s+CRC err")
                copp = last_str(stxt, r"COP-P\s+(lock|waiting peer PLCW)")
                log(f"  t={elapsed:.0f}s RSSI={rssi} SNR={snr} RX={rx} CRC={crc} COP-P={copp}")
                last_live = elapsed

            if not cfg_sent and elapsed >= 4.0:
                veh.send(b"t")
                # Station stays on ANSI dash here so `a`/`D` ARM still work.
                # RATE is on the dashboard frame (poll_dashboard_keys eats `t`).
                cfg_sent = True
                log(f"  t={elapsed:.0f}s vehicle CLI t (CFG); stn RATE via dash")

            if not diag_sent and elapsed >= 8.0:
                veh.send(b"q")
                time.sleep(0.4)
                veh.send(b"d")
                time.sleep(0.6)
                veh.send(b"z")
                diag_sent = True
                log(f"  t={elapsed:.0f}s vehicle debug d (RegVersion + RATE)")

            if not t10_checked and elapsed >= 12.0:
                vtxt = veh.snapshot_text()
                stxt = stn.snapshot_text() if stn is not None else ""
                tx = last_int(vtxt, r"TX:\s+(\d+)\s+sent")
                rx = last_int(stxt, r"Pkts:\s+(\d+)") or last_int(stxt, r"RX:\s+(\d+)\s+pkts")
                log(f"  t={elapsed:.0f}s positive-control TX={tx} RX={rx}")
                if tx == 0 and (rx is None or rx == 0) and stn is not None:
                    log("  STOP: both tx=0 after 10s — radio/seating, do not score")
                    break
                if tx == 0 and stn is None:
                    log("  note: TX=0 at 12s (vehicle-only; station CDC down)")
                t10_checked = True

            if args.arm_at >= 0 and not armed and elapsed >= args.arm_at:
                if stn is not None:
                    stn.send(b"a")
                    time.sleep(0.35)
                    stn.send(b"ARM\r")
                    armed = True
                    log(f"  t={elapsed:.0f}s station ARM confirm")
                else:
                    armed = True
                    log(f"  t={elapsed:.0f}s ARM skipped — station CDC unavailable")

            if args.disarm_at >= 0 and armed and not disarmed and elapsed >= args.disarm_at:
                if stn is not None:
                    stn.send(b"D")
                    log(f"  t={elapsed:.0f}s station DISARM")
                disarmed = True

            if not cfg2_sent and elapsed >= args.duration - 8.0:
                try:
                    veh.send(b"t")
                    if stn is not None:
                        # After ARM/DISARM: exit dash once (`x`), then `z`/`t`/`s`.
                        # Do not send `x` again (main-menu `x` is flight-erase).
                        stn.send(b"x")
                        time.sleep(0.4)
                        stn.send(b"z")
                        time.sleep(0.25)
                        stn.send(b"t")
                        time.sleep(0.35)
                        stn.send(b"s")
                    log(f"  t={elapsed:.0f}s CLI t (end CFG + RATE) veh+stn")
                except Exception as e:
                    log(f"  t={elapsed:.0f}s end-CLI soft-fail ({e!r}) — continue to score")
                cfg2_sent = True

            time.sleep(0.2)
    finally:
        time.sleep(0.5)
        veh.stop()
        if stn is not None:
            stn.stop()
        try:
            veh_ser.close()
        except OSError:
            pass
        if stn_ser is not None:
            try:
                stn_ser.close()
            except OSError:
                pass

    elapsed = time.monotonic() - t0
    vtxt = veh.snapshot_text()
    stxt = stn.snapshot_text() if stn is not None else (
        f"STATION_CDC_UNAVAILABLE {stn_open_err}\n"
        "operator visual: RSSI bar YELLOW\n"
    )
    summary = score(args.cell, vtxt, stxt, elapsed, paper, layout=args.layout)
    if stn_open_err:
        summary += f"stn_cdc: UNAVAILABLE ({stn_open_err})\n"
    with open(score_path, "w", encoding="utf-8") as fh:
        fh.write(summary)
    print("--- score ---")
    print(summary, end="")
    print(f"wrote {score_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
