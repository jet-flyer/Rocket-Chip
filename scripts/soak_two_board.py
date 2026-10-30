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

# Version banner only, e.g. "flight-e0a307e". A bare hex run is the tree id.
VEH_GIT_RE = r"\bflight-([0-9a-f]{7,40})\b"

# The station prints COP-P status in two forms:
#   Menu/status, with a colon ("COP-P: lock", "COP-P: waiting peer PLCW"):
#     src/cli/rc_os_commands.cpp and src/active_objects/ao_rcos.cpp:200.
#   Dashboard, no colon ("COP-P lock", "COP-P waiting"):
#     src/cli/rc_os_dashboard_format.h:469 and :478.
# The dashboard shows lock only when the peer PLCW is set, the RF link is
# live and the MAC is active. The menu shows lock whenever the PLCW is set.
# So the progress line can say waiting while the final score says lock.
# That is expected.
COP_P_RE = r"COP-P:?\s+(lock|waiting(?: peer PLCW)?)"

# CLI `t` LINK line. Pattern copied from scripts/baseline_lora.py:209-212.
LINK_RE = re.compile(
    r"\bLINK delivered=(\d+) missing=(\d+) crc_fail=(\d+) "
    r"copp_rtx=(\d+) air_ms=(\d+) air_bytes=(\d+) copp_sync_loss=(\d+)"
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


def last_link(text: str) -> dict[str, int] | None:
    """Last CLI `t` LINK line, or None when the log has none."""
    hits = list(LINK_RE.finditer(text))
    if not hits:
        return None
    match = hits[-1]
    return {
        "delivered": int(match.group(1)),
        "missing": int(match.group(2)),
        "crc_fail": int(match.group(3)),
        "copp_rtx": int(match.group(4)),
        "air_ms": int(match.group(5)),
        "air_bytes": int(match.group(6)),
        "copp_sync_loss": int(match.group(7)),
    }


def strip_ansi_text(text: str) -> str:
    blob = text.encode("utf-8", errors="replace")
    return _ANSI_RE.sub(b"", blob).decode("utf-8", errors="replace")


# CLI `t` / debug `r`: "RATE window_ms=..." then indented integer rows.
# "RATE:" is accepted. RATE_HZ is a different block and is not read.
_RATE_HDR_RE = re.compile(
    r"(?m)^(?:\[[^\]]*\][ \t]*)?RATE:?[ \t]+window_ms=(\d+)"
)
# Integer only. "9.9" and "10" of "10.1" do not match.
_RATE_INT_RE = re.compile(r"\b([A-Za-z_][A-Za-z0-9_]*)=(\d+)(?![\d.])")

RATE_KEYS = (
    "nav_submit", "pltu_post", "tx_start", "tx_done",
    "tx_busy_drop", "tx_hold_replace", "rx_crc_ok", "rx_crc_fail",
    "station_tx",
)


def parse_rate_block(text: str) -> dict[str, int] | None:
    """Last RATE block. Integers on the indented rows. Not RATE_HZ."""
    clean = strip_ansi_text(text)
    headers = list(_RATE_HDR_RE.finditer(clean))
    if not headers:
        return None
    hdr = headers[-1]
    nl = clean.find("\n", hdr.start())
    body = ""
    if nl >= 0:
        rows: list[str] = []
        for line in clean[nl + 1:].split("\n"):
            if line.startswith(" ") or line.startswith("\t"):
                rows.append(line)
                continue
            break
        body = "\n".join(rows)
    out: dict[str, int] = {"window_ms": int(hdr.group(1))}
    for match in _RATE_INT_RE.finditer(body):
        out[match.group(1)] = int(match.group(2))
    return out


def last_rate(text: str, key: str) -> int | None:
    block = parse_rate_block(text)
    if block is None:
        return None
    return block.get(key)


def rate_hz(count: int | None, window_ms: int | None) -> str | None:
    if count is None or window_ms is None or window_ms < 1000:
        return None
    return f"{count / (window_ms / 1000.0):.2f}"


def layout_label(layout: str) -> str:
    if layout == "standard-desk":
        return "Nathan's standard desk layout"
    return "NON-STANDARD: " + layout


def tx_sent(text: str) -> int | None:
    return last_int(text, r"TX:\s+(\d+)\s+sent")


def tx_fail(text: str) -> int | None:
    return last_int(text, r"TX:\s+\d+\s+sent\s+(\d+)\s+fail")


def link_field(text: str, key: str) -> int | None:
    row = last_link(text)
    if row is None:
        return None
    return row[key]


# Dashboard "Pkts" / "CRC" and CLI "RX: N pkts" / "N CRC err" are both
# RadioAoState.rx_count and rx_crc_errors (rc_os_dashboard.cpp build_frame;
# rc_os_commands.cpp cmd_radio_status). Start is the dashboard row. End is
# the CLI row. The CLI row is preferred at end so a later dashboard frame
# does not replace the end snapshot.
def dash_pkts(text: str) -> int | None:
    return last_int(text, r"Pkts:\s+(\d+)")


def cli_rx_pkts(text: str) -> int | None:
    return last_int(text, r"RX:\s+(\d+)\s+pkts")


def dash_crc(text: str) -> int | None:
    return last_int(text, r"CRC:\s+(\d+)")


def cli_crc_err(text: str) -> int | None:
    return last_int(text, r"(\d+)\s+CRC err")


def stn_pkts_start(text: str) -> int | None:
    pkts = dash_pkts(text)
    if pkts is not None:
        return pkts
    return cli_rx_pkts(text)


def stn_pkts_end(text: str) -> int | None:
    rx = cli_rx_pkts(text)
    if rx is not None:
        return rx
    return dash_pkts(text)


def stn_crc_start(text: str) -> int | None:
    crc = dash_crc(text)
    if crc is not None:
        return crc
    return cli_crc_err(text)


def stn_crc_end(text: str) -> int | None:
    crc = cli_crc_err(text)
    if crc is not None:
        return crc
    return dash_crc(text)


_CFG_RE = re.compile(
    r"CFG:\s*BW=(\d+)\s+SF=(\d+)\s+CR=(\d+)\s+nav=(\d+)Hz"
)


def paper_report(veh: str, paper: dict) -> dict[str, object]:
    """Paper ToA from the last vehicle CFG line. Args are the fallback.

    paper_toa() does not take CR. SF, BW, and nav Hz come from CFG.
    """
    hits = list(_CFG_RE.finditer(veh))
    if hits:
        bw = int(hits[-1].group(1))
        sf = int(hits[-1].group(2))
        nav_hz = int(hits[-1].group(4))
        nav_ms = paper_toa(sf, bw, 63)
        plcw_ms = paper_toa(sf, bw, 14)
        duty: object = None
        if nav_hz > 0:
            period_ms = 1000.0 / float(nav_hz)
            duty = f"{(nav_ms + plcw_ms) / period_ms * 100:.0f}"
        return {
            "nav_ms": f"{nav_ms:.1f}",
            "plcw_ms": f"{plcw_ms:.1f}",
            "duty_pct": duty,
            "nav_hz": nav_hz,
            "source": "vehicle CFG",
        }
    return {
        "nav_ms": paper.get("nav_ms"),
        "plcw_ms": paper.get("plcw_ms"),
        "duty_pct": paper.get("duty_pct"),
        "nav_hz": paper.get("nav_hz"),
        "source": "cli args (no CFG line)",
    }


def score(cell: str, veh: str, stn: str, duration_s: float, paper: dict,
          layout: str = "standard-desk", uplink_disarm_sent: int = 0,
          veh_start: str | None = None, stn_start: str | None = None,
          delta_window_s: float | None = None) -> str:
    lines = []
    def add(k: str, v: object) -> None:
        lines.append(f"{k}: {v}")

    add("cell", cell)
    add("duration_s", f"{duration_s:.1f}")
    add("layout", layout_label(layout))
    add("veh_git", last_str(veh, VEH_GIT_RE))
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
    add("veh_copp", last_str(veh, COP_P_RE))
    add("stn_copp", last_str(stn, COP_P_RE))
    for prefix, blob in (("veh", veh), ("stn", stn)):
        row = last_link(blob)
        add(f"{prefix}_link_delivered", None if row is None else row["delivered"])
        add(f"{prefix}_link_crc_fail", None if row is None else row["crc_fail"])
        if prefix == "stn":
            # Station missing, copp_rtx and copp_sync_loss are written once,
            # below, with the _since_boot names.
            continue
        add(f"{prefix}_link_missing", None if row is None else row["missing"])
        add(f"{prefix}_copp_rtx", None if row is None else row["copp_rtx"])
        add(f"{prefix}_copp_sync_loss", None if row is None else row["copp_sync_loss"])
    add("stn_rf_link", last_str(stn, r"(RF Link: [^\n]+)"))
    add("arm_sent", "ARM sent" in stn or "[CMD] ARM sent" in stn)
    add("uplink_disarm_sent", uplink_disarm_sent)
    add("arm_refused", "[SC] LoRa command refused" in stn or "[SC] LoRa command refused" in veh)
    add("fd_arm", bool(re.search(r"\[fd\].*ARM|ARMED|arm_ok", veh, re.I)))
    add("waiting_packets", "Waiting for vehicle packets" in stn)
    reported = paper_report(veh, paper)
    add("paper_nav_toa_ms", reported["nav_ms"])
    add("paper_plcw_toa_ms", reported["plcw_ms"])
    add("paper_duty_pct", reported["duty_pct"])
    add("paper_nav_hz", reported["nav_hz"])
    add("paper_source", reported["source"])

    veh_win = last_rate(veh, "window_ms")
    stn_win = last_rate(stn, "window_ms")
    add("veh_rate_window_ms", veh_win)
    add("stn_rate_window_ms", stn_win)
    for key in RATE_KEYS:
        vn = last_rate(veh, key)
        sn = last_rate(stn, key)
        add(f"veh_{key}_n", vn)
        add(f"stn_{key}_n", sn)
        add(f"veh_{key}_hz", rate_hz(vn, veh_win))
        add(f"stn_{key}_hz", rate_hz(sn, stn_win))

    if delta_window_s is None:
        add("delta_window_s", None)
    else:
        add("delta_window_s", f"{float(delta_window_s):.1f}")

    have_veh = veh_start is not None
    have_stn = stn_start is not None
    v0 = veh_start if have_veh else ""
    s0 = stn_start if have_stn else ""

    def add_span(name: str, start: int | None, end: int | None,
                 have_start: bool) -> None:
        # No start snapshot, or no start value: do not use 0 or the boot total.
        if not have_start or start is None:
            add(f"{name}_start", None)
            add(f"{name}_end", end)
            add(f"{name}_delta", None)
            return
        add(f"{name}_start", start)
        add(f"{name}_end", end)
        if end is None:
            add(f"{name}_delta", None)
            return
        if end < start:
            add(f"{name}_delta", None)
            add(f"{name}_delta_note", "counter went back (reboot?)")
            return
        add(f"{name}_delta", end - start)

    add_span("veh_tx_sent", tx_sent(v0) if have_veh else None, tx_sent(veh), have_veh)
    add_span("veh_tx_fail", tx_fail(v0) if have_veh else None, tx_fail(veh), have_veh)
    add_span(
        "stn_rx_pkts",
        stn_pkts_start(s0) if have_stn else None,
        stn_pkts_end(stn),
        have_stn,
    )
    add_span(
        "stn_crc_err",
        stn_crc_start(s0) if have_stn else None,
        stn_crc_end(stn),
        have_stn,
    )
    add_span(
        "veh_copp_rtx",
        link_field(v0, "copp_rtx") if have_veh else None,
        link_field(veh, "copp_rtx"),
        have_veh,
    )
    add_span(
        "veh_copp_sync_loss",
        link_field(v0, "copp_sync_loss") if have_veh else None,
        link_field(veh, "copp_sync_loss"),
        have_veh,
    )
    add_span(
        "veh_link_missing",
        link_field(v0, "missing") if have_veh else None,
        link_field(veh, "missing"),
        have_veh,
    )
    # Station missing, copp_rtx, and copp_sync_loss are the end LINK line.
    # They count since boot. The start path does not open the station menu.
    for boot_name, boot_key in (
        ("stn_link_missing", "missing"),
        ("stn_copp_rtx", "copp_rtx"),
        ("stn_copp_sync_loss", "copp_sync_loss"),
    ):
        add(f"{boot_name}_since_boot", link_field(stn, boot_key))
    add(
        "stn_link_counts_note",
        "since boot; run started right after a power cycle only if the operator says so",
    )

    veh_rate_end = parse_rate_block(veh) or {}
    stn_rate_end = parse_rate_block(stn) or {}
    veh_rate_0 = (parse_rate_block(v0) or {}) if have_veh else {}
    stn_rate_0 = (parse_rate_block(s0) or {}) if have_stn else {}
    for key in RATE_KEYS:
        add_span(
            f"veh_{key}_n",
            veh_rate_0.get(key) if have_veh else None,
            veh_rate_end.get(key),
            have_veh,
        )
        add_span(
            f"stn_{key}_n",
            stn_rate_0.get(key) if have_stn else None,
            stn_rate_end.get(key),
            have_stn,
        )

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
    ap.add_argument("--tag", default="run")
    ap.add_argument("--duration", type=float, default=120.0)
    ap.add_argument("--arm-at", type=float, default=90.0,
                    help="seconds; <0 skips ARM")
    ap.add_argument("--disarm-at", type=float, default=110.0)
    ap.add_argument(
        "--uplink-disarm-every",
        type=float,
        default=0.0,
        help="seconds between station D (uplink DISARM); 0 = off; needs --arm-at -1",
    )
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
    ap.add_argument(
        "--layout",
        default="standard-desk",
        help="physical layout for the score file (free text; default standard-desk)",
    )
    args = ap.parse_args()
    # A repeated DISARM would clear the ARM window. Refuse before any port opens.
    if args.uplink_disarm_every > 0 and args.arm_at >= 0:
        print("uplink DISARM test needs --arm-at -1", file=sys.stderr)
        return 1

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
        "nav_hz": args.nav_hz,
    }

    def log(msg: str) -> None:
        print(msg, flush=True)

    log(f"capture {args.cell} {args.tag} {args.duration:.0f}s")
    log(f"  vehicle {VEH_PORT} ({VEH_SER}) -> {veh_path}")
    log(f"  station {STN_PORT} ({STN_SER}) -> {stn_path}")
    log(
        f"  paper ToA nav={nav_ms:.1f}ms plcw={plcw_ms:.1f}ms "
        f"duty@{args.nav_hz}Hz={paper['duty_pct']}% (from args, before CFG)"
    )

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
    uplink_disarm_sent = 0
    next_uplink_disarm = 15.0
    start_marked = False
    veh_start_txt: str | None = None
    stn_start_txt: str | None = None
    t_start_mark: float | None = None
    t_end_mark: float | None = None

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
                copp = last_str(stxt, COP_P_RE)
                log(f"  t={elapsed:.0f}s RSSI={rssi} SNR={snr} RX={rx} CRC={crc} COP-P={copp}")
                last_live = elapsed

            if not cfg_sent and elapsed >= 4.0:
                veh.send(b"t")
                # Station stays on ANSI dash so `a`/`D` ARM still work.
                # RATE is CLI `t` / debug `r`, scored from the end dump.
                cfg_sent = True
                log(f"  t={elapsed:.0f}s vehicle CLI t (CFG); stn RATE at end CLI t")

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
                # After the positive-control check. Deadline is this 12 s pass.
                # Vehicle `t` only. Station start is the dashboard already in
                # the log. Do not send a station key (end `x` leaves the menu).
                if not start_marked:
                    try:
                        before = veh.snapshot_text()
                        before_win = last_rate(before, "window_ms")
                        veh.send(b"t")
                        answered = False
                        deadline = time.monotonic() + 2.0
                        while time.monotonic() < deadline:
                            time.sleep(0.05)
                            cur = veh.snapshot_text()
                            if len(cur) <= len(before):
                                continue
                            tail = cur[len(before):]
                            win = last_rate(cur, "window_ms")
                            if (
                                win is not None
                                and win != before_win
                                and "LINK delivered=" in tail
                            ):
                                answered = True
                                break
                        if not answered:
                            log(
                                f"  t={elapsed:.0f}s start `t` no answer in 2 s; "
                                "vehicle start values may be old or None"
                            )
                    except Exception as exc:
                        log(f"  t={elapsed:.0f}s start-CLI soft-fail ({exc!r})")
                    # Offset is len(snapshot_text()) at the mark. Keep that
                    # string. A later ANSI byte must not move the cut.
                    veh_now = veh.snapshot_text()
                    veh_start_off = len(veh_now)
                    veh_start_txt = veh_now[:veh_start_off]
                    if stn is not None:
                        stn_now = stn.snapshot_text()
                        stn_start_off = len(stn_now)
                        stn_start_txt = stn_now[:stn_start_off]
                    t_start_mark = time.monotonic()
                    start_marked = True
                    log(
                        f"  t={time.monotonic() - t0:.0f}s start snapshot "
                        "(vehicle t; station dashboard)"
                    )

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

            # Dashboard only: t = 15 s until the end sequence. "D" is DISARM.
            # Do not send x, X, or b here (menu, erase, or the wrong command).
            if (
                args.uplink_disarm_every > 0
                and stn is not None
                and not cfg2_sent
                and 15.0 <= elapsed < args.duration - 8.0
                and elapsed >= next_uplink_disarm
            ):
                stn.send(b"D")
                uplink_disarm_sent += 1
                log(
                    f"  t={elapsed:.0f}s station D "
                    f"(uplink DISARM {uplink_disarm_sent})"
                )
                step = args.uplink_disarm_every
                next_uplink_disarm += step
                if next_uplink_disarm <= elapsed:
                    behind = int((elapsed - next_uplink_disarm) / step) + 1
                    next_uplink_disarm += behind * step

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
                t_end_mark = time.monotonic()
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
    )
    window_s = None
    if t_start_mark is not None and t_end_mark is not None:
        window_s = t_end_mark - t_start_mark
    summary = score(
        args.cell, vtxt, stxt, elapsed, paper,
        layout=args.layout, uplink_disarm_sent=uplink_disarm_sent,
        veh_start=veh_start_txt, stn_start=stn_start_txt,
        delta_window_s=window_s,
    )
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
