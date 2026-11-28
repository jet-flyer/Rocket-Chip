#!/usr/bin/env python3
"""Two-board radio-link bench (SESSION_CHECKLIST 6a).

Before a radio push, this script checks the link between the vehicle board
and the station board. The pre-push gate runs it when a push changes a
[radio] path (scripts/ci/pre_push_gate.py):

    python scripts/radio_link_bench.py --commit <sha>

The gate also passes --vehicle-serial and --station-serial: the USB serials
the role benches used, so this script tests the same two boards.

The gate runs the copy of this file in its own checkout. It does not need
the file in the pushed commit.

Steps. Each step prints "STEP <name>: PASS|FAIL". The last line is
"RESULT: n/m PASS" or "RESULT: n/m FAIL".

1. open-ports. Find both boards, check the USB serials, open the ports.
2. power-2dBm. CLI t on both boards: "PA: 2 dBm (reg)" (the radio
   register readback) and "pwr=2dBm" in the CFG line. Any other value,
   "PA: unread" or "PA: unverified" fails. This is the desk rule of
   SESSION_CHECKLIST 6a item 2.
3. link-up. Station packets rise or COP-P lock; vehicle TX rises.
4. round-trip. Telemetry rises, then a pad D (DISARM) gets an uplink ACK.
5. forced-tx-timeout. Vehicle debug menu: q, o, z. Both log lines
   "RADIO: TX timeout armed" and "RADIO: TX timeout fired" must appear.
   Then the link must still run: vehicle TX rises, the TX fail count does
   not rise, station packets rise, and COP-P lock stays if it was there.
   Vehicle only. The station has no o command (src/cli/cli_menus.h).
6. hop-500/10, then hop-back-250/10. One NAV_PRESET hop (COMM_CHANGE)
   from 250/10 to 500/10, and the hops back to 250/10.
7. power-2dBm-after-hop. Same check as step 2, after the hops.

The script records the LINK counters (delivered, missing, crc_fail, ...)
of both boards in its output. It does not use them as pass limits.

Exit codes: 0 all steps passed. 1 a step failed. 2 setup failed (board
not found, serial mismatch, no banner, bad boards.local.json).

Board choice, in this order: --vehicle-serial / --station-serial, then
scripts/ci/boards.local.json (or RC_BOARDS_CONFIG), then the
RC_VEHICLE_USB_SERIAL / RC_STATION_USB_SERIAL variables, then the banner
peek of scripts/_rc_test_common.py (the same fallback the role benches
use). --vehicle-port and --station-port force a port.

Does not flash. Does not attach OpenOCD. Does not send CLI keys b, B, k or
K (main/pad b is the find-me beacon; debug k is I2C quiesce+reboot; see
src/cli/cli_menus.h). Version is read from the connect banner
(cli_print_boot_summary) or from CLI t (``ID: flight-<hash>``), never via
debug b. The script never sends v: in the cal menu v is "save cal".

Serial open matches soak_two_board.py / baseline_lora.py: dsrdtr=False,
dtr=False, open, then dtr=True so TinyUSB CDC talks (FLASHING rule 5).

Station hop key sequence, from source, not guesswork:

- The ANSI pad (ao_rcos.cpp poll_dashboard_keys) accepts only x (menu),
  a (ARM) and D (DISARM). n does nothing on the pad.
- After lowercase x, station main s opens settings; settings n is
  "next radio preset" (ActionId::kCatalogNavNext).
- Do not send X after leaving the pad: station main X is DISARM
  (fold_case false).

Catalog (include/rocketchip/radio_config_table.h) and the boot default
(kDefaultRocketRadioConfig: 250 kHz, 10 Hz, SF7, CR 4/5, 2 dBm):

    n: 250/10 -> 500/10

n only advances. Hop-back to 250/10 therefore transits
500/10 -> 125/2 -> 250/5 -> 125/5 -> 250/10 (next_fit skips leftover
125/10, whose nav PLTU does not fit a 100 ms slot). Those 125 transits
are required for hop-back; --include-125 reports them as named steps.

Later list (not in this script): a station TX-timeout hook, and a hook to
drop the vehicle COMM_CHANGE echo (host test only today:
test_starcom_comm_change.cpp Sim.drop_vehicle_echoes).

Wait budgets in this file are harness patience. A step fails when the
expected line does not arrive in time. No wait budget is a link-quality
limit. Pass and fail use counters that must rise, lock flags and named log
lines. They never use a rate or a loss limit.

Positive-control signals (HW_GATE_DISCIPLINE Rule 1), not "nothing failed":

- Link up: station Pkts increment and/or dashboard "COP-P lock"; vehicle
  CLI t "TX: N sent" increment and/or "COP-P: lock".
- Round trip: further station packet increment plus an uplink ACK
  (DISARM from the pad, capital D only; look for "[CMD] ACK'd" or
  "ACK <ms>"). QUERY_RADIO_CONFIG has no CLI key in cli_menus.h.
- Hop: station "NAV_PRESET COMM_CHANGE BW..", vehicle
  "[SC] COMM_CHANGE remote armed" and "remote apply", station
  "[SC] COMM_CHANGE ok", then CFG on both sides and link resume.
"""
from __future__ import annotations

import argparse
import datetime as dt
import os
import re
import sys
import threading
import time
from dataclasses import dataclass, field

import serial
import serial.tools.list_ports

_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (_HERE, os.path.join(_HERE, "ci")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import _rc_test_common as rct  # noqa: E402
import board_config  # noqa: E402

# TinyUSB CDC identity: one definition, in scripts/_rc_test_common.py.
ROCKETCHIP_USB_VID = rct.ROCKETCHIP_USB_VID
ROCKETCHIP_USB_PID = rct.ROCKETCHIP_USB_PID

FORBIDDEN_KEYS = frozenset(b"bBkK")

# Desk radio power. Source: SESSION_CHECKLIST 6a item 2 (desk radio power
# stays at 2 dBm on both boards); the firmware default is the same
# (kDefaultRocketRadioConfig).
DESK_POWER_DBM = 2

# Forced TX timeout. The latch fires on the next TX start (ao_radio.cpp
# radio_start_tx). With the link up, a TX start happens at least once per
# nav period. The window is a number of nav periods, with a floor. It is
# harness patience, not a link limit.
FORCED_TX_ALLOWANCE_PERIODS = 10
FORCED_TX_MIN_S = 2.0
# Listen time for "packets keep rising". Same value the round-trip step used
# before this script had a forced-timeout step.
OBSERVE_S = 4.0

# Must match include/rocketchip/radio_config_table.h (do not reorder).
# idx 1 (125/10) is chip-legal leftover: nav PLTU ToA does not fit 10 Hz.
CATALOG: tuple[tuple[int, int], ...] = (
    (125, 5),
    (125, 10),
    (250, 10),
    (500, 10),
    (125, 2),
    (250, 5),
)
SKIP_FIT = frozenset({(125, 10)})
DEFAULT_PRESET = (250, 10)
TARGET_HOP = (500, 10)

_ANSI_RE = re.compile(
    rb"\x1b\[[?0-9;]*[A-Za-z]|\x1b\].*?(?:\x07|\x1b\\)|\x1b[()][0-9A-Za-z]"
)

# CLI t LINK line. Copied from soak_two_board.py / baseline_lora.py.
LINK_RE = re.compile(
    r"\bLINK delivered=(\d+) missing=(\d+) crc_fail=(\d+) "
    r"copp_rtx=(\d+) air_ms=(\d+) air_bytes=(\d+) copp_sync_loss=(\d+)"
)
LINK_KEYS = ("delivered", "missing", "crc_fail", "copp_rtx", "air_ms",
             "air_bytes", "copp_sync_loss")
CFG_RE = re.compile(
    r"CFG:\s*BW=(\d+)\s+SF=(\d+)\s+CR=(\d+)\s+nav=(\d+)Hz\s+pwr=(\d+)dBm",
    re.I,
)
HOP_ACCEPT_RE = re.compile(
    r"NAV_PRESET COMM_CHANGE BW(\d+)\s+(\d+)Hz SF(\d+)\s+\(idx\s+(\d+)",
    re.I,
)
TX_SENT_RE = re.compile(r"TX:\s+(\d+)\s+sent", re.I)
TX_FAIL_RE = re.compile(r"TX:\s+\d+\s+sent\s+(\d+)\s+fail", re.I)
RX_PKTS_RE = re.compile(r"RX:\s+(\d+)\s+pkts", re.I)
DASH_PKTS_RE = re.compile(r"Pkts:\s+(\d+)", re.I)
ACK_MS_RE = re.compile(r"ACK\s+(\d+)\s*ms", re.I)
ACK_CMD_RE = re.compile(r"\[CMD\]\s+ACK'd\s+\(seq=(\d+)\)", re.I)
NO_ACK_RE = re.compile(r"\[CMD\]\s+pending cleared \(no ACK", re.I)
ID_FLIGHT_RE = re.compile(r"\bID:\s+flight-([0-9a-f]{7,40})\b", re.I)
BANNER_RE = re.compile(r"RocketChip\s+(\S+)\s+(\S+)-(\S+)")
UPTIME_RE = re.compile(r"Uptime:\s+(\d+)s", re.I)
# CLI t: "PA: 2 dBm (reg)" from the radio registers (rc_os_commands.cpp),
# or "PA: unread" / "PA: unverified" (rfm95w_decode_tx_power_dbm).
PA_RE = re.compile(r"\bPA:\s+(-?\d+)\s*dBm\s*\(reg\)", re.I)
PA_BAD_RE = re.compile(r"\bPA:\s+(unread|unverified)\b", re.I)
PROMPT_RE = re.compile(r"\[(main|cal|flight|debug|settings|clock|gps)\]")
TX_ARMED_LINE = "RADIO: TX timeout armed"
TX_FIRED_LINE = "RADIO: TX timeout fired"

def catalog_index(bw: int, nav: int) -> int | None:
    for i, row in enumerate(CATALOG):
        if row == (bw, nav):
            return i
    return None


def next_fit(idx: int) -> int:
    """radio_config_next_fit: next row whose nav PLTU ToA fits commanded Hz."""
    n = len(CATALOG)
    start = 0 if idx < 0 or idx >= n else (idx + 1) % n
    for k in range(n):
        i = (start + k) % n
        if CATALOG[i] not in SKIP_FIT:
            return i
    raise RuntimeError("no catalog row fits nav ToA")


def hop_path(start: tuple[int, int], dest: tuple[int, int]) -> list[tuple[int, int]]:
    """Forward n path from start to dest, exclusive of start, inclusive of dest."""
    si = catalog_index(*start)
    di = catalog_index(*dest)
    if si is None or di is None:
        raise ValueError(f"preset not in catalog: {start} -> {dest}")
    out: list[tuple[int, int]] = []
    i = si
    for _ in range(len(CATALOG) + 1):
        i = next_fit(i)
        out.append(CATALOG[i])
        if i == di:
            return out
    raise RuntimeError(f"no forward path {start} -> {dest}")


def ts() -> str:
    return dt.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def last_int(text: str, pattern: str | re.Pattern[str]) -> int | None:
    hits = list(re.finditer(pattern, text, re.I) if isinstance(pattern, str)
                else pattern.finditer(text))
    if not hits:
        return None
    return int(hits[-1].group(1))


def last_cfg(text: str) -> tuple[int, int, int] | None:
    """Last CLI t CFG: (bw, nav, pwr_dbm) or None."""
    hits = list(CFG_RE.finditer(text))
    if not hits:
        return None
    m = hits[-1]
    return int(m.group(1)), int(m.group(4)), int(m.group(5))


def last_hop_accept(text: str) -> tuple[int, int, int] | None:
    """Last NAV_PRESET COMM_CHANGE accept: (bw, nav, idx) or None."""
    hits = list(HOP_ACCEPT_RE.finditer(text))
    if not hits:
        return None
    m = hits[-1]
    return int(m.group(1)), int(m.group(2)), int(m.group(4))


def has_copp_lock(text: str) -> bool:
    # Dashboard: "COP-P lock". CLI t / banner: "COP-P: lock".
    return bool(re.search(r"COP-P:?\s+lock", text, re.I))


def last_pkts(text: str) -> int | None:
    return last_int(text, DASH_PKTS_RE) or last_int(text, RX_PKTS_RE)


def open_cdc(port: str) -> serial.Serial:
    """Open TinyUSB CDC without peek_banner / find_target_port.

    Open with DTR low (FLASHING rule 5: pyserial default DTR-high pulse
    USB-resets Core 0 into bootrom). Then SETDTR so tud_cdc_connected()
    is true and rc_log drains. Do not use DTR_CONTROL_HANDSHAKE
    (dsrdtr=True) - that never presents CDC DTR.
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


def attached_ports() -> dict[str, str]:
    """USB serial (upper case) -> COM port of the attached RocketChip CDC ports."""
    out: dict[str, str] = {}
    for info in serial.tools.list_ports.comports():
        if (getattr(info, "vid", None) == ROCKETCHIP_USB_VID
                and getattr(info, "pid", None) == ROCKETCHIP_USB_PID
                and info.serial_number and info.device):
            out[info.serial_number.upper()] = info.device
    return out


def port_serial(port: str) -> str | None:
    """USB serial of a RocketChip CDC port (enumeration only), or None."""
    return rct.board_usb_serial(port)


@dataclass
class BoardPick:
    port: str
    serial: str | None
    how: str


def resolve_boards(args: argparse.Namespace,
                   cfg_fn=board_config.load,
                   ports_fn=attached_ports,
                   serial_of=port_serial,
                   banner_fn=rct.find_vehicle_and_station_ports,
                   ) -> dict[str, BoardPick] | str:
    """Pick the vehicle and station port. Returns {role: BoardPick} or an
    error string. Order: --<role>-serial, boards.local.json, RC_*_USB_SERIAL
    (board_config.role_serial), then the banner peek."""
    try:
        cfg = cfg_fn()
    except board_config.BoardConfigError as exc:
        return str(exc)
    attached = ports_fn()
    picks: dict[str, BoardPick] = {}
    banner_roles: list[str] = []
    for role in ("vehicle", "station"):
        port_arg = getattr(args, f"{role}_port")
        serial_arg = getattr(args, f"{role}_serial")
        if serial_arg:
            want, source, strict = serial_arg.strip().upper(), f"--{role}-serial", True
        else:
            want, source, strict = board_config.role_serial(role, cfg)
        if port_arg:
            picks[role] = BoardPick(port_arg, want, f"{role}: --{role}-port {port_arg}")
        elif want:
            port = attached.get(want)
            if port is None:
                return (f"{role} board {want} ({source}) is not attached. "
                        f"Attached: {', '.join(sorted(attached)) or 'none'}")
            picks[role] = BoardPick(port, want,
                                    f"{role}: USB serial {want} ({source}) -> {port}")
        elif strict:
            return (f"no {role} board is listed in {board_config.config_path()}. "
                    f"Add it, then run again.")
        else:
            banner_roles.append(role)
    if banner_roles:
        veh_p, stn_p, _vb, _sb, reason = banner_fn(
            station_override=picks["station"].port if "station" in picks else None,
            vehicle_override=picks["vehicle"].port if "vehicle" in picks else None,
        )
        if veh_p is None or stn_p is None:
            return f"banner peek found no board pair: {reason}"
        found = {"vehicle": veh_p, "station": stn_p}
        for role in banner_roles:
            picks[role] = BoardPick(found[role], None,
                                    f"{role}: banner peek -> {found[role]}")
    for role, pick in picks.items():
        got = serial_of(pick.port)
        if pick.serial and got and got.upper() != pick.serial.upper():
            return (f"{role} {pick.port}: USB serial {got} does not match "
                    f"{pick.serial}")
        if got:
            pick.serial = got.upper()
    if picks["vehicle"].port == picks["station"].port:
        return "vehicle and station resolved to the same port"
    return picks

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
            with self._lock:
                self.raw.extend(chunk)
                self._fh.write(stamped)
                self._fh.flush()

    def send(self, data: bytes) -> None:
        for b in data:
            if b in FORBIDDEN_KEYS:
                raise RuntimeError(
                    f"refusing to send CLI key {data!r} on {self.name}: "
                    f"b/B is the find-me beacon, k/K is I2C quiesce+reboot"
                )
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

    def snapshot_from(self, origin: int) -> str:
        text = self.snapshot_text()
        return text[origin:] if origin <= len(text) else ""

    def mark(self) -> int:
        return len(self.snapshot_text())

    def stop(self) -> None:
        self._stop.set()
        self._thr.join(timeout=2.0)
        try:
            self._fh.close()
        except OSError:
            pass


@dataclass
class Step:
    name: str
    ok: bool | None = None
    detail: str = ""


@dataclass
class Bench:
    veh: PortLogger
    stn: PortLogger
    deadline: float
    key_gap_s: float = 0.45
    steps: list[Step] = field(default_factory=list)

    def remain(self) -> float:
        return max(0.0, self.deadline - time.monotonic())

    def cap(self, seconds: float) -> float:
        return min(seconds, self.remain())

    def log(self, msg: str) -> None:
        print(msg, flush=True)

    def record(self, name: str, ok: bool, detail: str) -> Step:
        step = Step(name, ok, detail)
        self.steps.append(step)
        status = "PASS" if ok else "FAIL"
        self.log(f"STEP {name}: {status}  {detail}")
        return step

    def wait(self, timeout: float, pred, label: str) -> bool:
        timeout = self.cap(timeout)
        if timeout <= 0:
            return False
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if pred():
                return True
            time.sleep(0.05)
        self.log(f"  timeout ({timeout:.0f}s) waiting for {label}")
        return False

    def send_stn(self, keys: str, after: float | None = None) -> None:
        gap = self.key_gap_s if after is None else after
        for ch in keys:
            self.stn.send(ch.encode("ascii"))
            time.sleep(gap)

    def send_veh(self, keys: str, after: float | None = None) -> None:
        gap = self.key_gap_s if after is None else after
        for ch in keys:
            self.veh.send(ch.encode("ascii"))
            time.sleep(gap)

    def stn_on_pad(self, text: str | None = None) -> bool:
        t = self.stn.snapshot_text() if text is None else text
        # After x, enter_cli_menu prints "Station RX" and help, then [main].
        menu = max(t.rfind("Station RX"), t.rfind("[main]"), t.rfind("[settings]"),
                   t.rfind("[debug]"), t.rfind("[clock]"), t.rfind("[gps]"))
        pad = max(t.rfind("=== RocketChip Ground Station ==="),
                  t.rfind("'x' menu"))
        return pad >= 0 and pad > menu

    def enter_station_settings(self, timeout: float = 8.0) -> bool:
        """Leave the ANSI pad if needed, then open the settings menu."""
        text = self.stn.snapshot_text()
        if "[settings]" in text[text.rfind("[main]") if "[main]" in text else 0:]:
            # In settings if the latest prompt is [settings], not a later [main].
            last_settings = text.rfind("[settings]")
            last_main = text.rfind("[main]")
            last_pad = max(text.rfind("=== RocketChip Ground Station ==="),
                           text.rfind("'x' menu"))
            if last_settings > last_main and last_settings > last_pad:
                return True
        if self.stn_on_pad(text):
            mark = self.stn.mark()
            self.send_stn("x")
            if not self.wait(
                timeout,
                lambda: (
                    "[main]" in self.stn.snapshot_from(mark)
                    or "Station RX" in self.stn.snapshot_from(mark)
                ),
                "station menu after x",
            ):
                return False
        mark = self.stn.mark()
        self.send_stn("s")
        return self.wait(
            timeout,
            lambda: "[settings]" in self.stn.snapshot_from(mark),
            "[settings] prompt",
        )

    def return_station_main(self, timeout: float = 6.0) -> bool:
        text = self.stn.snapshot_text()
        if self.stn_on_pad(text):
            mark = self.stn.mark()
            self.send_stn("x")
            return self.wait(
                timeout,
                lambda: "[main]" in self.stn.snapshot_from(mark)
                or "Station RX" in self.stn.snapshot_from(mark),
                "station menu after x",
            )
        last_settings = text.rfind("[settings]")
        last_main = text.rfind("[main]")
        last_debug = text.rfind("[debug]")
        if last_settings > last_main or last_debug > last_main:
            mark = self.stn.mark()
            self.send_stn("z")
            return self.wait(
                timeout,
                lambda: "[main]" in self.stn.snapshot_from(mark),
                "[main] after z",
            )
        return True

    def vehicle_t(self) -> str:
        """Send t on the vehicle. Wait for the last line (LINK) so the PA
        and CFG lines are in the text that comes back."""
        mark = self.veh.mark()
        self.send_veh("t")
        self.wait(
            3.0,
            lambda: "LINK delivered=" in self.veh.snapshot_from(mark),
            "vehicle CLI t (LINK line)",
        )
        return self.veh.snapshot_from(mark)

    def station_t(self) -> str:
        if not self.return_station_main():
            return ""
        mark = self.stn.mark()
        self.send_stn("t")
        self.wait(
            3.0,
            lambda: "LINK delivered=" in self.stn.snapshot_from(mark),
            "station CLI t (LINK line)",
        )
        return self.stn.snapshot_from(mark)

    def vehicle_menu(self, timeout: float = 4.0) -> str | None:
        """Name of the vehicle menu from its newest prompt. h is help in
        every menu, so it is safe and gives a fresh prompt."""
        mark = self.veh.mark()
        self.send_veh("h")
        self.wait(
            timeout,
            lambda: PROMPT_RE.search(self.veh.snapshot_from(mark)) is not None,
            "vehicle prompt after h",
        )
        hits = PROMPT_RE.findall(self.veh.snapshot_from(mark)) or \
            PROMPT_RE.findall(self.veh.snapshot_text())
        return hits[-1] if hits else None

    def vehicle_to_main(self, timeout: float = 4.0) -> bool:
        """True when the vehicle is at [main]. Leaves debug, settings,
        flight, clock and gps with z. Does not leave the cal menu (back is x
        there, and cal has write keys), so that case returns False."""
        menu = self.vehicle_menu(timeout)
        if menu == "main":
            return True
        if menu in ("debug", "settings", "flight", "clock", "gps"):
            mark = self.veh.mark()
            self.send_veh("z")
            return self.wait(
                timeout,
                lambda: "[main]" in self.veh.snapshot_from(mark),
                "[main] after z",
            )
        return False

def last_link(text: str) -> dict[str, int] | None:
    """Last LINK line of a CLI t dump as {name: value}, or None."""
    hits = list(LINK_RE.finditer(text))
    if not hits:
        return None
    return dict(zip(LINK_KEYS, (int(v) for v in hits[-1].groups())))


def log_link(bench: "Bench", label: str, text: str) -> None:
    """Print the LINK counters. Recorded only: no pass limit uses them."""
    link = last_link(text)
    shown = " ".join(f"{k}={v}" for k, v in link.items()) if link else "not printed"
    bench.log(f"  LINK {label}: {shown} (recorded, not a pass limit)")


def power_verdict(text: str) -> tuple[bool, str]:
    """Check one board's CLI t text against the desk power rule.
    Pass only for "PA: 2 dBm (reg)" and "pwr=2dBm" in the CFG line."""
    bad = PA_BAD_RE.search(text)
    if bad:
        return False, f"PA readback is {bad.group(1)} (the radio register was not verified)"
    pa = PA_RE.findall(text)
    if not pa:
        return False, "no PA line in the t output"
    cfg = last_cfg(text)
    if cfg is None:
        return False, "no CFG line in the t output"
    pa_dbm, cfg_dbm = int(pa[-1]), cfg[2]
    if pa_dbm != DESK_POWER_DBM or cfg_dbm != DESK_POWER_DBM:
        return False, (f"PA {pa_dbm} dBm (reg), CFG pwr={cfg_dbm} dBm; "
                       f"the desk rule is {DESK_POWER_DBM} dBm")
    return True, f"PA {pa_dbm} dBm (reg), CFG pwr={cfg_dbm} dBm"


def step_power(bench: "Bench", name: str) -> bool:
    """Hard check of the desk power rule on both boards (CLI t)."""
    if not bench.vehicle_to_main():
        bench.record(name, False,
                     "vehicle is not at the [main] prompt; will not send t")
        return False
    vt = bench.vehicle_t()
    st = bench.station_t()
    log_link(bench, f"vehicle ({name})", vt)
    log_link(bench, f"station ({name})", st)
    v_ok, v_msg = power_verdict(vt)
    s_ok, s_msg = power_verdict(st) if st else (False, "no station t output")
    bench.record(name, v_ok and s_ok, f"vehicle {v_msg}; station {s_msg}")
    return v_ok and s_ok


def forced_tx_window_s(nav_hz: int) -> float:
    """Wait budget for the forced TX timeout: FORCED_TX_ALLOWANCE_PERIODS
    nav periods, not less than FORCED_TX_MIN_S. Harness patience."""
    if nav_hz <= 0:
        return FORCED_TX_MIN_S
    return max(FORCED_TX_MIN_S, FORCED_TX_ALLOWANCE_PERIODS / float(nav_hz))


def step_forced_tx_timeout(bench: "Bench") -> bool:
    """Force one radio HW TX timeout on the vehicle (debug o) and see it
    fire. Then prove the link still runs: the forced path returns early in
    handle_tx_timeout (ao_radio.cpp), so a clean log line alone is not enough."""
    name = "forced-tx-timeout"
    if not bench.vehicle_to_main():
        bench.record(name, False,
                     "vehicle is not at the [main] prompt; will not send q or o")
        return False
    if not bench.stn_on_pad() and not _return_to_pad(bench):
        bench.record(name, False, "station is not on the ANSI pad; cannot read Pkts")
        return False
    vt0 = bench.vehicle_t()
    cfg = last_cfg(vt0)
    tx0 = last_int(vt0, TX_SENT_RE)
    fail0 = last_int(vt0, TX_FAIL_RE)
    if cfg is None or tx0 is None or fail0 is None:
        bench.record(name, False, "could not read CFG and TX counts from vehicle t")
        return False
    window = forced_tx_window_s(cfg[1])
    lock0 = has_copp_lock(vt0)

    mark_q = bench.veh.mark()
    bench.send_veh("q")
    in_debug = bench.wait(4.0, lambda: "[debug]" in bench.veh.snapshot_from(mark_q),
                          "vehicle [debug] prompt")
    armed = fired = False
    fired_after = None
    if in_debug:
        mark_o = bench.veh.mark()
        t_o = time.monotonic()
        bench.send_veh("o", after=0.1)
        armed = bench.wait(3.0, lambda: TX_ARMED_LINE in bench.veh.snapshot_from(mark_o),
                           TX_ARMED_LINE)
        if armed:
            fired = bench.wait(window,
                               lambda: TX_FIRED_LINE in bench.veh.snapshot_from(mark_o),
                               TX_FIRED_LINE)
            fired_after = time.monotonic() - t_o
        bench.send_veh("z")
    back = bench.vehicle_to_main()
    if not (in_debug and armed and fired):
        bench.record(
            name, False,
            f"debug menu={in_debug} armed={armed} fired={fired} "
            f"(window {window:.1f}s, nav {cfg[1]}Hz). "
            f"o needs a ROCKETCHIP_DEV_MODE image (the gate presets have it).")
        return False
    if not back:
        bench.record(name, False, "vehicle did not return to [main] after z")
        return False

    # The link must still run after the forced timeout.
    pk_a = last_pkts(bench.stn.snapshot_text())
    s_mark = bench.stn.mark()
    time.sleep(min(OBSERVE_S, bench.cap(OBSERVE_S)))
    pk_b = last_pkts(bench.stn.snapshot_text())
    s_lock = has_copp_lock(bench.stn.snapshot_from(s_mark))
    vt1 = bench.vehicle_t()
    tx1 = last_int(vt1, TX_SENT_RE)
    fail1 = last_int(vt1, TX_FAIL_RE)
    lock1 = has_copp_lock(vt1)
    log_link(bench, "vehicle (forced-tx-timeout)", vt1)
    problems = []
    if tx1 is None or tx1 <= tx0:
        problems.append(f"vehicle TX did not rise ({tx0} -> {tx1})")
    if fail1 is None or fail1 > fail0:
        problems.append(f"vehicle TX fail count rose ({fail0} -> {fail1})")
    if pk_a is None or pk_b is None or pk_b <= pk_a:
        problems.append(f"station Pkts did not rise ({pk_a} -> {pk_b})")
    if lock0 and not (lock1 and s_lock):
        problems.append(f"COP-P lock was up and is not (vehicle={lock1} station={s_lock})")
    detail = (f"armed, fired after {fired_after:.2f}s (window {window:.1f}s, "
              f"nav {cfg[1]}Hz); vehicle TX {tx0} -> {tx1}, fail {fail0} -> {fail1}; "
              f"station Pkts {pk_a} -> {pk_b}; COP-P lock before={lock0} "
              f"after vehicle={lock1} station={s_lock}")
    if problems:
        detail += "; PROBLEM: " + "; ".join(problems)
    bench.record(name, not problems, detail)
    return not problems

def run_self_check() -> int:
    """Catalog / key-guard / parser checks. Opens no ports."""
    checks: list[tuple[bool, str]] = []

    def check(cond: bool, msg: str) -> None:
        checks.append((cond, msg))
        print(f"  {'PASS' if cond else 'FAIL'}  {msg}")

    check(next_fit(0) == 2, "next_fit(0) == 2 (125/5 skips 125/10 -> 250/10)")
    check(next_fit(1) == 2, "next_fit(1) == 2 (leftover 125/10 -> 250/10)")
    check(next_fit(2) == 3, "next_fit(2) == 3 (250/10 -> 500/10)")
    check(next_fit(3) == 4, "next_fit(3) == 4 (500/10 -> 125/2)")
    check(next_fit(5) == 0, "next_fit(5) == 0 (250/5 wrap -> 125/5)")
    path = hop_path((250, 10), (250, 10))
    check(
        path == [(500, 10), (125, 2), (250, 5), (125, 5), (250, 10)],
        f"hop-back path from 250/10: {path}",
    )
    check(hop_path((250, 10), (500, 10)) == [(500, 10)], "250/10 -> 500/10 is one n")
    check(ord("b") in FORBIDDEN_KEYS and ord("K") in FORBIDDEN_KEYS, "forbidden b/B/k/K")
    check(ord("x") not in FORBIDDEN_KEYS and ord("D") not in FORBIDDEN_KEYS,
          "x and D remain allowed (pad menu / DISARM)")
    check(ord("n") not in FORBIDDEN_KEYS and ord("s") not in FORBIDDEN_KEYS,
          "s and n remain allowed (settings / next preset)")
    check(all(ord(c) not in FORBIDDEN_KEYS for c in "qozht"),
          "q o z h t remain allowed (debug menu, help, radio status)")

    good = ("CFG: BW=250 SF=7 CR=5 nav=10Hz pwr=2dBm\nPA: 2 dBm (reg)\n"
            "LINK delivered=5 missing=1 crc_fail=2 copp_rtx=3 air_ms=4 "
            "air_bytes=6 copp_sync_loss=7\n")
    check(power_verdict(good)[0], "power: PA 2 dBm (reg) and pwr=2dBm passes")
    check(not power_verdict(good.replace("PA: 2 dBm", "PA: 5 dBm"))[0],
          "power: PA 5 dBm fails")
    check(not power_verdict(good.replace("pwr=2dBm", "pwr=5dBm"))[0],
          "power: CFG pwr=5dBm fails")
    check(not power_verdict(good.replace("PA: 2 dBm (reg)", "PA: unread"))[0],
          "power: PA unread fails")
    check(not power_verdict(good.replace("PA: 2 dBm (reg)", "PA: unverified"))[0],
          "power: PA unverified fails")
    check(not power_verdict("CFG: BW=250 SF=7 CR=5 nav=10Hz pwr=2dBm\n")[0],
          "power: no PA line fails")
    check(not power_verdict("PA: 2 dBm (reg)\n")[0], "power: no CFG line fails")
    link = last_link(good)
    check(link is not None and link["crc_fail"] == 2 and link["copp_sync_loss"] == 7,
          "LINK line parses")
    check(forced_tx_window_s(10) == FORCED_TX_MIN_S and forced_tx_window_s(2) == 5.0
          and forced_tx_window_s(0) == FORCED_TX_MIN_S,
          "forced-timeout window: floor at 10 Hz, 5 s at 2 Hz, floor at 0 Hz")
    check(last_int("TX: 12 sent  3 fail  tx=idle", TX_FAIL_RE) == 3,
          "TX fail count parses")

    # Board choice. No port opens: the config, the port list and the serial
    # lookup are stubs.
    check(ROCKETCHIP_USB_PID == 0x0009 and ROCKETCHIP_USB_VID == 0x2E8A,
          "USB VID:PID come from _rc_test_common (2E8A:0009)")
    cfg = {
        "AAAA": board_config.Board("AAAA", "vehicle"),
        "BBBB": board_config.Board("BBBB", "station"),
    }
    ports = {"AAAA": "COM_V", "BBBB": "COM_S"}
    ns = argparse.Namespace(vehicle_port=None, station_port=None,
                            vehicle_serial=None, station_serial=None)
    got = resolve_boards(ns, cfg_fn=lambda: cfg, ports_fn=lambda: ports,
                         serial_of=lambda p: {"COM_V": "AAAA", "COM_S": "BBBB"}.get(p))
    check(isinstance(got, dict) and got["vehicle"].port == "COM_V"
          and got["station"].port == "COM_S",
          "boards: boards.local.json serials pick the ports")
    got = resolve_boards(ns, cfg_fn=lambda: cfg, ports_fn=lambda: {"AAAA": "COM_V"},
                         serial_of=lambda p: "AAAA")
    check(isinstance(got, str) and "BBBB" in got and "not attached" in got,
          "boards: an unattached listed board is an error")
    ns2 = argparse.Namespace(vehicle_port=None, station_port=None,
                             vehicle_serial="bbbb", station_serial="aaaa")
    got = resolve_boards(ns2, cfg_fn=lambda: cfg, ports_fn=lambda: ports,
                         serial_of=lambda p: {"COM_V": "AAAA", "COM_S": "BBBB"}.get(p))
    check(isinstance(got, dict) and got["vehicle"].port == "COM_S",
          "boards: --vehicle-serial beats boards.local.json")
    got = resolve_boards(ns, cfg_fn=lambda: {"AAAA": cfg["AAAA"]}, ports_fn=lambda: ports,
                         serial_of=lambda p: None)
    check(isinstance(got, str) and "no station board" in got,
          "boards: a role missing from boards.local.json is an error")
    calls: list[dict] = []

    def fake_banner(**kw):
        calls.append(kw)
        return "COM_V", "COM_S", None, None, ""

    got = resolve_boards(ns, cfg_fn=lambda: None, ports_fn=lambda: {},
                         serial_of=lambda p: None, banner_fn=fake_banner)
    check(isinstance(got, dict) and got["station"].port == "COM_S" and len(calls) == 1,
          "boards: without a config the banner peek picks the pair")

    passed = sum(1 for ok, _ in checks if ok)
    total = len(checks)
    print(f"RESULT: {passed}/{total} {'PASS' if passed == total else 'FAIL'}")
    return 0 if passed == total else 1


def parse_args(argv: list[str]) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Two-board radio-link bench (power, round trip, forced TX "
                    "timeout, preset hop and back)."
    )
    ap.add_argument("--commit", default=None,
                    help="commit sha; accepted for pre_push_gate.py --commit")
    ap.add_argument("--vehicle-port", default=None)
    ap.add_argument("--station-port", default=None)
    ap.add_argument("--vehicle-serial", default=None,
                    help="vehicle USB serial (default: boards.local.json)")
    ap.add_argument("--station-serial", default=None,
                    help="station USB serial (default: boards.local.json)")
    ap.add_argument("--include-125", action="store_true",
                    help="name the 125 kHz transits on the hop-back path as steps")
    ap.add_argument("--max-runtime", type=float, default=120.0,
                    help="overall deadline in seconds (default 120)")
    ap.add_argument("--log-dir", default=None,
                    help="log directory (default <repo>/logs/radio_link_bench)")
    ap.add_argument("--self-check", action="store_true",
                    help="catalog, key-guard and parser checks only; opens no ports")
    ap.add_argument("--key-gap", type=float, default=0.45,
                    help="seconds between station/vehicle CLI keys")
    return ap.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.self_check:
        return run_self_check()

    if args.commit:
        print(f"commit: {args.commit}", flush=True)

    t_start = time.monotonic()
    deadline = t_start + args.max_runtime

    log_dir = args.log_dir or os.path.normpath(os.path.join(_HERE, "..", "logs", "radio_link_bench"))
    os.makedirs(log_dir, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y-%m-%d_%H%M%S")
    veh_path = os.path.join(log_dir, f"{stamp}_veh.txt")
    stn_path = os.path.join(log_dir, f"{stamp}_stn.txt")

    picks = resolve_boards(args)
    if isinstance(picks, str):
        print(f"STEP open-ports: FAIL  {picks}", flush=True)
        print("RESULT: 0/1 FAIL", flush=True)
        return 2
    veh_port, stn_port = picks["vehicle"].port, picks["station"].port
    print(picks["vehicle"].how, flush=True)
    print(picks["station"].how, flush=True)
    print(f"  vehicle {veh_port} -> {veh_path}", flush=True)
    print(f"  station {stn_port} -> {stn_path}", flush=True)

    veh_ser = None
    stn_ser = None
    veh = None
    stn = None
    try:
        try:
            veh_ser = open_cdc(veh_port)
            print(f"  {veh_port} open ok", flush=True)
        except (serial.SerialException, OSError, PermissionError) as exc:
            print(f"STEP open-ports: FAIL  vehicle {veh_port}: {exc}", flush=True)
            print("RESULT: 0/1 FAIL", flush=True)
            return 2
        try:
            stn_ser = open_cdc(stn_port)
            print(f"  {stn_port} open ok", flush=True)
        except (serial.SerialException, OSError, PermissionError) as exc:
            print(f"STEP open-ports: FAIL  station {stn_port}: {exc}", flush=True)
            print("RESULT: 0/1 FAIL", flush=True)
            return 2

        print(f"board_usb_serial: {picks['vehicle'].serial or 'unknown'}", flush=True)
        print(f"board_usb_serial: {picks['station'].serial or 'unknown'}", flush=True)

        veh = PortLogger("veh", veh_ser, veh_path)
        stn = PortLogger("stn", stn_ser, stn_path)
        veh.start()
        stn.start()
        bench = Bench(veh=veh, stn=stn, deadline=deadline, key_gap_s=args.key_gap)
        bench.record("open-ports", True, f"vehicle {veh_port}, station {stn_port}")

        rc = run_bench(bench, args)
    finally:
        time.sleep(0.3)
        if veh is not None:
            veh.stop()
        if stn is not None:
            stn.stop()
        for ser in (veh_ser, stn_ser):
            if ser is not None:
                try:
                    ser.close()
                except OSError:
                    pass

    elapsed = time.monotonic() - t_start
    print(f"elapsed: {elapsed:.1f}s (budget {args.max_runtime:.0f}s)", flush=True)
    return rc

def run_bench(bench: Bench, args: argparse.Namespace) -> int:
    # ------------------------------------------------------------------
    # Step: banners (no b / no k). Then link up from live counters.
    # ------------------------------------------------------------------
    def banners_ok() -> bool:
        v = bench.veh.snapshot_text()
        s = bench.stn.snapshot_text()
        veh_ok = bool(BANNER_RE.search(v) or "RocketChip" in v)
        stn_ok = bool(
            BANNER_RE.search(s)
            or "RocketChip Ground Station" in s
            or "RocketChip" in s
        )
        return veh_ok and stn_ok

    if not bench.wait(10.0, banners_ok, "connect banners on both boards"):
        v = bench.veh.snapshot_text()
        s = bench.stn.snapshot_text()
        bench.record(
            "link-up",
            False,
            "no RocketChip connect banner on "
            + ("vehicle" if "RocketChip" not in v else "station")
            + " (read banner/help, not CLI b)",
        )
        return finish(bench, 2)

    v_banner = BANNER_RE.search(bench.veh.snapshot_text())
    s_banner = BANNER_RE.search(bench.stn.snapshot_text())
    if v_banner:
        bench.log(f"  vehicle banner: RocketChip {v_banner.group(1)} "
                  f"{v_banner.group(2)}-{v_banner.group(3)}")
    if s_banner:
        bench.log(f"  station banner: RocketChip {s_banner.group(1)} "
                  f"{s_banner.group(2)}-{s_banner.group(3)}")
    v_up = last_int(bench.veh.snapshot_text(), UPTIME_RE)
    if v_up is not None:
        bench.log(f"  vehicle Uptime: {v_up}s (banner, not CLI b)")

    # Desk power rule on both boards. A wrong or unread value fails the run.
    # CLI t takes the station off the pad, so go back before link-up.
    if not step_power(bench, "power-2dBm"):
        return finish(bench, 1)
    if not _return_to_pad(bench):
        bench.record("link-up", False,
                     "station did not return to the ANSI pad after CLI t")
        return finish(bench, 1)

    # Baseline dashboard pkts, then wait for increment and/or COP-P lock.
    time.sleep(min(1.0, bench.cap(1.0)))
    s0 = bench.stn.snapshot_text()
    pkts0 = last_pkts(s0)
    # Vehicle t for TX / COP-P / CFG (t is radio status, not b).
    vt0 = bench.vehicle_t()
    tx0 = last_int(vt0, TX_SENT_RE)
    cfg0 = last_cfg(vt0) or last_cfg(bench.veh.snapshot_text())
    ident = ID_FLIGHT_RE.search(vt0) or ID_FLIGHT_RE.search(bench.veh.snapshot_text())
    if ident:
        bench.log(f"  vehicle ID: flight-{ident.group(1)} (CLI t, not b)")
    if cfg0:
        bench.log(f"  vehicle CFG at start: BW={cfg0[0]} nav={cfg0[1]}Hz pwr={cfg0[2]}dBm")

    def link_live() -> bool:
        s = bench.stn.snapshot_text()
        pkts = last_pkts(s)
        stn_ok = has_copp_lock(s) or (
            pkts is not None and pkts0 is not None and pkts > pkts0
        ) or (pkts is not None and pkts0 is None and pkts > 0)
        # Vehicle TX may increment without another t if we take a second sample.
        return stn_ok

    if not bench.wait(20.0, link_live, "station Pkts increment or COP-P lock"):
        # One more vehicle t before declaring fail, in case TX was zero at first.
        vt = bench.vehicle_t()
        tx = last_int(vt, TX_SENT_RE)
        s = bench.stn.snapshot_text()
        bench.record(
            "link-up",
            False,
            f"station Pkts={last_pkts(s)} (start {pkts0}), "
            f"COP-P lock={has_copp_lock(s)}, vehicle TX={tx}",
        )
        return finish(bench, 1)

    # Confirm vehicle TX as the other end of the link (second t if needed).
    time.sleep(min(2.0, bench.cap(2.0)))
    vt1 = bench.vehicle_t()
    tx1 = last_int(vt1, TX_SENT_RE)
    v_lock = has_copp_lock(vt1) or has_copp_lock(bench.veh.snapshot_text())
    s1 = bench.stn.snapshot_text()
    pkts1 = last_pkts(s1)
    stn_lock = has_copp_lock(s1)
    veh_live = v_lock or (tx0 is not None and tx1 is not None and tx1 > tx0) or (
        tx1 is not None and tx1 > 0
    )
    if not veh_live:
        bench.record(
            "link-up",
            False,
            f"vehicle TX={tx1} (start {tx0}), COP-P lock={v_lock}; "
            f"station Pkts={pkts1} COP-P lock={stn_lock}",
        )
        return finish(bench, 1)
    bench.record(
        "link-up",
        True,
        f"station Pkts={pkts1} (was {pkts0}) COP-P lock={stn_lock}; "
        f"vehicle TX={tx1} (was {tx0}) COP-P lock={v_lock}",
    )

    # ------------------------------------------------------------------
    # Step: short round trip (telemetry + uplink ACK via pad D / DISARM)
    # ------------------------------------------------------------------
    if not bench.stn_on_pad():
        # Should still be on the pad at this point; if we already left it,
        # we cannot send D without being on the pad. Do not send X (DISARM
        # on the menu). Fail this substep rather than risk X.
        if not _return_to_pad(bench):
            bench.record(
                "round-trip",
                False,
                "station is not on the ANSI pad; cannot send D (DISARM) "
                "without using menu X, which this script will not send",
            )
            return finish(bench, 1)

    pkts_a = last_pkts(bench.stn.snapshot_text())
    time.sleep(min(OBSERVE_S, bench.cap(OBSERVE_S)))
    s_telem = bench.stn.snapshot_text()
    pkts_b = last_pkts(s_telem)
    telem_ok = (
        (pkts_a is not None and pkts_b is not None and pkts_b > pkts_a)
        or has_copp_lock(s_telem)
    )
    if not telem_ok:
        bench.record(
            "round-trip",
            False,
            f"station telemetry did not advance in the listen window "
            f"(Pkts {pkts_a} -> {pkts_b})",
        )
        return finish(bench, 1)

    mark_ack = bench.stn.mark()
    # Capital D only, and only while the pad owns keys (poll_dashboard_keys).
    bench.send_stn("D", after=0.2)
    ack_ok = bench.wait(
        10.0,
        lambda: bool(ACK_CMD_RE.search(bench.stn.snapshot_from(mark_ack)))
        or bool(ACK_MS_RE.search(bench.stn.snapshot_from(mark_ack))),
        "[CMD] ACK'd or dashboard ACK <ms> after pad D",
    )
    tail = bench.stn.snapshot_from(mark_ack)
    if NO_ACK_RE.search(tail):
        ack_ok = False
    if not ack_ok:
        bench.record(
            "round-trip",
            False,
            "telemetry advanced, but no uplink ACK after pad D (DISARM). "
            f"Pkts {pkts_a} -> {pkts_b}. QUERY_RADIO_CONFIG has no CLI key.",
        )
        return finish(bench, 1)
    ack_ms = last_int(tail, ACK_MS_RE)
    ack_seq = last_int(tail, ACK_CMD_RE)
    detail = f"station Pkts {pkts_a} -> {pkts_b}"
    if ack_seq is not None:
        detail += f", [CMD] ACK'd (seq={ack_seq})"
    if ack_ms is not None:
        detail += f", ACK {ack_ms}ms"
    bench.record("round-trip", True, detail)

    # ------------------------------------------------------------------
    # Step: forced radio HW TX timeout on the vehicle (debug o)
    # ------------------------------------------------------------------
    if not step_forced_tx_timeout(bench):
        return finish(bench, 1)

    # ------------------------------------------------------------------
    # Step: hop 250/10 -> 500/10, then hop back to 250/10
    # ------------------------------------------------------------------
    current = _read_vehicle_preset(bench) or (cfg0[:2] if cfg0 else None)
    if current is None:
        bench.record("hop-500/10", False, "could not read vehicle CFG via CLI t")
        return finish(bench, 1)
    if current != DEFAULT_PRESET:
        bench.log(
            f"  start preset is BW{current[0]} {current[1]}Hz, not 250/10; "
            f"hopping forward to 250/10 first"
        )
        if not _hop_until(bench, current, DEFAULT_PRESET, "setup-250/10",
                          name_125=args.include_125, setup=True):
            return finish(bench, 1)
        current = DEFAULT_PRESET

    if not _one_hop(bench, current, TARGET_HOP, "hop-500/10"):
        return finish(bench, 1)
    current = TARGET_HOP

    back_path = hop_path(TARGET_HOP, DEFAULT_PRESET)
    # back_path ends at 250/10 and includes 125 transits.
    if not _follow_path(
        bench,
        current,
        back_path,
        step_name="hop-back-250/10",
        name_125=args.include_125,
    ):
        return finish(bench, 1)

    # Power again after the hops: a hop applies a new radio config.
    step_power(bench, "power-2dBm-after-hop")
    _return_to_pad(bench)

    if bench.remain() <= 0:
        bench.record(
            "deadline", False,
            f"overall deadline ({args.max_runtime:.0f}s) was reached; the run "
            f"is not complete",
        )

    return finish(bench, 0 if all(s.ok for s in bench.steps) else 1)


def _return_to_pad(bench: Bench) -> bool:
    """Best-effort return to the ANSI pad via station main z. Never sends X."""
    if bench.stn_on_pad():
        return True
    if not bench.return_station_main():
        return False
    mark = bench.stn.mark()
    bench.send_stn("z")  # station main z is "pad" (kReturnPad)
    return bench.wait(
        6.0,
        lambda: "RocketChip Ground Station" in bench.stn.snapshot_from(mark)
        or "'x' menu" in bench.stn.snapshot_from(mark),
        "ANSI pad after main z",
    )


def _read_vehicle_preset(bench: Bench) -> tuple[int, int] | None:
    text = bench.vehicle_t()
    cfg = last_cfg(text) or last_cfg(bench.veh.snapshot_text())
    if cfg is None:
        return None
    return cfg[0], cfg[1]


def _one_hop(bench: Bench, src: tuple[int, int], dest: tuple[int, int],
             name: str) -> bool:
    """Press station settings n once and require dest plus link resume."""
    if not bench.enter_station_settings():
        bench.record(name, False, "could not open station settings menu (x then s)")
        return False

    v_mark = bench.veh.mark()
    s_mark = bench.stn.mark()
    v_tx_before = last_int(bench.veh.snapshot_text(), TX_SENT_RE)
    s_rx_before = last_pkts(bench.stn.snapshot_text())

    bench.send_stn("n")
    # Station accept line, then remote armed / remote apply / ok.
    def hop_ok() -> bool:
        s = bench.stn.snapshot_from(s_mark)
        v = bench.veh.snapshot_from(v_mark)
        if "NAV_PRESET COMM_CHANGE refused" in s or "NAV_PRESET: no catalog" in s:
            return False
        accept = last_hop_accept(s)
        if accept is not None and (accept[0], accept[1]) != dest:
            # Wrong preset - do not treat as success.
            return False
        armed = "[SC] COMM_CHANGE remote armed" in v
        apply = "[SC] COMM_CHANGE remote apply" in v
        ok = "[SC] COMM_CHANGE ok" in s
        return bool(accept) and armed and apply and ok

    def hop_refused() -> bool:
        s = bench.stn.snapshot_from(s_mark)
        return "NAV_PRESET COMM_CHANGE refused" in s or "NAV_PRESET: no radio" in s

    reached = bench.wait(15.0, hop_ok, f"remote armed / remote apply / ok for {dest}")
    s_tail = bench.stn.snapshot_from(s_mark)
    v_tail = bench.veh.snapshot_from(v_mark)
    if hop_refused():
        bench.record(name, False, "station refused COMM_CHANGE (see NAV_PRESET line)")
        return False
    accept = last_hop_accept(s_tail)
    armed = "[SC] COMM_CHANGE remote armed" in v_tail
    apply = "[SC] COMM_CHANGE remote apply" in v_tail
    ok_line = "[SC] COMM_CHANGE ok" in s_tail
    if "COMM_CHANGE no echo" in s_tail or "COMM_CHANGE revert hail" in s_tail:
        bench.record(
            name,
            False,
            "station reported no echo / revert hail during this hop "
            f"(armed={armed} apply={apply} ok={ok_line} accept={accept})",
        )
        return False
    if not reached:
        bench.record(
            name,
            False,
            f"missing hop lines for BW{dest[0]} {dest[1]}Hz: "
            f"accept={accept} remote armed={armed} remote apply={apply} "
            f"ok={ok_line}",
        )
        return False

    # Link resume + CFG on both sides. Need main menu for t.
    # Sample every ~1.5 s - do not send t on the 50 ms wait poll.
    time.sleep(min(2.0, bench.cap(2.0)))
    ok_resume = False
    resume_deadline = time.monotonic() + bench.cap(15.0)
    last_v = last_s = None
    while time.monotonic() < resume_deadline:
        vt2 = bench.vehicle_t()
        st2 = bench.station_t()
        last_v = last_cfg(vt2) or last_cfg(bench.veh.snapshot_text())
        last_s = last_cfg(st2) or last_cfg(bench.stn.snapshot_text())
        v_match = last_v is not None and (last_v[0], last_v[1]) == dest
        s_match = last_s is not None and (last_s[0], last_s[1]) == dest
        v_tx = last_int(vt2, TX_SENT_RE)
        s_rx = last_int(st2, RX_PKTS_RE) or last_pkts(bench.stn.snapshot_text())
        v_live = has_copp_lock(vt2) or (
            v_tx_before is not None and v_tx is not None and v_tx > v_tx_before
        )
        s_live = has_copp_lock(st2) or (
            s_rx_before is not None and s_rx is not None and s_rx > s_rx_before
        )
        if v_match and s_match and v_live and s_live:
            ok_resume = True
            break
        time.sleep(min(1.5, max(0.0, resume_deadline - time.monotonic())))
    if not ok_resume:
        bench.log(f"  timeout waiting for link resume on BW{dest[0]} {dest[1]}Hz")
    vt = last_v or last_cfg(bench.veh.snapshot_text())
    st = last_s or last_cfg(bench.stn.snapshot_text())
    if not ok_resume:
        bench.record(
            name,
            False,
            f"hop lines seen ({src} -> {dest}) but link did not resume or CFG "
            f"mismatch (vehicle={vt} station={st})",
        )
        return False
    bench.record(
        name,
        True,
        f"{src[0]}/{src[1]} -> {dest[0]}/{dest[1]}: "
        f"NAV_PRESET COMM_CHANGE, remote armed, remote apply, ok; "
        f"vehicle CFG={vt} station CFG={st}; link resumed",
    )
    return True


def _follow_path(
    bench: Bench,
    current: tuple[int, int],
    path: list[tuple[int, int]],
    step_name: str,
    name_125: bool,
    setup: bool = False,
) -> bool:
    cur = current
    transit_ok = True
    named_fail = False
    for dest in path:
        is_125 = dest[0] == 125
        if name_125 and is_125:
            hop_name = f"{'setup-' if setup else ''}hop-{dest[0]}/{dest[1]}"
        elif setup and dest == path[-1]:
            hop_name = f"setup-{dest[0]}/{dest[1]}"
        else:
            hop_name = f"transit-{dest[0]}/{dest[1]}"
        ok = _one_hop(bench, cur, dest, hop_name)
        if not ok:
            transit_ok = False
            named_fail = True
            break
        cur = dest
    if named_fail:
        return False
    # If 125 hops were recorded as their own STEPs, still record hop-back.
    last = path[-1] if path else current
    if step_name not in {s.name for s in bench.steps}:
        # If we already recorded hop-back via the loop, skip. Otherwise
        # summarise. When name_125, the loop used transit-/hop- names, so
        # we still want the umbrella hop-back step.
        bench.record(
            step_name,
            transit_ok,
            f"catalog path {' -> '.join(f'{a}/{b}' for a, b in [current, *path])}",
        )
    return transit_ok and cur == last


def _hop_until(
    bench: Bench,
    current: tuple[int, int],
    dest: tuple[int, int],
    name: str,
    name_125: bool,
    setup: bool,
) -> bool:
    try:
        path = hop_path(current, dest)
    except (ValueError, RuntimeError) as exc:
        bench.record(name, False, str(exc))
        return False
    return _follow_path(bench, current, path, name, name_125, setup=setup)


def finish(bench: Bench, code: int) -> int:
    passed = sum(1 for s in bench.steps if s.ok)
    total = len(bench.steps)
    # Hidden transit names still count; they are real steps.
    if any(s.ok is False for s in bench.steps) and code == 0:
        code = 1
    if code == 0 and total > 0 and passed == total:
        print(f"RESULT: {passed}/{total} PASS", flush=True)
        return 0
    print(f"RESULT: {passed}/{total} FAIL", flush=True)
    for s in bench.steps:
        if s.ok is False:
            print(f"  failed {s.name}: {s.detail}", flush=True)
    return code if code != 0 else 1


if __name__ == "__main__":
    sys.exit(main())
