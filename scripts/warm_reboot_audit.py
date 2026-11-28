#!/usr/bin/env python3
"""
RocketChip Warm-Reboot Audit (R-22)

R-25-exec step 11 (2026-05-13): redesigned against the single flight
binary per council-APPROVED Approach A
(docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md). The 4-iteration
draft + council verdict captured at
`C:\\Users\\pow-w\\.claude\\plans\\snoopy-wibbling-noodle.md` is the
authoritative design; this implementation inherits the 7 council
amendments (G-W1 counter-increment positive control, G-W4 same,
chip-serial emission, operator runbook, downgrade-G-W3-to-soft framing,
follow-on PR log, explicit Rule-4 labeling).

Would have caught the SIO_FIFO_IRQ wedge that R-19 fixed if it had
existed in earlier cycles.

Gates (per HW_GATE_DISCIPLINE.md Rule 1 positive-control requirement):

  G-W1 (AIRCR via probe) — HARD with Core-1 counter-increment check
    Probe writes AIRCR -> verified reset (below) -> open menu ->
    Debug submenu 's' twice (>=1.2s apart) -> IMU/baro counters
    must advance + Hardware Status 14/14 OK. AIRCR is per-processor
    on RP2350 -- banner alone proves Core 0 only; the counter
    check is the actual Core-1 alive signal (R-19 wedge scenario).

  G-W2 (3x probe reset run) — HARD
    Probe `monitor reset run` x3, each a verified reset -> peek_banner ->
    Banner identity stable across 3 boots. Probe path resets both
    cores cleanly.

  G-W3 (Nx picotool burst) — HARD
    picotool reboot -f --ser <serial> x N, each a verified reset ->
    final peek_banner.
    Risk #3: picotool itself can flake (non-zero exit, device not
    found, timeout). That FAIL is a tool failure; the operator may record
    G-W3 as soft for that run. The script never downgrades on its own,
    and Risk #3 does not cover a reboot command that exits 0 without a
    reset, a port that does not come back, or an unchanged boot marker:
    those are FAIL.

  G-W4 (compound AIRCR + picotool) — HARD with G-W1 counter check
    G-W1 sequence + 2x G-W3 reboots, each a verified reset -> final
    peek_banner.

Verified reset (every gate): a reset command's exit status does not
prove a reset. `picotool reboot -f` has exited 0 with the app still
running (Tiny 2350, 2026-11-21: heartbeats kept counting for 16 s), and
a "port is present" poll passes on a board that never left. A reboot
counts only when
  (a) the board's own CDC port (followed by USB serial) is seen to
      vanish within a bounded time after the command, then return, and
  (b) the connect banner's boot marker, "Uptime: Ns"
      (cli_print_boot_summary, src/cli/rc_os_commands.cpp), shows a boot
      after the command: lower than the pre-command reading plus the
      elapsed time, and no more than the time since the command.
Anything else is a FAIL with the reason printed.

Uses peek_banner from _rc_test_common.py for every banner read
(proven Windows USB CDC survival pattern: thread-with-join-timeout,
OSError-22 tolerance). watchdog_s=300 self-kills on wedge.

Usage:
    python scripts/warm_reboot_audit.py [--chip-serial SERIAL]
        [--burst N] [--skip-aircr] [--skip-picotool] [--verbose]
        [--watchdog-s N]

Exit codes:
    0 = all enabled gates passed
    1 = one or more gate failures
    2 = setup error / watchdog fired / probe not available
"""

import argparse
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Callable, Optional

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from _rc_test_common import (  # noqa: E402
    Banner,
    Role,
    arm_test_mode_via_probe,
    banner_uptime,
    board_usb_serial,
    peek_banner,
    open_classified_port,
    enter_cli_menu,
    find_target_port,
    rc_test,
    TARGET_VEHICLE_ANY,
    TARGET_STATION_ANY,
    TARGET_EITHER_ANY,
    ROCKETCHIP_USB_VID,
    ROCKETCHIP_USB_PID,
)

GDB = r'C:\Users\pow-w\.pico-sdk\toolchain\14_2_Rel1\bin\arm-none-eabi-gdb.exe'
OPENOCD_PORT = 3333
PICOTOOL = r'C:\Users\pow-w\.pico-sdk\picotool\2.2.0\picotool\picotool.exe'

# Default knobs
DEFAULT_BURST = 5
PROBE_RESET_SETTLE_S = 3.0   # post-AIRCR re-enum window
PICOTOOL_RESET_SETTLE_S = 4.0  # post-picotool re-enum window
GDB_TIMEOUT_S = 20.0          # one gdb -batch run
PICOTOOL_TIMEOUT_S = 15.0     # one picotool reboot run
# Verified reset (see module docstring).
PORT_POLL_S = 0.1             # port-list poll while the command runs
PORT_GONE_AFTER_CMD_S = 5.0   # port must vanish this soon after the command exits
PORT_RETURN_TIMEOUT_S = 15.0  # port must return this soon after it vanished
# Without a reset the post-command reading is more than
# before + elapsed - 1 s (banner resolution) - read latency, so a 3 s
# tolerance covers up to 2 s of latency between the firmware printing
# "Uptime: Ns" and banner_uptime returning it.
UPTIME_TOL_S = 3
# A reset is only distinguishable once the board has been up longer than
# the tolerance plus 1 s resolution plus latency on the earlier reading.
MIN_UPTIME_BEFORE_S = UPTIME_TOL_S + 3
MARKER_READ_TRIES = 3         # banner_uptime has no retry of its own
MARKER_RETRY_S = 0.6          # Windows CDC release race (see peek_banner)
COUNTER_DELTA_WINDOW_S = 1.2  # >=1.2s between counter reads
IMU_DELTA_MIN = 100           # Core 1 IMU runs at ~1kHz -> ~1000 in 1.2s
BARO_DELTA_MIN = 5            # Baro runs at ~50Hz -> ~60 in 1.2s


# ============================================================================
# Probe + serial helpers
# ============================================================================

def _gdb_args(*ex_cmds) -> list:
    args = [GDB, '-batch',
            '-ex', f'target extended-remote localhost:{OPENOCD_PORT}']
    for c in ex_cmds:
        args.extend(['-ex', c])
    return args


def _gdb_run(*ex_cmds, timeout=GDB_TIMEOUT_S):
    try:
        r = subprocess.run(_gdb_args(*ex_cmds), capture_output=True,
                           text=True, timeout=timeout)
        return r.returncode == 0, r.stdout + r.stderr
    except subprocess.TimeoutExpired:
        return False, '<GDB TIMEOUT>'


def _probe_listening() -> bool:
    """OpenOCD on :3333 — quick connect probe."""
    ok, _ = _gdb_run('monitor halt', timeout=5.0)
    return ok


def _popen(args: list) -> subprocess.Popen:
    """Start a reset command without waiting, so the port can be watched
    while it runs (picotool -f drops and restores the CDC port before it
    exits)."""
    return subprocess.Popen(args, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True)


def _start_probe_reset_run() -> subprocess.Popen:
    """`monitor reset run` — resets both cores via bootrom."""
    return _popen(_gdb_args('monitor reset run'))


def _start_probe_aircr_reset() -> subprocess.Popen:
    """Write 0x05FA0004 to AIRCR (SYSRESETREQ) via probe."""
    # AIRCR = 0xE000ED0C; SYSRESETREQ bit = 2; VECTKEY = 0x05FA0000.
    return _popen(_gdb_args(
        'monitor halt',
        'monitor mww 0xE000ED0C 0x05FA0004',
        'monitor resume',
    ))


def _start_picotool_reboot(chip_serial: str) -> subprocess.Popen:
    return _popen([PICOTOOL, 'reboot', '-f', '--ser', chip_serial])


def _detect_vehicle_chip_serial() -> str:
    """Read picotool info -a output, extract serial of attached RP2350."""
    try:
        r = subprocess.run(
            [PICOTOOL, 'info', '-a'],
            capture_output=True, text=True, timeout=10.0,
        )
        for ln in r.stdout.splitlines():
            ln = ln.strip()
            if ln.startswith('flash id:'):
                return ln.split(':', 1)[1].strip()
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass
    return ''


# ============================================================================
# Verified reset: port gone then back, and a boot marker that moved
# ============================================================================

# Host clock and sleep for the reset checks (replaced by the unit tests).
_clock = time.monotonic
_sleep = time.sleep


@dataclass(frozen=True)
class BootMarker:
    """One boot-marker reading: the connect banner's "Uptime: Ns"."""
    uptime_s: int
    t: float         # host monotonic clock when the line was read


@dataclass
class Board:
    """The board under test. Followed across resets by its USB serial
    (the COM name can change on re-enumeration); by port name only when
    the port reports no serial. `marker` is the latest boot-marker
    reading, None when unknown."""
    port: str
    usb_serial: Optional[str]
    marker: Optional[BootMarker] = None


# ResetWatch.kind values.
RESET_SEEN = 'reset seen'
RESET_CMD_FAILED = 'command failed'   # tool failure (Risk #3 for picotool)
RESET_NONE = 'no reset'               # port never left
RESET_NO_RETURN = 'no return'         # port left, did not come back
RESET_ABSENT = 'absent'               # port not there before the command


@dataclass(frozen=True)
class ResetWatch:
    kind: str
    reason: str
    port: Optional[str]       # the board's port after it came back
    t_cmd: float              # host clock when the command started
    gone_s: Optional[float]   # command start -> port gone
    back_s: Optional[float]   # port gone -> port back

    @property
    def ok(self) -> bool:
        return self.kind == RESET_SEEN


def _list_comports() -> list:
    import serial.tools.list_ports
    return list(serial.tools.list_ports.comports())


def make_locator(board: Board,
                 comports: Optional[Callable[[], list]] = None
                 ) -> Callable[[], Optional[str]]:
    """Enumeration only (never opens a port): the board's port name now,
    or None when it is not on the bus. Only RocketChip VID:PID ports
    count, so the board in BOOTSEL (vendor interface, no CDC) is absent,
    and a second attached board is never mistaken for this one."""
    comports = comports or _list_comports

    def locate() -> Optional[str]:
        for p in comports():
            if (getattr(p, 'vid', None) != ROCKETCHIP_USB_VID or
                    getattr(p, 'pid', None) != ROCKETCHIP_USB_PID):
                continue
            if board.usb_serial:
                if (getattr(p, 'serial_number', None) or '').upper() == \
                        board.usb_serial.upper():
                    return p.device
            elif p.device == board.port:
                return p.device
        return None
    return locate


def _cmd_output_tail(proc) -> str:
    try:
        out = proc.communicate(timeout=2.0)[0] or ''
    except Exception:  # noqa: BLE001 -- diagnostics only
        return ''
    lines = [ln.strip() for ln in out.splitlines() if ln.strip()]
    return f' ({lines[-1]})' if lines else ''


def watch_reset(start: Callable[[], object],
                locate: Callable[[], Optional[str]],
                cmd_timeout_s: float,
                gone_after_cmd_s: float = PORT_GONE_AFTER_CMD_S,
                return_timeout_s: float = PORT_RETURN_TIMEOUT_S,
                poll_s: float = PORT_POLL_S,
                clock: Optional[Callable[[], float]] = None,
                sleep: Optional[Callable[[float], None]] = None) -> ResetWatch:
    """Start a reset command and watch the board's port while it runs and
    after. `start()` returns a Popen-like object (poll / kill /
    communicate). A reset is seen only when the command exits 0, the port
    vanished (during the command or within gone_after_cmd_s after it
    exits), and it then came back within return_timeout_s of vanishing.
    A port enumeration that raises is skipped, not read as absent."""
    clock = clock or _clock
    sleep = sleep or _sleep

    def fail(kind, reason, t_cmd, gone_s=None):
        return ResetWatch(kind, reason, None, t_cmd, gone_s, None)

    try:
        before = locate()
    except Exception:  # noqa: BLE001 -- enumeration failure
        before = None
    if before is None:
        return fail(RESET_ABSENT, 'board port not present before the command',
                    clock())
    t_cmd = clock()
    try:
        proc = start()
    except OSError as e:
        return fail(RESET_CMD_FAILED, f'command did not start ({e})', t_cmd)
    rc = None
    t_done = None
    gone_at = None
    back_at = None
    port = None
    while True:
        now = clock()
        if rc is None:
            rc = proc.poll()
            if rc is not None:
                t_done = now
                if rc != 0:
                    return fail(RESET_CMD_FAILED,
                                f'command exit {rc}{_cmd_output_tail(proc)}',
                                t_cmd)
            elif now - t_cmd > cmd_timeout_s:
                proc.kill()
                _cmd_output_tail(proc)   # reap
                return fail(RESET_CMD_FAILED,
                            f'command timed out after {cmd_timeout_s:.0f}s',
                            t_cmd)
        try:
            seen = locate()
            sampled = True
        except Exception:  # noqa: BLE001 -- skip this sample
            seen, sampled = None, False
        if sampled:
            if seen is None:
                if gone_at is None:
                    gone_at = now
                back_at, port = None, None   # wait for the final return
            elif gone_at is not None and back_at is None:
                back_at, port = now, seen
        if rc == 0 and back_at is not None:
            return ResetWatch(RESET_SEEN, 'port gone, then back', port, t_cmd,
                              gone_at - t_cmd, back_at - gone_at)
        if gone_at is None and t_done is not None and \
                now - t_done > gone_after_cmd_s:
            return fail(RESET_NONE,
                        'no reset: command exited 0 but the board port never '
                        f'left the bus ({now - t_cmd:.1f}s watched)', t_cmd)
        if gone_at is not None and back_at is None and \
                now - gone_at > return_timeout_s:
            return fail(RESET_NO_RETURN,
                        f'board port left the bus and did not return within '
                        f'{return_timeout_s:.0f}s', t_cmd, gone_at - t_cmd)
        sleep(poll_s)


def classify_reset(before: BootMarker, t_cmd: float,
                   after: Optional[BootMarker],
                   tol_s: int = UPTIME_TOL_S) -> tuple:
    """(True, detail) if the boot marker shows a boot after the command,
    else (False, reason). Without a reset the uptime has grown by the
    elapsed time; a reset makes it lower than that, and no more than the
    time since the command (both within tol_s for the 1 s resolution and
    read latency)."""
    if after is None:
        return False, ('boot marker unreadable after the reset (no '
                       '"Uptime: Ns" in the connect banner)')
    elapsed = after.t - before.t
    if after.uptime_s + tol_s >= before.uptime_s + elapsed:
        return False, (f'no reset: boot marker unchanged (Uptime '
                       f'{before.uptime_s}s -> {after.uptime_s}s over '
                       f'{elapsed:.1f}s)')
    since_cmd = after.t - t_cmd
    if after.uptime_s > since_cmd + tol_s:
        return False, (f'boot predates the command (Uptime '
                       f'{after.uptime_s}s, command {since_cmd:.1f}s ago)')
    return True, f'Uptime {before.uptime_s}s -> {after.uptime_s}s'


def _read_boot_marker(port: str,
                      clock: Optional[Callable[[], float]] = None,
                      sleep: Optional[Callable[[float], None]] = None,
                      read: Optional[Callable[[str], Optional[int]]] = None
                      ) -> Optional[BootMarker]:
    """Fresh open, connect banner, "Uptime: Ns" (banner_uptime). Retried:
    an open right after another close can lose the Windows CDC release
    race. None if no reading (also a station in MAVLink output mode,
    which prints no banner)."""
    clock = clock or _clock
    sleep = sleep or _sleep
    read = read or banner_uptime
    for attempt in range(MARKER_READ_TRIES):
        u = read(port)
        if u is not None:
            return BootMarker(u, clock())
        if attempt < MARKER_READ_TRIES - 1:
            sleep(MARKER_RETRY_S)
    return None


def reboot_step(tag: str, what: str, board: Board,
                start: Callable[[], object], cmd_timeout_s: float,
                settle_s: float,
                cmd_fail_note: str = '',
                read_marker: Optional[Callable[[str], Optional[BootMarker]]] = None,
                comports: Optional[Callable[[], list]] = None,
                clock: Optional[Callable[[], float]] = None,
                sleep: Optional[Callable[[float], None]] = None) -> bool:
    """One verified reset. Prints FAIL with the reason and returns False
    unless the port was seen to leave and return and the boot marker
    moved. On success board.port / board.marker are the new boot's."""
    clock = clock or _clock
    sleep = sleep or _sleep
    read_marker = read_marker or (lambda p: _read_boot_marker(p, clock, sleep))
    if board.marker is None:
        board.marker = read_marker(board.port)
        if board.marker is None:
            print(f'[{tag}] FAIL: {what}: boot marker unreadable before the '
                  f'command (no "Uptime: Ns" in the connect banner on '
                  f'{board.port})')
            return False
    up_now = board.marker.uptime_s + (clock() - board.marker.t)
    if up_now < MIN_UPTIME_BEFORE_S:
        sleep(MIN_UPTIME_BEFORE_S - up_now)
    w = watch_reset(start, make_locator(board, comports), cmd_timeout_s,
                    clock=clock, sleep=sleep)
    if not w.ok:
        board.marker = None
        note = cmd_fail_note if w.kind == RESET_CMD_FAILED else ''
        print(f'[{tag}] FAIL: {what}: {w.reason}{note}')
        return False
    board.port = w.port
    sleep(settle_s)
    after = read_marker(board.port)
    ok, detail = classify_reset(board.marker, w.t_cmd, after)
    board.marker = after
    if not ok:
        print(f'[{tag}] FAIL: {what}: {detail}')
        return False
    print(f'[{tag}]   {what}: reset verified (port gone at '
          f'+{w.gone_s:.1f}s, back {w.back_s:.1f}s later on {board.port}; '
          f'{detail})')
    return True


# ============================================================================
# Core-1 alive positive-control: q->s + q->b read
# ============================================================================

def _read_sensor_counts(port: str, timeout_s: float = 6.0) -> dict:
    """Open the port, enter menu, send q->s, parse IMU/baro counts.
    Returns {'imu': int, 'baro': int, 'gps': int, 'mag': int} or {}."""
    import re
    import serial
    try:
        with open_classified_port(port, target=TARGET_EITHER_ANY) as ser:
            # Make sure we're at the main menu first
            enter_cli_menu(ser)
            time.sleep(0.1)
            # Enter Debug submenu, then 's' for sensor counts
            ser.write(b'q')
            time.sleep(0.2)
            ser.read(4096)  # drain menu banner
            ser.write(b's')
            time.sleep(0.4)
            buf = ser.read(8192).decode('utf-8', errors='replace')
            # Exit back to main menu
            ser.write(b'z')
            time.sleep(0.1)
    except (serial.SerialException, OSError):
        return {}
    m = re.search(
        r'Reads:\s+I=(\d+)\s+M=(\d+)\s+B=(\d+)\s+G=(\d+)', buf)
    if not m:
        return {}
    return {
        'imu': int(m.group(1)),
        'mag': int(m.group(2)),
        'baro': int(m.group(3)),
        'gps': int(m.group(4)),
    }


def _read_hw_status_ok(port: str) -> tuple:
    """Read the Hardware Status block (q->b), return (pass, total)
    parsed from 'Hardware: N/N OK' line, or (0, 0) on failure."""
    import re
    import serial
    try:
        with open_classified_port(port, target=TARGET_EITHER_ANY) as ser:
            enter_cli_menu(ser)
            time.sleep(0.1)
            ser.write(b'q')
            time.sleep(0.2)
            ser.read(8192)
            ser.write(b'b')
            time.sleep(0.5)
            buf = ser.read(16384).decode('utf-8', errors='replace')
            ser.write(b'z')
            time.sleep(0.1)
    except (serial.SerialException, OSError):
        return (0, 0)
    m = re.search(r'Hardware:\s+(\d+)/(\d+)\s+OK', buf)
    if not m:
        return (0, 0)
    return (int(m.group(1)), int(m.group(2)))


# ============================================================================
# Gates
# ============================================================================

def _baseline_banner(port: str) -> Banner:
    return peek_banner(port)


# Tool failure of picotool itself (not a no-reset outcome): Risk #3.
PICOTOOL_FAIL_NOTE = (' -- picotool tool failure; a Risk #3 soft downgrade '
                      'is an operator call, never automatic')


def _banner_matches(b: Banner, baseline: Banner) -> bool:
    return (b.role == baseline.role and b.build == baseline.build and
            b.version == baseline.version)


def _picotool_reboot_step(tag: str, i: int, board: Board,
                          chip_serial: str) -> bool:
    if not chip_serial:
        print(f'[{tag}] FAIL: picotool reboot #{i}: no chip serial')
        return False
    return reboot_step(tag, f'picotool reboot #{i}', board,
                       lambda: _start_picotool_reboot(chip_serial),
                       PICOTOOL_TIMEOUT_S, PICOTOOL_RESET_SETTLE_S,
                       cmd_fail_note=PICOTOOL_FAIL_NOTE)


def _gate_w1_aircr(board: Board, baseline: Banner, verbose: bool) -> bool:
    """G-W1: AIRCR reset, Core 1 still alive."""
    print('[G-W1] AIRCR reset via probe...')
    if not reboot_step('G-W1', 'AIRCR reset', board, _start_probe_aircr_reset,
                       GDB_TIMEOUT_S, PROBE_RESET_SETTLE_S):
        return False
    # Sample 1: counters at t0
    c1 = _read_sensor_counts(board.port)
    if not c1:
        print('[G-W1] FAIL: could not read sensor counts (sample 1)')
        return False
    if verbose:
        print(f'[G-W1]   sample 1: imu={c1["imu"]} baro={c1["baro"]}')
    time.sleep(COUNTER_DELTA_WINDOW_S)
    # Sample 2: counters at t1
    c2 = _read_sensor_counts(board.port)
    if not c2:
        print('[G-W1] FAIL: could not read sensor counts (sample 2)')
        return False
    if verbose:
        print(f'[G-W1]   sample 2: imu={c2["imu"]} baro={c2["baro"]}')
    imu_delta = c2['imu'] - c1['imu']
    baro_delta = c2['baro'] - c1['baro']
    if imu_delta < IMU_DELTA_MIN:
        print(f'[G-W1] FAIL: Core 1 IMU stuck (delta {imu_delta} < {IMU_DELTA_MIN})')
        return False
    if baro_delta < BARO_DELTA_MIN:
        print(f'[G-W1] FAIL: Core 1 baro stuck (delta {baro_delta} < {BARO_DELTA_MIN})')
        return False
    # Hardware Status check
    hw_pass, hw_total = _read_hw_status_ok(board.port)
    if hw_total == 0:
        print('[G-W1] FAIL: could not read Hardware Status block')
        return False
    if hw_pass != hw_total:
        print(f'[G-W1] FAIL: Hardware Status degraded ({hw_pass}/{hw_total})')
        return False
    print(f'[G-W1] PASS: Core 1 alive (imu+{imu_delta}, baro+{baro_delta}, '
          f'HW {hw_pass}/{hw_total} OK)')
    return True


def _gate_w2_probe_reset_x3(board: Board, baseline: Banner,
                            verbose: bool) -> bool:
    """G-W2: 3x probe reset run, banner stable."""
    print('[G-W2] 3x probe reset run...')
    for i in range(1, 4):
        if not reboot_step('G-W2', f'probe reset run #{i}', board,
                           _start_probe_reset_run, GDB_TIMEOUT_S, 2.0):
            return False
        b = peek_banner(board.port)
        if not b or not b.is_known():
            print(f'[G-W2] FAIL: banner unreadable after reset #{i}')
            return False
        if verbose:
            print(f'[G-W2]   boot #{i}: {b.short_summary()}')
        if not _banner_matches(b, baseline):
            print(f'[G-W2] FAIL: banner drift at reset #{i} '
                  f'(was {baseline.short_summary()}, now {b.short_summary()})')
            return False
    print('[G-W2] PASS: banner stable across 3 verified probe resets')
    return True


def _gate_w3_picotool_burst(board: Board, baseline: Banner, burst: int,
                            chip_serial: str, verbose: bool) -> bool:
    """G-W3: N x picotool reboot -f --ser, each a verified reset, banner
    stable."""
    print(f'[G-W3] {burst}x picotool reboot (--ser {chip_serial})...')
    for i in range(1, burst + 1):
        if not _picotool_reboot_step('G-W3', i, board, chip_serial):
            return False
        if verbose:
            b = peek_banner(board.port)
            if b and b.is_known():
                print(f'[G-W3]   boot #{i}: {b.short_summary()}')
    # Final banner compare
    final = peek_banner(board.port)
    if not final or not final.is_known():
        print('[G-W3] FAIL: final banner unreadable')
        return False
    if not _banner_matches(final, baseline):
        print(f'[G-W3] FAIL: banner drift post-burst '
              f'(was {baseline.short_summary()}, now {final.short_summary()})')
        return False
    print(f'[G-W3] PASS: banner stable after {burst} verified picotool reboots')
    return True


def _gate_w4_compound(board: Board, baseline: Banner, chip_serial: str,
                      verbose: bool) -> bool:
    """G-W4: G-W1 (AIRCR + counter check) + 2x picotool, each reset
    verified."""
    print('[G-W4] Compound: AIRCR + counter-check + 2x picotool...')
    if not _gate_w1_aircr(board, baseline, verbose):
        print('[G-W4] FAIL: G-W1 subsequence')
        return False
    # 2 picotool reboots
    for i in range(1, 3):
        if not _picotool_reboot_step('G-W4', i, board, chip_serial):
            return False
    final = peek_banner(board.port)
    if not final or not final.is_known():
        print('[G-W4] FAIL: final banner unreadable')
        return False
    if not _banner_matches(final, baseline):
        print(f'[G-W4] FAIL: banner drift post-compound '
              f'(was {baseline.short_summary()}, now {final.short_summary()})')
        return False
    print('[G-W4] PASS: compound sequence clean')
    return True


# ============================================================================
# Main
# ============================================================================

@rc_test(target=TARGET_EITHER_ANY, watchdog_s=300)
def main():
    parser = argparse.ArgumentParser(
        description='R-22 warm-reboot audit (R-25-exec redesign 2026-05-13)')
    parser.add_argument('--role', choices=['vehicle', 'station'],
                        default='vehicle',
                        help='Target role (default: vehicle). Station '
                             'auto-skips G-W1/G-W4 -- Core-1 counter '
                             'check has no station-side equivalent.')
    parser.add_argument('--elf', default=None,
                        help='ELF path for probe arming (default: '
                             'build_flight/rocketchip.elf for vehicle, '
                             'build_station_flight/rocketchip.elf for station)')
    parser.add_argument('--chip-serial', default=None,
                        help='Chip serial for picotool --ser '
                             '(auto-detected if omitted)')
    parser.add_argument('--burst', type=int, default=DEFAULT_BURST,
                        help=f'picotool burst count (default {DEFAULT_BURST})')
    parser.add_argument('--skip-aircr', action='store_true',
                        help='Skip G-W1 + G-W4 (AIRCR-related gates)')
    parser.add_argument('--skip-picotool', action='store_true',
                        help='Skip G-W3 + G-W4 (picotool-related gates)')
    parser.add_argument('--skip-arm', action='store_true',
                        help='Skip the test-mode probe-arm step '
                             '(use if firmware is already in test mode)')
    parser.add_argument('--verbose', action='store_true')
    parser.add_argument('--watchdog-s', type=int, default=300,
                        help='Wall-clock watchdog (default 300s)')
    args = parser.parse_args()

    # Tool availability check
    if not _probe_listening():
        print('SETUP ERROR: OpenOCD probe not listening on '
              f'127.0.0.1:{OPENOCD_PORT}. Start probe and retry.')
        sys.exit(2)

    # Role-specific target + ELF default
    if args.role == 'station':
        target = TARGET_STATION_ANY
        default_elf = 'build_station_flight/rocketchip.elf'
    else:
        target = TARGET_VEHICLE_ANY
        default_elf = 'build_flight/rocketchip.elf'
    elf_path = args.elf or default_elf

    # Chip-serial detection (per ArduPilot amendment — anti-Frankenstein)
    chip_serial = args.chip_serial or _detect_vehicle_chip_serial()
    if not chip_serial and not args.skip_picotool:
        print('SETUP ERROR: chip serial not detected. '
              'Use --chip-serial or --skip-picotool.')
        sys.exit(2)

    # Arm test mode via probe (writes kTestModeMagic, resets).
    # Both vehicle and station will boot into kMenu when armed
    # (forced in AO_RCOS_start when magic was observed).
    if not args.skip_arm:
        print(f'[ARM] Arming test mode via probe ({elf_path})...')
        if not arm_test_mode_via_probe(elf_path):
            print('SETUP ERROR: failed to arm test mode via probe.')
            sys.exit(2)
        time.sleep(3.0)  # let boot-time-window capture the magic + AOs start

    # Initial port + baseline banner
    port, info = find_target_port(target)
    if not port:
        print(f'SETUP ERROR: {args.role} port not found ({info})')
        sys.exit(2)
    baseline = peek_banner(port)
    if not baseline or not baseline.is_known():
        print('SETUP ERROR: baseline banner could not be parsed')
        sys.exit(2)
    print(f'baseline: {baseline.short_summary()}  port={port}  role={args.role}')
    usb_serial = board_usb_serial(port)
    board = Board(port=port, usb_serial=usb_serial)
    print('board followed across resets by '
          + (f'USB serial {usb_serial}' if usb_serial else
             f'port name {port} (no USB serial reported)'))
    print(f'chip_serial={chip_serial}')

    # Run gates. G-W1 + G-W4's Core-1 counter check is vehicle-only;
    # station auto-skips them (no fast-incrementing counter on station-
    # side q->s output; AO_Radio_get_state()->rx_count only advances
    # when vehicle is TXing, which is not guaranteed during the audit).
    is_station = (args.role == 'station')
    results = {}
    if args.skip_aircr:
        print('[G-W1] SKIP (--skip-aircr)')
        print('[G-W4] SKIP (--skip-aircr)')
    elif is_station:
        print('[G-W1] SKIP (station: no Core-1 counter check applicable)')
        print('[G-W4] SKIP (station: no Core-1 counter check applicable)')
    else:
        results['G-W1'] = _gate_w1_aircr(board, baseline, args.verbose)
    results['G-W2'] = _gate_w2_probe_reset_x3(board, baseline, args.verbose)
    if args.skip_picotool:
        print('[G-W3] SKIP (--skip-picotool)')
        print('[G-W4] SKIP (--skip-picotool)')
    else:
        results['G-W3'] = _gate_w3_picotool_burst(
            board, baseline, args.burst, chip_serial, args.verbose)
    if (not args.skip_aircr and not args.skip_picotool and not is_station):
        results['G-W4'] = _gate_w4_compound(
            board, baseline, chip_serial, args.verbose)

    # Summary
    print('')
    print('=== Warm-Reboot Audit Summary ===')
    failed = 0
    for name, ok in results.items():
        print(f'  {name}: {"PASS" if ok else "FAIL"}')
        if not ok:
            failed += 1
    print(f'chip_serial={chip_serial}')
    print(f'===  {len(results) - failed}/{len(results)} PASS  ===')
    return 0 if failed == 0 else 1


if __name__ == '__main__':
    main()
