#!/usr/bin/env python3
"""Host test: scripts/warm_reboot_audit.py reset verification.

No hardware. The port list, the reset command and the connect banner's
"Uptime: Ns" are faked on a simulated clock. Covers: a real reboot passes;
a reboot command that exits 0 without resetting fails (the Tiny 2350
`picotool reboot -f` false pass); a port that never returns fails; an
unchanged boot marker fails; plus the command-failure, boot-before-command,
unreadable-marker, board-identity and G-W3 gate cases.
"""
from __future__ import annotations

import contextlib
import io
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import warm_reboot_audit as wra  # noqa: E402
from _rc_test_common import Banner, Build, Role  # noqa: E402

FAILS = 0


def check(name: str, ok: bool, detail: object = '') -> None:
    global FAILS
    print(f'  [{"PASS" if ok else "FAIL"}] {name}')
    if not ok:
        FAILS += 1
        if detail:
            print(f'         {detail}')


VID, PID = wra.ROCKETCHIP_USB_VID, wra.ROCKETCHIP_USB_PID
SERIAL = 'E4632C1A2B3C4D5E'


class Clock:
    def __init__(self) -> None:
        self.t = 1000.0

    def now(self) -> float:
        return self.t

    def sleep(self, s: float) -> None:
        self.t += max(0.0, s)


@dataclass
class PortInfo:
    device: str
    vid: Optional[int]
    pid: Optional[int]
    serial_number: Optional[str]


class Sim:
    """One board on a simulated clock.

    mode 'reset'     : the command resets it: port absent from +gone_at to
                       +back_at after the command starts, new boot at +boot_at.
    mode 'no-reset'  : command exits 0, board keeps running (Tiny 2350 case).
    mode 'no-return' : port leaves at +gone_at and never comes back.
    mode 'usb-only'  : port drops and returns, but the MCU did not reboot
                       (uptime keeps counting), e.g. a USB-only re-enumerate.
    exit_at / rc     : when the command process exits, with what code.
    """

    def __init__(self, clk: Clock, mode: str = 'reset', rc: int = 0,
                 exit_at: float = 1.0, gone_at: float = 0.3,
                 back_at: float = 2.0, boot_at: float = 0.5,
                 port_after: str = 'COM7', uptime0: float = 60.0) -> None:
        self.clk = clk
        self.mode = mode
        self.rc = rc
        self.exit_at = exit_at
        self.gone_at = gone_at
        self.back_at = back_at
        self.boot_at = boot_at
        self.port = 'COM7'
        self.port_after = port_after
        self.boot_t = clk.now() - uptime0
        self.t_start: Optional[float] = None
        self.starts = 0
        self.extra_ports: list = []

    # Popen-like process
    def start(self):
        self.t_start = self.clk.now()
        self.starts += 1
        sim = self

        class Proc:
            def poll(self_inner):
                if sim.clk.now() - sim.t_start >= sim.exit_at:
                    sim._apply()
                    return sim.rc
                return None

            def kill(self_inner):
                pass

            def communicate(self_inner, timeout=None):
                return ('picotool: ERROR: something\n', None)
        return Proc()

    def _since(self) -> Optional[float]:
        return None if self.t_start is None else self.clk.now() - self.t_start

    def _apply(self) -> None:
        """Commit a finished reset (new boot time, new COM name)."""
        if self.t_start is None:
            return
        if self.mode == 'reset' and self.rc == 0:
            new_boot = self.t_start + self.boot_at
            if self.boot_t < new_boot <= self.clk.now():
                self.boot_t = new_boot

    def present(self) -> bool:
        d = self._since()
        if d is None or self.rc != 0:
            return True
        if self.mode in ('reset', 'usb-only'):
            return not (self.gone_at <= d < self.back_at)
        if self.mode == 'no-return':
            return d < self.gone_at
        return True

    def comports(self) -> list:
        ports = list(self.extra_ports)
        if self.present():
            d = self._since()
            name = self.port_after if (d is not None and d >= self.back_at) else self.port
            ports.append(PortInfo(name, VID, PID, SERIAL))
        return ports

    def uptime(self, port: str) -> Optional[int]:
        self.clk.sleep(0.4)       # banner read latency
        if not self.present():
            return None
        d = self._since()
        if self.mode == 'reset' and self.rc == 0 and d is not None and d >= self.boot_at:
            self.boot_t = max(self.boot_t, self.t_start + self.boot_at)
        return int(self.clk.now() - self.boot_t)


def step(sim: Sim, clk: Clock, board: Optional[wra.Board] = None,
         note: str = '', read=None):
    board = board or wra.Board(port='COM7', usb_serial=SERIAL)
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        ok = wra.reboot_step(
            'T', 'picotool reboot #1', board, sim.start, cmd_timeout_s=15.0,
            settle_s=1.0, cmd_fail_note=note,
            read_marker=read or (lambda p: wra._read_boot_marker(
                p, clk.now, clk.sleep, read=sim.uptime)),
            comports=sim.comports, clock=clk.now, sleep=clk.sleep)
    return ok, out.getvalue(), board


def test_required_cases() -> None:
    print('required cases (reboot_step):')
    clk = Clock()
    sim = Sim(clk, 'reset', port_after='COM9')
    ok, out, board = step(sim, clk)
    check('real reboot (port gone then back, marker drops) passes',
          ok and 'reset verified' in out, out)
    check('real reboot: board followed by USB serial onto the new COM name',
          board.port == 'COM9', board.port)
    check('real reboot: marker carried forward for the next reboot',
          board.marker is not None and board.marker.uptime_s < 10, board.marker)

    clk = Clock()
    ok, out, board = step(Sim(clk, 'no-reset'), clk)
    check('no reset (exit 0, port never leaves) fails',
          not ok and 'FAIL' in out and 'no reset' in out
          and 'never left' in out, out)
    check('no reset: marker reset to unknown after a failed step',
          board.marker is None)

    clk = Clock()
    ok, out, _ = step(Sim(clk, 'no-return'), clk)
    check('port never returns fails', not ok and 'did not return' in out, out)

    clk = Clock()
    ok, out, _ = step(Sim(clk, 'usb-only'), clk)
    check('marker unchanged (port flapped, MCU kept running) fails',
          not ok and 'boot marker unchanged' in out, out)


def test_watch_reset() -> None:
    print('watch_reset:')
    clk = Clock()
    sim = Sim(clk, 'reset', exit_at=3.0, gone_at=0.3, back_at=2.0)
    w = wra.watch_reset(sim.start, wra.make_locator(
        wra.Board('COM7', SERIAL), sim.comports), 15.0,
        clock=clk.now, sleep=clk.sleep)
    check('port back before the command exits (picotool -f): waits for exit, passes',
          w.ok and clk.now() - w.t_cmd >= 3.0, w)
    check('gone/back timings reported', w.gone_s is not None and
          abs(w.gone_s - 0.3) < 0.15 and w.back_s is not None
          and abs(w.back_s - 1.7) < 0.15, w)

    clk = Clock()
    sim = Sim(clk, 'reset', exit_at=0.2, gone_at=3.0, back_at=4.5, boot_at=3.2)
    w = wra.watch_reset(sim.start, wra.make_locator(
        wra.Board('COM7', SERIAL), sim.comports), 15.0,
        clock=clk.now, sleep=clk.sleep)
    check('port leaves 2.8 s after the command exits (inside the window): passes',
          w.ok, w)

    clk = Clock()
    sim = Sim(clk, 'reset', exit_at=0.2, gone_at=0.2 + wra.PORT_GONE_AFTER_CMD_S + 1,
              back_at=30.0, boot_at=29.0)
    w = wra.watch_reset(sim.start, wra.make_locator(
        wra.Board('COM7', SERIAL), sim.comports), 15.0,
        clock=clk.now, sleep=clk.sleep)
    check('port leaves after the bounded window: no reset',
          w.kind == wra.RESET_NONE, w)

    clk = Clock()
    sim = Sim(clk, 'reset', rc=-7)
    w = wra.watch_reset(sim.start, wra.make_locator(
        wra.Board('COM7', SERIAL), sim.comports), 15.0,
        clock=clk.now, sleep=clk.sleep)
    check('non-zero exit is a command failure with the tool output',
          w.kind == wra.RESET_CMD_FAILED and 'exit -7' in w.reason
          and 'picotool: ERROR' in w.reason, w)

    clk = Clock()
    sim = Sim(clk, 'no-reset', exit_at=1e9)
    w = wra.watch_reset(sim.start, wra.make_locator(
        wra.Board('COM7', SERIAL), sim.comports), 15.0,
        clock=clk.now, sleep=clk.sleep)
    check('hung command times out (bounded)',
          w.kind == wra.RESET_CMD_FAILED and 'timed out' in w.reason, w)

    clk = Clock()
    sim = Sim(clk, 'reset')
    sim.port = 'COM3'
    w = wra.watch_reset(sim.start, wra.make_locator(
        wra.Board('COM7', None), lambda: []), 15.0,
        clock=clk.now, sleep=clk.sleep)
    check('board absent before the command: fails, command not started',
          w.kind == wra.RESET_ABSENT and sim.starts == 0, w)

    clk = Clock()
    sim = Sim(clk, 'reset')
    calls = {'n': 0}

    def flaky():
        calls['n'] += 1
        if calls['n'] in (2, 3):
            raise OSError('enumeration failed')
        return sim.comports()
    w = wra.watch_reset(sim.start, wra.make_locator(
        wra.Board('COM7', SERIAL), flaky), 15.0, clock=clk.now, sleep=clk.sleep)
    check('enumeration error is skipped, not read as the port leaving', w.ok
          and w.gone_s is not None and w.gone_s >= 0.3 - 1e-9, w)

    clk = Clock()
    sim = Sim(clk, 'no-reset')
    calls = {'n': 0}

    def errs_only():
        calls['n'] += 1
        if calls['n'] > 1:
            raise OSError('enumeration failed')
        return sim.comports()
    w = wra.watch_reset(sim.start, wra.make_locator(
        wra.Board('COM7', SERIAL), errs_only), 15.0,
        clock=clk.now, sleep=clk.sleep)
    check('enumeration errors alone never count as a reset',
          w.kind == wra.RESET_NONE, w)


def test_locator() -> None:
    print('make_locator:')
    ports = [PortInfo('COM4', VID, PID, 'OTHERBOARD'),
             PortInfo('COM5', 0x2E8A, 0x000C, SERIAL.lower()),  # debug probe
             PortInfo('COM8', VID, PID, SERIAL.lower())]
    loc = wra.make_locator(wra.Board('COM7', SERIAL), lambda: ports)
    check('by USB serial (case-insensitive), not VID:PID alone, not the probe',
          loc() == 'COM8', loc())
    loc = wra.make_locator(wra.Board('COM7', SERIAL), lambda: ports[:2])
    check('other board present, ours gone (BOOTSEL / reset): absent', loc() is None)
    loc = wra.make_locator(wra.Board('COM4', None), lambda: ports)
    check('no USB serial: follows the port name', loc() == 'COM4', loc())


def test_classify_reset() -> None:
    print('classify_reset:')
    b = wra.BootMarker(60, 100.0)
    ok, _ = wra.classify_reset(b, 101.0, wra.BootMarker(5, 108.0))
    check('drop + boot after the command: reset', ok)
    ok, why = wra.classify_reset(b, 101.0, wra.BootMarker(68, 108.0))
    check('uptime grew by the elapsed time: unchanged', not ok
          and 'unchanged' in why, why)
    # No reset, 1 s resolution and ~1.9 s read latency: 69.9 s expected, 68 read.
    ok, why = wra.classify_reset(b, 101.0, wra.BootMarker(68, 109.9))
    check('1 s resolution + read latency still reads as unchanged', not ok
          and 'unchanged' in why, why)
    ok, why = wra.classify_reset(b, 101.0, wra.BootMarker(20, 108.0))
    check('drop but the boot predates the command: fails', not ok
          and 'predates' in why, why)
    ok, why = wra.classify_reset(b, 101.0, None)
    check('no marker after: fails', not ok and 'unreadable' in why, why)
    # Worst case the gates allow: board up exactly MIN_UPTIME_BEFORE_S
    # (reading 1 s low), reset right at the command, read 1 s later.
    lo = wra.BootMarker(wra.MIN_UPTIME_BEFORE_S - 1, 100.0)
    ok, why = wra.classify_reset(lo, 100.0, wra.BootMarker(1, 101.0))
    check('MIN_UPTIME_BEFORE_S leaves a fast reset distinguishable', ok, why)


def test_step_edges() -> None:
    print('reboot_step edges:')
    clk = Clock()
    sim = Sim(clk, 'reset', rc=1)
    ok, out, _ = step(sim, clk, note=wra.PICOTOOL_FAIL_NOTE)
    check('picotool exit != 0: FAIL with the Risk #3 tool-failure note',
          not ok and 'Risk #3' in out and 'never automatic' in out, out)

    clk = Clock()
    ok, out, _ = step(Sim(clk, 'no-reset'), clk, note=wra.PICOTOOL_FAIL_NOTE)
    check('no-reset FAIL does not offer the Risk #3 downgrade',
          not ok and 'Risk #3' not in out, out)

    clk = Clock()
    sim = Sim(clk, 'reset')
    ok, out, _ = step(sim, clk, read=lambda p: None)
    check('marker unreadable before: FAIL, command not started',
          not ok and 'before the command' in out and sim.starts == 0, out)

    clk = Clock()
    sim = Sim(clk, 'reset')
    board = wra.Board('COM7', SERIAL, marker=wra.BootMarker(1, clk.now()))
    ok, out, _ = step(sim, clk, board=board)
    check('fresh boot (uptime 1 s): waits to MIN_UPTIME_BEFORE_S, then passes',
          ok and sim.t_start is not None
          and sim.t_start - 1000.0 >= wra.MIN_UPTIME_BEFORE_S - 1 - 1e-9, out)

    clk = Clock()
    reads = iter([None, None, 42])
    m = wra._read_boot_marker('COM7', clk.now, clk.sleep, read=lambda p: next(reads))
    check('marker read retries the Windows CDC release race',
          m is not None and m.uptime_s == 42, m)
    clk = Clock()
    n = {'reads': 0}

    def never(p):
        n['reads'] += 1
        return None
    m = wra._read_boot_marker('COM7', clk.now, clk.sleep, read=never)
    check('marker read gives up after MARKER_READ_TRIES (bounded)',
          m is None and n['reads'] == wra.MARKER_READ_TRIES, n)


def test_gate_w3() -> None:
    print('G-W3 gate (old false pass now fails):')
    baseline = Banner(role=Role.VEHICLE, build=Build.FLIGHT,
                      version='0.16.3-dev', board=None)

    def run(mode: str, burst: int = 3):
        clk = Clock()
        sim = Sim(clk, mode)
        saved = (wra._clock, wra._sleep, wra._start_picotool_reboot,
                 wra._list_comports, wra.banner_uptime, wra.peek_banner)
        wra._clock, wra._sleep = clk.now, clk.sleep
        wra._start_picotool_reboot = lambda ser: sim.start()
        wra._list_comports = sim.comports
        wra.banner_uptime = sim.uptime
        wra.peek_banner = lambda p: baseline
        out = io.StringIO()
        try:
            with contextlib.redirect_stdout(out):
                ok = wra._gate_w3_picotool_burst(
                    wra.Board('COM7', SERIAL), baseline, burst, 'CHIP', False)
        finally:
            (wra._clock, wra._sleep, wra._start_picotool_reboot,
             wra._list_comports, wra.banner_uptime, wra.peek_banner) = saved
        return ok, out.getvalue(), sim

    ok, out, sim = run('reset')
    check('3 real reboots: G-W3 PASS', ok and 'PASS' in out and sim.starts == 3
          and out.count('reset verified') == 3, out)
    ok, out, sim = run('no-reset')
    check('reboot -f exits 0, board keeps running: G-W3 FAIL at #1',
          not ok and '[G-W3] FAIL: picotool reboot #1' in out
          and 'no reset' in out and sim.starts == 1, out)
    check('the old wait (port present -> return) is gone',
          not hasattr(wra, '_wait_for_device_serial'))


def main() -> int:
    test_required_cases()
    test_watch_reset()
    test_locator()
    test_classify_reset()
    test_step_edges()
    test_gate_w3()
    print(f'\n{"FAIL" if FAILS else "PASS"}: {FAILS} failure(s)')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
