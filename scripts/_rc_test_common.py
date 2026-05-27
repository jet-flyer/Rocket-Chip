#!/usr/bin/env python3
"""Shared helpers for RocketChip host-side test scripts.

Council-reviewed 2026-04-27 (ArduPilot Core Contributor, Retired NASA/JPL
Avionics Lead, Embedded Systems Professor, Senior Aerospace Student).
Origin: 2026-04-27 incident triage --- station_bench_sim was sending
station-only key sequences (notably 'x', which is Erase-Flights confirm
on vehicle main menu) to vehicle firmware because of a stale fallback in
its auto-detect path. The lesson generalised: tests written against a
single board/configuration silently rot as the project grows multi-tier.

This module is the foundation for closing the rot. It provides:

- start_watchdog(deadline_s, label)
    Daemon thread that calls os._exit(2) at deadline. Bypasses Python
    finalizers (intentional --- we are already wedged) and is the only
    reliable escape from a hung serial.Serial() open in C library code
    on Windows USB CDC.

- Banner (frozen dataclass) + Role + Build (enums)
    Immutable classification of what firmware a serial port has on it,
    parsed from the boot banner / help text content. NEVER from VID/PID
    alone --- both vehicle and station expose VID 0x2E8A PID 0x0009.

- classify_banner(text) -> Banner
    Pure function: text in, classification out. Handles vehicle, station,
    bench/flight, version capture, board name extraction.

- peek_banner(port_name, ...) -> Banner
    Thread-bounded open + brief read. PermissionError or held-port can
    no longer block. Returns Banner with role=UNKNOWN if classification
    fails for any reason.

- find_target_port(target, override, verbose) -> (port, banner) or (None, reason)
    Strict, classifier-based port detection. NO fallback to "first
    candidate." If no port matches the requested target, returns
    (None, reason) so the caller can exit with code 2 (skip).

- @rc_test(target=..., watchdog_s=...)
    Decorator that:
      * stashes the declared target on the function for introspection
      * starts a watchdog (if watchdog_s given)
      * enforces the 0/1/2 exit-code contract
      * wraps uncaught exceptions to exit 1 with stderr summary
      * treats KeyboardInterrupt as exit 2 (skip --- user-initiated)

The decorator is metadata + wrapper; it does NOT take over arg parsing
or port opening. Scripts keep their existing main() shape and call
find_target_port + open_classified_port themselves. This is the lightest-
touch refactor that still gives us a uniform contract.

Exit-code contract (council-enforced, identical across all scripts):
    0 = pass / success
    1 = real test failure (block in CI)
    2 = environment skip (no target present, wrong build, watchdog
        fired, user interrupt) --- pre-commit hooks treat as SKIP

Usage:
    from _rc_test_common import (
        rc_test, find_target_port, open_classified_port,
        TARGET_VEHICLE_ANY,
    )

    @rc_test(target=TARGET_VEHICLE_ANY, watchdog_s=120)
    def main():
        parser = argparse.ArgumentParser(...)
        # ... script-specific args ...
        args = parser.parse_args()

        port, banner = find_target_port(TARGET_VEHICLE_ANY, override=args.port)
        if port is None:
            print(f'INFO: skipping --- {banner}')
            return 2

        with open_classified_port(port, target=TARGET_VEHICLE_ANY) as ser:
            # ... run tests ...
            return 0

    if __name__ == '__main__':
        main()
"""

from __future__ import annotations

import enum
import os
import re
import sys
import threading
import time
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Callable, Iterator, Optional, Tuple

import serial
import serial.tools.list_ports


# ============================================================================
# Constants
# ============================================================================

ROCKETCHIP_USB_VID = 0x2E8A
ROCKETCHIP_USB_PID = 0x0009

# Banner content tokens used for role classification. All comparisons are
# done lowercase. Keep these aligned with the firmware banners printed by:
#   - src/cli/rc_os_commands.cpp (vehicle boot banner, preflight)
#   - src/cli/rc_os.cpp (vehicle main-menu help, [main]/[debug]/[cal]/[flight])
#   - src/active_objects/ao_rcos.cpp:enter_cli_menu (station kMenu banner)
#   - src/cli/rc_os_dashboard.cpp (station kAnsi dashboard frame text)
_STATION_TOKENS = ('ground station', 'fruit jam', 'station rx')
_VEHICLE_TOKENS = ('profile: rocket', 'erase all',
                   'h-help  p-preflight  c-calibration  f-flight')

# Mode-distinguishing tokens. Station boots in kAnsi (dashboard) and only
# enters kMenu via 'x'; vehicle boots straight into kMenu. Dashboard tokens
# are unique frame fragments that don't appear in any kMenu output.
_KMENU_TOKENS = ('[main]', '[debug]', '[cal]', '[flight]', '--- help ---')
_DASHBOARD_TOKENS = (
    '=== rocketchip ground station ===',  # full dashboard header
    "'m' mode cycle  'x' menu",           # dashboard footer
    'waiting for vehicle packets',         # dashboard "no telem" state
)

# Build tag suffix on the boot banner:
#   "RocketChip vX.Y.Z RCOS vA.B.C flight-<sha>"
# Post-R-26 (2026-05-15) `kBuildConfig` is hardcoded "flight" — the historical
# dev-/bench- suffixes can no longer be emitted by any firmware build.
_RE_BUILD_TAG = re.compile(r'\bflight-([0-9a-f]{6,12})\b')
_RE_VERSION = re.compile(r'rocketchip\s+v(\d+\.\d+\.\d+)')
_RE_BOARD = re.compile(r'board:\s*([^\n\r]+)')


# ============================================================================
# Watchdog
# ============================================================================

def start_watchdog(deadline_s: float, label: str = 'rc_test') -> threading.Thread:
    """Start a daemon thread that hard-kills the process at deadline.

    Uses os._exit(2) intentionally --- bypasses Python finalizers because
    we are typically already wedged in a serial.Serial() C call when the
    watchdog fires. The 2 exit code matches the project-wide "skip / could
    not run" semantics.

    Returns the thread so callers can introspect (rare) but normally just
    fire-and-forget.
    """
    def _bark() -> None:
        time.sleep(deadline_s)
        sys.stderr.write(
            f'\n[WATCHDOG] {label} exceeded --max-runtime '
            f'{deadline_s:.0f}s wall-clock; force-exiting (code 2)\n')
        sys.stderr.flush()
        os._exit(2)

    t = threading.Thread(target=_bark, daemon=True, name=f'watchdog-{label}')
    t.start()
    return t


# ============================================================================
# Target classification
# ============================================================================

class Role(enum.Enum):
    VEHICLE = 'vehicle'
    STATION = 'station'
    EITHER  = 'either'    # rare --- only Stage T multi-port runners use this
    UNKNOWN = 'unknown'


class Build(enum.Enum):
    # Post-R-26 (2026-05-15) firmware only emits `flight-<sha>` on the boot
    # banner. FLIGHT is the only meaningful value; ANY skips the build check
    # entirely; UNKNOWN means the regex didn't match (e.g. station dashboard
    # banner has no build tag, garbled banner, etc.).
    FLIGHT  = 'flight'
    ANY     = 'any'
    UNKNOWN = 'unknown'


@dataclass(frozen=True)
class Target:
    """What firmware configuration a host script requires.

    Frozen so a Target value can never be mutated mid-test --- a script's
    declared target is immutable contract.
    """
    role: Role
    build: Build = Build.ANY

    def __str__(self) -> str:
        if self.build is Build.ANY:
            return f'{self.role.value}-any'
        return f'{self.role.value}-{self.build.value}'

    def matches(self, banner: 'Banner') -> bool:
        """Does the firmware described by `banner` satisfy this target?

        EITHER means "vehicle OR station", NOT "anything goes" --- an
        unknown / unclassified banner is rejected even by EITHER. The
        only way through with role=UNKNOWN is to never have classification
        gating in the first place, which is not what callers want.
        """
        if banner.role is Role.UNKNOWN:
            return False
        if self.role is Role.EITHER:
            # Accept any *known* role.
            pass
        elif banner.role is not self.role:
            return False
        if self.build is Build.ANY:
            return True
        if banner.build is Build.UNKNOWN:
            # Banner didn't expose a build tag (typical for station
            # dashboard, garbled boot output, etc.). Permissive on UNKNOWN
            # — let the script try, it'll fail at runtime if the firmware
            # actually doesn't match.
            return True
        return banner.build is self.build


# Convenience constants --- the targets we actually use.
TARGET_VEHICLE_ANY    = Target(Role.VEHICLE, Build.ANY)
TARGET_VEHICLE_FLIGHT = Target(Role.VEHICLE, Build.FLIGHT)
TARGET_STATION_ANY    = Target(Role.STATION, Build.ANY)
TARGET_STATION_FLIGHT = Target(Role.STATION, Build.FLIGHT)
TARGET_EITHER_ANY     = Target(Role.EITHER,  Build.ANY)


# =============================================================================
# Station state hygiene helper (added 2026-05-27 during WSL pivot baseline)
#
# When the "default dashboard functionality" was added, station firmware now
# boots into (or can legitimately be in) kmenu state after flashes, reboots,
# or prior 'x' navigation. The bench script's early safety guard (post-open
# re-classify) was written when station always presented dashboard on first
# open, and it treats kmenu as "vehicle (kmenu)" even on known station hardware.
#
# This helper is called early for station targets to drive the board into the
# dashboard state the existing classification + tests expect, without weakening
# the safety rule that we must never send station-only keys (especially 'x' =
# Erase-Flights) to a vehicle.
# =============================================================================

def ensure_station_in_dashboard_state(ser: serial.Serial, max_attempts: int = 5) -> bool:
    """Drive a station board from kmenu (if present) back to a clean dashboard state.

    Returns True if we end up seeing dashboard tokens, False on timeout.
    This is the "ensure good state" step for station_bench_sim.py.
    """
    for attempt in range(max_attempts):
        ser.reset_input_buffer()
        time.sleep(0.6)
        data = ser.read(12000).decode('utf-8', errors='replace')

        if any(tok in data.lower() for tok in _DASHBOARD_TOKENS):
            return True

        if any(tok in data.lower() for tok in _KMENU_TOKENS):
            # 'x' from kmenu on station should return us toward dashboard
            # (or at worst cycle safely). This is the same key used in the
            # navigation helpers, just applied here early for classification.
            ser.write(b'x')
            time.sleep(1.4)
            continue

        # Unclear state after flash/reboot — send 'x' as a safe probe
        ser.write(b'x')
        time.sleep(1.2)

    # Final read — best effort
    ser.reset_input_buffer()
    time.sleep(0.4)
    data = ser.read(8000).decode('utf-8', errors='replace')
    return any(tok in data.lower() for tok in _DASHBOARD_TOKENS)


class Mode(enum.Enum):
    """Active CLI mode of the firmware at the time we peeked.

    Vehicle boots in kMenu and never enters a dashboard.
    Station boots in kAnsi (dashboard) and enters kMenu via 'x'.
    The kMavlink / kCsv station modes are dashboard-equivalent for
    keystroke handling (most keys ignored).
    """
    KMENU      = 'kmenu'       # CLI prompt visible ([main] / [debug] / [cal] / [flight])
    DASHBOARD  = 'dashboard'   # station kAnsi --- only 'x', 'a', 'D', 'm' accepted
    UNKNOWN    = 'unknown'


@dataclass(frozen=True)
class Banner:
    """Immutable firmware classification from a banner peek.

    `raw` is the lowercase text we classified from --- kept for diagnostics
    so callers don't have to re-read the port to print useful errors.

    `mode` is the active CLI mode the firmware was in when we peeked.
    Critical for scripts that intend to send CLI keys: a station fresh
    out of boot is in DASHBOARD and must be transitioned to KMENU via
    enter_cli_menu(ser) before any CLI command will be honored.
    """
    role: Role
    build: Build
    version: Optional[str]    # e.g. "0.16.0"
    board: Optional[str]      # e.g. "Adafruit Feather RP2350 HSTX"
    mode: Mode = Mode.UNKNOWN
    raw: str = ''

    def is_vehicle(self) -> bool: return self.role is Role.VEHICLE
    def is_station(self) -> bool: return self.role is Role.STATION
    def is_known(self) -> bool:   return self.role is not Role.UNKNOWN
    def in_dashboard(self) -> bool: return self.mode is Mode.DASHBOARD
    def in_kmenu(self) -> bool: return self.mode is Mode.KMENU

    def short_summary(self) -> str:
        """One-line description suitable for INFO/ERROR messages."""
        if not self.is_known():
            return 'unknown firmware'
        parts = [self.role.value]
        if self.build is not Build.UNKNOWN and self.build is not Build.ANY:
            parts.append(self.build.value)
        if self.version is not None:
            parts.append(f'v{self.version}')
        if self.mode is not Mode.UNKNOWN:
            parts.append(f'({self.mode.value})')
        return ' '.join(parts)


def classify_banner(text: str) -> Banner:
    """Pure classifier: banner text -> Banner. Never raises.

    Detects role (vehicle/station/unknown), build type (bench/flight),
    version, board, and active CLI mode (kmenu/dashboard/unknown).
    """
    t = (text or '').lower()

    role = Role.UNKNOWN
    if any(tok in t for tok in _STATION_TOKENS):
        role = Role.STATION
    elif any(tok in t for tok in _VEHICLE_TOKENS):
        role = Role.VEHICLE

    build = Build.UNKNOWN
    m_build = _RE_BUILD_TAG.search(t)
    if m_build is not None:
        build = Build.FLIGHT

    version: Optional[str] = None
    m_ver = _RE_VERSION.search(t)
    if m_ver is not None:
        version = m_ver.group(1)

    board: Optional[str] = None
    m_brd = _RE_BOARD.search(t)
    if m_brd is not None:
        board = m_brd.group(1).strip()

    # Mode detection. Dashboard markers are checked first because a
    # dashboard frame can also contain residual kMenu text from history
    # buffers; if we see active dashboard markers, prefer that
    # classification --- the firmware will not honor CLI keys until we
    # exit dashboard.
    mode = Mode.UNKNOWN
    if any(tok in t for tok in _DASHBOARD_TOKENS):
        mode = Mode.DASHBOARD
    elif any(tok in t for tok in _KMENU_TOKENS):
        mode = Mode.KMENU

    return Banner(role=role, build=build, version=version, board=board,
                  mode=mode, raw=t)


# ============================================================================
# Port probing
# ============================================================================

def peek_banner(port_name: str,
                peek_timeout: float = 2.0,
                open_timeout: float = 3.0,
                retries: int = 2,
                retry_delay: float = 0.6) -> Banner:
    """Briefly open `port_name`, read whatever banner is available, classify.

    Uses thread-with-join-timeout because serial.Serial() can block in C
    library code on Windows USB CDC if the port is held by another
    process or the device is wedged. Python's own timeout parameter
    applies only to reads, not to the open itself.

    Retries once by default --- on Windows, calling peek_banner twice
    back-to-back on the same port hits a USB CDC release race where the
    second open returns PermissionError because the OS has not yet
    finished closing the previous handle. A short retry_delay between
    attempts is sufficient to clear it.

    Returns Banner with role=UNKNOWN on persistent failure (port held,
    no response, classification fails). Never raises.

    Strategy: read passively first --- many builds emit a fresh boot
    banner on USB CDC connect. Only send 'h' (help) if the passive read
    is short, to avoid sending keystrokes to firmware that may interpret
    them differently than expected.
    """
    text = ''
    for attempt in range(retries):
        result: list[str] = ['']

        def _go() -> None:
            try:
                s = serial.Serial(port_name, 115200, timeout=0.3,
                                  write_timeout=0.5)
                time.sleep(0.5)
                data = s.read(8000)
                if len(data) < 64:
                    s.write(b'h')
                    s.flush()
                    time.sleep(peek_timeout)
                    data += s.read(8000)
                s.close()
                result[0] = data.decode('utf-8', errors='replace')
            except (serial.SerialException, OSError, PermissionError):
                pass

        t = threading.Thread(target=_go, daemon=True,
                             name=f'peek-{port_name}-{attempt}')
        t.start()
        t.join(timeout=open_timeout)
        text = result[0]
        if text:
            break
        if attempt < retries - 1:
            time.sleep(retry_delay)
    return classify_banner(text)


def _candidate_ports() -> list[str]:
    """All USB CDC ports matching RocketChip VID:PID."""
    return [
        info.device for info in serial.tools.list_ports.comports()
        if info.vid == ROCKETCHIP_USB_VID and info.pid == ROCKETCHIP_USB_PID
    ]


def find_target_port(target: Target,
                     override: Optional[str] = None,
                     verbose: bool = False) -> Tuple[Optional[str], object]:
    """Locate a serial port whose firmware satisfies `target`.

    Returns (port_name, banner) on success, or (None, reason_string) on
    failure. NEVER falls back to a non-matching port --- the caller is
    expected to exit with code 2 (skip) when target is not present.

    `override` (e.g. from argparse --port) is honored but still
    classified. A user-specified port that's the wrong target is refused
    with the same exit-2 semantics --- "manual override doesn't bypass
    safety" is policy.
    """
    if override is not None:
        banner = peek_banner(override)
        if not banner.is_known():
            if verbose:
                print(f'  --port {override}: unknown firmware')
            return None, f'--port {override} returned no recognisable banner'
        if not target.matches(banner):
            if verbose:
                print(f'  --port {override}: {banner.short_summary()} '
                      f'(needed {target})')
            return None, (f'--port {override} is {banner.short_summary()}, '
                          f'not {target}')
        return override, banner

    candidates = _candidate_ports()
    if verbose:
        print(f'  candidates (VID 0x{ROCKETCHIP_USB_VID:04X} PID '
              f'0x{ROCKETCHIP_USB_PID:04X}): {candidates}')
    if not candidates:
        return None, 'no RocketChip USB CDC ports plugged in'

    # First pass: exact match.
    seen: list[Tuple[str, Banner]] = []
    for port in candidates:
        banner = peek_banner(port)
        seen.append((port, banner))
        if verbose:
            print(f'  {port}: {banner.short_summary()}')
        if target.matches(banner) and banner.is_known():
            return port, banner

    # No exact match. Build a useful diagnostic.
    if not any(b.is_known() for _, b in seen):
        return None, f'{len(seen)} candidate(s) but none responded to a banner peek'

    summaries = ', '.join(f'{p}={b.short_summary()}' for p, b in seen)
    return None, f'no port matches {target}: {summaries}'


def find_vehicle_and_station_ports(
    station_override: Optional[str] = None,
    vehicle_override: Optional[str] = None,
    verbose: bool = False,
) -> Tuple[Optional[str], Optional[str], Optional[Banner], Optional[Banner], str]:
    """Resolve two distinct RocketChip USB CDC ports: vehicle + station.

    For Stage T dual-port harnesses. Manual overrides are classified the
    same way as ``find_target_port``; remaining roles are filled by scanning
    unplugged candidates (first station + first vehicle that match).

    Returns ``(vehicle_port, station_port, veh_banner, stn_banner, reason)``.
    On failure all ports and banners are ``None`` and ``reason`` explains why.
    """
    st_p: Optional[str] = None
    st_b: Optional[Banner] = None
    veh_p: Optional[str] = None
    veh_b: Optional[Banner] = None

    if station_override is not None:
        p, br = find_target_port(
            TARGET_STATION_ANY, override=station_override, verbose=verbose)
        if p is None or not isinstance(br, Banner):
            return None, None, None, None, str(br)
        st_p, st_b = p, br

    if vehicle_override is not None:
        p, br = find_target_port(
            TARGET_VEHICLE_ANY, override=vehicle_override, verbose=verbose)
        if p is None or not isinstance(br, Banner):
            return None, None, None, None, str(br)
        veh_p, veh_b = p, br

    if st_p is not None and veh_p is not None and st_p == veh_p:
        return (None, None, None, None,
                'station and vehicle cannot be the same port')

    used = {x for x in (st_p, veh_p) if x is not None}
    for dev in _candidate_ports():
        if dev in used:
            continue
        b = peek_banner(dev)
        if not b.is_known():
            continue
        if st_p is None and TARGET_STATION_ANY.matches(b):
            st_p, st_b = dev, b
            used.add(dev)
            continue
        if veh_p is None and TARGET_VEHICLE_ANY.matches(b):
            veh_p, veh_b = dev, b
            used.add(dev)

    if st_p is None:
        return (None, None, None, None,
                'no station port found (plug station or use --station-port)')
    if veh_p is None:
        return (None, None, None, None,
                'no vehicle port found (plug vehicle or use --vehicle-port)')

    return veh_p, st_p, veh_b, st_b, ''


def enter_cli_menu(ser: serial.Serial,
                   settle_s: float = 1.0,
                   verify: bool = True) -> bool:
    """Transition firmware from kAnsi dashboard -> kMenu CLI.

    Station firmware boots into the kAnsi dashboard, which only honors
    keys 'x' (enter menu), 'a' (ARM-confirm), 'D' (DISARM), 'm' (mode
    cycle). All other keys are ignored. Scripts that intend to send CLI
    commands (q, d, b, p, ...) MUST first send 'x' to enter kMenu.

    On vehicle this is a no-op since vehicle boots in kMenu directly ---
    sending 'x' to vehicle main menu would trigger the Erase-Flights
    confirm prompt, so we ONLY send 'x' if we have evidence we're in a
    dashboard state.

    Strategy:
      1. Drain any pending output, send 'h' to elicit a response.
      2. Classify the response.
         - kmenu -> already there, return True.
         - dashboard -> send 'x', drain, send 'h' again, re-classify.
         - unknown -> last resort: send '\\n' (rejects any erase confirm)
                       then 'h', re-classify.
      3. If verify=True, return True iff final classification is kmenu.

    Returns True on success, False if we couldn't reach kmenu.
    Never raises; safe to call defensively on any opened port.
    """
    def _peek_mode() -> Mode:
        try:
            ser.reset_input_buffer()
        except (serial.SerialException, OSError):
            pass
        try:
            ser.write(b'h')
            ser.flush()
        except (serial.SerialException, OSError):
            return Mode.UNKNOWN
        time.sleep(settle_s)
        try:
            data = ser.read(16000)
        except (serial.SerialException, OSError):
            return Mode.UNKNOWN
        return classify_banner(data.decode('utf-8', errors='replace')).mode

    mode = _peek_mode()
    if mode is Mode.KMENU:
        return True

    if mode is Mode.DASHBOARD:
        try:
            ser.write(b'x')
            ser.flush()
        except (serial.SerialException, OSError):
            return False
        time.sleep(settle_s)
        try:
            ser.read(16000)
        except (serial.SerialException, OSError):
            pass
        return _peek_mode() is Mode.KMENU if verify else True

    # Unknown: defensively reject any pending y/n confirm prompt with '\n'
    # (which is NOT 'yes\n' so any erase prompt rejects), then re-peek.
    try:
        ser.write(b'\n')
        ser.flush()
    except (serial.SerialException, OSError):
        return False
    time.sleep(0.3)
    return _peek_mode() is Mode.KMENU if verify else False


@contextmanager
def open_classified_port(port_name: str,
                         target: Target,
                         baud: int = 115200,
                         timeout: float = 0.1,
                         retries: int = 5,
                         retry_delay: float = 1.5,
                         open_timeout: float = 3.0,
                         post_open_re_classify: bool = True,
                         auto_enter_cli_menu: Optional[bool] = None
                         ) -> Iterator[serial.Serial]:
    """Open a port with retries, re-verify target, optionally enter kMenu.

    Per JPL council guidance: every script that opens a port and sends
    destructive keys should re-classify after `connect()` returns.
    Defaults to ON. Set post_open_re_classify=False only when the script
    will not send any keystrokes that depend on firmware role.

    `auto_enter_cli_menu` controls whether we transition station
    firmware from its default kAnsi dashboard into kMenu before yielding
    the port to the caller. Three values:
      - None (default): enter kMenu iff target is station (because
        station boots in dashboard and most scripts want CLI access).
      - True:  always enter kMenu.
      - False: leave the port in whatever mode it was in after open
        (use this only if the script is itself dashboard-aware, e.g.
        mavlink_validate.py or stage_t_wilson_ci.py which toggle modes).

    On Windows USB CDC, serial.Serial() can hang in C-library code; each
    open attempt is bounded by `open_timeout` via thread-with-join.

    Raises RuntimeError if the port cannot be opened, or
    SystemExit(2) if post-open re-classify reveals the wrong firmware
    (with a clear stderr message).
    """
    last_err: object = None
    ser: Optional[serial.Serial] = None

    for attempt in range(retries):
        result: list[object] = [None]

        def _try_open() -> None:
            try:
                p = serial.Serial(port_name, baud, timeout=timeout)
                time.sleep(1.0)
                p.read(10000)
                time.sleep(0.3)
                p.read(10000)
                result[0] = p
            except (serial.SerialException, OSError, PermissionError) as exc:
                result[0] = exc

        t = threading.Thread(target=_try_open, daemon=True,
                             name=f'open-{port_name}-{attempt}')
        t.start()
        t.join(timeout=open_timeout)
        if t.is_alive():
            last_err = (f'open hung > {open_timeout:.1f}s '
                        f'(port held by another process?)')
        elif isinstance(result[0], serial.Serial):
            ser = result[0]
            if attempt > 0:
                print(f'  (connected on attempt {attempt + 1})')
            break
        else:
            last_err = result[0]

        if attempt < retries - 1:
            print(f'  open attempt {attempt + 1}/{retries}: {last_err}; '
                  f'retrying in {retry_delay}s')
            time.sleep(retry_delay)

    if ser is None:
        raise RuntimeError(
            f'Could not open {port_name} after {retries} attempts: {last_err}')

    if post_open_re_classify:
        # Read whatever the firmware emitted on connect; if too short,
        # nudge with 'h' (help) which is a safe no-op on every menu and
        # in the station dashboard (where it's silently ignored).
        time.sleep(0.3)
        data = ser.read(16000)
        if len(data) < 64:
            ser.write(b'h')
            ser.flush()
            time.sleep(1.0)
            data += ser.read(16000)
        banner = classify_banner(data.decode('utf-8', errors='replace'))
        if not target.matches(banner):
            sys.stderr.write(
                f'ERROR: post-open re-classify on {port_name} found '
                f'{banner.short_summary()}, needed {target}.\n'
                f'  Did the board hot-swap, or is the firmware out of date?\n')
            sys.stderr.flush()
            ser.close()
            sys.exit(2)

        # Station boots in kAnsi dashboard. Most scripts want kMenu so
        # they can send CLI keys. Default to entering kMenu for station
        # targets unless the caller opts out (mavlink_validate, the
        # stage_t scripts, anyone explicitly mode-aware).
        if auto_enter_cli_menu is None:
            should_enter = banner.is_station()
        else:
            should_enter = bool(auto_enter_cli_menu)
        if should_enter:
            ok = enter_cli_menu(ser, settle_s=1.0, verify=True)
            if not ok:
                sys.stderr.write(
                    f'WARNING: could not confirm kMenu mode on {port_name} '
                    f'after sending \'x\'. Script may need to retry navigation '
                    f'manually.\n')
                sys.stderr.flush()

    try:
        yield ser
    finally:
        try:
            ser.close()
        except (serial.SerialException, OSError):
            pass


# ============================================================================
# @rc_test decorator
# ============================================================================

def rc_test(*,
            target: Target,
            watchdog_s: Optional[float] = None,
            label: Optional[str] = None
            ) -> Callable[[Callable[..., Optional[int]]], Callable[..., None]]:
    """Declare a script's target firmware + enforce the exit-code contract.

    Per the 2026-04-27 council review, the decorator:
      * stashes `target` on the function (and on the wrapper) for
        machine-readable introspection by CI / dev tooling
      * starts a watchdog daemon if `watchdog_s` is given
      * enforces the 0 / 1 / 2 exit-code contract:
          - main returns int     -> sys.exit(int)
          - main returns None    -> sys.exit(0)
          - SystemExit propagates as-is (allows main() to do sys.exit(2)
            for skip without uncaught-exception handling)
          - KeyboardInterrupt    -> sys.exit(2) (skip --- user-initiated)
          - Any other Exception  -> stderr summary + sys.exit(1)

    The decorator is intentionally lightweight. It does NOT take over arg
    parsing, port opening, or test orchestration --- scripts keep their
    existing main() shape and call find_target_port + open_classified_port
    explicitly. This is the lightest-touch refactor that still gives
    every script a uniform contract.

    Usage:
        @rc_test(target=TARGET_VEHICLE_ANY, watchdog_s=120)
        def main():
            ...
            return 0

        if __name__ == '__main__':
            main()
    """
    def decorator(fn: Callable[..., Optional[int]]) -> Callable[..., None]:
        # Stash on the original function for introspection (e.g. by a
        # future "list every test target" utility).
        fn.__rc_target__ = target           # type: ignore[attr-defined]
        fn.__rc_watchdog_s__ = watchdog_s   # type: ignore[attr-defined]

        def wrapped(*args, **kwargs) -> None:
            if watchdog_s is not None:
                start_watchdog(watchdog_s, label=label or fn.__module__)
            try:
                rc = fn(*args, **kwargs)
                sys.exit(0 if rc is None else int(rc))
            except SystemExit:
                raise
            except KeyboardInterrupt:
                sys.stderr.write('\n[INFO] interrupted by user (exit 2)\n')
                sys.stderr.flush()
                sys.exit(2)
            except Exception as exc:  # noqa: BLE001 -- top-level guard, intentional
                import traceback
                sys.stderr.write(
                    f'\n[ERROR] uncaught {type(exc).__name__}: {exc}\n')
                traceback.print_exc(file=sys.stderr)
                sys.stderr.flush()
                sys.exit(1)

        wrapped.__name__ = fn.__name__
        wrapped.__doc__ = fn.__doc__
        wrapped.__rc_target__ = target           # type: ignore[attr-defined]
        wrapped.__rc_watchdog_s__ = watchdog_s   # type: ignore[attr-defined]
        return wrapped

    return decorator


# ============================================================================
# Test-mode arming via debug probe
# ============================================================================
# R-25-exec (2026-05-13, Approach A): the bench tier no longer exists.
# Test affordances (fault_force_*, debug submenu state-mutating commands)
# live in the single flight binary, gated by rc::test_mode_active(). To
# arm test mode, a host-side script:
#   1. Halts the chip via probe (OpenOCD :3333).
#   2. Writes rc::kTestModeMagic (0x7E57BABE) to rc::g_test_mode_arm_magic.
#   3. Resets to run; test_mode_init() reads + clears the magic on boot.
#   4. Within ~30 s of boot AND while flight phase == kIdle, the gate
#      evaluates to enabled.
# See safety/test_mode.h for the three-condition AND gate.

_TEST_MODE_MAGIC = 0x7E57BABE  # matches rc::kTestModeMagic in safety/test_mode.h
_GDB_PATH = r'C:\Users\pow-w\.pico-sdk\toolchain\14_2_Rel1\bin\arm-none-eabi-gdb.exe'


def arm_test_mode_via_probe(elf_path: str,
                            gdb_port: int = 3333,
                            timeout_s: float = 20.0) -> bool:
    """Arm test mode by probe-writing the SRAM magic and resetting.

    Caller must have OpenOCD listening on `gdb_port` (default 3333) and
    a flashed firmware that matches the supplied ELF. Returns True if
    the GDB session completed without error, False otherwise.

    The runtime confirms arming via the banner / preflight VERDICT
    when the flight binary boots into kIdle within the kTestModeArmWindowMs
    window; this function only writes the magic + triggers a reset.
    """
    import subprocess
    cmds = [
        _GDB_PATH, elf_path, '-batch',
        '-ex', f'target extended-remote localhost:{gdb_port}',
        '-ex', 'monitor reset halt',
        '-ex', f'set var rc::g_test_mode_arm_magic = {_TEST_MODE_MAGIC:#x}',
        '-ex', 'monitor resume',
        '-ex', 'detach',
    ]
    try:
        r = subprocess.run(cmds, capture_output=True, text=True,
                           timeout=timeout_s)
        return r.returncode == 0
    except subprocess.TimeoutExpired:
        return False


__all__ = [
    # Watchdog
    'start_watchdog',
    # Classification
    'Role', 'Build', 'Mode', 'Target', 'Banner',
    'TARGET_VEHICLE_ANY', 'TARGET_VEHICLE_FLIGHT',
    'TARGET_STATION_ANY', 'TARGET_STATION_FLIGHT',
    'TARGET_EITHER_ANY',
    'classify_banner',
    # Port probing + navigation
    'peek_banner', 'find_target_port', 'find_vehicle_and_station_ports',
    'open_classified_port', 'enter_cli_menu',
    'ROCKETCHIP_USB_VID', 'ROCKETCHIP_USB_PID',
    # Test-mode arming (R-25-exec, Approach A)
    'arm_test_mode_via_probe',
    # Decorator
    'rc_test',
]
