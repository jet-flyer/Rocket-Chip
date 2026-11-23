"""Board roles from local config, not code.

scripts/ci/boards.local.json (gitignored; copy boards.example.json) maps each
board's USB serial to a role (vehicle / station) and says whether the debug
probe is attached to it. RC_BOARDS_CONFIG overrides the path.

Lookup order for a role's USB serial:
  1. boards.local.json, when it exists (strict: only the listed serial is
     used for that role; a board whose serial is not listed is reported,
     never guessed into a role).
  2. RC_VEHICLE_USB_SERIAL / RC_STATION_USB_SERIAL.
  3. The existing banner peek (VID:PID + banner classification).

The probe stays optional. The probe flag only says a probe MAY be on that
board; the gate still identifies the probe target by chip ID and offers
SWD only when the chip ID matches a board flagged "probe": true.
Expected hardware (Nathan, 2026-11-23): a board entry may name its profile
("board"). board_profiles.json (tracked) gives each part a level: required
(the IMU and baro, any supported part), expected, optional (radio, GPS) or
absent. gate_inhibits.json (tracked) holds per-branch entries: a warning id to
show as INHIBITED, or a role:part that is not fitted for that work (N/A by
name; a required part cannot be marked). Entries match the pushed-to branch,
so they end when it merges; an optional until date ends them sooner.
Failures are never inhibited. The bench gets the result as --expect.
Pure stdlib, no serial I/O.
"""
from __future__ import annotations

import datetime
import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Tuple

CONFIG_ENV = 'RC_BOARDS_CONFIG'
DEFAULT_PATH = Path(__file__).resolve().with_name('boards.local.json')
ROLE_ENV = {'vehicle': 'RC_VEHICLE_USB_SERIAL', 'station': 'RC_STATION_USB_SERIAL'}
SOURCE_FILE = 'boards.local.json'
SOURCE_BANNER = 'banner peek'

# Expected hardware (Nathan, 2026-11-23): each board in boards.local.json may
# name its board profile ("board"). Profiles (board_profiles.json, tracked)
# say which parts that board normally carries; gate_inhibits.json (tracked)
# holds per-branch exceptions. No "board" key: today's bench behaviour.
PROFILES_FILE = 'board_profiles.json'
INHIBITS_FILE = 'gate_inhibits.json'
PARTS = ('imu', 'baro', 'radio', 'gps')
# required: must be detected, cannot be marked not fitted.
# expected: must be detected unless a branch inhibit marks it not fitted.
# absent:   not expected; if detected, a warning (never a fail).
LEVELS = ('required', 'expected', 'optional', 'absent')
# What a bench run is told per part (the --expect argument).
FITTED, NOT_FITTED, ABSENT = 'fitted', 'not-fitted', 'absent'
# optional: usually fitted but never mandatory (GPS, Nathan 2026-11-23).
# Detected = checked; not detected = N/A by name; never a fail or warning.
OPTIONAL = 'optional'
EXPECT_STATES = (FITTED, NOT_FITTED, OPTIONAL, ABSENT)
INHIBIT_KINDS = ('warning', 'not-fitted')


class BoardConfigError(ValueError):
    """boards.local.json exists but is not usable. Never silently ignored."""


@dataclass(frozen=True)
class Board:
    serial: str          # upper-case hex, as the USB serial prints
    role: str            # 'vehicle' / 'station'
    probe: bool = False  # debug probe attached to this board
    board: Optional[str] = None  # board profile name (board_profiles.json)


def config_path(env: Mapping[str, str] = os.environ) -> Path:
    return Path(env[CONFIG_ENV]) if env.get(CONFIG_ENV) else DEFAULT_PATH


def load(path: Optional[Path] = None,
         env: Mapping[str, str] = os.environ) -> Optional[Dict[str, Board]]:
    """USB serial (upper) -> Board, or None when the file does not exist.
    A file that exists but is malformed raises BoardConfigError."""
    p = Path(path) if path is not None else config_path(env)
    if not p.is_file():
        return None
    try:
        data = json.loads(p.read_text(encoding='utf-8'))
    except (OSError, ValueError) as e:
        raise BoardConfigError(f'{p}: not valid JSON ({e})') from e
    rows = data.get('boards') if isinstance(data, dict) else None
    if not isinstance(rows, list):
        raise BoardConfigError(f'{p}: expected {{"boards": [...]}}')
    out: Dict[str, Board] = {}
    roles_seen: Dict[str, str] = {}
    for i, r in enumerate(rows):
        if not isinstance(r, dict):
            raise BoardConfigError(f'{p}: boards[{i}] is not an object')
        ser = str(r.get('serial', '')).strip().upper()
        role = r.get('role')
        probe = r.get('probe', False)
        board = r.get('board')
        if board is not None and (not isinstance(board, str) or not board.strip()):
            raise BoardConfigError(f'{p}: boards[{i}] board must be a profile name')
        if not ser:
            raise BoardConfigError(f'{p}: boards[{i}] has no serial')
        if role not in ROLE_ENV:
            raise BoardConfigError(f'{p}: boards[{i}] role {role!r} is not '
                                   f'one of {sorted(ROLE_ENV)}')
        if not isinstance(probe, bool):
            raise BoardConfigError(f'{p}: boards[{i}] probe must be true/false')
        if ser in out:
            raise BoardConfigError(f'{p}: serial {ser} listed twice')
        if role in roles_seen:
            raise BoardConfigError(f'{p}: role {role} listed for both '
                                   f'{roles_seen[role]} and {ser}; one board per role')
        roles_seen[role] = ser
        out[ser] = Board(ser, role, probe, board.strip() if board else None)
    return out


def role_serial(role: str, cfg: Optional[Dict[str, Board]],
                env: Mapping[str, str] = os.environ) -> Tuple[Optional[str], str, bool]:
    """(serial or None, source, strict). strict: only that serial may be
    used for the role (config present). Not strict: prefer it, else banner."""
    if cfg is not None:
        hit = next((b.serial for b in cfg.values() if b.role == role), None)
        return hit, SOURCE_FILE, True
    v = (env.get(ROLE_ENV[role]) or '').strip()
    if v:
        return v.upper(), ROLE_ENV[role], False
    return None, SOURCE_BANNER, False


def role_of(serial: str, cfg: Optional[Dict[str, Board]]) -> Optional[str]:
    if cfg is None or not serial:
        return None
    b = cfg.get(serial.upper())
    return b.role if b else None


def unknown_serials(attached: Iterable[str],
                    cfg: Optional[Dict[str, Board]]) -> List[str]:
    """Attached serials not listed in the config (empty without a config)."""
    if cfg is None:
        return []
    return sorted({s.upper() for s in attached if s and s.upper() not in cfg})


def probe_flagged(serial: str, cfg: Optional[Dict[str, Board]]) -> bool:
    b = cfg.get(serial.upper()) if (cfg is not None and serial) else None
    return bool(b and b.probe)


def unknown_message(serial: str) -> str:
    return (f'board_config: USB serial {serial} is not in {SOURCE_FILE}: '
            'reported, not used for any role. Add it to the file to use it.')


# ---------------------------------------------------------------------------
# Expected hardware: board profiles + per-branch inhibits
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Profile:
    name: str
    role: str
    parts: Mapping[str, str]  # part -> level


@dataclass(frozen=True)
class Inhibit:
    branch: str
    kind: str              # 'warning' or 'not-fitted'
    target: str            # warning id, or '<role>:<part>'
    reason: str
    until: Optional[str]   # optional YYYY-MM-DD (inclusive)


def _json(text: str, where: str):
    try:
        return json.loads(text)
    except ValueError as e:
        raise BoardConfigError(f'{where}: not valid JSON ({e})') from e


def parse_profiles(text: str, where: str = PROFILES_FILE) -> Dict[str, Profile]:
    data = _json(text, where)
    rows = data.get('profiles') if isinstance(data, dict) else None
    if not isinstance(rows, dict) or not rows:
        raise BoardConfigError(f'{where}: expected {{"profiles": {{name: ...}}}}')
    out: Dict[str, Profile] = {}
    for name, r in rows.items():
        if not isinstance(r, dict):
            raise BoardConfigError(f'{where}: profile {name} is not an object')
        role = r.get('role')
        if role not in ROLE_ENV:
            raise BoardConfigError(f'{where}: profile {name} role {role!r} is not '
                                   f'one of {sorted(ROLE_ENV)}')
        parts = r.get('parts')
        if not isinstance(parts, dict):
            raise BoardConfigError(f'{where}: profile {name} has no parts object')
        for part, level in parts.items():
            if part not in PARTS:
                raise BoardConfigError(f'{where}: profile {name} part {part!r} is '
                                       f'not one of {list(PARTS)}')
            if level not in LEVELS:
                raise BoardConfigError(f'{where}: profile {name} {part} level '
                                       f'{level!r} is not one of {list(LEVELS)}')
        missing = [x for x in PARTS if x not in parts]
        if missing:
            raise BoardConfigError(f'{where}: profile {name} does not say '
                                   f'{", ".join(missing)} (every part is listed)')
        out[name] = Profile(name, role, dict(parts))
    return out


def parse_inhibits(text: str, where: str = INHIBITS_FILE) -> List[Inhibit]:
    data = _json(text, where)
    rows = data.get('inhibits') if isinstance(data, dict) else None
    if not isinstance(rows, list):
        raise BoardConfigError(f'{where}: expected {{"inhibits": [...]}}')
    out: List[Inhibit] = []
    for i, r in enumerate(rows):
        if not isinstance(r, dict):
            raise BoardConfigError(f'{where}: inhibits[{i}] is not an object')
        branch, kind = r.get('branch'), r.get('kind')
        reason, until = r.get('reason'), r.get('until')
        if not isinstance(branch, str) or not branch.strip():
            raise BoardConfigError(f'{where}: inhibits[{i}] needs a branch')
        if kind not in INHIBIT_KINDS:
            raise BoardConfigError(f'{where}: inhibits[{i}] kind {kind!r} is not '
                                   f'one of {list(INHIBIT_KINDS)}')
        if not isinstance(reason, str) or not reason.strip():
            raise BoardConfigError(f'{where}: inhibits[{i}] needs a reason')
        if until is not None:
            try:
                datetime.date.fromisoformat(str(until))
            except ValueError as e:
                raise BoardConfigError(f'{where}: inhibits[{i}] until must be '
                                       f'YYYY-MM-DD') from e
        if kind == 'warning':
            target = r.get('id')
            if not isinstance(target, str) or not target.strip():
                raise BoardConfigError(f'{where}: inhibits[{i}] warning needs an id')
        else:
            role, part = r.get('role'), r.get('part')
            if role not in ROLE_ENV or part not in PARTS:
                raise BoardConfigError(f'{where}: inhibits[{i}] not-fitted needs a '
                                       f'role ({sorted(ROLE_ENV)}) and a part '
                                       f'({list(PARTS)})')
            target = f'{role}:{part}'
        out.append(Inhibit(branch.strip(), kind, target.strip(), reason.strip(),
                           str(until) if until is not None else None))
    return out


def branch_of(ref: str) -> str:
    """The branch name in a ref: refs/heads/foo/bar (the pushed-to ref in
    the hook) or refs/remotes/origin/foo/bar (the upstream in --plan) ->
    foo/bar. Anything else is returned as is."""
    if ref.startswith('refs/heads/'):
        return ref[len('refs/heads/'):]
    if ref.startswith('refs/remotes/'):
        return ref[len('refs/remotes/'):].partition('/')[2]
    return ref


def active_inhibits(inhibits: Iterable[Inhibit], branch: str,
                    today: datetime.date) -> Tuple[List[Inhibit], List[Inhibit]]:
    """(active, expired) for a push to branch. Other branches' entries are
    neither: they end when their branch merges (a push to main never
    matches them)."""
    act: List[Inhibit] = []
    old: List[Inhibit] = []
    for x in inhibits:
        if x.branch != branch:
            continue
        if x.until and datetime.date.fromisoformat(x.until) < today:
            old.append(x)
        else:
            act.append(x)
    return act, old


def expectations(profile: Profile, role: str,
                 active: Iterable[Inhibit]) -> Dict[str, str]:
    """part -> fitted / not-fitted / absent for one bench run."""
    if profile.role != role:
        raise BoardConfigError(f'board profile {profile.name} is a {profile.role} '
                               f'board, but it is listed as the {role}')
    off = {x.target.split(':', 1)[1] for x in active
           if x.kind == 'not-fitted' and x.target.split(':', 1)[0] == role}
    out: Dict[str, str] = {}
    for part in PARTS:
        level = profile.parts[part]
        if part in off:
            if level == 'required':
                raise BoardConfigError(f'{role} {part} is required on '
                                       f'{profile.name}; it cannot be marked '
                                       f'not fitted ({INHIBITS_FILE})')
            if level == 'absent':
                raise BoardConfigError(f'{role} {part} is already absent on '
                                       f'{profile.name}; remove the not-fitted '
                                       f'entry ({INHIBITS_FILE})')
            out[part] = NOT_FITTED
        else:
            out[part] = {'absent': ABSENT, 'optional': OPTIONAL}.get(level, FITTED)
    return out


def format_expect(exp: Mapping[str, str]) -> str:
    return ','.join(f'{p}={exp[p]}' for p in PARTS if p in exp)


def parse_expect(text: Optional[str]) -> Optional[Dict[str, str]]:
    """--expect "imu=fitted,radio=not-fitted,..." -> dict; None/'' -> None
    (legacy bench behaviour). Every part must be named once."""
    if not text:
        return None
    out: Dict[str, str] = {}
    for item in text.split(','):
        part, _, state = item.strip().partition('=')
        if part not in PARTS or state not in EXPECT_STATES or part in out:
            raise BoardConfigError(f'--expect: bad item {item!r} (parts '
                                   f'{list(PARTS)}, states {list(EXPECT_STATES)})')
        out[part] = state
    missing = [p for p in PARTS if p not in out]
    if missing:
        raise BoardConfigError(f'--expect: missing {", ".join(missing)}')
    return out


# Bench output lines the gate reads (bench scripts print, gate parses).
HW_NA_PREFIX = 'hw_na:'      # hw_na: <check name> (<part> not fitted)
HW_WARN_PREFIX = 'hw_warn:'  # hw_warn: <warning id> <message>


def hw_na_line(check: str, part: str) -> str:
    return f'{HW_NA_PREFIX} {check} ({part} not fitted)'


def hw_warn_line(role: str, part: str, what: str) -> str:
    return f'{HW_WARN_PREFIX} hw-unexpected:{role}:{part} {what}'
