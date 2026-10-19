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
Pure stdlib, no serial I/O.
"""
from __future__ import annotations

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


class BoardConfigError(ValueError):
    """boards.local.json exists but is not usable. Never silently ignored."""


@dataclass(frozen=True)
class Board:
    serial: str          # upper-case hex, as the USB serial prints
    role: str            # 'vehicle' / 'station'
    probe: bool = False  # debug probe attached to this board


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
        out[ser] = Board(ser, role, probe)
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
