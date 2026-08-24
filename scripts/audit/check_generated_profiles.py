#!/usr/bin/env python3
"""A/B: committed generate_profile.py outputs vs a fresh generate to tempfile.

Does not overwrite committed headers. Unexpected drift fails.
Host ctest: scripts_generated_profiles.
"""
from __future__ import annotations

import difflib
import hashlib
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
GEN = ROOT / "scripts" / "generate_profile.py"

PAIRS = [
    (ROOT / "profiles" / "rocket.cfg",
     "kDefaultRocketProfile",
     ROOT / "src" / "flight_director" / "mission_profile_data.h"),
    (ROOT / "profiles" / "hab.cfg",
     "kHabProfile",
     ROOT / "test" / "test_hab_profile_data.h"),
]


def _run(cfg: Path, symbol: str, out: Path) -> None:
    r = subprocess.run(
        [sys.executable, str(GEN), str(cfg), "--symbol", symbol, "--output", str(out)],
        cwd=ROOT, capture_output=True, text=True)
    if r.returncode != 0:
        sys.stderr.write(r.stdout + r.stderr)
        raise SystemExit(r.returncode)
    for stream in (r.stdout, r.stderr):
        for line in stream.splitlines():
            if "unknown field" in line.lower() or line.startswith("WARNING"):
                print(line)


def _bytes(p: Path) -> bytes:
    return p.read_bytes().replace(b"\r\n", b"\n")


def main() -> int:
    failed = 0
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        for cfg, symbol, committed in PAIRS:
            fresh = tmp / committed.name
            _run(cfg, symbol, fresh)
            a = _bytes(committed)
            b = _bytes(fresh)
            rel = committed.relative_to(ROOT).as_posix()
            if a == b:
                print(f"OK {rel}")
                continue
            failed += 1
            print(f"DRIFT: {rel} != generate({cfg.name})")
            print(f"  committed sha256 {hashlib.sha256(a).hexdigest()[:16]}")
            print(f"  generated sha256 {hashlib.sha256(b).hexdigest()[:16]}")
            diff = difflib.unified_diff(
                a.decode("utf-8").splitlines(True),
                b.decode("utf-8").splitlines(True),
                fromfile=f"committed/{rel}",
                tofile=f"generate/{cfg.name}",
                n=3)
            shown = 0
            for line in diff:
                sys.stdout.write(line)
                shown += 1
                if shown >= 200:
                    print("... [diff truncated]")
                    break
    if failed:
        print("Re-run: python scripts/generate_profile.py <cfg> --symbol ... --output ...")
        print("Do not hand-edit generated headers. Edit the .cfg or the generator.")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
