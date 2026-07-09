#!/usr/bin/env python3
"""PreToolUse (Read/Glob/list/grep-like): advisory graphify nudge.

Always exit 0 — see graphify_pretool_bash.py header.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

_EXTS = {
    ".py",
    ".js",
    ".ts",
    ".tsx",
    ".jsx",
    ".go",
    ".rs",
    ".java",
    ".c",
    ".h",
    ".cpp",
    ".hpp",
    ".cc",
    ".cs",
    ".md",
    ".sh",
    ".txt",
    ".cmake",
}


def _payload() -> dict:
    try:
        raw = sys.stdin.read()
        if not raw.strip():
            return {}
        return json.loads(raw)
    except Exception:
        return {}


def _tool_input(data: dict) -> dict:
    ti = data.get("toolInput") or data.get("tool_input") or data
    return ti if isinstance(ti, dict) else {}


def _should_nudge(ti: dict) -> bool:
    vals = [
        str(ti.get("file_path") or ti.get("target_file") or ""),
        str(ti.get("path") or ""),
        str(ti.get("pattern") or ""),
        str(ti.get("glob") or ""),
    ]
    joined = " ".join(vals).lower().replace("\\", "/")
    if "graphify-out/" in joined:
        return False
    for v in vals:
        if not v:
            continue
        name = v.lower().replace("\\", "/").rsplit("/", 1)[-1]
        if "." in name and ("." + name.rsplit(".", 1)[-1]) in _EXTS:
            return True
    return False


def main() -> int:
    data = _payload()
    ti = _tool_input(data)
    if not _should_nudge(ti):
        return 0
    if not (Path.cwd() / "graphify-out" / "graph.json").is_file():
        return 0

    msg = (
        "MANDATORY: graphify-out/graph.json exists. "
        "You MUST run graphify before reading source files. Use: "
        '`graphify query "<question>"`, `graphify explain "<concept>"`, or '
        '`graphify path "<A>" "<B>"`. Only read raw files after graphify has '
        "oriented you, or to modify/debug specific lines."
    )
    out = {
        "hookSpecificOutput": {
            "hookEventName": "PreToolUse",
            "additionalContext": msg,
        }
    }
    print(json.dumps(out), flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        sys.exit(0)
