#!/usr/bin/env python3
"""PreToolUse for read/glob tools — Grok-compatible graphify advisory.

See graphify_pretool_bash.py for root-cause notes (decision protocol + path).
"""
from __future__ import annotations

import json
import os
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


def _is_grok() -> bool:
    return bool(
        os.environ.get("GROK_HOOK_EVENT")
        or os.environ.get("GROK_SESSION_ID")
        or os.environ.get("GROK_WORKSPACE_ROOT")
    )


def _workspace() -> Path:
    for key in ("GROK_WORKSPACE_ROOT", "CLAUDE_PROJECT_DIR"):
        v = os.environ.get(key)
        if v:
            p = Path(v)
            if p.is_dir():
                return p
    return Path.cwd()


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


def _emit_grok_allow() -> None:
    print(json.dumps({"decision": "allow"}), flush=True)


def main() -> int:
    data = _payload()
    ti = _tool_input(data)
    ws = _workspace()

    if _is_grok():
        _emit_grok_allow()
        return 0

    if _should_nudge(ti) and (ws / "graphify-out" / "graph.json").is_file():
        msg = (
            "MANDATORY: graphify-out/graph.json exists. "
            "You MUST run graphify before reading source files. Use: "
            '`graphify query "<question>"`, `graphify explain "<concept>"`, or '
            '`graphify path "<A>" "<B>"`. Only read raw files after graphify has '
            "oriented you, or to modify/debug specific lines."
        )
        print(
            json.dumps(
                {
                    "hookSpecificOutput": {
                        "hookEventName": "PreToolUse",
                        "additionalContext": msg,
                    }
                }
            ),
            flush=True,
        )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception:
        try:
            if _is_grok():
                _emit_grok_allow()
        except Exception:
            pass
        raise SystemExit(0)
