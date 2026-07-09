#!/usr/bin/env python3
"""PreToolUse (Bash/run_terminal_command): advisory graphify nudge.

Always exit 0. Grok records any non-zero as UI "failed with exit code N"
even when fail-open (docs/user-guide/10-hooks.md). Windows Store python3
stubs and bash one-liners were producing that noise.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path


def _payload() -> dict:
    try:
        raw = sys.stdin.read()
        if not raw.strip():
            return {}
        return json.loads(raw)
    except Exception:
        return {}


def _command(data: dict) -> str:
    # Grok: toolInput / Claude: tool_input; sometimes nested under tool_input
    ti = data.get("toolInput") or data.get("tool_input") or data
    if not isinstance(ti, dict):
        return ""
    cmd = ti.get("command") or ""
    return str(cmd)


def main() -> int:
    data = _payload()
    cmd = _command(data).lower()
    needles = (
        "grep",
        "rg ",
        "ripgrep",
        " find ",
        "find ",
        "fd ",
        "ack ",
        " ag ",
    )
    # also match leading rg/find
    hit = any(n.strip() in cmd for n in needles) or cmd.startswith(
        ("rg ", "rg\t", "find ", "fd ", "grep ", "ack ", "ag ")
    )
    if not hit:
        return 0

    root = Path.cwd()
    if not (root / "graphify-out" / "graph.json").is_file():
        return 0

    msg = (
        "MANDATORY: graphify-out/graph.json exists. "
        'You MUST run `graphify query "<question>"` before grepping raw files. '
        "Only grep after graphify has oriented you, or to modify/debug specific lines."
    )
    out = {
        "hookSpecificOutput": {
            "hookEventName": "PreToolUse",
            "additionalContext": msg,
        }
    }
    # Also emit Grok-friendly allow + context if host ignores hookSpecificOutput
    print(json.dumps(out), flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        sys.exit(0)
