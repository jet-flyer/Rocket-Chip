#!/usr/bin/env python3
"""PreToolUse for shell tools — Grok-compatible graphify advisory.

Root causes of project/settings pre_tool_use[0] exit-code-1 on Grok:

1) Protocol: Grok expects stdout ``{"decision":"allow"|"deny"}``. Claude-style
   ``hookSpecificOutput`` / ``additionalContext`` is malformed on Grok and is
   recorded as fail-open failure in the TUI (even when the tool still runs).
   See ~/.grok/docs/user-guide/10-hooks.md

2) Spawn path: ``${CLAUDE_PROJECT_DIR}/scripts/...`` often does not expand on
   Windows under Grok, so Python tried to open a missing path and exited 1/2.

Fix: run as ``python scripts/hooks/graphify_pretool_bash.py`` with cwd=workspace
(Grok sets that). Under Grok env always emit decision:allow. Claude Code can
still get additionalContext for search-like commands.
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path


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


def _command(data: dict) -> str:
    ti = data.get("toolInput") or data.get("tool_input") or data
    if not isinstance(ti, dict):
        return ""
    return str(ti.get("command") or "")


def _is_search_cmd(cmd: str) -> bool:
    c = f" {cmd.lower()} "
    for token in (
        " grep ",
        " rg ",
        " ripgrep ",
        " find ",
        " fd ",
        " ack ",
        " ag ",
    ):
        if token in c:
            return True
    cl = cmd.lower().lstrip()
    return cl.startswith(
        ("grep ", "rg ", "rg\t", "find ", "fd ", "ack ", "ag ", "ripgrep ")
    )


def _graph_exists(ws: Path) -> bool:
    return (ws / "graphify-out" / "graph.json").is_file()


def _emit_grok_allow() -> None:
    print(json.dumps({"decision": "allow"}), flush=True)


def main() -> int:
    # Always succeed for the OS; Grok only cares about decision JSON + exit 0.
    data = _payload()
    cmd = _command(data)
    ws = _workspace()

    if _is_grok():
        _emit_grok_allow()
        return 0

    # Claude Code: optional search-command nudge
    if _is_search_cmd(cmd) and _graph_exists(ws):
        msg = (
            "MANDATORY: graphify-out/graph.json exists. "
            'You MUST run `graphify query "<question>"` before grepping raw files. '
            "Only grep after graphify has oriented you, or to modify/debug specific lines."
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
