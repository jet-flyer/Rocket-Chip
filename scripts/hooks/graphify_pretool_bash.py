#!/usr/bin/env python3
"""PreToolUse for shell tools: graphify-first advisory.

Root cause of project/settings pre_tool_use[0] exit-code noise on Grok:
  Grok PreToolUse expects stdout decision JSON:
    {"decision": "allow"} | {"decision": "deny", "reason": "..."}
  (see ~/.grok/docs/user-guide/10-hooks.md)

  Graphify's stock Claude one-liners emitted Claude-only:
    {"hookSpecificOutput":{"hookEventName":"PreToolUse","additionalContext":"..."}}
  Grok treats non-decision stdout as malformed hook output → fail-open but
  records "failed with exit code 1" in the TUI even when the tool still runs.

This script:
  - Always exits 0
  - On Grok: always emits valid {"decision":"allow"} (nudge lives in .grok/rules)
  - On Claude Code: may emit hookSpecificOutput additionalContext for grep-like cmds
  - Handles toolInput (Grok) and tool_input (Claude); empty/bad stdin is fine
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
            return Path(v)
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


def _graph_exists() -> bool:
    return (_workspace() / "graphify-out" / "graph.json").is_file()


def main() -> int:
    data = _payload()
    cmd = _command(data)

    if _is_grok():
        # Valid Grok allow decision. Graphify mandate is in .grok/rules/graphify.md;
        # Grok has no documented additionalContext injection on PreToolUse.
        print(json.dumps({"decision": "allow"}), flush=True)
        return 0

    # Claude Code path: optional advisory context for search-like shell cmds
    if _is_search_cmd(cmd) and _graph_exists():
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
        # Never leave Grok without a parseable allow if we somehow error mid-flight
        if _is_grok():
            try:
                print(json.dumps({"decision": "allow"}), flush=True)
            except Exception:
                pass
        raise SystemExit(0)
