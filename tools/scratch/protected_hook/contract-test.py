#!/usr/bin/env python3
"""
Contract test for protected-file-pretool.py (category model).

Asserts exit code 0 always, and the right permissionDecision per category:
  Hard-Protected / Historical            -> ask  (any edit)
  Cadence CHANGELOG: addition            -> allow
  Cadence CHANGELOG: destructive edit    -> ask
  Cadence PROJECT_STATUS: any edit       -> ask
  Unlisted source file                   -> allow
  Non-edit tool / empty stdin            -> allow
"""
import json
import os
import subprocess
import sys

HOOK = "C:/Users/pow-w/Documents/Rocket-Chip/scripts/hooks/protected-file-pretool.py"
PROJECT = "C:/Users/pow-w/Documents/Rocket-Chip"


def run(event):
    env = dict(os.environ)
    env["CLAUDE_PROJECT_DIR"] = PROJECT
    env.pop("GROK_WORKSPACE_ROOT", None)
    p = subprocess.run(["py", "-3", "-u", HOOK],
                       input=json.dumps(event), capture_output=True, text=True, env=env)
    try:
        dec = json.loads(p.stdout)["hookSpecificOutput"]["permissionDecision"]
    except Exception:
        dec = None
    return p.returncode, dec, p


CASES = []
def case(name, ev, exp): CASES.append((name, ev, exp))


def edit(path, old="X", new="Y"):
    return {"tool_name": "Edit",
            "tool_input": {"file_path": path, "old_string": old, "new_string": new},
            "cwd": PROJECT}


# Unlisted -> allow
case("unlisted-src", edit("src/main.cpp"), "allow")

# Hard-Protected -> ask
case("hard-sad", edit("docs/SAD.md"), "ask")
case("hard-readme", edit("README.md"), "ask")
case("hard-coding-std", edit("standards/CODING_STANDARDS.md"), "ask")
case("hard-eskf-glob", edit("docs/decisions/ESKF/foo.md"), "ask")

# Historical -> ask (any edit, incl. additions)
case("hist-lessons", edit("docs/agents/LESSONS_LEARNED.md", old="", new="new entry"), "ask")
case("hist-decisions-glob", edit("docs/decisions/SOME_DECISION.md"), "ask")
case("hist-audits-glob", edit("docs/audits/AUDIT_2026.md"), "ask")

# Cadence CHANGELOG: pure addition -> allow ; destructive -> ask
case("cadence-changelog-add",
     edit("CHANGELOG.md", old="## existing\nline", new="## existing\nline\n## new entry"),
     "allow")
case("cadence-changelog-add-empty",
     edit("CHANGELOG.md", old="", new="## brand new"), "allow")
case("cadence-changelog-destructive",
     edit("CHANGELOG.md", old="## existing entry", new="## REWRITTEN entry"), "ask")
# Mid-file insertion: new entry inserted BETWEEN header and prior entry -> allow
case("cadence-changelog-insert-middle",
     edit("CHANGELOG.md",
          old="# Changelog\n\n### old-entry",
          new="# Changelog\n\n### new-entry\n\n### old-entry"), "allow")
# Line-level destructive: a line removed -> ask
case("cadence-changelog-delete-line",
     edit("CHANGELOG.md",
          old="# Changelog\n\n### keep\n\n### remove-me",
          new="# Changelog\n\n### keep"), "ask")

# Cadence PROJECT_STATUS: any edit -> ask
case("cadence-projstatus-add",
     edit("docs/PROJECT_STATUS.md", old="", new="appended"), "ask")
case("cadence-projstatus-edit",
     edit("docs/PROJECT_STATUS.md", old="Phase 2", new="Phase 3"), "ask")

# Whiteboard removed from protection -> allow
case("wb-unprotected", edit("AGENT_WHITEBOARD.md", old="row", new="changed"), "allow")

# Non-edit tool -> allow
case("read-noop", {"tool_name": "Read", "tool_input": {"file_path": "docs/SAD.md"}, "cwd": PROJECT}, "allow")

# Empty stdin -> allow
case("empty", {}, "allow")

# MultiEdit on hard-protected -> ask
case("multiedit-hard",
     {"tool_name": "MultiEdit",
      "tool_input": {"file_path": "docs/IVP.md", "edits": [{"old_string": "a", "new_string": "b"}]},
      "cwd": PROJECT}, "ask")

# Write (overwrite) to CHANGELOG -> ask (destructive: full overwrite of existing file)
case("changelog-write-overwrite",
     {"tool_name": "Write", "tool_input": {"file_path": "CHANGELOG.md", "content": "x"}, "cwd": PROJECT},
     "ask")


def main():
    passed = failed = 0
    for name, ev, exp in CASES:
        code, dec, p = run(ev)
        ok = (code == 0) and (dec == exp)
        print(f"[{'PASS' if ok else 'FAIL'}] {name}: exit={code} decision={dec} (want {exp})")
        if ok:
            passed += 1
        else:
            failed += 1
            print(f"        stdout={p.stdout.strip()!r}")
            print(f"        stderr={p.stderr.strip()!r}")
    print(f"\n{passed} passed, {failed} failed")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
