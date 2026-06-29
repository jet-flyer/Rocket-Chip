#!/usr/bin/env python3
"""
PreToolUse hook: protected-file guard for RocketChip.

Replaces the old "scan the chat transcript for an authorization phrase" design
(which was structurally leaky -- "don't edit X" false-positives, cross-file
bleed, stale auth, self-authorization from the assistant's own narration) with
a deterministic, category-driven model that escalates to the user via the
`ask` permission decision instead of guessing.

------------------------------------------------------------------------------
CATEGORIES  (parsed from docs/agents/PROTECTED_FILES.md `###` headers)
------------------------------------------------------------------------------
  Hard-Protected      -> any edit  -> ASK
  Historical (frozen) -> any edit  -> ASK
  Checklist-cadence   -> per-file interim rule:
        CHANGELOG.md            : pure addition -> ALLOW; destructive -> ASK
        docs/PROJECT_STATUS.md  : any edit      -> ASK
     (A future session-checklist skill will manage these properly.)
  Unlisted            -> ALLOW

"Pure addition" = the edit only inserts text: the entire existing `old_string`
is preserved verbatim inside `new_string` (substring), OR old_string is empty.
Any edit that removes or rewrites existing bytes is "destructive". A Write to an
existing file (full overwrite) is treated as destructive; creating a new file is
an addition.

------------------------------------------------------------------------------
HOOK CONTRACT  (Claude Code; Grok Build CLI is config-compatible)
------------------------------------------------------------------------------
Decision via stdout JSON on exit 0:
    {"hookSpecificOutput": {"hookEventName": "PreToolUse",
        "permissionDecision": "allow" | "ask" | "deny",
        "permissionDecisionReason": "..."}}
  exit 0  -> stdout JSON parsed (allow/ask/deny). "ask" shows the user a prompt
             (verified working on this build via capability probe 2026-06-28).
  exit 2  -> hard block, stdout ignored, stderr shown. We don't use it.
  other   -> non-blocking error. We FAIL OPEN (allow, exit 0) on any internal
             error so a bug in the guard never wedges editing.

Project root: CLAUDE_PROJECT_DIR (Claude Code) / GROK_WORKSPACE_ROOT (Grok),
falling back to the stdin `cwd`, then process cwd -- so the wired command does
not depend on any one env var expanding.
"""

import json
import os
import sys
import traceback
from pathlib import Path

# Set ROCKETCHIP_HOOK_DEBUG=1 to append invocation traces next to this script.
_DEBUG = os.environ.get("ROCKETCHIP_HOOK_DEBUG") == "1"

CAT_HARD = "hard"
CAT_HISTORICAL = "historical"
CAT_CADENCE = "cadence"

# Maps a `###` header in PROTECTED_FILES.md to a category key.
_HEADER_CATEGORY = {
    "hard-protected": CAT_HARD,
    "historical (frozen)": CAT_HISTORICAL,
    "historical": CAT_HISTORICAL,
    "checklist-cadence": CAT_CADENCE,
}


def _debug_log(msg: str) -> None:
    if not _DEBUG:
        return
    try:
        log = Path(__file__).resolve().parent / "protected-hook-debug.log"
        with log.open("a", encoding="utf-8") as f:
            f.write(msg + "\n")
    except Exception:
        pass


def normalize_path(p: str) -> str:
    if not p:
        return ""
    p = p.replace("\\", "/").strip()
    if p.startswith("./"):
        p = p[2:]
    return p.lower()


def load_protected_categories(workspace_root: Path) -> dict:
    """Parse PROTECTED_FILES.md into {normalized_path_or_glob: category}.

    Only mines lines under the "## Protected File List" section, and assigns
    each entry to the most recent `###` category header. `####` lines are
    visual grouping (ignored for categorization). Returns {} if the doc is
    absent (safe as a global hook -- no protection where no list).
    """
    md = workspace_root / "docs" / "agents" / "PROTECTED_FILES.md"
    if not md.exists():
        return {}

    text = md.read_text(encoding="utf-8", errors="ignore")
    out = {}
    in_list = False
    category = None

    for raw in text.splitlines():
        s = raw.strip()
        if s.startswith("## Protected File List"):
            in_list = True
            continue
        if in_list and s.startswith("## ") and not s.startswith("### "):
            break  # left the Protected File List section
        if not in_list:
            continue

        if s.startswith("### "):
            header = s[4:].strip().lower()
            category = _HEADER_CATEGORY.get(header)
            continue
        if s.startswith("#### "):
            continue  # visual grouping only

        if s.startswith("- ") and category:
            # entry form: - `path` - description   (or - path - description)
            body = s[2:].strip()
            # take the first backticked token if present, else up to " - "
            if body.startswith("`"):
                end = body.find("`", 1)
                cand = body[1:end] if end > 0 else body
            else:
                cand = body.split(" - ")[0].strip().strip("`")
            cand = cand.strip()
            if ("/" in cand or "." in cand or cand.endswith("/*")) and len(cand) > 1:
                out[normalize_path(cand)] = category

    # The list doc itself is always hard-protected even if mis-edited.
    out.setdefault("docs/agents/protected_files.md", CAT_HARD)
    return out


def categorize(target: str, cat_map: dict):
    """Return the category for a target path, or None if unprotected."""
    t = normalize_path(target)
    if not t:
        return None
    for entry, cat in cat_map.items():
        e = normalize_path(entry)
        if not e:
            continue
        if e.endswith("/*") or e.endswith("/"):
            prefix = e.rstrip("/*").rstrip("/")
            if t == prefix or t.startswith(prefix + "/"):
                return cat
        else:
            if t == e or t.endswith("/" + e):
                return cat
            # basename match for top-level files (e.g. README.md, CHANGELOG.md)
            if "/" not in e and t.endswith("/" + e):
                return cat
    return None


def is_pure_addition(tool_name: str, tool_input: dict, target_abs: str) -> bool:
    """True if this edit only inserts content (no existing bytes removed)."""
    tn = tool_name.lower()

    if "write" in tn:
        # Write creates or fully overwrites. New file = addition; overwrite of
        # an existing non-empty file = destructive.
        try:
            p = Path(target_abs)
            if not p.exists() or p.stat().st_size == 0:
                return True
        except Exception:
            return False
        return False

    if "multiedit" in tn or "multi_edit" in tn:
        edits = tool_input.get("edits") or []
        if not edits:
            return False
        return all(_edit_is_addition(e.get("old_string", ""), e.get("new_string", ""))
                   for e in edits)

    # Edit / search_replace
    return _edit_is_addition(tool_input.get("old_string", ""),
                             tool_input.get("new_string", ""))


def _edit_is_addition(old: str, new: str) -> bool:
    """True if going old -> new only INSERTS content (nothing removed/changed).

    Substring ('old in new') only catches pure append/prepend; it wrongly flags
    a legitimate insertion BETWEEN existing lines (e.g. a new CHANGELOG entry at
    the top, between the header and the previous entry) as destructive. Use a
    line-level diff: the edit is an addition iff every diff op is 'equal' or
    'insert' -- no 'delete' or 'replace'.
    """
    old = old or ""
    new = new or ""
    if old == "":
        return True
    if old == new:
        return True
    import difflib
    # Compare line CONTENT (no line endings) so that appending after a final
    # line lacking a trailing newline -- which turns "line" into "line\n" -- is
    # still seen as a pure addition rather than a 'replace' of that last line.
    old_lines = old.splitlines()
    new_lines = new.splitlines()
    sm = difflib.SequenceMatcher(a=old_lines, b=new_lines, autojunk=False)
    for tag, _i1, _i2, _j1, _j2 in sm.get_opcodes():
        if tag in ("delete", "replace"):
            return False
    return True


# --------------------------------------------------------------------------
# Decision emission
# --------------------------------------------------------------------------
def _emit(decision: str, reason: str = "") -> None:
    _debug_log("decision=%s reason=%s" % (decision, reason))
    out = {"hookSpecificOutput": {
        "hookEventName": "PreToolUse",
        "permissionDecision": decision,
    }}
    if reason:
        out["hookSpecificOutput"]["permissionDecisionReason"] = reason
    print(json.dumps(out))
    sys.exit(0)


_PROTECTED_REL = Path("docs") / "agents" / "PROTECTED_FILES.md"


def resolve_workspace_root(file_path: str, event: dict) -> Path:
    """Find the repo root that owns the file being edited.

    Critical: do NOT blindly trust CLAUDE_PROJECT_DIR -- Claude Code sets it to
    the launch dir, which may be an ancestor of the actual repo (e.g. launched
    from the home dir with the repo as an additional working dir). That made the
    hook read a non-existent PROTECTED_FILES.md and allow everything.

    Strategy, most-specific first:
      1. Walk UP from the target file's directory; first ancestor that contains
         docs/agents/PROTECTED_FILES.md wins. (Works regardless of launch dir.)
      2. Else try each candidate root (cwd, CLAUDE_PROJECT_DIR, GROK_WORKSPACE_ROOT)
         that actually has the doc.
      3. Else fall back to cwd/env (yields 0 entries -> fail-open allow).
    """
    try:
        start = Path(file_path)
        if not start.is_absolute():
            base = event.get("cwd") or os.getcwd()
            start = (Path(base) / start)
        start = start.resolve()
        for anc in [start] + list(start.parents):
            if (anc / _PROTECTED_REL).exists():
                return anc
    except Exception:
        pass

    for cand in (event.get("cwd"),
                 os.environ.get("CLAUDE_PROJECT_DIR"),
                 os.environ.get("GROK_WORKSPACE_ROOT"),
                 os.getcwd()):
        if not cand:
            continue
        try:
            p = Path(cand).resolve()
            if (p / _PROTECTED_REL).exists():
                return p
        except Exception:
            continue

    return Path(event.get("cwd") or os.getcwd()).resolve()


def main():
    try:
        raw = sys.stdin.read()
        event = json.loads(raw) if raw.strip() else {}
        _debug_log("--- invoked --- tool=%r" % (event.get("tool_name") or event.get("toolName")))

        tool_name = (event.get("tool_name") or event.get("toolName") or "").lower()
        tool_input = event.get("tool_input") or event.get("toolInput") or {}
        if not isinstance(tool_input, dict):
            tool_input = {}

        edit_tools = ("search_replace", "edit", "write", "multiedit", "multi_edit")
        if not any(t in tool_name for t in edit_tools):
            _emit("allow")

        file_path = tool_input.get("file_path") or tool_input.get("path") or ""
        if not file_path:
            _emit("allow")

        workspace_root = resolve_workspace_root(str(file_path), event)
        cat_map = load_protected_categories(workspace_root)
        category = categorize(str(file_path), cat_map)
        _debug_log("ws=%r entries=%d file=%r category=%r"
                   % (str(workspace_root), len(cat_map), file_path, category))

        if category is None:
            _emit("allow")  # unlisted -> normal flow

        name = Path(str(file_path)).name

        if category == CAT_HARD:
            _emit("ask", f"'{file_path}' is a hard-protected file. Approve this edit?")

        if category == CAT_HISTORICAL:
            _emit("ask",
                  f"'{file_path}' is a historical/frozen record (snapshot or "
                  f"append-only). Approve editing existing content?")

        if category == CAT_CADENCE:
            nlow = normalize_path(str(file_path))
            # PROJECT_STATUS amends in place -> always ask (interim).
            if nlow.endswith("project_status.md"):
                _emit("ask",
                      f"'{file_path}' is a checklist-cadence doc edited in place. "
                      f"Approve this edit? (A session-checklist skill will manage "
                      f"this later.)")
            # CHANGELOG and other append-only cadence docs: additions pass.
            if is_pure_addition(tool_name, tool_input, str(file_path)):
                _emit("allow")
            _emit("ask",
                  f"'{file_path}' is append-only (checklist-cadence). This edit "
                  f"removes or rewrites existing content. Approve?")

        # Unknown category label -> be safe, ask.
        _emit("ask", f"'{file_path}' is protected ({category}). Approve this edit?")

    except SystemExit:
        raise
    except Exception:
        traceback.print_exc(file=sys.stderr)
        # Fail open so a guard bug never wedges editing.
        try:
            _emit("allow")
        except SystemExit:
            raise
        except Exception:
            print(json.dumps({"hookSpecificOutput": {
                "hookEventName": "PreToolUse", "permissionDecision": "allow"}}))
            sys.exit(0)


if __name__ == "__main__":
    main()
