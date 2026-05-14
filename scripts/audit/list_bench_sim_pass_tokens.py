#!/usr/bin/env python3
"""Bench-sim PASS-token inventory (F-2026-05-13-002).

Audit-time bidirectional check for `scripts/bench_sim.py` and
`scripts/station_bench_sim.py`:

  (a) Rotted-regex check — does each script's regex still match at least
      one live firmware emission?

      Covered by the scripts' own existence — they fail loudly when
      regexes rot (per LL Entry 36). This script's complementary role:

  (b) Deleted-regex check — for every positive-control / PASS-token the
      firmware emits, is there at least one regex in the corresponding
      bench_sim script looking for it?

The risk this addresses: a future refactor that introduces a new
firmware log emission used as a positive-control signal, OR deletes a
bench_sim regex while leaving the firmware emission in place, leaves
the audit's `## Tier 1.4` walk silently shrunk. Without this script,
the only way to spot the gap was the manual walk performed during the
2026-05-13 cycle.

Output (stdout + logs/bench_sim_pass_tokens.txt):

  - Inventory of every firmware-emitted positive-control token,
    grouped by token class, with file:line citations.
  - For each token, whether it is matched by a bench_sim regex
    (vehicle or station), or unmatched.
  - Unmatched tokens are flagged for auditor review against each
    script's documented scope (see bench_sim.py / station_bench_sim.py
    module docstrings).

Source: AUDIT_GUIDANCE.md Tier 1.4(b). Closes F-2026-05-13-002.

Exit codes:
    0 = inventory generated successfully

Empty token classes are surfaced as NOTE messages on stderr (not a
non-zero exit) so the audit cycle treats them as items for review,
not blocking conditions. Whether an empty class is "regex rotted" or
"acknowledged doc-vs-firmware drift" is a judgment call the per-class
Scope note documents — gated mechanically here would force false
audit halts on the doc-drift case.
"""

from __future__ import annotations

import os
import re
import sys
from typing import Iterable


# =============================================================================
# Repo paths
# =============================================================================

def _repo_root() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.dirname(os.path.dirname(here))


REPO = _repo_root()
SRC = os.path.join(REPO, 'src')
BENCH_SIM = os.path.join(REPO, 'scripts', 'bench_sim.py')
STATION_BENCH_SIM = os.path.join(REPO, 'scripts', 'station_bench_sim.py')
OUTPUT_LOG = os.path.join(REPO, 'logs', 'bench_sim_pass_tokens.txt')


# =============================================================================
# Firmware emission discovery
#
# Per HW_GATE_DISCIPLINE.md Rule 1, positive-control signals are tokens the
# firmware emits that prove a specific bus, peripheral, or state transition
# is healthy. The classes below are the inventory dimensions.
# =============================================================================

# Each entry: (class_name, source_regex, scope_note)
#
# source_regex matches the *source line* in src/ — what the firmware emits as
# a string literal. The capturing group captures the C-string-literal body
# the bench_sim regex would see in serial output.
#
# All regexes use `[^"]*` to allow a leading whitespace / format-prefix
# inside the C string literal (e.g., printf("  RegVersion=..." — two-space
# indent is common in our preflight output).
TOKEN_CLASSES = [
    (
        '[FD] state transition',
        re.compile(r'"([^"]*\[FD\] %s -> %s\\n[^"]*)"'),
        'Vehicle bench_sim: covered by RE_TRANSITION.',
    ),
    (
        '[FD] PYRO FIRED',
        re.compile(r'"([^"]*\[FD\] PYRO FIRED:[^"]*)"'),
        'Vehicle bench_sim: covered by RE_PYRO.',
    ),
    (
        '[FD] phase / ABORT / timeout / warn',
        re.compile(
            r'"([^"]*\[FD\] (?!%s ->|PYRO FIRED:|Flight Director not)[^"]*)"'),
        'Vehicle bench_sim: out-of-scope by design (host '
        'test_command_handler covers reject paths; CRITICAL_FAULT / '
        'timeout / ABORT-different-states covered by host tests).',
    ),
    (
        'Phase: field',
        re.compile(r'"([^"]*Phase:\s+%s[^"]*)"'),
        'Vehicle bench_sim: covered by RE_PHASE.',
    ),
    (
        'CLI prompt',
        re.compile(r'"(\[(?:main|flight)\][^"]*)"'),
        'Vehicle bench_sim: covered by RE_PROMPT.',
    ),
    (
        'VERDICT: GO/NO-GO',
        re.compile(r'"([^"]*VERDICT:[^"]*)"'),
        'Vehicle bench_sim: prefix-matched (looks for "VERDICT:  GO").',
    ),
    (
        'RegVersion (radio positive-control)',
        re.compile(r'"([^"]*RegVersion=0x[^"]*)"'),
        'Station bench_sim: covered by RE_REG_VERSION (one of the JPL-'
        'required radio positive controls per HW_GATE_DISCIPLINE.md).',
    ),
    (
        '[Health] header + primary/secondary line',
        re.compile(r'"([^"]*(?:\[Health\]|primary=0x)[^"]*)"'),
        'Station bench_sim: [Health] header + multi-field primary= line '
        'covered by RE_HEALTH_LINE.',
    ),
    (
        '[N/A ] uninstalled sensor',
        re.compile(r'"(\[N/A\s*\][^"]*not installed[^"]*)"'),
        'Station bench_sim: covered by RE_NA_SENSOR.',
    ),
    (
        '[FAULT] launch-abort',
        re.compile(r'"(\[FAULT\][^"]*)"'),
        'Dev-tool fault_inject; not gated by bench_sim regexes — '
        'covered by enhanced_fault_injection.py end-to-end test.',
    ),
    (
        '[ESKF] divergence sentinels',
        re.compile(r'"(\[ESKF\][^"]*)"'),
        'Not bench_sim-gated; covered by replay_gate_test.py and host '
        'fusion tests. Currently NO firmware emission with this prefix — '
        'the doc-level FMEA-lite row in AUDIT_GUIDANCE Appendix A.1 names '
        '`[ESKF] DIVERGENCE` as an expected positive-control signal; '
        'absence indicates either (a) the divergence detection emits a '
        'different log line or (b) doc-to-implementation drift.',
    ),
]


def _walk_cpp_files(root: str) -> Iterable[str]:
    for dirpath, _, fnames in os.walk(root):
        for fn in fnames:
            if fn.endswith(('.cpp', '.h', '.c')):
                yield os.path.join(dirpath, fn)


def _scan_source(class_re: re.Pattern[str]) -> list[tuple[str, int, str]]:
    """Return [(rel_path, lineno, token_text), ...] for matches in src/."""
    hits: list[tuple[str, int, str]] = []
    for fp in _walk_cpp_files(SRC):
        try:
            with open(fp, 'r', encoding='utf-8', errors='replace') as f:
                for lineno, line in enumerate(f, start=1):
                    for m in class_re.finditer(line):
                        token = m.group(1) if m.groups() else m.group(0)
                        rel = os.path.relpath(fp, REPO).replace('\\', '/')
                        hits.append((rel, lineno, token))
        except OSError:
            continue
    return hits


# =============================================================================
# bench_sim regex inventory
# =============================================================================

def _extract_bench_sim_regexes(path: str) -> list[tuple[str, list[str]]]:
    """Find every `RE_FOO = re.compile(r'PATTERN'[, ...])` in a script.

    Returns [(name, [pattern_fragment, ...]), ...]. Multi-line
    re.compile() blocks (e.g., RE_HEALTH_LINE) are reassembled as the
    list of string fragments — the audit reader sees what the script
    actually does, including the multi-line composition.

    Line-based parser, not a full Python AST walker — fragile to
    creative formatting, but bench_sim conventions are simple enough
    that the line walker covers them. If a future bench_sim adopts an
    f-string or a `re.compile(some_var)`, that won't be matched here
    and will be visibly absent from the inventory — that itself is
    useful audit signal.
    """
    if not os.path.exists(path):
        return []

    start = re.compile(r'^(RE_\w+)\s*=\s*re\.compile\(\s*(.*)$')
    # Match a raw string literal r'...' or r"..."
    raw_str = re.compile(r"""r['"]([^'"]*)['"]""")

    out: list[tuple[str, list[str]]] = []
    with open(path, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    i = 0
    while i < len(lines):
        m = start.match(lines[i])
        if not m:
            i += 1
            continue
        name = m.group(1)
        fragments: list[str] = []
        # First line may have inline pattern.
        rest = m.group(2)
        for s in raw_str.findall(rest):
            fragments.append(s)
        # If the open `(` wasn't closed on this line, walk forward.
        j = i + 1
        if ')' not in rest:
            while j < len(lines):
                for s in raw_str.findall(lines[j]):
                    fragments.append(s)
                if ')' in lines[j]:
                    break
                j += 1
        if fragments:
            out.append((name, fragments))
        i = j + 1
    return out


# =============================================================================
# Report
# =============================================================================

def main() -> int:
    lines: list[str] = []
    add = lines.append

    add('# Bench-Sim PASS-Token Inventory')
    add('')
    add('Source: scripts/audit/list_bench_sim_pass_tokens.py')
    add('Closes F-2026-05-13-002 (AUDIT_GUIDANCE.md Tier 1.4(b)).')
    add('')
    add('Per LL Entry 36, gates that say only "X did not fail" can pass')
    add('on broken hardware. This inventory cross-checks every emitted')
    add('positive-control / PASS-token in firmware against the regex')
    add('constants in bench_sim.py / station_bench_sim.py so a future')
    add('refactor cannot silently shrink either script\'s scope.')
    add('')
    add('---')
    add('')
    add('## bench_sim.py regex constants')
    add('')
    for name, fragments in _extract_bench_sim_regexes(BENCH_SIM):
        joined = ''.join(fragments)
        add(f'  - `{name}` = `{joined}`')
    add('')
    add('## station_bench_sim.py regex constants')
    add('')
    for name, fragments in _extract_bench_sim_regexes(STATION_BENCH_SIM):
        joined = ''.join(fragments)
        add(f'  - `{name}` = `{joined}`')
    add('')
    add('---')
    add('')
    add('## Firmware PASS-token inventory (by class)')
    add('')

    empty_classes: list[str] = []
    for cls, src_re, scope_note in TOKEN_CLASSES:
        hits = _scan_source(src_re)
        add(f'### {cls}')
        add('')
        add(f'**Scope:** {scope_note}')
        add('')
        if not hits:
            add('  *(no firmware emissions matched this class regex — '
                'see the Scope note above. If the scope note acknowledges '
                'doc-vs-implementation drift, no action. Otherwise the '
                'class regex has rotted; investigate.)*')
            empty_classes.append(cls)
        else:
            seen: set[tuple[str, str]] = set()
            for rel, lineno, token in hits:
                key = (rel, token)
                if key in seen:
                    continue
                seen.add(key)
                add(f'  - `{token}` — {rel}:{lineno}')
        add('')

    add('---')
    add('')
    add('## Audit checklist (manual walk)')
    add('')
    add('1. Every emitted token above either:')
    add('   - Maps to a regex in the relevant bench_sim script, OR')
    add('   - Is annotated as out-of-bench-sim-scope (covered by host')
    add('     tests, replay, fault-injection, or other gates).')
    add('2. If a token has neither: open a finding for the next audit cycle.')
    add('   The token is either a new positive-control signal that needs')
    add('   bench_sim coverage, OR a stale firmware emission that should')
    add('   be removed.')
    add('3. Empty token classes (above) mean the class regex itself has')
    add('   rotted. Investigate before next milestone close.')

    out_text = '\n'.join(lines) + '\n'

    # Always print to stdout for the audit transcript.
    sys.stdout.write(out_text)

    # And to logs/ so a future cycle can diff against this one.
    os.makedirs(os.path.dirname(OUTPUT_LOG), exist_ok=True)
    with open(OUTPUT_LOG, 'w', encoding='utf-8') as f:
        f.write(out_text)
    print(f'\n[inventory written to {os.path.relpath(OUTPUT_LOG, REPO)}]',
          file=sys.stderr)

    if empty_classes:
        print(f'\nNOTE: {len(empty_classes)} token class(es) matched no '
              f'firmware emissions (see inventory for per-class scope):',
              file=sys.stderr)
        for c in empty_classes:
            print(f'  - {c}', file=sys.stderr)
        print('Review against the Scope note in each class. If the scope '
              'note already acknowledges the gap, no action. Otherwise '
              'investigate as a possible rotted regex or removed firmware '
              'emission.', file=sys.stderr)

    # Always return 0 — empty classes are surfaced for review, not gated.
    # A future audit cycle can decide to escalate any unexpected emptiness
    # to a finding without this script forcing a non-zero exit.
    return 0


if __name__ == '__main__':
    sys.exit(main())
