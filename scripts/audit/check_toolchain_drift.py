#!/usr/bin/env python3
"""Toolchain drift check (F-2026-05-13-003).

Mechanically pulls upstream-latest versions for the toolchain components
the project pins (Pico SDK, picotool, GCC ARM, OpenOCD Pi fork, Pico
Probe firmware, CMake), reads the project-pinned versions from
CMakeLists.txt, reads the locally installed versions from
~/.pico-sdk/<component>/, and reports drift.

Replaces the manual walk that `docs/audits/TOOLCHAIN_VERSION_AUDIT_2026-04-27.md`
performed by hand. Audit-time confidence improvement only — production
firmware behavior is unaffected by drift in this script's output (the
pinned versions in CMakeLists.txt are what get built).

Source: AUDIT_GUIDANCE.md Tier 2.3. Closes F-2026-05-13-003.

Output: stdout table + logs/toolchain_drift.txt.

Network: uses urllib + GitHub Releases API (no token needed for low-rate
read). Falls back to NOTE when network is unavailable; this script is
designed to not block the audit if the network is flaky.

Exit codes:
    0 = drift check completed (possibly with NOTE-level findings)
    1 = catastrophic failure (CMakeLists.txt not readable etc.)
"""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
import urllib.error
import urllib.request
from typing import Optional


def _repo_root() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.dirname(os.path.dirname(here))


REPO = _repo_root()
OUTPUT_LOG = os.path.join(REPO, 'logs', 'toolchain_drift.txt')

# Local SDK install root — same path the Pico SDK installer uses on Windows
# and Linux.
PICO_SDK_HOME = os.path.expanduser('~/.pico-sdk')

# Network timeout (seconds) for upstream-latest pulls. Keep short so a
# flaky network doesn't slow the audit; failures degrade gracefully to
# NOTE-level findings.
HTTP_TIMEOUT_S = 10


# =============================================================================
# CMakeLists.txt pin parser
# =============================================================================

def _parse_cmake_pins(path: str) -> dict[str, str]:
    """Return {pin_name: value} for set(<pin> <value>) lines we care about."""
    pat = re.compile(
        r'^\s*set\s*\(\s*(sdkVersion|picotoolVersion|toolchainVersion)\s+'
        r'([\w.\-+]+)\s*\)\s*$', re.MULTILINE)
    cmake_min = re.compile(
        r'^\s*cmake_minimum_required\s*\(\s*VERSION\s+([\w.]+)\s*\)',
        re.MULTILINE)
    pins: dict[str, str] = {}
    try:
        with open(path, 'r', encoding='utf-8') as f:
            text = f.read()
    except OSError as e:
        print(f'ERROR: cannot read {path}: {e}', file=sys.stderr)
        sys.exit(1)
    for m in pat.finditer(text):
        pins[m.group(1)] = m.group(2)
    cm = cmake_min.search(text)
    if cm:
        pins['cmake_minimum_required'] = cm.group(1)
    return pins


# =============================================================================
# Local install discovery
# =============================================================================

def _local_pico_sdk_versions() -> dict[str, list[str]]:
    """Walk ~/.pico-sdk/<component>/ to find installed version directories."""
    out: dict[str, list[str]] = {}
    if not os.path.isdir(PICO_SDK_HOME):
        return out
    for comp in ('sdk', 'picotool', 'toolchain', 'openocd'):
        d = os.path.join(PICO_SDK_HOME, comp)
        if os.path.isdir(d):
            try:
                out[comp] = sorted(
                    e for e in os.listdir(d)
                    if os.path.isdir(os.path.join(d, e)))
            except OSError:
                out[comp] = []
        else:
            out[comp] = []
    return out


def _local_cmake_version() -> Optional[str]:
    """Run `cmake --version`, parse first line."""
    try:
        r = subprocess.run(
            ['cmake', '--version'], capture_output=True, text=True,
            timeout=5, check=False)
        if r.returncode != 0:
            return None
        first = r.stdout.splitlines()[0] if r.stdout else ''
        m = re.search(r'(\d+\.\d+\.\d+)', first)
        return m.group(1) if m else None
    except (OSError, subprocess.TimeoutExpired):
        return None


# =============================================================================
# Upstream-latest pull
# =============================================================================

class NetworkError(Exception):
    pass


def _http_get_json(url: str) -> dict:
    req = urllib.request.Request(
        url,
        headers={
            'Accept': 'application/vnd.github+json',
            'User-Agent': 'rocketchip-audit-toolchain-drift',
        },
    )
    try:
        with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT_S) as resp:
            return json.loads(resp.read().decode('utf-8'))
    except (urllib.error.URLError, urllib.error.HTTPError,
            json.JSONDecodeError, TimeoutError, OSError) as e:
        raise NetworkError(f'fetch failed: {e}') from e


def _gh_latest_release_tag(owner: str, repo: str) -> str:
    data = _http_get_json(
        f'https://api.github.com/repos/{owner}/{repo}/releases/latest')
    tag = data.get('tag_name') or data.get('name') or ''
    if not tag:
        raise NetworkError('no tag_name in response')
    return tag


def _gh_branch_head_sha(owner: str, repo: str, branch: str) -> tuple[str, str]:
    """Return (sha, commit_date) for a branch HEAD."""
    data = _http_get_json(
        f'https://api.github.com/repos/{owner}/{repo}/commits/{branch}')
    sha = data.get('sha', '')[:7]
    date = data.get('commit', {}).get('committer', {}).get('date', '')
    return sha, date


def _cmake_latest_version() -> str:
    data = _http_get_json(
        'https://api.github.com/repos/Kitware/CMake/releases/latest')
    tag = data.get('tag_name', '')
    # tag_name is e.g. "v4.3.2"
    return tag.lstrip('v')


# =============================================================================
# Drift assessment
# =============================================================================

def _norm(s: str) -> str:
    """Normalize version strings for comparison."""
    return s.replace('v', '').strip()


def _drift(local: Optional[str], upstream: Optional[str]) -> str:
    if upstream is None:
        return 'unknown'
    if local is None:
        return 'unknown'
    if _norm(local) == _norm(upstream):
        return 'clean'
    return 'drift'


# =============================================================================
# Report
# =============================================================================

def main() -> int:
    pins = _parse_cmake_pins(os.path.join(REPO, 'CMakeLists.txt'))
    local = _local_pico_sdk_versions()
    local_cmake = _local_cmake_version()

    network_failures: list[tuple[str, str]] = []

    def safe_call(label: str, fn):
        try:
            return fn()
        except NetworkError as e:
            network_failures.append((label, str(e)))
            return None

    upstream_sdk = safe_call(
        'Pico SDK', lambda: _gh_latest_release_tag('raspberrypi', 'pico-sdk'))
    upstream_picotool = safe_call(
        'picotool', lambda: _gh_latest_release_tag('raspberrypi', 'picotool'))
    upstream_debugprobe = safe_call(
        'debugprobe',
        lambda: _gh_latest_release_tag('raspberrypi', 'debugprobe'))
    upstream_openocd_head = safe_call(
        'OpenOCD (rpi-common HEAD)',
        lambda: _gh_branch_head_sha(
            'raspberrypi', 'openocd', 'rpi-common'))
    upstream_cmake = safe_call('CMake', _cmake_latest_version)

    lines: list[str] = []
    add = lines.append
    add('# Toolchain Drift Check')
    add('')
    add('Source: scripts/audit/check_toolchain_drift.py')
    add('Closes F-2026-05-13-003 (AUDIT_GUIDANCE.md Tier 2.3).')
    add('')
    add('Mechanical replacement for the manual walk in')
    add('docs/audits/TOOLCHAIN_VERSION_AUDIT_2026-04-27.md. Re-run at every')
    add('milestone audit; compare with prior run to detect drift.')
    add('')
    add('---')
    add('')
    add('## Components')
    add('')
    add('| Component | Project pin | Local install | Upstream latest | Status |')
    add('|---|---|---|---|---|')

    def row(name: str, pin: Optional[str], local_v: Optional[str],
            upstream: Optional[str]):
        status = _drift(local_v, upstream)
        pin_s = pin or '—'
        local_s = local_v or '—'
        up_s = upstream or '*(no network)*'
        add(f'| {name} | {pin_s} | {local_s} | {up_s} | {status} |')

    # Pico SDK
    local_sdks = local.get('sdk', [])
    row('Pico SDK',
        pins.get('sdkVersion'),
        local_sdks[-1] if local_sdks else None,
        upstream_sdk)

    # picotool
    local_picotools = local.get('picotool', [])
    row('picotool',
        pins.get('picotoolVersion'),
        local_picotools[-1] if local_picotools else None,
        upstream_picotool)

    # GCC ARM (toolchain) — upstream version flow is via ARM-software downloads,
    # not GitHub Releases. Skipped from automated pull; manual annotation only.
    local_toolchains = local.get('toolchain', [])
    row('GCC ARM (toolchain)',
        pins.get('toolchainVersion'),
        local_toolchains[-1] if local_toolchains else None,
        None)
    add('| | | | *(manual: see arm-developer.com toolchain releases)* | |')

    # OpenOCD (Pi fork) — no releases, track rpi-common branch HEAD
    local_openocds = local.get('openocd', [])
    if upstream_openocd_head:
        upstream_openocd_str = (
            f'{upstream_openocd_head[0]} ({upstream_openocd_head[1][:10]})')
    else:
        upstream_openocd_str = None
    row('OpenOCD (Pi fork)',
        '— (no pin; branch-tracked)',
        local_openocds[-1] if local_openocds else None,
        upstream_openocd_str)

    # debugprobe firmware
    row('Pico Probe firmware',
        '— (probe USB descriptor)',
        '*(read from probe — not in ~/.pico-sdk/)*',
        upstream_debugprobe)

    # CMake
    row('CMake (local)',
        pins.get('cmake_minimum_required', '—') + ' (minimum)',
        local_cmake,
        upstream_cmake)

    add('')
    add('---')
    add('')
    add('## Notes')
    add('')
    if network_failures:
        add(f'**Network-fallback components ({len(network_failures)}):**')
        add('')
        for label, err in network_failures:
            add(f'  - {label}: {err}')
        add('')
        add('Re-run when network is available. Network failure does not')
        add('block the audit — the manual TOOLCHAIN_VERSION_AUDIT row walk')
        add('remains available as a fallback (see')
        add('`docs/audits/TOOLCHAIN_VERSION_AUDIT_2026-04-27.md`).')
        add('')
    else:
        add('All upstream-latest versions pulled successfully.')
        add('')
    add('**Drift definition:** `clean` if local installed version matches')
    add('upstream-latest exactly. `drift` otherwise. `unknown` if either')
    add('side could not be determined (no local install discovered, or')
    add('network failed).')
    add('')
    add('**Pinned is not drift:** the project intentionally pins to specific')
    add('GCC ARM and OpenOCD versions (see TOOLCHAIN_VERSION_AUDIT for')
    add('rationale). A `drift` status on those is informational, not an')
    add('audit finding by itself — the next-action is to evaluate whether')
    add('the pin should move, per the criteria in that doc.')
    add('')
    add('**Pico Probe firmware** is the one component this script cannot')
    add('determine the local version for — it lives on the probe hardware,')
    add('read via USB descriptor. Report fills in the upstream column for')
    add('reference; auditor compares against the probe\'s string.')

    out_text = '\n'.join(lines) + '\n'

    # Force stdout writes to tolerate non-UTF-8 codepage (Windows cp1252).
    # The file write below is always UTF-8 — that's the canonical artifact.
    try:
        sys.stdout.write(out_text)
    except UnicodeEncodeError:
        sys.stdout.buffer.write(out_text.encode('utf-8', errors='replace'))

    os.makedirs(os.path.dirname(OUTPUT_LOG), exist_ok=True)
    with open(OUTPUT_LOG, 'w', encoding='utf-8') as f:
        f.write(out_text)
    print(f'\n[drift report written to {os.path.relpath(OUTPUT_LOG, REPO)}]',
          file=sys.stderr)

    return 0


if __name__ == '__main__':
    sys.exit(main())
