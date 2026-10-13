#!/usr/bin/env python3
"""Host test: pre-push gate stdin cases (githooks pre-push). No hardware.

Builds a throwaway git repo with scripts/ci/firmware_paths.txt, fakes the
remote state with refs/remotes/origin/*, and feeds git-style stdin lines
("<local-ref> <local-sha> <remote-ref> <remote-sha>") to
scripts/ci/pre_push_gate.py. Nothing is pushed anywhere.

Cases: new branch (remote sha zeroes), delete (local sha zeroes),
annotated tag (tag object peeled to its commit), lightweight tag,
non-release tag, tag on a tree, and a mixed push where one bad line
aborts the whole push.

Bench key v2 and note reuse (in-process, fake tools and fake USB serials):
the key is deterministic; it changes with the compiler, picotool and
preset; a missing tool, a failed run or an empty version line fails closed;
a note is reused only with PASS + roles + the same board serials attached;
fresh mode (--fresh or RC_BENCH_FRESH=1) ignores notes; the bench output's
board_usb_serial line goes into the note; the build's compiler must match
the key.
"""
from __future__ import annotations

import contextlib
import io
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
GATE = HERE / 'ci' / 'pre_push_gate.py'
GATE_NO_OPENOCD = (
    'import os, sys; sys.path.insert(0, os.path.dirname(sys.argv[1])); '
    'import pre_push_gate as pg; pg.openocd_listening = lambda: False; '
    'sys.exit(pg.main(sys.argv[1:]))')
LIST = HERE / 'ci' / 'firmware_paths.txt'
ZERO = '0' * 40
FAILS = 0


def check(name: str, ok: bool, detail: str = '') -> None:
    global FAILS
    print(f'  [{"PASS" if ok else "FAIL"}] {name}')
    if not ok:
        FAILS += 1
        if detail:
            print('    ' + detail.replace('\n', '\n    '))


def git(repo: Path, *args: str) -> str:
    return subprocess.run(['git', *args], cwd=str(repo), check=True,
                          capture_output=True, text=True).stdout.strip()


def commit(repo: Path, path: str, text: str, msg: str) -> str:
    p = repo / path
    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open('a', encoding='utf-8') as f:
        f.write(text + '\n')
    git(repo, 'add', path)
    git(repo, 'commit', '-q', '-m', msg)
    return git(repo, 'rev-parse', 'HEAD')


def gate(repo: Path, lines: list, **env_extra: str) -> tuple:
    env = dict(os.environ, PATH=os.environ.get('PATH', ''), **env_extra)
    # Run the real gate with OpenOCD reported as down, so a bench PC with
    # OpenOCD running gives the same result as a PC without it: the gate
    # blocks before the bench worktree, the build and the flash step.
    r = subprocess.run([sys.executable, '-c', GATE_NO_OPENOCD, str(GATE), 'origin', 'file:///nowhere'],
                       cwd=str(repo), input='\n'.join(lines) + '\n',
                       capture_output=True, text=True, env=env)
    return r.returncode, r.stdout + r.stderr


def fake_tool(d: Path, name: str, line: str, rc: int = 0) -> Path:
    """A program that prints ``line`` (nothing if empty) and exits ``rc``."""
    d.mkdir(parents=True, exist_ok=True)
    if os.name == 'nt':
        p = d / f'{name}.cmd'
        p.write_text('@echo off\n' + (f'echo {line}\n' if line else '')
                     + f'exit /b {rc}\n', encoding='utf-8')
    else:
        p = d / name
        p.write_text('#!/bin/sh\n' + (f"echo '{line}'\n" if line else '')
                     + f'exit {rc}\n', encoding='utf-8')
        p.chmod(0o755)
    return p


@contextlib.contextmanager
def env_set(**kv: str):
    old = {k: os.environ.get(k) for k in kv}
    os.environ.update(kv)
    try:
        yield
    finally:
        for k, v in old.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v


def captured(fn, *a, **kw) -> str:
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        fn(*a, **kw)
    return buf.getvalue()


GCC_A = 'arm-none-eabi-gcc (Arm GNU Toolchain 14.2.Rel1 (Build arm-14.52)) 14.2.1 20241119'
GCC_B = 'arm-none-eabi-gcc (Arm GNU Toolchain 15.1.Rel1 (Build arm-15.7)) 15.1.0 20250417'
PT_A = 'picotool v2.2.0-a4 (Linux, GNU-14.2.0, Release)'
PT_B = 'picotool v2.3.0 (Linux, GNU-14.2.0, Release)'
PA_A = 'pioasm version: 2.2.0'
PA_B = 'pioasm version: 2.3.0'


def fake_sdk(td: Path) -> Path:
    """A git checkout with tag 2.2.0 and one submodule (like the Pico SDK)."""
    sub = td / 'subsrc'
    sub.mkdir()
    git(sub, 'init', '-q', '-b', 'main')
    git(sub, 'config', 'user.name', 'test')
    git(sub, 'config', 'user.email', 'test@example.invalid')
    commit(sub, 'tusb.h', '// tusb', 'tusb')
    sdk = td / 'sdk'
    sdk.mkdir()
    git(sdk, 'init', '-q', '-b', 'master')
    git(sdk, 'config', 'user.name', 'test')
    git(sdk, 'config', 'user.email', 'test@example.invalid')
    commit(sdk, 'pico_sdk_version.cmake', 'set(PICO_SDK_VERSION 2.2.0)', 'sdk')
    git(sdk, '-c', 'protocol.file.allow=always', 'submodule', '-q', 'add',
        sub.as_uri(), 'lib/tinyusb')
    git(sdk, 'commit', '-q', '-m', 'add tinyusb')
    git(sdk / 'lib' / 'tinyusb', 'config', 'user.name', 'test')
    git(sdk / 'lib' / 'tinyusb', 'config', 'user.email', 'test@example.invalid')
    git(sdk, 'tag', '2.2.0')
    return sdk


def main() -> int:
    td = Path(tempfile.mkdtemp(prefix='rc_prepush_'))
    try:
        tools = td / 'tools'
        gcc_a = fake_tool(tools / 'a', 'arm-none-eabi-gcc', GCC_A)
        gcc_b = fake_tool(tools / 'b', 'arm-none-eabi-gcc', GCC_B)
        pt_a = fake_tool(tools / 'a', 'picotool', PT_A)
        pt_b = fake_tool(tools / 'b', 'picotool', PT_B)
        pa_a = fake_tool(tools / 'a', 'pioasm', PA_A)
        sdk = fake_sdk(td)
        # Every gate run below (subprocess and in-process) sees these tools.
        os.environ['RC_ARM_GCC'] = str(gcc_a)
        os.environ['RC_PICOTOOL'] = str(pt_a)
        os.environ['RC_PIOASM'] = str(pa_a)
        os.environ['RC_PICO_SDK'] = str(sdk)
        os.environ.pop('RC_BENCH_FRESH', None)
        repo = td / 'repo'
        repo.mkdir()
        git(repo, 'init', '-q', '-b', 'main')
        git(repo, 'config', 'user.name', 'test')
        git(repo, 'config', 'user.email', 'test@example.invalid')
        (repo / 'scripts' / 'ci').mkdir(parents=True)
        shutil.copy(LIST, repo / 'scripts' / 'ci' / 'firmware_paths.txt')
        commit(repo, 'scripts/ci/firmware_paths.txt', '', 'list file')
        base = commit(repo, 'src/main.cpp', '// base', 'base')
        git(repo, 'remote', 'add', 'origin', 'file:///nowhere')
        git(repo, 'update-ref', 'refs/remotes/origin/main', base)  # fake remote state

        print('test_new_branch_docs_only (remote sha zeroes)')
        git(repo, 'checkout', '-q', '-b', 'feat')
        d = commit(repo, 'docs/x.md', 'x', 'docs only')
        rc, out = gate(repo, [f'refs/heads/feat {d} refs/heads/feat {ZERO}'])
        check('rc 0', rc == 0, out)
        check('only the new commit counted (rev-list --not --remotes)',
              '1 commit(s), 0 firmware' in out, out)

        print('test_new_branch_firmware (remote sha zeroes)')
        f = commit(repo, 'src/main.cpp', '// change', 'fw change')
        rc, out = gate(repo, [f'refs/heads/feat {f} refs/heads/feat {ZERO}'])
        check('2 new commits, 1 firmware, not the whole history',
              '2 commit(s), 1 firmware' in out, out)
        check('bench needed -> blocks without OpenOCD (rc 1)',
              rc == 1 and 'OpenOCD' in out, out)

        print('test_shape_warns_but_does_not_block')
        # six firmware commits, one of them an isolate class: warning only.
        tip = f
        for n in range(5):
            tip = commit(repo, 'src/main.cpp', f'// {n}', f'fw {n}')
        tip = commit(repo, 'src/flight_director/guard_evaluator.cpp', '// g', 'guard change')
        rc, out = gate(repo, [f'refs/heads/feat {tip} refs/heads/feat {ZERO}'])
        check('shape warning printed', 'WARNING' in out, out)
        check('shape does not block (still the OpenOCD block)',
              rc == 1 and 'OpenOCD' in out and 'firmware commits in one push' in out, out)

        print('test_delete (local sha zeroes)')
        rc, out = gate(repo, [f'(delete) {ZERO} refs/heads/old {base}'])
        check('rc 0, no bench', rc == 0 and 'bench' not in out.lower(), out)

        print('test_annotated_release_tag (tag object peeled)')
        git(repo, 'tag', '-a', 'v1.0.0', '-m', 'rel', f)
        tag_obj = git(repo, 'rev-parse', 'v1.0.0')
        check('tag sha is the tag object, not the commit', tag_obj != f)
        rc, out = gate(repo, [f'refs/tags/v1.0.0 {tag_obj} refs/tags/v1.0.0 {ZERO}'])
        check('no PASS note -> rc 1', rc == 1 and 'no bench PASS note' in out, out)
        check('message names the peeled commit', f[:12] in out, out)
        sys.path.insert(0, str(HERE))
        sys.path.insert(0, str(HERE / 'ci'))
        import firmware_tree as ft
        import pre_push_gate as pg
        key = pg.bench_key(repo, f, write=True)[0]
        git(repo, 'notes', '--ref', 'refs/notes/rc-bench', 'add', '-f', '-m',
            'rc-bench v1\nresult: PASS\nroles: vehicle', key)
        rc, out = gate(repo, [f'refs/tags/v1.0.0 {tag_obj} refs/tags/v1.0.0 {ZERO}'])
        check('PASS note on peeled commit -> rc 0',
              rc == 0 and 'bench PASS note found' in out, out)

        print('test_lightweight_release_tag')
        git(repo, 'tag', 'v1.1.0', d)
        rc, out = gate(repo, [f'refs/tags/v1.1.0 {d} refs/tags/v1.1.0 {ZERO}'])
        check('lightweight tag on unbenched commit -> rc 1', rc == 1, out)

        print('test_non_release_tag')
        git(repo, 'tag', 'scratch', d)
        rc, out = gate(repo, [f'refs/tags/scratch {d} refs/tags/scratch {ZERO}'])
        check('non-release tag -> rc 0', rc == 0, out)

        print('test_tag_on_tree')
        tree = git(repo, 'rev-parse', f'{d}^{{tree}}')
        git(repo, 'tag', 'v9.0.0', tree)
        rc, out = gate(repo, [f'refs/tags/v9.0.0 {tree} refs/tags/v9.0.0 {ZERO}'])
        check('release-pattern tag on a tree -> rc 0 (nothing to gate)', rc == 0, out)

        print('test_starcom_tag_is_not_a_release_tag')
        git(repo, 'tag', '-a', 'starcom-v0.2.25', '-m', 'sc', d)
        sc = git(repo, 'rev-parse', 'starcom-v0.2.25')
        rc, out = gate(repo, [f'refs/tags/starcom-v0.2.25 {sc} refs/tags/starcom-v0.2.25 {ZERO}'])
        check('starcom-v* -> rc 0 (not a flight release)', rc == 0, out)

        print('test_mixed_push_one_bad_line_aborts_all')
        rc, out = gate(repo, [
            f'(delete) {ZERO} refs/heads/old {base}',
            f'refs/heads/feat {d} refs/heads/feat {ZERO}',
            f'refs/tags/v1.1.0 {d} refs/tags/v1.1.0 {ZERO}',
        ])
        check('rc 1 (Git aborts the whole push)', rc == 1, out)

        print('test_radio_push_warns_until_the_bench_script_exists')
        git(repo, 'checkout', '-q', '-b', 'radio', base)
        rsha = commit(repo, 'src/drivers/rfm95w.cpp', '// radio', 'radio change')
        rc, out = gate(repo, [f'refs/heads/radio {rsha} refs/heads/radio {ZERO}'])
        check('warns that the radio rule is not active',
              'NOT radio-benched' in out, out)
        check('does not block on the missing script; fails closed at OpenOCD',
              rc == 1 and 'OpenOCD' in out, out)

        print('test_existing_branch_update (remote..local)')
        git(repo, 'update-ref', 'refs/remotes/origin/feat', d)
        rc, out = gate(repo, [f'refs/heads/feat {f} refs/heads/feat {d}'])
        check('1 commit, 1 firmware', '1 commit(s), 1 firmware' in out, out)

        bench_key_v2_tests(repo, td, ft, pg, f, d, base,
                           gcc_a, gcc_b, pt_a, pt_b, tools, sdk)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    print('ALL CHECKS PASS' if FAILS == 0 else f'{FAILS} CHECK(S) FAILED')
    return 0 if FAILS == 0 else 1


def bench_key_v2_tests(repo, td, ft, pg, f, d, base,
                       gcc_a, gcc_b, pt_a, pt_b, tools, sdk) -> None:
    presets = dict(pg.PRESETS)

    print('test_bench_key_v2_deterministic_and_tool_inputs')
    k1 = ft.bench_key(repo, f, presets)
    check('same inputs -> same key', k1 == ft.bench_key(repo, f, presets))
    check('gate key == firmware_tree key', k1 == pg.bench_key(repo, f, False)[0])
    with env_set(RC_ARM_GCC=str(gcc_b)):
        check('new compiler version -> new key', ft.bench_key(repo, f, presets) != k1)
    with env_set(RC_PICOTOOL=str(pt_b)):
        check('new picotool version -> new key', ft.bench_key(repo, f, presets) != k1)
    check('other preset name -> new key',
          ft.bench_key(repo, f, dict(presets, vehicle='vehicle-x')) != k1)
    pa_b = fake_tool(tools / 'b', 'pioasm', PA_B)
    with env_set(RC_PIOASM=str(pa_b)):
        check('new pioasm version -> new key', ft.bench_key(repo, f, presets) != k1)
    sdk_head = git(sdk, 'rev-parse', 'HEAD')
    sub_head = git(sdk / 'lib' / 'tinyusb', 'rev-parse', 'HEAD')
    hdr = ft.key_input_header(ft.tool_versions(repo, f), presets).decode()
    sdk_shown = os.path.normpath(str(sdk)).replace('\\', '/')
    sdk_shown = sdk_shown.lower() if os.name == 'nt' else sdk_shown
    want_hdr = ('rc-bench-key v2\n[tool]\n'
                f'arm-none-eabi-gcc: {GCC_A}\npicotool: {PT_A}\npioasm: {PA_A}\n'
                f'pico-sdk: {sdk_shown}\npico-sdk-describe: 2.2.0\n'
                f'pico-sdk-head: {sdk_head}\n'
                f'pico-sdk-submodules: lib/tinyusb=ok:{sub_head}\n'
                'preset: station=station-flight vehicle=vehicle-flight\n')
    check('key text: rc-bench-key v2 and every [tool] line', hdr == want_hdr,
          hdr + '\n--- want ---\n' + want_hdr)

    print('test_bench_key_pico_sdk')
    (sdk / 'local.c').write_text('// local edit\n')
    git(sdk, 'add', 'local.c')
    v = ft.tool_versions(repo, f)
    check('dirty SDK -> describe 2.2.0-dirty', v['pico-sdk-describe'] == '2.2.0-dirty', v)
    check('dirty SDK -> new key', ft.bench_key(repo, f, presets) != k1)
    git(sdk, 'commit', '-q', '-m', 'local')
    check('new SDK commit -> new key', ft.bench_key(repo, f, presets) != k1)
    git(sdk, 'reset', '-q', '--hard', 'HEAD~1')
    check('SDK back at 2.2.0 -> same key again', ft.bench_key(repo, f, presets) == k1)
    gdir = git(repo, 'rev-parse', '--absolute-git-dir')
    with env_set(GIT_DIR=gdir, GIT_INDEX_FILE=str(Path(gdir) / 'index')):
        # A hook exports GIT_DIR for this repo. The SDK read must ignore it.
        check('hook GIT_DIR set -> same key (SDK read uses its own repo)',
              ft.bench_key(repo, f, presets) == k1)
        check('clean_git_env drops GIT_DIR and GIT_INDEX_FILE',
              not {'GIT_DIR', 'GIT_INDEX_FILE'} & set(ft.clean_git_env()))
    commit(sdk / 'lib' / 'tinyusb', 'tusb.h', '// moved', 'move sub')
    v = ft.tool_versions(repo, f)
    check('submodule moved -> state "moved" and new key',
          'lib/tinyusb=moved:' in v['pico-sdk-submodules']
          and ft.bench_key(repo, f, presets) != k1, v['pico-sdk-submodules'])
    git(sdk / 'lib' / 'tinyusb', 'reset', '-q', '--hard', 'HEAD~1')
    check('normalize_submodules: states, sorted, describe dropped',
          ft.normalize_submodules(' aaa lib/z (v1)\n-bbb lib/a\n+ccc lib/m (x)\n')
          == 'lib/a=uninit:bbb; lib/m=moved:ccc; lib/z=ok:aaa')
    check('docs-only commit keeps the key (same tools)',
          ft.bench_key(repo, d, presets) == ft.bench_key(repo, base, presets))

    print('test_bench_key_fails_closed')

    def raises(fn, text: str) -> bool:
        try:
            fn()
        except ft.FirmwareTreeError as exc:
            return text in str(exc)
        return False
    empty = fake_tool(tools / 'e', 'arm-none-eabi-gcc', '')
    bad = fake_tool(tools / 'x', 'arm-none-eabi-gcc', 'oops', rc=3)
    with env_set(RC_ARM_GCC=str(empty)):
        check('empty version output -> error',
              raises(lambda: ft.bench_key(repo, f, presets), 'never hashes an empty'))
    with env_set(RC_ARM_GCC=str(bad)):
        check('tool exits non-zero -> error',
              raises(lambda: ft.bench_key(repo, f, presets), 'exit 3'))
    with env_set(RC_PICOTOOL=str(td / 'no-such-picotool')):
        check('override path missing -> error',
              raises(lambda: ft.bench_key(repo, f, presets), 'is not a file'))
    check('no preset -> error', raises(lambda: ft.bench_key(repo, f, {}), 'preset'))
    plain = td / 'plain-sdk'
    plain.mkdir()
    with env_set(RC_PICO_SDK=str(plain)):
        check('SDK that is not a git checkout -> error, no version-name fallback',
              raises(lambda: ft.bench_key(repo, f, presets), 'git'))
    with env_set(RC_PICO_SDK=str(td / 'no-such-sdk')):
        check('missing SDK dir -> error',
              raises(lambda: ft.bench_key(repo, f, presets), 'not a directory'))
    with env_set(RC_PICO_SDK=str(sdk / 'lib')):
        check('SDK subdir (not the checkout top) -> error',
              raises(lambda: ft.bench_key(repo, f, presets), 'not the top'))

    print('test_sdk_path_lookup')
    old_sdk = os.environ.pop('RC_PICO_SDK')
    old_env_sdk = os.environ.pop('PICO_SDK_PATH', None)
    try:
        with env_set(HOME=str(td / 'h'), USERPROFILE=str(td / 'h')):
            check('no cache, no pico-vscode -> in-repo submodule',
                  ft.sdk_path(repo, f) == repo / 'pico-sdk')
            bf = repo / 'build_flight'
            bf.mkdir()
            (bf / 'CMakeCache.txt').write_text(f'PICO_SDK_PATH:PATH={sdk.as_posix()}\n')
            check('flight build CMakeCache.txt PICO_SDK_PATH wins',
                  ft.sdk_path(repo, f) == Path(sdk.as_posix()))
            bs = repo / 'build_station_flight'
            bs.mkdir()
            (bs / 'CMakeCache.txt').write_text('PICO_SDK_PATH:PATH=/elsewhere/sdk\n')
            check('two flight builds on different SDKs -> error',
                  raises(lambda: ft.sdk_path(repo, f), 'different Pico SDKs'))
            shutil.rmtree(bf)
            shutil.rmtree(bs)
            (repo / 'CMakeLists.txt').write_text('set(sdkVersion 2.2.0)\n')
            git(repo, 'add', 'CMakeLists.txt')
            git(repo, 'commit', '-q', '-m', 'pin sdk')
            pin_c = git(repo, 'rev-parse', 'HEAD')
            write_vs = td / 'h' / '.pico-sdk' / 'cmake' / 'pico-vscode.cmake'
            write_vs.parent.mkdir(parents=True)
            write_vs.write_text('# vscode\n')
            check('pico-vscode.cmake present -> ~/.pico-sdk/sdk/<sdkVersion>',
                  ft.sdk_path(repo, pin_c) == td / 'h' / '.pico-sdk' / 'sdk' / '2.2.0',
                  str(ft.sdk_path(repo, pin_c)))
            git(repo, 'reset', '-q', '--hard', 'HEAD~1')
    finally:
        os.environ['RC_PICO_SDK'] = old_sdk
        if old_env_sdk is not None:
            os.environ['PICO_SDK_PATH'] = old_env_sdk
    old_which, old_env = ft.shutil.which, os.environ.pop('RC_ARM_GCC')
    try:
        ft.shutil.which = lambda _n: None
        with env_set(HOME=str(td), USERPROFILE=str(td)):
            check('tool not found anywhere -> error naming RC_ARM_GCC',
                  raises(lambda: ft.resolve_tool('arm-none-eabi-gcc', repo, f),
                         'set RC_ARM_GCC='))
            (repo / 'CMakeLists.txt').write_text('set(toolchainVersion 9_9_Rel9)\n')
            git(repo, 'add', 'CMakeLists.txt')
            git(repo, 'commit', '-q', '-m', 'pins')
            pin_commit = git(repo, 'rev-parse', 'HEAD')
            pinned = (td / '.pico-sdk' / 'toolchain' / '9_9_Rel9' / 'bin')
            pinned.mkdir(parents=True)
            # Lookup only (not run): the name the resolver expects on this OS.
            (pinned / ('arm-none-eabi-gcc.exe' if os.name == 'nt'
                       else 'arm-none-eabi-gcc')).write_text('')
            got = ft.resolve_tool('arm-none-eabi-gcc', repo, pin_commit)
            check('pinned Pico VS Code toolchain found from CMakeLists.txt',
                  got.parent == pinned and got.stem == 'arm-none-eabi-gcc', str(got))
            git(repo, 'reset', '-q', '--hard', 'HEAD~1')
    finally:
        ft.shutil.which = old_which
        os.environ['RC_ARM_GCC'] = old_env
    rc, out = gate(repo, [f'refs/heads/feat {f} refs/heads/feat {d}'],
                   RC_ARM_GCC=str(empty))
    check('gate blocks with the reason when the key cannot be made',
          rc == 1 and 'PRE-PUSH BLOCKED' in out and 'empty version' in out, out)

    print('test_note_reuse_needs_same_boards')
    key = pg.bench_key(repo, f, write=True)[0]

    def note(*extra: str) -> None:
        git(repo, 'notes', '--ref', pg.NOTES_REF, 'add', '-f', '-m',
            '\n'.join(['rc-bench v2', 'result: PASS', 'roles: vehicle station',
                       *extra]), key)

    def plan(attached, fresh=False) -> str:
        old = pg.attached_board_serials
        pg.attached_board_serials = lambda: attached
        try:
            return captured(pg.handle_branch, repo, 'origin', 'refs/heads/feat',
                            f, d, True, fresh)
        finally:
            pg.attached_board_serials = old
    # A firmware change benches BOTH roles (2026-10-08).
    out = plan({'SER1', 'SER2'})
    check('firmware change -> roles vehicle and station',
          "roles=['vehicle', 'station']" in out, out)
    note('boards: vehicle=SER1 station=SER2')
    out = plan({'SER1', 'SER2', 'SER9'})
    check('same boards attached -> skip', 'skip flash and bench' in out, out)
    out = plan({'SER1', 'SER3'})
    check('other station board attached -> bench, names the serial',
          'would bench' in out and 'SER2' in out and 'not attached' in out, out)
    out = plan(set())
    check('no board attached -> bench', 'would bench' in out, out)
    out = plan(None)
    check('serials unreadable -> bench', 'would bench' in out and 'pyserial' in out, out)
    out = plan({'SER1', 'SER2'}, fresh=True)
    check('fresh mode -> bench even with a matching note',
          'would bench' in out and 'fresh mode' in out, out)
    note()
    out = plan({'SER1', 'SER2'})
    check('note without boards: line -> bench', 'no USB serial' in out, out)
    note('boards: vehicle=unknown station=SER2')
    out = plan({'unknown', 'SER2'})
    check('boards: vehicle=unknown is never reused', 'would bench' in out, out)
    git(repo, 'notes', '--ref', pg.NOTES_REF, 'add', '-f', '-m',
        'rc-bench v2\nresult: PASS\nroles: vehicle\nboards: vehicle=SER1', key)
    out = plan({'SER1', 'SER2'})
    check('vehicle-only note does not cover a firmware push (station needed)',
          'would bench' in out and 'station' in out, out)
    note('boards: vehicle=SER1 station=SER2')
    with env_set(RC_ARM_GCC=str(gcc_b)):
        out = plan({'SER1', 'SER2'})
    check('compiler update -> new key, old note not found',
          'would bench' in out and 'no note for key' in out, out)
    ok, why = pg.note_covers('result: PASS\nroles: vehicle station radio-link\n'
                             'boards: vehicle=SER1', ['vehicle'], True, {'SER1'})
    check('radio push needs both board serials', not ok and 'station' in why, why)
    ok, _ = pg.note_covers('result: PASS\nroles: vehicle station radio-link\n'
                           'boards: vehicle=SER1 station=SER2', ['vehicle'], True,
                           {'SER1', 'SER2'})
    check('radio push with both boards attached -> reuse', ok)

    print('test_attached_serials_skip_the_debug_probe')
    try:
        import serial.tools.list_ports  # noqa: F401
        from types import SimpleNamespace
        from unittest.mock import patch
        ports = [SimpleNamespace(device='COM5', vid=0x2E8A, pid=0x0009,
                                 serial_number='02FBDDB8E1CA1281'),
                 SimpleNamespace(device='COM7', vid=0x2E8A, pid=0x0009,
                                 serial_number='BEC71B8EDC6AEBD1'),
                 SimpleNamespace(device='COM4', vid=0x2E8A, pid=0x000C,
                                 serial_number='E663AC91D3487137')]
        with patch('serial.tools.list_ports.comports', return_value=ports):
            got = pg.attached_board_serials()
        check('two boards, not the debug probe (PID 0x000C)',
              got == {'02FBDDB8E1CA1281', 'BEC71B8EDC6AEBD1'}, got)
    except ImportError:
        print('  [SKIP] pyserial not installed')

    print('test_fresh_mode_switches')
    with env_set(RC_BENCH_FRESH='1'):
        check('RC_BENCH_FRESH=1 -> fresh', pg.fresh_mode(['gate']))
    with env_set(RC_BENCH_FRESH='0'):
        check('RC_BENCH_FRESH=0 -> not fresh', not pg.fresh_mode(['gate']))
    check('--fresh -> fresh', pg.fresh_mode(['gate', '--plan', '--fresh']))
    check('nothing set -> not fresh', not pg.fresh_mode(['gate']))
    rc, out = gate(repo, [f'refs/heads/feat {f} refs/heads/feat {d}'],
                   RC_BENCH_FRESH='1')
    check('real push in fresh mode: says so, ignores the note, benches '
          '(fails closed at OpenOCD here)',
          rc == 1 and 'FRESH MODE' in out and 'fresh mode: notes ignored' in out
          and 'OpenOCD' in out, out)

    print('test_bench_output_board_serial_goes_to_note')
    wt = td / 'wt'
    fake = wt / 'scripts' / 'fake_bench.py'
    fake.parent.mkdir(parents=True)
    fake.write_text("print('board_usb_serial: E6614C311B4A2E2F')\n"
                    "print('RESULT: 2/2 PASS')\n")
    res, ser = pg.run_bench(wt, 'fake_bench.py', f, 'vehicle bench')
    check('result and serial parsed', (res, ser) == ('2/2 PASS', 'E6614C311B4A2E2F'),
          f'{res} {ser}')
    fake.write_text("print('RESULT: 2/2 PASS')\n")
    out = captured(lambda: print(pg.run_bench(wt, 'fake_bench.py', f, 'vehicle bench')))
    check('no serial line -> unknown + warning',
          "'unknown'" in out and 'WARNING' in out, out)

    print('test_pass_note_format (bench steps faked, no board)')
    saved = {n: getattr(pg, n) for n in ('openocd_listening', 'prepare_worktree',
                                          'build_role', 'flash_recorded', 'run_bench')}
    saved_tree = ft.elf_tree_id
    try:
        pg.openocd_listening = lambda: True
        pg.prepare_worktree = lambda *a: None
        ft.elf_tree_id = lambda *a, **k: 'feedface' * 5
        pg.build_role = lambda wt, r, want, keyed: (wt / 'rocketchip.elf', 'abc123')
        pg.flash_recorded = lambda elf: True
        pg.run_bench = lambda wt, script, sha, label: (
            '2/2 PASS', 'SER1' if label.startswith('vehicle') else 'SER2')
        with env_set(RC_BENCH_FRESH='1'):
            out = captured(pg.handle_branch, repo, 'origin', 'refs/heads/feat',
                           f, d, False, True)
    finally:
        for n, v in saved.items():
            setattr(pg, n, v)
        ft.elf_tree_id = saved_tree
    body = git(repo, 'notes', '--ref', pg.NOTES_REF, 'show', key)
    want_lines = ['rc-bench v2', 'result: PASS', f'commit: {f}',
                  'firmware_tree: ' + 'feedface' * 5, 'roles: vehicle station',
                  'boards: vehicle=SER1 station=SER2', f'arm-none-eabi-gcc: {GCC_A}',
                  f'picotool: {PT_A}', f'pioasm: {PA_A}', 'pico-sdk-describe: 2.2.0',
                  'preset: station=station-flight vehicle=vehicle-flight',
                  'fresh: yes', 'vehicle: 2/2 PASS image flight-abc123',
                  'station: 2/2 PASS image flight-abc123']
    check('note has the v2 lines', all(ln in body.splitlines() for ln in want_lines),
          body + '\n---\n' + out)
    check('citation line printed', 'Verified at push:' in out, out)
    out = plan({'SER1', 'SER2'})
    check('the new note is reused with the same boards', 'skip flash and bench' in out, out)

    print('test_role_table_drives_flash_and_openocd')
    check('every role: build dir, preset, bench script, flash path',
          all(len(v) == 4 and v[3] in (pg.FLASH_SWD, pg.FLASH_USB)
              for v in pg.ROLES.values()), str(pg.ROLES))
    for r, v in pg.ROLES.items():
        txt = pg.flash_instructions(td, td / 'wt', r, td / 'x.elf', 'PT')
        if v[3] == pg.FLASH_USB:
            check(f'{r}: USB path only (picotool load, --record-only, LED + CDC)',
                  'PT load' in txt and '--record-only' in txt
                  and 'LED + CDC' in txt and 'Path 2' not in txt, txt)
        else:
            check(f'{r}: both documented paths (picotool, flash script)',
                  'Path 1' in txt and 'PT load' in txt and 'Path 2' in txt, txt)
        check(f'{r}: instructions name the role', txt.lstrip().startswith(r), txt)
    check('station-only push needs no OpenOCD', not pg.needs_openocd(['station']))
    check('vehicle + station push needs OpenOCD', pg.needs_openocd(['vehicle', 'station']))

    print('test_main_worktree_lookup (git worktree list, first line)')
    from types import SimpleNamespace
    from unittest.mock import patch
    wt2 = td / 'wt2'
    git(repo, 'worktree', 'add', '-q', '--detach', str(wt2), f)
    got = pg.main_worktree(wt2)
    check('from a linked worktree -> the main worktree folder',
          got is not None and got.resolve() == repo.resolve(), str(got))
    check('missing folder -> None (no exception)', pg.main_worktree(td / 'nope') is None)
    for label, ret in (('git error', SimpleNamespace(returncode=128, stdout='')),
                       ('empty output', SimpleNamespace(returncode=0, stdout='')),
                       ('first line is not a worktree entry',
                        SimpleNamespace(returncode=0, stdout='bare\n'))):
        with patch.object(pg.subprocess, 'run', return_value=ret):
            check(f'{label} -> None', pg.main_worktree(repo) is None)
    swd = [r for r, v in pg.ROLES.items() if v[3] == pg.FLASH_SWD][0]
    elf = td / 'wt' / 'build' / 'rocketchip.elf'
    txt = pg.flash_instructions(got, td / 'wt', swd, elf, 'PT')
    check('OpenOCD step names the main worktree folder and the start script',
          f'Start OpenOCD from {got}:' in txt and pg.OPENOCD_SCRIPT in txt
          and 'Never start it from the bench worktree' in txt
          and 'locks the folder it starts in' in txt, txt)
    txt = pg.flash_instructions(None, td / 'wt', swd, elf, 'PT')
    check('lookup failed -> generic main-checkout text, no fixed path',
          f'Start OpenOCD from {pg.MAIN_FALLBACK}:' in txt, txt)
    check('start script exists in the repo',
          (HERE / pg.OPENOCD_SCRIPT.replace('\\', '/').split('/', 1)[1]).is_file(),
          pg.OPENOCD_SCRIPT)
    for r in pg.ROLES:
        txt = pg.flash_instructions(got, td / 'wt', r, elf, 'PT')
        check(f'{r}: Path 1 is picotool load -f, then --record-only, LED + CDC, '
              'no reset halt',
              txt.index('Path 1') < txt.index('PT load') < txt.index('--record-only')
              < txt.index('LED + CDC') and 'No reset halt' in txt, txt)
    git(repo, 'worktree', 'remove', '--force', str(wt2))

    print('test_build_list_completeness_blocks')
    import check_elf_paths as cep
    old = cep.check
    try:
        def fake_check(wt, bdirs):
            r = cep.Result()
            r.uncovered = {'tools/x.h': {'build_flight:deps'}}
            return r
        cep.check = fake_check
        try:
            pg.check_list_complete(td, td, 'vehicle')
            check('uncovered file -> block', False)
        except pg.GateFail as exc:
            check('uncovered file -> block, names it', 'tools/x.h' in str(exc), str(exc))
        cep.check = lambda wt, bdirs: cep.Result()
        try:
            pg.check_list_complete(td, td, 'vehicle')
            check('complete list -> ok', True)
        except pg.GateFail as exc:
            check('complete list -> ok', False, str(exc))
    finally:
        cep.check = old

    print('test_build_compiler_must_match_key')
    bdir = td / 'bdir'
    bdir.mkdir()
    keyed = ft.tool_versions(repo, f)

    def cache(gcc: Path, sdk_dir: Path, extra: str = '') -> None:
        (bdir / 'CMakeCache.txt').write_text(
            f'CMAKE_C_COMPILER:STRING={gcc}\nPICO_SDK_PATH:PATH={sdk_dir}\n'
            'picotool_DIR:PATH=picotool_DIR-NOTFOUND\n' + extra)

    def blocks(text: str) -> bool:
        try:
            pg.check_build_tools(bdir, 'vehicle', keyed, repo)
        except pg.GateFail as exc:
            return text in str(exc)
        return False
    cache(gcc_a, sdk)
    try:
        pg.check_build_tools(bdir, 'vehicle', keyed, repo)
        check('same compiler and SDK -> ok', True)
    except pg.GateFail as exc:
        check('same compiler and SDK -> ok', False, str(exc))
    cache(gcc_b, sdk)
    check('other compiler -> block, names RC_ARM_GCC', blocks('RC_ARM_GCC'))
    other_sdk = td / 'sdk2'
    shutil.copytree(sdk, other_sdk, symlinks=True)
    cache(gcc_a, other_sdk)
    check('build used another SDK path -> block, names RC_PICO_SDK', blocks('RC_PICO_SDK'))
    cache(gcc_a, td / 'plain-sdk')
    check('build SDK not a git checkout -> block', blocks('git'))
    if os.name != 'nt':
        pdir = tools / 'pio_dir'
        fake_tool(pdir, 'pioasm', PA_B)
        cache(gcc_a, sdk, f'pioasm_DIR:PATH={pdir}\n')
        check('build used another pioasm -> block, names RC_PIOASM', blocks('RC_PIOASM'))
        cache(gcc_a, sdk)
        (bdir / 'build.ninja').write_text(
            f'build x.pio.h: CUSTOM_COMMAND x.pio\n  COMMAND = cd /b && {pdir}/pioasm -o c-sdk x.pio x.pio.h\n')
        check('pioasm read from build.ninja (pico-vscode, no pioasm_DIR in cache)',
              ft.tool_from_cache('pioasm', bdir) == pdir / 'pioasm'
              and blocks('RC_PIOASM'), str(ft.tool_from_cache('pioasm', bdir)))
        (bdir / 'build.ninja').unlink()
    else:
        print('  [SKIP] pioasm_DIR check needs a real pioasm.exe on Windows')


if __name__ == '__main__':
    sys.exit(main())
