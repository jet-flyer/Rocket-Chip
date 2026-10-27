#!/usr/bin/env python3
"""Host test: firmware path excludes and the firmware-tree id.

No hardware. Checks parse, match, that a Markdown change does not change
the tree id, that a .cpp change does, and that cmake/rc_version.cmake
prints the same id as scripts/firmware_tree.py.
"""
from __future__ import annotations

import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path, PurePosixPath

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import firmware_tree as ft  # noqa: E402

# The pre-commit hook exports GIT_DIR/GIT_INDEX_FILE for the real repo. Drop them so
# every git call here (incl. firmware_tree._git and cmake -P) stays in the temp repo.
for _name in ft.GIT_REPO_ENV_VARS:
    os.environ.pop(_name, None)

FAILS = 0


def check(name: str, ok: bool, detail: object = '') -> None:
    global FAILS
    print(f'  [{"PASS" if ok else "FAIL"}] {name}')
    if not ok:
        FAILS += 1
        if detail:
            print('    ' + str(detail).replace('\n', '\n    '))


def git(repo: Path, *args: str) -> str:
    # clean_git_env: under a hook, GIT_DIR/GIT_INDEX_FILE would aim these calls at the real repo.
    r = subprocess.run(['git', *args], cwd=str(repo), capture_output=True, text=True,
                       env=ft.clean_git_env())
    if r.returncode != 0:
        raise RuntimeError(f'git {" ".join(args)} failed: {r.stderr.strip()}')
    return r.stdout


def cmake_tree_id(source: Path) -> str:
    cmake = shutil.which('cmake')
    if not cmake:
        raise RuntimeError('cmake is not on PATH')
    td = Path(tempfile.mkdtemp(prefix='rc_fw_cmake_'))
    try:
        r = subprocess.run(
            [cmake, f'-DRC_FW_TREE_SOURCE={source}', '-P',
             str(ROOT / 'cmake' / 'rc_version.cmake')],
            cwd=str(td), capture_output=True, text=True, env=ft.clean_git_env())
        if r.returncode != 0:
            raise RuntimeError((r.stderr or r.stdout).strip())
        lines = [ln.strip() for ln in r.stdout.splitlines() if ln.strip()]
        if not lines:
            raise RuntimeError(f'cmake printed no id: {r.stderr.strip()}')
        return lines[-1]
    finally:
        shutil.rmtree(td, ignore_errors=True)


def assert_ids(repo: Path, label: str) -> None:
    py = ft.elf_tree_id(repo, 'HEAD')
    cm = cmake_tree_id(repo)
    dirty = ft.dirty_firmware_paths(repo)
    want = f'{py}-dirty' if dirty else py
    check(label, cm == want, f'cmake={cm}\npython={py}\ndirty={dirty}')


def test_parse_and_match() -> None:
    print('test_parse_and_match')
    text = (
        '!**/*.md\n'
        '[elf]\n'
        'src/\n'
        '!**/*.md\n'
        '[radio]\n'
        'src/starcom_adapt/\n'
        '!src/only_radio.md\n'
        '[station-only]\n'
        'src/cli/rc_os_dashboard*\n'
    )
    sections = ft.parse(text)
    check('exclude before a section is global',
          sections[ft.EXCLUDE_KEY] == ['**/*.md', 'src/only_radio.md'],
          sections[ft.EXCLUDE_KEY])
    check('exclude lines are not section entries',
          sections['elf'] == ['src/'] and sections['radio'] == ['src/starcom_adapt/'])
    check('duplicate glob is stored once',
          ft.parse('!**/*.md\n[elf]\nsrc/\n!**/*.md\n')[ft.EXCLUDE_KEY] == ['**/*.md'])
    check('md is not in elf', not ft.in_section('src/a.md', sections, 'elf'))
    check('cpp is in elf', ft.in_section('src/a.cpp', sections, 'elf'))
    check('exclude written under radio still drops elf',
          not ft.in_section('src/only_radio.md', sections, 'elf'))
    check('readme is not in radio',
          not ft.in_section('src/starcom_adapt/README.md', sections, 'radio'))
    check('radio cpp still matches',
          ft.in_section('src/starcom_adapt/byte_pump.cpp', sections, 'radio'))
    check('prefix* still matches a cpp name',
          ft.in_section('src/cli/rc_os_dashboard.cpp', sections, 'station-only'))
    check('prefix* does not keep an excluded md',
          not ft.in_section('src/cli/rc_os_dashboard.md', sections, 'station-only'))
    check('! entry is not an include',
          not ft.entry_matches('src/a.cpp', '!**/*.md'))
    check('exclude blocks an include entry',
          not ft.entry_matches('src/a.md', 'src/', ['**/*.md']))
    check('cpp include still matches',
          ft.entry_matches('src/a.cpp', 'src/', ['**/*.md']))
    for bad, needle in (
        ('[elf]\nsrc/\n!\n', 'empty exclude'),
        ('[elf]\nfoo*\n', 'no *'),
        ('plain\n[elf]\nsrc/\n', 'before any'),
    ):
        try:
            ft.parse(bad)
            check(f'parse rejects {needle}', False)
        except ft.FirmwareTreeError as exc:
            check(f'parse rejects {needle}', needle in str(exc), exc)

    cases = (
        ('README.md', '**/*.md', True),
        ('src/a.md', '**/*.md', True),
        ('src/a/b.md', '**/*.md', True),
        ('src/a.cpp', '**/*.md', False),
        ('src/a.md', '*.md', False),
        ('README.md', '*.md', True),
        ('src/a.md', 'src/**/*.md', True),
        ('src/a/b.md', 'src/**/*.md', True),
        ('src/a.md', 'src/*.md', True),
        ('src/a/b.md', 'src/*.md', False),
        ('a/b.md', 'a/**/b.md', True),
        ('a/x/y/b.md', 'a/**/b.md', True),
        ('src/foo.md.bak', '**/*.md', False),
        ('src/my note.md', '**/*.md', True),
    )
    for path, pattern, exp in cases:
        check(f'glob {pattern} ~ {path}', ft.glob_matches(path, pattern) is exp)
        check(f'pathlib agrees {pattern} ~ {path}',
              PurePosixPath(path).full_match(pattern) is exp)


def test_tree_and_cmake(td: Path) -> None:
    print('test_tree_and_cmake')
    repo = td / 'repo'
    repo.mkdir()
    hooks = td / 'nohooks'
    hooks.mkdir()
    git(repo, 'init', '-q', '-b', 'main')
    git(repo, 'config', 'user.name', 'test')
    git(repo, 'config', 'user.email', 'test@example.invalid')
    git(repo, 'config', 'core.autocrlf', 'false')
    git(repo, 'config', 'commit.gpgsign', 'false')
    git(repo, 'config', 'core.hooksPath', str(hooks))
    listed = (
        '!**/*.md\n'
        '[elf]\n'
        'src/\n'
        '[radio]\n'
        'src/starcom_adapt/\n'
        '!src/only_radio.md\n'
        '[gate]\n'
        'scripts/ci/\n'
    )
    files = {
        'scripts/ci/firmware_paths.txt': listed,
        'src/a.cpp': 'int a;\n',
        'src/a.md': '# a\n',
        'src/sub/b.md': '# b\n',
        'src/my note.md': '# space\n',
        'src/only_radio.md': '# radio placement\n',
        'src/starcom_adapt/x.cpp': 'int x;\n',
        'src/starcom_adapt/README.md': '# readme\n',
        'README.md': '# top\n',
        'a/b.md': '# ab\n',
        'a/x/y/b.md': '# axy\n',
    }
    for rel, body in files.items():
        path = repo / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(body.encode('utf-8'))
    git(repo, 'add', '-A')
    git(repo, 'commit', '-q', '-m', 'base')

    def git_glob(pattern: str) -> set[str]:
        raw = subprocess.run(
            ['git', 'ls-files', '-z', '--', f':(glob){pattern}'],
            cwd=str(repo), capture_output=True, env=ft.clean_git_env())
        if raw.returncode != 0:
            raise RuntimeError(raw.stderr.decode())
        return {p.decode('utf-8') for p in raw.stdout.split(b'\0') if p}

    listed_paths = {p.decode('utf-8') for p in subprocess.run(
        ['git', 'ls-files', '-z'], cwd=str(repo), capture_output=True, env=ft.clean_git_env()).stdout.split(b'\0') if p}
    for pattern in ('**/*.md', '*.md', 'src/**/*.md', 'src/*.md', 'a/**/b.md'):
        got = {p for p in listed_paths if ft.glob_matches(p, pattern)}
        want = git_glob(pattern)
        check(f'glob matches git :(glob){pattern}', got == want,
              f'only python {sorted(got - want)}\nonly git {sorted(want - got)}')

    id1 = ft.elf_tree_id(repo, 'HEAD')
    check('base id is a blob id', len(id1) == 40 and 'unknown' not in id1, id1)
    for rel in ('src/a.md', 'src/sub/b.md', 'src/my note.md',
                'src/only_radio.md', 'src/starcom_adapt/README.md'):
        (repo / rel).write_bytes(b'# changed\n')
    git(repo, 'add', '-A')
    git(repo, 'commit', '-q', '-m', 'md only')
    id2 = ft.elf_tree_id(repo, 'HEAD')
    check('markdown commit does not change the tree id', id2 == id1, f'{id1} -> {id2}')
    assert_ids(repo, 'cmake matches python after the markdown commit')

    (repo / 'src/a.cpp').write_bytes(b'int a_changed;\n')
    git(repo, 'add', '-A')
    git(repo, 'commit', '-q', '-m', 'cpp')
    id3 = ft.elf_tree_id(repo, 'HEAD')
    check('cpp commit changes the tree id', id3 != id2, id3)
    assert_ids(repo, 'cmake matches python after the cpp commit')

    (repo / 'src/a.md').write_bytes(b'# dirty md\n')
    (repo / 'src/new.md').write_bytes(b'# new\n')
    check('dirty markdown is not a firmware path',
          ft.dirty_firmware_paths(repo) == [], ft.dirty_firmware_paths(repo))
    assert_ids(repo, 'dirty markdown does not mark the cmake id dirty')

    (repo / 'src/a.cpp').write_bytes(b'int dirty;\n')
    dirty = ft.dirty_firmware_paths(repo)
    check('dirty cpp is a firmware path', dirty == ['src/a.cpp'], dirty)
    assert_ids(repo, 'dirty cpp marks the cmake id dirty')


def test_this_repo() -> None:
    print('test_this_repo')
    sections = ft.load(ROOT)
    check('this repo excludes **/*.md', '**/*.md' in sections[ft.EXCLUDE_KEY],
          sections[ft.EXCLUDE_KEY])
    check('no exclude line stored as an elf path',
          all(not e.startswith('!') for e in sections['elf']))
    check('src markdown is not elf',
          not ft.in_section('src/starcom_adapt/README.md', sections, 'elf'))
    check('src markdown is not radio',
          not ft.in_section('src/starcom_adapt/README.md', sections, 'radio'))
    check('radio cpp stays in elf and radio',
          ft.in_section('src/starcom_adapt/byte_pump.cpp', sections, 'elf')
          and ft.in_section('src/starcom_adapt/byte_pump.cpp', sections, 'radio'))
    check('markdown has no isolate class',
          ft.classes('src/starcom_adapt/README.md', sections) == [])
    check('cpp keeps the radio-adapter class',
          'radio-adapter' in ft.classes('src/starcom_adapt/byte_pump.cpp', sections))
    assert_ids(ROOT, 'this repo: cmake tree id equals python')


def main() -> int:
    td = Path(tempfile.mkdtemp(prefix='rc_fwtree_'))
    try:
        test_parse_and_match()
        test_tree_and_cmake(td)
        test_this_repo()
    finally:
        shutil.rmtree(td, ignore_errors=True)
    print('ALL CHECKS PASS' if FAILS == 0 else f'{FAILS} CHECK(S) FAILED')
    return 0 if FAILS == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
