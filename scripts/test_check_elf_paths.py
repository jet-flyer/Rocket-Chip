#!/usr/bin/env python3
"""Host test: scripts/ci/check_elf_paths.py (list-completeness check).

No hardware. Part A uses canned ninja output (always runs). Part B builds
a tiny CMake + Ninja fixture with the host C compiler and runs the real
queries (ninja -t deps, compile_commands.json, ninja -t inputs <elf>,
ninja -t query build.ninja). Part B is SKIPPED when cmake, ninja or a C
compiler is missing.
"""
from __future__ import annotations

import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / 'ci'))
sys.path.insert(0, str(HERE))
import check_elf_paths as cep  # noqa: E402

FAILS = 0


def check(name: str, ok: bool, detail: str = '') -> None:
    global FAILS
    print(f'  [{"PASS" if ok else "FAIL"}] {name}')
    if not ok:
        FAILS += 1
        if detail:
            print('    ' + str(detail).replace('\n', '\n    '))


def write(p: Path, text: str = '') -> Path:
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(text, encoding='utf-8')
    return p


def part_a(td: Path) -> None:
    print('test_parsers')
    deps = ('CMakeFiles/rocketchip.dir/src/main.c.obj: #deps 3, deps mtime 1 (VALID)\n'
            '    ../src/main.c\n    ../include/a.h\n    C:/sdk/x.h\n\n'
            'other.obj: #deps 1, deps mtime 1 (STALE)\n    ../src/b.h\n')
    check('deps: indented lines only',
          cep.parse_deps(deps) == ['../src/main.c', '../include/a.h', 'C:/sdk/x.h', '../src/b.h'])
    check('inputs: one per line', cep.parse_inputs('a\n\nb\n') == ['a', 'b'])
    q = ('build.ninja:\n  input: RERUN_CMAKE\n    | /r/CMakeLists.txt\n'
         '    | /r/cmake/x.cmake\n    /r/explicit.cmake\n    || /r/order.txt\n'
         '  outputs:\n    /r/b/other\n')
    check('query: input part only, | and || stripped',
          cep.parse_query_inputs(q) == ['/r/CMakeLists.txt', '/r/cmake/x.cmake',
                                        '/r/explicit.cmake', '/r/order.txt'],
          cep.parse_query_inputs(q))
    check('compdb: directory + file',
          cep.parse_compdb('[{"directory": "/b", "file": "../src/m.c"}]')
          == [os.path.join('/b', '../src/m.c')])

    print('test_classify_canned')
    repo = td / 'repoA'
    write(repo / 'scripts/ci/firmware_paths.txt',
          '[elf]\nsrc/\ninclude/\nCMakeLists.txt\npico-sdk\n'
          '[elf-exempt]\n.git/\n')
    for f in ('src/main.c', 'include/a.h', 'tools/x.h', 'CMakeLists.txt', '.git/HEAD',
              'pico-sdk/src/rp2_common/foo.h', 'build_flight/CMakeCache.txt',
              'build_flight/generated/version.h'):
        write(repo / f)
    ext = write(td / 'home/.pico-sdk/sdk/2.2.0/src/common/bar.h')
    bdir = repo / 'build_flight'
    raw = {
        'deps': ['../src/main.c', '../include/a.h', '../tools/x.h',
                 str(ext), 'generated/version.h', '../src/gone.h',
                 '../pico-sdk/src/rp2_common/foo.h'],
        'compdb': [str(repo / 'src/main.c')],
        'inputs': [str(repo / 'src/main.c'), 'generated/version.h'],
        'regen': [str(repo / 'CMakeLists.txt'), str(ext), str(repo / '.git/HEAD')],
    }
    res = cep.check(repo, [bdir], query=lambda b: raw)
    c = res.counts['build_flight:deps']
    check('uncovered repo file found', set(res.uncovered) == {'tools/x.h'}, res.uncovered)
    check('submodule file covered by gitlink entry', c['covered'] == 3, c)
    check('generated build-dir file ignored', c['generated'] == 1, c)
    check('external file ignored', c['external'] == 1, c)
    check('missing (deleted) file skipped', c['missing'] == 1, c)
    check('[elf-exempt] path accepted, counted as exempt',
          res.counts['build_flight:regen']['exempt'] == 1, res.counts)
    want = ('INFO: the build used a Pico SDK outside the repo: '
            + (td / 'home/.pico-sdk/sdk/2.2.0').resolve().as_posix() + '.')
    got = cep.report(res)
    if os.name == 'nt':  # paths are case-folded on Windows (os.path.normcase)
        want, got = want.lower(), got.lower()
    check('Pico SDK outside the repo -> INFO line with the SDK top dir',
          want in got, cep.report(res))
    try:
        cep.check(repo, [bdir], query=lambda b: dict(raw, inputs=[]))
        check('empty query fails closed', False)
    except cep.CheckError as exc:
        check('empty query fails closed', 'inputs' in str(exc), exc)


FIXTURE_CMAKE = '''cmake_minimum_required(VERSION 3.20)
project(fx C)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(${CMAKE_SOURCE_DIR}/extra/flags.cmake)
add_custom_command(
  OUTPUT ${CMAKE_BINARY_DIR}/generated/blink.pio.h
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/pio/blink.pio
          ${CMAKE_BINARY_DIR}/generated/blink.pio.h
  DEPENDS ${CMAKE_SOURCE_DIR}/pio/blink.pio)
add_executable(rocketchip src/main.c ${CMAKE_BINARY_DIR}/generated/blink.pio.h)
target_include_directories(rocketchip PRIVATE include tools ${CMAKE_BINARY_DIR}/generated)
set_target_properties(rocketchip PROPERTIES SUFFIX .elf
  LINK_DEPENDS ${CMAKE_SOURCE_DIR}/ld/memmap.ld)
'''


def part_b(td: Path) -> None:
    print('test_real_cmake_ninja_fixture')
    cmake = shutil.which('cmake')
    ninja = shutil.which('ninja')
    cc = shutil.which('cc') or shutil.which('gcc') or shutil.which('clang')
    if not (cmake and ninja and cc):
        print('  [SKIP] needs cmake, ninja and a host C compiler')
        return
    repo = td / 'repoB'
    write(repo / 'CMakeLists.txt', FIXTURE_CMAKE)
    write(repo / 'extra/flags.cmake', 'add_compile_definitions(FX=1)\n')
    write(repo / 'pio/blink.pio', '/* pio */\n')
    write(repo / 'ld/memmap.ld', '/* ld */\n')
    write(repo / 'include/a.h', '#define A 1\n')
    write(repo / 'tools/x.h', '#define X 2\n')
    write(repo / 'src/main.c', '#include "a.h"\n#include "x.h"\n#include "blink.pio.h"\n'
          'int main(void) { return A + X; }\n')
    lst = write(repo / 'scripts/ci/firmware_paths.txt',
                '[elf]\nsrc/\ninclude/\nCMakeLists.txt\n')
    bdir = repo / 'build_flight'
    r = subprocess.run([cmake, '-S', str(repo), '-B', str(bdir), '-G', 'Ninja',
                        f'-DCMAKE_MAKE_PROGRAM={ninja}'], capture_output=True, text=True)
    r2 = subprocess.run([cmake, '--build', str(bdir)], capture_output=True, text=True)
    if r.returncode or r2.returncode:
        check('fixture builds', False, r.stdout + r.stderr + r2.stdout + r2.stderr)
        return
    res = cep.check(repo, [bdir])
    print(cep.report(res))
    want = {'tools/x.h': 'deps', 'extra/flags.cmake': 'regen',
            'pio/blink.pio': 'inputs', 'ld/memmap.ld': 'inputs'}
    for path, src in want.items():
        check(f'{path} not in [elf] -> found by {src}',
              any(s.endswith(':' + src) for s in res.uncovered.get(path, ())),
              res.uncovered)
    check('nothing else flagged', set(res.uncovered) == set(want), res.uncovered)
    check('generated pio header ignored',
          res.counts['build_flight:inputs']['generated'] >= 1, res.counts)
    write(lst, '[elf]\nsrc/\ninclude/\nCMakeLists.txt\ntools/\nextra/\npio/\nld/\n')
    res = cep.check(repo, [bdir])
    check('all paths in [elf] -> OK', res.ok, cep.report(res))
    env = dict(os.environ)
    rc = subprocess.run([sys.executable, str(HERE / 'ci' / 'check_elf_paths.py'),
                         str(bdir)], cwd=str(repo), capture_output=True, text=True,
                        env=env)
    check('CLI exit 0 and OK line when the list is complete',
          rc.returncode == 0 and 'check_elf_paths: OK' in rc.stdout, rc.stdout + rc.stderr)


def main() -> int:
    td = Path(tempfile.mkdtemp(prefix='rc_elfpaths_'))
    try:
        part_a(td)
        part_b(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    print('ALL CHECKS PASS' if FAILS == 0 else f'{FAILS} CHECK(S) FAILED')
    return 0 if FAILS == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
