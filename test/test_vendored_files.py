"""
The vendored SCServo files are byte-for-byte the ones THIRD_PARTY.md records.

THIRD_PARTY.md says the 14 vendored files are not edited, and the package leans on that in two
ways nothing else checks. The ServoBus wrapper relies on details of them that are not a stable API,
and code and tests across the package cite them by line number. The behavioural tests in
test/test_servo_bus.cpp catch a change only on the paths they drive: a comment edit, a whitespace
cleanup, or any edit to SMSBL/SMSCL/SCSCL (compiled, never linked) passes every other test in the
suite. This one does not.

The recorded table is the fenced block between the vendored-sha256 markers in THIRD_PARTY.md, in
`sha256sum` format, so a person can check it with `sha256sum -c` as well.

Four cross-checks keep the gate from passing vacuously:
  - the table must hold exactly EXPECTED_FILE_COUNT rows, so an empty or truncated table fails;
  - every file whose name starts with an upper-case letter under include/ or src/ must be in the
    table (every driver-owned source name is lower-case), so a newly vendored file cannot slip in
    unrecorded;
  - the CMake lint exclusion list and the scservo target must name exactly the tabled files;
  - two self-tests run the digest comparison over a synthetic tree whose hashes are written out
    below, and require it to report a flipped byte and a deleted file.

One more case reads THIRD_PARTY.md's prose: its "Licensing" section must reproduce both MIT
notices of the upstream sources, the two copyright lines and the whole permission text, because
the MIT license asks that copies carry them.

No fixture reads a file: each case first asserts that THIRD_PARTY.md exists and then parses it, so
a missing or malformed table is a failed case, never an error at setup. No motors, no port, no ROS
graph, no git. The test reads the SOURCE tree (ament_add_pytest_test runs this file from the
source directory, so __file__ is there), because the claim is about the sources.
"""

import hashlib
from pathlib import Path
import re

import pytest

PKG_ROOT = Path(__file__).resolve().parents[1]
THIRD_PARTY = PKG_ROOT / 'THIRD_PARTY.md'
CMAKELISTS = PKG_ROOT / 'CMakeLists.txt'

# Pinned independently of every list this test compares, so shrinking all of them together still
# fails. Change it only together with THIRD_PARTY.md, add_library(scservo ...) and
# AMENT_LINT_AUTO_FILE_EXCLUDE, in the same reviewed change.
EXPECTED_FILE_COUNT = 14

BLOCK = re.compile(
    r'<!-- vendored-sha256:begin -->\s*```text\n(?P<body>.*?)```\s*<!-- vendored-sha256:end -->',
    re.S)
ROW = re.compile(r'^(?P<sha>[0-9a-f]{64})  (?P<path>(?:include|src)/[A-Za-z0-9_]+\.(?:h|cpp))$')

# The self-tests' synthetic tree: two small files with fixed content, and their sha256 written
# out by hand (`printf 'synthetic vendored header\n' | sha256sum`), so the comparison is checked
# against known digests rather than against digests it computed itself.
SYNTHETIC_FILES = {
    'include/ONE.h': b'synthetic vendored header\n',
    'src/TWO.cpp': b'synthetic vendored source\n',
}
SYNTHETIC_TABLE = {
    'include/ONE.h': '70b01f2ed3185c8deb0c6d52a0a2871ea08baeb3a6b767ca954af10fddaffde2',
    'src/TWO.cpp': '0e387f7d7031cfa70be2ceac2d77b3c7dde1407d62d0f22e235528670205575d',
}


# What the "Licensing" section must contain, compared with its whitespace collapsed: the two
# upstream copyright lines and the MIT permission text they are published under.
MIT_COPYRIGHT_LINES = (
    'Copyright (c) 2024 FTServo',
    'Copyright (c) 2025 Aditya Kamath (Kamath Robotics)',
)
MIT_BODY = (
    'Permission is hereby granted, free of charge, to any person obtaining a copy of this '
    'software and associated documentation files (the "Software"), to deal in the Software '
    'without restriction, including without limitation the rights to use, copy, modify, merge, '
    'publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons '
    'to whom the Software is furnished to do so, subject to the following conditions: '
    'The above copyright notice and this permission notice shall be included in all copies or '
    'substantial portions of the Software. '
    'THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, '
    'INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR '
    'PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE '
    'FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR '
    'OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER '
    'DEALINGS IN THE SOFTWARE.')
MIT_PERMISSION_SENTENCE = (
    'The above copyright notice and this permission notice shall be included in all copies or '
    'substantial portions of the Software.')


def section(markdown_text, title):
    """Return the text from the `## <title>` heading to the next `## ` heading, or None."""
    match = re.search(r'^## ' + re.escape(title) + r'[ \t]*$(?P<body>.*?)(?=^## |\Z)',
                      markdown_text, re.S | re.M)
    return None if match is None else match.group('body')


def parse_table(markdown_text):
    """Return {path: sha256} from the marked block; raise ValueError on anything malformed."""
    blocks = BLOCK.findall(markdown_text)
    if len(blocks) != 1:
        raise ValueError(f'expected exactly one vendored-sha256 block, found {len(blocks)}')
    table = {}
    for line in blocks[0].splitlines():
        if not line.strip():
            continue
        match = ROW.match(line)
        if match is None:
            raise ValueError(f'malformed row: {line!r}')
        if match.group('path') in table:
            raise ValueError(f'duplicate row: {match.group("path")}')
        table[match.group('path')] = match.group('sha')
    return table


def digest(path):
    """Return the sha256 of a file's bytes, as lower-case hex."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def mismatches(root, table):
    """Return the tabled paths under root whose bytes do not hash to the recorded value."""
    return sorted(path for path, sha in table.items()
                  if not (root / path).is_file() or digest(root / path) != sha)


def cmake_list(text, opener):
    """Return the whitespace-separated entries between `opener` and the next ')'."""
    assert opener in text, f'CMakeLists.txt has no {opener!r}'
    start = text.index(opener) + len(opener)
    return text[start:text.index(')', start)].split()


def load_table():
    """Parse THIRD_PARTY.md's table; a malformed table fails the calling case on an assertion."""
    try:
        return parse_table(THIRD_PARTY.read_text(encoding='utf-8'))
    except ValueError as error:
        raise AssertionError(f'THIRD_PARTY.md: {error}') from error


def write_synthetic_tree(root):
    for relative, data in SYNTHETIC_FILES.items():
        (root / relative).parent.mkdir(parents=True, exist_ok=True)
        (root / relative).write_bytes(data)


def test_the_table_lists_exactly_the_expected_number_of_files():
    assert THIRD_PARTY.is_file(), 'THIRD_PARTY.md is missing'
    table = load_table()
    assert len(table) == EXPECTED_FILE_COUNT, (
        f'THIRD_PARTY.md lists {len(table)} vendored files, expected {EXPECTED_FILE_COUNT}: '
        f'{sorted(table)}')


def test_every_vendored_file_matches_its_recorded_sha256():
    assert THIRD_PARTY.is_file(), 'THIRD_PARTY.md is missing'
    table = load_table()
    bad = mismatches(PKG_ROOT, table)
    assert bad == [], (
        f'vendored files differ from THIRD_PARTY.md: {bad}. These files are read-only; put the '
        'change in ServoBus or a driver-side copy instead (THIRD_PARTY.md, "The rule").')


def test_every_upper_case_source_file_is_in_the_table():
    assert THIRD_PARTY.is_file(), 'THIRD_PARTY.md is missing'
    table = load_table()
    found = sorted(p.relative_to(PKG_ROOT).as_posix()
                   for d in ('include', 'src') for p in (PKG_ROOT / d).rglob('*')
                   if p.is_file() and p.name[:1].isupper())
    assert found == sorted(table), (
        'the upper-case (vendored) files under include/ and src/ are not the ones THIRD_PARTY.md '
        f'lists: not in the table {sorted(set(found) - set(table))}, '
        f'in the table but absent {sorted(set(table) - set(found))}')


def test_the_lint_exclusion_list_names_exactly_the_tabled_files():
    assert THIRD_PARTY.is_file(), 'THIRD_PARTY.md is missing'
    assert CMAKELISTS.is_file(), 'CMakeLists.txt is missing'
    table = load_table()
    excluded = cmake_list(CMAKELISTS.read_text(encoding='utf-8'),
                          'set(AMENT_LINT_AUTO_FILE_EXCLUDE')
    assert sorted(excluded) == sorted(table)


def test_the_scservo_target_compiles_exactly_the_tabled_sources():
    assert THIRD_PARTY.is_file(), 'THIRD_PARTY.md is missing'
    assert CMAKELISTS.is_file(), 'CMakeLists.txt is missing'
    table = load_table()
    sources = cmake_list(CMAKELISTS.read_text(encoding='utf-8'), 'add_library(scservo STATIC')
    assert sorted(sources) == sorted(p for p in table if p.endswith('.cpp'))


def test_the_comparison_reports_a_one_byte_change(tmp_path):
    # The gate that cannot fail is the failure mode this package keeps finding. Prove this one can:
    # a synthetic tree that matches its table, then one byte flipped in one file.
    write_synthetic_tree(tmp_path)
    assert mismatches(tmp_path, SYNTHETIC_TABLE) == []
    victim = tmp_path / 'src' / 'TWO.cpp'
    data = bytearray(victim.read_bytes())
    data[len(data) // 2] ^= 0x01
    victim.write_bytes(bytes(data))
    assert mismatches(tmp_path, SYNTHETIC_TABLE) == ['src/TWO.cpp']


def test_a_missing_file_is_reported(tmp_path):
    write_synthetic_tree(tmp_path)
    (tmp_path / 'include' / 'ONE.h').unlink()
    assert mismatches(tmp_path, SYNTHETIC_TABLE) == ['include/ONE.h']


def test_the_licensing_section_reproduces_both_mit_notices():
    assert THIRD_PARTY.is_file(), 'THIRD_PARTY.md is missing'
    body = section(THIRD_PARTY.read_text(encoding='utf-8'), 'Licensing')
    assert body is not None, 'THIRD_PARTY.md has no "## Licensing" section'
    text = re.sub(r'\s+', ' ', body)
    wanted = MIT_COPYRIGHT_LINES + (MIT_PERMISSION_SENTENCE, MIT_BODY)
    missing = [piece for piece in wanted if piece not in text]
    assert missing == [], (
        'THIRD_PARTY.md, "Licensing", does not reproduce the MIT notices in full; missing: '
        f'{missing}')


@pytest.mark.parametrize('broken', [
    'no block at all',
    '<!-- vendored-sha256:begin -->\n```text\n' + 'A' * 64 + '  src/SCS.cpp\n```\n'
    '<!-- vendored-sha256:end -->',
    '<!-- vendored-sha256:begin -->\n```text\n' + 'a' * 64 + ' src/SCS.cpp\n```\n'
    '<!-- vendored-sha256:end -->',
    '<!-- vendored-sha256:begin -->\n```text\n' + ('a' * 64 + '  src/SCS.cpp\n') * 2 + '```\n'
    '<!-- vendored-sha256:end -->',
], ids=['no_block', 'upper_case_hex', 'one_space_separator', 'duplicate_row'])
def test_a_malformed_table_is_rejected(broken):
    with pytest.raises(ValueError):
        parse_table(broken)
