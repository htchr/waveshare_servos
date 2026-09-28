"""
The vendored SCServo files are byte-for-byte the ones THIRD_PARTY.md records.

Also checks the table size, the upper-case file names, the CMake lists and the MIT notices.
"""

import hashlib
from pathlib import Path
import re

import pytest

PKG_ROOT = Path(__file__).resolve().parents[1]
THIRD_PARTY = PKG_ROOT / 'THIRD_PARTY.md'
CMAKELISTS = PKG_ROOT / 'CMakeLists.txt'

# Pinned apart from the lists it checks, so shrinking all of them fails. Change it only with
# THIRD_PARTY.md, add_library(scservo ...) and AMENT_LINT_AUTO_FILE_EXCLUDE.
EXPECTED_FILE_COUNT = 14

BLOCK = re.compile(
    r'<!-- vendored-sha256:begin -->\s*```text\n(?P<body>.*?)```\s*<!-- vendored-sha256:end -->',
    re.S)
ROW = re.compile(r'^(?P<sha>[0-9a-f]{64})  (?P<path>(?:include|src)/[A-Za-z0-9_]+\.(?:h|cpp))$')

# Synthetic tree for the self-tests. The digests are computed by hand (printf ... | sha256sum),
# so the check uses known values, not values it computed itself.
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
    # Prove the comparison can fail: a matching synthetic tree, then one flipped byte.
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
