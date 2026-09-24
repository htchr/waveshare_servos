"""
package.xml and CHANGELOG.rst describe one release, and no release file points outside the repo.

Four things are checked here, none of which any other test in the package can see:
  - package.xml is a valid format-3 manifest, and each <license> names a license file that is
    inside the package and holds the expected text (its sha256 is pinned below);
  - CHANGELOG.rst exists, parses, is warning-free reStructuredText with no comment (a docutils
    comment node anywhere, a list item included), its version sections are dated, the versions
    strictly descend in file order and no date is later than the one above it (a same-day patch
    release passes), and its highest version is the one package.xml declares;
  - the files a clone reads first (README.md, CHANGELOG.rst, THIRD_PARTY.md, package.xml,
    CMakeLists.txt, the container files and the example files) name no development document that
    is not in the repository;
  - the release documents carry no placeholder.

No fixture reads a file: each case first asserts that the file it needs exists, so a missing
document is a failed case, never an error at setup; a case that reads several files collects every
problem into one list and asserts once, so its message shows every reason together. The pattern
self-test uses synthetic strings only. No motors, no port, no ROS graph, no git. The test reads
the SOURCE tree through its own __file__.
"""

import datetime
import hashlib
import io
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from catkin_pkg.changelog import get_changelog_from_path
from catkin_pkg.package import InvalidPackage, parse_package
import docutils.core
import docutils.nodes

PKG_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_XML = PKG_ROOT / 'package.xml'
CHANGELOG = PKG_ROOT / 'CHANGELOG.rst'

# The license texts the <license file="..."> attributes name, pinned by content: an emptied or
# edited license file is as wrong as a missing one. Change a digest only together with its file.
LICENSE_SHA256 = {
    'LICENSE': '3972dc9744f6499f0f9b2dbf76696f2ae7ad8af9b23dde66d6af86c9dfb36986',
    'LICENSES/Apache-2.0.txt': 'cfc7749b96f63bd31c3c42b5c471bf756814053e847c10f3eb003417bc523d30',
}

# The files a user of a clone reads or runs, checked for pointers to documents a clone lacks.
RELEASE_FILES = (
    'README.md', 'CHANGELOG.rst', 'THIRD_PARTY.md', 'package.xml', 'CMakeLists.txt',
    '.devcontainer/devcontainer.json', 'docker/Dockerfile', 'docker/setup.sh')
# ... plus every file of these types under these directories (the example a user copies).
EXAMPLE_DIRS = ('description', 'bringup')
EXAMPLE_SUFFIXES = ('.xacro', '.urdf', '.yaml', '.py', '.rviz', '.xml')
# The example files that exist today. The directory walk must find at least these, so a walk that
# silently finds nothing fails instead of checking nothing.
KNOWN_EXAMPLE_FILES = (
    'bringup/config/example_controllers.yaml',
    'bringup/launch/example.launch.py',
    'description/ros2_control/example.ros2_control.xacro',
    'description/rviz/example_ws.rviz',
    'description/urdf/example.urdf.xacro',
)

# A development document, an evidence directory, a spec or plan section, a review-finding id: a
# clone has none of them, so a release file that names one points at nothing.
POINTER_PATTERN = (
    r'jazzy\.md|_evidence|PHASE\d|CLAUDE\.md|(?<![\w/])context/|review fix F\d+|DEVIATIONS\.md'
    r'|recon/|\bPhase \d|\([A-Z]\.\d+\b')
POINTER_POSITIVES = (
    'see jazzy.md', 'phase3_evidence/x', 'PHASE2_SPEC 4.1', 'CLAUDE.md', 'context/a.xlsx',
    'review fix F33', '(Phase 6 item 4)', '(E.1)', '(C.1, about')
POINTER_NEGATIVES = ('the jazzy branch', 'rclcpp context', 'phase of the moon', '(see above)')

PLACEHOLDER = re.compile(r'\[B-README|\bTBD\b|\bTODO\b|CONFIRM WITH THE USER|YYYY-MM-DD')
PLACEHOLDER_FILES = ('README.md', 'CHANGELOG.rst', 'THIRD_PARTY.md', 'package.xml')
PLACEHOLDER_SAMPLES = (
    'see [B-README BR-14]', 'TBD', 'a TODO here', '<!-- CONFIRM WITH THE USER -->',
    '1.0.0 (YYYY-MM-DD)')

VERSION_LINE = re.compile(r'^\d+\.\d+\.\d+\b')
UNDERLINE = re.compile(r'^-+\s*$')
DATED_HEADING = re.compile(r'^(?P<version>\d+\.\d+\.\d+) \((?P<date>\d{4}-\d{2}-\d{2})\)$')


def pointer_pattern():
    """Return the compiled pattern that finds a pointer to a document a clone does not have."""
    return re.compile(POINTER_PATTERN)


def release_files():
    """Return the package-relative paths R7 reads: the fixed list plus the example files."""
    walked = sorted(
        path.relative_to(PKG_ROOT).as_posix()
        for directory in EXAMPLE_DIRS for path in (PKG_ROOT / directory).rglob('*')
        if path.is_file() and path.suffix in EXAMPLE_SUFFIXES
        and '__pycache__' not in path.relative_to(PKG_ROOT).parts)
    return list(RELEASE_FILES) + walked


def changelog_versions():
    """Return catkin_pkg's versions of CHANGELOG.rst, highest first."""
    try:
        changelog = get_changelog_from_path(str(CHANGELOG), 'waveshare_servos')
    except Exception as error:  # catkin_pkg raises several types on a malformed changelog
        raise AssertionError(f'CHANGELOG.rst does not parse: {error!r}') from error
    assert changelog is not None, 'CHANGELOG.rst could not be read'
    return [version for version, _, _ in changelog.foreach_version(reverse=True)]


def version_headings(text):
    """Return (line number, heading text) for every version heading, in file order."""
    lines = text.splitlines()
    return [(index + 1, line.rstrip()) for index, line in enumerate(lines[:-1])
            if VERSION_LINE.match(line) and UNDERLINE.match(lines[index + 1])]


def rst_doctree(text):
    """Parse this reStructuredText; return (doctree, every message of level WARNING or above)."""
    stream = io.StringIO()
    doctree = docutils.core.publish_doctree(
        text, source_path=str(CHANGELOG),
        settings_overrides={'warning_stream': stream, 'report_level': 2, 'halt_level': 5,
                            'file_insertion_enabled': False, 'raw_enabled': False})
    messages = [' '.join(node.astext().split())
                for node in doctree.findall(docutils.nodes.system_message) if node['level'] >= 2]
    if stream.getvalue().strip():
        messages.append('docutils reported: ' + ' '.join(stream.getvalue().split()))
    return doctree, messages


def rst_comments(doctree):
    """Return one problem per comment node of this doctree, wherever it sits, in document order."""
    problems = []
    for node in doctree.findall(docutils.nodes.comment):
        where = f'CHANGELOG.rst:{node.line}' if node.line is not None else 'CHANGELOG.rst'
        text = ' '.join(node.astext().split())
        problems.append(f'{where}: an RST comment: {text!r}' if text
                        else f'{where}: an empty RST comment')
    return problems


def test_package_xml_is_a_valid_format_3_manifest():
    assert PACKAGE_XML.is_file(), 'package.xml is missing'
    try:
        package = parse_package(str(PACKAGE_XML))
        package.validate()
    except InvalidPackage as error:
        raise AssertionError(f'package.xml is not a valid manifest: {error}') from error
    assert str(package.package_format) == '3', (
        f'package.xml is format {package.package_format}, expected 3')


def test_every_license_names_a_file_that_exists():
    assert PACKAGE_XML.is_file(), 'package.xml is missing'
    licenses = ET.parse(PACKAGE_XML).getroot().findall('license')
    problems = [] if licenses else ['package.xml declares no <license>']
    for element in licenses:
        name = (element.text or '').strip()
        named = element.get('file')
        if not named:
            problems.append(f'<license>{name}</license> has no file attribute')
        elif Path(named).is_absolute() or PKG_ROOT not in (PKG_ROOT / named).resolve().parents:
            problems.append(f'<license file="{named}">{name}</license>: not a path inside the '
                            'package (REP 149: relative to package.xml)')
        elif not (PKG_ROOT / named).is_file():
            problems.append(f'<license file="{named}">{name}</license>: {named} does not exist')
        elif named not in LICENSE_SHA256:
            problems.append(f'<license file="{named}">{name}</license>: {named} has no pinned '
                            'sha256 in this test')
        elif hashlib.sha256((PKG_ROOT / named).read_bytes()).hexdigest() != LICENSE_SHA256[named]:
            problems.append(f'<license file="{named}">{name}</license>: {named} is not the '
                            'expected license text (its sha256 differs from the pinned one)')
    assert not problems, 'package.xml:\n' + '\n'.join(problems)


def test_changelog_exists_and_parses():
    assert CHANGELOG.is_file(), 'CHANGELOG.rst is missing'
    versions = changelog_versions()
    assert versions, 'CHANGELOG.rst: catkin_pkg finds no version section in it'


def test_newest_changelog_version_is_the_package_version():
    assert PACKAGE_XML.is_file(), 'package.xml is missing'
    assert CHANGELOG.is_file(), 'CHANGELOG.rst is missing'
    declared = (ET.parse(PACKAGE_XML).getroot().findtext('version') or '').strip()
    versions = changelog_versions()
    assert versions, 'CHANGELOG.rst: catkin_pkg finds no version section in it'
    assert versions[0] == declared, (
        f'the highest CHANGELOG.rst version is {versions[0]}, but package.xml declares '
        f'{declared!r}; bump both together')


def test_changelog_sections_are_dated_and_strictly_descend_in_file_order():
    assert CHANGELOG.is_file(), 'CHANGELOG.rst is missing'
    headings = version_headings(CHANGELOG.read_text(encoding='utf-8'))
    problems = [] if headings else ['CHANGELOG.rst has no version heading']
    previous = None
    for lineno, heading in headings:
        match = DATED_HEADING.match(heading)
        if match is None:
            problems.append(f'CHANGELOG.rst:{lineno}: {heading!r} is not "X.Y.Z (YYYY-MM-DD)"')
            continue
        version = tuple(int(part) for part in match.group('version').split('.'))
        try:
            date = datetime.date.fromisoformat(match.group('date'))
        except ValueError:
            problems.append(f'CHANGELOG.rst:{lineno}: {heading!r} has no valid date')
            continue
        if previous is not None:
            if not version < previous[0]:
                problems.append(f'CHANGELOG.rst:{lineno}: {heading!r} does not descend from '
                                f'the section above it')
            if date > previous[1]:
                problems.append(f'CHANGELOG.rst:{lineno}: {heading!r} is dated after the '
                                f'section above it')
        previous = (version, date)
    assert not problems, '\n'.join(problems)


def test_changelog_is_warning_free_and_has_no_comment_blocks():
    assert CHANGELOG.is_file(), 'CHANGELOG.rst is missing'
    doctree, messages = rst_doctree(CHANGELOG.read_text(encoding='utf-8'))
    problems = rst_comments(doctree)
    problems += [f'CHANGELOG.rst: {message}' for message in messages]
    assert not problems, '\n'.join(problems)


def test_release_files_point_at_nothing_outside_the_repository():
    pattern = pointer_pattern()
    files = release_files()
    problems = [f'the example walk did not find {known}'
                for known in KNOWN_EXAMPLE_FILES if known not in files]
    for relative in files:
        path = PKG_ROOT / relative
        if not path.is_file():
            problems.append(f'{relative} is missing')
            continue
        for lineno, line in enumerate(path.read_text(encoding='utf-8').splitlines(), 1):
            for match in pattern.finditer(line):
                problems.append(f'{relative}:{lineno}: {match.group(0)!r} in {line.strip()!r}')
    assert not problems, (
        'release files point at documents a clone does not have:\n' + '\n'.join(problems))


def test_the_pointer_pattern_matches_each_kind():
    pattern = pointer_pattern()
    missed = [text for text in POINTER_POSITIVES if not pattern.search(text)]
    wrongly = [text for text in POINTER_NEGATIVES if pattern.search(text)]
    assert (missed, wrongly) == ([], []), (
        f'the pointer pattern misses {missed} and wrongly matches {wrongly}')


def test_release_documents_have_no_placeholders():
    missed = [text for text in PLACEHOLDER_SAMPLES if not PLACEHOLDER.search(text)]
    problems = [f'the placeholder pattern misses {text!r}' for text in missed]
    for relative in PLACEHOLDER_FILES:
        path = PKG_ROOT / relative
        if not path.is_file():
            problems.append(f'{relative} is missing')
            continue
        for lineno, line in enumerate(path.read_text(encoding='utf-8').splitlines(), 1):
            for match in PLACEHOLDER.finditer(line):
                problems.append(f'{relative}:{lineno}: {match.group(0)!r} in {line.strip()!r}')
    assert not problems, 'placeholders left in the release documents:\n' + '\n'.join(problems)
