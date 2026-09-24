"""
README.md agrees with the code, and its links and anchors resolve.

The README's reference sections are written from the code by hand, so nothing but this test keeps
them from drifting. Eight blocks are fenced by HTML-comment markers that render as nothing,
`<!-- reference:<name>:begin -->` ... `<!-- reference:<name>:end -->`, each on its own line:
  - hardware-parameters, joint-parameters, state-interfaces: pipe tables whose first column is
    one backticked name, compared as sets with the driver's own name tables; the hardware table's
    `default` column is compared with src/driver_defaults.hpp as well, and a known parameter
    whose `k<Name>` constant is missing there is a failure, not a skipped row;
  - status-bits: a pipe table of bits 0-7 whose `name the driver prints` column must spell the
    driver's bit names in bit order;
  - exit-codes: a bullet list of the tools' exit codes, compared with `enum class Exit`;
  - launch-arguments: a pipe table of the example launch file's arguments and their defaults;
  - tool-parameters: a pipe table, one row per tool, of the parameter names each tool accepts and
    the ranges of its id parameters;
  - log-lines: a bullet list of every driver and tool log line the README quotes, with `<name>`
    where the program substitutes text; each literal fragment of eight characters or more must
    occur in the package's own C++ string literals, and a line with no such fragment at all is
    a failure, because nothing of it would be checked.
A missing block is a failed case, never a skip. Every code-side extractor has a pinned count
(the constants below), so an extractor that silently matches nothing fails.

It also checks what a reader clicks and what the code cites: the release documents are linked,
every relative link and anchor in README.md and THIRD_PARTY.md resolves, no document cites a
driver file by line number, and the README headings that comments in the code point at exist.

No fixture reads a file: each case first asserts that the file it needs exists, and a case that
reads several files collects every problem into one list. The self-tests use synthetic input or
code files only. No motors, no port, no ROS graph, no git. The test reads the SOURCE tree through
its own __file__.
"""

import ast
from pathlib import Path
import re
import textwrap
from urllib.parse import unquote

PKG_ROOT = Path(__file__).resolve().parents[1]
README = PKG_ROOT / 'README.md'
THIRD_PARTY = PKG_ROOT / 'THIRD_PARTY.md'
CHANGELOG = PKG_ROOT / 'CHANGELOG.rst'

DRIVER_CPP = PKG_ROOT / 'src' / 'waveshare_servos.cpp'
UNITS_CPP = PKG_ROOT / 'src' / 'units.cpp'
DEFAULTS_HPP = PKG_ROOT / 'src' / 'driver_defaults.hpp'
TOOLS_HPP = PKG_ROOT / 'src' / 'servo_tools.hpp'
TOOL_PARAMS_CPP = PKG_ROOT / 'src' / 'tool_params.cpp'
LAUNCH_PY = PKG_ROOT / 'bringup' / 'launch' / 'example.launch.py'

# The 14 vendored SCServo files (THIRD_PARTY.md). A line citation into one of them is allowed:
# those lines move only when the library is refreshed from upstream.
VENDORED_NAMES = frozenset({
    'INST.h', 'SCS.h', 'SCSCL.h', 'SCSerial.h', 'SCServo.h', 'SMSBL.h', 'SMSCL.h', 'SMS_STS.h',
    'SCS.cpp', 'SCSCL.cpp', 'SCSerial.cpp', 'SMSBL.cpp', 'SMSCL.cpp', 'SMS_STS.cpp'})

# Pinned code-side counts. Each changes only together with the code it counts and the README.
EXPECTED_HARDWARE_PARAMS = 11
EXPECTED_JOINT_PARAMS = 7
EXPECTED_STATE_INTERFACES = 9
EXPECTED_STATUS_BITS = 8
EXPECTED_EXIT_CODES = (0, 1, 2, 3, 4, 5, 6, 7, 64, 70, 130)
EXPECTED_LAUNCH_ARGUMENTS = 4
EXPECTED_TOOLS = 4
EXPECTED_TOOL_PARAMETER_NAMES = 12
EXPECTED_DEFAULTS = 11
EXPECTED_ID_LIMITS = 3
# Comments in the code that send the reader to a README heading by quoting it.
EXPECTED_README_QUOTES = 6
# Lines in the README's reference:log-lines block. Change it together with the block.
EXPECTED_LOG_LINES = 65

# README headings that other files cite by their text; renaming one breaks the citation.
CITED_HEADINGS = (
    'Running the bus slower, or off the control thread',
    '`io_timeout_ms`, and how often a transaction fails',
    'Recovery after a servo is lost',
    'Hardware parameters',
)
RELEASE_DOCUMENTS = ('LICENSE', 'CHANGELOG.rst', 'THIRD_PARTY.md')

# The slug rule, pinned: text as the heading is written -> the anchor a link must use.
PINNED_SLUGS = (
    ('Recovery after a servo is lost', 'recovery-after-a-servo-is-lost'),
    ('`io_timeout_ms`, and how often a transaction fails',
     'io_timeout_ms-and-how-often-a-transaction-fails'),
    ('The one local fix: commit `d846222`, `src/SCSerial.cpp`',
     'the-one-local-fix-commit-d846222-srcscserialcpp'),
    ('**Bold** words and [a link](CHANGELOG.rst)', 'bold-words-and-a-link'),
)

# The id ranges the tool-parameters table must state, as names of driver_defaults.hpp constants.
ID_RANGES = {
    'start_id': ('kIdMin0', 'kIdMax'),
    'new_id': ('kIdMin', 'kIdMax'),
    'id': ('kIdMin0', 'kIdMax'),
}

# Log-line fragments shorter than this are not checked: they match too much to mean anything.
MIN_FRAGMENT = 8

FENCE = re.compile(r'^\s*(`{3,}|~{3,})')
HEADING = re.compile(r'^ {0,3}#{1,6}[ \t]+(.*?)(?:[ \t]+#+)?[ \t]*$')
CODE_SPAN = re.compile(r'(?<!`)(`+)(?!`)((?:(?!\n[ \t]*\n).)+?)(?<!`)\1(?!`)', re.S)
HTML_COMMENT = re.compile(r'<!--.*?-->', re.S)
LINK = re.compile(
    r'\]\(\s*<?((?:[^\s()<>]|\([^\s()<>]*\))+)>?(?:\s+(?:"[^"]*"|\'[^\']*\'))?\s*\)')
CITATION = re.compile(
    r'(?<![\w./-])(/?(?:[\w.-]+/)*[\w.-]+\.(?:cpp|hpp|h|py|sh|xacro|yaml|xml|txt|rst|md))'
    r':(\d+(?:-\d+)?)(?![\w-])')
README_QUOTE = re.compile(r'README(?:\'s)?,? "([^"]+)"')
CONVERSION = re.compile(
    r'%[-+ #0]*(?:\d+|\*)?(?:\.(?:\d+|\*))?(?:hh|h|ll|l|j|z|t|L)?[diouxXeEfFgGaAcspn%]')
# Joins literal groups and stands for a printf conversion: no fragment of a README line holds it.
SENTINEL = '\x00'
# A placeholder (`<name>`) or an elision (`...`, with the spaces around it) in a quoted log line:
# the text on either side is checked separately.
GAP = re.compile(r'<[^<>]+>|\s*(?:\.\.\.|\u2026)\s*')
ESCAPES = {'n': '\n', 't': '\t', 'r': '\r', '0': '\0', 'a': '\a', 'b': '\b', 'f': '\f',
           'v': '\v'}


# ---------------------------------------------------------------------------------------------
# Markdown helpers


def outside_fences(markdown_text):
    """Return the lines of the text with every fenced code block's lines blanked."""
    lines = markdown_text.splitlines()
    fence = None
    for index, line in enumerate(lines):
        match = FENCE.match(line)
        if fence is None:
            if match:
                fence = match.group(1)
                lines[index] = ''
        else:
            lines[index] = ''
            if (match and match.group(1)[0] == fence[0] and len(match.group(1)) >= len(fence)
                    and line.strip() == match.group(1)):
                fence = None
    return lines


def headings(markdown_text):
    """Return the text of every ATX heading outside fenced code blocks, in document order."""
    return [match.group(1).strip() for match in map(HEADING.match, outside_fences(markdown_text))
            if match]


def slug(heading_text):
    """Return the anchor GitHub gives a heading with this text (the pinned rule)."""
    text = re.sub(r'\[([^\]]*)\]\([^)]*\)', r'\1', heading_text)
    text = text.replace('`', '').replace('*', '').strip().lower()
    return re.sub(r'[^\w\- ]', '', text).replace(' ', '-')


def heading_slugs(markdown_text):
    """Return the anchor of every heading, in order; a repeated one gets -1, -2, ... appended."""
    seen = {}
    slugs = []
    for heading in headings(markdown_text):
        base = slug(heading)
        slugs.append(base if base not in seen else f'{base}-{seen[base]}')
        seen[base] = seen.get(base, 0) + 1
    return slugs


def links(markdown_text):
    """Return (line number, target) of every inline link outside code and HTML comments."""
    def blank(match):
        return re.sub(r'[^\n]', ' ', match.group(0))
    text = '\n'.join(outside_fences(markdown_text))
    text = CODE_SPAN.sub(blank, HTML_COMMENT.sub(blank, text))
    return [(text.count('\n', 0, match.start()) + 1, match.group(1))
            for match in LINK.finditer(text)]


def reference_block(markdown_text, name):
    """Return the lines between the reference:<name> markers, or None if there is no block."""
    lines = [line.strip() for line in markdown_text.splitlines()]
    begins = [i for i, line in enumerate(lines) if line == f'<!-- reference:{name}:begin -->']
    ends = [i for i, line in enumerate(lines) if line == f'<!-- reference:{name}:end -->']
    if not begins and not ends:
        return None
    assert len(begins) == 1 and len(ends) == 1 and begins[0] < ends[0], (
        f'README.md: the reference:{name} markers are not one begin/end pair (begin on lines '
        f'{[i + 1 for i in begins]}, end on lines {[i + 1 for i in ends]})')
    return markdown_text.splitlines()[begins[0] + 1:ends[0]]


def pipe_table(lines):
    """Return (header cells, rows of cells) of the first pipe table in these lines."""
    def cells(line):
        inner = line.strip()
        inner = inner[1:] if inner.startswith('|') else inner
        inner = inner[:-1] if inner.endswith('|') and not inner.endswith('\\|') else inner
        return [cell.strip().replace('\\|', '|') for cell in re.split(r'(?<!\\)\|', inner)]

    def is_separator(line):
        return '|' in line and all(re.fullmatch(r':?-+:?', cell) for cell in cells(line))

    for index in range(len(lines) - 1):
        if '|' in lines[index] and is_separator(lines[index + 1]):
            rows = []
            for line in lines[index + 2:]:
                if '|' not in line:
                    break
                rows.append(cells(line))
            return cells(lines[index]), rows
    return [], []


def backticked(text):
    """Return every backticked token in this text, without the backticks."""
    return [match.group(2).strip() for match in CODE_SPAN.finditer(text)]


def line_citations(text):
    """Return every path:N or path:N-M citation of a source or document file in this text."""
    return [f'{match.group(1)}:{match.group(2)}' for match in CITATION.finditer(text)]


# ---------------------------------------------------------------------------------------------
# Code-side extractors


def code_of(path):
    """Return a C++ file's text with its comments removed (string literals kept)."""
    return ''.join(raw for kind, raw, _ in cpp_pieces(path.read_text(encoding='utf-8'))
                   if kind != 'comment')


def initializer(path, name):
    """Return the text of the brace initializer of the C++ variable `name`."""
    match = re.search(re.escape(name) + r'\b[^=;{]*=\s*\{(.*?)\};', code_of(path), re.S)
    assert match is not None, f'{path.name} has no initializer for {name}'
    return match.group(1)


def hardware_params():
    """Return the names in kKnownHardwareParams, in order."""
    return cpp_string_literals(initializer(DRIVER_CPP, 'kKnownHardwareParams'))


def joint_params():
    """Return the names in kKnownJointParams, in order."""
    return cpp_string_literals(initializer(DRIVER_CPP, 'kKnownJointParams'))


def state_interfaces():
    """Return kStateKindNames as interface names, in order."""
    body = initializer(DRIVER_CPP, 'kStateKindNames')
    return [name.lower() for name in re.findall(r'\bHW_IF_([A-Z0-9_]+)\b', body)]


def status_bits():
    """Return kStatusBitNames, in bit order."""
    return cpp_string_literals(initializer(UNITS_CPP, 'kStatusBitNames'))


def exit_codes():
    """Return the values of enum class Exit, in order."""
    match = re.search(r'enum class Exit\s*:\s*int\s*\{(.*?)\};', code_of(TOOLS_HPP), re.S)
    assert match is not None, f'{TOOLS_HPP.name} has no enum class Exit'
    return [int(value) for value in re.findall(r'\bk\w+\s*=\s*(\d+)', match.group(1))]


def launch_arguments():
    """Return {name: default_value} of the example launch file's DeclareLaunchArgument calls."""
    arguments = {}
    for node in ast.walk(ast.parse(LAUNCH_PY.read_text(encoding='utf-8'))):
        if not isinstance(node, ast.Call):
            continue
        function = node.func
        called = function.id if isinstance(function, ast.Name) else getattr(function, 'attr', '')
        if called != 'DeclareLaunchArgument':
            continue
        keywords = {keyword.arg: keyword.value for keyword in node.keywords}
        name = node.args[0] if node.args else keywords.get('name')
        default = keywords.get('default_value')
        arguments[getattr(name, 'value', None)] = getattr(default, 'value', None)
    return arguments


def tool_parameters():
    """Return {tool: accepted parameter names} from accepted_names() in src/tool_params.cpp."""
    match = re.search(r'accepted_names\(Tool tool\)\s*\{(.*?)\n\}', code_of(TOOL_PARAMS_CPP),
                      re.S)
    assert match is not None, f'{TOOL_PARAMS_CPP.name} has no accepted_names(Tool tool)'
    return {re.sub(r'(?<!^)(?=[A-Z])', '_', name).lower(): cpp_string_literals(names)
            for name, names in re.findall(r'case Tool::k(\w+):\s*return\s*\{([^}]*)\};',
                                          match.group(1))}


def namespace_body(path, name):
    match = re.search(r'namespace ' + name + r'\s*\{(.*?)\}', code_of(path), re.S)
    assert match is not None, f'{path.name} has no namespace {name}'
    return match.group(1)


def driver_defaults():
    """Return {constant name: value} of the defaults namespace of src/driver_defaults.hpp."""
    values = {}
    body = namespace_body(DEFAULTS_HPP, 'defaults')
    for name, value in re.findall(r'constexpr\s[^=;]*?\b(k[A-Z]\w*)\s*=\s*([^;]+);', body):
        value = value.strip()
        if value.startswith('"'):
            values[name] = ''.join(cpp_string_literals(value))
        elif value in ('true', 'false'):
            values[name] = value == 'true'
        elif re.fullmatch(r'[-+]?\d+', value):
            values[name] = int(value)
        else:
            values[name] = float(value.rstrip('fF'))
    return values


def id_limits():
    """Return {constant name: value} of the limits namespace of src/driver_defaults.hpp."""
    body = namespace_body(DEFAULTS_HPP, 'limits')
    return {name: int(value) for name, value in re.findall(r'\b(k\w+)\s*=\s*(\d+)', body)}


def readme_quotes():
    """Return (file, quote) for every comment in the code that quotes a README heading."""
    quotes = []
    for directory in ('src', 'include', 'test', 'description', 'bringup'):
        for path in sorted((PKG_ROOT / directory).rglob('*')):
            relative = path.relative_to(PKG_ROOT)
            hidden = any(part.startswith('.') or part == '__pycache__' for part in relative.parts)
            if hidden or not path.is_file() or path.name in VENDORED_NAMES:
                continue
            lines = path.read_text(encoding='utf-8', errors='replace').splitlines()
            text = ' '.join(re.sub(r'^\s*(?://+|#+|<!--|-->)', '', line) for line in lines)
            quotes += [(relative.as_posix(), quote)
                       for quote in README_QUOTE.findall(' '.join(text.split()))]
    return quotes


# ---------------------------------------------------------------------------------------------
# Log-line matching


def unescape(body):
    """Return the value of a C++ string literal's body."""
    def value(match):
        escape = match.group(1)
        if escape[0] == 'x':
            return chr(int(escape[1:], 16))
        if escape[0] in '01234567' and len(escape) > 1:
            return chr(int(escape, 8))
        return ESCAPES.get(escape, escape)
    return re.sub(r'\\(x[0-9a-fA-F]+|[0-7]{1,3}|.)', value, body, flags=re.S)


def cpp_pieces(source):
    """Split C++ source into (kind, raw text, value): 'code', 'comment', 'string' and 'char'."""
    pieces = []
    start = index = 0

    def flush(end):
        if end > start:
            pieces.append(('code', source[start:end], None))

    while index < len(source):
        if source.startswith('//', index):
            end = source.find('\n', index)
            end = len(source) if end == -1 else end
            flush(index)
            pieces.append(('comment', source[index:end], None))
        elif source.startswith('/*', index):
            end = source.find('*/', index + 2)
            end = len(source) if end == -1 else end + 2
            flush(index)
            pieces.append(('comment', source[index:end], None))
        elif source[index] == '"':
            raw = re.search(r'(?:^|\W)(?:u8|u|U|L)?R$', source[max(0, index - 3):index])
            if raw:
                delimiter = source[index + 1:source.index('(', index)]
                close = source.index(')' + delimiter + '"', index)
                value = source[index + 2 + len(delimiter):close]
                end = close + len(delimiter) + 2
            else:
                end = index + 1
                while end < len(source) and source[end] != '"':
                    end += 2 if source[end] == '\\' else 1
                value = unescape(source[index + 1:end])
                end += 1
            flush(index)
            pieces.append(('string', source[index:end], value))
        elif source[index] == "'" and not (index and source[index - 1].isalnum()):
            end = index + 1
            while end < len(source) and source[end] != "'":
                end += 2 if source[end] == '\\' else 1
            end += 1
            flush(index)
            pieces.append(('char', source[index:end], None))
        else:
            index += 1
            continue
        start = index = end
    flush(len(source))
    return pieces


def cpp_string_literals(source):
    """Return the string literals of C++ source, adjacent literals joined as the compiler does."""
    literals = []
    current = None  # the literal being joined, until something other than a literal follows it
    macro = False  # a PRIu64-style macro between two literals
    for kind, raw, value in cpp_pieces(source):
        if kind == 'string':
            # PRIu64 and its kin expand to a literal that completes a printf conversion.
            current = value if current is None else current + ('d' if macro else '') + value
            macro = False
        elif kind == 'comment' or current is None or (kind == 'code' and not raw.strip()):
            continue
        elif kind == 'code' and not macro and re.fullmatch(r'\s*(?:PRI|SCN)\w+\s*', raw):
            macro = True
        else:
            literals.append(current)
            current, macro = None, False
    if current is not None:
        literals.append(current)
    return literals


def literal_corpus(literals):
    """Return one searchable text of these literals, printf conversions made unmatchable."""
    def conversion(match):
        return '%' if match.group(0) == '%%' else SENTINEL
    return SENTINEL.join(CONVERSION.sub(conversion, literal) for literal in literals)


def fragments(line):
    """Return the literal fragments of a quoted log line long enough to be checked."""
    return [fragment for fragment in GAP.split(line) if len(fragment) >= MIN_FRAGMENT]


def missing_fragments(line, corpus):
    """Return the literal fragments of a quoted log line that do not occur in the corpus."""
    return [fragment for fragment in fragments(line) if fragment not in corpus]


# ---------------------------------------------------------------------------------------------
# Glue shared by the cases


def reference_table(markdown_text, name, required_headers, problems):
    """Return the rows of a reference table as {header: cell}; append every problem found."""
    lines = reference_block(markdown_text, name)
    if lines is None:
        problems.append(f'README.md has no reference:{name} block')
        return []
    header, rows = pipe_table(lines)
    if not header:
        problems.append(f'the reference:{name} block holds no pipe table')
        return []
    keys = [cell.strip().lower() for cell in header]
    for required in required_headers:
        if required not in keys:
            problems.append(f'the reference:{name} table has no {required!r} column: {header}')
    return [dict(zip(keys, row)) for row in rows]


def named_rows(rows, name, problems):
    """Return (row, name) for each row whose first cell is one backticked name; report others."""
    named = []
    for row in rows:
        cell = next(iter(row.values()), '')
        tokens = backticked(cell)
        if len(tokens) != 1:
            problems.append(f'reference:{name}: the first cell {cell!r} does not hold exactly one '
                            'backticked name')
            continue
        named.append((row, tokens[0]))
    return named


def compare_names(readme_names, code_names, name, source, problems):
    """Append every difference between the README's names and the code's."""
    for extra in sorted(set(readme_names) - set(code_names)):
        problems.append(f'reference:{name} lists {extra!r}, which {source} does not have')
    for missing in sorted(set(code_names) - set(readme_names)):
        problems.append(f'reference:{name} lacks {missing!r}, which {source} has')
    for repeated in sorted({n for n in readme_names if readme_names.count(n) > 1}):
        problems.append(f'reference:{name} lists {repeated!r} more than once')


def readme_text():
    assert README.is_file(), 'README.md is missing'
    return README.read_text(encoding='utf-8')


def link_path(target):
    """Return a link target's path part: no #anchor or ?query, percent-decoded, no './'."""
    path = unquote(target.split('#', 1)[0].split('?', 1)[0])
    while path.startswith('./'):
        path = path[2:]
    return path


def is_external(target):
    return re.match(r'^[A-Za-z][A-Za-z0-9+.-]*:', target) is not None


def camel(name):
    return ''.join(part.capitalize() for part in name.split('_'))


def same_default(readme_token, code_value):
    """Compare a README default with a driver_defaults.hpp value: numbers numerically."""
    if isinstance(code_value, bool):
        return readme_token == ('true' if code_value else 'false')
    if isinstance(code_value, (int, float)):
        try:
            return float(readme_token) == float(code_value)
        except ValueError:
            return False
    return readme_token.strip('"') == code_value


# ---------------------------------------------------------------------------------------------
# M1-M5: links, anchors, slugs


def test_readme_links_the_release_documents():
    targets = {link_path(target) for _, target in links(readme_text())}
    missing = [document for document in RELEASE_DOCUMENTS if document not in targets]
    assert missing == [], f'README.md does not link {missing}'


def test_relative_links_resolve_to_files_in_the_package():
    problems = []
    for document in (README, THIRD_PARTY):
        if not document.is_file():
            problems.append(f'{document.name} is missing')
            continue
        for lineno, target in links(document.read_text(encoding='utf-8')):
            if is_external(target) or target.startswith('#'):
                continue
            path = link_path(target)
            resolved = (document.parent / path.lstrip('/')).resolve()
            inside = resolved == PKG_ROOT or PKG_ROOT in resolved.parents
            if not path or not inside or not resolved.exists():
                problems.append(f'{document.name}:{lineno}: ({target}) is not a file in the '
                                'package')
    assert not problems, 'broken relative links:\n' + '\n'.join(problems)


def test_in_page_and_cross_file_anchors_resolve():
    problems = []
    texts = {}
    for document in (README, THIRD_PARTY):
        if document.is_file():
            texts[document] = document.read_text(encoding='utf-8')
        else:
            problems.append(f'{document.name} is missing')
    for document, text in texts.items():
        for lineno, target in links(text):
            if is_external(target) or '#' not in target:
                continue
            path, anchor = target.split('#', 1)
            linked = document if not link_path(path) else (
                document.parent / link_path(path).lstrip('/')).resolve()
            if linked.suffix.lower() != '.md' or not linked.is_file():
                continue  # not a markdown page, or missing: the relative-link case reports it
            if linked not in texts:
                texts[linked] = linked.read_text(encoding='utf-8')
            if unquote(anchor) not in heading_slugs(texts[linked]):
                problems.append(f'{document.name}:{lineno}: ({target}): {linked.name} has no '
                                f'heading with the anchor #{anchor}')
    assert not problems, 'broken anchors:\n' + '\n'.join(problems)


def test_slugger_matches_the_pinned_slugs():
    wrong = [(text, slug(text), want) for text, want in PINNED_SLUGS if slug(text) != want]
    assert wrong == [], f'(heading, slug, pinned slug): {wrong}'
    document = textwrap.dedent("""\
        # Title

        ## Setup

        ```bash
        # a comment, not a heading
        ```

        ### Setup ###

        ## Setup
        """)
    assert heading_slugs(document) == ['title', 'setup', 'setup-1', 'setup-2']


def test_link_extractor_ignores_code_and_finds_links():
    document = textwrap.dedent("""\
        See [the changelog](CHANGELOG.rst) and `[not a link](in-a-code-span.md)`.

        ```markdown
        [not a link either](in-a-fence.md)
        ```

        <!-- [not rendered](in-a-comment.md) -->
        Then [recovery](#recovery-after-a-servo-is-lost), ``[`also` code](span.md)``.
        """)
    assert links(document) == [(1, 'CHANGELOG.rst'), (8, '#recovery-after-a-servo-is-lost')]


# ---------------------------------------------------------------------------------------------
# M6-M12, M15: the reference blocks against the code


def test_hardware_parameter_table_matches_the_driver():
    problems = []
    rows = reference_table(readme_text(), 'hardware-parameters', ('default',), problems)
    named = named_rows(rows, 'hardware-parameters', problems)
    known = hardware_params()
    compare_names([name for _, name in named], known, 'hardware-parameters',
                  'kKnownHardwareParams', problems)
    defaults = driver_defaults()
    for row, name in named:
        if name not in known:
            continue  # reported by compare_names above
        constant = 'k' + camel(name)
        if constant not in defaults:
            problems.append(f'reference:hardware-parameters: {name} has no default {constant} '
                            'in driver_defaults.hpp, so its default cannot be compared')
            continue
        tokens = backticked(row.get('default', ''))
        if not tokens:
            problems.append(f'reference:hardware-parameters: {name} has no backticked default')
        elif not same_default(tokens[0], defaults[constant]):
            problems.append(f'reference:hardware-parameters: {name} defaults to {tokens[0]!r} '
                            f'in the README but {defaults[constant]!r} in the driver ({constant})')
    assert not problems, '\n'.join(problems)


def test_joint_parameter_table_matches_the_driver():
    problems = []
    rows = reference_table(readme_text(), 'joint-parameters', (), problems)
    names = [name for _, name in named_rows(rows, 'joint-parameters', problems)]
    compare_names(names, joint_params(), 'joint-parameters', 'kKnownJointParams', problems)
    assert not problems, '\n'.join(problems)


def test_state_interface_table_matches_the_driver():
    problems = []
    rows = reference_table(readme_text(), 'state-interfaces', (), problems)
    names = [name for _, name in named_rows(rows, 'state-interfaces', problems)]
    compare_names(names, state_interfaces(), 'state-interfaces', 'kStateKindNames', problems)
    assert not problems, '\n'.join(problems)


def test_status_bit_table_matches_the_driver():
    problems = []
    column = 'name the driver prints'
    rows = reference_table(readme_text(), 'status-bits', (column,), problems)
    by_bit = {}
    for row in rows:
        cell = next(iter(row.values()), '')
        bit = cell.strip().strip('`').strip()
        names = backticked(row.get(column, ''))
        if not re.fullmatch(r'[0-7]', bit) or bit in by_bit or not names:
            problems.append(f'reference:status-bits: malformed or repeated row {row}')
            continue
        by_bit[bit] = names[0]
    readme_names = [by_bit.get(str(bit)) for bit in range(8)]
    code_names = status_bits()
    if readme_names != code_names:
        problems.append(f'reference:status-bits names bits 0-7 {readme_names}, but '
                        f'kStatusBitNames is {code_names}')
    assert not problems, '\n'.join(problems)


def test_exit_code_list_matches_the_tools():
    problems = []
    lines = reference_block(readme_text(), 'exit-codes')
    if lines is None:
        problems.append('README.md has no reference:exit-codes block')
        lines = []
    codes = []
    for line in lines:
        if not line.startswith('-'):
            continue  # a blank line or the continuation of a wrapped bullet
        match = re.match(r'^- `(\d+)`(?: \(scan only\))?: \S', line)
        if match is None:
            problems.append(f'reference:exit-codes: malformed line {line!r}')
            continue
        codes.append(int(match.group(1)))
    for repeated in sorted({code for code in codes if codes.count(code) > 1}):
        problems.append(f'reference:exit-codes lists {repeated} more than once')
    code_values = exit_codes()
    if sorted(set(codes)) != sorted(code_values):
        problems.append(f'reference:exit-codes lists {sorted(set(codes))}, but enum class Exit '
                        f'has {sorted(code_values)}')
    assert not problems, '\n'.join(problems)


def test_launch_argument_table_matches_the_launch_file():
    problems = []
    rows = reference_table(readme_text(), 'launch-arguments', ('default',), problems)
    named = named_rows(rows, 'launch-arguments', problems)
    code = launch_arguments()
    compare_names([name for _, name in named], list(code), 'launch-arguments',
                  'example.launch.py', problems)
    for row, name in named:
        if name not in code:
            continue
        tokens = backticked(row.get('default', ''))
        if not tokens or tokens[0] != code[name]:
            problems.append(f'reference:launch-arguments: {name} defaults to '
                            f'{tokens[:1]} in the README but {code[name]!r} in the launch file')
    assert not problems, '\n'.join(problems)


def test_code_extractors_find_the_pinned_counts():
    tools = tool_parameters()
    found = {
        'hardware parameters': len(hardware_params()),
        'joint parameters': len(joint_params()),
        'state interfaces': len(state_interfaces()),
        'status bits': len(status_bits()),
        'exit codes': tuple(exit_codes()),
        'launch arguments': len(launch_arguments()),
        'tools': len(tools),
        'tool parameter names': sum(len(names) for names in tools.values()),
        'driver defaults': len(driver_defaults()),
        'id limits': len(id_limits()),
    }
    pinned = {
        'hardware parameters': EXPECTED_HARDWARE_PARAMS,
        'joint parameters': EXPECTED_JOINT_PARAMS,
        'state interfaces': EXPECTED_STATE_INTERFACES,
        'status bits': EXPECTED_STATUS_BITS,
        'exit codes': EXPECTED_EXIT_CODES,
        'launch arguments': EXPECTED_LAUNCH_ARGUMENTS,
        'tools': EXPECTED_TOOLS,
        'tool parameter names': EXPECTED_TOOL_PARAMETER_NAMES,
        'driver defaults': EXPECTED_DEFAULTS,
        'id limits': EXPECTED_ID_LIMITS,
    }
    wrong = {key: (found[key], pinned[key]) for key in pinned if found[key] != pinned[key]}
    assert wrong == {}, f'(found, pinned) per extractor: {wrong}'


def test_tool_parameter_table_matches_the_tools():
    problems = []
    rows = reference_table(readme_text(), 'tool-parameters', ('tool', 'parameters', 'ranges'),
                           problems)
    code = tool_parameters()
    limits = id_limits()
    every_name = {name for names in code.values() for name in names}
    tools = []
    for row in rows:
        tool_tokens = backticked(row.get('tool', ''))
        if len(tool_tokens) != 1:
            problems.append(f'reference:tool-parameters: the tool cell of {row} does not hold '
                            'exactly one backticked name')
            continue
        tool = tool_tokens[0]
        tools.append(tool)
        if tool not in code:
            continue  # reported by compare_names below
        listed = backticked(row.get('parameters', ''))
        if sorted(listed) != sorted(code[tool]):
            problems.append(f'reference:tool-parameters: {tool} lists {sorted(listed)}, but '
                            f'accepted_names() gives {sorted(code[tool])}')
        # Each id parameter's range: the first a..b after its backticked name, before the next
        # backticked parameter name, in the ranges cell.
        pieces = re.split(r'(`[^`]+`)', row.get('ranges', ''))
        stated = {}
        for index, piece in enumerate(pieces):
            if piece.strip('`') not in ID_RANGES or not piece.startswith('`'):
                continue
            tail = ''
            for following in pieces[index + 1:]:
                if following.startswith('`') and following.strip('`') in every_name:
                    break
                tail += following
            found = re.search(r'(\d+)\s*\.\.\s*(\d+)', tail)
            if found:
                stated[piece.strip('`')] = (int(found.group(1)), int(found.group(2)))
        for name in sorted(set(code[tool]) & set(ID_RANGES)):
            want = tuple(limits.get(constant) for constant in ID_RANGES[name])
            if name not in stated:
                problems.append(f'reference:tool-parameters: {tool} states no a..b range for '
                                f'{name} (expected {want[0]}..{want[1]})')
            elif stated[name] != want:
                problems.append(f'reference:tool-parameters: {tool} gives {name} the range '
                                f'{stated[name][0]}..{stated[name][1]}, but driver_defaults.hpp '
                                f'gives {want[0]}..{want[1]}')
    compare_names(tools, list(code), 'tool-parameters', 'accepted_names()', problems)
    assert not problems, '\n'.join(problems)


# ---------------------------------------------------------------------------------------------
# M13, M14: line citations, and the headings the code cites


def test_no_line_number_citations_outside_the_vendored_files():
    sample = 'see src/servo_bus.cpp:123, `CMakeLists.txt:46-49` and src/SCS.cpp:279 at 12:00'
    extracted = line_citations(sample)
    problems = []
    if extracted != ['src/servo_bus.cpp:123', 'CMakeLists.txt:46-49', 'src/SCS.cpp:279']:
        problems.append(f'the citation extractor finds {extracted} in {sample!r}')
    for document in (README, THIRD_PARTY, CHANGELOG):
        if not document.is_file():
            problems.append(f'{document.name} is missing')
            continue
        text = document.read_text(encoding='utf-8')
        for lineno, line in enumerate(text.splitlines(), 1):
            for citation in line_citations(line):
                if citation.split(':')[0].rsplit('/', 1)[-1] not in VENDORED_NAMES:
                    problems.append(f'{document.name}:{lineno}: {citation}')
    assert not problems, (
        'line-number citations of non-vendored files (name the function, class or log text '
        'instead):\n' + '\n'.join(problems))


def test_headings_the_code_cites_exist():
    readme_headings = headings(readme_text())
    problems = [f'README.md has no heading {heading!r}'
                for heading in CITED_HEADINGS if heading not in readme_headings]
    quotes = readme_quotes()
    if len(quotes) != EXPECTED_README_QUOTES:
        problems.append(f'the code quotes README headings {len(quotes)} times, expected '
                        f'{EXPECTED_README_QUOTES}: {quotes}')
    plain = readme_headings + [heading.replace('`', '') for heading in readme_headings]
    for source, quote in quotes:
        if not any(quote in heading for heading in plain):
            problems.append(f'{source} quotes {quote!r}, which is part of no README heading')
    assert not problems, '\n'.join(problems)


# ---------------------------------------------------------------------------------------------
# M16, M17: the quoted log lines


def test_quoted_log_lines_appear_in_the_sources():
    problems = []
    lines = reference_block(readme_text(), 'log-lines')
    if lines is None:
        problems.append('README.md has no reference:log-lines block')
        lines = []
    quoted = []
    for line in lines:
        if not line.strip():
            continue
        match = re.match(r'^- (`+) ?(.+?) ?\1\s*$', line)
        if match is None:
            problems.append(f'reference:log-lines: malformed line {line!r}')
            continue
        quoted.append(match.group(2))
    if len(quoted) != EXPECTED_LOG_LINES:
        problems.append(f'reference:log-lines holds {len(quoted)} lines, expected '
                        f'{EXPECTED_LOG_LINES}')
    sources = sorted(p for d in ('src', 'include') for suffix in ('*.cpp', '*.hpp')
                     for p in (PKG_ROOT / d).glob(suffix) if p.name not in VENDORED_NAMES)
    literals = []
    for source in sources:
        literals += cpp_string_literals(source.read_text(encoding='utf-8'))
    corpus = literal_corpus(literals)
    for line in quoted:
        if not fragments(line):
            problems.append(f'reference:log-lines: {line!r} has no literal fragment of '
                            f'{MIN_FRAGMENT} characters or more, so nothing of it is checked')
        for fragment in missing_fragments(line, corpus):
            problems.append(f'reference:log-lines: {fragment!r} (from {line!r}) occurs in no '
                            'string literal of the driver or the tools')
    assert not problems, '\n'.join(problems)


def test_log_line_matcher_joins_literals_and_splits_placeholders():
    source = textwrap.dedent("""\
        RCLCPP_ERROR(
          get_logger(), "motor id '%d' stopped answering after %d attempts; dropping it from "
          "the read cycle", id, fails);  // "a comment's quote is no literal"
        const std::string text = "port '" + port + "' is busy\\n";
        /* "nor is a block comment's" */ const char c = '"';
        """)
    literals = cpp_string_literals(source)
    assert literals == [
        "motor id '%d' stopped answering after %d attempts; dropping it from the read cycle",
        "port '", "' is busy\n"], f'literals: {literals}'
    corpus = literal_corpus(literals)
    true_line = ("motor id '<N>' stopped answering after <n> attempts; dropping it from the read "
                 'cycle')
    assert missing_fragments(true_line, corpus) == []
    elided = "motor id '<N>' stopped answering ... the read cycle"
    assert missing_fragments(elided, corpus) == []
    reworded = "motor id '<N>' stopped responding after <n> attempts"
    assert missing_fragments(reworded, corpus) == ["' stopped responding after "]
    spanning = "motor id '3' stopped answering after <n> attempts"
    assert missing_fragments(spanning, corpus) == ["motor id '3' stopped answering after "]
    comment = "a comment's quote is no literal"
    assert missing_fragments(comment, corpus) == [comment]
