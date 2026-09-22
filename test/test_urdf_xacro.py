"""
The shipped example description renders, with and without use_mock_hardware.

description/urdf/example.urdf.xacro plus description/ros2_control/example.ros2_control.xacro are
what `ros2 launch waveshare_servos example.launch.py` renders at startup -- the launch file builds
the same argv at bringup/launch/example.launch.py:61-77 -- and they are the first two files a new
user copies. Nothing else in the suite renders them, so until this file existed a typo in the
macro's argument plumbing reached the bench before it reached a test.

The property worth the most here is the mock swap guarded by the list lookup at
description/ros2_control/example.ros2_control.xacro:12-13. With use_mock_hardware true the
<hardware> block must carry mock_components/GenericSystem and NO <param> children at all, so that
a mock run has no serial path anywhere in the description for anything to open; with it false it
must carry waveshare_servos/WaveshareServos plus the port and baudrate that were passed in. An
unrecognised spelling must abort the render rather than fall back to either branch - see
test_an_unrecognised_use_mock_hardware_aborts_the_render for why that is a safety property and not
pedantry.

Everything in this file is pure text work: rendering a xacro opens no serial port, and no
assertion here needs a servo attached.
"""

import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro

# $(find waveshare_servos) inside the URDF resolves through get_package_share_directory
# (xacro/substitution_args.py:138-140), so the include would come from the share directory even if
# this test rendered the source copy. Rendering the share copy instead also proves that
# `install(DIRECTORY description/ ...)` (CMakeLists.txt:104-107) shipped BOTH files.
#
# What this test cannot tell you is whether the share copy is a real file or a symlink. The
# documented loop builds with --symlink-install (CLAUDE.md), so on this bench share/.../urdf/
# example.urdf.xacro is a symlink back into src and rendering it is rendering the source; a plain
# `colcon build` would make it a copy and this test would not notice the difference. Nothing here
# exercises the copy path, and no test in this package does -- worth knowing if an install()
# regression ever has to be chased, because a rule that ships the wrong tree can only show up in a
# copy install.
# The property that does hold either way, and the one actually worth having, is simpler: a
# description file the install() rules forgot to name has no path under share at all, symlink or
# copy, and every render in this file then fails on a missing file.
EXAMPLE_URDF_XACRO = (get_package_share_directory('waveshare_servos') +
                      '/description/urdf/example.urdf.xacro')

XACRO_NS = '{http://www.ros.org/wiki/xacro}'

REAL_PLUGIN = 'waveshare_servos/WaveshareServos'
MOCK_PLUGIN = 'mock_components/GenericSystem'

# The four joints of the example, ids 1-4, matching the four servos on the reference bench
# (description/ros2_control/example.ros2_control.xacro:72-143).
JOINT_NAMES = ('joint1', 'joint2', 'joint3', 'joint4')

# The exact spellings the list lookup at example.ros2_control.xacro:13 accepts - it indexes
# ['false', '0', 'true', '1'] with str(use_mock_hardware).strip().lower(), so case and surrounding
# space are the only freedom, and both are exercised below.
TRUE_SPELLINGS = ('true', 'True', 'TRUE', '1', ' true ')
FALSE_SPELLINGS = ('false', 'False', 'FALSE', '0', ' FALSE ')


def render(**mappings):
    """
    Render the example with these xacro arguments and return its <robot> root.

    xacro.process_file is the entry point that RAISES xacro.XacroException on a bad argument
    (xacro/__init__.py:1105-1125). xacro.process() must not be used here: it wraps _process
    (:1177-1185), whose except Exception branch calls sys.exit(2) at :1165 -- and :1152 does the
    same for a parse error -- so the abort case would arrive as a bare SystemExit carrying no
    message.

    The document is re-parsed with ElementTree because ElementTree drops comments and minidom does
    not. The render contains a commented-out `<ros2_control ... rw_rate="50" is_async="true">`
    example (example.ros2_control.xacro:28) and several paragraphs of prose that mention <param>,
    so a regex or a substring test over the rendered text matches a comment and silently passes.
    """
    return ET.fromstring(xacro.process_file(EXAMPLE_URDF_XACRO, mappings=mappings).toxml())


def hardware_of(root):
    """Return the <hardware> child of the single live <ros2_control> block of a render."""
    blocks = root.findall('ros2_control')
    assert len(blocks) == 1, (
        f'expected exactly one <ros2_control> block in the render, found {len(blocks)}: '
        f'{[b.get("name") for b in blocks]}')
    return blocks[0].find('hardware')


def params_of(hardware):
    """Return the <param> children of a <hardware> block as a name -> text dict."""
    return {p.get('name'): p.text for p in hardware.findall('param')}


def test_the_example_declares_exactly_the_three_documented_arguments():
    """
    The launch file passes port, baudrate and use_mock_hardware and nothing else.

    xacro accepts an undeclared mapping silently and renders with exit 0, so a renamed or dropped
    <xacro:arg> does not fail any render - the argument simply stops having an effect and the
    default applies. That is precisely how use_mock_hardware could quietly stop working, hence the
    declaration itself is pinned here rather than only its behaviour. gui is deliberately absent:
    it is a launch-file argument only (bringup/launch/example.launch.py:45-49).
    """
    declared = [(arg.get('name'), arg.get('default'))
                for arg in ET.parse(EXAMPLE_URDF_XACRO).getroot().findall(XACRO_NS + 'arg')]
    assert declared == [('port', '/dev/ttyACM0'),
                        ('baudrate', '1000000'),
                        ('use_mock_hardware', 'false')]


def test_the_default_render_selects_the_real_driver_on_the_bench_port():
    """
    With no arguments at all the example is the real driver on /dev/ttyACM0 at 1 Mbaud.

    This is what `ros2 launch waveshare_servos example.launch.py` does with no arguments, so the
    defaults are part of the example's contract and not an implementation detail.
    """
    hardware = hardware_of(render())
    assert hardware.find('plugin').text == REAL_PLUGIN
    assert params_of(hardware) == {'port': '/dev/ttyACM0', 'baudrate': '1000000'}


@pytest.mark.parametrize('spelling', FALSE_SPELLINGS)
def test_a_false_spelling_selects_the_real_driver_and_forwards_port_and_baudrate(spelling):
    """
    Every accepted false-ish spelling keeps the driver and passes the values through.

    The port and baudrate here are deliberately not the defaults: an assertion against
    /dev/ttyACM0 and 1000000 would still pass if the macro dropped its arguments on the floor and
    fell back to its own defaults (example.ros2_control.xacro:5).
    """
    hardware = hardware_of(render(port='/tmp/waveshare_render_test_pty',
                                  baudrate='115200',
                                  use_mock_hardware=spelling))
    assert hardware.find('plugin').text == REAL_PLUGIN
    assert params_of(hardware) == {'port': '/tmp/waveshare_render_test_pty',
                                   'baudrate': '115200'}


@pytest.mark.parametrize('spelling', TRUE_SPELLINGS)
def test_a_true_spelling_selects_mock_components_and_emits_no_param_at_all(spelling):
    """
    Every accepted true-ish spelling swaps in the loopback simulator and drops every <param>.

    The empty param set is the load-bearing half. mock_components/GenericSystem ignores the params
    it does not know, so a leaked <param name="port"> would not fail a mock run - it would simply
    sit in the description, and the next reader would reasonably assume the mock talks to it. The
    stronger guarantee the example makes is that a mock render contains no serial path anywhere,
    which is why the absence of the port string from the whole document is checked too. That text
    check is safe only because this path is unique to this test and appears in no comment; see
    render() for why structure is never asserted that way.
    """
    port = '/nonexistent/waveshare_render_test'
    root = render(port=port, baudrate='115200', use_mock_hardware=spelling)
    hardware = hardware_of(root)
    assert hardware.find('plugin').text == MOCK_PLUGIN
    assert hardware.findall('param') == [], (
        f'the mock branch leaked {sorted(params_of(hardware))} into <hardware>')
    assert port not in ET.tostring(root, encoding='unicode'), (
        f'a mock render still names the serial path {port} somewhere in the description')


@pytest.mark.parametrize('spelling', ['yes', 'no', 'Yes', '2', 'maybe', 'off'])
def test_an_unrecognised_use_mock_hardware_aborts_the_render(spelling):
    """
    An unrecognised spelling must abort, never fall back to a branch.

    This is a safety property, not tidiness. The guard at example.ros2_control.xacro:12-13 is a
    list lookup rather than a comparison exactly because a plain `use_mock_hardware == 'true'`
    would evaluate use_mock_hardware:=yes as FALSE, load the real driver and open the real serial
    port on a bench somebody believed was running in simulation. Aborting the render aborts the
    launch, which is the only outcome that cannot move a motor by accident.
    """
    with pytest.raises(xacro.XacroException) as raised:
        render(use_mock_hardware=spelling)
    assert 'is not in list' in str(raised.value)


def test_an_empty_use_mock_hardware_aborts_through_the_python_api():
    """
    The empty string aborts here - but only because this test uses the Python API.

    Recorded deliberately, because the two entry points disagree and the difference is invisible
    from either side alone. Through the API the empty string reaches the list lookup and raises;
    from the command line `use_mock_hardware:=` never becomes a mapping at all, because
    xacro/cli.py:84-88 requires both halves of the assignment to be non-empty and drops the pair
    without a word, so the launch path renders the real driver from the `false` default instead.
    Do not read this case as a claim about `ros2 launch`.
    """
    with pytest.raises(xacro.XacroException) as raised:
        render(use_mock_hardware='')
    assert 'is not in list' in str(raised.value)


def test_the_joint_subtrees_are_identical_in_the_mock_and_real_renders():
    """
    use_mock_hardware changes <hardware> and nothing else.

    The four <joint> blocks are shared verbatim between the branches (they sit outside the
    xacro:if / xacro:unless pair), and that sharing is what makes a mock run worth anything: ids,
    types, offsets, command-interface limits and state interfaces are then the same description
    the driver gets. Compared canonicalised and whitespace-stripped so the check is structural
    rather than a diff of the renderer's indentation.
    """
    def joints(**mappings):
        block = render(**mappings).findall('ros2_control')[0]
        return [ET.canonicalize(ET.tostring(joint, encoding='unicode'), strip_text=True)
                for joint in block.findall('joint')]

    mock = joints(use_mock_hardware='true')
    real = joints(use_mock_hardware='false')
    assert [ET.fromstring(joint).get('name') for joint in real] == list(JOINT_NAMES)
    assert mock == real


def test_the_rendered_ros2_control_block_keeps_its_declared_shape():
    """
    One live <ros2_control>, named as the launch expects, with the tuning attributes off.

    rw_rate and is_async are ATTRIBUTES of <ros2_control>, and an attribute the parser does not
    recognise is dropped without a word, so a mistake in them looks like nothing happened. The
    example ships a commented-out copy of both (example.ros2_control.xacro:24-29) together with
    the measurements that argue against them: rw_rate 50 halves bus occupancy but fails this
    package's own velocity gate on a wheel near 2 rad/s, and is_async changed no measurable number
    at four servos. Asserting their absence is what makes un-commenting that block a test failure
    and therefore a decision, rather than a silent change to what the example recommends.

    The link and joint counts belong here too: every joint of the <ros2_control> block must also
    exist in the URDF or the description is rejected at load time, so the two must move together.
    """
    root = render()
    assert root.get('name') == 'waveshare_servos'
    assert len(root.findall('link')) == 5
    assert [joint.get('name') for joint in root.findall('joint')] == list(JOINT_NAMES)

    block = root.findall('ros2_control')[0]
    assert block.attrib == {'name': 'example_ws_ros2_control', 'type': 'system'}
    assert [joint.get('name') for joint in block.findall('joint')] == list(JOINT_NAMES)


def test_the_command_interface_limits_match_the_urdf_joint_limits():
    """
    The <command_interface> min/max equal the URDF <limit> of the same joint.

    This is the one cross-file invariant both descriptions declare load-bearing and nothing
    checked until now. example.urdf.xacro:26-28 says "Keep them numerically equal to the
    <command_interface> min/max in the ros2_control block beside this file: ros2_control merges
    the two and enforces whichever is tighter", and example.ros2_control.xacro:66-71 repeats it
    with the consequence: a disagreement makes the description behave differently depending on the
    controller manager's enforce_command_limits flag, so the same URDF would clamp one way under
    this example's config (the flag is off, and the driver clamps to the ros2_control numbers on
    its own) and another way under a user's. Tightening one side only is a silent change in
    behaviour that passed every other case in this file.

    Both sides are parsed as floats rather than compared as text, so 9.2 against 9.20 is a pass;
    what must not differ is the number. The pairs actually compared are collected and asserted
    against the expected list at the end, because the loop would otherwise pass vacuously on a
    render that had lost its <command_interface> elements altogether.

    The wheels are the documented exception: lower/upper are read only for revolute and prismatic
    joints and are silently discarded for continuous ones (example.urdf.xacro:28-30), so joint3
    and joint4 declare neither a position command interface nor a lower/upper pair, and this case
    asserts that absence instead of a bound.
    """
    root = render()
    urdf_joints = {joint.get('name'): joint for joint in root.findall('joint')}
    compared = []

    for control_joint in root.findall('ros2_control')[0].findall('joint'):
        name = control_joint.get('name')
        assert name in urdf_joints, (
            f'<ros2_control> declares {name}, which has no URDF <joint> of that name; '
            f'ros2_control rejects such a description at load time')
        limit = urdf_joints[name].find('limit')
        assert limit is not None, f'the URDF <joint name="{name}"> carries no <limit>'
        commands = {interface.get('name'): {param.get('name'): float(param.text)
                                            for param in interface.findall('param')}
                    for interface in control_joint.findall('command_interface')}
        continuous = urdf_joints[name].get('type') == 'continuous'

        if continuous:
            assert 'position' not in commands, (
                f'{name} is continuous, so a position command interface has no URDF bound to be '
                f'merged with; either give the joint a bounded type or drop the interface')
            assert limit.get('lower') is None and limit.get('upper') is None, (
                f'{name} is continuous, so lower/upper are discarded by the URDF parser and '
                f'declaring them only invites the two files to disagree invisibly')
        else:
            position = commands['position']
            assert position['min'] == float(limit.get('lower')), (
                f'{name}: position command min {position["min"]} != URDF lower '
                f'{limit.get("lower")}')
            assert position['max'] == float(limit.get('upper')), (
                f'{name}: position command max {position["max"]} != URDF upper '
                f'{limit.get("upper")}')
            compared.append((name, 'position'))

        # The URDF <limit velocity=...> is a magnitude and bounds both directions, so it has to
        # match the max and the negated min of the velocity command interface.
        velocity = commands['velocity']
        assert velocity['max'] == float(limit.get('velocity')), (
            f'{name}: velocity command max {velocity["max"]} != URDF velocity '
            f'{limit.get("velocity")}')
        assert velocity['min'] == -float(limit.get('velocity')), (
            f'{name}: velocity command min {velocity["min"]} != -(URDF velocity '
            f'{limit.get("velocity")})')
        compared.append((name, 'velocity'))

    assert compared == [('joint1', 'position'), ('joint1', 'velocity'),
                        ('joint2', 'position'), ('joint2', 'velocity'),
                        ('joint3', 'velocity'), ('joint4', 'velocity')]
