"""
The example description renders correctly with and without use_mock_hardware.

Mock: GenericSystem, no <param>. Real: the driver, given port and baudrate. Unknown: no render.
"""

import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro

# Render the installed share copy: a description file that install() misses fails every render.
# See docs/development.md, "Launch and render tests".
EXAMPLE_URDF_XACRO = (get_package_share_directory('waveshare_servos') +
                      '/description/urdf/example.urdf.xacro')

XACRO_NS = '{http://www.ros.org/wiki/xacro}'

REAL_PLUGIN = 'waveshare_servos/WaveshareServos'
MOCK_PLUGIN = 'mock_components/GenericSystem'

# The four joints of the example, ids 1-4, as on the reference bench.
JOINT_NAMES = ('joint1', 'joint2', 'joint3', 'joint4')

# The spellings the xacro list lookup accepts. It strips and lower-cases the value, so only
# case and outer spaces can vary.
TRUE_SPELLINGS = ('true', 'True', 'TRUE', '1', ' true ')
FALSE_SPELLINGS = ('false', 'False', 'FALSE', '0', ' FALSE ')


def render(**mappings):
    """
    Render the example with these xacro arguments and return its <robot> root.

    process_file raises; process() exits. ElementTree drops all comments, so no check sees them.
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
    The example declares port, baudrate and use_mock_hardware, with defaults, and nothing else.

    xacro ignores an undeclared argument, so a renamed <xacro:arg> fails no render.
    """
    declared = [(arg.get('name'), arg.get('default'))
                for arg in ET.parse(EXAMPLE_URDF_XACRO).getroot().findall(XACRO_NS + 'arg')]
    assert declared == [('port', '/dev/ttyACM0'),
                        ('baudrate', '1000000'),
                        ('use_mock_hardware', 'false')]


def test_the_default_render_selects_the_real_driver_on_the_bench_port():
    """
    With no arguments the example selects the real driver on /dev/ttyACM0 at 1 Mbaud.

    These defaults are what the example launch uses with no arguments.
    """
    hardware = hardware_of(render())
    assert hardware.find('plugin').text == REAL_PLUGIN
    assert params_of(hardware) == {'port': '/dev/ttyACM0', 'baudrate': '1000000'}


@pytest.mark.parametrize('spelling', FALSE_SPELLINGS)
def test_a_false_spelling_selects_the_real_driver_and_forwards_port_and_baudrate(spelling):
    """
    Every accepted false spelling keeps the driver and forwards port and baudrate.

    The values are not the defaults, so a macro that falls back to its own defaults fails.
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
    Every accepted true spelling selects the mock with no <param> and no serial path.

    GenericSystem ignores unknown params, so a leaked port param would not fail a mock run.
    """
    port = '/nonexistent/waveshare_render_test'
    root = render(port=port, baudrate='115200', use_mock_hardware=spelling)
    hardware = hardware_of(root)
    assert hardware.find('plugin').text == MOCK_PLUGIN
    assert hardware.findall('param') == [], (
        f'the mock branch leaked {sorted(params_of(hardware))} into <hardware>')
    # A safe text search: this path is in no comment.
    assert port not in ET.tostring(root, encoding='unicode'), (
        f'a mock render still names the serial path {port} somewhere in the description')


@pytest.mark.parametrize('spelling', ['yes', 'no', 'Yes', '2', 'maybe', 'off'])
def test_an_unrecognised_use_mock_hardware_aborts_the_render(spelling):
    """
    An unrecognised use_mock_hardware spelling aborts the render and selects no branch.

    A plain comparison would read 'yes' as false and open the real port.
    """
    with pytest.raises(xacro.XacroException) as raised:
        render(use_mock_hardware=spelling)
    assert 'is not in list' in str(raised.value)


def test_an_empty_use_mock_hardware_aborts_through_the_python_api():
    """
    An empty use_mock_hardware aborts the render, but only through the Python API.

    On the command line xacro drops an empty `use_mock_hardware:=` and renders the real driver.
    """
    # See docs/setup.md, "Try it without hardware".
    with pytest.raises(xacro.XacroException) as raised:
        render(use_mock_hardware='')
    assert 'is not in list' in str(raised.value)


def test_the_joint_subtrees_are_identical_in_the_mock_and_real_renders():
    """
    use_mock_hardware changes <hardware> only: the <joint> blocks are the same in both renders.

    The blocks are compared in canonical form, so indentation does not count.
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
    One <ros2_control> block, named as the launch expects, with no rw_rate or is_async.

    Enable either attribute only on purpose, and change this test with it.
    """
    # See docs/bus-timing.md, "rw_rate and is_async".
    root = render()
    assert root.get('name') == 'waveshare_servos'
    assert len(root.findall('link')) == 5
    assert [joint.get('name') for joint in root.findall('joint')] == list(JOINT_NAMES)

    block = root.findall('ros2_control')[0]
    assert block.attrib == {'name': 'example_ws_ros2_control', 'type': 'system'}
    assert [joint.get('name') for joint in block.findall('joint')] == list(JOINT_NAMES)


def test_the_command_interface_limits_match_the_urdf_joint_limits():
    """
    Each <command_interface> min/max equals the URDF <limit> of the same joint.

    The wheels (continuous) must have no position interface and no lower/upper.
    """
    # ros2_control merges the two and enforces the tighter one.
    # See docs/configuration.md, "Command interfaces and limits".
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
