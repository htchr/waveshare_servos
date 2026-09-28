"""
Launch test: the shipped example on mock hardware. It opens no serial port.

Checks the nodes, the mock component, the controller states and the joints on /joint_states.
"""

# Off the bench: use_mock_hardware:=true and a nonexistent port share one path, so the guard
# and test_2 read back the result. See docs/development.md, "Keep tests off the bench".

import os
import time
import unittest
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from controller_manager.controller_manager_services import list_hardware_components
from controller_manager.test_utils import check_controllers_running
from controller_manager.test_utils import check_if_js_published
from controller_manager.test_utils import check_node_running
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest
import rclpy
import xacro

CONTROLLER_MANAGER = '/controller_manager'

# The <ros2_control> block name in example.urdf.xacro, as list_hardware_components reports it.
HARDWARE_COMPONENT = 'example_ws_ros2_control'
MOCK_PLUGIN = 'mock_components/GenericSystem'

# The guard renders these same values, so removing use_mock_hardware here makes it raise.
LAUNCH_ARGUMENTS = {
    'use_mock_hardware': 'true',
    'gui': 'false',
    'port': '/nonexistent/waveshare_launch_test',
}

# The launch arguments that xacro declares (gui is launch-only). xacro writes its defaults into
# the dict it gets, so the guard gets a new dict, never LAUNCH_ARGUMENTS itself.
XACRO_ARGUMENTS = ('port', 'baudrate', 'use_mock_hardware')

# The file the launch renders, found the same way: through the share directory, not src/.
EXAMPLE_URDF_XACRO = os.path.join(
    get_package_share_directory('waveshare_servos'), 'description', 'urdf', 'example.urdf.xacro'
)

# Spawned and activated by one spawner, in this order.
ACTIVE_CONTROLLERS = [
    'joint_state_broadcaster',
    'joint_trajectory_position_controller',
    'joint_velocity_controller',
]

# Spawned --inactive by a second spawner: it shares the joint3/joint4 velocity interfaces with
# joint_velocity_controller. This check fails if the --inactive flag is removed.
INACTIVE_CONTROLLERS = ['diff_drive_controller']

# joint_state_broadcaster publishes every joint and check_if_js_published compares sets, so
# this list must be exact.
JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']

# Seconds per wait. When nothing starts, the test takes about 295 s; the ctest TIMEOUT is 330.
# Change both together. See docs/development.md, "Launch and render tests".
STARTUP_TIMEOUT = 30.0


def assert_description_renders_to_the_mock(mappings):
    """
    Raise unless these xacro arguments render the mock hardware with no serial port.

    Runs before launch, so a failure starts no process. It raises, so it works under python -O.
    """
    if 'port' not in mappings:
        raise RuntimeError(
            'the mock-hardware guard needs a port mapping to look for; LAUNCH_ARGUMENTS no '
            'longer carries one, so this test can no longer prove where the driver would point')
    # process_file raises XacroException; xacro.process() calls sys.exit(2) with no message.
    root = ET.fromstring(xacro.process_file(EXAMPLE_URDF_XACRO, mappings=mappings).toxml())

    blocks = root.findall('ros2_control')
    if len(blocks) != 1:
        raise RuntimeError(
            f'expected exactly one <ros2_control> block in the render, found {len(blocks)}: '
            f'{[block.get("name") for block in blocks]}')
    hardware = blocks[0].find('hardware')
    plugin = None if hardware is None else hardware.find('plugin')
    plugin_name = None if plugin is None else plugin.text
    if plugin_name != MOCK_PLUGIN:
        raise RuntimeError(
            f'{EXAMPLE_URDF_XACRO} rendered with {sorted(mappings.items())} selects the hardware '
            f'plugin {plugin_name!r}, not {MOCK_PLUGIN!r}. Launching it would load the real '
            'driver, so no launch description is returned and nothing is started.')
    leaked = sorted(param.get('name') for param in hardware.findall('param'))
    if 'port' in leaked:
        raise RuntimeError(
            f'the mock branch emitted a <param name="port"> ({leaked} in <hardware>). '
            'mock_components/GenericSystem ignores params it does not know, so this would not '
            'fail a mock run -- it would just leave a serial path in the description for the '
            'next reader, or the next driver, to use.')
    # A text search finds the port anywhere in the document. It is safe because this path is
    # unique to this test and is in no comment of the description files.
    if mappings['port'] in ET.tostring(root, encoding='unicode'):
        raise RuntimeError(
            f'a mock render still names the serial path {mappings["port"]} somewhere in the '
            'description')


# ReadyToTest waits 45 s (default 15 s) for a busy machine. keep_alive stops an early node exit
# from ending the launch with an opaque _LaunchDiedException instead of a test failure.
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
@launch_testing.ready_to_test_action_timeout(45)
def generate_test_description():
    """Include the shipped example launch file with the mock-hardware safety arguments."""
    # Check the rendered description before any process starts. A raise here starts nothing.
    assert_description_renders_to_the_mock(
        {name: value for name, value in LAUNCH_ARGUMENTS.items() if name in XACRO_ARGUMENTS}
    )

    example_launch = os.path.join(
        get_package_share_directory('waveshare_servos'), 'launch', 'example.launch.py'
    )
    example = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(example_launch),
        launch_arguments=LAUNCH_ARGUMENTS.items(),
    )
    return LaunchDescription([example, launch_testing.actions.ReadyToTest()])


class TestExampleLaunchOnMockHardware(unittest.TestCase):
    """Assertions against the running stack, in startup order."""

    # unittest sorts methods by name: the numeric prefixes keep startup order, safety check first.

    @classmethod
    def setUpClass(cls):
        """Create one node for the whole class; the service calls below all reuse it."""
        rclpy.init()
        cls.node = rclpy.create_node('test_example_launch')

    @classmethod
    def tearDownClass(cls):
        """Tear the node down before the launch is shut down."""
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_1_nodes_running(self):
        """The controller manager and robot_state_publisher come up."""
        # The controller manager gets its description from robot_state_publisher. Without that
        # node, test_2 sees an empty component list.
        check_node_running(self.node, 'controller_manager', timeout=STARTUP_TIMEOUT)
        check_node_running(self.node, 'robot_state_publisher', timeout=STARTUP_TIMEOUT)

    def test_2_hardware_component_is_the_mock(self):
        """The loaded hardware plugin is the mock, not the driver."""
        # Poll: the service answers before the description arrives, and until then it returns an
        # empty list.
        deadline = time.monotonic() + STARTUP_TIMEOUT
        components = []
        while time.monotonic() < deadline:
            components = list_hardware_components(
                self.node, CONTROLLER_MANAGER, service_timeout=STARTUP_TIMEOUT
            ).component
            if components:
                break
            time.sleep(0.2)

        self.assertEqual(
            [c.name for c in components],
            [HARDWARE_COMPONENT],
            'expected exactly the one <ros2_control> block of the example description',
        )
        component = components[0]
        # plugin_name is current (class_type is deprecated). Exact match, so an empty or renamed
        # plugin fails too.
        self.assertEqual(
            component.plugin_name,
            MOCK_PLUGIN,
            'the launch loaded a hardware plugin other than the mock -- if this says '
            'waveshare_servos/WaveshareServos then this test just tried to open a serial port',
        )
        # An inactive component would read and write nothing, so /joint_states in test_5 would
        # stall with no explanation of why.
        self.assertEqual(component.state.label, 'active')

    def test_3_controllers_active(self):
        """The three controllers the example activates are active."""
        check_controllers_running(
            self.node, ACTIVE_CONTROLLERS, state='active', timeout=STARTUP_TIMEOUT
        )

    def test_4_diff_drive_controller_inactive(self):
        """diff_drive_controller is loaded and configured but NOT active -- see the constant."""
        check_controllers_running(
            self.node, INACTIVE_CONTROLLERS, state='inactive', timeout=STARTUP_TIMEOUT
        )

    def test_5_joint_states_published(self):
        """/joint_states carries exactly joint1..joint4."""
        # Joint names only, no values: the mock never advances a velocity-commanded joint's
        # position, so joint3 and joint4 stay at 0.0.
        check_if_js_published('/joint_states', JOINTS)


@launch_testing.post_shutdown_test()
class TestExampleLaunchShutdown(unittest.TestCase):
    """Assertions once the launch has been shut down."""

    def test_spawners_exited_cleanly(self, proc_info):
        """Both spawner processes report success."""
        # Both processes are named 'spawner': strict_proc_matching=False checks both. A spawner
        # exits after it spawns, so a nonzero code means a controller did not load.
        launch_testing.asserts.assertExitCodes(
            proc_info, process='spawner', strict_proc_matching=False
        )
