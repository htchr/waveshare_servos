"""
Launch test for the shipped example, brought up on mock hardware (jazzy.md Phase 5 item 4).

Starts bringup/launch/example.launch.py with mock_components/GenericSystem in place of the
driver and then asserts, from inside the running stack, everything the example promises: the
controller manager and robot_state_publisher are up, the loaded hardware component really is the
mock one, three controllers are active and the fourth is deliberately inactive, and
/joint_states carries exactly the four joints of the description.

This test opens no serial port. Read the block below before changing a launch argument.
"""

# ---------------------------------------------------------------------------------------------
# WHAT KEEPS THIS TEST OFF THE REAL BUS, AND WHICH PARTS OF IT ARE ACTUALLY INDEPENDENT
#
# example.launch.py defaults `port` to /dev/ttyACM0 (bringup/launch/example.launch.py:28-32) and
# `gui` to true (:45-49). On this bench four real servos hang off that port, and the example's
# joint_trajectory_position_controller claims position AND velocity command interfaces on two of
# them. So a launch test that forgets one argument does not merely fail -- it takes the bus and
# drives the hardware while nobody is expecting it to move.
#
# An earlier version of this block called the items below "four safety layers ... each is enough
# on a good day". That was false and is corrected here: two of them are one mechanism. Read them
# as two groups.
#
# GROUP 1 -- ONE SHARED PATH. `use_mock_hardware:='true'` and
# `port:='/nonexistent/waveshare_launch_test'` are not independent. Both travel exactly the same
# route: the LAUNCH_ARGUMENTS dict below, the matching DeclareLaunchArgument entries
# (bringup/launch/example.launch.py:27-50), the single Command([...]) that appends the three
# `name:=` pairs (:61-77), and the <xacro:arg> block that receives them
# (description/urdf/example.urdf.xacro:6-8, forwarded into the macro at :110-113). One plausible
# edit breaks both at once: swap that Command([...]) for a pre-rendered URDF, or refactor it into
# a helper that loses the `name:=` pairs, and the macro falls back to ITS OWN defaults
# (description/ros2_control/example.ros2_control.xacro:5) -- the real driver on /dev/ttyACM0.
# test/test_urdf_xacro.py cannot see that edit: it renders the .xacro directly and never
# exercises the launch file's argument forwarding.
#
#   1. use_mock_hardware:='true' -- the value that actually prevents the open.
#      description/ros2_control/example.ros2_control.xacro:46-59 then emits
#      <plugin>mock_components/GenericSystem</plugin> and omits the <param name="port"> element
#      entirely, so the rendered description contains no serial path at all. Note that an
#      unrecognised spelling does not fall back to the real driver: the xacro list lookup at
#      example.ros2_control.xacro:12-13 raises and aborts the render, and with it the launch.
#   2. port:='/nonexistent/waveshare_launch_test' -- what makes the OTHER failure of this same
#      path cheap. If the mock swap itself regresses (a renamed xacro arg, a bad merge, an
#      inverted branch) while the three arguments still arrive, the real driver loads, tries to
#      open THIS path, fails, and turns the test red instead of seizing the live bus. A
#      nonexistent path is a bug report; the default path is a moving robot. It buys nothing
#      against the edit described above, because that edit throws this value away along with the
#      other two.
#
# GROUP 2 -- THE TWO CHECKS THAT VERIFY INSTEAD OF ASKING. Items 1 and 2 are a request; these two
# read back what the request produced, on the two sides the request cannot vouch for itself:
#
#   3. assert_description_renders_to_the_mock(), called from generate_test_description() before
#      the LaunchDescription is returned. It renders the installed share copy of the description
#      itself (xacro.process_file, the same file the launch file's Command resolves through
#      FindPackageShare) and REFUSES to hand back a launch description unless the one <hardware>
#      block names the mock plugin, carries no <param name="port">, and the port string appears
#      nowhere in the document. Two things make it worth its lines: it fails CLOSED -- anything
#      unexpected, including a render that raises, is an error and no process is ever spawned --
#      and it fails BEFORE startup, which items 1-2 structurally cannot, being a request that is
#      only granted once ros2_control_node has already loaded a component. What it does NOT cover
#      is the launch file forwarding the arguments at all: it renders from LAUNCH_ARGUMENTS, so a
#      launch file that quietly stopped passing the pairs would still render a mock here. That
#      case is item 4's, and item 4 is necessarily after the fact.
#   4. test_2_hardware_component_is_the_mock -- configuration is not proof. Item 3 is what the
#      description WOULD render; item 4 is what the running controller manager actually loaded.
#      It is the one check here that survives somebody rewiring the argument plumbing entirely,
#      and the price of that reach is that it can only speak once the stack is up, i.e. after the
#      port would already have been opened. Hence 3 AND 4, not 3 instead of 4.
#
# `gui:='false'` is in the dict too but is not a safety measure: it keeps rviz2 out of the
# launch. No assertion needs it, and it is one fewer process to bring down cleanly at shutdown.
# ---------------------------------------------------------------------------------------------

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

# The <ros2_control> block is named at description/urdf/example.urdf.xacro:110, and that name is
# what list_hardware_components reports back.
HARDWARE_COMPONENT = 'example_ws_ros2_control'
MOCK_PLUGIN = 'mock_components/GenericSystem'

# The arguments this test hands to the example launch file (group 1 of the safety block above).
# A named dict rather than an inline literal so that item 3 checks the very same values the
# launch consumes: edit one of these and the pre-flight render re-reads the edit, so dropping
# use_mock_hardware here makes the render select the real driver and the guard raise.
LAUNCH_ARGUMENTS = {
    'use_mock_hardware': 'true',
    'gui': 'false',
    'port': '/nonexistent/waveshare_launch_test',
}

# Of those, the ones that reach the description. gui is a launch-file argument only
# (bringup/launch/example.launch.py:45-49) and xacro does not declare it. baudrate is absent on
# purpose: this test does not override it, and the launch file's default (1000000, :33-37) is the
# same value as the <xacro:arg> default (description/urdf/example.urdf.xacro:7), so leaving it
# out of the mappings renders exactly what the launch will render. (xacro fills a declared
# default straight INTO the mappings dict it was handed -- process_doc aliases it as
# substitution_args_context['arg'] at xacro/__init__.py:1049 and writes defaults into it at
# :955-956 -- which is both why the guard's error message names baudrate anyway and why the
# dict passed to it is built fresh at every call rather than being LAUNCH_ARGUMENTS itself.)
XACRO_ARGUMENTS = ('port', 'baudrate', 'use_mock_hardware')

# The file the launch file's Command([...]) renders, reached the same way it reaches it: through
# the share directory (bringup/launch/example.launch.py:65-72), not through src/.
EXAMPLE_URDF_XACRO = os.path.join(
    get_package_share_directory('waveshare_servos'), 'description', 'urdf', 'example.urdf.xacro'
)

# Spawned and activated by one spawner, in this command-line order
# (bringup/launch/example.launch.py:133-146).
ACTIVE_CONTROLLERS = [
    'joint_state_broadcaster',
    'joint_trajectory_position_controller',
    'joint_velocity_controller',
]

# Spawned --inactive by a SECOND spawner process (bringup/launch/example.launch.py:162-174),
# because it claims the same joint3/joint4 velocity command interfaces as
# joint_velocity_controller and ros2_control gives each command interface to one controller only.
# jazzy.md's Phase 5 item 4 says "the three controllers", which predates Phase 4 adding this one.
# Asserting the inactive state is load-bearing rather than decorative: without it the suite stops
# noticing the day the --inactive flag is dropped, and the launch would then either fail to
# activate diff_drive_controller or -- worse, if the spawner order flipped -- leave the example's
# own velocity controller unable to claim its interfaces.
INACTIVE_CONTROLLERS = ['diff_drive_controller']

# joint_state_broadcaster has no `joints:` parameter (bringup/config/example_controllers.yaml),
# so it publishes every joint the hardware component exports. check_if_js_published compares
# SETS and lengths, not a subset, so this list has to be exact.
JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']

# Generous because ctest may run this on a loaded machine and because every spawner on the
# machine serialises on ~/.ros/locks/ros2-control-controller-spawner.lock
# (bringup/launch/example.launch.py:156-161).
#
# Budget, because this and the ctest TIMEOUT are different things and only the second one can
# turn a readable assertion failure into an unreadable "test timed out". A healthy run is about
# 7 s end to end (measured). Two of the helpers below spend their timeout TWICE, which is what
# makes the worst case so much larger than it looks:
#   * check_controllers_running -- once waiting for the controller node to appear in the graph,
#     then again polling list_controllers for the state (controller_manager/test_utils.py:59-119
#     under /opt/ros/jazzy/lib/python3.12/site-packages).
#   * check_if_js_published -- it hard-codes 20 s (test_utils.py:134), but WaitForTopics.wait()
#     waits that long for a publisher to connect and then that long again for the message event
#     (launch_testing_ros/wait_for_topics.py:111-116), so test_5's share of the budget is 40 s,
#     not 20 s.
# So a run in which nothing at all comes up costs roughly 45 (ReadyToTest) + 60 (test_1, two
# check_node_running calls) + 30 (test_2's own polling deadline) + 60 (test_3) + 60 (test_4)
# + 40 (test_5), near 295 s. CMakeLists.txt therefore gives this test TIMEOUT 330 -- that sum
# plus headroom for launch shutdown and the post-shutdown test. Raise this constant only
# together with that TIMEOUT, and lower the ctest TIMEOUT below the sum only if you are content
# for a total-failure run to be reported as a timeout instead of an assertion.
STARTUP_TIMEOUT = 30.0


def assert_description_renders_to_the_mock(mappings):
    """
    Refuse to launch unless these xacro arguments really render the mock hardware.

    Item 3 of the safety block at the top of this file: the one check that does not travel the
    launch file's argument path and the only one that can speak BEFORE a process is spawned.
    Raises rather than asserts, because this runs in generate_test_description() rather than in a
    test method: nothing here is rewritten by pytest's assertion machinery, and a `raise` is also
    the one form that survives python -O.

    Fails closed in every direction. A render that raises propagates, an unexpected shape raises
    here, and neither returns a LaunchDescription -- so the failure modes all end with no
    ros2_control_node, which is the only state in which /dev/ttyACM0 is certainly untouched.
    xacro.process_file is text work only: it opens the .xacro files and nothing else.
    """
    if 'port' not in mappings:
        raise RuntimeError(
            'the mock-hardware guard needs a port mapping to look for; LAUNCH_ARGUMENTS no '
            'longer carries one, so this test can no longer prove where the driver would point')
    # process_file (xacro/__init__.py:1105-1125), not process() (:1177-1185): process() wraps
    # _process, which catches the exception and calls sys.exit(2) (:1165), and a bare SystemExit
    # here would say nothing about which argument was wrong. See test/test_urdf_xacro.py's
    # render() docstring for the same reasoning.
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
    # The whole-document check is the one that would survive the port moving out of <hardware>
    # into some other element. It is safe as a text match only because this path is unique to
    # this test and appears in no comment of either description file.
    if mappings['port'] in ET.tostring(root, encoding='unicode'):
        raise RuntimeError(
            f'a mock render still names the serial path {mappings["port"]} somewhere in the '
            'description')


# ReadyToTest bounds only how long launch waits for the processes to START, and defaults to 15 s
# (launch_testing/loader.py:98-101). Four nodes plus a xacro render on a busy machine can pass
# that; 45 s costs nothing when things are healthy. keep_alive is a separate concern: the
# LaunchService runs with shutdown_when_idle=not keep_alive (launch_testing/test_runner.py:161),
# and without it a controller manager that dies early ends the launch and surfaces as an opaque
# _LaunchDiedException instead of the assertion failure that explains what broke.
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
@launch_testing.ready_to_test_action_timeout(45)
def generate_test_description():
    """Include the shipped example launch file with the mock-hardware safety arguments."""
    # Before anything is constructed, let alone spawned: prove that the description these
    # arguments select is the mock one. Raising here means launch_testing never gets a
    # LaunchDescription, so not one process starts.
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

    # unittest runs test methods in alphabetical order, so the numeric prefixes are what put
    # these in a useful sequence: a failure in the earliest one is the one worth reading, and
    # checking the mock plugin before anything commands a joint keeps the safety check first.

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
        # The controller manager takes the description from the /robot_description topic that
        # robot_state_publisher latches, so if the second of these is missing the first will be
        # up but will have loaded no hardware component at all -- which is exactly the failure
        # test_2 would otherwise report as a confusing empty component list.
        check_node_running(self.node, 'controller_manager', timeout=STARTUP_TIMEOUT)
        check_node_running(self.node, 'robot_state_publisher', timeout=STARTUP_TIMEOUT)

    def test_2_hardware_component_is_the_mock(self):
        """Item 4 of the safety block: the loaded plugin really is the mock, not the driver."""
        # Poll rather than call once. The controller manager answers this service as soon as it
        # is up, but it only has a component after it has received /robot_description, so a
        # single early call legitimately returns an empty list.
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
        # plugin_name is the live field; class_type beside it is marked DEPRECATED in
        # controller_manager_msgs/msg/HardwareComponentState.msg. Equality rather than a
        # "waveshare" substring check, so that an empty or renamed plugin also fails loudly.
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
        # No assertion about the VALUES on this topic, and deliberately none about motion:
        # mock_components/GenericSystem never advances a velocity-commanded joint's position
        # state, so joint3 and joint4 read a constant 0.0 however hard anything commands them
        # (description/ros2_control/example.ros2_control.xacro:37-45). A wheel-motion check here
        # would only pass on real hardware, which this test must never touch.
        check_if_js_published('/joint_states', JOINTS)


@launch_testing.post_shutdown_test()
class TestExampleLaunchShutdown(unittest.TestCase):
    """Assertions once the launch has been shut down."""

    def test_spawners_exited_cleanly(self, proc_info):
        """Both spawner processes report success."""
        # Both are named 'spawner', and resolveProcesses matches on a substring of the process
        # name and then errors on a multiple match while strict_proc_matching is left at its
        # default True (launch_testing/util/proc_lookup.py:118-140). strict_proc_matching=False
        # is what makes this cover BOTH of them rather than raising. Their exit codes are the
        # only place a controller that failed to load shows up as a process-level failure: the
        # spawner returns as soon as it has spawned (spawner.py:600-601, unload_on_kill off), so
        # these codes are already final long before shutdown.
        launch_testing.asserts.assertExitCodes(
            proc_info, process='spawner', strict_proc_matching=False
        )
