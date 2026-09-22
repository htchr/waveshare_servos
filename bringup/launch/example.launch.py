# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM0',
            description='Serial port of the bus servo adapter. Ignored under mock hardware.',
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='1000000',
            description='Bus baud rate. Ignored under mock hardware.',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Swap the driver for mock_components/GenericSystem, which opens no '
                        'serial port. Accepts true, false, 1 or 0 in any case; any other value '
                        'aborts the xacro render, and with it the launch.',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
        ),
    ]

    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    gui = LaunchConfiguration('gui')

    # Command concatenates this list with no separator, so every space is an element of its own.
    # xacro accepts arguments a document does not declare, silently and with exit status 0, so
    # these three name:= pairs only do anything because description/urdf/example.urdf.xacro
    # declares them and forwards them into the example_ws_ros2_control macro call.
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('waveshare_servos'),
                    'description',
                    'urdf',
                    'example.urdf.xacro',
                ]
            ),
            ' ', 'port:=', port,
            ' ', 'baudrate:=', baudrate,
            ' ', 'use_mock_hardware:=', use_mock_hardware,
        ]
    )
    # The rendered URDF is XML, not YAML. Without an explicit ``value_type=str`` launch tries to
    # YAML-parse it and aborts the whole launch the moment the document contains anything YAML
    # considers syntax (a colon in an XML comment is enough).
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('waveshare_servos'),
            'config',
            'example_controllers.yaml',
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('waveshare_servos'), 'description/rviz', 'example_ws.rviz']
    )

    # The controller manager takes the robot description from the /robot_description topic that
    # robot_state_publisher latches (transient-local), so there is no ~/robot_description
    # subscription for a remapping to match and this node carries none. output='both' puts both the
    # driver's log lines and the vendored serial layer's raw stdout on the console and in
    # launch.log; output='log' hid only the latter.
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers],
        output='both',
    )
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )
    # RViz can start straight away: robot_state_publisher publishes /robot_description with
    # transient-local durability, so a late subscriber still receives it. The saved config must ask
    # for that durability, which is why example_ws.rviz sets the RobotModel description topic to
    # Transient Local - with Volatile there, RViz would silently show no model.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='both',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(gui),
    )

    # One spawner, three controllers, in this order. Without --activate-as-group the spawner loads,
    # configures and activates them strictly in command-line order with one switch each, so
    # joint_state_broadcaster comes up first and a controller that fails to activate does not take
    # the others down with it. The spawner waits for the controller manager indefinitely
    # (--controller-manager-timeout defaults to 0.0), so no timer or event handler is needed to
    # sequence it after control_node. The controller manager already has this YAML as its own
    # parameters; --param-file makes the spawner self-contained as well.
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='both',
        arguments=[
            'joint_state_broadcaster',
            'joint_trajectory_position_controller',
            'joint_velocity_controller',
            '--controller-manager',
            '/controller_manager',
            '--param-file',
            robot_controllers,
        ],
    )

    # diff_drive_controller drives joint3/joint4 through the same velocity command interfaces as
    # joint_velocity_controller, and ros2_control gives each command interface to one controller
    # only, so the two can never be active together. It is loaded and configured but left inactive:
    # it claims no interface and cannot move a servo, while its cmd_vel and odom topics exist from
    # startup. Swap with
    #   ros2 control switch_controllers --strict \
    #     --deactivate joint_velocity_controller --activate diff_drive_controller
    #
    # Ordering between the two spawner PROCESSES is a race, not a sequence: every spawner on this
    # machine serialises on one lock file (~/.ros/locks/ros2-control-controller-spawner.lock) and
    # holds it while it waits for the controller manager, so this one often loads first. That is
    # safe only because --inactive claims nothing. Two consequences: do not read "diff_drive is
    # inactive" as "diff_drive was spawned last", and do not run this launch at the same time as
    # anything else that spawns controllers (test/hil_check.sh does).
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='both',
        arguments=[
            'diff_drive_controller',
            '--inactive',
            '--controller-manager',
            '/controller_manager',
            '--param-file',
            robot_controllers,
        ],
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        rviz_node,
        controller_spawner,
        diff_drive_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
