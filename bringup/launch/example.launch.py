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

    # Command joins this list with no separator, so each space is its own element. xacro ignores
    # an undeclared arg silently, so example.urdf.xacro must declare all three.
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
    # value_type=str: the URDF is XML, and launch would otherwise YAML-parse it (a colon in an
    # XML comment aborts the launch).
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

    # Takes the URDF from robot_state_publisher's transient-local /robot_description; no remap.
    # output='both' also shows the vendored serial layer's raw stdout.
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
    # example_ws.rviz reads /robot_description as Transient Local; with Volatile, RViz would show
    # no model and no error.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='both',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(gui),
    )

    # One switch per controller, in this order (no --activate-as-group), so one failure does not
    # stop the others. The spawner waits for the controller manager with no timeout.
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

    # Inactive, so it claims nothing and the two spawners may run in either order; it shares
    # joint3/joint4 with joint_velocity_controller. See docs/setup.md, "Drive a differential base".
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
