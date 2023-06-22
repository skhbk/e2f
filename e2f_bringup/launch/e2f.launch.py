#  Copyright 2023 Sakai Hibiki
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    args = []
    args.append(DeclareLaunchArgument("use_fake_hardware", default_value="false"))
    args.append(DeclareLaunchArgument("device_name", default_value="/dev/ttyUSB0"))
    args.append(DeclareLaunchArgument("baud_rate", default_value="1000000"))
    args.append(DeclareLaunchArgument("dynamixel_id", default_value="1"))

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("e2f_description"), "urdf", "e2f.urdf.xacro"]
            ),
            " ",
            "use_fake_hardware:=",
            LaunchConfiguration("use_fake_hardware"),
            " ",
            "device_name:=",
            LaunchConfiguration("device_name"),
            " ",
            "baud_rate:=",
            LaunchConfiguration("baud_rate"),
            " ",
            "dynamixel_id:=",
            LaunchConfiguration("dynamixel_id"),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    initial_controllers = PathJoinSubstitution(
        [FindPackageShare("e2f_bringup"), "config", "e2f_controllers.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, initial_controllers],
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration("use_fake_hardware")),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        condition=UnlessCondition(LaunchConfiguration("use_fake_hardware")),
    )

    gripper_command_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_command_controller", "-c", "/controller_manager"],
        condition=UnlessCondition(LaunchConfiguration("use_fake_hardware")),
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("use_fake_hardware")),
    )

    # Rviz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("e2f_bringup"), "rviz", "view.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace="",
        arguments=["-d", rviz_config_file],
        emulate_tty=True,
    )

    actions = [
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        gripper_command_controller_spawner,
        joint_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(args + actions)
