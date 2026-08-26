# Copyright 2026 WheelHub Intelligent
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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def resolve_arm_config(arm_type_str: str):
    if any(x in arm_type_str for x in ("1.0", "10", "1_0")):
        return "openarm_v1.0", "openarm_v10.urdf.xacro"
    return "openarm_v2.0", "openarm_v20.urdf.xacro"


def generate_robot_description(context, description_package, arm_type, use_fake_hardware):
    description_package_str = context.perform_substitution(description_package)
    arm_type_str = context.perform_substitution(arm_type)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)

    folder_name, file_name = resolve_arm_config(arm_type_str)
    xacro_path = os.path.join(
        get_package_share_directory(description_package_str),
        "assets", "robot", folder_name, "urdf", file_name
    )

    return xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": arm_type_str,
            "bimanual": "true",
            "use_fake_hardware": use_fake_hardware_str,
            "ros2_control": "true",
        },
    ).toprettyxml(indent="  ")


def spawn_nodes(context, description_package, arm_type, use_fake_hardware, controllers_file):
    robot_description = generate_robot_description(
        context, description_package, arm_type, use_fake_hardware,
    )
    controllers_file_str = context.perform_substitution(controllers_file)
    robot_description_param = {"robot_description": robot_description}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description_param, controllers_file_str],
    )

    return [robot_state_pub_node, control_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("description_package", default_value="openarm_description"),
        DeclareLaunchArgument("arm_type", default_value="openarm_v2.0"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("controllers_file", default_value="controllers_openarm.yaml"),
    ]

    description_package = LaunchConfiguration("description_package")
    arm_type = LaunchConfiguration("arm_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    controllers_file = PathJoinSubstitution(
        [FindPackageShare("whi_arm_interface"), "config", LaunchConfiguration("controllers_file")]
    )

    spawn_func = OpaqueFunction(
        function=spawn_nodes,
        args=[description_package, arm_type, use_fake_hardware, controllers_file],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    left_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "--controller-manager", "/controller_manager"],
    )

    right_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "--controller-manager", "/controller_manager"],
    )

    left_gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_gripper_controller", "--controller-manager", "/controller_manager"],
    )

    right_gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_gripper_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription(
        declared_arguments + [spawn_func, TimerAction(period=2.0,
            actions=[
                jsb_spawner,
                left_arm_spawner,
                right_arm_spawner,
                left_gripper_spawner,
                right_gripper_spawner
        ])]
    )