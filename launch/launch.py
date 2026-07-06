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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import State
import subprocess

def launch_setup(context, *args, **kwargs):
    # Input parameters declaration
    # evaluate substitutions at runtime
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    arm = LaunchConfiguration("arm").perform(context)
    arm_model = LaunchConfiguration("arm_model").perform(context)

    # robot description
    if arm == "fr":
        config_pkg_name = f"{arm}{arm_model}_moveit_config"
    else:
        config_pkg_name = f"{arm}_{arm_model}_moveit_config"

    urdf_file_name = f"{arm}_{arm_model}.urdf.xacro"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(config_pkg_name), 'config', urdf_file_name]
            ),
        ]
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[{
            'robot_description': robot_description_content,
        }],
    )

    controller_params_file = PathJoinSubstitution([
        FindPackageShare("whi_arm_interface"),
        "config",
        f"controllers_{arm}_{arm_model}.yaml",
    ])
    controller_params = RewrittenYaml(
        source_file=controller_params_file,
        root_key=namespace,
        param_rewrites={
            # 'enable_odom': LaunchConfiguration('enable_odom'),
            # 'enable_odom_tf': PythonExpression([
            #     "'false' if ('",
            #     LaunchConfiguration('enable_odom_tf'),
            #     "' == 'false' or '",
            #     LaunchConfiguration('use_ekf'),
            #     "' == 'true') else 'true'"
            # ])
        },
        convert_types=True
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[
            controller_params,
        ],
        remappings=[
            ('~/robot_description', 'robot_description'),
        ],
        output='both',
    )

    spawn_joint_state_broadcaster_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        parameters=[
            controller_params,
        ],
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_robot_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        parameters=[
            controller_params,
        ],
        arguments=["scaled_joint_trajectory_controller"],
        output="screen",
    )

    # Delay start of robot_drive_controller after `joint_state_broadcaster`
    delay_robot_drive_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_controller,
            on_exit=[spawn_robot_drive_controller],
        )
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("whi_arm_interface"), "launch", "rviz_config.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_controller,
            on_exit=[rviz_node],
        )
    )

    launch_nodes = [
        node_robot_state_publisher,
        control_node,
        spawn_joint_state_broadcaster_controller,
        delay_robot_drive_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
    ]

    return launch_nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='',
            description='top-level namespace'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument("arm", default_value="jaka",
            description="the arm series"),
        DeclareLaunchArgument("arm_model", default_value="zu5",
            description="the arm model"),
        DeclareLaunchArgument("start_rviz", default_value="false",
            description="start RViz for visualization"),
        OpaqueFunction(function=launch_setup)
    ])
