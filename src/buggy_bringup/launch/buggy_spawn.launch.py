#!/usr/bin/env python3
"""
buggy_spawn.launch.py  v3.0 — SRM Autonomous Buggy
────────────────────────────────────────────────────
Spawns buggy at BUGGY_HUB roundabout (0, 0)
Facing North (+Y) toward SRM_IST (0, 50)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    description_pkg   = get_package_share_directory('buggy_description')
    xacro_file        = os.path.join(description_pkg, 'urdf', 'buggy_urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ],
        output='screen',
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_srm_buggy',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'srm_aquila_buggy',
            '-x', '-11.0',   # inside buggy shelter bay, just off roundabout
            '-y', '0.0',
            '-z', '0.425',
            '-Y', '1.5708',  # facing North toward SRM_IST
        ],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
    ])