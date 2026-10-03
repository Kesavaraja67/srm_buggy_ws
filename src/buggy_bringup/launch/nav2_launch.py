#!/usr/bin/env python3
"""
nav2_launch.py — SRM Autonomous Buggy
──────────────────────────────────────────────────
Launches Gazebo + Nav2 stack with the saved LiDAR map.
Uses AMCL for localization and Nav2 for path planning + control.

Usage:
  ros2 launch buggy_bringup nav2_launch.py

After launch (~40 seconds):
  An xterm window will appear asking you to pick a destination (A/B/C/D).
  The buggy will then navigate autonomously using the LiDAR map + AMCL + Nav2!
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    bringup_pkg     = get_package_share_directory('buggy_bringup')
    description_pkg = get_package_share_directory('buggy_description')
    gazebo_ros_pkg  = get_package_share_directory('gazebo_ros')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    world_file = os.path.join(bringup_pkg, 'worlds', 'srm_campus.world')
    xacro_file = os.path.join(description_pkg, 'urdf', 'buggy_urdf.xacro')
    map_file   = os.path.join(bringup_pkg, 'maps', 'my_campus_map.yaml')
    nav2_params_file = os.path.join(bringup_pkg, 'config', 'nav2_params.yaml')

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str
    )

    # ── Launch arguments ──────────────────────────────────────
    declare_gui     = DeclareLaunchArgument('gui',     default_value='true')
    declare_verbose = DeclareLaunchArgument('verbose', default_value='false')
    gui     = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')

    # ── 1. Gazebo world ───────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'gui':   gui,
            'verbose': verbose,
        }.items(),
    )

    # ── 2. Robot description ──────────────────────────────────
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
        parameters=[{
            'use_sim_time': True,
            'source_list': ['steering_joints']
        }],
    )

    # ── 3. Steering visualizer ────────────────────────────────
    steering_visualizer = Node(
        package='buggy_brain',
        executable='steering_visualizer',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── 4. Spawn buggy ────────────────────────────────────────
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'srm_aquila_buggy',
            '-x', '-13.0',
            '-y',  '0.0',
            '-z',  '0.2',
            '-Y',  '0.0',
        ],
        output='screen',
    )

    # ── 5. Nav2 bringup (map_server + AMCL + planner + controller) ─
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params_file,
            'use_sim_time': 'True',
            'autostart': 'True',
            'use_composition': 'True',
        }.items(),
    )

    # ── 6. RViz2 with Nav2 config ─────────────────────────────
    rviz_config = os.path.join(nav2_bringup_pkg, 'rviz', 'nav2_default_view.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ── 7. Nav2 Destination Sender — runs in xterm for interactive input ──
    # Uses xterm so stdin works properly and the menu is always visible.
    # Delayed 40s so Nav2 + AMCL are fully active before user picks destination.
    nav2_destination_sender_xterm = ExecuteProcess(
        cmd=[
            'xterm',
            '-title', 'SRM Buggy — Select Destination',
            '-geometry', '60x20',
            '-fa', 'Monospace',
            '-fs', '12',
            '-e',
            'bash', '-c',
            'source /opt/ros/humble/setup.bash && '
            'source ~/workspaces/srm_buggy_ws/install/setup.bash && '
            'ros2 run buggy_brain nav2_destination_sender; '
            'echo "Press Enter to close..."; read',
        ],
        output='screen',
    )

    # Add actions to LaunchDescription
    ld = LaunchDescription()

    ld.add_action(declare_gui)
    ld.add_action(declare_verbose)

    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(steering_visualizer)

    # Wait 5s for Gazebo/Robot to settle, then spawn buggy
    ld.add_action(TimerAction(
        period=5.0,
        actions=[spawn_entity]
    ))

    # Wait 15s for spawn to complete, then start Nav2
    ld.add_action(TimerAction(
        period=15.0,
        actions=[nav2_bringup]
    ))

    # Wait 25s for Nav2 to be active, then open RViz
    ld.add_action(TimerAction(
        period=25.0,
        actions=[rviz]
    ))

    # Wait 60s for everything to be ready, then show destination menu
    ld.add_action(TimerAction(
        period=60.0,
        actions=[nav2_destination_sender_xterm]
    ))

    return ld
