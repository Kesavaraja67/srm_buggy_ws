#!/usr/bin/env python3
"""
teleop.launch.py — SRM Autonomous Buggy
─────────────────────────────────────────────────
Phase 1: Manual keyboard control of the buggy.
Launches Gazebo + robot ONLY (no brain/autonomous nodes).

Controls (teleop_twist_keyboard):
  i        = forward
  ,        = backward
  j        = turn left
  l        = turn right
  k        = full STOP
  q/z      = increase/decrease max speed
  w/x      = increase/decrease linear speed only

Usage:
  ros2 launch buggy_bringup teleop.launch.py
"""
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    bringup_pkg     = get_package_share_directory('buggy_bringup')
    description_pkg = get_package_share_directory('buggy_description')
    gazebo_ros_pkg  = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(bringup_pkg, 'worlds', 'srm_campus.world')
    xacro_file = os.path.join(description_pkg, 'urdf', 'buggy_urdf.xacro')

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str
    )

    declare_gui     = DeclareLaunchArgument('gui',     default_value='true')
    declare_verbose = DeclareLaunchArgument('verbose', default_value='false')
    gui     = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')

    # ── 1. Gazebo world ───────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'gui': gui, 'verbose': verbose}.items(),
    )

    # ── 2. Robot state publishers ──────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}, {'use_sim_time': True}],
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

    # ── 3. Visualizer for steering joints ──────────────────────
    steering_visualizer = Node(
        package='buggy_brain',
        executable='steering_visualizer',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── 3. Spawn buggy at bay exit ─────────────────────────────
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'srm_aquila_buggy',
            '-x', '-9.0',    # Bay exit — clear forward view
            '-y',  '0.0',
            '-z',  '0.0',
            '-Y',  '0.0',   # Facing East
        ],
        output='screen',
    )

    # ── 4. Arrow-key teleop — xterm runs wrapper shell script ──────────────────
    # arrow_teleop is a Python script; xterm cannot exec Python scripts directly.
    # The shell wrapper sources ROS env + calls python3 arrow_teleop.py.
    wrapper = os.path.join(bringup_pkg, '..', '..', '..', '..',
                           'src', 'buggy_bringup', 'scripts', 'run_arrow_teleop.sh')
    wrapper = os.path.normpath(wrapper)
    # Fall back to src path if install symlink not present
    wrapper_src = os.path.expanduser(
        '~/workspaces/srm_buggy_ws/src/buggy_bringup/scripts/run_arrow_teleop.sh')
    if not os.path.exists(wrapper) and os.path.exists(wrapper_src):
        wrapper = wrapper_src
    teleop = ExecuteProcess(
        cmd=['xterm', '-e', 'bash', wrapper],
        output='screen',
    )

    return LaunchDescription([
        declare_gui,
        declare_verbose,
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        teleop,
        steering_visualizer,
    ])
