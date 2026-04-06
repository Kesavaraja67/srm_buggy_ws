#!/usr/bin/env python3
"""
slam_mapping.launch.py — SRM Autonomous Buggy
──────────────────────────────────────────────────────────
Phase 2: Drive manually while slam_toolbox builds a real LiDAR map.

Steps:
  1. Launch this file
  2. Drive with the keyboard in the xterm window (i/j/l/k keys)
  3. Watch the map grow in RViz2
  4. When campus coverage is good → save the map:
       ros2 run nav2_map_server map_saver_cli -f ~/srm_campus_map

Usage:
  ros2 launch buggy_bringup slam_mapping.launch.py
"""
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    bringup_pkg     = get_package_share_directory('buggy_bringup')
    description_pkg = get_package_share_directory('buggy_description')
    gazebo_ros_pkg  = get_package_share_directory('gazebo_ros')

    world_file      = os.path.join(bringup_pkg,     'worlds',  'srm_campus.world')
    xacro_file      = os.path.join(description_pkg, 'urdf',    'buggy_urdf.xacro')
    slam_config     = os.path.join(bringup_pkg,     'config',  'slam_config.yaml')
    rviz_config     = os.path.join(bringup_pkg,     'config',  'rviz_slam.rviz')

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str
    )

    declare_gui     = DeclareLaunchArgument('gui',     default_value='true')
    declare_verbose = DeclareLaunchArgument('verbose', default_value='false')

    # ── 1. Gazebo world ───────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world':   world_file,
            'gui':     LaunchConfiguration('gui'),
            'verbose': LaunchConfiguration('verbose'),
        }.items(),
    )

    # ── 2. Robot description ──────────────────────────────────
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

    # ── 3. Spawn buggy ────────────────────────────────────────
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'srm_aquila_buggy',
            '-x', '-9.0', '-y', '0.0', '-z', '0.0', '-Y', '0.0',
        ],
        output='screen',
    )

    # ── 4. SLAM Toolbox — builds map from /scan + /odom ───────
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, {'use_sim_time': True}],
    )

    # ── 5. RViz2 — shows map being built ──────────────────────
    # Delayed by 5s so Gazebo initialises first
    rviz2 = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
                output='screen',
            )
        ],
    )

    # ── 6. Arrow-key teleop — xterm runs wrapper shell script ──────────────────
    wrapper = os.path.join(bringup_pkg, '..', '..', '..', '..',
                           'src', 'buggy_bringup', 'scripts', 'run_arrow_teleop.sh')
    wrapper = os.path.normpath(wrapper)
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
        slam_toolbox,
        rviz2,
        teleop,
        steering_visualizer,
    ])
