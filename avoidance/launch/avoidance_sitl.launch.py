#!/usr/bin/env python3
"""
Launch PX4 SITL with Gazebo
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    parameters_file_path = os.path.dirname(os.path.realpath(__file__))

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='iris_depth_camera'),
        DeclareLaunchArgument('vehicle', default_value='iris_obs_avoid'),

        # Launch PX4 SITL
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('px4'), 'launch', 'px4.launch.py'))
        ),

        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'))
        ),

        # Spawn the model
        Node(
            package='gazebo_ros',
            node_executable='spawn_entity.py',
            arguments=[
                '-entity', LaunchConfiguration('vehicle'), '-database', LaunchConfiguration('model')],
            output='screen'
        )
    ])
