#!/usr/bin/env python3
"""
Launch PX4 SITL with Gazebo
"""

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir


def generate_launch_description():
    parameters_file_path = os.path.dirname(os.path.realpath(__file__))

    return LaunchDescription([
        # Define a static transform from a camera internal frame to the fcu for every camera used
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            name='tf_depth_camera',
            arguments=['0', '0', '0', '-1.57', '0',
                       '-1.57', 'fcu camera_link', '10'],
            output='screen'
        ),

        # Launch PX4 SITL and Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/avoidance_sitl.launch.py'])
        ),

        # Launch stereo_image_proc node which runs OpenCV's block matching
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('stereo_image_proc'), 'launch', 'stereo_image_proc.launch.py'))
        )
    ])
