#!/usr/bin/env python

import os.path as osp

from launch import LaunchDescription, logging
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import \
    PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import LaunchConfiguration, PythonExpression

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():

    camera_frame_name = 'camera_frame'  # Realsense camera frame name

    configurable_parameters_realsense = [{'name': 'camera_name',                  'default': 'camera', 'description': 'camera unique name'},
                           {'name': 'serial_no',                    'default': '', 'description': 'choose device by serial number'},
                           {'name': 'usb_port_id',                  'default': '', 'description': 'choose device by usb port id'},
                           {'name': 'device_type',                  'default': '', 'description': 'choose device by type'},
                           {'name': 'config_file',                  'default': '', 'description': 'yaml config file'},
                           {'name': 'enable_pointcloud',            'default': 'true', 'description': 'enable pointcloud'},
                           {'name': 'unite_imu_method',             'default': '', 'description': '[copy|linear_interpolation]'},
                           {'name': 'json_file_path',               'default': '', 'description': 'allows advanced configuration'},
                           {'name': 'output',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},
                           {'name': 'depth_width',                  'default': '256', 'description': 'depth image width'},
                           {'name': 'depth_height',                 'default': '144', 'description': 'depth image height'},
                           {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'color_width',                  'default': '1280', 'description': 'color image width'},
                           {'name': 'color_height',                 'default': '720', 'description': 'color image height'},
                           {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'infra_width',                  'default': '640', 'description': 'infra width'},
                           {'name': 'infra_height',                 'default': '480', 'description': 'infra width'},
                           {'name': 'enable_infra1',                'default': 'false', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra2',                'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'infra_rgb',                    'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'fisheye_width',                'default': '848', 'description': 'fisheye width'},
                           {'name': 'fisheye_height',               'default': '800', 'description': 'fisheye width'},
                           {'name': 'enable_fisheye1',              'default': 'false', 'description': 'enable fisheye1 stream'},
                           {'name': 'enable_fisheye2',              'default': 'false', 'description': 'enable fisheye2 stream'},
                           {'name': 'confidence_width',             'default': '640', 'description': 'depth image width'},
                           {'name': 'confidence_height',            'default': '480', 'description': 'depth image height'},
                           {'name': 'enable_confidence',            'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'fisheye_fps',                  'default': '30.', 'description': ''},
                           {'name': 'depth_fps',                    'default': '90.', 'description': ''},
                           {'name': 'confidence_fps',               'default': '30.', 'description': ''},
                           {'name': 'infra_fps',                    'default': '30.', 'description': ''},
                           {'name': 'color_fps',                    'default': '30.', 'description': ''},
                           {'name': 'gyro_fps',                     'default': '400.', 'description': ''},
                           {'name': 'accel_fps',                    'default': '250.', 'description': ''},
                           {'name': 'enable_gyro',                  'default': 'false', 'description': ''},
                           {'name': 'enable_accel',                 'default': 'false', 'description': ''},
                           {'name': 'pointcloud_texture_stream',    'default': 'RS2_STREAM_COLOR', 'description': 'testure stream for pointcloud'},
                           {'name': 'pointcloud_texture_index',     'default': '0', 'description': 'testure stream index for pointcloud'},
                           {'name': 'enable_sync',                  'default': 'false', 'description': ''},
                           {'name': 'align_depth',                  'default': 'false', 'description': ''},
                           {'name': 'filters',                      'default': '', 'description': ''},
                           {'name': 'clip_distance',                'default': '-2.', 'description': ''},
                           {'name': 'linear_accel_cov',             'default': '0.01', 'description': ''},
                           {'name': 'initial_reset',                'default': 'false', 'description': ''},
                           {'name': 'allow_no_texture_points',      'default': 'false', 'description': ''},
                           {'name': 'calib_odom_file',              'default': '', 'description': ''},
                           {'name': 'topic_odom_in',                'default': '', 'description': 'topic for T265 wheel odometry'},
                          ]

    realsense2_node = LaunchDescription(declare_configurable_parameters(configurable_parameters_realsense) + [
        # Realsense
        Node(
            package='realsense2_camera',
            namespace=LaunchConfiguration("camera_name"),
            name=LaunchConfiguration("camera_name"),
            executable='realsense2_camera_node',
            parameters = [set_configurable_parameters(configurable_parameters_realsense)],
            output='screen',
            emulate_tty=True,
            )
    ])

    tf2_static_pub_node = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='tf_depth_camera',
                arguments=['0', '0', '0',
                           '-1.57', '0', '-1.57',
                           'local_origin_odom', camera_frame_name],
                output='screen')

    tf2_static_pub_node2 = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='tf_frd_ned',
                arguments=['0', '0', '0',
                           '1.57', '0', '3.14',
                          'base_frame', 'base_frame_ned'],
                output='screen')

    rviz2_node = Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', '/home/karidrone/git/global_planner_ws/src/PX4-global-planner-ros2/global_planner/resources/global_planner.rviz'])

    gp_params = {'frame_id': 'base_frame',
                 'agent_number': 14,
                 'position_mode': 'local_position',
                 'world_path': '/home/user/git/global_planner_ws/src/PX4-global-planner-ros2/avoidance/sim/worlds/simple_obstacle.yaml',
                 'pointcloud_topics': ['/camera/depth/color/points'],
                 'start_pos_x': 0.0,
                 'start_pos_y': 0.0,
                 'start_pos_z': 3.0,
                 'min_altitude': 1,
                 'max_altitude': 5,
                 'max_cell_risk': 0.5,
                 'smooth_factor': 10.0,
                 'vert_to_hor_cost': 1.0,
                 'risk_factor': 500.0,
                 'neighbor_risk_flow': 1.0,
                 'explore_penalty': 0.005,
                 'up_cost': 3.0,
                 'down_cost': 1.0,
                 'search_time': 0.5,
                 'min_overestimate_factor': 1.03,
                 'max_overestimate_factor': 2.0,
                 'risk_threshold_risk_based_speedup': 0.5,
                 'default_speed': 1.0,
                 'max_speed': 3.0,
                 'max_iterations': 2000,
                 'goal_is_blocked': False,
                 'current_cell_blocked': False,
                 'goal_must_be_free': True,
                 'use_current_yaw': True,
                 'use_risk_heuristics': True,
                 'use_speedup_heuristics': True,
                 'use_risk_based_speedup': True}
    
    gp_node = Node(package='global_planner',
                 executable='global_planner_node',
                 output='screen',
                 parameters=[gp_params])

    octomap_params = {'resolution': 1.0,
              'frame_id': 'base_frame',
              'base_frame_id': 'base_footprint',
              'height_map': True,
              'colored_map': False,
              'color_factor': 0.8,
              'filter_ground': False,
              'filter_speckles': False,
              'compress_map': True,
              'incremental_2D_projection': False,
              'sensor_model/max_range': 6.0,
              'sensor_model/hit': 0.9,
              'sensor_model/miss': 0.45,
              'sensor_model/min': 0.01,
              'sensor_model/max': 0.99,
              'pointcloud_max_x': 100.0,
              'pointcloud_max_y': 100.0,
              'pointcloud_max_z': 100.0,
              'pointcloud_min_x': -100.0,
              'pointcloud_min_y': -100.0,
              'pointcloud_min_z': -100.0,
              'occupancy_min_z': 0.5,
              'color/r': 0.0,
              'color/g': 0.0,
              'color/b': 1.0,
              'color/a': 1.0,
              'color_free/r': 0.0,
              'color_free/g': 0.0,
              'color_free/b': 1.0,
              'color_free/a': 1.0,
              'publish_free_space': False,
    }

    # octomap_remap = [('cloud_in', '/camera/points')]
    
    octomap_node = Node(package='octomap_server2',
                 executable='octomap_server',
                 output='log',
                 # remappings=octomap_remap,
                 parameters=[octomap_params])
    return LaunchDescription([realsense2_node, tf2_static_pub_node, tf2_static_pub_node2, octomap_node, gp_node]) #, rviz2_node])
