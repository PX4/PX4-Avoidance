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

    rs2_node_param = [{'frame_name': camera_frame_name},
                      {'depth_width': 480},
                      {'depth_height': 270},
                      {'depth_fps': 5}]
 
    realsense2_node = Node(package='rs2_light_wrapper',
            executable='rs2_light_node',
            parameters = rs2_node_param,
            output='screen',
            )

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

    gp_params = {'frame_id': 'base_frame',
                 'position_mode': 'local_position',
                 'pointcloud_topics': ['/rs2_pc'],
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
                 'max_speed': 2.0,
                 'max_iterations': 1000,
                 'goal_is_blocked': False,
                 'current_cell_blocked': False,
                 'goal_must_be_free': True,
                 'use_current_yaw': True,
                 'use_risk_heuristics': True,
                 'use_speedup_heuristics': True,
                 'use_risk_based_speedup': False}

    # Remapping rules for using other types of topic name instead of default PubSubTopic names
    agent_id = 14
    gp_remap = [('/VehicleAttitude_PubSubTopic', '/agent{}/vehicle_attitude'.format(agent_id)),
            ('/VehicleLocalPosition_PubSubTopic', '/agent{}/vehicle_local_position'.format(agent_id)),
            ('/VehicleGlobalPosition_PubSubTopic','/agent{}/vehicle_global_position'.format(agent_id)),
            ('/VehicleStatus_PubSubTopic', '/agent{}/vehicle_status'.format(agent_id)),
            ('/VehicleCommand_PubSubTopic','/agent{}/vehicle_command'.format(agent_id))]
    
    gp_node = Node(package='global_planner',
                 executable='global_planner_node',
                 output='screen',
                 remappings = gp_remap,
                 parameters=[gp_params])

    octomap_params = {'resolution': 0.5,
              'frame_id': 'base_frame',
              'base_frame_id': 'base_frame',
              'height_map': True,
              'colored_map': False,
              'color_factor': 0.8,
              'filter_ground': True,
              'filter_speckles': False,
              'compress_map': True,
              'incremental_2D_projection': False,
              'sensor_model/min_range': 0.5,
              'sensor_model/max_range': 5.0,
              'sensor_model/hit': 0.7,
              'sensor_model/miss': 0.4,
              'sensor_model/min': 0.2,
              'sensor_model/max': 0.7,
              'pointcloud_max_x': 50.0,
              'pointcloud_max_y': 50.0,
              'pointcloud_max_z': 50.0,
              'pointcloud_min_x': -50.0,
              'pointcloud_min_y': -50.0,
              'pointcloud_min_z': -50.0,
              'occupancy_min_z': 0.3,
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

    octomap_node = Node(package='octomap_server2',
                 executable='octomap_server',
                 output='log',
                 parameters=[octomap_params])
    return LaunchDescription([realsense2_node, tf2_static_pub_node, tf2_static_pub_node2, octomap_node, gp_node]) #, rviz2_node])
