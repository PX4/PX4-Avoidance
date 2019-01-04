#!/bin/bash 


# Disable and enable auto-exposure for all cameras as it doesn not work at startup
rosrun dynamic_reconfigure dynparam set /camera_front/realsense2_camera_manager rs435_depth_enable_auto_exposure 0
rosrun dynamic_reconfigure dynparam set /camera_front/realsense2_camera_manager rs435_depth_enable_auto_exposure 1
rosrun dynamic_reconfigure dynparam set /camera_left/realsense2_camera_manager rs435_depth_enable_auto_exposure 0
rosrun dynamic_reconfigure dynparam set /camera_left/realsense2_camera_manager rs435_depth_enable_auto_exposure 1
rosrun dynamic_reconfigure dynparam set /camera_right/realsense2_camera_manager rs435_depth_enable_auto_exposure 0
rosrun dynamic_reconfigure dynparam set /camera_right/realsense2_camera_manager rs435_depth_enable_auto_exposure 1

# Set the depth visual preset to HIGH_ACCURACY mode for all cameras
#rosrun dynamic_reconfigure dynparam set /camera_front/realsense2_camera_manager rs435_depth_visual_preset 3
#rosrun dynamic_reconfigure dynparam set /camera_left/realsense2_camera_manager rs435_depth_visual_preset 3
#rosrun dynamic_reconfigure dynparam set /camera_right/realsense2_camera_manager rs435_depth_visual_preset 3
