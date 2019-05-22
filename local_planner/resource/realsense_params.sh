#!/bin/bash
# Disable and enable auto-exposure for all cameras as it does not work at startup
rosrun dynamic_reconfigure dynparam set /camera_front/stereo_module enable_auto_exposure 0
rosrun dynamic_reconfigure dynparam set /camera_front/stereo_module enable_auto_exposure 1
