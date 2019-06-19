#!/bin/bash
# Disable and enable auto-exposure for all cameras as it does not work at startup
rosrun dynamic_reconfigure dynparam set /sc_node depth_range_mode 3
