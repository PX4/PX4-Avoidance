# Running on Odroid

## Installation

This has only been tested with **Ubuntu 14.04** and  **ROS-Indigo**.
Shoud also work with **Ubuntu 16.04** and **ROS-Kinetic**, 
but keep in mind that some ROS-topics will not be compatible between different ROS-versions

**Install the following ROS-packages**

- PX4/Avoidance
	- odroid branch
- PX4/uvc_ros_driver
	- half_resolution branch
- PX4/disparity_to_point_cloud
	- horizontal_filter branch 		(For just the horizontal pairs)
	- camera_triplet_odroid branch	(For fusing horizontal and vertical pairs)
- OctoMap/octomap_mapping
- mavlink/mavros

**Running**

Add the ip address of the odroid to */etc/hosts* under the name odroid.

Also make sure to share keys with the odroid such that it is possible to login with ssh without typing a password

Run the [runOdroid](https://github.com/PX4/avoidance/blob/master/resource/odroid/runOdroid) 
script from a laptop to connect to the odroid and launch the ROS-nodes. 
Set a '2D Nav Goal' in rviz to send a goal to the planner, you should now see the planned path in rviz.
