#!/bin/bash

cat > local_planner/launch/avoidance.launch <<- EOM
<launch>
    <arg name="ns" default="/"/>
    <arg name="fcu_url" default="udp://:14540@localhost:14557"/>
    <arg name="gcs_url" default="" />   <!-- GCS link is provided by SITL -->
    <arg name="tgt_system" default="1" />
    <arg name="tgt_component" default="1" />

    <!-- Launch MavROS -->
    <group ns="\$(arg ns)">
        <include file="\$(find mavros)/launch/node.launch">
            <arg name="pluginlists_yaml" value="\$(find mavros)/launch/px4_pluginlists.yaml" />
            <!-- Need to change the config file to get the tf topic and get local position in terms of local origin -->
            <arg name="config_yaml" value="\$(find local_planner)/resource/px4_config.yaml" />
            <arg name="fcu_url" value="\$(arg fcu_url)" />
            <arg name="gcs_url" value="\$(arg gcs_url)" />
            <arg name="tgt_system" value="\$(arg tgt_system)" />
            <arg name="tgt_component" value="\$(arg tgt_component)" />
        </include>
    </group>

    <!-- Launch cameras -->
EOM

# Fix the on/of script for realsense auto-exposure
cat > local_planner/resource/realsense_params.sh <<- EOM
#!/bin/bash
# Disable and enable auto-exposure for all cameras as it does not work at startup
EOM

# Set the frame rate to 15 if it is undefined
if [ -z $DEPTH_CAMERA_FRAME_RATE ]; then
  DEPTH_CAMERA_FRAME_RATE=15
fi

REALSENSE_CAMERA_USED=0
# The CAMERA_CONFIGS string has semi-colon separated camera configurations
IFS=";"
for camera in $CAMERA_CONFIGS; do
	IFS="," #Inside each camera configuration, the parameters are comma-separated
	set $camera
	if [[ $# != 9 ]]; then
		echo "Invalid camera configuration $camera"
	else
		echo "Adding camera $1 of type $2 with serial number $3"
		if [[ $camera_topics == "" ]]; then
			camera_topics="/$1/depth/points"
		else
			camera_topics="$camera_topics,/$1/depth/points"
		fi

    # Append to the launch file
    
    if  [[ $2 == "realsense" ]]; then
    REALSENSE_CAMERA_USED=1
    cat >>    local_planner/launch/avoidance.launch <<- EOM
			    <node pkg="tf" type="static_transform_publisher" name="tf_$1"
			       args="$4 $5 $6 $7 $8 $9 fcu $1_link 10"/>
			    <include file="\$(find local_planner)/launch/rs_depthcloud.launch">
			       <arg name="namespace"             value="$1" />
			       <arg name="tf_prefix"             value="$1" />
			       <arg name="serial_no"             value="$3"/>
			       <arg name="depth_fps"             value="$DEPTH_CAMERA_FRAME_RATE"/>
			       <arg name="enable_pointcloud"     value="false"/>
			       <arg name="enable_fisheye"        value="false"/>
			    </include>
            
		EOM
		
		# Append to the realsense auto exposure toggling
		echo "rosrun dynamic_reconfigure dynparam set /$1/stereo_module enable_auto_exposure 0
		rosrun dynamic_reconfigure dynparam set /$1/stereo_module enable_auto_exposure 1
		" >> local_planner/resource/realsense_params.sh
		
	elif  [[ $2 == "struct_core" ]]; then
	
	cat >>    local_planner/launch/avoidance.launch <<- EOM
			    <node pkg="tf" type="static_transform_publisher" name="tf_$1"
			       args="$4 $5 $6 $7 $8 $9 fcu $1_FLU 10"/>
			    
			    <rosparam command="load" file="\$(find struct_core_ros)/launch/sc.yaml"/>
			    <node pkg="struct_core_ros" type="sc" name="$1"/>
            
		EOM
	else
	echo "Unknown camera type $2 in CAMERA_CONFIGS"
	fi
	fi
done

if [ ! -z $VEHICLE_CONFIG ]; then
cat >> local_planner/launch/avoidance.launch <<- EOM
    <node name="dynparam" pkg="dynamic_reconfigure" type="dynparam" args="load local_planner_node $VEHICLE_CONFIG" />
EOM
echo "Adding vehicle paramters: $VEHICLE_CONFIG"
fi

cat >> local_planner/launch/avoidance.launch <<- EOM
    <!-- Launch avoidance -->
    <env name="ROSCONSOLE_CONFIG_FILE" value="\$(find local_planner)/resource/custom_rosconsole.conf"/>
    <arg name="pointcloud_topics" default="[$camera_topics]"/>

    <node name="local_planner_node" pkg="local_planner" type="local_planner_node" output="screen" >
      <param name="goal_x_param" value="0" />
      <param name="goal_y_param" value="0"/>
      <param name="goal_z_param" value="4" />
      <rosparam param="pointcloud_topics" subst_value="True">\$(arg pointcloud_topics)</rosparam>
    </node>
    
EOM

if  [[ $REALSENSE_CAMERA_USED == 1 ]]; then
cat >>    local_planner/launch/avoidance.launch <<- EOM
    <!-- switch off and on auto exposure of Realsense cameras, as it does not work on startup -->
    <node name="set_RS_param" pkg="local_planner" type="realsense_params.sh" />
    
EOM
fi

cat >> local_planner/launch/avoidance.launch <<- EOM
</launch>
EOM

# Set the frame rate in the JSON file as well
sed -i '/stream-fps/c\    \"stream-fps\": \"'$DEPTH_CAMERA_FRAME_RATE'\",' local_planner/resource/stereo_calib.json
