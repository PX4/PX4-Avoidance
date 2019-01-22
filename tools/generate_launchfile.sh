#!/bin/bash
cat > local_planner/launch/avoidance.launch <<- EOM
<launch>


    <arg name="mavros_transformation" default="0" />
    
    <param name="use_sim_time" value="false" />

    <node pkg="tf" type="static_transform_publisher" name="tf_90_deg"
          args="0 0 0 \$(arg mavros_transformation) 0 0 world local_origin 10"/>
                  
    <arg name="headless" default="false"/>
    <arg name="ns" default="/"/>
    <arg name="build" default="px4_sitl_default"/>
    <arg name="fcu_url" default="udp://:14540@localhost:14557"/>
    <arg name="gcs_url" default="" />   <!-- GCS link is provided by SITL -->
    <arg name="tgt_system" default="1" />
    <arg name="tgt_component" default="1" />
    <arg name="est" default="ekf2"/>
    <arg name="depth_fps"            default="30"/>
    <arg name="infra1_fps"           default="30"/>
    <arg name="infra2_fps"           default="30"/>

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

# The CAMERA_CONFIGS string has semi-colon separated camera configurations
IFS=";"
for camera in $CAMERA_CONFIGS; do
	IFS="," #Inside each camera configuration, the parameters are comma-separated
	set $camera
	if [[ $# != 8 ]]; then
		echo "Invalid camera configuration $camera"
	else
		echo "Adding camera $1 with serial number $2"
		if [[ $camera_topics == "" ]]; then
			camera_topics="/$1/depth/points"
		else
			camera_topics="$camera_topics,/$1/depth/points"
		fi
		cat >> local_planner/launch/avoidance.launch <<- EOM
			<node pkg="tf" type="static_transform_publisher" name="tf_$1"
			 args="$3 $4 $5 $6 $7 $8 fcu_$1_link 10"/>
			<include file="\$(find local_planner)/launch/rs_depthcloud.launch">
				<arg name="namespace"             value="$1" />
				<arg name="tf_prefix"             value="$1" />
				<arg name="serial_no"             value="$2"/>
				<arg name="depth_fps"             value="\$(arg depth_fps)"/>
				<arg name="infra1_fps"            value="\$(arg infra1_fps)"/>
				<arg name="infra2_fps"            value="\$(arg infra2_fps)"/>
				<arg name="enable_pointcloud"     value="false"/>      
				<arg name="enable_imu"            value="false"/>
				<arg name="enable_fisheye"        value="false"/>
			</include>
		EOM
	fi
done

cat >> local_planner/launch/avoidance.launch <<- EOM
    <!-- Launch avoidance -->
    <env name="ROSCONSOLE_CONFIG_FILE" value="\$(find local_planner)/resource/custom_rosconsole.conf"/>
    <rosparam command="load" file="\$(find local_planner)/cfg/params.yaml"/>
    <arg name="pointcloud_topics" default="[$camera_topics]"/>

    <node name="local_planner_node" pkg="local_planner" type="local_planner_node" output="screen" >
      <param name="goal_x_param" value="15" />
      <param name="goal_y_param" value="15"/>
      <param name="goal_z_param" value="4" />
      <rosparam param="pointcloud_topics" subst_value="True">\$(arg pointcloud_topics)</rosparam> 
    </node>
    
    <!-- switch off and on auto exposure of Realsense cameras, as it does not work on startup -->
    <node name="set_RS_param" pkg="local_planner" type="realsense_params.sh" />

</launch>
EOM
