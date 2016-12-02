#ifndef GLOBAL_PLANNER_LOCAL_PLANNER_NODE_H
#define GLOBAL_PLANNER_LOCAL_PLANNER_NODE_H

#include <iostream>
#include <math.h> 


#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h> // transformPointCloud
#include <pcl_conversions/pcl_conversions.h>  // fromROSMsg
#include <pcl_ros/point_cloud.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h> 

#include "local_planner.h"


class LocalPlannerNode{

public: 
	LocalPlannerNode();
	~LocalPlannerNode();

private:
	ros::NodeHandle nh_;
	LocalPlanner local_planner;

	nav_msgs::Path path_actual;
    nav_msgs::Path path_ideal;
    visualization_msgs::MarkerArray marker_array;
	geometry_msgs::PoseArray pose_array;

	int i = 0 ;

	//subscribers
	ros::Subscriber pointcloud_sub_ ;
	ros::Subscriber pose_sub_ ; 


	//publishers
	ros::Publisher pose_array_pub_;
	ros::Publisher local_pointcloud_pub_;
	ros::Publisher front_pointcloud_pub_;
	ros::Publisher cached_pointcloud_pub_ ;
	ros::Publisher path_candidates_pub_;
    ros::Publisher path_rejected_pub_;
    ros::Publisher path_blocked_pub_;
	ros::Publisher path_selected_pub_;
	ros::Publisher marker_pub_;

    tf::TransformListener tf_listener_;

    void positionCallback(const geometry_msgs::PoseStamped input);
	void pointCloudCallback(const sensor_msgs::PointCloud2 input);
	void publishAll();
	void fillPath(const geometry_msgs::PoseStamped);
	void publishMarker();


};


#endif // GLOBAL_PLANNER_LOCAL_PLANNER_NODE_H
