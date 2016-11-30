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

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include "local_planner.h"


class LocalPlannerNode{

public: 
	LocalPlannerNode();
	~LocalPlannerNode();

private:
	ros::NodeHandle nh_;
	LocalPlanner local_planner;

	//subscribers
	ros::Subscriber pointcloud_sub_ ;
	ros::Subscriber pose_sub_ ; 


	//publishers
	ros::Publisher local_pointcloud_pub_;
	ros::Publisher front_pointcloud_pub_;

    tf::TransformListener tf_listener_;

    void positionCallback(const geometry_msgs::PoseStamped input);
	void pointCloudCallback(const sensor_msgs::PointCloud2 input);
};


#endif // GLOBAL_PLANNER_LOCAL_PLANNER_NODE_H
