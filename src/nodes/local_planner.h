#ifndef GLOBAL_PLANNER_LOCAL_PLANNER_H
#define GLOBAL_PLANNER_LOCAL_PLANNER_H

#include <iostream>
#include <math.h> 

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_ros/conversions.h>

class LocalPlanner{

public:
	pcl::PointCloud<pcl::PointXYZ> final_cloud;

	LocalPlanner();
	~LocalPlanner();

private:

	bool obstacle;

	void filterPointCloud(pcl::PointCloud<pcl::PointXYZ>& );

}

#endif // GLOBAL_PLANNER_LOCAL_PLANNER_H