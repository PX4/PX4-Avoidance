#ifndef GLOBAL_PLANNER_LOCAL_PLANNER_H
#define GLOBAL_PLANNER_LOCAL_PLANNER_H

#include <iostream>
#include <math.h> 

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <nav_msgs/GridCells.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_ros/conversions.h>

#include <opencv2/imgproc/imgproc.hpp>


#define PI 3.14159265
#define alpha_res 10
#define grid_length 360/alpha_res

float distance2d(geometry_msgs::Point a, geometry_msgs::Point b);

class Histogram 
{ 
	int array[grid_length][grid_length];

	public: 

		Histogram() {}
		
		~Histogram() {} 
		
		int get( int x, int y ) const {
		 	return array[x][y]; 
		} 

		void set( int x, int y , int value) {
		 	array[x][y] = value; 
		}

		int setZero() {
			memset(array, 0, sizeof(array)); 
		} 
};

class LocalPlanner {

public:
	pcl::PointCloud<pcl::PointXYZ> final_cloud;
	
	octomap::Pointcloud octomapCloud;

	bool obstacle;
	bool first_brake = false;
	bool braking_param = false;  

	geometry_msgs::Point pose, min, max, min_cache, max_cache, front, back, half_cache, goal, obs, stop_pose;
	geometry_msgs::Point p1;
	geometry_msgs::Vector3Stamped waypt;

	ros::Time last_pose_time;

	int init = 0;

	float min_x = 1.5, max_x = 1.5, min_y = 1.5, max_y = 1.5, min_z = 1.5, max_z = 1.5; 
	float back_x = 0, front_x = 4.5, back_y = 0.6, front_y = 0.6, back_z = 0.6, front_z = 0.6;
	float min_cache_x = 2.5, max_cache_x = 2.5, min_cache_y = 2.5, max_cache_y = 2.5, min_cache_z = 2.5, max_cache_z = 2.5; 
	float rad = 1;

	double previous_pose_x, previous_pose_z;
	double velocity_x;
	double path_y_tolerance_param = 0.2;
	double path_z_tolerance_param = 0.7;
	double waypoint_radius_param = 0.25;
	double x_brake_cost_param = 2.0;
	double z_brake_cost_param = 8.0;
	double goal_cost_param = 2.0;
	double smooth_cost_param = 1.5;

	Histogram polar_histogram;

	nav_msgs::GridCells path_candidates;
	nav_msgs::GridCells path_selected;
	nav_msgs::GridCells path_rejected;
	nav_msgs::GridCells path_blocked;

	
	

	LocalPlanner();
	~LocalPlanner();
	
	void setPose(const geometry_msgs::PoseStamped input);
	void setLimits();
	void filterPointCloud(pcl::PointCloud<pcl::PointXYZ>& );
	bool obstacleAhead();
	void createPolarHistogram();
	void findFreeDirections();
	geometry_msgs::Vector3Stamped getWaypointFromAngle(int e, int z);
	double costFunction(int e, int z);
	void calculateCostMap();
	void getNextWaypoint();
	bool checkForCollision();

};


#endif // GLOBAL_PLANNER_LOCAL_PLANNER_H