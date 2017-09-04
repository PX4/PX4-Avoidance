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
#include <geometry_msgs/Twist.h>

#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_ros/conversions.h>

#include <opencv2/imgproc/imgproc.hpp>

#include "avoidance/common.h"


#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/crop_box.h>


#define PI 3.14159265
#define alpha_res 10
#define grid_length_z 360/alpha_res
#define grid_length_e 180/alpha_res

float distance3DCartesian(geometry_msgs::Point a, geometry_msgs::Point b);
float distance2DPolar(int e1, int z1, int e2, int z2);

class Histogram 
{ 
	int array[grid_length_e][grid_length_z];

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

	bool first_reach = true;
	bool obstacle = false; 
	bool set_first_yaw = true;
	bool reach_altitude = false;

	geometry_msgs::Point min_box, max_box, goal;
	geometry_msgs::PoseStamped pose, waypt_stop, waypt_p, last_waypt_p; 
	geometry_msgs::Vector3Stamped waypt, last_waypt, setpoint, current_goal, last_goal, not_smooth_wp;
	geometry_msgs::Point p1;
	geometry_msgs::TwistStamped curr_vel;
	geometry_msgs::Point ext_p1, ext_p2;

	nav_msgs::Path path_msg;

	int init = 0;
	int cost_type;

	float min_box_x = 10.0, max_box_x = 10.0, min_box_y = 6.5, max_box_y = 6.5, min_box_z = 2.0, max_box_z = 2.0; 
	float rad = 1.0;

	double velocity_x, velocity_y, velocity_z, velocity_mod;
	double speed = 2.0;
	double min_speed = 2.0;
	double max_speed = 3.0;
	double goal_cost_param = 2.0;
	double smooth_cost_param = 1.5;
	double prior_cost_param = 17.5;
	double goal_x_param;
	double goal_y_param;
	double goal_z_param;
	double curr_yaw, last_yaw;
	double deceleration_limit = 1.5*9.8066;
	double min_dist_pose_obst;
	 

	Histogram polar_histogram;

	nav_msgs::GridCells path_candidates;
	nav_msgs::GridCells path_selected;
	nav_msgs::GridCells path_rejected;
	nav_msgs::GridCells path_blocked;
	nav_msgs::GridCells path_extended;

	nav_msgs::GridCells Ppath_candidates;
	nav_msgs::GridCells Ppath_selected;
	nav_msgs::GridCells Ppath_rejected;
	nav_msgs::GridCells Ppath_blocked;

	
    std::vector<int> cost_idx_sorted;

  //  std::vector<float> heigh_prior{0.0, 0.0098, 0.0927, 0.473, 1.1746, 2.0522, 3.259, 7.106, 13.777, 20.41, 27.59, 35.16};
	//std::vector<float> accumulated_height_prior{0.0, 0.000098, 0.000927, 0.0473, 0.011746, 0.020522, 0.03259, 0.07106, 0.13777, 0.2041, 0.2759, 0.3516};
    std::vector<float> accumulated_height_prior{1.0, 0.9999, 0.9990, 0.9952, 0.9882, 0.9794, 0.9674, 0.9289, 0.8622, 0.7958, 0.7240, 0.6483, 0.5752, 0.5132, 0.4535, 0.4020, 0.3525, 0.3090, 0.2670, 0.2300, 0.2066, 0.1831};
    std::vector<float> height_prior{0.000057, 0.00052, 0.00369, 0.01176, 0.0166, 0.01728, 0.04255, 0.11559, 0.1315, 0.1357, 0.1556, 0.1464};

    std::vector<geometry_msgs::Point> extension_points;
    float coef1, coef2, coef3, coef4;

    cv::Scalar mean_x, mean_y, mean_z;
    cv::Scalar stddev_x, stddev_y, stddev_z;

    std::vector<float> cloud_time, polar_time, free_time, cost_time, collision_time;
    


	LocalPlanner();
	~LocalPlanner();
	
	void setPose(const geometry_msgs::PoseStamped msg);
	void setLimitsBoundingBox();
	void setVelocity();
	void setGoal();
	void filterPointCloud(pcl::PointCloud<pcl::PointXYZ>& );
	bool obstacleAhead();
	void createPolarHistogram();
	void initGridCells(nav_msgs::GridCells *cell);
	void findFreeDirections();
	geometry_msgs::Vector3Stamped getWaypointFromAngle(int e, int z);
	double costFunction(int e, int z);
	void calculateCostMap();
	void getNextWaypoint();
	bool checkForCollision();
	void goFast();
	geometry_msgs::PoseStamped createPoseMsg(geometry_msgs::Vector3Stamped waypt, double yaw);
	double nextYaw(geometry_msgs::Vector3Stamped u, geometry_msgs::Vector3Stamped v, double last_yaw);
	void reachGoalAltitudeFirst();
	void getPathMsg();
	bool withinGoalRadius();
	void publishPathCells(double e, double z, int path_type);
	void checkSpeed();
	bool hasSameYawAndAltitude(geometry_msgs::PoseStamped msg1, geometry_msgs::PoseStamped msg2);
	geometry_msgs::Point fromPolarToCartesian(int e, int z);
};


#endif // GLOBAL_PLANNER_LOCAL_PLANNER_H