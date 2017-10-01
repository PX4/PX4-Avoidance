#ifndef GLOBAL_PLANNER_LOCAL_PLANNER_H
#define GLOBAL_PLANNER_LOCAL_PLANNER_H

#include <iostream>
#include <math.h> 
#include <Eigen/Dense>

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

#include <opencv2/imgproc/imgproc.hpp>

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
	bool reached_goal = false;

	geometry_msgs::Point min_box, max_box, goal;
	geometry_msgs::PoseStamped pose, waypt_p, last_waypt_p, last_last_waypt_p; 
	geometry_msgs::Vector3Stamped waypt, last_waypt;
	geometry_msgs::Point ext_p1, ext_p2;
	geometry_msgs::TwistStamped curr_vel;

	nav_msgs::Path path_msg;
	nav_msgs::GridCells path_candidates;
	nav_msgs::GridCells path_selected;
	nav_msgs::GridCells path_rejected;
	nav_msgs::GridCells path_blocked;
	nav_msgs::GridCells path_extended;
	nav_msgs::GridCells path_waypoints;

	int init = 0;

	float min_box_x = 2.0, max_box_x = 2.0, min_box_y = 2.0, max_box_y = 2.0, min_box_z = 0.5, max_box_z = 0.5; 
	float rad = 1.0;
	float min_distance;

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
	double yaw_reached_goal;

	Histogram polar_histogram;

    std::vector<float> accumulated_height_prior{1.0, 0.9999, 0.9990, 0.9952, 0.9882, 0.9794, 0.9674, 0.9289, 0.8622, 0.7958, 0.7240, 0.6483, 0.5752, 0.5132, 0.4535, 0.4020, 0.3525, 0.3090, 0.2670, 0.2300, 0.2066, 0.1831};
    std::vector<float> height_prior{0.000057, 0.00052, 0.00369, 0.01176, 0.0166, 0.01728, 0.04255, 0.11559, 0.1315, 0.1357, 0.1556, 0.1464};
    
    std::vector<float> cost_path_candidates;
    std::vector<int> cost_idx_sorted;

    std::vector<geometry_msgs::Point> extension_points;
    float coef1, coef2, coef3, coef4;
    cv::Scalar mean_x, mean_y, mean_z;
    cv::Scalar stddev_x, stddev_y, stddev_z;

    std::vector<float> cloud_time, polar_time, free_time, cost_time, collision_time;
    std::clock_t t_prev = 0.0f;


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
	double nextYaw(geometry_msgs::PoseStamped u, geometry_msgs::Vector3Stamped v, double last_yaw);
	void reachGoalAltitudeFirst();
	geometry_msgs::Vector3Stamped smoothWaypoint();
	void getPathMsg();
	bool withinGoalRadius();
	void checkSpeed();
	bool hasSameYawAndAltitude(geometry_msgs::PoseStamped msg1, geometry_msgs::PoseStamped msg2);
	geometry_msgs::Point fromPolarToCartesian(int e, int z);
	void extendPowerline();
};


#endif // GLOBAL_PLANNER_LOCAL_PLANNER_H