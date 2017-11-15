#ifndef LOCAL_PLANNER_LOCAL_PLANNER_H
#define LOCAL_PLANNER_LOCAL_PLANNER_H

#include <iostream>
#include <fstream>
#include <math.h> 
#include <Eigen/Dense>
#include <math.h>
#include <string>
#include <vector>

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

#include <local_planner/LocalPlannerNodeConfig.h>
#include <dynamic_reconfigure/server.h>


#define PI 3.14159265
#define alpha_res 6
#define grid_length_z 360/alpha_res
#define grid_length_e 180/alpha_res
#define age_lim 0
#define min_bin 1.5
<<<<<<< ec49835a6d8f4e6ad9fc2de8eb36193b22454b56

//#define h_fov 59.0
//#define v_fov 46.0
=======
#define h_fov 59.0
#define v_fov 46.0
>>>>>>> Determine FOV and give proirity to the new histogram inside the FOV
//#define n_fields_90 round(90.0/alpha_res)
//#define n_fields_hfov std::floor((grid_length_z-h_fov/alpha_res)/2)
//#define n_fields_vfov std::floor((grid_length_e-v_fov/alpha_res)/2)

float distance3DCartesian(geometry_msgs::Point a, geometry_msgs::Point b);
float distance2DPolar(int e1, int z1, int e2, int z2);
float computeL2Dist(geometry_msgs::PoseStamped pose, pcl::PointCloud<pcl::PointXYZ>::iterator pcl_it);

class Histogram
{
   int resolution;
   int z_dim;
   int e_dim;
  std::vector<std::vector<double> > bin;
  std::vector<std::vector<double> > age;
  std::vector<std::vector<double> > dist;

 public:
  Histogram(const int res)
      : resolution { res },
        z_dim { 360 / resolution },
        e_dim { 180 / resolution } {

    bin.resize(e_dim);
    age.resize(e_dim);
    dist.resize(e_dim);
    for (int i = 0; i < e_dim; ++i) {
      bin[i].resize(z_dim);
      age[i].resize(z_dim);
      dist[i].resize(z_dim);
    }
    setZero();
  }

  ~Histogram() {
  }

  double get_bin(int x, int y) const {
    return bin[x][y];
  }
  double get_age(int x, int y) const {
    return age[x][y];
  }

  double get_dist(int x, int y) const {
    return dist[x][y];
  }

  void set_bin(int x, int y, double value) {
    bin[x][y] = value;
  }
  void set_age(int x, int y, double value) {
    age[x][y] = value;
  }
  void set_dist(int x, int y, double value) {
    dist[x][y] = value;
  }

  void upsample() {
    resolution = resolution / 2;
    z_dim = 2 * z_dim;
    e_dim = 2 * e_dim;
    std::vector < std::vector<double> > temp_bin;
    std::vector < std::vector<double> > temp_age;
    std::vector < std::vector<double> > temp_dist;
    temp_bin.resize(e_dim);
    temp_age.resize(e_dim);
    temp_dist.resize(e_dim);
    for (int i = 0; i < e_dim; ++i) {
      temp_bin[i].resize(z_dim);
      temp_age[i].resize(z_dim);
      temp_dist[i].resize(z_dim);
    }
    for (int i = 0; i < e_dim; ++i) {
      for (int j = 0; j < z_dim; ++j) {
        int i_lowres = floor(i / 2);
        int j_lowres = floor(j / 2);
        temp_bin[i][j] = bin[i_lowres][j_lowres];
        temp_age[i][j] = age[i_lowres][j_lowres];
        temp_dist[i][j] = dist[i_lowres][j_lowres];
      }
    }
    bin = temp_bin;
    age = temp_age;
    dist = temp_dist;
  }

  void setZero() {
    for (int i = 0; i < e_dim; ++i) {
      for (int j = 0; j < z_dim; ++j) {
        bin[i][j] = 0.0;
        age[i][j] = 0.0;
        dist[i][j] = 0.0;
      }
    }
  }
};



class LocalPlanner {

public:

	pcl::PointCloud<pcl::PointXYZ> final_cloud_;
  pcl::PointCloud<pcl::PointXYZ> reprojected_points;
	pcl::PointCloud<pcl::PointXYZ> complete_cloud_;
  pcl::PointCloud<pcl::PointXYZ> reprojected_points_;

  bool new_cloud_ = false;
	bool first_reach_ = true;
	bool obstacle_ = false;
	bool set_first_yaw_ = true;
	bool reach_altitude_ = false;
	bool reached_goal_ = false;
	bool first_brake_ = true;
	int adapted_min_bin_ = 100;
	bool waypoint_outside_FOV_ = false;


	geometry_msgs::Point min_box_, max_box_, goal_, pose_stop_;
	geometry_msgs::PoseStamped pose_, waypt_p_, last_waypt_p_, last_last_waypt_p_;
	geometry_msgs::Vector3Stamped waypt_;
	geometry_msgs::TwistStamped curr_vel_;

	nav_msgs::Path path_msg_;
	nav_msgs::GridCells path_candidates_;
	nav_msgs::GridCells path_selected_;
	nav_msgs::GridCells path_rejected_;
	nav_msgs::GridCells path_blocked_;
	nav_msgs::GridCells path_waypoints_;

	int n_call_hist_ = 0;
	int init = 0;
	int stop_in_front_;
	double min_box_x_, max_box_x_, min_box_y_, max_box_y_, min_box_z_, max_box_z_;
	double rad_ = 1.0;
	float min_distance_;
	int z_FOV_max_, z_FOV_min_, e_FOV_max_, e_FOV_min_;
	double velocity_x_, velocity_y_, velocity_z_, velocity_mod_;
	double speed = 1.0;
	double min_speed_;
	double max_speed_;
	double goal_cost_param_;
	double smooth_cost_param_;
	double prior_cost_param_;
	double goal_x_param_;
	double goal_y_param_;
	double goal_z_param_;
	double curr_yaw_, last_yaw_;
	double yaw_reached_goal_;
	double max_accel_xy_;
	double max_accel_z_;
	double keep_distance_;

	Histogram polar_histogram_ = Histogram(alpha_res);
	Histogram polar_histogram_old_ = Histogram(alpha_res);
	Histogram polar_histogram_est_ = Histogram(2*alpha_res);

	geometry_msgs::Point position_old_;

    std::vector<float> accumulated_height_prior{1.0, 0.9999, 0.9990, 0.9952, 0.9882, 0.9794, 0.9674, 0.9289, 0.8622, 0.7958, 0.7240, 0.6483, 0.5752, 0.5132, 0.4535, 0.4020, 0.3525, 0.3090, 0.2670, 0.2300, 0.2066, 0.1831};
    std::vector<float> height_prior{0.000057, 0.00052, 0.00369, 0.01176, 0.0166, 0.01728, 0.04255, 0.11559, 0.1315, 0.1357, 0.1556, 0.1464};
    
    std::vector<float> cost_path_candidates_;
    std::vector<int> cost_idx_sorted_;

    std::vector<float> cloud_time_, polar_time_, free_time_, cost_time_, collision_time_;
    std::clock_t t_prev = 0.0f;

	LocalPlanner();
	~LocalPlanner();
	
	void setPose(const geometry_msgs::PoseStamped msg);
	void setLimitsBoundingBox();
	void setVelocity();
	void setGoal();
	bool isPointWithinBoxBoundaries(pcl::PointCloud<pcl::PointXYZ>::iterator pcl_it);
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
	void stopInFrontObstacles();
	geometry_msgs::Point fromPolarToCartesian(int e, int z, double radius, geometry_msgs::Point pos);
};


#endif // LOCAL_PLANNER_LOCAL_PLANNER_H
