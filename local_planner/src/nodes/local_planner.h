#ifndef LOCAL_PLANNER_LOCAL_PLANNER_H
#define LOCAL_PLANNER_LOCAL_PLANNER_H

#include <iostream>
#include <fstream>
#include <math.h> 
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <deque>

#include <ros/ros.h>

#include "histogram.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/crop_box.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/filters/extract_indices.h>


#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>

#include <opencv2/imgproc/imgproc.hpp>


#include <local_planner/LocalPlannerNodeConfig.h>
#include <dynamic_reconfigure/server.h>


#define PI 3.14159265
#define alpha_res 6
#define grid_length_z 360/alpha_res
#define grid_length_e 180/alpha_res
#define age_lim 100
#define min_bin 1.5
#define h_fov 59.0
#define v_fov 46.0

float distance3DCartesian(geometry_msgs::Point a, geometry_msgs::Point b);
float distance2DPolar(int e1, int z1, int e2, int z2);
float computeL2Dist(geometry_msgs::PoseStamped pose, pcl::PointCloud<pcl::PointXYZ>::iterator pcl_it);

class LocalPlanner
{

 private:

  bool first_reach_ = true;
  bool obstacle_ = false;
  bool set_first_yaw_ = true;
  bool reach_altitude_ = false;
  bool reached_goal_ = false;
  bool first_brake_ = true;
  bool waypoint_outside_FOV_ = false;
  bool print_height_map_ = true;
  bool over_obstacle_ = false;
  bool is_near_min_height_ = false;
  bool too_low_ = false;
  bool new_cloud_ = false;
  bool use_ground_detection_;
  bool use_back_off_ = true;
  bool log_data_to_txt_file_ = true;
  bool ground_detected_ = false;
  bool box_size_increase_ = true;
  bool no_progress_rise_ = false;
  bool back_off_ = false;
  bool only_yawed_ = false;

  int stop_in_front_;
  int n_call_hist_ = 0;
  int e_FOV_max_, e_FOV_min_;
  int dist_incline_window_size_ = 50;

  double local_planner_mode_;
  double distance_to_closest_point_;
  double min_dist_to_ground_;
  double velocity_x_, velocity_y_, velocity_z_, velocity_mod_;
  double speed_ = 1.0;
  double curr_yaw_, last_yaw_;
  double yaw_reached_goal_;
  double min_speed_;
  double max_speed_;
  double goal_cost_param_;
  double smooth_cost_param_;
  double smooth_cost_param_adapted_;
  double max_accel_xy_;
  double max_accel_z_;
  double keep_distance_;
  double ground_dist_;
  double min_flight_height_;
  double begin_rise_;
  double height_change_cost_param_ = 5;
  double integral_time_old_ = 0;
  double safety_radius_ = 25;
  double min_box_x_, max_box_x_, min_box_y_, max_box_y_, min_box_z_, max_box_z_;
  double min_groundbox_x_ = 10, max_groundbox_x_ = 10, min_groundbox_y_ = 10, max_groundbox_y_ = 10, min_groundbox_z_ = 2.0;
  double ground_inlier_distance_threshold_;
  double ground_inlier_angle_threshold_;
  double no_progress_slope_;
  double progress_slope_;
  double rise_factor_no_progress_;

  std::string log_name_;

  std::vector<double> ground_heights_;
  std::vector<double> ground_xmax_;
  std::vector<double> ground_xmin_;
  std::vector<double> ground_ymax_;
  std::vector<double> ground_ymin_;
  std::vector<int> e_FOV_idx_;
  std::vector<int> z_FOV_idx_;
  std::deque<double> goal_dist_incline_;
  std::vector<float> cost_path_candidates_;
  std::vector<int> cost_idx_sorted_;
  std::vector<float> cloud_time_, polar_time_, free_time_, cost_time_, collision_time_;

  std::clock_t t_prev_ = 0.0f;

  pcl::PointCloud<pcl::PointXYZ> final_cloud_, ground_cloud_, reprojected_points_, complete_cloud_;

  geometry_msgs::PoseStamped pose_;
  geometry_msgs::PoseStamped take_off_pose_;
  geometry_msgs::Point min_box_, max_box_, pose_stop_;
  geometry_msgs::Point min_groundbox_, max_groundbox_;
  geometry_msgs::Point goal_;
  geometry_msgs::Point closest_point_;
  geometry_msgs::Point back_off_point_;
  geometry_msgs::Point position_old_;
  geometry_msgs::PoseStamped last_waypt_p_, last_last_waypt_p_;
  geometry_msgs::Vector3Stamped waypt_;
  geometry_msgs::PoseStamped waypt_p_;
  geometry_msgs::TwistStamped curr_vel_;
  geometry_msgs::Point closest_point_on_ground_;
  geometry_msgs::Quaternion ground_orientation_;

  nav_msgs::Path path_msg_;
  nav_msgs::GridCells path_candidates_;
  nav_msgs::GridCells path_selected_;
  nav_msgs::GridCells path_rejected_;
  nav_msgs::GridCells path_blocked_;
  nav_msgs::GridCells path_ground_;
  nav_msgs::GridCells path_waypoints_;

  Histogram polar_histogram_ = Histogram(alpha_res);
  Histogram polar_histogram_old_ = Histogram(alpha_res);
  Histogram polar_histogram_est_ = Histogram(2 * alpha_res);

  void setLimitsBoundingBox();
  bool isPointWithinHistogramBox(pcl::PointCloud<pcl::PointXYZ>::iterator pcl_it);
  bool isPointWithinGroundBox(pcl::PointCloud<pcl::PointXYZ>::iterator pcl_it);
  void logData();
  void fitPlane();
  bool obstacleAhead();
  void calculateFOV();
  void createPolarHistogram();
  void printHistogram(Histogram hist);
  void initGridCells(nav_msgs::GridCells *cell);
  void findFreeDirections();
  void calculateCostMap();
  void getNextWaypoint();
  void getMinFlightHeight();
  bool checkForCollision();
  void goFast();
  void backOff();
  void setVelocity();
  void updateCostParameters();
  void reachGoalAltitudeFirst();
  void getPathMsg();
  bool withinGoalRadius();
  void checkSpeed();
  void stopInFrontObstacles();
  double nextYaw(geometry_msgs::PoseStamped u, geometry_msgs::Vector3Stamped v, double last_yaw);
  bool hasSameYawAndAltitude(geometry_msgs::PoseStamped msg1, geometry_msgs::PoseStamped msg2);
  geometry_msgs::Vector3Stamped getWaypointFromAngle(int e, int z);
  geometry_msgs::PoseStamped createPoseMsg(geometry_msgs::Vector3Stamped waypt, double yaw);
  geometry_msgs::Vector3Stamped smoothWaypoint();

 public:

  bool currently_armed = false;
  bool offboard = false;

  std::vector<float> algorithm_total_time;

  double goal_x_param;
  double goal_y_param;
  double goal_z_param;

  LocalPlanner();
  ~LocalPlanner();

  void setPose(const geometry_msgs::PoseStamped msg);
  double costFunction(int e, int z);
  void resetHistogramCounter();
  void setGoal();
  void filterPointCloud(pcl::PointCloud<pcl::PointXYZ>&);
  geometry_msgs::Point fromPolarToCartesian(int e, int z, double radius, geometry_msgs::Point pos);
  void dynamicReconfigureSetParams(avoidance::LocalPlannerNodeConfig & config, uint32_t level);
  void printAlgorithmStatistics();
  bool groundDetected();
  void getBoundingBoxSize(double &min_x, double &max_x, double &min_y, double &max_y, double &min_z, double &max_z);
  void getGroundBoxSize(double &min_x, double &max_x, double &min_y, double &max_y, double &min_z);
  void getPosition(geometry_msgs::PoseStamped &pos);
  void getGoalPosition(geometry_msgs::Point &goal);
  void getCloudsForVisualization(pcl::PointCloud<pcl::PointXYZ> &final_cloud, pcl::PointCloud<pcl::PointXYZ> &ground_cloud, pcl::PointCloud<pcl::PointXYZ> &reprojected_points);
  void getCandidateDataForVisualization(nav_msgs::GridCells &path_candidates, nav_msgs::GridCells &path_selected, nav_msgs::GridCells &path_rejected, nav_msgs::GridCells &path_blocked, nav_msgs::GridCells &path_ground);
  void getPathData(nav_msgs::Path &path_msg, geometry_msgs::PoseStamped &waypt_p);
  void getGroundDataForVisualization(geometry_msgs::Point &closest_point_on_ground, geometry_msgs::Quaternion &ground_orientation, std::vector<double> &ground_heights, std::vector<double> &ground_xmax, std::vector<double> &ground_xmin, std::vector<double> &ground_ymax, std::vector<double> &ground_ymin);
  void setCurrentVelocity(geometry_msgs::TwistStamped vel);
};

#endif // LOCAL_PLANNER_LOCAL_PLANNER_H
