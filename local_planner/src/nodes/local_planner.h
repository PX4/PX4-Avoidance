#ifndef LOCAL_PLANNER_LOCAL_PLANNER_H
#define LOCAL_PLANNER_LOCAL_PLANNER_H

#include <iostream>
#include <fstream>
#include <math.h>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <deque>
#include <mutex>
#include <limits>

#include <ros/ros.h>

#include "histogram.h"
#include "tree_node.h"
#include "box.h"
#include "planner_functions.h"
#include "star_planner.h"

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
#define ALPHA_RES 6
#define GRID_LENGTH_Z 360/ALPHA_RES
#define GRID_LENGTH_E 180/ALPHA_RES
#define AGE_LIM 100
#define MIN_BIN 1.5
#define H_FOV 59.0
#define V_FOV 46.0
#define INF  std::numeric_limits<double>::infinity()

float distance3DCartesian(geometry_msgs::Point a, geometry_msgs::Point b);
float distance2DPolar(int e1, int z1, int e2, int z2);

class LocalPlanner
{

 private:

  bool obstacle_ = false;
  bool reached_goal_ = false;
  bool first_brake_ = true;
  bool waypoint_outside_FOV_ = false;
  bool over_obstacle_ = false;
  bool is_near_min_height_ = false;
  bool too_low_ = false;
  bool use_back_off_ = true;
  bool log_data_to_txt_file_ = true;
  bool ground_detected_ = false;
  bool back_off_ = false;
  bool only_yawed_ = false;
  bool use_VFH_star_ = true;
  bool hist_is_empty_ = false;
  bool stop_in_front_;
  bool adapt_cost_params_;

  int e_FOV_max_, e_FOV_min_;
  int dist_incline_window_size_ = 50;
  int origin_;
  int tree_age_ = 0;


  double local_planner_last_mode_;
  double smooth_go_fast_;
  double local_planner_mode_;
  double velocity_x_, velocity_y_, velocity_z_, velocity_mod_;
  double curr_yaw_, last_yaw_;
  double yaw_reached_goal_;
  double min_speed_;
  double max_speed_;
  double goal_cost_param_;
  double smooth_cost_param_;
  double max_accel_xy_;
  double max_accel_z_;
  double keep_distance_;
  double ground_dist_;
  double min_flight_height_;
  double begin_rise_;
  double height_change_cost_param_ = 4;
  double height_change_cost_param_adapted_ = 4;
  double integral_time_old_ = 0;
  double ground_inlier_distance_threshold_;
  double ground_inlier_angle_threshold_;
  double no_progress_slope_;
  double min_plane_points_ = 100;
  double min_plane_percentage_ = 0.6;
  double tree_node_distance_ = 0.5;

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
  std::vector<int> closed_set_;
  std::vector<double> reprojected_points_age_;
  std::vector<double> reprojected_points_dist_;
  std::vector<geometry_msgs::Point> path_node_positions_;
  std::vector<int> path_node_origins_;

  std::vector<TreeNode> tree_;

  StarPlanner star_planner_;

  std::clock_t t_prev_ = 0.0f;

  pcl::PointCloud<pcl::PointXYZ> ground_cloud_, reprojected_points_;

  geometry_msgs::PoseStamped pose_;
  geometry_msgs::Point min_box_, max_box_, pose_stop_;
  geometry_msgs::Point min_groundbox_, max_groundbox_;
  geometry_msgs::PoseStamped take_off_pose_;
  geometry_msgs::Point goal_;
  geometry_msgs::Point back_off_point_;
  geometry_msgs::Point position_old_;
  geometry_msgs::PoseStamped last_waypt_p_, last_last_waypt_p_;
  geometry_msgs::Vector3Stamped waypt_;
  geometry_msgs::Vector3Stamped waypt_adapted_;
  geometry_msgs::Vector3Stamped waypt_smoothed_;
  geometry_msgs::Vector3Stamped hover_point_;
  geometry_msgs::Vector3Stamped last_hist_waypt_;
  geometry_msgs::PoseStamped waypt_p_;
  geometry_msgs::TwistStamped curr_vel_;
  geometry_msgs::Point closest_point_on_ground_;
  geometry_msgs::Quaternion ground_orientation_;

  nav_msgs::Path path_msg_;
  nav_msgs::GridCells FOV_cells_;
  nav_msgs::GridCells path_candidates_;
  nav_msgs::GridCells path_selected_;
  nav_msgs::GridCells path_rejected_;
  nav_msgs::GridCells path_blocked_;
  nav_msgs::GridCells path_ground_;
  nav_msgs::GridCells path_waypoints_;

  Histogram polar_histogram_ = Histogram(ALPHA_RES);
  Histogram polar_histogram_old_ = Histogram(ALPHA_RES);

  void logData();
  void fitPlane();
  bool obstacleAhead();
  void reprojectPoints();
//  void calculateFOV(std::vector<int> &z_FOV_idx, int &e_FOV_min, int &e_FOV_max, double yaw, double pitch);
//  Histogram createPolarHistogram(std::vector<int> z_FOV_idx, int e_FOV_min, int e_FOV_max);
//  void printHistogram(Histogram hist);
//  void initGridCells(nav_msgs::GridCells *cell);
//  void findFreeDirections(Histogram histogram);
//  void calculateCostMap();
  void getNextWaypoint();
  void getMinFlightHeight();
  void goFast();
  void backOff();
  void setVelocity();
  void evaluateProgressRate();
  void reachGoalAltitudeFirst();
  void getPathMsg();
  bool withinGoalRadius();
//  bool getDirectionFromTree();
  void getDirectionFromCostMap();
  void checkSpeed();
  void stopInFrontObstacles();
//  void buildLookAheadTree();
//  double treeCostFunction(int node_number);
//  double treeHeuristicFunction(int node_number);
//  double indexAngleDifference(int a, int b);
//  void angleToIndex(int &e_idx, int &z_idx, int e, int z);
  double nextYaw(geometry_msgs::PoseStamped u, geometry_msgs::Vector3Stamped v, double last_yaw);
  bool hasSameYawAndAltitude(geometry_msgs::PoseStamped msg1, geometry_msgs::PoseStamped msg2);
  geometry_msgs::Vector3Stamped getWaypointFromAngle(int e, int z);
  geometry_msgs::PoseStamped createPoseMsg(geometry_msgs::Vector3Stamped waypt, double yaw);
  geometry_msgs::Vector3Stamped smoothWaypoint(geometry_msgs::Vector3Stamped wp);
  int getMinFlightElevationIndex();

 public:

  bool currently_armed_ = false;
  bool offboard_ = false;
  bool use_ground_detection_;
  bool new_cloud_;
  bool hovering_ = false;
  bool use_avoid_sphere_ = false;
  bool reach_altitude_ = false;

  std::vector<float> algorithm_total_time_;

  double goal_x_param_;
  double goal_y_param_;
  double goal_z_param_;
  double pointcloud_timeout_hover_;
  double pointcloud_timeout_land_;
  double local_planner_mode_;
  double starting_height_;

  pcl::PointCloud<pcl::PointXYZ> complete_cloud_;
  geometry_msgs::PoseStamped take_off_pose_;

  double min_dist_to_ground_;
  double distance_to_closest_point_;
  double safety_radius_ = 25;
  double min_cloud_size_ = 160;
  double min_dist_backoff_;
  double avoid_radius_;
  double speed_ = 1.0;

  int counter_close_points_backoff_ = 0;
  int avoid_sphere_age_ = 1000;

  Box histogram_box_size_;
  Box ground_box_size_;
  Box histogram_box_;
  Box ground_box_;

  pcl::PointCloud<pcl::PointXYZ> complete_cloud_, final_cloud_;

  geometry_msgs::Point closest_point_;
  geometry_msgs::Point avoid_centerpoint_;

  std::string log_name_;

  LocalPlanner();
  ~LocalPlanner();

  void setPose(const geometry_msgs::PoseStamped msg);
//  double costFunction(int e, int z);
  void setGoal();
  void hover();
  void detectGround(pcl::PointCloud<pcl::PointXYZ>&);
  void determineStrategy();
//  geometry_msgs::Point fromPolarToCartesian(int e, int z, double radius, geometry_msgs::Point pos);
  void dynamicReconfigureSetParams(avoidance::LocalPlannerNodeConfig & config, uint32_t level);
  void printAlgorithmStatistics();
  bool groundDetected();
  void getBoundingBoxSize(double &min_x, double &max_x, double &min_y, double &max_y, double &min_z, double &max_z);
  void getGroundBoxSize(double &min_x, double &max_x, double &min_y, double &max_y, double &min_z);
  void getPosition(geometry_msgs::PoseStamped &pos);
  void getGoalPosition(geometry_msgs::Point &goal);
  void getAvoidSphere(geometry_msgs::Point &center, double &radius, int &avoid_sphere_age, bool &use_avoid_sphere);
  void getCloudsForVisualization(pcl::PointCloud<pcl::PointXYZ> &final_cloud, pcl::PointCloud<pcl::PointXYZ> &ground_cloud, pcl::PointCloud<pcl::PointXYZ> &reprojected_points);
  void getCandidateDataForVisualization(nav_msgs::GridCells &path_candidates, nav_msgs::GridCells &path_selected, nav_msgs::GridCells &path_rejected, nav_msgs::GridCells &path_blocked, nav_msgs::GridCells &FOV_cells, nav_msgs::GridCells &path_ground);
  void getPathData(nav_msgs::Path &path_msg, geometry_msgs::PoseStamped &waypt_p);
  void getGroundDataForVisualization(geometry_msgs::Point &closest_point_on_ground, geometry_msgs::Quaternion &ground_orientation, std::vector<double> &ground_heights, std::vector<double> &ground_xmax, std::vector<double> &ground_xmin, std::vector<double> &ground_ymax, std::vector<double> &ground_ymin);
  void setCurrentVelocity(geometry_msgs::TwistStamped vel);
  void useHoverPoint();
  void getTree(std::vector<TreeNode> &tree, std::vector<int> &closed_set, std::vector<geometry_msgs::Point> &path_node_positions);
  void getWaypoints(geometry_msgs::Vector3Stamped &waypt, geometry_msgs::Vector3Stamped &waypt_adapted, geometry_msgs::Vector3Stamped &waypt_smoothed);
};

#endif // LOCAL_PLANNER_H
