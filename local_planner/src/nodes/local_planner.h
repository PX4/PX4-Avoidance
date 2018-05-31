#ifndef LOCAL_PLANNER_LOCAL_PLANNER_H
#define LOCAL_PLANNER_LOCAL_PLANNER_H

#include <math.h>
#include <Eigen/Dense>
#include <deque>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>

#include "box.h"
#include "common.h"
#include "ground_detector.h"
#include "histogram.h"
#include "planner_functions.h"
#include "star_planner.h"
#include "tree_node.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <dynamic_reconfigure/server.h>
#include <local_planner/LocalPlannerNodeConfig.h>

class LocalPlanner {
 private:
  bool use_back_off_;
  bool use_VFH_star_;
  bool adapt_cost_params_;
  bool stop_in_front_;
  bool use_avoid_sphere_;
  bool use_ground_detection_;

  bool reach_altitude_ = false;
  bool obstacle_ = false;
  bool first_brake_ = true;
  bool waypoint_outside_FOV_ = false;
  bool over_obstacle_ = false;
  bool is_near_min_height_ = false;
  bool too_low_ = false;
  bool ground_detected_ = false;
  bool back_off_ = false;
  bool only_yawed_ = false;
  bool hist_is_empty_ = false;
  bool tree_available_ = false;
  bool reached_goal_ = false;

  int e_FOV_max_, e_FOV_min_;
  int dist_incline_window_size_ = 50;
  int origin_;
  int tree_age_ = 0;
  int childs_per_node_;
  int n_expanded_nodes_;
  int reproj_age_;
  int counter_close_points_backoff_ = 0;
  int avoid_sphere_age_ = 1000;

  double velocity_x_, velocity_y_, velocity_z_, velocity_mod_;
  double curr_yaw_, last_yaw_;
  double yaw_reached_goal_;
  double min_speed_;
  double max_speed_;
  double goal_cost_param_;
  double smooth_cost_param_;
  double height_change_cost_param_ = 4;
  double height_change_cost_param_adapted_ = 4;
  double max_accel_xy_;
  double max_accel_z_;
  double keep_distance_;
  double integral_time_old_ = 0.0;
  double no_progress_slope_;
  double tree_node_distance_;
  double min_flight_height_ = 0.0;
  double ground_margin_ = 0.0;
  double new_yaw_;
  double min_dist_to_ground_;
  double distance_to_closest_point_;
  double safety_radius_ = 25.0;
  double min_cloud_size_ = 160.0;
  double min_dist_backoff_;
  double avoid_radius_;
  double speed_ = 1.0;

  std::vector<int> e_FOV_idx_;
  std::vector<int> z_FOV_idx_;
  std::deque<double> goal_dist_incline_;
  std::vector<float> cost_path_candidates_;
  std::vector<int> cost_idx_sorted_;
  std::vector<float> ground_time_, cloud_time_, polar_time_, free_time_,
      cost_time_, collision_time_;
  std::vector<int> closed_set_;
  std::vector<double> reprojected_points_age_;
  std::vector<double> reprojected_points_dist_;
  std::vector<float> tree_time_;

  std::vector<TreeNode> tree_;
  StarPlanner star_planner_;

  std::clock_t t_prev_ = 0.0f;

  pcl::PointCloud<pcl::PointXYZ> reprojected_points_, final_cloud_;

  Box histogram_box_;

  geometry_msgs::PoseStamped pose_;
  geometry_msgs::Point min_box_, max_box_, pose_stop_;
  geometry_msgs::Point min_groundbox_, max_groundbox_;
  geometry_msgs::Point goal_;
  geometry_msgs::Point back_off_point_;
  geometry_msgs::Point back_off_start_point_;
  geometry_msgs::PoseStamped offboard_pose_;
  geometry_msgs::Point position_old_;
  geometry_msgs::Point closest_point_;
  geometry_msgs::Point avoid_centerpoint_;
  geometry_msgs::PoseStamped last_waypt_p_, last_last_waypt_p_;
  geometry_msgs::Vector3Stamped waypt_;
  geometry_msgs::Vector3Stamped waypt_adapted_;
  geometry_msgs::Vector3Stamped waypt_smoothed_;
  geometry_msgs::Vector3Stamped hover_point_;
  geometry_msgs::Vector3Stamped last_hist_waypt_;
  geometry_msgs::PoseStamped waypt_p_;
  geometry_msgs::TwistStamped curr_vel_;

  nav_msgs::Path path_msg_;
  nav_msgs::GridCells FOV_cells_;
  nav_msgs::GridCells path_candidates_;
  nav_msgs::GridCells path_selected_;
  nav_msgs::GridCells path_rejected_;
  nav_msgs::GridCells path_blocked_;
  nav_msgs::GridCells path_ground_;
  nav_msgs::GridCells path_waypoints_;

  Histogram polar_histogram_ = Histogram(ALPHA_RES);
  Histogram to_fcu_histogram_ = Histogram(ALPHA_RES);

  void logData();
  void fitPlane();
  void reprojectPoints(Histogram histogram);
  void getNextWaypoint();
  void goFast();
  void backOff();
  void setVelocity();
  void evaluateProgressRate();
  void reachGoalAltitudeFirst();
  void getPathMsg();
  bool withinGoalRadius();
  void getDirectionFromCostMap();
  void adaptSpeed();
  void stopInFrontObstacles();
  void updateObstacleDistanceMsg(Histogram hist);
  void updateObstacleDistanceMsg();
  geometry_msgs::Vector3Stamped smoothWaypoint(
      geometry_msgs::Vector3Stamped wp);
  void create2DObstacleRepresentation(const bool send_to_fcu);

 public:
  GroundDetector ground_detector_;

  bool currently_armed_ = false;
  bool offboard_ = false;
  bool smooth_waypoints_ = true;
  bool send_obstacles_fcu_ = false;

  std::vector<float> algorithm_total_time_;

  double goal_x_param_;
  double goal_y_param_;
  double goal_z_param_;
  double pointcloud_timeout_hover_;
  double pointcloud_timeout_land_;
  double local_planner_mode_;
  double starting_height_ = 0.0;

  geometry_msgs::PoseStamped take_off_pose_;

  sensor_msgs::LaserScan distance_data_ = {};

  Box histogram_box_size_;

  pcl::PointCloud<pcl::PointXYZ> complete_cloud_;

  std::string log_name_;

  LocalPlanner();
  ~LocalPlanner();

  void setPose(const geometry_msgs::PoseStamped msg);
  void setGoal();
  void determineStrategy();
  void dynamicReconfigureSetParams(avoidance::LocalPlannerNodeConfig &config,
                                   uint32_t level);
  void printAlgorithmStatistics();
  void getPosition(geometry_msgs::PoseStamped &pos);
  void getGoalPosition(geometry_msgs::Point &goal);
  void getAvoidSphere(geometry_msgs::Point &center, double &radius,
                      int &avoid_sphere_age, bool &use_avoid_sphere);
  void getCloudsForVisualization(
      pcl::PointCloud<pcl::PointXYZ> &final_cloud,
      pcl::PointCloud<pcl::PointXYZ> &ground_cloud,
      pcl::PointCloud<pcl::PointXYZ> &reprojected_points);
  void getCandidateDataForVisualization(nav_msgs::GridCells &path_candidates,
                                        nav_msgs::GridCells &path_selected,
                                        nav_msgs::GridCells &path_rejected,
                                        nav_msgs::GridCells &path_blocked,
                                        nav_msgs::GridCells &FOV_cells,
                                        nav_msgs::GridCells &path_ground);
  void getPathData(nav_msgs::Path &path_msg,
                   geometry_msgs::PoseStamped &waypt_p);
  void setCurrentVelocity(geometry_msgs::TwistStamped vel);
  void getTree(std::vector<TreeNode> &tree, std::vector<int> &closed_set,
               std::vector<geometry_msgs::Point> &path_node_positions,
               bool &tree_available);
  void getWaypoints(geometry_msgs::Vector3Stamped &waypt,
                    geometry_msgs::Vector3Stamped &waypt_adapted,
                    geometry_msgs::Vector3Stamped &waypt_smoothed);
  void sendObstacleDistanceDataToFcu(sensor_msgs::LaserScan &obstacle_distance);
  void runPlanner();
};

#endif  // LOCAL_PLANNER_H
