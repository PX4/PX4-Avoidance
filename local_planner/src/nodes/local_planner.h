#ifndef LOCAL_PLANNER_LOCAL_PLANNER_H
#define LOCAL_PLANNER_LOCAL_PLANNER_H

#include <math.h>
#include <Eigen/Dense>
#include <chrono>
#include <deque>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <mutex>
#include <string>
#include <thread>
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

#include <dynamic_reconfigure/server.h>
#include <local_planner/LocalPlannerNodeConfig.h>

enum waypoint_choice { hover, costmap, tryPath, direct, reachHeight, goBack };

struct avoidanceOutput {
  waypoint_choice waypoint_type;

  geometry_msgs::PoseStamped pose;
  bool obstacle_ahead;
  bool reach_altitude;
  double min_speed;
  double max_speed;
  double velocity_sigmoid_slope;
  ros::Time last_path_time;

  bool use_avoid_sphere;
  int avoid_sphere_age;
  geometry_msgs::Point avoid_centerpoint;
  double avoid_radius;

  bool use_ground_detection;
  bool over_obstacle;
  bool is_near_min_height;
  bool too_low;
  double min_flight_height;

  geometry_msgs::Point back_off_point;
  geometry_msgs::Point back_off_start_point;
  double min_dist_backoff;

  geometry_msgs::PoseStamped take_off_pose;
  geometry_msgs::PoseStamped offboard_pose;

  double costmap_direction_e;
  double costmap_direction_z;
  std::vector<geometry_msgs::Point> path_node_positions;
};

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
  bool hist_is_empty_ = false;

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
  double min_speed_;
  double max_speed_;
  double goal_cost_param_;
  double smooth_cost_param_;
  double height_change_cost_param_ = 4;
  double height_change_cost_param_adapted_ = 4;
  double max_accel_xy_;
  double max_accel_z_;
  double keep_distance_;
  ros::Time integral_time_old_;
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
  double relevance_margin_z_degree_ = 40;
  double relevance_margin_e_degree_ = 25;
  double velocity_sigmoid_slope_ = 1;
  double min_realsense_dist_ = 0.2;
  double costmap_direction_e_;
  double costmap_direction_z_;

  waypoint_choice waypoint_type_;
  ros::Time last_path_time_;

  std::vector<int> e_FOV_idx_;
  std::vector<int> z_FOV_idx_;
  std::deque<double> goal_dist_incline_;
  std::vector<float> cost_path_candidates_;
  std::vector<int> cost_idx_sorted_;
  std::vector<int> closed_set_;
  std::vector<double> reprojected_points_age_;
  std::vector<double> reprojected_points_dist_;

  std::vector<TreeNode> tree_;
  StarPlanner star_planner_;

  pcl::PointCloud<pcl::PointXYZ> reprojected_points_, final_cloud_;

  Box histogram_box_;

  geometry_msgs::PoseStamped pose_;
  geometry_msgs::Point min_box_, max_box_, pose_stop_;
  geometry_msgs::Point min_groundbox_, max_groundbox_;
  geometry_msgs::Point goal_;
  geometry_msgs::Point back_off_point_;
  geometry_msgs::Point back_off_start_point_;
  geometry_msgs::Point position_old_;
  geometry_msgs::Point closest_point_;
  geometry_msgs::Point avoid_centerpoint_;
  geometry_msgs::TwistStamped curr_vel_;

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
  void setVelocity();
  void evaluateProgressRate();
  void getDirectionFromCostMap();
  void stopInFrontObstacles();
  void updateObstacleDistanceMsg(Histogram hist);
  void updateObstacleDistanceMsg();
  void create2DObstacleRepresentation(const bool send_to_fcu);

 public:
  GroundDetector ground_detector_;
  Box histogram_box_size_;

  bool use_vel_setpoints_;
  bool currently_armed_ = false;
  bool offboard_ = false;
  bool mission_ = false;
  bool smooth_waypoints_ = true;
  bool send_obstacles_fcu_ = false;
  bool stop_in_front_active_ = false;

  double goal_x_param_;
  double goal_y_param_;
  double goal_z_param_;
  double pointcloud_timeout_hover_;
  double pointcloud_timeout_land_;
  double starting_height_ = 0.0;
  double speed_ = 1.0;
  std::string log_name_;

  geometry_msgs::PoseStamped take_off_pose_;
  geometry_msgs::PoseStamped offboard_pose_;
  sensor_msgs::LaserScan distance_data_ = {};
  pcl::PointCloud<pcl::PointXYZ> complete_cloud_;

  LocalPlanner();
  ~LocalPlanner();

  void setPose(const geometry_msgs::PoseStamped msg);
  void setGoal();
  void determineStrategy();
  void dynamicReconfigureSetParams(avoidance::LocalPlannerNodeConfig &config,
                                   uint32_t level);
  void getPosition(geometry_msgs::PoseStamped &pos);
  void getGoalPosition(geometry_msgs::Point &goal);
  void getAvoidSphere(geometry_msgs::Point &center, double &radius,
                      int &avoid_sphere_age, bool &use_avoid_sphere);
  void getCloudsForVisualization(
      pcl::PointCloud<pcl::PointXYZ> &final_cloud,
      pcl::PointCloud<pcl::PointXYZ> &ground_cloud,
      pcl::PointCloud<pcl::PointXYZ> &reprojected_points);
  void getCandidateDataForVisualization(nav_msgs::GridCells& path_candidates,
                                        nav_msgs::GridCells& path_selected,
                                        nav_msgs::GridCells& path_rejected,
                                        nav_msgs::GridCells& path_blocked,
                                        nav_msgs::GridCells& FOV_cells,
                                        nav_msgs::GridCells& path_ground);
  void setCurrentVelocity(const geometry_msgs::TwistStamped& vel);
  void getTree(std::vector<TreeNode> &tree, std::vector<int> &closed_set,
               std::vector<geometry_msgs::Point> &path_node_positions);
  void sendObstacleDistanceDataToFcu(sensor_msgs::LaserScan &obstacle_distance);
  void runPlanner();
  void getAvoidanceOutput(avoidanceOutput &out);
};

#endif  // LOCAL_PLANNER_H
