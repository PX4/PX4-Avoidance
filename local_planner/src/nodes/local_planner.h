#ifndef LOCAL_PLANNER_LOCAL_PLANNER_H
#define LOCAL_PLANNER_LOCAL_PLANNER_H

#include <sensor_msgs/image_encodings.h>
#include "avoidance_output.h"
#include "box.h"
#include "candidate_direction.h"
#include "cost_parameters.h"
#include "histogram.h"

#include <dynamic_reconfigure/server.h>
#include <local_planner/LocalPlannerNodeConfig.h>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>

#include <ros/time.h>
#include <deque>
#include <string>
#include <vector>

namespace avoidance {

class StarPlanner;
class TreeNode;

class LocalPlanner {
 private:
  bool use_back_off_;
  bool use_VFH_star_;
  bool adapt_cost_params_;
  bool stop_in_front_;

  bool reach_altitude_ = false;
  bool obstacle_ = false;
  bool first_brake_ = true;
  bool waypoint_outside_FOV_ = false;
  bool back_off_ = false;
  bool hist_is_empty_ = false;

  int e_FOV_max_, e_FOV_min_;
  size_t dist_incline_window_size_ = 50;
  int origin_;
  int tree_age_ = 0;
  int children_per_node_;
  int n_expanded_nodes_;
  int reproj_age_;
  int counter_close_points_backoff_ = 0;

  double velocity_mod_;
  double curr_yaw_, last_yaw_;
  double min_speed_;
  double max_speed_;
  double keep_distance_;
  ros::Time integral_time_old_;
  double no_progress_slope_;
  double tree_node_distance_;
  double new_yaw_;
  double distance_to_closest_point_;
  double safety_radius_ = 25.0;
  double min_cloud_size_ = 160.0;
  double min_dist_backoff_;
  double relevance_margin_z_degree_ = 40;
  double relevance_margin_e_degree_ = 25;
  double velocity_sigmoid_slope_ = 1;
  double min_realsense_dist_ = 0.2;
  float costmap_direction_e_;
  float costmap_direction_z_;

  waypoint_choice waypoint_type_;
  ros::Time last_path_time_;

  std::vector<int> e_FOV_idx_;
  std::vector<int> z_FOV_idx_;
  std::deque<double> goal_dist_incline_;
  std::vector<float> cost_path_candidates_;
  std::vector<int> cost_idx_sorted_;
  std::vector<int> closed_set_;
  std::vector<int> reprojected_points_age_;

  std::vector<TreeNode> tree_;
  std::unique_ptr<StarPlanner> star_planner_;
  costParameters cost_params_;

  pcl::PointCloud<pcl::PointXYZ> reprojected_points_, final_cloud_;

  geometry_msgs::PoseStamped pose_;
  Eigen::Vector3f goal_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f back_off_point_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f back_off_start_point_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f position_old_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f closest_point_ = Eigen::Vector3f::Zero();
  geometry_msgs::TwistStamped curr_vel_;

  Histogram polar_histogram_ = Histogram(ALPHA_RES);
  Histogram to_fcu_histogram_ = Histogram(ALPHA_RES);
  Eigen::MatrixXf cost_matrix_;
  std::vector<candidateDirection> candidate_vector_;

  void fitPlane();
  void reprojectPoints(Histogram histogram);
  void setVelocity();
  void evaluateProgressRate();
  void stopInFrontObstacles();
  void updateObstacleDistanceMsg(Histogram hist);
  void updateObstacleDistanceMsg();
  void create2DObstacleRepresentation(const bool send_to_fcu);
  sensor_msgs::Image generateHistogramImage(Histogram &histogram);

 public:
  double h_FOV_ = 59.0;
  double v_FOV_ = 46.0;
  Box histogram_box_;
  sensor_msgs::Image histogram_image_;
  bool use_vel_setpoints_;
  bool currently_armed_ = false;
  bool offboard_ = false;
  bool mission_ = false;
  bool smooth_waypoints_ = true;
  bool send_obstacles_fcu_ = false;
  bool stop_in_front_active_ = false;
  bool disable_rise_to_goal_altitude_ = false;

  double pointcloud_timeout_hover_;
  double pointcloud_timeout_land_;
  double starting_height_ = 0.0;
  double speed_ = 1.0;
  double ground_distance_ = 2.0;

  geometry_msgs::PoseStamped take_off_pose_;
  geometry_msgs::PoseStamped offboard_pose_;
  sensor_msgs::LaserScan distance_data_ = {};
  geometry_msgs::Point last_sent_waypoint_;

  // complete_cloud_ contains n complete clouds from the cameras
  std::vector<pcl::PointCloud<pcl::PointXYZ>> complete_cloud_;

  LocalPlanner();
  ~LocalPlanner();

  void setPose(const geometry_msgs::PoseStamped msg);
  void setGoal(const geometry_msgs::Point &goal);
  geometry_msgs::Point getGoal();
  void applyGoal();
  void dynamicReconfigureSetParams(avoidance::LocalPlannerNodeConfig &config,
                                   uint32_t level);
  geometry_msgs::PoseStamped getPosition();
  void getCloudsForVisualization(
      pcl::PointCloud<pcl::PointXYZ> &final_cloud,
      pcl::PointCloud<pcl::PointXYZ> &reprojected_points);
  void setCurrentVelocity(const geometry_msgs::TwistStamped &vel);
  void getTree(std::vector<TreeNode> &tree, std::vector<int> &closed_set,
               std::vector<geometry_msgs::Point> &path_node_positions);
  void sendObstacleDistanceDataToFcu(sensor_msgs::LaserScan &obstacle_distance);
  avoidanceOutput getAvoidanceOutput();

  void determineStrategy();
  void runPlanner();
};
}

#endif  // LOCAL_PLANNER_H
