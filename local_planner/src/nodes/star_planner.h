#ifndef STAR_PLANNER_H
#define STAR_PLANNER_H

#include <math.h>
#include <vector>
#include "box.h"
#include "common.h"
#include "histogram.h"
#include "planner_functions.h"
#include "tree_node.h"

#include <math.h>
#include <Eigen/Dense>
#include <deque>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <ros/ros.h>

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

#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>

#include <dynamic_reconfigure/server.h>
#include <local_planner/LocalPlannerNodeConfig.h>

class StarPlanner {
  double h_FOV_ = 59.0;
  double v_FOV_ = 46.0;
  int childs_per_node_ = 1;
  int n_expanded_nodes_ = 5;
  double tree_node_distance_ = 1.0;
  double tree_discount_factor_ = 0.8;
  double goal_cost_param_;
  double smooth_cost_param_;
  double height_change_cost_param_adapted_;
  double height_change_cost_param_;
  double curr_yaw_;
  double min_flight_height_;
  double ground_margin_;
  double min_cloud_size_;
  double min_dist_backoff_;
  double min_realsense_dist_;

  std::vector<double> reprojected_points_age_;
  std::vector<double> reprojected_points_dist_;
  std::vector<int> path_node_origins_;

  std::vector<pcl::PointCloud<pcl::PointXYZ>> complete_cloud_;
  pcl::PointCloud<pcl::PointXYZ> reprojected_points_;

  geometry_msgs::Point goal_;
  geometry_msgs::PoseStamped pose_;

  nav_msgs::GridCells path_waypoints_;
  Box histogram_box_;

 public:
  std::vector<geometry_msgs::Point> path_node_positions_;
  geometry_msgs::Point obstacle_position_;
  std::vector<int> closed_set_;
  int tree_age_;
  std::vector<TreeNode> tree_;

  StarPlanner();
  ~StarPlanner();

  void setParams(double min_cloud_size, double min_dist_backoff,
                 const nav_msgs::GridCells& path_waypoints, double curr_yaw,
                 double min_realsense_dist);
  void setFOV(double h_FOV, double v_FOV);
  void setReprojectedPoints(
      const pcl::PointCloud<pcl::PointXYZ>& reprojected_points,
      const std::vector<double>& reprojected_points_age,
      const std::vector<double>& reprojected_points_dist);
  void setCostParams(double goal_cost_param, double smooth_cost_param,
                     double height_change_cost_param_adapted,
                     double height_change_cost_param);
  void setPose(const geometry_msgs::PoseStamped& pose);
  void setBoxSize(const Box& histogram_box);
  void setGoal(const geometry_msgs::Point& pose);
  void setCloud(
      const std::vector<pcl::PointCloud<pcl::PointXYZ>>& complete_cloud);
  double treeCostFunction(int node_number);
  double treeHeuristicFunction(int node_number);
  void buildLookAheadTree();
  void dynamicReconfigureSetStarParams(
      avoidance::LocalPlannerNodeConfig& config, uint32_t level);
};

#endif  // STAR_PLANNER_H
