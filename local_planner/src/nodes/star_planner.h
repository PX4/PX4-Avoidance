#ifndef STAR_PLANNER_H
#define STAR_PLANNER_H

#include <vector>
#include <math.h>
#include "tree_node.h"
#include "box.h"
#include "histogram.h"
#include "planner_functions.h"
#include "common.h"


#include <iostream>
#include <fstream>
#include <math.h>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <deque>
#include <limits>

#include <ros/ros.h>

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
#define inf  std::numeric_limits<double>::infinity()

class StarPlanner
{

  bool tree_available_ = false;

  int childs_per_node_ = 1;
  int n_expanded_nodes_ = 5;
  double tree_node_distance_ = 1.0;
  double tree_discount_factor_ = 0.8;
  double min_cloud_size_;
  double min_dist_backoff_;
  double goal_cost_param_;
  double smooth_cost_param_;
  double height_change_cost_param_adapted_;
  double height_change_cost_param_;
  double curr_yaw_;

  Box histogram_box_;
  Box histogram_box_size_;

  std::vector<double> reprojected_points_age_;
  std::vector<double> reprojected_points_dist_;
  std::vector<int> path_node_origins_;

  pcl::PointCloud<pcl::PointXYZ> complete_cloud_;
  pcl::PointCloud<pcl::PointXYZ> reprojected_points_;

  geometry_msgs::Point goal_;
  geometry_msgs::PoseStamped pose_;

  nav_msgs::GridCells path_waypoints_;




 public:

  std::vector<geometry_msgs::Point> path_node_positions_;
  std::vector<int> closed_set_;
  int tree_age_;
  std::vector<TreeNode> tree_;
  bool tree_new_ = false;


  StarPlanner();
  ~StarPlanner();

  void setParams(double min_cloud_size, double min_dist_backoff, nav_msgs::GridCells path_waypoints, double curr_yaw);
  void setReprojectedPoints(pcl::PointCloud<pcl::PointXYZ> reprojected_points, std::vector<double> reprojected_points_age, std::vector<double> reprojected_points_dist);
  void setCostParams(double goal_cost_param, double smooth_cost_param, double height_change_cost_param_adapted, double height_change_cost_param);
  void setPose(geometry_msgs::PoseStamped pose);
  void setGoal(geometry_msgs::Point pose);
  void setBoxSize(Box histogram_box_size);
  void setCloud(pcl::PointCloud<pcl::PointXYZ> complete_cloud);
  double treeCostFunction(int node_number);
  double treeHeuristicFunction(int node_number);
  void buildLookAheadTree(double origin_yaw);
  bool getDirectionFromTree(nav_msgs::GridCells &path_waypoints);
  void dynamicReconfigureSetStarParams(avoidance::LocalPlannerNodeConfig & config, uint32_t level);

};

#endif // STAR_PLANNER_H
