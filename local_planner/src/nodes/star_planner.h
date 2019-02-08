#ifndef STAR_PLANNER_H
#define STAR_PLANNER_H

#include "box.h"
#include "cost_parameters.h"
#include "histogram.h"

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/GridCells.h>

#include <dynamic_reconfigure/server.h>
#include <local_planner/LocalPlannerNodeConfig.h>

#include <vector>

namespace avoidance {
class TreeNode;

class StarPlanner {
  double h_FOV_ = 59.0;
  double v_FOV_ = 46.0;
  int children_per_node_ = 1;
  int n_expanded_nodes_ = 5;
  double tree_node_distance_ = 1.0;
  double tree_discount_factor_ = 0.8;
  double curr_yaw_;
  double min_cloud_size_;
  double min_dist_backoff_;
  double min_realsense_dist_;

  std::vector<int> reprojected_points_age_;
  std::vector<int> path_node_origins_;

  pcl::PointCloud<pcl::PointXYZ> pointcloud_;
  pcl::PointCloud<pcl::PointXYZ> reprojected_points_;

  Eigen::Vector3f goal_;
  geometry_msgs::PoseStamped pose_;
  costParameters cost_params_;

 public:
  std::vector<geometry_msgs::Point> path_node_positions_;
  geometry_msgs::Point obstacle_position_;
  std::vector<int> closed_set_;
  int tree_age_;
  std::vector<TreeNode> tree_;

  StarPlanner();
  ~StarPlanner();

  void setParams(double min_cloud_size, double min_dist_backoff,
                 double curr_yaw, double min_realsense_dist,
                 costParameters cost_params);
  void setFOV(double h_FOV, double v_FOV);
  void setReprojectedPoints(
      const pcl::PointCloud<pcl::PointXYZ>& reprojected_points,
      const std::vector<int>& reprojected_points_age);
  void setPose(const geometry_msgs::PoseStamped& pose);
  void setGoal(const geometry_msgs::Point& pose);
  void setCloud(const pcl::PointCloud<pcl::PointXYZ>& cropped_cloud);
  double treeCostFunction(int node_number);
  double treeHeuristicFunction(int node_number);
  void buildLookAheadTree();
  void dynamicReconfigureSetStarParams(
      const avoidance::LocalPlannerNodeConfig& config, uint32_t level);
};
}
#endif  // STAR_PLANNER_H
