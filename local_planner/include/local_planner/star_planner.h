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
  float h_FOV_ = 59.0f;
  float v_FOV_ = 46.0f;
  int children_per_node_ = 1;
  int n_expanded_nodes_ = 5;
  int n_points_occupied_ = 10;
  float tree_node_distance_ = 1.0f;
  float tree_discount_factor_ = 0.8f;
  float curr_yaw_;

  std::vector<int> reprojected_points_age_;
  std::vector<int> path_node_origins_;

  pcl::PointCloud<pcl::PointXYZ> pointcloud_;
  pcl::PointCloud<pcl::PointXYZ> reprojected_points_;

  Eigen::Vector3f goal_;
  geometry_msgs::PoseStamped pose_;
  costParameters cost_params_;

 protected:
  /**
  * @brief     computes the cost of a node
  * @param[in] node_number, sequential number of entry in the tree
  * @returns
  **/
  float treeCostFunction(int node_number);

  /**
  * @brief     computes the heuristic for a node
  * @param[in] node_number, sequential number of entry in the tree
  * @returns
  **/
  float treeHeuristicFunction(int node_number);

 public:
  std::vector<geometry_msgs::Point> path_node_positions_;
  std::vector<int> closed_set_;
  int tree_age_;
  std::vector<TreeNode> tree_;

  StarPlanner();
  ~StarPlanner() = default;

  /**
  * @brief     setter method for costMatrix paramters
  * @param[in] cost_params, parameters for the histogram cost function
  **/
  void setParams(costParameters cost_params);

  /**
  * @brief     setter method for Fielf of View
  * @param[in] h_FOV, horizontal Field of View [deg]
  * @param[in] v_FOV, vertical Field of View [deg]
  **/
  void setFOV(float h_FOV, float v_FOV);

  /**
  * @brief     setter method for reprojected pointcloud
  * @param[in] reprojected_points, pointcloud from previous frames reprojected
  *            around the vehicle current position
  * @param[in] reprojected_points_age, array containing the age of each
  *            reprojected point
  **/
  void setReprojectedPoints(
      const pcl::PointCloud<pcl::PointXYZ>& reprojected_points,
      const std::vector<int>& reprojected_points_age);

  /**
  * @brief     setter method for vehicle position
  * @param[in] pose, vehicle current position and orientation
  * @param[in] curr_yaw, vehicle current yaw
  **/
  void setPose(const geometry_msgs::PoseStamped& pose, float curr_yaw);

  /**
  * @brief     setter method for current goal
  * @param[in] goal, current goal position
  **/
  void setGoal(const geometry_msgs::Point& pose);

  /**
  * @brief     setter method for pointcloud
  * @param[in] cropped_cloud, current point cloud cropped around the vehicle
  **/
  void setCloud(const pcl::PointCloud<pcl::PointXYZ>& cropped_cloud);

  /**
  * @brief     build tree of candidates directions towards the goal
  **/
  void buildLookAheadTree();

  /**
  * @brief     setter method for server paramters
  **/
  void dynamicReconfigureSetStarParams(
      const avoidance::LocalPlannerNodeConfig& config, uint32_t level);
};
}
#endif  // STAR_PLANNER_H
