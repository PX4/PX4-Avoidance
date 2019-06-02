#ifndef STAR_PLANNER_H
#define STAR_PLANNER_H

#include "avoidance/histogram.h"
#include "box.h"
#include "cost_parameters.h"

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <nav_msgs/GridCells.h>

#include <dynamic_reconfigure/server.h>
#include <local_planner/LocalPlannerNodeConfig.h>

#include <vector>

namespace avoidance {
class TreeNode;

class StarPlanner {
  int children_per_node_ = 1;
  int n_expanded_nodes_ = 5;
  float tree_node_distance_ = 1.0f;
  float tree_discount_factor_ = 0.8f;
  float max_path_length_ = 4.f;
  float curr_yaw_histogram_frame_deg_ = 90.f;
  float smoothing_margin_degrees_ = 30.f;

  std::vector<int> path_node_origins_;

  pcl::PointCloud<pcl::PointXYZI> cloud_;

  Eigen::Vector3f goal_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f projected_last_wp_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f position_ = Eigen::Vector3f(NAN, NAN, NAN);
  costParameters cost_params_;

 protected:
  /**
  * @brief     computes the cost of a node
  * @param[in] node_number, sequential number of entry in the tree
  * @returns
  **/
  float treeCostFunction(int node_number) const;

  /**
  * @brief     computes the heuristic for a node
  * @param[in] node_number, sequential number of entry in the tree
  * @returns
  **/
  float treeHeuristicFunction(int node_number) const;

 public:
  std::vector<Eigen::Vector3f> path_node_positions_;
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
  * @brief     setter method for last sent waypoint
  * @param[in] projected_last_wp, last waypoint projected out to goal distance
  **/
  void setLastDirection(const Eigen::Vector3f& projected_last_wp);

  /**
  * @brief     setter method for star_planner pointcloud
  * @param[in] cloud, processed data already cropped and combined with history
  **/
  void setPointcloud(const pcl::PointCloud<pcl::PointXYZI>& cloud);

  /**
  * @brief     setter method for vehicle position
  * @param[in] pos, vehicle current position and orientation
  * @param[in] curr_yaw, vehicle current yaw
  **/
  void setPose(const Eigen::Vector3f& pos, float curr_yaw);

  /**
  * @brief     setter method for current goal
  * @param[in] goal, current goal position
  **/
  void setGoal(const Eigen::Vector3f& pose);

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
