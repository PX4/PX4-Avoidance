#ifndef STAR_PLANNER_H
#define STAR_PLANNER_H

#include "avoidance/histogram.h"
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
  float max_path_length_ = 4.f;
  float smoothing_margin_degrees_ = 30.f;
  float tree_heuristic_weight_ = 10.0f;
  float max_sensor_range_ = 15.f;
  float min_sensor_range_ = 0.2f;

  pcl::PointCloud<pcl::PointXYZI> cloud_;

  Eigen::Vector3f goal_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f position_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f velocity_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f closest_pt_ = Eigen::Vector3f(NAN, NAN, NAN);
  costParameters cost_params_;

 protected:
  /**
  * @brief     computes the heuristic for a node
  * @param[in] node_number, sequential number of entry in the tree
  * @returns
  **/
  float treeHeuristicFunction(int node_number) const;

 public:
  std::vector<Eigen::Vector3f> path_node_positions_;
  std::vector<int> closed_set_;
  std::vector<TreeNode> tree_;

  StarPlanner();
  ~StarPlanner() = default;

  /**
  * @brief     setter method for costMatrix paramters
  * @param[in] cost_params, parameters for the histogram cost function
  **/
  void setParams(costParameters cost_params);

  /**
  * @brief     setter method for star_planner pointcloud
  * @param[in] cloud, processed data already cropped and combined with history
  **/
  void setPointcloud(const pcl::PointCloud<pcl::PointXYZI>& cloud);

  /**
  * @brief     setter method for vehicle position
  * @param[in] vehicle current position
  **/
  void setPose(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel);

  /**
  * @brief     setter method for vehicle position projection on the line between the current and previous goal
  * @param[in] closest_pt, projection point
  **/
  void setClosestPointOnLine(const Eigen::Vector3f& closest_pt);

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
  void dynamicReconfigureSetStarParams(const avoidance::LocalPlannerNodeConfig& config, uint32_t level);
};
}
#endif  // STAR_PLANNER_H
