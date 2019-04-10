#ifndef TREE_NODE_H
#define TREE_NODE_H

#include <Eigen/Core>
#include <vector>
#include "trajectory_simulator.h"

namespace avoidance {

class TreeNode {
  Eigen::Vector3f actual_position_;

 public:
  float total_cost_;
  float heuristic_;
  float last_e_;
  float last_z_;
  int origin_;
  int depth_;
  float yaw_;
  TrajectorySimulator sim_;
  std::vector<simulation_state> actual_states_to_node_;
  Eigen::Vector3f commanded_direction_;

  TreeNode(int from, int d, const Eigen::Vector3f& commanded_direction,
           const simulation_state& state, const simulation_limits& limits);
  ~TreeNode() = default;

  /**
  * @brief     setter method for heuristic and cost of a tree node
  * @param[in] h, heuristic
  * @param[in] c, cost
  **/
  void setCosts(float h, float c);

  /**
  * @brief     getter method for tree node position
  * @returns   node position in 3D cartesian coordinates
  **/
  Eigen::Vector3f getPosition() const;
};
}

#endif  // TREE_NODE_H
