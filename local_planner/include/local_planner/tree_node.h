#ifndef TREE_NODE_H
#define TREE_NODE_H

#include <Eigen/Core>
#include <vector>
#include "trajectory_simulator.h"

namespace avoidance {

class TreeNode {
 public:
  float total_cost_;
  float heuristic_;
  int origin_;
  bool closed_;
  simulation_state state;

  TreeNode();
  TreeNode(int from, const simulation_state& start_state);
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
  Eigen::Vector3f getVelocity() const;
};
}

#endif  // TREE_NODE_H
