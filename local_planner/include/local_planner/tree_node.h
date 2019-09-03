#ifndef TREE_NODE_H
#define TREE_NODE_H

#include <Eigen/Core>
#include <vector>
#include "trajectory_simulator.h"

namespace avoidance {

class TreeNode {
 public:
  float total_cost_;
  float cost_;
  float heuristic_;
  int origin_;
  bool closed_;
  simulation_state state;    // State containing position, velocity and time of the drone
  Eigen::Vector3f setpoint;  // Setpoint required to send to PX4 in order to get to this state

  TreeNode() = default;
  TreeNode(int from, const simulation_state& start_state, const Eigen::Vector3f& sp, const float cost);
  ~TreeNode() = default;

  /**
  * @brief     setter method for heuristic and cost of a tree node
  * @param[in] h, heuristic
  * @param[in] c, cost
  **/
  void setCosts(float h, float c);

  /**
  * @defgroup  getterFunctions
  * @brief     getter methods for tree node position, velocity and setpoint
  * @returns   node position in 3D cartesian coordinates
  * @{
  **/
  Eigen::Vector3f getPosition() const;
  Eigen::Vector3f getVelocity() const;
  Eigen::Vector3f getSetpoint() const;
  /** @} */  // end of doxygen group getterFunctions
};
}

#endif  // TREE_NODE_H
