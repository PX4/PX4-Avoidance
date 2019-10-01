#include "local_planner/tree_node.h"

namespace avoidance {

TreeNode::TreeNode(int parent, const simulation_state& start_state, const Eigen::Vector3f& sp)
    : total_cost_{0.0f}, heuristic_{0.0f}, parent_{parent}, closed_{false}, state(start_state), setpoint(sp) {}

void TreeNode::setCosts(float h, float c) {
  heuristic_ = h;
  total_cost_ = c;
}

Eigen::Vector3f TreeNode::getPosition() const { return state.position; }
Eigen::Vector3f TreeNode::getVelocity() const { return state.velocity; }
Eigen::Vector3f TreeNode::getSetpoint() const { return setpoint; }
}
