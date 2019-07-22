#include "local_planner/tree_node.h"

namespace avoidance {

TreeNode::TreeNode() : total_cost_{0.0f}, heuristic_{0.0f}, origin_{0}, closed_{false} {
  state.position = Eigen::Vector3f::Zero();
  state.velocity = Eigen::Vector3f::Zero();
  state.acceleration = Eigen::Vector3f::Zero();
}

TreeNode::TreeNode(int from, const simulation_state& start_state)
    : total_cost_{0.0f}, heuristic_{0.0f}, origin_{from}, closed_{false} {
  state.position = start_state.position;
  state.velocity = start_state.velocity;
  state.acceleration = start_state.acceleration;
  state.time = start_state.time;
}

void TreeNode::setCosts(float h, float c) {
  heuristic_ = h;
  total_cost_ = c;
}

Eigen::Vector3f TreeNode::getPosition() const { return state.position; }
Eigen::Vector3f TreeNode::getVelocity() const { return state.velocity; }
}
