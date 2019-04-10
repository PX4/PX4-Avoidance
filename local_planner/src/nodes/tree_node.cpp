#include "local_planner/tree_node.h"

namespace avoidance {

TreeNode::TreeNode(int from, int d, const Eigen::Vector3f& commanded_direction,
                   const simulation_state& state,
                   const simulation_limits& limits)
    : total_cost_{0.0f},
      heuristic_{0.0f},
      last_e_{0.0f},
      last_z_{0.0f},
      origin_{from},
      depth_{d},
      yaw_{0.0f},
      sim_(limits, state),
      commanded_direction_{commanded_direction} {
  actual_position_ = state.position;
}

void TreeNode::setCosts(float h, float c) {
  heuristic_ = h;
  total_cost_ = c;
}

Eigen::Vector3f TreeNode::getPosition() const { return actual_position_; }
}
