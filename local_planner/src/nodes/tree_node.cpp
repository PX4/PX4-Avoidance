#include "tree_node.h"

namespace avoidance {

TreeNode::TreeNode()
    : total_cost_{0.0f},
      heuristic_{0.0f},
      last_e_{0.0f},
      last_z_{0.0f},
      origin_{0},
      depth_{0},
      yaw_{0.0f} {
  position_ = Eigen::Vector3f::Zero();
}

TreeNode::TreeNode(int from, int d, const Eigen::Vector3f& pos)
    : total_cost_{0.0f},
      heuristic_{0.0f},
      last_e_{0.0f},
      last_z_{0.0f},
      origin_{from},
      depth_{d},
      yaw_{0.0f} {
  position_ = pos;
}

TreeNode::~TreeNode() {}

void TreeNode::setCosts(float h, float c) {
  heuristic_ = h;
  total_cost_ = c;
}

Eigen::Vector3f TreeNode::getPosition() { return position_; }
}
