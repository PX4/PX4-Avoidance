#include "tree_node.h"

namespace avoidance {

TreeNode::TreeNode()
    : total_cost_{0.0},
      heuristic_{0.0},
      last_e_{0},
      last_z_{0},
      origin_{0},
      depth_{0},
      yaw_{0.0} {
  position_ = Eigen::Vector3f::Zero();
}

TreeNode::TreeNode(int from, int d, const Eigen::Vector3f& pos)
    : total_cost_{0.0},
      heuristic_{0.0},
      last_e_{0},
      last_z_{0},
      origin_{from},
      depth_{d},
      yaw_{0.0} {
  position_ = pos;
}

TreeNode::~TreeNode() {}

void TreeNode::setCosts(double h, double c) {
  heuristic_ = h;
  total_cost_ = c;
}

Eigen::Vector3f TreeNode::getPosition() { return position_; }
}
