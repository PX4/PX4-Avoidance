#include "tree_node.h"

TreeNode::TreeNode()
    : heuristic_{0.0},
      total_cost_{0.0},
      origin_{0},
      depth_{0},
      last_e_{0},
      last_z_{0},
      yaw_{0.0} {
  position_.x = 0.0;
  position_.y = 0.0;
  position_.z = 0.0;
}

TreeNode::TreeNode(int from, int d, geometry_msgs::Point pos)
    : heuristic_{0.0},
      total_cost_{0.0},
      origin_{from},
      depth_{d},
      last_e_{0},
      last_z_{0},
      yaw_{0.0} {
  position_ = pos;
}

TreeNode::~TreeNode() {}

void TreeNode::setCosts(double h, double c) {
  heuristic_ = h;
  total_cost_ = c;
}

geometry_msgs::Point TreeNode::getPosition() { return position_; }
