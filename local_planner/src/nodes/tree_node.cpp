#include "tree_node.h"

TreeNode::TreeNode()
    : heuristic { 0 },
      total_cost { 0 },
      origin { 0 },
      depth { 0 },
      last_e { 0 },
      last_z { 0 } {

  position.x = 0;
  position.y = 0;
  position.z = 0;
}

TreeNode::TreeNode(int from, int d, geometry_msgs::Point pos)
    : heuristic { 0 },
      total_cost { 0 },
      origin { from },
      depth { d },
      last_e { 0 },
      last_z { 0 } {

  position = pos;
}

TreeNode::~TreeNode() {
}


void TreeNode::setCosts(double h, double c){
  heuristic = h;
  total_cost = c;
}

geometry_msgs::Point TreeNode::getPosition(){
  return position;
}

