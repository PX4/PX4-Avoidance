#ifndef TREE_NODE_H
#define TREE_NODE_H

#include <geometry_msgs/Point.h>
#include <math.h>
#include <vector>

class TreeNode {
  geometry_msgs::Point position_;

 public:
  double total_cost_;
  double heuristic_;
  int last_e_;
  int last_z_;
  int origin_;
  int depth_;
  double yaw_;

  TreeNode();
  TreeNode(int from, int d, geometry_msgs::Point pos);
  ~TreeNode();

  void setCosts(double h, double c);
  geometry_msgs::Point getPosition();
};

#endif  // TREE_NODE_H
