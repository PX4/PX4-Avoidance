#ifndef TREE_NODE_H
#define TREE_NODE_H

#include <math.h>
#include <eigen3/Eigen/Core>
#include <vector>

namespace avoidance {

class TreeNode {
  Eigen::Vector3f position_;

 public:
  double total_cost_;
  double heuristic_;
  float last_e_;
  float last_z_;
  int origin_;
  int depth_;
  double yaw_;

  TreeNode();
  TreeNode(int from, int d, const Eigen::Vector3f& pos);
  ~TreeNode();

  void setCosts(double h, double c);
  Eigen::Vector3f getPosition();
};
}

#endif  // TREE_NODE_H
