#ifndef TREE_NODE_H
#define TREE_NODE_H

#include <Eigen/Core>
#include <vector>

namespace avoidance {

class TreeNode {
  Eigen::Vector3f position_;

 public:
  float total_cost_;
  float heuristic_;
  float last_e_;
  float last_z_;
  int origin_;
  int depth_;
  float yaw_;

  TreeNode();
  TreeNode(int from, int d, const Eigen::Vector3f& pos);
  ~TreeNode();

  void setCosts(float h, float c);
  Eigen::Vector3f getPosition();
};
}

#endif  // TREE_NODE_H
