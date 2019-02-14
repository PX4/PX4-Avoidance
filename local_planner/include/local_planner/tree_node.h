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
  ~TreeNode() = default;

  /**
  * @brief     setter method for heuristic and cost of a tree node
  * @param[in] h, heuristic
  * @param[in] c, cost
  **/
  void setCosts(float h, float c);

  /**
  * @brief     getter method for tree node position
  * @returns   node position in 3D cartesian coordinates
  **/
  Eigen::Vector3f getPosition();
};
}

#endif  // TREE_NODE_H
