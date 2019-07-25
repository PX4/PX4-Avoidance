#ifndef TREE_NODE_H
#define TREE_NODE_H

#include <std_msgs/ColorRGBA.h>
#include <Eigen/Core>
#include <vector>

namespace avoidance {

class TreeNode {
  Eigen::Vector3f position_;
  Eigen::Vector3f velocity_;

 public:
  float total_cost_;
  float distance_cost = 0.0f;
  float velocity_cost = 0.0f;
  float yaw_cost = 0.0f;
  float pitch_cost = 0.0f;
  float heuristic_;
  int origin_;
  bool closed_;

  TreeNode();
  TreeNode(int from, const Eigen::Vector3f& pos, const Eigen::Vector3f& vel);
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
  Eigen::Vector3f getPosition() const;
  Eigen::Vector3f getVelocity() const;

  std_msgs::ColorRGBA getDominantTerm() const {
    std_msgs::ColorRGBA c;
    c.r = distance_cost/total_cost_;
    c.g = velocity_cost/total_cost_;
    c.b = (pitch_cost + yaw_cost)/total_cost_;
    c.a = 1.0f;
    // if (distance_cost >= velocity_cost && distance_cost >= yaw_cost && distance_cost >= pitch_cost) {
    //   c.r = 1.0f;
    //   c.g = 0.0f;
    //   c.b = 0.0f;
    //   c.a = 1.0f;
    // }
    // if (velocity_cost >= distance_cost && velocity_cost >= yaw_cost && velocity_cost >= pitch_cost) {
    //   c.r = 0.0f;
    //   c.g = 1.0f;
    //   c.b = 0.0f;
    //   c.a = 1.0f;
    // }
    // if (yaw_cost >= distance_cost && yaw_cost >= velocity_cost && yaw_cost >= pitch_cost) {
    //   c.r = 0.0f;
    //   c.g = 0.0f;
    //   c.b = 1.0f;
    //   c.a = 1.0f;
    // }
    // if (pitch_cost >= distance_cost && pitch_cost >= velocity_cost && pitch_cost >= yaw_cost) {
    //   c.r = 0.5f;
    //   c.g = 0.0f;
    //   c.b = 0.5f;
    //   c.a = 1.0f;
    // }
    return c;
  }
};
}

#endif  // TREE_NODE_H
