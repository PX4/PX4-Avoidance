#pragma once

#include <ros/time.h>
#include <Eigen/Dense>

#include <vector>

namespace avoidance {

enum waypoint_choice { hover, tryPath, direct, reachHeight };

struct avoidanceOutput {
  float cruise_velocity;     // mission cruise velocity
  ros::Time last_path_time;  // finish built time for the VFH+* tree

  std::vector<Eigen::Vector3f> path_node_positions;  // array of tree nodes
                                                     // position, each node
                                                     // is the minimum cost
                                                     // node for each tree
                                                     // depth level
};
}
