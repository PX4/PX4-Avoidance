#pragma once

#include <ros/time.h>
#include <Eigen/Dense>

#include <vector>

namespace avoidance {

enum waypoint_choice { hover, tryPath, direct, reachHeight };

struct avoidanceOutput {
  waypoint_choice waypoint_type = hover;
  bool obstacle_ahead;    // true is there is an obstacle ahead of the vehicle
  float cruise_velocity;  // mission cruise velocity
  float tree_node_duration;
  ros::Time last_path_time;  // finish built time for the VFH+* tree

  Eigen::Vector3f take_off_pose;  // last vehicle position when not armed

  std::vector<Eigen::Vector3f> path_node_setpoints;  // array of setpoints
};
}
