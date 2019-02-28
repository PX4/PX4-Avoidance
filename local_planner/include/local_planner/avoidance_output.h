#pragma once

#include <ros/time.h>
#include <Eigen/Dense>

#include <vector>

namespace avoidance {

enum waypoint_choice { hover, costmap, tryPath, direct, reachHeight, goBack };

struct avoidanceOutput {
  waypoint_choice waypoint_type;
  bool obstacle_ahead;  // true is there is an obstacle ahead of the vehicle
  float velocity_around_obstacles;    // maximal velocity in the proximity of
                                      // obstacles
  float velocity_far_from_obstacles;  // maximal velocity with no obstacles in
                                      // sight
  ros::Time last_path_time;           // finish built time for the VFH+* tree

  Eigen::Vector3f back_off_point;  // closest point to the vehicle in the cloud
  Eigen::Vector3f back_off_start_point;  // vehicle position when a point in the
  // cloud is closer than  min_dist_backoff
  float min_dist_backoff;  // distance between the vehicle and the closest
                           // point in the cloud

  Eigen::Vector3f take_off_pose;  // last vehicle position when not armed

  float costmap_direction_e;  // elevation angle of the minimum cost histogram
                              // cell
  float
      costmap_direction_z;  // azimuth angle of the minimum cost histogram cell
  std::vector<Eigen::Vector3f> path_node_positions;  // array of tree nodes
                                                     // position, each node
                                                     // is the minimum cost
                                                     // node for each tree
                                                     // depth level
};
}
