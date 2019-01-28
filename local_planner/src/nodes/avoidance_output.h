#pragma once

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include <vector>

namespace avoidance {

enum waypoint_choice { hover, costmap, tryPath, direct, reachHeight, goBack };

struct avoidanceOutput {
  waypoint_choice waypoint_type;

  geometry_msgs::PoseStamped pose;
  bool obstacle_ahead;
  bool reach_altitude;
  double min_speed;
  double max_speed;
  double velocity_sigmoid_slope;
  ros::Time last_path_time;

  geometry_msgs::Point back_off_point;
  geometry_msgs::Point back_off_start_point;
  double min_dist_backoff;

  geometry_msgs::PoseStamped take_off_pose;
  geometry_msgs::PoseStamped offboard_pose;

  double costmap_direction_e;
  double costmap_direction_z;
  std::vector<geometry_msgs::Point> path_node_positions;
};
}
