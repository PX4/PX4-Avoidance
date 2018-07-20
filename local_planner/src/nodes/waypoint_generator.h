#ifndef WAYPOINT_GENERATOR_H
#define WAYPOINT_GENERATOR_H

#include <math.h>
#include <Eigen/Dense>
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <ros/ros.h>

#include "common.h"
#include "planner_functions.h"
#include "local_planner.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>


struct waypointResult {
  waypoint_choice waypoint_type;
  nav_msgs::Path path;
  geometry_msgs::PoseStamped position_waypoint;
  geometry_msgs::Twist velocity_waypoint;
  geometry_msgs::Point goto_position;
  geometry_msgs::Point adapted_goto_position;
  geometry_msgs::Point smoothed_goto_position;
};

class WaypointGenerator {
 private:
  avoidanceOutput planner_info_;
  waypointResult output_;
  waypoint_choice last_wp_type_;

  geometry_msgs::PoseStamped pose_;
  geometry_msgs::Point goal_;
  double curr_yaw_;
  double curr_vel_magnitude_;
  ros::Time update_time_;
  geometry_msgs::TwistStamped curr_vel_;

  bool reached_goal_;
  bool reach_altitude_ = false;
  bool waypoint_outside_FOV_;
  bool only_yawed_;
  double last_yaw_;
  double yaw_reached_goal_;
  double new_yaw_;
  double speed_ = 1.0;
  int e_FOV_max_, e_FOV_min_;

  geometry_msgs::Point hover_position_;
  geometry_msgs::PoseStamped last_position_waypoint_;
  geometry_msgs::PoseStamped last_last_position_waypoint_;

  ros::Time velocity_time_;
  std::vector<int> z_FOV_idx_;
  std::clock_t last_t_smooth_ = 0.0f;

  void calculateWaypoint();
  void updateState();
  void goFast();
  void backOff();
  void transformPositionToVelocityWaypoint();
  bool withinGoalRadius();
  void reachGoalAltitudeFirst();
  void smoothWaypoint();
  void adaptSpeed();
  void getPathMsg();

 public:
  void getWaypoints(waypointResult &output);
  void setPlannerInfo(avoidanceOutput input);
  void updateState(geometry_msgs::PoseStamped act_pose,
                   geometry_msgs::PoseStamped goal,
                   geometry_msgs::TwistStamped vel, bool stay, ros::Time t);

  WaypointGenerator();
  ~WaypointGenerator();
};

#endif  // WAYPOINT_GENERATOR_H
