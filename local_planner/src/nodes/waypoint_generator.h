#ifndef WAYPOINT_GENERATOR_H
#define WAYPOINT_GENERATOR_H

#include "avoidance_output.h"

#include <Eigen/Dense>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <ros/time.h>

#include <string>
#include <vector>

namespace avoidance {

struct waypointResult {
  waypoint_choice waypoint_type;
  geometry_msgs::PoseStamped position_waypoint;
  geometry_msgs::Twist velocity_waypoint;
  geometry_msgs::Point goto_position;           // correction direction, dist=1
  geometry_msgs::Point adapted_goto_position;   // correction direction & dist
  geometry_msgs::Point smoothed_goto_position;  // what is sent to the drone
};

struct waypointGenerator_params {
  double goal_acceptance_radius_in;
  double goal_acceptance_radius_out;
  double factor_close_to_goal_start_speed_limitation;
  double factor_close_to_goal_stop_speed_limitation;
  double min_speed_close_to_goal;
  double max_speed_close_to_goal_factor;
};

class WaypointGenerator {
 private:
  avoidanceOutput planner_info_;
  waypointResult output_;
  waypoint_choice last_wp_type_;

  Eigen::Vector3f smoothed_goto_location_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f smoothed_goto_location_velocity_ = Eigen::Vector3f::Zero();

  geometry_msgs::PoseStamped pose_;
  Eigen::Vector3f goal_;
  double curr_yaw_;
  double curr_vel_magnitude_;
  ros::Time update_time_;
  geometry_msgs::TwistStamped curr_vel_;
  ros::Time last_time_{0.};
  ros::Time current_time_{0.};

  double smoothing_speed_{10.};

  bool reached_goal_;
  bool limit_speed_close_to_goal_ = false;
  bool waypoint_outside_FOV_;
  bool only_yawed_;
  double last_yaw_;
  double yaw_reached_goal_;
  double new_yaw_;
  double speed_ = 1.0;
  int e_FOV_max_, e_FOV_min_;
  double h_FOV_ = 59.0;
  double v_FOV_ = 46.0;

  Eigen::Vector3f hover_position_;
  geometry_msgs::PoseStamped last_position_waypoint_;
  Eigen::Vector2f last_velocity_{0.f, 0.f};  ///< last vehicle's velocity

  ros::Time velocity_time_;
  std::vector<int> z_FOV_idx_;

  void calculateWaypoint();
  void updateState();
  void goFast();
  void backOff();
  void transformPositionToVelocityWaypoint();
  bool withinGoalRadius();
  void reachGoalAltitudeFirst();
  void smoothWaypoint(double dt);
  void adaptSpeed();
  void getPathMsg();

 public:
  waypointGenerator_params param_;
  waypointResult getWaypoints();
  void setPlannerInfo(const avoidanceOutput& input);
  void setFOV(double h_FOV, double v_FOV);
  void updateState(const geometry_msgs::PoseStamped& act_pose,
                   const geometry_msgs::PoseStamped& goal,
                   const geometry_msgs::TwistStamped& vel, bool stay,
                   ros::Time t);

  /**
   * Set the responsiveness of the smoothing. Set to 0 to disable.
   */
  void setSmoothingSpeed(double smoothing_speed) {
    smoothing_speed_ = smoothing_speed;
  }

  WaypointGenerator();
  ~WaypointGenerator();
};
}
#endif  // WAYPOINT_GENERATOR_H
