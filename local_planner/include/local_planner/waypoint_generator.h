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
  float goal_acceptance_radius_in;
  float goal_acceptance_radius_out;
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
  float curr_yaw_;
  float curr_vel_magnitude_;
  geometry_msgs::TwistStamped curr_vel_;
  ros::Time last_time_{99999.};
  ros::Time current_time_{99999.};

  float smoothing_speed_xy_{10.};
  float smoothing_speed_z_{30.};

  bool reached_goal_;
  float last_yaw_;
  float yaw_reached_goal_;
  float new_yaw_;
  float new_yaw_velocity_ = 0.0f;
  float speed_ = 1.0f;
  float h_FOV_ = 59.0f;
  float v_FOV_ = 46.0f;

  Eigen::Vector3f hover_position_;
  geometry_msgs::PoseStamped last_position_waypoint_;
  Eigen::Vector2f last_velocity_{0.f, 0.f};  ///< last vehicle's velocity

  ros::Time velocity_time_;

  /**
  * @brief     computes position and velocity waypoints based on the input
  *            waypoint_choice
  **/
  void calculateWaypoint();
  /**
  * @brief     computes waypoints when there isn't any obstacle
  **/
  void goStraight();
  /**
  * @brief     computes waypoints to move away from an obstacle
  **/
  void backOff();
  /**
  * @brief     transform a position waypoint into a velocity waypoint
  **/
  void transformPositionToVelocityWaypoint();
  /**
  * @brief     checks if the goal has been reached
  * @returns   true, if the goal has been reached
  **/
  bool withinGoalRadius();
  /**
  * @brief     checks if the goal altitude has been reached. If not, it computes
  *            waypoints to climb to the goal altitude
  **/
  void reachGoalAltitudeFirst();
  /**
  * @brief     smooths waypoints with a critically damped PD controller
  * @param[in] dt, time elapsed between two cycles
  **/
  void smoothWaypoint(float dt);
  /**
  * @brief     set next yaw, smoothed with a critically damped PD controller
  * @param[in] dt, time elapsed between two cycles
  **/
  void nextSmoothYaw(float dt);
  /**
  * @brief     change speed depending on the presence of obstacles, proximity to
  *            the goal, and waypoint lying with the FOV
  **/
  void adaptSpeed();
  /**
  * @brief     adjust waypoints based on new velocity calculation, proximity to
  *            goal, smoothing, climing to goal height. Compute waypoint
  *            orientation
  **/
  void getPathMsg();

 public:
  waypointGenerator_params param_;
  /**
  * @brief     getter method for position and velocity waypoints to be sent to
  *            the FCU
  * @returns   struct with position and velocity waypoints and intermediate
  *            results
  **/
  waypointResult getWaypoints();
  /**
  * @brief     update WaypointGenerator with the latest results of the planning
  *            algorithm
  * @param[in] input, local_planner algorithm result
  **/
  void setPlannerInfo(const avoidanceOutput& input);
  /**
  * @brief set horizontal and vertical Field of View based on camera matrix
  * @param[in] h_FOV, horizontal Field of View [deg]
  * @param[in] v_FOV, vertical Field of View [deg]
  **/
  void setFOV(float h_FOV, float v_FOV);
  /**
  * @brief update with FCU vehice states
  * @param[in] act_pose, current vehicle position and orientation
  * @param[in] goal, current goal
  * @param[in] vel, current vehicle velocity
  * @param[in] stay, true if the vehicle is loitering
  * @param[in] t, update system time
  **/
  void updateState(const geometry_msgs::PoseStamped& act_pose,
                   const geometry_msgs::PoseStamped& goal,
                   const geometry_msgs::TwistStamped& vel, bool stay);

  /**
  * @brief set the responsiveness of the smoothing
  * @param[in] smoothing_speed_xy, set to 0 to disable
  * @param[in] smoothing_speed_z, set to 0 to disable
  **/
  void setSmoothingSpeed(float smoothing_speed_xy, float smoothing_speed_z) {
    smoothing_speed_xy_ = smoothing_speed_xy;
    smoothing_speed_z_ = smoothing_speed_z;
  }

  /**
  * @brief     getter method for the system time
  * @returns   current ROS time
  **/
  virtual ros::Time getSystemTime();

  WaypointGenerator() = default;
  virtual ~WaypointGenerator() = default;
};
}
#endif  // WAYPOINT_GENERATOR_H
