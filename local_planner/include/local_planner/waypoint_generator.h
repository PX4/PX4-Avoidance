#ifndef WAYPOINT_GENERATOR_H
#define WAYPOINT_GENERATOR_H

#include <avoidance/usm.h>
#include "avoidance/common.h"
#include "avoidance_output.h"

#include <Eigen/Dense>

#include <ros/time.h>

#include <string>
#include <vector>

namespace avoidance {

enum class PlannerState { TRY_PATH, ALTITUDE_CHANGE, LOITER, DIRECT };
std::string toString(PlannerState state);  // for logging

struct waypointResult {
  avoidance::PlannerState waypoint_type;
  Eigen::Vector3f position_wp;
  Eigen::Quaternionf orientation_wp;
  Eigen::Vector3f linear_velocity_wp;
  Eigen::Vector3f angular_velocity_wp;
  Eigen::Vector3f goto_position;           // correction direction, dist=1
  Eigen::Vector3f adapted_goto_position;   // correction direction & dist
  Eigen::Vector3f smoothed_goto_position;  // what is sent to the drone
};

class WaypointGenerator : public usm::StateMachine<PlannerState> {
 private:
  avoidanceOutput planner_info_;
  waypointResult output_;

  std::mutex running_mutex_;
  Eigen::Vector3f smoothed_goto_location_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f smoothed_goto_location_velocity_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f position_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f velocity_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f goal_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f prev_goal_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector2f closest_pt_ = Eigen::Vector2f(NAN, NAN);
  Eigen::Vector3f tmp_goal_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f desired_vel_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f change_altitude_pos_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f hover_position_ = Eigen::Vector3f(NAN, NAN, NAN);

  float curr_yaw_rad_ = NAN;
  float curr_pitch_deg_ = NAN;
  ros::Time last_time_{99999.};
  ros::Time current_time_{99999.};

  float smoothing_speed_xy_{10.f};
  float smoothing_speed_z_{3.0f};

  bool is_airborne_ = false;
  bool is_land_waypoint_{false};
  bool is_takeoff_waypoint_{false};
  bool reach_altitude_offboard_{false};
  bool auto_land_{false};
  bool loiter_{false};
  float setpoint_yaw_rad_ = 0.0f;
  float setpoint_yaw_velocity_ = 0.0f;
  float heading_at_goal_rad_ = NAN;
  float yaw_reach_height_rad_ = NAN;
  float speed_ = 0.0f;

  std::vector<FOV> fov_fcu_frame_;

  NavigationState nav_state_ = NavigationState::none;

  ros::Time velocity_time_;

  // state
  bool trigger_reset_ = false;
  bool state_changed_ = false;
  PlannerState prev_slp_state_ = PlannerState::TRY_PATH;
  usm::Transition runTryPath();
  usm::Transition runAltitudeChange();
  usm::Transition runLoiter();
  usm::Transition runDirect();

  /**
  * @brief iterate the statemachine
  */
  usm::Transition runCurrentState() override final;

  /**
  * @brief the setup of the statemachine
  */
  PlannerState chooseNextState(PlannerState currentState, usm::Transition transition) override final;

  /**
  * @brief     computes position and velocity waypoints based on the input
  *            waypoint_choice
  **/
  void calculateWaypoint();

  /**
  * @brief     transform a position waypoint into a velocity waypoint
  **/
  void transformPositionToVelocityWaypoint();

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
  * @param[in] dt, time elapsed between two cycles
  **/
  void adaptSpeed(float dt);
  /**
  * @brief     adjust waypoints based on new velocity calculation, proximity to
  *            goal, smoothing, climing to goal height. Compute waypoint
  *            orientation
  **/
  void getPathMsg();

  bool isAltitudeChange();

 public:
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
  * @param[in] index of the camera
  * @param[in] FOV structures defining the FOV of the specific camera
  **/
  void setFOV(int i, const FOV& fov);

  /**
  * @brief update with FCU vehice states
  * @param[in] act_pose, current vehicle position
  * @param[in] q, current vehicle orientation
  * @param[in] goal, current goal position
  * @param[in] prev_goal, previous goal position
  * @param[in] vel, current vehicle velocity
  * @param[in] stay, true if the vehicle is loitering
  * @param[in] is_airborne, true if the vehicle is armed and in avoidance enabled mode
  * @param[in] nav_state, vehicle navigation state
  * @param[in] is_land_waypoint, true if the current mission item is a land waypoint
  * @param[in] is_takeoff_waypoint, true if the current mission item is a takeoff waypoint
  * @param[in] desired_vel, velocity setpoint from the Firmware
  **/
  void updateState(const Eigen::Vector3f& act_pose, const Eigen::Quaternionf& q, const Eigen::Vector3f& goal,
                   const Eigen::Vector3f& prev_goal, const Eigen::Vector3f& vel, bool stay, bool is_airborne,
                   const NavigationState& nav_state, const bool is_land_waypoint, const bool is_takeoff_waypoint,
                   const Eigen::Vector3f& desired_vel);

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

  /**
  * @brief     getter method to visualize offtrack state
  * @param[in] closest_pt, vehicle position projection on the line previous to
  * current goal
  * @param[in] deg60_pt, 60 degrees angle entry point to line previous to
  * current goal from current vehicle postion
  **/
  void getOfftrackPointsForVisualization(Eigen::Vector3f& closest_pt, Eigen::Vector3f& deg60_pt);

  WaypointGenerator();
  virtual ~WaypointGenerator() = default;
};
}
#endif  // WAYPOINT_GENERATOR_H
