#ifndef GLOBAL_PLANNER_WAYPOINT_GENERATOR_H
#define GLOBAL_PLANNER_WAYPOINT_GENERATOR_H

#include <avoidance/usm.h>
#include <Eigen/Dense>
#include "avoidance/common.h"
#include "global_planner/avoidance_output.h"

#include <ros/time.h>

#include <string>
#include <vector>

namespace global_planner {

enum class PlannerState { NAVIGATE, LOITER, DIRECT };
// std::string toString(PlannerState state);  // for logging

struct waypointResult {
  global_planner::PlannerState waypoint_type;
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

  Eigen::Vector3f smoothed_goto_location_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f smoothed_goto_location_velocity_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f position_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f loiter_position_ = Eigen::Vector3f(0.0, 0.0, 3.5);
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
  float smoothing_speed_z_{10.0f};

  bool is_airborne_ = false;
  bool is_land_waypoint_{false};
  bool is_takeoff_waypoint_{false};
  bool reach_altitude_offboard_{false};
  bool auto_land_{false};
  bool loiter_{false};
  bool path_in_collision_{false};
  float setpoint_yaw_rad_ = 0.0f;
  float setpoint_yaw_velocity_ = 0.0f;
  float heading_at_goal_rad_ = NAN;
  float yaw_reach_height_rad_ = NAN;
  float speed_ = 1.0f;
  std::vector<avoidance::FOV> fov_fcu_frame_;

  avoidance::NavigationState nav_state_ = avoidance::NavigationState::none;

  //   // state
  bool trigger_reset_ = false;
  bool state_changed_ = false;
  bool planner_path_exists = false;
  PlannerState prev_planner_state_ = PlannerState::DIRECT;
  usm::Transition runLoiter();
  usm::Transition runDirect();
  usm::Transition runNavigate();

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
  * @brief     smooths waypoints with a critically damped PD controller
  * @param[in] dt, time elapsed between two cycles
  **/
  void smoothWaypoint(float dt);
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

  //   bool isAltitudeChange();

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
                   const avoidance::NavigationState& nav_state, const bool is_land_waypoint,
                   const bool is_takeoff_waypoint, const Eigen::Vector3f& desired_vel, const bool path_in_collision);

  bool getSetpointFromPath(const std::vector<Eigen::Vector3f>& path, const ros::Time& path_generation_time,
                           float velocity, Eigen::Vector3f& setpoint);
//   /**
//   * @brief set the responsiveness of the smoothing
//   * @param[in] smoothing_speed_xy, set to 0 to disable
//   * @param[in] smoothing_speed_z, set to 0 to disable
//   **/
//   void setSmoothingSpeed(float smoothing_speed_xy, float smoothing_speed_z) {
//     smoothing_speed_xy_ = smoothing_speed_xy;
//     smoothing_speed_z_ = smoothing_speed_z;
//   }

//   /**
//   * @brief     getter method for the system time
//   * @returns   current ROS time
//   **/
//   virtual ros::Time getSystemTime();

  WaypointGenerator();
  virtual ~WaypointGenerator() = default;
};
}
#endif  // GLOBAL_PLANNER_WAYPOINT_GENERATOR_H
