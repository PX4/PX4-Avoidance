#include <ros/param.h>

#include "avoidance/common.h"
#include "global_planner/waypoint_generator.h"

namespace global_planner {

WaypointGenerator::WaypointGenerator() : usm::StateMachine<PlannerState>(PlannerState::LOITER) {}

using global_planner::PlannerState;
std::string toString(PlannerState state) {
  std::string state_str = "unknown";
  switch (state) {
    case PlannerState::NAVIGATE:
      state_str = "NAVIGATE";
      break;
    case PlannerState::LOITER:
      state_str = "LOITER";
      break;
    case PlannerState::DIRECT:
      state_str = "DIRECT";
      break;
  }
  return state_str;
}

PlannerState WaypointGenerator::chooseNextState(PlannerState currentState, usm::Transition transition) {
  prev_planner_state_ = currentState;
  state_changed_ = true;

  // clang-format off
  USM_TABLE(
      currentState, PlannerState::DIRECT,
      USM_STATE(transition, PlannerState::NAVIGATE, USM_MAP(usm::Transition::NEXT1, PlannerState::DIRECT);
         USM_MAP(usm::Transition::NEXT2, PlannerState::LOITER));
      USM_STATE(transition, PlannerState::LOITER, USM_MAP(usm::Transition::NEXT1, PlannerState::DIRECT);
         USM_MAP(usm::Transition::NEXT2, PlannerState::NAVIGATE));
      USM_STATE(transition, PlannerState::DIRECT, USM_MAP(usm::Transition::NEXT1, PlannerState::NAVIGATE);
         USM_MAP(usm::Transition::NEXT2, PlannerState::LOITER));
            // clang-format on
            );
}

usm::Transition WaypointGenerator::runCurrentState() {
  if (trigger_reset_) {
    trigger_reset_ = false;
    return usm::Transition::ERROR;
  }
  usm::Transition t;
  switch (getState()) {
    case PlannerState::NAVIGATE:
      t = runNavigate();
      break;

    case PlannerState::LOITER:
      t = runLoiter();
      break;

    case PlannerState::DIRECT:
      t = runDirect();
      break;
  }
  state_changed_ = false;
  return t;
}

usm::Transition WaypointGenerator::runLoiter() {
  // Loiter in position when a collision free path cannot be found
  output_.goto_position = loiter_position_;

  // Escape lotier when planner comes up with a path
  planner_path_exists = bool(planner_info_.path_node_positions.size() > 0);

  getPathMsg();
  if (planner_path_exists) {
    if (path_in_collision_)
      return usm::Transition::NEXT2;  // NAVIGATE
    else
      return usm::Transition::NEXT1;  // DIRECT
  } else
    return usm::Transition::REPEAT;
}

usm::Transition WaypointGenerator::runDirect() {
  Eigen::Vector3f dir = (goal_ - position_).normalized();
  output_.goto_position = position_ + dir;

  ROS_INFO("[WG] Going straight to selected waypoint: [%f, %f, %f].", output_.goto_position.x(),
           output_.goto_position.y(), output_.goto_position.z());

  getPathMsg();

  planner_path_exists = bool(planner_info_.path_node_positions.size() > 0);
  // Update Loiter postiion
  loiter_position_ = position_;
  if (path_in_collision_) {
    if (planner_path_exists) {
      // Start navigating with path planner when there is a collision
      return usm::Transition::NEXT1;  // NAVIGATE
    } else {
      return usm::Transition::NEXT2;  // Loiter
    }
  } else {
    return usm::Transition::REPEAT;
  }
}

usm::Transition WaypointGenerator::runNavigate() {
  Eigen::Vector3f setpoint = position_;
  const bool tree_available = getSetpointFromPath(planner_info_.path_node_positions, planner_info_.last_path_time,
                                                  planner_info_.cruise_velocity, setpoint);
  output_.goto_position = position_ + (setpoint - position_).normalized();
  getPathMsg();
  planner_path_exists = bool(planner_info_.path_node_positions.size() > 2);
  loiter_position_ = position_;
  if (!planner_path_exists) {
    if (path_in_collision_)
      return usm::Transition::NEXT2;  // Loiter
    else
      return usm::Transition::NEXT1;  // Direct
  } else
    return usm::Transition::REPEAT;
}

void WaypointGenerator::calculateWaypoint() {
  ROS_DEBUG("\033[1;32m[WG] Generate Waypoint, current position: [%f, %f, %f].\033[0m", position_.x(), position_.y(),
            position_.z());
  output_.linear_velocity_wp = Eigen::Vector3f(NAN, NAN, NAN);

  // Timing
  last_time_ = current_time_;
  current_time_ = ros::Time::now();

  iterateOnce();
  output_.waypoint_type = getState();
  if (getState() != prev_planner_state_) {
    std::string state_str = toString(getState());
    ROS_DEBUG("\033[1;36m [WGN] Update to %s state \n \033[0m", state_str.c_str());
  }
}

void WaypointGenerator::updateState(const Eigen::Vector3f& act_pose, const Eigen::Quaternionf& q,
                                    const Eigen::Vector3f& goal, const Eigen::Vector3f& prev_goal,
                                    const Eigen::Vector3f& vel, bool stay, bool is_airborne,
                                    const avoidance::NavigationState& nav_state, const bool is_land_waypoint,
                                    const bool is_takeoff_waypoint, const Eigen::Vector3f& desired_vel,
                                    bool path_in_collision) {
  position_ = act_pose;
  velocity_ = vel;
  goal_ = goal;
  prev_goal_ = prev_goal;
  curr_yaw_rad_ = avoidance::getYawFromQuaternion(q) * avoidance::DEG_TO_RAD;
  curr_pitch_deg_ = avoidance::getPitchFromQuaternion(q);
  nav_state_ = nav_state;
  is_land_waypoint_ = is_land_waypoint;
  is_takeoff_waypoint_ = is_takeoff_waypoint;
  desired_vel_ = desired_vel;
  loiter_ = stay;
  path_in_collision_ = path_in_collision;

  is_airborne_ = is_airborne;

  // Initialize the smoothing point to current location, if it is undefined or
  // the  vehicle is not flying autonomously yet
  if (!is_airborne_ || !smoothed_goto_location_.allFinite() || !smoothed_goto_location_velocity_.allFinite()) {
    smoothed_goto_location_ = position_;
    smoothed_goto_location_velocity_ = Eigen::Vector3f::Zero();
    setpoint_yaw_rad_ = curr_yaw_rad_;
    setpoint_yaw_velocity_ = 0.f;
    reach_altitude_offboard_ = false;
  }

  // If we're changing altitude by Firmware setpoints, keep reinitializing the smoothing
  if (auto_land_) {
    smoothed_goto_location_ = position_;
    smoothed_goto_location_velocity_ = Eigen::Vector3f::Zero();
  }
}

// void WaypointGenerator::transformPositionToVelocityWaypoint() {
//   output_.linear_velocity_wp = output_.position_wp - position_;
//   output_.angular_velocity_wp.x() = 0.0f;
//   output_.angular_velocity_wp.y() = 0.0f;
//   output_.angular_velocity_wp.z() = getAngularVelocity(setpoint_yaw_rad_, curr_yaw_rad_);
// }

void WaypointGenerator::smoothWaypoint(float dt) {
  // If the smoothing speed is set to zero, dont smooth, aka use adapted
  // waypoint directly
  if (smoothing_speed_xy_ < 0.01f || smoothing_speed_z_ < 0.01f) {
    output_.smoothed_goto_position = output_.adapted_goto_position;
    return;
  }

  // Smooth differently in xz than in z
  const Eigen::Array3f P_constant(smoothing_speed_xy_, smoothing_speed_xy_, smoothing_speed_z_);
  const Eigen::Array3f D_constant = 2 * P_constant.sqrt();

  const Eigen::Vector3f desired_location = output_.adapted_goto_position;
  // Prevent overshoot when drone is close to goal
  const Eigen::Vector3f desired_velocity =
      (desired_location - goal_).norm() < 0.1 ? Eigen::Vector3f::Zero() : velocity_;

  Eigen::Vector3f location_diff = desired_location - smoothed_goto_location_;
  if (!location_diff.allFinite() || location_diff.norm() > 10.0) {
    location_diff = Eigen::Vector3f::Zero();
  }

  Eigen::Vector3f velocity_diff = desired_velocity - smoothed_goto_location_velocity_;
  if (!velocity_diff.allFinite()|| velocity_diff.norm() > 100.0) {
    velocity_diff = Eigen::Vector3f::Zero();
  }

  const Eigen::Vector3f p = location_diff.array() * P_constant;
  const Eigen::Vector3f d = velocity_diff.array() * D_constant;
  smoothed_goto_location_velocity_ += (p + d) * dt;
  smoothed_goto_location_ += smoothed_goto_location_velocity_ * dt;
  output_.smoothed_goto_position = output_.adapted_goto_position;

  ROS_DEBUG("[WG] Smoothed GoTo location: %f, %f, %f, with dt=%f", output_.smoothed_goto_position.x(),
            output_.smoothed_goto_position.y(), output_.smoothed_goto_position.z(), dt);
}

void WaypointGenerator::adaptSpeed() {
  speed_ = planner_info_.cruise_velocity;

  // If the goal is so close, that the speed-adapted way point would overreach
  float goal_dist = (goal_ - position_).norm();
  if (goal_dist < 1.f) {
    output_.adapted_goto_position = goal_;

    // First time we reach this goal, remember the heading
    if (!std::isfinite(heading_at_goal_rad_)) {
      heading_at_goal_rad_ = curr_yaw_rad_;
    }
    setpoint_yaw_rad_ = heading_at_goal_rad_;
  } else {
    // Scale the pose_to_wp by the speed
    Eigen::Vector3f pose_to_wp = output_.goto_position - position_;
    if (pose_to_wp.norm() > 0.1f) pose_to_wp.normalize();
    pose_to_wp *= std::min(speed_, goal_dist);

    heading_at_goal_rad_ = NAN;
    output_.adapted_goto_position = position_ + pose_to_wp;
  }

  ROS_INFO("[WG] Speed adapted WP: [%f %f %f].", output_.adapted_goto_position.x(), output_.adapted_goto_position.y(),
           output_.adapted_goto_position.z());
}

// create the message that is sent to the UAV
void WaypointGenerator::getPathMsg() {
  output_.adapted_goto_position = output_.goto_position;

  float time_diff_sec = static_cast<float>((current_time_ - last_time_).toSec());
  float dt = time_diff_sec > 0.0f ? time_diff_sec : 0.0001f;

  if (!auto_land_) {
    // in auto_land the z is only velocity controlled. Therefore we don't run the smoothing.
    adaptSpeed();
    smoothWaypoint(dt);
  }
  output_.position_wp = output_.smoothed_goto_position;
  output_.angular_velocity_wp = Eigen::Vector3f::Zero();
  avoidance::createPoseMsg(output_.position_wp, output_.orientation_wp, output_.adapted_goto_position,
                           setpoint_yaw_rad_);
}

waypointResult WaypointGenerator::getWaypoints() {
  calculateWaypoint();
  return output_;
}

void WaypointGenerator::setPlannerInfo(const avoidanceOutput& input) { planner_info_ = input; }

bool WaypointGenerator::getSetpointFromPath(const std::vector<Eigen::Vector3f>& path,
                                            const ros::Time& path_generation_time, float velocity,
                                            Eigen::Vector3f& setpoint) {
  int num_pathsegments = path.size();

  // path contains nothing meaningful
  if (num_pathsegments < 2) {
    return false;
  }

  // path only has one segment: return end of that segment as setpoint
  if (num_pathsegments == 2) {
    setpoint = path[0];
    return true;
  }

  // step through the path until the point where we should be if we had traveled perfectly with velocity along it
  float distance_left = (ros::Time::now() - path_generation_time).toSec() * velocity;

  for (int i = 1; i < num_pathsegments; i++) {
    Eigen::Vector3f path_segment = path[i] - path[i - 1];

    if (distance_left > path_segment.norm()) {
      distance_left -= path_segment.norm();
    } else {
      setpoint = (distance_left / path_segment.norm()) * path_segment + path[i - 1];
      return true;
    }
  }
  return false;
}
}