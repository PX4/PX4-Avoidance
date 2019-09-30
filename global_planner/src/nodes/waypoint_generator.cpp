#include <ros/param.h>

#include "avoidance/common.h"
#include "global_planner/waypoint_generator.h"

namespace global_planner {

WaypointGenerator::WaypointGenerator() : usm::StateMachine<PlannerState>(PlannerState::LOITER) {}
// ros::Time WaypointGenerator::getSystemTime() { return ros::Time::now(); }

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
      currentState, PlannerState::LOITER,
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
  std::cout << toString(getState()) << std::endl;
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
  if (state_changed_ || hover_position_.array().hasNaN()) {
    hover_position_ = position_;
  }
  output_.goto_position = hover_position_;
  ROS_DEBUG("[WG] Hover at: [%f, %f, %f].", output_.goto_position.x(), output_.goto_position.y(),
            output_.goto_position.z());
  getPathMsg();

  // TODO: Implement collision checker, start the navigate state if a collision is found
  if (!collision_on_direct_path_) {
    return usm::Transition::NEXT1;  // DIRECT
  } else if (loiter_) {
    // Loiter if there would be a collision and the trajectory has not yet been calculated
    return usm::Transition::REPEAT;  // LOITER
  } else {
    // Start navigating with path planner when there is a collision and a path is available
    return usm::Transition::NEXT2;  // NAVIGATE
  }
}

usm::Transition WaypointGenerator::runDirect() {
  Eigen::Vector3f dir = (goal_ - position_).normalized();
  output_.goto_position = position_ + dir;
  std::cout << "[RunDirect] position: " << position_.transpose() << " goal: " << goal_.transpose()
            << " dir: " << dir.transpose() << std::endl;

  ROS_INFO("[WG] Going straight to selected waypoint: [%f, %f, %f].", output_.goto_position.x(),
           output_.goto_position.y(), output_.goto_position.z());

  getPathMsg();

  // TODO: Implement collision checker, start the navigate state if a collision is found
  if (!collision_on_direct_path_) {
    return usm::Transition::REPEAT;
  } else if (loiter_) {
    // Loiter if there would be a collision and the trajectory has not yet been calculated
    return usm::Transition::NEXT2;  // LOITER
  } else {
    // Start navigating with path planner when there is a collision and a path is available
    return usm::Transition::NEXT1;  // NAVIGATE
  }
}

usm::Transition WaypointGenerator::runNavigate() {
  Eigen::Vector3f setpoint = position_;
  bool valid_path = getSetpointFromPath(planner_info_.path_node_positions, planner_info_.last_path_time,
                                        planner_info_.cruise_velocity, setpoint);

  if ((setpoint - position_).norm() > 0.001f) {
    output_.goto_position = position_ + (setpoint - position_).normalized();
  } else {
    output_.goto_position = position_;  // hovering
  }

  ROS_INFO("\033[1;32m[WG] running navigate: path valid = %.0f \033[0m", (double)valid_path);

  getPathMsg();

  // TODO: Implement collision checker, start the navigate state if a collision is found
  if (!collision_on_direct_path_) {
    return usm::Transition::NEXT1;  // DIRECT
  } else if (loiter_ || !valid_path) {
    // Loiter if there would be a collision and the trajectory has not yet been calculated
    return usm::Transition::NEXT2;  // LOITER
  } else {
    // Continue navigating with path planner when there is a collision and a path is available
    return usm::Transition::REPEAT;  // NAVIGATE
  }
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
                                    const bool collision_on_direct_path) {
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
  collision_on_direct_path_ = collision_on_direct_path;

  is_airborne_ = is_airborne;

  ROS_INFO("\033[1;32m[WG] new data: loiter = %.0f, collision_on_direct_path = %.0f \033[0m", (double)stay,
           (double)collision_on_direct_path);

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
  if (!velocity_diff.allFinite() || velocity_diff.norm() > 100.0) {
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

bool WaypointGenerator::getSetpointFromPath(const std::vector<Eigen::Vector3f>& path,
                                            const ros::Time& path_generation_time, float velocity,
                                            Eigen::Vector3f& setpoint) {
  int i = path.size();
  ROS_INFO("\033[1;32m[WG] path size %.0f \033[0m", (double)i);

  // debug
  for (int j = 0; j < i; j++) {
    ROS_INFO("\033[1;32m[WG] node %.0f: [%f %f %f]  \033[0m", (double)j, (double)path[j].x(), (double)path[j].y(),
             (double)path[j].z());
  }

  // path contains nothing meaningful
  if (i < 2) {
    ROS_INFO("\033[1;32m[WG] path smaller than 2 nodes \033[0m");
    return false;
  }

  // path only has one segment: return end of that segment as setpoint
  if (i == 2) {
    setpoint = path[0];
    return true;
  }

  // step through the path until the point where we should be if we had traveled perfectly with velocity along it
  Eigen::Vector3f path_segment = path[i - 3] - path[i - 2];
  float distance_left = (ros::Time::now() - path_generation_time).toSec() * velocity;
  setpoint = path[i - 2] + (distance_left / path_segment.norm()) * path_segment;

  for (i = path.size() - 3; i > 0 && distance_left > path_segment.norm(); --i) {
    distance_left -= path_segment.norm();
    path_segment = path[i - 1] - path[i];
    setpoint = path[i] + (distance_left / path_segment.norm()) * path_segment;
  }

  // If we excited because we're past the last node of the path, the path is no longer valid!
  return distance_left < path_segment.norm();
}

// create the message that is sent to the UAV
void WaypointGenerator::getPathMsg() {
  output_.adapted_goto_position = output_.goto_position;

  ROS_INFO("[WG] original waypoint: [%f %f %f].\n", output_.goto_position.x(), output_.goto_position.y(),
           output_.goto_position.z());

  float time_diff_sec = static_cast<float>((current_time_ - last_time_).toSec());
  float dt = time_diff_sec > 0.0f ? time_diff_sec : 0.0001f;

  if (!auto_land_) {
    // in auto_land the z is only velocity controlled. Therefore we don't run the smoothing.
    adaptSpeed();
    smoothWaypoint(dt);
  }

  ROS_INFO("[WG] Final waypoint: [%f %f %f]. %f %f %f \n", output_.smoothed_goto_position.x(),
           output_.smoothed_goto_position.y(), output_.smoothed_goto_position.z(), output_.linear_velocity_wp.x(),
           output_.linear_velocity_wp.y(), output_.linear_velocity_wp.z());
  output_.position_wp = output_.smoothed_goto_position;
  output_.angular_velocity_wp = Eigen::Vector3f::Zero();
  avoidance::createPoseMsg(output_.position_wp, output_.orientation_wp, output_.smoothed_goto_position,
                           setpoint_yaw_rad_);
}

waypointResult WaypointGenerator::getWaypoints() {
  calculateWaypoint();
  return output_;
}

void WaypointGenerator::setPlannerInfo(const avoidanceOutput& input) { planner_info_ = input; }
}
