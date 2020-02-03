#include "local_planner/waypoint_generator.h"

#include "avoidance/common.h"
#include "local_planner/planner_functions.h"

#include <ros/param.h>

#define normXY() topRows<2>().norm()

namespace avoidance {

WaypointGenerator::WaypointGenerator() : usm::StateMachine<PlannerState>(PlannerState::LOITER) {}
ros::Time WaypointGenerator::getSystemTime() { return ros::Time::now(); }

using avoidance::PlannerState;
std::string toString(PlannerState state) {
  std::string state_str = "unknown";
  switch (state) {
    case PlannerState::TRY_PATH:
      state_str = "TRY_PATH";
      break;
    case PlannerState::ALTITUDE_CHANGE:
      state_str = "ALTITUDE CHANGE";
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
  prev_slp_state_ = currentState;
  state_changed_ = true;

  // clang-format off
  USM_TABLE(
      currentState, PlannerState::LOITER,
      USM_STATE(transition, PlannerState::TRY_PATH, USM_MAP(usm::Transition::NEXT1, PlannerState::ALTITUDE_CHANGE);
         USM_MAP(usm::Transition::NEXT2, PlannerState::DIRECT);
         USM_MAP(usm::Transition::NEXT3, PlannerState::LOITER));
      USM_STATE(transition, PlannerState::LOITER, USM_MAP(usm::Transition::NEXT1, PlannerState::TRY_PATH));
      USM_STATE(transition, PlannerState::DIRECT, USM_MAP(usm::Transition::NEXT1, PlannerState::TRY_PATH);
         USM_MAP(usm::Transition::NEXT2, PlannerState::ALTITUDE_CHANGE);
         USM_MAP(usm::Transition::NEXT3, PlannerState::LOITER));
      USM_STATE(transition, PlannerState::ALTITUDE_CHANGE, USM_MAP(usm::Transition::NEXT1, PlannerState::TRY_PATH);
         USM_MAP(usm::Transition::NEXT2, PlannerState::LOITER)));
  // clang-format on
}

usm::Transition WaypointGenerator::runCurrentState() {
  if (trigger_reset_) {
    trigger_reset_ = false;
    return usm::Transition::ERROR;
  }

  usm::Transition t;
  switch (getState()) {
    case PlannerState::TRY_PATH:
      t = runTryPath();
      break;

    case PlannerState::ALTITUDE_CHANGE:
      t = runAltitudeChange();
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

usm::Transition WaypointGenerator::runTryPath() {
  Eigen::Vector3f setpoint = position_;
  const bool tree_available = getSetpointFromPath(planner_info_.path_node_positions, planner_info_.last_path_time,
                                                  planner_info_.cruise_velocity, getSystemTime(), setpoint);

  Eigen::Vector3f goto_position = position_ + (setpoint - position_).normalized();
  if (goto_position.hasNaN()) {
    output_.goto_position = position_;
  } else {
    output_.goto_position = goto_position;
  }
  getPathMsg();
  if (loiter_) {
    return usm::Transition::NEXT3;  // LOITER
  } else if (isAltitudeChange()) {
    return usm::Transition::NEXT1;  // ALTITUDE_CHANGE
  } else if (tree_available) {
    ROS_DEBUG("[WG] Using calculated tree\n");
    return usm::Transition::REPEAT;
  } else {
    return usm::Transition::NEXT2;  // DIRECT
  }
}

usm::Transition WaypointGenerator::runAltitudeChange() {
  if (state_changed_) {
    yaw_reach_height_rad_ = curr_yaw_rad_;
    change_altitude_pos_ = position_;
  }
  if (nav_state_ == NavigationState::offboard) {
    // goto_position is a unit vector pointing straight up/down from current
    // location
    output_.goto_position = position_;
    goal_.x() = position_.x();  // Needed so adaptSpeed can clamp to goal
    goal_.y() = position_.y();

    // Only move the setpoint if drone is in the air
    if (is_airborne_) {
      // Ascend/Descend to goal altitude
      if (position_.z() <= goal_.z()) {
        output_.goto_position.z() += 1.0f;
      } else {
        output_.goto_position.z() -= 1.0f;
      }
    }
  } else {
    if (velocity_.normXY() > 0.5f) {
      // First decelerate
      output_.linear_velocity_wp.topRows<2>() = Eigen::Vector2f::Zero();
      output_.linear_velocity_wp.z() = NAN;
      output_.position_wp.topRows<2>() = Eigen::Vector2f(NAN, NAN);
      output_.position_wp.z() = position_.z();
      output_.goto_position = position_;
      output_.adapted_goto_position = position_;
      output_.smoothed_goto_position = position_;
      change_altitude_pos_ = position_;
    } else {
      // Change altitude
      output_.goto_position.topRows<2>() = change_altitude_pos_.topRows<2>();
      output_.goto_position.z() = goal_.z();

      if (auto_land_) {
        output_.goto_position.topRows<2>() = change_altitude_pos_.topRows<2>();
        output_.adapted_goto_position = change_altitude_pos_;
        output_.smoothed_goto_position = change_altitude_pos_;
        output_.adapted_goto_position.z() = goal_.z();
        output_.smoothed_goto_position.z() = goal_.z();
      }
      output_.linear_velocity_wp = desired_vel_;
    }
  }
  getPathMsg();

  if (loiter_) {
    return usm::Transition::NEXT2;  // LOITER
  } else if (isAltitudeChange()) {
    return usm::Transition::REPEAT;  // ALTITUDE_CHANGE
  } else {
    return usm::Transition::NEXT1;  // TRY_PATH
  }
}
usm::Transition WaypointGenerator::runLoiter() {
  if (state_changed_ || hover_position_.array().hasNaN()) {
    hover_position_ = position_;
  }
  output_.goto_position = hover_position_;
  ROS_DEBUG("[WG] Hover at: [%f, %f, %f].", output_.goto_position.x(), output_.goto_position.y(),
            output_.goto_position.z());
  getPathMsg();

  if (loiter_) {
    return usm::Transition::REPEAT;
  } else {
    return usm::Transition::NEXT1;  // TRY_PATH
  }
}

usm::Transition WaypointGenerator::runDirect() {
  Eigen::Vector3f dir = (goal_ - position_).normalized();
  output_.goto_position = position_ + dir;

  ROS_DEBUG("[WG] Going straight to selected waypoint: [%f, %f, %f].", output_.goto_position.x(),
            output_.goto_position.y(), output_.goto_position.z());

  getPathMsg();

  Eigen::Vector3f setpoint;
  if (getSetpointFromPath(planner_info_.path_node_positions, planner_info_.last_path_time,
                          planner_info_.cruise_velocity, getSystemTime(), setpoint)) {
    return usm::Transition::NEXT1;  // TRY_PATH
  } else if (isAltitudeChange()) {
    return usm::Transition::NEXT2;  // ALTITUDE_CHANGE
  } else if (loiter_) {
    return usm::Transition::NEXT3;  // LOITER
  } else {
    return usm::Transition::REPEAT;
  }
}

void WaypointGenerator::calculateWaypoint() {
  ROS_DEBUG("\033[1;32m[WG] Generate Waypoint, current position: [%f, %f, %f].\033[0m", position_.x(), position_.y(),
            position_.z());
  output_.linear_velocity_wp = Eigen::Vector3f(NAN, NAN, NAN);

  // Timing
  last_time_ = current_time_;
  current_time_ = getSystemTime();

  iterateOnce();
  output_.waypoint_type = getState();
  if (getState() != prev_slp_state_) {
    std::string state_str = toString(getState());
    ROS_DEBUG("\033[1;36m [WGN] Update to %s state \n \033[0m", state_str.c_str());
  }
}

void WaypointGenerator::setFOV(int i, const FOV& fov) {
  std::lock_guard<std::mutex> lock(running_mutex_);
  if (i >= fov_fcu_frame_.size()) {
    fov_fcu_frame_.resize(i + 1);
  }
  fov_fcu_frame_[i] = fov;
}

void WaypointGenerator::updateState(const Eigen::Vector3f& act_pose, const Eigen::Quaternionf& q,
                                    const Eigen::Vector3f& goal, const Eigen::Vector3f& prev_goal,
                                    const Eigen::Vector3f& vel, bool stay, bool is_airborne,
                                    const NavigationState& nav_state, const bool is_land_waypoint,
                                    const bool is_takeoff_waypoint, const Eigen::Vector3f& desired_vel) {
  std::lock_guard<std::mutex> lock(running_mutex_);
  position_ = act_pose;
  velocity_ = vel;
  goal_ = goal;
  prev_goal_ = prev_goal;
  curr_yaw_rad_ = getYawFromQuaternion(q) * DEG_TO_RAD;
  curr_pitch_deg_ = getPitchFromQuaternion(q);
  nav_state_ = nav_state;
  is_land_waypoint_ = is_land_waypoint;
  is_takeoff_waypoint_ = is_takeoff_waypoint;
  desired_vel_ = desired_vel;
  loiter_ = stay;

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

void WaypointGenerator::transformPositionToVelocityWaypoint() {
  output_.linear_velocity_wp = output_.position_wp - position_;
  output_.angular_velocity_wp.x() = 0.0f;
  output_.angular_velocity_wp.y() = 0.0f;
  output_.angular_velocity_wp.z() = getAngularVelocity(setpoint_yaw_rad_, curr_yaw_rad_);
}

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
  if (!location_diff.allFinite()) {
    location_diff = Eigen::Vector3f::Zero();
  }

  Eigen::Vector3f velocity_diff = desired_velocity - smoothed_goto_location_velocity_;
  if (!velocity_diff.allFinite()) {
    velocity_diff = Eigen::Vector3f::Zero();
  }

  const Eigen::Vector3f p = location_diff.array() * P_constant;
  const Eigen::Vector3f d = velocity_diff.array() * D_constant;

  smoothed_goto_location_velocity_ += (p + d) * dt;
  smoothed_goto_location_ += smoothed_goto_location_velocity_ * dt;
  output_.smoothed_goto_position = smoothed_goto_location_;

  ROS_DEBUG("[WG] Smoothed GoTo location: %f, %f, %f, with dt=%f", output_.smoothed_goto_position.x(),
            output_.smoothed_goto_position.y(), output_.smoothed_goto_position.z(), dt);
}

void WaypointGenerator::nextSmoothYaw(float dt) {
  // Use xy smoothing constant for yaw, since this makes more sense than z,
  // and we dont want to introduce yet another parameter

  float desired_setpoint_yaw_rad =
      (position_ - output_.goto_position).normXY() > 0.1f ? nextYaw(position_, output_.goto_position) : curr_yaw_rad_;

  if (getState() == PlannerState::ALTITUDE_CHANGE) {
    desired_setpoint_yaw_rad = yaw_reach_height_rad_;
  }

  // If smoothing is disabled, set yaw to face goal directly
  if (smoothing_speed_xy_ <= 0.01f) {
    setpoint_yaw_rad_ = desired_setpoint_yaw_rad;
    return;
  }

  const float P_constant_xy = smoothing_speed_xy_;
  const float D_constant_xy = 2.f * std::sqrt(P_constant_xy);  // critically damped

  const float desired_yaw_velocity = 0.0f;

  float yaw_diff = wrapAngleToPlusMinusPI(
      std::isfinite(desired_setpoint_yaw_rad) ? desired_setpoint_yaw_rad - setpoint_yaw_rad_ : 0.0f);

  const float p = yaw_diff * P_constant_xy;
  const float d = (desired_yaw_velocity - setpoint_yaw_velocity_) * D_constant_xy;

  setpoint_yaw_velocity_ += (p + d) * dt;
  setpoint_yaw_rad_ += setpoint_yaw_velocity_ * dt;
  setpoint_yaw_rad_ = wrapAngleToPlusMinusPI(setpoint_yaw_rad_);
}

void WaypointGenerator::adaptSpeed(float dt) {
  // lowpass filter the speed to get smoother accelerations
  const float filter_time_constant = 0.9f;
  const float alpha = dt / (filter_time_constant + dt);
  const float last_speed = speed_;

  // at startup parameters are NAN (avoid propagating)
  if (std::isfinite(planner_info_.cruise_velocity)) {
    speed_ = planner_info_.cruise_velocity;
  } else {
    speed_ = 2.f;
  }

  // If the goal is so close, that the speed-adapted way point would overreach
  float goal_dist = (goal_ - position_).norm();
  if (goal_dist < 1.f) {
    output_.goto_position = goal_;
    speed_ = goal_dist;
    // First time we reach this goal, remember the heading
    if (!std::isfinite(heading_at_goal_rad_)) {
      heading_at_goal_rad_ = curr_yaw_rad_;
    }
    setpoint_yaw_rad_ = heading_at_goal_rad_;
  } else {
    // Scale the speed by a factor that is 0 if the waypoint is outside the FOV
    if (getState() != PlannerState::ALTITUDE_CHANGE) {
      PolarPoint p_pol_fcu = cartesianToPolarFCU(output_.goto_position, position_);
      p_pol_fcu.e -= curr_pitch_deg_;
      p_pol_fcu.z -= RAD_TO_DEG * curr_yaw_rad_;
      wrapPolar(p_pol_fcu);
      speed_ *= scaleToFOV(fov_fcu_frame_, p_pol_fcu);
    }
    heading_at_goal_rad_ = NAN;
  }

  speed_ = alpha * speed_ + (1.f - alpha) * last_speed;
  speed_ = std::min(speed_, goal_dist);

  // Scale the pose_to_wp by the speed
  Eigen::Vector3f pose_to_wp = output_.goto_position - position_;
  if (pose_to_wp.norm() > 0.1f) pose_to_wp.normalize();
  pose_to_wp *= speed_;

  output_.adapted_goto_position = position_ + pose_to_wp;

  ROS_INFO("[WG] Speed adapted WP: [%f %f %f].", output_.adapted_goto_position.x(), output_.adapted_goto_position.y(),
           output_.adapted_goto_position.z());
}

// create the message that is sent to the UAV
void WaypointGenerator::getPathMsg() {
  output_.adapted_goto_position = output_.goto_position;

  float time_diff_sec = static_cast<float>((current_time_ - last_time_).toSec());
  float dt = time_diff_sec > 0.0f ? time_diff_sec : 0.0001f;

  // set the yaw at the setpoint based on our smoothed location
  nextSmoothYaw(dt);

  if (!auto_land_) {
    // in auto_land the z is only velocity controlled. Therefore we don't run the smoothing.
    adaptSpeed(dt);
    smoothWaypoint(dt);
  }

  ROS_INFO("[WG] Final waypoint: [%f %f %f]. %f %f %f \n", output_.smoothed_goto_position.x(),
           output_.smoothed_goto_position.y(), output_.smoothed_goto_position.z(), output_.linear_velocity_wp.x(),
           output_.linear_velocity_wp.y(), output_.linear_velocity_wp.z());
  createPoseMsg(output_.position_wp, output_.orientation_wp, output_.smoothed_goto_position, setpoint_yaw_rad_);
}

waypointResult WaypointGenerator::getWaypoints() {
  std::lock_guard<std::mutex> lock(running_mutex_);
  calculateWaypoint();
  return output_;
}

bool WaypointGenerator::isAltitudeChange() {
  bool rtl_descend = false;
  bool rtl_climb = false;

  if (position_.z() > (goal_.z() - 0.8f)) {
    rtl_descend = false;
    rtl_climb = false;
  } else if (goal_.z() > position_.z()) {
    rtl_descend = false;
    rtl_climb = true;
  } else {
    rtl_descend = true;
    rtl_climb = false;
  }

  const bool offboard_goal_altitude_not_reached = nav_state_ == NavigationState::offboard && !reach_altitude_offboard_;
  const bool auto_takeoff = nav_state_ == NavigationState::auto_takeoff ||
                            (nav_state_ == NavigationState::mission && is_takeoff_waypoint_) ||
                            (nav_state_ == NavigationState::auto_rtl && rtl_climb);
  auto_land_ = nav_state_ == NavigationState::auto_land ||
               (nav_state_ == NavigationState::mission && is_land_waypoint_) ||
               (nav_state_ == NavigationState::auto_rtl && rtl_descend);
  const bool need_to_change_altitude = offboard_goal_altitude_not_reached || auto_takeoff || auto_land_;
  if (need_to_change_altitude) {
    if (nav_state_ == NavigationState::offboard) {
      if (position_.z() >= goal_.z()) {
        reach_altitude_offboard_ = true;
        return false;
      }
    }
    ROS_INFO("\033[1;35m[OA] Reach height first \033[0m");
    return true;
  }

  return false;
}

void WaypointGenerator::setPlannerInfo(const avoidanceOutput& input) {
  std::lock_guard<std::mutex> lock(running_mutex_);
  planner_info_ = input;
}

void WaypointGenerator::getOfftrackPointsForVisualization(Eigen::Vector3f& closest_pt, Eigen::Vector3f& deg60_pt) {
  std::lock_guard<std::mutex> lock(running_mutex_);
  closest_pt.x() = closest_pt_.x();
  closest_pt.y() = closest_pt_.y();
  closest_pt.z() = goal_.z();
  deg60_pt = tmp_goal_;
}
}
