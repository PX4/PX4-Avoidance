#include "local_planner/waypoint_generator.h"

#include "avoidance/common.h"
#include "local_planner/planner_functions.h"

#include <ros/param.h>

#define normXY() topRows<2>().norm()

namespace avoidance {

ros::Time WaypointGenerator::getSystemTime() { return ros::Time::now(); }

void WaypointGenerator::calculateWaypoint() {
  ROS_DEBUG("\033[1;32m[WG] Generate Waypoint, current position: [%f, %f, %f].\033[0m", position_.x(), position_.y(),
            position_.z());
  output_.waypoint_type = planner_info_.waypoint_type;
  output_.linear_velocity_wp = Eigen::Vector3f(NAN, NAN, NAN);

  // Timing
  last_time_ = current_time_;
  current_time_ = getSystemTime();

  switch (planner_info_.waypoint_type) {
    case hover: {
      if (last_wp_type_ != hover) {
        hover_position_ = position_;
      }
      output_.goto_position = hover_position_;
      ROS_DEBUG("[WG] Hover at: [%f, %f, %f].", output_.goto_position.x(), output_.goto_position.y(),
                output_.goto_position.z());
      getPathMsg();
      break;
    }

    case tryPath: {
      Eigen::Vector3f setpoint = position_;
      if (getSetpointFromPath(planner_info_.path_node_positions, planner_info_.last_path_time,
                              planner_info_.cruise_velocity, setpoint)) {
        output_.goto_position = position_ + (setpoint - position_).normalized();
        ROS_DEBUG("[WG] Using calculated tree\n");
      } else {
        ROS_DEBUG("[WG] No valid tree, going straight");
        output_.waypoint_type = direct;

        // calculate the vehicle position on the line between the previous and
        // current goal
        Eigen::Vector2f u_prev_to_goal = (goal_ - prev_goal_).head<2>().normalized();
        Eigen::Vector2f prev_to_pos = (position_ - prev_goal_).head<2>();
        Eigen::Vector2f pos_2f = position_.head<2>();
        closest_pt_ = prev_goal_.head<2>() + (u_prev_to_goal * u_prev_to_goal.dot(prev_to_pos));

        // if the vehicle is more than the cruise velocity away from the line
        // previous to current goal, set temporary goal on the line  entering
        // at 60 degrees
        if ((pos_2f - closest_pt_).norm() > planner_info_.cruise_velocity) {
          float len = (pos_2f - closest_pt_).norm() * std::cos(DEG_TO_RAD * 60.0f) / std::sin(DEG_TO_RAD * 60.0f);
          tmp_goal_.x() = closest_pt_.x() + len * u_prev_to_goal.x();
          tmp_goal_.y() = closest_pt_.y() + len * u_prev_to_goal.y();
          tmp_goal_.z() = goal_.z();

          Eigen::Vector3f dir = (tmp_goal_ - position_).normalized();
          output_.goto_position = position_ + dir;
        } else {
          goStraight();
        }
      }
      getPathMsg();
      break;
    }

    case direct: {
      ROS_DEBUG("[WG] No obstacle ahead, going straight");
      goStraight();
      getPathMsg();
      break;
    }

    case reachHeight: {
      ROS_DEBUG("[WG] Reaching height first");
      if (last_wp_type_ != reachHeight) {
        yaw_reach_height_rad_ = curr_yaw_rad_;
        change_altitude_pos_ = position_;
      }
      reachGoalAltitudeFirst();
      getPathMsg();
      break;
    }
  }
  last_wp_type_ = planner_info_.waypoint_type;
}

void WaypointGenerator::setFOV(int i, const FOV& fov) {
  if (i < fov_fcu_frame_.size()) {
    fov_fcu_frame_[i] = fov;
  } else {
    fov_fcu_frame_.push_back(fov);
  }
}

void WaypointGenerator::updateState(const Eigen::Vector3f& act_pose, const Eigen::Quaternionf& q,
                                    const Eigen::Vector3f& goal, const Eigen::Vector3f& prev_goal,
                                    const Eigen::Vector3f& vel, bool stay, bool is_airborne,
                                    const NavigationState& nav_state, const bool is_land_waypoint,
                                    const bool is_takeoff_waypoint, const Eigen::Vector3f& desired_vel) {
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

  if (stay) {
    planner_info_.waypoint_type = hover;
  }
  is_airborne_ = is_airborne;

  // Initialize the smoothing point to current location, if it is undefined or
  // the  vehicle is not flying autonomously yet
  if (!is_airborne_ || !smoothed_goto_location_.allFinite() || !smoothed_goto_location_velocity_.allFinite()) {
    smoothed_goto_location_ = position_;
    smoothed_goto_location_velocity_ = Eigen::Vector3f::Zero();
    setpoint_yaw_rad_ = curr_yaw_rad_;
    setpoint_yaw_velocity_ = 0.f;
    reach_altitude_ = false;
  }

  // If we're changing altitude by Firmware setpoints, keep reinitializing the smoothing
  if (auto_land_) {
    smoothed_goto_location_ = position_;
    smoothed_goto_location_velocity_ = Eigen::Vector3f::Zero();
  }
}

// if there isn't any obstacle in front of the UAV, increase cruising speed
void WaypointGenerator::goStraight() {
  Eigen::Vector3f dir = (goal_ - position_).normalized();
  output_.goto_position = position_ + dir;

  ROS_DEBUG("[WG] Going straight to selected waypoint: [%f, %f, %f].", output_.goto_position.x(),
            output_.goto_position.y(), output_.goto_position.z());
}

void WaypointGenerator::transformPositionToVelocityWaypoint() {
  output_.linear_velocity_wp = output_.position_wp - position_;
  output_.angular_velocity_wp.x() = 0.0f;
  output_.angular_velocity_wp.y() = 0.0f;
  output_.angular_velocity_wp.z() = getAngularVelocity(setpoint_yaw_rad_, curr_yaw_rad_);
}

// when taking off, first publish waypoints to reach the goal altitude
void WaypointGenerator::reachGoalAltitudeFirst() {
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
      output_.goto_position = goal_;

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

  if (planner_info_.waypoint_type == reachHeight) {
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
    // Scale the speed by a factor that is 0 if the waypoint is outside the FOV
    if (output_.waypoint_type != reachHeight) {
      PolarPoint p_pol_fcu = cartesianToPolarFCU(output_.goto_position, position_);
      p_pol_fcu.e -= curr_pitch_deg_;
      p_pol_fcu.z -= RAD_TO_DEG * curr_yaw_rad_;
      wrapPolar(p_pol_fcu);
      speed_ *= scaleToFOV(fov_fcu_frame_, p_pol_fcu);
    }

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

  // set the yaw at the setpoint based on our smoothed location
  nextSmoothYaw(dt);

  if (!auto_land_) {
    // in auto_land the z is only velocity controlled. Therefore we don't run the smoothing.
    adaptSpeed();
    smoothWaypoint(dt);
  }

  ROS_INFO("[WG] Final waypoint: [%f %f %f]. %f %f %f \n", output_.smoothed_goto_position.x(),
           output_.smoothed_goto_position.y(), output_.smoothed_goto_position.z(), output_.linear_velocity_wp.x(),
           output_.linear_velocity_wp.y(), output_.linear_velocity_wp.z());

  createPoseMsg(output_.position_wp, output_.orientation_wp, output_.smoothed_goto_position, setpoint_yaw_rad_);
}

waypointResult WaypointGenerator::getWaypoints() {
  changeAltitude();
  calculateWaypoint();
  return output_;
}

void WaypointGenerator::changeAltitude() {
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

  const bool offboard_goal_altitude_not_reached = nav_state_ == NavigationState::offboard && !reach_altitude_;
  const bool auto_takeoff = nav_state_ == NavigationState::auto_takeoff ||
                            (nav_state_ == NavigationState::mission && is_takeoff_waypoint_) ||
                            (nav_state_ == NavigationState::auto_rtl && rtl_climb);
  auto_land_ = nav_state_ == NavigationState::auto_land ||
               (nav_state_ == NavigationState::mission && is_land_waypoint_) ||
               (nav_state_ == NavigationState::auto_rtl && rtl_descend);
  const bool need_to_change_altitude = offboard_goal_altitude_not_reached || auto_takeoff || auto_land_;
  if (need_to_change_altitude) {
    planner_info_.waypoint_type = reachHeight;

    if (nav_state_ == NavigationState::offboard) {
      if (position_.z() > goal_.z()) {
        reach_altitude_ = true;
        planner_info_.waypoint_type = direct;
      }
    }

    ROS_INFO("\033[1;35m[OA] Reach height first \033[0m");
  }
}

void WaypointGenerator::setPlannerInfo(const avoidanceOutput& input) { planner_info_ = input; }

void WaypointGenerator::getOfftrackPointsForVisualization(Eigen::Vector3f& closest_pt, Eigen::Vector3f& deg60_pt) {
  closest_pt.x() = closest_pt_.x();
  closest_pt.y() = closest_pt_.y();
  closest_pt.z() = goal_.z();
  deg60_pt = tmp_goal_;
}
}
