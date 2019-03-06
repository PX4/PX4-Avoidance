#include "local_planner/waypoint_generator.h"

#include "local_planner/common.h"
#include "local_planner/planner_functions.h"

#include <ros/param.h>

#include <tf/transform_listener.h>

namespace avoidance {

ros::Time WaypointGenerator::getSystemTime() { return ros::Time::now(); }

void WaypointGenerator::calculateWaypoint() {
  ROS_DEBUG(
      "\033[1;32m[WG] Generate Waypoint, current position: [%f, %f, "
      "%f].\033[0m",
      position_.x(), position_.y(), position_.z());
  output_.waypoint_type = planner_info_.waypoint_type;

  // Timing
  last_time_ = current_time_;
  current_time_ = getSystemTime();

  switch (planner_info_.waypoint_type) {
    case hover: {
      if (last_wp_type_ != hover) {
        hover_position_ = position_;
      }
      output_.goto_position = hover_position_;
      ROS_DEBUG("[WG] Hover at: [%f, %f, %f].", output_.goto_position.x(),
                output_.goto_position.y(), output_.goto_position.z());
      getPathMsg();
      break;
    }
    case costmap: {
      PolarPoint p_pol(planner_info_.costmap_direction_e,
                       planner_info_.costmap_direction_z, 1.0);
      output_.goto_position = polarToCartesian(p_pol, position_);
      ROS_DEBUG("[WG] Costmap to: [%f, %f, %f].", output_.goto_position.x(),
                output_.goto_position.y(), output_.goto_position.z());
      getPathMsg();
      break;
    }

    case tryPath: {
      PolarPoint p_pol(0.0f, 0.0f, 0.0f);
      bool tree_available = getDirectionFromTree(
          p_pol, planner_info_.path_node_positions, position_, goal_);

      float dist_goal = (goal_ - position_).norm();
      ros::Duration since_last_path =
          getSystemTime() - planner_info_.last_path_time;
      if (tree_available &&
          (planner_info_.obstacle_ahead || dist_goal > 4.0f) &&
          since_last_path < ros::Duration(5)) {
        ROS_DEBUG("[WG] Use calculated tree\n");
        p_pol.r = 1.0;
        output_.goto_position = polarToCartesian(p_pol, position_);
      } else {
        ROS_DEBUG("[WG] No valid tree, going straight");
        goStraight();
        output_.waypoint_type = direct;
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
      reachGoalAltitudeFirst();
      getPathMsg();
      break;
    }
    case goBack: {
      ROS_DEBUG("[WG] Too close, backing off");
      backOff();
      break;
    }
  }
  last_wp_type_ = planner_info_.waypoint_type;
  last_yaw_ = curr_yaw_;
}

void WaypointGenerator::setFOV(float h_FOV, float v_FOV) {
  h_FOV_ = h_FOV;
  v_FOV_ = v_FOV;
}

void WaypointGenerator::updateState(const Eigen::Vector3f& act_pose,
                                    const Eigen::Quaternionf& q,
                                    const Eigen::Vector3f& goal,
                                    const Eigen::Vector3f& vel, bool stay,
                                    bool is_airborne) {
  position_ = act_pose;
  velocity_ = vel;
  goal_ = goal;
  tf::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
  tf::Matrix3x3 tf_m(tf_q);
  double roll, pitch, yaw;
  tf_m.getRPY(roll, pitch, yaw);
  curr_yaw_ = static_cast<float>(yaw);

  if (stay) {
    planner_info_.waypoint_type = hover;
  }
  is_airborne_ = is_airborne;

  // Initialize the smoothing point to current location, if it is undefined or
  // the  vehicle is not flying autonomously yet
  if (!is_airborne_ || !smoothed_goto_location_.allFinite() ||
      !smoothed_goto_location_velocity_.allFinite()) {
    smoothed_goto_location_ = position_;
    smoothed_goto_location_velocity_ = Eigen::Vector3f::Zero();
  }
}

// if there isn't any obstacle in front of the UAV, increase cruising speed
void WaypointGenerator::goStraight() {
  Eigen::Vector3f dir = (goal_ - position_).normalized();
  output_.goto_position = position_ + dir;

  ROS_DEBUG("[WG] Going straight to selected waypoint: [%f, %f, %f].",
            output_.goto_position.x(), output_.goto_position.y(),
            output_.goto_position.z());
}

void WaypointGenerator::backOff() {
  Eigen::Vector3f dir = position_ - planner_info_.back_off_point;
  dir.z() = 0;
  dir.normalize();
  dir *= 0.5f;

  output_.goto_position = position_ + dir;
  output_.goto_position.z() = planner_info_.back_off_start_point.z();

  createPoseMsg(output_.position_wp, output_.orientation_wp,
                output_.goto_position, last_yaw_);
  transformPositionToVelocityWaypoint();

  ROS_DEBUG("[WG] Backoff Point: [%f, %f, %f].",
            planner_info_.back_off_point.x(), planner_info_.back_off_point.y(),
            planner_info_.back_off_point.z());
  ROS_DEBUG("[WG] Back off selected direction: [%f, %f, %f].", dir.x(), dir.y(),
            dir.z());
}

void WaypointGenerator::transformPositionToVelocityWaypoint() {
  output_.linear_velocity_wp = output_.position_wp - position_;
  output_.angular_velocity_wp.x() = 0.0f;
  output_.angular_velocity_wp.y() = 0.0f;
  output_.angular_velocity_wp.z() =
      getAngularVelocity(setpoint_yaw_, curr_yaw_);
}

// when taking off, first publish waypoints to reach the goal altitude
void WaypointGenerator::reachGoalAltitudeFirst() {
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
}

void WaypointGenerator::smoothWaypoint(float dt) {
  // If the smoothing speed is set to zero, dont smooth, aka use adapted
  // waypoint directly
  if (smoothing_speed_xy_ < 0.01f || smoothing_speed_z_ < 0.01f) {
    output_.smoothed_goto_position = output_.adapted_goto_position;
    return;
  }

  // Smooth differently in xz than in z
  const Eigen::Array3f P_constant(smoothing_speed_xy_, smoothing_speed_xy_,
                                  smoothing_speed_z_);
  const Eigen::Array3f D_constant = 2 * P_constant.sqrt();

  const Eigen::Vector3f desired_location = output_.adapted_goto_position;

  // Prevent overshoot when drone is close to goal
  const Eigen::Vector3f desired_velocity =
      (desired_location - goal_).norm() < 0.1 ? Eigen::Vector3f::Zero()
                                              : velocity_;

  Eigen::Vector3f location_diff = desired_location - smoothed_goto_location_;
  if (!location_diff.allFinite()) {
    location_diff = Eigen::Vector3f::Zero();
  }

  Eigen::Vector3f velocity_diff =
      desired_velocity - smoothed_goto_location_velocity_;
  if (!velocity_diff.allFinite()) {
    velocity_diff = Eigen::Vector3f::Zero();
  }

  const Eigen::Vector3f p = location_diff.array() * P_constant;
  const Eigen::Vector3f d = velocity_diff.array() * D_constant;

  smoothed_goto_location_velocity_ += (p + d) * dt;
  smoothed_goto_location_ += smoothed_goto_location_velocity_ * dt;
  output_.smoothed_goto_position = smoothed_goto_location_;

  ROS_DEBUG("[WG] Smoothed GoTo location: %f, %f, %f, with dt=%f",
            output_.smoothed_goto_position.x(),
            output_.smoothed_goto_position.y(),
            output_.smoothed_goto_position.z(), dt);
}

void WaypointGenerator::nextSmoothYaw(float dt) {
  // Use xy smoothing constant for yaw, since this makes more sense than z,
  // and we dont want to introduce yet another parameter
  const float P_constant_xy = smoothing_speed_xy_;
  const float D_constant_xy =
      2.f * std::sqrt(P_constant_xy);  // critically damped

  const float desired_setpoint_yaw = nextYaw(position_, output_.goto_position);
  const float desired_yaw_velocity = 0.0;

  float yaw_diff = std::isfinite(desired_setpoint_yaw)
                       ? desired_setpoint_yaw - setpoint_yaw_
                       : 0.0f;

  wrapAngleToPlusMinusPI(yaw_diff);
  const float p = yaw_diff * P_constant_xy;
  const float d =
      (desired_yaw_velocity - setpoint_yaw_velocity_) * D_constant_xy;

  setpoint_yaw_velocity_ += (p + d) * dt;
  setpoint_yaw_ += setpoint_yaw_velocity_ * dt;
  wrapAngleToPlusMinusPI(setpoint_yaw_);
}

void WaypointGenerator::adaptSpeed() {
  if (!planner_info_.obstacle_ahead) {
    speed_ = planner_info_.velocity_far_from_obstacles;
  } else {
    speed_ = planner_info_.velocity_around_obstacles;
  }

  // If the goal is so close, that the speed-adapted way point would overreach
  Eigen::Vector3f pose_to_goal = goal_ - position_;
  if (pose_to_goal.norm() < speed_) {
    output_.adapted_goto_position = goal_;

    // First time we reach this goal, remember the heading
    if (!std::isfinite(heading_at_goal_)) {
      heading_at_goal_ = curr_yaw_;
    }
    setpoint_yaw_ = heading_at_goal_;

  } else {
    // Scale the speed by a factor that is 0 if the waypoint is outside the FOV
    if (output_.waypoint_type != reachHeight) {
      float angle_diff_deg =
          std::abs(nextYaw(position_, output_.goto_position) - curr_yaw_) *
          180.f / M_PI_F;
      angle_diff_deg =
          std::min(angle_diff_deg, std::abs(360.f - angle_diff_deg));
      angle_diff_deg =
          std::min(h_FOV_ / 2, angle_diff_deg);  // Clamp at h_FOV/2
      speed_ *= (1.0f - 2 * angle_diff_deg / h_FOV_);
    }

    // Scale the pose_to_wp by the speed
    Eigen::Vector3f pose_to_wp = output_.goto_position - position_;
    if (pose_to_wp.norm() > 0.1f) pose_to_wp.normalize();
    pose_to_wp *= speed_;

    heading_at_goal_ = NAN;
    output_.adapted_goto_position = position_ + pose_to_wp;
  }

  ROS_INFO("[WG] Speed adapted WP: [%f %f %f].",
           output_.adapted_goto_position.x(), output_.adapted_goto_position.y(),
           output_.adapted_goto_position.z());
}

// create the message that is sent to the UAV
void WaypointGenerator::getPathMsg() {
  output_.adapted_goto_position = output_.goto_position;

  float time_diff_sec =
      static_cast<float>((current_time_ - last_time_).toSec());
  float dt = time_diff_sec > 0.0f ? time_diff_sec : 0.0001f;

  // set the yaw at the setpoint based on our smoothed location
  nextSmoothYaw(dt);

  // adapt waypoint to suitable speed (slow down if waypoint is out of FOV)
  adaptSpeed();
  smoothWaypoint(dt);

  ROS_DEBUG(
      "[WG] Final waypoint: [%f %f %f].", output_.smoothed_goto_position.x(),
      output_.smoothed_goto_position.y(), output_.smoothed_goto_position.z());
  createPoseMsg(output_.position_wp, output_.orientation_wp,
                output_.smoothed_goto_position, setpoint_yaw_);
  transformPositionToVelocityWaypoint();
}

waypointResult WaypointGenerator::getWaypoints() {
  calculateWaypoint();
  return output_;
}

void WaypointGenerator::setPlannerInfo(const avoidanceOutput& input) {
  planner_info_ = input;
}
}
