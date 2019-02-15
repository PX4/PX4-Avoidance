#include "local_planner/waypoint_generator.h"

#include "local_planner/common.h"
#include "local_planner/planner_functions.h"

#include <tf/transform_listener.h>

namespace avoidance {

ros::Time WaypointGenerator::getSystemTime() { return ros::Time::now(); }

void WaypointGenerator::calculateWaypoint() {
  ROS_DEBUG(
      "\033[1;32m[WG] Generate Waypoint, current position: [%f, %f, "
      "%f].\033[0m",
      pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z);
  output_.waypoint_type = planner_info_.waypoint_type;

  // Timing
  last_time_ = current_time_;
  current_time_ = getSystemTime();

  switch (planner_info_.waypoint_type) {
    case hover: {
      if (last_wp_type_ != hover) {
        hover_position_ = toEigen(pose_.pose.position);
      }
      output_.goto_position = toPoint(hover_position_);
      ROS_DEBUG("[WG] Hover at: [%f, %f, %f].", output_.goto_position.x,
                output_.goto_position.y, output_.goto_position.z);
      getPathMsg();
      break;
    }
    case costmap: {
      PolarPoint p_pol(planner_info_.costmap_direction_e,
                       planner_info_.costmap_direction_z, 1.0);
      output_.goto_position =
          toPoint(polarToCartesian(p_pol, pose_.pose.position));
      ROS_DEBUG("[WG] Costmap to: [%f, %f, %f].", output_.goto_position.x,
                output_.goto_position.y, output_.goto_position.z);
      getPathMsg();
      break;
    }

    case tryPath: {
      PolarPoint p_pol(0.0f, 0.0f, 0.0f);
      bool tree_available =
          getDirectionFromTree(p_pol, planner_info_.path_node_positions,
                               toEigen(pose_.pose.position));

      float dist_goal = (goal_ - toEigen(pose_.pose.position)).norm();
      ros::Duration since_last_path =
          getSystemTime() - planner_info_.last_path_time;
      if (tree_available &&
          (planner_info_.obstacle_ahead || dist_goal > 4.0f) &&
          since_last_path < ros::Duration(5)) {
        ROS_DEBUG("[WG] Use calculated tree\n");
        p_pol.r = 1.0;
        output_.goto_position =
            toPoint(polarToCartesian(p_pol, pose_.pose.position));
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
  last_position_waypoint_ = output_.position_waypoint;
  last_yaw_ = curr_yaw_;
  last_velocity_ =
      Eigen::Vector2f(curr_vel_.twist.linear.x, curr_vel_.twist.linear.y);
}

void WaypointGenerator::setFOV(float h_FOV, float v_FOV) {
  h_FOV_ = h_FOV;
  v_FOV_ = v_FOV;
}

void WaypointGenerator::updateState(const geometry_msgs::PoseStamped& act_pose,
                                    const geometry_msgs::PoseStamped& goal,
                                    const geometry_msgs::TwistStamped& vel,
                                    bool stay, ros::Time t) {
  if ((goal_ - toEigen(goal.pose.position)).norm() > 0.1f) {
    reached_goal_ = false;
    limit_speed_close_to_goal_ = false;
  }
  update_time_ = t;
  pose_ = act_pose;
  curr_vel_ = vel;
  goal_ = toEigen(goal.pose.position);
  curr_yaw_ = static_cast<float>(tf::getYaw(pose_.pose.orientation));

  tf::Quaternion q(pose_.pose.orientation.x, pose_.pose.orientation.y,
                   pose_.pose.orientation.z, pose_.pose.orientation.w);
  tf::Matrix3x3 m(q);

  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  z_FOV_idx_.clear();
  calculateFOV(h_FOV_, v_FOV_, z_FOV_idx_, e_FOV_min_, e_FOV_max_,
               static_cast<float>(yaw), static_cast<float>(pitch));

  curr_vel_magnitude_ =
      Eigen::Vector3f(curr_vel_.twist.linear.x, curr_vel_.twist.linear.y,
                      curr_vel_.twist.linear.z)
          .norm();

  if (stay) {
    planner_info_.waypoint_type = hover;
  }
}

// if there isn't any obstacle in front of the UAV, increase cruising speed
void WaypointGenerator::goStraight() {
  Eigen::Vector3f dir = (goal_ - toEigen(pose_.pose.position)).normalized();
  output_.goto_position = toPoint(toEigen(pose_.pose.position) + dir);

  ROS_DEBUG("[WG] Going straight to selected waypoint: [%f, %f, %f].",
            output_.goto_position.x, output_.goto_position.y,
            output_.goto_position.z);
}

void WaypointGenerator::backOff() {
  Eigen::Vector3f dir =
      (toEigen(pose_.pose.position) - toEigen(planner_info_.back_off_point));
  dir.z() = 0;
  dir.normalize();
  dir *= 0.5f;

  output_.goto_position = toPoint(toEigen(pose_.pose.position) + dir);
  output_.goto_position.z = planner_info_.back_off_start_point.z;

  output_.position_waypoint = createPoseMsg(output_.goto_position, last_yaw_);
  transformPositionToVelocityWaypoint();
  last_yaw_ = curr_yaw_;

  ROS_DEBUG("[WG] Backoff Point: [%f, %f, %f].", planner_info_.back_off_point.x,
            planner_info_.back_off_point.y, planner_info_.back_off_point.z);
  ROS_DEBUG("[WG] Back off selected direction: [%f, %f, %f].", dir.x(), dir.y(),
            dir.z());
}

void WaypointGenerator::transformPositionToVelocityWaypoint() {
  output_.velocity_waypoint.linear.x =
      output_.position_waypoint.pose.position.x - pose_.pose.position.x;
  output_.velocity_waypoint.linear.y =
      output_.position_waypoint.pose.position.y - pose_.pose.position.y;
  output_.velocity_waypoint.linear.z =
      output_.position_waypoint.pose.position.z - pose_.pose.position.z;
  output_.velocity_waypoint.angular.x = 0.0;
  output_.velocity_waypoint.angular.y = 0.0;
  output_.velocity_waypoint.angular.z = getAngularVelocity(new_yaw_, curr_yaw_);
}

// check if the UAV has reached the goal set for the mission
bool WaypointGenerator::withinGoalRadius() {
  float dist = (goal_ - toEigen(pose_.pose.position)).norm();

  if (dist < param_.goal_acceptance_radius_in) {
    if (!reached_goal_) {
      yaw_reached_goal_ = tf::getYaw(pose_.pose.orientation);
    }
    reached_goal_ = true;
  } else if (dist > param_.goal_acceptance_radius_out) {
    reached_goal_ = false;
  }
  return reached_goal_;
}

// when taking off, first publish waypoints to reach the goal altitude
void WaypointGenerator::reachGoalAltitudeFirst() {
  // Constant which defines the distance how far above the drone we set the
  // setpoint
  const float RISE_CARROT_STICK_LENGTH = 0.5;

  output_.goto_position = pose_.pose.position;
  if (pose_.pose.position.z <= goal_(2) - RISE_CARROT_STICK_LENGTH) {
    output_.goto_position.z += RISE_CARROT_STICK_LENGTH;
  } else {
    output_.goto_position.z = goal_(2);
  }

  // Will be overwritten by smoothing
  output_.adapted_goto_position = output_.goto_position;
  output_.smoothed_goto_position = output_.goto_position;
}

void WaypointGenerator::smoothWaypoint(float dt) {
  // In first iteration (very small or invalid dt), initialize the smoothing
  // to start at the current drone location
  if( dt <= 0.01 ){
    smoothed_goto_location_ = toEigen(pose_.pose.position);
    output_.smoothed_goto_position = pose_.pose.position;//needs to be local position!
    return;
  }

  // If the smoothing speed is set to zero, dont smooth, aka use adapted waypoint
  // directly
  if (smoothing_speed_xy_ < 0.01 || smoothing_speed_xy_ < 0.01) {
    output_.smoothed_goto_position = output_.adapted_goto_position;
    return;
  }

  // Smooth differently in xz than in z
  const float P_constant_xy = smoothing_speed_xy_;
  const float P_constant_z = smoothing_speed_z_;
  const float D_constant_xy = 2.f * std::sqrt(P_constant_xy);  // critically damped
  const float D_constant_z = 2.f * std::sqrt(P_constant_z);
  const Eigen::Vector3f P_constant(P_constant_xy, P_constant_xy, P_constant_z);
  const Eigen::Vector3f D_constant(D_constant_xy, D_constant_xy, D_constant_z);

  const Eigen::Vector3f desired_location =
      toEigen(output_.adapted_goto_position);
  const Eigen::Vector3f desired_velocity = toEigen(curr_vel_.twist.linear);

  Eigen::Vector3f location_diff = desired_location - smoothed_goto_location_;
  if (!location_diff.allFinite()) {
    location_diff = Eigen::Vector3f::Zero();
  }

  Eigen::Vector3f velocity_diff =
      desired_velocity - smoothed_goto_location_velocity_;
  if (!velocity_diff.allFinite()) {
    velocity_diff = Eigen::Vector3f::Zero();
  }

  const Eigen::Vector3f p = location_diff.array() * P_constant.array();
  const Eigen::Vector3f d = velocity_diff.array() * D_constant.array();

  smoothed_goto_location_velocity_ += (p + d) * dt;
  smoothed_goto_location_ += smoothed_goto_location_velocity_ * dt;

  output_.smoothed_goto_position = toPoint(smoothed_goto_location_);
  ROS_DEBUG("[WG] Smoothed GoTo location: %f, %f, %f, with dt=%f",
  output_.smoothed_goto_position.x, output_.smoothed_goto_position.y,
  output_.smoothed_goto_position.z, dt);
}

void WaypointGenerator::nextSmoothYaw(float dt) {
  // Use xy smoothing constant for yaw, since this makes more sense than z,
  // and we dont want to introduce yet another parameter
  const float P_constant_xy = smoothing_speed_xy_;
  const float D_constant_xy = 2.f * std::sqrt(P_constant_xy);  // critically damped

  const float desired_new_yaw = nextYaw(pose_, output_.goto_position);
  const float desired_yaw_velocity = 0.0;

  float yaw_diff =
      std::isfinite(desired_new_yaw) ? desired_new_yaw - new_yaw_ : 0.0f;

  wrapAngleToPlusMinusPI(yaw_diff);
  const float p = yaw_diff * P_constant_xy;
  const float d = (desired_yaw_velocity - new_yaw_velocity_) * D_constant_xy;

  new_yaw_velocity_ += (p + d) * dt;
  new_yaw_ += new_yaw_velocity_ * dt;
  wrapAngleToPlusMinusPI(new_yaw_);
}

void WaypointGenerator::adaptSpeed() {
  ros::Duration since_last_velocity = getSystemTime() - velocity_time_;
  float since_last_velocity_sec =
      static_cast<float>(since_last_velocity.toSec());

  if (!planner_info_.obstacle_ahead) {
    speed_ = std::min(speed_, planner_info_.velocity_far_from_obstacles);
    speed_ = velocityLinear(planner_info_.velocity_far_from_obstacles,
                            planner_info_.velocity_sigmoid_slope, speed_,
                            since_last_velocity_sec);
  } else {
    speed_ = std::min(speed_, planner_info_.velocity_around_obstacles);
    speed_ = velocityLinear(planner_info_.velocity_around_obstacles,
                            planner_info_.velocity_sigmoid_slope, speed_,
                            since_last_velocity_sec);
  }

  // check if new point lies in FOV
  PolarPoint p_pol = cartesianToPolar(toEigen(output_.adapted_goto_position),
                                      toEigen(pose_.pose.position));
  Eigen::Vector2i p_index = polarToHistogramIndex(p_pol, ALPHA_RES);

  if (std::find(z_FOV_idx_.begin(), z_FOV_idx_.end(), p_index.x()) !=
      z_FOV_idx_.end()) {
    waypoint_outside_FOV_ = false;
  } else {
    waypoint_outside_FOV_ = true;
    if (planner_info_.reach_altitude && !reached_goal_) {
      int ind_dist = 100;
      int i = 0;
      for (std::vector<int>::iterator it = z_FOV_idx_.begin();
           it != z_FOV_idx_.end(); ++it) {
        if (std::abs(z_FOV_idx_[i] - p_index.x()) < ind_dist) {
          ind_dist = std::abs(z_FOV_idx_[i] - p_index.x());
        }
        i++;
      }
      float angle_diff = static_cast<float>(std::abs(ALPHA_RES * ind_dist));
      float hover_angle = 30.0f;
      angle_diff = std::min(angle_diff, hover_angle);
      speed_ = speed_ * (1.0f - angle_diff / hover_angle);
    }
  }
  velocity_time_ = getSystemTime();

  // calculate correction for computation delay
  float since_update_sec =
      static_cast<float>((getSystemTime() - update_time_).toSec());
  speed_ += (since_update_sec * curr_vel_magnitude_);

  // break before goal: if the vehicle is closer to the goal than a velocity
  // dependent distance, the speed is limited
  Eigen::Vector3f pos_to_goal =
      (goal_ - toEigen(pose_.pose.position)).cwiseAbs();

  if (pos_to_goal.x() < param_.factor_close_to_goal_start_speed_limitation *
                            curr_vel_magnitude_ &&
      pos_to_goal.y() < param_.factor_close_to_goal_start_speed_limitation *
                            curr_vel_magnitude_) {
    limit_speed_close_to_goal_ = true;
  } else if (pos_to_goal.x() >
                 param_.factor_close_to_goal_stop_speed_limitation *
                     curr_vel_magnitude_ ||
             pos_to_goal.y() >
                 param_.factor_close_to_goal_stop_speed_limitation *
                     curr_vel_magnitude_) {
    limit_speed_close_to_goal_ = false;
  }
  if (limit_speed_close_to_goal_) {
    speed_ = std::min(
        speed_, param_.max_speed_close_to_goal_factor * pos_to_goal.norm());
    speed_ = std::max(speed_, param_.min_speed_close_to_goal);
  }

  // set waypoint to correct speed
  Eigen::Vector3f pose_to_wp =
      toEigen(output_.adapted_goto_position) - toEigen(pose_.pose.position);
  if (pose_to_wp.norm() > 0.01f) pose_to_wp.normalize();

  pose_to_wp *= speed_;

  output_.adapted_goto_position =
      toPoint(toEigen(pose_.pose.position) + pose_to_wp);

  ROS_DEBUG("[WG] Speed adapted WP: [%f %f %f].",
            output_.adapted_goto_position.x, output_.adapted_goto_position.y,
            output_.adapted_goto_position.z);
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

  // change waypoint if drone is at goal or above
  if (withinGoalRadius()) {
    output_.smoothed_goto_position = toPoint(goal_);
  }

  if (reached_goal_ && std::isfinite(yaw_reached_goal_)) {
    new_yaw_ = yaw_reached_goal_;
  }

  ROS_DEBUG("[WG] Final waypoint: [%f %f %f].",
            output_.smoothed_goto_position.x, output_.smoothed_goto_position.y,
            output_.smoothed_goto_position.z);
  output_.position_waypoint =
      createPoseMsg(output_.smoothed_goto_position, new_yaw_);
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
