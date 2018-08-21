#include "waypoint_generator.h"

WaypointGenerator::WaypointGenerator() {}

WaypointGenerator::~WaypointGenerator() {}

void WaypointGenerator::calculateWaypoint() {
  ROS_DEBUG(
      "\033[1;32m[WG] Generate Waypoint, current position: [%f, %f, "
      "%f].\033[0m",
      pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z);
  output_.waypoint_type = planner_info_.waypoint_type;

  switch (planner_info_.waypoint_type) {
    case hover: {
      if (last_wp_type_ != hover) {
        hover_position_ = pose_.pose.position;
      }
      output_.goto_position = hover_position_;
      ROS_DEBUG("[WG] Hover at: [%f, %f, %f].", output_.goto_position.x,
                output_.goto_position.y, output_.goto_position.z);
      getPathMsg();
      break;
    }
    case costmap: {
      output_.goto_position = fromPolarToCartesian(
          planner_info_.costmap_direction_e, planner_info_.costmap_direction_z,
          1.0, planner_info_.pose.pose.position);
      ROS_DEBUG("[WG] Costmap to: [%f, %f, %f].", output_.goto_position.x,
                output_.goto_position.y, output_.goto_position.z);
      getPathMsg();
      break;
    }

    case tryPath: {
      geometry_msgs::Point p;
      bool tree_available = getDirectionFromTree(
          p, planner_info_.path_node_positions, pose_.pose.position, goal_);
      double dist_goal = distance3DCartesian(goal_, pose_.pose.position);
      ros::Duration since_last_path = ros::Time::now() - planner_info_.last_path_time;
      if (tree_available && (planner_info_.obstacle_ahead || dist_goal > 4.0) && since_last_path < ros::Duration(5)) {
        ROS_DEBUG("[WG] Use calculated tree\n");
        output_.goto_position = fromPolarToCartesian(
            p.x, p.y, 1.0, planner_info_.pose.pose.position);
        getPathMsg();
      } else {
        ROS_DEBUG("[WG] No valid tree, go fast");
        goFast();
        output_.waypoint_type = direct;
      }
      break;
    }

    case direct: {
      ROS_DEBUG("[WG] No obstacle ahead, go fast");
      goFast();
      break;
    }

    case reachHeight: {
      ROS_DEBUG("[WG] Reach height first, go fast");
      goFast();
      break;
    }
    case goBack: {
      ROS_DEBUG("[WG] Too close, back off");
      backOff();
      break;
    }
  }
  last_wp_type_ = planner_info_.waypoint_type;
}

void WaypointGenerator::updateState(geometry_msgs::PoseStamped act_pose,
                                    geometry_msgs::PoseStamped goal,
                                    geometry_msgs::TwistStamped vel, bool stay,
                                    ros::Time t) {
  update_time_ = t;
  pose_ = act_pose;
  curr_vel_ = vel;
  goal_ = goal.pose.position;
  curr_yaw_ = tf::getYaw(pose_.pose.orientation);

  tf::Quaternion q(pose_.pose.orientation.x, pose_.pose.orientation.y,
                   pose_.pose.orientation.z, pose_.pose.orientation.w);
  tf::Matrix3x3 m(q);

  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  z_FOV_idx_.clear();
  calculateFOV(z_FOV_idx_, e_FOV_min_, e_FOV_max_, yaw, pitch);

  double velocity_x_ = curr_vel_.twist.linear.x;
  double velocity_y_ = curr_vel_.twist.linear.y;
  double velocity_z_ = curr_vel_.twist.linear.z;
  curr_vel_magnitude_ =
      sqrt(pow(velocity_x_, 2) + pow(velocity_y_, 2) + pow(velocity_z_, 2));

  if (stay) {
    planner_info_.waypoint_type = hover;
  }
}

// if there isn't any obstacle in front of the UAV, increase cruising speed
void WaypointGenerator::goFast() {
  tf::Vector3 vec;
  vec.setX(goal_.x - pose_.pose.position.x);
  vec.setY(goal_.y - pose_.pose.position.y);
  vec.setZ(goal_.z - pose_.pose.position.z);

  vec.normalize();

  output_.goto_position.x = pose_.pose.position.x + vec.getX();
  output_.goto_position.y = pose_.pose.position.y + vec.getY();
  output_.goto_position.z = pose_.pose.position.z + vec.getZ();

  // Prevent downward motion or move up if too close to ground
  if (planner_info_.use_ground_detection) {
    vec.normalize();

    if (planner_info_.over_obstacle &&
        pose_.pose.position.z <= planner_info_.min_flight_height &&
        output_.goto_position.z <= planner_info_.min_flight_height) {
      if ((planner_info_.min_flight_height - pose_.pose.position.z) > 0.5) {
        output_.goto_position.z = pose_.pose.position.z + 0.5;
      } else {
        output_.goto_position.z = planner_info_.min_flight_height;
      }
      planner_info_.too_low = true;
      ROS_INFO(
          "\033[1;36m [WG] Go Fast: Flight altitude too low (Minimal flight "
          "height: "
          "%f ) rising.\n \033[0m",
          planner_info_.min_flight_height);
    }
    if (planner_info_.over_obstacle &&
        pose_.pose.position.z > planner_info_.min_flight_height &&
        pose_.pose.position.z < planner_info_.min_flight_height + 0.5 &&
        vec.getZ() < 0) {
      output_.goto_position.z = pose_.pose.position.z;
      planner_info_.is_near_min_height = true;
      ROS_INFO(
          "\033[1;36m [WG] Go Fast: Preventing downward motion (Minimal flight "
          "height: %f ) \n \033[0m",
          planner_info_.min_flight_height);
    }
  }

  ROS_DEBUG("[WG] Go fast selected waypoint: [%f, %f, %f].",
            output_.goto_position.x, output_.goto_position.y,
            output_.goto_position.z);

  getPathMsg();
}

void WaypointGenerator::backOff() {
  tf::Vector3 vec;
  vec.setX(pose_.pose.position.x - planner_info_.back_off_point.x);
  vec.setY(pose_.pose.position.y - planner_info_.back_off_point.y);
  vec.setZ(0);
  vec.normalize();
  double new_len = 0.5;
  vec *= new_len;

  output_.goto_position.x = pose_.pose.position.x + vec.getX();
  output_.goto_position.y = pose_.pose.position.y + vec.getY();
  output_.goto_position.z = planner_info_.back_off_start_point.z;

  output_.position_waypoint = createPoseMsg(output_.goto_position, last_yaw_);
  output_.path.poses.push_back(output_.position_waypoint);
  transformPositionToVelocityWaypoint();
  last_yaw_ = curr_yaw_;

  ROS_DEBUG("[WG] Backoff Point: [%f, %f, %f].", planner_info_.back_off_point.x,
            planner_info_.back_off_point.y, planner_info_.back_off_point.z);
  ROS_DEBUG("[WG] Back off selected direction: [%f, %f, %f].", vec.getX(),
            vec.getY(), vec.getZ());
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
  geometry_msgs::Point a;
  a.x = std::abs(goal_.x - pose_.pose.position.x);
  a.y = std::abs(goal_.y - pose_.pose.position.y);
  a.z = std::abs(goal_.z - pose_.pose.position.z);
  float goal_acceptance_radius = 0.5f;

  if (a.x < goal_acceptance_radius && a.y < goal_acceptance_radius &&
      a.z < goal_acceptance_radius) {
    if (!reached_goal_) {
      yaw_reached_goal_ = tf::getYaw(pose_.pose.orientation);
    }
    reached_goal_ = true;
    return true;
  } else if (planner_info_.over_obstacle && a.x < goal_acceptance_radius &&
             a.y < goal_acceptance_radius) {
    if (!reached_goal_) {
      yaw_reached_goal_ = tf::getYaw(pose_.pose.orientation);
    }
    reached_goal_ = true;
    return true;
  } else
    reached_goal_ = false;
  return false;
}

// when taking off, first publish waypoints to reach the goal altitude
void WaypointGenerator::reachGoalAltitudeFirst() {
  output_.goto_position.x = planner_info_.offboard_pose.pose.position.x;
  output_.goto_position.y = planner_info_.offboard_pose.pose.position.y;
  output_.goto_position.z = pose_.pose.position.z + 0.5;

  //if goal lies directly overhead, do not yaw
  double x_diff = std::abs(goal_.x - pose_.pose.position.x);
  double y_diff = std::abs(goal_.y - pose_.pose.position.y);
  float goal_acceptance_radius = 1.0f;
  if (x_diff < goal_acceptance_radius && y_diff < goal_acceptance_radius) {
  	new_yaw_ = curr_yaw_;
  }

  //constrain speed
  geometry_msgs::Point pose_to_wp;
  pose_to_wp.x = output_.goto_position.x - pose_.pose.position.x;
  pose_to_wp.y = output_.goto_position.y - pose_.pose.position.y;
  pose_to_wp.z = output_.goto_position.z - pose_.pose.position.z;
  normalize(pose_to_wp);
  pose_to_wp.x *= planner_info_.min_speed;
  pose_to_wp.y *= planner_info_.min_speed;
  pose_to_wp.z *= planner_info_.min_speed;
  output_.goto_position.x = pose_.pose.position.x + pose_to_wp.x;
  output_.goto_position.y = pose_.pose.position.y + pose_to_wp.y;
  output_.goto_position.z = pose_.pose.position.z + pose_to_wp.z;
}

void WaypointGenerator::smoothWaypoint(double dt) {

  Eigen::Vector2f vel_cur_xy(curr_vel_.twist.linear.x, curr_vel_.twist.linear.y);
  Eigen::Vector2f vel_sp_xy(
      (output_.adapted_goto_position.x - last_position_waypoint_.pose.position.x) / dt,
      (output_.adapted_goto_position.y - last_position_waypoint_.pose.position.y) / dt);

  Eigen::Vector2f accel_diff = (vel_sp_xy - vel_cur_xy) / dt;
  Eigen::Vector2f accel_cur = (vel_cur_xy - last_velocity_) / dt;
  Eigen::Vector2f jerk_diff = (accel_diff - accel_cur) / dt;
  float max_jerk = max_jerk_limit_param_;

  // velocity-dependent max jerk
  if (min_jerk_limit_param_ > 0.001f) {
    max_jerk *= vel_cur_xy.norm();
    if (max_jerk < min_jerk_limit_param_) max_jerk = min_jerk_limit_param_;
  }

  if (jerk_diff.squaredNorm() > max_jerk * max_jerk && max_jerk > 0.001f) {
    jerk_diff = max_jerk * jerk_diff.normalized();
    vel_sp_xy = (jerk_diff * dt + accel_cur) * dt + vel_cur_xy;
  }

  output_.smoothed_goto_position.x = last_position_waypoint_.pose.position.x + vel_sp_xy(0) * dt;
  output_.smoothed_goto_position.y = last_position_waypoint_.pose.position.y + vel_sp_xy(1) * dt;
  output_.smoothed_goto_position.z = output_.adapted_goto_position.z;

  ROS_DEBUG("[WG] Smoothed waypoint: [%f %f %f].", output_.smoothed_goto_position.x,
		  output_.smoothed_goto_position.y, output_.smoothed_goto_position.z);
}

void WaypointGenerator::adaptSpeed() {
  ros::Duration since_last_velocity = ros::Time::now() - velocity_time_;
  double since_last_velocity_sec = since_last_velocity.toSec();

  if (!planner_info_.obstacle_ahead) {
    speed_ = std::min(speed_, planner_info_.max_speed);
    speed_ = velocityLinear(planner_info_.max_speed, 0.0,
                            planner_info_.velocity_sigmoid_slope, speed_,
                            since_last_velocity_sec);
  } else {
    speed_ = std::min(speed_, planner_info_.min_speed);
    speed_ = velocityLinear(planner_info_.min_speed, 0.0,
                            planner_info_.velocity_sigmoid_slope, speed_,
                            since_last_velocity_sec);
  }

  // check if new point lies in FOV
  int e_angle =
      elevationAnglefromCartesian(output_.adapted_goto_position.x, output_.adapted_goto_position.y, output_.adapted_goto_position.z, pose_.pose.position);
  int z_angle =
      azimuthAnglefromCartesian(output_.adapted_goto_position.x, output_.adapted_goto_position.y, output_.adapted_goto_position.z, pose_.pose.position);

  int e_index = elevationAngletoIndex(e_angle, ALPHA_RES);
  int z_index = azimuthAngletoIndex(z_angle, ALPHA_RES);

  if (std::find(z_FOV_idx_.begin(), z_FOV_idx_.end(), z_index) != z_FOV_idx_.end()) {
    waypoint_outside_FOV_ = false;
  } else {
    waypoint_outside_FOV_ = true;
    if (planner_info_.reach_altitude && !reached_goal_) {
      int ind_dist = 100;
      int i = 0;
      for (std::vector<int>::iterator it = z_FOV_idx_.begin(); it != z_FOV_idx_.end();
           ++it) {
        if (std::abs(z_FOV_idx_[i] - z_index) < ind_dist) {
          ind_dist = std::abs(z_FOV_idx_[i] - z_index);
        }
        i++;
      }
      double angle_diff = std::abs(ALPHA_RES * ind_dist);
      double hover_angle = 30;
      angle_diff = std::min(angle_diff, hover_angle);
      speed_ = speed_ * (1.0 - angle_diff / hover_angle);
      only_yawed_ = false;
      if (speed_ < 0.01) {
        only_yawed_ = true;
      }
    }
  }
  velocity_time_ = ros::Time::now();

  // calculate correction for computation delay
  ros::Duration since_update = ros::Time::now() - update_time_;
  double since_update_sec = since_update.toSec();
  double delta_dist = since_update_sec * curr_vel_magnitude_;
  speed_ += delta_dist;

  // set waypoint to correct speed
  geometry_msgs::Point pose_to_wp;
  pose_to_wp.x = output_.adapted_goto_position.x - pose_.pose.position.x;
  pose_to_wp.y = output_.adapted_goto_position.y - pose_.pose.position.y;
  pose_to_wp.z = output_.adapted_goto_position.z - pose_.pose.position.z;
  normalize(pose_to_wp);
  pose_to_wp.x *= speed_;
  pose_to_wp.y *= speed_;
  pose_to_wp.z *= speed_;

  output_.adapted_goto_position.x = pose_.pose.position.x + pose_to_wp.x;
  output_.adapted_goto_position.y = pose_.pose.position.y + pose_to_wp.y;
  output_.adapted_goto_position.z = pose_.pose.position.z + pose_to_wp.z;
  ROS_DEBUG("[WG] Speed adapted WP: [%f %f %f].", output_.adapted_goto_position.x,
		  output_.adapted_goto_position.y, output_.adapted_goto_position.z);
}

// create the message that is sent to the UAV
void WaypointGenerator::getPathMsg() {
  output_.path.header.frame_id = "/world";
  output_.adapted_goto_position = output_.goto_position;

  const ros::Time now = ros::Time::now();
  ros::Duration time_diff = now - last_time_;
  double dt = time_diff.toSec() > 0.0 ? time_diff.toSec() : 0.004;

  // If avoid sphere is used, project waypoint on sphere
  if (planner_info_.use_avoid_sphere && planner_info_.avoid_sphere_age < 100 &&
		  planner_info_.reach_altitude && !reached_goal_) {
    output_.adapted_goto_position = getSphereAdaptedWaypoint(
        pose_.pose.position, output_.goto_position,
        planner_info_.avoid_centerpoint, planner_info_.avoid_radius);
    ROS_DEBUG("[WG] Sphere adapted WP: [%f %f %f].",
              output_.adapted_goto_position.x, output_.adapted_goto_position.y,
              output_.adapted_goto_position.z);
  }

  // adapt waypoint to suitable speed (slow down if waypoint is out of FOV)
  new_yaw_ = nextYaw(pose_, output_.adapted_goto_position);
  adaptSpeed();
  output_.smoothed_goto_position = output_.adapted_goto_position;

  // go to flight height first or smooth wp
  if (!planner_info_.reach_altitude) {
    reachGoalAltitudeFirst();
    output_.adapted_goto_position = output_.goto_position;
    output_.smoothed_goto_position = output_.goto_position;
    ROS_DEBUG("[WG] after altitude func: [%f %f %f].",
              output_.smoothed_goto_position.x,
              output_.smoothed_goto_position.y,
              output_.smoothed_goto_position.z);
    ROS_DEBUG("[WG] pose altitude func: [%f %f %f].", pose_.pose.position.x,
              pose_.pose.position.y, pose_.pose.position.z);
  } else {
    smoothWaypoint(dt);
  }

  // change waypoint if drone is at goal or above
  if (withinGoalRadius()) {
    bool over_goal = false;
    if (planner_info_.use_ground_detection) {
      if (planner_info_.over_obstacle &&
          (planner_info_.is_near_min_height || planner_info_.too_low)) {
        over_goal = true;
        output_.smoothed_goto_position.x = goal_.x;
        output_.smoothed_goto_position.y = goal_.y;
        if (pose_.pose.position.z < goal_.z) {
          output_.smoothed_goto_position.z = goal_.z;
          ROS_DEBUG("[WG] Rising to goal");
        } else {
          output_.smoothed_goto_position.z = planner_info_.min_flight_height;
          ROS_INFO("[WG] Above Goal cannot go lower: Hovering");
        }
      }
    }
    if (!over_goal) {
      output_.smoothed_goto_position.x = goal_.x;
      output_.smoothed_goto_position.y = goal_.y;
      output_.smoothed_goto_position.z = goal_.z;
    }
  }

  if (reached_goal_) {
    new_yaw_ = yaw_reached_goal_;
  }

  ROS_DEBUG("[WG] Final waypoint: [%f %f %f].",
            output_.smoothed_goto_position.x, output_.smoothed_goto_position.y,
            output_.smoothed_goto_position.z);
  output_.position_waypoint =
      createPoseMsg(output_.smoothed_goto_position, new_yaw_);
  transformPositionToVelocityWaypoint();

  output_.path.poses.push_back(output_.position_waypoint);
  last_position_waypoint_ = output_.position_waypoint;
  last_yaw_ = curr_yaw_;
  last_velocity_ = Eigen::Vector2f(curr_vel_.twist.linear.x, curr_vel_.twist.linear.y);
  last_time_ = now;
}

void WaypointGenerator::getWaypoints(waypointResult &output) {
  calculateWaypoint();
  output = output_;
}

void WaypointGenerator::setPlannerInfo(avoidanceOutput input) {
  planner_info_ = input;
}
