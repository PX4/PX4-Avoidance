#include "local_planner.h"

LocalPlanner::LocalPlanner() {}

LocalPlanner::~LocalPlanner() {}

// update UAV pose
void LocalPlanner::setPose(const geometry_msgs::PoseStamped msg) {
  pose_.header = msg.header;
  pose_.pose.position = msg.pose.position;
  pose_.pose.orientation = msg.pose.orientation;
  curr_yaw_ = tf::getYaw(msg.pose.orientation);
  star_planner_.setPose(pose_);
  ground_detector_.setPose(pose_);

  if (!currently_armed_) {
    take_off_pose_.header = msg.header;
    take_off_pose_.pose.position = msg.pose.position;
    take_off_pose_.pose.orientation = msg.pose.orientation;

    time_t t = time(0);
    struct tm *now = localtime(&t);
    std::string buffer(80, '\0');
    strftime(&buffer[0], buffer.size(), "%F-%H-%M", now);
    log_name_ = buffer;

    reach_altitude_ = false;
  }

  if (!offboard_) {
    offboard_pose_.header = msg.header;
    offboard_pose_.pose.position = msg.pose.position;
    offboard_pose_.pose.orientation = msg.pose.orientation;
  }

  setVelocity();
}

// set parameters changed by dynamic rconfigure
void LocalPlanner::dynamicReconfigureSetParams(
    avoidance::LocalPlannerNodeConfig &config, uint32_t level) {
  histogram_box_size_.xmin_ = config.min_box_x_;
  histogram_box_size_.xmax_ = config.max_box_x_;
  histogram_box_size_.ymin_ = config.min_box_y_;
  histogram_box_size_.ymax_ = config.max_box_y_;
  histogram_box_size_.zmin_ = config.min_box_z_;
  histogram_box_size_.zmax_ = config.max_box_z_;
  min_dist_to_ground_ = config.min_dist_to_ground_;
  goal_cost_param_ = config.goal_cost_param_;
  smooth_cost_param_ = config.smooth_cost_param_;
  min_speed_ = config.min_speed_;
  max_speed_ = config.max_speed_;
  max_accel_z_ = config.max_accel_z_;
  keep_distance_ = config.keep_distance_;
  reproj_age_ = config.reproj_age_;

  no_progress_slope_ = config.no_progress_slope_;
  min_cloud_size_ = config.min_cloud_size_;
  avoid_radius_ = config.avoid_radius_;
  min_dist_backoff_ = config.min_dist_backoff_;
  pointcloud_timeout_hover_ = config.pointcloud_timeout_hover_;
  pointcloud_timeout_land_ = config.pointcloud_timeout_land_;
  childs_per_node_ = config.childs_per_node_;
  n_expanded_nodes_ = config.n_expanded_nodes_;
  tree_node_distance_ = config.tree_node_distance_;

  if (goal_z_param_ != config.goal_z_param) {
    goal_z_param_ = config.goal_z_param;
    setGoal();
  }

  stop_in_front_ = config.stop_in_front_;
  use_avoid_sphere_ = config.use_avoid_sphere_;
  use_ground_detection_ = config.use_ground_detection_;
  use_back_off_ = config.use_back_off_;
  use_VFH_star_ = config.use_VFH_star_;
  adapt_cost_params_ = config.adapt_cost_params_;
  send_obstacles_fcu_ = config.send_obstacles_fcu_;

  ROS_DEBUG("Dynamic reconfigure call");
  star_planner_.dynamicReconfigureSetStarParams(config, level);
  ground_detector_.dynamicReconfigureSetGroundParams(config, level);
}

// log Data
void LocalPlanner::logData() {
  if (currently_armed_ && offboard_) {
    // Print general Algorithm Data
    std::ofstream myfile1(("LocalPlanner_" + log_name_).c_str(),
                          std::ofstream::app);
    myfile1 << pose_.header.stamp.sec << "\t" << pose_.header.stamp.nsec << "\t"
            << pose_.pose.position.x << "\t" << pose_.pose.position.y << "\t"
            << pose_.pose.position.z << "\t" << waypt_p_.pose.position.x << "\t"
            << waypt_p_.pose.position.y << "\t" << waypt_p_.pose.position.z
            << "\t" << local_planner_mode_ << "\t" << reached_goal_ << "\t"
            << reach_altitude_ << "\t" << use_ground_detection_ << "\t"
            << obstacle_ << "\t" << height_change_cost_param_adapted_ << "\t"
            << over_obstacle_ << "\t" << too_low_ << "\t" << is_near_min_height_
            << "\t" << goal_.x << "\t" << goal_.y << "\t" << goal_.z << "\t"
            << algorithm_total_time_[algorithm_total_time_.size() - 1] << "\t"
            << tree_time_[tree_time_.size() - 1] << "\n";
    myfile1.close();

    ground_detector_.logData(log_name_);
  }
}

void LocalPlanner::setVelocity() {
  velocity_x_ = curr_vel_.twist.linear.x;
  velocity_y_ = curr_vel_.twist.linear.y;
  velocity_z_ = curr_vel_.twist.linear.z;
  velocity_mod_ =
      sqrt(pow(velocity_x_, 2) + pow(velocity_y_, 2) + pow(velocity_z_, 2));
}

void LocalPlanner::setGoal() {
  goal_.x = goal_x_param_;
  goal_.y = goal_y_param_;
  goal_.z = goal_z_param_;
  reached_goal_ = false;
  ROS_INFO("===== Set Goal ======: [%f, %f, %f].", goal_.x, goal_.y, goal_.z);
  initGridCells(&path_waypoints_);
  path_waypoints_.cells.push_back(pose_.pose.position);
  star_planner_.setGoal(goal_);
}

void LocalPlanner::runPlanner() {
  histogram_box_.setLimitsHistogramBox(pose_.pose.position,
                                       histogram_box_size_);

  if (use_ground_detection_ && currently_armed_) {
    if (offboard_) {
      std::clock_t t1 = std::clock();
      ground_detector_.initializeGroundBox(min_dist_to_ground_);
      ground_detector_.ground_box_.setLimitsGroundBox(
          pose_.pose.position, ground_detector_.ground_box_size_,
          min_dist_to_ground_);
      ground_detector_.setParams(min_dist_to_ground_, min_cloud_size_);
      ground_detector_.detectGround(complete_cloud_);
      ground_time_.push_back((std::clock() - t1) /
                             (double)(CLOCKS_PER_SEC / 1000));
    } else {
      ground_detector_.reset();
    }
  }

  geometry_msgs::Point temp_sphere_center;
  int sphere_points_counter = 0;
  std::clock_t t2 = std::clock();
  filterPointCloud(final_cloud_, closest_point_, temp_sphere_center,
                   distance_to_closest_point_, counter_close_points_backoff_,
                   sphere_points_counter, complete_cloud_, min_cloud_size_,
                   min_dist_backoff_, avoid_radius_, histogram_box_,
                   pose_.pose.position);

  safety_radius_ = adaptSafetyMarginHistogram(
      distance_to_closest_point_, final_cloud_.points.size(), min_cloud_size_);
  cloud_time_.push_back((std::clock() - t2) / (double)(CLOCKS_PER_SEC / 1000));

  if (use_avoid_sphere_ && reach_altitude_) {
    calculateSphere(avoid_centerpoint_, avoid_sphere_age_, temp_sphere_center,
                    sphere_points_counter, speed_);
  }

  determineStrategy();
}

void LocalPlanner::create2DObstacleRepresentation(const bool send_to_fcu) {
  // construct histogram if it is needed for the current local_planner_mode_
  // or if it is required by the FCU
  reprojectPoints(polar_histogram_);
  Histogram propagated_histogram = Histogram(2 * ALPHA_RES);
  Histogram new_histogram = Histogram(ALPHA_RES);
  to_fcu_histogram_.setZero();

  propagateHistogram(propagated_histogram, reprojected_points_,
                     reprojected_points_age_, reprojected_points_dist_, pose_);
  generateNewHistogram(new_histogram, final_cloud_, pose_);
  combinedHistogram(hist_is_empty_, new_histogram, propagated_histogram,
                    waypoint_outside_FOV_, z_FOV_idx_, e_FOV_min_, e_FOV_max_);
  if (send_to_fcu) {
    compressHistogramElevation(to_fcu_histogram_, new_histogram);
    updateObstacleDistanceMsg(to_fcu_histogram_);
  }
  polar_histogram_ = new_histogram;
}

void LocalPlanner::determineStrategy() {
  star_planner_.tree_age_++;
  tree_time_.push_back(0);

  if (!reach_altitude_) {
    ROS_INFO("\033[1;32m Reach height (%f) first: Go fast\n \033[0m",
             starting_height_);
    local_planner_mode_ = 0;
    goFast();

    if (send_obstacles_fcu_) {
      create2DObstacleRepresentation(true);
    }
  } else if (final_cloud_.points.size() > min_cloud_size_ && stop_in_front_ &&
             reach_altitude_) {
    obstacle_ = true;
    ROS_INFO("\033[1;32m There is an Obstacle Ahead stop in front\n \033[0m");
    local_planner_mode_ = 3;
    stopInFrontObstacles();

    if (send_obstacles_fcu_) {
      create2DObstacleRepresentation(true);
    }
  } else {
    if (((counter_close_points_backoff_ > 20 &&
          final_cloud_.points.size() > min_cloud_size_) ||
         back_off_) &&
        reach_altitude_ && use_back_off_) {
      local_planner_mode_ = 4;
      ROS_INFO("\033[1;32m There is an Obstacle too close! Back off\n \033[0m");
      if (!back_off_) {
        back_off_point_ = closest_point_;
        back_off_start_point_ = pose_.pose.position;
        back_off_ = true;
      }
      backOff();
      if (send_obstacles_fcu_) {
        create2DObstacleRepresentation(true);
      }

    } else {
      evaluateProgressRate();

      tf::Quaternion q(pose_.pose.orientation.x, pose_.pose.orientation.y,
                       pose_.pose.orientation.z, pose_.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      z_FOV_idx_.clear();
      calculateFOV(z_FOV_idx_, e_FOV_min_, e_FOV_max_, yaw, pitch);

      // visualization of FOV in RViz
      initGridCells(&FOV_cells_);
      geometry_msgs::Point p;
      for (int j = e_FOV_min_; j <= e_FOV_max_; j++) {
        for (int i = 0; i < z_FOV_idx_.size(); i++) {
          p.x = elevationIndexToAngle(j, ALPHA_RES);
          p.y = azimuthIndexToAngle(z_FOV_idx_[i], ALPHA_RES);
          p.z = 0;
          FOV_cells_.cells.push_back(p);
        }
      }

      create2DObstacleRepresentation(send_obstacles_fcu_);

      // decide how to proceed
      if (hist_is_empty_) {
        obstacle_ = false;
        local_planner_mode_ = 1;
        geometry_msgs::Point p;
        tree_available_ = getDirectionFromTree(
            p, tree_available_, star_planner_.path_node_positions_,
            pose_.pose.position, false);
        double dist_goal = distance3DCartesian(goal_, pose_.pose.position);
        if (tree_available_ && dist_goal > 4.0) {
          path_waypoints_.cells.push_back(p);
          ROS_INFO(
              "\033[1;32m There is NO Obstacle Ahead reuse old Tree\n \033[0m");
          getNextWaypoint();
        } else {
          goFast();
          ROS_INFO("\033[1;32m There is NO Obstacle Ahead go Fast\n \033[0m");
        }
      }

      if (!hist_is_empty_ && reach_altitude_) {
        obstacle_ = true;
        ROS_INFO(
            "\033[1;32m There is an Obstacle Ahead use Histogram\n \033[0m");
        local_planner_mode_ = 2;

        findFreeDirections(
            polar_histogram_, safety_radius_, path_candidates_, path_selected_,
            path_rejected_, path_blocked_, path_ground_, path_waypoints_,
            cost_path_candidates_, goal_, pose_, position_old_,
            goal_cost_param_, smooth_cost_param_,
            height_change_cost_param_adapted_, height_change_cost_param_, -1,
            false, only_yawed_, ALPHA_RES);

        if (use_VFH_star_) {
          star_planner_.ground_detector_ = GroundDetector(ground_detector_);
          star_planner_.setParams(min_cloud_size_, min_dist_backoff_,
                                  path_waypoints_, curr_yaw_,
                                  use_ground_detection_);
          star_planner_.setReprojectedPoints(reprojected_points_,
                                             reprojected_points_age_,
                                             reprojected_points_dist_);
          star_planner_.setCostParams(goal_cost_param_, smooth_cost_param_,
                                      height_change_cost_param_adapted_,
                                      height_change_cost_param_);
          star_planner_.setBoxSize(histogram_box_size_);
          star_planner_.setCloud(complete_cloud_);

          std::clock_t start_time = std::clock();
          star_planner_.buildLookAheadTree();
          tree_available_ = true;
          tree_time_[tree_time_.size() - 1] =
              ((std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));

          geometry_msgs::Point p;
          tree_available_ = getDirectionFromTree(
              p, tree_available_, star_planner_.path_node_positions_,
              pose_.pose.position, true);
          path_waypoints_.cells.push_back(p);
        } else {
          int e_min_idx = -1;
          if (use_ground_detection_) {
            min_flight_height_ = ground_detector_.getMinFlightHeight(
                pose_, curr_vel_, over_obstacle_, min_flight_height_,
                ground_margin_);
            e_min_idx = ground_detector_.getMinFlightElevationIndex(
                pose_, min_flight_height_, ALPHA_RES);
            ground_detector_.getHeightInformation(over_obstacle_, too_low_,
                                                  is_near_min_height_);
            ground_margin_ = ground_detector_.getMargin();
            if (over_obstacle_) {
              ROS_DEBUG("\033[1;36m Minimal flight height: %f) \n \033[0m",
                        min_flight_height_);
            }
          }

          findFreeDirections(
              polar_histogram_, safety_radius_, path_candidates_,
              path_selected_, path_rejected_, path_blocked_, path_ground_,
              path_waypoints_, cost_path_candidates_, goal_, pose_,
              position_old_, goal_cost_param_, smooth_cost_param_,
              height_change_cost_param_adapted_, height_change_cost_param_,
              e_min_idx, over_obstacle_, only_yawed_, ALPHA_RES);
          if (calculateCostMap(cost_path_candidates_, cost_idx_sorted_)) {
            local_planner_mode_ = 3;
            stopInFrontObstacles();
            stop_in_front_ = true;
            ROS_INFO(
                "\033[1;31m All directions blocked: Stopping in front "
                "obstacle. \n \033[0m");
          } else {
            getDirectionFromCostMap();
          }
        }
        if (!stop_in_front_) {
          getNextWaypoint();
        }
      }

      first_brake_ = true;
    }
  }
}

void LocalPlanner::updateObstacleDistanceMsg(Histogram hist) {
  sensor_msgs::LaserScan msg = {};
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.angle_increment = ALPHA_RES * M_PI / 180.0f;
  msg.range_min = 0.2f;
  msg.range_max = 20.0f;

  // turn idxs 180 degress to point to local north instead of south
  std::vector<int> z_FOV_idx_north;

  for (int i = 0; i < z_FOV_idx_.size(); i++) {
    int new_idx = z_FOV_idx_[i] + GRID_LENGTH_Z / 2;

    if (new_idx >= GRID_LENGTH_Z) {
      new_idx = new_idx - GRID_LENGTH_Z;
    }

    z_FOV_idx_north.push_back(new_idx);
  }

  for (int idx = 0; idx < GRID_LENGTH_Z; idx++) {
    if (std::find(z_FOV_idx_north.begin(), z_FOV_idx_north.end(), idx) ==
        z_FOV_idx_north.end()) {
      msg.ranges.push_back(UINT16_MAX);
      continue;
    }

    int hist_idx = idx - GRID_LENGTH_Z / 2;

    if (hist_idx < 0) {
      hist_idx = hist_idx + GRID_LENGTH_Z;
    }

    if (hist.get_dist(0, hist_idx) == 0.0) {
      msg.ranges.push_back(msg.range_max + 1.0f);
    } else {
      msg.ranges.push_back(hist.get_dist(0, hist_idx));
    }
  }

  distance_data_ = msg;
}

void LocalPlanner::updateObstacleDistanceMsg() {
  sensor_msgs::LaserScan msg = {};
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.angle_increment = ALPHA_RES * M_PI / 180.0f;
  msg.range_min = 0.2f;
  msg.range_max = 20.0f;

  distance_data_ = msg;
}

// get 3D points from old histogram
void LocalPlanner::reprojectPoints(Histogram histogram) {
  double n_points = 0;
  double dist, age;
  geometry_msgs::Point temp_array[4];
  reprojected_points_age_.clear();
  reprojected_points_dist_.clear();

  reprojected_points_.points.clear();
  reprojected_points_.header.stamp = final_cloud_.header.stamp;
  reprojected_points_.header.frame_id = "world";

  for (int e = 0; e < GRID_LENGTH_E; e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      if (histogram.get_bin(e, z) != 0) {
        n_points++;
        // transform from array index to angle
        double beta_e = elevationIndexToAngle(e, ALPHA_RES);
        double beta_z = azimuthIndexToAngle(z, ALPHA_RES);

        // transform from Polar to Cartesian
        temp_array[0] =
            fromPolarToCartesian(beta_e + ALPHA_RES / 2, beta_z + ALPHA_RES / 2,
                                 histogram.get_dist(e, z), position_old_);
        temp_array[1] =
            fromPolarToCartesian(beta_e - ALPHA_RES / 2, beta_z + ALPHA_RES / 2,
                                 histogram.get_dist(e, z), position_old_);
        temp_array[2] =
            fromPolarToCartesian(beta_e + ALPHA_RES / 2, beta_z - ALPHA_RES / 2,
                                 histogram.get_dist(e, z), position_old_);
        temp_array[3] =
            fromPolarToCartesian(beta_e - ALPHA_RES / 2, beta_z - ALPHA_RES / 2,
                                 histogram.get_dist(e, z), position_old_);

        for (int i = 0; i < 4; i++) {
          dist = distance3DCartesian(pose_.pose.position, temp_array[i]);
          age = histogram.get_age(e, z);

          if (dist < 2 * histogram_box_size_.xmax_ && dist > 0.3 &&
              age < reproj_age_) {
            reprojected_points_.points.push_back(pcl::PointXYZ(
                temp_array[i].x, temp_array[i].y, temp_array[i].z));
            reprojected_points_age_.push_back(age);
            reprojected_points_dist_.push_back(dist);
          }
        }
      }
    }
  }
}

// calculate the correct weight between fly over and fly around
void LocalPlanner::evaluateProgressRate() {
  if (reach_altitude_ && adapt_cost_params_ && !reached_goal_) {
    double goal_dist = distance3DCartesian(pose_.pose.position, goal_);
    double goal_dist_old = distance3DCartesian(position_old_, goal_);
    double time = std::clock() / (double)(CLOCKS_PER_SEC / 1000);
    double incline = (goal_dist - goal_dist_old) / (time - integral_time_old_);
    integral_time_old_ = time;

    goal_dist_incline_.push_back(incline);
    if (goal_dist_incline_.size() > dist_incline_window_size_) {
      goal_dist_incline_.pop_front();
    }

    double sum_incline = 0;
    int n_incline = 0;
    for (int i = 0; i < goal_dist_incline_.size(); i++) {
      sum_incline += goal_dist_incline_[i];
      n_incline++;
    }
    double avg_incline = sum_incline / n_incline;

    if (avg_incline > no_progress_slope_ &&
        goal_dist_incline_.size() == dist_incline_window_size_) {
      if (height_change_cost_param_adapted_ > 0.75) {
        height_change_cost_param_adapted_ -= 0.02;
      }
    }
    if (avg_incline < no_progress_slope_) {
      if (height_change_cost_param_adapted_ <
          height_change_cost_param_ - 0.03) {
        height_change_cost_param_adapted_ += 0.03;
      }
    }
    ROS_DEBUG("Progress rate to goal: %f, adapted height change cost: %f .",
              avg_incline, height_change_cost_param_adapted_);
  } else {
    height_change_cost_param_adapted_ = height_change_cost_param_;
  }
}

// get waypoint from sorted cost list
void LocalPlanner::getDirectionFromCostMap() {
  geometry_msgs::Point p;
  p.x = path_candidates_.cells[cost_idx_sorted_[0]].x;
  p.y = path_candidates_.cells[cost_idx_sorted_[0]].y;
  p.z = path_candidates_.cells[cost_idx_sorted_[0]].z;
  path_selected_.cells.push_back(p);
  path_waypoints_.cells.push_back(p);
}

// check that the selected direction is really free and transform it into a
// waypoint.
void LocalPlanner::getNextWaypoint() {
  int waypoint_index = path_waypoints_.cells.size();
  int e_angle = path_waypoints_.cells[waypoint_index - 1].x;
  int z_angle = path_waypoints_.cells[waypoint_index - 1].y;

  int e_index = elevationAngletoIndex(e_angle, ALPHA_RES);
  int z_index = azimuthAngletoIndex(z_angle, ALPHA_RES);

  geometry_msgs::Vector3Stamped setpoint =
      getWaypointFromAngle(e_angle, z_angle, pose_.pose.position);

  waypt_ = setpoint;

  ROS_DEBUG("Selected waypoint: [%f, %f, %f].", waypt_.vector.x,
            waypt_.vector.y, waypt_.vector.z);

  getPathMsg();
}

// if there isn't any obstacle in front of the UAV, increase cruising speed
void LocalPlanner::goFast() {
  tf::Vector3 vec;
  vec.setX(goal_.x - pose_.pose.position.x);
  vec.setY(goal_.y - pose_.pose.position.y);
  vec.setZ(goal_.z - pose_.pose.position.z);
  double new_len = vec.length() < 1.0 ? vec.length() : speed_;

  vec.normalize();
  vec *= new_len;

  waypt_.vector.x = pose_.pose.position.x + vec.getX();
  waypt_.vector.y = pose_.pose.position.y + vec.getY();
  waypt_.vector.z = pose_.pose.position.z + vec.getZ();

  // Prevent downward motion or move up if too close to ground
  if (use_ground_detection_) {
    vec.normalize();
    min_flight_height_ = ground_detector_.getMinFlightHeight(
        pose_, curr_vel_, over_obstacle_, min_flight_height_, ground_margin_);
    ground_detector_.getHeightInformation(over_obstacle_, too_low_,
                                          is_near_min_height_);
    ground_margin_ = ground_detector_.getMargin();

    if (over_obstacle_ && pose_.pose.position.z <= min_flight_height_ &&
        waypt_.vector.z <= min_flight_height_) {
      if ((min_flight_height_ - pose_.pose.position.z) > 0.5) {
        waypt_.vector.z = pose_.pose.position.z + 0.5;
      } else {
        waypt_.vector.z = min_flight_height_;
      }
      too_low_ = true;
      ROS_INFO(
          "\033[1;36m Go Fast: Flight altitude too low (Minimal flight height: "
          "%f ) rising.\n \033[0m",
          min_flight_height_);
    }
    if (over_obstacle_ && pose_.pose.position.z > min_flight_height_ &&
        pose_.pose.position.z < min_flight_height_ + 0.5 && vec.getZ() < 0) {
      waypt_.vector.z = pose_.pose.position.z;
      is_near_min_height_ = true;
      ROS_INFO(
          "\033[1;36m Go Fast: Preventing downward motion (Minimal flight "
          "height: %f ) \n \033[0m",
          min_flight_height_);
    }
  }

  // fill direction as straight ahead
  geometry_msgs::Point p;
  p.x = 0;
  p.y = 90;
  p.z = 0;
  path_waypoints_.cells.push_back(p);

  // reset candidates for visualization
  initGridCells(&path_candidates_);
  initGridCells(&path_rejected_);
  initGridCells(&path_blocked_);
  initGridCells(&path_selected_);
  initGridCells(&path_ground_);

  ROS_DEBUG("Go fast selected direction: [%f, %f, %f].", vec.getX(), vec.getY(),
            vec.getZ());
  ROS_DEBUG("Go fast selected waypoint: [%f, %f, %f].", waypt_.vector.x,
            waypt_.vector.y, waypt_.vector.z);

  getPathMsg();
}

void LocalPlanner::backOff() {
  tf::Vector3 vec;
  vec.setX(pose_.pose.position.x - back_off_point_.x);
  vec.setY(pose_.pose.position.y - back_off_point_.y);
  vec.setZ(0);
  vec.normalize();
  double new_len = 0.5;
  vec *= new_len;

  waypt_.vector.x = pose_.pose.position.x + vec.getX();
  waypt_.vector.y = pose_.pose.position.y + vec.getY();
  waypt_.vector.z = back_off_start_point_.z;

  // fill direction as straight ahead
  geometry_msgs::Point p;
  p.x = 0;
  p.y = 90;
  p.z = 0;
  path_waypoints_.cells.push_back(p);

  double dist = distance3DCartesian(pose_.pose.position, back_off_point_);
  if (dist > min_dist_backoff_ + 1.0) {
    back_off_ = false;
  }

  waypt_p_ = createPoseMsg(waypt_, last_yaw_);
  path_msg_.poses.push_back(waypt_p_);
  curr_yaw_ = last_yaw_;
  position_old_ = pose_.pose.position;

  ROS_DEBUG("Backoff Point: [%f, %f, %f].", back_off_point_.x,
            back_off_point_.y, back_off_point_.z);
  ROS_DEBUG("Distance to Back off Point: %f", dist);
  ROS_DEBUG("Back off selected direction: [%f, %f, %f].", vec.getX(),
            vec.getY(), vec.getZ());
  ROS_DEBUG("Back off selected waypoint: [%f, %f, %f].", waypt_.vector.x,
            waypt_.vector.y, waypt_.vector.z);
}

// check if the UAV has reached the goal set for the mission
bool LocalPlanner::withinGoalRadius() {
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
  } else if (over_obstacle_ && a.x < goal_acceptance_radius &&
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
void LocalPlanner::reachGoalAltitudeFirst() {
  starting_height_ =
      std::max(goal_.z - 0.5, take_off_pose_.pose.position.z + 1.0);
  if (pose_.pose.position.z < starting_height_) {
    waypt_.vector.x = offboard_pose_.pose.position.x;
    waypt_.vector.y = offboard_pose_.pose.position.y;
    waypt_.vector.z = pose_.pose.position.z + 0.5;
  } else {
    reach_altitude_ = true;
    getPathMsg();
  }
}

// smooth trajectory by liming the maximim accelleration possible
geometry_msgs::Vector3Stamped LocalPlanner::smoothWaypoint(
    geometry_msgs::Vector3Stamped wp) {
  geometry_msgs::Vector3Stamped smooth_waypt;
  std::clock_t t = std::clock();
  float dt = (t - t_prev_) / (float)(CLOCKS_PER_SEC);
  dt = dt > 0.0f ? dt : 0.004f;
  t_prev_ = t;

  Eigen::Vector2f vel_xy(curr_vel_.twist.linear.x, curr_vel_.twist.linear.y);
  Eigen::Vector2f vel_waypt_xy(
      (wp.vector.x - last_waypt_p_.pose.position.x) / dt,
      (wp.vector.y - last_waypt_p_.pose.position.y) / dt);
  Eigen::Vector2f vel_waypt_xy_prev(
      (last_waypt_p_.pose.position.x - last_last_waypt_p_.pose.position.x) / dt,
      (last_waypt_p_.pose.position.y - last_last_waypt_p_.pose.position.y) /
          dt);
  Eigen::Vector2f acc_waypt_xy((vel_waypt_xy - vel_waypt_xy_prev) / dt);

  if (acc_waypt_xy.norm() > (acc_waypt_xy.norm() / 2.0f)) {
    vel_xy = (acc_waypt_xy.norm() / 2.0f) * acc_waypt_xy.normalized() * dt +
             vel_waypt_xy_prev;
  }

  smooth_waypt.vector.x = last_waypt_p_.pose.position.x + vel_xy(0) * dt;
  smooth_waypt.vector.y = last_waypt_p_.pose.position.y + vel_xy(1) * dt;
  smooth_waypt.vector.z = wp.vector.z;

  ROS_DEBUG("Smoothed waypoint: [%f %f %f].", smooth_waypt.vector.x,
            smooth_waypt.vector.y, smooth_waypt.vector.z);
  return smooth_waypt;
}

void LocalPlanner::adaptSpeed() {
  double new_yaw = nextYaw(pose_, waypt_adapted_, last_yaw_);
  if (hasSameYawAndAltitude(last_waypt_p_, waypt_adapted_, new_yaw,
                            last_yaw_) &&
      !obstacle_) {
    speed_ = std::min(max_speed_, speed_ + 0.1);
  } else {
    speed_ = min_speed_;
  }

  // check if new point lies in FOV
  double wp_dist = sqrt((waypt_adapted_.vector.x - pose_.pose.position.x) *
                            (waypt_adapted_.vector.x - pose_.pose.position.x) +
                        (waypt_adapted_.vector.y - pose_.pose.position.y) *
                            (waypt_adapted_.vector.y - pose_.pose.position.y) +
                        (waypt_adapted_.vector.z - pose_.pose.position.z) *
                            (waypt_adapted_.vector.z - pose_.pose.position.z));

  int e_angle = elevationAnglefromCartesian(
      waypt_adapted_.vector.x, waypt_adapted_.vector.y, waypt_adapted_.vector.z,
      pose_.pose.position);
  int z_angle = azimuthAnglefromCartesian(
      waypt_adapted_.vector.x, waypt_adapted_.vector.y, waypt_adapted_.vector.z,
      pose_.pose.position);

  int e_index = elevationAngletoIndex(e_angle, ALPHA_RES);
  int z_index = azimuthAngletoIndex(z_angle, ALPHA_RES);

  if (std::find(z_FOV_idx_.begin(), z_FOV_idx_.end(), z_index) !=
      z_FOV_idx_.end()) {
    waypoint_outside_FOV_ = false;
  } else {
    waypoint_outside_FOV_ = true;
    if (reach_altitude_ && !reached_goal_ && obstacle_ && !back_off_) {
      int ind_dist = 100;
      int i = 0;
      for (std::vector<int>::iterator it = z_FOV_idx_.begin();
           it != z_FOV_idx_.end(); ++it) {
        if (std::abs(z_FOV_idx_[i] - z_index) < ind_dist) {
          ind_dist = std::abs(z_FOV_idx_[i] - z_index);
        }
        i++;
      }
      double angle_diff = ALPHA_RES * ind_dist;
      double hover_angle = 30;
      angle_diff = std::min(angle_diff, hover_angle);
      speed_ = speed_ * (1.0 - angle_diff / hover_angle);
      only_yawed_ = false;
      if (speed_ < 0.01) {
        only_yawed_ = true;
      }
    }
  }

  // set waypoint to correct speed
  geometry_msgs::Point pose_to_wp;
  pose_to_wp.x = waypt_adapted_.vector.x - pose_.pose.position.x;
  pose_to_wp.y = waypt_adapted_.vector.y - pose_.pose.position.y;
  pose_to_wp.z = waypt_adapted_.vector.z - pose_.pose.position.z;
  normalize(pose_to_wp);
  pose_to_wp.x *= speed_;
  pose_to_wp.y *= speed_;
  pose_to_wp.z *= speed_;

  waypt_adapted_.vector.x = pose_.pose.position.x + pose_to_wp.x;
  waypt_adapted_.vector.y = pose_.pose.position.y + pose_to_wp.y;
  waypt_adapted_.vector.z = pose_.pose.position.z + pose_to_wp.z;
  ROS_DEBUG("Speed adapted WP: [%f %f %f].", waypt_adapted_.vector.x,
            waypt_adapted_.vector.y, waypt_adapted_.vector.z);
}

// create the message that is sent to the UAV
void LocalPlanner::getPathMsg() {
  path_msg_.header.frame_id = "/world";
  last_last_waypt_p_ = last_waypt_p_;
  last_waypt_p_ = waypt_p_;
  last_yaw_ = curr_yaw_;
  waypt_adapted_ = waypt_;

  // If avoid sphere is used, project waypoint on sphere
  if (use_avoid_sphere_ && avoid_sphere_age_ < 100 && reach_altitude_ &&
      !reached_goal_ && !back_off_) {
    waypt_adapted_ = getSphereAdaptedWaypoint(
        pose_.pose.position, waypt_, avoid_centerpoint_, avoid_radius_);
    ROS_DEBUG("Sphere adapted WP: [%f %f %f].", waypt_adapted_.vector.x,
              waypt_adapted_.vector.y, waypt_adapted_.vector.z);
  }

  // adapt waypoint to suitable speed (slow down if waypoint is out of FOV)
  new_yaw_ = nextYaw(pose_, waypt_adapted_, last_yaw_);
  adaptSpeed();
  waypt_smoothed_ = waypt_adapted_;

  // go to flight height first or smooth wp
  if (!reach_altitude_) {
    reachGoalAltitudeFirst();
    waypt_adapted_ = waypt_;
    waypt_smoothed_ = waypt_;
  } else {
    if (!only_yawed_) {
      if (!reached_goal_ && !stop_in_front_ && smooth_waypoints_) {
        waypt_smoothed_ = smoothWaypoint(waypt_adapted_);
        new_yaw_ = nextYaw(pose_, waypt_smoothed_, last_yaw_);
      }
    }
  }

  if (back_off_ && use_back_off_) {
    new_yaw_ = last_yaw_;
    double dist = distance3DCartesian(pose_.pose.position, back_off_point_);
    if (dist > min_dist_backoff_ + 1.0) {
      back_off_ = false;
    }
    waypt_adapted_ = waypt_;
    waypt_smoothed_ = waypt_;
  }

  // change waypoint if drone is at goal or above
  if (withinGoalRadius()) {
    bool over_goal = false;
    if (use_ground_detection_) {
      if (over_obstacle_ && (is_near_min_height_ || too_low_)) {
        over_goal = true;
        waypt_smoothed_.vector.x = goal_.x;
        waypt_smoothed_.vector.y = goal_.y;
        if (pose_.pose.position.z < goal_.z) {
          waypt_smoothed_.vector.z = goal_.z;
          ROS_DEBUG("Rising to goal");
        } else {
          waypt_smoothed_.vector.z = min_flight_height_;
          ROS_INFO("Above Goal cannot go lower: Hoovering");
        }
      }
    }
    if (!over_goal) {
      waypt_smoothed_.vector.x = goal_.x;
      waypt_smoothed_.vector.y = goal_.y;
      waypt_smoothed_.vector.z = goal_.z;
    }
  }

  if (reached_goal_) {
    new_yaw_ = yaw_reached_goal_;
  }

  ROS_DEBUG("Final waypoint: [%f %f %f].", waypt_smoothed_.vector.x,
            waypt_smoothed_.vector.y, waypt_smoothed_.vector.z);
  waypt_p_ = createPoseMsg(waypt_smoothed_, new_yaw_);

  path_msg_.poses.push_back(waypt_p_);
  curr_yaw_ = new_yaw_;
  last_last_waypt_p_ = last_waypt_p_;
  last_waypt_p_ = waypt_p_;
  last_yaw_ = curr_yaw_;
  position_old_ = pose_.pose.position;
  smooth_waypoints_ = true;
}

void LocalPlanner::printAlgorithmStatistics() {
  ROS_DEBUG("Current pose: [%f, %f, %f].", pose_.pose.position.x,
            pose_.pose.position.y, pose_.pose.position.z);
  ROS_DEBUG("Velocity: [%f, %f, %f], module: %f.", velocity_x_, velocity_y_,
            velocity_z_, velocity_mod_);

  logData();

  if (withinGoalRadius()) {
    ROS_INFO("Goal Reached: Hoovering");
    cv::Scalar mean, std;
    ROS_DEBUG("------------ TIMING ----------- \n");
    cv::meanStdDev(algorithm_total_time_, mean, std);
    ROS_DEBUG("total mean %f std %f \n", mean[0], std[0]);
    cv::meanStdDev(cloud_time_, mean, std);
    ROS_DEBUG("cloud mean %f std %f \n", mean[0], std[0]);
    cv::meanStdDev(ground_time_, mean, std);
    ROS_DEBUG("ground detection mean %f std %f \n", mean[0], std[0]);
    cv::meanStdDev(tree_time_, mean, std);
    ROS_DEBUG("tree build mean %f std %f \n", mean[0], std[0]);
    ROS_DEBUG("------------------------------- \n");
  }
}

// stop in front of an obstacle at a distance defined by the variable
// keep_distance_
void LocalPlanner::stopInFrontObstacles() {
  if (first_brake_) {
    double braking_distance =
        fabsf(distance_to_closest_point_ - keep_distance_);
    Eigen::Vector2f pose_to_goal(goal_.x - pose_.pose.position.x,
                                 goal_.y - pose_.pose.position.y);
    goal_.x = pose_.pose.position.x +
              (braking_distance * pose_to_goal(0) / pose_to_goal.norm());
    goal_.y = pose_.pose.position.y +
              (braking_distance * pose_to_goal(1) / pose_to_goal.norm());
    first_brake_ = false;
  }
  ROS_INFO("New Stop Goal: [%.2f %.2f %.2f], obstacle distance %.2f. ", goal_.x,
           goal_.y, goal_.z, distance_to_closest_point_);
  goFast();
}

void LocalPlanner::getPosition(geometry_msgs::PoseStamped &pos) { pos = pose_; }

void LocalPlanner::getGoalPosition(geometry_msgs::Point &goal) { goal = goal_; }
void LocalPlanner::getAvoidSphere(geometry_msgs::Point &center, double &radius,
                                  int &sphere_age, bool &use_avoid_sphere) {
  center = avoid_centerpoint_;
  radius = avoid_radius_;
  sphere_age = avoid_sphere_age_;
  use_avoid_sphere = use_avoid_sphere_;
}

void LocalPlanner::getCloudsForVisualization(
    pcl::PointCloud<pcl::PointXYZ> &final_cloud,
    pcl::PointCloud<pcl::PointXYZ> &ground_cloud,
    pcl::PointCloud<pcl::PointXYZ> &reprojected_points) {
  final_cloud = final_cloud_;
  reprojected_points = reprojected_points_;
  if (use_ground_detection_) {
    ground_detector_.getGroundCloudForVisualization(ground_cloud);
  } else {
    pcl::PointCloud<pcl::PointXYZ> ground_cloud;
  }
}

void LocalPlanner::getCandidateDataForVisualization(
    nav_msgs::GridCells &path_candidates, nav_msgs::GridCells &path_selected,
    nav_msgs::GridCells &path_rejected, nav_msgs::GridCells &path_blocked,
    nav_msgs::GridCells &FOV_cells, nav_msgs::GridCells &path_ground) {
  path_candidates = path_candidates_;
  path_selected = path_selected_;
  path_rejected = path_rejected_;
  path_blocked = path_blocked_;
  path_ground = path_ground_;
  FOV_cells = FOV_cells_;
}

void LocalPlanner::getPathData(nav_msgs::Path &path_msg,
                               geometry_msgs::PoseStamped &waypt_p) {
  path_msg = path_msg_;
  waypt_p = waypt_p_;
}

void LocalPlanner::setCurrentVelocity(geometry_msgs::TwistStamped vel) {
  curr_vel_ = vel;
}

void LocalPlanner::getTree(
    std::vector<TreeNode> &tree, std::vector<int> &closed_set,
    std::vector<geometry_msgs::Point> &path_node_positions,
    bool &tree_available) {
  tree = star_planner_.tree_;
  closed_set = star_planner_.closed_set_;
  path_node_positions = star_planner_.path_node_positions_;
  tree_available = tree_available_;
}

void LocalPlanner::getWaypoints(geometry_msgs::Vector3Stamped &waypt,
                                geometry_msgs::Vector3Stamped &waypt_adapted,
                                geometry_msgs::Vector3Stamped &waypt_smoothed) {
  waypt = waypt_;
  waypt_adapted = waypt_adapted_;
  waypt_smoothed = waypt_smoothed_;
}

void LocalPlanner::sendObstacleDistanceDataToFcu(
    sensor_msgs::LaserScan &obstacle_distance) {
  obstacle_distance = distance_data_;
}
