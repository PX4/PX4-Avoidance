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

  if (!offboard_ && !mission_) {
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
  relevance_margin_e_degree_ = config.relevance_margin_e_degree_;
  relevance_margin_z_degree_ = config.relevance_margin_z_degree_;
  velocity_sigmoid_slope_ = config.velocity_sigmoid_slope_;

  no_progress_slope_ = config.no_progress_slope_;
  min_cloud_size_ = config.min_cloud_size_;
  min_realsense_dist_ = config.min_realsense_dist_;
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

  use_vel_setpoints_ = config.use_vel_setpoints_;
  stop_in_front_ = config.stop_in_front_;
  use_avoid_sphere_ = config.use_avoid_sphere_;
  use_ground_detection_ = config.use_ground_detection_;
  use_back_off_ = config.use_back_off_;
  use_VFH_star_ = config.use_VFH_star_;
  adapt_cost_params_ = config.adapt_cost_params_;
  send_obstacles_fcu_ = config.send_obstacles_fcu_;

  star_planner_.dynamicReconfigureSetStarParams(config, level);
  ground_detector_.dynamicReconfigureSetGroundParams(config, level);

  ROS_DEBUG("\033[0;35m[OA] Dynamic reconfigure call \033[0m");
}

// log Data
void LocalPlanner::logData() {
  if (currently_armed_ && offboard_) {
    // Print general Algorithm Data
    std::ofstream myfile1(("LocalPlanner_" + log_name_).c_str(),
                          std::ofstream::app);
    myfile1 << pose_.header.stamp.sec << "\t" << pose_.header.stamp.nsec << "\t"
            << pose_.pose.position.x << "\t" << pose_.pose.position.y << "\t"
            << pose_.pose.position.z << "\t" << reach_altitude_ << "\t"
            << use_ground_detection_ << "\t" << obstacle_ << "\t"
            << height_change_cost_param_adapted_ << "\t" << over_obstacle_
            << "\t" << too_low_ << "\t" << is_near_min_height_ << "\t"
            << goal_.x << "\t" << goal_.y << "\t" << goal_.z  << "\n";
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
  ROS_INFO("===== Set Goal ======: [%f, %f, %f].", goal_.x, goal_.y, goal_.z);
  initGridCells(&path_waypoints_);
  path_waypoints_.cells.push_back(pose_.pose.position);
  star_planner_.setGoal(goal_);
  goal_dist_incline_.clear();
}

void LocalPlanner::runPlanner() {
  // reset candidates for visualization
  initGridCells(&path_candidates_);
  initGridCells(&path_rejected_);
  initGridCells(&path_blocked_);
  initGridCells(&path_selected_);
  initGridCells(&path_ground_);
  stop_in_front_active_ = false;

  histogram_box_.setLimitsHistogramBox(pose_.pose.position,
                                       histogram_box_size_);

  if (use_ground_detection_ && currently_armed_) {
    if (offboard_ || mission_) {
      ground_detector_.initializeGroundBox(min_dist_to_ground_);
      ground_detector_.ground_box_.setLimitsGroundBox(
          pose_.pose.position, ground_detector_.ground_box_size_,
          min_dist_to_ground_);
      ground_detector_.setParams(min_dist_to_ground_, min_cloud_size_);
      ground_detector_.detectGround(complete_cloud_);

      min_flight_height_ = ground_detector_.getMinFlightHeight(
          pose_, curr_vel_, over_obstacle_, min_flight_height_, ground_margin_);
      ground_detector_.getHeightInformation(over_obstacle_, too_low_,
                                            is_near_min_height_);
      ground_margin_ = ground_detector_.getMargin();
    } else {
      ground_detector_.reset();
    }
  }

  geometry_msgs::Point temp_sphere_center;
  int sphere_points_counter = 0;
  filterPointCloud(final_cloud_, closest_point_, temp_sphere_center,
                   distance_to_closest_point_, counter_close_points_backoff_,
                   sphere_points_counter, complete_cloud_, min_cloud_size_,
                   min_dist_backoff_, avoid_radius_, histogram_box_,
                   pose_.pose.position, min_realsense_dist_);

  safety_radius_ = adaptSafetyMarginHistogram(
      distance_to_closest_point_, final_cloud_.points.size(), min_cloud_size_);

  if (use_avoid_sphere_ && reach_altitude_) {
    calculateSphere(avoid_centerpoint_, avoid_sphere_age_, temp_sphere_center,
                    sphere_points_counter, speed_);
  }

  determineStrategy();
}

void LocalPlanner::create2DObstacleRepresentation(const bool send_to_fcu) {
  // construct histogram if it is needed
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

  if (!reach_altitude_) {
    starting_height_ =
        std::max(goal_.z - 0.5, take_off_pose_.pose.position.z + 1.0);
    ROS_INFO("\033[1;35m[OA] Reach height (%f) first: Go fast\n \033[0m",
             starting_height_);
    waypoint_type_ = reachHeight;

    if (pose_.pose.position.z > starting_height_) {
      reach_altitude_ = true;
      waypoint_type_ = direct;
    }

    if (send_obstacles_fcu_) {
      create2DObstacleRepresentation(true);
    }
  } else if (final_cloud_.points.size() > min_cloud_size_ && stop_in_front_ &&
             reach_altitude_) {
    obstacle_ = true;
    ROS_INFO(
        "\033[1;35m[OA] There is an Obstacle Ahead stop in front\n \033[0m");
    stopInFrontObstacles();
    waypoint_type_ = direct;

    if (send_obstacles_fcu_) {
      create2DObstacleRepresentation(true);
    }
  } else {
    if (((counter_close_points_backoff_ > 200 &&
          final_cloud_.points.size() > min_cloud_size_) ||
         back_off_) &&
        reach_altitude_ && use_back_off_) {
      if (!back_off_) {
        back_off_point_ = closest_point_;
        back_off_start_point_ = pose_.pose.position;
        back_off_ = true;
      } else {
        double dist = distance3DCartesian(pose_.pose.position, back_off_point_);
        if (dist > min_dist_backoff_ + 1.0) {
          back_off_ = false;
        }
      }
      waypoint_type_ = goBack;
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

      // check histogram relevance
      bool hist_relevant = true;
      if (!hist_is_empty_) {
        hist_relevant = false;
        int relevance_margin_z_cells =
            std::ceil(relevance_margin_z_degree_ / ALPHA_RES);
        int relevance_margin_e_cells =
            std::ceil(relevance_margin_e_degree_ / ALPHA_RES);
        int n_occupied_cells = 0;

        int goal_e_angle = elevationAnglefromCartesian(
            goal_.x, goal_.y, goal_.z, pose_.pose.position);
        int goal_z_angle = azimuthAnglefromCartesian(goal_.x, goal_.y, goal_.z,
                                                     pose_.pose.position);

        int goal_e_index = elevationAngletoIndex(goal_e_angle, ALPHA_RES);
        int goal_z_index = azimuthAngletoIndex(goal_z_angle, ALPHA_RES);

        for (int e = goal_e_index - relevance_margin_e_cells;
             e < goal_e_index + relevance_margin_e_cells; e++) {
          for (int z = goal_z_index - relevance_margin_z_cells;
               z < goal_z_index + relevance_margin_z_cells; z++) {
            if (polar_histogram_.get_bin(e, z) > 0) {
              n_occupied_cells++;
            }
          }
        }
        if (n_occupied_cells > 0) {
          hist_relevant = true;
        }
      }

      // decide how to proceed
      if (hist_is_empty_ || !hist_relevant) {
        obstacle_ = false;
        waypoint_type_ = tryPath;
      }

      if (!hist_is_empty_ && hist_relevant && reach_altitude_) {
        obstacle_ = true;

        findFreeDirections(
            polar_histogram_, safety_radius_, path_candidates_, path_selected_,
            path_rejected_, path_blocked_, path_ground_, path_waypoints_,
            cost_path_candidates_, goal_, pose_, position_old_,
            goal_cost_param_, smooth_cost_param_,
            height_change_cost_param_adapted_, height_change_cost_param_, -1,
            false, velocity_mod_ < 0.1, ALPHA_RES);

        if (use_VFH_star_) {
          star_planner_.ground_detector_ = GroundDetector(ground_detector_);
          star_planner_.setParams(min_cloud_size_, min_dist_backoff_,
                                  path_waypoints_, curr_yaw_,
                                  use_ground_detection_, min_realsense_dist_);
          star_planner_.setReprojectedPoints(reprojected_points_,
                                             reprojected_points_age_,
                                             reprojected_points_dist_);
          star_planner_.setCostParams(goal_cost_param_, smooth_cost_param_,
                                      height_change_cost_param_adapted_,
                                      height_change_cost_param_);
          star_planner_.setBoxSize(histogram_box_size_);
          star_planner_.setCloud(complete_cloud_);
          star_planner_.buildLookAheadTree();

          waypoint_type_ = tryPath;
          last_path_time_ = ros::Time::now();
        } else {
          int e_min_idx = -1;
          if (use_ground_detection_) {
            e_min_idx = ground_detector_.getMinFlightElevationIndex(
                pose_, min_flight_height_, ALPHA_RES);
            if (over_obstacle_) {
              ROS_DEBUG("\033[1;36m[OA] Minimal flight height: %f) \n \033[0m",
                        min_flight_height_);
            }
          }

          findFreeDirections(
              polar_histogram_, safety_radius_, path_candidates_,
              path_selected_, path_rejected_, path_blocked_, path_ground_,
              path_waypoints_, cost_path_candidates_, goal_, pose_,
              position_old_, goal_cost_param_, smooth_cost_param_,
              height_change_cost_param_adapted_, height_change_cost_param_,
              e_min_idx, over_obstacle_, velocity_mod_ < 0.1, ALPHA_RES);
          if (calculateCostMap(cost_path_candidates_, cost_idx_sorted_)) {
            stopInFrontObstacles();
            waypoint_type_ = direct;
            stop_in_front_ = true;
            ROS_INFO(
                "\033[1;35m[OA] All directions blocked: Stopping in front "
                "obstacle. \n \033[0m");
          } else {
            getDirectionFromCostMap();
            waypoint_type_ = costmap;
          }
        }
      }

      first_brake_ = true;
    }
  }
  position_old_ = pose_.pose.position;
}

void LocalPlanner::updateObstacleDistanceMsg(Histogram hist) {
  sensor_msgs::LaserScan msg = {};
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "local_origin";
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
  msg.header.frame_id = "local_origin";
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
  reprojected_points_.header.frame_id = "local_origin";

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
  if (reach_altitude_ && adapt_cost_params_) {
    double goal_dist = distance3DCartesian(pose_.pose.position, goal_);
    double goal_dist_old = distance3DCartesian(position_old_, goal_);

    ros::Time time = ros::Time::now();
    ros::Duration time_diff = time - integral_time_old_;
    double incline = (goal_dist - goal_dist_old) / (time_diff.toSec());
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
    ROS_DEBUG(
        "\033[0;35m[OA] Progress rate to goal: %f, adapted height change cost: "
        "%f .\033[0m",
        avg_incline, height_change_cost_param_adapted_);
  } else {
    height_change_cost_param_adapted_ = height_change_cost_param_;
  }
}

// get waypoint from sorted cost list
void LocalPlanner::getDirectionFromCostMap() {
  costmap_direction_e_ = path_candidates_.cells[cost_idx_sorted_[0]].x;
  costmap_direction_z_ = path_candidates_.cells[cost_idx_sorted_[0]].y;
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
    stop_in_front_active_ = true;
  }
  ROS_INFO(
      "\033[0;35m [OA] New Stop Goal: [%.2f %.2f %.2f], obstacle distance "
      "%.2f. \033[0m",
      goal_.x, goal_.y, goal_.z, distance_to_closest_point_);
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
    nav_msgs::GridCells& path_candidates, nav_msgs::GridCells& path_selected,
    nav_msgs::GridCells& path_rejected, nav_msgs::GridCells& path_blocked,
    nav_msgs::GridCells& FOV_cells, nav_msgs::GridCells& path_ground) {
  path_candidates = path_candidates_;
  path_selected = path_selected_;
  path_rejected = path_rejected_;
  path_blocked = path_blocked_;
  path_ground = path_ground_;
  FOV_cells = FOV_cells_;
}

void LocalPlanner::setCurrentVelocity(const geometry_msgs::TwistStamped& vel) {
  curr_vel_ = vel;
}

void LocalPlanner::getTree(
    std::vector<TreeNode> &tree, std::vector<int> &closed_set,
    std::vector<geometry_msgs::Point> &path_node_positions) {
  tree = star_planner_.tree_;
  closed_set = star_planner_.closed_set_;
  path_node_positions = star_planner_.path_node_positions_;
}

void LocalPlanner::sendObstacleDistanceDataToFcu(
    sensor_msgs::LaserScan &obstacle_distance) {
  obstacle_distance = distance_data_;
}

void LocalPlanner::getAvoidanceOutput(avoidanceOutput &out) {
  out.waypoint_type = waypoint_type_;

  out.pose = pose_;
  out.obstacle_ahead = obstacle_;
  out.reach_altitude = reach_altitude_;
  out.min_speed = min_speed_;
  out.max_speed = max_speed_;
  out.velocity_sigmoid_slope = velocity_sigmoid_slope_;
  out.last_path_time = last_path_time_;

  out.use_avoid_sphere = use_avoid_sphere_;
  out.avoid_sphere_age = avoid_sphere_age_;
  out.avoid_centerpoint = avoid_centerpoint_;
  out.avoid_radius = avoid_radius_;

  out.use_ground_detection = use_ground_detection_;
  out.over_obstacle = over_obstacle_;
  out.is_near_min_height = is_near_min_height_;
  out.too_low = too_low_;
  out.min_flight_height = min_flight_height_;

  out.back_off_point = back_off_point_;
  out.back_off_start_point = back_off_start_point_;
  out.min_dist_backoff = min_dist_backoff_;

  out.take_off_pose = take_off_pose_;
  out.offboard_pose = offboard_pose_;

  out.costmap_direction_e = costmap_direction_e_;
  out.costmap_direction_z = costmap_direction_z_;
  out.path_node_positions = star_planner_.path_node_positions_;
}
