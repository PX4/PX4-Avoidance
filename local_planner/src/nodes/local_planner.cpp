#include "local_planner.h"

LocalPlanner::LocalPlanner() {
}

LocalPlanner::~LocalPlanner() {
}

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
    struct tm * now = localtime(&t);
    std::string buffer(80, '\0');
    strftime(&buffer[0], buffer.size(), "%F-%H-%M", now);
    log_name_ = buffer;

    reach_altitude_ = false;
  }

  if(!offboard_){
    reach_altitude_ = false;
  }

  setVelocity();
}

// set parameters changed by dynamic rconfigure
void LocalPlanner::dynamicReconfigureSetParams(avoidance::LocalPlannerNodeConfig & config, uint32_t level) {
  histogram_box_size_.xmin = config.min_box_x_;
  histogram_box_size_.xmax = config.max_box_x_;
  histogram_box_size_.ymin = config.min_box_y_;
  histogram_box_size_.ymax = config.max_box_y_;
  histogram_box_size_.zmin = config.min_box_z_;
  histogram_box_size_.zmax = config.max_box_z_;
  min_dist_to_ground_ = config.min_dist_to_ground_;
  goal_cost_param_ = config.goal_cost_param_;
  smooth_cost_param_ = config.smooth_cost_param_;
  min_speed_ = config.min_speed_;
  max_speed_ = config.max_speed_;
  max_accel_z_ = config.max_accel_z_;
  keep_distance_ = config.keep_distance_;

  no_progress_slope_ = config.no_progress_slope_;
  min_cloud_size_ = config.min_cloud_size_;
  avoid_radius_ = config.avoid_radius_;
  min_dist_backoff_ = config.min_dist_backoff_;
  pointcloud_timeout_hover_ = config.pointcloud_timeout_hover_;
  pointcloud_timeout_land_ = config.pointcloud_timeout_land_;
  childs_per_node_ = config.childs_per_node_;
  n_expanded_nodes_ = config.n_expanded_nodes_;
  tree_node_distance_  = config.tree_node_distance_;

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

  std::cout << "Calling rconf\n";
  star_planner_.dynamicReconfigureSetStarParams(config, level);
  ground_detector_.dynamicReconfigureSetGroundParams(config, level);
}

// log Data
void LocalPlanner::logData() {

  if (currently_armed_ && offboard_) {
    //Print general Algorithm Data
    std::ofstream myfile1(("LocalPlanner_" + log_name_).c_str(), std::ofstream::app);
    myfile1 << pose_.header.stamp.sec << "\t" << pose_.header.stamp.nsec << "\t" << pose_.pose.position.x << "\t" << pose_.pose.position.y << "\t" << pose_.pose.position.z << "\t" << local_planner_mode_ << "\t" << reached_goal_ << "\t" << "\t"
        << use_ground_detection_ << "\t" << obstacle_ << "\t" << height_change_cost_param_adapted_ << "\t" << over_obstacle_ << "\t" << too_low_ << "\t" << is_near_min_height_ << "\t" << goal_.x << "\t" << goal_.y << "\t" << goal_.z << "\t"
        << hovering_ << "\t" << algorithm_total_time_[algorithm_total_time_.size() - 1] << "\t" << tree_time_[tree_time_.size() - 1] << "\n";
    myfile1.close();

    ground_detector_.logData(log_name_);
  }
}

void LocalPlanner::setVelocity() {
  velocity_x_ = curr_vel_.twist.linear.x;
  velocity_y_ = curr_vel_.twist.linear.y;
  velocity_z_ = curr_vel_.twist.linear.z;
  velocity_mod_ = sqrt(pow(velocity_x_, 2) + pow(velocity_y_, 2) + pow(velocity_z_, 2));
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

void LocalPlanner::determineStrategy() {
  star_planner_.tree_age_++;
  star_planner_.tree_new_ = false;
  tree_time_.push_back(0);

  if(!reach_altitude_){
    std::cout << "\033[1;32m Reach height ("<<starting_height_<<") first: Go fast\n \033[0m";
    local_planner_mode_ = 0;
    goFast();
  } else if (final_cloud_.points.size() > min_cloud_size_ && stop_in_front_ && reach_altitude_) {
    obstacle_ = true;
    std::cout << "\033[1;32m There is an Obstacle Ahead stop in front\n \033[0m";
    local_planner_mode_ = 3;
    stopInFrontObstacles();
  } else {
    if (((counter_close_points_backoff_ > 20 && final_cloud_.points.size() > min_cloud_size_) || back_off_) && reach_altitude_ && use_back_off_) {
      local_planner_mode_ = 4;
      std::cout << "\033[1;32m There is an Obstacle too close! Back off\n \033[0m";
      if (!back_off_) {
        back_off_point_ = closest_point_;
        back_off_ = true;
      }
      backOff();

    } else {
      evaluateProgressRate();
      reprojectPoints();

      tf::Quaternion q(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      z_FOV_idx_.clear();
      calculateFOV(z_FOV_idx_, e_FOV_min_, e_FOV_max_, yaw, pitch);

      //visualization of FOV in RViz
      initGridCells(&FOV_cells_);
      geometry_msgs::Point p;
      for (int j = e_FOV_min_; j <= e_FOV_max_; j++) {
        for (int i = 0; i < z_FOV_idx_.size(); i++) {
          p.x = elevationIndexToAngle(j, alpha_res);
          p.y = azimuthIndexToAngle(z_FOV_idx_[i], alpha_res);
          p.z = 0;
          FOV_cells_.cells.push_back(p);
        }
      }

      Histogram propagated_histogram = propagateHistogram(reprojected_points_, reprojected_points_age_, reprojected_points_dist_, pose_);
      Histogram new_histogram = generateNewHistogram(final_cloud_, pose_);
      Histogram polar_histogram_ = combinedHistogram(hist_is_empty_, new_histogram, propagated_histogram, waypoint_outside_FOV_, z_FOV_idx_, e_FOV_min_, e_FOV_max_);
      polar_histogram_old_ = polar_histogram_;

      //decide how to proceed
      if (hist_is_empty_) {
        obstacle_ = false;
        local_planner_mode_ = 1;
        if (star_planner_.getDirectionFromTree(path_waypoints_)) {
          std::cout << "\033[1;32m There is NO Obstacle Ahead reuse old Tree\n \033[0m";
          getNextWaypoint();
        } else {
          goFast();
          std::cout << "\033[1;32m There is NO Obstacle Ahead go Fast\n \033[0m";
        }
      }

      if (!hist_is_empty_ && reach_altitude_) {
        obstacle_ = true;
        std::cout << "\033[1;32m There is an Obstacle Ahead use Histogram\n \033[0m";
        local_planner_mode_ = 2;

        if (use_VFH_star_) {
          star_planner_.ground_detector_ = GroundDetector(ground_detector_);
          star_planner_.setParams(min_cloud_size_, min_dist_backoff_, path_waypoints_, curr_yaw_, use_ground_detection_);
          star_planner_.setReprojectedPoints(reprojected_points_, reprojected_points_age_, reprojected_points_dist_);
          star_planner_.setCostParams(goal_cost_param_, smooth_cost_param_, height_change_cost_param_adapted_, height_change_cost_param_);
          star_planner_.setBoxSize(histogram_box_size_);
          star_planner_.setCloud(complete_cloud_);

          std::clock_t start_time = std::clock();
          star_planner_.buildLookAheadTree(yaw);
          tree_time_[tree_time_.size() - 1] = ((std::clock() - start_time) / (double) (CLOCKS_PER_SEC / 1000));

          star_planner_.getDirectionFromTree(path_waypoints_);
        } else {

          int e_min_idx = -1;
          if (use_ground_detection_) {
            min_flight_height_ = ground_detector_.getMinFlightHeight(pose_, curr_vel_, over_obstacle_, min_flight_height_, ground_margin_);
            e_min_idx = ground_detector_.getMinFlightElevationIndex(pose_, min_flight_height_, alpha_res);
            ground_detector_.getFlags(over_obstacle_, too_low_, is_near_min_height_);
            ground_margin_ = ground_detector_.getMargin();
            if (over_obstacle_) {
              std::cout << "\033[1;36m Minimal flight height: " << min_flight_height_ << "\n \033[0m";
            }
          }

          findFreeDirections(polar_histogram_, safety_radius_, path_candidates_, path_selected_, path_rejected_, path_blocked_, path_ground_, path_waypoints_, cost_path_candidates_, goal_, pose_, position_old_, goal_cost_param_, smooth_cost_param_,
                             height_change_cost_param_adapted_, height_change_cost_param_, e_min_idx, over_obstacle_, only_yawed_, alpha_res);
          calculateCostMap(cost_path_candidates_, cost_idx_sorted_);
          getDirectionFromCostMap();
        }

        getNextWaypoint();
      }

      first_brake_ = true;
    }
  }
}

// get 3D points from old histogram
void LocalPlanner::reprojectPoints() {

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
      if (polar_histogram_old_.get_bin(e, z) != 0) {
        n_points++;
        //transform from array index to angle
        double beta_e = elevationIndexToAngle(e, ALPHA_RES);
        double beta_z = azimuthIndexToAngle(z, ALPHA_RES);

        //transform from Polar to Cartesian
        temp_array[0] = fromPolarToCartesian(beta_e + ALPHA_RES / 2, beta_z + ALPHA_RES / 2, polar_histogram_old_.get_dist(e, z), position_old_);
        temp_array[1] = fromPolarToCartesian(beta_e - ALPHA_RES / 2, beta_z + ALPHA_RES / 2, polar_histogram_old_.get_dist(e, z), position_old_);
        temp_array[2] = fromPolarToCartesian(beta_e + ALPHA_RES / 2, beta_z - ALPHA_RES / 2, polar_histogram_old_.get_dist(e, z), position_old_);
        temp_array[3] = fromPolarToCartesian(beta_e - ALPHA_RES / 2, beta_z - ALPHA_RES / 2, polar_histogram_old_.get_dist(e, z), position_old_);

        for (int i = 0; i < 4; i++) {
          dist = distance3DCartesian(pose_.pose.position, temp_array[i]);
          age = polar_histogram_old_.get_age(e, z);

          if (dist < 2 * histogram_box_size_.xmax && dist > 0.3 && age < AGE_LIM) {
            reprojected_points_.points.push_back(pcl::PointXYZ(temp_array[i].x, temp_array[i].y, temp_array[i].z));
            reprojected_points_age_.push_back(age);
            reprojected_points_dist_.push_back(dist);
          }
        }
      }
    }
  }
}

bool LocalPlanner::getDirectionFromTree() {
  if (tree_available_) {
    int size = path_node_positions_.size();
    geometry_msgs::Point p;

    if (tree_new_) {
      int goal_node_nr = path_node_origins_[size-2];
      p.x = tree_[goal_node_nr].last_e;
      p.y = tree_[goal_node_nr].last_z;
      p.z = 0;

      path_selected_.cells.push_back(p);
      path_waypoints_.cells.push_back(p);

    } else {
      int min_dist_idx = 0;
      int second_min_dist_idx = 0;
      double min_dist = inf;

      std::vector<double> distances;
      for (int i = 0; i < size; i++) {
        distances.push_back(distance3DCartesian(pose_.pose.position, path_node_positions_[i]));
        if (distances[i] < min_dist) {
          second_min_dist_idx = min_dist_idx;
          min_dist = distances[i];
          min_dist_idx = i;
        }
      }
      if (min_dist > 3.0 || min_dist_idx == 0) {
        tree_available_ = false;
      } else {
        int wp_idx = std::min(min_dist_idx, second_min_dist_idx);
        if (distances[wp_idx] < 0.3 && wp_idx != 0) {
          wp_idx--;
        }
        int wp_z = floor(atan2(path_node_positions_[wp_idx].x - pose_.pose.position.x, path_node_positions_[wp_idx].y - pose_.pose.position.y) * 180.0 / PI);  //azimuthal angle
        int wp_e = floor(atan((path_node_positions_[wp_idx].z - pose_.pose.position.z) / sqrt(pow((path_node_positions_[wp_idx].x - pose_.pose.position.x), 2) + pow((path_node_positions_[wp_idx].y - pose_.pose.position.y), 2))) * 180.0 / PI);

        p.x = wp_e;
        p.y = wp_z;
        p.z = 0;

        path_selected_.cells.push_back(p);
        path_waypoints_.cells.push_back(p);
      }
    }
  }

  return tree_available_;
}

void LocalPlanner::buildLookAheadTree(){
  std::clock_t start_time = std::clock();
  new_cloud_ = false;
  tree_.clear();
  closed_set_.clear();
  //insert first node
  tree_.push_back(TreeNode(0, 0, pose_.pose.position));
  tree_.back().setCosts(treeHeuristicFunction(0), treeHeuristicFunction(0));

  int origin = 0;
  double min_c = INF;
  int min_c_node = 0;
  int n = 0;

  while (n < n_expanded_nodes_) {
    geometry_msgs::Point origin_position = tree_[origin_].getPosition();
    bool add_nodes = true;


    //crop pointcloud
    histogram_box_.setLimitsHistogramBox(origin_position, histogram_box_size_);
    filterPointCloud(complete_cloud_);
    //if too close to obstacle, we do not want to go there (equivalent to back off)
    if (counter_close_points_backoff_ > 20 && final_cloud_.points.size() > 160) {
      tree_[origin_].total_cost = INF;
      add_nodes = false;
    }

    if (add_nodes || origin_ == 0) {
      //build new histogram
      std::vector<int> z_FOV_idx;
      int e_FOV_min, e_FOV_max;
      calculateFOV(z_FOV_idx, e_FOV_min, e_FOV_max, tree_[origin_].yaw, 0.0);  //assume pitch is zero at every node
      Histogram histogram = createPolarHistogram(z_FOV_idx, e_FOV_min, e_FOV_max);

      //calculate candidates
      findFreeDirections(histogram);
      calculateCostMap();

      //insert new nodes
      int depth = tree_[origin_].depth + 1;

      int goal_z = floor(atan2(goal_.x - origin_position.x, goal_.y - origin_position.y) * 180.0 / PI);  //azimuthal angle
      int goal_e = floor(atan((goal_.z - origin_position.z) / sqrt(pow((goal_.x - origin_position.x), 2) + pow((goal_.y - origin_position.y), 2))) * 180.0 / PI);
      int goal_e_idx = (goal_e - alpha_res + 90) / alpha_res;
      int goal_z_idx = (goal_z - alpha_res + 180) / alpha_res;

      int childs = std::min(childs_per_node_, (int)path_candidates_.cells.size());
      for (int i = 0; i < childs; i++) {
        int e = path_candidates_.cells[cost_idx_sorted_[i]].x;
        int z = path_candidates_.cells[cost_idx_sorted_[i]].y;
        geometry_msgs::Point node_location = fromPolarToCartesian(e, z, tree_node_distance_, origin_position);

        tree_.push_back(TreeNode(origin_, depth, node_location));
        tree_.back().last_e = e;
        tree_.back().last_z = z;
        double h = treeHeuristicFunction(tree_.size() - 1);
        double c = treeCostFunction(tree_.size() - 1);
        tree_.back().heuristic = h;
        tree_.back().total_cost = tree_[origin_].total_cost - tree_[origin_].heuristic + c + h;
        double dx = node_location.x - origin_position.x;
        double dy = node_location.y - origin_position.y;
        tree_.back().yaw = atan2(dy, dx);

        if (c < min_c) {
          min_c = c;
          min_c_node = tree_.size() - 1;
        }
      }
    }
    closed_set_.push_back(origin_);
    n++;

    //find best node to continue
    double minimal_cost = INF;
    for (int i = 0; i < tree_.size(); i++) {
      bool closed = false;
      for (int j = 0; j < closed_set_.size(); j++) {
        if (closed_set_[j] == i) {
          closed = true;
        }
      }
      if (tree_[i].total_cost < minimal_cost && !closed) {
        minimal_cost = tree_[i].total_cost;
        origin_ = i;
      }
    }
  }
  //smoothing between trees
  int tree_end = origin_;
  path_node_positions_.clear();
  path_node_origins_.clear();
  while (tree_end > 0) {
    path_node_origins_.push_back(tree_end);
    path_node_positions_.push_back(tree_[tree_end].getPosition());
    tree_end = tree_[tree_end].origin;
  }
  path_node_positions_.push_back(tree_[0].getPosition());
  path_node_origins_.push_back(0);
  tree_age_ = 0;


ROS_INFO("Tree calculated in %2.2fms.",(std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
}

double LocalPlanner::treeCostFunction(int node_number) {
  int origin = tree_[node_number].origin;
  int e = tree_[node_number].last_e;
  int z = tree_[node_number].last_z;
  geometry_msgs::Point origin_position = tree_[origin].getPosition();
  int goal_z = floor(atan2(goal_.x - origin_position.x, goal_.y - origin_position.y) * 180.0 / PI);
  int goal_e = floor(atan((goal_.z - origin_position.z) / sqrt(pow((goal_.x - origin_position.x), 2) + pow((goal_.y - origin_position.y), 2))) * 180.0 / PI);

  double curr_yaw_z = std::round((-curr_yaw_ * 180.0 / PI + 270.0) / alpha_res) - 1;

  double target_cost = 5 * indexAngleDifference(z, goal_z) + 5 * indexAngleDifference(e, goal_e);  //include effective direction?
  double turning_cost = 3*indexAngleDifference(z, curr_yaw_z);  //maybe include pitching cost?

  int last_e = tree_[origin].last_e;
  int last_z = tree_[origin].last_z;

  double smooth_cost = 5 * indexAngleDifference(z, last_z) + 5 * indexAngleDifference(e, last_e);

  double smooth_cost_to_old_tree = 0;
  if(tree_age_<10){
    int partner_node_idx = path_node_positions_.size() - 1 - tree_[node_number].depth;
    if(partner_node_idx >= 0){
      geometry_msgs::Point partner_node_position = path_node_positions_[partner_node_idx];
      geometry_msgs::Point node_position = tree_[node_number].getPosition();
      double dist = distance3DCartesian(partner_node_position, node_position);
      smooth_cost_to_old_tree = 250*dist;
    }
  }

  return std::pow(tree_discount_factor_, tree_[node_number].depth) * (target_cost + smooth_cost + smooth_cost_to_old_tree + turning_cost);

}
double LocalPlanner::treeHeuristicFunction(int node_number) {
  geometry_msgs::Point node_position = tree_[node_number].getPosition();
  int goal_z = floor(atan2(goal_.x - node_position.x, goal_.y - node_position.y) * 180.0 / PI);  //azimuthal angle
  int goal_e = floor(atan((goal_.z - node_position.z) / sqrt(pow((goal_.x - node_position.x), 2) + pow((goal_.y - node_position.y), 2))) * 180.0 / PI);

  //  double turning_cost = 2*indexAngleDifference(z, curr_yaw_z);  //maybe include pitching cost?
  double smooth_cost = 1 * indexAngleDifference(goal_z, tree_[node_number].last_z) + 1 * indexAngleDifference(goal_e, tree_[node_number].last_e);

  return std::pow(tree_discount_factor_, tree_[node_number].depth) * (smooth_cost);
}

double LocalPlanner::indexAngleDifference(int a, int b) {
return std::min(std::min(std::abs(a - b), std::abs(a - b - 360)), std::abs(a - b + 360));
}

//calculate the correct weight between fly over and fly around
void LocalPlanner::evaluateProgressRate() {

  if (reach_altitude_ && adapt_cost_params_) {
    double goal_dist = distance3DCartesian(pose_.pose.position, goal_);
    double goal_dist_old = distance3DCartesian(position_old_, goal_);
    double time = std::clock() / (double) (CLOCKS_PER_SEC / 1000);
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

    if (avg_incline > no_progress_slope_ && goal_dist_incline_.size() == dist_incline_window_size_) {
      if (height_change_cost_param_adapted_ > 0.75) {
        height_change_cost_param_adapted_ -= 0.02;
      }
    }
    if (avg_incline < no_progress_slope_) {
      if (height_change_cost_param_adapted_ < height_change_cost_param_ - 0.03) {
        height_change_cost_param_adapted_ += 0.03;
      }
    }
    ROS_INFO("Progress rate to goal: %f, adapted height change cost: %f .", avg_incline, height_change_cost_param_adapted_);
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

// check that the selected direction is really free and transform it into a waypoint.
void LocalPlanner::getNextWaypoint() {

  int waypoint_index = path_waypoints_.cells.size();
  int e_angle = path_waypoints_.cells[waypoint_index - 1].x;
  int z_angle = path_waypoints_.cells[waypoint_index - 1].y;

  int e_index = elevationAngletoIndex(e_angle, ALPHA_RES);
  int z_index = azimuthAngletoIndex(z_angle, ALPHA_RES);

  geometry_msgs::Vector3Stamped setpoint = getWaypointFromAngle(e_angle, z_angle, pose_.pose.position);

  if (std::find(z_FOV_idx_.begin(), z_FOV_idx_.end(), z_index) != z_FOV_idx_.end()) {
    waypoint_outside_FOV_ = false;
  } else {
    waypoint_outside_FOV_ = true;
  }
  waypt_ = setpoint;

  ROS_INFO("Selected waypoint: [%f, %f, %f].", waypt_.vector.x, waypt_.vector.y, waypt_.vector.z);

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

  //Prevent downward motion or move up if too close to ground
  if (use_ground_detection_) {
    vec.normalize();
    min_flight_height_ = ground_detector_.getMinFlightHeight(pose_, curr_vel_, over_obstacle_, min_flight_height_, ground_margin_);
    ground_detector_.getFlags(over_obstacle_, too_low_, is_near_min_height_);
    ground_margin_ = ground_detector_.getMargin();

    if (over_obstacle_ && pose_.pose.position.z <= min_flight_height_ && waypt_.vector.z <= min_flight_height_) {
      if ((min_flight_height_ - pose_.pose.position.z) > 0.5) {
        waypt_.vector.z = pose_.pose.position.z + 0.5;
      } else {
        waypt_.vector.z = min_flight_height_;
      }
      too_low_ = true;
      std::cout << "\033[1;36m Go Fast: Flight altitude too low (Minimal flight height: " << min_flight_height_ << ") rising.\n \033[0m";
    }
    if (over_obstacle_ && pose_.pose.position.z > min_flight_height_ && pose_.pose.position.z < min_flight_height_ + 0.5 && vec.getZ() < 0) {
      waypt_.vector.z = pose_.pose.position.z;
      is_near_min_height_ = true;
      std::cout << "\033[1;36m Go Fast: Preventing downward motion (Minimal flight height: " << min_flight_height_ << ") \n \033[0m";
    }
  }

  // fill direction as straight ahead
  geometry_msgs::Point p;
  p.x = 0;
  p.y = 90;
  p.z = 0;
  path_waypoints_.cells.push_back(p);

  //reset candidates for visualization
  initGridCells(&path_candidates_);
  initGridCells(&path_rejected_);
  initGridCells(&path_blocked_);
  initGridCells(&path_selected_);
  initGridCells(&path_ground_);

  ROS_INFO("Go fast selected direction: [%f, %f, %f].", vec.getX(), vec.getY(), vec.getZ());
  ROS_INFO("Go fast selected waypoint: [%f, %f, %f].", waypt_.vector.x, waypt_.vector.y, waypt_.vector.z);

  getPathMsg();

}

void LocalPlanner::backOff() {

  tf::Vector3 vec;
  vec.setX(pose_.pose.position.x - back_off_point_.x);
  vec.setY(pose_.pose.position.y - back_off_point_.y);
  vec.setZ(0);
  vec.normalize();
  double new_len = speed_;
  vec *= new_len;

  waypt_.vector.x = pose_.pose.position.x + vec.getX();
  waypt_.vector.y = pose_.pose.position.y + vec.getY();
  waypt_.vector.z = pose_.pose.position.z + vec.getZ();

  // fill direction as straight ahead
  geometry_msgs::Point p; p.x = 0; p.y = 90; p.z = 0;
  path_waypoints_.cells.push_back(p);

  double dist = distance3DCartesian(pose_.pose.position, back_off_point_);
  if (dist > min_dist_backoff_ + 1.0) {
    back_off_ = false;
  }

  waypt_p_ = createPoseMsg(waypt_, last_yaw_);
  path_msg_.poses.push_back(waypt_p_);
  curr_yaw_ = last_yaw_;
  position_old_ = pose_.pose.position;

  std::cout << "Distance to Backoff Point: " << dist << "\n";
  ROS_INFO("Back off selected direction: [%f, %f, %f].", vec.getX(), vec.getY(), vec.getZ());
  ROS_INFO("Back off selected waypoint: [%f, %f, %f].", waypt_.vector.x, waypt_.vector.y, waypt_.vector.z);
}

// check if the UAV has reached the goal set for the mission
bool LocalPlanner::withinGoalRadius() {
  geometry_msgs::Point a;
  a.x = std::abs(goal_.x - pose_.pose.position.x);
  a.y = std::abs(goal_.y - pose_.pose.position.y);
  a.z = std::abs(goal_.z - pose_.pose.position.z);
  float goal_acceptance_radius = 0.5f;

  if (a.x < goal_acceptance_radius && a.y < goal_acceptance_radius && a.z < goal_acceptance_radius) {
    if (!reached_goal_) {
      yaw_reached_goal_ = tf::getYaw(pose_.pose.orientation);
    }
    reached_goal_ = true;
    return true;
  } else if (over_obstacle_ && a.x < goal_acceptance_radius && a.y < goal_acceptance_radius) {
    if (!reached_goal_) {
      yaw_reached_goal_ = tf::getYaw(pose_.pose.orientation);
    }
    reached_goal_ = true;
    return true;
  } else
    return false;
}

geometry_msgs::PoseStamped LocalPlanner::createPoseMsg(geometry_msgs::Vector3Stamped waypt, double yaw) {
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = "/world";
  pose_msg.pose.position.x = waypt.vector.x;
  pose_msg.pose.position.y = waypt.vector.y;
  pose_msg.pose.position.z = waypt.vector.z;
  pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  return pose_msg;
}

// calculate the yaw for the next waypoint
double LocalPlanner::nextYaw(geometry_msgs::PoseStamped u, geometry_msgs::Vector3Stamped v, double last_yaw_) {
  double dx = v.vector.x - u.pose.position.x;
  double dy = v.vector.y - u.pose.position.y;

  if (reached_goal_){
    return yaw_reached_goal_;
  }

  return atan2(dy, dx);
}

// when taking off, first publish waypoints to reach the goal altitude
void LocalPlanner::reachGoalAltitudeFirst(){
  starting_height_ = std::max(goal_.z -0.5, take_off_pose_.pose.position.z + 1.0);
  if (pose_.pose.position.z < starting_height_) {
      waypt_.vector.x = pose_.pose.position.x;
      waypt_.vector.y = pose_.pose.position.y;
      waypt_.vector.z = pose_.pose.position.z + 0.5;
  } else {
    reach_altitude_ = true;
    printf("Reached altitude %f, now going towards the goal_. \n\n", pose_.pose.position.z);
    getPathMsg();
  }
}

// smooth trajectory by liming the maximim accelleration possible
geometry_msgs::Vector3Stamped LocalPlanner::smoothWaypoint(geometry_msgs::Vector3Stamped wp) {
  geometry_msgs::Vector3Stamped smooth_waypt;
  std::clock_t t = std::clock();
  float dt = (t - t_prev_) / (float) (CLOCKS_PER_SEC);
  dt = dt > 0.0f ? dt : 0.004f;
  t_prev_ = t;
  Eigen::Vector2f vel_xy(curr_vel_.twist.linear.x, curr_vel_.twist.linear.y);
  Eigen::Vector2f vel_waypt_xy((wp.vector.x - last_waypt_p_.pose.position.x) / dt, (wp.vector.y - last_waypt_p_.pose.position.y) / dt);
  Eigen::Vector2f vel_waypt_xy_prev((last_waypt_p_.pose.position.x - last_last_waypt_p_.pose.position.x) / dt, (last_waypt_p_.pose.position.y - last_last_waypt_p_.pose.position.y) / dt);
  Eigen::Vector2f acc_waypt_xy((vel_waypt_xy - vel_waypt_xy_prev) / dt);

  if (acc_waypt_xy.norm() > (acc_waypt_xy.norm() / 2.0f)) {
    vel_xy = (acc_waypt_xy.norm() / 2.0f) * acc_waypt_xy.normalized() * dt + vel_waypt_xy_prev;
  }

  float vel_waypt_z = (wp.vector.z - last_waypt_p_.pose.position.z) / dt;
  float max_acc_z, vel_z;
  float vel_waypt_z_prev = (last_waypt_p_.pose.position.z - last_last_waypt_p_.pose.position.z) / dt;
  float acc_waypt_z = (vel_waypt_z - vel_waypt_z_prev) / dt;

  max_acc_z = (acc_waypt_z < 0.0f) ? -(max_accel_z_) : (max_accel_z_);
  if (fabsf(acc_waypt_z) > fabsf(max_acc_z)) {
    vel_z = max_acc_z * dt + vel_waypt_z_prev;
  }

  smooth_waypt.vector.x = last_waypt_p_.pose.position.x + vel_xy(0) * dt;
  smooth_waypt.vector.y = last_waypt_p_.pose.position.y + vel_xy(1) * dt;
  smooth_waypt.vector.z = wp.vector.z;

  ROS_INFO("Smoothed waypoint: [%f %f %f].", smooth_waypt.vector.x, smooth_waypt.vector.y, smooth_waypt.vector.z);
  return smooth_waypt;
}

// create the message that is sent to the UAV
void LocalPlanner::getPathMsg() {
  path_msg_.header.frame_id = "/world";
  last_last_waypt_p_ = last_waypt_p_;
  last_waypt_p_ = waypt_p_;
  last_yaw_ = curr_yaw_;
  waypt_adapted_ = waypt_;

  //If avoid sphere is used, project waypoint on sphere
  if (use_avoid_sphere_ && avoid_sphere_age_ < 100 && reach_altitude_ && !reached_goal_ && !back_off_) {
    double sphere_hysteresis_radius = 1.3 * avoid_radius_;

    double dist = sqrt(
        (waypt_.vector.x - avoid_centerpoint_.x) * (waypt_.vector.x - avoid_centerpoint_.x) + (waypt_.vector.y - avoid_centerpoint_.y) * (waypt_.vector.y - avoid_centerpoint_.y)
            + (waypt_.vector.z - avoid_centerpoint_.z) * (waypt_.vector.z - avoid_centerpoint_.z));
    if (dist < sphere_hysteresis_radius) {
      //put waypoint closer to equator
      if (waypt_.vector.z < avoid_centerpoint_.z) {
        waypt_adapted_.vector.z = waypt_.vector.z + 0.25 * std::abs(waypt_.vector.z - avoid_centerpoint_.z);
      } else {
        waypt_adapted_.vector.z = waypt_.vector.z - 0.25 * std::abs(waypt_.vector.z - avoid_centerpoint_.z);
      }
      //increase angle from pole
      Eigen::Vector3f center_to_wp(waypt_adapted_.vector.x - avoid_centerpoint_.x, waypt_adapted_.vector.y - avoid_centerpoint_.y, waypt_adapted_.vector.z - avoid_centerpoint_.z);
      Eigen::Vector2f center_to_wp_2D(waypt_adapted_.vector.x - avoid_centerpoint_.x, waypt_adapted_.vector.y - avoid_centerpoint_.y);
      Eigen::Vector2f pose_to_center_2D(pose_.pose.position.x - avoid_centerpoint_.x, pose_.pose.position.y - avoid_centerpoint_.y);
      center_to_wp_2D = center_to_wp_2D.normalized();
      pose_to_center_2D = pose_to_center_2D.normalized();
      double cos_theta = center_to_wp_2D[0] * pose_to_center_2D[0] + center_to_wp_2D[1] * pose_to_center_2D[1];
      Eigen::Vector2f n(center_to_wp_2D[0] - cos_theta * pose_to_center_2D[0], center_to_wp_2D[1] - cos_theta * pose_to_center_2D[1]);
      n = n.normalized();
      double cos_new_theta = 0.9 * cos_theta;
      double sin_new_theta = sin(acos(cos_new_theta));
      Eigen::Vector2f center_to_wp_2D_new(cos_new_theta * pose_to_center_2D[0] + sin_new_theta * n[0], cos_new_theta * pose_to_center_2D[1] + sin_new_theta * n[1]);
      center_to_wp = center_to_wp.normalized();
      Eigen::Vector3f center_to_wp_new(center_to_wp_2D_new[0], center_to_wp_2D_new[1], center_to_wp[2]);

      //project on sphere
      center_to_wp_new = center_to_wp_new.normalized();

      //hysteresis
      if (dist < avoid_radius_) {
        center_to_wp_new *= avoid_radius_;
        waypt_adapted_.vector.x = avoid_centerpoint_.x + center_to_wp_new[0];
        waypt_adapted_.vector.y = avoid_centerpoint_.y + center_to_wp_new[1];
        waypt_adapted_.vector.z = avoid_centerpoint_.z + center_to_wp_new[2];
        std::cout << "\033[1;36m Inside sphere \n \033[0m";
      } else {
        center_to_wp_new *= dist;
        double radius_percentage = (dist - avoid_radius_) / (sphere_hysteresis_radius - avoid_radius_);  //1 at hysteresis rad, 0 at avoid rad
        waypt_adapted_.vector.x = (1.0 - radius_percentage) * (avoid_centerpoint_.x + center_to_wp_new[0]) + radius_percentage * waypt_adapted_.vector.x;
        waypt_adapted_.vector.y = (1.0 - radius_percentage) * (avoid_centerpoint_.y + center_to_wp_new[1]) + radius_percentage * waypt_adapted_.vector.y;
        waypt_adapted_.vector.z = (1.0 - radius_percentage) * (avoid_centerpoint_.z + center_to_wp_new[2]) + radius_percentage * waypt_adapted_.vector.z;
        std::cout << "\033[1;36m Inside sphere hysteresis \n \033[0m";
      }
    }

    //check if new point lies in FOV
    int e_angle = elevationAnglefromCartesian(waypt_adapted_.vector.x, waypt_adapted_.vector.y, waypt_adapted_.vector.z, pose_.pose.position);
    int z_angle = azimuthAnglefromCartesian(waypt_adapted_.vector.x, waypt_adapted_.vector.y, waypt_adapted_.vector.z, pose_.pose.position);

    int e_index = elevationAngletoIndex(e_angle, alpha_res);
    int z_index = azimuthAngletoIndex(z_angle, alpha_res);

    if (std::find(z_FOV_idx_.begin(), z_FOV_idx_.end(), z_index) != z_FOV_idx_.end()) {
      waypoint_outside_FOV_ = false;
    } else {
      waypoint_outside_FOV_ = true;
    }
  }

  double new_yaw = nextYaw(pose_, waypt_adapted_, last_yaw_);
  only_yawed_ = false;

  //If the waypoint is not inside the FOV, only yaw and not move
  if (waypoint_outside_FOV_ && reach_altitude_ && !reached_goal_ && obstacle_ && !back_off_) {
    waypt_adapted_.vector.x = pose_.pose.position.x;
    waypt_adapted_.vector.y = pose_.pose.position.y;
    waypt_adapted_.vector.z = pose_.pose.position.z;
    only_yawed_ = true;
  } else {                    //set waypoint to correct speed
    tf::Vector3 pose_to_wp;
    pose_to_wp.setX(waypt_adapted_.vector.x - pose_.pose.position.x);
    pose_to_wp.setY(waypt_adapted_.vector.y - pose_.pose.position.y);
    pose_to_wp.setZ(waypt_adapted_.vector.z - pose_.pose.position.z);
    pose_to_wp.normalize();
    pose_to_wp *= speed_;

    waypt_adapted_.vector.x = pose_.pose.position.x + pose_to_wp.getX();
    waypt_adapted_.vector.y = pose_.pose.position.y + pose_to_wp.getY();
    waypt_adapted_.vector.z = pose_.pose.position.z + pose_to_wp.getZ();
  }

  waypt_smoothed_ = waypt_adapted_;

  if (!reach_altitude_) {
    reachGoalAltitudeFirst();
    waypt_adapted_ = waypt_;
    waypt_smoothed_ = waypt_;
  } else {
    if (!only_yawed_) {
      if (!reached_goal_ && !stop_in_front_) {
        if (smooth_go_fast_ < 1 && local_planner_mode_ == 1) {
          waypt_smoothed_.vector.x = smooth_go_fast_ * waypt_adapted_.vector.x + (1.0 - smooth_go_fast_) * last_hist_waypt_.vector.x;
          waypt_smoothed_.vector.y = smooth_go_fast_ * waypt_adapted_.vector.y + (1.0 - smooth_go_fast_) * last_hist_waypt_.vector.y;
          waypt_smoothed_.vector.z = smooth_go_fast_ * waypt_adapted_.vector.z + (1.0 - smooth_go_fast_) * last_hist_waypt_.vector.z;
        }
        waypt_ = smoothWaypoint();
        new_yaw = nextYaw(pose_, waypt_, last_yaw_);
      }
    }
  }


  if(back_off_ && use_back_off_){
    new_yaw = last_yaw_;
    double dist = distance3DCartesian(pose_.pose.position, back_off_point_);
    if (dist > min_dist_backoff_ + 1.0) {
      back_off_ = false;
    }
    waypt_smoothed_ = waypt_;
  }


  if (withinGoalRadius()) {
    bool over_goal = false;
    if (use_ground_detection_) {
      if (over_obstacle_ && (is_near_min_height_ || too_low_)) {
        over_goal = true;
        waypt_smoothed_.vector.x = goal_.x;
        waypt_smoothed_.vector.y = goal_.y;
        if (pose_.pose.position.z < goal_.z) {
          waypt_smoothed_.vector.z = goal_.z;
          ROS_INFO("Rising to goal");
        } else {
          waypt_smoothed_.vector.z = min_flight_height_;
          ROS_INFO("Above Goal cannot go lower: Hoovering");
        }
      }
    }
    if (!over_goal) {
      ROS_INFO("Goal Reached: Hoovering");
      waypt_smoothed_.vector.x = goal_.x;
      waypt_smoothed_.vector.y = goal_.y;
      waypt_smoothed_.vector.z = goal_.z;
    }
  }

  ROS_INFO("Final waypoint: [%f %f %f].", waypt_smoothed_.vector.x, waypt_smoothed_.vector.y, waypt_smoothed_.vector.z);
  waypt_p_ = createPoseMsg(waypt_smoothed_, new_yaw);

  path_msg_.poses.push_back(waypt_p_);
  curr_yaw_ = new_yaw;
  last_last_waypt_p_ = last_waypt_p_;
  last_waypt_p_ = waypt_p_;
  last_yaw_ = curr_yaw_;
  position_old_ = pose_.pose.position;
  checkSpeed();
}

void LocalPlanner::useHoverPoint() {


}


void LocalPlanner::printAlgorithmStatistics() {
  ROS_INFO("Current pose: [%f, %f, %f].", pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z);
  ROS_INFO("Velocity: [%f, %f, %f], module: %f.", velocity_x_, velocity_y_, velocity_z_, velocity_mod_);

  logData();

  if (withinGoalRadius()) {
    cv::Scalar mean, std;
    printf("----------------------------------- \n");
    cv::meanStdDev(algorithm_total_time_, mean, std);
    printf("total mean %f std %f \n", mean[0], std[0]);
    cv::meanStdDev(cloud_time_, mean, std);
    printf("cloud mean %f std %f \n", mean[0], std[0]);
    cv::meanStdDev(polar_time_, mean, std);
    printf("polar mean %f std %f \n", mean[0], std[0]);
    cv::meanStdDev(free_time_, mean, std);
    printf("free mean %f std %f \n", mean[0], std[0]);
    cv::meanStdDev(cost_time_, mean, std);
    printf("cost mean %f std %f \n", mean[0], std[0]);
    printf("----------------------------------- \n");
  }
}

void LocalPlanner::checkSpeed() {
  if (hasSameYawAndAltitude(last_waypt_p_, waypt_p_) && !obstacle_) {
    speed_ = std::min(max_speed_, speed_ + 0.1);
  } else {
    speed_ = min_speed_;
  }
}

// stop in front of an obstacle at a distance defined by the variable keep_distance_
void LocalPlanner::stopInFrontObstacles() {

  if (first_brake_) {
    double braking_distance = fabsf(distance_to_closest_point_ - keep_distance_);
    Eigen::Vector2f pose_to_goal(goal_.x - pose_.pose.position.x, goal_.y - pose_.pose.position.y);
    goal_.x = pose_.pose.position.x + (braking_distance * pose_to_goal(0) / pose_to_goal.norm());
    goal_.y = pose_.pose.position.y + (braking_distance * pose_to_goal(1) / pose_to_goal.norm());
    first_brake_ = false;
  }
  ROS_INFO("New Stop Goal: [%.2f %.2f %.2f], obstacle distance %.2f. ", goal_.x, goal_.y, goal_.z, distance_to_closest_point_);
  goFast();
}

void LocalPlanner::getPosition(geometry_msgs::PoseStamped &pos) {
  pos = pose_;
}

void LocalPlanner::getGoalPosition(geometry_msgs::Point &goal) {
  goal = goal_;
}
void LocalPlanner::getAvoidSphere(geometry_msgs::Point &center, double &radius, int &sphere_age, bool &use_avoid_sphere) {
  center = avoid_centerpoint_;
  radius = avoid_radius_;
  sphere_age = avoid_sphere_age_;
  use_avoid_sphere = use_avoid_sphere_;
}

void LocalPlanner::getCloudsForVisualization(pcl::PointCloud<pcl::PointXYZ> &final_cloud, pcl::PointCloud<pcl::PointXYZ> &ground_cloud, pcl::PointCloud<pcl::PointXYZ> &reprojected_points) {
  final_cloud = final_cloud_;
  reprojected_points = reprojected_points_;
  if (use_ground_detection_) {
    ground_detector_.getGroundCloudForVisualization(ground_cloud);
  } else {
    pcl::PointCloud < pcl::PointXYZ > ground_cloud;
  }
}

void LocalPlanner::getCandidateDataForVisualization(nav_msgs::GridCells &path_candidates, nav_msgs::GridCells &path_selected, nav_msgs::GridCells &path_rejected, nav_msgs::GridCells &path_blocked, nav_msgs::GridCells &FOV_cells,
                                                    nav_msgs::GridCells &path_ground) {
  path_candidates = path_candidates_;
  path_selected = path_selected_;
  path_rejected = path_rejected_;
  path_blocked = path_blocked_;
  path_ground = path_ground_;
  FOV_cells = FOV_cells_;
}

void LocalPlanner::getPathData(nav_msgs::Path &path_msg, geometry_msgs::PoseStamped &waypt_p) {
  path_msg = path_msg_;
  waypt_p = waypt_p_;
}

void LocalPlanner::setCurrentVelocity(geometry_msgs::TwistStamped vel) {
  curr_vel_ = vel;
}

void LocalPlanner::getTree(std::vector<TreeNode> &tree, std::vector<int> &closed_set, std::vector<geometry_msgs::Point> &path_node_positions) {
  tree = star_planner_.tree_;
  closed_set = star_planner_.closed_set_;
  path_node_positions = star_planner_.path_node_positions_;
}

void LocalPlanner::getWaypoints(geometry_msgs::Vector3Stamped &waypt, geometry_msgs::Vector3Stamped &waypt_adapted, geometry_msgs::Vector3Stamped &waypt_smoothed) {
  waypt = waypt_;
  waypt_adapted = waypt_adapted_;
  waypt_smoothed = waypt_smoothed_;
}
