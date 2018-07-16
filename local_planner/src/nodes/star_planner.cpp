#include "star_planner.h"

StarPlanner::StarPlanner() {}

StarPlanner::~StarPlanner() {}

// set parameters changed by dynamic rconfigure
void StarPlanner::dynamicReconfigureSetStarParams(
    avoidance::LocalPlannerNodeConfig& config, uint32_t level) {
  childs_per_node_ = config.childs_per_node_;
  n_expanded_nodes_ = config.n_expanded_nodes_;
  tree_node_distance_ = config.tree_node_distance_;
  tree_discount_factor_ = config.tree_discount_factor_;
}

void StarPlanner::setParams(const double& min_cloud_size, const double& min_dist_backoff,
                            const nav_msgs::GridCells& path_waypoints, const double& curr_yaw,
                            bool use_ground_detection, const double& min_realsense_dist) {

  path_waypoints_ = path_waypoints;
  curr_yaw_ = curr_yaw;
  use_ground_detection_ = use_ground_detection;
  min_cloud_size_ = min_cloud_size;
  min_dist_backoff_ = min_dist_backoff;
  min_realsense_dist_ = min_realsense_dist;
}

void StarPlanner::setPose(const geometry_msgs::PoseStamped& pose) { pose_ = pose; }

void StarPlanner::setCloud(const pcl::PointCloud<pcl::PointXYZ>& complete_cloud) {
  complete_cloud_ = complete_cloud;
}

void StarPlanner::setBoxSize(const Box& histogram_box_size) {
  histogram_box_size_ = histogram_box_size;
}

void StarPlanner::setGoal(const geometry_msgs::Point& goal) {
  goal_ = goal;
  tree_age_ = 1000;
}

void StarPlanner::setCostParams(const double& goal_cost_param,
                                const double& smooth_cost_param,
                                const double& height_change_cost_param_adapted,
                                const double& height_change_cost_param) {
  goal_cost_param_ = goal_cost_param;
  smooth_cost_param_ = smooth_cost_param;
  height_change_cost_param_adapted_ = height_change_cost_param_adapted;
  height_change_cost_param_ = height_change_cost_param;
}

void StarPlanner::setReprojectedPoints(
    const pcl::PointCloud<pcl::PointXYZ>& reprojected_points,
    const std::vector<double>& reprojected_points_age,
    const std::vector<double>& reprojected_points_dist) {
  reprojected_points_ = reprojected_points;
  reprojected_points_age_ = reprojected_points_age;
  reprojected_points_dist_ = reprojected_points_dist;
}

double StarPlanner::treeCostFunction(int node_number) {
  int origin = tree_[node_number].origin_;
  int e = tree_[node_number].last_e_;
  int z = tree_[node_number].last_z_;
  geometry_msgs::Point origin_position = tree_[origin].getPosition();
  int goal_z =
      azimuthAnglefromCartesian(goal_.x, goal_.y, goal_.z, origin_position);
  int goal_e =
      elevationAnglefromCartesian(goal_.x, goal_.y, goal_.z, origin_position);

  double target_cost =
      2 * indexAngleDifference(z, goal_z) +
      50 * indexAngleDifference(e, goal_e);  // include effective direction?
  double turning_cost =
      1 *
      indexAngleDifference(z, tree_[0].yaw_);  // maybe include pitching cost?

  int last_e = tree_[origin].last_e_;
  int last_z = tree_[origin].last_z_;

  double smooth_cost =
      2 * indexAngleDifference(z, last_z) + 5 * indexAngleDifference(e, last_e);

  double smooth_cost_to_old_tree = 0.0;
  if (tree_age_ < 10) {
    int partner_node_idx =
        path_node_positions_.size() - 1 - tree_[node_number].depth_;
    if (partner_node_idx >= 0) {
      geometry_msgs::Point partner_node_position =
          path_node_positions_[partner_node_idx];
      geometry_msgs::Point node_position = tree_[node_number].getPosition();
      double dist = distance3DCartesian(partner_node_position, node_position);
      smooth_cost_to_old_tree = 200 * dist / (0.5 * tree_[node_number].depth_);
    }
  }

  return std::pow(tree_discount_factor_, tree_[node_number].depth_) *
         (target_cost + smooth_cost + smooth_cost_to_old_tree + turning_cost);
}
double StarPlanner::treeHeuristicFunction(int node_number) {
  geometry_msgs::Point node_position = tree_[node_number].getPosition();
  int goal_z =
      azimuthAnglefromCartesian(goal_.x, goal_.y, goal_.z, node_position);
  int goal_e =
      elevationAnglefromCartesian(goal_.x, goal_.y, goal_.z, node_position);

  int origin = tree_[node_number].origin_;
  geometry_msgs::Point origin_position = tree_[origin].getPosition();
  double origin_goal_dist = distance3DCartesian(goal_, origin_position);
  double goal_dist = distance3DCartesian(goal_, node_position);
  double goal_cost = (goal_dist / origin_goal_dist - 0.9) * 5000;

  //  double turning_cost = 2*indexAngleDifference(z, curr_yaw_z);
  double smooth_cost =
      10 * (1 * indexAngleDifference(goal_z, tree_[node_number].last_z_) +
            1 * indexAngleDifference(goal_e, tree_[node_number].last_e_));

  return std::pow(tree_discount_factor_, tree_[node_number].depth_) *
         (smooth_cost + goal_cost);
}

void StarPlanner::buildLookAheadTree() {
  nav_msgs::GridCells path_candidates;
  nav_msgs::GridCells path_selected;
  nav_msgs::GridCells path_rejected;
  nav_msgs::GridCells path_blocked;
  nav_msgs::GridCells path_ground;
  std::vector<float> cost_path_candidates;
  std::vector<int> cost_idx_sorted;
  std::clock_t start_time = std::clock();
  tree_.clear();
  closed_set_.clear();

  // insert first node
  tree_.push_back(TreeNode(0, 0, pose_.pose.position));
  tree_.back().setCosts(treeHeuristicFunction(0), treeHeuristicFunction(0));
  tree_.back().yaw_ = std::round((-curr_yaw_ * 180.0 / M_PI)) +
                      90;  // from radian to angle and shift reference to y-axis
  tree_.back().last_z_ = tree_.back().yaw_;

  int origin = 0;
  int n = 0;

  while (n < n_expanded_nodes_) {
    geometry_msgs::Point origin_position = tree_[origin].getPosition();
    int old_origin = tree_[origin].origin_;
    geometry_msgs::Point origin_origin_position =
        tree_[old_origin].getPosition();

    bool node_valid = true;

    // crop pointcloud
    pcl::PointCloud<pcl::PointXYZ> cropped_cloud;
    geometry_msgs::Point temp_sphere_center;  // unused
    int sphere_points_counter = 0;            // unused
    geometry_msgs::Point closest_point;       // unused
    double avoid_radius = 0.0;                // unused
    bool hist_is_empty = false;               // unused
    int backoff_points_counter = 0;
    double distance_to_closest_point;
    histogram_box_.setLimitsHistogramBox(origin_position, histogram_box_size_);

    filterPointCloud(cropped_cloud, closest_point, temp_sphere_center,
                     distance_to_closest_point, backoff_points_counter,
                     sphere_points_counter, complete_cloud_, min_cloud_size_,
                     min_dist_backoff_, avoid_radius, histogram_box_,
                     origin_position, min_realsense_dist_);
    double safety_radius = adaptSafetyMarginHistogram(
        distance_to_closest_point, cropped_cloud.points.size(),
        min_cloud_size_);

    if (origin != 0 && backoff_points_counter > 20 &&
        cropped_cloud.points.size() > 160) {
      tree_[origin].total_cost_ = HUGE_VAL;
      node_valid = false;
    }

    if (node_valid) {
      // build new histogram
      std::vector<int> z_FOV_idx;
      int e_FOV_min, e_FOV_max;
      calculateFOV(z_FOV_idx, e_FOV_min, e_FOV_max, tree_[origin].yaw_,
                   0.0);  // assume pitch is zero at every node

      Histogram propagated_histogram = Histogram(2 * ALPHA_RES);
      Histogram histogram = Histogram(ALPHA_RES);

      propagateHistogram(propagated_histogram, reprojected_points_,
                         reprojected_points_age_, reprojected_points_dist_,
                         pose_);
      generateNewHistogram(histogram, cropped_cloud, pose_);
      combinedHistogram(hist_is_empty, histogram, propagated_histogram, false,
                        z_FOV_idx, e_FOV_min, e_FOV_max);

      // calculate candidates
      int e_min_idx = -1;
      if (use_ground_detection_) {
        // calculate velocity direction
        geometry_msgs::TwistStamped velocity;
        velocity.twist.linear.x = origin_position.x - origin_origin_position.x;
        velocity.twist.linear.y = origin_position.y - origin_origin_position.y;
        velocity.twist.linear.z = origin_position.z - origin_origin_position.z;

        min_flight_height_ = ground_detector_.getMinFlightHeight(
            pose_, velocity, over_obstacle_, min_flight_height_,
            ground_margin_);
        e_min_idx = ground_detector_.getMinFlightElevationIndex(
            pose_, min_flight_height_, 2 * ALPHA_RES);
        ground_detector_.getHeightInformation(over_obstacle_, too_low_,
                                              is_near_min_height_);
        ground_margin_ = ground_detector_.getMargin();
      }

      histogram.downsample();
      findFreeDirections(histogram, 25, path_candidates, path_selected,
                         path_rejected, path_blocked, path_ground,
                         path_waypoints_, cost_path_candidates, goal_, pose_,
                         origin_origin_position, goal_cost_param_,
                         smooth_cost_param_, height_change_cost_param_adapted_,
                         height_change_cost_param_, e_min_idx, over_obstacle_,
                         false, 2 * ALPHA_RES);

      if (calculateCostMap(cost_path_candidates, cost_idx_sorted)) {
        tree_[origin].total_cost_ = HUGE_VAL;
      } else {
        // insert new nodes
        int depth = tree_[origin].depth_ + 1;

        int goal_z = floor(
            atan2(goal_.x - origin_position.x, goal_.y - origin_position.y) *
            180.0 / M_PI);  // azimuthal angle
        int goal_e = floor(atan((goal_.z - origin_position.z) /
                                sqrt(pow((goal_.x - origin_position.x), 2) +
                                     pow((goal_.y - origin_position.y), 2))) *
                           180.0 / M_PI);
        int goal_e_idx = (goal_e - ALPHA_RES + 90) / ALPHA_RES;
        int goal_z_idx = (goal_z - ALPHA_RES + 180) / ALPHA_RES;

        int childs = 0;
        for (int i = 0; i < (int)path_candidates.cells.size(); i++) {
          int e = path_candidates.cells[cost_idx_sorted[i]].x;
          int z = path_candidates.cells[cost_idx_sorted[i]].y;

          // check if another close node has been added
          geometry_msgs::Point node_location =
              fromPolarToCartesian(e, z, tree_node_distance_, origin_position);
          int close_nodes = 0;
          for (int i = 0; i < tree_.size(); i++) {
            double dist =
                distance3DCartesian(tree_[i].getPosition(), node_location);
            if (dist < 0.2) {
              close_nodes++;
            }
          }

          if (childs < childs_per_node_ && close_nodes == 0) {
            tree_.push_back(TreeNode(origin, depth, node_location));
            tree_.back().last_e_ = e;
            tree_.back().last_z_ = z;
            double h = treeHeuristicFunction(tree_.size() - 1);
            double c = treeCostFunction(tree_.size() - 1);
            tree_.back().heuristic_ = h;
            tree_.back().total_cost_ =
                tree_[origin].total_cost_ - tree_[origin].heuristic_ + c + h;
            double dx = node_location.x - origin_position.x;
            double dy = node_location.y - origin_position.y;
            tree_.back().yaw_ = atan2(dy, dx);
            childs++;
          }
        }
      }
    }

    closed_set_.push_back(origin);
    n++;

    // find best node to continue
    double minimal_cost = HUGE_VAL;
    for (int i = 0; i < tree_.size(); i++) {
      bool closed = false;
      for (int j = 0; j < closed_set_.size(); j++) {
        if (closed_set_[j] == i) {
          closed = true;
        }
      }
      if (tree_[i].total_cost_ < minimal_cost && !closed) {
        minimal_cost = tree_[i].total_cost_;
        origin = i;
      }
    }
  }
  // smoothing between trees
  int tree_end = origin;
  path_node_positions_.clear();
  path_node_origins_.clear();
  while (tree_end > 0) {
    path_node_origins_.push_back(tree_end);
    path_node_positions_.push_back(tree_[tree_end].getPosition());
    tree_end = tree_[tree_end].origin_;
  }
  path_node_positions_.push_back(tree_[0].getPosition());
  path_node_origins_.push_back(0);
  tree_age_ = 0;

  ROS_INFO("\033[0;35m[SP]Tree calculated in %2.2fms.\033[0m",
           (std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
}
