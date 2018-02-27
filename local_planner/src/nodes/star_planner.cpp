#include "star_planner.h"

StarPlanner::StarPlanner() {
}

StarPlanner::~StarPlanner() {
}


// set parameters changed by dynamic rconfigure
void StarPlanner::dynamicReconfigureSetStarParams(avoidance::LocalPlannerNodeConfig & config,uint32_t level){
  childs_per_node_ = config.childs_per_node_;
  n_expanded_nodes_ = config.n_expanded_nodes_;
  tree_node_distance_ = config.tree_node_distance_;
  tree_discount_factor_ = config.tree_discount_factor_;
}

void StarPlanner::setParams(nav_msgs::GridCells path_waypoints, double curr_yaw, bool use_ground_detection){
  path_waypoints_ = path_waypoints;
  curr_yaw_ = curr_yaw;
  use_ground_detection_ = use_ground_detection;
}

void StarPlanner::setPose(geometry_msgs::PoseStamped  pose){
  pose_ = pose;
}

void StarPlanner::setCloud(pcl::PointCloud<pcl::PointXYZ> cropped_cloud){
  cropped_cloud_ = cropped_cloud;
}

void StarPlanner::setGoal(geometry_msgs::Point goal){
  goal_ = goal;
  tree_new_ = false;
  tree_age_ = 1000;
}

void StarPlanner::setCostParams(double goal_cost_param, double smooth_cost_param, double height_change_cost_param_adapted, double height_change_cost_param){
  goal_cost_param_ = goal_cost_param;
  smooth_cost_param_ = smooth_cost_param;
  height_change_cost_param_adapted_ = height_change_cost_param_adapted;
  height_change_cost_param_ = height_change_cost_param;
}

void StarPlanner::setReprojectedPoints(pcl::PointCloud<pcl::PointXYZ> reprojected_points, std::vector<double> reprojected_points_age, std::vector<double> reprojected_points_dist){
  reprojected_points_ = reprojected_points;
  reprojected_points_age_ = reprojected_points_age;
  reprojected_points_dist_ = reprojected_points_dist;
}

double StarPlanner::treeCostFunction(int node_number) {
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
double StarPlanner::treeHeuristicFunction(int node_number) {
  geometry_msgs::Point node_position = tree_[node_number].getPosition();
  int goal_z = floor(atan2(goal_.x - node_position.x, goal_.y - node_position.y) * 180.0 / PI);  //azimuthal angle
  int goal_e = floor(atan((goal_.z - node_position.z) / sqrt(pow((goal_.x - node_position.x), 2) + pow((goal_.y - node_position.y), 2))) * 180.0 / PI);

  //  double turning_cost = 2*indexAngleDifference(z, curr_yaw_z);  //maybe include pitching cost?
  double smooth_cost = 1 * indexAngleDifference(goal_z, tree_[node_number].last_z) + 1 * indexAngleDifference(goal_e, tree_[node_number].last_e);

  return std::pow(tree_discount_factor_, tree_[node_number].depth) * (smooth_cost);
}


void StarPlanner::buildLookAheadTree(double origin_yaw){

  tree_available_ = true;
  tree_new_ = true;

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

  //insert first node
  tree_.push_back(TreeNode(0, 0, pose_.pose.position));
  tree_.back().setCosts(treeHeuristicFunction(0), treeHeuristicFunction(0));
  tree_.back().yaw = origin_yaw;

  int origin = 0;
  double min_c = inf;
  int min_c_node = 0;
  int n = 0;

  while (n < n_expanded_nodes_) {
    geometry_msgs::Point origin_position = tree_[origin].getPosition();
    int old_origin = tree_[origin].origin;
    geometry_msgs::Point origin_origin_position = tree_[old_origin].getPosition();
    bool hist_is_empty = false; //unused

     //build new histogram
    std::vector<int> z_FOV_idx;
    int e_FOV_min, e_FOV_max;
    calculateFOV(z_FOV_idx, e_FOV_min, e_FOV_max, tree_[origin].yaw, 0.0);  //assume pitch is zero at every node

    Histogram propagated_histogram = Histogram(2 * alpha_res);
    Histogram histogram = Histogram(alpha_res);

    propagateHistogram(propagated_histogram, reprojected_points_, reprojected_points_age_, reprojected_points_dist_, pose_);
    generateNewHistogram(histogram, cropped_cloud_, pose_);
    combinedHistogram(hist_is_empty, histogram, propagated_histogram, false, z_FOV_idx, e_FOV_min, e_FOV_max);

    //calculate candidates
    int e_min_idx = -1;
    if (use_ground_detection_) {
      //calculate velocity direction
      geometry_msgs::TwistStamped velocity;
      velocity.twist.linear.x = origin_position.x - origin_origin_position.x;
      velocity.twist.linear.y = origin_position.y - origin_origin_position.y;
      velocity.twist.linear.z = origin_position.z - origin_origin_position.z;

      min_flight_height_ = ground_detector_.getMinFlightHeight(pose_, velocity, over_obstacle_, min_flight_height_, ground_margin_);
      e_min_idx = ground_detector_.getMinFlightElevationIndex(pose_, min_flight_height_, 2 * alpha_res);
      ground_detector_.getFlags(over_obstacle_, too_low_, is_near_min_height_);
      ground_margin_ = ground_detector_.getMargin();
    }

    histogram.downsample();
    findFreeDirections(histogram, 25, path_candidates, path_selected, path_rejected, path_blocked, path_ground, path_waypoints_, cost_path_candidates, goal_, pose_, origin_origin_position, goal_cost_param_, smooth_cost_param_,
                       height_change_cost_param_adapted_, height_change_cost_param_, e_min_idx, over_obstacle_, false, 2 * alpha_res);

    if (calculateCostMap(cost_path_candidates, cost_idx_sorted)) {
      tree_[origin].total_cost = inf;
    } else {
      //insert new nodes
      int depth = tree_[origin].depth + 1;

      int goal_z = floor(atan2(goal_.x - origin_position.x, goal_.y - origin_position.y) * 180.0 / PI);//azimuthal angle
      int goal_e = floor(atan((goal_.z - origin_position.z) / sqrt(pow((goal_.x - origin_position.x), 2) + pow((goal_.y - origin_position.y), 2))) * 180.0 / PI);
      int goal_e_idx = (goal_e - alpha_res + 90) / alpha_res;
      int goal_z_idx = (goal_z - alpha_res + 180) / alpha_res;

      int childs = 0;
      for (int i = 0; i < (int)path_candidates.cells.size(); i++) {
        int e = path_candidates.cells[cost_idx_sorted[i]].x;
        int z = path_candidates.cells[cost_idx_sorted[i]].y;

        geometry_msgs::Point node_location = fromPolarToCartesian(e, z, tree_node_distance_, origin_position);
        double dist_to_obstacle = distance3DCartesian(node_location, obstacle_position_);

        if(dist_to_obstacle > 2.0 && childs < childs_per_node_) {
          tree_.push_back(TreeNode(origin, depth, node_location));
          tree_.back().last_e = e;
          tree_.back().last_z = z;
          double h = treeHeuristicFunction(tree_.size() - 1);
          double c = treeCostFunction(tree_.size() - 1);
          tree_.back().heuristic = h;
          tree_.back().total_cost = tree_[origin].total_cost - tree_[origin].heuristic + c + h;
          double dx = node_location.x - origin_position.x;
          double dy = node_location.y - origin_position.y;
          tree_.back().yaw = atan2(dy, dx);
          childs ++;
        }
      }
    }

    closed_set_.push_back(origin);
    n++;

    //find best node to continue
    double minimal_cost = inf;
    for (int i = 0; i < tree_.size(); i++) {
      bool closed = false;
      for (int j = 0; j < closed_set_.size(); j++) {
        if (closed_set_[j] == i) {
          closed = true;
        }
      }
      if (tree_[i].total_cost < minimal_cost && !closed) {
        minimal_cost = tree_[i].total_cost;
        origin = i;
      }
    }
  }
  //smoothing between trees
  int tree_end = origin;
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


bool StarPlanner::getDirectionFromTree(nav_msgs::GridCells &path_waypoints) {
  if (tree_available_) {
    int size = path_node_positions_.size();
    geometry_msgs::Point p;

    if (tree_new_) {
      int goal_node_nr = path_node_origins_[size-2];
      p.x = tree_[goal_node_nr].last_e;
      p.y = tree_[goal_node_nr].last_z;
      p.z = 0;

      path_waypoints.cells.push_back(p);
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

      int wp_idx = std::min(min_dist_idx, second_min_dist_idx);
      if (min_dist > 3.0 || wp_idx == 0) {
        tree_available_ = false;
      } else {

//        if (distances[wp_idx] < 0.3 && wp_idx != 0) {
//          wp_idx--;
//        }
        double cos_alpha = (tree_node_distance_ * tree_node_distance_ + distances[wp_idx] * distances[wp_idx] - distances[wp_idx + 1] * distances[wp_idx + 1]) / (2 * tree_node_distance_ * distances[wp_idx]);
        double l_front = distances[wp_idx] * cos_alpha;
        double l_frac = l_front / tree_node_distance_;

        geometry_msgs::Point mean_point;
        mean_point.x = (1.0 - l_frac) * path_node_positions_[wp_idx].x + l_frac * path_node_positions_[wp_idx - 1].x;
        mean_point.y = (1.0 - l_frac) * path_node_positions_[wp_idx].y + l_frac * path_node_positions_[wp_idx - 1].y;
        mean_point.z = (1.0 - l_frac) * path_node_positions_[wp_idx].z + l_frac * path_node_positions_[wp_idx - 1].z;

        int wp_z = floor(atan2(mean_point.x - pose_.pose.position.x, mean_point.y - pose_.pose.position.y) * 180.0 / PI);  //azimuthal angle
        int wp_e = floor(atan((mean_point.z - pose_.pose.position.z) / sqrt(pow((mean_point.x - pose_.pose.position.x), 2) + pow((mean_point.y - pose_.pose.position.y), 2))) * 180.0 / PI);

        p.x = wp_e;
        p.y = wp_z;
        p.z = 0;

        path_waypoints.cells.push_back(p);
      }
    }
  }

  return tree_available_;
}
