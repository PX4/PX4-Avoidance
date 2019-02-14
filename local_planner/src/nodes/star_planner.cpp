#include "local_planner/star_planner.h"
#include "local_planner/common.h"
#include "local_planner/planner_functions.h"
#include "local_planner/tree_node.h"

#include <ros/console.h>

namespace avoidance {

StarPlanner::StarPlanner() : tree_age_(0) {}

// set parameters changed by dynamic rconfigure
void StarPlanner::dynamicReconfigureSetStarParams(
    const avoidance::LocalPlannerNodeConfig& config, uint32_t level) {
  children_per_node_ = config.children_per_node_;
  n_expanded_nodes_ = config.n_expanded_nodes_;
  tree_node_distance_ = static_cast<float>(config.tree_node_distance_);
  tree_discount_factor_ = static_cast<float>(config.tree_discount_factor_);
  n_points_occupied_ = config.n_points_occupied_;
}

void StarPlanner::setParams(costParameters cost_params) {
  cost_params_ = cost_params;
}

void StarPlanner::setFOV(float h_FOV, float v_FOV) {
  h_FOV_ = h_FOV;
  v_FOV_ = v_FOV;
}

void StarPlanner::setPose(const geometry_msgs::PoseStamped& pose,
                          float curr_yaw) {
  pose_ = pose;
  curr_yaw_ = curr_yaw;
}

void StarPlanner::setCloud(
    const pcl::PointCloud<pcl::PointXYZ>& cropped_cloud) {
  pointcloud_ = cropped_cloud;
}

void StarPlanner::setGoal(const geometry_msgs::Point& goal) {
  goal_ = toEigen(goal);
  tree_age_ = 1000;
}

void StarPlanner::setReprojectedPoints(
    const pcl::PointCloud<pcl::PointXYZ>& reprojected_points,
    const std::vector<int>& reprojected_points_age) {
  reprojected_points_ = reprojected_points;
  reprojected_points_age_ = reprojected_points_age;
}

float StarPlanner::treeCostFunction(int node_number) {
  int origin = tree_[node_number].origin_;
  float e = tree_[node_number].last_e_;
  float z = tree_[node_number].last_z_;
  Eigen::Vector3f origin_position = tree_[origin].getPosition();
  PolarPoint goal_pol = cartesianToPolar(goal_, origin_position);

  float target_cost =
      indexAngleDifference(z, goal_pol.z) +
      10.0f *
          indexAngleDifference(e, goal_pol.e);  // include effective direction?
  float turning_cost =
      5.0f *
      indexAngleDifference(z, tree_[0].yaw_);  // maybe include pitching cost?

  float last_e = tree_[origin].last_e_;
  float last_z = tree_[origin].last_z_;

  float smooth_cost = 5.0f * (2.0f * indexAngleDifference(z, last_z) +
                              5.0f * indexAngleDifference(e, last_e));

  float smooth_cost_to_old_tree = 0.0f;
  if (tree_age_ < 10) {
    int partner_node_idx =
        path_node_positions_.size() - 1 - tree_[node_number].depth_;
    if (partner_node_idx >= 0) {
      geometry_msgs::Point partner_node_position =
          path_node_positions_[partner_node_idx];
      Eigen::Vector3f node_position = tree_[node_number].getPosition();
      float dist = (toEigen(partner_node_position) - node_position).norm();
      smooth_cost_to_old_tree =
          200.0f * dist /
          (0.5f * static_cast<float>(tree_[node_number].depth_));
    }
  }

  return std::pow(tree_discount_factor_,
                  static_cast<float>(tree_[node_number].depth_)) *
         (target_cost + smooth_cost + smooth_cost_to_old_tree + turning_cost);
}
float StarPlanner::treeHeuristicFunction(int node_number) {
  Eigen::Vector3f node_position = tree_[node_number].getPosition();
  PolarPoint goal_pol = cartesianToPolar(goal_, node_position);

  int origin = tree_[node_number].origin_;
  Eigen::Vector3f origin_position = tree_[origin].getPosition();
  float origin_goal_dist = (goal_ - origin_position).norm();
  float goal_dist = (goal_ - node_position).norm();
  float goal_cost = (goal_dist / origin_goal_dist - 0.9f) * 5000.0f;

  float smooth_cost =
      10.0f * (indexAngleDifference(goal_pol.z, tree_[node_number].last_z_) +
               indexAngleDifference(goal_pol.e, tree_[node_number].last_e_));

  return std::pow(tree_discount_factor_,
                  static_cast<float>(tree_[node_number].depth_)) *
         (smooth_cost + goal_cost);
}

void StarPlanner::buildLookAheadTree() {
  std::clock_t start_time = std::clock();
  tree_.clear();
  closed_set_.clear();

  // insert first node
  tree_.push_back(TreeNode(0, 0, toEigen(pose_.pose.position)));
  tree_.back().setCosts(treeHeuristicFunction(0), treeHeuristicFunction(0));
  tree_.back().yaw_ =
      std::round((-curr_yaw_ * 180.0f / M_PI_F)) +
      90.0f;  // from radian to angle and shift reference to y-axis
  tree_.back().last_z_ = tree_.back().yaw_;

  int origin = 0;

  for (int n = 0; n < n_expanded_nodes_; n++) {
    Eigen::Vector3f origin_position = tree_[origin].getPosition();
    int old_origin = tree_[origin].origin_;
    Eigen::Vector3f origin_origin_position = tree_[old_origin].getPosition();
    bool hist_is_empty = false;  // unused

    // build new histogram
    std::vector<int> z_FOV_idx;
    int e_FOV_min, e_FOV_max;
    calculateFOV(h_FOV_, v_FOV_, z_FOV_idx, e_FOV_min, e_FOV_max,
                 tree_[origin].yaw_,
                 0.0f);  // assume pitch is zero at every node

    Histogram propagated_histogram = Histogram(2 * ALPHA_RES);
    Histogram histogram = Histogram(ALPHA_RES);

    propagateHistogram(propagated_histogram, reprojected_points_,
                       reprojected_points_age_, origin_position);
    generateNewHistogram(histogram, pointcloud_, origin_position,
                         n_points_occupied_);
    combinedHistogram(hist_is_empty, histogram, propagated_histogram, false,
                      z_FOV_idx, e_FOV_min, e_FOV_max);

    // calculate candidates
    Eigen::MatrixXf cost_matrix;
    std::vector<candidateDirection> candidate_vector;
    getCostMatrix(histogram, goal_, origin_position, origin_origin_position,
                  cost_params_, false, cost_matrix);
    getBestCandidatesFromCostMatrix(cost_matrix, children_per_node_,
                                    candidate_vector);

    // add candidates as nodes
    if (candidate_vector.empty()) {
      tree_[origin].total_cost_ = HUGE_VAL;
    } else {
      // insert new nodes
      int depth = tree_[origin].depth_ + 1;
      int children = 0;
      for (candidateDirection candidate : candidate_vector) {
        PolarPoint p_pol(candidate.elevation_angle, candidate.azimuth_angle,
                         tree_node_distance_);

        // check if another close node has been added
        Eigen::Vector3f node_location =
            polarToCartesian(p_pol, toPoint(origin_position));
        int close_nodes = 0;
        for (size_t i = 0; i < tree_.size(); i++) {
          float dist = (tree_[i].getPosition() - node_location).norm();
          if (dist < 0.2f) {
            close_nodes++;
          }
        }

        if (children < children_per_node_ && close_nodes == 0) {
          tree_.push_back(TreeNode(origin, depth, node_location));
          tree_.back().last_e_ = p_pol.e;
          tree_.back().last_z_ = p_pol.z;
          float h = treeHeuristicFunction(tree_.size() - 1);
          float c = treeCostFunction(tree_.size() - 1);
          tree_.back().heuristic_ = h;
          tree_.back().total_cost_ =
              tree_[origin].total_cost_ - tree_[origin].heuristic_ + c + h;
          Eigen::Vector3f diff = node_location - origin_position;
          tree_.back().yaw_ = atan2(diff.y(), diff.x());
          children++;
        }
      }
    }

    closed_set_.push_back(origin);

    // find best node to continue
    float minimal_cost = HUGE_VAL;
    for (size_t i = 0; i < tree_.size(); i++) {
      bool closed = false;
      for (size_t j = 0; j < closed_set_.size(); j++) {
        if (closed_set_[j] == (int)i) {
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
    path_node_positions_.push_back(toPoint(tree_[tree_end].getPosition()));
    tree_end = tree_[tree_end].origin_;
  }
  path_node_positions_.push_back(toPoint(tree_[0].getPosition()));
  path_node_origins_.push_back(0);
  tree_age_ = 0;

  ROS_INFO(
      "\033[0;35m[SP]Tree (%.0f nodes, %.0f path nodes, %.0f expanded) "
      "calculated in %2.2fms.\033[0m",
      (double)tree_.size(), (double)path_node_positions_.size(),
      (double)closed_set_.size(),
      (std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
  for (int j = 0; j < path_node_positions_.size(); j++) {
    ROS_DEBUG("\033[0;35m[SP] node %.0f : [ %f, %f, %f]\033[0m", (double)j,
              (double)path_node_positions_[j].x,
              (double)path_node_positions_[j].y,
              (double)path_node_positions_[j].z);
  }
}
}
