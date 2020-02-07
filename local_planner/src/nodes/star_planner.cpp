#include "local_planner/star_planner.h"

#include "avoidance/common.h"
#include "local_planner/planner_functions.h"
#include "local_planner/tree_node.h"

#include <ros/console.h>

namespace avoidance {

StarPlanner::StarPlanner() {}

// set parameters changed by dynamic rconfigure
void StarPlanner::dynamicReconfigureSetStarParams(const avoidance::LocalPlannerNodeConfig& config, uint32_t level) {
  children_per_node_ = config.children_per_node_;
  n_expanded_nodes_ = config.n_expanded_nodes_;
  tree_node_distance_ = static_cast<float>(config.tree_node_distance_);
  max_path_length_ = static_cast<float>(config.max_sensor_range_);
  smoothing_margin_degrees_ = static_cast<float>(config.smoothing_margin_degrees_);
  tree_heuristic_weight_ = static_cast<float>(config.tree_heuristic_weight_);
  max_sensor_range_ = static_cast<float>(config.max_sensor_range_);
  min_sensor_range_ = static_cast<float>(config.min_sensor_range_);
}

void StarPlanner::setParams(costParameters cost_params) { cost_params_ = cost_params; }

void StarPlanner::setPose(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel) {
  position_ = pos;
  velocity_ = vel;
}

void StarPlanner::setGoal(const Eigen::Vector3f& goal) { goal_ = goal; }

void StarPlanner::setPointcloud(const pcl::PointCloud<pcl::PointXYZI>& cloud) { cloud_ = cloud; }

void StarPlanner::setClosestPointOnLine(const Eigen::Vector3f& closest_pt) { closest_pt_ = closest_pt; }

float StarPlanner::treeHeuristicFunction(int node_number) const {
  return (goal_ - tree_[node_number].getPosition()).norm() * tree_heuristic_weight_;
}

void StarPlanner::buildLookAheadTree() {
  std::clock_t start_time = std::clock();

  Histogram histogram(ALPHA_RES);
  std::vector<uint8_t> cost_image_data;
  std::vector<candidateDirection> candidate_vector;
  Eigen::MatrixXf cost_matrix;

  bool is_expanded_node = true;

  tree_.clear();
  closed_set_.clear();

  // insert first node
  tree_.push_back(TreeNode(0, position_, velocity_));
  tree_.back().setCosts(treeHeuristicFunction(0), treeHeuristicFunction(0));

  int origin = 0;
  for (int n = 0; n < n_expanded_nodes_ && is_expanded_node; n++) {
    Eigen::Vector3f origin_position = tree_[origin].getPosition();
    Eigen::Vector3f origin_velocity = tree_[origin].getVelocity();

    histogram.setZero();
    generateNewHistogram(histogram, cloud_, origin_position);

    // calculate candidates
    cost_matrix.fill(0.f);
    cost_image_data.clear();
    candidate_vector.clear();
    getCostMatrix(histogram, goal_, origin_position, origin_velocity, cost_params_, smoothing_margin_degrees_,
                  closest_pt_, max_sensor_range_, min_sensor_range_, cost_matrix, cost_image_data);
    getBestCandidatesFromCostMatrix(cost_matrix, children_per_node_, candidate_vector);

    // add candidates as nodes
    if (candidate_vector.empty()) {
      tree_[origin].total_cost_ = HUGE_VAL;
    } else {
      // insert new nodes
      int children = 0;
      for (candidateDirection candidate : candidate_vector) {
        PolarPoint candidate_polar = candidate.toPolar(tree_node_distance_);
        Eigen::Vector3f node_location = polarHistogramToCartesian(candidate_polar, origin_position);
        Eigen::Vector3f node_velocity = node_location - origin_position;  // todo: simulate!

        // check if another close node has been added
        int close_nodes = 0;
        for (size_t i = 0; i < tree_.size(); i++) {
          float dist = (tree_[i].getPosition() - node_location).norm();
          if (dist < 0.2f) {
            close_nodes++;
            break;
          }
        }

        if (children < children_per_node_ && close_nodes == 0) {
          tree_.push_back(TreeNode(origin, node_location, node_velocity));
          float h = treeHeuristicFunction(tree_.size() - 1);
          tree_.back().heuristic_ = h;
          tree_.back().total_cost_ = tree_[origin].total_cost_ - tree_[origin].heuristic_ + candidate.cost + h;
          tree_.back().depth_ = tree_[origin].depth_ + 1;
          children++;
        }
      }
    }

    closed_set_.push_back(origin);
    tree_[origin].closed_ = true;

    // find best node to continue
    float minimal_cost = HUGE_VAL;
    is_expanded_node = false;
    for (size_t i = 0; i < tree_.size(); i++) {
      if (!(tree_[i].closed_)) {
        float node_distance = (tree_[i].getPosition() - position_).norm();
        if (tree_[i].total_cost_ < minimal_cost && node_distance < max_path_length_) {
          minimal_cost = tree_[i].total_cost_;
          origin = i;
          is_expanded_node = true;
        }
      }
    }

    cost_image_data.clear();
    candidate_vector.clear();
  }

  // find best node to follow, taking into account A* completion
  int max_depth = 0;
  int max_depth_index = 0;
  for (size_t i = 0; i < tree_.size(); i++) {
    if (!(tree_[i].closed_)) {
      if (tree_[i].depth_ > max_depth) {
        max_depth = tree_[i].depth_;
        max_depth_index = i;
      }
    }
  }

  // build final tree
  int tree_end = max_depth_index;
  path_node_positions_.clear();
  while (tree_end > 0) {
    path_node_positions_.push_back(tree_[tree_end].getPosition());
    tree_end = tree_[tree_end].origin_;
  }
  path_node_positions_.push_back(tree_[0].getPosition());

  ROS_INFO("\033[0;35m[SP]Tree (%lu nodes, %lu path nodes, %lu expanded) calculated in %2.2fms.\033[0m", tree_.size(),
           path_node_positions_.size(), closed_set_.size(),
           static_cast<double>((std::clock() - start_time) / static_cast<double>(CLOCKS_PER_SEC / 1000)));

#ifndef DISABLE_SIMULATION  // For large trees, this could be very slow!
  for (int j = 0; j < path_node_positions_.size(); j++) {
    ROS_DEBUG("\033[0;35m[SP] node %i : [ %f, %f, %f]\033[0m", j, path_node_positions_[j].x(),
              path_node_positions_[j].y(), path_node_positions_[j].z());
  }
#endif
}
}
