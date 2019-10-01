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
  tree_node_duration_ = static_cast<float>(config.tree_node_duration_);
  max_path_length_ = static_cast<float>(config.max_sensor_range_);
  smoothing_margin_degrees_ = static_cast<float>(config.smoothing_margin_degrees_);
  tree_heuristic_weight_ = static_cast<float>(config.tree_heuristic_weight_);
  max_sensor_range_ = static_cast<float>(config.max_sensor_range_);
  min_sensor_range_ = static_cast<float>(config.min_sensor_range_);
}

void StarPlanner::setParams(const costParameters& cost_params, const simulation_limits& limits, float acc_rad) {
  cost_params_ = cost_params;
  lims_ = limits;
  acceptance_radius_ = acc_rad;
}

void StarPlanner::setPose(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel, const Eigen::Quaternionf& q) {
  position_ = pos;
  velocity_ = vel;
  q_ = q;
}

void StarPlanner::setGoal(const Eigen::Vector3f& goal) { goal_ = goal; }

void StarPlanner::setPointcloud(const kdtree_t& cloud) { cloud_ = cloud; }

void StarPlanner::setClosestPointOnLine(const Eigen::Vector3f& closest_pt) { closest_pt_ = closest_pt; }

float StarPlanner::treeHeuristicFunction(int node_number) const {
  return (goal_ - tree_[node_number].getPosition()).norm() * tree_heuristic_weight_;
}

void StarPlanner::buildLookAheadTree() {
  std::clock_t start_time = std::clock();
  // Simple 6-way unit direction setpoints allowed only.
  // TODO: If compute allowws, make this more fine-grained
  // These are in a shitty local-aligned but body-centered frame
  const std::vector<Eigen::Vector3f> candidates{
      Eigen::Vector3f{1.0f, 0.0f, 0.0f},      Eigen::Vector3f{0.0f, 1.0f, 0.0f},
      Eigen::Vector3f{0.0f, 0.0f, 1.0f},      Eigen::Vector3f{-1.0f, 0.0f, 0.0f},
      Eigen::Vector3f{0.0f, -1.0f, 0.0f},     Eigen::Vector3f{0.0f, 0.0f, -1.0f},
      Eigen::Vector3f{0.707f, 0.707f, 0.0f},  Eigen::Vector3f{0.707f, -0.707f, 0.0f},
      Eigen::Vector3f{-0.707f, 0.707f, 0.0f}, Eigen::Vector3f{-0.707f, -0.707f, 0.0f}};
  bool has_reached_goal = false;

  tree_.clear();
  closed_set_.clear();

  // insert first node
  simulation_state start_state;
  start_state.position = position_;
  start_state.velocity = velocity_;
  start_state.acceleration = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  start_state.time = ros::Time::now().toSec();
  tree_.push_back(TreeNode(0, start_state, Eigen::Vector3f::Zero()));
  tree_.back().setCosts(treeHeuristicFunction(0), treeHeuristicFunction(0));

  int current_node = 0;
  while (!has_reached_goal) {
    Eigen::Vector3f current_node_position = tree_[current_node].getPosition();
    Eigen::Vector3f current_node_velocity = tree_[current_node].getVelocity();

    simulation_limits limits = lims_;
    simulation_state state = tree_[current_node].state;
    limits.max_xy_velocity_norm = std::min(
        std::min(
            computeMaxSpeedFromBrakingDistance(lims_.max_jerk_norm, lims_.max_acceleration_norm,
                                               (state.position - goal_).head<2>().norm()),
            computeMaxSpeedFromBrakingDistance(lims_.max_jerk_norm, lims_.max_acceleration_norm, max_sensor_range_)),
        lims_.max_xy_velocity_norm);

    // If we reach the acceptance radius or the sensor horizon, add goal as last node and exit
    if ((tree_[current_node].getPosition() - goal_).norm() < acceptance_radius_ ||
        (tree_[current_node].getPosition() - position_).norm() >= max_sensor_range_) {
      tree_.push_back(
          TreeNode(current_node + 1, simulation_state(0.f, goal_), goal_ - tree_[current_node].getPosition()));
      tree_.back().total_cost_ = tree_[current_node].total_cost_;
      tree_.back().heuristic_ = 0.f;
      closed_set_.push_back(current_node);
      closed_set_.push_back(tree_.size() - 1);
      has_reached_goal = true;
      break;
    }

    // Expand this node: add all candidates
    for (const auto& candidate : candidates) {
      simulation_state state = tree_[current_node].state;
      TrajectorySimulator sim(limits, state, 0.1f);  // todo: parameterize simulation step size [s]
      std::vector<simulation_state> trajectory = sim.generate_trajectory(q_ * candidate, tree_node_duration_);

      // Only add the candidate as a node if it is significantly far away from any current node
      const float minimal_distance = 0.2f;  // must be at least 1m away
      bool is_useful_node = true;
      for (const auto& n : tree_) {
        if ((n.getPosition() - trajectory.back().position).norm() < minimal_distance) {
          is_useful_node = false;
          break;
        }
      }
      if (is_useful_node) {
        tree_.push_back(TreeNode(current_node, trajectory.back(), q_ * candidate));
        float h = treeHeuristicFunction(tree_.size() - 1);
        tree_.back().heuristic_ = h;
        tree_.back().total_cost_ = tree_[current_node].total_cost_ - tree_[current_node].heuristic_ +
                                   simpleCost(tree_.back(), goal_, cost_params_, cloud_) + h;
      }
    }

    closed_set_.push_back(current_node);
    tree_[current_node].closed_ = true;

    // find best node to continue
    float minimal_cost = FLT_MAX;
    for (size_t i = 0; i < tree_.size(); i++) {
      if (!(tree_[i].closed_)) {
        if (tree_[i].total_cost_ < minimal_cost) {
          minimal_cost = tree_[i].total_cost_;
          current_node = i;
        }
      }
    }

    // if there is only one node in the tree, we already expanded it.
    if (tree_.size() <= 1) {
      has_reached_goal = true;
    }
  }

  // Get setpoints into member vector
  int tree_end = current_node;
  path_node_setpoints_.clear();
  while (tree_end > 0) {
    path_node_setpoints_.push_back(tree_[tree_end].getSetpoint());
    tree_end = tree_[tree_end].origin_;
  }

  path_node_setpoints_.push_back(tree_[0].getSetpoint());
  if ((path_node_setpoints_.size() - 2) >= 0) {
    starting_direction_ = path_node_setpoints_[path_node_setpoints_.size() - 2];
  }
}
}
