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
}

void StarPlanner::setParams(const costParameters& cost_params, const simulation_limits& limits) {
  cost_params_ = cost_params;
  lims_ = limits;
}

void StarPlanner::setPose(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel) {
  position_ = pos;
  velocity_ = vel;
}

void StarPlanner::setGoal(const Eigen::Vector3f& goal) { goal_ = goal; }

void StarPlanner::setPointcloud(const pcl::PointCloud<pcl::PointXYZI>& cloud) { cloud_ = cloud; }

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
  simulation_state start_state;
  start_state.position = position_;
  start_state.velocity = velocity_;
  start_state.acceleration = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  start_state.time = ros::Time::now().toSec();
  tree_.push_back(TreeNode(0, start_state, Eigen::Vector3f::Zero()));
  tree_.back().setCosts(treeHeuristicFunction(0), treeHeuristicFunction(0));

  int origin = 0;
  for (int n = 0; n < n_expanded_nodes_ && is_expanded_node; n++) {
    Eigen::Vector3f origin_position = tree_[origin].getPosition();
    Eigen::Vector3f origin_velocity = tree_[origin].getVelocity();
    PolarPoint facing_goal = cartesianToPolarHistogram(goal_, origin_position);
    float distance_to_goal = (goal_ - origin_position).norm();

    histogram.setZero();
    generateNewHistogram(histogram, cloud_, origin_position);

    // calculate candidates
    cost_matrix.fill(0.f);
    cost_image_data.clear();
    candidate_vector.clear();
    getCostMatrix(histogram, goal_, origin_position, origin_velocity, cost_params_, smoothing_margin_degrees_,
                  cost_matrix, cost_image_data);
    getBestCandidatesFromCostMatrix(cost_matrix, children_per_node_, candidate_vector);

    // add candidates as nodes
    if (candidate_vector.empty()) {
      tree_[origin].total_cost_ = HUGE_VAL;
    } else {
      // insert new nodes
      int children = 0;
      for (candidateDirection candidate : candidate_vector) {
        simulation_state state = tree_[origin].state;
        TrajectorySimulator sim(lims_, state, 0.05f);  // todo: parameterize simulation step size [s]
        std::vector<simulation_state> trajectory = sim.generate_trajectory(candidate.toEigen(), tree_node_duration_);

        // Ignore node if it brings us farther away from the goal
        // todo: this breaks being able to get out of concave obstacles! But it helps to not "overplan"
        if ((trajectory.back().position - goal_).norm() > (trajectory.front().position - goal_).norm()) {
          continue;
        }

        // check if another close node has been added
        int close_nodes = 0;
        for (size_t i = 0; i < tree_.size(); i++) {
          float dist = (tree_[i].getPosition() - trajectory.back().position).norm();
          if (dist < 0.2f) {
            close_nodes++;
            break;
          }
        }

        if (children < children_per_node_ && close_nodes == 0) {
          tree_.push_back(TreeNode(origin, trajectory.back(), candidate.toEigen()));
          float h = treeHeuristicFunction(tree_.size() - 1);
          tree_.back().heuristic_ = h;
          tree_.back().total_cost_ = tree_[origin].total_cost_ - tree_[origin].heuristic_ + candidate.cost + h;
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

  // Get setpoints into member vector
  int tree_end = origin;
  path_node_setpoints_.clear();
  while (tree_end > 0) {
    path_node_setpoints_.push_back(tree_[tree_end].getSetpoint());
    tree_end = tree_[tree_end].origin_;
  }
  path_node_setpoints_.push_back(tree_[0].getSetpoint());
}
}
