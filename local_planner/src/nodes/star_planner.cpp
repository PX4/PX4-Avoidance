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

void StarPlanner::setPose(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel) {
  position_ = pos;
  velocity_ = vel;
}

void StarPlanner::setGoal(const Eigen::Vector3f& goal) { goal_ = goal; }

void StarPlanner::setPointcloud(const pcl::PointCloud<pcl::PointXYZI>& cloud) { cloud_ = cloud; }

void StarPlanner::setClosestPointOnLine(const Eigen::Vector3f& closest_pt) { closest_pt_ = closest_pt; }

float StarPlanner::treeHeuristicFunction(const Eigen::Vector3f &node_position) const {
  // return (goal_ - tree_[node_number].getSetpoint()).norm() * tree_heuristic_weight_;
  return (goal_ - node_position).norm() * tree_heuristic_weight_;

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
  tree_.push_back(TreeNode(0, position_));
  tree_.back().setCosts(treeHeuristicFunction(tree_.back().getSetpoint()), treeHeuristicFunction(tree_.back().getSetpoint()));

  int origin = 0;
  for (int n = 0; n < n_expanded_nodes_ && is_expanded_node; n++) {
    Eigen::Vector3f origin_position = tree_[origin].getSetpoint();
    Eigen::Vector3f origin_velocity = Eigen::Vector3f(0.0f, 0.0f, 0.0f); //tree_[origin].getVelocity();

    histogram.setZero();
    generateNewHistogram(histogram, cloud_, origin_position);

    // calculate candidates
    cost_matrix.fill(0.f);
    cost_image_data.clear();
    candidate_vector.clear();
    getCostMatrix(histogram, goal_, origin_position, origin_velocity, cost_params_, smoothing_margin_degrees_,
                  closest_pt_, max_sensor_range_, min_sensor_range_, cost_matrix, cost_image_data);

    iterable_priority_queue<candidateDirection, std::vector<candidateDirection>, std::less<candidateDirection>> queue;
    for (int row_index = 0; row_index < cost_matrix.rows(); row_index++) {
      for (int col_index = 0; col_index < cost_matrix.cols(); col_index++) {
        PolarPoint p_pol = histogramIndexToPolar(row_index, col_index, ALPHA_RES, 1.0);
        float cost = cost_matrix(row_index, col_index);
        candidateDirection candidate(cost, p_pol.e, p_pol.z);
        float h = treeHeuristicFunction(origin_position + candidate.toEigen());
        candidate.cost += h;

        // simulation_state state = tree_[origin].state;
        // TrajectorySimulator sim(lims_, state, 0.05f);  // todo: parameterize simulation step size [s]
        // simulation_state trajectory = sim.generate_trajectory_endpoint(candidate.toEigen(), tree_node_duration_);
        candidate.trajectory_endpoint.position = origin_position + candidate.toEigen(); //trajectory;

        // check if another close node has been added
        int close_nodes = 0;
        for (size_t i = 0; i < tree_.size(); i++) {
          float dist = (tree_[i].getSetpoint() - candidate.trajectory_endpoint.position).norm();
          if (dist < 0.2f) {
            close_nodes++;
            break;
          }
        }

        for (auto it = queue.begin(); it != queue.end(); it++) {
          float dist = ((*it).trajectory_endpoint.position - candidate.trajectory_endpoint.position).norm();
          if (dist < 0.2f) {
            close_nodes++;
            break;
          }
        }

        if (queue.size() < children_per_node_ && close_nodes == 0) {
          queue.push(candidate);
        } else if (candidate < queue.top() && close_nodes == 0) {
          queue.push(candidate);
          queue.pop();
        }
      }
    }

    std::vector<candidateDirection> candidate_vector;
    candidate_vector.reserve(queue.size());
    while (!queue.empty()) {
      candidate_vector.push_back(queue.top());
      queue.pop();
    }
    std::reverse(candidate_vector.begin(), candidate_vector.end());

    // add candidates as nodes
    if (candidate_vector.empty()) {
      tree_[origin].total_cost_ = HUGE_VAL;
    } else {
      // insert new nodes
      int children = 0;
      for (candidateDirection candidate : candidate_vector) {

        // Ignore node if it brings us farther away from the goal
        // todo: this breaks being able to get out of concave obstacles! But it helps to not "overplan"
        if ((candidate.trajectory_endpoint.position - goal_).norm() > (tree_[origin].getSetpoint() - goal_).norm()) {
          continue;
        }

        // check if another close node has been added
        // int close_nodes = 0;
        // for (size_t i = 0; i < tree_.size(); i++) {
        //   float dist = (tree_[i].getPosition() - trajectory.back().position).norm();
        //   if (dist < 0.2f) {
        //     close_nodes++;
        //     break;
        //   }
        // }

        // if (children < children_per_node_ && close_nodes == 0) {
          tree_.push_back(TreeNode(origin, candidate.trajectory_endpoint.position));
          // float h = treeHeuristicFunction(tree_.size() - 1);
          // tree_.back().heuristic_ = h;
          float h = treeHeuristicFunction(candidate.trajectory_endpoint.position);

          tree_.back().total_cost_ = tree_[origin].total_cost_ + candidate.cost; // + h;
          // printf("origin %d pos (%f %f %f) cost %f h %f tot cost %f \n", origin, candidate.trajectory_endpoint.position.x(), candidate.trajectory_endpoint.position.y(), candidate.trajectory_endpoint.position.z(),
          // candidate.cost - h, h, tree_.back().total_cost_);
          children++;
        // }
      }
    }


    closed_set_.push_back(origin);
    tree_[origin].closed_ = true;

    // find best node to continue
    float minimal_cost = HUGE_VAL;
    is_expanded_node = false;
    for (size_t i = 0; i < tree_.size(); i++) {
      if (!(tree_[i].closed_)) {
        float node_distance = (tree_[i].getSetpoint() - position_).norm();
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

  for (size_t i = 0; i < path_node_setpoints_.size(); i++) {
    printf("(%f %f %f) ", path_node_setpoints_[i].x(), path_node_setpoints_[i].y(), path_node_setpoints_[i].z());
  }


}
}
