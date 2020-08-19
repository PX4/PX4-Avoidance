#ifndef GLOBAL_PLANNER_ANALYSIS_H_
#define GLOBAL_PLANNER_ANALYSIS_H_

#include "global_planner/cell.h"
#include "global_planner/node.h"

// This file consists mostly of ugly debug functions which contain no logic

namespace global_planner {

// Print information about all the costs and heuristics on the path
template <typename GlobalPlanner>
void printPathStats(GlobalPlanner* global_planner, const std::vector<Cell>& path, const Cell& start_parent,
                    const Cell& start, const Cell& goal, double total_cost) {
  if (path.size() == 0) {
    printf("PATH IS EMPTY");
    return;
  }

  printf("\n\n\nPath analysis: \n");

  // Loop through the path to get real values to compare to the heuristics
  double total_dist_cost = 0.0;
  double total_risk_cost = 0.0;
  double total_alt_change_cost = 0.0;
  double total_smooth_cost = 0.0;
  for (int i = 2; i < path.size(); ++i) {
    Node last_node = Node(path[i - 1], path[i - 2]);
    Node curr_node = Node(path[i], path[i - 1]);
    if (curr_node.cell_.zIndex() - curr_node.parent_.zIndex() == 0) {
      total_dist_cost += global_planner->getEdgeDist(curr_node.parent_, curr_node.cell_);
    } else {
      total_alt_change_cost += global_planner->getEdgeDist(curr_node.parent_, curr_node.cell_);
    }
    total_risk_cost += global_planner->risk_factor_ * global_planner->getRisk(curr_node);
    total_smooth_cost += global_planner->smooth_factor_ * global_planner->getTurnSmoothness(last_node, curr_node);
  }

  double curr_cost = 0.0;

  Node first_node = Node(start, start_parent);
  printf(
      "Cell:\t \tcurrCo \theuri \ttoGoal \tOvEst \t|| \tEdgeC  \tEdgeD \tEdgeR "
      "\tEdgeS \t||\theuris \t\tDist \t\tRisk  \t\tAlti   \t\tSmooth\n");
  printf("%s (parent) \n", start_parent.asString().c_str());
  printf("%s: \t%3.2f \t%3.2f \t%3.2f \t%3.2f \t|| \n", start.asString().c_str(), curr_cost,
         global_planner->getHeuristic(first_node, goal), total_cost,
         total_cost / global_planner->getHeuristic(first_node, goal));

  for (int i = 2; i < path.size(); ++i) {
    Node last_node = Node(path[i - 1], path[i - 2]);
    Node curr_node = Node(path[i], path[i - 1]);
    curr_cost += global_planner->getEdgeCost(last_node, curr_node);
    double heuristic = global_planner->getHeuristic(curr_node, goal);
    double actual_cost = total_cost - curr_cost;
    double ov_est = actual_cost / heuristic;

    double edge_c = global_planner->getEdgeCost(last_node, curr_node);
    double edge_d = global_planner->getEdgeDist(curr_node.parent_, curr_node.cell_);
    double edge_r = global_planner->risk_factor_ * global_planner->getRisk(curr_node);
    double edge_s = global_planner->smooth_factor_ * global_planner->getTurnSmoothness(last_node, curr_node);

    if (curr_node.cell_.zIndex() - curr_node.parent_.zIndex() == 0) {
      total_dist_cost -= edge_d;
    } else {
      total_alt_change_cost -= edge_d;
    }
    total_risk_cost -= edge_r;
    total_smooth_cost -= edge_s;

    double dist_heuristics = curr_node.cell_.diagDistance2D(goal);  // Lower bound for distance on a grid
    double risk_h = global_planner->riskHeuristic(curr_node.cell_, goal);
    double alt_heuristics =
        global_planner->altitudeHeuristic(curr_node.cell_, goal);  // Lower bound cost due to altitude change
    double smooth_heuristics = global_planner->smoothnessHeuristic(curr_node, goal);
    printf("%s: \t%3.2f \t%3.2f \t%3.2f \t%3.2f", curr_node.cell_.asString().c_str(), curr_cost, heuristic, actual_cost,
           ov_est);
    printf("\t|| \t%3.2f \t%3.2f \t%3.2f \t%3.2f", edge_c, edge_d, edge_r, edge_s);
    printf(
        "\t|| \t%3.2f (%3.2f) \t%3.2f (%3.2f) \t%3.2f (%3.2f) \t%3.2f (%3.2f) "
        "\t%3.2f (%3.2f)\n",
        heuristic, actual_cost, dist_heuristics, total_dist_cost, risk_h, total_risk_cost, alt_heuristics,
        total_alt_change_cost, smooth_heuristics, total_smooth_cost);

    if (smooth_heuristics > 4 * global_planner->smooth_factor_) {
      Node u = curr_node;
      double u_ang = (u.cell_ - u.parent_).angle();
      double goal_ang = (goal - u.cell_).angle();
      double ang_diff = goal_ang - u_ang;
      double ang_diff2 = angleToRange(ang_diff);
      double ang_diff3 = std::fabs(ang_diff2);                      // positive angle difference
      double num_45_deg_turns = std::ceil(ang_diff3 / (M_PI / 4));  // Minimum number of 45-turns to goal
      printf("\t|| \t%3.2f \t%3.2f \t%3.2f \t%3.2f \t%3.2f \t%3.2f \n", u_ang, goal_ang, ang_diff, ang_diff2, ang_diff3,
             num_45_deg_turns);
      ROS_INFO("WTF? \n %f %f \n\n\n\n\n\n\n\n\n\n\n\n", angleToRange(5.5), angleToRange(-5.5));
    }
  }
  printf("\n\n");
}

// Print information about a point, mostly the risk of the containing Cell
template <typename GlobalPlanner>
void printPointStats(GlobalPlanner* global_planner, double x, double y, double z) {
  Cell cell(x, y, z);
  ROS_INFO("\n\nDEBUG INFO FOR %s", cell.asString().c_str());
  ROS_INFO("Rist cost: %2.2f", global_planner->risk_factor_ * global_planner->getRisk(cell));
  ROS_INFO("getRisk: %2.2f", global_planner->getRisk(cell));
  ROS_INFO("singleCellRisk: %2.2f", global_planner->getSingleCellRisk(cell));
  ROS_INFO(
      "Neighbors:\n \t %2.2f \t \t \t %2.2f \n %2.2f \t \t %2.2f \n \t %2.2f "
      "\t \t \t %2.2f",
      global_planner->getSingleCellRisk(Cell(x, y + 1, z)), global_planner->getSingleCellRisk(Cell(x, y, z + 1)),
      global_planner->getSingleCellRisk(Cell(x - 1, y, z)), global_planner->getSingleCellRisk(Cell(x + 1, y, z)),
      global_planner->getSingleCellRisk(Cell(x, y - 1, z)), global_planner->getSingleCellRisk(Cell(x, y, z - 1)));

  double heuristics = global_planner->getHeuristic(Node(cell, cell), global_planner->goal_pos_);
  ROS_INFO("Heuristics: %2.2f", heuristics);

  octomap::OcTreeNode* node = global_planner->octree_->search(x, y, z);
  if (node) {
    double prob = octomap::probability(node->getValue());
    double post_prob = posterior(global_planner->getAltPrior(cell), prob);
    ROS_INFO("prob: %2.2f \t post_prob: %2.2f", prob, post_prob);
    if (global_planner->occupied_.find(cell) != global_planner->occupied_.end()) {
      ROS_INFO("Cell in occupied, posterior: %2.2f", post_prob);
    } else {
      ROS_INFO("Cell NOT in occupied, posterior: %2.2f", global_planner->explore_penalty_ * post_prob);
    }
  } else {
    ROS_INFO("Cell not in tree, prob: %2.2f", global_planner->explore_penalty_ * global_planner->getAltPrior(cell));
  }
}

}  // namespace global_planner

#endif /* GLOBAL_PLANNER_ANALYSIS_H_ */
