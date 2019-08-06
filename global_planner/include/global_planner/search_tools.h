#ifndef GLOBAL_PLANNER_SEARCH_TOOLS_H_
#define GLOBAL_PLANNER_SEARCH_TOOLS_H_

#include <string>

#include "global_planner/bezier.h"
#include "global_planner/cell.h"
#include "global_planner/node.h"
#include "global_planner/visitor.h"

// This file consists of general search tools

namespace global_planner {

struct PathInfo {
  bool is_blocked;
  double cost;
  double dist;
  double risk;
  double smoothness;
};

struct SearchInfo {
  SearchInfo() : found_path(false), num_iter(0), search_time(0.0) {}
  SearchInfo(bool found_path_, int num_iter_, double search_time_)
      : found_path(found_path_), num_iter(num_iter_), search_time(search_time_) {}
  bool found_path;
  int num_iter;
  double search_time;  // in micro seconds
};

inline void printSearchInfo(SearchInfo info, std::string node_type = "Node", double overestimate_factor = 1.0) {
  double avg_time = info.search_time / info.num_iter;
  std::cout << std::setw(20) << std::left << node_type << std::setw(10) << std::setprecision(3) << avg_time
            << std::setw(10) << std::setprecision(3) << overestimate_factor << std::setw(10) << info.num_iter
            << std::setw(10) << 0.0;
}

// Returns a path where corners are smoothed with quadratic Bezier-curves
inline nav_msgs::Path smoothPath(const nav_msgs::Path& path) {
  if (path.poses.size() < 3) {
    return path;
  }

  nav_msgs::Path smooth_path;
  smooth_path.header = path.header;

  // Repeat the first and last points to get the first half of the first edge
  // and the second half of the last edge
  smooth_path.poses.push_back((path.poses.front()));
  for (int i = 2; i < path.poses.size(); i++) {
    geometry_msgs::Point p0 = path.poses[i - 2].pose.position;
    geometry_msgs::Point p1 = path.poses[i - 1].pose.position;
    geometry_msgs::Point p2 = path.poses[i].pose.position;
    p0 = middlePoint(p0, p1);
    p2 = middlePoint(p1, p2);

    std::vector<geometry_msgs::Point> smooth_turn = threePointBezier(p0, p1, p2);
    for (const auto& point : smooth_turn) {
      geometry_msgs::PoseStamped pose_msg = path.poses.front();  // Copy the original header info
      pose_msg.pose.position = point;
      smooth_path.poses.push_back(pose_msg);
    }
  }
  smooth_path.poses.push_back((path.poses.back()));
  return smooth_path;
}

// Returns a simpler path without increasing the cost much. It iteratively
// removes the vertices which can be removed without increasing the cost by more
// than simplify_margin
template <typename GlobalPlanner>
std::vector<Cell> simplifyPath(GlobalPlanner* global_planner, std::vector<Cell>& path, double simplify_margin = 1.01,
                               double max_iter = 100, bool decelerate_at_end = true) {
  if (path.size() < 3) {
    // Can not simplify a trivial path
    return path;
  }

  // Start with the original path
  std::vector<Cell> curr_path = path;
  for (int j = 0; j < max_iter; ++j) {
    // The first two vertices cannot be removed
    std::vector<Cell> simple_path{curr_path[0], curr_path[1]};
    int i = 3;
    for (; i < curr_path.size(); i += 2) {
      Node parent(curr_path[i - 2], curr_path[i - 3]);
      Node u(curr_path[i - 1], curr_path[i - 2]);
      Node v(curr_path[i], curr_path[i - 1]);
      Node w(curr_path[i], curr_path[i - 2]);
      // w is the new edge instead of u and v
      double curr_cost = global_planner->getEdgeCost(parent, u) + global_planner->getEdgeCost(u, v);
      double new_cost = global_planner->getEdgeCost(parent, w);
      if (new_cost > simplify_margin * curr_cost) {
        // The cost of w is too high, can't simplify this part of the path
        simple_path.push_back(curr_path[i - 1]);
      }
      simple_path.push_back(curr_path[i]);
    }

    // Make sure to add the last vertex
    if (i == curr_path.size()) {
      simple_path.push_back(curr_path[i - 1]);
    }

    if (simple_path.size() == curr_path.size()) {
      // Last iteration didn't remove any vertices, can't simplify more
      break;
    }
    curr_path = simple_path;
  }

  if (decelerate_at_end) {
    // Doubling the last point gives a triplet which stops at the end
    curr_path.push_back(curr_path.back());
  }
  return curr_path;
}

template <typename GlobalPlanner>
inline SearchInfo findSmoothPath(GlobalPlanner* global_planner, std::vector<Cell>& path, const NodePtr& s,
                                 const GoalCell& t, int max_iterations = 2000) {
  NullVisitor visitor;
  return findSmoothPath(global_planner, path, s, t, max_iterations, visitor);
}

// A* to find a path from start to t, true iff it found a path
template <typename GlobalPlanner, typename Visitor>
inline SearchInfo findSmoothPath(GlobalPlanner* global_planner, std::vector<Cell>& path, const NodePtr& s,
                                 const GoalCell& t, int max_iterations, Visitor& visitor) {
  // Initialize containers
  NodePtr best_goal_node;
  visitor.init();

  std::unordered_set<NodePtr, HashNodePtr, EqualsNodePtr> seen_nodes;
  std::unordered_map<NodePtr, NodePtr, HashNodePtr, EqualsNodePtr> parent;
  std::unordered_map<NodePtr, double, HashNodePtr, EqualsNodePtr> distance;
  std::priority_queue<PointerNodeDistancePair, std::vector<PointerNodeDistancePair>, CompareDist> pq;
  pq.push(std::make_pair(s, 0.0));
  distance[s] = 0.0;
  int num_iter = 0;

  std::clock_t start_time = std::clock();
  while (!pq.empty() && num_iter < max_iterations) {
    PointerNodeDistancePair u_node_dist = pq.top();
    pq.pop();
    NodePtr u = u_node_dist.first;
    if (seen_nodes.find(u) != seen_nodes.end()) {
      continue;
    }
    seen_nodes.insert(u);
    visitor.popNode(u);

    if (t.withinPlanRadius(u->cell_)) {
      best_goal_node = u;
      break;  // Found a path
    }
    num_iter++;

    for (const NodePtr& v : u->getNeighbors()) {
      if (!global_planner->isLegal(*v)) {
        continue;
      }
      double new_dist = distance[u] + global_planner->getEdgeCost(*u, *v);
      double old_dist = getWithDefault(distance, v, INFINITY);
      if (new_dist < old_dist) {
        // Found a better path to v, have to add v to the queue
        parent[v] = u;
        distance[v] = new_dist;
        // TODO: try Dynamic Weighting instead of a constant overestimate_factor
        double overestimated_heuristic = new_dist + global_planner->getHeuristic(*v, t);
        pq.push(PointerNodeDistancePair(v, overestimated_heuristic));
        visitor.perNeighbor(u, v);
      }
    }
  }
  double total_time = clocksToMicroSec(start_time, std::clock());
  // printf("%2.2f" total_time);
  double average_iter_time = total_time / num_iter;

  if (best_goal_node == nullptr || !t.withinPlanRadius(best_goal_node->cell_)) {
    return SearchInfo(false, num_iter, total_time);  // No path found
  }

  // Get the path by walking from t back to s (excluding s)
  NodePtr walker = best_goal_node;
  while (walker != s) {
    path.push_back(walker->cell_);
    walker = parent[walker];
  }
  path.push_back(s->cell_);
  path.push_back(s->parent_);
  std::reverse(path.begin(), path.end());

  // printPathStats(this, path, s->parent_, s->cell_, t,
  // distance[best_goal_node]); ROS_INFO("Found path with %d iterations,
  // itDistSquared: %.3f", num_iter, num_iter / squared(path.size())); printf("
  // %2.1f µs \t\t %d \t\t %2.2f \t", average_iter_time, num_iter,
  // distance[best_goal_node]);
  return SearchInfo(true, num_iter, total_time);
}

// Searches for a path from s to t at max_altitude_, fills path if it finds one
template <typename GlobalPlanner>
bool find2DPath(GlobalPlanner* global_planner, std::vector<Cell>& path, const Cell& s, const Cell& t,
                const Cell& start_parent, double alt) {
  std::vector<Cell> up_path;
  Cell above_s(Cell(s.xIndex(), s.yIndex(), alt));
  Cell above_t(Cell(t.xIndex(), t.yIndex(), alt));
  bool found_up_path = findPathOld(global_planner, up_path, s, above_s, s, true);
  std::vector<Cell> down_path;
  bool found_down_path = findPathOld(global_planner, down_path, above_t, t, above_t, true);
  std::vector<Cell> vert_path;
  bool found_vert_path = findPathOld(global_planner, vert_path, above_s, above_t, above_s, false);

  if (found_up_path && found_vert_path && found_down_path) {
    path = up_path;
    for (const Cell& c : vert_path) {
      path.push_back(c);
    }
    for (const Cell& c : down_path) {
      path.push_back(c);
    }
    return true;
  }
  return false;
}

// TODO: Run search backwards to quickly find impossible scenarios and/or find
// exact heuristics for nodes close to goal
template <typename GlobalPlanner>
bool reverseSearch(GlobalPlanner* global_planner, const Cell& t) {
  std::map<Cell, double> cost;
  std::priority_queue<CellDistancePair, std::vector<CellDistancePair>, CompareDist> pq;
  std::unordered_set<Cell> seen_cells;
  int num_iter = 0;
  cost[t] = global_planner->getRisk(t);
  pq.push(std::make_pair(t, cost[t]));

  while (!pq.empty() && num_iter++ < 100) {
    CellDistancePair u_cell_dist = pq.top();
    pq.pop();
    Cell u = u_cell_dist.first;
    if (seen_cells.find(u) != seen_cells.end()) {
      continue;
    }
    seen_cells.insert(u);
    global_planner->bubble_risk_cache_[u] = cost[u];
    global_planner->bubble_cost_ = cost[u];
    global_planner->bubble_radius_ = std::max(global_planner->bubble_radius_, u.distance3D(t));

    for (const Cell& v : u.getNeighbors()) {
      double new_cost = cost[u] + u.distance3D(v) * global_planner->risk_factor_ * global_planner->getRisk(v);
      double old_cost = getWithDefault(cost, v, INFINITY);
      if (new_cost < old_cost) {
        cost[v] = new_cost;
        pq.push(std::make_pair(v, new_cost));
      }
    }
  }
  // TODO: return?
}

// A* to find a path from s to t, true iff it found a path
template <typename GlobalPlanner>
bool findPathOld(GlobalPlanner* global_planner, std::vector<Cell>& path, const Cell& s, const Cell& t,
                 const Cell& start_parent, bool is_3D) {
  // Initialize containers
  // global_planner->seen_.clear();
  std::unordered_set<Cell> seen;
  std::map<Cell, Cell> parent;
  std::map<Cell, double> distance;
  std::priority_queue<CellDistancePair, std::vector<CellDistancePair>, CompareDist> pq;
  pq.push(std::make_pair(s, 0.0));
  distance[s] = 0.0;
  int num_iter = 0;
  // double min_dist_heuristics = INFINITY;

  std::clock_t start_time;
  start_time = std::clock();
  // Search until all reachable cells have been found, it runs out of time or t
  // is found,
  while (!pq.empty() && num_iter < global_planner->max_iterations_) {
    CellDistancePair u_cell_dist = pq.top();
    pq.pop();
    Cell u = u_cell_dist.first;
    double d = distance[u];
    if (seen.find(u) != seen.end()) {
      continue;
    }
    seen.insert(u);
    num_iter++;
    if (u == t) {
      break;  // Found a path
    }

    std::vector<CellDistancePair> neighbors;
    global_planner->getOpenNeighbors(u, neighbors, is_3D);
    for (const auto& v_cell_dist : neighbors) {
      Cell v = v_cell_dist.first;
      double edge_cost = v_cell_dist.second;  // Use getEdgeCost
      double risk = global_planner->risk_factor_ * global_planner->getRisk(v);
      if (risk > global_planner->risk_factor_ * global_planner->max_cell_risk_ && v != t) {
        continue;
      }
      double new_dist = d + edge_cost + risk;
      double old_dist = getWithDefault(distance, v, INFINITY);
      if (new_dist < old_dist) {
        // Found a better path to v, have to add v to the queue
        parent[v] = u;
        distance[v] = new_dist;
        // TODO: try Dynamic Weighting instead of a constant overestimate_factor
        double heuristic = v.diagDistance2D(t);  // Lower bound for distance on a grid
        heuristic +=
            global_planner->up_cost_ * std::max(0,
                                                t.zIndex() - v.zIndex());  // Minumum cost for increasing altitude
        heuristic += std::max(0, v.zIndex() - t.zIndex());                 // Minumum cost for decreasing altitude
        // if (isNearWall(v)) {
        //   heuristic -= 10;
        // }
        // if (v.diagDistance2D(t) < min_dist_heuristics) {
        //   heuristic -= 20;
        // }
        double overestimated_heuristic = new_dist + global_planner->overestimate_factor_ * heuristic;
        pq.push(std::make_pair(v, overestimated_heuristic));
      }
    }
  }
  double average_iter_time = (std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000000) / num_iter;

  if (seen.find(t) == seen.end()) {
    // No path found
    return false;
  }

  // Get the path by walking from t back to s
  Cell walker = t;
  while (walker != s) {
    path.push_back(walker);
    walker = parent[walker];
  }
  path.push_back(s);
  path.push_back(start_parent);

  std::reverse(path.begin(), path.end());

  // ROS_INFO("Found path with %d iterations, itDistSquared: %.3f", num_iter,
  // num_iter / squared(path.size()));
  printf("%2.2f \t\t %2.2f \t\t %d \t\t %2.2f \t", average_iter_time, global_planner->overestimate_factor_, num_iter,
         distance[t]);

  return true;
}

}  // namespace global_planner
#endif /* GLOBAL_PLANNER_SEARCH_TOOLS_H_ */
