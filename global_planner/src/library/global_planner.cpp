#include "global_planner/global_planner.h"

namespace global_planner {

// Returns the XY-angle between u and v, or if v is directly above/below u, it
// returns last_yaw
double nextYaw(Cell u, Cell v, double last_yaw) {
  int dx = v.xIndex() - u.xIndex();
  int dy = v.yIndex() - u.yIndex();
  if (dx == 0 && dy == 0) {
    return last_yaw;  // Going up or down
  }
  return std::atan2(dy, dx);
}

GlobalPlanner::GlobalPlanner() { calculateAccumulatedHeightPrior(); }
GlobalPlanner::~GlobalPlanner() {}

// Fills accumulated_alt_prior_ such that accumulated_alt_prior_[i] =
// sum(alt_prior_[0:i]) Used to get the pior risk of vertical movement
void GlobalPlanner::calculateAccumulatedHeightPrior() {
  double sum = 0.0;
  for (double p : alt_prior_) {
    sum += p;
    accumulated_alt_prior_.push_back(sum);
  }
}

// Updates the current pose and keeps track of the path back
void GlobalPlanner::setPose(const geometry_msgs::PoseStamped& new_pose) {
  curr_pos_ = new_pose.pose.position;
  curr_yaw_ = tf::getYaw(new_pose.pose.orientation);
  Cell curr_cell = Cell(curr_pos_);
  if (!going_back_ && (path_back_.empty() || curr_cell != path_back_.back())) {
    // Keep track of where we have been, add current position to path_back_ if
    // it is different from last one
    path_back_.push_back(curr_cell);
  }
}

// Sets a new mission goal
void GlobalPlanner::setGoal(const GoalCell& goal) {
  goal_pos_ = goal;
  going_back_ = false;
  goal_is_blocked_ = false;
  heuristic_cache_.clear();
  bubble_risk_cache_.clear();
}

// Sets path to be the current path
void GlobalPlanner::setPath(const std::vector<Cell>& path) {
  curr_path_info_ = getPathInfo(path);
  curr_path_ = path;

  path_cells_.clear();
  for (int i = 2; i < path.size(); ++i) {
    Node node(path[i], path[i - 1]);
    for (const Cell& cell : node.getCells()) {
      path_cells_.insert(cell);
    }
  }
}

void GlobalPlanner::setFrame(std::string frame_id) { frame_id_ = frame_id; }

// Returns false iff current path has an obstacle
// Going through the octomap can take more than 50 ms for 100m x 100m explored
// map
void GlobalPlanner::updateFullOctomap(octomap::AbstractOcTree* tree) {
  risk_cache_.clear();
  if (octree_) {
    delete octree_;
  }
  octree_ = dynamic_cast<octomap::OcTree*>(tree);
  octree_resolution_ = octree_->getResolution();
}

// TODO: simplify and return neighbors
// Fills neighbors with the 8 horizontal and 2 vertical non-occupied neigbors
void GlobalPlanner::getOpenNeighbors(const Cell& cell, std::vector<CellDistancePair>& neighbors, bool is_3D) {
  // It's long because it uses the minimum number of 'if's
  double x = cell.xIndex();
  double y = cell.yIndex();
  double z = cell.zIndex();
  Cell forw = Cell(std::tuple<int, int, int>(x + 1, y, z));
  Cell back = Cell(std::tuple<int, int, int>(x - 1, y, z));
  Cell left = Cell(std::tuple<int, int, int>(x, y - 1, z));
  Cell righ = Cell(std::tuple<int, int, int>(x, y + 1, z));
  Cell forw_left = Cell(std::tuple<int, int, int>(x + 1, y - 1, z));
  Cell forw_right = Cell(std::tuple<int, int, int>(x + 1, y + 1, z));
  Cell back_left = Cell(std::tuple<int, int, int>(x - 1, y - 1, z));
  Cell back_right = Cell(std::tuple<int, int, int>(x - 1, y + 1, z));
  Cell up = Cell(std::tuple<int, int, int>(x, y, z + 1));
  Cell down = Cell(std::tuple<int, int, int>(x, y, z - 1));

  neighbors.push_back(std::make_pair(forw, 1.0));
  neighbors.push_back(std::make_pair(forw_left, 1.41));
  neighbors.push_back(std::make_pair(forw_right, 1.41));
  neighbors.push_back(std::make_pair(back, 1.0));
  neighbors.push_back(std::make_pair(back_left, 1.41));
  neighbors.push_back(std::make_pair(back_right, 1.41));
  neighbors.push_back(std::make_pair(left, 1.0));
  neighbors.push_back(std::make_pair(righ, 1.0));
  // Vertical neighbors
  if (is_3D && z < max_altitude_) {
    neighbors.push_back(std::make_pair(up, up_cost_));
  }
  if (is_3D && z > min_altitude_) {
    neighbors.push_back(std::make_pair(down, down_cost_));
  }
}

// Returns true if cell has an occupied neighbor
bool GlobalPlanner::isNearWall(const Cell& cell) {
  for (const Cell& neighbor : cell.getDiagonalNeighbors()) {
    if (isOccupied(neighbor)) {
      return true;
    }
  }
  return false;
}

// The distance between two adjacent cells
double GlobalPlanner::getEdgeDist(const Cell& u, const Cell& v) {
  double xy_diff = u.distance2D(v);
  double z_diff = v.zPos() - u.zPos();
  double up_diff = up_cost_ * std::max(z_diff, 0.0);
  double down_diff = down_cost_ * std::max(-z_diff, 0.0);
  return xy_diff + up_diff + down_diff;
}

// Risk without looking at the neighbors
double GlobalPlanner::getSingleCellRisk(const Cell& cell) {
  if (cell.zIndex() < 1 || !octree_) {
    return 1.0;  // Octomap does not keep track of the ground
  }
  // octomap::OcTreeNode* node = octree_->search(cell.xPos(), cell.yPos(),
  // cell.zPos());
  int octree_depth = std::min(16, 17 - int(CELL_SCALE + 0.1));
  octomap::OcTreeNode* node = octree_->search(cell.xPos(), cell.yPos(), cell.zPos(), octree_depth);
  if (node) {
    // TODO: posterior in log-space
    double log_odds = node->getValue();
    // double parentLogOdds = parent->getValue();
    double post_prob = posterior(getAltPrior(cell), octomap::probability(log_odds));
    // double post_prob = posterior(0.06, octomap::probability(log_odds));
    // // If the cell has been seen
    if (occupied_.find(cell) != occupied_.end()) {
      // If an obstacle has at some point been spotted it is 'known space'
      return post_prob;
    } else if (log_odds > 0) {
      // ROS_INFO("Cell %s is not in occupied_ but has > 50\% risk",
      // cell.asString().c_str());
      return post_prob;
    }
    // No obstacle spotted (all measurements hint towards it being free)
    return explore_penalty_ * post_prob;
  }
  // No measurements at all
  return explore_penalty_ * getAltPrior(cell);  // Risk for unexplored cells
}

double GlobalPlanner::getAltPrior(const Cell& cell) {
  int index = std::round(cell.zPos());

  if (index > alt_prior_.size() - 1) {
    return alt_prior_.back();
  } else if (index < 0) {
    return alt_prior_.front();
  } else {
    return alt_prior_.at(index);
  }
}

bool GlobalPlanner::isOccupied(const Cell& cell) { return getSingleCellRisk(cell) > 0.5; }

bool GlobalPlanner::isLegal(const Node& node) {
  return node.cell_.zPos() < max_altitude_ && getRisk(node) < max_cell_risk_;
}

double GlobalPlanner::getRisk(const Cell& cell) {
  if (risk_cache_.find(cell) != risk_cache_.end()) {
    return risk_cache_[cell];
  }

  double risk = getSingleCellRisk(cell);
  int radius = static_cast<int>(std::ceil(robot_radius_ / octree_resolution_));
  for (const Cell& neighbor : cell.getFlowNeighbors(radius)) {
    risk += neighbor_risk_flow_ * getSingleCellRisk(neighbor);
  }

  risk_cache_[cell] = risk;
  return risk;
}

double GlobalPlanner::getRisk(const Node& node) {
  double risk = 0.0;
  std::unordered_set<Cell> nodeCells = node.getCells();
  for (const Cell& cell : nodeCells) {
    risk += getRisk(cell);
  }
  return risk / nodeCells.size() * node.getLength();
}

// Returns the risk of the quadratic Bezier curve defined by poses
// TODO: think about this
double GlobalPlanner::getRiskOfCurve(const std::vector<geometry_msgs::PoseStamped>& msg) {
  if (msg.size() != 3) {
    ROS_INFO("Bezier msg must have 3 points");
    return -1;
  }

  // Estimate the length of the curve, TODO: check if this makes sense
  double length = distance(msg[0], msg[1]) + distance(msg[1], msg[2]);
  int num_steps = std::ceil(length / CELL_SCALE);
  // Calculate points on the curve, around one point per cell
  auto points = threePointBezier(msg[0].pose.position, msg[1].pose.position, msg[2].pose.position, 4 * num_steps);

  double risk = 0.0;
  visitor_.seen_.clear();
  for (auto point : points) {
    risk += getRisk(Cell(point));
    visitor_.seen_.insert(Cell(point));
  }
  return risk;
}

// Returns the amount of rotation needed to go from u to v
double GlobalPlanner::getTurnSmoothness(const Node& u, const Node& v) {
  double turn = u.getRotation(v);
  return turn * turn;  // Squaring makes large turns more costly
}

// Returns the total cost of the edge from u to v
double GlobalPlanner::getEdgeCost(const Node& u, const Node& v) {
  double dist_cost = getEdgeDist(u.cell_, v.cell_);
  // double risk_cost = u.cell_.distance3D(v.cell_) * risk_factor_ *
  // getRisk(v.cell_);
  double risk_cost = risk_factor_ * getRisk(v);
  double smooth_cost = smooth_factor_ * getTurnSmoothness(u, v);
  if (u.cell_.distance3D(Cell(curr_pos_)) < 3 && norm(curr_vel_) > 1) {
    smooth_cost *= 2;  // Penalty for a early turn when the velocity is high
  }
  return dist_cost + risk_cost + smooth_cost;
}

// Returns a heuristic for the cost of risk for going from u to goal
// The heuristic is the cost of risk through unknown environment
double GlobalPlanner::riskHeuristic(const Cell& u, const Cell& goal) {
  // return riskHeuristicReverseCache(u, goal);  // REVERSE_SEARCH

  if (u == goal) {
    return 0.0;
  }
  double unexplored_risk = (1.0 + 6.0 * neighbor_risk_flow_) * explore_penalty_ * risk_factor_;
  double xy_dist = u.diagDistance2D(goal) - 1.0;  // XY distance excluding the goal cell
  double xy_risk = xy_dist * unexplored_risk * getAltPrior(u);
  double z_risk =
      unexplored_risk * std::abs(accumulated_alt_prior_[u.zIndex()] - accumulated_alt_prior_[goal.zIndex()]);
  // TODO: instead of subtracting 1 from the xy_dist, subtract 1 from the
  // combined xy and z dist
  double goal_risk = getRisk(goal) * risk_factor_;
  return xy_risk + z_risk + goal_risk;
}

double GlobalPlanner::riskHeuristicReverseCache(const Cell& u, const Cell& goal) {
  if (bubble_risk_cache_.find(u) != bubble_risk_cache_.end()) {
    return bubble_risk_cache_[u];
  }
  if (u == goal) {
    return 0.0;
  }
  double dist_to_bubble = std::max(0.0, u.diagDistance3D(goal) - bubble_radius_);
  double unexplored_risk = (1.0 + 6.0 * neighbor_risk_flow_) * explore_penalty_ * risk_factor_;
  double heuristic = bubble_cost_ + dist_to_bubble * unexplored_risk * getAltPrior(u);
  // bubble_risk_cache_[u] = heuristic;
  return heuristic;
}

// Returns a heuristic for the cost of turning for going from u to goal
double GlobalPlanner::smoothnessHeuristic(const Node& u, const Cell& goal) {
  if (u.cell_.xIndex() == goal.xIndex() && u.cell_.yIndex() == goal.yIndex()) {
    return 0.0;  // directly above or below the goal
  }
  if (u.cell_.xIndex() == u.parent_.xIndex() && u.cell_.yIndex() == u.parent_.yIndex()) {
    // Vertical motion not directly above or below the goal, must change to
    // horizontal movement
    return smooth_factor_ * vert_to_hor_cost_;
  }

  double u_ang = (u.cell_ - u.parent_).angle();  // Current orientation
  double goal_ang = (goal - u.cell_).angle();    // Direction of goal
  double ang_diff = goal_ang - u_ang;
  ang_diff = std::fabs(angleToRange(ang_diff));     // Rotation needed
  double num_45_deg_turns = ang_diff / (M_PI / 4);  // Minimum number of 45-turns to goal

  // If there is height difference we also need to change to vertical movement
  // at least once
  // TODO: simplify
  int altitude_change = (u.cell_.zIndex() == goal.zIndex()) ? 0 : 1;

  return smooth_factor_ * (num_45_deg_turns + altitude_change);
}

// Returns a heuristic for the cost of reaching the altitude of goal
double GlobalPlanner::altitudeHeuristic(const Cell& u, const Cell& goal) {
  double diff = goal.zIndex() - u.zIndex();
  // Either multiply by up_cost_ or down_cost_, depending on if we are belove or
  // above the goal
  double cost = (diff > 0) ? (up_cost_ * std::abs(diff)) : (down_cost_ * std::abs(diff));
  return cost;
}

// Returns a heuristic of going from u to goal
double GlobalPlanner::getHeuristic(const Node& u, const Cell& goal) {
  if (heuristic_cache_.find(u) != heuristic_cache_.end()) {
    // return heuristic_cache_[u];    // TODO: Needs to account for different
    // overestimating factors
  }

  // Only overestimate the distance
  double heuristic = overestimate_factor_ * u.cell_.diagDistance2D(goal);
  heuristic += altitudeHeuristic(u.cell_, goal);  // Lower bound cost due to altitude change
  heuristic += smoothnessHeuristic(u, goal);      // Lower bound cost due to turning
  if (use_risk_heuristics_) {
    heuristic += riskHeuristic(u.cell_, goal);  // Risk through a straight-line path of unexplored space
  }
  if (use_speedup_heuristics_) {
    heuristic += visitor_.seen_count_[u.cell_];
  }
  heuristic_cache_[u] = heuristic;
  return heuristic;
}

geometry_msgs::PoseStamped GlobalPlanner::createPoseMsg(const Cell& cell, double yaw) {
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = frame_id_;
  pose_msg.pose.position = cell.toPoint();
  pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  return pose_msg;
}

nav_msgs::Path GlobalPlanner::getPathMsg() { return getPathMsg(curr_path_); }

nav_msgs::Path GlobalPlanner::getPathMsg(const std::vector<Cell>& path) {
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = frame_id_;

  if (path.size() == 0) {
    return path_msg;
  }

  // Use actual position instead of the center of the cell
  double last_yaw = curr_yaw_;

  for (int i = 0; i < path.size() - 1; ++i) {
    Cell p = path[i];
    double new_yaw = nextYaw(p, path[i + 1], last_yaw);
    // if (new_yaw != last_yaw) {   // only publish corner points
    path_msg.poses.push_back(createPoseMsg(p, new_yaw));
    // }
    last_yaw = new_yaw;
  }
  Cell last_point = path[path.size() - 1];  // Last point should have the same yaw as the previous point
  path_msg.poses.push_back(createPoseMsg(last_point, last_yaw));
  return path_msg;
}

PathWithRiskMsg GlobalPlanner::getPathWithRiskMsg() {
  nav_msgs::Path path_msg = getPathMsg();
  PathWithRiskMsg risk_msg;
  risk_msg.header = path_msg.header;
  risk_msg.poses = path_msg.poses;

  for (const auto& pose : path_msg.poses) {
    double risk = getRisk(Cell(pose.pose.position));
    risk_msg.risks.push_back(risk);
  }
  return risk_msg;
}

// Returns details of the cost of the path
PathInfo GlobalPlanner::getPathInfo(const std::vector<Cell>& path) {
  PathInfo path_info = {};
  for (int i = 2; i < path.size(); ++i) {
    Node curr_node = Node(path[i], path[i - 1]);
    Node last_node = Node(path[i - 1], path[i - 2]);
    double cell_risk = getRisk(curr_node);
    path_info.dist += getEdgeDist(last_node.cell_, curr_node.cell_);
    path_info.risk += risk_factor_ * cell_risk;
    path_info.cost += getEdgeCost(last_node, curr_node);
    path_info.is_blocked |= cell_risk > max_cell_risk_;
    path_info.smoothness += smooth_factor_ * getTurnSmoothness(last_node, curr_node);
  }
  return path_info;
}

// Chooses the node-type of the search
NodePtr GlobalPlanner::getStartNode(const Cell& start, const Cell& parent, const std::string& type) {
  if (type == "Node") {
    return NodePtr(new Node(start, parent));
  }
  if (type == "NodeWithoutSmooth") {
    return NodePtr(new NodeWithoutSmooth(start, parent));
  }
  if (type == "SpeedNode") {
    return NodePtr(new SpeedNode(start, parent));
  } else {
    return NodePtr(new Node(start, parent));
  }
}

// Calls different search functions to find a path
bool GlobalPlanner::findPath(std::vector<Cell>& path) {
  // Start from a position thats a bit ahead [s = curr_pos + (search_time_ *
  // curr_vel_)]
  Cell s(addPoints(curr_pos_, scalePoint(curr_vel_, search_time_)));
  GoalCell t = goal_pos_;
  // Cell parent_of_s = s.getNeighborFromYaw(curr_yaw_ + M_PI); // The cell
  // behind the start cell Cell parent_of_s = Cell(curr_pos_);
  Cell parent_of_s(subtractPoints(curr_pos_, scalePoint(curr_vel_, search_time_)));
  if (!use_current_yaw_) {
    Cell parent_of_s = s;  // Ignore the current yaw
  }

  ROS_INFO("Planning a path from %s to %s", s.asString().c_str(), t.asString().c_str());
  ROS_INFO("curr_pos_: %2.2f,%2.2f,%2.2f\t s: %2.2f,%2.2f,%2.2f", curr_pos_.x, curr_pos_.y, curr_pos_.z, s.xPos(),
           s.yPos(), s.zPos());

  bool found_path = false;
  double best_path_cost = INFINITY;
  overestimate_factor_ = max_overestimate_factor_;
  int iter_left = max_iterations_;
  int last_iter = 0;

  // reverseSearch(t); // REVERSE_SEARCH

  printf("Search              iter_time overest   num_iter  path_cost \n");
  while (overestimate_factor_ >= min_overestimate_factor_ && iter_left > last_iter) {
    std::vector<Cell> new_path;
    SearchInfo search_info;
    std::string node_type = default_node_type_;
    if (overestimate_factor_ > 1.5) {
      // Use a cheap search for higher overestimate
      // found_new_path = findPathOld(new_path, s, t, parent_of_s, true);  // No
      // need to search with smoothness
      node_type = "NodeWithoutSmooth";
    }

    NodePtr start_node = getStartNode(s, parent_of_s, node_type);
    search_info = findSmoothPath(this, new_path, start_node, t, iter_left, visitor_);
    printSearchInfo(search_info, node_type, overestimate_factor_);

    if (search_info.found_path) {
      PathInfo path_info = getPathInfo(new_path);
      printf("(cost: %2.2f, dist: %2.2f, risk: %2.2f, smooth: %2.2f) \n", path_info.cost, path_info.dist,
             path_info.risk, path_info.smoothness);
      if (true || path_info.cost <= best_path_cost) {
        // TODO: always use the newest path?
        best_path_cost = path_info.cost;
        path = new_path;
        found_path = true;
      }
    } else {
      break;
    }
    last_iter = search_info.num_iter;
    iter_left -= last_iter;
    overestimate_factor_ = (overestimate_factor_ - 1.0) / 4.0 + 1.0;
  }

  // Last resort, try 2d search at max_altitude_
  if (!found_path) {
    printf("No path found, search in 2D \n");
    max_iterations_ = 5000;
    found_path = find2DPath(this, path, s, t, parent_of_s, max_altitude_);
  }

  return found_path;
}

// Returns true iff a path needs to be published, either a new path or a path
// back The path is then stored in this.pathMsg
bool GlobalPlanner::getGlobalPath() {
  Cell s = Cell(curr_pos_);
  Cell t = Cell(goal_pos_);
  current_cell_blocked_ = isOccupied(s);

  if (goal_must_be_free_ && getRisk(t) > max_cell_risk_) {
    // If goal is occupied, no path is published
    ROS_INFO("Goal position is occupied");
    goal_is_blocked_ = true;
    return false;
  } else if (current_cell_blocked_) {
    // If current position is occupied the way back is published
    ROS_INFO("Current position is occupied, going back.");
    // goBack();
    // return true;
    return false;
  } else {
    // Both current position and goal are free, try to find a path
    std::vector<Cell> path;
    if (!findPath(path)) {
      double goal_risk = getRisk(t);
      ROS_INFO("  Failed to find a path, risk of t: %3.2f", goal_risk);
      goal_is_blocked_ = true;
      return false;
    }
    setPath(path);
    return true;
  }
}

// Sets the current path to be the path back until a safe cell is reached
// Then the mission can be tried again or a new mission can be set
void GlobalPlanner::goBack() {
  ROS_INFO("  GO BACK ");
  going_back_ = true;
  std::vector<Cell> new_path = path_back_;
  std::reverse(new_path.begin(), new_path.end());

  // Follow the path back until the risk is low
  for (int i = 1; i < new_path.size() - 1; ++i) {
    if (i > 5 && getRisk(new_path[i]) < 0.5) {
      new_path.resize(i + 1);                        // new_path is the last i+1 positions of path_back_
      path_back_.resize(path_back_.size() - i - 2);  // Remove part of path_back_ that is also in new_path
      break;
    }
  }
  curr_path_ = new_path;
  goal_pos_ = GoalCell(new_path[new_path.size() - 1], 1.0);
}

void GlobalPlanner::stop() {
  setGoal(GoalCell(curr_pos_));
  setPath({curr_pos_});
}

void GlobalPlanner::setRobotRadius(double radius) { robot_radius_ = radius; }

}  // namespace global_planner
