#ifndef GLOBAL_PLANNER_GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_GLOBAL_PLANNER_H

#include <algorithm>        // std::reverse
#include <limits>           // numeric_limits
#include <math.h>           // abs
#include <queue>            // std::priority_queue
#include <string>
#include <tuple>
#include <unordered_map>  
#include <unordered_set>  

#include <nav_msgs/Path.h>
#include <tf/transform_listener.h> // getYaw createQuaternionMsgFromYaw 

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

#include "avoidance/cell.h"
#include "avoidance/common.h"
#include "avoidance/node.h"

namespace avoidance {

struct PathInfo {
  bool is_blocked;
  double cost;
  double dist;
  double risk;
  double smoothness;
};

class GlobalPlanner {
 public:
  octomap::OcTree* octree_ = NULL;
  // std::vector<double> height_prior_ {  1.0, 0.1, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05};
  // std::vector<double> height_prior_ { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
  //                                   0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  std::vector<double> height_prior_ { 1.0, 0.2, 0.1333, 0.1, 0.833, 0.05, 0.33,
                                    0.025, 0.0166, 0.0125, 0.001, 0.001, 0.001};

  // Needed to quickly estimate the risk of vertical movement
  std::vector<double> accumulated_height_prior_; // accumulated_height_prior_[i] = sum(height_prior_[0:i])

  std::unordered_map<Cell, double> seen_count_;       // number of times a cell was explored in last search
  std::unordered_map<Cell, double> risk_cache_;       // Cache of getRisk(Cell)
  std::unordered_map<Cell, double> bubble_risk_cache_; // Cache the risk of the safest path from Cell to t
  std::unordered_map<Node, double> heuristic_cache_;  // Cache of getHeuristic(Node) (and later reverse search)
  double bubble_cost_ = 0.0;  // Minimum risk for the safest path from a cell outside of the bubble to t
  double bubble_radius_ = 0.0;  // The maximum distance from a cell within the bubble to t

  std::unordered_set<Cell> seen_;        // Cells that were explored in last search
  std::unordered_set<Cell> occupied_;    // Cells which have at some point contained an obstacle point
  std::unordered_set<Cell> path_cells_;   // Cells that are on current path, and may not be blocked

  // TODO: rename and remove not needed
  std::vector<Cell> path_back_;
  geometry_msgs::Point curr_pos_;
  double curr_yaw_;
  geometry_msgs::Vector3 curr_vel_;
  GoalCell goal_pos_ = GoalCell(0.5, 0.5, 3.5);
  bool going_back_ = true;        // we start by just finding the start position

  double overestimate_factor;
  int min_altitude_ = 1;
  int max_altitude_ = 10;
  double max_cell_risk_ = 0.2;
  double smooth_factor_ = 10.0;
  double vert_to_hor_cost_ = 1.0;   // The cost of changing between vertical and horizontal motion (TODO: use it)
  double risk_factor_ = 500.0;
  double neighbor_risk_flow_ = 1.0;
  double expore_penalty_ = 0.005;
  double up_cost_ = 3.0;
  double down_cost_ = 1.0;
  double search_time_ = 0.5;      // The time it takes to find a path in worst case
  int max_iterations_ = 2000;
  int last_iterations_ = 0;
  std::vector<Cell> curr_path_;
  PathInfo curr_path_info_;
  bool goal_is_blocked_ = false;
  bool use_risk_heuristics_ = true;
  bool use_speedup_heuristics_ = true;

  GlobalPlanner();
  ~GlobalPlanner();

  void calculateAccumulatedHeightPrior();

  void setPose(const geometry_msgs::PoseStamped & new_pose);
  void setGoal(const GoalCell & goal);
  void setPath(const std::vector<Cell> & path);

  bool updateFullOctomap(const octomap_msgs::Octomap & msg);
  
  void getOpenNeighbors(const Cell & cell, std::vector<CellDistancePair> & neighbors, bool is_3D);
  bool isNearWall(const Cell & cell);

  double getEdgeDist(const Cell & u, const Cell & v);
  double getSingleCellRisk(const Cell & cell);
  bool isOccupied(const Cell & cell);
  double getRisk(const Cell & cell);
  double getRisk(const Node & node);
  double getTurnSmoothness(const Node & u, const Node & v);
  double getEdgeCost(const Node & u, const Node & v);

  double riskHeuristic(const Cell & u, const Cell & goal);
  double riskHeuristicReverseCache(const Cell & u, const Cell & goal);
  double smoothnessHeuristic(const Node & u, const Cell & goal);
  double altitudeHeuristic(const Cell & u, const Cell & goal);
  double getHeuristic(const Node & u, const Cell & goal);
  
  geometry_msgs::PoseStamped createPoseMsg(const Cell & cell, double yaw);
  nav_msgs::Path getPathMsg();

  PathInfo getPathInfo(const std::vector<Cell> & path);
  void printPathStats(const std::vector<Cell> & path, const Cell & start_parent, const Cell & start,
                      const Cell & goal, double total_cost);
  
  bool findPath(std::vector<Cell> & path);
  bool find2DPath(std::vector<Cell> & path, const Cell & s, const Cell & t, const Cell & start_parent);
  bool reverseSearch(const Cell & t);
  bool findPathOld(std::vector<Cell> & path, const Cell & s, const Cell & t, const Cell & start_parent, bool is_3D);
  bool findSmoothPath(std::vector<Cell> & path, const NodePtr & s, const GoalCell & t);
  
  bool getGlobalPath();
  void goBack();
  void stop();
};

} // namespace avoidance

#endif // GLOBAL_PLANNER_GLOBAL_PLANNER_H
