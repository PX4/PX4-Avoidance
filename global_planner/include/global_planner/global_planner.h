#ifndef GLOBAL_PLANNER_GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_GLOBAL_PLANNER_H

#include <math.h>     // abs
#include <algorithm>  // std::reverse
#include <limits>     // numeric_limits
#include <queue>      // std::priority_queue
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>  // getYaw createQuaternionMsgFromYaw

#include <octomap/OcTree.h>
#include <octomap/octomap.h>

#include <global_planner/GlobalPlannerNodeConfig.h>
#include <global_planner/PathWithRiskMsg.h>
#include "global_planner/analysis.h"
#include "global_planner/cell.h"
#include "global_planner/common.h"
#include "global_planner/common_ros.h"
#include "global_planner/node.h"
#include "global_planner/search_tools.h"
#include "global_planner/visitor.h"

namespace global_planner {

class GlobalPlanner {
 public:
  octomap::OcTree* octree_ = NULL;
  // std::vector<double> alt_prior_ {  1.0, 0.1, 0.05, 0.05, 0.05, 0.05, 0.05,
  // 0.05, 0.05, 0.05, 0.05, 0.05}; std::vector<double> alt_prior_ { 0.1, 0.1,
  // 0.1, 0.1, 0.1, 0.1, 0.1,
  //                                   0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  std::vector<double> alt_prior_{1.0,    0.2,   0.1333, 0.1,   0.0833, 0.05,  0.033, 0.025, 0.0166,
                                 0.0125, 0.001, 0.001,  0.001, 0.001,  0.001, 0.001, 0.001, 0.001,
                                 0.001,  0.001, 0.001,  0.001, 0.001,  0.001, 0.001};

  // Needed to quickly estimate the risk of vertical movement
  std::vector<double> accumulated_alt_prior_;  // accumulated_alt_prior_[i] =
                                               // sum(alt_prior_[0:i])

  std::unordered_map<Cell, double> risk_cache_;         // Cache of getRisk(Cell)
  std::unordered_map<Cell, double> bubble_risk_cache_;  // Cache the risk of the safest path from Cell to t
  std::unordered_map<Node, double> heuristic_cache_;    // Cache of
                                                        // getHeuristic(Node) (and
                                                        // later reverse search)
  double bubble_cost_ = 0.0;                            // Minimum risk for the safest path from a cell
                                                        // outside of the bubble to t
  double bubble_radius_ = 0.0;                          // The maximum distance from a cell within the bubble to t

  std::unordered_set<Cell> occupied_;    // Cells which have at some point contained an obstacle point
  std::unordered_set<Cell> path_cells_;  // Cells that are on current path, and may not be blocked

  // TODO: rename and remove not needed
  std::vector<Cell> path_back_;
  geometry_msgs::Point curr_pos_;
  double curr_yaw_;
  geometry_msgs::Vector3 curr_vel_;
  GoalCell goal_pos_ = GoalCell(0.5, 0.5, 3.5);
  bool going_back_ = true;  // we start by just finding the start position

  // Dynamic reconfigure parameters
  int min_altitude_ = 1;
  int max_altitude_ = 10;
  double max_cell_risk_ = 0.2;
  double smooth_factor_ = 10.0;
  double vert_to_hor_cost_ = 1.0;  // The cost of changing between vertical and
                                   // horizontal motion (TODO: use it)
  double risk_factor_ = 500.0;
  double neighbor_risk_flow_ = 1.0;
  double explore_penalty_ = 0.005;
  double up_cost_ = 3.0;
  double down_cost_ = 1.0;
  double search_time_ = 0.5;  // The time it takes to find a path in worst case
  double min_overestimate_factor_ = 1.03;
  double max_overestimate_factor_ = 2.0;
  double risk_threshold_risk_based_speedup_ = 0.5;
  double default_speed_ = 1.0;  // Default speed of flight.
  double max_speed_ = 3.0;      // Maximum speed of flight.
  int max_iterations_ = 2000;
  bool goal_is_blocked_ = false;
  bool current_cell_blocked_ = false;
  bool goal_must_be_free_ = true;  // If false, the planner may try to find a path close to the goal
  bool use_current_yaw_ = true;    // The current orientation is factored into the smoothness
  bool use_risk_heuristics_ = true;
  bool use_speedup_heuristics_ = true;
  bool use_risk_based_speedup_ = true;
  std::string default_node_type_ = "SpeedNode";
  std::string frame_id_ = "world";

  double overestimate_factor_ = max_overestimate_factor_;
  std::vector<Cell> curr_path_;
  PathInfo curr_path_info_;
  SearchVisitor<std::unordered_set<Cell>, std::unordered_map<Cell, double> > visitor_;

  GlobalPlanner();
  ~GlobalPlanner();

  void calculateAccumulatedHeightPrior();

  void setPose(const geometry_msgs::PoseStamped& new_pose);
  void setGoal(const GoalCell& goal);
  void setPath(const std::vector<Cell>& path);
  void setFrame(std::string frame_id);

  void updateFullOctomap(octomap::AbstractOcTree* tree);

  void getOpenNeighbors(const Cell& cell, std::vector<CellDistancePair>& neighbors, bool is_3D);
  bool isNearWall(const Cell& cell);

  double getEdgeDist(const Cell& u, const Cell& v);
  double getSingleCellRisk(const Cell& cell);
  double getAltPrior(const Cell& cell);
  bool isOccupied(const Cell& cell);
  bool isLegal(const Node& node);
  double getRisk(const Cell& cell);
  double getRisk(const Node& node);
  double getRiskOfCurve(const std::vector<geometry_msgs::PoseStamped>& msg);
  double getTurnSmoothness(const Node& u, const Node& v);
  double getEdgeCost(const Node& u, const Node& v);

  double riskHeuristic(const Cell& u, const Cell& goal);
  double riskHeuristicReverseCache(const Cell& u, const Cell& goal);
  double smoothnessHeuristic(const Node& u, const Cell& goal);
  double altitudeHeuristic(const Cell& u, const Cell& goal);
  double getHeuristic(const Node& u, const Cell& goal);

  geometry_msgs::PoseStamped createPoseMsg(const Cell& cell, double yaw);
  nav_msgs::Path getPathMsg();
  nav_msgs::Path getPathMsg(const std::vector<Cell>& path);
  PathWithRiskMsg getPathWithRiskMsg();
  PathInfo getPathInfo(const std::vector<Cell>& path);

  NodePtr getStartNode(const Cell& start, const Cell& parent, const std::string& type);
  bool findPath(std::vector<Cell>& path);

  bool getGlobalPath();
  void goBack();
  void stop();
  void setRobotRadius(double radius);

 private:
  double robot_radius_;
  double octree_resolution_;
};

}  // namespace global_planner

#endif  // GLOBAL_PLANNER_GLOBAL_PLANNER_H
