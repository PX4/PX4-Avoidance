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
  bool foundPath;
  double cost;
  double dist;
  double risk;
  double smoothness;
};

class GlobalPlanner {
 public:
  octomap::OcTree* octree = NULL;
  std::vector<double> heightPrior { 1.0, 0.5, 0.3, 0.2, 0.1, 0.05, 0.01,
                                    0.01, 0.01, 0.01, 0.01, 0.01, 0.01 };
  // std::vector<double> heightPrior { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
  //                                   0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  // std::vector<double> heightPrior { 1.0, 0.2, 0.1333, 0.1, 0.833, 0.05, 0.33,
  //                                   0.025, 0.0166, 0.0125, 0.001, 0.001, 0.001};

  // Needed to quickly estimate the risk of vertical movement
  std::vector<double> accumulatedHeightPrior; // accumulatedHeightPrior[i] = sum(heightPrior[0:i])

  std::unordered_map<Cell, double> occProb;   // OctoMap probability of Cell being occupied
  std::unordered_map<Cell, double> seenCount; // number of times a cell was explored in last search
  std::unordered_map<Cell, double> riskCache; // Cache of getRisk(Cell)
  std::unordered_map<Node, double> heuristicCache; // Cache of getHeuristic(Node) (and later reverse search)

  std::unordered_set<Cell> seen;        // Cells that were explored in last search
  std::unordered_set<Cell> occupied;    // Cells such that occProp[Cell] > maxPathProp
  std::unordered_set<Cell> pathCells;   // Cells that are on current path, and may not be blocked

  // TODO: rename and remove not needed
  std::vector<Cell> pathBack;
  geometry_msgs::Point currPos;
  double currYaw;
  geometry_msgs::Vector3 currVel;
  Cell goalPos = Cell(0.5, 0.5, 3.5);
  bool goingBack = true;        // we start by just finding the start position

  double overEstimateFactor = 4.0;
  int minHeight = 1;
  int maxHeight = 10;
  double maxPathProb = 0.0;
  double maxBailProb = 1.0;     // Must be >= 0 (50%) because of the fixed uniform prior in OctoMap
  double maxCellRisk = 0.5;
  double smoothFactor = 5.0;
  double vertToHorCost = 1.0;   // The cost of changing between vertical and horizontal motion
  double riskFactor = 200.0;
  double neighborRiskFlow = 1.0;
  double explorePenalty = 0.015;
  double upCost = 3.0;
  double downCost = 1.0;
  double searchTime = 1.0;      // The time it takes to find a path in worst case
  double inf = std::numeric_limits<double>::infinity();
  int maxIterations = 2000;
  int lastIterations = 0;
  std::vector<Cell> lastPath;   // should be currPath
  PathInfo lastPathInfo;
  bool goalIsBlocked = false;
  bool useRiskHeuristics = true;
  bool useSpeedUpHeuristics = true;

  GlobalPlanner();
  ~GlobalPlanner();

  void calculateAccumulatedHeightPrior();

  void setPose(const geometry_msgs::PoseStamped & newPose);
  void setGoal(const Cell & goal);
  void setPath(const std::vector<Cell> & path);

  bool updateFullOctomap(const octomap_msgs::Octomap & msg);
  
  void getOpenNeighbors(const Cell & cell, std::vector<CellDistancePair> & neighbors, bool is3D);
  bool isNearWall(const Cell & cell);

  double getEdgeDist(const Cell & u, const Cell & v);
  double getSingleCellRisk(const Cell & cell);
  bool isOccupied(const Cell & cell);
  double getRisk(const Cell & cell);
  double getTurnSmoothness(const Node & u, const Node & v);
  double getEdgeCost(const Node & u, const Node & v);

  double riskHeuristic(const Cell & u, const Cell & goal);
  double smoothnessHeuristic(const Node & u, const Cell & goal);
  double altitudeHeuristic(const Cell & u, const Cell & goal);
  double getHeuristic(const Node & u, const Cell & goal);
  
  geometry_msgs::PoseStamped createPoseMsg(const Cell cell, double yaw);
  nav_msgs::Path getPathMsg();

  PathInfo getPathInfo(const std::vector<Cell> & path, const Node lastNode);
  void printPathStats(const std::vector<Cell> & path, const Cell startParent, const Cell start,
                                   const Cell goal, double totalDistance, std::map<Node, double> & distance);
  
  bool FindPath(std::vector<Cell> & path);
  bool Find2DPath(std::vector<Cell> & path, const Cell & s, Cell t);
  bool FindPathOld(std::vector<Cell> & path, const Cell & s, const Cell t, bool is3D);
  bool FindSmoothPath(std::vector<Cell> & path, const Cell & s, const Cell & t, const Cell & parent);
  
  bool getGlobalPath();
  void goBack();
};

} // namespace avoidance

#endif // GLOBAL_PLANNER_GLOBAL_PLANNER_H
