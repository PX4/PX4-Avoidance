#include "avoidance/global_planner.h"

namespace avoidance {

// Returns the XY-angle between u and v, or if v is directly above/below u, it returns lastYaw 
double nextYaw(Cell u, Cell v, double lastYaw) {
  int dx = v.x() - u.x();
  int dy = v.y() - u.y();
  if (dx == 0 && dy == 0) {
    return lastYaw;   // Going up or down
  }
  return atan2(dy, dx);
}

GlobalPlanner::GlobalPlanner()  {
  calculateAccumulatedHeightPrior();
}
GlobalPlanner::~GlobalPlanner() {}


// Fills accumulatedHeightPrior such that accumulatedHeightPrior[i] = sum(heightPrior[0:i])
// Used to get the pior risk of vertical movement
void GlobalPlanner::calculateAccumulatedHeightPrior() {
  double sum = 0.0;
  for (double p : heightPrior) {
    sum += p;
    accumulatedHeightPrior.push_back(sum);
  }
}

// Updates the current pose and keeps track of the path back
void GlobalPlanner::setPose(const geometry_msgs::PoseStamped & newPose) {
  currPos = newPose.pose.position;
  currYaw = tf::getYaw(newPose.pose.orientation);
  Cell currCell = Cell(currPos);
  if (!goingBack && (pathBack.empty() || currCell != pathBack.back())) {
    // Keep track of where we have been, add current position to pathBack if it is different from last one
    pathBack.push_back(currCell);
  }
}

// Sets a new mission goal, not used for temporary goals, e.g. goBack()
void GlobalPlanner::setGoal(const Cell & goal) {
  goalPos = goal;
  goingBack = false;
  goalIsBlocked = false;
  heuristicCache.clear();
  bubbleRiskCache.clear();
}

// Sets path to be the current path
void GlobalPlanner::setPath(const std::vector<Cell> & path) {
  currPathInfo = getPathInfo(path);
  currPath = path;

  pathCells.clear();
  for (int i=2; i < path.size(); ++i) {
    Cell p = path[i];
    Cell lastP = path[i-1];

    pathCells.insert(p);
    if (p.x() != lastP.x() && p.y() != lastP.y()) {
      // For diagonal edges we need the two common neighbors of p and lastP to be non occupied
      pathCells.insert(Cell(p.xPos(), lastP.yPos(), p.zPos()));
      pathCells.insert(Cell(lastP.xPos(), p.yPos(), p.zPos()));
    }
  }
}


// Returns false iff current path has an obstacle
// Going through the octomap can take more than 50 ms for 100m x 100m explored map 
bool GlobalPlanner::updateFullOctomap(const octomap_msgs::Octomap & msg) {
  riskCache.clear();
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(msg);
  if (octree) {
    delete octree;
  }
  octree = dynamic_cast<octomap::OcTree*>(tree);

  // Check if the risk of the current path has increased 
  if (!currPath.empty()) {
    PathInfo newInfo = getPathInfo(currPath);
    if (newInfo.isBlocked || newInfo.risk > currPathInfo.risk + 10) {
      ROS_INFO("Risk increase");
      return false;
    }
  }
  return true;
}

// TODO: simplify and return neighbors
// Fills neighbors with the 8 horizontal and 2 vertical non-occupied neigbors
void GlobalPlanner::getOpenNeighbors(const Cell & cell, 
                                     std::vector<CellDistancePair> & neighbors,
                                     bool is3D) {
  // It's long because it uses the minimum number of 'if's 
  double x = cell.x();
  double y = cell.y();
  double z = cell.z();
  Cell forw = Cell(std::tuple<int,int,int>(x+1, y, z));
  Cell back = Cell(std::tuple<int,int,int>(x-1, y, z));
  Cell left = Cell(std::tuple<int,int,int>(x, y-1, z));
  Cell righ = Cell(std::tuple<int,int,int>(x, y+1, z));
  Cell forwLeft = Cell(std::tuple<int,int,int>(x+1, y-1, z));
  Cell forwRigh = Cell(std::tuple<int,int,int>(x+1, y+1, z));
  Cell backLeft = Cell(std::tuple<int,int,int>(x-1, y-1, z));
  Cell backRigh = Cell(std::tuple<int,int,int>(x-1, y+1, z));
  Cell up = Cell(std::tuple<int,int,int>(x, y, z+1));
  Cell down = Cell(std::tuple<int,int,int>(x, y, z-1));

  // bool forwOpen = occupied.find(forw) == occupied.end();
  // bool backOpen = occupied.find(back) == occupied.end();
  // bool leftOpen = occupied.find(left) == occupied.end();
  // bool righOpen = occupied.find(righ) == occupied.end();
  // bool forwLeftOpen = occupied.find(forwLeft) == occupied.end();
  // bool forwRighOpen = occupied.find(forwRigh) == occupied.end();
  // bool backLeftOpen = occupied.find(backLeft) == occupied.end();
  // bool backRighOpen = occupied.find(backRigh) == occupied.end();
  // bool upOpen = occupied.find(up) == occupied.end();
  // bool downOpen = occupied.find(down) == occupied.end();

  neighbors.push_back(std::make_pair(forw, 1.0));
  neighbors.push_back(std::make_pair(forwLeft, 1.41));
  neighbors.push_back(std::make_pair(forwRigh, 1.41));
  neighbors.push_back(std::make_pair(back, 1.0));
  neighbors.push_back(std::make_pair(backLeft, 1.41));
  neighbors.push_back(std::make_pair(backRigh, 1.41));
  neighbors.push_back(std::make_pair(left, 1.0));
  neighbors.push_back(std::make_pair(righ, 1.0));
  // Vertical neighbors
  if (is3D && z < maxHeight) {
    neighbors.push_back(std::make_pair(up, upCost));
  }
  if (is3D && z > minHeight) {
    neighbors.push_back(std::make_pair(down, downCost));
  }
}

// Returns true if cell has an occupied neighbor 
bool GlobalPlanner::isNearWall(const Cell & cell){
  for (Cell neighbor : cell.getDiagonalNeighbors()) {
    if (isOccupied(neighbor)) {
      return true;
    }
  }
  return false;
}

// The distance between two adjacent cells
double GlobalPlanner::getEdgeDist(const Cell & u, const Cell & v) {
  double zDiff = v.zPos() - u.zPos();
  double xyDiff = u.distance2D(v);
  if (zDiff > 0) {
    return xyDiff + zDiff * upCost;
  }
  return xyDiff + std::abs(zDiff) * downCost; 
}

// Risk without looking at the neighbors
double GlobalPlanner::getSingleCellRisk(const Cell & cell) {
  if (cell.z() < 1) {
    return 1.0;   // Octomap does not keep track of the ground
  }
  octomap::OcTreeNode* node = octree->search(cell.xPos(), cell.yPos(), cell.zPos());
  // octomap::OcTreeNode* parent = octree->search(cell.xPos(), cell.yPos(), cell.zPos(), 15);
  // if (occProb.find(cell) != occProb.end()) {
  if (node) {
    // TODO: update in log-space
    // double logOdds = occProb[cell];
    double logOdds = node->getValue();
    // double parentLogOdds = parent->getValue();
    return posterior(heightPrior[cell.z()], octomap::probability(logOdds));     // If the cell has been seen
    // return posterior(0.06, octomap::probability(logOdds));     // If the cell has been seen
  }
  return explorePenalty * heightPrior[cell.z()];    // Risk for unexplored cells
}

bool GlobalPlanner::isOccupied(const Cell & cell) {
  return getSingleCellRisk(cell) > 0.5;
}


// Returns the risk from the cell, its neighbors and the prior
double GlobalPlanner::getRisk(const Cell & cell) {
  if (riskCache.find(cell) != riskCache.end()) {
    return riskCache[cell];
  }

  double risk = getSingleCellRisk(cell);
  for (Cell neighbor : cell.getFlowNeighbors()) {
    risk += neighborRiskFlow * getSingleCellRisk(neighbor);
  }

  riskCache[cell] = risk;  
  return risk;      
}

// Returns the amount of rotation needed to go from u to v
double GlobalPlanner::getTurnSmoothness(const Node & u, const Node & v) {
  Cell uDiff = u.cell - u.parent;
  Cell vDiff = v.cell - v.parent;

  int num45DegTurns;
  if (uDiff.x() == 0 && uDiff.y() == 0 && vDiff.x() == 0 && vDiff.y() == 0) {
    num45DegTurns = 0;    // Maintaining vertical motion 
  }
  else if ((uDiff.x() == 0 && uDiff.y() == 0) || (vDiff.x() == 0 && vDiff.y() == 0)) {
    num45DegTurns = vertToHorCost;    // Starting or ending vertical motion
  }
  else if (uDiff.x() == -vDiff.x() && uDiff.y() == -vDiff.y()){
    num45DegTurns = 4;    // 180 degrees, the formula below doesn't work for this case 
  }
  else {
    num45DegTurns = std::abs(uDiff.x() - vDiff.x()) + std::abs(uDiff.y() - vDiff.y());
  }
  return num45DegTurns * num45DegTurns;     // Squaring makes large turns more costly
}

// Returns the total cost of the edge from u to v
double GlobalPlanner::getEdgeCost(const Node & u, const Node & v) {
  double distCost = getEdgeDist(u.cell, v.cell);
  double riskCost = u.cell.distance3D(v.cell) * riskFactor * getRisk(v.cell);
  double smoothCost = smoothFactor * getTurnSmoothness(u, v);
  if (u.cell.distance3D(Cell(currPos)) < 3 && norm(currVel) > 1){
    smoothCost *= 2;
  }
  return distCost + riskCost + smoothCost;
}


// Returns a heuristic for the cost of risk for going from u to goal
// The heuristic is the cost of risk through unknown environment
double GlobalPlanner::riskHeuristic(const Cell & u, const Cell & goal) {
  return riskHeuristicReverseCache(u, goal);  // REVERSE_SEARCH

  if (u == goal) {
    return 0.0;
  }  
  double unexploredRisk = (1.0 + 6.0 * neighborRiskFlow) * explorePenalty * riskFactor;  
  double xyDist = u.diagDistance2D(goal) - 1.0;   // XY distance excluding the goal cell
  double xyRisk = xyDist * unexploredRisk * heightPrior[u.z()];
  double zRisk = std::abs(accumulatedHeightPrior[u.z()] - accumulatedHeightPrior[goal.z()]) * unexploredRisk;
  // TODO: instead of subtracting 1 from the xyDist, subtract 1 from the combined xy and z dist
  double goalRisk = getRisk(goal) * riskFactor;
  return xyRisk + zRisk + goalRisk;
}

double GlobalPlanner::riskHeuristicReverseCache(const Cell & u, const Cell & goal) {
  if (bubbleRiskCache.find(u) != bubbleRiskCache.end()) {
    return bubbleRiskCache[u];
  }
  if (u == goal) {
    return 0.0;
  }
  double distToBubble = std::max(0.0, u.diagDistance3D(goal) - bubbleRadius);
  double unexploredRisk = (1.0 + 6.0 * neighborRiskFlow) * explorePenalty * riskFactor; 
  double heuristic = bubbleCost + distToBubble * unexploredRisk * heightPrior[u.z()];
  // bubbleRiskCache[u] = heuristic;
  return heuristic;
}

// Returns a heuristic for the cost of turning for going from u to goal
double GlobalPlanner::smoothnessHeuristic(const Node & u, const Cell & goal) {
  if (u.cell.x() == goal.x() && u.cell.y() == goal.y()) { 
    return 0.0;     // directly above or below the goal
  }
  if (u.cell.x() == u.parent.x() && u.cell.y() == u.parent.y()) { 
    // Vertical motion not directly above or below the goal, must change to horizontal movement
    return smoothFactor * vertToHorCost;     
  }
 
  double angU = (u.cell - u.parent).angle();      // Current orientation
  double angGoal = (goal - u.cell).angle();       // Direction of goal
  double angDiff = angGoal - angU;
  angDiff = std::fabs(angleToRange(angDiff));     // Rotation needed
  int num45DegTurns = std::ceil(angDiff / (M_PI/4) - 0.01);   // Minimum number of 45-turns to goal

  // If there is height difference we also need to change to vertical movement at least once 
  int altitudeChange = u.cell.z() == goal.z() ? 0 : 1;

  return smoothFactor * (num45DegTurns + altitudeChange);
}

// Returns a heuristic for the cost of reaching the altitude of goal
double GlobalPlanner::altitudeHeuristic(const Cell & u, const Cell & goal) {
  double diff = goal.z() - u.z();
  // Either multiply by upCost or downCost, depending on if we are belove or above the goal
  double cost = diff > 0 ? upCost * std::abs(diff) : downCost * std::abs(diff);
  return cost;
}

// Returns a heuristic of going from u to goal
double GlobalPlanner::getHeuristic(const Node & u, const Cell & goal) {
  // Only overestimate the distance
  if (heuristicCache.find(u) != heuristicCache.end()) {
    return heuristicCache[u];
  }

  double heuristic = overEstimateFactor * u.cell.diagDistance2D(goal);
  heuristic += altitudeHeuristic(u.cell, goal);        // Lower bound cost due to altitude change
  heuristic += smoothnessHeuristic(u, goal);           // Lower bound cost due to turning  
  if (useRiskHeuristics){
    heuristic += riskHeuristic(u.cell, goal);          // Risk through a straight-line path of unexplored space
  }
  if (useSpeedUpHeuristics) {
    heuristic += seenCount[u.cell];
  }
  heuristicCache[u] = heuristic;
  return heuristic;
}


geometry_msgs::PoseStamped GlobalPlanner::createPoseMsg(const Cell cell, double yaw) {
  geometry_msgs::PoseStamped poseMsg;
  poseMsg.header.frame_id="/world";
  poseMsg.pose.position = cell.toPoint();
  poseMsg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  return poseMsg;
}

nav_msgs::Path GlobalPlanner::getPathMsg() {
  nav_msgs::Path pathMsg;
  pathMsg.header.frame_id="/world";

  if (currPath.size() == 0) {
    return pathMsg;
  }

  // Use actual position instead of the center of the cell
  double lastYaw = currYaw;

  for (int i=0; i < currPath.size()-1; ++i) {
    Cell p = currPath[i];
    double newYaw = nextYaw(p, currPath[i+1], lastYaw);
    // if (newYaw != lastYaw) {   // only publish corner points
      pathMsg.poses.push_back(createPoseMsg(p, newYaw));
    // }
    lastYaw = newYaw;
  }
  Cell lastPoint = currPath[currPath.size()-1];   // Last point should have the same yaw as the previous point
  pathMsg.poses.push_back(createPoseMsg(lastPoint, lastYaw));
  return pathMsg;
}

// Returns details of the cost of the path
// TODO: should not depend on startNode
PathInfo GlobalPlanner::getPathInfo(const std::vector<Cell> & path) {
  PathInfo pathInfo = {};
  for(int i=2; i < path.size(); ++i) {
    Node currNode = Node(path[i], path[i-1]);
    Node lastNode = Node(path[i-1], path[i-2]);
    double cellRisk = getRisk(currNode.cell);
    pathInfo.cost += getEdgeCost(lastNode, currNode);
    pathInfo.dist += getEdgeDist(currNode.parent, currNode.cell);
    pathInfo.risk += riskFactor * cellRisk;
    pathInfo.isBlocked |= cellRisk > maxCellRisk;
    pathInfo.smoothness += smoothFactor * getTurnSmoothness(lastNode, currNode);
  }
  return pathInfo;
}

// Prints details about the cost and heuristic functions for every step of the path
void GlobalPlanner::printPathStats(const std::vector<Cell> & path, 
                                   const Cell startParent, const Cell start,
                                   const Cell goal, double totalCost, 
                                   std::map<Node, double> & distance) {

  if (path.size() == 0) {
    printf("PATH IS EMPTY");
    return;
  }

  printf("\n\n\nPath analysis: \n");

  // Loop through the path to get real values to compare to the heuristics
  double totalDistCost = 0.0;
  double totalRiskCost = 0.0;
  double totalAltChangeCost = 0.0;
  double totalSmoothCost = 0.0;
  Node lastNode = Node(start, startParent);
  for(int i=0; i < path.size(); ++i) {
    Node currNode = Node(path[i], lastNode.cell);
    if (currNode.cell.z() - currNode.parent.z() == 0) {
      totalDistCost += getEdgeDist(currNode.parent, currNode.cell);
    }
    else {
      totalAltChangeCost += getEdgeDist(currNode.parent, currNode.cell);
    }
    totalRiskCost += riskFactor * getRisk(currNode.cell);
    totalSmoothCost += smoothFactor * getTurnSmoothness(lastNode, currNode);
    lastNode = currNode;
  }

  double currCost = 0.0;
  lastNode = Node(start, startParent);

  printf("Cell:\t \tcurrCo \theuri \ttoGoal \tOvEst \t|| \tEdgeC  \tEdgeD \tEdgeR \tEdgeS \t||\theuris \t\tDist \t\tRisk  \t\tAlti   \t\tSmooth\n");
  printf("%s (parent) \n", startParent.asString().c_str());
  printf("%s: \t%3.2f \t%3.2f \t%3.2f \t%3.2f \t|| \n", start.asString().c_str(), 
          currCost, getHeuristic(lastNode, goal), totalCost, totalCost / getHeuristic(lastNode, goal));

  for(int i=0; i < path.size(); ++i) {
    Node currNode = Node(path[i], lastNode.cell);
    currCost += getEdgeCost(lastNode, currNode);
    double heuristic = getHeuristic(currNode, goal);
    double actualCost = totalCost - currCost;
    double ovEst = actualCost / heuristic;

    double edgeC = getEdgeCost(lastNode, currNode);
    double edgeD = getEdgeDist(currNode.parent, currNode.cell);
    double edgeR = riskFactor * getRisk(currNode.cell);
    double edgeS = smoothFactor * getTurnSmoothness(lastNode, currNode);

    if (currNode.cell.z() - currNode.parent.z() == 0) {
      totalDistCost -= edgeD;
    }
    else {
      totalAltChangeCost -= edgeD;
    }
    totalRiskCost -= edgeR;
    totalSmoothCost -= edgeS;

    double distHeuristic = currNode.cell.diagDistance2D(goal);           // Lower bound for distance on a grid 
    double riskH = riskHeuristic(currNode.cell, goal);
    double altHeuristic = altitudeHeuristic(currNode.cell, goal);         // Lower bound cost due to altitude change
    double smoothHeuristic = smoothnessHeuristic(currNode, goal);  
    printf("%s: \t%3.2f \t%3.2f \t%3.2f \t%3.2f", currNode.cell.asString().c_str(), 
            currCost, heuristic, actualCost, ovEst);
    printf("\t|| \t%3.2f \t%3.2f \t%3.2f \t%3.2f", edgeC, edgeD, edgeR, edgeS);
    printf("\t|| \t%3.2f (%3.2f) \t%3.2f (%3.2f) \t%3.2f (%3.2f) \t%3.2f (%3.2f) \t%3.2f (%3.2f)\n", 
      heuristic, actualCost, distHeuristic, totalDistCost, riskH, totalRiskCost, altHeuristic, totalAltChangeCost, smoothHeuristic, totalSmoothCost);


    if (smoothHeuristic > 4 * smoothFactor) {
      Node u = currNode;
      double angU = (u.cell - u.parent).angle();
      double angGoal = (goal - u.cell).angle(); 
      double angDiff = angGoal - angU;  
      double angDiff2 = angleToRange(angDiff);   
      double angDiff3 = std::fabs(angDiff2);       // positive angle difference
      double num45DegTurns = std::ceil(angDiff3 / (M_PI/4));    // Minimum number of 45-turns to goal
      printf("\t|| \t%3.2f \t%3.2f \t%3.2f \t%3.2f \t%3.2f \t%3.2f \n", angU, angGoal, angDiff, angDiff2, angDiff3, num45DegTurns);
      ROS_INFO("WTF? \n %f %f \n\n\n\n\n\n\n\n\n\n\n\n", angleToRange(5.5), angleToRange(-5.5));
    }

    lastNode = currNode;
  }
  printf("\n\n");
}



// Calls different search functions to find a path 
bool GlobalPlanner::FindPath(std::vector<Cell> & path) {
  Cell s = Cell(currPos.x + searchTime * currVel.x, 
                currPos.y + searchTime * currVel.y, 
                currPos.z + searchTime * currVel.z);
  Cell t = goalPos;
  Cell parentOfS = s.getNeighborFromYaw(currYaw + M_PI); // The cell behind the start cell
      
  ROS_INFO("Planning a path from %s to %s", s.asString().c_str(), t.asString().c_str());
  ROS_INFO("currPos: %2.2f,%2.2f,%2.2f\t s: %2.2f,%2.2f,%2.2f", 
                      currPos.x, currPos.y, currPos.z, s.xPos(), s.yPos(), s.zPos());

  bool foundPath = false;
  double bestPathCost = inf;
  overEstimateFactor = 2.0;
  maxIterations = 10000;

  reverseSearch(t); // REVERSE_SEARCH

  while (overEstimateFactor >= 1.03 && maxIterations > lastIterations) {
    std::vector<Cell> newPath;
    bool foundNewPath;
    if (overEstimateFactor > 3) {
      foundNewPath = FindPathOld(newPath, s, t, parentOfS, true);  // No need to search with smoothness
    } 
    else {
      foundNewPath = FindSmoothPath(newPath, s, t, parentOfS);
    }

    if (foundNewPath) {
      PathInfo pathInfo = getPathInfo(newPath);
      printf("(cost: %2.2f, dist: %2.2f, risk: %2.2f, smooth: %2.2f) \n", pathInfo.cost, pathInfo.dist, pathInfo.risk, pathInfo.smoothness);
      if (pathInfo.cost < bestPathCost) {
        bestPathCost = pathInfo.cost;
        path = newPath;
        foundPath = true;
      }
    }
    else {
      break;
    }
      maxIterations -= lastIterations;
      overEstimateFactor = (overEstimateFactor - 1.0) / 4.0 + 1.0;
  }

  // Last resort, try 2d search at maxHeight
  if (!foundPath) {
    maxIterations = 5000;
    foundPath = Find2DPath(path, s, t, parentOfS);
  }

  return foundPath;
}

// Searches for a path from s to t at maxHeight, fills path if it finds one
bool GlobalPlanner::Find2DPath(std::vector<Cell> & path, const Cell & s, const Cell t, const Cell & startParent) {
  std::vector<Cell> upPath;
  Cell aboveS(Cell(s.x(), s.y(), maxHeight));
  Cell aboveT(Cell(t.x(), t.y(), maxHeight));
  bool foundUpPath = FindPathOld(upPath, s, aboveS, s, true);
  std::vector<Cell> downPath;
  bool foundDownPath = FindPathOld(downPath, aboveT, t, aboveT, true);
  std::vector<Cell> verticalPath;
  bool foundVerticalPath = FindPathOld(verticalPath, aboveS, aboveT, aboveS, false);

  if (foundUpPath && foundVerticalPath && foundDownPath) {
    path = upPath;
    for (Cell c : verticalPath) {
      path.push_back(c);
    }
    for (Cell c : downPath) {
      path.push_back(c);
    }
    return true;
  }
  return false;
}

// TODO: Run search backwards to quickly find impossible scenarios and/or find exact heuristics for nodes close to goal
bool GlobalPlanner::reverseSearch(const Cell t) {
  std::map<Cell, double> cost;
  std::priority_queue<CellDistancePair, std::vector<CellDistancePair>, CompareDist> pq;
  std::unordered_set<Cell> seenCells;
  int numIter = 0;
  cost[t] = getRisk(t);
  pq.push(std::make_pair(t, cost[t]));

  while (!pq.empty() && numIter++ < 100) {
    CellDistancePair cellDistU = pq.top(); pq.pop();
    Cell u = cellDistU.first;
    if (seenCells.find(u) != seenCells.end()) {
      continue;
    }
    seenCells.insert(u);
    bubbleRiskCache[u] = cost[u];
    bubbleCost = cost[u];
    bubbleRadius = std::max(bubbleRadius, u.distance3D(t));
    
    for (Cell v : u.getNeighbors()) {
      double newCost = cost[u] + u.distance3D(v) * riskFactor * getRisk(v);
      double oldCost = inf;
      if (cost.find(v) != cost.end()) {
        oldCost = cost[v];  // TODO: Default dict
      }
      if (newCost < oldCost) {
        cost[v] = newCost;
        pq.push(std::make_pair(v, newCost));
      }
    }
  }
}

// A* to find a path from s to t, true iff it found a path
bool GlobalPlanner::FindPathOld(std::vector<Cell> & path, const Cell & s, 
                                const Cell t, const Cell & startParent, bool is3D) {

  // Initialize containers
  seen.clear();
  std::map<Cell, Cell> parent;
  std::map<Cell, double> distance;
  std::priority_queue<CellDistancePair, std::vector<CellDistancePair>, CompareDist> pq;                 
  pq.push(std::make_pair(s, 0.0));
  distance[s] = 0.0;
  int numIter = 0;
  double minDistHeuristic = inf;

  std::clock_t    startTime;
  startTime = std::clock();
  // Search until all reachable cells have been found, it runs out of time or t is found,
  while (!pq.empty() && numIter < maxIterations) {
    CellDistancePair cellDistU = pq.top(); pq.pop();
    Cell u = cellDistU.first;
    double d = distance[u];
    if (seen.find(u) != seen.end()) {
      continue;
    }
    seen.insert(u);
    numIter++;
    if (u == t) {
      break;  // Found a path
    }

    std::vector<CellDistancePair> neighbors;
    getOpenNeighbors(u, neighbors, is3D);
    for (auto cellDistV : neighbors) {
      Cell v = cellDistV.first;
      double costOfEdge = cellDistV.second;   // Use getEdgeCost
      double risk = riskFactor * getRisk(v);
      if (risk > riskFactor * maxCellRisk && v != t) {
        continue;
      }
      double newDist = d + costOfEdge + risk;
      double oldDist = inf;
      if (distance.find(v) != distance.end()) {
        oldDist = distance[v];
      }
      if (newDist < oldDist) {
        // Found a better path to v, have to add v to the queue 
        parent[v] = u;
        distance[v] = newDist;
        // TODO: try Dynamic Weighting instead of a constant overEstimateFactor
        double heuristic = v.diagDistance2D(t);                 // Lower bound for distance on a grid 
        heuristic += upCost * std::max(0, t.z() - v.z());       // Minumum cost for increasing altitude
        heuristic += std::max(0, v.z() - t.z());                // Minumum cost for decreasing altitude
        // if (isNearWall(v)) {
        //   heuristic -= 10;
        // }
        // if (v.diagDistance2D(t) < minDistHeuristic) {
        //   heuristic -= 20;
        // }
        double overestimatedHeuristic = newDist + overEstimateFactor * heuristic;
        pq.push(std::make_pair(v, overestimatedHeuristic));
      }
    }
  }
  printf("Average iteration time: %2.1f µs \n", (std::clock() - startTime) / (double)(CLOCKS_PER_SEC / 1000000) / numIter);

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
  path.push_back(startParent);

  std::reverse(path.begin(),path.end());

  lastIterations = numIter;
  // ROS_INFO("Found path with %d iterations, itDistSquared: %.3f", numIter, numIter / squared(path.size()));
  printf("overEstimateFactor: %2.2f, \t numIter: %d \t path cost: %2.2f \t", overEstimateFactor, numIter, distance[t]);
  
  return true;
}  

// A* to find a path from start to t, true iff it found a path
bool GlobalPlanner::FindSmoothPath(std::vector<Cell> & path, const Cell & start, 
                                   const Cell & t, const Cell & startParent) {

  // Initialize containers
  Node s = Node(start, startParent);
  Node bestGoalNode;
  seen.clear();
  seenCount.clear();

  std::unordered_set<Node> seenNodes;
  std::unordered_map<Node, Node> parent;
  std::unordered_map<Node, double> distance;
  std::priority_queue<NodeDistancePair, std::vector<NodeDistancePair>, CompareDist> pq;                 
  pq.push(std::make_pair(s, 0.0));
  distance[s] = 0.0;
  int numIter = 0;

  std::clock_t startTime;
  startTime = std::clock();
  while (!pq.empty() && numIter < maxIterations) {
    NodeDistancePair nodeDistU = pq.top(); pq.pop();
    Node u = nodeDistU.first;
    if (seenNodes.find(u) != seenNodes.end()) {
      continue;
    }
    seenNodes.insert(u);
    if (u.cell == t) {
      bestGoalNode = u;
      break;  // Found a path
    }
    numIter++;

    for (Node v : u.getNeighbors()) {
      double newDist = distance[u] + getEdgeCost(u, v); 
      double oldDist = inf;
      if (distance.find(v) != distance.end()) {
        oldDist = distance[v];
      }
      if (newDist < oldDist) {
      // Found a better path to v, have to add v to the queue 
        if (seenCount.find(v.cell) == seenCount.end()) {
          seenCount[v.cell] = 0.0;
        }
        parent[v] = u;
        distance[v] = newDist;
        // TODO: try Dynamic Weighting instead of a constant overEstimateFactor
        double overestimatedHeuristic = newDist + getHeuristic(v, t);
        pq.push(NodeDistancePair(v, overestimatedHeuristic));
        seenCount[v.cell] += 1.0;
        seen.insert(v.cell);
      }
    }
  }

  if (bestGoalNode.cell != t) {
    return false;   // No path found
  }
  printf("Average iteration time: %2.1f µs \n", (std::clock() - startTime) / (double)(CLOCKS_PER_SEC / 1000000) / numIter);

  // Get the path by walking from t back to s (excluding s)
  Node walker = bestGoalNode;
  while (walker != s) {
    path.push_back(walker.cell);
    walker = parent[walker];
  }
  path.push_back(start);
  path.push_back(startParent);
  std::reverse(path.begin(),path.end());

  // printPathStats(path, startParent, start, t, distance[bestGoalNode], distance);
  lastIterations = numIter;
  // ROS_INFO("Found path with %d iterations, itDistSquared: %.3f", numIter, numIter / squared(path.size()));
  printf("overEstimateFactor: %2.2f, \t numIter: %d \t path cost: %2.2f \t", overEstimateFactor, numIter, distance[bestGoalNode]);
  
  return true;
}

// Returns true iff a path needs to be published, either a new path or a path back
// The path is then stored in this.pathMsg
bool GlobalPlanner::getGlobalPath() {
  Cell s = Cell(currPos);
  Cell t = Cell(goalPos);
  
  if (getRisk(t) > maxCellRisk) {
    // If goal is occupied, no path is published
    ROS_INFO("Goal position is occupied");
    goalIsBlocked = true;
    return false;
  }
  // else if (isOccupied(s)) {
  //   // If current position is occupied the way back is published
  //   ROS_INFO("Current position is occupied, going back.");
  //   goBack();
  //   return true;
  // }
  else {
    // Both current position and goal are free, try to find a path
    std::vector<Cell> path;
    if (!FindPath(path)) {
      double riskOfGoal = getRisk(t);
      ROS_INFO("  Failed to find a path, risk of t: %3.2f", riskOfGoal);
      goalIsBlocked = true;
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
  goingBack = true;
  std::vector<Cell> newPath = pathBack;
  std::reverse(newPath.begin(), newPath.end());

  // Follow the path back until the risk is low
  for (int i=1; i < newPath.size()-1; ++i){
    if (i > 5 && getRisk(newPath[i]) < 0.5) {
      newPath.resize(i+1);                    // newPath is the last i+1 positions of pathBack
      pathBack.resize(pathBack.size()-i-2);   // Remove part of pathBack that is also in newPath
      break;
    }    
  }
  currPath = newPath;
  goalPos = newPath[newPath.size()-1];
}

void GlobalPlanner::stop() {
  ROS_INFO("  STOP  ");
  setGoal(Cell(currPos));
}

} // namespace avoidance
