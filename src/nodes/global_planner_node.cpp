#include "global_planner_node.h"

namespace avoidance {

GlobalPlannerNode::GlobalPlannerNode() {
  ros::NodeHandle nh;

  cmd_octomap_full_sub_ = nh.subscribe("/octomap_full", 1, &GlobalPlannerNode::OctomapFullCallback, this);
  cmd_ground_truth_sub_ = nh.subscribe("/mavros/local_position/pose", 1,&GlobalPlannerNode::PositionCallback, this);
  velocity_sub_ = nh.subscribe("/mavros/local_position/velocity", 1,&GlobalPlannerNode::VelocityCallback, this);
  cmd_clicked_point_sub_ = nh.subscribe("/clicked_point", 1,&GlobalPlannerNode::ClickedPointCallback, this);
  laser_sensor_sub_ = nh.subscribe("/scan", 1,&GlobalPlannerNode::LaserSensorCallback, this);

  cmd_global_path_pub_ = nh.advertise<nav_msgs::Path>("/global_path", 10);
  cmd_actual_path_pub_ = nh.advertise<nav_msgs::Path>("/actual_path", 10);
  cmd_clicked_point_pub_ = nh.advertise<geometry_msgs::PointStamped>("/global_goal", 10);
  cmd_explored_cells_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/explored_cells", 10);

  actualPath.header.frame_id="/world";
}

GlobalPlannerNode::~GlobalPlannerNode() { }

void GlobalPlannerNode::SetNewGoal(Cell goal) {
  ROS_INFO("========== Set goal : %s ==========", goal.asString().c_str());
  global_planner.setGoal(goal);
  geometry_msgs::PointStamped pointMsg;
  pointMsg.header.frame_id = "/world";
  pointMsg.point = goal.toPoint();
  cmd_clicked_point_pub_.publish(pointMsg);
  PlanPath();
}

void GlobalPlannerNode::VelocityCallback(const geometry_msgs::TwistStamped& msg) {
  global_planner.currVel = msg.twist.linear;
  global_planner.currVel = rotateToWorldCoordinates(global_planner.currVel); // 90 deg fix
}

void GlobalPlannerNode::PositionCallback(const geometry_msgs::PoseStamped& msg) {

  auto rot_msg = msg;
  rot_msg = rotatePoseMsgToWorld(rot_msg); // 90 deg fix
  global_planner.setPose(rot_msg);

  double distToGoal = global_planner.goalPos.manhattanDist(global_planner.currPos.x, global_planner.currPos.y, global_planner.currPos.z);
  if (fileGoals.size() > 0 && (distToGoal < 1.0 || global_planner.goalIsBlocked)) {
    // If there is another goal and we are either at current goal or it is blocked, we set a new goal
    if (!global_planner.goalIsBlocked) {
      ROS_INFO("Actual travel distance: %2.2f \t Actual energy usage: %2.2f", pathLength(actualPath), pathEnergy(actualPath, global_planner.upCost));
      ROS_INFO("Reached current goal %s, %d goals left\n\n", global_planner.goalPos.asString().c_str(), (int) fileGoals.size());
    }
    Cell newGoal = fileGoals.front();
    fileGoals.erase(fileGoals.begin());
    SetNewGoal(newGoal);
  }

  // Keep track of and publish the actual travel trajectory
  if (numPositionMessages++ % 50 == 0) {
    rot_msg.header.frame_id = "/world";
    actualPath.poses.push_back(rot_msg);
    cmd_actual_path_pub_.publish(actualPath);
  }
}

void GlobalPlannerNode::ClickedPointCallback(const geometry_msgs::PointStamped& msg) {
  SetNewGoal(Cell(msg.point.x, msg.point.y, 3.0));
}


void GlobalPlannerNode::LaserSensorCallback(const sensor_msgs::LaserScan& msg) {
  double minRange = msg.range_max;
  for (double range : msg.ranges) {
    minRange = range < msg.range_min ? minRange : std::min(minRange, range);
  }
  if (!global_planner.goingBack && minRange < 0.5) {
    ROS_INFO("CRASH!!! Distance to obstacle: %2.2f\n\n\n", minRange);
    if (global_planner.pathBack.size() > 3) {
      global_planner.goBack();
      PublishPath();
    }
  }
}

void GlobalPlannerNode::OctomapFullCallback(
    const octomap_msgs::Octomap& msg) {

  if (numOctomapMessages++ % 10 > 0) {
    return;     // We get too many of those messages. Only process 1/10 of them
  }

  if (!global_planner.updateFullOctomap(msg)) {
    // Part of the current path is blocked
    ROS_INFO("  Path is bad, planning a new path \n");
    PlanPath();                               // Plan a whole new path
  }
}

void GlobalPlannerNode::PlanPath() {
  ROS_INFO("Start planning path.");
  ROS_INFO("OctoMap memory usage: %2.3f MB", global_planner.octree->memoryUsage() / 1000000.0);
  bool foundPath = global_planner.getGlobalPath();
  PublishExploredCells();
  if (!foundPath) {
    ROS_INFO("Failed to find a path");
  }
  PublishPath();
}

void GlobalPlannerNode::PublishPath() {
  auto pathMsg = global_planner.getPathMsg();
  cmd_global_path_pub_.publish(pathMsg);
}

void GlobalPlannerNode::PublishExploredCells() {
  // Publish the cells that were explored in the last search
  visualization_msgs::MarkerArray msg;

  // The first marker deletes the ones from previous search
  int id = 0;
  visualization_msgs::Marker marker;
  marker.id = id;
  marker.action = 3;        // same as visualization_msgs::Marker::DELETEALL
  msg.markers.push_back(marker);
  
  id = 1;
  for (const auto cell : global_planner.seen) {
    visualization_msgs::Marker marker;
    marker.id = id++;
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time();
    marker.pose.position = cell.toPoint();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.1;

    // Just a hack to get the (almost) color spectrum depending on height
    // h=0 -> blue    h=0.5 -> green  h=1 -> red
    // double h = (cell.zPos()-1.0) / 7.0;                // height from 1 to 8 meters
    // double h = 0.5;                                    // single color (green)
    // risk from 0% to 100%, sqrt is used to increase difference in low risk
    double h = std::sqrt(global_planner.getRisk(cell));    
    marker.color.r = std::max(0.0, 2*h-1);
    marker.color.g = 1.0 - 2.0 * std::abs(h - 0.5);
    marker.color.b = std::max(0.0, 1.0 - 2*h);
    marker.color.a = 1.0;

    if (!global_planner.octree->search(cell.xPos(), cell.yPos(), cell.zPos())) {
    // if (global_planner.occProb.find(cell) != global_planner.occProb.end()) {
      // Unknown space
      marker.color.r = marker.color.g = marker.color.b = 0.2; // Dark gray
    }
    msg.markers.push_back(marker);
  }
  cmd_explored_cells_pub_.publish(msg);
}

} // namespace avoidance

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_planner_node");
  avoidance::GlobalPlannerNode global_planner_node;

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  if (args.size() > 1) {
    ROS_INFO("    ARGS: %s", args.at(1).c_str());
    std::ifstream wp_file(args.at(1).c_str());
    if (wp_file.is_open()) {
      double x, y, z;
      // Only read complete waypoints.
      while (wp_file >> x >> y >> z) {
        global_planner_node.fileGoals.push_back(avoidance::Cell(x, y, z));
      }
      wp_file.close();
      ROS_INFO("  Read %d waypoints.", global_planner_node.fileGoals.size());
    }
    else {
      ROS_ERROR_STREAM("Unable to open goal file: " << args.at(1));
      return -1;
    }
  }
  else {
    ROS_INFO("  No goal file given.");
  }

  ros::spin();

  return 0;
}
