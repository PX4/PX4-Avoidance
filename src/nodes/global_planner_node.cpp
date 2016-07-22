#include "global_planner_node.h"

namespace avoidance {

GlobalPlannerNode::GlobalPlannerNode() {
  ros::NodeHandle nh;

  octomap_full_sub_ = nh.subscribe("/octomap_full", 1, &GlobalPlannerNode::OctomapFullCallback, this);
  ground_truth_sub_ = nh.subscribe("/mavros/local_position/pose", 1, &GlobalPlannerNode::PositionCallback, this);
  velocity_sub_ = nh.subscribe("/mavros/local_position/velocity", 1, &GlobalPlannerNode::VelocityCallback, this);
  clicked_point_sub_ = nh.subscribe("/clicked_point", 1, &GlobalPlannerNode::ClickedPointCallback, this);
  laser_sensor_sub_ = nh.subscribe("/scan", 1, &GlobalPlannerNode::LaserSensorCallback, this);
  depth_camera_sub_ = nh.subscribe("/camera/depth/points", 1, &GlobalPlannerNode::DepthCameraCallback, this);

  global_path_pub_ = nh.advertise<nav_msgs::Path>("/global_path", 10);
  actual_path_pub_ = nh.advertise<nav_msgs::Path>("/actual_path", 10);
  clicked_point_pub_ = nh.advertise<geometry_msgs::PointStamped>("/global_goal", 10);
  explored_cells_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/explored_cells", 10);

  actual_path_.header.frame_id="/world";
  listener_.waitForTransform("/local_origin","/world", ros::Time(0), ros::Duration(3.0));
}

GlobalPlannerNode::~GlobalPlannerNode() { }

void GlobalPlannerNode::SetNewGoal(const GoalCell & goal) {
  ROS_INFO("========== Set goal : %s ==========", goal.asString().c_str());
  global_planner_.setGoal(goal);
  geometry_msgs::PointStamped pointMsg;
  pointMsg.header.frame_id = "/world";
  pointMsg.point = goal.toPoint();
  clicked_point_pub_.publish(pointMsg);
  PlanPath();
}

void GlobalPlannerNode::VelocityCallback(const geometry_msgs::TwistStamped & msg) {
  // global_planner_.curr_vel_ = rotateToWorldCoordinates(global_planner_.curr_vel_); // 90 deg fix

  auto transformed_msg = transformTwistMsg(listener_, "world", "local_origin", msg);
  global_planner_.curr_vel_ = transformed_msg.twist.linear;
}

void GlobalPlannerNode::PositionCallback(const geometry_msgs::PoseStamped & msg) {

  auto rot_msg = msg;
  // rot_msg = rotatePoseMsgToWorld(rot_msg); // 90 deg fix
  listener_.transformPose("world", ros::Time(0), msg, "local_origin", rot_msg);
  global_planner_.setPose(rot_msg);

  bool is_in_goal = global_planner_.goal_pos_.contains(Cell(global_planner_.curr_pos_));
  if (waypoints_.size() > 0 && (is_in_goal || global_planner_.goal_is_blocked_)) {
    // If there is another goal and we are either at current goal or it is blocked, we set a new goal
    if (!global_planner_.goal_is_blocked_) {
      ROS_INFO("Actual travel distance: %2.2f \t Actual energy usage: %2.2f", pathLength(actual_path_), pathEnergy(actual_path_, global_planner_.up_cost_));
      ROS_INFO("Reached current goal %s, %d goals left\n\n", global_planner_.goal_pos_.asString().c_str(), (int) waypoints_.size());
    }
    GoalCell new_goal = waypoints_.front();
    waypoints_.erase(waypoints_.begin());
    SetNewGoal(new_goal);
  }

  else if (global_planner_.goal_is_blocked_) {
    // Goal is blocked but there is no other goal in waypoints_
    ROS_INFO("  STOP  ");
    SetNewGoal(GoalCell(global_planner_.curr_pos_));
  }

  // Keep track of and publish the actual travel trajectory
  if (num_pos_msg_++ % 50 == 0) {
    rot_msg.header.frame_id = "/world";
    actual_path_.poses.push_back(rot_msg);
    actual_path_pub_.publish(actual_path_);
  }
}

void GlobalPlannerNode::ClickedPointCallback(const geometry_msgs::PointStamped & msg) {
  SetNewGoal(GoalCell(msg.point.x, msg.point.y, 3.0));
}


void GlobalPlannerNode::LaserSensorCallback(const sensor_msgs::LaserScan & msg) {
  double min_range = msg.range_max;
  for (double range : msg.ranges) {
    // TODO: simplify
    min_range = range < msg.range_min ? min_range : std::min(min_range, range);
  }
  if (!global_planner_.going_back_ && min_range < 0.5) {
    ROS_INFO("CRASH!!! Distance to obstacle: %2.2f\n\n\n", min_range);
    if (global_planner_.path_back_.size() > 3) {
      global_planner_.goBack();
      PublishPath();
    }
  }
}

void GlobalPlannerNode::OctomapFullCallback(const octomap_msgs::Octomap & msg) {
  if (num_octomap_msg_++ % 10 > 0) {
    return;     // We get too many of those messages. Only process 1/10 of them
  }

  if (!global_planner_.updateFullOctomap(msg)) {
    // Part of the current path is blocked
    ROS_INFO("  Path is bad, planning a new path \n");
    PlanPath();                               // Plan a whole new path
  }
}

// Go through obstacle points and store them
void GlobalPlannerNode::DepthCameraCallback(const sensor_msgs::PointCloud2 & msg) {
  try {
    // Transform msg from camera frame to world frame
    ros::Time now = ros::Time::now();
    listener_.waitForTransform("/world", "/camera_link", now, ros::Duration(5.0));
    tf::StampedTransform transform;
    listener_.lookupTransform("/world", "/camera_link", now, transform);
    sensor_msgs::PointCloud2 transformed_msg;
    pcl_ros::transformPointCloud("/world", transform, msg, transformed_msg);
    pcl::PointCloud<pcl::PointXYZ> cloud; // Easier to loop through pcl::PointCloud
    pcl::fromROSMsg(transformed_msg, cloud); 

    // Store the obstacle points
    for (auto p : cloud) {
      if (!isnan(p.x)) {
        Cell occupied_cell(p.x, p.y, p.z);
        global_planner_.occupied_.insert(occupied_cell);
      }
    }
  }
  catch (tf::TransformException const & ex) {
    ROS_DEBUG("%s",ex.what());
    ROS_WARN("Transformation not available");
  }
}

void GlobalPlannerNode::PlanPath() {
  ROS_INFO("Start planning path.");
  ROS_INFO("OctoMap memory usage: %2.3f MB", global_planner_.octree_->memoryUsage() / 1000000.0);
  bool found_path = global_planner_.getGlobalPath();
  PublishExploredCells();
  if (!found_path) {
    ROS_INFO("Failed to find a path");
  }
  else if (global_planner_.overestimate_factor > 1.3) {
    int curr_path_length = global_planner_.curr_path_.size();
    if (curr_path_length > 10) {
      printf("\n ===== Half-way path ====== \n");
      waypoints_.insert(waypoints_.begin(), global_planner_.goal_pos_);
      Cell middle_cell = global_planner_.curr_path_[curr_path_length / 2];
      SetNewGoal(GoalCell(middle_cell, curr_path_length / 4));
      return;
    }
  }
  PublishPath();
}

void GlobalPlannerNode::PublishPath() {
  auto path_msg = global_planner_.getPathMsg();
  global_path_pub_.publish(path_msg);
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
  for (const auto cell : global_planner_.seen_) {
  // for (auto const& x : global_planner_.bubble_risk_cache_) {
    // Cell cell = x.first;

    visualization_msgs::Marker marker;
    marker.id = id++;
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time();
    marker.pose.position = cell.toPoint();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.1;

    // double h = (cell.zPos()-1.0) / 7.0;                // height from 1 to 8 meters
    // double h = 0.5;                                    // single color (green)
    // risk from 0% to 100%, sqrt is used to increase difference in low risk
    double h = std::sqrt(global_planner_.getRisk(cell));
    // double h = global_planner_.getHeuristic(Node(cell, cell), global_planner_.goal_pos_) / global_planner_.curr_path_info_.cost;

    // A hack to get the (almost) color spectrum depending on height
    // h=0 -> blue    h=0.5 -> green  h=1 -> red
    marker.color.r = std::max(0.0, 2*h-1);
    marker.color.g = 1.0 - 2.0 * std::abs(h - 0.5);
    marker.color.b = std::max(0.0, 1.0 - 2*h);
    marker.color.a = 1.0;

    if (!global_planner_.octree_->search(cell.xPos(), cell.yPos(), cell.zPos())) {
      // Unknown space
      marker.color.r = marker.color.g = marker.color.b = 0.2; // Dark gray
    }
    msg.markers.push_back(marker);
  }
  explored_cells_pub_.publish(msg);
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
        global_planner_node.waypoints_.push_back(avoidance::Cell(x, y, z));
      }
      wp_file.close();
      ROS_INFO("  Read %d waypoints.", global_planner_node.waypoints_.size());
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
