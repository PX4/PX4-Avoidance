#include "global_planner_node.h"

namespace global_planner {

GlobalPlannerNode::GlobalPlannerNode() {
  nh_ = ros::NodeHandle("~");

  // Set up Dynamic Reconfigure Server
  dynamic_reconfigure::Server<
      global_planner::GlobalPlannerNodeConfig>::CallbackType f;
  f = boost::bind(&GlobalPlannerNode::dynamicReconfigureCallback, this, _1, _2);
  server_.setCallback(f);

  // Read Ros parameters
  readParams();

  // Subscribers
  octomap_full_sub_ = nh_.subscribe(
      "/octomap_full", 1, &GlobalPlannerNode::octomapFullCallback, this);
  ground_truth_sub_ = nh_.subscribe("/mavros/local_position/pose", 1,
                                    &GlobalPlannerNode::positionCallback, this);
  velocity_sub_ = nh_.subscribe("/mavros/local_position/velocity", 1,
                                &GlobalPlannerNode::velocityCallback, this);
  clicked_point_sub_ = nh_.subscribe(
      "/clicked_point", 1, &GlobalPlannerNode::clickedPointCallback, this);
  three_point_sub_ = nh_.subscribe(
      "/three_points", 1, &GlobalPlannerNode::threePointCallback, this);
  move_base_simple_sub_ =
      nh_.subscribe("/move_base_simple/goal", 1,
                    &GlobalPlannerNode::moveBaseSimpleCallback, this);
  laser_sensor_sub_ =
      nh_.subscribe("/scan", 1, &GlobalPlannerNode::laserSensorCallback, this);
  depth_camera_sub_ = nh_.subscribe(
      "/camera/depth/points", 1, &GlobalPlannerNode::depthCameraCallback, this);

  // Publishers
  three_points_pub_ = nh_.advertise<nav_msgs::Path>("/three_points", 10);
  three_points_smooth_pub_ =
      nh_.advertise<nav_msgs::Path>("/three_points_smooth", 10);
  three_points_revised_pub_ =
      nh_.advertise<nav_msgs::Path>("/three_points_revised", 10);
  global_path_pub_ = nh_.advertise<nav_msgs::Path>("/global_path", 10);
  global_temp_path_pub_ =
      nh_.advertise<nav_msgs::Path>("/global_temp_path", 10);
  actual_path_pub_ = nh_.advertise<nav_msgs::Path>("/actual_path", 10);
  smooth_path_pub_ = nh_.advertise<nav_msgs::Path>("/smooth_path", 10);
  global_goal_pub_ =
      nh_.advertise<geometry_msgs::PointStamped>("/global_goal", 10);
  global_temp_goal_pub_ =
      nh_.advertise<geometry_msgs::PointStamped>("/global_temp_goal", 10);
  explored_cells_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/explored_cells", 10);

  actual_path_.header.frame_id = "/world";
  listener_.waitForTransform("/fcu", "/world", ros::Time(0),
                             ros::Duration(3.0));
}

GlobalPlannerNode::~GlobalPlannerNode() {}

// Read Ros parameters
void GlobalPlannerNode::readParams() {
  double x, y, z;
  nh_.param<double>("start_pos_x", x, 0.5);
  nh_.param<double>("start_pos_y", y, 0.5);
  nh_.param<double>("start_pos_z", z, 3.5);
  global_planner_.goal_pos_ = GoalCell(x, y, z);
}

// Sets a new goal, plans a path to it and publishes some info
void GlobalPlannerNode::setNewGoal(const GoalCell& goal) {
  ROS_INFO("========== Set goal : %s ==========", goal.asString().c_str());
  global_planner_.setGoal(goal);
  publishGoal(goal);
  planPath();
}

// Sets the next waypoint to be the current goal
void GlobalPlannerNode::popNextGoal() {
  if (!waypoints_.empty()) {
    // Set the first goal in waypoints_ as the new goal
    GoalCell new_goal = waypoints_.front();
    waypoints_.erase(waypoints_.begin());
    setNewGoal(new_goal);
  } else if (global_planner_.goal_is_blocked_) {
    // Goal is blocked but there is no other goal in waypoints_, just stop
    ROS_INFO("  STOP  ");
    global_planner_.stop();
    publishPath();
  }
}

// Plans a new path and publishes it
void GlobalPlannerNode::planPath() {
  std::clock_t start_time = std::clock();
  if (global_planner_.octree_) {
    ROS_INFO("OctoMap memory usage: %2.3f MB",
             global_planner_.octree_->memoryUsage() / 1000000.0);
  }

  bool found_path = global_planner_.getGlobalPath();

  // Publish even though no path is found
  publishExploredCells();
  publishPath();

  if (!found_path) {
    // TODO: popNextGoal(), instead of checking if goal_is_blocked in
    // positionCallback?
    ROS_INFO("Failed to find a path");
  } else if (global_planner_.overestimate_factor_ > 1.05) {
    // The path is not good enough, set an intermediate goal on the path
    setIntermediateGoal();
  }
  printf("Total time: %2.2f ms \n",
         (std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
}

// Sets a temporary goal on the path to the current goal
void GlobalPlannerNode::setIntermediateGoal() {
  int curr_path_length = global_planner_.curr_path_.size();
  if (curr_path_length > 10) {
    printf("\n ===== Half-way path ====== \n");
    waypoints_.insert(waypoints_.begin(), global_planner_.goal_pos_);
    Cell middle_cell = global_planner_.curr_path_[curr_path_length / 2];
    setNewGoal(GoalCell(middle_cell, curr_path_length / 4, true));
  }
}

void GlobalPlannerNode::dynamicReconfigureCallback(
    global_planner::GlobalPlannerNodeConfig& config, uint32_t level) {
  // global_planner_
  global_planner_.min_altitude_ = config.min_altitude_;
  global_planner_.max_altitude_ = config.max_altitude_;
  global_planner_.max_cell_risk_ = config.max_cell_risk_;
  global_planner_.smooth_factor_ = config.smooth_factor_;
  global_planner_.vert_to_hor_cost_ = config.vert_to_hor_cost_;
  global_planner_.risk_factor_ = config.risk_factor_;
  global_planner_.neighbor_risk_flow_ = config.neighbor_risk_flow_;
  global_planner_.expore_penalty_ = config.expore_penalty_;
  global_planner_.up_cost_ = config.up_cost_;
  global_planner_.down_cost_ = config.down_cost_;
  global_planner_.search_time_ = config.search_time_;
  global_planner_.min_overestimate_factor_ = config.min_overestimate_factor_;
  global_planner_.max_overestimate_factor_ = config.max_overestimate_factor_;
  global_planner_.max_iterations_ = config.max_iterations_;
  global_planner_.goal_must_be_free_ = config.goal_must_be_free_;
  global_planner_.use_current_yaw_ = config.use_current_yaw_;
  global_planner_.use_risk_heuristics_ = config.use_risk_heuristics_;
  global_planner_.use_speedup_heuristics_ = config.use_speedup_heuristics_;

  // global_planner_node
  clicked_goal_alt_ = config.clicked_goal_alt_;
  clicked_goal_radius_ = config.clicked_goal_radius_;
  simplify_iterations_ = config.simplify_iterations_;
  simplify_margin_ = config.simplify_margin_;

  // cell
  if (level == 2) {
    CELL_SCALE = config.CELL_SCALE;
  }

  // node
  if (level == 4) {
    SPEEDNODE_RADIUS = config.SPEEDNODE_RADIUS;
    global_planner_.default_node_type_ = config.default_node_type_;
  }
}

void GlobalPlannerNode::velocityCallback(
    const geometry_msgs::TwistStamped& msg) {
  auto transformed_msg =
      transformTwistMsg(listener_, "world", "local_origin", msg);  // 90 deg fix
  global_planner_.curr_vel_ = transformed_msg.twist.linear;
}

// Sets the current position and checks if the current goal has been reached
void GlobalPlannerNode::positionCallback(
    const geometry_msgs::PoseStamped& msg) {
  // Update position
  auto rot_msg = msg;
  listener_.transformPose("world", ros::Time(0), msg, "local_origin",
                          rot_msg);  // 90 deg fix
  global_planner_.setPose(rot_msg);

  // Check if a new goal is needed
  bool is_in_goal =
      global_planner_.goal_pos_.withinPositionRadius(global_planner_.curr_pos_);
  if (is_in_goal || global_planner_.goal_is_blocked_) {
    popNextGoal();
  }

  // Print and publish info
  if (is_in_goal && !waypoints_.empty()) {
    ROS_INFO("Reached current goal %s, %d goals left\n\n",
             global_planner_.goal_pos_.asString().c_str(),
             (int)waypoints_.size());
    ROS_INFO("Actual travel distance: %2.2f \t Actual energy usage: %2.2f",
             pathLength(actual_path_),
             pathEnergy(actual_path_, global_planner_.up_cost_));
  }
  if (num_pos_msg_++ % 10 == 0) {
    // Keep track of and publish the actual travel trajectory
    // ROS_INFO("Travelled path extended");
    rot_msg.header.frame_id = "/world";
    actual_path_.poses.push_back(rot_msg);
    actual_path_pub_.publish(actual_path_);
  }
}

void GlobalPlannerNode::clickedPointCallback(
    const geometry_msgs::PointStamped& msg) {
  printPointInfo(msg.point.x, msg.point.y, msg.point.z);

  geometry_msgs::PoseStamped pose;
  pose.header = msg.header;
  pose.pose.position = msg.point;
  pose.pose.position.z = global_planner_.curr_pos_.z;
  last_clicked_points.push_back(pose);
  if (last_clicked_points.size() >= 3) {
    nav_msgs::Path three_points;
    three_points.header = msg.header;
    three_points.poses = last_clicked_points;
    last_clicked_points.clear();
    three_points_pub_.publish(three_points);
    three_points_smooth_pub_.publish(threePointBezier(three_points));
    double risk = global_planner_.getRiskOfCurve(three_points.poses);
    ROS_INFO("Risk of curve: %2.2f \n", risk);
  }
}

void GlobalPlannerNode::threePointCallback(const nav_msgs::Path& msg) {
  double risk = global_planner_.getRiskOfCurve(msg.poses);
  ROS_INFO("Risk of curve: %2.2f \n", risk);

  nav_msgs::Path new_msg;
  new_msg.header = msg.header;
  if (risk > 1.0) {
    // Current path is too risky, propose an alternative
    std::vector<Cell> new_path;
    Cell parent(msg.poses[0].pose.position);
    Cell s = Cell(interpolate(msg.poses[0].pose.position,
                              msg.poses[1].pose.position, 0.25));
    Cell t = GoalCell(Cell(msg.poses[2].pose.position), 5.0);
    auto start_node = global_planner_.getStartNode(s, parent, "SpeedNode");
    auto search_res = findSmoothPath(&global_planner_, new_path, start_node, t);
    new_msg = global_planner_.getPathMsg(new_path);
  }
  three_points_revised_pub_.publish(smoothPath(new_msg));
}

void GlobalPlannerNode::moveBaseSimpleCallback(
    const geometry_msgs::PoseStamped& msg) {
  setNewGoal(GoalCell(msg.pose.position.x, msg.pose.position.y,
                      clicked_goal_alt_, clicked_goal_radius_));
}

// If the laser senses something too close to current position, it is considered
// a crash
void GlobalPlannerNode::laserSensorCallback(const sensor_msgs::LaserScan& msg) {
  if (global_planner_.going_back_) {
    return;  // Don't deal with the same crash again
  }

  double ignore_dist =
      msg.range_min;        // Too close, probably part of the vehicle
  double crash_dist = 0.5;  // Otherwise, a measurement below this is a crash
  for (double range : msg.ranges) {
    if (ignore_dist < range && range < crash_dist) {
      if (global_planner_.path_back_.size() > 3) {
        // Don't complain about crashing on take-off
        ROS_INFO("CRASH!!! Distance to obstacle: %2.2f\n\n\n", range);
        global_planner_.goBack();
        publishPath();
      }
    }
  }
}

// Check if the current path is blocked
void GlobalPlannerNode::octomapFullCallback(const octomap_msgs::Octomap& msg) {
  if (num_octomap_msg_++ % 10 > 0) {
    return;  // We get too many of those messages. Only process 1/10 of them
  }

  bool current_path_is_ok = global_planner_.updateFullOctomap(msg);
  if (!current_path_is_ok) {
    ROS_INFO("  Path is bad, planning a new path \n");
    if (global_planner_.goal_pos_.is_temporary_) {
      popNextGoal();  // Throw away temporary goal
    } else {
      planPath();  // Plan a whole new path
    }
  }
}

// Go through obstacle points and store them
void GlobalPlannerNode::depthCameraCallback(
    const sensor_msgs::PointCloud2& msg) {
  try {
    // Transform msg from camera frame to world frame
    ros::Time now = ros::Time::now();
    listener_.waitForTransform("/world", "/camera_link", now,
                               ros::Duration(5.0));
    tf::StampedTransform transform;
    listener_.lookupTransform("/world", "/camera_link", now, transform);
    sensor_msgs::PointCloud2 transformed_msg;
    pcl_ros::transformPointCloud("/world", transform, msg, transformed_msg);
    pcl::PointCloud<pcl::PointXYZ>
        cloud;  // Easier to loop through pcl::PointCloud
    pcl::fromROSMsg(transformed_msg, cloud);

    // Store the obstacle points
    for (const auto& p : cloud) {
      if (!std::isnan(p.x)) {
        // TODO: Not all points end up here
        Cell occupied_cell(p.x, p.y, p.z);
        global_planner_.occupied_.insert(occupied_cell);
      }
    }
  } catch (tf::TransformException const& ex) {
    ROS_DEBUG("%s", ex.what());
    ROS_WARN("Transformation not available (/world to /camera_link");
  }
}

// Publish the position of goal
void GlobalPlannerNode::publishGoal(const GoalCell& goal) {
  geometry_msgs::PointStamped pointMsg;
  pointMsg.header.frame_id = "/world";
  pointMsg.point = goal.toPoint();

  // Always publish as temporary to remove any obsolete temporary path
  global_temp_goal_pub_.publish(pointMsg);
  if (!goal.is_temporary_) {
    global_goal_pub_.publish(pointMsg);
  }
}

// Publish the current path
void GlobalPlannerNode::publishPath() {
  auto path_msg = global_planner_.getPathMsg();
  PathWithRiskMsg risk_msg = global_planner_.getPathWithRiskMsg();
  // Always publish as temporary to remove any obsolete temporary path
  global_temp_path_pub_.publish(path_msg);
  if (!global_planner_.goal_pos_.is_temporary_) {
    global_path_pub_.publish(path_msg);
  }
  smooth_path_pub_.publish(smoothPath(path_msg));

  auto simple_path = simplifyPath(&global_planner_, global_planner_.curr_path_,
                                  simplify_iterations_, simplify_margin_);
  auto simple_path_msg = global_planner_.getPathMsg(simple_path);
  global_temp_path_pub_.publish(simple_path_msg);
  smooth_path_pub_.publish(smoothPath(simple_path_msg));
}

// Publish the cells that were explored in the last search
// Can be tweeked to publish other info (path_cells)
void GlobalPlannerNode::publishExploredCells() {
  visualization_msgs::MarkerArray msg;

  // The first marker deletes the ones from previous search
  int id = 0;
  visualization_msgs::Marker marker;
  marker.id = id;
  marker.action = 3;  // same as visualization_msgs::Marker::DELETEALL
  msg.markers.push_back(marker);

  id = 1;
  for (const auto& cell : global_planner_.visitor_.seen_) {
    // for (auto const& x : global_planner_.bubble_risk_cache_) {
    // Cell cell = x.first;

    // double hue = (cell.zPos()-1.0) / 7.0;                // height from 1 to
    // 8 meters double hue = 0.5;                                    // single
    // color (green) double hue = global_planner_.getHeuristic(Node(cell, cell),
    // global_planner_.goal_pos_) / global_planner_.curr_path_info_.cost; The
    // color is the square root of the risk, shows difference in low risk
    double hue = std::sqrt(global_planner_.getRisk(cell));
    auto color = spectralColor(hue);
    if (!global_planner_.octree_->search(cell.xPos(), cell.yPos(),
                                         cell.zPos())) {
      // Unknown space
      color.r = color.g = color.b = 0.2;  // Dark gray
    }
    visualization_msgs::Marker marker =
        createMarker(id++, cell.toPoint(), color);

    // risk from 0% to 100%, sqrt is used to increase difference in low risk
    msg.markers.push_back(marker);
  }
  explored_cells_pub_.publish(msg);
}

// Prints information about the point, mostly the risk of the containing cell
void GlobalPlannerNode::printPointInfo(double x, double y, double z) {
  // Update explored cells
  publishExploredCells();
  printPointStats(&global_planner_, x, y, z);
}

}  // namespace global_planner

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_planner_node");
  global_planner::GlobalPlannerNode global_planner_node;

  // Read waypoints from file, if any
  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  if (args.size() > 1) {
    ROS_INFO("    ARGS: %s", args.at(1).c_str());
    std::ifstream wp_file(args.at(1).c_str());
    if (wp_file.is_open()) {
      double x, y, z;
      while (wp_file >> x >> y >> z) {
        global_planner_node.waypoints_.push_back(global_planner::Cell(x, y, z));
      }
      wp_file.close();
      ROS_INFO("  Read %d waypoints.", static_cast<int>(global_planner_node.waypoints_.size()));
    } else {
      ROS_ERROR_STREAM("Unable to open goal file: " << args.at(1));
      return -1;
    }
  } else {
    ROS_INFO("  No goal file given.");
  }

  ros::spin();
  return 0;
}
