#include "global_planner/global_planner_node.h"

namespace global_planner {

GlobalPlannerNode::GlobalPlannerNode(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      avoidance_node_(nh, nh_private),
      cmdloop_dt_(0.1),
      plannerloop_dt_(1.0) {
  // Set up Dynamic Reconfigure Server
  dynamic_reconfigure::Server<
      global_planner::GlobalPlannerNodeConfig>::CallbackType f;
  f = boost::bind(&GlobalPlannerNode::dynamicReconfigureCallback, this, _1, _2);
  server_.setCallback(f);

#ifndef DISABLE_SIMULATION
  world_visualizer_.reset(new avoidance::WorldVisualizer(nh_));
#endif

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
  move_base_simple_sub_ =
      nh_.subscribe("/move_base_simple/goal", 1,
                    &GlobalPlannerNode::moveBaseSimpleCallback, this);
  laser_sensor_sub_ =
      nh_.subscribe("/scan", 1, &GlobalPlannerNode::laserSensorCallback, this);
  depth_camera_sub_ = nh_.subscribe(
      "/camera/depth/points", 1, &GlobalPlannerNode::depthCameraCallback, this);
  fcu_input_sub_ =
      nh_.subscribe("/mavros/trajectory/desired", 1,
                    &GlobalPlannerNode::fcuInputGoalCallback, this);
  path_sub_ = nh_.subscribe("/global_temp_path", 1,
                            &GlobalPlannerNode::receivePath, this);

  // Publishers
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
  mavros_waypoint_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "/mavros/setpoint_position/local", 10);
  mavros_obstacle_free_path_pub_ = nh_.advertise<mavros_msgs::Trajectory>(
      "/mavros/trajectory/generated", 10);
  current_waypoint_publisher_ =
      nh_.advertise<geometry_msgs::PoseStamped>("/current_setpoint", 10);

  actual_path_.header.frame_id = "/world";
  listener_.waitForTransform("/fcu", "/world", ros::Time(0),
                             ros::Duration(3.0));
  listener_.waitForTransform("/local_origin", "/world", ros::Time(0),
                             ros::Duration(3.0));

  ros::TimerOptions cmdlooptimer_options(
      ros::Duration(cmdloop_dt_),
      boost::bind(&GlobalPlannerNode::cmdLoopCallback, this, _1),
      &cmdloop_queue_);
  cmdloop_timer_ = nh_.createTimer(cmdlooptimer_options);

  cmdloop_spinner_.reset(new ros::AsyncSpinner(1, &cmdloop_queue_));
  cmdloop_spinner_->start();

  ros::TimerOptions plannerlooptimer_options(
      ros::Duration(plannerloop_dt_),
      boost::bind(&GlobalPlannerNode::plannerLoopCallback, this, _1),
      &plannerloop_queue_);
  plannerloop_timer_ = nh_.createTimer(plannerlooptimer_options);

  plannerloop_spinner_.reset(new ros::AsyncSpinner(1, &plannerloop_queue_));
  plannerloop_spinner_->start();

  current_goal_.header.frame_id = "/world";
  current_goal_.pose.position = start_pos_;
  current_goal_.pose.orientation = tf::createQuaternionMsgFromYaw(start_yaw_);
  last_goal_ = current_goal_;

  speed_ = 2.0;

  start_time_ = ros::Time::now();
}

GlobalPlannerNode::~GlobalPlannerNode() {}

// Read Ros parameters
void GlobalPlannerNode::readParams() {
  nh_.param<double>("start_pos_x", start_pos_.x, 0.5);
  nh_.param<double>("start_pos_y", start_pos_.y, 0.5);
  nh_.param<double>("start_pos_z", start_pos_.z, 3.5);
  global_planner_.goal_pos_ =
      GoalCell(start_pos_.x, start_pos_.y, start_pos_.z);
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
  listener_.transformPose("world", ros::Time(0), msg, "local_origin",
                          last_pos_);  // 90 deg fix

  global_planner_.setPose(last_pos_);

  // Check if a new goal is needed
  if (num_pos_msg_++ % 10 == 0) {
    // Keep track of and publish the actual travel trajectory
    // ROS_INFO("Travelled path extended");
    last_pos_.header.frame_id = "/world";
    actual_path_.poses.push_back(last_pos_);
    actual_path_pub_.publish(actual_path_);
  }

  position_received_ = true;

  // Check if we are close enough to current goal to get the next part of the
  // path
  if (path_.size() > 0 && isCloseToGoal()) {
    // TODO: get yawdiff(yaw1, yaw2)
    double yaw1 = tf::getYaw(current_goal_.pose.orientation);
    double yaw2 = tf::getYaw(last_pos_.pose.orientation);
    double yaw_diff = std::abs(yaw2 - yaw1);
    // Transform yaw_diff to [0, 2*pi]
    yaw_diff -= std::floor(yaw_diff / (2 * M_PI)) * (2 * M_PI);
    double max_yaw_diff = M_PI / 1.0;
    if (yaw_diff < max_yaw_diff || yaw_diff > 2 * M_PI - max_yaw_diff) {
      // If we are facing the right direction, then pop the first point of the
      // path
      last_goal_ = current_goal_;
      current_goal_ = path_[0];
      path_.erase(path_.begin());
    }
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
}

void GlobalPlannerNode::moveBaseSimpleCallback(
    const geometry_msgs::PoseStamped& msg) {
  setNewGoal(GoalCell(msg.pose.position.x, msg.pose.position.y,
                      clicked_goal_alt_, clicked_goal_radius_));
}

void GlobalPlannerNode::fcuInputGoalCallback(
    const mavros_msgs::Trajectory& msg) {
  const GoalCell new_goal =
      GoalCell(msg.point_2.position.x, msg.point_2.position.y,
               msg.point_2.position.z, 1.0);
  if (msg.point_valid[1] == true &&
      ((std::fabs(global_planner_.goal_pos_.xPos() - new_goal.xPos()) >
        0.001) ||
       (std::fabs(global_planner_.goal_pos_.yPos() - new_goal.yPos()) >
        0.001))) {
    setNewGoal(new_goal);
  }
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

void GlobalPlannerNode::receivePath(const nav_msgs::Path& msg) {
  setCurrentPath(msg.poses);
}

void GlobalPlannerNode::setCurrentPath(
    const std::vector<geometry_msgs::PoseStamped>& poses) {
  path_.clear();

  if (poses.size() < 2) {
    ROS_INFO("  Received empty path\n");
    return;
  }
  last_goal_ = poses[0];
  current_goal_ = poses[1];

  for (int i = 2; i < poses.size(); ++i) {
    path_.push_back(poses[i]);
  }
}

void GlobalPlannerNode::cmdLoopCallback(const ros::TimerEvent& event) {
  hover_ = false;

  // Check if all information was received
  ros::Time now = ros::Time::now();
  last_wp_time_ = ros::Time::now();

  ros::Duration since_last_cloud = now - last_wp_time_;
  ros::Duration since_start = now - start_time_;

  avoidance_node_.checkFailsafe(since_last_cloud, since_start, hover_);
  publishSetpoint();
}

void GlobalPlannerNode::plannerLoopCallback(const ros::TimerEvent& event) {
  bool is_in_goal =
      global_planner_.goal_pos_.withinPositionRadius(global_planner_.curr_pos_);
  if (is_in_goal || global_planner_.goal_is_blocked_) {
    popNextGoal();
  }

  // If the current cell is blocked, try finding a path again
  if (global_planner_.current_cell_blocked_) {
    planPath();
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

void GlobalPlannerNode::publishSetpoint() {
  // Vector pointing from current position to the current goal
  tf::Vector3 vec = toTfVector3(
      subtractPoints(current_goal_.pose.position, last_pos_.pose.position));
  // If we are less than 1.0 away, then we should stop at the goal
  double new_len = vec.length() < 1.0 ? vec.length() : speed_;
  vec.normalize();
  vec *= new_len;

  auto setpoint = current_goal_;  // The intermediate position sent to Mavros
  setpoint.pose.position.x = last_pos_.pose.position.x + vec.getX();
  setpoint.pose.position.y = last_pos_.pose.position.y + vec.getY();
  setpoint.pose.position.z = last_pos_.pose.position.z + vec.getZ();

  // Publish setpoint for vizualization
  current_waypoint_publisher_.publish(setpoint);

  listener_.transformPose("local_origin", ros::Time(0), setpoint, "world",
                          setpoint);

  // Publish setpoint to Mavros
  mavros_waypoint_publisher_.publish(setpoint);
  mavros_msgs::Trajectory obst_free_path = {};
  avoidance::transformPoseToTrajectory(obst_free_path, setpoint);
  mavros_obstacle_free_path_pub_.publish(obst_free_path);
}

bool GlobalPlannerNode::isCloseToGoal() {
  return distance(current_goal_, last_pos_) < 1.5;
}

}  // namespace global_planner

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_planner_node");

  ros::NodeHandle nh("~");
  ros::NodeHandle nh_private("");

  global_planner::GlobalPlannerNode global_planner_node(nh, nh_private);

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
      ROS_INFO("  Read %d waypoints.",
               static_cast<int>(global_planner_node.waypoints_.size()));
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
