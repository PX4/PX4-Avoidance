#include "global_planner/global_planner_node.h"

namespace global_planner {

GlobalPlannerNode::GlobalPlannerNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      avoidance_node_(nh, nh_private),
      cmdloop_dt_(0.1),
      plannerloop_dt_(1.0),
      mapupdate_dt_(0.2),
      start_yaw_(0.0) {
  // Set up Dynamic Reconfigure Server
  dynamic_reconfigure::Server<global_planner::GlobalPlannerNodeConfig>::CallbackType f;
  f = boost::bind(&GlobalPlannerNode::dynamicReconfigureCallback, this, _1, _2);
  server_.setCallback(f);

#ifndef DISABLE_SIMULATION
  world_visualizer_.reset(new avoidance::WorldVisualizer(nh_, ros::this_node::getName()));
#endif

  avoidance_node_.init();
  // Read Ros parameters
  readParams();

  // Subscribers
  octomap_full_sub_ = nh_.subscribe("octomap_full", 1, &GlobalPlannerNode::octomapFullCallback, this);
  ground_truth_sub_ = nh_.subscribe("mavros/local_position/pose", 1, &GlobalPlannerNode::positionCallback, this);
  velocity_sub_ = nh_.subscribe("mavros/local_position/velocity", 1, &GlobalPlannerNode::velocityCallback, this);
  clicked_point_sub_ = nh_.subscribe("clicked_point", 1, &GlobalPlannerNode::clickedPointCallback, this);
  move_base_simple_sub_ = nh_.subscribe("move_base_simple/goal", 1, &GlobalPlannerNode::moveBaseSimpleCallback, this);
  fcu_input_sub_ = nh_.subscribe("mavros/trajectory/desired", 1, &GlobalPlannerNode::fcuInputGoalCallback, this);

  // Publishers
  global_temp_path_pub_ = nh_.advertise<nav_msgs::Path>("global_temp_path", 10);
  actual_path_pub_ = nh_.advertise<nav_msgs::Path>("actual_path", 10);
  smooth_path_pub_ = nh_.advertise<nav_msgs::Path>("smooth_path", 10);
  global_goal_pub_ = nh_.advertise<geometry_msgs::PointStamped>("global_goal", 10);
  global_temp_goal_pub_ = nh_.advertise<geometry_msgs::PointStamped>("global_temp_goal", 10);
  explored_cells_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("explored_cells", 10);
  mavros_waypoint_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  mavros_obstacle_free_path_pub_ = nh_.advertise<mavros_msgs::Trajectory>("mavros/trajectory/generated", 10);
  current_waypoint_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("current_setpoint", 10);
  pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_in", 10);

  actual_path_.header.frame_id = frame_id_;

  ros::TimerOptions cmdlooptimer_options(ros::Duration(cmdloop_dt_),
                                         boost::bind(&GlobalPlannerNode::cmdLoopCallback, this, _1), &cmdloop_queue_);
  cmdloop_timer_ = nh_.createTimer(cmdlooptimer_options);

  cmdloop_spinner_.reset(new ros::AsyncSpinner(1, &cmdloop_queue_));
  cmdloop_spinner_->start();

  ros::TimerOptions plannerlooptimer_options(ros::Duration(plannerloop_dt_),
                                             boost::bind(&GlobalPlannerNode::plannerLoopCallback, this, _1),
                                             &plannerloop_queue_);
  plannerloop_timer_ = nh_.createTimer(plannerlooptimer_options);

  plannerloop_spinner_.reset(new ros::AsyncSpinner(1, &plannerloop_queue_));
  plannerloop_spinner_->start();

  current_goal_.header.frame_id = frame_id_;
  current_goal_.pose.position = start_pos_;
  current_goal_.pose.orientation = tf::createQuaternionMsgFromYaw(start_yaw_);
  last_goal_ = current_goal_;

  speed_ = global_planner_.default_speed_;
  start_time_ = ros::Time::now();
}

GlobalPlannerNode::~GlobalPlannerNode() {}

// Read Ros parameters
void GlobalPlannerNode::readParams() {
  std::vector<std::string> camera_topics;

  nh_.param<double>("start_pos_x", start_pos_.x, 0.5);
  nh_.param<double>("start_pos_y", start_pos_.y, 0.5);
  nh_.param<double>("start_pos_z", start_pos_.z, 3.5);
  nh_.param<std::string>("frame_id", frame_id_, "local_origin");
  nh_.getParam("pointcloud_topics", camera_topics);
  if (!nh_.hasParam("camera_frame_id")) {
    nh_.setParam("camera_frame_id", "camera_link");
  } else {
    nh_.getParam("camera_frame_id", camera_frame_id_);
  }

  initializeCameraSubscribers(camera_topics);
  global_planner_.goal_pos_ = GoalCell(start_pos_.x, start_pos_.y, start_pos_.z);
  double robot_radius;
  nh_.param<double>("robot_radius", robot_radius, 0.5);
  global_planner_.setFrame(frame_id_);
  global_planner_.setRobotRadius(robot_radius);
}

void GlobalPlannerNode::initializeCameraSubscribers(std::vector<std::string>& camera_topics) {
  cameras_.resize(camera_topics.size());

  for (size_t i = 0; i < camera_topics.size(); i++) {
    cameras_[i].pointcloud_sub_ = nh_.subscribe(camera_topics[i], 1, &GlobalPlannerNode::depthCameraCallback, this);
  }
}

// Sets a new goal, plans a path to it and publishes some info
void GlobalPlannerNode::setNewGoal(const GoalCell& goal) {
  ROS_INFO("========== Set goal : %s ==========", goal.asString().c_str());
  global_planner_.setGoal(goal);
  publishGoal(goal);
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
  }
}

// Plans a new path and publishes it
void GlobalPlannerNode::planPath() {
  std::clock_t start_time = std::clock();
  if (global_planner_.octree_) {
    ROS_INFO("OctoMap memory usage: %2.3f MB", global_planner_.octree_->memoryUsage() / 1000000.0);
  }

  bool found_path = global_planner_.getGlobalPath();

  if (!found_path) {
    // TODO: popNextGoal(), instead of checking if goal_is_blocked in
    // positionCallback?
    ROS_INFO("Failed to find a path");
  } else if (global_planner_.overestimate_factor_ > 1.05) {
    // The path is not good enough, set an intermediate goal on the path
    setIntermediateGoal();
  }
  printf("Total time: %2.2f ms \n", (std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
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

void GlobalPlannerNode::dynamicReconfigureCallback(global_planner::GlobalPlannerNodeConfig& config, uint32_t level) {
  // global_planner_
  global_planner_.min_altitude_ = config.min_altitude_;
  global_planner_.max_altitude_ = config.max_altitude_;
  global_planner_.max_cell_risk_ = config.max_cell_risk_;
  global_planner_.smooth_factor_ = config.smooth_factor_;
  global_planner_.vert_to_hor_cost_ = config.vert_to_hor_cost_;
  global_planner_.risk_factor_ = config.risk_factor_;
  global_planner_.neighbor_risk_flow_ = config.neighbor_risk_flow_;
  global_planner_.explore_penalty_ = config.explore_penalty_;
  global_planner_.up_cost_ = config.up_cost_;
  global_planner_.down_cost_ = config.down_cost_;
  global_planner_.search_time_ = config.search_time_;
  global_planner_.min_overestimate_factor_ = config.min_overestimate_factor_;
  global_planner_.max_overestimate_factor_ = config.max_overestimate_factor_;
  global_planner_.risk_threshold_risk_based_speedup_ = config.risk_threshold_risk_based_speedup_;
  global_planner_.default_speed_ = config.default_speed_;
  global_planner_.max_speed_ = config.max_speed_;
  global_planner_.max_iterations_ = config.max_iterations_;
  global_planner_.goal_must_be_free_ = config.goal_must_be_free_;
  global_planner_.use_current_yaw_ = config.use_current_yaw_;
  global_planner_.use_risk_heuristics_ = config.use_risk_heuristics_;
  global_planner_.use_speedup_heuristics_ = config.use_speedup_heuristics_;
  global_planner_.use_risk_based_speedup_ = config.use_risk_based_speedup_;

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

void GlobalPlannerNode::velocityCallback(const geometry_msgs::TwistStamped& msg) {
  global_planner_.curr_vel_ = msg.twist.linear;
}

// Sets the current position and checks if the current goal has been reached
void GlobalPlannerNode::positionCallback(const geometry_msgs::PoseStamped& msg) {
  // Update position
  last_pos_ = msg;
  global_planner_.setPose(last_pos_);

  // Check if a new goal is needed
  if (num_pos_msg_++ % 10 == 0) {
    // Keep track of and publish the actual travel trajectory
    // ROS_INFO("Travelled path extended");
    last_pos_.header.frame_id = frame_id_;
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

void GlobalPlannerNode::clickedPointCallback(const geometry_msgs::PointStamped& msg) {
  printPointInfo(msg.point.x, msg.point.y, msg.point.z);

  geometry_msgs::PoseStamped pose;
  pose.header = msg.header;
  pose.pose.position = msg.point;
  pose.pose.position.z = global_planner_.curr_pos_.z;
  last_clicked_points.push_back(pose);
}

void GlobalPlannerNode::moveBaseSimpleCallback(const geometry_msgs::PoseStamped& msg) {
  setNewGoal(GoalCell(msg.pose.position.x, msg.pose.position.y, clicked_goal_alt_, clicked_goal_radius_));
}

void GlobalPlannerNode::fcuInputGoalCallback(const mavros_msgs::Trajectory& msg) {
  const GoalCell new_goal = GoalCell(msg.point_2.position.x, msg.point_2.position.y, msg.point_2.position.z, 1.0);
  if (msg.point_valid[1] == true && ((std::fabs(global_planner_.goal_pos_.xPos() - new_goal.xPos()) > 0.001) ||
                                     (std::fabs(global_planner_.goal_pos_.yPos() - new_goal.yPos()) > 0.001))) {
    setNewGoal(new_goal);
  }
}

// Check if the current path is blocked
void GlobalPlannerNode::octomapFullCallback(const octomap_msgs::Octomap& msg) {
  std::lock_guard<std::mutex> lock(mutex_);

  ros::Time current = ros::Time::now();
  // Update map at a fixed rate. This is useful on setting replanning rates for the planner.
  if ((current - last_wp_time_).toSec() < mapupdate_dt_) {
    return;
  }
  last_wp_time_ = ros::Time::now();

  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(msg);

  global_planner_.updateFullOctomap(tree);
}

// Go through obstacle points and store them
void GlobalPlannerNode::depthCameraCallback(const sensor_msgs::PointCloud2& msg) {
  try {
    // Transform msg from camera frame to world frame
    ros::Time now = ros::Time::now();
    listener_.waitForTransform(frame_id_, camera_frame_id_, now, ros::Duration(5.0));
    tf::StampedTransform transform;
    listener_.lookupTransform(frame_id_, camera_frame_id_, now, transform);
    sensor_msgs::PointCloud2 transformed_msg;
    pcl_ros::transformPointCloud(frame_id_, transform, msg, transformed_msg);
    pcl::PointCloud<pcl::PointXYZ> cloud;  // Easier to loop through pcl::PointCloud
    pcl::fromROSMsg(transformed_msg, cloud);

    // Store the obstacle points
    for (const auto& p : cloud) {
      if (!std::isnan(p.x)) {
        // TODO: Not all points end up here
        Cell occupied_cell(p.x, p.y, p.z);
        global_planner_.occupied_.insert(occupied_cell);
      }
    }
    pointcloud_pub_.publish(msg);
  } catch (tf::TransformException const& ex) {
    ROS_DEBUG("%s", ex.what());
    ROS_WARN("Transformation not available (%s to %s)", frame_id_.c_str(), camera_frame_id_.c_str());
  }
}

void GlobalPlannerNode::setCurrentPath(const std::vector<geometry_msgs::PoseStamped>& poses) {
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

  ros::Duration since_last_cloud = now - last_wp_time_;
  ros::Duration since_start = now - start_time_;

  avoidance_node_.checkFailsafe(since_last_cloud, since_start, hover_);
  publishSetpoint();
}

void GlobalPlannerNode::plannerLoopCallback(const ros::TimerEvent& event) {
  std::lock_guard<std::mutex> lock(mutex_);
  bool is_in_goal = global_planner_.goal_pos_.withinPositionRadius(global_planner_.curr_pos_);
  if (is_in_goal || global_planner_.goal_is_blocked_) {
    popNextGoal();
  }

  planPath();

  // Print and publish info
  if (is_in_goal && !waypoints_.empty()) {
    ROS_INFO("Reached current goal %s, %d goals left\n\n", global_planner_.goal_pos_.asString().c_str(),
             (int)waypoints_.size());
    ROS_INFO("Actual travel distance: %2.2f \t Actual energy usage: %2.2f", pathLength(actual_path_),
             pathEnergy(actual_path_, global_planner_.up_cost_));
  }

  publishPath();
}

// Publish the position of goal
void GlobalPlannerNode::publishGoal(const GoalCell& goal) {
  geometry_msgs::PointStamped pointMsg;
  pointMsg.header.frame_id = frame_id_;
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
  setCurrentPath(path_msg.poses);
  smooth_path_pub_.publish(smoothPath(path_msg));

  auto simple_path = simplifyPath(&global_planner_, global_planner_.curr_path_, simplify_iterations_, simplify_margin_);
  auto simple_path_msg = global_planner_.getPathMsg(simple_path);
  global_temp_path_pub_.publish(simple_path_msg);
  setCurrentPath(simple_path_msg.poses);
  smooth_path_pub_.publish(smoothPath(simple_path_msg));
}

// Prints information about the point, mostly the risk of the containing cell
void GlobalPlannerNode::printPointInfo(double x, double y, double z) {
  // Update explored cells
  printPointStats(&global_planner_, x, y, z);
}

void GlobalPlannerNode::publishSetpoint() {
  // Vector pointing from current position to the current goal
  tf::Vector3 vec = toTfVector3(subtractPoints(current_goal_.pose.position, last_pos_.pose.position));
  if (global_planner_.use_speedup_heuristics_) {
    Cell cur_cell =
        global_planner::Cell(last_pos_.pose.position.x, last_pos_.pose.position.y, last_pos_.pose.position.z);
    double cur_risk = std::sqrt(global_planner_.getRisk(cur_cell));
    if (cur_risk >= global_planner_.risk_threshold_risk_based_speedup_) {  // If current risk is too high(more than
                                                                           // risk_threshold_risk_based_speedup_), set
                                                                           // speed as low to stable flight.
      speed_ = global_planner_.default_speed_;
    } else {  // If current risk is low, speed up for fast flight.
      speed_ = global_planner_.default_speed_ +
               (global_planner_.max_speed_ - global_planner_.default_speed_) * (1 - cur_risk);
    }
  } else {  // If risk based speed up is not activated, use default_speed_.
    speed_ = global_planner_.default_speed_;
  }

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

  // Publish setpoint to Mavros
  mavros_waypoint_publisher_.publish(setpoint);
  mavros_msgs::Trajectory obst_free_path = {};
  geometry_msgs::Twist velocity_setpoint{};
  velocity_setpoint.linear.x = NAN;
  velocity_setpoint.linear.y = NAN;
  velocity_setpoint.linear.z = NAN;
  avoidance::transformToTrajectory(obst_free_path, setpoint, velocity_setpoint);
  mavros_obstacle_free_path_pub_.publish(obst_free_path);
}

bool GlobalPlannerNode::isCloseToGoal() { return distance(current_goal_, last_pos_) < speed_; }

}  // namespace global_planner
