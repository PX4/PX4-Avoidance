#include "global_planner/global_planner_node.h"

namespace global_planner {

GlobalPlannerNode::GlobalPlannerNode()
    : Node("global_planner_node"), gp_cmdloop_dt_(200ms), gp_plannerloop_dt_(1000ms), start_yaw_(0.0) {
  RCLCPP_INFO_ONCE(this->get_logger(), "GlobalPlannerNode STARTED!");

#ifndef DISABLE_SIMULATION
  world_visualizer_ = std::make_shared<avoidance::WorldVisualizer>();
  world_visualizer_executor_.add_node(world_visualizer_);
#endif

  // Read Ros parameters
  readParams();

  // Subscribers
  rclcpp::QoS qos_default = rclcpp::SystemDefaultsQoS();
  rclcpp::QoS qos_best_effort = rclcpp::QoS(5).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  octomap_full_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap_full", qos_best_effort, std::bind(&GlobalPlannerNode::octomapFullCallback, this, _1));
  attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/VehicleAttitude_PubSubTopic", qos_best_effort, std::bind(&GlobalPlannerNode::attitudeCallback, this, _1));
  clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", qos_default, std::bind(&GlobalPlannerNode::clickedPointCallback, this, _1));
  move_base_simple_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", qos_default, std::bind(&GlobalPlannerNode::moveBaseSimpleCallback, this, _1));
  local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/VehicleLocalPosition_PubSubTopic", qos_best_effort,
      std::bind(&GlobalPlannerNode::localPositionCallback, this, _1));
  global_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
      "/VehicleGlobalPosition_PubSubTopic", qos_best_effort,
      std::bind(&GlobalPlannerNode::globalPositionCallback, this, _1));
  status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      "/VehicleStatus_PubSubTopic", qos_best_effort,
      std::bind(&GlobalPlannerNode::vehicleStatusCallback, this, _1));

  // Publishers
  global_temp_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_temp_path", 10);
  actual_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/actual_path", 10);
  smooth_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/smooth_path", 10);
  global_goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/global_goal", 10);
  global_temp_goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/global_temp_goal", 10);
  explored_cells_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/explored_cells", 10);
  vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/VehicleCommand_PubSubTopic", 10);
  current_waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_setpoint", 10);
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_in", 10);

  transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  actual_path_.header.frame_id = frame_id_;

  gp_cmdloop_timer_ = this->create_wall_timer(gp_cmdloop_dt_, [&]() { cmdLoopCallback(); });
  gp_plannerloop_timer_ = this->create_wall_timer(gp_plannerloop_dt_, [&]() { plannerLoopCallback(); });

  current_goal_.header.frame_id = frame_id_;
  current_goal_.pose.position = start_pos_;
  current_goal_.pose.orientation = avoidance::createQuaternionMsgFromYaw(start_yaw_);
  last_goal_ = current_goal_;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface =
      std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  start_time_ = rclcpp::Clock().now();
}

GlobalPlannerNode::~GlobalPlannerNode() {}

void GlobalPlannerNode::readParams() {
  std::vector<std::string> camera_topics;

  frame_id_ = this->declare_parameter("frame_id", "/base_frame");
  start_pos_.x = this->declare_parameter("start_pos_x", 0.0);
  start_pos_.y = this->declare_parameter("start_pos_y", 0.0);
  start_pos_.z = this->declare_parameter("start_pos_z", 3.0);
  global_planner_.min_altitude_ = this->declare_parameter("min_altitude", global_planner_.min_altitude_);
  global_planner_.max_altitude_ = this->declare_parameter("max_altitude", global_planner_.max_altitude_);
  global_planner_.max_cell_risk_ = this->declare_parameter("max_cell_risk", global_planner_.max_cell_risk_);
  global_planner_.smooth_factor_ = this->declare_parameter("smooth_factor", global_planner_.smooth_factor_);
  global_planner_.vert_to_hor_cost_ = this->declare_parameter("vert_to_hor_cost", global_planner_.vert_to_hor_cost_);
  global_planner_.risk_factor_ = this->declare_parameter("risk_factor", global_planner_.risk_factor_);
  global_planner_.neighbor_risk_flow_ =
      this->declare_parameter("neighbor_risk_flow", global_planner_.neighbor_risk_flow_);
  global_planner_.explore_penalty_ = this->declare_parameter("explore_penalty", global_planner_.explore_penalty_);
  global_planner_.up_cost_ = this->declare_parameter("up_cost", global_planner_.up_cost_);
  global_planner_.down_cost_ = this->declare_parameter("down_cost", global_planner_.down_cost_);
  global_planner_.search_time_ = this->declare_parameter("search_time", global_planner_.search_time_);
  global_planner_.min_overestimate_factor_ =
      this->declare_parameter("min_overestimate_factor", global_planner_.min_overestimate_factor_);
  global_planner_.max_overestimate_factor_ =
      this->declare_parameter("max_overestimate_factor", global_planner_.max_overestimate_factor_);
  global_planner_.risk_threshold_risk_based_speedup_ =
      this->declare_parameter("risk_threshold_risk_based_speedup", global_planner_.risk_threshold_risk_based_speedup_);
  global_planner_.default_speed_ = this->declare_parameter("default_speed", global_planner_.default_speed_);
  global_planner_.max_speed_ = this->declare_parameter("max_speed", global_planner_.max_speed_);
  global_planner_.max_iterations_ = this->declare_parameter("max_iterations", global_planner_.max_iterations_);
  global_planner_.goal_is_blocked_ = this->declare_parameter("goal_is_blocked", global_planner_.goal_is_blocked_);
  global_planner_.current_cell_blocked_ =
      this->declare_parameter("current_cell_blocked", global_planner_.current_cell_blocked_);
  global_planner_.goal_must_be_free_ = this->declare_parameter("goal_must_be_free", global_planner_.goal_must_be_free_);
  global_planner_.use_current_yaw_ = this->declare_parameter("use_current_yaw", global_planner_.use_current_yaw_);
  global_planner_.use_risk_heuristics_ =
      this->declare_parameter("use_risk_heuristics", global_planner_.use_risk_heuristics_);
  global_planner_.use_speedup_heuristics_ =
      this->declare_parameter("use_speedup_heuristics", global_planner_.use_speedup_heuristics_);
  global_planner_.use_risk_based_speedup_ =
      this->declare_parameter("use_risk_based_speedup", global_planner_.use_risk_based_speedup_);
  global_planner_.position_mode_ = this->declare_parameter("position_mode", global_planner_.position_mode_);

  camera_topics = this->declare_parameter("pointcloud_topics", camera_topics);
  // camera_topics.push_back("/camera/points");

  initializeCameraSubscribers(camera_topics);
  global_planner_.goal_pos_ = GoalCell(start_pos_.x, start_pos_.y, start_pos_.z);
  double robot_radius;
  this->get_parameter_or("robot_radius", robot_radius, 1.0);
  global_planner_.setFrame(frame_id_);
  global_planner_.setRobotRadius(robot_radius);

  // Position for PX4 sitl default location
  global_planner_.ref_point_.latitude = 47.3977508;
  global_planner_.ref_point_.longitude = 8.5456073;
  global_planner_.ref_point_.altitude = 488.10101318359375;

  global_planner_.ref_point_.latitude = 
    this->declare_parameter("ref_point_lat", global_planner_.ref_point_.latitude);
  global_planner_.ref_point_.longitude = 
    this->declare_parameter("ref_point_long", global_planner_.ref_point_.longitude);
  global_planner_.ref_point_.altitude = 
    this->declare_parameter("ref_point_alt", global_planner_.ref_point_.altitude);
}

void GlobalPlannerNode::initializeCameraSubscribers(std::vector<std::string>& camera_topics) {
  cameras_.resize(camera_topics.size());
  rclcpp::QoS qos_best_effort = rclcpp::QoS(5).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  for (size_t i = 0; i < camera_topics.size(); i++) {
    cameras_[i].pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        camera_topics[i], qos_best_effort, std::bind(&GlobalPlannerNode::depthCameraCallback, this, _1));
  }
}

// Sets a new goal, plans a path to it and publishes some info
void GlobalPlannerNode::setNewGoal(const GoalCell& goal) {
  RCLCPP_INFO(this->get_logger(), "========== Set goal : %s ==========", goal.asString().c_str());
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
    // RCLCPP_INFO(this->get_logger(), "  STOP  ");
    global_planner_.stop();
  }
}

// Plans a new path and publishes it
void GlobalPlannerNode::planPath() {
  std::clock_t start_time = std::clock();
  if (global_planner_.octree_) {
    RCLCPP_INFO(this->get_logger(), "OctoMap memory usage: %2.3f MB", global_planner_.octree_->memoryUsage() /
    1000000.0);
  }

  bool found_path = global_planner_.getGlobalPath();

  if (!found_path) {
    // TODO: popNextGoal(), instead of checking if goal_is_blocked in
    // localPositionCallback?
    // RCLCPP_INFO(this->get_logger(), "Failed to find a path");
  } else if (global_planner_.overestimate_factor_ > 1.05) {
    // The path is not good enough, set an intermediate goal on the path
    setIntermediateGoal();
  }
  // printf("Total time: %2.2f ms \n", (std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
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

void GlobalPlannerNode::attitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
  tf2::Quaternion q(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
  double yaw, pitch, roll;
  tf2::getEulerYPR(q, yaw, pitch, roll);
  tf2::Quaternion q_NED;
  q_NED.setRPY(yaw + 3.14, -pitch, -roll - 3.141592);
  geometry_msgs::msg::Quaternion quat_geomsg;
  tf2::convert(q_NED, quat_geomsg);

  geometry_msgs::msg::TransformStamped tfmsg;
  tfmsg.header.stamp = rclcpp::Clock().now();
  tfmsg.header.frame_id = "local_origin";
  tfmsg.child_frame_id = "local_origin_odom";
  tfmsg.transform.rotation = quat_geomsg;
  transform_broadcaster_->sendTransform(tfmsg);
}

// Sets the current position and checks if the current goal has been reached
void GlobalPlannerNode::localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
  if (global_planner_.position_mode_.compare("local_position") == 0) {
    geometry_msgs::msg::TransformStamped tfmsg;
    tfmsg.header.stamp = rclcpp::Clock().now();
    tfmsg.header.frame_id = "base_frame_ned";
    tfmsg.child_frame_id = "local_origin";
    tfmsg.transform.translation.x = msg->x;
    tfmsg.transform.translation.y = msg->y;
    tfmsg.transform.translation.z = msg->z;
    transform_broadcaster_->sendTransform(tfmsg);

    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = msg->x;
    pose.pose.position.y = msg->y;
    pose.pose.position.z = msg->z;
    pose.pose.orientation = avoidance::createQuaternionMsgFromYaw(msg->heading);

    geometry_msgs::msg::PoseStamped transformed_pose = avoidance::transformNEDandENU(pose);

    // Update position
    last_pos_ = transformed_pose;
    global_planner_.setPose(transformed_pose, msg->heading);

    // Update velocity (considering NED to ENU transformation)
    geometry_msgs::msg::Vector3 vel;
    vel.x = msg->vy;
    vel.y = msg->vx;
    vel.z = -(msg->vz);
    global_planner_.curr_vel_ = vel;

    // Check if a new goal is needed
    if (num_local_pos_msg_++ % 10 == 0) {
      last_pos_.header.frame_id = frame_id_;
      actual_path_.poses.push_back(last_pos_);
      actual_path_pub_->publish(actual_path_);
    }

    position_received_ = true;

    // Check if we are close enough to current goal to get the next part of the
    // path
    if (path_.size() > 0 && isCloseToGoal()) {
      double yaw1 = tf2::getYaw(current_goal_.pose.orientation);
      double yaw2 = tf2::getYaw(last_pos_.pose.orientation);
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

  // Update velocity (considering NED to ENU transformation)
  geometry_msgs::msg::Vector3 vel;
  vel.x = msg->vy;
  vel.y = msg->vx;
  vel.z = -(msg->vz);
  global_planner_.curr_vel_ = vel;
}

void GlobalPlannerNode::globalPositionCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
  geographic_msgs::msg::GeoPoint cur_point;
  cur_point.latitude = msg->lat;
  cur_point.longitude = msg->lon;
  cur_point.altitude = msg->alt;

  geometry_msgs::msg::Point local_pos = LLH2NED(global_planner_.ref_point_, cur_point);

  geometry_msgs::msg::TransformStamped tfmsg;
  tfmsg.header.stamp = rclcpp::Clock().now();
  tfmsg.header.frame_id = "base_frame_ned";
  tfmsg.child_frame_id = "local_origin";
  tfmsg.transform.translation.x = local_pos.x;
  tfmsg.transform.translation.y = local_pos.y;
  tfmsg.transform.translation.z = local_pos.z;
  transform_broadcaster_->sendTransform(tfmsg);

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = local_pos.x;
  pose.pose.position.y = local_pos.y;
  pose.pose.position.z = local_pos.z;
  pose.pose.orientation = avoidance::createQuaternionMsgFromYaw(0);

  geometry_msgs::msg::PoseStamped transformed_pose = avoidance::transformNEDandENU(pose);

  // Update position
  last_pos_ = transformed_pose;
  global_planner_.setPose(transformed_pose, 0);

  // Check if a new goal is needed
  if (num_global_pos_msg_++ % 10 == 0) {
    last_pos_.header.frame_id = frame_id_;
    actual_path_.poses.push_back(last_pos_);
    actual_path_pub_->publish(actual_path_);
  }

  position_received_ = true;

  // Check if we are close enough to current goal to get the next part of the
  // path
  if (path_.size() > 0 && isCloseToGoal()) {
    double yaw1 = tf2::getYaw(current_goal_.pose.orientation);
    double yaw2 = tf2::getYaw(last_pos_.pose.orientation);
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

void GlobalPlannerNode::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
  last_vehicle_status_.timestamp = msg->timestamp;
  last_vehicle_status_.nav_state = msg->nav_state;
  last_vehicle_status_.nav_state_timestamp = msg->nav_state_timestamp;
  last_vehicle_status_.system_id = msg->system_id;
  last_vehicle_status_.component_id = msg->component_id;
}

void GlobalPlannerNode::clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  printPointInfo(msg->point.x, msg->point.y, msg->point.z);

  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg->header;
  pose.pose.position = msg->point;
  pose.pose.position.z = global_planner_.curr_pos_.z;
  last_clicked_points.push_back(pose);
}

void GlobalPlannerNode::moveBaseSimpleCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  setNewGoal(GoalCell(msg->pose.position.x, msg->pose.position.y, 3.0));
}

// // Check if the current path is blocked
void GlobalPlannerNode::octomapFullCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);

  rclcpp::Time current = rclcpp::Clock().now();
  // Update map at a fixed rate. This is useful on setting replanning rates for the planner.
  if ((current - last_wp_time_).seconds() < mapupdate_dt_) {
    return;
  }
  last_wp_time_ = rclcpp::Clock().now();

  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);

  global_planner_.updateFullOctomap(tree);
}

// Go through obstacle points and store them
void GlobalPlannerNode::depthCameraCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  msg->header.stamp = rclcpp::Clock().now();
  msg->header.frame_id = "camera_frame";
  pointcloud_pub_->publish(*msg);
}

void GlobalPlannerNode::setCurrentPath(const std::vector<geometry_msgs::msg::PoseStamped>& poses) {
  path_.clear();

  if (poses.size() < 2) {
    // RCLCPP_INFO(this->get_logger(), "  Received empty path\n");
    return;
  }
  last_goal_ = poses[0];
  current_goal_ = poses[1];

  for (int i = 2; i < poses.size(); ++i) {
    path_.push_back(poses[i]);
  }
}

void GlobalPlannerNode::cmdLoopCallback() {
  hover_ = false;

  // Check if all information was received
  rclcpp::Time now = rclcpp::Clock().now();

  rclcpp::Duration since_last_cloud = now - last_wp_time_;
  rclcpp::Duration since_start = now - start_time_;

  avoidance_node_.checkFailsafe(since_last_cloud, since_start, hover_);
  publishSetpoint();

#ifndef DISABLE_SIMULATION
  world_visualizer_executor_.spin_some();
#endif
}

void GlobalPlannerNode::plannerLoopCallback() {
  std::lock_guard<std::mutex> lock(mutex_);
  bool is_in_goal = global_planner_.goal_pos_.withinPositionRadius(global_planner_.curr_pos_);
  if (is_in_goal || global_planner_.goal_is_blocked_) {
    popNextGoal();
  }

  planPath();

  // Print and publish info
  if (is_in_goal && !waypoints_.empty()) {
    RCLCPP_INFO(this->get_logger(), "Reached current goal %s, %d goals left\n\n",
                global_planner_.goal_pos_.asString().c_str(), (int)waypoints_.size());
    RCLCPP_INFO(this->get_logger(), "Actual travel distance: %2.2f \t Actual energy usage: %2.2f",
                pathLength(actual_path_), pathEnergy(actual_path_, global_planner_.up_cost_));
  }

  publishPath();
}

// Publish the position of goal
void GlobalPlannerNode::publishGoal(const GoalCell& goal) {
  geometry_msgs::msg::PointStamped pointMsg;
  pointMsg.header.frame_id = frame_id_;
  pointMsg.point = goal.toPoint();

  // Always publish as temporary to remove any obsolete temporary path
  global_temp_goal_pub_->publish(pointMsg);
  if (!goal.is_temporary_) {
    global_goal_pub_->publish(pointMsg);
  }
}

// Publish the current path
void GlobalPlannerNode::publishPath() {
  auto path_msg = global_planner_.getPathMsg();
  // PathWithRiskMsg risk_msg = global_planner_.getPathWithRiskMsg();
  // Always publish as temporary to remove any obsolete temporary path
  global_temp_path_pub_->publish(path_msg);
  setCurrentPath(path_msg.poses);
  smooth_path_pub_->publish(smoothPath(path_msg));

  // auto simple_path = simplifyPath(&global_planner_, global_planner_.curr_path_, simplify_iterations_,
  // simplify_margin_);
  // auto simple_path_msg = global_planner_.getPathMsg(simple_path);
  // global_temp_path_pub_->publish(simple_path_msg);
  // setCurrentPath(simple_path_msg.poses);
  // smooth_path_pub_->publish(smoothPath(simple_path_msg));
}

// Prints information about the point, mostly the risk of the containing cell
void GlobalPlannerNode::printPointInfo(double x, double y, double z) {
  // Update explored cells
  printPointStats(&global_planner_, x, y, z);
}

void GlobalPlannerNode::publishSetpoint() {
  // For the safty reason, publishing setpoint should work only if navigation state is AUTO LOITER mode.
  if (last_vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER) {
    // Vector pointing from current position to the current goal
    tf2::Vector3 vec = toTfVector3(subtractPoints(current_goal_.pose.position, last_pos_.pose.position));

    // If we are less than 1.0 away, then we should stop at the goal
    if (vec.length() < 1.0)
      return;
    // double new_len = vec.length() < 1.0 ? vec.length() : global_planner_.default_speed_;
    vec.normalize();
    vec *= global_planner_.default_speed_;

    // RCLCPP_INFO(this->get_logger(), "Current goal : %lf %lf %lf",
    //   current_goal_.pose.position.x, current_goal_.pose.position.y, current_goal_.pose.position.z);
    // RCLCPP_INFO(this->get_logger(), "Last pos : %lf %lf %lf",
    //   last_pos_.pose.position.x, last_pos_.pose.position.y, last_pos_.pose.position.z);
    // RCLCPP_INFO(this->get_logger(), "Vec : %lf %lf %lf",
    //   vec.x(), vec.y(), vec.z());

    // To reduce noisy command, ignore small vector values of x,y,z.
    if (std::abs(vec.x()) <= 0.1)
      vec.setX(0);
    if (std::abs(vec.y()) <= 0.1)
      vec.setY(0);
    if (std::abs(vec.z()) <= 0.1)
      vec.setZ(0);

    auto setpoint = current_goal_;  // The intermediate position sent to Mavros
    setpoint.pose.position.x = last_pos_.pose.position.x + vec.getX();
    setpoint.pose.position.y = last_pos_.pose.position.y + vec.getY();
    setpoint.pose.position.z = last_pos_.pose.position.z + vec.getZ();

    geometry_msgs::msg::PoseStamped NED_setpoint = avoidance::transformNEDandENU(setpoint);
    // Publish setpoint for vizualization
    // current_waypoint_publisher_->publish(setpoint);
    geographic_msgs::msg::GeoPoint setpoint_geopoint = NED2LLH(global_planner_.ref_point_, NED_setpoint.pose.position);
    auto reposition_cmd = px4_msgs::msg::VehicleCommand();
    reposition_cmd.target_system = last_vehicle_status_.system_id;
    reposition_cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_REPOSITION;
    reposition_cmd.param1 = -1.0;
    reposition_cmd.param2 = 1.0;
    reposition_cmd.param3 = 0.0;

    double yaw = atan2(vec.getX(), vec.getY());
    reposition_cmd.param4 = yaw;  // yaw
    reposition_cmd.param5 = setpoint_geopoint.latitude;
    reposition_cmd.param6 = setpoint_geopoint.longitude;
    reposition_cmd.param7 = setpoint_geopoint.altitude;
    reposition_cmd.from_external = true;
    vehicle_command_pub_->publish(reposition_cmd);
  }
}

bool GlobalPlannerNode::isCloseToGoal() { return distance(current_goal_, last_pos_) < global_planner_.default_speed_ * 1.5; }

}  // namespace global_planner
