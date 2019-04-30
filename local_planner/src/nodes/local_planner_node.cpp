#include "local_planner/local_planner_node.h"

#include "local_planner/local_planner.h"
#include "local_planner/planner_functions.h"
#include "local_planner/tree_node.h"
#include "local_planner/waypoint_generator.h"

#include <boost/algorithm/string.hpp>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

namespace avoidance {

LocalPlannerNode::LocalPlannerNode(const ros::NodeHandle& nh,
                                   const ros::NodeHandle& nh_private,
                                   const bool tf_spin_thread)
    : nh_(nh), nh_private_(nh_private), spin_dt_(0.1) {
  local_planner_.reset(new LocalPlanner());
  wp_generator_.reset(new WaypointGenerator());

  readParams();

  tf_listener_ = new tf::TransformListener(
      ros::Duration(tf::Transformer::DEFAULT_CACHE_TIME), tf_spin_thread);

  // Set up Dynamic Reconfigure Server
  server_ = new dynamic_reconfigure::Server<avoidance::LocalPlannerNodeConfig>(
      config_mutex_, nh_);
  dynamic_reconfigure::Server<avoidance::LocalPlannerNodeConfig>::CallbackType
      f;
  f = boost::bind(&LocalPlannerNode::dynamicReconfigureCallback, this, _1, _2);
  server_->setCallback(f);

  // initialize standard subscribers
  pose_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped&>(
      "/mavros/local_position/pose", 1, &LocalPlannerNode::positionCallback,
      this);
  velocity_sub_ = nh_.subscribe<const geometry_msgs::TwistStamped&>(
      "/mavros/local_position/velocity_local", 1,
      &LocalPlannerNode::velocityCallback, this);
  state_sub_ =
      nh_.subscribe("/mavros/state", 1, &LocalPlannerNode::stateCallback, this);
  clicked_point_sub_ = nh_.subscribe(
      "/clicked_point", 1, &LocalPlannerNode::clickedPointCallback, this);
  clicked_goal_sub_ =
      nh_.subscribe("/move_base_simple/goal", 1,
                    &LocalPlannerNode::clickedGoalCallback, this);
  fcu_input_sub_ = nh_.subscribe("/mavros/trajectory/desired", 1,
                                 &LocalPlannerNode::fcuInputGoalCallback, this);
  goal_topic_sub_ = nh_.subscribe("/input/goal_position", 1,
                                  &LocalPlannerNode::updateGoalCallback, this);
  distance_sensor_sub_ = nh_.subscribe(
      "/mavros/altitude", 1, &LocalPlannerNode::distanceSensorCallback, this);
  px4_param_sub_ = nh_.subscribe("/mavros/param/param_value", 1,
                                 &LocalPlannerNode::px4ParamsCallback, this);
  mavros_vel_setpoint_pub_ = nh_.advertise<geometry_msgs::Twist>(
      "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
  mavros_pos_setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "/mavros/setpoint_position/local", 10);
  mavros_obstacle_free_path_pub_ = nh_.advertise<mavros_msgs::Trajectory>(
      "/mavros/trajectory/generated", 10);
  mavros_obstacle_distance_pub_ =
      nh_.advertise<sensor_msgs::LaserScan>("/mavros/obstacle/send", 10);
  mavros_system_status_pub_ =
      nh_.advertise<mavros_msgs::CompanionProcessStatus>(
          "/mavros/companion_process/status", 1);
  get_px4_param_client_ =
      nh_.serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");

  // initialize visualization topics
  visualizer_.initializePublishers(nh_);
#ifndef DISABLE_SIMULATION
  world_visualizer_.initializePublishers(nh_);
#endif

  // pass initial goal into local planner
  local_planner_->applyGoal();

  local_planner_->disable_rise_to_goal_altitude_ =
      disable_rise_to_goal_altitude_;
  status_msg_.state = (int)MAV_STATE::MAV_STATE_BOOT;

  hover_ = false;
  planner_is_healthy_ = true;
  startup_ = true;
  callPx4Params_ = true;
  armed_ = false;
  start_time_ = ros::Time::now();
}

LocalPlannerNode::~LocalPlannerNode() {
  should_exit_ = true;
  data_ready_cv_.notify_all();

  delete server_;
  delete tf_listener_;
}

void LocalPlannerNode::startNode() {
  ros::TimerOptions timer_options(
      ros::Duration(spin_dt_),
      boost::bind(&LocalPlannerNode::cmdLoopCallback, this, _1),
      &cmdloop_queue_);
  cmdloop_timer_ = nh_.createTimer(timer_options);

  cmdloop_spinner_.reset(new ros::AsyncSpinner(1, &cmdloop_queue_));
  cmdloop_spinner_->start();
}

void LocalPlannerNode::readParams() {
  // Parameter from launch file
  auto goal = toPoint(local_planner_->getGoal());
  nh_.param<double>("goal_x_param", goal.x, 9.0);
  nh_.param<double>("goal_y_param", goal.y, 13.0);
  nh_.param<double>("goal_z_param", goal.z, 3.5);
  nh_.param<bool>("disable_rise_to_goal_altitude",
                  disable_rise_to_goal_altitude_, false);
  nh_.param<bool>("accept_goal_input_topic", accept_goal_input_topic_, false);

  std::vector<std::string> camera_topics;
  nh_.getParam("pointcloud_topics", camera_topics);
  initializeCameraSubscribers(camera_topics);

  nh_.param<std::string>("world_name", world_path_, "");
  goal_msg_.pose.position = goal;
}

void LocalPlannerNode::initializeCameraSubscribers(
    std::vector<std::string>& camera_topics) {
  cameras_.resize(camera_topics.size());

  // create sting containing the topic with the camera info from
  // the pointcloud topic
  std::string s;
  s.reserve(50);
  std::vector<std::string> camera_info(camera_topics.size(), s);

  for (size_t i = 0; i < camera_topics.size(); i++) {
    cameras_[i].trans_ready_mutex_.reset(new std::mutex);
    cameras_[i].trans_ready_cv_.reset(new std::condition_variable);
    cameras_[i].cloud_ready_mutex_.reset(new std::mutex);
    cameras_[i].cloud_ready_cv_.reset(new std::condition_variable);
    cameras_[i].transformed_ = false;

    cameras_[i].pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        camera_topics[i], 1,
        boost::bind(&LocalPlannerNode::pointCloudCallback, this, _1, i));
    cameras_[i].topic_ = camera_topics[i];
    cameras_[i].received_ = false;

    // get each namespace in the pointcloud topic and construct the camera_info
    // topic
    std::vector<std::string> name_space;
    boost::split(name_space, camera_topics[i], [](char c) { return c == '/'; });
    for (int k = 0, name_spaces = name_space.size() - 1; k < name_spaces; ++k) {
      camera_info[i].append(name_space[k]);
      camera_info[i].append("/");
    }
    camera_info[i].append("camera_info");
    cameras_[i].camera_info_sub_ = nh_.subscribe<sensor_msgs::CameraInfo>(
        camera_info[i], 1,
        boost::bind(&LocalPlannerNode::cameraInfoCallback, this, _1, i));
    cameras_[i].transform_thread_ =
        std::thread(&LocalPlannerNode::pointCloudTransformThread, this, i);
  }
}

size_t LocalPlannerNode::numReceivedClouds() {
  size_t num_received_clouds = 0;
  for (size_t i = 0; i < cameras_.size(); i++) {
    if (cameras_[i].received_) num_received_clouds++;
  }
  return num_received_clouds;
}

size_t LocalPlannerNode::numTransformedClouds() {
  size_t num_transformed_clouds = 0;
  for (size_t i = 0; i < cameras_.size(); i++) {
    std::unique_lock<std::mutex> lk(*(cameras_[i].trans_ready_mutex_));
    cameras_[i].trans_ready_cv_->wait_for(
        lk, std::chrono::milliseconds(30),
        [this, i] { return cameras_[i].transformed_; });
    if (cameras_[i].transformed_) num_transformed_clouds++;
  }
  return num_transformed_clouds;
}

void LocalPlannerNode::updatePlanner() {
  if (cameras_.size() == numReceivedClouds() && cameras_.size() != 0) {
    if (cameras_.size() == numTransformedClouds()) {
      if (running_mutex_.try_lock()) {
        updatePlannerInfo();
        // reset all clouds to not yet received
        for (size_t i = 0; i < cameras_.size(); i++) {
          cameras_[i].received_ = false;
        }
        wp_generator_->setPlannerInfo(local_planner_->getAvoidanceOutput());
        running_mutex_.unlock();
        // Wake up the planner
        std::unique_lock<std::mutex> lck(data_ready_mutex_);
        data_ready_ = true;
        data_ready_cv_.notify_one();
      }
    }
  }
}

bool LocalPlannerNode::canUpdatePlannerInfo() {
  // Check if we have a transformation available at the time of the current
  // point cloud
  size_t missing_transforms = 0;
  for (size_t i = 0; i < cameras_.size(); ++i) {
    if (!tf_listener_->canTransform(
            "/local_origin", cameras_[i].newest_cloud_msg_.header.frame_id,
            ros::Time(0))) {
      missing_transforms++;
    }
  }

  return missing_transforms == 0;
}
void LocalPlannerNode::updatePlannerInfo() {
  // update the point cloud
  local_planner_->original_cloud_vector_.clear();
  for (size_t i = 0; i < cameras_.size(); ++i) {
    try {
      local_planner_->original_cloud_vector_.push_back(
          std::move(cameras_[i].pcl_cloud));
    } catch (tf::TransformException& ex) {
      ROS_ERROR("Received an exception trying to transform a pointcloud: %s",
                ex.what());
    }
  }

  // update position
  local_planner_->setPose(toEigen(newest_pose_.pose.position),
                          toEigen(newest_pose_.pose.orientation));

  // Update velocity
  local_planner_->setCurrentVelocity(toEigen(vel_msg_.twist.linear));

  // update state
  local_planner_->currently_armed_ = armed_;

  // update goal
  if (new_goal_) {
    local_planner_->setGoal(toEigen(goal_msg_.pose.position));
    new_goal_ = false;
  }

  // update ground distance
  if (ros::Time::now() - ground_distance_msg_.header.stamp <
      ros::Duration(0.5)) {
    local_planner_->ground_distance_ = ground_distance_msg_.bottom_clearance;
  } else {
    local_planner_->ground_distance_ = 2.0;  // in case where no range data is
    // available assume vehicle is close to ground
  }

  // update last sent waypoint
  local_planner_->last_sent_waypoint_ = toEigen(newest_waypoint_position_);
}

void LocalPlannerNode::positionCallback(const geometry_msgs::PoseStamped& msg) {
  last_pose_ = newest_pose_;
  newest_pose_ = msg;
  position_received_ = true;

#ifndef DISABLE_SIMULATION
  // visualize drone in RVIZ
  if (!world_path_.empty()) {
    if (world_visualizer_.visualizeDrone(msg)) {
      ROS_WARN("Failed to visualize drone in RViz");
    }
  }
#endif
}

void LocalPlannerNode::velocityCallback(
    const geometry_msgs::TwistStamped& msg) {
  vel_msg_ = msg;
}

void LocalPlannerNode::stateCallback(const mavros_msgs::State& msg) {
  armed_ = msg.armed;

  if (msg.mode == "AUTO.MISSION") {
    nav_state_ = NavigationState::mission;
  } else if (msg.mode == "AUTO.TAKEOFF") {
    nav_state_ = NavigationState::auto_takeoff;
  } else if (msg.mode == "AUTO.LAND") {
    nav_state_ = NavigationState::auto_land;
  } else if (msg.mode == "AUTO.RTL") {
    nav_state_ = NavigationState::auto_rtl;
  } else if (msg.mode == "AUTO.RTGS") {
    nav_state_ = NavigationState::auto_rtgs;
  } else if (msg.mode == "OFFBOARD") {
    nav_state_ = NavigationState::offboard;
  } else {
    nav_state_ = NavigationState::none;
  }
}

void LocalPlannerNode::cmdLoopCallback(const ros::TimerEvent& event) {
  hover_ = false;

#ifdef DISABLE_SIMULATION
  startup_ = false;
#else
  // visualize world in RVIZ
  if (!world_path_.empty() && startup_) {
    if (world_visualizer_.visualizeRVIZWorld(world_path_)) {
      ROS_WARN("Failed to visualize Rviz world");
    }
    startup_ = false;
  }

#endif

  // Process callbacks & wait for a position update
  ros::Time start_query_position = ros::Time::now();
  while (!position_received_ && ros::ok()) {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    ros::Duration since_query = ros::Time::now() - start_query_position;
    if (since_query > ros::Duration(local_planner_->timeout_termination_)) {
      status_msg_.state = (int)MAV_STATE::MAV_STATE_FLIGHT_TERMINATION;
      publishSystemStatus();
      if (!position_not_received_error_sent_) {
        // clang-format off
        ROS_WARN("\033[1;33m Planner abort: missing required data \n \033[0m");
        ROS_WARN("----------------------------- Debugging Info -----------------------------");
        ROS_WARN("Local planner has not received a position from FCU, check the following: ");
        ROS_WARN("1. Check cables connecting PX4 autopilot with onboard computer");
        ROS_WARN("2. Set PX4 parameter MAV_1_MODE to onbard or external vision");
        ROS_WARN("3. Set correct fcu_url in local_planner launch file:");
        ROS_WARN("   Example direct connection to serial port: /dev/ttyUSB0:921600");
        ROS_WARN("   Example connection over mavlink router: udp://:14540@localhost:14557");
        ROS_WARN("--------------------------------------------------------------------------");
        // clang-format on
        position_not_received_error_sent_ = true;
      }
    }
  }

  // Check if all information was received
  ros::Time now = ros::Time::now();
  ros::Duration since_last_cloud = now - last_wp_time_;
  ros::Duration since_start = now - start_time_;

  checkFailsafe(since_last_cloud, since_start, planner_is_healthy_, hover_);

  // If planner is not running, update planner info and get last results
  updatePlanner();

  // send waypoint
  if (!never_run_ && planner_is_healthy_) {
    calculateWaypoints(hover_);
    if (!hover_) status_msg_.state = (int)MAV_STATE::MAV_STATE_ACTIVE;
  } else {
    for (size_t i = 0; i < cameras_.size(); ++i) {
      // once the camera info have been set once, unsubscribe from topic
      cameras_[i].camera_info_sub_.shutdown();
    }
  }

  position_received_ = false;

  // publish system status
  if (now - t_status_sent_ > ros::Duration(0.2)) publishSystemStatus();

  return;
}
void LocalPlannerNode::calculateWaypoints(bool hover) {
  bool is_airborne = armed_ && (nav_state_ != NavigationState::none);

  wp_generator_->updateState(
      toEigen(newest_pose_.pose.position),
      toEigen(newest_pose_.pose.orientation), toEigen(goal_msg_.pose.position),
      toEigen(prev_goal_.pose.position), toEigen(vel_msg_.twist.linear), hover,
      is_airborne);
  waypointResult result = wp_generator_->getWaypoints();

  Eigen::Vector3f closest_pt = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f deg60_pt = Eigen::Vector3f(NAN, NAN, NAN);
  wp_generator_->getOfftrackPointsForVisualization(closest_pt, deg60_pt);

  last_waypoint_position_ = newest_waypoint_position_;
  newest_waypoint_position_ = toPoint(result.smoothed_goto_position);
  last_adapted_waypoint_position_ = newest_adapted_waypoint_position_;
  newest_adapted_waypoint_position_ = toPoint(result.adapted_goto_position);

  // visualize waypoint topics
  visualizer_.visualizeWaypoints(result.goto_position,
                                 result.adapted_goto_position,
                                 result.smoothed_goto_position);
  visualizer_.publishPaths(last_pose_.pose.position, newest_pose_.pose.position,
                           last_waypoint_position_, newest_waypoint_position_,
                           last_adapted_waypoint_position_,
                           newest_adapted_waypoint_position_);
  visualizer_.publishCurrentSetpoint(
      toTwist(result.linear_velocity_wp, result.angular_velocity_wp),
      result.waypoint_type, newest_pose_.pose.position);

  visualizer_.publishOfftrackPoints(closest_pt, deg60_pt);

  // send waypoints to mavros
  mavros_msgs::Trajectory obst_free_path = {};
  if (local_planner_->use_vel_setpoints_) {
    mavros_vel_setpoint_pub_.publish(
        toTwist(result.linear_velocity_wp, result.angular_velocity_wp));
    transformVelocityToTrajectory(
        obst_free_path,
        toTwist(result.linear_velocity_wp, result.angular_velocity_wp));
  } else {
    mavros_pos_setpoint_pub_.publish(
        toPoseStamped(result.position_wp, result.orientation_wp));
    transformPoseToTrajectory(
        obst_free_path,
        toPoseStamped(result.position_wp, result.orientation_wp));
  }
  mavros_obstacle_free_path_pub_.publish(obst_free_path);
}

void LocalPlannerNode::publishSystemStatus() {
  status_msg_.header.stamp = ros::Time::now();
  status_msg_.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
  mavros_system_status_pub_.publish(status_msg_);
  t_status_sent_ = ros::Time::now();
}

void LocalPlannerNode::clickedPointCallback(
    const geometry_msgs::PointStamped& msg) {
  printPointInfo(msg.point.x, msg.point.y, msg.point.z);
}

void LocalPlannerNode::clickedGoalCallback(
    const geometry_msgs::PoseStamped& msg) {
  new_goal_ = true;
  goal_msg_ = msg;
  /* Selecting the goal from Rviz sets x and y. Get the z coordinate set in
   * the launch file */
  goal_msg_.pose.position.z = local_planner_->getGoal().z();
}

void LocalPlannerNode::updateGoalCallback(
    const visualization_msgs::MarkerArray& msg) {
  if (accept_goal_input_topic_ && msg.markers.size() > 0) {
    goal_msg_.pose = msg.markers[0].pose;
    new_goal_ = true;
  }
}

void LocalPlannerNode::fcuInputGoalCallback(
    const mavros_msgs::Trajectory& msg) {
  if ((msg.point_valid[1] == true) &&
      (toEigen(goal_msg_.pose.position) - toEigen(msg.point_2.position))
              .norm() > 0.01f) {
    new_goal_ = true;
    prev_goal_ = goal_msg_;
    goal_msg_.pose.position = msg.point_2.position;
  }
}

void LocalPlannerNode::distanceSensorCallback(
    const mavros_msgs::Altitude& msg) {
  if (!std::isnan(msg.bottom_clearance)) {
    ground_distance_msg_ = msg;
    visualizer_.publishGround(local_planner_->getPosition(),
                              local_planner_->histogram_box_.radius_,
                              local_planner_->ground_distance_);
  }
}

void LocalPlannerNode::px4ParamsCallback(const mavros_msgs::Param& msg) {
  // collect all px4 parameters needed for model based trajectory planning
  // when adding new parameter to the struct ModelParameters,
  // add new else if case with correct value type

  if (msg.param_id == "MPC_ACC_DOWN_MAX") {
    ROS_INFO("parameter acceleration down is set from  %f to %f \n",
             local_planner_->px4_.param_mpc_acc_down_max, msg.value.real);
    local_planner_->px4_.param_mpc_acc_down_max = msg.value.real;
  } else if (msg.param_id == "MPC_ACC_HOR") {
    ROS_INFO("parameter acceleration horizontal is set from  %f to %f \n",
             local_planner_->px4_.param_mpc_acc_hor, msg.value.real);
    local_planner_->px4_.param_mpc_acc_hor = msg.value.real;
  } else if (msg.param_id == "MPC_ACC_UP_MAX") {
    ROS_INFO("parameter acceleration up is set from  %f to %f \n",
             local_planner_->px4_.param_acc_up_max, msg.value.real);
    local_planner_->px4_.param_acc_up_max = msg.value.real;
  } else if (msg.param_id == "MPC_AUTO_MODE") {
    ROS_INFO("parameter auto mode is set from  %i to %li \n",
             local_planner_->px4_.param_mpc_auto_mode, msg.value.integer);
    local_planner_->px4_.param_mpc_auto_mode = msg.value.integer;
  } else if (msg.param_id == "MPC_JERK_MIN") {
    ROS_INFO("parameter jerk minimum is set from  %f to %f \n",
             local_planner_->px4_.param_mpc_jerk_min, msg.value.real);
    local_planner_->px4_.param_mpc_jerk_min = msg.value.real;
  } else if (msg.param_id == "MPC_JERK_MAX") {
    ROS_INFO("parameter jerk maximum is set from  %f to %f \n",
             local_planner_->px4_.param_mpc_jerk_max, msg.value.real);
    local_planner_->px4_.param_mpc_jerk_max = msg.value.real;
  } else if (msg.param_id == "MPC_LAND_SPEED") {
    ROS_INFO("parameter landing speed is set from  %f to %f \n",
             local_planner_->px4_.param_mpc_land_speed, msg.value.real);
    local_planner_->px4_.param_mpc_land_speed = msg.value.real;
  } else if (msg.param_id == "MPC_TKO_SPEED") {
    ROS_INFO("parameter takeoff speed is set from  %f to %f \n",
             local_planner_->px4_.param_mpc_tko_speed, msg.value.real);
    local_planner_->px4_.param_mpc_tko_speed = msg.value.real;
  } else if (msg.param_id == "MPC_XY_CRUISE") {
    ROS_INFO("parameter velocity horizontal is set from  %f to %f \n",
             local_planner_->px4_.param_mpc_xy_cruise, msg.value.real);
    local_planner_->px4_.param_mpc_xy_cruise = msg.value.real;
  } else if (msg.param_id == "MPC_Z_VEL_MAX_DN") {
    ROS_INFO("parameter velocity down is set from  %f to %f \n",
             local_planner_->px4_.param_mpc_vel_max_dn, msg.value.real);
    local_planner_->px4_.param_mpc_vel_max_dn = msg.value.real;
  } else if (msg.param_id == "MPC_Z_VEL_MAX_UP") {
    ROS_INFO("parameter velocity up is set from  %f to %f \n",
             local_planner_->px4_.param_mpc_z_vel_max_up, msg.value.real);
    local_planner_->px4_.param_mpc_z_vel_max_up = msg.value.real;
  } else if (msg.param_id == "MPC_COL_PREV_D") {
    ROS_INFO("parameter collision prevention distance is set from  %f to %f \n",
             local_planner_->px4_.param_mpc_col_prev_d, msg.value.real);
    local_planner_->px4_.param_mpc_col_prev_d = msg.value.real;
  }
}

void LocalPlannerNode::checkPx4Parameters() {
  while (!should_exit_) {
    { std::unique_lock<std::mutex> lk(px4_params_mutex_); }

    if (should_exit_) break;

    mavros_msgs::ParamGet req;
    req.request.param_id = "MPC_XY_CRUISE";
    if (get_px4_param_client_.call(req) && req.response.success) {
      local_planner_->px4_.param_mpc_xy_cruise = req.response.value.real;
    }

    req.response.success = false;
    req.request.param_id = "MPC_COL_PREV_D";
    if (get_px4_param_client_.call(req) && req.response.success) {
      local_planner_->px4_.param_mpc_col_prev_d = req.response.value.real;
    }
    std::this_thread::sleep_for(std::chrono::seconds(30));
  }
}

void LocalPlannerNode::printPointInfo(double x, double y, double z) {
  Eigen::Vector3f drone_pos = local_planner_->getPosition();
  int beta_z = floor((atan2(x - drone_pos.x(), y - drone_pos.y()) * 180.0 /
                      M_PI));  //(-180. +180]
  int beta_e =
      floor((atan((z - drone_pos.z()) /
                  (Eigen::Vector2f(x, y) - drone_pos.topRows<2>()).norm()) *
             180.0 / M_PI));  //(-90.+90)

  beta_z = beta_z + (ALPHA_RES - beta_z % ALPHA_RES);  //[-170,+190]
  beta_e = beta_e + (ALPHA_RES - beta_e % ALPHA_RES);  //[-80,+90]

  ROS_INFO("----- Point: %f %f %f -----\n", x, y, z);
  ROS_INFO("Elevation %d Azimuth %d \n", beta_e, beta_z);
  ROS_INFO("-------------------------------------------- \n");
}

void LocalPlannerNode::pointCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg, int index) {
  cameras_[index].newest_cloud_msg_ = *msg;  // FIXME: avoid a copy
  cameras_[index].received_ = true;

  {
    std::unique_lock<std::mutex> lck(*(cameras_[index].cloud_ready_mutex_));
    cameras_[index].transformed_ = false;
    cameras_[index].cloud_ready_cv_->notify_one();
  }
}

void LocalPlannerNode::cameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& msg, int index) {
  // calculate the horizontal and vertical field of view from the image size and
  // focal length:
  // h_fov = 2 * atan (image_width / (2 * focal_length_x))
  // v_fov = 2 * atan (image_height / (2 * focal_length_y))
  // Assumption: if there are n cameras the total horizonal field of view is n
  // times the horizontal field of view of a single camera
  local_planner_->h_FOV_deg_ = static_cast<float>(
      static_cast<double>(cameras_.size()) * 2.0 *
      atan(static_cast<double>(msg->width) / (2.0 * msg->K[0])) * 180.0 / M_PI);
  local_planner_->v_FOV_deg_ = static_cast<float>(
      2.0 * atan(static_cast<double>(msg->height) / (2.0 * msg->K[4])) * 180.0 /
      M_PI);
  wp_generator_->setFOV(local_planner_->h_FOV_deg_, local_planner_->v_FOV_deg_);
}

void LocalPlannerNode::fillUnusedTrajectoryPoint(
    mavros_msgs::PositionTarget& point) {
  point.position.x = NAN;
  point.position.y = NAN;
  point.position.z = NAN;
  point.velocity.x = NAN;
  point.velocity.y = NAN;
  point.velocity.z = NAN;
  point.acceleration_or_force.x = NAN;
  point.acceleration_or_force.y = NAN;
  point.acceleration_or_force.z = NAN;
  point.yaw = NAN;
  point.yaw_rate = NAN;
}

void LocalPlannerNode::transformPoseToTrajectory(
    mavros_msgs::Trajectory& obst_avoid, geometry_msgs::PoseStamped pose) {
  obst_avoid.header = pose.header;
  obst_avoid.type = 0;  // MAV_TRAJECTORY_REPRESENTATION::WAYPOINTS
  obst_avoid.point_1.position.x = pose.pose.position.x;
  obst_avoid.point_1.position.y = pose.pose.position.y;
  obst_avoid.point_1.position.z = pose.pose.position.z;
  obst_avoid.point_1.velocity.x = NAN;
  obst_avoid.point_1.velocity.y = NAN;
  obst_avoid.point_1.velocity.z = NAN;
  obst_avoid.point_1.acceleration_or_force.x = NAN;
  obst_avoid.point_1.acceleration_or_force.y = NAN;
  obst_avoid.point_1.acceleration_or_force.z = NAN;
  obst_avoid.point_1.yaw = tf::getYaw(pose.pose.orientation);
  obst_avoid.point_1.yaw_rate = NAN;

  fillUnusedTrajectoryPoint(obst_avoid.point_2);
  fillUnusedTrajectoryPoint(obst_avoid.point_3);
  fillUnusedTrajectoryPoint(obst_avoid.point_4);
  fillUnusedTrajectoryPoint(obst_avoid.point_5);

  obst_avoid.time_horizon = {NAN, NAN, NAN, NAN, NAN};

  obst_avoid.point_valid = {true, false, false, false, false};
}

void LocalPlannerNode::transformVelocityToTrajectory(
    mavros_msgs::Trajectory& obst_avoid, geometry_msgs::Twist vel) {
  obst_avoid.header.stamp = ros::Time::now();
  obst_avoid.type = 0;  // MAV_TRAJECTORY_REPRESENTATION::WAYPOINTS
  obst_avoid.point_1.position.x = NAN;
  obst_avoid.point_1.position.y = NAN;
  obst_avoid.point_1.position.z = NAN;
  obst_avoid.point_1.velocity.x = vel.linear.x;
  obst_avoid.point_1.velocity.y = vel.linear.y;
  obst_avoid.point_1.velocity.z = vel.linear.z;
  obst_avoid.point_1.acceleration_or_force.x = NAN;
  obst_avoid.point_1.acceleration_or_force.y = NAN;
  obst_avoid.point_1.acceleration_or_force.z = NAN;
  obst_avoid.point_1.yaw = NAN;
  obst_avoid.point_1.yaw_rate = -vel.angular.z;

  fillUnusedTrajectoryPoint(obst_avoid.point_2);
  fillUnusedTrajectoryPoint(obst_avoid.point_3);
  fillUnusedTrajectoryPoint(obst_avoid.point_4);
  fillUnusedTrajectoryPoint(obst_avoid.point_5);

  obst_avoid.time_horizon = {NAN, NAN, NAN, NAN, NAN};

  obst_avoid.point_valid = {true, false, false, false, false};
}

void LocalPlannerNode::dynamicReconfigureCallback(
    avoidance::LocalPlannerNodeConfig& config, uint32_t level) {
  std::lock_guard<std::mutex> guard(running_mutex_);
  local_planner_->dynamicReconfigureSetParams(config, level);
  wp_generator_->setSmoothingSpeed(config.smoothing_speed_xy_,
                                   config.smoothing_speed_z_);
  rqt_param_config_ = config;
}

void LocalPlannerNode::publishLaserScan() const {
  if (local_planner_->px4_.param_mpc_col_prev_d > 0) {
    sensor_msgs::LaserScan distance_data_to_fcu;
    local_planner_->getObstacleDistanceData(distance_data_to_fcu);
    mavros_obstacle_distance_pub_.publish(distance_data_to_fcu);
  }
}

void LocalPlannerNode::threadFunction() {
  while (!should_exit_) {
    // wait for data
    {
      std::unique_lock<std::mutex> lk(data_ready_mutex_);
      data_ready_cv_.wait(lk, [this] { return data_ready_ && !should_exit_; });
      data_ready_ = false;
    }

    if (should_exit_) break;

    {
      std::lock_guard<std::mutex> guard(running_mutex_);
      never_run_ = false;
      std::clock_t start_time_ = std::clock();
      local_planner_->runPlanner();
      visualizer_.visualizePlannerData(
          *(local_planner_.get()), newest_waypoint_position_,
          newest_adapted_waypoint_position_, newest_pose_);
      publishLaserScan();
      last_wp_time_ = ros::Time::now();

      ROS_DEBUG("\033[0;35m[OA]Planner calculation time: %2.2f ms \n \033[0m",
                (std::clock() - start_time_) / (double)(CLOCKS_PER_SEC / 1000));
    }
  }
}

void LocalPlannerNode::checkFailsafe(ros::Duration since_last_cloud,
                                     ros::Duration since_start,
                                     bool& planner_is_healthy, bool& hover) {
  ros::Duration timeout_termination =
      ros::Duration(local_planner_->timeout_termination_);
  ros::Duration timeout_critical =
      ros::Duration(local_planner_->timeout_critical_);

  if (since_last_cloud > timeout_termination &&
      since_start > timeout_termination) {
    if (planner_is_healthy) {
      planner_is_healthy = false;
      status_msg_.state = (int)MAV_STATE::MAV_STATE_FLIGHT_TERMINATION;
      ROS_WARN("\033[1;33m Planner abort: missing required data \n \033[0m");
    }
  } else {
    if (since_last_cloud > timeout_critical && since_start > timeout_critical) {
      if (position_received_) {
        hover = true;
        status_msg_.state = (int)MAV_STATE::MAV_STATE_CRITICAL;
        std::string not_received = "";
        for (size_t i = 0; i < cameras_.size(); i++) {
          if (!cameras_[i].received_) {
            not_received.append(" , no cloud received on topic ");
            not_received.append(cameras_[i].topic_);
          }
        }
        if (!canUpdatePlannerInfo()) {
          not_received.append(" , missing transforms ");
        }
        ROS_WARN(
            "\033[1;33m Pointcloud timeout %s (Hovering at current position) "
            "\n "
            "\033[0m",
            not_received.c_str());
      } else {
        ROS_WARN(
            "\033[1;33m Pointcloud timeout: No position received, no WP to "
            "output.... \n \033[0m");
      }
    }
  }
}

void LocalPlannerNode::pointCloudTransformThread(int index) {
  while (!should_exit_) {
    {
      std::unique_lock<std::mutex> lk(*(cameras_[index].cloud_ready_mutex_));
      cameras_[index].cloud_ready_cv_->wait(lk);
    }

    if (should_exit_) break;

    if (tf_listener_->canTransform(
            "/local_origin", cameras_[index].newest_cloud_msg_.header.frame_id,
            ros::Time(0))) {
      try {
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        // transform message to pcl type
        pcl::fromROSMsg(cameras_[index].newest_cloud_msg_, pcl_cloud);

        // remove nan padding
        std::vector<int> dummy_index;
        dummy_index.reserve(pcl_cloud.points.size());
        pcl::removeNaNFromPointCloud(pcl_cloud, pcl_cloud, dummy_index);

        // transform cloud to /local_origin frame
        pcl_ros::transformPointCloud("/local_origin", pcl_cloud, pcl_cloud,
                                     *tf_listener_);

        std::unique_lock<std::mutex> lk(*(cameras_[index].cloud_ready_mutex_));
        cameras_[index].transformed_ = true;
        cameras_[index].pcl_cloud = std::move(pcl_cloud);
      } catch (tf::TransformException& ex) {
        ROS_ERROR("Received an exception trying to transform a pointcloud: %s",
                  ex.what());
      }
    }
    {
      std::unique_lock<std::mutex> lk(*(cameras_[index].trans_ready_mutex_));
      cameras_[index].trans_ready_cv_->notify_one();
    }
  }
}
}
