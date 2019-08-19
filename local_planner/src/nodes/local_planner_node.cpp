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

LocalPlannerNode::LocalPlannerNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                                   const bool tf_spin_thread)
    : nh_(nh), nh_private_(nh_private), spin_dt_(0.1), tf_buffer_(5.f) {
  local_planner_.reset(new LocalPlanner());
  wp_generator_.reset(new WaypointGenerator());
  avoidance_node_.reset(new AvoidanceNode(nh, nh_private));

#ifndef DISABLE_SIMULATION
  world_visualizer_.reset(new WorldVisualizer(nh_));
#endif

  readParams();

  tf_listener_ = new tf::TransformListener(ros::Duration(tf::Transformer::DEFAULT_CACHE_TIME), tf_spin_thread);

  // Set up Dynamic Reconfigure Server
  server_ = new dynamic_reconfigure::Server<avoidance::LocalPlannerNodeConfig>(config_mutex_, nh_);
  dynamic_reconfigure::Server<avoidance::LocalPlannerNodeConfig>::CallbackType f;
  f = boost::bind(&LocalPlannerNode::dynamicReconfigureCallback, this, _1, _2);
  server_->setCallback(f);

  // initialize standard subscribers
  pose_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped&>("/mavros/local_position/pose", 1,
                                                               &LocalPlannerNode::positionCallback, this);
  velocity_sub_ = nh_.subscribe<const geometry_msgs::TwistStamped&>("/mavros/local_position/velocity_local", 1,
                                                                    &LocalPlannerNode::velocityCallback, this);
  state_sub_ = nh_.subscribe("/mavros/state", 1, &LocalPlannerNode::stateCallback, this);
  clicked_point_sub_ = nh_.subscribe("/clicked_point", 1, &LocalPlannerNode::clickedPointCallback, this);
  clicked_goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &LocalPlannerNode::clickedGoalCallback, this);
  fcu_input_sub_ = nh_.subscribe("/mavros/trajectory/desired", 1, &LocalPlannerNode::fcuInputGoalCallback, this);
  goal_topic_sub_ = nh_.subscribe("/input/goal_position", 1, &LocalPlannerNode::updateGoalCallback, this);
  distance_sensor_sub_ = nh_.subscribe("/mavros/altitude", 1, &LocalPlannerNode::distanceSensorCallback, this);
  mavros_vel_setpoint_pub_ = nh_.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
  mavros_pos_setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  mavros_obstacle_free_path_pub_ = nh_.advertise<mavros_msgs::Trajectory>("/mavros/trajectory/generated", 10);
  mavros_obstacle_distance_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/mavros/obstacle/send", 10);

  // initialize visualization topics
  visualizer_.initializePublishers(nh_);

  // pass initial goal into local planner
  local_planner_->applyGoal();

  setSystemStatus(MAV_STATE::MAV_STATE_BOOT);

  hover_ = false;
  planner_is_healthy_ = true;
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
  ros::TimerOptions timer_options(ros::Duration(spin_dt_), boost::bind(&LocalPlannerNode::cmdLoopCallback, this, _1),
                                  &cmdloop_queue_);
  cmdloop_timer_ = nh_.createTimer(timer_options);

  cmdloop_spinner_.reset(new ros::AsyncSpinner(1, &cmdloop_queue_));
  cmdloop_spinner_->start();

  avoidance_node_->init();
}

void LocalPlannerNode::readParams() {
  // Parameter from launch file
  auto goal = toPoint(local_planner_->getGoal());
  nh_.param<double>("goal_x_param", goal.x, 9.0);
  nh_.param<double>("goal_y_param", goal.y, 13.0);
  nh_.param<double>("goal_z_param", goal.z, 3.5);
  nh_.param<bool>("accept_goal_input_topic", accept_goal_input_topic_, false);

  std::vector<std::string> camera_topics;
  nh_.getParam("pointcloud_topics", camera_topics);
  initializeCameraSubscribers(camera_topics);

  goal_msg_.pose.position = goal;
  new_goal_ = true;
}

void LocalPlannerNode::initializeCameraSubscribers(std::vector<std::string>& camera_topics) {
  cameras_.resize(camera_topics.size());

  for (size_t i = 0; i < camera_topics.size(); i++) {
    cameras_[i].cloud_msg_mutex_.reset(new std::mutex);
    cameras_[i].transformed_cloud_mutex_.reset(new std::mutex);
    cameras_[i].cloud_ready_cv_.reset(new std::condition_variable);
    cameras_[i].transformed_ = false;

    cameras_[i].pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        camera_topics[i], 1, boost::bind(&LocalPlannerNode::pointCloudCallback, this, _1, i));
    cameras_[i].topic_ = camera_topics[i];
    cameras_[i].received_ = false;
    cameras_[i].transform_thread_ = std::thread(&LocalPlannerNode::pointCloudTransformThread, this, i);
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
    std::lock_guard<std::mutex> transformed_cloud_guard(*(cameras_[i].transformed_cloud_mutex_));
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

void LocalPlannerNode::updatePlannerInfo() {
  // update the point cloud
  local_planner_->original_cloud_vector_.clear();
  for (size_t i = 0; i < cameras_.size(); ++i) {
    std::lock_guard<std::mutex> transformed_cloud_guard(*(cameras_[i].transformed_cloud_mutex_));
    try {
      local_planner_->original_cloud_vector_.push_back(std::move(cameras_[i].pcl_cloud));
      cameras_[i].transformed_ = false;
      local_planner_->setFOV(i, cameras_[i].fov_fcu_frame_);
      wp_generator_->setFOV(i, cameras_[i].fov_fcu_frame_);
    } catch (tf::TransformException& ex) {
      ROS_ERROR("Received an exception trying to transform a pointcloud: %s", ex.what());
    }
  }

  // update pose
  local_planner_->setState(toEigen(newest_pose_.pose.position), toEigen(vel_msg_.twist.linear),
                           toEigen(newest_pose_.pose.orientation));

  // update state
  local_planner_->currently_armed_ = armed_;

  // update goal
  if (new_goal_) {
    local_planner_->setGoal(toEigen(goal_msg_.pose.position));
    new_goal_ = false;
  }

  // update last sent waypoint
  local_planner_->last_sent_waypoint_ = toEigen(newest_waypoint_position_);
}

void LocalPlannerNode::positionCallback(const geometry_msgs::PoseStamped& msg) {
  last_pose_ = newest_pose_;
  newest_pose_ = msg;
  position_received_ = true;
}

void LocalPlannerNode::velocityCallback(const geometry_msgs::TwistStamped& msg) { vel_msg_ = msg; }

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

  // Process callbacks & wait for a position update
  ros::Time start_query_position = ros::Time::now();
  while (!position_received_ && ros::ok()) {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    ros::Duration since_query = ros::Time::now() - start_query_position;
    if (since_query > ros::Duration(local_planner_->timeout_termination_)) {
      setSystemStatus(MAV_STATE::MAV_STATE_FLIGHT_TERMINATION);
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

  checkFailsafe(since_last_cloud, since_start, hover_);

  // If planner is not running, update planner info and get last results
  updatePlanner();

  // update the Firmware paramters
  local_planner_->px4_ = avoidance_node_->getPX4Parameters();

  local_planner_->mission_item_speed_ = avoidance_node_->getMissionItemSpeed();

  // send waypoint
  if (avoidance_node_->getSystemStatus() == MAV_STATE::MAV_STATE_ACTIVE) calculateWaypoints(hover_);

  position_received_ = false;

  return;
}

void LocalPlannerNode::setSystemStatus(MAV_STATE state) { avoidance_node_->setSystemStatus(state); }

MAV_STATE LocalPlannerNode::getSystemStatus() { return avoidance_node_->getSystemStatus(); }

void LocalPlannerNode::calculateWaypoints(bool hover) {
  bool is_airborne = armed_ && (nav_state_ != NavigationState::none);

  wp_generator_->updateState(toEigen(newest_pose_.pose.position), toEigen(newest_pose_.pose.orientation),
                             toEigen(goal_msg_.pose.position), toEigen(prev_goal_.pose.position),
                             toEigen(vel_msg_.twist.linear), hover, is_airborne, nav_state_, is_land_waypoint_,
                             is_takeoff_waypoint_, toEigen(desired_vel_msg_.twist.linear));
  waypointResult result = wp_generator_->getWaypoints();

  Eigen::Vector3f closest_pt = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f deg60_pt = Eigen::Vector3f(NAN, NAN, NAN);
  wp_generator_->getOfftrackPointsForVisualization(closest_pt, deg60_pt);

  last_waypoint_position_ = newest_waypoint_position_;
  newest_waypoint_position_ = toPoint(result.smoothed_goto_position);
  last_adapted_waypoint_position_ = newest_adapted_waypoint_position_;
  newest_adapted_waypoint_position_ = toPoint(result.adapted_goto_position);

  // visualize waypoint topics
  visualizer_.visualizeWaypoints(result.goto_position, result.adapted_goto_position, result.smoothed_goto_position);
  visualizer_.publishPaths(last_pose_.pose.position, newest_pose_.pose.position, last_waypoint_position_,
                           newest_waypoint_position_, last_adapted_waypoint_position_,
                           newest_adapted_waypoint_position_);
  visualizer_.publishCurrentSetpoint(toTwist(result.linear_velocity_wp, result.angular_velocity_wp),
                                     result.waypoint_type, newest_pose_.pose.position);

  visualizer_.publishOfftrackPoints(closest_pt, deg60_pt);

  // send waypoints to mavros
  mavros_msgs::Trajectory obst_free_path = {};
  transformToTrajectory(obst_free_path, toPoseStamped(result.position_wp, result.orientation_wp),
                        toTwist(result.linear_velocity_wp, result.angular_velocity_wp));
  mavros_pos_setpoint_pub_.publish(toPoseStamped(result.position_wp, result.orientation_wp));

  mavros_obstacle_free_path_pub_.publish(obst_free_path);
}

void LocalPlannerNode::clickedPointCallback(const geometry_msgs::PointStamped& msg) {
  printPointInfo(msg.point.x, msg.point.y, msg.point.z);
}

void LocalPlannerNode::clickedGoalCallback(const geometry_msgs::PoseStamped& msg) {
  new_goal_ = true;
  goal_msg_ = msg;
  /* Selecting the goal from Rviz sets x and y. Get the z coordinate set in
   * the launch file */
  goal_msg_.pose.position.z = local_planner_->getGoal().z();
}

void LocalPlannerNode::updateGoalCallback(const visualization_msgs::MarkerArray& msg) {
  if (accept_goal_input_topic_ && msg.markers.size() > 0) {
    goal_msg_.pose = msg.markers[0].pose;
    new_goal_ = true;
  }
}

void LocalPlannerNode::fcuInputGoalCallback(const mavros_msgs::Trajectory& msg) {
  bool update =
      ((avoidance::toEigen(msg.point_2.position) - avoidance::toEigen(goal_mission_item_msg_.pose.position)).norm() >
       0.01) ||
      !std::isfinite(goal_msg_.pose.position.x) || !std::isfinite(goal_msg_.pose.position.y);
  if ((msg.point_valid[0] == true) && update) {
    new_goal_ = true;
    prev_goal_ = goal_msg_;
    goal_msg_.pose.position = msg.point_1.position;
    desired_vel_msg_.twist.linear = msg.point_1.velocity;
    is_land_waypoint_ = (msg.command[0] == static_cast<int>(MavCommand::MAV_CMD_NAV_LAND));
    is_takeoff_waypoint_ = (msg.command[0] == static_cast<int>(MavCommand::MAV_CMD_NAV_TAKEOFF));
  }
  if (msg.point_valid[1] == true) {
    goal_mission_item_msg_.pose.position = msg.point_2.position;
    if (msg.command[1] == UINT16_MAX) {
      goal_msg_.pose.position = msg.point_2.position;
      desired_vel_msg_.twist.linear.x = NAN;
      desired_vel_msg_.twist.linear.y = NAN;
      desired_vel_msg_.twist.linear.z = NAN;
    }
    desired_yaw_setpoint_ = msg.point_2.yaw;
    desired_yaw_speed_setpoint_ = msg.point_2.yaw_rate;
  }
}

void LocalPlannerNode::distanceSensorCallback(const mavros_msgs::Altitude& msg) {
  if (!std::isnan(msg.bottom_clearance)) {
    ground_distance_msg_ = msg;
  }
}

void LocalPlannerNode::transformBufferThread() {
  // wait until all pointclouds were received for the first time and added to the transform list
  while (!should_exit_) {
    bool all_tf_registered = true;
    for (auto const& camera : cameras_) {
      all_tf_registered = all_tf_registered && camera.transform_registered_;
    }
    if (all_tf_registered) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  // grab transforms from tf and store them into the buffer
  while (!should_exit_) {
    for (auto const& frame_pair : buffered_transforms_) {
      tf::StampedTransform transform;

      if (tf_listener_->canTransform(frame_pair.second, frame_pair.first, ros::Time(0))) {
        try {
          tf_listener_->lookupTransform(frame_pair.second, frame_pair.first, ros::Time(0), transform);
          tf_buffer_.insertTransform(frame_pair.first, frame_pair.second, transform);
        } catch (tf::TransformException& ex) {
          ROS_ERROR("Received an exception trying to transform a pointcloud: %s", ex.what());
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void LocalPlannerNode::printPointInfo(double x, double y, double z) {
  Eigen::Vector3f drone_pos = local_planner_->getPosition();
  int beta_z = floor((atan2(x - drone_pos.x(), y - drone_pos.y()) * 180.0 / M_PI));  //(-180. +180]
  int beta_e = floor((atan((z - drone_pos.z()) / (Eigen::Vector2f(x, y) - drone_pos.topRows<2>()).norm()) * 180.0 /
                      M_PI));  //(-90.+90)

  beta_z = beta_z + (ALPHA_RES - beta_z % ALPHA_RES);  //[-170,+190]
  beta_e = beta_e + (ALPHA_RES - beta_e % ALPHA_RES);  //[-80,+90]

  ROS_INFO("----- Point: %f %f %f -----\n", x, y, z);
  ROS_INFO("Elevation %d Azimuth %d \n", beta_e, beta_z);
  ROS_INFO("-------------------------------------------- \n");
}

void LocalPlannerNode::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg, int index) {
  std::lock_guard<std::mutex> lck(*(cameras_[index].cloud_msg_mutex_));
  cameras_[index].newest_cloud_msg_ = *msg;  // FIXME: avoid a copy
  cameras_[index].received_ = true;
  if (!cameras_[index].transform_registered_) {
    std::pair<std::string, std::string> transform_frames;
    transform_frames.first = msg->header.frame_id;
    transform_frames.second = "/local_origin";
    buffered_transforms_.push_back(transform_frames);
    cameras_[index].transform_registered_ = true;
  }
  cameras_[index].cloud_ready_cv_->notify_one();
}

void LocalPlannerNode::dynamicReconfigureCallback(avoidance::LocalPlannerNodeConfig& config, uint32_t level) {
  std::lock_guard<std::mutex> guard(running_mutex_);
  local_planner_->dynamicReconfigureSetParams(config, level);
  wp_generator_->setSmoothingSpeed(config.smoothing_speed_xy_, config.smoothing_speed_z_);
  rqt_param_config_ = config;
}

void LocalPlannerNode::publishLaserScan() const {
  // inverted logic to make sure values like NAN default to sending the message
  if (!(local_planner_->px4_.param_mpc_col_prev_d < 0)) {
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
      std::clock_t start_time_ = std::clock();
      local_planner_->runPlanner();
      visualizer_.visualizePlannerData(*(local_planner_.get()), newest_waypoint_position_,
                                       newest_adapted_waypoint_position_, newest_pose_);
      publishLaserScan();
      last_wp_time_ = ros::Time::now();

      ROS_DEBUG("\033[0;35m[OA]Planner calculation time: %2.2f ms \n \033[0m",
                (std::clock() - start_time_) / (double)(CLOCKS_PER_SEC / 1000));
    }
  }
}

void LocalPlannerNode::checkFailsafe(ros::Duration since_last_cloud, ros::Duration since_start, bool& hover) {
  avoidance_node_->checkFailsafe(since_last_cloud, since_start, hover);
}

void LocalPlannerNode::pointCloudTransformThread(int index) {
  while (!should_exit_) {
    {
      std::unique_lock<std::mutex> cloud_msg_lock(*(cameras_[index].cloud_msg_mutex_));
      cameras_[index].cloud_ready_cv_->wait(cloud_msg_lock);
    }
    while (cameras_[index].transformed_ == false) {
      if (should_exit_) break;

      std::unique_ptr<std::lock_guard<std::mutex>> cloud_msg_lock(
          new std::lock_guard<std::mutex>(*(cameras_[index].cloud_msg_mutex_)));

      tf::StampedTransform transform;
      if (tf_buffer_.getTransform(cameras_[index].newest_cloud_msg_.header.frame_id, "/local_origin",
                                  cameras_[index].newest_cloud_msg_.header.stamp, transform)) {
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        // transform message to pcl type
        pcl::fromROSMsg(cameras_[index].newest_cloud_msg_, pcl_cloud);
        cloud_msg_lock.reset();

        // remove nan padding and compute fov
        pcl::PointCloud<pcl::PointXYZ> maxima = removeNaNAndGetMaxima(pcl_cloud);
        pcl_ros::transformPointCloud("fcu", maxima, maxima, *tf_listener_);
        updateFOVFromMaxima(cameras_[index].fov_fcu_frame_, maxima);

        // transform cloud to /local_origin frame
        pcl_ros::transformPointCloud(pcl_cloud, pcl_cloud, transform);
        pcl_cloud.header.frame_id = "/local_origin";

        std::lock_guard<std::mutex> transformed_cloud_guard(*(cameras_[index].transformed_cloud_mutex_));
        cameras_[index].transformed_ = true;
        cameras_[index].pcl_cloud = std::move(pcl_cloud);

      } else {
        cloud_msg_lock.reset();
        ros::Duration(0.001).sleep();
      }
    }
  }
}
}
