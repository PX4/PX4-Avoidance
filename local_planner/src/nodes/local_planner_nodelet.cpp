#include "local_planner/local_planner_nodelet.h"

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

LocalPlannerNodelet::LocalPlannerNodelet() : tf_buffer_(5.f), spin_dt_(0.1) {}

LocalPlannerNodelet::~LocalPlannerNodelet() {
  should_exit_ = true;
  {
    std::lock_guard<std::mutex> guard(buffered_transforms_mutex_);
    tf_buffer_cv_.notify_all();
  }

  {
    std::lock_guard<std::mutex> guard(transformed_cloud_mutex_);
    transformed_cloud_cv_.notify_all();
  }

  for (size_t i = 0; i < cameras_.size(); ++i) {
    {
      std::lock_guard<std::mutex> guard(*cameras_[i].camera_mutex_);
      cameras_[i].camera_cv_->notify_all();
    }
    if (cameras_[i].transform_thread_.joinable()) cameras_[i].transform_thread_.join();
  }

  if (worker.joinable()) worker.join();
  if (worker_tf_listener.joinable()) worker_tf_listener.join();

  if (server_ != nullptr) delete server_;
  if (tf_listener_ != nullptr) delete tf_listener_;
}

void LocalPlannerNodelet::onInit() {
  NODELET_DEBUG("Initializing nodelet...");
  InitializeNodelet();

  startNode();

  worker = std::thread(&LocalPlannerNodelet::threadFunction, this);
  worker_tf_listener = std::thread(&LocalPlannerNodelet::transformBufferThread, this);
  // Set up Dynamic Reconfigure Server
  server_ = new dynamic_reconfigure::Server<avoidance::LocalPlannerNodeConfig>(config_mutex_, getPrivateNodeHandle());
  dynamic_reconfigure::Server<avoidance::LocalPlannerNodeConfig>::CallbackType f;
  f = boost::bind(&LocalPlannerNodelet::dynamicReconfigureCallback, this, _1, _2);
  server_->setCallback(f);
}

void LocalPlannerNodelet::InitializeNodelet() {
  nh_ = ros::NodeHandle("");
  nh_private_ = ros::NodeHandle("~");

  local_planner_.reset(new LocalPlanner());
  wp_generator_.reset(new WaypointGenerator());
  avoidance_node_.reset(new AvoidanceNode(nh_, nh_private_));

#ifndef DISABLE_SIMULATION
  world_visualizer_.reset(new WorldVisualizer(nh_, nodelet::Nodelet::getName()));
#endif

  readParams();

  tf_listener_ = new tf::TransformListener(ros::Duration(tf::Transformer::DEFAULT_CACHE_TIME), true);

  // initialize standard subscribers
  pose_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped&>("mavros/local_position/pose", 1,
                                                               &LocalPlannerNodelet::positionCallback, this);
  velocity_sub_ = nh_.subscribe<const geometry_msgs::TwistStamped&>("mavros/local_position/velocity_local", 1,
                                                                    &LocalPlannerNodelet::velocityCallback, this);
  state_sub_ = nh_.subscribe("mavros/state", 1, &LocalPlannerNodelet::stateCallback, this);
  clicked_point_sub_ = nh_.subscribe("clicked_point", 1, &LocalPlannerNodelet::clickedPointCallback, this);
  clicked_goal_sub_ = nh_.subscribe("move_base_simple/goal", 1, &LocalPlannerNodelet::clickedGoalCallback, this);
  fcu_input_sub_ = nh_.subscribe("mavros/trajectory/desired", 1, &LocalPlannerNodelet::fcuInputGoalCallback, this);
  goal_topic_sub_ = nh_.subscribe("input/goal_position", 1, &LocalPlannerNodelet::updateGoalCallback, this);
  distance_sensor_sub_ = nh_.subscribe("mavros/altitude", 1, &LocalPlannerNodelet::distanceSensorCallback, this);
  mavros_vel_setpoint_pub_ = nh_.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
  mavros_pos_setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  mavros_obstacle_free_path_pub_ = nh_.advertise<mavros_msgs::Trajectory>("mavros/trajectory/generated", 10);
  mavros_obstacle_distance_pub_ = nh_.advertise<sensor_msgs::LaserScan>("mavros/obstacle/send", 10);

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

void LocalPlannerNodelet::startNode() {
  ros::TimerOptions timer_options(ros::Duration(spin_dt_), boost::bind(&LocalPlannerNodelet::cmdLoopCallback, this, _1),
                                  &cmdloop_queue_);
  cmdloop_timer_ = nh_.createTimer(timer_options);

  cmdloop_spinner_.reset(new ros::AsyncSpinner(1, &cmdloop_queue_));
  cmdloop_spinner_->start();

  avoidance_node_->init();
}

void LocalPlannerNodelet::readParams() {
  // Parameter from launch file
  Eigen::Vector3d goal_d = goal_position_.cast<double>();
  nh_private_.param<double>("goal_x_param", goal_d.x(), 0.0);
  nh_private_.param<double>("goal_y_param", goal_d.y(), 0.0);
  nh_private_.param<double>("goal_z_param", goal_d.z(), 0.0);
  nh_private_.param<bool>("accept_goal_input_topic", accept_goal_input_topic_, false);
  goal_position_ = goal_d.cast<float>();

  std::vector<std::string> camera_topics;
  nh_private_.getParam(nodelet::Nodelet::getName() + "/pointcloud_topics", camera_topics);

  initializeCameraSubscribers(camera_topics);

  new_goal_ = true;
}

void LocalPlannerNodelet::initializeCameraSubscribers(std::vector<std::string>& camera_topics) {
  cameras_.resize(camera_topics.size());

  for (size_t i = 0; i < camera_topics.size(); i++) {
    cameras_[i].camera_mutex_.reset(new std::mutex);
    cameras_[i].camera_cv_.reset(new std::condition_variable);

    cameras_[i].received_ = false;
    cameras_[i].transformed_ = false;

    cameras_[i].pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        camera_topics[i], 1, boost::bind(&LocalPlannerNodelet::pointCloudCallback, this, _1, i));
    cameras_[i].topic_ = camera_topics[i];
    cameras_[i].transform_thread_ = std::thread(&LocalPlannerNodelet::pointCloudTransformThread, this, i);
  }
}

size_t LocalPlannerNodelet::numTransformedClouds() {
  size_t num_transformed_clouds = 0;
  for (size_t i = 0; i < cameras_.size(); i++) {
    std::lock_guard<std::mutex> transformed_cloud_guard(*(cameras_[i].camera_mutex_));
    if (cameras_[i].transformed_) num_transformed_clouds++;
  }
  return num_transformed_clouds;
}

void LocalPlannerNodelet::updatePlannerInfo() {
  // update the point cloud
  local_planner_->original_cloud_vector_.resize(cameras_.size());
  for (size_t i = 0; i < cameras_.size(); ++i) {
    std::lock_guard<std::mutex> transformed_cloud_guard(*(cameras_[i].camera_mutex_));
    try {
      std::swap(local_planner_->original_cloud_vector_[i], cameras_[i].transformed_cloud_);
      cameras_[i].transformed_cloud_.clear();
      cameras_[i].transformed_ = false;
      local_planner_->setFOV(i, cameras_[i].fov_fcu_frame_);
      wp_generator_->setFOV(i, cameras_[i].fov_fcu_frame_);
    } catch (tf::TransformException& ex) {
      ROS_ERROR("Received an exception trying to transform a pointcloud: %s", ex.what());
    }
  }

  // update pose
  local_planner_->setState(newest_position_, velocity_, newest_orientation_);

  // update state
  local_planner_->currently_armed_ = armed_;

  // update goal
  if (new_goal_) {
    local_planner_->setGoal(goal_position_);
    local_planner_->setPreviousGoal(prev_goal_position_);
    new_goal_ = false;
  }

  // update last sent waypoint
  local_planner_->last_sent_waypoint_ = newest_waypoint_position_;

  // update the Firmware parameters
  local_planner_->px4_ = avoidance_node_->getPX4Parameters();
  local_planner_->mission_item_speed_ = avoidance_node_->getMissionItemSpeed();
}

void LocalPlannerNodelet::positionCallback(const geometry_msgs::PoseStamped& msg) {
  last_position_ = newest_position_;
  newest_position_ = toEigen(msg.pose.position);
  newest_orientation_ = toEigen(msg.pose.orientation);

  position_received_ = true;
}

void LocalPlannerNodelet::velocityCallback(const geometry_msgs::TwistStamped& msg) {
  velocity_ = toEigen(msg.twist.linear);
}

void LocalPlannerNodelet::stateCallback(const mavros_msgs::State& msg) {
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
  } else if (msg.mode == "AUTO.LOITER") {
    nav_state_ = NavigationState::auto_loiter;
  } else if (msg.mode == "OFFBOARD") {
    nav_state_ = NavigationState::offboard;
  } else {
    nav_state_ = NavigationState::none;
  }
}

void LocalPlannerNodelet::cmdLoopCallback(const ros::TimerEvent& event) {
  std::lock_guard<std::mutex> lock(waypoints_mutex_);
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
        ROS_WARN("\033[1;33m Planner abort: missing required data from FCU \n \033[0m");
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

  // send waypoint
  if (avoidance_node_->getSystemStatus() == MAV_STATE::MAV_STATE_ACTIVE) {
    calculateWaypoints(hover_);
  }

  position_received_ = false;

  return;
}

void LocalPlannerNodelet::setSystemStatus(MAV_STATE state) { avoidance_node_->setSystemStatus(state); }

MAV_STATE LocalPlannerNodelet::getSystemStatus() { return avoidance_node_->getSystemStatus(); }

void LocalPlannerNodelet::calculateWaypoints(bool hover) {
  bool is_airborne = armed_ && (nav_state_ != NavigationState::none);

  wp_generator_->updateState(newest_position_, newest_orientation_, goal_position_, prev_goal_position_, velocity_,
                             hover, is_airborne, nav_state_, is_land_waypoint_, is_takeoff_waypoint_,
                             desired_velocity_);
  waypointResult result = wp_generator_->getWaypoints();

  Eigen::Vector3f closest_pt = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f deg60_pt = Eigen::Vector3f(NAN, NAN, NAN);
  wp_generator_->getOfftrackPointsForVisualization(closest_pt, deg60_pt);

  last_waypoint_position_ = newest_waypoint_position_;
  newest_waypoint_position_ = result.smoothed_goto_position;
  last_adapted_waypoint_position_ = newest_adapted_waypoint_position_;
  newest_adapted_waypoint_position_ = result.adapted_goto_position;

  // visualize waypoint topics
  visualizer_.visualizeWaypoints(result.goto_position, result.adapted_goto_position, result.smoothed_goto_position);
  visualizer_.publishPaths(last_position_, newest_position_, last_waypoint_position_, newest_waypoint_position_,
                           last_adapted_waypoint_position_, newest_adapted_waypoint_position_);
  visualizer_.publishCurrentSetpoint(toTwist(result.linear_velocity_wp, result.angular_velocity_wp),
                                     result.waypoint_type, newest_position_);

  visualizer_.publishOfftrackPoints(closest_pt, deg60_pt);

  // send waypoints to mavros
  mavros_msgs::Trajectory obst_free_path = {};
  transformToTrajectory(obst_free_path, toPoseStamped(result.position_wp, result.orientation_wp),
                        toTwist(result.linear_velocity_wp, result.angular_velocity_wp));
  mavros_pos_setpoint_pub_.publish(toPoseStamped(result.position_wp, result.orientation_wp));

  mavros_obstacle_free_path_pub_.publish(obst_free_path);
}

void LocalPlannerNodelet::clickedPointCallback(const geometry_msgs::PointStamped& msg) {
  printPointInfo(msg.point.x, msg.point.y, msg.point.z);
}

void LocalPlannerNodelet::clickedGoalCallback(const geometry_msgs::PoseStamped& msg) {
  new_goal_ = true;
  prev_goal_position_ = goal_position_;
  goal_position_ = toEigen(msg.pose.position);
  /* Selecting the goal from Rviz sets x and y. Get the z coordinate set in
   * the launch file */
  goal_position_.z() = local_planner_->getGoal().z();
}

void LocalPlannerNodelet::updateGoalCallback(const visualization_msgs::MarkerArray& msg) {
  if (accept_goal_input_topic_ && msg.markers.size() > 0) {
    prev_goal_position_ = goal_position_;
    goal_position_ = toEigen(msg.markers[0].pose.position);
    new_goal_ = true;
  }
}

void LocalPlannerNodelet::fcuInputGoalCallback(const mavros_msgs::Trajectory& msg) {
  bool update =
      ((avoidance::toEigen(msg.point_2.position) - avoidance::toEigen(goal_mission_item_msg_.pose.position)).norm() >
       0.01) ||
      !std::isfinite(goal_position_(0)) || !std::isfinite(goal_position_(1));
  if ((msg.point_valid[0] == true) && update) {
    new_goal_ = true;
    prev_goal_position_ = goal_position_;
    goal_position_ = toEigen(msg.point_1.position);
    desired_velocity_ = toEigen(msg.point_1.velocity);
    is_land_waypoint_ = (msg.command[0] == static_cast<int>(MavCommand::MAV_CMD_NAV_LAND));
    is_takeoff_waypoint_ = (msg.command[0] == static_cast<int>(MavCommand::MAV_CMD_NAV_TAKEOFF));
  }
  if (msg.point_valid[1] == true) {
    goal_mission_item_msg_.pose.position = msg.point_2.position;
    if (msg.command[1] == UINT16_MAX) {
      goal_position_ = toEigen(msg.point_2.position);
      desired_velocity_ << NAN, NAN, NAN;
    }
    desired_yaw_setpoint_ = msg.point_2.yaw;
    desired_yaw_speed_setpoint_ = msg.point_2.yaw_rate;
  }
}

void LocalPlannerNodelet::distanceSensorCallback(const mavros_msgs::Altitude& msg) {
  if (!std::isnan(msg.bottom_clearance)) {
    ground_distance_msg_ = msg;
  }
}

void LocalPlannerNodelet::transformBufferThread() {
  // grab transforms from tf and store them into the buffer
  while (!should_exit_) {
    {
      std::lock_guard<std::mutex> guard(buffered_transforms_mutex_);
      for (auto const& frame_pair : buffered_transforms_) {
        tf::StampedTransform transform;

        if (tf_listener_->canTransform(frame_pair.second, frame_pair.first, ros::Time(0))) {
          try {
            tf_listener_->lookupTransform(frame_pair.second, frame_pair.first, ros::Time(0), transform);
            tf_buffer_.insertTransform(frame_pair.first, frame_pair.second, transform);
          } catch (tf::TransformException& ex) {
            ROS_ERROR("Received an exception trying to get transform: %s", ex.what());
          }
        }
      }
      tf_buffer_cv_.notify_all();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void LocalPlannerNodelet::printPointInfo(double x, double y, double z) {
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

void LocalPlannerNodelet::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg, int index) {
  std::lock_guard<std::mutex> lck(*(cameras_[index].camera_mutex_));

  auto timeSinceLast = [&]() -> ros::Duration {
    ros::Time lastCloudReceived = pcl_conversions::fromPCL(cameras_[index].untransformed_cloud_.header.stamp);
    return msg->header.stamp - lastCloudReceived;
  };

  if (cameras_[index].received_ && timeSinceLast() < ros::Duration(0.3)) {
    return;
  }

  pcl::fromROSMsg(*msg, cameras_[index].untransformed_cloud_);
  cameras_[index].received_ = true;
  cameras_[index].camera_cv_->notify_all();

  // this runs once at the beginning to get the transforms
  if (!cameras_[index].transform_registered_) {
    std::lock_guard<std::mutex> tf_list_guard(buffered_transforms_mutex_);
    std::pair<std::string, std::string> transform_frames;
    transform_frames.first = msg->header.frame_id;
    transform_frames.second = "local_origin";
    buffered_transforms_.push_back(transform_frames);
    transform_frames.second = "fcu";
    buffered_transforms_.push_back(transform_frames);
    cameras_[index].transform_registered_ = true;
  }
}

void LocalPlannerNodelet::dynamicReconfigureCallback(avoidance::LocalPlannerNodeConfig& config, uint32_t level) {
  std::lock_guard<std::mutex> guard(running_mutex_);
  local_planner_->dynamicReconfigureSetParams(config, level);
  wp_generator_->setSmoothingSpeed(config.smoothing_speed_xy_, config.smoothing_speed_z_);
  rqt_param_config_ = config;
}

void LocalPlannerNodelet::publishLaserScan() const {
  // inverted logic to make sure values like NAN default to sending the message
  if (!(local_planner_->px4_.param_cp_dist < 0)) {
    sensor_msgs::LaserScan distance_data_to_fcu;
    local_planner_->getObstacleDistanceData(distance_data_to_fcu);

    // only send message if planner had a chance to fill it with valid data
    if (distance_data_to_fcu.angle_increment > 0.f) {
      mavros_obstacle_distance_pub_.publish(distance_data_to_fcu);
    }
  }
}

void LocalPlannerNodelet::threadFunction() {
  while (!should_exit_) {
    ros::Time start_time = ros::Time::now();

    while ((cameras_.size() == 0 || cameras_.size() != numTransformedClouds()) && !should_exit_) {
      std::unique_lock<std::mutex> lock(transformed_cloud_mutex_);
      transformed_cloud_cv_.wait_for(lock, std::chrono::milliseconds(5000));
    }

    if (should_exit_) break;

    {
      std::lock_guard<std::mutex> guard(running_mutex_);
      updatePlannerInfo();
      local_planner_->runPlanner();

      visualizer_.visualizePlannerData(*(local_planner_.get()), newest_waypoint_position_,
                                       newest_adapted_waypoint_position_, newest_position_, newest_orientation_);
      publishLaserScan();

      std::lock_guard<std::mutex> lock(waypoints_mutex_);
      wp_generator_->setPlannerInfo(local_planner_->getAvoidanceOutput());
      last_wp_time_ = ros::Time::now();
    }

    if (should_exit_) break;

    ros::Duration loop_time = last_wp_time_ - start_time;
    ros::Duration required_delay = ros::Duration(spin_dt_) - loop_time;
    if (required_delay > ros::Duration(0)) {
      required_delay.sleep();
    }
  }
}

void LocalPlannerNodelet::checkFailsafe(ros::Duration since_last_cloud, ros::Duration since_start, bool& hover) {
  avoidance_node_->checkFailsafe(since_last_cloud, since_start, hover);
}

void LocalPlannerNodelet::pointCloudTransformThread(int index) {
  while (!should_exit_) {
    bool waiting_on_transform = false;
    bool waiting_on_cloud = false;
    {
      std::lock_guard<std::mutex> camera_lock(*(cameras_[index].camera_mutex_));

      if (cameras_[index].received_) {
        tf::StampedTransform cloud_transform;
        tf::StampedTransform fcu_transform;

        if (tf_buffer_.getTransform(cameras_[index].untransformed_cloud_.header.frame_id, "local_origin",
                                    pcl_conversions::fromPCL(cameras_[index].untransformed_cloud_.header.stamp),
                                    cloud_transform) &&
            tf_buffer_.getTransform(cameras_[index].untransformed_cloud_.header.frame_id, "fcu",
                                    pcl_conversions::fromPCL(cameras_[index].untransformed_cloud_.header.stamp),
                                    fcu_transform)) {
          // remove nan padding and compute fov
          pcl::PointCloud<pcl::PointXYZ> maxima = removeNaNAndGetMaxima(cameras_[index].untransformed_cloud_);

          // update point cloud FOV
          pcl_ros::transformPointCloud(maxima, maxima, fcu_transform);
          updateFOVFromMaxima(cameras_[index].fov_fcu_frame_, maxima);

          // transform cloud to local_origin frame
          pcl_ros::transformPointCloud(cameras_[index].untransformed_cloud_, cameras_[index].transformed_cloud_,
                                       cloud_transform);
          cameras_[index].transformed_cloud_.header.frame_id = "local_origin";
          cameras_[index].transformed_cloud_.header.stamp = cameras_[index].untransformed_cloud_.header.stamp;

          cameras_[index].transformed_ = true;
          cameras_[index].received_ = false;
          waiting_on_cloud = true;
          std::lock_guard<std::mutex> lock(transformed_cloud_mutex_);
          transformed_cloud_cv_.notify_all();
        } else {
          waiting_on_transform = true;
        }
      } else {
        waiting_on_cloud = true;
      }
    }

    if (should_exit_) {
      break;
    }

    if (waiting_on_transform) {
      std::unique_lock<std::mutex> lck(buffered_transforms_mutex_);
      tf_buffer_cv_.wait_for(lck, std::chrono::milliseconds(5000));
    } else if (waiting_on_cloud) {
      std::unique_lock<std::mutex> lck(*(cameras_[index].camera_mutex_));
      cameras_[index].camera_cv_->wait_for(lck, std::chrono::milliseconds(5000));
    }
  }
}
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(avoidance::LocalPlannerNodelet, nodelet::Nodelet);
