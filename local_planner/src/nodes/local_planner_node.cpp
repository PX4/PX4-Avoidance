#include "local_planner_node.h"

#include "common.h"
#include "local_planner.h"
#include "planner_functions.h"
#include "tree_node.h"
#include "waypoint_generator.h"

#include <boost/algorithm/string.hpp>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

namespace avoidance {

LocalPlannerNode::LocalPlannerNode() {
  local_planner_.reset(new LocalPlanner());
  wp_generator_.reset(new WaypointGenerator());
  nh_ = ros::NodeHandle("~");
  readParams();

  // Set up Dynamic Reconfigure Server
  server_ = new dynamic_reconfigure::Server<avoidance::LocalPlannerNodeConfig>(
      config_mutex_, nh_);
  dynamic_reconfigure::Server<avoidance::LocalPlannerNodeConfig>::CallbackType
      f;
  f = boost::bind(&LocalPlannerNode::dynamicReconfigureCallback, this, _1, _2);
  server_->setCallback(f);

  // disable memory if using more than one camera
  if (cameras_.size() > 1) {
    config_mutex_.lock();
    rqt_param_config_.reproj_age_ = 0;
    config_mutex_.unlock();
    server_->updateConfig(rqt_param_config_);
    dynamicReconfigureCallback(rqt_param_config_, 1);
  }

  // initialize subscribers and publishers
  pose_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped&>(
      "/mavros/local_position/pose", 1, &LocalPlannerNode::positionCallback,
      this);
  velocity_sub_ = nh_.subscribe<const geometry_msgs::TwistStamped&>(
      "/mavros/local_position/velocity", 1, &LocalPlannerNode::velocityCallback,
      this);
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

  world_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/world", 1);
  drone_pub_ = nh_.advertise<visualization_msgs::Marker>("/drone", 1);
  local_pointcloud_pub_ =
      nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/local_pointcloud", 1);
  reprojected_points_pub_ =
      nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/reprojected_points", 1);
  bounding_box_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/bounding_box", 1);
  ground_measurement_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/ground_measurement", 1);
  original_wp_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/original_waypoint", 1);
  adapted_wp_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/adapted_waypoint", 1);
  smoothed_wp_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/smoothed_waypoint", 1);
  complete_tree_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/complete_tree", 1);
  tree_path_pub_ = nh_.advertise<visualization_msgs::Marker>("/tree_path", 1);
  marker_blocked_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/blocked_marker", 1);
  marker_rejected_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/rejected_marker", 1);
  marker_candidates_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/candidates_marker", 1);
  marker_FOV_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/FOV_marker", 1);
  marker_selected_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/selected_marker", 1);
  marker_goal_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/goal_position", 1);
  path_actual_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/path_actual", 1);
  path_waypoint_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/path_waypoint", 1);
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
  current_waypoint_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/current_setpoint", 1);
  takeoff_pose_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/take_off_pose", 1);
  offboard_pose_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/offboard_pose", 1);
  initial_height_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/initial_height", 1);
  histogram_image_pub_ =
      nh_.advertise<sensor_msgs::Image>("/histogram_image", 1);

  mavros_set_mode_client_ =
      nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

  local_planner_->applyGoal();
}

LocalPlannerNode::~LocalPlannerNode() { delete server_; }

void LocalPlannerNode::readParams() {
  // Parameter from launch file
  auto goal = local_planner_->getGoal();
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

  // Read in parameter for waypoint generator
  waypointGenerator_params new_params;
  nh_.param<float>("goal_acceptance_radius_in",
                   new_params.goal_acceptance_radius_in, 0.5f);
  nh_.param<float>("goal_acceptance_radius_out",
                   new_params.goal_acceptance_radius_out, 1.5f);
  nh_.param<float>("factor_close_to_goal_start_speed_limitation",
                   new_params.factor_close_to_goal_start_speed_limitation,
                   3.0f);
  nh_.param<float>("factor_close_to_goal_stop_speed_limitation",
                   new_params.factor_close_to_goal_stop_speed_limitation, 4.0f);
  nh_.param<float>("max_speed_close_to_goal_factor",
                   new_params.max_speed_close_to_goal_factor, 0.1f);
  nh_.param<float>("min_speed_close_to_goal",
                   new_params.min_speed_close_to_goal, 0.5f);

  wp_generator_->param_ = new_params;
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
    cameras_[i].pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        camera_topics[i], 1,
        boost::bind(&LocalPlannerNode::pointCloudCallback, this, _1, i));
    cameras_[i].topic_ = camera_topics[i];

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
  }
}

size_t LocalPlannerNode::numReceivedClouds() {
  size_t num_received_clouds = 0;
  for (size_t i = 0; i < cameras_.size(); i++) {
    if (cameras_[i].received_) num_received_clouds++;
  }
  return num_received_clouds;
}

bool LocalPlannerNode::canUpdatePlannerInfo() {
  // Check if we have a transformation available at the time of the current
  // point cloud
  size_t missing_transforms = 0;
  for (size_t i = 0; i < cameras_.size(); ++i) {
    if (!tf_listener_.canTransform(
            "/local_origin", cameras_[i].newest_cloud_msg_.header.frame_id,
            ros::Time(0))) {
      missing_transforms++;
    }
  }

  return missing_transforms == 0;
}
void LocalPlannerNode::updatePlannerInfo() {
  // update the point cloud
  local_planner_->complete_cloud_.clear();
  for (size_t i = 0; i < cameras_.size(); ++i) {
    sensor_msgs::PointCloud2 pc2cloud_world;
    pcl::PointCloud<pcl::PointXYZ> complete_cloud;
    try {
      tf::StampedTransform transform;
      tf_listener_.lookupTransform(
          "/local_origin", cameras_[i].newest_cloud_msg_.header.frame_id,
          ros::Time(0), transform);
      pcl_ros::transformPointCloud("/local_origin", transform,
                                   cameras_[i].newest_cloud_msg_,
                                   pc2cloud_world);
      pcl::fromROSMsg(pc2cloud_world, complete_cloud);
      local_planner_->complete_cloud_.push_back(std::move(complete_cloud));
    } catch (tf::TransformException& ex) {
      ROS_ERROR("Received an exception trying to transform a pointcloud: %s",
                ex.what());
    }
  }

  // update position
  local_planner_->setPose(newest_pose_);

  // Update velocity
  local_planner_->setCurrentVelocity(vel_msg_);

  // update state
  local_planner_->currently_armed_ = armed_;
  local_planner_->offboard_ = offboard_;
  local_planner_->mission_ = mission_;

  // update goal
  if (new_goal_) {
    local_planner_->setGoal(goal_msg_.pose.position);
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
}

void LocalPlannerNode::positionCallback(const geometry_msgs::PoseStamped& msg) {
  last_pose_ = newest_pose_;
  newest_pose_ = msg;
  curr_yaw_ = tf::getYaw(msg.pose.orientation);
  position_received_ = true;

  // visualize drone in RVIZ
  visualization_msgs::Marker marker;
  if (!world_path_.empty()) {
    if (!visualizeDrone(msg, marker)) {
      drone_pub_.publish(marker);
    }
  }
}

void LocalPlannerNode::velocityCallback(
    const geometry_msgs::TwistStamped& msg) {
  vel_msg_ = msg;
}

void LocalPlannerNode::stateCallback(const mavros_msgs::State& msg) {
  armed_ = msg.armed;

  if (msg.mode == "AUTO.MISSION") {
    offboard_ = false;
    mission_ = true;
  } else if (msg.mode == "OFFBOARD") {
    offboard_ = true;
    mission_ = false;
  } else {
    offboard_ = false;
    mission_ = false;
  }
}

void LocalPlannerNode::publishPaths() {
  // publish actual path
  visualization_msgs::Marker path_actual_marker;
  path_actual_marker.header.frame_id = "local_origin";
  path_actual_marker.header.stamp = ros::Time::now();
  path_actual_marker.id = path_length_;
  path_actual_marker.type = visualization_msgs::Marker::LINE_STRIP;
  path_actual_marker.action = visualization_msgs::Marker::ADD;
  path_actual_marker.pose.orientation.w = 1.0;
  path_actual_marker.scale.x = 0.03;
  path_actual_marker.color.a = 1.0;
  path_actual_marker.color.r = 0.0;
  path_actual_marker.color.g = 1.0;
  path_actual_marker.color.b = 0.0;

  path_actual_marker.points.push_back(last_pose_.pose.position);
  path_actual_marker.points.push_back(newest_pose_.pose.position);
  path_actual_pub_.publish(path_actual_marker);

  // publish path set by calculated waypoints
  visualization_msgs::Marker path_waypoint_marker;
  path_waypoint_marker.header.frame_id = "local_origin";
  path_waypoint_marker.header.stamp = ros::Time::now();
  path_waypoint_marker.id = path_length_;
  path_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
  path_waypoint_marker.action = visualization_msgs::Marker::ADD;
  path_waypoint_marker.pose.orientation.w = 1.0;
  path_waypoint_marker.scale.x = 0.02;
  path_waypoint_marker.color.a = 1.0;
  path_waypoint_marker.color.r = 1.0;
  path_waypoint_marker.color.g = 0.0;
  path_waypoint_marker.color.b = 0.0;

  path_waypoint_marker.points.push_back(last_waypoint_position_);
  path_waypoint_marker.points.push_back(newest_waypoint_position_);
  path_waypoint_pub_.publish(path_waypoint_marker);

  path_length_++;
}

void LocalPlannerNode::initMarker(visualization_msgs::MarkerArray* marker,
                                  nav_msgs::GridCells& path, const float red,
                                  const float green, const float blue) {
  visualization_msgs::Marker m;
  m.header.frame_id = "local_origin";
  m.header.stamp = ros::Time::now();
  m.type = visualization_msgs::Marker::CUBE;
  m.action = 3;
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.color.a = 1.0;
  m.lifetime = ros::Duration();
  m.id = 0;
  marker->markers.push_back(m);

  geometry_msgs::PoseStamped drone_pos = local_planner_->getPosition();

  for (size_t i = 0; i < path.cells.size(); i++) {
    m.id = i + 1;
    m.action = visualization_msgs::Marker::ADD;
    PolarPoint p_pol(path.cells[i].x, path.cells[i].y, 1.0f);
    m.pose.position = toPoint(polarToCartesian(p_pol, drone_pos.pose.position));

    m.color.r = red;
    m.color.g = green;
    m.color.b = blue;

    marker->markers.push_back(m);
  }
}

void LocalPlannerNode::publishMarkerBlocked(nav_msgs::GridCells& path_blocked) {
  visualization_msgs::MarkerArray marker_blocked;
  initMarker(&marker_blocked, path_blocked, 0.0, 0.0, 1.0);
  marker_blocked_pub_.publish(marker_blocked);
}

void LocalPlannerNode::publishMarkerRejected(
    nav_msgs::GridCells& path_rejected) {
  visualization_msgs::MarkerArray marker_rejected;
  initMarker(&marker_rejected, path_rejected, 1.0, 0.0, 0.0);
  marker_rejected_pub_.publish(marker_rejected);
}

void LocalPlannerNode::publishMarkerCandidates(
    nav_msgs::GridCells& path_candidates) {
  visualization_msgs::MarkerArray marker_candidates;
  initMarker(&marker_candidates, path_candidates, 0.0, 1.0, 0.0);
  marker_candidates_pub_.publish(marker_candidates);
}

void LocalPlannerNode::publishMarkerSelected(
    nav_msgs::GridCells& path_selected) {
  visualization_msgs::MarkerArray marker_selected;
  initMarker(&marker_selected, path_selected, 0.8, 0.16, 0.8);
  marker_selected_pub_.publish(marker_selected);
}

void LocalPlannerNode::publishMarkerFOV(nav_msgs::GridCells& FOV_cells) {
  visualization_msgs::MarkerArray FOV_marker;
  initMarker(&FOV_marker, FOV_cells, 0.16, 0.8, 0.8);
  marker_FOV_pub_.publish(FOV_marker);
}

void LocalPlannerNode::publishGoal() {
  visualization_msgs::MarkerArray marker_goal;
  visualization_msgs::Marker m;

  geometry_msgs::Point goal = local_planner_->getGoal();

  m.header.frame_id = "local_origin";
  m.header.stamp = ros::Time::now();
  m.type = visualization_msgs::Marker::SPHERE;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 0.5;
  m.scale.y = 0.5;
  m.scale.z = 0.5;
  m.color.a = 1.0;
  m.color.r = 1.0;
  m.color.g = 1.0;
  m.color.b = 0.0;
  m.lifetime = ros::Duration();
  m.id = 0;
  m.pose.position = goal;
  marker_goal.markers.push_back(m);
  marker_goal_pub_.publish(marker_goal);
}

void LocalPlannerNode::publishReachHeight() {
  visualization_msgs::Marker m;
  m.header.frame_id = "local_origin";
  m.header.stamp = ros::Time::now();
  m.type = visualization_msgs::Marker::CUBE;
  m.pose.position.x = local_planner_->take_off_pose_.pose.position.x;
  m.pose.position.y = local_planner_->take_off_pose_.pose.position.y;
  m.pose.position.z = local_planner_->starting_height_;
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = 0.0;
  m.pose.orientation.w = 1.0;
  m.scale.x = 10;
  m.scale.y = 10;
  m.scale.z = 0.001;
  m.color.a = 0.5;
  m.color.r = 0.0;
  m.color.g = 0.0;
  m.color.b = 1.0;
  m.lifetime = ros::Duration(0.5);
  m.id = 0;

  initial_height_pub_.publish(m);

  visualization_msgs::Marker t;
  t.header.frame_id = "local_origin";
  t.header.stamp = ros::Time::now();
  t.type = visualization_msgs::Marker::SPHERE;
  t.action = visualization_msgs::Marker::ADD;
  t.scale.x = 0.2;
  t.scale.y = 0.2;
  t.scale.z = 0.2;
  t.color.a = 1.0;
  t.color.r = 1.0;
  t.color.g = 0.0;
  t.color.b = 0.0;
  t.lifetime = ros::Duration();
  t.id = 0;
  t.pose.position = local_planner_->take_off_pose_.pose.position;
  takeoff_pose_pub_.publish(t);

  visualization_msgs::Marker a;
  a.header.frame_id = "local_origin";
  a.header.stamp = ros::Time::now();
  a.type = visualization_msgs::Marker::SPHERE;
  a.action = visualization_msgs::Marker::ADD;
  a.scale.x = 0.2;
  a.scale.y = 0.2;
  a.scale.z = 0.2;
  a.color.a = 1.0;
  a.color.r = 0.5;
  a.color.g = 0.0;
  a.color.b = 0.5;
  a.lifetime = ros::Duration();
  a.id = 0;
  a.pose.position = local_planner_->offboard_pose_.pose.position;
  offboard_pose_pub_.publish(a);
}

void LocalPlannerNode::publishBox() {
  visualization_msgs::MarkerArray marker_array;
  geometry_msgs::PoseStamped drone_pos = local_planner_->getPosition();

  visualization_msgs::Marker box;
  box.header.frame_id = "local_origin";
  box.header.stamp = ros::Time::now();
  box.id = 0;
  box.type = visualization_msgs::Marker::SPHERE;
  box.action = visualization_msgs::Marker::ADD;
  box.pose.position.x = drone_pos.pose.position.x;
  box.pose.position.y = drone_pos.pose.position.y;
  box.pose.position.z = drone_pos.pose.position.z;
  box.pose.orientation.x = 0.0;
  box.pose.orientation.y = 0.0;
  box.pose.orientation.z = 0.0;
  box.pose.orientation.w = 1.0;
  box.scale.x = 2.0 * local_planner_->histogram_box_.radius_;
  box.scale.y = 2.0 * local_planner_->histogram_box_.radius_;
  box.scale.z = 2.0 * local_planner_->histogram_box_.radius_;
  box.color.a = 0.5;
  box.color.r = 0.0;
  box.color.g = 1.0;
  box.color.b = 0.0;
  marker_array.markers.push_back(box);

  visualization_msgs::Marker plane;
  plane.header.frame_id = "local_origin";
  plane.header.stamp = ros::Time::now();
  plane.id = 1;
  plane.type = visualization_msgs::Marker::CUBE;
  plane.action = visualization_msgs::Marker::ADD;
  plane.pose.position.x = drone_pos.pose.position.x;
  plane.pose.position.y = drone_pos.pose.position.y;
  plane.pose.position.z = local_planner_->histogram_box_.zmin_;
  plane.pose.orientation.x = 0.0;
  plane.pose.orientation.y = 0.0;
  plane.pose.orientation.z = 0.0;
  plane.pose.orientation.w = 1.0;
  plane.scale.x = 2.0 * local_planner_->histogram_box_.radius_;
  plane.scale.y = 2.0 * local_planner_->histogram_box_.radius_;
  plane.scale.z = 0.001;
  plane.color.a = 0.5;
  plane.color.r = 0.0;
  plane.color.g = 1.0;
  plane.color.b = 0.0;
  marker_array.markers.push_back(plane);

  bounding_box_pub_.publish(marker_array);
}

void LocalPlannerNode::publishWaypoints(bool hover) {
  const ros::Time now = ros::Time::now();

  wp_generator_->updateState(newest_pose_, goal_msg_, vel_msg_, hover, now);
  waypointResult result = wp_generator_->getWaypoints();

  visualization_msgs::Marker sphere1;
  visualization_msgs::Marker sphere2;
  visualization_msgs::Marker sphere3;

  sphere1.header.frame_id = "local_origin";
  sphere1.header.stamp = now;
  sphere1.id = 0;
  sphere1.type = visualization_msgs::Marker::SPHERE;
  sphere1.action = visualization_msgs::Marker::ADD;
  sphere1.pose.position.x = result.goto_position.x;
  sphere1.pose.position.y = result.goto_position.y;
  sphere1.pose.position.z = result.goto_position.z;
  sphere1.pose.orientation.x = 0.0;
  sphere1.pose.orientation.y = 0.0;
  sphere1.pose.orientation.z = 0.0;
  sphere1.pose.orientation.w = 1.0;
  sphere1.scale.x = 0.2;
  sphere1.scale.y = 0.2;
  sphere1.scale.z = 0.2;
  sphere1.color.a = 0.8;
  sphere1.color.r = 0.5;
  sphere1.color.g = 1.0;
  sphere1.color.b = 0.0;

  sphere2.header.frame_id = "local_origin";
  sphere2.header.stamp = now;
  sphere2.id = 0;
  sphere2.type = visualization_msgs::Marker::SPHERE;
  sphere2.action = visualization_msgs::Marker::ADD;
  sphere2.pose.position.x = result.adapted_goto_position.x;
  sphere2.pose.position.y = result.adapted_goto_position.y;
  sphere2.pose.position.z = result.adapted_goto_position.z;
  sphere2.pose.orientation.x = 0.0;
  sphere2.pose.orientation.y = 0.0;
  sphere2.pose.orientation.z = 0.0;
  sphere2.pose.orientation.w = 1.0;
  sphere2.scale.x = 0.2;
  sphere2.scale.y = 0.2;
  sphere2.scale.z = 0.2;
  sphere2.color.a = 0.8;
  sphere2.color.r = 1.0;
  sphere2.color.g = 1.0;
  sphere2.color.b = 0.0;

  sphere3.header.frame_id = "local_origin";
  sphere3.header.stamp = now;
  sphere3.id = 0;
  sphere3.type = visualization_msgs::Marker::SPHERE;
  sphere3.action = visualization_msgs::Marker::ADD;
  sphere3.pose.position.x = result.smoothed_goto_position.x;
  sphere3.pose.position.y = result.smoothed_goto_position.y;
  sphere3.pose.position.z = result.smoothed_goto_position.z;
  sphere3.pose.orientation.x = 0.0;
  sphere3.pose.orientation.y = 0.0;
  sphere3.pose.orientation.z = 0.0;
  sphere3.pose.orientation.w = 1.0;
  sphere3.scale.x = 0.2;
  sphere3.scale.y = 0.2;
  sphere3.scale.z = 0.2;
  sphere3.color.a = 0.8;
  sphere3.color.r = 1.0;
  sphere3.color.g = 0.5;
  sphere3.color.b = 0.0;

  original_wp_pub_.publish(sphere1);
  adapted_wp_pub_.publish(sphere2);
  smoothed_wp_pub_.publish(sphere3);

  last_waypoint_position_ = newest_waypoint_position_;
  newest_waypoint_position_ = result.smoothed_goto_position;
  publishPaths();
  publishSetpoint(result.velocity_waypoint, result.waypoint_type);

  // to mavros

  mavros_msgs::Trajectory obst_free_path = {};
  if (local_planner_->use_vel_setpoints_) {
    mavros_vel_setpoint_pub_.publish(result.velocity_waypoint);
    transformVelocityToTrajectory(obst_free_path, result.velocity_waypoint);
  } else {
    mavros_pos_setpoint_pub_.publish(result.position_waypoint);
    transformPoseToTrajectory(obst_free_path, result.position_waypoint);
  }
  mavros_obstacle_free_path_pub_.publish(obst_free_path);
}

void LocalPlannerNode::publishHistogramImage() {
  histogram_image_pub_.publish(local_planner_->histogram_image_);
}

void LocalPlannerNode::publishTree() {
  visualization_msgs::Marker tree_marker;
  tree_marker.header.frame_id = "local_origin";
  tree_marker.header.stamp = ros::Time::now();
  tree_marker.id = 0;
  tree_marker.type = visualization_msgs::Marker::LINE_LIST;
  tree_marker.action = visualization_msgs::Marker::ADD;
  tree_marker.pose.orientation.w = 1.0;
  tree_marker.scale.x = 0.05;
  tree_marker.color.a = 0.8;
  tree_marker.color.r = 0.4;
  tree_marker.color.g = 0.0;
  tree_marker.color.b = 0.6;

  visualization_msgs::Marker path_marker;
  path_marker.header.frame_id = "local_origin";
  path_marker.header.stamp = ros::Time::now();
  path_marker.id = 0;
  path_marker.type = visualization_msgs::Marker::LINE_LIST;
  path_marker.action = visualization_msgs::Marker::ADD;
  path_marker.pose.orientation.w = 1.0;
  path_marker.scale.x = 0.05;
  path_marker.color.a = 0.8;
  path_marker.color.r = 1.0;
  path_marker.color.g = 0.0;
  path_marker.color.b = 0.0;

  std::vector<TreeNode> tree;
  std::vector<int> closed_set;

  local_planner_->getTree(tree, closed_set, path_node_positions_);

  for (size_t i = 0; i < closed_set.size(); i++) {
    int node_nr = closed_set[i];
    geometry_msgs::Point p1 = toPoint(tree[node_nr].getPosition());
    int origin = tree[node_nr].origin_;
    geometry_msgs::Point p2 = toPoint(tree[origin].getPosition());
    tree_marker.points.push_back(p1);
    tree_marker.points.push_back(p2);
  }

  for (size_t i = 1; i < path_node_positions_.size(); i++) {
    path_marker.points.push_back(path_node_positions_[i - 1]);
    path_marker.points.push_back(path_node_positions_[i]);
  }

  complete_tree_pub_.publish(tree_marker);
  tree_path_pub_.publish(path_marker);
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
  goal_msg_.pose.position.z = local_planner_->getGoal().z;
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
  if (mission_ && (msg.point_valid[1] == true) &&
      ((std::fabs(goal_msg_.pose.position.x - msg.point_2.position.x) >
        0.001) ||
       (std::fabs(goal_msg_.pose.position.y - msg.point_2.position.y) >
        0.001) ||
       (std::fabs(goal_msg_.pose.position.z - msg.point_2.position.z) >
        0.001))) {
    new_goal_ = true;
    goal_msg_.pose.position.x = msg.point_2.position.x;
    goal_msg_.pose.position.y = msg.point_2.position.y;
    goal_msg_.pose.position.z = msg.point_2.position.z;
  }
}

void LocalPlannerNode::distanceSensorCallback(
    const mavros_msgs::Altitude& msg) {
  if (!std::isnan(msg.bottom_clearance)) {
    ground_distance_msg_ = msg;
    publishGround();
  }
}
void LocalPlannerNode::publishGround() {
  geometry_msgs::PoseStamped drone_pos = local_planner_->getPosition();
  visualization_msgs::Marker plane;
  plane.header.frame_id = "local_origin";
  plane.header.stamp = ros::Time::now();
  plane.id = 1;
  plane.type = visualization_msgs::Marker::CUBE;
  plane.action = visualization_msgs::Marker::ADD;
  plane.pose.position.x = drone_pos.pose.position.x;
  plane.pose.position.y = drone_pos.pose.position.y;
  plane.pose.position.z =
      drone_pos.pose.position.z - local_planner_->ground_distance_;
  plane.pose.orientation.x = 0.0;
  plane.pose.orientation.y = 0.0;
  plane.pose.orientation.z = 0.0;
  plane.pose.orientation.w = 1.0;
  plane.scale.x = 2.0 * local_planner_->histogram_box_.radius_;
  plane.scale.y = 2.0 * local_planner_->histogram_box_.radius_;
  plane.scale.z = 0.001;
  ;
  plane.color.a = 0.5;
  plane.color.r = 0.0;
  plane.color.g = 0.0;
  plane.color.b = 1.0;
  ground_measurement_pub_.publish(plane);
}

void LocalPlannerNode::printPointInfo(double x, double y, double z) {
  geometry_msgs::PoseStamped drone_pos = local_planner_->getPosition();
  int beta_z = floor(
      (atan2(x - drone_pos.pose.position.x, y - drone_pos.pose.position.y) *
       180.0 / M_PI));  //(-180. +180]
  int beta_e = floor((atan((z - drone_pos.pose.position.z) /
                           sqrt((x - drone_pos.pose.position.x) *
                                    (x - drone_pos.pose.position.x) +
                                (y - drone_pos.pose.position.y) *
                                    (y - drone_pos.pose.position.y))) *
                      180.0 / M_PI));  //(-90.+90)

  beta_z = beta_z + (ALPHA_RES - beta_z % ALPHA_RES);  //[-170,+190]
  beta_e = beta_e + (ALPHA_RES - beta_e % ALPHA_RES);  //[-80,+90]

  printf("----- Point: %f %f %f -----\n", x, y, z);
  printf("Elevation %d Azimuth %d \n", beta_e, beta_z);
  printf("-------------------------------------------- \n");
}

void LocalPlannerNode::pointCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg, int index) {
  cameras_[index].newest_cloud_msg_ = *msg;  // FIXME: avoid a copy
  cameras_[index].received_ = true;
}

void LocalPlannerNode::cameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& msg, int index) {
  // calculate the horizontal and vertical field of view from the image size and
  // focal length:
  // h_fov = 2 * atan (image_width / (2 * focal_length_x))
  // v_fov = 2 * atan (image_height / (2 * focal_length_y))
  // Assumption: if there are n cameras the total horizonal field of view is n
  // times the horizontal field of view of a single camera
  local_planner_->h_FOV_ = static_cast<double>(cameras_.size()) * 2.0 *
                           atan(msg->width / (2.0 * msg->K[0])) * 180.0 / M_PI;
  local_planner_->v_FOV_ =
      2.0 * atan(msg->height / (2.0 * msg->K[4])) * 180.0 / M_PI;
  wp_generator_->setFOV(local_planner_->h_FOV_, local_planner_->v_FOV_);
}

void LocalPlannerNode::publishSetpoint(const geometry_msgs::Twist& wp,
                                       waypoint_choice& waypoint_type) {
  visualization_msgs::Marker setpoint;
  setpoint.header.frame_id = "local_origin";
  setpoint.header.stamp = ros::Time::now();
  setpoint.id = 0;
  setpoint.type = visualization_msgs::Marker::ARROW;
  setpoint.action = visualization_msgs::Marker::ADD;

  geometry_msgs::Point tip;
  tip.x = newest_pose_.pose.position.x + wp.linear.x;
  tip.y = newest_pose_.pose.position.y + wp.linear.y;
  tip.z = newest_pose_.pose.position.z + wp.linear.z;
  setpoint.points.push_back(newest_pose_.pose.position);
  setpoint.points.push_back(tip);
  setpoint.scale.x = 0.1;
  setpoint.scale.y = 0.1;
  setpoint.scale.z = 0.1;
  setpoint.color.a = 1.0;

  switch (waypoint_type) {
    case hover: {
      setpoint.color.r = 1.0;
      setpoint.color.g = 1.0;
      setpoint.color.b = 0.0;
      break;
    }
    case costmap: {
      setpoint.color.r = 0.0;
      setpoint.color.g = 1.0;
      setpoint.color.b = 0.0;
      break;
    }
    case tryPath: {
      setpoint.color.r = 0.0;
      setpoint.color.g = 1.0;
      setpoint.color.b = 0.0;
      break;
    }
    case direct: {
      setpoint.color.r = 0.0;
      setpoint.color.g = 0.0;
      setpoint.color.b = 1.0;
      break;
    }
    case reachHeight: {
      setpoint.color.r = 1.0;
      setpoint.color.g = 0.0;
      setpoint.color.b = 1.0;
      break;
    }
    case goBack: {
      setpoint.color.r = 1.0;
      setpoint.color.g = 0.0;
      setpoint.color.b = 0.0;
      break;
    }
  }

  current_waypoint_pub_.publish(setpoint);
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

void LocalPlannerNode::publishPlannerData() {
  pcl::PointCloud<pcl::PointXYZ> final_cloud, reprojected_points;
  local_planner_->getCloudsForVisualization(final_cloud, reprojected_points);
  local_pointcloud_pub_.publish(final_cloud);
  reprojected_points_pub_.publish(reprojected_points);

  publishTree();

  last_wp_time_ = ros::Time::now();

  nav_msgs::GridCells path_candidates, path_selected, path_rejected,
      path_blocked, FOV_cells;
  local_planner_->getCandidateDataForVisualization(
      path_candidates, path_selected, path_rejected, path_blocked, FOV_cells);

  if (local_planner_->send_obstacles_fcu_) {
    sensor_msgs::LaserScan distance_data_to_fcu;
    local_planner_->sendObstacleDistanceDataToFcu(distance_data_to_fcu);
    mavros_obstacle_distance_pub_.publish(distance_data_to_fcu);
  }

  publishMarkerCandidates(path_candidates);
  publishMarkerSelected(path_selected);
  publishMarkerRejected(path_rejected);
  publishMarkerBlocked(path_blocked);
  publishMarkerFOV(FOV_cells);
  publishGoal();
  publishBox();
  publishReachHeight();
  publishHistogramImage();
}

void LocalPlannerNode::dynamicReconfigureCallback(
    avoidance::LocalPlannerNodeConfig& config, uint32_t level) {
  std::lock_guard<std::mutex> guard(running_mutex_);
  local_planner_->dynamicReconfigureSetParams(config, level);
  wp_generator_->setSmoothingSpeed(config.smoothing_speed_);
  rqt_param_config_ = config;
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
      std::clock_t start_time = std::clock();
      local_planner_->runPlanner();
      publishPlannerData();

      ROS_DEBUG("\033[0;35m[OA]Planner calculation time: %2.2f ms \n \033[0m",
                (std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
    }
  }
}
}
