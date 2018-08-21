#include "local_planner_node.h"

LocalPlannerNode::LocalPlannerNode() {
  nh_ = ros::NodeHandle("~");
  readParams();

  // Set up Dynamic Reconfigure Server
  dynamic_reconfigure::Server<avoidance::LocalPlannerNodeConfig>::CallbackType
      f;
  f = boost::bind(&LocalPlannerNode::dynamicReconfigureCallback, this, _1, _2);
  server_.setCallback(f);

  pointcloud_sub_ = nh_.subscribe<const sensor_msgs::PointCloud2&>(
      depth_points_topic_, 1, &LocalPlannerNode::pointCloudCallback, this);
  pose_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped&>(
      "/mavros/local_position/pose", 1, &LocalPlannerNode::positionCallback,
      this);
  velocity_sub_ = nh_.subscribe<const geometry_msgs::TwistStamped&>("/mavros/local_position/velocity", 1,
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

  local_pointcloud_pub_ =
      nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/local_pointcloud", 1);
  ground_pointcloud_pub_ =
      nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/ground_pointcloud", 1);
  reprojected_points_pub_ =
      nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/reprojected_points", 1);
  bounding_box_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/bounding_box", 1);
  groundbox_pub_ = nh_.advertise<visualization_msgs::Marker>("/ground_box", 1);
  avoid_sphere_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/avoid_sphere", 1);
  original_wp_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/original_waypoint", 1);
  adapted_wp_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/adapted_waypoint", 1);
  smoothed_wp_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/smoothed_waypoint", 1);
  ground_est_pub_ = nh_.advertise<visualization_msgs::Marker>("/ground_est", 1);
  height_map_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/height_map", 1);
  complete_tree_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/complete_tree", 1);
  tree_path_pub_ = nh_.advertise<visualization_msgs::Marker>("/tree_path", 1);
  marker_blocked_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/blocked_marker", 1);
  marker_rejected_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/rejected_marker", 1);
  marker_candidates_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/candidates_marker", 1);
  marker_ground_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/ground_marker", 1);
  marker_FOV_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/FOV_marker", 1);
  marker_selected_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/selected_marker", 1);
  marker_goal_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/goal_position", 1);
  waypoint_pub_ = nh_.advertise<nav_msgs::Path>("/waypoint", 1);
  path_pub_ = nh_.advertise<nav_msgs::Path>("/path_actual", 1);
  bounding_box_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/bounding_box", 1);
  mavros_vel_setpoint_pub_ = nh_.advertise<geometry_msgs::Twist>(
      "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
  mavros_pos_setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
  "/mavros/setpoint_position/local", 10);
  mavros_obstacle_free_path_pub_ = nh_.advertise<mavros_msgs::Trajectory>(
      "/mavros/trajectory/generated", 10);
  mavros_obstacle_distance_pub_ =
      nh_.advertise<sensor_msgs::LaserScan>("/mavros/obstacle/send", 10);
  current_waypoint_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/current_setpoint", 1);
  log_name_pub_ = nh_.advertise<std_msgs::String>("/log_name", 1);
  takeoff_pose_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/take_off_pose", 1);
  offboard_pose_pub_ =
        nh_.advertise<visualization_msgs::Marker>("/offboard_pose", 1);
  initial_height_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/initial_height", 1);

  mavros_set_mode_client_ =
      nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

  local_planner_.setGoal();
}

LocalPlannerNode::~LocalPlannerNode() {}

void LocalPlannerNode::readParams() {
  nh_.param<double>("goal_x_param", local_planner_.goal_x_param_, 9.0);
  nh_.param<double>("goal_y_param", local_planner_.goal_y_param_, 13.0);
  nh_.param<double>("goal_z_param", local_planner_.goal_z_param_, 3.5);
  nh_.param<std::string>("depth_points_topic", depth_points_topic_,
                         "/camera/depth/points");
  goal_msg_.pose.position.x = local_planner_.goal_x_param_;
  goal_msg_.pose.position.y = local_planner_.goal_y_param_;
  goal_msg_.pose.position.z = local_planner_.goal_z_param_;
}

bool LocalPlannerNode::canUpdatePlannerInfo() {
  // Check if we have a transformation available at the time of the current point cloud
  return tf_listener_.canTransform("/local_origin", newest_point_cloud_.header.frame_id,
                               newest_point_cloud_.header.stamp);
}
void LocalPlannerNode::updatePlannerInfo() {

  // update the point cloud
  sensor_msgs::PointCloud2 pc2cloud_world;
  try {
    tf::StampedTransform transform;
    tf_listener_.lookupTransform("/local_origin", newest_point_cloud_.header.frame_id,
                                 newest_point_cloud_.header.stamp, transform);
    pcl_ros::transformPointCloud("/local_origin", transform, newest_point_cloud_, pc2cloud_world);
    pcl::fromROSMsg(pc2cloud_world, local_planner_.complete_cloud_);
  } catch (tf::TransformException &ex) {
    ROS_ERROR(
        "Received an exception trying to transform a point from "
        "\"front_camera_optical_frame\" to \"local_origin\": %s",
        ex.what());
  }

  // update position
  local_planner_.setPose(newest_pose_);

  // Update velocity
  local_planner_.setCurrentVelocity(vel_msg_);

  // update state
  local_planner_.currently_armed_ = armed_;
  local_planner_.offboard_ = offboard_;
  local_planner_.mission_ = mission_;

  // update goal
  if (new_goal_) {
    local_planner_.goal_x_param_ = goal_msg_.pose.position.x;
    local_planner_.goal_y_param_ = goal_msg_.pose.position.y;
    local_planner_.goal_z_param_ = goal_msg_.pose.position.z;
    local_planner_.setGoal();
    new_goal_ = false;
  }else{
	goal_msg_.pose.position.z = local_planner_.goal_z_param_;
  }
}

void LocalPlannerNode::positionCallback(const geometry_msgs::PoseStamped& msg) {
  newest_pose_ = msg;
  publishPath(newest_pose_);
  curr_yaw_ = tf::getYaw(msg.pose.orientation);
  position_received_ = true;
}

void LocalPlannerNode::velocityCallback(const geometry_msgs::TwistStamped& msg) {
  vel_msg_ = msg;
}

void LocalPlannerNode::stateCallback(const mavros_msgs::State& msg) {
  armed_ = msg.armed;

  if (msg.mode == "AUTO.MISSION") {
    offboard_ = false;
    mission_ = true;
  }

  if (msg.mode == "OFFBOARD") {
    offboard_ = true;
    mission_ = false;
  }
}

void LocalPlannerNode::publishPath(const geometry_msgs::PoseStamped& msg) {
  path_actual_.header.stamp = msg.header.stamp;
  path_actual_.header.frame_id = msg.header.frame_id;
  path_actual_.poses.push_back(msg);
}

void LocalPlannerNode::initMarker(visualization_msgs::MarkerArray *marker,
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

  geometry_msgs::PoseStamped drone_pos;
  local_planner_.getPosition(drone_pos);

  for (int i = 0; i < path.cells.size(); i++) {
    m.id = i + 1;
    m.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point p =
        fromPolarToCartesian((int)path.cells[i].x, (int)path.cells[i].y, 1.0,
                             drone_pos.pose.position);
    m.pose.position = p;

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

void LocalPlannerNode::publishMarkerGround(nav_msgs::GridCells& path_ground) {
  visualization_msgs::MarkerArray marker_ground;
  initMarker(&marker_ground, path_ground, 0.16, 0.8, 0.8);
  marker_ground_pub_.publish(marker_ground);
}

void LocalPlannerNode::publishMarkerFOV(nav_msgs::GridCells& FOV_cells) {
  visualization_msgs::MarkerArray FOV_marker;
  initMarker(&FOV_marker, FOV_cells, 0.16, 0.8, 0.8);
  marker_FOV_pub_.publish(FOV_marker);
}

void LocalPlannerNode::publishGoal() {
  visualization_msgs::MarkerArray marker_goal;
  visualization_msgs::Marker m;

  geometry_msgs::Point goal;
  local_planner_.getGoalPosition(goal);

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

void LocalPlannerNode::publishGround() {
  visualization_msgs::Marker m;
  geometry_msgs::Point closest_point_on_ground;
  geometry_msgs::Quaternion ground_orientation;
  std::vector<double> ground_heights;
  std::vector<double> ground_xmax;
  std::vector<double> ground_xmin;
  std::vector<double> ground_ymax;
  std::vector<double> ground_ymin;
  local_planner_.ground_detector_.getGroundDataForVisualization(
      closest_point_on_ground, ground_orientation, ground_heights, ground_xmax,
      ground_xmin, ground_ymax, ground_ymin);
  m.header.frame_id = "local_origin";
  m.header.stamp = ros::Time::now();
  m.type = visualization_msgs::Marker::CUBE;
  m.pose.position.x = closest_point_on_ground.x;
  m.pose.position.y = closest_point_on_ground.y;
  m.pose.position.z = closest_point_on_ground.z;
  m.pose.orientation = ground_orientation;
  m.scale.x = 10;
  m.scale.y = 10;
  m.scale.z = 0.001;
  m.color.a = 0.5;
  m.color.r = 0.0;
  m.color.g = 0.0;
  m.color.b = 1.0;
  m.lifetime = ros::Duration(0.5);
  m.id = 0;

  ground_est_pub_.publish(m);

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker g;
  g.header.frame_id = "local_origin";
  g.header.stamp = ros::Time::now();
  g.type = visualization_msgs::Marker::CUBE;
  g.action = 3;
  g.scale.x = 0.1;
  g.scale.y = 0.1;
  g.scale.z = 0.5;
  g.color.a = 0.5;
  g.lifetime = ros::Duration();
  g.id = 0;
  marker_array.markers.push_back(g);

  for (int i = 0; i < ground_heights.size(); i++) {
    g.id = i + 1;
    g.action = visualization_msgs::Marker::ADD;
    g.scale.x = std::abs(ground_xmax[i] - ground_xmin[i]);
    g.scale.y = std::abs(ground_ymax[i] - ground_ymin[i]);

    g.pose.position.x =
        ground_xmax[i] - 0.5 * std::abs(ground_xmax[i] - ground_xmin[i]);
    g.pose.position.y =
        ground_ymax[i] - 0.5 * std::abs(ground_ymax[i] - ground_ymin[i]);
    g.pose.position.z = ground_heights[i] - 0.5 * g.scale.z;

    g.color.r = 1;
    g.color.g = 0;
    g.color.b = 0;

    marker_array.markers.push_back(g);
  }
  height_map_pub_.publish(marker_array);
}

void LocalPlannerNode::publishReachHeight() {
  visualization_msgs::Marker m;
  m.header.frame_id = "local_origin";
  m.header.stamp = ros::Time::now();
  m.type = visualization_msgs::Marker::CUBE;
  m.pose.position.x = local_planner_.take_off_pose_.pose.position.x;
  m.pose.position.y = local_planner_.take_off_pose_.pose.position.y;
  m.pose.position.z = local_planner_.starting_height_;
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
  t.pose.position = local_planner_.take_off_pose_.pose.position;
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
  a.pose.position = local_planner_.offboard_pose_.pose.position;
  offboard_pose_pub_.publish(a);
}

void LocalPlannerNode::publishBox() {
  geometry_msgs::PoseStamped drone_pos;
  local_planner_.getPosition(drone_pos);
  visualization_msgs::Marker box;
  box.header.frame_id = "local_origin";
  box.header.stamp = ros::Time::now();
  box.id = 0;
  box.type = visualization_msgs::Marker::CUBE;
  box.action = visualization_msgs::Marker::ADD;
  box.pose.position.x = drone_pos.pose.position.x +
                        0.5 * (local_planner_.histogram_box_size_.xmax_ -
                               local_planner_.histogram_box_size_.xmin_);
  box.pose.position.y = drone_pos.pose.position.y +
                        0.5 * (local_planner_.histogram_box_size_.ymax_ -
                               local_planner_.histogram_box_size_.ymin_);
  box.pose.position.z = drone_pos.pose.position.z +
                        0.5 * (local_planner_.histogram_box_size_.zmax_ -
                               local_planner_.histogram_box_size_.zmin_);
  box.pose.orientation.x = 0.0;
  box.pose.orientation.y = 0.0;
  box.pose.orientation.z = 0.0;
  box.pose.orientation.w = 1.0;
  box.scale.x = local_planner_.histogram_box_size_.xmax_ +
                local_planner_.histogram_box_size_.xmin_;
  box.scale.y = local_planner_.histogram_box_size_.ymax_ +
                local_planner_.histogram_box_size_.ymin_;
  box.scale.z = local_planner_.histogram_box_size_.zmax_ +
                local_planner_.histogram_box_size_.zmin_;
  box.color.a = 0.5;
  box.color.r = 0.0;
  box.color.g = 1.0;
  box.color.b = 0.0;
  bounding_box_pub_.publish(box);

  visualization_msgs::Marker groundbox;
  groundbox.header.frame_id = "local_origin";
  groundbox.header.stamp = ros::Time::now();
  groundbox.id = 0;
  groundbox.type = visualization_msgs::Marker::CUBE;
  groundbox.action = visualization_msgs::Marker::ADD;
  groundbox.pose.position.x =
      drone_pos.pose.position.x +
      0.5 * (local_planner_.ground_detector_.ground_box_size_.xmax_ -
             local_planner_.ground_detector_.ground_box_size_.xmin_);
  groundbox.pose.position.y =
      drone_pos.pose.position.y +
      0.5 * (local_planner_.ground_detector_.ground_box_size_.ymax_ -
             local_planner_.ground_detector_.ground_box_size_.ymin_);
  groundbox.pose.position.z =
      drone_pos.pose.position.z -
      local_planner_.ground_detector_.ground_box_size_.zmin_;
  groundbox.pose.orientation.x = 0.0;
  groundbox.pose.orientation.y = 0.0;
  groundbox.pose.orientation.z = 0.0;
  groundbox.pose.orientation.w = 1.0;
  groundbox.scale.x = local_planner_.ground_detector_.ground_box_size_.xmax_ +
                      local_planner_.ground_detector_.ground_box_size_.xmin_;
  groundbox.scale.y = local_planner_.ground_detector_.ground_box_size_.ymax_ +
                      local_planner_.ground_detector_.ground_box_size_.ymin_;
  groundbox.scale.z =
      2 * local_planner_.ground_detector_.ground_box_size_.zmin_;
  groundbox.color.a = 0.5;
  groundbox.color.r = 1.0;
  groundbox.color.g = 0.0;
  groundbox.color.b = 0.0;
  groundbox_pub_.publish(groundbox);
}

void LocalPlannerNode::publishAvoidSphere() {
  int age;
  local_planner_.getAvoidSphere(avoid_centerpoint_, avoid_radius_, age,
                                use_sphere_);
  visualization_msgs::Marker sphere;
  sphere.header.frame_id = "local_origin";
  sphere.header.stamp = ros::Time::now();
  sphere.id = 0;
  sphere.type = visualization_msgs::Marker::SPHERE;
  sphere.action = visualization_msgs::Marker::ADD;
  sphere.pose.position.x = avoid_centerpoint_.x;
  sphere.pose.position.y = avoid_centerpoint_.y;
  sphere.pose.position.z = avoid_centerpoint_.z;
  sphere.pose.orientation.x = 0.0;
  sphere.pose.orientation.y = 0.0;
  sphere.pose.orientation.z = 0.0;
  sphere.pose.orientation.w = 1.0;
  sphere.scale.x = 2 * avoid_radius_;
  sphere.scale.y = 2 * avoid_radius_;
  sphere.scale.z = 2 * avoid_radius_;
  sphere.color.a = 0.5;
  sphere.color.r = 0.0;
  sphere.color.g = 1.0;
  sphere.color.b = 1.0;
  if (use_sphere_ && age < 100) {
    avoid_sphere_pub_.publish(sphere);
  } else {
    sphere.color.a = 0;
    avoid_sphere_pub_.publish(sphere);
  }
}

void LocalPlannerNode::publishWaypoints(bool hover) {
  waypointResult result;
  const ros::Time now = ros::Time::now();

  wp_generator_.updateState(newest_pose_, goal_msg_, vel_msg_, hover, now);
  wp_generator_.getWaypoints(result);

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
  waypoint_pub_.publish(result.path);
  path_pub_.publish(path_actual_);
  publishSetpoint(result.velocity_waypoint, result.waypoint_type);

  // to mavros

  mavros_msgs::Trajectory obst_free_path = {};
  if(local_planner_.use_vel_setpoints_){
      mavros_vel_setpoint_pub_.publish(result.velocity_waypoint);
      transformVelocityToTrajectory(obst_free_path, result.velocity_waypoint);
  }else{
	  mavros_pos_setpoint_pub_.publish(result.position_waypoint);
	  transformPoseToTrajectory(obst_free_path, result.position_waypoint);
  }
  mavros_obstacle_free_path_pub_.publish(obst_free_path);
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

  local_planner_.getTree(tree, closed_set, path_node_positions_);

  for (int i = 0; i < closed_set.size(); i++) {
    int node_nr = closed_set[i];
    geometry_msgs::Point p1 = tree[node_nr].getPosition();
    int origin = tree[node_nr].origin_;
    geometry_msgs::Point p2 = tree[origin].getPosition();
    tree_marker.points.push_back(p1);
    tree_marker.points.push_back(p2);
  }

  if (path_node_positions_.size() > 0) {
    for (int i = 0; i < (path_node_positions_.size() - 1); i++) {
      path_marker.points.push_back(path_node_positions_[i]);
      path_marker.points.push_back(path_node_positions_[i + 1]);
    }
  }

  complete_tree_pub_.publish(tree_marker);
  tree_path_pub_.publish(path_marker);
}

void LocalPlannerNode::clickedPointCallback(
    const geometry_msgs::PointStamped &msg) {
  printPointInfo(msg.point.x, msg.point.y, msg.point.z);
}

void LocalPlannerNode::clickedGoalCallback(
    const geometry_msgs::PoseStamped &msg) {
  new_goal_ = true;
  goal_msg_ = msg;
  /* Selecting the goal from Rviz sets x and y. Get the z coordinate set in
   * the launch file */
  goal_msg_.pose.position.z = local_planner_.goal_z_param_;
}

void LocalPlannerNode::fcuInputGoalCallback(
    const mavros_msgs::Trajectory &msg) {
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

void LocalPlannerNode::printPointInfo(double x, double y, double z) {
  geometry_msgs::PoseStamped drone_pos;
  local_planner_.getPosition(drone_pos);
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

void LocalPlannerNode::pointCloudCallback(const sensor_msgs::PointCloud2& msg) {
  newest_point_cloud_ = msg; // FIXME: avoid a copy
}

void LocalPlannerNode::publishSetpoint(const geometry_msgs::Twist& wp,
                                       waypoint_choice& waypoint_type) {
  visualization_msgs::Marker setpoint;
  setpoint.header.frame_id = "local_origin";
  setpoint.header.stamp = ros::Time::now();
  setpoint.id = 0;
  setpoint.type = visualization_msgs::Marker::ARROW;
  setpoint.action = visualization_msgs::Marker::ADD;
  double length =
      std::sqrt(wp.linear.x * wp.linear.x + wp.linear.y * wp.linear.y +
                wp.linear.z * wp.linear.z);
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
    mavros_msgs::PositionTarget &point) {
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
    mavros_msgs::Trajectory &obst_avoid, geometry_msgs::PoseStamped pose) {
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
    mavros_msgs::Trajectory &obst_avoid, geometry_msgs::Twist vel) {
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
  pcl::PointCloud<pcl::PointXYZ> final_cloud, ground_cloud, reprojected_points;
  local_planner_.getCloudsForVisualization(final_cloud, ground_cloud,
                                           reprojected_points);
  local_pointcloud_pub_.publish(final_cloud);
  ground_pointcloud_pub_.publish(ground_cloud);
  reprojected_points_pub_.publish(reprojected_points);

  publishTree();

  ros::Duration time_diff = ros::Time::now() - last_wp_time_;
  last_wp_time_ = ros::Time::now();
  ros::Duration max_diff =
      ros::Duration(1.2 * pointcloud_timeout_hover_.toSec());

  nav_msgs::GridCells path_candidates, path_selected, path_rejected,
      path_blocked, path_ground, FOV_cells;
  local_planner_.getCandidateDataForVisualization(
      path_candidates, path_selected, path_rejected, path_blocked, FOV_cells,
      path_ground);

  if (local_planner_.send_obstacles_fcu_) {
    sensor_msgs::LaserScan distance_data_to_fcu;
    local_planner_.sendObstacleDistanceDataToFcu(distance_data_to_fcu);
    mavros_obstacle_distance_pub_.publish(distance_data_to_fcu);
  }

  publishMarkerCandidates(path_candidates);
  publishMarkerSelected(path_selected);
  publishMarkerRejected(path_rejected);
  publishMarkerBlocked(path_blocked);
  publishMarkerGround(path_ground);
  publishMarkerFOV(FOV_cells);
  publishGoal();
  publishBox();
  publishAvoidSphere();
  publishReachHeight();
  publishGround();
}

void LocalPlannerNode::dynamicReconfigureCallback(
    avoidance::LocalPlannerNodeConfig &config, uint32_t level) {
  std::lock_guard<std::mutex> guard(running_mutex_);
  local_planner_.dynamicReconfigureSetParams(config, level);
  wp_generator_.setMinJerkLimit(config.min_jerk_limit_);
  wp_generator_.setMaxJerkLimit(config.max_jerk_limit_);
}

void LocalPlannerNode::threadFunction() {
  while (!should_exit_) {
    // wait for data
    {
      std::unique_lock<std::mutex> lk(data_ready_mutex_);
      data_ready_cv_.wait(lk, [this]{return data_ready_ && !should_exit_;});
      data_ready_ = false;
    }

    if (should_exit_) break;

    {
      std::lock_guard<std::mutex> guard(running_mutex_);
      never_run_ = false;
      std::clock_t start_time = std::clock();
      local_planner_.runPlanner();
      publishPlannerData();

      ROS_DEBUG("\033[0;35m[OA]Planner calculation time: %2.2f ms \n \033[0m",
                (std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
    }

    // publish log name
    std_msgs::String msg;
    msg.data = local_planner_.log_name_;
    log_name_pub_.publish(msg);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "local_planner_node");
  LocalPlannerNode Node;
  ros::Duration(2).sleep();
  ros::Time start_time = ros::Time::now();
  bool hover = false;
  avoidanceOutput planner_output;

  std::thread worker(&LocalPlannerNode::threadFunction, &Node);

  // spin node, execute callbacks
  while (ros::ok()) {
    hover = false;

    // Process callbacks & wait for a position update
    while (!Node.position_received_ && ros::ok()) {
      ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    }

    // Check if all information was received
    ros::Time now = ros::Time::now();
    ros::Duration pointcloud_timeout_land =
    ros::Duration(Node.local_planner_.pointcloud_timeout_land_);
    ros::Duration since_last_cloud = now - Node.last_wp_time_;
    ros::Duration since_start = now - start_time;

    if (since_last_cloud > pointcloud_timeout_land &&
        since_start > pointcloud_timeout_land) {
      mavros_msgs::SetMode mode_msg;
      mode_msg.request.custom_mode = "AUTO.LAND";
      if (Node.mavros_set_mode_client_.call(mode_msg) &&
          mode_msg.response.mode_sent) {
        ROS_WARN("\033[1;33m Pointcloud timeout: Landing \n \033[0m");
      } else {
        ROS_ERROR("\033[1;33m Pointcloud timeout: Landing failed! \n \033[0m");
      }
    } else {
      if (Node.never_run_) {
        if (Node.position_received_) {
          hover = true;
          ROS_INFO(
              "\033[1;33m Pointcloud timeout: Hovering at current position \n "
              "\033[0m");
        } else {
          ROS_WARN(
              "\033[1;33m Pointcloud timeout: No position received, no WP to "
              "output.... \n \033[0m");
        }
      }
    }

    // If planner is not running, update planner info and get last results
    if (Node.canUpdatePlannerInfo()) {
      if (Node.running_mutex_.try_lock()) {
        Node.updatePlannerInfo();
        Node.local_planner_.getAvoidanceOutput(planner_output);
        Node.wp_generator_.setPlannerInfo(planner_output);
        if(Node.local_planner_.stop_in_front_active_){
           Node.local_planner_.getGoalPosition(Node.goal_msg_.pose.position);
        }
        Node.running_mutex_.unlock();
        // Wake up the planner
        std::unique_lock<std::mutex> lck(Node.data_ready_mutex_);
        Node.data_ready_ = true;
        Node.data_ready_cv_.notify_one();
      }
    }

    // send waypoint
    if (!Node.never_run_) {
      Node.publishWaypoints(hover);
    }

    Node.position_received_ = false;
  }

  Node.should_exit_ = true;
  Node.data_ready_cv_.notify_all();
  worker.join();
  return 0;
}
