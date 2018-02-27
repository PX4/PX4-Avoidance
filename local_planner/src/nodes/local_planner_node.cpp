#include "local_planner_node.h"

LocalPlannerNode::LocalPlannerNode() {

  //global queue node handle
  nh_ = ros::NodeHandle("~");

  readParams();

  // Set up Dynamic Reconfigure Server
  dynamic_reconfigure::Server<avoidance::LocalPlannerNodeConfig>::CallbackType f;
  f = boost::bind(&LocalPlannerNode::dynamicReconfigureCallback, this, _1, _2);
  server_.setCallback(f);

  pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(depth_points_topic_, 1, &LocalPlannerNode::pointCloudCallback, this);
  pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &LocalPlannerNode::positionCallback, this);
  velocity_sub_ = nh_.subscribe("/mavros/local_position/velocity", 1, &LocalPlannerNode::velocityCallback, this);
  state_sub_ = nh_.subscribe("/mavros/state", 1, &LocalPlannerNode::stateCallback, this);
  clicked_point_sub_ = nh_.subscribe("/clicked_point", 1, &LocalPlannerNode::clickedPointCallback, this);
  clicked_goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &LocalPlannerNode::clickedGoalCallback, this);

  local_pointcloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/local_pointcloud", 1);
  ground_pointcloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/ground_pointcloud", 1);
  reprojected_points_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/reprojected_points", 1);
  bounding_box_pub_ = nh_.advertise<visualization_msgs::Marker>("/bounding_box", 1);
  groundbox_pub_ = nh_.advertise<visualization_msgs::Marker>("/ground_box", 1);
  avoid_sphere_pub_ = nh_.advertise<visualization_msgs::Marker>("/avoid_sphere", 1);
  original_wp_pub_ = nh_.advertise<visualization_msgs::Marker>("/original_waypoint", 1);
  adapted_wp_pub_ = nh_.advertise<visualization_msgs::Marker>("/adapted_waypoint", 1);
  smoothed_wp_pub_ = nh_.advertise<visualization_msgs::Marker>("/smoothed_waypoint", 1);
  ground_est_pub_ = nh_.advertise<visualization_msgs::Marker>("/ground_est", 1);
  height_map_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/height_map", 1);
  complete_tree_pub_ = nh_.advertise<visualization_msgs::Marker>("/complete_tree", 1);
  tree_path_pub_ = nh_.advertise<visualization_msgs::Marker>("/tree_path", 1);
  marker_blocked_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/blocked_marker", 1);
  marker_rejected_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/rejected_marker", 1);
  marker_candidates_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/candidates_marker", 1);
  marker_ground_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/ground_marker", 1);
  marker_FOV_pub_= nh_.advertise<visualization_msgs::MarkerArray>("/FOV_marker", 1);
  marker_selected_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/selected_marker", 1);
  marker_goal_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/goal_position", 1);
  waypoint_pub_ = nh_.advertise<nav_msgs::Path>("/waypoint", 1);
  path_pub_ = nh_.advertise<nav_msgs::Path>("/path_actual", 1);
  bounding_box_pub_ = nh_.advertise<visualization_msgs::Marker>("/bounding_box", 1);
  mavros_waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  current_waypoint_pub_ = nh_.advertise<visualization_msgs::Marker>("/current_setpoint", 1);
  log_name_pub_ = nh_.advertise<std_msgs::String>("/log_name", 1);
  takeoff_pose_pub_ = nh_.advertise<visualization_msgs::Marker>("/take_off_pose", 1);
  initial_height_pub_ = nh_.advertise<visualization_msgs::Marker>("/initial_height", 1);

  mavros_set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

  local_planner_.setGoal();
}

LocalPlannerNode::~LocalPlannerNode() {}

void LocalPlannerNode::readParams() {
  nh_.param<double>("goal_x_param", local_planner_.goal_x_param_, 9);
  nh_.param<double>("goal_y_param", local_planner_.goal_y_param_, 13);
  nh_.param<double>("goal_z_param", local_planner_.goal_z_param_, 3.5);
  nh_.param<std::string>("depth_points_topic", depth_points_topic_, "/camera/depth/points");
}

void LocalPlannerNode::positionCallback(const geometry_msgs::PoseStamped msg) {
  auto rot_msg = msg;
  tf_listener_.transformPose("world", ros::Time(0), msg, "local_origin", rot_msg);
  local_planner_.setPose(rot_msg);
  publishPath(rot_msg);
  hover_current_pose_ = msg;
  position_received_ = true;
}

void LocalPlannerNode::velocityCallback(const geometry_msgs::TwistStamped msg) {
  auto transformed_msg = avoidance::transformTwistMsg(tf_listener_, "/world", "/local_origin", msg); // 90 deg fix
  local_planner_.setCurrentVelocity(transformed_msg);
}

void LocalPlannerNode::stateCallback(const mavros_msgs::State msg) {
  local_planner_.currently_armed_ = msg.armed;
  if(msg.mode == "OFFBOARD"){
    local_planner_.offboard_ = true;
  }else{
    local_planner_.offboard_ = false;
  }
}

void LocalPlannerNode::publishPath(const geometry_msgs::PoseStamped msg) {
  path_actual.header.stamp = msg.header.stamp;
  path_actual.header.frame_id = msg.header.frame_id;
  path_actual.poses.push_back(msg);
}

void LocalPlannerNode::initMarker(visualization_msgs::MarkerArray *marker, nav_msgs::GridCells path, float red, float green, float blue) {
  visualization_msgs::Marker m;
  m.header.frame_id = "world";
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

  for (int i=0; i < path.cells.size(); i++) {
    m.id = i+1;
    m.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point p = fromPolarToCartesian((int)path.cells[i].x, (int)path.cells[i].y, 1.0, drone_pos.pose.position);
    m.pose.position = p;

    m.color.r = red;
    m.color.g = green;
    m.color.b = blue;

    marker->markers.push_back(m);
  }
}

void LocalPlannerNode::publishMarkerBlocked(nav_msgs::GridCells path_blocked) {
  visualization_msgs::MarkerArray marker_blocked;
  initMarker(&marker_blocked, path_blocked, 0.0, 0.0, 1.0);
  marker_blocked_pub_.publish(marker_blocked);
}

void LocalPlannerNode::publishMarkerRejected(nav_msgs::GridCells path_rejected) {
  visualization_msgs::MarkerArray marker_rejected;
  initMarker(&marker_rejected, path_rejected, 1.0, 0.0, 0.0);
  marker_rejected_pub_.publish(marker_rejected);
}

void LocalPlannerNode::publishMarkerCandidates(nav_msgs::GridCells path_candidates) {
  visualization_msgs::MarkerArray marker_candidates;
  initMarker(&marker_candidates, path_candidates, 0.0, 1.0, 0.0);
  marker_candidates_pub_.publish(marker_candidates);
}

void LocalPlannerNode::publishMarkerSelected(nav_msgs::GridCells path_selected) {
  visualization_msgs::MarkerArray marker_selected;
  initMarker(&marker_selected, path_selected, 0.8, 0.16, 0.8);
  marker_selected_pub_.publish(marker_selected);
}

void LocalPlannerNode::publishMarkerGround(nav_msgs::GridCells path_ground) {
  visualization_msgs::MarkerArray marker_ground;
  initMarker(&marker_ground, path_ground, 0.16, 0.8, 0.8);
  marker_ground_pub_.publish(marker_ground);
}

void LocalPlannerNode::publishMarkerFOV(nav_msgs::GridCells FOV_cells) {
  visualization_msgs::MarkerArray FOV_marker;
  initMarker(&FOV_marker, FOV_cells, 0.16, 0.8, 0.8);
  marker_FOV_pub_.publish(FOV_marker);
}

void LocalPlannerNode::publishGoal() {
  visualization_msgs::MarkerArray marker_goal;
  visualization_msgs::Marker m;

  geometry_msgs::Point  goal;
  local_planner_.getGoalPosition(goal);

  m.header.frame_id = "world";
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
  local_planner_.ground_detector_.getGroundDataForVisualization(closest_point_on_ground, ground_orientation, ground_heights, ground_xmax, ground_xmin, ground_ymax, ground_ymin);
  m.header.frame_id = "world";
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
  g.header.frame_id = "world";
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

  for (int i=0; i < ground_heights.size(); i++) {
      g.id = i+1;
      g.action = visualization_msgs::Marker::ADD;
      g.scale.x = std::abs(ground_xmax[i] - ground_xmin[i]);
      g.scale.y = std::abs(ground_ymax[i] - ground_ymin[i]);

      g.pose.position.x = ground_xmax[i] - 0.5 * std::abs(ground_xmax[i] - ground_xmin[i]);
      g.pose.position.y = ground_ymax[i] - 0.5 * std::abs(ground_ymax[i] - ground_ymin[i]);
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
  m.header.frame_id = "world";
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
  t.header.frame_id = "world";
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
  box.pose.position.x = drone_pos.pose.position.x + 0.5 * (local_planner_.histogram_box_size_.xmax - local_planner_.histogram_box_size_.xmin);
  box.pose.position.y = drone_pos.pose.position.y + 0.5 * (local_planner_.histogram_box_size_.ymax - local_planner_.histogram_box_size_.ymin);
  box.pose.position.z = drone_pos.pose.position.z + 0.5 * (local_planner_.histogram_box_size_.zmax - local_planner_.histogram_box_size_.zmin);
  box.pose.orientation.x = 0.0;
  box.pose.orientation.y = 0.0;
  box.pose.orientation.z = 0.0;
  box.pose.orientation.w = 1.0;
  box.scale.x = local_planner_.histogram_box_size_.xmax + local_planner_.histogram_box_size_.xmin;
  box.scale.y = local_planner_.histogram_box_size_.ymax + local_planner_.histogram_box_size_.ymin;
  box.scale.z = local_planner_.histogram_box_size_.zmax + local_planner_.histogram_box_size_.zmin;
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
  groundbox.pose.position.x = drone_pos.pose.position.x + 0.5 * (local_planner_.ground_detector_.ground_box_size_.xmax - local_planner_.ground_detector_.ground_box_size_.xmin);
  groundbox.pose.position.y = drone_pos.pose.position.y + 0.5 * (local_planner_.ground_detector_.ground_box_size_.ymax - local_planner_.ground_detector_.ground_box_size_.ymin);
  groundbox.pose.position.z = drone_pos.pose.position.z - local_planner_.ground_detector_.ground_box_size_.zmin;
  groundbox.pose.orientation.x = 0.0;
  groundbox.pose.orientation.y = 0.0;
  groundbox.pose.orientation.z = 0.0;
  groundbox.pose.orientation.w = 1.0;
  groundbox.scale.x = local_planner_.ground_detector_.ground_box_size_.xmax + local_planner_.ground_detector_.ground_box_size_.xmin;
  groundbox.scale.y = local_planner_.ground_detector_.ground_box_size_.ymax + local_planner_.ground_detector_.ground_box_size_.ymin;
  groundbox.scale.z = 2 * local_planner_.ground_detector_.ground_box_size_.zmin;
  groundbox.color.a = 0.5;
  groundbox.color.r = 1.0;
  groundbox.color.g = 0.0;
  groundbox.color.b = 0.0;
  groundbox_pub_.publish(groundbox);
}

void LocalPlannerNode::publishAvoidSphere() {
  geometry_msgs::Point center;
  double radius;
  int age;
  bool use_sphere;
  local_planner_.getAvoidSphere(center, radius, age, use_sphere);
  visualization_msgs::Marker sphere;
  sphere.header.frame_id = "local_origin";
  sphere.header.stamp = ros::Time::now();
  sphere.id = 0;
  sphere.type = visualization_msgs::Marker::SPHERE;
  sphere.action = visualization_msgs::Marker::ADD;
  sphere.pose.position.x = center.x;
  sphere.pose.position.y = center.y;
  sphere.pose.position.z = center.z;
  sphere.pose.orientation.x = 0.0;
  sphere.pose.orientation.y = 0.0;
  sphere.pose.orientation.z = 0.0;
  sphere.pose.orientation.w = 1.0;
  sphere.scale.x = 2*radius;
  sphere.scale.y = 2*radius;
  sphere.scale.z = 2*radius;
  sphere.color.a = 0.5;
  sphere.color.r = 0.0;
  sphere.color.g = 1.0;
  sphere.color.b = 1.0;
  if(use_sphere && age < 100){
    avoid_sphere_pub_.publish(sphere);
  }else{
    sphere.color.a = 0;
    avoid_sphere_pub_.publish(sphere);
  }
}

void LocalPlannerNode::publishWaypoints() {
  geometry_msgs::Vector3Stamped waypt_original;
  geometry_msgs::Vector3Stamped waypt_adapted;
  geometry_msgs::Vector3Stamped waypt_smoothed;
  local_planner_.getWaypoints(waypt_original, waypt_adapted, waypt_smoothed);
  visualization_msgs::Marker sphere1;
  visualization_msgs::Marker sphere2;
  visualization_msgs::Marker sphere3;


  sphere1.header.frame_id = "local_origin";
  sphere1.header.stamp = ros::Time::now();
  sphere1.id = 0;
  sphere1.type = visualization_msgs::Marker::SPHERE;
  sphere1.action = visualization_msgs::Marker::ADD;
  sphere1.pose.position.x = waypt_original.vector.x;
  sphere1.pose.position.y = waypt_original.vector.y;
  sphere1.pose.position.z = waypt_original.vector.z;
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
  sphere2.header.stamp = ros::Time::now();
  sphere2.id = 0;
  sphere2.type = visualization_msgs::Marker::SPHERE;
  sphere2.action = visualization_msgs::Marker::ADD;
  sphere2.pose.position.x = waypt_adapted.vector.x;
  sphere2.pose.position.y = waypt_adapted.vector.y;
  sphere2.pose.position.z = waypt_adapted.vector.z;
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
  sphere3.header.stamp = ros::Time::now();
  sphere3.id = 0;
  sphere3.type = visualization_msgs::Marker::SPHERE;
  sphere3.action = visualization_msgs::Marker::ADD;
  sphere3.pose.position.x = waypt_smoothed.vector.x;
  sphere3.pose.position.y = waypt_smoothed.vector.y;
  sphere3.pose.position.z = waypt_smoothed.vector.z;
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
  std::vector<geometry_msgs::Point> path_node_positions;
  local_planner_.getTree(tree, closed_set, path_node_positions);

  for(int i=0; i<closed_set.size(); i++){
    int node_nr = closed_set[i];
    geometry_msgs::Point p1 = tree[node_nr].getPosition();
    int origin = tree[node_nr].origin;
    geometry_msgs::Point p2 = tree[origin].getPosition();
    tree_marker.points.push_back(p1);
    tree_marker.points.push_back(p2);
  }

  if (path_node_positions.size() > 0) {
    for (int i = 0; i < (path_node_positions.size() - 1); i++) {
      path_marker.points.push_back(path_node_positions[i]);
      path_marker.points.push_back(path_node_positions[i + 1]);
    }
  }

  complete_tree_pub_.publish(tree_marker);
  tree_path_pub_.publish(path_marker);
}

void LocalPlannerNode::clickedPointCallback(const geometry_msgs::PointStamped &msg) {
  printPointInfo(msg.point.x, msg.point.y, msg.point.z);
}

void LocalPlannerNode::clickedGoalCallback(const geometry_msgs::PoseStamped &msg) {
  local_planner_.goal_x_param_ = msg.pose.position.x;
  local_planner_.goal_y_param_ = msg.pose.position.y;
  local_planner_.setGoal();
}

void LocalPlannerNode::printPointInfo(double x, double y, double z) {
  geometry_msgs::PoseStamped drone_pos;
  local_planner_.getPosition(drone_pos);
  int beta_z = floor((atan2(x - drone_pos.pose.position.x, y - drone_pos.pose.position.y)*180.0/PI)); //(-180. +180]
  int beta_e = floor((atan((z - drone_pos.pose.position.z) / sqrt((x - drone_pos.pose.position.x)*(x - drone_pos.pose.position.x) + (y - drone_pos.pose.position.y) * (y - drone_pos.pose.position.y)))*180.0/PI)); //(-90.+90)

  beta_z = beta_z + (ALPHA_RES - beta_z%ALPHA_RES); //[-170,+190]
  beta_e = beta_e + (ALPHA_RES - beta_e%ALPHA_RES); //[-80,+90]

//  float cost = local_planner_.costFunction(beta_e-10, beta_z-10);

  printf("----- Point: %f %f %f -----\n",x,y,z);
  printf("Elevation %d Azimuth %d \n",beta_e, beta_z);
//  printf("Cost %f \n", cost);
  printf("-------------------------------------------- \n");
}

void LocalPlannerNode::pointCloudCallback(const sensor_msgs::PointCloud2 msg) {
  pcl::PointCloud<pcl::PointXYZ> complete_cloud;
  sensor_msgs::PointCloud2 pc2cloud_world;
  try {
    tf_listener_.waitForTransform("/world",msg.header.frame_id, msg.header.stamp, ros::Duration(3.0));
    tf::StampedTransform transform;
    tf_listener_.lookupTransform("/world", msg.header.frame_id, msg.header.stamp, transform);
    pcl_ros::transformPointCloud("/world", transform, msg, pc2cloud_world);
    pcl::fromROSMsg(pc2cloud_world, complete_cloud);
    local_planner_.complete_cloud_ = complete_cloud;
    point_cloud_updated_ = true;
  } catch(tf::TransformException& ex) {
    ROS_ERROR("Received an exception trying to transform a point from \"camera_optical_frame\" to \"world\": %s", ex.what());
  }
}

void LocalPlannerNode::publishSetpoint(const geometry_msgs::PoseStamped wp, double mode) {

  visualization_msgs::Marker setpoint;
  setpoint.header.frame_id = "local_origin";
  setpoint.header.stamp = ros::Time::now();
  setpoint.id = 0;
  setpoint.type = visualization_msgs::Marker::ARROW;
  setpoint.action = visualization_msgs::Marker::ADD;
  setpoint.pose = wp.pose;
  setpoint.scale.x = 1.0;
  setpoint.scale.y = 0.1;
  setpoint.scale.z = 0.1;
  setpoint.color.a = 1.0;

  if (mode == 0) {          //reach altitude
    setpoint.color.r = 1.0;
    setpoint.color.g = 0.0;
    setpoint.color.b = 1.0;
  } else if (mode == 1) {   //go fast
    setpoint.color.r = 0.0;
    setpoint.color.g = 0.0;
    setpoint.color.b = 1.0;
  } else if (mode == 2) {   //obstacle ahead
    setpoint.color.r = 0.0;
    setpoint.color.g = 1.0;
    setpoint.color.b = 0.0;
  } else if (mode == 3) {   //stop in front
    setpoint.color.r = 0.0;
    setpoint.color.g = 1.0;
    setpoint.color.b = 1.0;
  } else if (mode == 4) {   //back off
    setpoint.color.r = 1.0;
    setpoint.color.g = 0.0;
    setpoint.color.b = 0.0;
  }else if (mode == 5) {   //hover
    setpoint.color.r = 1.0;
    setpoint.color.g = 1.0;
    setpoint.color.b = 0.0;
  }

  current_waypoint_pub_.publish(setpoint);
}

void LocalPlannerNode::publishAll() {

  pcl::PointCloud<pcl::PointXYZ> final_cloud, ground_cloud, reprojected_points;
  local_planner_.getCloudsForVisualization(final_cloud, ground_cloud, reprojected_points);
  local_pointcloud_pub_.publish(final_cloud);
  ground_pointcloud_pub_.publish(ground_cloud);
  reprojected_points_pub_.publish(reprojected_points);

  nav_msgs::Path path_msg;
  geometry_msgs::PoseStamped waypt_p;
  local_planner_.getPathData(path_msg, waypt_p);
  waypoint_pub_.publish(path_msg);
  path_pub_.publish(path_actual);


 //choose setpoint color depending on mode
  double mode = local_planner_.local_planner_mode_;
  publishSetpoint(waypt_p, mode);

  tf_listener_.transformPose("local_origin", ros::Time(0), waypt_p, "world", waypt_p);
  mavros_waypoint_pub_.publish(waypt_p);
  hover_point_ = waypt_p;

  nav_msgs::GridCells path_candidates, path_selected, path_rejected, path_blocked, path_ground, FOV_cells;
  local_planner_.getCandidateDataForVisualization(path_candidates, path_selected, path_rejected, path_blocked, FOV_cells, path_ground);

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
  publishTree();
  publishWaypoints();
  publishGround();
}

void LocalPlannerNode::dynamicReconfigureCallback(avoidance::LocalPlannerNodeConfig & config,
                                                   uint32_t level) {

  local_planner_.dynamicReconfigureSetParams(config, level);
  depth_points_topic_ = config.callback_topic;
  pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(depth_points_topic_, 1, &LocalPlannerNode::pointCloudCallback, this);
}

void LocalPlannerNode::threadFunction() {
  std::unique_lock < std::timed_mutex > lock(variable_mutex_, std::defer_lock);
  while (true) {
    std::clock_t start_time = std::clock();
    if (point_cloud_updated_) {
      lock.lock();
      point_cloud_updated_ = false;
      never_run_ = false;
      local_planner_.runPlanner();
      publishAll();
      lock.unlock();

      printf("Total time: %2.2f ms \n", (std::clock() - start_time) / (double) (CLOCKS_PER_SEC / 1000));
      local_planner_.algorithm_total_time_.push_back((std::clock() - start_time) / (double) (CLOCKS_PER_SEC / 1000));

      local_planner_.printAlgorithmStatistics();

      //Store timing for timeout detection
      pointcloud_time_old_ = pointcloud_time_now_;
      pcl_conversions::fromPCL(local_planner_.complete_cloud_.header.stamp, pointcloud_time_now_);

      //publish log name
      std_msgs::String msg;
      msg.data = local_planner_.log_name_;
      log_name_pub_.publish(msg);
    }
  }
}



int main(int argc, char** argv) {
  ros::init(argc, argv, "local_planner_node");
  LocalPlannerNode * NodePtr = new LocalPlannerNode();
  ros::Duration(2).sleep();
  ros::Rate rate(10.0);
  ros::Time start_time = ros::Time::now();
  NodePtr->point_cloud_updated_ = false;
  NodePtr->never_run_ = true;
  NodePtr->position_received_ = false;
  bool writing = false;

  std::thread worker(&LocalPlannerNode::threadFunction, NodePtr);
  std::unique_lock < std::timed_mutex > lock(NodePtr->variable_mutex_, std::defer_lock);

  while (ros::ok()) {
    std::clock_t t_loop1 = std::clock();
    writing = false;

    //spin node, execute callbacks
    if (lock.try_lock_for(std::chrono::milliseconds(20))) {
      ros::spinOnce();
      lock.unlock();
      writing = true;
    }
    std::clock_t t_loop2 = std::clock();

    //Check if pointcloud message is published fast enough
    ros::Time now = ros::Time::now();
    ros::Duration pointcloud_timeout_hover = ros::Duration(NodePtr->local_planner_.pointcloud_timeout_hover_);
    ros::Duration pointcloud_timeout_land = ros::Duration(NodePtr->local_planner_.pointcloud_timeout_land_);
    ros::Duration since_last_cloud = now - NodePtr->pointcloud_time_now_;
    ros::Duration since_start = now - start_time;
    bool landing = false;
    bool hovering = false;

    if (since_last_cloud > pointcloud_timeout_land && since_start > pointcloud_timeout_land) {
      mavros_msgs::SetMode mode_msg;
      mode_msg.request.custom_mode = "AUTO.LAND";
      landing = true;
      if (NodePtr->mavros_set_mode_client_.call(mode_msg) && mode_msg.response.mode_sent) {
        std::cout << "\033[1;33m Pointcloud timeout: Landing \n \033[0m";
      } else {
        std::cout << "\033[1;33m Pointcloud timeout: Landing failed! \n \033[0m";
      }
    } else if (since_last_cloud > pointcloud_timeout_hover && since_start > pointcloud_timeout_hover) {
      if (!NodePtr->never_run_) {
        std::cout << "\033[1;33m Pointcloud timeout: Repeating last waypoint \n \033[0m";
        NodePtr->local_planner_.useHoverPoint();
        nav_msgs::Path path_msg;
        geometry_msgs::PoseStamped waypt_p;
        NodePtr->local_planner_.getPathData(path_msg, waypt_p);
        NodePtr->waypoint_pub_.publish(path_msg);
        NodePtr->publishSetpoint(waypt_p, 5);
        NodePtr->mavros_waypoint_pub_.publish(NodePtr->hover_point_);
        hovering = true;
      } else {
        if (NodePtr->position_received_) {
          geometry_msgs::PoseStamped drone_pos;
          NodePtr->local_planner_.getPosition(drone_pos);
          NodePtr->publishSetpoint(drone_pos, 5);
          NodePtr->mavros_waypoint_pub_.publish(NodePtr->hover_current_pose_);
          hovering = true;
          std::cout << "\033[1;33m Pointcloud timeout: Hovering at current position \n \033[0m";
        } else {
          std::cout << "\033[1;33m Pointcloud timeout: No position received, no WP to output.... \n \033[0m";
        }
      }
    }

    if (NodePtr->local_planner_.currently_armed_ && NodePtr->local_planner_.offboard_) {
      ros::Duration time_diff = NodePtr->pointcloud_time_now_ - NodePtr->pointcloud_time_old_;
      std::ofstream myfile(("PointcloudTimes_" + NodePtr->local_planner_.log_name_).c_str(), std::ofstream::app);
      myfile << NodePtr->pointcloud_time_now_.sec << "\t" << NodePtr->pointcloud_time_now_.nsec << "\t" << time_diff << "\t" << hovering << "\t" << landing << "\t" << writing << "\t" << (t_loop2 - t_loop1) / (double) (CLOCKS_PER_SEC / 1000) << "\t"
          << (std::clock() - t_loop2) / (double) (CLOCKS_PER_SEC / 1000) << "\n";
      myfile.close();
    }

    rate.sleep();
  }

  worker.join();
  delete NodePtr;
  return 0;
}
