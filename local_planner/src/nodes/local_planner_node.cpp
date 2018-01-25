#include "local_planner_node.h"

LocalPlannerNode::LocalPlannerNode() {
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
  ground_est_pub_ = nh_.advertise<visualization_msgs::Marker>("/ground_est", 1);
  height_map_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/height_map", 1);
  marker_blocked_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/blocked_marker", 1);
  marker_rejected_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/rejected_marker", 1);
  marker_candidates_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/candidates_marker", 1);
  marker_ground_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/ground_marker", 1);
  marker_selected_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/selected_marker", 1);
  marker_goal_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/goal_position", 1);
  waypoint_pub_ = nh_.advertise<nav_msgs::Path>("/waypoint", 1);
  path_pub_ = nh_.advertise<nav_msgs::Path>("/path_actual", 1);
  bounding_box_pub_ = nh_.advertise<visualization_msgs::Marker>("/bounding_box", 1);
  mavros_waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  current_waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/current_setpoint", 1);

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
    geometry_msgs::Point p = local_planner_.fromPolarToCartesian((int)path.cells[i].x, (int)path.cells[i].y, 1.0, drone_pos.pose.position);
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
  local_planner_.getGroundDataForVisualization(closest_point_on_ground,ground_orientation, ground_heights, ground_xmax, ground_xmin, ground_ymax, ground_ymin);
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

void LocalPlannerNode::publishBox() {
  geometry_msgs::PoseStamped drone_pos;
  local_planner_.getPosition(drone_pos);
  double min_x, max_x, min_y, max_y, min_z, max_z;
  local_planner_.getBoundingBoxSize(min_x, max_x, min_y, max_y, min_z, max_z);
  visualization_msgs::Marker box;
  box.header.frame_id = "local_origin";
  box.header.stamp = ros::Time::now();
  box.id = 0;
  box.type = visualization_msgs::Marker::CUBE;
  box.action = visualization_msgs::Marker::ADD;
  box.pose.position.x = drone_pos.pose.position.x + 0.5 * (max_x - min_x);
  box.pose.position.y = drone_pos.pose.position.y + 0.5 * (max_y - min_y);
  box.pose.position.z = drone_pos.pose.position.z + 0.5 * (max_z - min_z);
  box.pose.orientation.x = 0.0;
  box.pose.orientation.y = 0.0;
  box.pose.orientation.z = 0.0;
  box.pose.orientation.w = 1.0;
  box.scale.x = max_x + min_x;
  box.scale.y = max_y + min_y;
  box.scale.z = max_z + min_z;
  box.color.a = 0.5;
  box.color.r = 0.0;
  box.color.g = 1.0;
  box.color.b = 0.0;
  bounding_box_pub_.publish(box);

  local_planner_.getGroundBoxSize(min_x, max_x, min_y, max_y, min_z);
  visualization_msgs::Marker groundbox;
  groundbox.header.frame_id = "local_origin";
  groundbox.header.stamp = ros::Time::now();
  groundbox.id = 0;
  groundbox.type = visualization_msgs::Marker::CUBE;
  groundbox.action = visualization_msgs::Marker::ADD;
  groundbox.pose.position.x = drone_pos.pose.position.x + 0.5 * (max_x - min_x);
  groundbox.pose.position.y = drone_pos.pose.position.y + 0.5 * (max_y - min_y);
  groundbox.pose.position.z = drone_pos.pose.position.z - min_z;
  groundbox.pose.orientation.x = 0.0;
  groundbox.pose.orientation.y = 0.0;
  groundbox.pose.orientation.z = 0.0;
  groundbox.pose.orientation.w = 1.0;
  groundbox.scale.x = max_x + min_x;
  groundbox.scale.y = max_y + min_y;
  groundbox.scale.z = 2 * min_z;
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
  sphere.color.r = 0.5;
  sphere.color.g = 0.0;
  sphere.color.b = 0.5;
  if(use_sphere && age < 100){
    avoid_sphere_pub_.publish(sphere);
  }else{
    sphere.color.a = 0;
    avoid_sphere_pub_.publish(sphere);
  }
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

  beta_z = beta_z + (alpha_res - beta_z%alpha_res); //[-170,+190]
  beta_e = beta_e + (alpha_res - beta_e%alpha_res); //[-80,+90]

  float cost = local_planner_.costFunction(beta_e-10, beta_z-10);

  printf("----- Point: %f %f %f -----\n",x,y,z);
  printf("Elevation %d Azimuth %d \n",beta_e, beta_z);
  printf("Cost %f \n", cost);
  printf("-------------------------------------------- \n");
}

void LocalPlannerNode::pointCloudCallback(const sensor_msgs::PointCloud2 msg) {
  geometry_msgs::PoseStamped drone_pos;
  local_planner_.getPosition(drone_pos);
  pcl::PointCloud<pcl::PointXYZ> complete_cloud;
  sensor_msgs::PointCloud2 pc2cloud_world;
  std::clock_t start_time = std::clock();
  try {
    tf_listener_.waitForTransform("/world",msg.header.frame_id, msg.header.stamp, ros::Duration(3.0));
    tf::StampedTransform transform;
    tf_listener_.lookupTransform("/world", msg.header.frame_id, msg.header.stamp, transform);
    pcl_ros::transformPointCloud("/world", transform, msg, pc2cloud_world);
    pcl::fromROSMsg(pc2cloud_world, complete_cloud);
    local_planner_.resetHistogramCounter();
    local_planner_.filterPointCloud(complete_cloud);
    publishAll();
  } catch(tf::TransformException& ex) {
    ROS_ERROR("Received an exception trying to transform a point from \"camera_optical_frame\" to \"world\": %s", ex.what());
  }

  printf("Total time: %2.2f ms \n", (std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
  local_planner_.algorithm_total_time_.push_back((std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));

  local_planner_.printAlgorithmStatistics();

  //Store timing for timeout detection
  pointcloud_time_old_ = pointcloud_time_now_;
  pointcloud_time_now_ = msg.header.stamp;
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

  current_waypoint_pub_.publish(waypt_p);
  tf_listener_.transformPose("local_origin", ros::Time(0), waypt_p, "world", waypt_p);
  mavros_waypoint_pub_.publish(waypt_p);

  nav_msgs::GridCells path_candidates, path_selected, path_rejected, path_blocked, path_ground;
  local_planner_.getCandidateDataForVisualization(path_candidates, path_selected, path_rejected, path_blocked, path_ground);

  publishMarkerCandidates(path_candidates);
  publishMarkerSelected(path_selected);
  publishMarkerRejected(path_rejected);
  publishMarkerBlocked(path_blocked);
  publishMarkerGround(path_ground);
  publishGoal();
  publishBox();
  publishAvoidSphere();

  if (local_planner_.groundDetected()) {
    publishGround();
  }
}

void LocalPlannerNode::dynamicReconfigureCallback(avoidance::LocalPlannerNodeConfig & config,
                                                   uint32_t level) {

  local_planner_.dynamicReconfigureSetParams(config, level);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_planner_node");
  LocalPlannerNode local_planner_node;
  ros::Duration(2).sleep();
  ros::Rate rate(20.0);
  ros::Time start_time = ros::Time::now();

  while (ros::ok()) {

    //Check if pointcloud message is published fast enough
    ros::Time now = ros::Time::now();
    ros::Duration pointcloud_timeout_hover = ros::Duration(local_planner_node.local_planner_.pointcloud_timeout_hover_);
    ros::Duration pointcloud_timeout_land = ros::Duration(local_planner_node.local_planner_.pointcloud_timeout_land_);
    ros::Duration since_last_cloud = now - local_planner_node.pointcloud_time_now_;
    ros::Duration since_start = now - start_time;
    bool landing = false;
    bool hovering = false;

    if (since_last_cloud > pointcloud_timeout_land && since_start > pointcloud_timeout_land) {
      mavros_msgs::SetMode mode_msg;
      mode_msg.request.custom_mode = "AUTO.LAND";
      landing = true;
      if (local_planner_node.mavros_set_mode_client_.call(mode_msg) && mode_msg.response.mode_sent) {
        std::cout << "\033[1;33m Pointcloud timeout: Landing \n \033[0m";
      } else {
        std::cout << "\033[1;33m Pointcloud timeout: Landing failed! \n \033[0m";
      }
    } else if (since_last_cloud > pointcloud_timeout_hover && since_start > pointcloud_timeout_hover) {
      local_planner_node.local_planner_.hover();
      nav_msgs::Path path_msg;
      geometry_msgs::PoseStamped waypt_p;
      local_planner_node.local_planner_.getPathData(path_msg, waypt_p);
      local_planner_node.waypoint_pub_.publish(path_msg);

      local_planner_node.current_waypoint_pub_.publish(waypt_p);
      local_planner_node.tf_listener_.transformPose("local_origin", ros::Time(0), waypt_p, "world", waypt_p);
      local_planner_node.mavros_waypoint_pub_.publish(waypt_p);
      hovering = true;
    }

    ros::Duration time_diff = local_planner_node.pointcloud_time_now_ - local_planner_node.pointcloud_time_old_;
    std::ofstream myfile(("PointcloudTimes_" + local_planner_node.local_planner_.log_name_).c_str(), std::ofstream::app);
    myfile << local_planner_node.pointcloud_time_now_.sec <<"\t"<<local_planner_node.pointcloud_time_now_.nsec << "\t" << time_diff<< "\t" << hovering << "\t" << landing<< "\n";
    myfile.close();

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
