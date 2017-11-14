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
  clicked_point_sub_ = nh_.subscribe("/clicked_point", 1, &LocalPlannerNode::clickedPointCallback, this);
  clicked_goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &LocalPlannerNode::clickedGoalCallback, this);

  local_pointcloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/local_pointcloud", 1);
  reprojected_points_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/reprojected_points", 1);
  transformed_pointcloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/transformed_pointcloud", 1);
  bounding_box_pub_ = nh_.advertise<visualization_msgs::Marker>("/bounding_box", 1);
  marker_blocked_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/blocked_marker", 1);
  marker_rejected_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/rejected_marker", 1);
  marker_candidates_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/candidates_marker", 1);
  marker_selected_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/selected_marker", 1);
  marker_goal_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/goal_position", 1);
  waypoint_pub_ = nh_.advertise<nav_msgs::Path>("/waypoint", 1);
  path_pub_ = nh_.advertise<nav_msgs::Path>("/path_actual", 1);
  bounding_box_pub_ = nh_.advertise<visualization_msgs::Marker>("/bounding_box", 1);
  mavros_waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  current_waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/current_setpoint", 1);

  local_planner.setGoal();
}

LocalPlannerNode::~LocalPlannerNode() {}

void LocalPlannerNode::readParams() {
  nh_.param<double>("goal_x_param", local_planner.goal_x_param_, 9);
  nh_.param<double>("goal_y_param", local_planner.goal_y_param_, 13);
  nh_.param<double>("goal_z_param", local_planner.goal_z_param_, 3.5);

  nh_.param<std::string>("depth_points_topic", depth_points_topic_, "/camera/depth/points");
}

void LocalPlannerNode::positionCallback(const geometry_msgs::PoseStamped msg) {
  auto rot_msg = msg;
  tf_listener_.transformPose("world", ros::Time(0), msg, "local_origin", rot_msg);
  local_planner.setPose(rot_msg);
  publishPath(rot_msg);
}

void LocalPlannerNode::velocityCallback(const geometry_msgs::TwistStamped msg) {
  auto transformed_msg = avoidance::transformTwistMsg(tf_listener_, "/world", "/local_origin", msg); // 90 deg fix
  local_planner.curr_vel_ = transformed_msg;
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

  for (int i=0; i < path.cells.size(); i++) {
    m.id = i+1;
    m.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point p = local_planner.fromPolarToCartesian((int)path.cells[i].x, (int)path.cells[i].y, local_planner.rad, local_planner.pose.pose.position);
    m.pose.position = p;

    m.color.r = red;
    m.color.g = green;
    m.color.b = blue;

    marker->markers.push_back(m);
  }
}

void LocalPlannerNode::publishMarkerBlocked() {
  visualization_msgs::MarkerArray marker_blocked;
  initMarker(&marker_blocked, local_planner.path_blocked_, 0.0, 0.0, 1.0);
  marker_blocked_pub_.publish(marker_blocked);
}

void LocalPlannerNode::publishMarkerRejected() {
  visualization_msgs::MarkerArray marker_rejected;
  initMarker(&marker_rejected, local_planner.path_rejected_, 1.0, 0.0, 0.0);
  marker_rejected_pub_.publish(marker_rejected);
}

void LocalPlannerNode::publishMarkerCandidates() {
  visualization_msgs::MarkerArray marker_candidates;
  initMarker(&marker_candidates, local_planner.path_candidates_, 0.0, 1.0, 0.0);
  marker_candidates_pub_.publish(marker_candidates);
}

void LocalPlannerNode::publishMarkerSelected() {
  visualization_msgs::MarkerArray marker_selected;
  initMarker(&marker_selected, local_planner.path_selected_, 0.8, 0.16, 0.8);
  marker_selected_pub_.publish(marker_selected);
}

void LocalPlannerNode::publishGoal() {
  visualization_msgs::MarkerArray marker_goal;
  visualization_msgs::Marker m;

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
  m.pose.position = local_planner.goal_;
  marker_goal.markers.push_back(m);
  marker_goal_pub_.publish(marker_goal);
}

void LocalPlannerNode::publishBox() {
  visualization_msgs::Marker box;
  box.header.frame_id = "local_origin";
  box.header.stamp = ros::Time::now();
  box.id = 0;
  box.type = visualization_msgs::Marker::CUBE;
  box.action = visualization_msgs::Marker::ADD;
  box.pose.position.x = local_planner.pose_.pose.position.x + 0.5*(local_planner.max_box_x_-local_planner.min_box_x_);
  box.pose.position.y = local_planner.pose_.pose.position.y + 0.5*(local_planner.max_box_y_-local_planner.min_box_y_);
  box.pose.position.z = local_planner.pose_.pose.position.z + 0.5*(local_planner.max_box_z_-local_planner.min_box_z_);
  box.pose.orientation.x = 0.0;
  box.pose.orientation.y = 0.0;
  box.pose.orientation.z = 0.0;
  box.pose.orientation.w = 1.0;
  box.scale.x = (local_planner.max_box_x_ + local_planner.min_box_x_);
  box.scale.y = (local_planner.max_box_y_ + local_planner.min_box_y_);
  box.scale.z = (local_planner.max_box_z_ + local_planner.min_box_z_);
  box.color.a = 0.5;
  box.color.r = 0.0;
  box.color.g = 1.0;
  box.color.b = 0.0;
  bounding_box_pub_.publish(box);
}

void LocalPlannerNode::clickedPointCallback(const geometry_msgs::PointStamped &msg) {
  printPointInfo(msg.point.x, msg.point.y, msg.point.z);
}

void LocalPlannerNode::clickedGoalCallback(const geometry_msgs::PoseStamped &msg) {
  local_planner.goal_x_param_ = msg.pose.position.x;
  local_planner.goal_y_param_ = msg.pose.position.y;
  local_planner.setGoal();
}

void LocalPlannerNode::printPointInfo(double x, double y, double z) {
  int beta_z = floor((atan2(x - local_planner.pose_.pose.position.x, y - local_planner.pose_.pose.position.y)*180.0/PI)); //(-180. +180]
  int beta_e = floor((atan((z - local_planner.pose_.pose.position.z) / sqrt((x - local_planner.pose_.pose.position.x)*(x - local_planner.pose_.pose.position.x) + (y - local_planner.pose_.pose.position.y) * (y - local_planner.pose_.pose.position.y)))*180.0/PI)); //(-90.+90)

  beta_z = beta_z + (alpha_res - beta_z%alpha_res); //[-170,+190]
  beta_e = beta_e + (alpha_res - beta_e%alpha_res); //[-80,+90]

  float cost = local_planner.costFunction(beta_e-10, beta_z-10);

  printf("----- Point: %f %f %f -----\n",x,y,z);
  printf("Elevation %d Azimuth %d \n",beta_e, beta_z);
  printf("Cost %f \n", cost);
  printf("-------------------------------------------- \n");
}

void LocalPlannerNode::pointCloudCallback(const sensor_msgs::PointCloud2 msg) {
  pcl::PointCloud<pcl::PointXYZ> complete_cloud;
  sensor_msgs::PointCloud2 pc2cloud_world;
  std::clock_t start_time = std::clock();
  try {
    tf_listener_.waitForTransform("/world",msg.header.frame_id, msg.header.stamp, ros::Duration(3.0));
    tf::StampedTransform transform;
    tf_listener_.lookupTransform("/world", msg.header.frame_id, msg.header.stamp, transform);
    pcl_ros::transformPointCloud("/world", transform, msg, pc2cloud_world);
    pcl::fromROSMsg(pc2cloud_world, complete_cloud);
    local_planner.new_cloud_ = true;
    local_planner.n_call_hist_ = 0;
    local_planner.filterPointCloud(complete_cloud);
    publishAll();
  } catch(tf::TransformException& ex) {
    ROS_ERROR("Received an exception trying to transform a point from \"camera_optical_frame\" to \"world\": %s", ex.what());
  }

  printf("Total time: %2.2f ms \n", (std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
  algo_time.push_back((std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));

  if (local_planner.withinGoalRadius()) {
    cv::Scalar mean, std;
    printf("----------------------------------- \n");
    cv::meanStdDev(algo_time, mean, std); printf("total mean %f std %f \n", mean[0], std[0]);
    cv::meanStdDev(local_planner.cloud_time_, mean, std); printf("cloud mean %f std %f \n", mean[0], std[0]);
    cv::meanStdDev(local_planner.polar_time_, mean, std); printf("polar mean %f std %f \n", mean[0], std[0]);
    cv::meanStdDev(local_planner.free_time_, mean, std); printf("free mean %f std %f \n", mean[0], std[0]);
    cv::meanStdDev(local_planner.cost_time_, mean, std); printf("cost mean %f std %f \n", mean[0], std[0]);
    cv::meanStdDev(local_planner.collision_time_, mean, std); printf("collision mean %f std %f \n", mean[0], std[0]);
    printf("----------------------------------- \n");
  }
}

void LocalPlannerNode::publishAll() {
  ROS_INFO("Current pose: [%f, %f, %f].", local_planner.pose_.pose.position.x, local_planner.pose_.pose.position.y, local_planner.pose_.pose.position.z);
  ROS_INFO("Velocity: [%f, %f, %f], module: %f.", local_planner.velocity_x_, local_planner.velocity_y_, local_planner.velocity_z_, local_planner.velocity_mod_);

  local_pointcloud_pub_.publish(local_planner.final_cloud_);
  waypoint_pub_.publish(local_planner.path_msg_);
  reprojected_points_pub_.publish(local_planner.reprojected_points);
  path_pub_.publish(path_actual);
  current_waypoint_pub_.publish(local_planner.waypt_p_);
  tf_listener_.transformPose("local_origin", ros::Time(0), local_planner.waypt_p_, "world", local_planner.waypt_p_);
  mavros_waypoint_pub_.publish(local_planner.waypt_p_);

  publishMarkerBlocked();
  publishMarkerCandidates();
  publishMarkerRejected();
  publishMarkerSelected();
  publishGoal();
  publishBox();
}

void LocalPlannerNode::dynamicReconfigureCallback(avoidance::LocalPlannerNodeConfig & config,
                                                   uint32_t level) {
  local_planner.min_box_x_ = config.min_box_x_;
  local_planner.max_box_x_ = config.max_box_x_;
  local_planner.min_box_y_ = config.min_box_y_;
  local_planner.max_box_y_ = config.max_box_y_;
  local_planner.min_box_z_ = config.min_box_z_;
  local_planner.max_box_z_ = config.max_box_z_;
  local_planner.rad_ = config.rad_;
  local_planner.goal_cost_param_ = config.goal_cost_param_;
  local_planner.smooth_cost_param_ = config.smooth_cost_param_;
  local_planner.prior_cost_param_ = config.prior_cost_param_;
  local_planner.min_speed_ = config.min_speed_;
  local_planner.max_speed_ = config.max_speed_;
  local_planner.max_accel_z_ = config.max_accel_z_;
  local_planner.stop_in_front_ = config.stop_in_front_;
  local_planner.keep_distance_ = config.keep_distance_;

  if (local_planner.goal_z_param_!= config.goal_z_param_) {
	  local_planner.goal_z_param_ = config.goal_z_param_;
	  local_planner.setGoal();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_planner_node");
  LocalPlannerNode local_planner_node;
  ros::Duration(2).sleep();
  ros::spin();

  return 0;
}
