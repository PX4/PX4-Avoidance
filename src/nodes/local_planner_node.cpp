#include "local_planner_node.h"

LocalPlannerNode::LocalPlannerNode() {
  nh_ = ros::NodeHandle("~"); 

	pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/omi_cam/point_cloud", 1, &LocalPlannerNode::pointCloudCallback, this);
	pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &LocalPlannerNode::positionCallback, this);
	velocity_sub_ = nh_.subscribe("/mavros/local_position/velocity", 1, &LocalPlannerNode::velocityCallback, this);
  clicked_point_sub_ = nh_.subscribe("/clicked_point", 1, &LocalPlannerNode::clickedPointCallback, this);

	local_pointcloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/local_pointcloud", 1);
  //marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>( "/visualization_marker", 1);
  marker_blocked_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/blocked_marker", 1);
  marker_rejected_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/rejected_marker", 1);
  marker_candidates_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/candidates_marker", 1);
  marker_selected_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/selected_marker", 1);
  marker_extended_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/extended_marker", 1);
  marker_goal_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/goal_position", 1);
  marker_normal_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/normal_powerline", 1);

  waypoint_pub_ = nh_.advertise<nav_msgs::Path>("/waypoint", 1);
  path_pub_ = nh_.advertise<nav_msgs::Path>("/path_actual", 1);

  mavros_waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  current_waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/current_setpoint", 1);

  readParams();
  local_planner.setGoal();
}

LocalPlannerNode::~LocalPlannerNode(){}

void LocalPlannerNode::positionCallback(const geometry_msgs::PoseStamped msg){
	
  auto rot_msg = msg;
  tf_listener_.transformPose("world", ros::Time(0), msg, "local_origin", rot_msg);
	local_planner.setPose(rot_msg);
	publishPath(rot_msg);
}

void LocalPlannerNode::velocityCallback(const geometry_msgs::TwistStamped msg) {

  auto transformed_msg = avoidance::transformTwistMsg(tf_listener_, "/world", "/local_origin", msg); // 90 deg fix
  local_planner.curr_vel = transformed_msg;
}

void LocalPlannerNode::readParams() {

  nh_.param<double>("goal_x_param", local_planner.goal_x_param, 9);
  nh_.param<double>("goal_y_param", local_planner.goal_y_param, 13);
  nh_.param<double>("goal_z_param", local_planner.goal_z_param, 3.5);
}

void LocalPlannerNode::publishPath(const geometry_msgs::PoseStamped msg) {

    path_actual.header.stamp = msg.header.stamp;
    path_actual.header.frame_id = msg.header.frame_id;
    path_actual.poses.push_back(msg);
}

void LocalPlannerNode::publishMarkerBlocked() {

  visualization_msgs::MarkerArray marker_blocked;
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
  marker_blocked.markers.push_back(m);

  blocked_id=1;
  for (int i=0; i<local_planner.Ppath_blocked.cells.size(); i++){

    m.id = blocked_id;
    m.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point p;
    p.x = local_planner.Ppath_blocked.cells[i].x;
    p.y = local_planner.Ppath_blocked.cells[i].y;
    p.z = local_planner.Ppath_blocked.cells[i].z;
    m.pose.position = p;

    m.color.r = 0.0;
    m.color.g = 0.0;
    m.color.b = 1.0;

    marker_blocked.markers.push_back(m);
    blocked_id++;
      
  } 

  marker_blocked_pub_.publish(marker_blocked);
}

void LocalPlannerNode::publishMarkerRejected() {

  visualization_msgs::MarkerArray marker_rejected;
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
  marker_rejected.markers.push_back(m);

  rejected_id=1;
  for (int i=0; i<local_planner.Ppath_rejected.cells.size(); i++){

    m.id = rejected_id;
    m.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point p;
    p.x = local_planner.Ppath_rejected.cells[i].x;
    p.y = local_planner.Ppath_rejected.cells[i].y;
    p.z = local_planner.Ppath_rejected.cells[i].z;
    m.pose.position = p;

    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;

    marker_rejected.markers.push_back(m);
    rejected_id++;
  } 

  marker_rejected_pub_.publish(marker_rejected);
}

void LocalPlannerNode::publishMarkerCandidates(){

  visualization_msgs::MarkerArray marker_candidates;
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
  marker_candidates.markers.push_back(m);

  candidates_id = 1;
  for (int i=0; i<local_planner.Ppath_candidates.cells.size(); i++){

    m.id = candidates_id;
    m.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point p;
    p.x = local_planner.Ppath_candidates.cells[i].x;
    p.y = local_planner.Ppath_candidates.cells[i].y;
    p.z = local_planner.Ppath_candidates.cells[i].z;
    m.pose.position = p;
        
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;

    marker_candidates.markers.push_back(m);
    candidates_id++;
  } 

  marker_candidates_pub_.publish(marker_candidates);
}

void LocalPlannerNode::publishMarkerSelected(){

  visualization_msgs::MarkerArray marker_selected;
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
  marker_selected.markers.push_back(m);

  selected_id = 1;
  for (int i=0; i<local_planner.Ppath_selected.cells.size(); i++){

    m.id = selected_id;
    m.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point p;
    p.x = local_planner.Ppath_selected.cells[i].x;
    p.y = local_planner.Ppath_selected.cells[i].y;
    p.z = local_planner.Ppath_selected.cells[i].z;
    m.pose.position = p;
      
    m.color.r = 0.8;
    m.color.g = 0.16;
    m.color.b = 0.8;

    marker_selected.markers.push_back(m);
    selected_id++;
  } 

  marker_selected_pub_.publish(marker_selected);
}

void LocalPlannerNode::publishMarkerExtended(){

  visualization_msgs::MarkerArray marker_extended;
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
  marker_extended.markers.push_back(m);

  extended_id = 1;
  for (int i=0; i<local_planner.path_extended.cells.size(); i++){

    m.id = extended_id;
    m.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point p;
    p.x = local_planner.path_extended.cells[i].x;
    p.y = local_planner.path_extended.cells[i].y;
    p.z = local_planner.path_extended.cells[i].z;
    m.pose.position = p;

    m.color.r = 0.5;
    m.color.g = 0.5;
    m.color.b = 0.5;

    marker_extended.markers.push_back(m);
    extended_id++;
  } 

  marker_extended_pub_.publish(marker_extended);
}

void LocalPlannerNode::publishGoal(){
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
    m.pose.position = local_planner.goal;
    marker_goal.markers.push_back(m);
    marker_goal_pub_.publish(marker_goal);  
}


void LocalPlannerNode::publishNormalToPowerline(){

  visualization_msgs::MarkerArray marker_normal;
  visualization_msgs::Marker m;

  m.header.frame_id = "world";
  m.header.stamp = ros::Time::now();
  m.type = visualization_msgs::Marker::ARROW;
  m.action = 3;
  m.scale.x = 0.1;
  m.scale.y = 0.2;
  m.scale.z = 0.2;
  m.color.a = 1.0;
  m.color.r = 0.0;
  m.color.g = 0.0;
  m.color.b = 0.0;
  m.lifetime = ros::Duration();
  m.id = 0;
  marker_normal.markers.push_back(m);
   
  if(local_planner.obstacleAhead() && local_planner.init!=0) {

    geometry_msgs::Point start, end;
    start.x = local_planner.mean_x[0]; start.y = local_planner.mean_y[0]; start.z = local_planner.mean_z[0];
    end.x = local_planner.mean_x[0] + local_planner.coef1;
    end.y = local_planner.mean_y[0] + local_planner.coef2;
    end.z = local_planner.mean_z[0] + local_planner.coef3;

    int ext_id = 1;
    m.id = ext_id;
    m.action = visualization_msgs::Marker::ADD;
    m.points.push_back(start);
    m.points.push_back(end);
    marker_normal.markers.push_back(m);
    
    marker_normal_pub_.publish(marker_normal);  
  }
}

void LocalPlannerNode::clickedPointCallback(const geometry_msgs::PointStamped & msg){
  printPointInfo(msg.point.x, msg.point.y, msg.point.z);
}

void LocalPlannerNode::printPointInfo(double x, double y, double z){
 
  double rad = 1.0;
  int e = std::round(180/PI*asin((z-local_planner.pose.pose.position.z)/rad));
  float t = (x-local_planner.pose.pose.position.x)/(rad*cos(PI/180*e));
  if (t==1.0)
    t=0.99999999999;
  int az = std::round(180/PI*asin(t));
  int azz = std::round(180/PI*acos(t));

  az = az + (alpha_res - az%alpha_res); //[-170,+190]
  azz = azz + (alpha_res - azz%alpha_res); //[-170,+190]
  e = e + (alpha_res - e%alpha_res); //[-80,+90]

  printf("----- Point: %f %f %f -----\n",x,y,z);
  printf("Elevation %d Azimuth %d %d \n",e,az,azz);

  printf("-------------------------------------------- \n");
}

void LocalPlannerNode::pointCloudCallback(const sensor_msgs::PointCloud2 msg){

	pcl::PointCloud<pcl::PointXYZ> complete_cloud;
  sensor_msgs::PointCloud2 pc2cloud_world;
  std::clock_t start_time = std::clock();
  try{
    tf_listener_.waitForTransform("/world", msg.header.frame_id, msg.header.stamp, ros::Duration(3.0));
    tf::StampedTransform transform;
    tf_listener_.lookupTransform("/world", msg.header.frame_id, msg.header.stamp, transform);
    pcl_ros::transformPointCloud("/world", transform, msg, pc2cloud_world);
    pcl::fromROSMsg(pc2cloud_world, complete_cloud); 
    local_planner.filterPointCloud(complete_cloud);
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"camera_optical_frame\" to \"world\": %s", ex.what());
  }

  if(local_planner.obstacleAhead() && local_planner.init!=0) {
    printf("There is an Obstacle Ahead \n");
	  local_planner.createPolarHistogram();
	  local_planner.findFreeDirections();
	  local_planner.calculateCostMap();
    local_planner.getNextWaypoint();
    local_planner.getPathMsg();
	}
  else{
    printf("There isn't any Obstacle Ahead \n");
    local_planner.goFast();
    local_planner.getPathMsg();     
  }

  printf("Total time: %2.2f ms \n", (std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
  algo_time.push_back((std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
  if (local_planner.withinGoalRadius() && local_planner.first_reach){
    cv::Scalar mean, std;
    printf("----------------------------------- \n");
    cv::meanStdDev(algo_time, mean, std); printf("total mean %f std %f \n", mean[0], std[0]);
    cv::meanStdDev(local_planner.cloud_time, mean, std); printf("cloud mean %f std %f \n", mean[0], std[0]);
    cv::meanStdDev(local_planner.polar_time, mean, std); printf("polar mean %f std %f \n", mean[0], std[0]);
    cv::meanStdDev(local_planner.free_time, mean, std); printf("free mean %f std %f \n", mean[0], std[0]);
    cv::meanStdDev(local_planner.cost_time, mean, std); printf("cost mean %f std %f \n", mean[0], std[0]);
    cv::meanStdDev(local_planner.collision_time, mean, std); printf("collision mean %f std %f \n", mean[0], std[0]);
    printf("----------------------------------- \n");
  }

  publishAll();

  if(local_planner.init == 0) { 
    ros::Duration(2).sleep();
    local_planner.init =1;
  }

}

void LocalPlannerNode::publishAll() {

  ROS_INFO("Current pose: [%f, %f, %f].", local_planner.pose.pose.position.x, local_planner.pose.pose.position.y, local_planner.pose.pose.position.z);
  ROS_INFO("Velocity: [%f, %f, %f], module: %f.", local_planner.velocity_x, local_planner.velocity_y, local_planner.velocity_z, local_planner.velocity_mod);

	local_pointcloud_pub_.publish(local_planner.final_cloud);
  waypoint_pub_.publish(local_planner.path_msg);
  path_pub_.publish(path_actual);
  current_waypoint_pub_.publish(local_planner.waypt_p);
  tf_listener_.transformPose("local_origin", ros::Time(0), local_planner.waypt_p, "world", local_planner.waypt_p);
  mavros_waypoint_pub_.publish(local_planner.waypt_p); 

	publishMarkerBlocked();
  publishMarkerCandidates();
  publishMarkerRejected();
  publishMarkerSelected();
  publishMarkerExtended();
  publishGoal();
//  publishNormalToPowerline();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_planner_node");
  LocalPlannerNode local_planner_node;
  ros::Duration(2).sleep(); 
  ros::spin();
  
  return 0;
}


 	
