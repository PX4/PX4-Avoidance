//  June/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "global_planner/octomap_rrt_planner.h"
#include "global_planner/octomap_ompl_rrt.h"

using namespace Eigen;
using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;

// Constructor
OctomapRrtPlanner::OctomapRrtPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      avoidance_node_(nh, nh_private),
      cmdloop_dt_(0.1),
      plannerloop_dt_(1.0),
      rrt_planner_(nh, nh_private) {
#ifndef DISABLE_SIMULATION
  world_visualizer_.reset(new avoidance::WorldVisualizer(nh_));
#endif

  cmdloop_timer_ = nh_.createTimer(ros::Duration(cmdloop_dt_), &OctomapRrtPlanner::cmdloopCallback,
                                   this);  // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(plannerloop_dt_), &OctomapRrtPlanner::statusloopCallback,
                                      this);  // Define timer for constant loop rate

  pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 1, &OctomapRrtPlanner::positionCallback, this);
  move_base_simple_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &OctomapRrtPlanner::moveBaseSimpleCallback, this);

  octomap_full_sub_ = nh_.subscribe("/octomap_full", 1, &OctomapRrtPlanner::octomapFullCallback, this);
  trajectory_pub_ = nh_.advertise<mavros_msgs::Trajectory>("/mavros/trajectory/generated", 10);
  global_path_pub_ = nh_.advertise<nav_msgs::Path>("/global_temp_path", 10);
  pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_in", 10);

  listener_.waitForTransform("/fcu", "/world", ros::Time(0), ros::Duration(3.0));
  listener_.waitForTransform("/local_origin", "/world", ros::Time(0), ros::Duration(3.0));

  std::vector<std::string> camera_topics;
  nh_.getParam("pointcloud_topics", camera_topics);
  initializeCameraSubscribers(camera_topics);

  rrt_planner_.setMap(octree_);

  goal_ << 0.0, 1.0, 0.0;
}
OctomapRrtPlanner::~OctomapRrtPlanner() {
  // Destructor
}

void OctomapRrtPlanner::cmdloopCallback(const ros::TimerEvent& event) {
  hover_ = false;

  // Check if all information was received
  ros::Time now = ros::Time::now();
  last_wp_time_ = ros::Time::now();

  ros::Duration since_last_cloud = now - last_wp_time_;
  ros::Duration since_start = now - start_time_;

  avoidance_node_.checkFailsafe(since_last_cloud, since_start, hover_);
  publishSetpoint();
}

void OctomapRrtPlanner::statusloopCallback(const ros::TimerEvent& event) {
  std::lock_guard<std::mutex> lock(mutex_);

  planWithSimpleSetup();
  publishPath();
}

// Check if the current path is blocked
void OctomapRrtPlanner::octomapFullCallback(const octomap_msgs::Octomap& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (num_octomap_msg_++ % 10 > 0) {
    return;  // We get too many of those messages. Only process 1/10 of them
  }

  if (octree_) {
    delete octree_;
  }
  octree_ = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(msg));
}

// Go through obstacle points and store them
void OctomapRrtPlanner::depthCameraCallback(const sensor_msgs::PointCloud2& msg) {
  try {
    // Transform msg from camera frame to world frame
    ros::Time now = ros::Time::now();
    listener_.waitForTransform("/world", "/camera_link", now, ros::Duration(5.0));
    tf::StampedTransform transform;
    listener_.lookupTransform("/world", "/camera_link", now, transform);
    sensor_msgs::PointCloud2 transformed_msg;
    pcl_ros::transformPointCloud("/world", transform, msg, transformed_msg);
    pcl::PointCloud<pcl::PointXYZ> cloud;  // Easier to loop through pcl::PointCloud
    pcl::fromROSMsg(transformed_msg, cloud);

    pointcloud_pub_.publish(msg);
  } catch (tf::TransformException const& ex) {
    ROS_DEBUG("%s", ex.what());
    ROS_WARN("Transformation not available (/world to /camera_link");
  }
}

void OctomapRrtPlanner::positionCallback(const geometry_msgs::PoseStamped& msg) {
  local_position_(0) = msg.pose.position.x;
  local_position_(1) = msg.pose.position.y;
  local_position_(2) = msg.pose.position.z;
}

void OctomapRrtPlanner::velocityCallback(const geometry_msgs::TwistStamped& msg) {
  local_velocity_(0) = msg.twist.linear.x;
  local_velocity_(1) = msg.twist.linear.y;
  local_velocity_(2) = msg.twist.linear.z;
}

void OctomapRrtPlanner::moveBaseSimpleCallback(const geometry_msgs::PoseStamped& msg) {
  goal_(0) = msg.pose.position.x;
  goal_(1) = msg.pose.position.y;
  goal_(2) = 3.0;
}

void OctomapRrtPlanner::publishSetpoint() {
  mavros_msgs::Trajectory msg;

  trajectory_pub_.publish(msg);
}

void OctomapRrtPlanner::publishPath() {
  nav_msgs::Path msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "local_origin";

  std::vector<geometry_msgs::PoseStamped> path;

  for (int i = 0; i < current_path_.size(); i++) {
    path.insert(path.begin(), vector3d2PoseStampedMsg(current_path_[i]));
  }
  std::cout << current_path_.size() << std::endl;
  msg.poses = path;
  global_path_pub_.publish(msg);
}

void OctomapRrtPlanner::initializeCameraSubscribers(std::vector<std::string>& camera_topics) {
  cameras_.resize(camera_topics.size());

  for (size_t i = 0; i < camera_topics.size(); i++) {
    cameras_[i].pointcloud_sub_ = nh_.subscribe(camera_topics[i], 1, &OctomapRrtPlanner::depthCameraCallback, this);
  }
}

geometry_msgs::PoseStamped OctomapRrtPlanner::vector3d2PoseStampedMsg(Eigen::Vector3d position) {
  geometry_msgs::PoseStamped encode_msg;
  encode_msg.header.stamp = ros::Time::now();
  encode_msg.header.frame_id = "local_origin";
  encode_msg.pose.orientation.w = 1.0;
  encode_msg.pose.orientation.x = 0.0;
  encode_msg.pose.orientation.y = 0.0;
  encode_msg.pose.orientation.z = 0.0;
  encode_msg.pose.position.x = position(0);
  encode_msg.pose.position.y = position(1);
  encode_msg.pose.position.z = position(2);

  return encode_msg;
}

void OctomapRrtPlanner::planWithSimpleSetup() {
  if (octree_) {
    Eigen::Vector3d lower, upper;
    lower << -10.0, -10.0, -10.0;
    upper << 10.0, 10.0, 10.0;
    rrt_planner_.setMap(octree_);
    rrt_planner_.setBounds(lower, upper);
    rrt_planner_.setupProblem();
    rrt_planner_.getPath(local_position_, goal_, &current_path_);
  }
}