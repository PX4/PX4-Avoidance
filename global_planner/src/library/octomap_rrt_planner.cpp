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
      cmdloop_dt_(0.05),
      plannerloop_dt_(1.0),
      rrt_planner_(nh, nh_private) {
#ifndef DISABLE_SIMULATION
  world_visualizer_.reset(new avoidance::WorldVisualizer(nh_));
#endif

  pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 1, &OctomapRrtPlanner::positionCallback, this);
  move_base_simple_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &OctomapRrtPlanner::moveBaseSimpleCallback, this);
  desiredtrajectory_sub_ =
      nh_.subscribe("/mavros/trajectory/desired", 1, &OctomapRrtPlanner::DesiredTrajectoryCallback, this);

  octomap_full_sub_ = nh_.subscribe("/octomap_full", 1, &OctomapRrtPlanner::octomapFullCallback, this);
  trajectory_pub_ = nh_.advertise<mavros_msgs::Trajectory>("/mavros/trajectory/generated", 10);
  global_path_pub_ = nh_.advertise<nav_msgs::Path>("/global_temp_path", 10);
  pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_in", 10);

  listener_.waitForTransform("/fcu", "/world", ros::Time(0), ros::Duration(3.0));
  listener_.waitForTransform("/local_origin", "/world", ros::Time(0), ros::Duration(3.0));

  std::vector<std::string> camera_topics;
  nh_.getParam("pointcloud_topics", camera_topics);
  nh_.param<std::string>("frame_id", frame_id_, "/local_origin");

  ros::TimerOptions cmdlooptimer_options(ros::Duration(cmdloop_dt_),
                                         boost::bind(&OctomapRrtPlanner::cmdLoopCallback, this, _1), &cmdloop_queue_);
  cmdloop_timer_ = nh_.createTimer(cmdlooptimer_options);

  cmdloop_spinner_.reset(new ros::AsyncSpinner(1, &cmdloop_queue_));
  cmdloop_spinner_->start();

  ros::TimerOptions plannerlooptimer_options(ros::Duration(plannerloop_dt_),
                                             boost::bind(&OctomapRrtPlanner::plannerLoopCallback, this, _1),
                                             &plannerloop_queue_);
  plannerloop_timer_ = nh_.createTimer(plannerlooptimer_options);

  plannerloop_spinner_.reset(new ros::AsyncSpinner(1, &plannerloop_queue_));
  plannerloop_spinner_->start();

  initializeCameraSubscribers(camera_topics);

  rrt_planner_.setMap(octree_);

  goal_ << 0.0, 1.0, 0.0;
  reference_att_.x() = 0.0;
  reference_att_.y() = 0.0;
  reference_att_.z() = 0.0;
  reference_att_.w() = 1.0;
}
OctomapRrtPlanner::~OctomapRrtPlanner() {
  // Destructor
}

void OctomapRrtPlanner::cmdLoopCallback(const ros::TimerEvent& event) {
  hover_ = false;

  // Check if all information was received
  ros::Time now = ros::Time::now();
  last_wp_time_ = ros::Time::now();

  ros::Duration since_last_cloud = now - last_wp_time_;
  ros::Duration since_start = now - start_time_;

  avoidance_node_.checkFailsafe(since_last_cloud, since_start, hover_);
  updateReference(now);
  publishSetpoint();
}

void OctomapRrtPlanner::plannerLoopCallback(const ros::TimerEvent& event) {
  std::lock_guard<std::mutex> lock(mutex_);

  planWithSimpleSetup();
  publishPath();  // For visualization
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

void OctomapRrtPlanner::DesiredTrajectoryCallback(const mavros_msgs::Trajectory& msg) {
  // Read waypoint from trajectory messages
  if (msg.point_valid[0]) {
    goal_(0) = msg.point_1.position.x;
    goal_(1) = msg.point_1.position.y;
    goal_(2) = msg.point_1.position.z;
  }
}

void OctomapRrtPlanner::moveBaseSimpleCallback(const geometry_msgs::PoseStamped& msg) {
  goal_(0) = msg.pose.position.x;
  goal_(1) = msg.pose.position.y;
  goal_(2) = 3.0;
}

void OctomapRrtPlanner::publishSetpoint() {
  mavros_msgs::Trajectory msg;
  Eigen::Vector3f reference_posf = reference_pos_.cast<float>();
  avoidance::transformToTrajectory(msg, avoidance::toPoseStamped(reference_posf, reference_att_));
  msg.header.frame_id = frame_id_;
  msg.header.stamp = ros::Time::now();
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

void OctomapRrtPlanner::updateReference(ros::Time current_time) { reference_pos_ << 0.0, 0.0, 3.0; }

void OctomapRrtPlanner::planWithSimpleSetup() {
  if (octree_) {
    // TODO: Define planning bounds based on goal and current location
    Eigen::Vector3d lower, upper;
    lower << -10.0, -10.0, -10.0;
    upper << 10.0, 10.0, 50.0;
    rrt_planner_.setMap(octree_);
    rrt_planner_.setBounds(lower, upper);
    rrt_planner_.setupProblem();
    rrt_planner_.getPath(local_position_, goal_, &current_path_);
    if (current_path_.empty()) {
      current_path_.emplace_back(reference_pos_);
      std::cout << "Path is empty" << std::endl;
    }
  }
}