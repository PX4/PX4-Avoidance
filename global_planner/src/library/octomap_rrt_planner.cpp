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
      plan_(false),
      rrt_planner_(nh, nh_private) {

#ifndef DISABLE_SIMULATION
  world_visualizer_.reset(new avoidance::WorldVisualizer(nh_, ros::this_node::getName()));
#endif

  avoidance_node_.init();

  pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 1, &OctomapRrtPlanner::positionCallback, this);
  move_base_simple_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &OctomapRrtPlanner::moveBaseSimpleCallback, this);
  desiredtrajectory_sub_ = nh_.subscribe("/mavros/trajectory/desired", 1, &OctomapRrtPlanner::DesiredTrajectoryCallback, this);
  octomap_full_sub_ = nh_.subscribe("/octomap_full", 1, &OctomapRrtPlanner::octomapFullCallback, this);
  trajectory_pub_ = nh_.advertise<mavros_msgs::Trajectory>("/mavros/trajectory/generated", 10);
  actual_path_pub_ = nh_.advertise<nav_msgs::Path>("/actual_path", 10);
  mavros_waypoint_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  current_waypoint_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/current_setpoint", 10);
  global_path_pub_ = nh_.advertise<nav_msgs::Path>("/global_temp_path", 10);
  pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_in", 10);

  // RRT planner parameters
  min_altitude_ = nh_.param<double>("min_altitude", 3.0);
  max_altitude_ = nh_.param<double>("max_altitude", 4.0);
  goal_radius_ = nh_.param<double>("goal_radius", 1.0);
  speed_ = nh_.param<double>("default_speed",2.0);
  goal_altitude_ = nh_.param<double>("goal_altitude", 3.5);
  cmdloop_dt_ = nh_.param<double>("control_loop_dt", 0.05);
  plannerloop_dt_ = nh_.param<double>("planner_loop_dt", 0.1);

  
  listener_.waitForTransform("/fcu", "/world", ros::Time(0), ros::Duration(3.0));
  listener_.waitForTransform("/local_origin", "/world", ros::Time(0), ros::Duration(3.0));

  std::vector<std::string> camera_topics;
  nh_.getParam("pointcloud_topics", camera_topics);
  nh_.param<std::string>("frame_id", frame_id_, "local_origin");

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

  actual_path_.header.frame_id = frame_id_;
  rrt_planner_.setMap(octree_);

  goal_ << 0.0, 1.0, 0.0;

  current_goal_.header.frame_id = frame_id_;
  current_goal_.pose.position.x = 0; current_goal_.pose.position.y = 0; current_goal_.pose.position.z = 3.5;
  current_goal_.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  previous_goal_ = current_goal_;
  
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
  
  if(plan_)
  {
    planWithSimpleSetup();
    
  }
    publishPath();  // For visualization
}

// Check if the current path is blocked
void OctomapRrtPlanner::octomapFullCallback(const octomap_msgs::Octomap& msg) {

  std::lock_guard<std::mutex> lock(mutex_);

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
  last_pos_ = msg;
  local_position_(0) = msg.pose.position.x;
  local_position_(1) = msg.pose.position.y;
  local_position_(2) = msg.pose.position.z;

  last_pos_.header.frame_id = frame_id_;
  actual_path_.poses.push_back(last_pos_);
  actual_path_pub_.publish(actual_path_);

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
    goal_(2) = goal_altitude_;
    global_goal_.pose.position.x = msg.point_1.position.x;
    global_goal_.pose.position.y = msg.point_1.position.y;
    global_goal_.pose.position.z = goal_altitude_;
   
  }
}

void OctomapRrtPlanner::moveBaseSimpleCallback(const geometry_msgs::PoseStamped& msg) {

  plan_ = true;
  goal_(0) = msg.pose.position.x;
  goal_(1) = msg.pose.position.y;
  goal_(2) = goal_altitude_;
  
  global_goal_.pose.position.x =  msg.pose.position.x;
  global_goal_.pose.position.y =  msg.pose.position.y;
  global_goal_.pose.position.z = goal_altitude_;
  
}

void OctomapRrtPlanner::publishSetpoint() {

  // Vector pointing from current position to the current goal
  tf::Vector3 vec = global_planner::toTfVector3(global_planner::subtractPoints(current_goal_.pose.position, last_pos_.pose.position));


  // If we are less than 1.0 away, then we should stop at the goal
  double new_len = vec.length() < goal_radius_ ? vec.length() : speed_;
  vec.normalize();
  vec *= new_len;

  auto setpoint = current_goal_;  // The intermediate position sent to Mavros
  setpoint.pose.position.x = last_pos_.pose.position.x + vec.getX();
  setpoint.pose.position.y = last_pos_.pose.position.y + vec.getY();
  setpoint.pose.position.z = last_pos_.pose.position.z + vec.getZ();
  setpoint.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_);

  previous_goal_ = current_goal_;

  mavros_waypoint_publisher_.publish(setpoint);

  // Publish setpoint for vizualization
  current_waypoint_publisher_.publish(setpoint);


  mavros_msgs::Trajectory msg;
  avoidance::transformToTrajectory(msg, setpoint);
  msg.header.frame_id = frame_id_;
  msg.header.stamp = ros::Time::now();
  //trajectory_pub_.publish(msg);
}

void OctomapRrtPlanner::publishPath() {
  nav_msgs::Path msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id_;

  std::vector<geometry_msgs::PoseStamped> path;

  for (int i = 0; i < current_path_.size(); i++) {
    path.insert(path.begin(), vector3d2PoseStampedMsg(current_path_[i]));
  }
  
  if(current_path_.size() > 2) {
    msg.poses = path;
    global_path_pub_.publish(msg);
    current_goal_ = path[2];
  }
    
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
  encode_msg.header.frame_id = frame_id_;
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

double OctomapRrtPlanner::planYaw()
{
  
  double dx = current_goal_.pose.position.x - last_pos_.pose.position.x;
  double dy = current_goal_.pose.position.y - last_pos_.pose.position.y;

  return atan2(dy,dx);

}

void OctomapRrtPlanner::planWithSimpleSetup() {

  if(!isCloseToGoal())
  {
    if(octree_) {
      Eigen::Vector3d in_lower, in_upper, out_lower,out_upper;
      in_lower << local_position_(0), local_position_(1), min_altitude_;
      in_upper << goal_(0), goal_(1), max_altitude_;

      checkBounds(in_lower,in_upper,out_lower,out_upper);

      rrt_planner_.setMap(octree_);
      rrt_planner_.setBounds(out_lower, out_upper);
      rrt_planner_.setupProblem();
      rrt_planner_.getPath(local_position_, goal_, &current_path_);
      yaw_ = planYaw();

      if (current_path_.empty()) {
        current_path_.push_back(reference_pos_);
        ROS_INFO("Path is empty");
      }
        
    }
  }
  else
  {
    plan_ = false;
    yaw_ = 0.0;
    ROS_INFO("Already at goal");
  }

} 

bool OctomapRrtPlanner::isCloseToGoal() { 
  
  double dist = poseDistance(global_goal_, last_pos_);

  if( dist < goal_radius_)
    return true;
  else 
    return false; 

}

double OctomapRrtPlanner::poseDistance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2) {
  
  double dx = p2.pose.position.x - p1.pose.position.x;
  double dy = p2.pose.position.y - p1.pose.position.y;
  double dz = p2.pose.position.z - p1.pose.position.z;

  Vector3d vec(dx,dy,dz);

  return  vec.norm();

}

void OctomapRrtPlanner::checkBounds(const Eigen::Vector3d& in_lower, const Eigen::Vector3d& in_upper, Eigen::Vector3d& out_lower, Eigen::Vector3d& out_upper ) {
  for(int i = 0; i < 3; i++) 
  {
    if(in_lower(i) < in_upper(i))
    { 
      out_lower(i) = in_lower(i);
      out_upper(i) = in_upper(i);
    } 
    else
    {
      out_lower(i) = in_upper(i);
      out_upper(i) = in_lower(i);
    }
  }  

}