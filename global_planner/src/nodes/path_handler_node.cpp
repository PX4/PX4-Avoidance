#include "path_handler_node.h"

using namespace avoidance;

namespace global_planner {

PathHandlerNode::PathHandlerNode() : spin_dt_(0.1) {
  nh_ = ros::NodeHandle("~");

  // Set up Dynamic Reconfigure Server
  dynamic_reconfigure::Server<
      global_planner::PathHandlerNodeConfig>::CallbackType f;
  f = boost::bind(&PathHandlerNode::dynamicReconfigureCallback, this, _1, _2);
  server_.setCallback(f);

  // Read Ros parameters
  readParams();

  // Subscribe to topics
  direct_goal_sub_ = nh_.subscribe("/initialpose", 1,
                                   &PathHandlerNode::receiveDirectGoal, this);
  path_sub_ = nh_.subscribe("/global_temp_path", 1,
                            &PathHandlerNode::receivePath, this);
  path_with_risk_sub_ =
      nh_.subscribe("/path_with_risk", 1, &PathHandlerNode::receivePath, this);
  ground_truth_sub_ = nh_.subscribe("/mavros/local_position/pose", 1,
                                    &PathHandlerNode::positionCallback, this);

  // Advertice topics
  mavros_waypoint_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "/mavros/setpoint_position/local", 10);
  mavros_obstacle_free_path_pub_ = nh_.advertise<mavros_msgs::Trajectory>(
      "/mavros/trajectory/generated", 10);
  current_waypoint_publisher_ =
      nh_.advertise<geometry_msgs::PoseStamped>("/current_setpoint", 10);
  mavros_system_status_pub_ =
      nh_.advertise<mavros_msgs::CompanionProcessStatus>(
          "/mavros/companion_process/status", 1);
  // avoidance_triplet_msg_publisher_ =
  // nh_.advertise<mavros_msgs::AvoidanceTriplet>("/mavros/avoidance_triplet",
  // 10);
  nh_.param<std::string>("world_name", world_path_, "");

  // Initialize goal
  current_goal_.header.frame_id = "/world";
  current_goal_.pose.position = start_pos_;
  current_goal_.pose.orientation = tf::createQuaternionMsgFromYaw(start_yaw_);
  last_goal_ = current_goal_;

#ifndef DISABLE_SIMULATION
  world_visualizer_.reset(new WorldVisualizer(nh_));
#endif

  listener_.waitForTransform("/local_origin", "/world", ros::Time(0),
                             ros::Duration(3.0));

  ros::TimerOptions timer_options(
      ros::Duration(spin_dt_),
      boost::bind(&PathHandlerNode::cmdLoopCallback, this, _1),
      &cmdloop_queue_);
  cmdloop_timer_ = nh_.createTimer(timer_options);

  cmdloop_spinner_.reset(new ros::AsyncSpinner(1, &cmdloop_queue_));
  cmdloop_spinner_->start();
}

PathHandlerNode::~PathHandlerNode() {}

void PathHandlerNode::readParams() {
  nh_.param<double>("start_pos_x", start_pos_.x, 0.5);
  nh_.param<double>("start_pos_y", start_pos_.y, 0.5);
  nh_.param<double>("start_pos_z", start_pos_.z, 3.5);
}

bool PathHandlerNode::isCloseToGoal() {
  return distance(current_goal_, last_pos_) < 1.5;
}

void PathHandlerNode::setCurrentPath(
    const std::vector<geometry_msgs::PoseStamped>& poses) {
  speed_ = min_speed_;
  path_.clear();

  if (poses.size() < 2) {
    ROS_INFO("  Received empty path\n");
    return;
  }
  last_goal_ = poses[0];
  current_goal_ = poses[1];

  for (int i = 2; i < poses.size(); ++i) {
    path_.push_back(poses[i]);
  }
}

void PathHandlerNode::dynamicReconfigureCallback(
    global_planner::PathHandlerNodeConfig& config, uint32_t level) {
  ignore_path_messages_ = config.ignore_path_messages_;
  min_speed_ = config.min_speed_;
  max_speed_ = config.max_speed_;
  direct_goal_alt_ = config.direct_goal_alt_;

  // Reset speed_
  speed_ = min_speed_;
}

void PathHandlerNode::cmdLoopCallback(const ros::TimerEvent& event) {

  ros::Time now = ros::Time::now();

  publishSetpoint();

  if (now - t_status_sent_ > ros::Duration(0.2)) publishSystemStatus();
}

void PathHandlerNode::receiveDirectGoal(
    const geometry_msgs::PoseWithCovarianceStamped& msg) {
  // Receive a goal without planning
  current_goal_.pose = msg.pose.pose;
  current_goal_.pose.position.z = direct_goal_alt_;  // Direct goal is 2D
  current_goal_.header = msg.header;
  std::vector<geometry_msgs::PoseStamped> path_with_direct_goal{
      current_goal_, current_goal_, current_goal_};
  setCurrentPath(path_with_direct_goal);
}

void PathHandlerNode::receivePath(const nav_msgs::Path& msg) {
  if (!ignore_path_messages_) {
    setCurrentPath(msg.poses);
  }
}

void PathHandlerNode::positionCallback(
    const geometry_msgs::PoseStamped& pose_msg) {
  listener_.transformPose("world", ros::Time(0), pose_msg, "local_origin",
                          last_pos_);

  // Check if we are close enough to current goal to get the next part of the
  // path
  if (path_.size() > 0 && isCloseToGoal()) {
    // TODO: get yawdiff(yaw1, yaw2)
    double yaw1 = tf::getYaw(current_goal_.pose.orientation);
    double yaw2 = tf::getYaw(last_pos_.pose.orientation);
    double yaw_diff = std::abs(yaw2 - yaw1);
    // Transform yaw_diff to [0, 2*pi]
    yaw_diff -= std::floor(yaw_diff / (2 * M_PI)) * (2 * M_PI);
    double max_yaw_diff = M_PI / 1.0;
    if (yaw_diff < max_yaw_diff || yaw_diff > 2 * M_PI - max_yaw_diff) {
      // If we are facing the right direction, then pop the first point of the
      // path
      last_goal_ = current_goal_;
      current_goal_ = path_[0];
      path_.erase(path_.begin());

      // If we are keeping the same direction and height increase speed
      // if (path_.size() > std::floor(speed_) &&
      // hasSameYawAndAltitude(current_goal_.pose,
      // path_[std::floor(speed_)].pose)) {
      //   speed_ = std::min(max_speed_, speed_ + 0.1);
      // }
      // else {
      //   speed_ = min_speed_;
      // }
    }
  }
}

void PathHandlerNode::publishSetpoint() {
  // Vector pointing from current position to the current goal
  tf::Vector3 vec = toTfVector3(
      subtractPoints(current_goal_.pose.position, last_pos_.pose.position));
  // If we are less than 1.0 away, then we should stop at the goal
  double new_len = vec.length() < 1.0 ? vec.length() : speed_;
  vec.normalize();
  vec *= new_len;

  auto setpoint = current_goal_;  // The intermediate position sent to Mavros
  setpoint.pose.position.x = last_pos_.pose.position.x + vec.getX();
  setpoint.pose.position.y = last_pos_.pose.position.y + vec.getY();
  setpoint.pose.position.z = last_pos_.pose.position.z + vec.getZ();

  // Publish setpoint for vizualization
  current_waypoint_publisher_.publish(setpoint);

  listener_.transformPose("local_origin", ros::Time(0), setpoint, "world",
                          setpoint);

  // Publish setpoint to Mavros
  mavros_waypoint_publisher_.publish(setpoint);
  mavros_msgs::Trajectory obst_free_path = {};
  transformPoseToTrajectory(obst_free_path, setpoint);
  mavros_obstacle_free_path_pub_.publish(obst_free_path);
}

// Publish companion process status
void PathHandlerNode::publishSystemStatus() {
  mavros_msgs::CompanionProcessStatus status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
  status_msg.state = static_cast<int>(MAV_STATE::MAV_STATE_ACTIVE);

  mavros_system_status_pub_.publish(status_msg);
  t_status_sent_ = ros::Time::now();
}

}  // namespace global_planner

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_handler_node");
  global_planner::PathHandlerNode path_handler_node;
  ros::spin();
  return 0;
}
