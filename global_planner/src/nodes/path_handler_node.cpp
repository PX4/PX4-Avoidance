#include "path_handler_node.h"

namespace global_planner {

PathHandlerNode::PathHandlerNode() {
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
  three_point_path_publisher_ =
      nh_.advertise<nav_msgs::Path>("/three_point_path", 10);
  three_point_msg_publisher_ =
      nh_.advertise<global_planner::ThreePointMsg>("/three_point_msg", 10);
  // avoidance_triplet_msg_publisher_ =
  // nh_.advertise<mavros_msgs::AvoidanceTriplet>("/mavros/avoidance_triplet",
  // 10);

  // Initialize goal
  current_goal_.header.frame_id = "/world";
  current_goal_.pose.position = start_pos_;
  current_goal_.pose.orientation = tf::createQuaternionMsgFromYaw(start_yaw_);
  last_goal_ = current_goal_;

  listener_.waitForTransform("/local_origin", "/world", ros::Time(0),
                             ros::Duration(3.0));
  ros::Rate rate(10);
  while (ros::ok()) {
    rate.sleep();
    ros::spinOnce();

    if (shouldPublishThreePoints()) {
      publishThreePointMsg();
    } else {
      publishSetpoint();
    }
  }
}

PathHandlerNode::~PathHandlerNode() {}

void PathHandlerNode::readParams() {
  nh_.param<double>("start_pos_x", start_pos_.x, 0.5);
  nh_.param<double>("start_pos_y", start_pos_.y, 0.5);
  nh_.param<double>("start_pos_z", start_pos_.z, 3.5);
}

bool PathHandlerNode::shouldPublishThreePoints() { return three_point_mode_; }

bool PathHandlerNode::isCloseToGoal() {
  if (shouldPublishThreePoints()) {
    // For three point messages we send a new message when we are close to the
    // end
    return distance(last_pos_.pose.position, current_goal_.pose.position) +
               0.5 >
           distance(last_pos_.pose.position, path_.front().pose.position);
    // return distance(last_pos_.pose.position, path_.front().pose.position)
    // < 1.0;
  }
  return distance(current_goal_, last_pos_) < 1.5;
}

double PathHandlerNode::getRiskOfCurve(
    const std::vector<geometry_msgs::PoseStamped>& poses) {
  double risk = 0.0;
  for (const auto& pose_msg : poses) {
    tf::Vector3 point = toTfVector3(pose_msg.pose.position);
    if (path_risk_.find(point) != path_risk_.end()) {
      risk += path_risk_[point];
    }
  }
  return risk;
}

void PathHandlerNode::setCurrentPath(
    const std::vector<geometry_msgs::PoseStamped>& poses) {
  speed_ = min_speed_;
  path_.clear();
  path_risk_.clear();

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
  three_point_mode_ = config.three_point_mode_;
  min_speed_ = config.min_speed_;
  max_speed_ = config.max_speed_;
  three_point_speed_ = config.three_point_speed_;
  direct_goal_alt_ = config.direct_goal_alt_;

  // Reset speed_
  speed_ = min_speed_;
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

void PathHandlerNode::receivePathWithRisk(const PathWithRiskMsg& msg) {
  if (!ignore_path_messages_) {
    setCurrentPath(msg.poses);
    if (msg.poses.size() != msg.risks.size()) {
      ROS_INFO("PathWithRiskMsg error: risks must be the same size as poses.");
      throw std::invalid_argument(
          "PathWithRiskMsg error: risks must be the same size as poses.");
    }
    for (int i = 0; i < msg.poses.size(); ++i) {
      tf::Vector3 point = toTfVector3(msg.poses[i].pose.position);
      path_risk_[point] = msg.risks[i];
    }
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

void PathHandlerNode::fillUnusedTrajectoryPoint(
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

void PathHandlerNode::transformPoseToObstacleAvoidance(
    mavros_msgs::Trajectory& obst_avoid, geometry_msgs::PoseStamped pose) {
  obst_avoid.header = pose.header;
  obst_avoid.type = 0;
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
  transformPoseToObstacleAvoidance(obst_free_path, setpoint);
  mavros_obstacle_free_path_pub_.publish(obst_free_path);
}

// Send a 3-point message representing the current path
void PathHandlerNode::publishThreePointMsg() {
  geometry_msgs::PoseStamped next_goal;
  if (!path_.empty()) {
    next_goal = path_.front();
  } else {
    next_goal = current_goal_;
  }
  nav_msgs::Path three_point_path;
  three_point_path.header.frame_id = "/world";
  three_point_path.poses = {last_goal_, current_goal_, next_goal};
  three_point_path.poses[0].pose.position =
      middlePoint(last_goal_.pose.position, current_goal_.pose.position);
  three_point_path.poses[2].pose.position =
      middlePoint(current_goal_.pose.position, next_goal.pose.position);
  // three_point_path = smoothPath(three_point_path);
  three_point_path_publisher_.publish(three_point_path);

  double risk = getRiskOfCurve(three_point_path.poses);
  // Send the three points as a ThreePointMessage
  ThreePointMsg three_point_msg;
  three_point_msg.prev = last_goal_.pose.position;
  three_point_msg.ctrl = current_goal_.pose.position;
  three_point_msg.next = path_.front().pose.position;
  double speed = distance(three_point_msg.prev, three_point_msg.next);
  three_point_msg.duration =
      std::max(1.0, speed / max_speed_) * three_point_speed_;
  // three_point_msg.duration = 1.0;
  three_point_msg.max_acc = risk;
  three_point_msg.acc_per_err = risk;
  three_point_msg_publisher_.publish(three_point_msg);

  // mavros_msgs::AvoidanceTriplet avoidance_triplet;
  // avoidance_triplet.prev = last_goal_.pose.position;
  // avoidance_triplet.ctrl = current_goal_.pose.position;
  // avoidance_triplet.next = path_.front().pose.position;
  // avoidance_triplet.duration = std::max(1.0, speed / 3.0);
  // // avoidance_triplet.duration = 1.0;
  // avoidance_triplet.max_acc = risk;
  // avoidance_triplet.acc_per_err = risk;
  // avoidance_triplet_msg_publisher_.publish(avoidance_triplet);
}

}  // namespace global_planner

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_handler_node");
  global_planner::PathHandlerNode path_handler_node;
  ros::spin();
  return 0;
}
